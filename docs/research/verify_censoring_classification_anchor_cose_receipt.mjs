#!/usr/bin/env node
import fs from "node:fs";
import crypto from "node:crypto";

const TS_SCHEMA="mycelix.continual-adaptation.censoring-classification-anchor-cose-receipt-ts-registry.v1";
const FIXTURE_SCHEMA="mycelix.continual-adaptation.censoring-classification-anchor-cose-receipt-fixture.v1";
const CAMPAIGN_SCHEMA="mycelix.continual-adaptation.censoring-classification-anchor-cose-receipt-campaign.v1";
const HEAD_SCHEMA="mycelix.continual-adaptation.censoring-classification-anchor-witness-vds-tree-head.v1";
const WREG_ID="mycelix.research.anchor-witness-registry.v2", VDS_ID="mycelix.research.anchor-statement-sequence.v1";
const TS_ID="mycelix.research.anchor-cose-receipt-ts-registry.v1", TS_KEY_ID="cose-test-ts-k1";
const EXPECTED_TS_REGISTRY_SHA="sha256:21772a0a1dbb88358c80e5297d4ec4dd82c2ba2bca474a334bae45535ec8b331";
const EXPECTED_TRUST_ROOT_SHA="sha256:99ec416914eb75a7c953fbc74045d25fc83cb894c6e7a5bcd899c757a736b9d7";
const ALG=-8, VDS=1;

const canonical=v=>v===null||typeof v!=="object"?JSON.stringify(v):Array.isArray(v)?"["+v.map(canonical).join(",")+"]":"{"+Object.keys(v).sort().map(k=>JSON.stringify(k)+":"+canonical(v[k])).join(",")+"}";
const H=b=>crypto.createHash("sha256").update(b).digest();
const digestObj=v=>"sha256:"+H(Buffer.from(canonical(v),"utf8")).toString("hex");
const nodeHash=(a,b)=>H(Buffer.concat([Buffer.from([1]),a,b]));
const leafHash=b=>H(Buffer.concat([Buffer.from([0]),b]));
const b64u=s=>{if(typeof s!=="string"||!/^[A-Za-z0-9_-]*$/.test(s))throw Error("base64url");return Buffer.from(s,"base64url");};
function cborHead(major,n){
  if(!Number.isSafeInteger(n)||n<0)throw Error("cbor-integer-range");
  if(n<24)return Buffer.from([(major<<5)|n]);
  if(n<256)return Buffer.from([(major<<5)|24,n]);
  if(n<65536){const b=Buffer.alloc(3);b[0]=(major<<5)|25;b.writeUInt16BE(n,1);return b;}
  if(n<2**32){const b=Buffer.alloc(5);b[0]=(major<<5)|26;b.writeUInt32BE(n,1);return b;}
  const b=Buffer.alloc(9);b[0]=(major<<5)|27;b.writeBigUInt64BE(BigInt(n),1);return b;
}
function cborEncode(v){
  if(v===null)return Buffer.from([0xf6]);if(v===false)return Buffer.from([0xf4]);if(v===true)return Buffer.from([0xf5]);
  if(typeof v==="number"){if(!Number.isSafeInteger(v))throw Error("cbor-integer-required");return v>=0?cborHead(0,v):cborHead(1,-1-v);}
  if(Buffer.isBuffer(v))return Buffer.concat([cborHead(2,v.length),v]);
  if(typeof v==="string"){const b=Buffer.from(v,"utf8");return Buffer.concat([cborHead(3,b.length),b]);}
  if(Array.isArray(v))return Buffer.concat([cborHead(4,v.length),...v.map(cborEncode)]);
  if(v instanceof Map){const pairs=[...v.entries()].map(([k,val])=>[cborEncode(k),cborEncode(val)]).sort((a,b)=>Buffer.compare(a[0],b[0]));return Buffer.concat([cborHead(5,pairs.length),...pairs.flat()]);}
  throw Error("cbor-unsupported-type");
}
function cborMapInGivenOrder(pairs){return Buffer.concat([cborHead(5,pairs.length),...pairs.flatMap(([k,v])=>[cborEncode(k),cborEncode(v)])]);}
function cborDecode(data,state={pos:0}){
  if(state.pos>=data.length)throw Error("cbor-truncated");
  const initial=data[state.pos++],major=initial>>5,ai=initial&31;
  if(major===7){if(ai===20)return false;if(ai===21)return true;if(ai===22)return null;throw Error("cbor-simple-or-float");}
  if(ai===31)throw Error("cbor-indefinite-length");
  let n;
  if(ai<24)n=ai;
  else if(ai===24){if(state.pos+1>data.length)throw Error("cbor-truncated");n=data[state.pos++];}
  else if(ai===25){if(state.pos+2>data.length)throw Error("cbor-truncated");n=data.readUInt16BE(state.pos);state.pos+=2;}
  else if(ai===26){if(state.pos+4>data.length)throw Error("cbor-truncated");n=data.readUInt32BE(state.pos);state.pos+=4;}
  else if(ai===27){if(state.pos+8>data.length)throw Error("cbor-truncated");const big=data.readBigUInt64BE(state.pos);state.pos+=8;if(big>BigInt(Number.MAX_SAFE_INTEGER))throw Error("cbor-integer-range");n=Number(big);}
  else throw Error("cbor-reserved-additional-info");
  if(major===0)return n;if(major===1)return -1-n;
  if(major===2||major===3){if(state.pos+n>data.length)throw Error("cbor-truncated");const b=data.subarray(state.pos,state.pos+n);state.pos+=n;return major===2?Buffer.from(b):b.toString("utf8");}
  if(major===4){const a=[];for(let i=0;i<n;i++)a.push(cborDecode(data,state));return a;}
  if(major===5){const m=new Map();for(let i=0;i<n;i++){const k=cborDecode(data,state),v=cborDecode(data,state);if(m.has(k))throw Error("cbor-invalid-or-duplicate-map-key");m.set(k,v);}return m;}
  throw Error("cbor-unsupported-major-type");
}
function decodeExact(b){const state={pos:0},v=cborDecode(b,state);if(state.pos!==b.length)throw Error("cbor-trailing-bytes");return v;}
function mth(entries){
  if(entries.length===0)return H(Buffer.alloc(0));if(entries.length===1)return leafHash(Buffer.from(canonical(entries[0]),"utf8"));
  const k=1<<((entries.length-1).toString(2).length-1);
  return nodeHash(mth(entries.slice(0,k)),mth(entries.slice(k)));
}
function inclusionRoot(index,size,entryBytes,path){
  if(!Number.isInteger(size)||!Number.isInteger(index)||size<=0||index<0||index>=size)return null;
  if(path.some(p=>!Buffer.isBuffer(p)||p.length!==32))return null;
  let fn=index,sn=size-1,r=leafHash(entryBytes);
  for(const p of path){
    if(sn===0)return null;
    if((fn&1)||fn===sn){
      r=nodeHash(p,r);
      if(!(fn&1))while(fn&&!(fn&1)){fn>>=1;sn>>=1;}
    }else r=nodeHash(r,p);
    fn>>=1;sn>>=1;
  }
  return sn===0?r:null;
}
function consistencyValid(oldSize,newSize,oldRoot,newRoot,path){
  if(!Number.isInteger(oldSize)||!Number.isInteger(newSize)||!(0<oldSize&&oldSize<newSize)||!path.length||path.some(p=>!Buffer.isBuffer(p)||p.length!==32))return false;
  let fn=oldSize-1,sn=newSize-1;while(fn&1){fn>>=1;sn>>=1;}
  let fr,sr,siblings;
  if(fn===0){fr=sr=oldRoot;siblings=path;}else{if(!path.length)return false;fr=sr=path[0];siblings=path.slice(1);}
  for(const p of siblings){
    if(sn===0)return false;
    if((fn&1)||fn===sn){
      fr=nodeHash(p,fr);sr=nodeHash(p,sr);
      if(!(fn&1))while(fn&&!(fn&1)){fn>>=1;sn>>=1;}
    }else sr=nodeHash(sr,p);
    fn>>=1;sn>>=1;
  }
  return sn===0&&Buffer.compare(fr,oldRoot)===0&&Buffer.compare(sr,newRoot)===0;
}
function publicKey(raw){return crypto.createPublicKey({key:Buffer.concat([Buffer.from("302a300506032b6570032100","hex"),raw]),format:"der",type:"spki"});}
function verifyEd(pub,sig,msg){try{return crypto.verify(null,msg,publicKey(pub),sig)}catch{return false;}}
function witnessKey(reg,w,kid,version){
  const wi=reg.witnesses?.[w],key=wi?.keys?.[kid];if(!key)return[null,"unknown-witness-or-key"];
  if(key.algorithm!=="Ed25519")return[null,"witness-key-algorithm"];
  if(version<key.valid_from_version)return[null,"witness-key-not-yet-valid"];
  if(key.valid_until_version!==null&&version>key.valid_until_version)return[null,"witness-key-expired"];
  if(key.revoked_at_version!==null&&version>=key.revoked_at_version)return[null,"witness-key-revoked"];
  if(key.status==="revoked")return[null,"witness-key-revoked"];
  try{const p=b64u(key.public_key);return p.length===32?[p,null]:[null,"witness-key-encoding"];}catch{return[null,"witness-key-encoding"];}
}
function verifyTreeHead(a,w,reg,entries){
  const fields=["schema","domain","algorithm","observer_id","key_id","registry_id","registry_version","vds_id","manifest_version","tree_size","root_hash","signature"].sort();
  if(!a||typeof a!=="object"||Object.keys(a).sort().join("|")!==fields.join("|"))return[null,"head-schema"];
  if(a.schema!==HEAD_SCHEMA||a.domain!==HEAD_SCHEMA)return[null,"head-domain"];
  if(a.algorithm!=="Ed25519")return[null,"head-algorithm"];
  if(a.observer_id!==w)return[null,"head-observer-binding"];
  if(a.registry_id!==WREG_ID||a.registry_version!==2)return[null,"head-registry-binding"];
  if(a.vds_id!==VDS_ID)return[null,"head-vds-binding"];
  const n=a.tree_size;if(!Number.isInteger(n)||n<1||n>entries.length)return[null,"head-tree-size"];
  let claimed;try{if(!/^sha256:[0-9a-f]{64}$/.test(a.root_hash))throw 0;claimed=Buffer.from(a.root_hash.slice(7),"hex");}catch{return[null,"head-root-encoding"];}
  if(Buffer.compare(mth(entries.slice(0,n)),claimed)!==0)return[null,"head-root-mismatch"];
  const[pub,e]=witnessKey(reg,w,a.key_id,a.manifest_version);if(e)return[null,"head-"+e];
  let sig;try{sig=b64u(a.signature);}catch{return[null,"head-signature-encoding"];}
  const payload={schema:HEAD_SCHEMA,domain:HEAD_SCHEMA,algorithm:"Ed25519",observer_id:w,key_id:a.key_id,witness_identity_commitment:reg.witnesses[w].identity_commitment,registry_id:a.registry_id,registry_version:a.registry_version,vds_id:a.vds_id,manifest_version:a.manifest_version,tree_size:n,root_hash:a.root_hash};
  return verifyEd(pub,sig,Buffer.from(canonical(payload),"utf8"))?[a,null]:[null,"head-signature-invalid"];
}
function validateHeadQuorum(hf,wreg,vds,name){
  if(wreg.registry_id!==WREG_ID||wreg.registry_version!==2)return[null,"witness-registry-binding"];
  const heads=hf.heads?.[name]?.attestations;if(!heads||Object.keys(heads).length<3)return[null,"head-below-threshold"];
  const vals=[];
  for(const[w,a]of Object.entries(heads)){const[v,e]=verifyTreeHead(a,w,wreg,vds.entries);if(e)return[null,e];vals.push(v);}
  const set=new Set(vals.map(h=>JSON.stringify([h.manifest_version,h.tree_size,h.root_hash])));if(set.size!==1)return[null,"head-equivocation"];
  return[{tree_size:vals[0].tree_size,root:Buffer.from(vals[0].root_hash.slice(7),"hex"),claims:vals},null];
}
function validCoseEnvelope(wire){
  if(!wire.length||wire[0]!==0xd2)return[null,"cose-tag"];
  let obj;try{obj=decodeExact(wire.subarray(1));}catch{return[null,"cbor-decode"]; }
  if(!Array.isArray(obj)||Buffer.compare(cborEncode(obj),wire.subarray(1))!==0)return[null,"cose-noncanonical-envelope"];
  if(obj.length!==4)return[null,"cose-sign1-structure"];
  const[protectedBytes,unprotected,payload,signature]=obj;
  if(!Buffer.isBuffer(protectedBytes)||!(unprotected instanceof Map)||!Buffer.isBuffer(signature)||signature.length!==64)return[null,"cose-field-type"];
  let protectedMap;try{protectedMap=decodeExact(protectedBytes);}catch{return[null,"cbor-decode"];}
  if(!(protectedMap instanceof Map)||Buffer.compare(cborEncode(protectedMap),protectedBytes)!==0)return[null,"cose-noncanonical-protected"];
  return[{object:obj,protectedBytes,protected:protectedMap,unprotected,payload,signature},null];
}
function mutateWire(fixture,kind,mutation){
  const wire=Buffer.from(fixture[kind==="inclusion"?"inclusion_receipt_cose_hex":"consistency_receipt_cose_hex"],"hex");
  if(mutation==="tag-removal")return wire.subarray(1);
  const[parsed,err]=validCoseEnvelope(wire);if(err)return wire;
  const obj=parsed.object,kindMap=obj[1];
  if(mutation==="signature-bitflip"){const s=Buffer.from(obj[3]);s[s.length-1]^=1;obj[3]=s;}
  else if(mutation==="attached-payload")obj[2]=Buffer.from(fixture.root_hash.slice(7),"hex");
  else if(mutation==="extra-unprotected-label")kindMap.set(123,true);
  else if(mutation==="algorithm-substitution"){const p=decodeExact(obj[0]);p.set(1,-7);obj[0]=cborEncode(p);}
  else if(mutation==="vds-substitution"){const p=decodeExact(obj[0]);p.set(395,999);obj[0]=cborEncode(p);}
  else if(mutation==="key-id-substitution"){const p=decodeExact(obj[0]);p.set(4,Buffer.from("attacker-k1"));obj[0]=cborEncode(p);}
  else if(mutation==="noncanonical-protected-map")obj[0]=cborMapInGivenOrder([[395,1],[4,Buffer.from("cose-test-ts-k1")],[1,-8]]);
  else if(mutation==="wrong-proof-label"){
    const vdp=kindMap.get(396),oldLabel=vdp.has(-2)?-2:-1,old=vdp.get(oldLabel);vdp.clear();vdp.set(396,new Map([[oldLabel===-2?-1:-2,old]]));
  }
  else if(["proof-path","proof-index-out-of-range","proof-tree-size","empty-inclusion-path","old-size-substitution","new-size-substitution"].includes(mutation)){
    const vdp=kindMap.get(396),label=kind==="inclusion"?-1:-2,proofBytes=vdp.get(label)[0],proof=decodeExact(proofBytes);
    if(mutation==="proof-path"){const p=Buffer.from(proof[2][0]);p[0]^=1;proof[2][0]=p;}
    else if(mutation==="proof-index-out-of-range")proof[1]=proof[0];
    else if(mutation==="proof-tree-size")proof[0]-=1;
    else if(mutation==="empty-inclusion-path")proof[2]=[];
    else if(mutation==="old-size-substitution")proof[0]-=1;
    else if(mutation==="new-size-substitution")proof[1]-=1;
    vdp.set(label,[cborEncode(proof)]);
  }
  else if(mutation==="drop-sign1-field")obj.pop();
  return Buffer.concat([Buffer.from([0xd2]),cborEncode(obj)]);
}
function verifyReceipt(kind,fixture,tsreg,wreg,vds,heads,trustRoot,wire){
  if(tsreg.schema!==TS_SCHEMA||tsreg.registry_id!==TS_ID||tsreg.registry_version!==1)return"registry-schema";
  if(digestObj(tsreg)!==EXPECTED_TS_REGISTRY_SHA)return"registry-pin";
  if(fixture.ts_registry_sha256!==EXPECTED_TS_REGISTRY_SHA||fixture.ts_registry_id!==TS_ID)return"fixture-registry-binding";
  if(digestObj(wreg)!==trustRoot.registry_sha256||digestObj(trustRoot)!==EXPECTED_TRUST_ROOT_SHA)return"witness-trust-root-pin";
  if(fixture.vds_id!==VDS_ID||vds.vds_id!==VDS_ID)return"vds-binding";
  if(mth(vds.entries.slice(0,fixture.tree_size)).toString("hex")!==fixture.root_hash.slice(7))return"fixture-root-binding";
  const[parsed,err]=validCoseEnvelope(wire);if(err)return err;
  const p=parsed.protected,unp=parsed.unprotected;
  if(p.size!==3||![1,4,395].every(k=>p.has(k)))return"protected-header-profile";
  if(p.get(1)!==ALG)return"algorithm-substitution";
  if(p.get(395)!==VDS)return"vds-substitution";
  if(!Buffer.isBuffer(p.get(4)))return"key-id-type";
  let kid;try{kid=p.get(4).toString("utf8");}catch{return"key-id-encoding";}
  const key=tsreg.keys?.[kid];if(!key||key.status!=="active"||key.algorithm!==ALG)return"unknown-or-inactive-ts-key";
  if(parsed.payload!==null)return"detached-payload-required";
  if(unp.size!==1||!unp.has(396)||!(unp.get(396) instanceof Map))return"unprotected-header-profile";
  const label=kind==="inclusion"?-1:-2,vdp=unp.get(396);
  if(vdp.size!==1||!vdp.has(label)||!Array.isArray(vdp.get(label))||vdp.get(label).length!==1)return"proof-label-or-count";
  const proofBlob=vdp.get(label)[0];if(!Buffer.isBuffer(proofBlob))return"proof-encoding";
  let proof;try{proof=decodeExact(proofBlob);}catch{return"cbor-decode";}
  if(Buffer.compare(cborEncode(proof),proofBlob)!==0)return"proof-noncanonical";
  if(!Array.isArray(proof)||proof.length!==3||!Array.isArray(proof[2]))return"proof-shape";
  let pub;try{pub=b64u(key.public_key);}catch{return"ts-public-key";}
  if(pub.length!==32)return"ts-public-key";
  const sig=parsed.signature;
  if(kind==="inclusion"){
    if(proof[0]!==fixture.tree_size||proof[1]!==fixture.leaf_index)return"inclusion-proof-context";
    const entryBytes=b64u(fixture.candidate_entry_base64url);
    if(Buffer.compare(Buffer.from(canonical(fixture.candidate_entry),"utf8"),entryBytes)!==0)return"candidate-entry-bytes-mismatch";
    if(canonical(fixture.candidate_entry)!==canonical(vds.entries[proof[1]]))return"candidate-entry-binding";
    const root=inclusionRoot(proof[1],proof[0],entryBytes,proof[2]);
    if(!root||root.toString("hex")!==fixture.root_hash.slice(7))return"inclusion-proof-invalid";
    const[q,qe]=validateHeadQuorum(heads,wreg,vds,"size_7");if(qe)return qe;
    if(q.tree_size!==proof[0]||Buffer.compare(q.root,root)!==0)return"head-quorum-binding";
    const ss=cborEncode(["Signature1",parsed.protectedBytes,Buffer.alloc(0),root]);
    if(!verifyEd(pub,sig,ss))return"cose-signature-invalid";
    return"receipt-inclusion-valid";
  }
  const[q7,e7]=validateHeadQuorum(heads,wreg,vds,"size_7");if(e7)return e7;
  const[q4,e4]=validateHeadQuorum(heads,wreg,vds,"size_4");if(e4)return e4;
  const newerRoot=q7.root;
  const ss=cborEncode(["Signature1",parsed.protectedBytes,Buffer.alloc(0),newerRoot]);
  if(!verifyEd(pub,sig,ss))return"cose-signature-invalid";
  if(proof[0]!==q4.tree_size||proof[1]!==q7.tree_size)return"consistency-proof-context";
  if(fixture.consistency_old_tree_size!==q4.tree_size||Buffer.from(fixture.consistency_old_root_hash.slice(7),"hex").compare(q4.root)!==0)return"old-head-binding";
  if(!consistencyValid(proof[0],proof[1],q4.root,newerRoot,proof[2]))return"consistency-proof-invalid";
  return"receipt-consistency-valid";
}
const args=process.argv.slice(2);if(args.length!==8){console.error("usage: verifier TS_REGISTRY COSE_FIXTURE CAMPAIGN WITNESS_REGISTRY TRUST_ROOT VDS_FIXTURE TREE_HEAD_FIXTURE REPORT");process.exit(2);}
const[tsP,fp,cp,wrp,trp,vdsp,hp,outp]=args;
const tsreg=JSON.parse(fs.readFileSync(tsP,"utf8")),fixture=JSON.parse(fs.readFileSync(fp,"utf8")),campaign=JSON.parse(fs.readFileSync(cp,"utf8")),wreg=JSON.parse(fs.readFileSync(wrp,"utf8")),trustRoot=JSON.parse(fs.readFileSync(trp,"utf8")),vds=JSON.parse(fs.readFileSync(vdsp,"utf8")),heads=JSON.parse(fs.readFileSync(hp,"utf8"));
if(fixture.schema!==FIXTURE_SCHEMA||campaign.schema!==CAMPAIGN_SCHEMA||campaign.case_count!==22||campaign.cases.length!==22)process.exit(1);
const ids=campaign.cases.map(x=>x.case_id);if(new Set(ids).size!==ids.length)process.exit(1);
for(const kind of ["inclusion","consistency"]){const r=verifyReceipt(kind,fixture,tsreg,wreg,vds,heads,trustRoot,Buffer.from(fixture[kind+"_receipt_cose_hex"],"hex"));if(r!==(kind==="inclusion"?"receipt-inclusion-valid":"receipt-consistency-valid")){console.error("baseline-"+kind+"="+r);process.exit(1);}}
const rows=[],failures=[];
for(const c of campaign.cases){
  const f=structuredClone(fixture),reg=structuredClone(tsreg),m=c.mutation;
  if(m==="fixture-root-substitution")f.root_hash="sha256:"+"a".repeat(64);
  if(m==="registry-key-substitution")reg.keys[TS_KEY_ID].public_key="AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA";
  const wire=m?mutateWire(f,c.kind,m):Buffer.from(f[c.kind+"_receipt_cose_hex"],"hex");
  const reason=verifyReceipt(c.kind,f,reg,wreg,vds,heads,trustRoot,wire);
  const verdict=["receipt-inclusion-valid","receipt-consistency-valid"].includes(reason)?"qualified":"unresolved";
  const row={case_id:c.case_id,expected_verdict:c.expected_verdict,actual_verdict:verdict,reason};rows.push(row);
  if(verdict!==c.expected_verdict)failures.push([c.case_id,c.expected_verdict,verdict,reason]);
}
fs.writeFileSync(outp,canonical({schema:"mycelix.continual-adaptation.censoring-classification-anchor-cose-receipt-report.v1",status:"research-evidence-only",case_count:rows.length,cases:rows,failures})+"\n");
console.log("cases="+rows.length+" failures="+failures.length);process.exit(failures.length?1:0);
