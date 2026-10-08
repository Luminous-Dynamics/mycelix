#!/usr/bin/env node
import fs from "node:fs";
import crypto from "node:crypto";

const SCHEMA="mycelix.continual-adaptation.censoring-classification-anchor-witness-vds-tree-head.v1";
const DOMAIN=SCHEMA,REGID="mycelix.research.anchor-witness-registry.v2",ALG="Ed25519";
const EXPECTED_REGISTRY_SHA="sha256:29eae4dcdec3709a3245efeb3d47e642a0bdc7f49b6f13041b6b42849088d5cf";
const VDS_ID="mycelix.research.anchor-statement-sequence.v1";

const canon=v=>v===null||typeof v!=="object"?JSON.stringify(v):Array.isArray(v)?"["+v.map(canon).join(",")+"]":"{"+Object.keys(v).sort().map(k=>JSON.stringify(k)+":"+canon(v[k])).join(",")+"}";
const H=b=>crypto.createHash("sha256").update(b).digest();
const node=(a,b)=>H(Buffer.concat([Buffer.from([1]),a,b]));
const leaf=e=>H(Buffer.concat([Buffer.from([0]),Buffer.from(canon(e),"utf8")]));
function mth(ds){const n=ds.length;if(n===0)return H(Buffer.alloc(0));if(n===1)return leaf(ds[0]);const k=1<<((n-1).toString(2).length-1);return node(mth(ds.slice(0,k)),mth(ds.slice(k)));}
function hh(s){if(typeof s!=="string"||!/^sha256:[0-9a-f]{64}$/.test(s))throw Error("hash");return Buffer.from(s.slice(7),"hex");}
function b64(s,n){if(typeof s!=="string"||!/^[A-Za-z0-9_-]+$/.test(s))throw Error("base64url");const b=Buffer.from(s,"base64url");if(n!==undefined&&b.length!==n)throw Error("length");return b;}
function digest(v){return "sha256:"+H(Buffer.from(canon(v),"utf8")).toString("hex");}
function pubKey(b){return crypto.createPublicKey({key:Buffer.concat([Buffer.from("302a300506032b6570032100","hex"),b]),format:"der",type:"spki"});}
function verify(pub,sig,msg){try{return crypto.verify(null,msg,pubKey(pub),sig)}catch{return false}}
function key(reg,w,kid,v){
  const k=reg.witnesses?.[w]?.keys?.[kid];if(!k)return[null,"unknown-key"];
  if(k.algorithm!==ALG)return[null,"key-algorithm"];
  if(v<k.valid_from_version)return[null,"key-not-yet-valid"];
  if(k.valid_until_version!==null&&v>k.valid_until_version)return[null,"key-expired-for-version"];
  if(k.revoked_at_version!==null&&v>=k.revoked_at_version)return[null,"key-revoked"];
  if(k.status==="revoked")return[null,"key-revoked"];
  try{return[b64(k.public_key,32),null]}catch{return[null,"key-public-key"]}
}
function payload(a,id){return{schema:SCHEMA,domain:DOMAIN,algorithm:ALG,observer_id:a.observer_id,key_id:a.key_id,witness_identity_commitment:id,registry_id:a.registry_id,registry_version:a.registry_version,vds_id:a.vds_id,manifest_version:a.manifest_version,tree_size:a.tree_size,root_hash:a.root_hash};}
function verifyHead(a,reg,entries){
  const fields=["schema","domain","algorithm","observer_id","key_id","registry_id","registry_version","vds_id","manifest_version","tree_size","root_hash","signature"].sort();
  if(!a||typeof a!=="object"||Array.isArray(a)||Object.keys(a).sort().join("|")!==fields.join("|"))return[null,"signature-wrapping-or-schema"];
  if(a.schema!==SCHEMA)return[null,"head-schema"];
  if(a.domain!==DOMAIN)return[null,"head-domain"];
  if(a.algorithm!==ALG)return[null,"head-algorithm"];
  if(a.registry_id!==REGID||a.registry_version!==2)return[null,"head-registry-binding"];
  if(a.vds_id!==VDS_ID)return[null,"head-vds-binding"];
  if(!reg.witnesses?.[a.observer_id])return[null,"unknown-witness"];
  if(!Number.isInteger(a.manifest_version)||a.manifest_version<1)return[null,"manifest-version"];
  if(!Number.isInteger(a.tree_size)||a.tree_size<1||a.tree_size>entries.length)return[null,"tree-size"];
  let root;try{root=hh(a.root_hash)}catch{return[null,"root-encoding"];}
  if(Buffer.compare(mth(entries.slice(0,a.tree_size)),root)!==0)return[null,"head-root-mismatch"];
  const [pub,e]=key(reg,a.observer_id,a.key_id,a.manifest_version);if(e)return[null,e];
  let sig;try{sig=b64(a.signature,64)}catch{return[null,"signature-encoding"];}
  return verify(pub,sig,Buffer.from(canon(payload(a,reg.witnesses[a.observer_id].identity_commitment)),"utf8"))?[a,null]:[null,"signature-invalid"];
}
function validateSet(map,reg,entries){
  if(Object.keys(map).length<3)return[null,"below-threshold"];
  const vals=[];
  for(const[w,a]of Object.entries(map)){const[v,e]=verifyHead(a,reg,entries);if(e)return[null,e];if(v.observer_id!==w)return[null,"observer-id-mismatch"];vals.push(v);}
  const target=new Set(vals.map(a=>JSON.stringify([a.manifest_version,a.tree_size,a.root_hash])));
  if(target.size!==1)return[null,"equivocation"];
  return[vals[0],null];
}
function consistency(m,n,first,second,path){
  if(!path.length||m<=0||m>=n)return false;
  let p=[...path];if((m&(m-1))===0)p=[first,...p];
  let fn=m-1,sn=n-1;while(fn&1){fn>>=1;sn>>=1;}
  let fr=p[0],sr=p[0];
  for(let i=1;i<p.length;i++){const c=p[i];if(sn===0)return false;if((fn&1)||fn===sn){fr=node(c,fr);sr=node(c,sr);if(!(fn&1))while(fn&&!((fn)&1)){fn>>=1;sn>>=1;}}else sr=node(sr,c);fn>>=1;sn>>=1;}
  return sn===0&&Buffer.compare(fr,first)===0&&Buffer.compare(sr,second)===0;
}
function evalCase(fixture,reg,c){
  const entries=fixture.entries,heads=fixture.heads,atts=structuredClone(heads[c.base].attestations);
  for(const w of c.remove_observers||[])delete atts[w];
  if(c.remove_observer)delete atts[c.remove_observer];
  if(c.replace_observer)atts[c.replace_observer]=structuredClone(heads[c.replace_with]);
  if(c.inject==="fork_size_4")atts.w02=structuredClone(heads.fork_size_4.attestation);
  if(c.duplicate_as)atts[c.duplicate_as]=structuredClone(heads.fork_size_4.attestation);
  if(c.mutate_observer)atts[c.mutate_observer][c.mutation]=c.mutation==="extra_field"?true:c.value;
  const[first,e1]=validateSet(atts,reg,entries);if(e1)return["unresolved",e1];
  if(c.second){
    const[second,e2]=validateSet(structuredClone(heads[c.second].attestations),reg,entries);if(e2)return["unresolved",e2];
    if(second.tree_size<first.tree_size)return["unresolved","rollback"];
    if(second.tree_size===first.tree_size)return first.root_hash===second.root_hash?["qualified","same-head"]:["unresolved","equivocation"];
    let p=fixture.consistency_proof_4_to_7.map(hh);if(c.proof_mutation==="replace-first")p[0]=H(p[0]);
    return consistency(first.tree_size,second.tree_size,hh(first.root_hash),hh(second.root_hash),p)?["qualified","consistency-proof"]:["unresolved","consistency-proof-invalid"];
  }
  return["qualified","authenticated-tree-head"];
}

const [regP,fixtureP,campaignP,outP]=process.argv.slice(2);if(!outP)process.exit(2);
const reg=JSON.parse(fs.readFileSync(regP,"utf8")),fixture=JSON.parse(fs.readFileSync(fixtureP,"utf8")),campaign=JSON.parse(fs.readFileSync(campaignP,"utf8"));
if(reg.registry_id!==REGID||reg.registry_version!==2||digest(reg)!==EXPECTED_REGISTRY_SHA)process.exit(1);
if(campaign.case_count!==21||!Array.isArray(campaign.cases)||campaign.cases.length!==21)process.exit(1);
const ids=campaign.cases.map(x=>x.case_id);if(new Set(ids).size!==ids.length)process.exit(1);
const rows=[],failures=[];
for(const c of campaign.cases){const[v,r]=evalCase(fixture,reg,c);const row={case_id:c.case_id,expected_verdict:c.expected_verdict,actual_verdict:v,reason:r};rows.push(row);if(v!==c.expected_verdict)failures.push([c.case_id,c.expected_verdict,v,r]);}
fs.writeFileSync(outP,canon({schema:"mycelix.continual-adaptation.censoring-classification-anchor-witness-vds-tree-head-report.v1",status:"research-evidence-only",case_count:rows.length,cases:rows,failures})+"\n");
console.log("cases="+rows.length+" failures="+failures.length);
process.exit(failures.length?1:0);
