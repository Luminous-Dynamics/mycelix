import fs from 'node:fs';
import crypto from 'node:crypto';

const REG_ID='mycelix.research.anchor-receipt-ts-registry.v1';
const TS_ID='mycelix.research.anchor-receipt-ts.v1';
const SCHEMA='mycelix.continual-adaptation.censoring-classification-anchor-receipt.v1';
const VDS_ID='mycelix.research.anchor-statement-sequence.v1';
const WREG='mycelix.research.anchor-witness-registry.v2';
const EXPECTED_TS_REG_SHA='sha256:efb549a023660010a07d70b7d78afbfd81664200fe4ecf1d0d542dc945f0bb54';

const canon=v=>v===null||typeof v!=='object'?JSON.stringify(v):Array.isArray(v)?'['+v.map(canon).join(',')+']':'{'+Object.keys(v).sort().map(k=>JSON.stringify(k)+':'+canon(v[k]))+'}';
const H=b=>crypto.createHash('sha256').update(b).digest();
const digest=v=>'sha256:'+H(Buffer.from(canon(v),'utf8')).toString('hex');
const leaf=e=>H(Buffer.concat([Buffer.from([0]),Buffer.from(canon(e),'utf8')]));
const node=(a,b)=>H(Buffer.concat([Buffer.from([1]),a,b]));
function mth(ds){const n=ds.length;if(n===0)return H(Buffer.alloc(0));if(n===1)return leaf(ds[0]);const k=1<<((n-1).toString(2).length-1);return node(mth(ds.slice(0,k)),mth(ds.slice(k)));}

function hh(s){if(typeof s!=='string'||!/^sha256:[0-9a-f]{64}$/.test(s))throw Error('hash');return Buffer.from(s.slice(7),'hex');}
function b64(s,n){if(typeof s!=='string'||!/^[A-Za-z0-9_-]+$/.test(s))throw Error('base64url');const b=Buffer.from(s,'base64url');if(n!==undefined&&b.length!==n)throw Error('length');return b;}
function pk(b){return crypto.createPublicKey({key:Buffer.concat([Buffer.from('302a300506032b6570032100','hex'),b]),format:'der',type:'spki'});}
function verify(pub,sig,msg){try{return crypto.verify(null,msg,pk(pub),sig)}catch{return false}}

function keyFor(reg,w,kid,v){const k=reg.witnesses?.[w]?.keys?.[kid];if(!k)return[null,'unknown-key'];if(k.algorithm!=='Ed25519')return[null,'key-algorithm'];if(v<k.valid_from_version)return[null,'key-not-yet-valid'];if(k.valid_until_version!==null&&v>k.valid_until_version)return[null,'key-expired-for-version'];if(k.revoked_at_version!==null&&v>=k.revoked_at_version)return[null,'key-revoked'];if(k.status==='revoked')return[null,'key-revoked'];return[b64(k.public_key,32),null];}

function verifyHead(a,w,reg,entries){
  const fields=['schema','domain','algorithm','observer_id','key_id','registry_id','registry_version','vds_id','manifest_version','tree_size','root_hash','signature'].sort();
  if(!a||typeof a!=='object'||Object.keys(a).sort().join('|')!==fields.join('|'))return[null,'head-schema'];
  if(a.schema!=='mycelix.continual-adaptation.censoring-classification-anchor-witness-vds-tree-head.v1')return[null,'head-schema'];\n  if(a.domain!=='mycelix.continual-adaptation.censoring-classification-anchor-witness-vds-tree-head.v1')return[null,'head-domain'];\n  if(a.algorithm!=='Ed25519')return[null,'head-algorithm'];\n  if(a.observer_id!==w)return[null,'head-observer-binding'];
  if(a.registry_id!==WREG||a.registry_version!==2)return[null,'head-registry-binding'];
  if(a.vds_id!==VDS_ID||a.tree_size!==7)return[null,'head-vds-binding'];
  let root;try{root=hh(a.root_hash)}catch{return[null,'head-root-encoding'];}
  if(Buffer.compare(mth(entries.slice(0,7)),root)!==0)return[null,'head-root-mismatch'];
  const[pub,e]=keyFor(reg,w,a.key_id,a.manifest_version);if(e)return[null,e];
  let sig;try{sig=b64(a.signature,64)}catch{return[null,'head-signature-encoding'];}
  const p={schema:a.schema,domain:a.domain,algorithm:a.algorithm,observer_id:w,key_id:a.key_id,witness_identity_commitment:reg.witnesses[w].identity_commitment,registry_id:a.registry_id,registry_version:a.registry_version,vds_id:a.vds_id,manifest_version:a.manifest_version,tree_size:a.tree_size,root_hash:a.root_hash};
  return verify(pub,sig,Buffer.from(canon(p),'utf8'))?[a,null]:[null,'head-signature-invalid'];
}

function headQuorum(headFixture,reg,entries,expectedRoot){
  const vals=[];
  for(const[w,a]of Object.entries(headFixture.heads.size_7.attestations)){
    const[v,e]=verifyHead(a,w,reg,entries);if(e)return[null,e];vals.push(v);
  }
  if(vals.length<3)return[null,'head-below-threshold'];
  const target=new Set(vals.map(a=>JSON.stringify([a.manifest_version,a.tree_size,a.root_hash])));if(target.size!==1)return[null,'head-equivocation'];
  const q=digest({tree_size:7,root_hash:expectedRoot,head_digests:vals.map(digest).sort()});
  if(headFixture.tree_head_quorum_digest&&q!==headFixture.tree_head_quorum_digest)return[null,'head-quorum-digest'];
  return[vals[0],null];
}

function verifyReceipt(r,ts,f){
  const fields=['schema','domain','algorithm','ts_id','key_id','claims','signature'].sort();
  if(!r||typeof r!=='object'||Object.keys(r).sort().join('|')!==fields.join('|'))return[null,'receipt-schema'];
  if(r.schema!==SCHEMA)return[null,'receipt-schema'];
  if(r.domain!==SCHEMA)return[null,'receipt-domain'];
  if(r.algorithm!=='Ed25519')return[null,'receipt-algorithm'];
  if(r.ts_id!==TS_ID||!ts.keys?.[r.key_id])return[null,'receipt-ts-binding'];
  const c=r.claims,req=['registry_id','registry_version','ts_id','vds_id','statement_id','statement_hash','manifest_version','tree_size','leaf_index','root_hash','tree_head_quorum_digest'].sort();
  if(!c||typeof c!=='object'||Object.keys(c).sort().join('|')!==req.join('|'))return[null,'receipt-claims-schema'];
  if(c.registry_id!==ts.registry_id||c.registry_version!==ts.registry_version)return[null,'receipt-registry-binding'];
  if(c.ts_id!==TS_ID||c.vds_id!==VDS_ID)return[null,'receipt-vds-binding'];
  if(c.tree_size!==f.tree_size||c.root_hash!==f.root_hash)return[null,'receipt-head-binding'];\n  if(ts.algorithm!=='Ed25519'||ts.keys[r.key_id].status!=='active')return[null,'receipt-key-lifecycle'];
  let sig;try{sig=b64(r.signature,64)}catch{return[null,'signature-encoding'];}
  if(!verify(b64(ts.keys[r.key_id].public_key,32),sig,Buffer.from(canon({schema:r.schema,domain:r.domain,algorithm:r.algorithm,ts_id:r.ts_id,key_id:r.key_id,claims:c}),'utf8')))return[null,'signature-invalid'];
  const i=c.leaf_index;if(!Number.isInteger(i)||i<0||i>=c.tree_size)return[null,'leaf-index'];
  const e=f.entries[i];
  if(c.statement_id!==e.statement_id||c.statement_hash!=='sha256:'+leaf(e).toString('hex'))return[null,'statement-binding'];
  if(c.manifest_version!==e.manifest_version)return[null,'manifest-binding'];
  return[c,null];
}

function inclusion(index,size,leafHash,root,path){
  if(size<=0||index<0||index>=size)return false;
  let fn=index,sn=size-1,r=leafHash;
  for(const p of path){if(sn===0)return false;if((fn&1)||fn===sn){r=node(p,r);if(!(fn&1))while(fn&&!((fn)&1)){fn>>=1;sn>>=1;}}else r=node(r,p);fn>>=1;sn>>=1;}
  return sn===0&&Buffer.compare(r,root)===0;
}

const args=process.argv.slice(2);if(args.length!==5){console.error('usage: receipt-verifier TS_REGISTRY RECEIPT_FIXTURE CAMPAIGN WITNESS_REGISTRY REPORT');process.exit(2);}
const[tsP,fP,cP,wP,outP]=args;
const ts=JSON.parse(fs.readFileSync(tsP,'utf8')),f=JSON.parse(fs.readFileSync(fP,'utf8')),camp=JSON.parse(fs.readFileSync(cP,'utf8')),wreg=JSON.parse(fs.readFileSync(wP,'utf8'));
if(ts.registry_id!==REG_ID||ts.registry_version!==1||digest(ts)!==EXPECTED_TS_REG_SHA||f.ts_registry_sha256!==EXPECTED_TS_REG_SHA||camp.case_count!==16||camp.cases.length!==16||!headFixture.heads?.size_7?.attestations)process.exit(1);
const rows=[],failures=[];
for(const c of camp.cases){
  const r=structuredClone(f.receipts[String(c.receipt_id??0)]);
  for(const[field,val]of c.mutate_receipt??[]){if(field in r)r[field]=val;else r.claims[field]=val;}
  const[rc,re]=verifyReceipt(r,ts,f);
  let verdict='unresolved',reason=re||null;
  if(!re){
    const[,he]=headQuorum(f,wreg);
    if(he)reason=he;
    else{
      let p=f.inclusion_proofs[String(rc.leaf_index)].map(hh);
      if(c.proof_mutation==='replace-first')p[0]=H(p[0]);else if(c.proof_mutation==='truncate')p=p.slice(0,-1);else if(c.proof_mutation==='append-extra')p.push(H(Buffer.from('extra')));else if(c.proof_mutation==='empty')p=[];
      if(inclusion(rc.leaf_index,rc.tree_size,hh(rc.statement_hash),hh(rc.root_hash),p)){verdict='qualified';reason='receipt-and-inclusion-proof';}else reason='inclusion-proof-invalid';
    }
  }
  const row={case_id:c.case_id,expected_verdict:c.expected_verdict,actual_verdict:verdict,reason};rows.push(row);if(verdict!==c.expected_verdict)failures.push([c.case_id,c.expected_verdict,verdict,reason]);
}
fs.writeFileSync(outP,canon({schema:'mycelix.continual-adaptation.censoring-classification-anchor-receipt-report.v1',status:'research-evidence-only',case_count:16,cases:rows,failures})+'\n');
console.log('cases='+rows.length+' failures='+failures.length);process.exit(failures.length?1:0);
