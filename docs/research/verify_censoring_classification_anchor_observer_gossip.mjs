#!/usr/bin/env node
import fs from "node:fs";
import crypto from "node:crypto";

const GREG_SCHEMA="mycelix.continual-adaptation.censoring-classification-anchor-observer-gossip-registry.v1";
const OBS_SCHEMA="mycelix.continual-adaptation.censoring-classification-anchor-observer-gossip-observation.v1";
const GDOMAIN="mycelix.continual-adaptation.censoring-classification-anchor-observer-gossip.v1";
const HEAD_SCHEMA="mycelix.continual-adaptation.censoring-classification-anchor-witness-vds-tree-head.v1";
const GREG_ID="mycelix.research.anchor-observer-gossip-registry.v1";
const WREG_ID="mycelix.research.anchor-witness-registry.v2";
const VDS_ID="mycelix.research.anchor-statement-sequence.v1";
const EXPECTED_GREG_SHA="sha256:df41b280e6250f28425f2944f20e417bb69fa522eed9c52ca6b45d6b7931dd8e";
const ALG="Ed25519";

const canon=v=>v===null||typeof v!=="object"?JSON.stringify(v):Array.isArray(v)?"["+v.map(canon).join(",")+"]":"{"+Object.keys(v).sort().map(k=>JSON.stringify(k)+":"+canon(v[k])).join(",")+"}";
const H=b=>crypto.createHash("sha256").update(b).digest();
const digest=v=>"sha256:"+H(Buffer.from(canon(v),"utf8")).toString("hex");
const b64=s=>Buffer.from(s,"base64url");
const pk=b=>crypto.createPublicKey({key:Buffer.concat([Buffer.from("302a300506032b6570032100","hex"),b]),format:"der",type:"spki"});
const verify=(pub,sig,msg)=>crypto.verify(null,msg,pk(pub),sig);
const node=(a,b)=>H(Buffer.concat([Buffer.from([1]),a,b]));

function keyFor(reg,w,kid,v){
  const k=reg.witnesses?.[w]?.keys?.[kid]; if(!k)return[null,"unknown-witness-or-key"];
  if(k.algorithm!==ALG)return[null,"key-algorithm"];
  if(v<k.valid_from_version)return[null,"key-not-yet-valid"];
  if(k.valid_until_version!==null&&v>k.valid_until_version)return[null,"key-expired-for-version"];
  if(k.revoked_at_version!==null&&v>=k.revoked_at_version)return[null,"key-revoked"];
  if(k.status==="revoked")return[null,"key-revoked"];
  try{return[b64(k.public_key),null]}catch{return[null,"key-public-key"];}
}
function verifySubjectHead(h,reg){
  const fields=["schema","domain","algorithm","observer_id","key_id","registry_id","registry_version","vds_id","manifest_version","tree_size","root_hash","signature"].sort();
  if(!h||typeof h!=="object"||Object.keys(h).sort().join("|")!==fields.join("|"))return[false,"head-schema"];
  if(h.schema!==HEAD_SCHEMA||h.domain!==HEAD_SCHEMA)return[false,"head-schema-or-domain"];
  if(h.algorithm!==ALG)return[false,"head-algorithm"];
  if(h.registry_id!==WREG_ID||h.registry_version!==2)return[false,"head-registry-binding"];
  if(h.vds_id!==VDS_ID)return[false,"head-vds-binding"];
  const wi=reg.witnesses?.[h.observer_id];if(!wi)return[false,"unknown-witness"];
  const [pub,e]=keyFor(reg,h.observer_id,h.key_id,h.manifest_version);if(e)return[false,e];
  let sig;try{sig=b64(h.signature)}catch{return[false,"head-signature-encoding"];}
  if(sig.length!==64)return[false,"head-signature-encoding"];
  const p={schema:HEAD_SCHEMA,domain:HEAD_SCHEMA,algorithm:ALG,observer_id:h.observer_id,key_id:h.key_id,witness_identity_commitment:wi.identity_commitment,registry_id:h.registry_id,registry_version:h.registry_version,vds_id:h.vds_id,manifest_version:h.manifest_version,tree_size:h.tree_size,root_hash:h.root_hash};
  return verify(pub,sig,Buffer.from(canon(p),"utf8"))?[true,null]:[false,"head-signature-invalid"];
}
function verifyRegistry(r){
  if(r.schema!==GREG_SCHEMA||r.status!=="research-fixture-only")return"registry-schema";
  if(r.registry_id!==GREG_ID||r.registry_version!==1)return"registry-identity";
  if(r.algorithm!==ALG||r.domain!==GDOMAIN)return"registry-profile";
  if(!r.monitors||typeof r.monitors!=="object"||Array.isArray(r.monitors)||!Object.keys(r.monitors).length)return"registry-monitors";
  const seen=new Set();
  for(const[m,v]of Object.entries(r.monitors)){
    if(v.status!=="active"||seen.has(v.key_id))return"monitor-schema";
    seen.add(v.key_id);try{if(b64(v.public_key).length!==32)throw 0}catch{return"monitor-public-key";}
  }
  return null;
}
function verifyObservation(o,reg){
  const fields=["schema","domain","algorithm","monitor_id","key_id","registry_id","registry_version","vds_id","subject_observer_id","subject_head_sha256","observed_tree_size","observed_root_hash","observation_sequence","signature"].sort();
  if(!o||typeof o!=="object"||Object.keys(o).sort().join("|")!==fields.join("|"))return[null,"gossip-envelope"];
  if(o.schema!==OBS_SCHEMA)return[null,"gossip-schema"];
  if(o.domain!==GDOMAIN)return[null,"gossip-domain"];
  if(o.algorithm!==ALG)return[null,"gossip-algorithm"];
  if(o.registry_id!==GREG_ID||o.registry_version!==1)return[null,"gossip-registry-binding"];
  if(o.vds_id!==VDS_ID)return[null,"gossip-vds-binding"];
  const m=reg.monitors?.[o.monitor_id];if(!m||m.status!=="active"||m.key_id!==o.key_id)return[null,"monitor-key"];
  let pub,sig;try{pub=b64(m.public_key);sig=b64(o.signature);}catch{return[null,"gossip-signature-encoding"];}
  if(pub.length!==32||sig.length!==64)return[null,"gossip-signature-encoding"];
  const p={...o};delete p.signature;
  return verify(pub,sig,Buffer.from(canon(p),"utf8"))?[o,null]:[null,"gossip-signature-invalid"];
}
function lookupHead(f,o){
  return Object.values(f.subject_heads||{}).find(h=>digest(h)===o.subject_head_sha256);
}
function authenticated(o,greg,wreg,fixture){
  const[,e]=verifyObservation(o,greg);if(e)return[null,e];
  const h=lookupHead(fixture,o);if(!h)return[null,"unknown-subject-head"];
  if(h.observer_id!==o.subject_observer_id||h.tree_size!==o.observed_tree_size||h.root_hash!==o.observed_root_hash)return[null,"subject-claim-binding"];
  const[,he]=verifySubjectHead(h,wreg);if(he)return[null,he];
  return[h,null];
}
function consistency(firstSize,secondSize,firstHash,secondHash,path){
  if(!path.length||!(0<firstSize&&firstSize<secondSize))return false;
  let p=[...path];if((firstSize&(firstSize-1))===0)p=[firstHash,...p];
  let fn=firstSize-1,sn=secondSize-1;
  while(fn&1){fn>>=1;sn>>=1;}
  let fr=p[0],sr=p[0];
  for(let i=1;i<p.length;i++){
    const c=p[i];if(sn===0)return false;
    if((fn&1)||fn===sn){
      fr=node(c,fr);sr=node(c,sr);
      if(!(fn&1))while(fn&&!((fn)&1)){fn>>=1;sn>>=1;}
    }else sr=node(sr,c);
    fn>>=1;sn>>=1;
  }
  return sn===0&&Buffer.compare(fr,firstHash)===0&&Buffer.compare(sr,secondHash)===0;
}
function evaluate(c,fixture,greg,wreg){
  const obs=structuredClone(fixture.observations);
  const t=c.subject||c.left;
  if(c.mutate){
    const o=obs[t];
    if(c.mutate==="root")o.observed_root_hash="sha256:"+"a".repeat(64);
    else if(c.mutate==="domain")o.domain="mycelix.attacker.v1";
    else if(c.mutate==="key_id")o.key_id="m99-k1";
    else if(c.mutate==="monitor_id")o.monitor_id="m99";
    else if(c.mutate==="subject_observer_id")o.subject_observer_id="w02";
    else if(c.mutate==="subject_head_sha256")o.subject_head_sha256="sha256:"+"b".repeat(64);
    else if(c.mutate==="vds_id")o.vds_id="mycelix.attacker.vds";
  }
  if(c.add_field)obs[c.subject].evil=true;
  if(c.same_monitor){obs[c.right].monitor_id=obs[c.left].monitor_id;obs[c.right].key_id=obs[c.left].key_id;}
  if(c.same_observation)obs[c.right]=structuredClone(obs[c.left]);
  if(c.kind==="single"){
    const[,e]=authenticated(obs[c.subject],greg,wreg,fixture);return e?["unresolved",e]:["qualified","authenticated-observation"];
  }
  const l=obs[c.left],r=obs[c.right];
  if(l.monitor_id===r.monitor_id)return["unresolved","monitor-independence"];
  const[lh,le]=authenticated(l,greg,wreg,fixture),[rh,re]=authenticated(r,greg,wreg,fixture);
  if(le)return["unresolved",le];if(re)return["unresolved",re];
  if(l.observed_tree_size===r.observed_tree_size)return l.observed_root_hash===r.observed_root_hash?["qualified","same-head-cross-observer"]:["unresolved","split-view"];
  if(r.observed_tree_size<l.observed_tree_size)return["unresolved","rollback"];
  if(c.drop_proof)return["unresolved","missing-consistency-proof"];
  let p=[Buffer.from(fixture.consistency_proof_4_to_7[0].slice(7),"hex")];
  if(c.mutate_proof==="replace-first")p[0]=H(p[0]);
  if(l.observed_tree_size!==4||r.observed_tree_size!==7)return["unresolved","unsupported-consistency-pair"];
  return consistency(l.observed_tree_size,r.observed_tree_size,Buffer.from(l.observed_root_hash.slice(7),"hex"),Buffer.from(r.observed_root_hash.slice(7),"hex"),p)?["qualified","cross-observer-consistency"]:["unresolved","consistency-proof-invalid"];
}
const args=process.argv.slice(2);if(args.length!==5){console.error("usage: verifier GOSSIP_REGISTRY FIXTURE CAMPAIGN WITNESS_REGISTRY REPORT");process.exit(2);}
const[gp,fp,cp,wp,op]=args;const greg=JSON.parse(fs.readFileSync(gp,"utf8")),fixture=JSON.parse(fs.readFileSync(fp,"utf8")),campaign=JSON.parse(fs.readFileSync(cp,"utf8")),wreg=JSON.parse(fs.readFileSync(wp,"utf8"));
if(verifyRegistry(greg)||digest(greg)!==EXPECTED_GREG_SHA||fixture.schema!=="mycelix.continual-adaptation.censoring-classification-anchor-observer-gossip-fixture.v1"||campaign.schema!=="mycelix.continual-adaptation.censoring-classification-anchor-observer-gossip-campaign.v1"||campaign.case_count!==16||campaign.cases.length!==16||campaign.expected_gossip_registry_sha256!==EXPECTED_GREG_SHA||campaign.witness_registry_id!==WREG_ID)process.exit(1);
const rows=[],failures=[];for(const c of campaign.cases){const[v,r]=evaluate(c,fixture,greg,wreg);const row={case_id:c.case_id,expected_verdict:c.expected_verdict,actual_verdict:v,reason:r};rows.push(row);if(v!==c.expected_verdict)failures.push([c.case_id,c.expected_verdict,v,r]);}
fs.writeFileSync(op,canon({schema:"mycelix.continual-adaptation.censoring-classification-anchor-observer-gossip-report.v1",status:"research-evidence-only",case_count:rows.length,cases:rows,failures})+"\n");console.log("cases="+rows.length+" failures="+failures.length);process.exit(failures.length?1:0);
