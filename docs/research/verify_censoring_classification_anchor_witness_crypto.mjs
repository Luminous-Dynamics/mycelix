#!/usr/bin/env node
import fs from "node:fs";
import crypto from "node:crypto";

const ROOT="mycelix.continual-adaptation.censoring-classification-anchor-witness-trust-root.v2";
const REG="mycelix.continual-adaptation.censoring-classification-anchor-witness-registry.v2";
const CP="mycelix.continual-adaptation.censoring-classification-anchor-witness-checkpoint.v3";
const CAMP="mycelix.continual-adaptation.censoring-classification-anchor-witness-crypto-campaign.v1";
const SIG="mycelix.continual-adaptation.censoring-classification-anchor-witness-signature.v2";
const REGID="mycelix.research.anchor-witness-registry.v2", ROOTID="mycelix.research.anchor-witness-root.v2";
const AUTH="mycelix.research.anchor-authority.v1", ALG="Ed25519";
const DOMAIN="mycelix.continual-adaptation.censoring-classification-anchor-witness-attestation.v3";

const canon=v=>v===null||typeof v!=="object"?JSON.stringify(v):Array.isArray(v)?"["+v.map(canon).join(",")+"]":"{"+Object.keys(v).sort().map(k=>JSON.stringify(k)+":"+canon(v[k])).join(",")+"}";
const digest=v=>"sha256:"+crypto.createHash("sha256").update(canon(v)).digest("hex");
const b64=v=>{if(typeof v!=="string"||!/^[A-Za-z0-9_-]+$/.test(v)||v.length!==Math.ceil(4*v.length/4)){} const b=Buffer.from(v,"base64url"); return b;};
const pubkey=b=>crypto.createPublicKey({key:Buffer.concat([Buffer.from("302a300506032b6570032100","hex"),b]),format:"der",type:"spki"});
const verify=(pub,sig,msg)=>crypto.verify(null,msg,pubkey(pub),sig);

const [expected,rootP,regP,baseP,fwdP,campP,outP]=process.argv.slice(2);
if(!outP) process.exit(2);
const root=JSON.parse(fs.readFileSync(rootP)),reg=JSON.parse(fs.readFileSync(regP)),base=JSON.parse(fs.readFileSync(baseP)),fwd=JSON.parse(fs.readFileSync(fwdP)),camp=JSON.parse(fs.readFileSync(campP));
const result=(v,r)=>({actual_verdict:v,reason:r});
const key=(w,kid,v)=>{
  const wi=reg.witnesses?.[w]; if(!wi)return[null,"unknown-witness"];
  const k=wi.keys?.[kid]; if(!k)return[null,"unknown-key"];
  if(k.algorithm!==ALG)return[null,"key-algorithm"];
  if(v<k.valid_from_version)return[null,"key-not-yet-valid"];
  if(k.valid_until_version!==null&&v>k.valid_until_version)return[null,"key-expired-for-version"];
  if(k.revoked_at_version!==null&&v>=k.revoked_at_version)return[null,"key-revoked"];
  if(k.status==="revoked")return[null,"key-revoked"];
  return[Buffer.from(k.public_key,"base64url"),null];
};
const att=(cp,w,a)=>{
  const keys=["schema","witness_id","key_id","algorithm","domain","claims","signature"];
  if(!a||typeof a!=="object"||Array.isArray(a)||Object.keys(a).sort().join("|")!==keys.slice().sort().join("|"))return[null,"signature-wrapping-or-schema"];
  if(a.schema!==SIG)return[null,"attestation-schema"];
  if(a.witness_id!==w)return[null,"witness-id-mismatch"];
  if(a.algorithm!==ALG)return[null,"algorithm-substitution"];
  if(a.domain!==DOMAIN)return[null,"domain-separation"];
  const c=a.claims, req=["registry_id","registry_version","root_reference_sha256","authority_id","manifest_version","manifest_sha256","previous_manifest_sha256"];
  if(!c||typeof c!=="object"||Object.keys(c).sort().join("|")!==req.sort().join("|"))return[null,"claims-schema"];
  if(c.registry_id!==REGID||c.registry_version!==reg.registry_version)return[null,"claims-registry-binding"];
  if(c.root_reference_sha256!==cp.root_reference_sha256||c.root_reference_sha256!==digest(root))return[null,"claims-root-binding"];
  if(c.authority_id!==AUTH)return[null,"claims-authority"];
  const [pub,e]=key(w,a.key_id,c.manifest_version); if(e)return[null,e];
  let sig; try{sig=Buffer.from(a.signature,"base64url");}catch{return[null,"signature-encoding"];}
  if(sig.length!==64)return[null,"signature-encoding"];
  const p={schema:SIG,domain:DOMAIN,algorithm:ALG,witness_id:w,key_id:a.key_id,witness_identity_commitment:reg.witnesses[w].identity_commitment,claims:c};
  return verify(pub,sig,Buffer.from(canon(p)))?[c,null]:[null,"signature-invalid"];
};
const consensus=cp=>{
  const ids=Object.keys(cp.witness_attestations||{}); if(ids.length<reg.threshold)return[null,"below-threshold"];
  const claims=[];
  for(const w of ids){if(!reg.witnesses?.[w])return[null,"unknown-witness"];const [c,e]=att(cp,w,cp.witness_attestations[w]);if(e)return[null,e];claims.push(c);}
  const t=new Set(claims.map(c=>JSON.stringify([c.authority_id,c.manifest_version,c.manifest_sha256,c.previous_manifest_sha256,c.root_reference_sha256])));
  return t.size===1?[claims[0],"ok"]:[null,"equivocation"];
};
const evaluate=c=>{
  const cp=structuredClone(c.base==="candidate"?fwd:base);
  const w=c.witness;
  if(c.remove_witness)delete cp.witness_attestations[c.remove_witness];
  for(const x of c.remove_witnesses||[])delete cp.witness_attestations[x];
  if(c.replacement_attestation&&!c.fork_witness)cp.witness_attestations[w]=structuredClone(c.replacement_attestation);
  if(c.fork_witness)cp.witness_attestations[c.fork_witness]=structuredClone(c.replacement_attestation);
  if(c.mutate_key_id)cp.witness_attestations[w].key_id=c.mutate_key_id;
  if(c.mutate_domain)cp.witness_attestations[w].domain=c.mutate_domain;
  if(c.mutate_algorithm)cp.witness_attestations[w].algorithm=c.mutate_algorithm;
  if(c.extra_field)cp.witness_attestations[w][c.extra_field]=true;
  if(c.apply_manifest)cp.witness_attestations[w].claims.manifest_sha256=c.apply_manifest;
  if(c.apply_predecessor)cp.witness_attestations[w].claims.previous_manifest_sha256=c.apply_predecessor;
  if(c.root_threshold!==undefined)root.threshold=c.root_threshold;
  if(c.root_registry_sha256!==undefined)root.registry_sha256=c.root_registry_sha256;
  const [claim,reason]=consensus(cp);
  if(!claim)return result("unresolved",reason);
  if(c.base==="candidate"){
    const [bc,br]=consensus(base); if(!bc)return result("unresolved","baseline-"+br);
    if(claim.manifest_version!==bc.manifest_version+1)return result("unresolved","forward-version");
    if(claim.previous_manifest_sha256!==bc.manifest_sha256)return result("unresolved","forward-predecessor");
  }
  return result("qualified","ok");
};
// Standard Ed25519 known-answer check (public vector + signature only).
const katPub=Buffer.from("d75a980182b10ab7d54bfed3c964073a0ee172f3daa62325af021a68f707511a","hex");
const katSig=Buffer.from("e5564300c360ac729086e2cc806e828a84877f1eb8e5d974d873e065224901555fb8821590a33bacc61e39701cf9b46bd25bf5f0595bbe24655141438e7a100b","hex");
if(!verify(katPub,katSig,Buffer.alloc(0)))process.exit(1);

const rows=[],failures=[];
for(const c of camp.cases){const r=evaluate(c),row={case_id:c.case_id,expected_verdict:c.expected_verdict,actual_verdict:r.actual_verdict,reason:r.reason};rows.push(row);if(row.actual_verdict!==row.expected_verdict)failures.push([row.case_id,row.expected_verdict,row.actual_verdict,row.reason]);}
fs.writeFileSync(outP,canon({schema:"mycelix.continual-adaptation.censoring-classification-anchor-witness-crypto-report.v2",status:"research-evidence-only",case_count:rows.length,ed25519_known_answer_test:"pass",cases:rows,failures})+"\n");
console.log("cases="+rows.length+" failures="+failures.length);
process.exit(failures.length?1:0);
