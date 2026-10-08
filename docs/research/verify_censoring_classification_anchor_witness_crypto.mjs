#!/usr/bin/env node
import fs from "node:fs";
import crypto from "node:crypto";

const ROOT_SCHEMA="mycelix.continual-adaptation.censoring-classification-anchor-witness-trust-root.v2";
const REG_SCHEMA="mycelix.continual-adaptation.censoring-classification-anchor-witness-registry.v2";
const CP_SCHEMA="mycelix.continual-adaptation.censoring-classification-anchor-witness-checkpoint.v3";
const CAMP_SCHEMA="mycelix.continual-adaptation.censoring-classification-anchor-witness-crypto-campaign.v1";
const SIG_SCHEMA="mycelix.continual-adaptation.censoring-classification-anchor-witness-signature.v2";
const REG_ID="mycelix.research.anchor-witness-registry.v2", ROOT_ID="mycelix.research.anchor-witness-root.v2";
const AUTH_ID="mycelix.research.anchor-authority.v1", ALG="Ed25519";
const DOMAIN="mycelix.continual-adaptation.censoring-classification-anchor-witness-attestation.v3";

function canonical(v){
  if(v===null||typeof v!=="object") return JSON.stringify(v);
  if(Array.isArray(v)) return "["+v.map(canonical).join(",")+"]";
  return "{"+Object.keys(v).sort().map(k=>JSON.stringify(k)+":"+canonical(v[k])).join(",")+"}";
}
function digest(v){return "sha256:"+crypto.createHash("sha256").update(Buffer.from(canonical(v),"utf8")).digest("hex");}
function b64d(s,n){
  if(typeof s!=="string"||!/^[A-Za-z0-9_-]+$/.test(s)||s.length!==Math.ceil(4*n/3))throw Error("base64url");
  const b=Buffer.from(s,"base64url");if(b.length!==n)throw Error("base64url-length");return b;
}
function pubKey(b){return crypto.createPublicKey({key:Buffer.concat([Buffer.from("302a300506032b6570032100","hex"),b]),format:"der",type:"spki"});}
function verifySig(pub,sig,msg){return crypto.verify(null,msg,pubKey(pub),sig);}

function validateRegistry(reg){
  if(reg.schema!==REG_SCHEMA||reg.status!=="research-witness-crypto-registry-only")return"registry-schema";
  if(reg.registry_id!==REG_ID||reg.registry_version!==2)return"registry-identity";
  const {threshold:q,max_faulty:f,witnesses:ws}=reg;
  if(!Number.isInteger(q)||!Number.isInteger(f)||!ws||typeof ws!=="object"||Array.isArray(ws))return"registry-quorum";
  const n=Object.keys(ws).length;if(n===0||q<1||q>n||f<0||2*q<=n+f)return"registry-quorum-intersection";
  if(JSON.stringify(reg.signature_profile)!==JSON.stringify({algorithm:ALG,encoding:"base64url-no-padding",domain:DOMAIN}))return"signature-profile";
  const seen=new Set(),publicSeen=new Set();
  for(const [w,wi] of Object.entries(ws)){
    if(wi.role!=="anchor-witness"||typeof wi.identity_commitment!=="string"||!wi.keys||typeof wi.keys!=="object"||Array.isArray(wi.keys)||!Object.keys(wi.keys).length)return"witness-schema";
    for(const [kid,k] of Object.entries(wi.keys)){
      if(seen.has(kid))return"duplicate-key-id";seen.add(kid);
      if(k.algorithm!==ALG)return"key-algorithm";
      let raw;try{raw=b64d(k.public_key,32);}catch{return"key-public-key";}
      const tag=raw.toString("hex");if(publicSeen.has(tag))return"duplicate-public-key";publicSeen.add(tag);
      const {valid_from_version:vf,valid_until_version:vu,revoked_at_version:rv}=k;
      if(!Number.isInteger(vf)||vf<1)return"key-valid-from";
      if(vu!==null&&(!Number.isInteger(vu)||vu<vf))return"key-valid-until";
      if(rv!==null&&!Number.isInteger(rv))return"key-revoked-at";
      if(!["active","retired","revoked"].includes(k.status))return"key-status";
      if(k.status==="active"&&(vu!==null||rv!==null))return"active-key-bounds";
      if((k.status==="retired"||k.status==="revoked")&&vu===null)return"bounded-key-missing-end";
      if(k.status==="revoked"&&rv===null)return"revoked-key-missing-revocation";
      if(k.status==="retired"&&rv!==null)return"retired-revocation-conflict";
      if(k.status!=="revoked"&&rv!==null)return"non-revoked-has-revocation";
      if(k.supersedes!==null&&k.supersedes!==undefined){
        const old=wi.keys[k.supersedes];if(!old)return"missing-superseded-key";
        if(old.valid_until_version===null||vf<=old.valid_until_version)return"invalid-rotation-lineage";
        if(old.superseded_by!==kid)return"asymmetric-successor-lineage";
      }
      if(k.superseded_by!==null&&k.superseded_by!==undefined){
        const nw=wi.keys[k.superseded_by];if(!nw)return"missing-successor-key";
        if(vu===null||nw.valid_from_version<=vu)return"invalid-successor-lineage";
        if(nw.supersedes!==kid)return"asymmetric-successor-lineage";
      }
    }
  }
  return null;
}
function validateRoot(root,reg,expected){
  if(digest(root)!==expected)return"root-pin";
  if(root.schema!==ROOT_SCHEMA||root.status!=="research-witness-crypto-trust-root-only")return"root-schema";
  if(root.trust_root_id!==ROOT_ID||root.registry_id!==REG_ID)return"root-identity";
  if(root.registry_version!==reg.registry_version||root.threshold!==reg.threshold||root.max_faulty!==reg.max_faulty)return"root-parameters";
  if(root.attestation_schema!==SIG_SCHEMA)return"root-attestation-schema";
  if(root.registry_sha256!==digest(reg))return"registry-root-binding";
  return validateRegistry(reg);
}
function validateCheckpoint(cp,root,reg){
  if(cp.schema!==CP_SCHEMA||cp.status!=="research-witness-crypto-checkpoint-only")return"checkpoint-schema";
  if(cp.root_reference_sha256!==digest(root))return"checkpoint-root-binding";
  if(cp.registry_id!==REG_ID||cp.registry_version!==reg.registry_version)return"checkpoint-registry-binding";
  if(!cp.witness_attestations||typeof cp.witness_attestations!=="object"||Array.isArray(cp.witness_attestations))return"checkpoint-attestations";
  return null;
}
function keyFor(reg,w,kid,version){
  const wi=reg.witnesses?.[w];if(!wi)return[null,"unknown-witness"];
  const k=wi.keys?.[kid];if(!k)return[null,"unknown-key"];
  if(k.algorithm!==ALG)return[null,"key-algorithm"];
  let pub;try{pub=b64d(k.public_key,32);}catch{return[null,"key-public-key"];}
  if(version<k.valid_from_version)return[null,"key-not-yet-valid"];
  if(k.valid_until_version!==null&&version>k.valid_until_version)return[null,"key-expired-for-version"];
  if(k.revoked_at_version!==null&&version>=k.revoked_at_version)return[null,"key-revoked"];
  if(k.status==="revoked")return[null,"key-revoked"];
  return[pub,null];
}
function verifyAtt(cp,w,a,root,reg){
  const fields=["schema","witness_id","key_id","algorithm","domain","claims","signature"];
  if(!a||typeof a!=="object"||Array.isArray(a)||Object.keys(a).sort().join("|")!==fields.join("|"))return[null,"signature-wrapping-or-schema"];
  if(a.schema!==SIG_SCHEMA)return[null,"attestation-schema"];
  if(a.witness_id!==w)return[null,"witness-id-mismatch"];
  if(a.algorithm!==ALG)return[null,"algorithm-substitution"];
  if(a.domain!==DOMAIN)return[null,"domain-separation"];
  const req=["registry_id","registry_version","root_reference_sha256","authority_id","manifest_version","manifest_sha256","previous_manifest_sha256"];
  if(!a.claims||typeof a.claims!=="object"||Object.keys(a.claims).sort().join("|")!==req.sort().join("|"))return[null,"claims-schema"];
  const c=a.claims;
  if(c.registry_id!==REG_ID||c.registry_version!==reg.registry_version)return[null,"claims-registry-binding"];
  if(c.root_reference_sha256!==cp.root_reference_sha256||c.root_reference_sha256!==digest(root))return[null,"claims-root-binding"];
  if(c.authority_id!==AUTH_ID||!Number.isInteger(c.manifest_version)||c.manifest_version<1)return[null,"claims-authority"];
  const [pub,e]=keyFor(reg,w,a.key_id,c.manifest_version);if(e)return[null,e];
  let sig;try{sig=b64d(a.signature,64);}catch{return[null,"signature-encoding"];}
  const p={schema:SIG_SCHEMA,domain:DOMAIN,algorithm:ALG,witness_id:w,key_id:a.key_id,witness_identity_commitment:reg.witnesses[w].identity_commitment,claims:c};
  return verifySig(pub,sig,Buffer.from(canonical(p),"utf8"))?[c,null]:[null,"signature-invalid"];
}
function consensus(cp,root,reg){
  const ids=Object.keys(cp.witness_attestations||{});if(ids.length<reg.threshold)return[null,"below-threshold"];
  const claims=[];
  for(const w of ids){if(!reg.witnesses?.[w])return[null,"unknown-witness"];const [c,e]=verifyAtt(cp,w,cp.witness_attestations[w],root,reg);if(e)return[null,e];claims.push(c);}
  const tuples=new Set(claims.map(c=>JSON.stringify([c.authority_id,c.manifest_version,c.manifest_sha256,c.previous_manifest_sha256,c.root_reference_sha256])));
  return tuples.size===1?[claims[0],"ok"]:[null,"equivocation"];
}
function evaluate(c,baseline,forward,root,reg,expected){
  const cp=structuredClone(c.base==="candidate"?forward:baseline),root2=structuredClone(root),reg2=structuredClone(reg),w=c.witness;
  if(c.remove_witness)delete cp.witness_attestations[c.remove_witness];
  for(const x of c.remove_witnesses||[])delete cp.witness_attestations[x];
  if(c.duplicate_witness||c.same_witness_twice)return["unresolved","duplicate-witness"];
  if(c.fork_witness)cp.witness_attestations[c.fork_witness]=structuredClone(c.replacement_attestation);
  else if(c.replacement_attestation)cp.witness_attestations[w]=structuredClone(c.replacement_attestation);
  if(c.mutate_key_id)cp.witness_attestations[w].key_id=c.mutate_key_id;
  if(c.mutate_domain)cp.witness_attestations[w].domain=c.mutate_domain;
  if(c.mutate_algorithm)cp.witness_attestations[w].algorithm=c.mutate_algorithm;
  if(c.extra_field)cp.witness_attestations[w][c.extra_field]=true;
  if(c.apply_manifest)cp.witness_attestations[w].claims.manifest_sha256=c.apply_manifest;
  if(c.apply_predecessor)cp.witness_attestations[w].claims.previous_manifest_sha256=c.apply_predecessor;
  if(c.root_threshold!==undefined)root2.threshold=c.root_threshold;
  if(c.root_registry_sha256!==undefined)root2.registry_sha256=c.root_registry_sha256;
  if(c.conflicting_root_reference)cp.root_reference_sha256=c.conflicting_root_reference;
  if(c.registry_threshold!==undefined)reg2.threshold=c.registry_threshold;
  if(c.registry_public_key_reuse){const[tw,tk,sw,sk]=c.registry_public_key_reuse;reg2.witnesses[tw].keys[tk].public_key=reg2.witnesses[sw].keys[sk].public_key;}
  if(c.registry_key_mutation){const[tw,tk,field,value]=c.registry_key_mutation;reg2.witnesses[tw].keys[tk][field]=value;}
  if(c.registry_witness_id_swap){const[a,b]=c.registry_witness_id_swap;[reg2.witnesses[a].identity_commitment,reg2.witnesses[b].identity_commitment]=[reg2.witnesses[b].identity_commitment,reg2.witnesses[a].identity_commitment];}
  if(c.unknown_witness)cp.witness_attestations[c.unknown_witness]=structuredClone(cp.witness_attestations.w01);
  if(c.reorder)cp.witness_attestations=Object.fromEntries(Object.entries(cp.witness_attestations).reverse());
  if(c.candidate_break_predecessor||c.candidate_manifest_sha256||c.candidate_gap){
    for(const a of Object.values(cp.witness_attestations)){
      if(c.candidate_break_predecessor)a.claims.previous_manifest_sha256="sha256:deadbeef";
      if(c.candidate_manifest_sha256)a.claims.manifest_sha256=c.candidate_manifest_sha256;
      if(c.candidate_gap)a.claims.manifest_version+=Number(c.candidate_gap);
    }
  }
  const re=validateRegistry(reg2);if(re)return["unresolved",re];
  const ro=validateRoot(root2,reg2,expected);if(ro)return["unresolved",ro];
  const ce=validateCheckpoint(cp,root2,reg2);if(ce)return["unresolved",ce];
  const [claim,reason]=consensus(cp,root2,reg2);if(!claim)return["unresolved",reason];
  if(c.base==="candidate"){
    const [bc,br]=consensus(baseline,root,reg);if(!bc)return["unresolved","baseline-"+br];
    if(claim.manifest_version!==bc.manifest_version+1)return["unresolved","forward-version"];
    if(claim.previous_manifest_sha256!==bc.manifest_sha256)return["unresolved","forward-predecessor"];
  }
  return["qualified","ok"];
}
function knownAnswerTest(){
  const p=Buffer.from("d75a980182b10ab7d54bfed3c964073a0ee172f3daa62325af021a68f707511a","hex");
  const s=Buffer.from("e5564300c360ac729086e2cc806e828a84877f1eb8e5d974d873e065224901555fb8821590a33bacc61e39701cf9b46bd25bf5f0595bbe24655141438e7a100b","hex");
  return verifySig(p,s,Buffer.alloc(0));
}
const args=process.argv.slice(2);if(args.length!==7){console.error("usage: verifier EXPECTED_ROOT_SHA TRUST_ROOT REGISTRY BASELINE FORWARD CAMPAIGN REPORT");process.exit(2);}
const [expected,rootP,regP,baseP,fwdP,campP,outP]=args;
if(!knownAnswerTest()){console.error("ed25519-known-answer-test-failed");process.exit(1);}
const root=JSON.parse(fs.readFileSync(rootP,"utf8")),reg=JSON.parse(fs.readFileSync(regP,"utf8")),baseline=JSON.parse(fs.readFileSync(baseP,"utf8")),forward=JSON.parse(fs.readFileSync(fwdP,"utf8")),campaign=JSON.parse(fs.readFileSync(campP,"utf8"));
if(campaign.schema!==CAMP_SCHEMA||campaign.expected_trust_root_sha256!==expected||campaign.case_count!==22||!Array.isArray(campaign.cases)||campaign.cases.length!==22)process.exit(1);
const ids=campaign.cases.map(c=>c.case_id);if(new Set(ids).size!==ids.length)process.exit(1);
const rows=[],failures=[];
for(const c of campaign.cases){const [v,r]=evaluate(c,baseline,forward,root,reg,expected);const row={case_id:c.case_id,expected_verdict:c.expected_verdict,actual_verdict:v,reason:r};rows.push(row);if(v!==c.expected_verdict)failures.push([c.case_id,c.expected_verdict,v,r]);}
fs.writeFileSync(outP,canonical({schema:"mycelix.continual-adaptation.censoring-classification-anchor-witness-crypto-report.v2",status:"research-evidence-only",case_count:22,ed25519_known_answer_test:"pass",cases:rows,failures})+"\n");
console.log("cases="+rows.length+" failures="+failures.length);process.exit(failures.length?1:0);
