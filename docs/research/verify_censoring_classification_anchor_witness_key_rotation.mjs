#!/usr/bin/env node
import fs from "node:fs";
import crypto from "node:crypto";

const ROT="mycelix.continual-adaptation.censoring-classification-anchor-witness-key-rotation.v1";
const CER="mycelix.continual-adaptation.censoring-classification-anchor-witness-key-rotation-ceremony.v1";
const ALG="Ed25519", REGID="mycelix.research.anchor-witness-registry.v2", PROPID="mycelix.research.anchor-witness-registry.v3";
const canon=v=>v===null||typeof v!=="object"?JSON.stringify(v):Array.isArray(v)?"["+v.map(canon).join(",")+"]":"{"+Object.keys(v).sort().map(k=>JSON.stringify(k)+":"+canon(v[k])).join(",")+"}";
const digest=v=>"sha256:"+crypto.createHash("sha256").update(canon(v)).digest("hex");
const b64=s=>Buffer.from(s,"base64url");
const pub=b=>crypto.createPublicKey({key:Buffer.concat([Buffer.from("302a300506032b6570032100","hex"),b]),format:"der",type:"spki"});
const verify=(p,s,m)=>{try{return crypto.verify(null,m,pub(p),s)}catch{return false}};
const key=(r,w,k)=>b64(r.witnesses[w].keys[k].public_key);
const payload=(role,w,k,r)=>({schema:ROT,domain:"mycelix.continual-adaptation.censoring-classification-anchor-witness-key-rotation."+role+".v1",algorithm:ALG,role,witness_id:w,key_id:k,rotation:r});

function evaluate(base,proposal,ceremony,c){
  const reg=structuredClone(base), p=structuredClone(proposal), x=structuredClone(ceremony), r=x.ceremony.rotation;
  if(c.remove_predecessor)delete x.ceremony.predecessor_approval;
  if(c.remove_successor)delete x.ceremony.successor_proof_of_possession;
  if(c.remove_governance)x.ceremony.governance_approvals=x.ceremony.governance_approvals.slice(0,-c.remove_governance);
  if(c.target&&c.mutate_domain)x.ceremony[c.target].domain=c.mutate_domain;
  if(c.target&&c.mutate_algorithm)x.ceremony[c.target].algorithm=c.mutate_algorithm;
  if(c.governance_replace)x.ceremony.governance_approvals[c.governance_index]=structuredClone(c.governance_replace);
  if(c.successor_key_mismatch)r.successor_public_key=c.successor_key_mismatch;
  if(c.activation_version!==undefined)r.activation_version=c.activation_version;
  if(c.predecessor_until!==undefined)r.predecessor_valid_until_version=c.predecessor_until;
  if(c.successor_from!==undefined)r.successor_valid_from_version=c.successor_from;
  if(c.proposal_digest)r.proposed_registry_sha256=c.proposal_digest;
  if(c.proposal_mutation)p.witnesses.w01.keys["w01-k2"][c.proposal_mutation[0]]=c.proposal_mutation[1];

  if(r.current_registry_id!==REGID||r.current_registry_version!==2)return["unresolved","current-registry-binding"];
  if(r.proposed_registry_id!==PROPID||r.proposed_registry_version!==3)return["unresolved","proposed-registry-binding"];
  if(digest(p)!==r.proposed_registry_sha256)return["unresolved","proposal-digest"];
  if(r.predecessor_valid_until_version!==r.activation_version-1||r.successor_valid_from_version!==r.activation_version)return["unresolved","rotation-version-overlap"];

  const old=p.witnesses.w01.keys["w01-k1"], nw=p.witnesses.w01.keys["w01-k2"];
  if(old.status!=="retired"||old.valid_until_version!==3||old.superseded_by!=="w01-k2")return["unresolved","predecessor-registry-state"];
  if(nw.status!=="active"||nw.valid_from_version!==4||nw.supersedes!=="w01-k1")return["unresolved","successor-registry-state"];
  if(nw.public_key!==r.successor_public_key)return["unresolved","successor-key-mismatch"];

  const pre=x.ceremony.predecessor_approval;
  if(!pre)return["unresolved","missing-predecessor-approval"];
  if(pre.witness_id!=="w01"||pre.key_id!=="w01-k1"||pre.algorithm!==ALG)return["unresolved","predecessor-envelope"];
  if(pre.domain!=="mycelix.continual-adaptation.censoring-classification-anchor-witness-key-rotation.predecessor-approval.v1")return["unresolved","predecessor-domain"];
  if(!verify(key(reg,"w01","w01-k1"),b64(pre.signature),Buffer.from(canon(payload("predecessor-approval","w01","w01-k1",r)))))return["unresolved","predecessor-signature"];

  const suc=x.ceremony.successor_proof_of_possession;
  if(!suc)return["unresolved","missing-successor-possession"];
  if(suc.witness_id!=="w01"||suc.key_id!=="w01-k2"||suc.algorithm!==ALG)return["unresolved","successor-envelope"];
  if(suc.domain!=="mycelix.continual-adaptation.censoring-classification-anchor-witness-key-rotation.successor-possession.v1")return["unresolved","successor-domain"];
  if(!verify(b64(r.successor_public_key),b64(suc.signature),Buffer.from(canon(payload("successor-possession","w01","w01-k2",r)))))return["unresolved","successor-signature"];

  const gov=x.ceremony.governance_approvals||[],seen=new Set();
  for(const a of gov){
    if(a.witness_id==="w01")return["unresolved","governance-target-separation"];
    if(seen.has(a.witness_id))return["unresolved","duplicate-governance-witness"];
    seen.add(a.witness_id);
    if(!reg.witnesses?.[a.witness_id]?.keys?.[a.key_id])return["unresolved","governance-key"];
    if(a.algorithm!==ALG)return["unresolved","governance-envelope"];
    if(a.domain!=="mycelix.continual-adaptation.censoring-classification-anchor-witness-key-rotation.governance-approval.v1")return["unresolved","governance-domain"];
    if(!verify(key(reg,a.witness_id,a.key_id),b64(a.signature),Buffer.from(canon(payload("governance-approval",a.witness_id,a.key_id,r)))))return["unresolved","governance-signature"];
  }
  if(seen.size<3)return["unresolved","governance-below-threshold"];
  return["qualified","rotation-authorized-and-possession-proven"];
}

const [rp,pp,cp,campp,outp]=process.argv.slice(2);if(!outp)process.exit(2);
const reg=JSON.parse(fs.readFileSync(rp,"utf8")),proposal=JSON.parse(fs.readFileSync(pp,"utf8")),ceremony=JSON.parse(fs.readFileSync(cp,"utf8")),campaign=JSON.parse(fs.readFileSync(campp,"utf8"));
if(ceremony.schema!==CER||campaign.case_count!==15||campaign.cases.length!==15)process.exit(1);
const ids=campaign.cases.map(c=>c.case_id);if(new Set(ids).size!==ids.length)process.exit(1);
const rows=[],failures=[];
for(const c of campaign.cases){const[v,r]=evaluate(reg,proposal,ceremony,c);const row={case_id:c.case_id,expected_verdict:c.expected_verdict,actual_verdict:v,reason:r};rows.push(row);if(v!==c.expected_verdict)failures.push([c.case_id,c.expected_verdict,v,r]);}
fs.writeFileSync(outp,canon({schema:"mycelix.continual-adaptation.censoring-classification-anchor-witness-key-rotation-report.v1",status:"research-evidence-only",case_count:15,cases:rows,failures})+"\n");
console.log("cases="+rows.length+" failures="+failures.length);process.exit(failures.length?1:0);
