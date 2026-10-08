#!/usr/bin/env node
import fs from "node:fs";
import crypto from "node:crypto";
const ROOT_SCHEMA="mycelix.continual-adaptation.censoring-classification-anchor-witness-trust-root.v1";
const REGISTRY_SCHEMA="mycelix.continual-adaptation.censoring-classification-anchor-witness-registry.v1";
const CHECKPOINT_SCHEMA="mycelix.continual-adaptation.censoring-classification-anchor-witness-checkpoint.v1";
const CAMPAIGN_SCHEMA="mycelix.continual-adaptation.censoring-classification-anchor-witness-campaign.v1";
const ATTESTATION_SCHEMA="mycelix.continual-adaptation.censoring-classification-anchor-witness-attestation.v1";
const ROOT_ID="mycelix.research.anchor-witness-root.v1", REGISTRY_ID="mycelix.research.anchor-witness-registry.v1", AUTHORITY_ID="mycelix.research.anchor-authority.v1";
function canonical(v){if(v===null||typeof v!=="object")return JSON.stringify(v);if(Array.isArray(v))return "["+v.map(canonical).join(",")+"]";return "{"+Object.keys(v).sort().map(k=>JSON.stringify(k)+":"+canonical(v[k])).join(",")+"}";}
function digest(v){return "sha256:"+crypto.createHash("sha256").update(Buffer.from(canonical(v),"utf8")).digest("hex");}
function clone(v){return structuredClone(v);}
function attestationCommitment(cp,w){return digest({schema:ATTESTATION_SCHEMA,registry_id:cp.registry_id,registry_version:cp.registry_version,root_reference_sha256:cp.root_reference_sha256,witness_id:w,authority_id:cp.authority_id,manifest_version:cp.manifest_version,manifest_sha256:cp.manifest_sha256,previous_manifest_sha256:cp.previous_manifest_sha256});}
function mutate(base,c,root,registry){
 const cp=clone(base), r=clone(root), reg=clone(registry);
 if(c.fork_witness){cp.manifest_sha256=c.fork_manifest_sha256;if(c.fork_predecessor!==undefined)cp.previous_manifest_sha256=c.fork_predecessor;cp.attestations[c.fork_witness]=attestationCommitment(cp,c.fork_witness);}
 if(c.candidate_break_predecessor)cp.previous_manifest_sha256="sha256:deadbeef";
 if(c.candidate_manifest_sha256)cp.manifest_sha256=c.candidate_manifest_sha256;
 if(c.candidate_gap)cp.manifest_version+=Number(c.candidate_gap);
 if(c.tamper_witness)cp[c.tamper_field]=c.tamper_value;
 if(c.root_threshold!==undefined)r.threshold=c.root_threshold;
 if(c.root_registry_sha256!==undefined)r.registry_sha256=c.root_registry_sha256;
 if(c.conflicting_root_reference)cp.root_reference_sha256=c.conflicting_root_reference;
 if(c.registry_threshold!==undefined)reg.threshold=c.registry_threshold;
 if(c.registry_witness_id_swap){const[a,b]=c.registry_witness_id_swap;[reg.witnesses[a].identity_commitment,reg.witnesses[b].identity_commitment]=[reg.witnesses[b].identity_commitment,reg.witnesses[a].identity_commitment];}
 return [cp,r,reg];
}
function validateRegistry(reg,root,expected){if(reg.schema!==REGISTRY_SCHEMA||reg.status!=="research-witness-registry-only")return true;if(reg.registry_id!==REGISTRY_ID||reg.registry_version!==1)return true;const n=Object.keys(reg.witnesses??{}).length,q=reg.threshold,f=reg.max_faulty;if(!Number.isInteger(q)||!Number.isInteger(f)||n===0||q<1||q>n||f<0||2*q<=n+f)return true;if(root.registry_sha256!==digest(reg))return true;return false;}
function validateRoot(root,reg,expected){if(digest(root)!==expected)return true;if(root.schema!==ROOT_SCHEMA||root.status!=="research-witness-trust-root-only")return true;if(root.trust_root_id!==ROOT_ID||root.registry_id!==REGISTRY_ID)return true;if(root.registry_version!==reg.registry_version||root.threshold!==reg.threshold)return true;return validateRegistry(reg,root,expected);}
function validateCheckpoint(cp,root,reg){if(cp.schema!==CHECKPOINT_SCHEMA||cp.status!=="research-witness-checkpoint-only")return true;if(cp.root_reference_sha256!==digest(root))return true;if(cp.registry_id!==REGISTRY_ID||cp.registry_version!==reg.registry_version)return true;if(cp.authority_id!==AUTHORITY_ID)return true;if(!Number.isInteger(cp.manifest_version)||cp.manifest_version<1)return true;if(!cp.attestations||typeof cp.attestations!=="object"||Array.isArray(cp.attestations))return true;return false;}
function evaluate(c,baseline,forward,root,reg,expected){
 let [cp,r,rg]=mutate(c.base==="candidate"?forward:baseline,c,root,reg);
 if(c.remove_witness)delete cp.attestations[c.remove_witness];
 for(const w of c.remove_witnesses??[])delete cp.attestations[w];
 if(c.unknown_witness)cp.attestations[c.unknown_witness]=cp.attestations.w01;
 if(c.duplicate_witness||c.same_witness_twice)return "unresolved";
 if(validateRoot(r,rg,expected)||validateCheckpoint(cp,r,rg))return "unresolved";
 const ids=Object.keys(cp.attestations);
 if(new Set(ids).size!==ids.length||ids.some(w=>!rg.witnesses[w])||ids.length<rg.threshold)return "unresolved";
 const tuples=[];
 for(const w of ids){if(cp.attestations[w]!==attestationCommitment(cp,w))return "unresolved";tuples.push(JSON.stringify([cp.authority_id,cp.manifest_version,cp.manifest_sha256,cp.previous_manifest_sha256,cp.root_reference_sha256]));}
 if(new Set(tuples).size!==1)return "unresolved";
 if(c.base==="candidate"){if(cp.manifest_version!==baseline.manifest_version+1)return "unresolved";if(cp.previous_manifest_sha256!==baseline.manifest_sha256)return "unresolved";}
 return "qualified";
}
const a=process.argv.slice(2);if(a.length!==7){console.error("usage: verify_anchor_witness.mjs EXPECTED_ROOT_SHA TRUST_ROOT.json REGISTRY.json BASELINE.json FORWARD.json CAMPAIGN.json REPORT.json");process.exit(2);}
const [expected,rootPath,regPath,basePath,fwdPath,campPath,reportPath]=a;
const root=JSON.parse(fs.readFileSync(rootPath,"utf8")),reg=JSON.parse(fs.readFileSync(regPath,"utf8")),baseline=JSON.parse(fs.readFileSync(basePath,"utf8")),forward=JSON.parse(fs.readFileSync(fwdPath,"utf8")),campaign=JSON.parse(fs.readFileSync(campPath,"utf8"));
if(campaign.schema!==CAMPAIGN_SCHEMA||campaign.expected_trust_root_sha256!==expected||campaign.cases.length!==24)process.exit(1);
const ids=campaign.cases.map(c=>c.case_id);if(new Set(ids).size!==ids.length||ids.some(x=>typeof x!=="string"))process.exit(1);
const rows=[],failures=[];for(const c of campaign.cases){const v=evaluate(c,baseline,forward,root,reg,expected);rows.push({case_id:c.case_id,expected_verdict:c.expected_verdict,actual_verdict:v});if(v!==c.expected_verdict)failures.push([c.case_id,c.expected_verdict,v]);}
fs.writeFileSync(reportPath,canonical({cases:rows,failures,schema:"mycelix.continual-adaptation.censoring-classification-anchor-witness-report.v1",status:"research-evidence-only"})+"\n","utf8");console.log("cases="+rows.length+" failures="+failures.length);process.exit(failures.length?1:0);
