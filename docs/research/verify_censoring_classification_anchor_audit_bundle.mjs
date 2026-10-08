#!/usr/bin/env node
import fs from "node:fs";
import crypto from "node:crypto";
import {execFileSync} from "node:child_process";

const BUNDLE_SCHEMA="mycelix.continual-adaptation.censoring-classification-anchor-audit-bundle.v1";
const CAMPAIGN_SCHEMA="mycelix.continual-adaptation.censoring-classification-anchor-audit-bundle-campaign.v1";
const WREG_ID="mycelix.research.anchor-witness-registry.v2";
const VDS_ID="mycelix.research.anchor-statement-sequence.v1";
const STACK_HEAD="61c46f16d32ee4d0bc42ffae76743caf19a76dd1";
const BUNDLE_ID="mycelix.audit-bundle.v1@"+STACK_HEAD;

const canon=v=>v===null||typeof v!=="object"?JSON.stringify(v):Array.isArray(v)?"["+v.map(canon).join(",")+"]":"{"+Object.keys(v).sort().map(k=>JSON.stringify(k)+":"+canon(v[k])).join(",")+"}";
const digest=v=>"sha256:"+crypto.createHash("sha256").update(Buffer.from(canon(v),"utf8")).digest("hex");
const gitBlob=p=>execFileSync("git",["hash-object",p],{encoding:"utf8"}).trim();

function containsSecret(v){
  const terms=new Set(["private_key","private_keys","secret","seed","secret_key"]);
  if(Array.isArray(v))return v.some(containsSecret);
  if(v&&typeof v==="object")return Object.entries(v).some(([k,x])=>terms.has(k.toLowerCase())||containsSecret(x));
  return false;
}
function validate(bundle,root){
  if(bundle.schema!==BUNDLE_SCHEMA)return"bundle-schema";
  if(bundle.status!=="research-evidence-only"||bundle.evidence_ceiling!=="replayable-binding-only")return"bundle-status";
  if(bundle.bundle_id!==BUNDLE_ID)return"bundle-id";
  if(bundle.hosted_status!=="unclaimed")return"hosted-claim-injection";
  const topo=new Set((bundle.topology||[]).map(x=>x.pr+"|"+x.head));
  const expected=new Set([
    "4848|5af2d8a2a4e42ccdcabc70160ad639d810642c86",
    "4870|2af796188bd7981fac111178141ae82165c2fc4e",
    "4873|e3a175b905a041a17630b92356e123ccba0d9175",
    "4874|b55af8c135a858abfb0663a762e7fd510e9ea6d2",
    "4875|7648801649b30bde5233ac380924a539b0cdbf35",
    "4876|1e352f442a4bc7bc537d0223cb39c5fe37944c56",
    "4877|"+STACK_HEAD
  ]);
  if(topo.size!==expected.size||[...expected].some(x=>!topo.has(x)))return"topology";
  const required=["witness_crypto_verifier","vds_verifier","tree_head_verifier","receipt_verifier","observer_gossip_verifier"];
  if(!bundle.decision_requires||bundle.decision_requires.hosted_pass!==false||required.some(k=>bundle.decision_requires[k]!==true))return"decision-requirement";
  for(const[name,pair]of Object.entries(bundle.artifacts||{})){
    if(!Array.isArray(pair)||pair.length!==2)return"artifact-record:"+name;
    const [rel,sha]=pair,p=rel;
    if(!fs.existsSync(p))return"artifact-missing:"+name;
    if(gitBlob(p)!==sha)return"artifact-sha:"+name;
    let obj;try{obj=JSON.parse(fs.readFileSync(p,"utf8"));}catch{return"artifact-json:"+name;}
    if(containsSecret(obj))return"secret-material:"+name;
  }
  const a=bundle.artifacts;
  const reg=JSON.parse(fs.readFileSync(a.witness_crypto_registry[0],"utf8"));
  const root=JSON.parse(fs.readFileSync(a.witness_crypto_root[0],"utf8"));
  const baseline=JSON.parse(fs.readFileSync(a.witness_crypto_baseline[0],"utf8"));
  const forward=JSON.parse(fs.readFileSync(a.witness_crypto_forward[0],"utf8"));
  const vds=JSON.parse(fs.readFileSync(a.vds_fixture[0],"utf8"));
  const heads=JSON.parse(fs.readFileSync(a.vds_tree_heads[0],"utf8"));
  const ts=JSON.parse(fs.readFileSync(a.receipt_ts_registry[0],"utf8"));
  const receipt=JSON.parse(fs.readFileSync(a.receipt_fixture[0],"utf8"));
  const gossip=JSON.parse(fs.readFileSync(a.gossip_fixture[0],"utf8"));
  const gossipReg=JSON.parse(fs.readFileSync(a.gossip_registry[0],"utf8"));

  if(reg.registry_id!==WREG_ID||reg.registry_version!==2)return"witness-registry-binding";
  if(root.registry_id!==WREG_ID||root.registry_version!==2)return"witness-root-binding";
  if(root.registry_sha256!==digest(reg))return"witness-root-digest";
  if(baseline.registry_id!==WREG_ID||forward.registry_id!==WREG_ID)return"witness-checkpoint-binding";
  if(vds.vds_id!==VDS_ID)return"vds-binding";
  if(heads.vds_id!==VDS_ID)return"tree-head-binding";
  if(receipt.ts_registry_sha256!==digest(ts))return"receipt-ts-registry-binding";
  if(receipt.vds_id!==VDS_ID)return"receipt-vds-binding";
  if(gossip.vds_id!==VDS_ID||gossipReg.registry_id!==bundle.bindings.gossip_registry_id)return"gossip-binding";
  if(bundle.bindings.witness_registry_id!==WREG_ID||bundle.bindings.vds_id!==VDS_ID)return"bundle-binding";
  return null;
}
function mutate(bundle,m){
  const b=structuredClone(bundle);
  switch(m){
    case"witness_crypto_registry_sha":b.artifacts.witness_crypto_registry[1]="0".repeat(40);break;
    case"witness_root_sha":b.artifacts.witness_crypto_root[1]="0".repeat(40);break;
    case"vds_fixture_sha":b.artifacts.vds_fixture[1]="0".repeat(40);break;
    case"tree_head_sha":b.artifacts.vds_tree_heads[1]="0".repeat(40);break;
    case"receipt_fixture_sha":b.artifacts.receipt_fixture[1]="0".repeat(40);break;
    case"gossip_fixture_sha":b.artifacts.gossip_fixture[1]="0".repeat(40);break;
    case"witness_registry_id":b.bindings.witness_registry_id="attacker.registry";break;
    case"vds_id":b.bindings.vds_id="attacker.vds";break;
    case"hosted_status":b.hosted_status="success";break;
    case"topology_head":b.topology[b.topology.length-1].head="0".repeat(40);break;
    case"decision_requirement":b.decision_requires.hosted_pass=true;break;case"witness_crypto_verifier":b.decision_requires.witness_crypto_verifier=false;break;case"vds_verifier":b.decision_requires.vds_verifier=false;break;case"tree_head_verifier":b.decision_requires.tree_head_verifier=false;break;case"receipt_verifier":b.decision_requires.receipt_verifier=false;break;case"gossip_verifier":b.decision_requires.observer_gossip_verifier=false;break;
    case"bundle_id":b.bundle_id="mycelix.audit-bundle.v1@attacker";break;
  }
  return b;
}

const [rootDir,bundlePath,campaignPath]=process.argv.slice(2);
if(!campaignPath)process.exit(2);
const bundle=JSON.parse(fs.readFileSync(bundlePath,"utf8"));
const campaign=JSON.parse(fs.readFileSync(campaignPath,"utf8"));
if(campaign.schema!==CAMPAIGN_SCHEMA||campaign.case_count!==18||campaign.cases.length!==18)process.exit(1);

const rows=[],failures=[];
for(const c of campaign.cases){
  const err=validate(mutate(bundle,c.mutate),rootDir);
  const verdict=err===null?"evidence-ready":"unresolved";
  const row={case_id:c.case_id,expected_verdict:c.expected_verdict,actual_verdict:verdict,reason:err??"bindings-and-git-object-identities-match"};
  rows.push(row);if(verdict!==c.expected_verdict)failures.push([c.case_id,c.expected_verdict,verdict,err]);
}
const report={schema:"mycelix.continual-adaptation.censoring-classification-anchor-audit-bundle-report.v1",status:"research-evidence-only",case_count:rows.length,cases:rows,failures};
fs.writeFileSync("audit-bundle-node.json",canon(report)+"\n");
console.log("cases="+rows.length+" failures="+failures.length);
process.exit(failures.length?1:0);
