#!/usr/bin/env node
import fs from "node:fs";
import path from "node:path";
import crypto from "node:crypto";
import {execFileSync} from "node:child_process";

const BUNDLE_SCHEMA="mycelix.continual-adaptation.censoring-classification-anchor-audit-bundle.v2";
const CAMPAIGN_SCHEMA="mycelix.continual-adaptation.censoring-classification-anchor-audit-bundle-campaign.v2";
const STACK_HEAD="62a12a46d450bc6f9fe8a4994c5fa05b44204011";
const BUNDLE_ID="mycelix.audit-bundle.v2@"+STACK_HEAD;
const WREG_ID="mycelix.research.anchor-witness-registry.v2";
const VDS_ID="mycelix.research.anchor-statement-sequence.v1";
const TS_ID="mycelix.research.anchor-cose-receipt-ts-registry.v1";
const GOSSIP_ID="mycelix.research.anchor-observer-gossip-registry.v1";
const COSE_REGISTRY_SHA="sha256:21772a0a1dbb88358c80e5297d4ec4dd82c2ba2bca474a334bae45535ec8b331";
const LEGACY_TS_REGISTRY_SHA="sha256:efb549a023660010a07d70b7d78afbfd81664200fe4ecf1d0d542dc945f0bb54";
const HEAD_KEYS=["size_4","size_7","fork_size_4","wrong_key","revoked_key","rollback_key","noncanonical"];
const REQUIRED_VERIFIERS=[
"audit_bundle_python","audit_bundle_node","witness_crypto_python","witness_crypto_node",
"vds_python","vds_node","rotation_python","rotation_node","tree_head_python","tree_head_node",
"legacy_receipt_python","legacy_receipt_node","static_gossip_python","static_gossip_node",
"cose_receipt_python","cose_receipt_node","gossip_simulation_python","gossip_simulation_node"
];
const EXPECTED_TOPOLOGY=[
[4848,"5af2d8a2a4e42ccdcabc70160ad639d810642c86"],
[4870,"2af796188bd7981fac111178141ae82165c2fc4e"],
[4873,"e3a175b905a041a17630b92356e123ccba0d9175"],
[4874,"b55af8c135a858abfb0663a762e7fd510e9ea6d2"],
[4875,"7648801649b30bde5233ac380924a539b0cdbf35"],
[4876,"1e352f442a4bc7bc537d0223cb39c5fe37944c56"],
[4877,"61c46f16d32ee4d0bc42ffae76743caf19a76dd1"],
[4879,"1c8f5a167c12ebed1e0bb4c09f1f7f8ee5a7e2c8"],
[4889,"c6e69d0c9969b7b6d7feece65c60ae7c913e980d"],
[4890,STACK_HEAD]
];
const PROHIBITED=new Set(["private_key","private_keys","secret_key","seed","private_seed","secret"]);
const canonical=v=>v===null||typeof v!=="object"?JSON.stringify(v):Array.isArray(v)?"["+v.map(canonical).join(",")+"]":"{"+Object.keys(v).sort().map(k=>JSON.stringify(k)+":"+canonical(v[k]))+"}";
const H=b=>crypto.createHash("sha256").update(b).digest();
const digestObj=v=>"sha256:"+H(Buffer.from(canonical(v),"utf8")).toString("hex");
function containsPrivate(v){
  if(Array.isArray(v))return v.some(containsPrivate);
  if(v&&typeof v==="object")return Object.entries(v).some(([k,x])=>PROHIBITED.has(k.toLowerCase())||containsPrivate(x));
  return false;
}
function gitBlob(file,root){return execFileSync("git",["hash-object",file],{cwd:root,encoding:"utf8"}).trim();}
function mth(entries){
  if(entries.length===0)return H(Buffer.alloc(0));
  if(entries.length===1)return H(Buffer.concat([Buffer.from([0]),Buffer.from(canonical(entries[0]),"utf8")]));
  const k=1<<((entries.length-1).toString(2).length-1);
  return H(Buffer.concat([Buffer.from([1]),mth(entries.slice(0,k)),mth(entries.slice(k))]));
}
function validate(bundle,root){
  if(bundle.schema!==BUNDLE_SCHEMA)return"bundle-schema";
  if(bundle.status!=="research-evidence-only"||bundle.evidence_ceiling!=="exact-input-and-verifier-binding-only")return"bundle-status";
  if(bundle.bundle_id!==BUNDLE_ID)return"bundle-id";
  if(bundle.hosted_status!=="unclaimed")return"hosted-claim-injection";
  const topo=new Set((bundle.topology||[]).map(x=>x.pr+"|"+x.head));
  if(topo.size!==EXPECTED_TOPOLOGY.length||EXPECTED_TOPOLOGY.some(([pr,sha])=>!topo.has(pr+"|"+sha)))return"topology";
  if(Object.keys(bundle.verifiers||{}).sort().join("|")!==[...REQUIRED_VERIFIERS].sort().join("|"))return"verifier-inventory";
  const req=bundle.bindings?.required_verifiers;
  if(!req||Object.keys(req).sort().join("|")!==[...REQUIRED_VERIFIERS,"hosted_pass"].sort().join("|"))return"verifier-requirement-inventory";
  if(REQUIRED_VERIFIERS.some(k=>req[k]!==true)||req.hosted_pass!==false)return"verifier-requirement-weakening";
  const claims=bundle.bindings?.security_claims;
  const claimKeys=["complete_scitt_interoperability","live_network_convergence","organizational_independence","private_key_custody_proven","hosted_pass"].sort();
  if(!claims||Object.keys(claims).sort().join("|")!==claimKeys.join("|")||claimKeys.some(k=>claims[k]!==false))return"claim-ceiling-injection";
  const arts=bundle.artifacts||{},vers=bundle.verifiers||{};
  for(const[group,items]of [["artifact",arts],["verifier",vers]]){
    for(const[name,pair]of Object.entries(items)){
      if(!Array.isArray(pair)||pair.length!==2)return group+"-record:"+name;
      const [rel,expected]=pair,file=path.resolve(root,rel);
      if(!fs.existsSync(file))return group+"-missing:"+name;
      if(gitBlob(file,root)!==expected)return group+"-sha:"+name;
      if(group==="artifact"){
        let obj;try{obj=JSON.parse(fs.readFileSync(file,"utf8"));}catch{return"artifact-json:"+name;}
        if(containsPrivate(obj))return"private-key-material:"+name;
      }
    }
  }
  const read=name=>JSON.parse(fs.readFileSync(path.resolve(root,arts[name][0]),"utf8"));
  const v1=read("audit_bundle_v1");
  if(v1.schema!=="mycelix.continual-adaptation.censoring-classification-anchor-audit-bundle.v1")return"v1-bundle-schema";
  if(v1.bundle_id!=="mycelix.audit-bundle.v1@61c46f16d32ee4d0bc42ffae76743caf19a76dd1")return"v1-bundle-id";
  if(canonical(v1.artifacts?.vds_tree_heads)!==canonical(arts.tree_heads))return"v1-tree-head-pin";
  const reg=read("witness_registry"),trust=read("witness_root");
  if(reg.registry_id!==WREG_ID||reg.registry_version!==2)return"witness-registry-binding";
  if(trust.registry_id!==WREG_ID||trust.registry_version!==2||trust.registry_sha256!==digestObj(reg))return"witness-root-binding";
  if(read("witness_baseline").registry_id!==WREG_ID||read("witness_forward").registry_id!==WREG_ID)return"witness-checkpoint-binding";
  const tree=read("tree_heads");
  if(tree.schema!=="mycelix.continual-adaptation.censoring-classification-anchor-witness-vds-tree-head-fixture.v1")return"tree-head-schema";
  if(tree.vds_id!==VDS_ID||canonical(Object.keys(tree.heads||{}).sort())!==canonical([...HEAD_KEYS].sort()))return"tree-head-topology";
  const entries=read("vds_fixture").entries;
  if(!Array.isArray(entries)||entries.length<7)return"vds-entries";
  for(const[name,size]of [["size_4",4],["size_7",7]]){
    const atts=tree.heads[name]?.attestations;if(!atts||Object.keys(atts).length!==4)return"tree-head-quorum:"+name;
    const roots=new Set(Object.values(atts).map(a=>a.root_hash)),sizes=new Set(Object.values(atts).map(a=>a.tree_size));
    if(roots.size!==1||sizes.size!==1||![...sizes].includes(size))return"tree-head-agreement:"+name;
    if([...roots][0]!=="sha256:"+mth(entries.slice(0,size)).toString("hex"))return"tree-head-merkle-root:"+name;
  }
  const legacyTs=read("legacy_receipt_registry"),legacyReceipt=read("legacy_receipt_fixture");
  if(digestObj(legacyTs)!==LEGACY_TS_REGISTRY_SHA||legacyReceipt.ts_registry_sha256!==digestObj(legacyTs))return"legacy-receipt-registry-binding";
  if(legacyReceipt.vds_id!==VDS_ID)return"legacy-receipt-vds-binding";
  const coseTs=read("cose_registry"),cose=read("cose_fixture"),coseCampaign=read("cose_campaign");
  if(digestObj(coseTs)!==COSE_REGISTRY_SHA||cose.ts_registry_sha256!==digestObj(coseTs)||cose.ts_registry_id!==TS_ID)return"cose-registry-binding";
  if(cose.vds_id!==VDS_ID||cose.vds_algorithm_id!==1||cose.root_hash!=="sha256:"+mth(entries.slice(0,cose.tree_size)).toString("hex"))return"cose-vds-binding";
  if(coseCampaign.case_count!==22||coseCampaign.cases.length!==22)return"cose-campaign-shape";
  const gossipReg=read("gossip_registry"),gossip=read("gossip_fixture"),gossipCampaign=read("gossip_campaign");
  if(gossipReg.registry_id!==GOSSIP_ID||gossip.vds_id!==VDS_ID)return"gossip-binding";
  if(gossipCampaign.case_count!==16||gossipCampaign.cases.length!==16)return"gossip-campaign-shape";
  const sim=read("gossip_simulation"),simCampaign=read("gossip_simulation_campaign");
  if(sim.schema!=="mycelix.continual-adaptation.censoring-classification-anchor-observer-gossip-simulation.v1")return"simulation-schema";
  if(simCampaign.case_count!==8||simCampaign.cases.length!==8)return"simulation-campaign-shape";
  if(bundle.bindings?.witness_registry_id!==WREG_ID||bundle.bindings?.vds_id!==VDS_ID)return"bundle-bindings";
  const profile=bundle.bindings?.cose;
  const expectedProfile={cose_tag:18,sign1:true,detached_payload:true,protected_labels:{alg:1,kid:4,vds:395},vds_algorithm:1,vdp_label:396,inclusion_proof_label:-1,consistency_proof_label:-2,cose_algorithm:-8};
  if(canonical(profile)!==canonical(expectedProfile))return"cose-profile-substitution";
  return null;
}
function mutate(bundle,mutation){
  const b=structuredClone(bundle),[type,name]=mutation.split(":",2);
  if(type==="artifact"&&b.artifacts[name])b.artifacts[name][1]="0".repeat(40);
  else if(type==="verifier"&&b.verifiers[name])b.verifiers[name][1]="0".repeat(40);
  else if(type==="binding"&&name==="vds_id")b.bindings.vds_id="attacker.vds";
  else if(type==="topology"&&name==="4890")b.topology.find(x=>x.pr===4890).head="0".repeat(40);
  else if(type==="disable_verifier"&&b.bindings.required_verifiers[name]!==undefined)b.bindings.required_verifiers[name]=false;
  else if(type==="claim"&&b.bindings.security_claims[name]!==undefined)b.bindings.security_claims[name]=true;
  else if(mutation==="hosted_status")b.hosted_status="success";
  return b;
}
const args=process.argv.slice(2);
if(args.length!==4){console.error("usage: verifier REPO_ROOT BUNDLE CAMPAIGN REPORT");process.exit(2);}
const[root,bundlePath,campaignPath,reportPath]=args;
const bundle=JSON.parse(fs.readFileSync(bundlePath,"utf8")),campaign=JSON.parse(fs.readFileSync(campaignPath,"utf8"));
if(campaign.schema!==CAMPAIGN_SCHEMA||campaign.case_count!==25||campaign.cases.length!==25)process.exit(1);
const ids=campaign.cases.map(c=>c.case_id);if(new Set(ids).size!==ids.length)process.exit(1);
const rows=[],failures=[];
for(const c of campaign.cases){
  const err=validate(mutate(bundle,c.mutation||""),root),verdict=err===null?"evidence-ready":"unresolved";
  const row={case_id:c.case_id,expected_verdict:c.expected_verdict,actual_verdict:verdict,reason:err??"inputs-verifiers-topology-and-claim-ceiling-match"};rows.push(row);
  if(verdict!==c.expected_verdict)failures.push([c.case_id,c.expected_verdict,verdict,err]);
}
fs.writeFileSync(reportPath,canonical({schema:"mycelix.continual-adaptation.censoring-classification-anchor-audit-bundle-report.v2",status:"research-evidence-only",case_count:rows.length,cases:rows,failures})+"\n");
console.log("cases="+rows.length+" failures="+failures.length);process.exit(failures.length?1:0);
