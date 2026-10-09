#!/usr/bin/env node
import fs from "node:fs";
import path from "node:path";
import os from "node:os";
import crypto from "node:crypto";

const RECEIPT_SCHEMA="mycelix.continual-adaptation.censoring-classification-execution-receipt.v1";
const PREDICATE_SCHEMA="https://luminousdynamics.io/attestations/mycelix-anchor-research-execution/v1";
const REPOSITORY="Luminous-Dynamics/mycelix";
const WORKFLOW_NAME="continual-adaptation-censoring-classification-provenance";
const WORKFLOW_PATH=".github/workflows/continual-adaptation-censoring-classification-provenance.yml";
const PAIRS=[
["fixed-classification","python-fixed.json","node-fixed.json",52],
["generated-classification","python-generated.json","node-generated.json",168],
["anchor-governance","anchor-governance-python.json","anchor-governance-node.json",24],
["witness-non-equivocation","witness-python.json","witness-node.json",24],
["witness-cryptographic-authentication","witness-crypto-python.json","witness-crypto-node.json",22],
["append-only-vds","witness-vds-python.json","witness-vds-node.json",15],
["governed-key-rotation","witness-rotation-python.json","witness-rotation-node.json",15],
["authenticated-tree-heads","witness-tree-head-python.json","witness-tree-head-node.json",21],
["legacy-inclusion-receipts","receipt-python.json","receipt-node.json",16],
["static-observer-gossip","observer-gossip-python.json","observer-gossip-node.json",16],
["audit-bundle-v1","audit-bundle-python.json","audit-bundle-node.json",18],
["cose-receipts","cose-receipt-python.json","cose-receipt-node.json",22],
["partitioned-gossip-simulation","gossip-simulation-python.json","gossip-simulation-node.json",8],
["audit-bundle-v2","audit-bundle-v2-python.json","audit-bundle-v2-node.json",34]
];
const SUPPORTING=[["generated-corpus-a","supporting/generated-a.json",168],["generated-corpus-b","supporting/generated-b.json",168]];
const canonical=v=>v===null||typeof v!=="object"?JSON.stringify(v):Array.isArray(v)?"["+v.map(canonical).join(",")+"]":"{"+Object.keys(v).sort().map(k=>JSON.stringify(k)+":"+canonical(v[k]))+"}";
const sha=b=>crypto.createHash("sha256").update(b).digest("hex");
const readJson=p=>JSON.parse(fs.readFileSync(p,"utf8"));
function validateReport(file,expected){
  const raw=fs.readFileSync(file),obj=JSON.parse(raw.toString("utf8")),cases=obj.cases,failures=obj.failures,explicitCount=Object.hasOwn(obj,"case_count"),n=explicitCount?obj.case_count:(Array.isArray(cases)?cases.length:null);
  if(!Array.isArray(cases)||!cases.length)return[null,"empty-or-missing-cases"];
  if(explicitCount&&(!Number.isInteger(n)||n!==cases.length))return[null,"case-count-mismatch"];
  if(expected!==null&&n!==expected)return[null,"unexpected-case-count"];
  if(!Array.isArray(failures)||failures.length!==0)return[null,"reported-failures"];
  const ids=cases.map(x=>x&&typeof x==="object"?(x.case_id??x.id):null);
  if(ids.some(x=>typeof x!=="string"||!x))return[null,"case-identity-missing"];
  if(new Set(ids).size!==ids.length)return[null,"duplicate-case-id"];
  for(const row of cases)if(row&&typeof row==="object"&&Object.hasOwn(row,"expected_verdict")){
    const actual=Object.hasOwn(row,"actual_verdict")?row.actual_verdict:row.verdict;
    if(actual===undefined||actual===null)return[null,"actual-verdict-missing"];
    if(actual!==row.expected_verdict)return[null,"verdict-mismatch"];
  }
  if(typeof obj.schema!=="string"||!obj.schema)return[null,"report-schema-missing"];
  return[{sha256:sha(raw),case_count:n,schema:obj.schema,case_ids_sha256:sha(Buffer.from(canonical(ids),"utf8"))},null];
}
function validateReceipt(receiptPath,root,eventPath){
  let receipt,event;try{receipt=readJson(receiptPath);event=readJson(eventPath);}catch(e){return[null,"input-json:"+e.message];}
  const wr=event.workflow_run;
  if(!wr||typeof wr!=="object")return[null,"workflow-run-event-missing"];
  if(!receipt||typeof receipt!=="object"||Object.keys(receipt).sort().join("|")!==["claim_ceiling","evidence","schema","source","status"].sort().join("|"))return[null,"receipt-envelope"];
  if(receipt.schema!==RECEIPT_SCHEMA)return[null,"receipt-schema"];
  if(receipt.status!=="research-evidence-only")return[null,"receipt-status"];
  const src=receipt.source,evidence=receipt.evidence,ceiling=receipt.claim_ceiling;
  if(!src||typeof src!=="object"||Object.keys(src).sort().join("|")!==["repository","workflow","workflow_ref","event_name","ref","checked_out_commit_sha","event_sha","pull_request_head_sha","pull_request_number","run_id","run_number","run_attempt"].sort().join("|"))return[null,"source-schema"];
  if(!evidence||typeof evidence!=="object"||Object.keys(evidence).sort().join("|")!==["report_pair_count","report_file_count","report_pairs","supporting_inputs","generated_corpus_a_equals_b"].sort().join("|"))return[null,"evidence-schema"];
  if(!ceiling||typeof ceiling!=="object"||Object.keys(ceiling).sort().join("|")!==["prior_verifier_steps_succeeded_at_receipt_creation","overall_workflow_conclusion","hosted_qualification_pass_claimed","qualification_authority","scitt_interoperability_claimed","live_network_convergence_claimed"].sort().join("|"))return[null,"claim-ceiling-schema"];
  if(!event.repository||event.repository.full_name!==REPOSITORY||src.repository!==REPOSITORY)return[null,"repository-binding"];
  if(!wr.repository||wr.repository.full_name!==REPOSITORY)return[null,"source-run-repository"];
  if(!wr.head_repository||wr.head_repository.full_name!==REPOSITORY)return[null,"source-run-head-repository"];
  if(wr.name!==WORKFLOW_NAME||String(wr.path||"").split("@")[0]!==WORKFLOW_PATH)return[null,"source-workflow-binding"];
  if(wr.event!=="push"||src.event_name!=="push")return[null,"source-event-not-push"];
  if(wr.head_branch!=="main"||src.ref!=="refs/heads/main")return[null,"source-branch-not-main"];
  if(wr.conclusion!=="success")return[null,"source-workflow-not-success"];
  const items=evidence.report_pairs;
  if(!Array.isArray(items)||items.length!==PAIRS.length||evidence.report_pair_count!==PAIRS.length)return[null,"report-pair-inventory"];
  if(evidence.report_file_count!==2*PAIRS.length)return[null,"report-file-count"];
  if(evidence.generated_corpus_a_equals_b!==true)return[null,"generated-corpus-identity-claim"];
  const support=evidence.supporting_inputs;
  if(!Array.isArray(support)||support.length!==SUPPORTING.length||new Set(support.map(x=>x?.name)).size!==SUPPORTING.length||SUPPORTING.some(([name])=>!support.some(x=>x?.name===name)))return[null,"supporting-input-inventory"];
  const expectedNames=new Set(PAIRS.flatMap(([,p,n])=>[p,n])),seen=new Set();
  const expectedByLayer=new Map(PAIRS.map(([name,p,n,count])=>[name,{p,n,count}]));
  const summary=[];
  const itemKeys=["name","python_file","node_file","sha256","schema","case_count","case_ids_sha256","failure_count","python_node_byte_identical"].sort().join("|");
  for(const item of items){
    if(!item||typeof item!=="object"||Object.keys(item).sort().join("|")!==itemKeys)return[null,"report-item-schema"];
    const layer=item.name,exp=expectedByLayer.get(layer);
    if(!exp)return[null,"unknown-report-layer"];
    if(item.python_file!=="reports/"+exp.p||item.node_file!=="reports/"+exp.n)return[null,"report-path-substitution:"+layer];
    if(item.python_node_byte_identical!==true||item.failure_count!==0)return[null,"report-parity-claim:"+layer];
    for(const [field,filename] of [["python_file",exp.p],["node_file",exp.n]]){
      const rel=item[field];if(seen.has(rel))return[null,"duplicate-report-path"];seen.add(rel);
      const file=path.join(root,rel);if(!fs.existsSync(file))return[null,"report-missing:"+filename];
      let meta,err;try{[meta,err]=validateReport(file,exp.count);}catch(e){return[null,filename+":json:"+e.message];}
      if(err)return[null,filename+":"+err];
      if(meta.sha256!==item.sha256)return[null,"report-sha:"+filename];
      if(meta.schema!==item.schema||meta.case_count!==item.case_count||meta.case_ids_sha256!==item.case_ids_sha256)return[null,"report-metadata:"+filename];
    }
    if(!fs.readFileSync(path.join(root,"reports",exp.p)).equals(fs.readFileSync(path.join(root,"reports",exp.n))))return[null,"python-node-report-mismatch:"+layer];
    summary.push({name:layer,sha256:item.sha256,case_count:item.case_count,report_schema:item.schema});
  }
  if(seen.size!==expectedNames.size||[...expectedNames].some(x=>!seen.has(x)))return[null,"report-inventory-mismatch"];
  for(const[name,rel,count]of SUPPORTING){
    const file=path.join(root,rel);if(!fs.existsSync(file))return[null,"supporting-input-missing:"+name];
    const raw=fs.readFileSync(file);let obj;try{obj=JSON.parse(raw.toString("utf8"));}catch{return[null,"supporting-input-json:"+name];}
    const cases=Array.isArray(obj)?obj:(obj&&typeof obj==="object"?obj.cases:null);
    if(!Array.isArray(cases)||cases.length!==count)return[null,"supporting-input-case-count:"+name];
    const pin=support.find(x=>x.name===name);
    if(!pin||pin.file!==rel||pin.sha256!==sha(raw)||pin.case_count!==count)return[null,"supporting-input-pin:"+name];
  }
  if(!fs.readFileSync(path.join(root,"supporting/generated-a.json")).equals(fs.readFileSync(path.join(root,"supporting/generated-b.json"))))return[null,"generated-corpus-not-deterministic"];
  if(src.workflow!==WORKFLOW_NAME)return[null,"source-workflow-name-binding"];
  const expectedSource={
    repository:REPOSITORY,workflow:WORKFLOW_NAME,event_name:"push",ref:"refs/heads/main",
    checked_out_commit_sha:wr.head_sha,event_sha:wr.head_sha,
    run_id:wr.id,run_number:wr.run_number,run_attempt:wr.run_attempt??1
  };
  for(const[k,v]of Object.entries(expectedSource))if(src[k]!==v)return[null,"source-metadata-binding:"+k];
  if(src.pull_request_head_sha!==null||src.pull_request_number!==null)return[null,"unexpected-pr-head"];
  if(src.workflow_ref!==REPOSITORY+"/"+WORKFLOW_PATH+"@refs/heads/main")return[null,"source-workflow-ref-binding"];
  if(ceiling.prior_verifier_steps_succeeded_at_receipt_creation!==true||ceiling.overall_workflow_conclusion!=="pending-downstream-observation")return[null,"claim-ceiling-context"];
  for(const key of ["hosted_qualification_pass_claimed","qualification_authority","scitt_interoperability_claimed","live_network_convergence_claimed"])if(ceiling[key]!==false)return[null,"qualification-claim-injection"];
  const predicate={
    schema:PREDICATE_SCHEMA,
    evidence_type:"mycelix-anchor-transparency-execution-receipt",
    source_workflow_run:{
      repository:REPOSITORY,workflow_name:wr.name,workflow_path:String(wr.path).split("@")[0],
      workflow_id:wr.workflow_id,run_id:wr.id,run_number:wr.run_number,run_attempt:wr.run_attempt??1,
      event:"push",branch:"main",head_sha:wr.head_sha,conclusion:"success"
    },
    receipt_sha256:sha(fs.readFileSync(receiptPath)),
    report_pairs:summary,report_pair_count:summary.length,report_file_count:seen.size,
    claim_ceiling:{
      qualification_decision:"not-claimed",hosted_qualification_pass:false,
      complete_scitt_interoperability:false,live_network_convergence:false,
      organizational_independence:false,private_key_custody_proven:false
    }
  };
  return[predicate,null];
}
function selfTest(){
  const sample={schema:"test.v1",case_count:2,cases:[{case_id:"a"},{case_id:"b"}],failures:[]};
  if(!validateReportBytes(Buffer.from(canonical(sample)),2)[0])throw Error("valid-report-rejected");
  const legacy={schema:"test.v1",cases:[{case_id:"a"},{case_id:"b"}],failures:[]};
  const [legacyMeta,legacyErr]=validateReportBytes(Buffer.from(canonical(legacy)),2);if(legacyErr||legacyMeta.case_count!==2)throw Error("legacy-report-without-count-rejected");
  const badLegacy={...legacy,case_count:3};if(validateReportBytes(Buffer.from(canonical(badLegacy)),2)[1]!=="case-count-mismatch")throw Error("incorrect-explicit-count-accepted");
  const fail=structuredClone(sample);fail.failures=[{case_id:"a"}];if(validateReportBytes(Buffer.from(canonical(fail)),2)[1]!=="reported-failures")throw Error("failures-accepted");
  const dup=structuredClone(sample);dup.cases=[{case_id:"a"},{case_id:"a"}];if(validateReportBytes(Buffer.from(canonical(dup)),2)[1]!=="duplicate-case-id")throw Error("duplicate-IDs-accepted");
  const mismatch=structuredClone(sample);mismatch.cases=[{case_id:"a",expected_verdict:"qualified",actual_verdict:"unresolved"},{case_id:"b"}];if(validateReportBytes(Buffer.from(canonical(mismatch)),2)[1]!=="verdict-mismatch")throw Error("mismatched-verdict-accepted");
  const missingActual=structuredClone(sample);missingActual.cases=[{case_id:"a",expected_verdict:"qualified"},{case_id:"b"}];if(validateReportBytes(Buffer.from(canonical(missingActual)),2)[1]!=="actual-verdict-missing")throw Error("missing-actual-verdict-accepted");
  const base=fs.mkdtempSync(path.join(os.tmpdir(),"mycelix-receipt-selftest-"));
  try{
    const artifact=path.join(base,"artifact"),reports=path.join(artifact,"reports"),support=path.join(artifact,"supporting");
    fs.mkdirSync(reports,{recursive:true});fs.mkdirSync(support,{recursive:true});
    const pairs=[];
    for(const [layer,pyname,nodename,count] of PAIRS){
      if(count===null)throw Error("self-test requires fixed count for "+layer);
      const ids=Array.from({length:count},(_,i)=>layer+"-"+String(i).padStart(3,"0"));
      const value={schema:"self-test."+layer,status:"research-evidence-only",case_count:count,
        cases:ids.map(case_id=>({case_id,expected_verdict:"qualified",actual_verdict:"qualified"})),failures:[]};
      const raw=Buffer.from(canonical(value)+"\n");
      fs.writeFileSync(path.join(reports,pyname),raw);fs.writeFileSync(path.join(reports,nodename),raw);
      const [meta,err]=validateReportBytes(raw,count);if(err)throw Error("synthetic report invalid:"+layer+":"+err);
      pairs.push({name:layer,python_file:"reports/"+pyname,node_file:"reports/"+nodename,
        sha256:meta.sha256,schema:meta.schema,case_count:meta.case_count,case_ids_sha256:meta.case_ids_sha256,
        failure_count:0,python_node_byte_identical:true});
    }
    const supportPins=[];
    for(const [name,rel,count] of SUPPORTING){
      const value={schema:"self-test.generated-corpus.v1",cases:Array.from({length:count},(_,i)=>({case_id:"generated-"+String(i).padStart(3,"0")}))};
      const raw=Buffer.from(canonical(value)+"\n");fs.writeFileSync(path.join(artifact,rel),raw);
      supportPins.push({name,file:rel,sha256:sha(raw),case_count:count});
    }
    const head="a".repeat(40);
    const wr={repository:{full_name:REPOSITORY},head_repository:{full_name:REPOSITORY},name:WORKFLOW_NAME,path:WORKFLOW_PATH,
      event:"push",head_branch:"main",conclusion:"success",head_sha:head,id:12345,run_number:77,run_attempt:2,workflow_id:888};
    const event={repository:{full_name:REPOSITORY},workflow_run:wr},eventPath=path.join(base,"event.json");
    fs.writeFileSync(eventPath,canonical(event)+"\n");
    const receipt={schema:RECEIPT_SCHEMA,status:"research-evidence-only",
      source:{repository:REPOSITORY,workflow:WORKFLOW_NAME,workflow_ref:REPOSITORY+"/"+WORKFLOW_PATH+"@refs/heads/main",
        event_name:"push",ref:"refs/heads/main",checked_out_commit_sha:head,event_sha:head,
        pull_request_head_sha:null,pull_request_number:null,run_id:12345,run_number:77,run_attempt:2},
      evidence:{report_pair_count:pairs.length,report_file_count:pairs.length*2,report_pairs:pairs,
        supporting_inputs:supportPins,generated_corpus_a_equals_b:true},
      claim_ceiling:{prior_verifier_steps_succeeded_at_receipt_creation:true,overall_workflow_conclusion:"pending-downstream-observation",
        hosted_qualification_pass_claimed:false,qualification_authority:false,scitt_interoperability_claimed:false,live_network_convergence_claimed:false}};
    const receiptPath=path.join(base,"receipt.json"),writeReceipt=v=>fs.writeFileSync(receiptPath,canonical(v)+"\n");
    writeReceipt(receipt);
    let [,err]=validateReceipt(receiptPath,artifact,eventPath);if(err)throw Error("valid synthetic receipt rejected:"+err);
    fs.writeFileSync(eventPath,canonical({...event,workflow_run:{...wr,conclusion:"failure"}})+"\n");
    [,err]=validateReceipt(receiptPath,artifact,eventPath);if(err!=="source-workflow-not-success")throw Error("failed workflow accepted:"+err);
    fs.writeFileSync(eventPath,canonical(event)+"\n");
    const forkEvent={...event,workflow_run:{...wr,head_repository:{full_name:"attacker/fork"},head_branch:"main"}};
    fs.writeFileSync(eventPath,canonical(forkEvent)+"\n");
    [,err]=validateReceipt(receiptPath,artifact,eventPath);if(err!=="source-run-head-repository")throw Error("fork with main-named branch accepted:"+err);
    fs.writeFileSync(eventPath,canonical(event)+"\n");
    const spoof=structuredClone(receipt);spoof.source.workflow_ref="attacker/repo/"+WORKFLOW_PATH+"@refs/heads/main";writeReceipt(spoof);
    [,err]=validateReceipt(receiptPath,artifact,eventPath);if(err!=="source-workflow-ref-binding")throw Error("cross-repository workflow ref accepted:"+err);
    writeReceipt(receipt);
    const tampered=path.join(reports,"python-fixed.json");fs.writeFileSync(tampered,Buffer.concat([fs.readFileSync(tampered),Buffer.from(" ")]));
    [,err]=validateReceipt(receiptPath,artifact,eventPath);if(err!=="report-sha:python-fixed.json")throw Error("tampered report accepted:"+err);
    const clean={schema:"self-test.fixed-classification",status:"research-evidence-only",case_count:52,
      cases:Array.from({length:52},(_,i)=>({case_id:"fixed-classification-"+String(i).padStart(3,"0"),expected_verdict:"qualified",actual_verdict:"qualified"})),failures:[]};
    fs.writeFileSync(tampered,canonical(clean)+"\n");
    const changed=structuredClone(receipt);changed.claim_ceiling.hosted_qualification_pass_claimed=true;writeReceipt(changed);
    [,err]=validateReceipt(receiptPath,artifact,eventPath);if(err!=="qualification-claim-injection")throw Error("claim injection accepted:"+err);
    const extra={...receipt,untrusted_extra_field:"present"};writeReceipt(extra);
    [,err]=validateReceipt(receiptPath,artifact,eventPath);if(err!=="receipt-envelope")throw Error("extra receipt field accepted:"+err);
  }finally{fs.rmSync(base,{recursive:true,force:true});}
  console.log("execution-evidence-verifier-self-test=pass");
}
function validateReportBytes(raw,expected){
  const obj=JSON.parse(raw.toString("utf8")),cases=obj.cases,failures=obj.failures,explicitCount=Object.hasOwn(obj,"case_count"),n=explicitCount?obj.case_count:(Array.isArray(cases)?cases.length:null);
  if(!Array.isArray(cases)||!cases.length)return[null,"empty-or-missing-cases"];
  if(explicitCount&&(!Number.isInteger(n)||n!==cases.length))return[null,"case-count-mismatch"];
  if(expected!==null&&n!==expected)return[null,"unexpected-case-count"];
  if(!Array.isArray(failures)||failures.length)return[null,"reported-failures"];
  const ids=cases.map(x=>x&&typeof x==="object"?(x.case_id??x.id):null);
  if(ids.some(x=>typeof x!=="string"||!x))return[null,"case-identity-missing"];
  if(new Set(ids).size!==ids.length)return[null,"duplicate-case-id"];
  for(const row of cases)if(row&&typeof row==="object"&&Object.hasOwn(row,"expected_verdict")){
    const actual=Object.hasOwn(row,"actual_verdict")?row.actual_verdict:row.verdict;
    if(actual===undefined||actual===null)return[null,"actual-verdict-missing"];
    if(actual!==row.expected_verdict)return[null,"verdict-mismatch"];
  }
  if(typeof obj.schema!=="string"||!obj.schema)return[null,"report-schema-missing"];
  return[{sha256:sha(raw),case_count:n,schema:obj.schema,case_ids_sha256:sha(Buffer.from(canonical(ids),"utf8"))},null];
}
const args=process.argv.slice(2);
if(args[0]==="self-test"){selfTest();process.exit(0);}
if(args[0]!=="verify"){console.error("usage: verifier self-test | verify --artifact-root DIR --receipt FILE --event JSON --predicate FILE");process.exit(2);}
const opts={};for(let i=1;i<args.length;i++){if(args[i].startsWith("--"))opts[args[i].slice(2)]=args[++i];}
for(const k of ["artifact-root","receipt","event","predicate"])if(!opts[k])process.exit(2);
const[predicate,err]=validateReceipt(opts.receipt,opts["artifact-root"],opts.event);
if(err){console.error("execution-evidence-rejected:"+err);process.exit(1);}
fs.writeFileSync(opts.predicate,canonical(predicate)+"\n");
console.log("execution-evidence-predicate=validated");
