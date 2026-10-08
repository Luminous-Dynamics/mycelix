#!/usr/bin/env node
import fs from "node:fs";
import crypto from "node:crypto";

const FIXTURE_SCHEMA="mycelix.continual-adaptation.censoring-classification-anchor-observer-gossip-simulation.v1";
const CAMPAIGN_SCHEMA="mycelix.continual-adaptation.censoring-classification-anchor-observer-gossip-simulation-campaign.v1";
const GREG_SCHEMA="mycelix.continual-adaptation.censoring-classification-anchor-observer-gossip-registry.v1";
const OBS_SCHEMA="mycelix.continual-adaptation.censoring-classification-anchor-observer-gossip-observation.v1";
const GDOMAIN="mycelix.continual-adaptation.censoring-classification-anchor-observer-gossip.v1";
const HEAD_SCHEMA="mycelix.continual-adaptation.censoring-classification-anchor-witness-vds-tree-head.v1";
const GREG_ID="mycelix.research.anchor-observer-gossip-registry.v1";
const WREG_ID="mycelix.research.anchor-witness-registry.v2";
const VDS_ID="mycelix.research.anchor-statement-sequence.v1";
const EXPECTED_GREG_SHA="sha256:df41b280e6250f28425f2944f20e417bb69fa522eed9c52ca6b45d6b7931dd8e";
const ALG="Ed25519";

const canonical=v=>v===null||typeof v!=="object"?JSON.stringify(v):Array.isArray(v)?"["+v.map(canonical).join(",")+"]":"{"+Object.keys(v).sort().map(k=>JSON.stringify(k)+":"+canonical(v[k]))+"}";
const H=b=>crypto.createHash("sha256").update(b).digest();
const digestObj=v=>"sha256:"+H(Buffer.from(canonical(v),"utf8")).toString("hex");
const b64u=s=>{if(typeof s!=="string"||!/^[A-Za-z0-9_-]+$/.test(s))throw Error("base64url");return Buffer.from(s,"base64url");};
const nodeHash=(a,b)=>H(Buffer.concat([Buffer.from([1]),a,b]));
const leafHash=b=>H(Buffer.concat([Buffer.from([0]),b]));
function mth(entries){if(!entries.length)return H(Buffer.alloc(0));if(entries.length===1)return leafHash(Buffer.from(canonical(entries[0]),"utf8"));const k=1<<((entries.length-1).toString(2).length-1);return nodeHash(mth(entries.slice(0,k)),mth(entries.slice(k)));}
function publicKey(raw){return crypto.createPublicKey({key:Buffer.concat([Buffer.from("302a300506032b6570032100","hex"),raw]),format:"der",type:"spki"});}
function verifyEd(pub,sig,msg){try{return crypto.verify(null,msg,publicKey(pub),sig)}catch{return false;}}

function verifyGossipRegistry(reg){
  if(reg.schema!==GREG_SCHEMA||reg.status!=="research-fixture-only")return"registry-schema";
  if(reg.registry_id!==GREG_ID||reg.registry_version!==1)return"registry-identity";
  if(reg.algorithm!==ALG||reg.domain!==GDOMAIN)return"registry-profile";
  if(!reg.monitors||typeof reg.monitors!=="object"||Array.isArray(reg.monitors)||!Object.keys(reg.monitors).length)return"registry-monitors";
  const seen=new Set();
  for(const[m,x]of Object.entries(reg.monitors)){
    if(x.status!=="active"||seen.has(x.key_id))return"monitor-schema";
    seen.add(x.key_id);try{if(b64u(x.public_key).length!==32)throw 0;}catch{return"monitor-public-key";}
  }
  return null;
}
function verifyGossip(obs,reg){
  const fields=["schema","domain","algorithm","monitor_id","key_id","registry_id","registry_version","vds_id","subject_observer_id","subject_head_sha256","observed_tree_size","observed_root_hash","observation_sequence","signature"].sort();
  if(!obs||typeof obs!=="object"||Object.keys(obs).sort().join("|")!==fields.join("|"))return[null,"gossip-envelope"];
  if(obs.schema!==OBS_SCHEMA)return[null,"gossip-schema"];
  if(obs.domain!==GDOMAIN)return[null,"gossip-domain"];
  if(obs.algorithm!==ALG)return[null,"gossip-algorithm"];
  if(obs.registry_id!==GREG_ID||obs.registry_version!==1)return[null,"gossip-registry-binding"];
  if(obs.vds_id!==VDS_ID)return[null,"gossip-vds-binding"];
  const monitor=reg.monitors?.[obs.monitor_id];
  if(!monitor||monitor.status!=="active"||monitor.key_id!==obs.key_id)return[null,"monitor-key"];
  let pub,sig;try{pub=b64u(monitor.public_key);sig=b64u(obs.signature);}catch{return[null,"gossip-signature-encoding"];}
  if(pub.length!==32||sig.length!==64)return[null,"gossip-signature-encoding"];
  const payload={...obs};delete payload.signature;
  return verifyEd(pub,sig,Buffer.from(canonical(payload),"utf8"))?[obs,null]:[null,"gossip-signature-invalid"];
}
function witnessKey(reg,w,kid,version){
  const wi=reg.witnesses?.[w],key=wi?.keys?.[kid];if(!key)return[null,"unknown-witness-or-key"];
  if(key.algorithm!=="Ed25519")return[null,"witness-key-algorithm"];
  if(version<key.valid_from_version)return[null,"witness-key-not-yet-valid"];
  if(key.valid_until_version!==null&&version>key.valid_until_version)return[null,"witness-key-expired"];
  if(key.revoked_at_version!==null&&version>=key.revoked_at_version)return[null,"witness-key-revoked"];
  if(key.status==="revoked")return[null,"witness-key-revoked"];
  try{const raw=b64u(key.public_key);return raw.length===32?[raw,null]:[null,"witness-key-encoding"];}catch{return[null,"witness-key-encoding"];}
}
function verifySubjectHead(head,reg){
  const fields=["schema","domain","algorithm","observer_id","key_id","registry_id","registry_version","vds_id","manifest_version","tree_size","root_hash","signature"].sort();
  if(!head||typeof head!=="object"||Object.keys(head).sort().join("|")!==fields.join("|"))return[false,"head-schema"];
  if(head.schema!==HEAD_SCHEMA||head.domain!==HEAD_SCHEMA)return[false,"head-schema-or-domain"];
  if(head.algorithm!==ALG)return[false,"head-algorithm"];
  if(head.registry_id!==WREG_ID||head.registry_version!==2)return[false,"head-registry-binding"];
  if(head.vds_id!==VDS_ID)return[false,"head-vds-binding"];
  const wi=reg.witnesses?.[head.observer_id];if(!wi)return[false,"unknown-witness"];
  const[pub,e]=witnessKey(reg,head.observer_id,head.key_id,head.manifest_version);if(e)return[false,e];
  let sig;try{sig=b64u(head.signature);}catch{return[false,"head-signature-encoding"];}
  if(sig.length!==64)return[false,"head-signature-encoding"];
  const payload={schema:HEAD_SCHEMA,domain:HEAD_SCHEMA,algorithm:ALG,observer_id:head.observer_id,key_id:head.key_id,witness_identity_commitment:wi.identity_commitment,registry_id:head.registry_id,registry_version:head.registry_version,vds_id:head.vds_id,manifest_version:head.manifest_version,tree_size:head.tree_size,root_hash:head.root_hash};
  return verifyEd(pub,sig,Buffer.from(canonical(payload),"utf8"))?[true,null]:[false,"head-signature-invalid"];
}
function verifyTreeHead(head,w,reg,entries){
  const fields=["schema","domain","algorithm","observer_id","key_id","registry_id","registry_version","vds_id","manifest_version","tree_size","root_hash","signature"].sort();
  if(!head||typeof head!=="object"||Object.keys(head).sort().join("|")!==fields.join("|"))return[null,"head-schema"];
  if(head.schema!==HEAD_SCHEMA||head.domain!==HEAD_SCHEMA)return[null,"head-domain"];
  if(head.algorithm!==ALG)return[null,"head-algorithm"];
  if(head.observer_id!==w)return[null,"head-observer-binding"];
  if(head.registry_id!==WREG_ID||head.registry_version!==2)return[null,"head-registry-binding"];
  if(head.vds_id!==VDS_ID)return[null,"head-vds-binding"];
  const n=head.tree_size;if(!Number.isInteger(n)||n<1||n>entries.length)return[null,"head-tree-size"];
  if(!/^sha256:[0-9a-f]{64}$/.test(head.root_hash)||mth(entries.slice(0,n)).toString("hex")!==head.root_hash.slice(7))return[null,"head-root-mismatch"];
  const[pub,e]=witnessKey(reg,w,head.key_id,head.manifest_version);if(e)return[null,"head-"+e];
  let sig;try{sig=b64u(head.signature);}catch{return[null,"head-signature-encoding"];}
  const payload={schema:HEAD_SCHEMA,domain:HEAD_SCHEMA,algorithm:ALG,observer_id:w,key_id:head.key_id,witness_identity_commitment:reg.witnesses[w].identity_commitment,registry_id:head.registry_id,registry_version:head.registry_version,vds_id:head.vds_id,manifest_version:head.manifest_version,tree_size:n,root_hash:head.root_hash};
  return verifyEd(pub,sig,Buffer.from(canonical(payload),"utf8"))?[head,null]:[null,"head-signature-invalid"];
}
function validateHeadQuorum(headFixture,wreg,vds,name){
  const heads=headFixture.heads?.[name]?.attestations;if(!heads||Object.keys(heads).length<3)return[null,"head-below-threshold"];
  const vals=[];
  for(const[w,h]of Object.entries(heads)){const[v,e]=verifyTreeHead(h,w,wreg,vds.entries);if(e)return[null,e];vals.push(v);}
  if(new Set(vals.map(h=>JSON.stringify([h.manifest_version,h.tree_size,h.root_hash]))).size!==1)return[null,"head-equivocation"];
  return[{tree_size:vals[0].tree_size,root:Buffer.from(vals[0].root_hash.slice(7),"hex"),claims:vals},null];
}
function lookupHead(gossipFixture,obs){
  return Object.entries(gossipFixture.subject_heads||{}).find(([,head])=>digestObj(head)===obs.subject_head_sha256)??null;
}
function validateObservation(obs,greg,wreg,gossipFixture,vds,headFixture,q4){
  const[,ge]=verifyGossip(obs,greg);if(ge)return[null,ge];
  const found=lookupHead(gossipFixture,obs);if(!found)return[null,"unknown-subject-head"];
  const[headId,head]=found;
  if(head.observer_id!==obs.subject_observer_id||head.tree_size!==obs.observed_tree_size||head.root_hash!==obs.observed_root_hash)return[null,"subject-claim-binding"];
  const[ok,he]=verifySubjectHead(head,wreg);if(!ok)return[null,he];
  if(headId==="w02-fork4"){
    if(head.tree_size!==4||Buffer.compare(Buffer.from(head.root_hash.slice(7),"hex"),q4.root)===0)return[null,"fork-head-not-conflicting"];
  }else if(mth(vds.entries.slice(0,head.tree_size)).toString("hex")!==head.root_hash.slice(7))return[null,"subject-head-root-mismatch"];
  return[{observation:obs,head,head_id:headId},null];
}
function flipSignature(obs){
  const out=structuredClone(obs),raw=b64u(out.signature);raw[raw.length-1]^=1;out.signature=raw.toString("base64url");return out;
}
function viewConflict(view){
  const values=[...view.values()];
  for(let i=0;i<values.length;i++)for(let j=i+1;j<values.length;j++){
    const a=values[i].observation,b=values[j].observation;
    if(a.observed_tree_size===b.observed_tree_size&&a.observed_root_hash!==b.observed_root_hash){
      return a.monitor_id===b.monitor_id?["monitor-equivocation","same-monitor-signed-conflicting-roots"]:["split-view-detected","authenticated-same-size-root-conflict"];
    }
  }
  return[null,null];
}
function compatibleSet(view,gossipFixture){
  const values=[...view.values()];
  if(values.length<2||new Set(values.map(x=>x.observation.monitor_id)).size<2)return[false,"insufficient-independent-observations"];
  const unique=new Set(values.map(x=>JSON.stringify([x.observation.observed_tree_size,x.observation.observed_root_hash])));
  const sizes=[...new Set(values.map(x=>x.observation.observed_tree_size))].sort((a,b)=>a-b);
  if(sizes.length>1){
    const path=gossipFixture.consistency_proof_4_to_7.map(x=>Buffer.from(x.slice(7),"hex"));
    for(let i=0;i<sizes.length-1;i++){
      const low=sizes[i],high=sizes[i+1];
      const lowRoots=[...new Set(values.filter(x=>x.observation.observed_tree_size===low).map(x=>x.observation.observed_root_hash))];
      const highRoots=[...new Set(values.filter(x=>x.observation.observed_tree_size===high).map(x=>x.observation.observed_root_hash))];
      if(lowRoots.length!==1||highRoots.length!==1)return[false,"ambiguous-tree-heads"];
      if(low!==4||high!==7||!consistencyValid(low,high,Buffer.from(lowRoots[0].slice(7),"hex"),Buffer.from(highRoots[0].slice(7),"hex"),path))return[false,"consistency-proof-invalid"];
    }
  }
  return[true,"all-observed-heads-compatible"];
}
function consistencyValid(oldSize,newSize,oldRoot,newRoot,path){
  if(!Number.isInteger(oldSize)||!Number.isInteger(newSize)||!(0<oldSize&&oldSize<newSize)||!path.length||path.some(p=>!Buffer.isBuffer(p)||p.length!==32))return false;
  let fn=oldSize-1,sn=newSize-1;while(fn&1){fn>>=1;sn>>=1;}
  let fr,sr,siblings;if(fn===0){fr=sr=oldRoot;siblings=path;}else{fr=sr=path[0];siblings=path.slice(1);}
  for(const p of siblings){
    if(sn===0)return false;
    if((fn&1)||fn===sn){fr=nodeHash(p,fr);sr=nodeHash(p,sr);if(!(fn&1))while(fn&&!(fn&1)){fn>>=1;sn>>=1;}}
    else sr=nodeHash(sr,p);
    fn>>=1;sn>>=1;
  }
  return sn===0&&Buffer.compare(fr,oldRoot)===0&&Buffer.compare(sr,newRoot)===0;
}
function simulateScenario(name,scenario,mutation,greg,wreg,gossipFixture,vds,headFixture,q4){
  const views={A:new Map(),B:new Map()},pending=[],sent=new Set(),log=[];
  let partitioned=false,duplicates=0,rejected=0,tick=-1;const rejectReasons=[];
  function ingest(agent,obsId,payload){
    const[checked,e]=validateObservation(payload,greg,wreg,gossipFixture,vds,headFixture,q4);
    if(e){rejected++;rejectReasons.push(e);log.push({event:"rejected",agent,observation_id:obsId,reason:e});return;}
    if(views[agent].has(obsId)){duplicates++;log.push({event:"duplicate-suppressed",agent,observation_id:obsId});return;}
    views[agent].set(obsId,checked);log.push({event:"accepted",agent,observation_id:obsId});
  }
  for(const[agent,ids]of Object.entries(scenario.initial_views||{})){
    if(!views[agent])throw Error("unknown-agent");
    for(const id of ids){if(!gossipFixture.observations[id])throw Error("unknown-observation");ingest(agent,id,structuredClone(gossipFixture.observations[id]));}
  }
  for(const event of scenario.events||[]){
    if(!Number.isInteger(event.tick)||event.tick<tick)throw Error("nonmonotonic-event-time");tick=event.tick;
    if(event.action==="partition"){partitioned=true;log.push({event:"partition",tick});}
    else if(event.action==="heal"){
      partitioned=false;const byId=new Map(pending.map(m=>[m.message_id,m]));
      const order=event.delivery_order||[],delivery=order.filter(id=>byId.has(id)).map(id=>byId.get(id));
      for(const m of pending)if(!new Set(order).has(m.message_id))delivery.push(m);
      pending.length=0;
      for(const m of delivery){ingest(m.to,m.observation_id,m.payload);log.push({event:"delivered",tick,message_id:m.message_id});}
    }else if(event.action==="send"){
      const{from,to,observation_id:obsId,message_id:msgId}=event;
      if(!views[from]||!views[to]||from===to||!views[from].has(obsId)||typeof msgId!=="string"||!msgId)throw Error("invalid-send");
      if(sent.has(msgId))throw Error("duplicate-message-id");sent.add(msgId);
      let payload=structuredClone(views[from].get(obsId).observation);
      const tamper=event.tamper_signature||(mutation==="tamper-message-signature"&&obsId==="F4"&&name==="tampered_gossip_message");
      if(tamper)payload=flipSignature(payload);
      const msg={message_id:msgId,from,to,observation_id:obsId,payload,sent_tick:tick};
      if(partitioned){pending.push(msg);log.push({event:"queued",tick,message_id:msgId});}
      else{ingest(to,obsId,payload);log.push({event:"delivered",tick,message_id:msgId});}
    }else throw Error("unknown-event-action");
  }
  let verdict=null,reason=null;
  const conflicts=Object.values(views).map(viewConflict);
  if(conflicts.some(([v])=>v==="monitor-equivocation")){verdict="monitor-equivocation";reason="same-monitor-signed-conflicting-roots";}
  else if(conflicts.some(([v])=>v==="split-view-detected")){verdict="split-view-detected";reason="authenticated-same-size-root-conflict";}
  else{
    const keysA=[...views.A.keys()].sort(),keysB=[...views.B.keys()].sort();
    if(keysA.length===0||keysA.join("|")!==keysB.join("|")){verdict="unresolved-local-only";reason="partition-or-no-cross-view-exchange";}
    else{
      const[ok,why]=compatibleSet(views.A,gossipFixture);
      if(ok){verdict="converged-consistent";reason="views-converged; all-heads-and-consistency-valid";}
      else{verdict="unresolved-local-only";reason=why;}
    }
  }
  const highWater={};
  for(const agent of ["A","B"]){
    const bySubject={};
    for(const entry of views[agent].values()){const o=entry.observation,s=o.subject_observer_id;bySubject[s]=Math.max(bySubject[s]||0,o.observed_tree_size);}
    highWater[agent]=Object.fromEntries(Object.keys(bySubject).sort().map(k=>[k,bySubject[k]]));
  }
  return {
    scenario:name,verdict,reason,
    view_observation_ids:{A:[...views.A.keys()].sort(),B:[...views.B.keys()].sort()},
    unique_observation_counts:{A:views.A.size,B:views.B.size},
    high_water_tree_sizes:highWater,
    duplicates_suppressed:duplicates,
    rejected_messages:rejected,
    rejected_reasons:rejectReasons.sort(),
    pending_messages:pending.length,
    partitioned_at_end:partitioned,
    baseline_size_4_quorum:Object.keys(headFixture.heads.size_4.attestations).length,
    signed_fork_head_count:1
  };
}
const args=process.argv.slice(2);if(args.length!==8){console.error("usage: verifier GOSSIP_REGISTRY SIM_FIXTURE CAMPAIGN GOSSIP_FIXTURE WITNESS_REGISTRY VDS_FIXTURE TREE_HEAD_FIXTURE REPORT");process.exit(2);}
const[gp,sp,cp,gfp,wrp,vdsp,hp,outp]=args;
const greg=JSON.parse(fs.readFileSync(gp,"utf8")),simFixture=JSON.parse(fs.readFileSync(sp,"utf8")),campaign=JSON.parse(fs.readFileSync(cp,"utf8")),gossipFixture=JSON.parse(fs.readFileSync(gfp,"utf8")),wreg=JSON.parse(fs.readFileSync(wrp,"utf8")),vds=JSON.parse(fs.readFileSync(vdsp,"utf8")),headFixture=JSON.parse(fs.readFileSync(hp,"utf8"));
if(simFixture.schema!==FIXTURE_SCHEMA||campaign.schema!==CAMPAIGN_SCHEMA||campaign.case_count!==8||campaign.cases.length!==8)process.exit(1);
if(verifyGossipRegistry(greg)||digestObj(greg)!==EXPECTED_GREG_SHA)process.exit(1);
const[q4,e4]=validateHeadQuorum(headFixture,wreg,vds,"size_4"),[q7,e7]=validateHeadQuorum(headFixture,wreg,vds,"size_7");
if(e4||e7){console.error("tree-head-quorum-preflight="+(e4||e7));process.exit(1);}
const forkHead=gossipFixture.subject_heads["w02-fork4"],[forkOk,forkErr]=verifySubjectHead(forkHead,wreg);
if(!forkOk||forkHead.root_hash===q4.root.toString("hex")){if(!forkOk)console.error("signed-fork-head-preflight="+forkErr);process.exit(1);}
for(const[id,obs]of Object.entries(gossipFixture.observations)){const[,e]=validateObservation(obs,greg,wreg,gossipFixture,vds,headFixture,q4);if(e){console.error("observation-preflight:"+id+"="+e);process.exit(1);}}
const rows=[],failures=[];
for(const c of campaign.cases){
  const scenario=simFixture.scenarios[c.scenario];if(!scenario)process.exit(1);
  const report=simulateScenario(c.scenario,scenario,c.mutation,greg,wreg,gossipFixture,vds,headFixture,q4);
  const row={case_id:c.case_id,expected_verdict:c.expected_verdict,...report};rows.push(row);
  if(row.verdict!==c.expected_verdict)failures.push([c.case_id,c.expected_verdict,row.verdict,row.reason]);
}
fs.writeFileSync(outp,canonical({schema:"mycelix.continual-adaptation.censoring-classification-anchor-observer-gossip-simulation-report.v1",status:"research-evidence-only",case_count:rows.length,cases:rows,failures})+"\n");
console.log("cases="+rows.length+" failures="+failures.length);process.exit(failures.length?1:0);
