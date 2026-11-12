use serde_json::Value;
use sha2::{Digest,Sha256};
use std::{collections::{BTreeMap,BTreeSet},env,fs,process::ExitCode};

const ID:&str="DELIB-DIST-INFO-001";
const N:usize=18;
const FAMILIES:[&str;6]=["complementary_facts","minority_decisive_fact","high_redundancy","silent_late_holder","stale_conflicting_evidence","rights_constrained_evidence"];
fn arr<'a>(v:&'a Value,k:&str)->&'a [Value]{v.get(k).and_then(Value::as_array).map(Vec::as_slice).unwrap_or(&[])}
fn s<'a>(v:&'a Value,k:&str)->Option<&'a str>{v.get(k)?.as_str().filter(|x|!x.trim().is_empty())}
fn num(v:&Value,k:&str)->Option<i64>{v.get(k)?.as_i64()}
fn ids(v:&Value)->BTreeSet<String>{v.as_array().into_iter().flatten().filter_map(Value::as_str).map(str::to_owned).collect()}
fn exact(v:&Value,keys:&[&str])->bool{
 let Some(m)=v.as_object() else{return false;};
 let got:BTreeSet<&str>=m.keys().map(String::as_str).collect();
 let want:BTreeSet<&str>=keys.iter().copied().collect();got==want
}
fn digest(v:&Value)->Result<String,String>{let b=serde_json::to_vec(v).map_err(|e|e.to_string())?;Ok(format!("{:x}",Sha256::digest(b)))}
fn allowed(r:&str)->bool{matches!(r,"shareable"|"shareable_with_scope_limit")}
fn assessment(v:&Value)->Result<&'static str,String>{
 let id=s(v,"scenario_id").ok_or("scenario ID missing")?;
 let agents=ids(&v["agents"]);if agents.len()!=arr(v,"agents").len()||agents.len()<3{return Err(format!("{id}: agent IDs must be unique and include at least three agents"));}
 let facts=arr(v,"facts");
 let mut fmap:BTreeMap<String,&Value>=BTreeMap::new();
 for f in facts{
  if !exact(f,&["fact_id","claim_id","holder_agent_id","visibility","decision_role","polarity","rights_status","version","evidence_status","initial_verification_status","source_origin_id","duplicate_of"]){return Err(format!("{id}: fact record has missing or extra fields"));}
  let fid=s(f,"fact_id").ok_or(format!("{id}: fact ID missing"))?.to_owned();
  if fmap.insert(fid.clone(),f).is_some(){return Err(format!("{id}: duplicate fact ID {fid}"));}
  if !agents.contains(s(f,"holder_agent_id").unwrap_or("")){return Err(format!("{id}: fact holder is not a declared agent"));}
  if !["shared","private"].contains(&s(f,"visibility").unwrap_or("")){return Err(format!("{id}: invalid visibility"));}
  if !["critical","supporting","decoy","irrelevant"].contains(&s(f,"decision_role").unwrap_or("")){return Err(format!("{id}: invalid decision role"));}
  if !["supports_A","supports_B","neutral"].contains(&s(f,"polarity").unwrap_or("")){return Err(format!("{id}: invalid polarity"));}
  if !["shareable","shareable_with_scope_limit","withheld_by_rights","unknown"].contains(&s(f,"rights_status").unwrap_or("")){return Err(format!("{id}: invalid rights status"));}
  if num(f,"version").unwrap_or(0)<1||s(f,"source_origin_id").is_none(){return Err(format!("{id}: version/origin missing"));}
  if !["current","superseded","disputed"].contains(&s(f,"evidence_status").unwrap_or("")){return Err(format!("{id}: invalid evidence status"));}
  if !["verified","unverified","unknown"].contains(&s(f,"initial_verification_status").unwrap_or("")){return Err(format!("{id}: invalid initial verification status"));}
 }
 for (fid,f) in &fmap{
  if let Some(parent)=f["duplicate_of"].as_str(){
   let p=fmap.get(parent).ok_or(format!("{id}: duplicate parent {parent} is missing"))?;
   if s(f,"source_origin_id")!=s(p,"source_origin_id"){return Err(format!("{id}: duplicate {fid} does not preserve original source identity"));}
  }
 }
 let views=arr(v,"initial_views");if views.len()!=agents.len(){return Err(format!("{id}: exactly one initial view per agent is required"));}
 let mut viewed_agents=BTreeSet::new();
 for view in views{
  if !exact(view,&["agent_id","visible_fact_ids"]){return Err(format!("{id}: candidate view has forbidden fields"));}
  let aid=s(view,"agent_id").ok_or(format!("{id}: view agent missing"))?;
  if !agents.contains(aid)||!viewed_agents.insert(aid.to_owned()){return Err(format!("{id}: invalid or duplicate initial view"));}
  let visible=ids(&view["visible_fact_ids"]);if visible.len()!=arr(view,"visible_fact_ids").len(){return Err(format!("{id}: duplicate visible fact IDs"));}
  for fid in &visible{
   let f=fmap.get(fid).ok_or(format!("{id}: candidate view references unknown fact {fid}"))?;
   if s(f,"visibility")!=Some("shared")&&s(f,"holder_agent_id")!=Some(aid){return Err(format!("{id}: private fact {fid} leaked to non-holder {aid}"));}
  }
  for (fid,f) in &fmap{
   let should=s(f,"visibility")==Some("shared")||s(f,"holder_agent_id")==Some(aid);
   if visible.contains(fid)!=should{return Err(format!("{id}: initial visibility contract mismatch for {fid}/{aid}"));}
  }
 }
 let mut reqs:BTreeMap<String,&Value>=BTreeMap::new();
 for r in arr(v,"requests"){
  if !exact(r,&["request_id","requester_agent_id","target_agent_id","fact_id","delivery_status"]){return Err(format!("{id}: request has missing or extra fields"));}
  let rid=s(r,"request_id").ok_or(format!("{id}: request ID missing"))?.to_owned();
  if reqs.insert(rid.clone(),r).is_some(){return Err(format!("{id}: duplicate request ID {rid}"));}
  if !agents.contains(s(r,"requester_agent_id").unwrap_or(""))||!agents.contains(s(r,"target_agent_id").unwrap_or("")){return Err(format!("{id}: requester/target agent missing"));}
  let fid=s(r,"fact_id").ok_or(format!("{id}: request fact missing"))?;
  let f=fmap.get(fid).ok_or(format!("{id}: request for unknown fact"))?;
  if s(r,"target_agent_id")!=s(f,"holder_agent_id"){return Err(format!("{id}: query was not routed to the assigned holder for {fid}"));}
  if !["created","delivered","not_delivered"].contains(&s(r,"delivery_status").unwrap_or("")){return Err(format!("{id}: invalid delivery state"));}
 }
 let mut responses:BTreeMap<String,&Value>=BTreeMap::new();
 for r in arr(v,"responses"){
  if !exact(r,&["response_id","request_id","responder_agent_id","fact_id","status"]){return Err(format!("{id}: response has missing or extra fields"));}
  let rid=s(r,"request_id").ok_or(format!("{id}: response request missing"))?;
  let req=reqs.get(rid).ok_or(format!("{id}: response refers to unknown request"))?;
  if s(req,"delivery_status")!=Some("delivered"){return Err(format!("{id}: response exists without delivered request"));}
  if s(r,"responder_agent_id")!=s(req,"target_agent_id")||s(r,"fact_id")!=s(req,"fact_id"){return Err(format!("{id}: response holder/fact does not match request"));}
  if responses.insert(rid.to_owned(),r).is_some(){return Err(format!("{id}: duplicate response for request"));}
  if !["disclosed","withheld_by_rights","no_response","disputed"].contains(&s(r,"status").unwrap_or("")){return Err(format!("{id}: invalid response state"));}
 }
 let mut receipts_by_fact:BTreeMap<String,Vec<&Value>>=BTreeMap::new();
 let mut receipt_ids=BTreeSet::new();
 for r in arr(v,"receipts"){
  if !exact(r,&["receipt_id","request_id","holder_agent_id","fact_id","evidence_id","source_origin_id","version","rights_status","verification_status","duplicate_of"]){return Err(format!("{id}: receipt has missing or extra fields"));}
  let rid=s(r,"request_id").ok_or(format!("{id}: receipt request missing"))?;
  let req=reqs.get(rid).ok_or(format!("{id}: receipt refers to unknown request"))?;
  let response=responses.get(rid).ok_or(format!("{id}: receipt without response"))?;
  if s(req,"delivery_status")!=Some("delivered")||s(response,"status")!=Some("disclosed"){return Err(format!("{id}: receipt does not follow delivered disclosure"));}
  let fid=s(r,"fact_id").ok_or(format!("{id}: receipt fact missing"))?;
  if s(req,"fact_id")!=Some(fid)||s(response,"fact_id")!=Some(fid){return Err(format!("{id}: receipt fact differs from request/response"));}
  let f=fmap.get(fid).ok_or(format!("{id}: receipt for unknown fact"))?;
  if s(r,"holder_agent_id")!=s(f,"holder_agent_id")||s(r,"source_origin_id")!=s(f,"source_origin_id")
    ||num(r,"version")!=num(f,"version")||s(r,"rights_status")!=s(f,"rights_status")
    ||r["duplicate_of"]!=f["duplicate_of"]{return Err(format!("{id}: receipt does not bind holder/source/version/rights exactly"));}
  let rid2=s(r,"receipt_id").ok_or(format!("{id}: receipt ID missing"))?;
  if !receipt_ids.insert(rid2.to_owned()){return Err(format!("{id}: duplicate receipt ID"));}
  receipts_by_fact.entry(fid.to_owned()).or_default().push(r);
 }
 let decision=&v["candidate_decision"];
 if !exact(decision,&["disposition","selected_option","completeness_claim","admitted_fact_ids","unresolved_claim_ids","reported_independent_source_count"]){return Err(format!("{id}: candidate decision has missing or extra fields"));}
 let disposition=s(decision,"disposition").ok_or(format!("{id}: decision disposition missing"))?;
 if !["decided","deferred","abstained"].contains(&disposition){return Err(format!("{id}: invalid decision disposition"));}
 let selected=decision["selected_option"].as_str();
 if disposition=="decided"&& !["A","B"].contains(&selected.unwrap_or("")){return Err(format!("{id}: decided state requires A or B"));}
 if disposition!="decided"&&selected.is_some(){return Err(format!("{id}: deferred/abstained state must not select an option"));}
 let completeness=s(decision,"completeness_claim").ok_or(format!("{id}: completeness claim missing"))?;
 if !["complete","incomplete","unknown"].contains(&completeness){return Err(format!("{id}: invalid completeness claim"));}
 let admitted=ids(&decision["admitted_fact_ids"]);
 if admitted.len()!=arr(decision,"admitted_fact_ids").len(){return Err(format!("{id}: duplicate admitted fact IDs"));}
 let unresolved=ids(&decision["unresolved_claim_ids"]);
 let oracle=&v["oracle"];
 if !exact(oracle,&["hidden_from_candidates","correct_option","critical_fact_ids","shareable_critical_fact_ids","restricted_critical_fact_ids"])
   ||oracle["hidden_from_candidates"].as_bool()!=Some(true){return Err(format!("{id}: oracle must be exact and hidden"));}
 let correct=s(oracle,"correct_option").ok_or(format!("{id}: oracle option missing"))?;
 let critical=ids(&oracle["critical_fact_ids"]);
 let shareable_critical=ids(&oracle["shareable_critical_fact_ids"]);
 let restricted_critical=ids(&oracle["restricted_critical_fact_ids"]);
 if !shareable_critical.is_subset(&critical)||!restricted_critical.is_subset(&critical){return Err(format!("{id}: oracle critical subsets inconsistent"));}
 for fid in critical.union(&shareable_critical).chain(restricted_critical.iter()){
  if !fmap.contains_key(fid){return Err(format!("{id}: oracle references unknown fact {fid}"));}
 }
 let computed_shareable:BTreeSet<String>=critical.iter().filter_map(|fid|{
  fmap.get(fid).filter(|f|allowed(s(f,"rights_status").unwrap_or(""))).map(|_|fid.clone())
 }).collect();
 if computed_shareable!=shareable_critical{return Err(format!("{id}: oracle shareable-critical set does not match fact rights"));}
 let computed_restricted:BTreeSet<String>=critical.iter().filter_map(|fid|{
  fmap.get(fid).filter(|f|!allowed(s(f,"rights_status").unwrap_or(""))).map(|_|fid.clone())
 }).collect();
 if computed_restricted!=restricted_critical{return Err(format!("{id}: oracle restricted-critical set does not match fact rights"));}
 for fid in &admitted{
  let f=fmap.get(fid).ok_or(format!("{id}: decision admitted unknown fact {fid}"))?;
  if !allowed(s(f,"rights_status").unwrap_or("")){return Ok("invalid");}
  if s(f,"evidence_status")!=Some("current"){return Ok("fail");}
  if s(f,"visibility")==Some("shared"){
   if s(f,"initial_verification_status")!=Some("verified"){return Ok("fail");}
  }else{
   let ok=receipts_by_fact.get(fid).into_iter().flatten().any(|r|{
    s(r,"verification_status")==Some("verified")&&num(r,"version")==num(f,"version")
   });
   if !ok{return Ok("fail");}
  }
 }
 let unique_origins:BTreeSet<String>=admitted.iter().filter_map(|fid|fmap.get(fid).and_then(|f|s(f,"source_origin_id")).map(str::to_owned)).collect();
 if num(decision,"reported_independent_source_count")!=Some(unique_origins.len() as i64){return Ok("fail");}
 // Any disclosure or receipt outside the declared disclosure scope is an invalid trace.
 for response in arr(v,"responses"){
  if s(response,"status")==Some("disclosed"){
   let f=fmap.get(s(response,"fact_id").unwrap_or("")).ok_or(format!("{id}: disclosure fact missing"))?;
   if !allowed(s(f,"rights_status").unwrap_or("")){return Ok("invalid");}
  }
 }
 // Current, admitted opposing claims form an explicit unresolved conflict.
 let mut polarities:BTreeMap<String,BTreeSet<String>>=BTreeMap::new();
 for fid in &admitted{
  let f=fmap.get(fid).unwrap();
  if s(f,"evidence_status")==Some("current"){
   let p=s(f,"polarity").unwrap_or("");
   if p=="supports_A"||p=="supports_B"{polarities.entry(s(f,"claim_id").unwrap_or("").to_owned()).or_default().insert(p.to_owned());}
  }
 }
 let conflicts:BTreeSet<String>=polarities.iter().filter_map(|(claim,ps)|if ps.len()>1{Some(claim.clone())}else{None}).collect();
 if !conflicts.is_subset(&unresolved){return Ok("fail");}
 let missing: BTreeSet<String>=shareable_critical.difference(&admitted).cloned().collect();
 if completeness=="complete"&&(!missing.is_empty()||!restricted_critical.is_empty()||!conflicts.is_empty()){return Ok("fail");}
 if disposition=="decided"{
  if selected!=Some(correct)||!missing.is_empty()||completeness!="complete"||!unresolved.is_empty(){return Ok("fail");}
  return Ok("pass");
 }
 if missing.is_empty()&&restricted_critical.is_empty()&&conflicts.is_empty(){return Ok("fail");}
 if completeness=="complete"{return Ok("fail");}
 Ok("indeterminate")
}
fn validate_corpus(v:&Value)->Result<BTreeMap<&'static str,usize>,String>{
 if s(v,"schema_version")!=Some("delib-dist-info-fixtures-v0.1")||s(v,"status")!=Some("synthetic_controls_only_agents_not_executed")||s(v,"protocol_id")!=Some(ID){return Err("fixture corpus identity/status mismatch".into());}
 if arr(v,"scenarios").len()!=N||v["fixture_count"].as_u64()!=Some(N as u64)||v["family_count"].as_u64()!=Some(FAMILIES.len() as u64){return Err("fixture/family count mismatch".into());}
 let mut ids_seen=BTreeSet::new();let mut family_seen=BTreeSet::new();let mut counts:BTreeMap<&'static str,usize>=BTreeMap::new();
 for x in arr(v,"scenarios"){
  let id=s(x,"scenario_id").ok_or("scenario ID missing")?;
  if !ids_seen.insert(id.to_owned()){return Err("duplicate scenario ID".into());}
  let family=s(x,"family").ok_or(format!("{id}: family missing"))?;
  if !FAMILIES.contains(&family){return Err(format!("{id}: unknown family"));}
  family_seen.insert(family.to_owned());
  let got=assessment(x)?;
  let expected=s(x,"expected_assessment").ok_or(format!("{id}: expected assessment missing"))?;
  if got!=expected{return Err(format!("{id}: expected assessment {expected}, independently computed {got}"));}
  *counts.entry(got).or_default()+=1;
  println!("{id}: candidate_view_sha256={} fact_registry_sha256={} receipt_set_sha256={}",
   digest(&Value::Array(arr(x,"initial_views").to_vec()))?,digest(&Value::Array(arr(x,"facts").to_vec()))?,digest(&Value::Array(arr(x,"receipts").to_vec()))?);
 }
 if family_seen.len()!=FAMILIES.len(){return Err("not all six fixture families are present".into());}
 Ok(counts)
}
fn schema_header(v:&Value,id:&str,required:&[&str])->Vec<String>{
 let mut e=Vec::new();
 if s(v,"$schema")!=Some("https://json-schema.org/draft/2020-12/schema"){e.push("schema dialect mismatch".to_string());}
 if s(v,"$id")!=Some(id){e.push("schema ID mismatch".into());}
 if s(v,"type")!=Some("object")||v["additionalProperties"]!=false{e.push("schema root must be closed object".into());}
 let got: BTreeSet<String>=ids(&v["required"]);
 for x in required{if !got.contains(*x){e.push(format!("schema root requires {x}"));}}
 e
}
fn run(a:&[String])->Result<(),String>{
 if a.len()!=4{return Err("usage: qualifier PROTOCOL PROTOCOL_SCHEMA FIXTURES FIXTURES_SCHEMA".into());}
 let read=|p:&str|->Result<Value,String>{serde_json::from_str(&fs::read_to_string(p).map_err(|e|e.to_string())?).map_err(|e|e.to_string())};
 let p=read(&a[0])?;let ps=read(&a[1])?;let f=read(&a[2])?;let fschema=read(&a[3])?;
 let mut errors=schema_header(&ps,"https://luminousdynamics.io/schemas/delib-dist-info-protocol-v0.1.schema.json",
  &["protocol_id","version","status","title","research_question","parent","ownership","external_comparators","metric_ids","implemented_metric_ids","deferred_metric_ids","family_ids","terminal_states","claim_ceiling"]);
 errors.extend(schema_header(&fschema,"https://luminousdynamics.io/schemas/delib-dist-info-fixtures-v0.1.schema.json",
  &["schema_version","status","protocol_id","fixture_count","family_count","scenarios"]));
 if !errors.is_empty(){return Err(errors.join("; "));}
 if s(&p,"protocol_id")!=Some(ID)||p["version"]!="0.1.0"||s(&p,"status")!=Some("synthetic_contract_qualifier"){return Err("protocol identity/version/status mismatch".into());}
 let all: BTreeSet<String>=ids(&p["metric_ids"]);
 let implemented:BTreeSet<String>=ids(&p["implemented_metric_ids"]);
 let deferred:BTreeSet<String>=ids(&p["deferred_metric_ids"]);
 if !implemented.is_disjoint(&deferred)||implemented.union(&deferred).cloned().collect::<BTreeSet<_>>()!=all{return Err("implemented/deferred metric partition mismatch".into());}
 let counts=validate_corpus(&f)?;
 println!("QUALIFIED: scenarios={} families={} pass={} fail={} indeterminate={} invalid={} agents_executed=false human_data_imported=false",
  arr(&f,"scenarios").len(),FAMILIES.len(),counts.get("pass").unwrap_or(&0),counts.get("fail").unwrap_or(&0),counts.get("indeterminate").unwrap_or(&0),counts.get("invalid").unwrap_or(&0));
 Ok(())
}
fn main()->ExitCode{match run(&env::args().skip(1).collect::<Vec<_>>()){Ok(())=>ExitCode::SUCCESS,Err(e)=>{eprintln!("NOT_QUALIFIED: {e}");ExitCode::FAILURE}}}

#[cfg(test)]
mod tests{
 use super::*;
 fn doc()->Value{serde_json::from_str(include_str!("../../DIST_INFO_FIXTURES_V0_1.json")).unwrap()}
 fn fx(id:&str)->Value{arr(&doc(),"scenarios").iter().find(|x|s(x,"scenario_id")==Some(id)).unwrap().clone()}
 #[test]fn all_18_frozen_synthetic_traces_qualify(){assert!(validate_corpus(&doc()).is_ok());}
 #[test]fn complementary_information_can_be_pooled(){assert_eq!(assessment(&fx("COMP-01-complete-pooling")).unwrap(),"pass");}
 #[test]fn missing_critical_fact_cannot_be_marked_complete(){assert_eq!(assessment(&fx("COMP-03-false-completeness-claim")).unwrap(),"fail");}
 #[test]fn private_fact_cannot_leak_to_another_initial_view(){let mut x=fx("COMP-01-complete-pooling");x["initial_views"][0]["visible_fact_ids"].as_array_mut().unwrap().push(Value::String("F-COMP-B".into()));assert!(assessment(&x).is_err());}
 #[test]fn request_must_target_the_assigned_holder(){let mut x=fx("COMP-01-complete-pooling");x["requests"][0]["target_agent_id"]=Value::String("AGENT-C".into());assert!(assessment(&x).is_err());}
 #[test]fn response_cannot_exist_without_delivery(){let mut x=fx("COMP-01-complete-pooling");x["requests"][0]["delivery_status"]=Value::String("not_delivered".into());assert!(assessment(&x).is_err());}
 #[test]fn receipt_must_follow_a_disclosed_response(){let mut x=fx("COMP-01-complete-pooling");x["responses"][0]["status"]=Value::String("no_response".into());assert!(assessment(&x).is_err());}
 #[test]fn minority_decisive_fact_changes_the_answer_only_when_elicited(){assert_eq!(assessment(&fx("MIN-01-minority-fact-elicited")).unwrap(),"pass");assert_eq!(assessment(&fx("MIN-02-minority-fact-ignored")).unwrap(),"fail");}
 #[test]fn repeated_attribution_does_not_create_independent_sources(){let x=fx("MIN-03-repetition-masquerades-as-consensus");assert_eq!(assessment(&x).unwrap(),"fail");}
 #[test]fn source_origin_is_deduplicated(){assert_eq!(assessment(&fx("RED-01-deduplicate-then-integrate")).unwrap(),"pass");}
 #[test]fn source_count_inflation_fails(){assert_eq!(assessment(&fx("RED-02-source-count-inflation")).unwrap(),"fail");}
 #[test]fn silent_holder_requires_targeted_inquiry(){assert_eq!(assessment(&fx("SIL-01-targeted-request-reaches-silent-holder")).unwrap(),"pass");assert_eq!(assessment(&fx("SIL-02-silent-holder-never-solicited")).unwrap(),"fail");}
 #[test]fn undelivered_request_remains_indeterminate(){assert_eq!(assessment(&fx("SIL-03-request-created-but-not-delivered")).unwrap(),"indeterminate");}
 #[test]fn superseded_evidence_must_not_enter_decision(){assert_eq!(assessment(&fx("TIME-01-stale-version-rejected-current-version-used")).unwrap(),"pass");assert_eq!(assessment(&fx("TIME-03-superseded-evidence-promoted")).unwrap(),"fail");}
 #[test]fn current_conflict_is_preserved_and_stops_decision(){assert_eq!(assessment(&fx("TIME-02-current-conflict-preserved-and-deferred")).unwrap(),"indeterminate");}
 #[test]fn rights_refusal_is_not_a_reasoning_failure_or_disclosure_permission(){assert_eq!(assessment(&fx("RIGHTS-01-refusal-retained-no-disclosure")).unwrap(),"indeterminate");}
 #[test]fn unknown_rights_cannot_be_promoted_to_disclosable(){assert_eq!(assessment(&fx("RIGHTS-02-unknown-rights-no-payload")).unwrap(),"indeterminate");}
 #[test]fn restricted_disclosure_is_invalid(){assert_eq!(assessment(&fx("RIGHTS-03-restricted-evidence-illegally-disclosed")).unwrap(),"invalid");}
 #[test]fn duplicate_origin_metadata_must_be_consistent(){let mut x=fx("RED-01-deduplicate-then-integrate");x["facts"][1]["source_origin_id"]=Value::String("OTHER".into());assert!(assessment(&x).is_err());}
 #[test]fn version_and_rights_in_receipt_are_bound(){let mut x=fx("COMP-01-complete-pooling");x["receipts"][0]["version"]=Value::from(99);assert!(assessment(&x).is_err());}
 #[test]fn unknown_hidden_holder_is_not_silently_accepted(){let mut x=fx("COMP-01-complete-pooling");x["facts"][0]["holder_agent_id"]=Value::String("GHOST".into());assert!(assessment(&x).is_err());}
 #[test]fn oracle_must_remain_hidden(){let mut x=fx("COMP-01-complete-pooling");x["oracle"]["hidden_from_candidates"]=Value::Bool(false);assert!(assessment(&x).is_err());}
}
