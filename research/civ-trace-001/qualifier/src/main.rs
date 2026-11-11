use serde_json::Value;
use std::collections::{BTreeMap, BTreeSet};
use std::env;
use std::fs;
use std::process::ExitCode;

const BENCHMARK_ID: &str = "CIV-TRACE-001";
const SCHEMA_VERSION: &str = "civ-trace-fixtures-v0.1";
const SCHEMA_ID: &str = "https://github.com/Luminous-Dynamics/mycelix/blob/main/research/civ-trace-001/CIV_TRACE_FIXTURES_V0_1.schema.json";
const FIXTURE_COUNT: usize = 36;
const FAMILIES: [&str; 6] = [
    "hindsight_isolation",
    "authority_at_event_time",
    "correction_supersession",
    "outcome_observation",
    "contestability",
    "privacy_projection",
];

fn text<'a>(v: &'a Value, key: &str) -> Option<&'a str> {
    v.get(key)?.as_str().filter(|s| !s.trim().is_empty())
}
fn number(v: &Value, key: &str) -> Option<u64> { v.get(key)?.as_u64() }
fn sha256(s: &str) -> bool {
    s.len() == 64 && s.bytes().all(|b| b.is_ascii_hexdigit() && !b.is_ascii_uppercase())
}
fn strings(v: &Value) -> BTreeSet<String> {
    v.as_array().into_iter().flatten().filter_map(Value::as_str).map(ToOwned::to_owned).collect()
}

fn validate_event(event: &Value) -> Result<String, String> {
    if event.as_object().map(serde_json::Map::len) != Some(16) {
        return Err("event must contain exactly sixteen declared properties".into());
    }
    let id = text(event, "event_id").ok_or_else(|| "event_id missing".to_string())?.to_string();
    if !["Evidence","Decision","AuthorityGrant","AuthorityRevoke","Attempt","EffectObservation","Challenge","Correction"].contains(&text(event,"kind").unwrap_or("")) {
        return Err(format!("{id}: unknown event kind"));
    }
    if number(event,"transaction_time").is_none_or(|n| n == 0) || number(event,"valid_from").is_none_or(|n| n == 0) {
        return Err(format!("{id}: event times must be positive integers"));
    }
    if !event["valid_until"].is_null() && event["valid_until"].as_u64().is_none_or(|n| n == 0) {
        return Err(format!("{id}: valid_until must be positive or null"));
    }
    for key in ["subject_ref","principal_ref","target_ref","status","supersedes_ref","issuer_ref","observer_ref"] {
        if text(event,key).is_none() { return Err(format!("{id}: {key} missing")); }
    }
    if let Some(until) = event["valid_until"].as_u64() {
        if until <= number(event,"valid_from").unwrap_or(0) {
            return Err(format!("{id}: validity interval is empty or inverted"));
        }
    }
    let kind = text(event,"kind").unwrap_or("");
    let status = text(event,"status").unwrap_or("");
    let valid_status = match kind {
        "Evidence" => ["Verified","Untrusted"].contains(&status),
        "Decision" => status == "Adopted",
        "AuthorityGrant" => status == "Granted",
        "AuthorityRevoke" => status == "Revoked",
        "Attempt" => status == "Succeeded",
        "EffectObservation" => ["Confirmed","Partial","Contradicted"].contains(&status),
        "Challenge" => ["Open","Resolved"].contains(&status),
        "Correction" => status == "Corrected",
        _ => false,
    };
    if !valid_status {
        return Err(format!("{id}: status {status} is invalid for event kind {kind}"));
    }
    if !["Public","Protected"].contains(&text(event,"visibility").unwrap_or("")) {
        return Err(format!("{id}: visibility invalid"));
    }
    if !text(event,"payload_sha256").is_some_and(sha256) {
        return Err(format!("{id}: payload digest malformed"));
    }
    if text(event,"kind")==Some("EffectObservation") && text(event,"observer_ref")==Some("NONE") {
        return Err(format!("{id}: EffectObservation requires an observer identity"));
    }
    if event["projection_includes_payload"].as_bool().is_none() {
        return Err(format!("{id}: projection_includes_payload must be boolean"));
    }
    if event["parent_refs"].as_array().is_none() {
        return Err(format!("{id}: parent_refs must be an array"));
    }
    let parents = event["parent_refs"].as_array().unwrap();
    if parents.iter().any(|x| x.as_str().is_none_or(|s| s.trim().is_empty())) || strings(&event["parent_refs"]).len() != parents.len() {
        return Err(format!("{id}: parent_refs must be unique non-empty strings"));
    }
    Ok(id)
}

fn authority(fixture: &Value, include_late: bool) -> &'static str {
    let events = fixture["events"].as_array().unwrap();
    let action_time = number(fixture,"action_time_unix").unwrap_or(0);
    let decision = text(fixture,"decision_ref").unwrap_or("");
    let principal = text(fixture,"principal_ref").unwrap_or("");
    let target = text(fixture,"target_ref").unwrap_or("");
    let grants = events.iter().filter(|e|
        text(e,"kind")==Some("AuthorityGrant")
        && text(e,"status")==Some("Granted")
        && text(e,"subject_ref")==Some(decision)
        && text(e,"principal_ref")==Some(principal)
        && text(e,"target_ref")==Some(target)
        && number(e,"valid_from").is_some_and(|t| t<=action_time)
        && e["valid_until"].as_u64().is_none_or(|t| action_time<t)
        && (include_late || number(e,"transaction_time").is_some_and(|t| t<=action_time))
    ).collect::<Vec<_>>();
    if grants.is_empty() { return "UNAUTHORIZED"; }
    if grants.len()!=1 { return "INDETERMINATE"; }
    let grant_id=text(grants[0],"event_id").unwrap_or("");
    let revoked=events.iter().any(|e|
        text(e,"kind")==Some("AuthorityRevoke")
        && text(e,"status")==Some("Revoked")
        && strings(&e["parent_refs"]).contains(grant_id)
        && number(e,"valid_from").is_some_and(|t|t<=action_time)
        && (include_late || number(e,"transaction_time").is_some_and(|t|t<=action_time))
    );
    if revoked {"UNAUTHORIZED"} else {"AUTHORIZED"}
}

fn outcome(fixture: &Value) -> &'static str {
    let attempt=text(fixture,"attempt_ref").unwrap_or("");
    let events=fixture["events"].as_array().unwrap();
    let attempt_event=events.iter().find(|e|
        text(e,"kind")==Some("Attempt") && text(e,"event_id")==Some(attempt)
    );
    let Some(attempt_event)=attempt_event else { return "UNOBSERVED"; };
    let executor=text(attempt_event,"issuer_ref").unwrap_or("");
    let observations=events.iter().filter(|e|
        text(e,"kind")==Some("EffectObservation") && strings(&e["parent_refs"]).contains(attempt)
    ).collect::<Vec<_>>();
    if observations.is_empty(){return "UNOBSERVED";}
    if observations.iter().any(|e|
        text(e,"observer_ref").is_none_or(|observer| observer=="NONE" || observer==executor)
    ){return "INDEPENDENCE_UNESTABLISHED";}
    let states=observations.iter().filter_map(|e|text(e,"status")).collect::<BTreeSet<_>>();
    if states.len()>1{return "CONTESTED";}
    match *states.iter().next().unwrap() {
        "Confirmed"=>"CONFIRMED",
        "Partial"=>"PARTIAL",
        "Contradicted"=>"CONTRADICTED",
        _=>"CONTESTED",
    }
}

fn challenge(fixture: &Value) -> &'static str {
    let decision=text(fixture,"decision_ref").unwrap_or("");
    let events=fixture["events"].as_array().unwrap();
    let challenges=events.iter().filter(|e|
        text(e,"kind")==Some("Challenge")
        && (text(e,"subject_ref")==Some(decision)||strings(&e["parent_refs"]).contains(decision))
    ).collect::<Vec<_>>();
    if challenges.iter().any(|e|text(e,"status")==Some("Open")) {"OPEN_UNRESOLVED"}
    else if challenges.iter().any(|e|text(e,"status")==Some("Resolved")) {"RESOLVED_PRESERVED"}
    else {"NONE"}
}

fn current_decision(fixture: &Value) -> String {
    let events=fixture["events"].as_array().unwrap();
    let mut current=text(fixture,"decision_ref").unwrap_or("").to_string();
    let mut seen=BTreeSet::from([current.clone()]);
    loop {
        let successors=events.iter().filter(|e|
            text(e,"kind")==Some("Decision") && text(e,"supersedes_ref")==Some(current.as_str())
        ).collect::<Vec<_>>();
        if successors.len()>1{return "AMBIGUOUS_SUCCESSORS".into();}
        if successors.is_empty(){return current;}
        current=text(successors[0],"event_id").unwrap_or("").to_string();
        if !seen.insert(current.clone()){return "CYCLE".into();}
    }
}

fn derive(fixture:&Value)->Result<Value,String>{
    let cutoff=number(fixture,"cutoff_unix").ok_or_else(||"cutoff_unix missing".to_string())?;
    let events=fixture["events"].as_array().ok_or_else(||"events must be an array".to_string())?;
    let known=events.iter().filter(|e|number(e,"transaction_time").is_some_and(|t|t<=cutoff))
        .filter_map(|e|text(e,"event_id").map(ToOwned::to_owned)).collect::<BTreeSet<_>>();
    let privacy=events.iter().any(|e|text(e,"visibility")==Some("Protected")&&e["projection_includes_payload"]==true);
    Ok(serde_json::json!({
        "known_event_ids":known.into_iter().collect::<Vec<_>>(),
        "authority_as_known_at_action":authority(fixture,false),
        "authority_current_assessment":authority(fixture,true),
        "outcome_state":outcome(fixture),
        "challenge_state":challenge(fixture),
        "current_decision_ref":current_decision(fixture),
        "privacy_violation":privacy
    }))
}

fn validate_fixture(fixture:&Value)->Result<(String,String),String>{
    if fixture.as_object().map(serde_json::Map::len)!=Some(10){return Err("fixture must have exactly ten declared properties".into());}
    let id=text(fixture,"fixture_id").ok_or_else(||"fixture_id missing".to_string())?.to_string();
    let family=text(fixture,"family").ok_or_else(||"family missing".to_string())?.to_string();
    if !FAMILIES.contains(&family.as_str()){return Err(format!("{id}: unknown family {family}"));}
    if number(fixture,"cutoff_unix").is_none_or(|x|x==0)||number(fixture,"action_time_unix").is_none_or(|x|x==0){return Err(format!("{id}: cutoff/action time invalid"));}
    if number(fixture,"action_time_unix").unwrap() < number(fixture,"cutoff_unix").unwrap() {
        return Err(format!("{id}: action_time_unix cannot precede the decision cutoff"));
    }
    for key in ["target_ref","principal_ref","decision_ref","attempt_ref"]{if text(fixture,key).is_none(){return Err(format!("{id}: {key} missing"));}}
    let events=fixture["events"].as_array().ok_or_else(||format!("{id}: events must be array"))?;
    if events.is_empty(){return Err(format!("{id}: events cannot be empty"));}
    let decision_ref=text(fixture,"decision_ref").unwrap();
    let attempt_ref=text(fixture,"attempt_ref").unwrap();
    if events.iter().filter(|e|text(e,"kind")==Some("Decision")&&text(e,"event_id")==Some(decision_ref)).count()!=1 {
        return Err(format!("{id}: decision_ref must resolve to exactly one Decision event"));
    }
    if events.iter().filter(|e|text(e,"kind")==Some("Attempt")&&text(e,"event_id")==Some(attempt_ref)).count()!=1 {
        return Err(format!("{id}: attempt_ref must resolve to exactly one Attempt event"));
    }
    let mut ids=BTreeSet::new();
    for event in events{
        let event_id=validate_event(event)?;
        if !ids.insert(event_id.clone()){return Err(format!("{id}: duplicate event ID {event_id}"));}
    }
    let derived=derive(fixture)?;
    if derived!=fixture["expected"]{return Err(format!("{id}: expected oracle mismatch; derived {derived}"));}
    Ok((id,family))
}

fn validate_schema(schema:&Value)->Result<(),String>{
    let items=&schema["properties"]["fixtures"]["items"];
    if text(schema,"$schema")!=Some("https://json-schema.org/draft/2020-12/schema")
        ||text(schema,"$id")!=Some(SCHEMA_ID)
        ||text(schema,"type")!=Some("object")
        ||schema["additionalProperties"]!=false
        ||schema["properties"]["fixtures"]["type"]!="array"
        ||items["additionalProperties"]!=false
        ||items["properties"]["events"]["items"]["additionalProperties"]!=false
        ||items["properties"]["expected"]["additionalProperties"]!=false{
        return Err("schema identity or closed nested-object contract mismatch".into());
    }
    Ok(())
}

fn validate_manifest(manifest:&Value)->Result<BTreeMap<String,usize>,Vec<String>>{
    let mut errors=Vec::new();
    if text(manifest,"schema_version")!=Some(SCHEMA_VERSION)
        ||text(manifest,"benchmark_id")!=Some(BENCHMARK_ID)
        ||text(manifest,"status")!=Some("synthetic_known_answer_design_unexecuted")
        ||manifest["fixture_count"].as_u64()!=Some(FIXTURE_COUNT as u64)
        ||manifest["family_count"].as_u64()!=Some(FAMILIES.len() as u64){
        errors.push("manifest identity/status/count mismatch".into());
    }
    let Some(fixtures)=manifest["fixtures"].as_array() else{return Err(vec!["fixtures must be array".into()]);};
    if fixtures.len()!=FIXTURE_COUNT{errors.push(format!("expected {FIXTURE_COUNT} fixtures, got {}",fixtures.len()));}
    let mut ids=BTreeSet::new();let mut counts=BTreeMap::<String,usize>::new();
    for (i,f) in fixtures.iter().enumerate(){
        match validate_fixture(f){
            Ok((id,family))=>{
                if !ids.insert(id.clone()){errors.push(format!("duplicate fixture ID {id}"));}
                *counts.entry(family).or_default()+=1;
            },
            Err(e)=>errors.push(format!("fixtures[{i}]: {e}"))
        }
    }
    for family in FAMILIES{
        if counts.get(family).copied().unwrap_or(0)!=6{errors.push(format!("family {family} must contain exactly six fixtures"));}
    }
    if errors.is_empty(){Ok(counts)}else{errors.sort();errors.dedup();Err(errors)}
}

fn read_json(path:&str)->Result<Value,String>{
    let source=fs::read_to_string(path).map_err(|e|format!("{path}: {e}"))?;
    serde_json::from_str(&source).map_err(|e|format!("{path}: invalid JSON: {e}"))
}
fn main()->ExitCode{
    let args=env::args().collect::<Vec<_>>();
    if args.len()!=3{eprintln!("usage: civ-trace-qualifier <fixtures.json> <schema.json>");return ExitCode::from(2);}
    let manifest=match read_json(&args[1]){Ok(v)=>v,Err(e)=>{eprintln!("{e}");return ExitCode::FAILURE;}};
    let schema=match read_json(&args[2]){Ok(v)=>v,Err(e)=>{eprintln!("{e}");return ExitCode::FAILURE;}};
    if let Err(e)=validate_schema(&schema){eprintln!("SCHEMA QUALIFICATION FAILED: {e}");return ExitCode::FAILURE;}
    match validate_manifest(&manifest){
        Ok(counts)=>{
            println!("QUALIFIED SYNTHETIC INSTITUTIONAL RECONSTRUCTION: fixtures={FIXTURE_COUNT} families={}",counts.len());
            for family in FAMILIES{println!("  {family}: {}",counts.get(family).copied().unwrap_or(0));}
            println!("NOT EXECUTED: no model trial, live governance event, legal review, authority or real-world outcome is implied.");
            ExitCode::SUCCESS
        },
        Err(errors)=>{
            eprintln!("CIV-TRACE QUALIFICATION FAILED ({} findings):",errors.len());
            for e in errors{eprintln!(" - {e}");}
            ExitCode::FAILURE
        }
    }
}

#[cfg(test)]
mod tests{
    use super::*;
    const MANIFEST:&str=include_str!("../../CIV_TRACE_FIXTURES_V0_1.json");
    fn parsed()->Value{serde_json::from_str(MANIFEST).expect("fixture JSON parses")}
    fn fixture(id:&str)->Value{parsed()["fixtures"].as_array().unwrap().iter().find(|f|text(f,"fixture_id")==Some(id)).unwrap().clone()}

    #[test]fn all_36_known_answer_fixtures_validate(){let c=validate_manifest(&parsed()).expect("manifest qualifies");assert_eq!(c.len(),6);assert!(c.values().all(|n|*n==6));}
    #[test]fn late_backdated_evidence_is_not_in_historical_knowledge(){let f=fixture("CIVT-HINDSIGHT-02");assert!(!strings(&f["expected"]["known_event_ids"]).contains("H-LATE-BACKDATED-2"));}
    #[test]fn late_revocation_changes_current_assessment_not_as_known_view(){let f=fixture("CIVT-AUTHORITY-04");assert_eq!(f["expected"]["authority_as_known_at_action"],"AUTHORIZED");assert_eq!(f["expected"]["authority_current_assessment"],"UNAUTHORIZED");}
    #[test]fn late_backdated_grant_does_not_change_what_was_known(){let f=fixture("CIVT-AUTHORITY-05");assert_eq!(f["expected"]["authority_as_known_at_action"],"UNAUTHORIZED");assert_eq!(f["expected"]["authority_current_assessment"],"AUTHORIZED");}
    #[test]fn executor_success_without_observation_is_unobserved(){let f=fixture("CIVT-OUTCOME-01");assert_eq!(f["expected"]["outcome_state"],"UNOBSERVED");}
    #[test]fn conflicting_outcome_observations_remain_contested(){let f=fixture("CIVT-OUTCOME-04");assert_eq!(f["expected"]["outcome_state"],"CONTESTED");}
    #[test]fn executor_identity_cannot_self_observe_an_effect(){let f=fixture("CIVT-OUTCOME-05");assert_eq!(f["expected"]["outcome_state"],"INDEPENDENCE_UNESTABLISHED");}
    #[test]fn observation_for_wrong_attempt_does_not_prove_outcome(){let f=fixture("CIVT-HINDSIGHT-06");assert_eq!(f["expected"]["outcome_state"],"UNOBSERVED");}
    #[test]fn unresolved_challenge_remains_visible(){let f=fixture("CIVT-CONTEST-04");assert_eq!(f["expected"]["challenge_state"],"OPEN_UNRESOLVED");}
    #[test]fn resolved_challenge_remains_in_history(){let f=fixture("CIVT-CONTEST-03");assert_eq!(f["expected"]["challenge_state"],"RESOLVED_PRESERVED");}
    #[test]fn supersession_selects_successor_without_deleting_predecessor(){let f=fixture("CIVT-SUPERSESSION-03");assert_eq!(f["expected"]["current_decision_ref"],"DEC3-3");assert!(f["events"].as_array().unwrap().iter().any(|e|text(e,"event_id")==Some("DEC-023")));}
    #[test]fn protected_payload_leak_is_explicit(){let f=fixture("CIVT-PRIVACY-02");assert_eq!(f["expected"]["privacy_violation"],true);}
    #[test]fn duplicate_event_id_is_rejected(){let mut f=fixture("CIVT-HINDSIGHT-01");let e=f["events"][0].clone();f["events"].as_array_mut().unwrap().push(e);assert!(validate_fixture(&f).is_err());}
    #[test]fn expected_knowledge_set_tampering_is_rejected(){let mut f=fixture("CIVT-HINDSIGHT-01");f["expected"]["known_event_ids"]=serde_json::json!([]);assert!(validate_fixture(&f).is_err());}
    #[test]fn missing_attempt_reference_is_invalid_trace(){let mut f=fixture("CIVT-OUTCOME-01");let attempt_ref=text(&f,"attempt_ref").unwrap().to_string();f["events"].as_array_mut().unwrap().retain(|e|text(e,"event_id")!=Some(attempt_ref.as_str()));assert!(validate_fixture(&f).is_err());}
    #[test]fn event_status_must_match_its_kind(){let mut f=fixture("CIVT-OUTCOME-01");let idx=f["events"].as_array().unwrap().iter().position(|e|text(e,"kind")==Some("Attempt")).unwrap();f["events"][idx]["status"]=Value::String("Confirmed".into());assert!(validate_fixture(&f).is_err());}
    #[test]fn event_digest_must_be_lowercase_sha256(){let mut f=fixture("CIVT-HINDSIGHT-01");f["events"][0]["payload_sha256"]=Value::String("A".repeat(64));assert!(validate_fixture(&f).is_err());}
}
