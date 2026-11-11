//! Synthetic-only observation stream identity and clock/sequence replay oracle.
#![forbid(unsafe_code)]

use serde::de::{MapAccess, SeqAccess, Visitor};
use serde::{Deserialize, Deserializer};
use serde_json::{Map, Value};
use std::collections::BTreeSet;
use std::fmt;

pub const PROFILE_JSON: &str = include_str!("../../../docs/metrology/BUILDING_MEASUREMENT_STREAM_PROFILE_V0_1.json");
pub const CORPUS_JSON: &str = include_str!("../../../docs/metrology/BUILDING_MEASUREMENT_STREAM_CORPUS_V0_1.json");
pub const SCHEMA_JSON: &str = include_str!("../../../docs/metrology/BUILDING_MEASUREMENT_STREAM_CORPUS_V0_1.schema.json");

const PROFILE_ID: &str = "MYC-BUILD-STREAM-TIMEBASE-PROFILE-V0.1";
const CORPUS_ID: &str = "BUILDING-MEASUREMENT-STREAM-CORPUS-V0.1";
const CASES: &[(&str,&str)] = &[
("MST-001","REVIEW_ELIGIBLE_ONLY"),("MST-002","REVIEW_ELIGIBLE_ONLY"),
("MST-003","CONFLICT_REJECTED"),("MST-004","GAP_INDETERMINATE"),
("MST-005","REVIEW_ELIGIBLE_ONLY"),("MST-006","NOT_COMPARABLE"),
("MST-007","INVALIDATED"),("MST-008","REVIEW_ELIGIBLE_ONLY"),
("MST-009","REVIEW_ELIGIBLE_ONLY"),("MST-010","MORE_EVIDENCE_REQUIRED"),
("MST-011","INVALIDATED"),("MST-012","INVALIDATED"),
("MST-013","INVALIDATED"),("MST-014","INVALIDATED"),
("MST-015","REJECTED_FOR_PURPOSE"),("MST-016","NOT_COMPARABLE"),
("MST-017","NOT_COMPARABLE"),("MST-018","NOT_COMPARABLE"),
("MST-019","MORE_EVIDENCE_REQUIRED"),("MST-020","MORE_EVIDENCE_REQUIRED"),
("MST-021","NOT_COMPARABLE"),("MST-022","NOT_COMPARABLE"),
("MST-023","MORE_EVIDENCE_REQUIRED"),("MST-024","NOT_COMPARABLE"),
("MST-025","REJECTED_TIME_BASIS_PROMOTION"),("MST-026","NOT_COMPARABLE"),
("MST-027","CONFLICT_REJECTED"),("MST-028","INVALIDATED"),
("MST-029","INVALIDATED"),("MST-030","INVALIDATED"),
("MST-031","NOT_COMPARABLE"),("MST-032","REJECTED_FOR_PURPOSE"),
("MST-033","INVALIDATED"),("MST-034","NOT_COMPARABLE"),
("MST-035","GAP_INDETERMINATE"),("MST-036","REJECTED_FOR_PURPOSE"),
("MST-037","INVALIDATED"),("MST-038","REJECTED_TIME_BASIS_PROMOTION"),
("MST-039","UNRELATED_STREAM_UNCHANGED"),("MST-040","TARGETED_INVALIDATION_ONLY")
];
const ROOT_FIELDS:&[&str]=&["schema_version","profile_id","stream","samples","epoch_transitions","clock_mapping","aggregation","invalidation"];
const STREAM_FIELDS:&[&str]=&["stream_id","instrument_id","channel_id","channel_profile_id","firmware_digest","protocol_digest","clock_domain_id","boot_epoch_id","sequence_scope","sequence_width_bits","wrap_policy","timestamp_basis","time_unit","epoch_origin","clock_source","sync_source","maximum_sync_uncertainty_ns","maximum_reorder_window_ns","maximum_precision_loss_ns","expected_sample_count","expected_sequence_start","expected_sequence_end","clock_step_state","instrument_identity_reused"];
const SAMPLE_FIELDS:&[&str]=&["sample_id","sequence","boot_epoch_id","clock_domain_id","instrument_id","channel_id","firmware_digest","protocol_digest","timestamp_basis","time_unit","epoch_origin","event_tick","acquisition_tick","collector_receipt_tick","clock_source","synchronization_uncertainty_ns","payload_digest","local_time","utc_offset_minutes","dst_fold","calendar_valid","leap_second_token","precision_loss_ns","latency_explanation_id"];
const TRANSITION_FIELDS:&[&str]=&["from_boot_epoch_id","to_boot_epoch_id","transition_kind","receipt_id"];
const MAPPING_FIELDS:&[&str]=&["state","mapping_id","revision","digest","source_clock_domain_id","target_clock_domain_id","valid_from_event_tick","valid_to_event_tick","offset_ns","drift_ppb","uncertainty_ns","residual_ns","source_receipt_id","retroactive_change"];
const AGGREGATION_FIELDS:&[&str]=&["requested","basis","integrates","rate_basis","imputation_policy","crosses_unresolved_clock_step","input_digest"];
const INVALIDATION_FIELDS:&[&str]=&["change_scope","change_target_stream_id","dependent_claim_stale","raw_samples_preserved","unrelated_streams_current"];

#[derive(Debug)]
struct UniqueJson(Value);
struct UniqueVisitor;
impl<'de> Deserialize<'de> for UniqueJson {
    fn deserialize<D:Deserializer<'de>>(d:D)->Result<Self,D::Error>{d.deserialize_any(UniqueVisitor)}
}
impl<'de> Visitor<'de> for UniqueVisitor {
    type Value=UniqueJson;
    fn expecting(&self,f:&mut fmt::Formatter<'_>)->fmt::Result{f.write_str("a JSON value without duplicate member names")}
    fn visit_bool<E:serde::de::Error>(self,v:bool)->Result<Self::Value,E>{Ok(UniqueJson(Value::Bool(v)))}
    fn visit_i64<E:serde::de::Error>(self,v:i64)->Result<Self::Value,E>{Ok(UniqueJson(Value::Number(v.into())))}
    fn visit_u64<E:serde::de::Error>(self,v:u64)->Result<Self::Value,E>{Ok(UniqueJson(Value::Number(v.into())))}
    fn visit_f64<E:serde::de::Error>(self,v:f64)->Result<Self::Value,E>{let n=serde_json::Number::from_f64(v).ok_or_else(||E::custom("non-finite number"))?;Ok(UniqueJson(Value::Number(n)))}
    fn visit_str<E:serde::de::Error>(self,v:&str)->Result<Self::Value,E>{Ok(UniqueJson(Value::String(v.to_owned())))}
    fn visit_string<E:serde::de::Error>(self,v:String)->Result<Self::Value,E>{Ok(UniqueJson(Value::String(v)))}
    fn visit_unit<E:serde::de::Error>(self)->Result<Self::Value,E>{Ok(UniqueJson(Value::Null))}
    fn visit_none<E:serde::de::Error>(self)->Result<Self::Value,E>{Ok(UniqueJson(Value::Null))}
    fn visit_seq<A:SeqAccess<'de>>(self,mut a:A)->Result<Self::Value,A::Error>{let mut v=Vec::new();while let Some(x)=a.next_element::<UniqueJson>()?{v.push(x.0);}Ok(UniqueJson(Value::Array(v)))}
    fn visit_map<A:MapAccess<'de>>(self,mut a:A)->Result<Self::Value,A::Error>{let mut v=Map::new();let mut seen=BTreeSet::new();while let Some(k)=a.next_key::<String>()?{if !seen.insert(k.clone()){return Err(serde::de::Error::custom(format!("duplicate member {k:?}")));}let x=a.next_value::<UniqueJson>()?;v.insert(k,x.0);}Ok(UniqueJson(Value::Object(v)))}
}
fn parse_unique(s:&str)->Result<Value,String>{let mut d=serde_json::Deserializer::from_str(s);let v=UniqueJson::deserialize(&mut d).map_err(|e|format!("STRICT_JSON_REJECTED:{e}"))?;d.end().map_err(|e|format!("JSON_TRAILING_DATA:{e}"))?;Ok(v.0)}
fn text<'a>(v:&'a Value,k:&str)->&'a str{v.get(k).and_then(Value::as_str).unwrap_or("")}
fn num(v:&Value,k:&str)->Option<i64>{v.get(k).and_then(Value::as_i64)}
fn flag(v:&Value,k:&str)->bool{v.get(k).and_then(Value::as_bool)==Some(true)}
fn exact(v:&Value,fields:&[&str])->bool{let Some(o)=v.as_object()else{return false;};o.len()==fields.len()&&fields.iter().all(|k|o.contains_key(*k))}
fn samples(v:&Value)->Vec<&Value>{v["samples"].as_array().map(|a|a.iter().collect()).unwrap_or_default()}

pub fn classify_record(r:&Value)->&'static str{
    if text(r,"schema_version")!="mycelix.building-measurement-stream/0.1"||text(r,"profile_id")!=PROFILE_ID{return "NOT_COMPARABLE";}
    let s=&r["stream"];let agg=&r["aggregation"];let map=&r["clock_mapping"];let inv=&r["invalidation"];
    let all=samples(r);
    if flag(s,"instrument_identity_reused"){return "INVALIDATED";}
    if s["sequence_width_bits"].is_null(){return "MORE_EVIDENCE_REQUIRED";}
    let Some(width)=num(s,"sequence_width_bits")else{return "MORE_EVIDENCE_REQUIRED";};
    if width<1||width>64{return "NOT_COMPARABLE";}
    if text(s,"clock_step_state")!="none"{return "INVALIDATED";}
    if flag(agg,"requested")&&(text(agg,"basis")!="event_time"||text(agg,"rate_basis")!="event_time"){return "REJECTED_TIME_BASIS_PROMOTION";}
    if text(agg,"imputation_policy")!="none"{return "REJECTED_FOR_PURPOSE";}
    if flag(agg,"crosses_unresolved_clock_step"){return "INVALIDATED";}
    if flag(agg,"requested")&&text(agg,"input_digest").is_empty(){return "MORE_EVIDENCE_REQUIRED";}
    if text(map,"state")=="unknown"{return "MORE_EVIDENCE_REQUIRED";}
    if flag(map,"retroactive_change"){return "INVALIDATED";}
    if text(map,"state")=="verified"{
        if num(map,"residual_ns").zip(num(map,"uncertainty_ns")).is_some_and(|(res,u)|res>u){return "REJECTED_FOR_PURPOSE";}
    }
    if text(s,"sequence_scope")!="stream_boot_epoch"||text(s,"clock_source").is_empty()||text(s,"sync_source").is_empty(){return "MORE_EVIDENCE_REQUIRED";}
    let mut unique:Vec<&Value>=Vec::new();
    for x in &all{
        if text(x,"instrument_id")!=text(s,"instrument_id")||text(x,"channel_id")!=text(s,"channel_id")
            ||text(x,"firmware_digest")!=text(s,"firmware_digest")||text(x,"protocol_digest")!=text(s,"protocol_digest"){return "INVALIDATED";}
        if text(x,"clock_source")!=text(s,"clock_source"){return "INVALIDATED";}
        if text(x,"timestamp_basis")!=text(s,"timestamp_basis")||text(x,"time_unit")!=text(s,"time_unit")||text(x,"epoch_origin")!=text(s,"epoch_origin"){return "NOT_COMPARABLE";}
        if text(x,"clock_domain_id")!=text(s,"clock_domain_id"){return "NOT_COMPARABLE";}
        if num(x,"synchronization_uncertainty_ns").zip(num(s,"maximum_sync_uncertainty_ns")).is_some_and(|(u,max)|u>max){return "REJECTED_FOR_PURPOSE";}
        if !matches!(text(x,"time_unit"),"ns"|"us"|"ms"|"s"){return "NOT_COMPARABLE";}
        let (Some(seq),Some(event),Some(acq),Some(receipt))=(num(x,"sequence"),num(x,"event_tick"),num(x,"acquisition_tick"),num(x,"collector_receipt_tick"))else{return "NOT_COMPARABLE";};
        if seq<0||event<0||acq<0||receipt<0{return "NOT_COMPARABLE";}
        if acq<event&&text(x,"latency_explanation_id").is_empty(){return "NOT_COMPARABLE";}
        if receipt<acq{return "NOT_COMPARABLE";}
        if num(x,"precision_loss_ns").zip(num(s,"maximum_precision_loss_ns")).is_some_and(|(loss,max)|loss>max){return "NOT_COMPARABLE";}
        if text(x,"timestamp_basis")=="local_wall_time"{
            if !flag(x,"calendar_valid"){return "NOT_COMPARABLE";}
            if text(x,"leap_second_token")=="present_unhandled"{return "MORE_EVIDENCE_REQUIRED";}
            if x["utc_offset_minutes"].is_null(){return "MORE_EVIDENCE_REQUIRED";}
        }
        if let Some(previous)=unique.iter().find(|p|text(p,"sample_id")==text(x,"sample_id")){
            if text(previous,"payload_digest")!=text(x,"payload_digest")||num(previous,"sequence")!=Some(seq)||text(previous,"instrument_id")!=text(x,"instrument_id"){return "CONFLICT_REJECTED";}
            continue;
        }
        let key_conflict=unique.iter().find(|p|text(p,"boot_epoch_id")==text(x,"boot_epoch_id")&&num(p,"sequence")==Some(seq));
        if let Some(previous)=key_conflict{
            if text(previous,"sample_id")==text(x,"sample_id")&&text(previous,"payload_digest")==text(x,"payload_digest"){continue;}
            return "CONFLICT_REJECTED";
        }
        unique.push(x);
    }
    if unique.is_empty()||num(s,"expected_sample_count")!=Some(unique.len() as i64){return "GAP_INDETERMINATE";}
    if text(map,"state")=="verified"{
        let min_tick=unique.iter().filter_map(|x|num(x,"event_tick")).min().unwrap_or(i64::MAX);
        let max_tick=unique.iter().filter_map(|x|num(x,"event_tick")).max().unwrap_or(i64::MIN);
        if text(map,"source_clock_domain_id")!=text(s,"clock_domain_id")||num(map,"valid_from_event_tick").is_none_or(|v|v>min_tick)||num(map,"valid_to_event_tick").is_none_or(|v|v<max_tick)||text(map,"source_receipt_id").is_empty(){return "NOT_COMPARABLE";}
    }else if text(map,"state")!="not_required"{return "MORE_EVIDENCE_REQUIRED";}
    if text(map,"state")=="not_required"&&text(s,"timestamp_basis")=="utc_unix"&&text(s,"clock_domain_id")!="UTC"{return "NOT_COMPARABLE";}
    if text(s,"timestamp_basis")=="local_wall_time"{
        for i in 0..unique.len(){for j in (i+1)..unique.len(){if text(unique[i],"local_time")==text(unique[j],"local_time")&&(unique[i]["dst_fold"].is_null()||unique[j]["dst_fold"].is_null()){return "MORE_EVIDENCE_REQUIRED";}}}
    }
    let mut max_event=i64::MIN;
    let reorder=num(s,"maximum_reorder_window_ns").unwrap_or(-1);
    for x in &all{
        let event=num(x,"event_tick").unwrap_or(i64::MIN);
        if event<max_event&&max_event-event>reorder{return "NOT_COMPARABLE";}
        max_event=max_event.max(event);
    }
    let mut epochs:BTreeSet<String>=BTreeSet::new();
    for x in &unique{epochs.insert(text(x,"boot_epoch_id").to_owned());}
    let epochs:Vec<String>=epochs.into_iter().collect();
    if epochs.len()>1{
        for i in 1..epochs.len(){
            let found=r["epoch_transitions"].as_array().is_some_and(|a|a.iter().any(|t|text(t,"from_boot_epoch_id")==epochs[i-1]&&text(t,"to_boot_epoch_id")==epochs[i]&&text(t,"transition_kind")=="reboot"&&!text(t,"receipt_id").is_empty()));
            if !found{return "INVALIDATED";}
        }
    }else if epochs.first().is_none_or(|e|e!=text(s,"boot_epoch_id")){return "INVALIDATED";}
    for epoch in &epochs{
        let mut group:Vec<&Value>=unique.iter().copied().filter(|x|text(x,"boot_epoch_id")==epoch).collect();
        group.sort_by_key(|x|num(x,"event_tick").unwrap_or(i64::MAX));
        for pair in group.windows(2){
            let prev=num(pair[0],"sequence").unwrap_or(-1);let next=num(pair[1],"sequence").unwrap_or(-1);
            if next==prev+1{continue;}
            let max=if width==64{i64::MAX}else{(1i64<<width)-1};
            if next==0&&prev==max&&text(s,"wrap_policy")=="declared_modulo"{continue;}
            if next<prev{return "INVALIDATED";}
            return "GAP_INDETERMINATE";
        }
    }
    if epochs.len()==1{
        let mut ordered=unique.clone();ordered.sort_by_key(|x|num(x,"event_tick").unwrap_or(i64::MAX));
        if !s["expected_sequence_start"].is_null()&&num(&ordered[0],"sequence")!=num(s,"expected_sequence_start"){return "GAP_INDETERMINATE";}
        if !s["expected_sequence_end"].is_null()&&num(ordered[ordered.len()-1],"sequence")!=num(s,"expected_sequence_end"){return "GAP_INDETERMINATE";}
    }
    match text(inv,"change_scope"){
        "other_stream"=>{
            if flag(inv,"dependent_claim_stale")||!flag(inv,"raw_samples_preserved")||!flag(inv,"unrelated_streams_current"){return "INVALIDATED";}
            return "UNRELATED_STREAM_UNCHANGED";
        }
        "this_stream"=>{
            if flag(inv,"dependent_claim_stale")&&flag(inv,"raw_samples_preserved")&&flag(inv,"unrelated_streams_current"){return "TARGETED_INVALIDATION_ONLY";}
            return "INVALIDATED";
        }
        _=>{}
    }
    "REVIEW_ELIGIBLE_ONLY"
}
pub fn validate_corpus(c:&Value)->Result<usize,String>{
    if !exact(c,&["schema_version","corpus_id","classification","claim_ceiling","profile_id","cases"])
        ||text(c,"schema_version")!="mycelix.building-measurement-stream-corpus/0.1"||text(c,"corpus_id")!=CORPUS_ID
        ||text(c,"classification")!="synthetic_clock_sequence_and_replay_cases"
        ||text(c,"claim_ceiling")!="synthetic_stream_ordering_and_replay_conformance_only"||text(c,"profile_id")!=PROFILE_ID
    {return Err("CORPUS_IDENTITY_INVALID".into());}
    let cases=c["cases"].as_array().ok_or("CASES_MISSING")?;
    if cases.len()!=CASES.len(){return Err("CASE_COUNT_INVALID".into());}
    for(case,(id,expected))in cases.iter().zip(CASES){
        if !exact(case,&["case_id","description","record","expected_disposition"])
            ||text(case,"case_id")!=*id||text(case,"expected_disposition")!=*expected
            ||!exact(&case["record"],ROOT_FIELDS)||!exact(&case["record"]["stream"],STREAM_FIELDS)
            ||!exact(&case["record"]["samples"][0],SAMPLE_FIELDS)
            ||case["record"]["samples"].as_array().is_none_or(Vec::is_empty)
        {return Err(format!("CASE_SHAPE_OR_PIN_INVALID:{id}"));}
        let actual=classify_record(&case["record"]);
        if actual!=*expected{return Err(format!("{id}:EXPECTED_{expected}:GOT_{actual}"));}
    }
    Ok(cases.len())
}
pub fn validate_tracked()->Result<usize,String>{
    let profile=parse_unique(PROFILE_JSON)?;let schema=parse_unique(SCHEMA_JSON)?;let corpus=parse_unique(CORPUS_JSON)?;
    if text(&profile,"profile_id")!=PROFILE_ID||schema["type"]!="object"||schema["additionalProperties"]!=false
        ||schema["properties"]["cases"]["minItems"]!=40||schema["properties"]["cases"]["maxItems"]!=40
        ||schema["$defs"]["record"]["additionalProperties"]!=false
    {return Err("SCHEMA_PROFILE_CONTRACT_INVALID".into());}
    validate_corpus(&corpus)
}

#[cfg(test)]
mod tests{
 use super::*;
 fn case(id:&str)->Value{let c=parse_unique(CORPUS_JSON).unwrap();c["cases"].as_array().unwrap().iter().find(|x|text(x,"case_id")==id).unwrap().clone()}
 #[test]fn tracked_forty_case_corpus_matches_pins(){assert_eq!(validate_tracked().unwrap(),40);}
 #[test]fn duplicate_json_members_rejected(){assert!(parse_unique(r#"{"x":{"a":1,"a":2}}"#).is_err());}
 #[test]fn conflicting_duplicate_packet_is_rejected(){assert_eq!(classify_record(&case("MST-003")["record"]),"CONFLICT_REJECTED");}
 #[test]fn identical_duplicate_packet_is_idempotent(){assert_eq!(classify_record(&case("MST-002")["record"]),"REVIEW_ELIGIBLE_ONLY");}
 #[test]fn sequence_gap_is_not_zero(){assert_eq!(classify_record(&case("MST-004")["record"]),"GAP_INDETERMINATE");}
 #[test]fn bounded_reordering_is_distinct_from_late_outside_window(){assert_eq!(classify_record(&case("MST-005")["record"]),"REVIEW_ELIGIBLE_ONLY");assert_eq!(classify_record(&case("MST-006")["record"]),"NOT_COMPARABLE");}
 #[test]fn reboot_requires_a_new_epoch_receipt(){assert_eq!(classify_record(&case("MST-007")["record"]),"INVALIDATED");assert_eq!(classify_record(&case("MST-008")["record"]),"REVIEW_ELIGIBLE_ONLY");}
 #[test]fn wrap_requires_width_and_declared_modulo(){assert_eq!(classify_record(&case("MST-009")["record"]),"REVIEW_ELIGIBLE_ONLY");assert_eq!(classify_record(&case("MST-010")["record"]),"MORE_EVIDENCE_REQUIRED");}
 #[test]fn clock_steps_and_source_changes_invalidate(){for id in ["MST-011","MST-012","MST-013","MST-014"]{assert_eq!(classify_record(&case(id)["record"]),"INVALIDATED");}}
 #[test]fn sync_uncertainty_is_purpose_bounded(){assert_eq!(classify_record(&case("MST-015")["record"]),"REJECTED_FOR_PURPOSE");}
 #[test]fn time_units_and_origins_cannot_be_inferred(){for id in ["MST-016","MST-017","MST-018"]{assert_eq!(classify_record(&case(id)["record"]),"NOT_COMPARABLE");}}
 #[test]fn local_calendar_and_leap_second_ambiguity_is_explicit(){for id in ["MST-019","MST-020","MST-023"]{assert_eq!(classify_record(&case(id)["record"]),"MORE_EVIDENCE_REQUIRED");}for id in ["MST-021","MST-022"]{assert_eq!(classify_record(&case(id)["record"]),"NOT_COMPARABLE");}}
 #[test]fn receipt_time_never_substitutes_for_event_time(){assert_eq!(classify_record(&case("MST-025")["record"]),"REJECTED_TIME_BASIS_PROMOTION");assert_eq!(classify_record(&case("MST-038")["record"]),"REJECTED_TIME_BASIS_PROMOTION");}
 #[test]fn identity_firmware_and_epoch_mismatch_invalidate(){for id in ["MST-027","MST-028","MST-029","MST-033"]{assert_eq!(classify_record(&case(id)["record"]),"INVALIDATED");}}
 #[test]fn correction_mapping_is_versioned_and_window_bounded(){assert_eq!(classify_record(&case("MST-030")["record"]),"INVALIDATED");assert_eq!(classify_record(&case("MST-031")["record"]),"NOT_COMPARABLE");assert_eq!(classify_record(&case("MST-032")["record"]),"REJECTED_FOR_PURPOSE");}
 #[test]fn aggregate_invalidation_is_targeted(){assert_eq!(classify_record(&case("MST-039")["record"]),"UNRELATED_STREAM_UNCHANGED");assert_eq!(classify_record(&case("MST-040")["record"]),"TARGETED_INVALIDATION_ONLY");}
}
