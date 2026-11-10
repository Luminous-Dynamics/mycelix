//! Synthetic-only condition-profile applicability oracle.
//! This does not calibrate instruments, evaluate a real building or authorize use.
#![forbid(unsafe_code)]

use serde::de::{MapAccess, SeqAccess, Visitor};
use serde::{Deserialize, Deserializer};
use serde_json::{Map, Value};
use std::collections::BTreeSet;
use std::fmt;

pub const CORPUS_JSON:&str=include_str!("../../../docs/metrology/BUILDING_CONDITION_APPLICABILITY_CORPUS_V0_1.json");
pub const PROFILE_SCHEMA_JSON:&str=include_str!("../../../docs/metrology/BUILDING_CONDITION_APPLICABILITY_PROFILE_V0_1.schema.json");
pub const CORPUS_SCHEMA_JSON:&str=include_str!("../../../docs/metrology/BUILDING_CONDITION_APPLICABILITY_SCHEMA_V0_1.json");

const EXPECTED_CASES:&[(&str,&str)]=&[
("CCA-001","WITHIN_DECLARED_ENVELOPE"),
("CCA-002","INDETERMINATE_BOUNDARY_OVERLAP"),
("CCA-003","OUTSIDE_DECLARED_ENVELOPE"),
("CCA-004","OUTSIDE_DECLARED_ENVELOPE"),
("CCA-005","MISSING_REQUIRED_CONDITION"),
("CCA-006","MISSING_REQUIRED_CONDITION"),
("CCA-007","EXPIRED_OR_INVALIDATED"),
("CCA-008","EXPIRED_OR_INVALIDATED"),
("CCA-009","REJECTED_FOR_DECLARED_PURPOSE"),
("CCA-010","NOT_COMPARABLE"),
("CCA-011","NOT_COMPARABLE"),
("CCA-012","NOT_COMPARABLE"),
("CCA-013","NOT_COMPARABLE"),
("CCA-014","NOT_COMPARABLE"),
("CCA-015","NOT_COMPARABLE"),
("CCA-016","NOT_COMPARABLE"),
("CCA-017","NOT_COMPARABLE"),
("CCA-018","NOT_COMPARABLE"),
("CCA-019","MISSING_REQUIRED_CONDITION"),
("CCA-020","NOT_COMPARABLE"),
("CCA-021","NOT_COMPARABLE"),
("CCA-022","NOT_COMPARABLE"),
("CCA-023","NOT_COMPARABLE"),
("CCA-024","OUTSIDE_DECLARED_ENVELOPE"),
("CCA-025","OUTSIDE_DECLARED_ENVELOPE"),
("CCA-026","INDETERMINATE_BOUNDARY_OVERLAP"),
("CCA-027","OUTSIDE_DECLARED_ENVELOPE"),
("CCA-028","INDETERMINATE_BOUNDARY_OVERLAP"),
("CCA-029","OUTSIDE_DECLARED_ENVELOPE"),
("CCA-030","OUTSIDE_DECLARED_ENVELOPE"),
("CCA-031","MISSING_REQUIRED_CONDITION"),
("CCA-032","MISSING_REQUIRED_CONDITION"),
("CCA-033","MISSING_REQUIRED_CONDITION"),
("CCA-034","EXPIRED_OR_INVALIDATED"),
("CCA-035","NOT_COMPARABLE"),
("CCA-036","MISSING_REQUIRED_CONDITION"),
("CCA-037","WITHIN_DECLARED_ENVELOPE"),
("CCA-038","WITHIN_DECLARED_ENVELOPE"),
("CCA-039","WITHIN_DECLARED_ENVELOPE"),
("CCA-040","INDETERMINATE_BOUNDARY_OVERLAP"),
];

const PROFILE_FIELDS:&[&str]=&["schema_version","profile_id","profile_revision","domain_id","classification","claim_ceiling","subject_instrument_id","sample_tick","timebase_id","clock_epoch_id","spatial_scope_id","dimensions"];
const DIMENSION_FIELDS:&[&str]=&["dimension_id","kind","quantity_kind","unit","minimum","maximum","allowed_values","method_id","method_revision","critical"];
const OBS_FIELDS:&[&str]=&["dimension_id","value","unit","uncertainty_value","uncertainty_unit","source_kind","source_id","source_state","source_receipt_id","calibration_state","rights_state","quality_state","method_id","method_revision","profile_id","profile_revision","timebase_id","clock_epoch_id","valid_from_tick","valid_to_tick","spatial_scope_id"];
const CORPUS_FIELDS:&[&str]=&["schema_version","corpus_id","classification","claim_ceiling","cases"];
const CASE_FIELDS:&[&str]=&["case_id","description","record","expected_disposition"];
const RECORD_FIELDS:&[&str]=&["profile","observations"];

#[derive(Debug)]struct UniqueJson(Value);
impl<'de> Deserialize<'de> for UniqueJson{
 fn deserialize<D>(d:D)->Result<Self,D::Error> where D:Deserializer<'de>{d.deserialize_any(UniqueVisitor)}
}
struct UniqueVisitor;
impl<'de> Visitor<'de> for UniqueVisitor{
 type Value=UniqueJson;
 fn expecting(&self,f:&mut fmt::Formatter<'_>)->fmt::Result{f.write_str("JSON without duplicate object member names")}
 fn visit_bool<E>(self,v:bool)->Result<Self::Value,E> where E:serde::de::Error{Ok(UniqueJson(Value::Bool(v)))}
 fn visit_i64<E>(self,v:i64)->Result<Self::Value,E> where E:serde::de::Error{Ok(UniqueJson(Value::Number(v.into())))}
 fn visit_u64<E>(self,v:u64)->Result<Self::Value,E> where E:serde::de::Error{Ok(UniqueJson(Value::Number(v.into())))}
 fn visit_f64<E>(self,v:f64)->Result<Self::Value,E> where E:serde::de::Error{let n=serde_json::Number::from_f64(v).ok_or_else(||E::custom("non-finite"))?;Ok(UniqueJson(Value::Number(n)))}
 fn visit_str<E>(self,v:&str)->Result<Self::Value,E> where E:serde::de::Error{Ok(UniqueJson(Value::String(v.into())))}
 fn visit_string<E>(self,v:String)->Result<Self::Value,E> where E:serde::de::Error{Ok(UniqueJson(Value::String(v)))}
 fn visit_unit<E>(self)->Result<Self::Value,E> where E:serde::de::Error{Ok(UniqueJson(Value::Null))}
 fn visit_none<E>(self)->Result<Self::Value,E> where E:serde::de::Error{Ok(UniqueJson(Value::Null))}
 fn visit_some<D>(self,d:D)->Result<Self::Value,D::Error> where D:Deserializer<'de>{UniqueJson::deserialize(d)}
 fn visit_seq<A>(self,mut a:A)->Result<Self::Value,A::Error> where A:SeqAccess<'de>{let mut v=Vec::new();while let Some(x)=a.next_element::<UniqueJson>()?{v.push(x.0);}Ok(UniqueJson(Value::Array(v)))}
 fn visit_map<A>(self,mut a:A)->Result<Self::Value,A::Error> where A:MapAccess<'de>{let mut v=Map::new();let mut seen=BTreeSet::new();while let Some(k)=a.next_key::<String>()?{if !seen.insert(k.clone()){return Err(serde::de::Error::custom(format!("duplicate member {k}")));}let x=a.next_value::<UniqueJson>()?;v.insert(k,x.0);}Ok(UniqueJson(Value::Object(v)))}
}
fn parse(s:&str)->Result<Value,String>{let mut d=serde_json::Deserializer::from_str(s);let x=UniqueJson::deserialize(&mut d).map_err(|e|format!("JSON_INVALID:{e}"))?;d.end().map_err(|e|format!("JSON_TRAILING:{e}"))?;Ok(x.0)}
fn txt<'a>(v:&'a Value,k:&str)->&'a str{v.get(k).and_then(Value::as_str).unwrap_or("")}
fn num(v:&Value,k:&str)->Option<f64>{v.get(k).and_then(Value::as_f64).filter(|x|x.is_finite())}
fn exact(v:&Value,keys:&[&str])->bool{let Some(o)=v.as_object()else{return false};o.len()==keys.len()&&keys.iter().all(|k|o.contains_key(*k))}
fn arr_eq(v:&Value,w:&[&str])->bool{v.as_array().is_some_and(|a|a.len()==w.len()&&a.iter().zip(w).all(|(x,y)|x.as_str()==Some(*y)))}
fn nonempty(v:&Value,k:&str)->bool{!txt(v,k).trim().is_empty()}

fn expected_dimensions(domain:&str)->Option<Vec<(&'static str,&'static str,&'static str,f64,f64,&'static str,&'static str)>>{
 match domain{
 "thermal-environment"=>Some(vec![
  ("ambient_temperature","numeric_range","degC",5.0,45.0,"ISO-7726-2025-TEMP","2025"),
  ("relative_humidity","numeric_range","%RH",15.0,85.0,"ISO-7726-2025-RH","2025"),
 ]),
 "water-closed-conduit"=>Some(vec![
  ("conduit_fully_charged","categorical","1",0.0,0.0,"ISO-4064-1-2024","2024"),
  ("flow_profile","categorical","1",0.0,0.0,"ISO-4064-1-2024","2024"),
  ("fluid_temperature","numeric_range","degC",5.0,60.0,"ISO-4064-1-2024","2024"),
 ]),
 "ac-power-quality-50hz"=>Some(vec![
  ("fundamental_frequency","numeric_range","Hz",49.0,51.0,"IEC-61000-4-30-2025","2025-COR1-2026-07"),
  ("wiring_topology","categorical","1",0.0,0.0,"IEC-61000-4-30-2025","2025-COR1-2026-07"),
 ]),
 "pm25-monitoring"=>Some(vec![
  ("relative_humidity","numeric_range","%RH",10.0,80.0,"ISO-16000-37-2019","2019"),
  ("aerosol_profile","categorical","1",0.0,0.0,"ISO-16000-37-2019","2019"),
 ]),
 _=>None
 }
}
fn expected_allowed_values(domain:&str,dimension:&str)->Option<&'static [&'static str]>{
 match (domain,dimension){
  ("water-closed-conduit","conduit_fully_charged")=>Some(&["true"]),
  ("water-closed-conduit","flow_profile")=>Some(&["within_declared_meter_installation_profile"]),
  ("ac-power-quality-50hz","wiring_topology")=>Some(&["single_phase_correctly_wired","three_phase_correctly_wired"]),
  ("pm25-monitoring","aerosol_profile")=>Some(&["reference_method_co_location_validated"]),
  _=>None
 }
}
fn validate_profile(p:&Value)->Result<(),String>{
 if !exact(p,PROFILE_FIELDS)||txt(p,"schema_version")!="mycelix.building-condition-profile/0.1"
 ||txt(p,"classification")!="synthetic_condition_applicability_profile"
 ||txt(p,"claim_ceiling")!="synthetic_condition_envelope_match_only"
 ||!nonempty(p,"profile_id")||!nonempty(p,"profile_revision")||!nonempty(p,"subject_instrument_id")
 ||!nonempty(p,"timebase_id")||!nonempty(p,"clock_epoch_id")||!nonempty(p,"spatial_scope_id")
 ||num(p,"sample_tick").is_none_or(|x|x<0.0||x.fract()!=0.0)
 {return Err("PROFILE_SHAPE_OR_ID_INVALID".into());}
 let expected=expected_dimensions(txt(p,"domain_id")).ok_or("PROFILE_DOMAIN_UNSUPPORTED")?;
 let dims=p["dimensions"].as_array().ok_or("PROFILE_DIMENSIONS_MISSING")?;
 if dims.len()!=expected.len(){return Err("PROFILE_DIMENSION_COUNT_INVALID".into());}
 for(d,(id,kind,unit,min,max,method,rev))in dims.iter().zip(expected){
  if !exact(d,DIMENSION_FIELDS)||txt(d,"dimension_id")!=id||txt(d,"kind")!=kind||txt(d,"unit")!=unit
  ||txt(d,"method_id")!=method||txt(d,"method_revision")!=rev||d["critical"].as_bool()!=Some(true)
  {return Err(format!("PROFILE_DIMENSION_DRIFT:{id}"));}
  if kind=="numeric_range"{
   if num(d,"minimum")!=Some(min)||num(d,"maximum")!=Some(max)||!d["allowed_values"].as_array().is_some_and(|x|x.is_empty()){return Err(format!("PROFILE_RANGE_DRIFT:{id}"));}
  }else{
   let Some(allowed)=expected_allowed_values(txt(p,"domain_id"),id)else{return Err(format!("PROFILE_CATEGORY_UNSUPPORTED:{id}"));};
   if !d["minimum"].is_null()||!d["maximum"].is_null()||!arr_eq(&d["allowed_values"],allowed){
    return Err(format!("PROFILE_CATEGORY_DRIFT:{id}"));
   }
  }
 }
 Ok(())
}
fn dimension<'a>(p:&'a Value,id:&str)->Option<&'a Value>{p["dimensions"].as_array()?.iter().find(|d|txt(d,"dimension_id")==id)}
fn classify(record:&Value)->&'static str{
 if !exact(record,RECORD_FIELDS){return "NOT_COMPARABLE";}
 let p=&record["profile"];
 if validate_profile(p).is_err(){return "NOT_COMPARABLE";}
 let observations=record["observations"].as_array();
 let Some(observations)=observations else{return "MISSING_REQUIRED_CONDITION";}
 let mut result=Vec::new();
 for d in p["dimensions"].as_array().expect("validated profile dimensions"){
  let id=txt(d,"dimension_id");
  let found:Vec<&Value>=observations.iter().filter(|o|txt(o,"dimension_id")==id).collect();
  if found.is_empty(){result.push("MISSING_REQUIRED_CONDITION");continue;}
  if found.len()!=1{result.push("NOT_COMPARABLE");continue;}
  let o=found[0];
  if !exact(o,OBS_FIELDS)||txt(o,"profile_id")!=txt(p,"profile_id")||txt(o,"profile_revision")!=txt(p,"profile_revision")
   ||txt(o,"unit")!=txt(d,"unit")||txt(o,"method_id")!=txt(d,"method_id")||txt(o,"method_revision")!=txt(d,"method_revision")
   ||txt(o,"timebase_id")!=txt(p,"timebase_id")||txt(o,"clock_epoch_id")!=txt(p,"clock_epoch_id")
   ||txt(o,"spatial_scope_id")!=txt(p,"spatial_scope_id")
   ||num(o,"valid_from_tick").is_none_or(|x|x.fract()!=0.0||x>num(p,"sample_tick").unwrap_or(-1.0))
   ||num(o,"valid_to_tick").is_none_or(|x|x.fract()!=0.0||x<num(p,"sample_tick").unwrap_or(-1.0))
   ||num(o,"valid_to_tick").unwrap_or(-1.0)<num(o,"valid_from_tick").unwrap_or(0.0)
  {return "NOT_COMPARABLE";}
  if !nonempty(o,"source_id")||txt(o,"source_id")==txt(p,"subject_instrument_id"){return "NOT_COMPARABLE";}
  if !nonempty(o,"source_receipt_id"){return "MISSING_REQUIRED_CONDITION";}
  if txt(o,"rights_state")=="revoked"{return "REJECTED_FOR_DECLARED_PURPOSE";}
  if txt(o,"calibration_state")=="expired"||matches!(txt(o,"quality_state"),"stale"|"device_fault"|"out_of_range"){return "EXPIRED_OR_INVALIDATED";}
  if txt(o,"rights_state")!="authorized"||txt(o,"source_state")!="independently_admitted"
   ||!matches!(txt(o,"calibration_state"),"current"|"not_applicable")||txt(o,"quality_state")!="valid"
  {return "MISSING_REQUIRED_CONDITION";}
  match txt(d,"kind"){
   "numeric_range"=>{
    if txt(o,"source_kind")!="reference_instrument"||num(o,"value").is_none()
     ||num(o,"uncertainty_value").is_none_or(|x|x<0.0)||txt(o,"uncertainty_unit")!=txt(d,"unit")
    {return "MISSING_REQUIRED_CONDITION";}
    let y=num(o,"value").unwrap();let u=num(o,"uncertainty_value").unwrap();
    let lo=y-u;let hi=y+u;
    let min=num(d,"minimum").unwrap();let max=num(d,"maximum").unwrap();
    if hi<min||lo>max{result.push("OUTSIDE_DECLARED_ENVELOPE");}
    else if lo>=min&&hi<=max{result.push("WITHIN_DECLARED_ENVELOPE");}
    else{result.push("INDETERMINATE_BOUNDARY_OVERLAP");}
   },
   "categorical"=>{
    if txt(o,"source_kind")!="verified_configuration"||!o["value"].is_string()||!o["uncertainty_value"].is_null()||txt(o,"uncertainty_unit")!="NOT_APPLICABLE"
    {return "MISSING_REQUIRED_CONDITION";}
    let allowed=d["allowed_values"].as_array().unwrap_or(&Vec::new());
    if allowed.iter().any(|v|v.as_str()==Some(txt(o,"value"))){result.push("WITHIN_DECLARED_ENVELOPE");}
    else{result.push("OUTSIDE_DECLARED_ENVELOPE");}
   },
   _=>return "NOT_COMPARABLE"
  }
 }
 if result.contains(&"OUTSIDE_DECLARED_ENVELOPE"){return "OUTSIDE_DECLARED_ENVELOPE";}
 if result.contains(&"INDETERMINATE_BOUNDARY_OVERLAP"){return "INDETERMINATE_BOUNDARY_OVERLAP";}
 if result.contains(&"NOT_COMPARABLE"){return "NOT_COMPARABLE";}
 if result.contains(&"MISSING_REQUIRED_CONDITION"){return "MISSING_REQUIRED_CONDITION";}
 "WITHIN_DECLARED_ENVELOPE"
}
pub fn validate_corpus(c:&Value)->Result<usize,String>{
 if !exact(c,CORPUS_FIELDS)||txt(c,"schema_version")!="mycelix.building-condition-applicability-corpus/0.1"
 ||txt(c,"corpus_id")!="BUILDING-CONDITION-APPLICABILITY-CORPUS-V0.1"
 ||txt(c,"classification")!="synthetic_condition_applicability_cases"
 ||txt(c,"claim_ceiling")!="synthetic_condition_envelope_match_only"
 {return Err("CORPUS_IDENTITY_INVALID".into());}
 let cases=c["cases"].as_array().ok_or("CASES_MISSING")?;
 if cases.len()!=EXPECTED_CASES.len(){return Err("CASE_COUNT_INVALID".into());}
 for(case,(id,want))in cases.iter().zip(EXPECTED_CASES){
  if !exact(case,CASE_FIELDS)||txt(case,"case_id")!=*id||txt(case,"description").len()<20||txt(case,"expected_disposition")!=*want
  {return Err(format!("CASE_EXPECTATION_DRIFT:{id}"));}
  validate_record_shape(&case["record"])?;
  let actual=classify(&case["record"]);
  if actual!=*want{return Err(format!("CASE_DISPOSITION_MISMATCH:{id}:expected={want}:actual={actual}"));}
 }
 Ok(cases.len())
}
fn schema_object_matches(schema:&Value,fields:&[&str])->bool{
 schema["type"]=="object"&&schema["additionalProperties"]==false
  &&arr_eq(&schema["required"],fields)
  &&exact(&schema["properties"],fields)
}
fn validate_schema_contracts(profile_schema:&Value,corpus_schema:&Value)->Result<(),String>{
 if txt(profile_schema,"title")!="Mycelix Building Condition Profile v0.1"
  ||profile_schema["type"]!="object"||profile_schema["additionalProperties"]!=false
  ||!arr_eq(&profile_schema["required"],PROFILE_FIELDS)
  ||!exact(&profile_schema["properties"],PROFILE_FIELDS)
  ||!schema_object_matches(&profile_schema["$defs"]["dimension"],DIMENSION_FIELDS)
  ||profile_schema["properties"]["dimensions"]["items"]["$ref"]!="#/$defs/dimension"
 {return Err("PROFILE_SCHEMA_CONTRACT_INVALID".into());}
 if txt(corpus_schema,"title")!="Mycelix Building Condition Applicability Corpus v0.1"
  ||corpus_schema["type"]!="object"||corpus_schema["additionalProperties"]!=false
  ||!arr_eq(&corpus_schema["required"],CORPUS_FIELDS)
  ||!exact(&corpus_schema["properties"],CORPUS_FIELDS)
  ||corpus_schema["properties"]["cases"]["minItems"]!=EXPECTED_CASES.len()
  ||corpus_schema["properties"]["cases"]["maxItems"]!=EXPECTED_CASES.len()
  ||corpus_schema["properties"]["cases"]["items"]["$ref"]!="#/$defs/case"
  ||!schema_object_matches(&corpus_schema["$defs"]["case"],CASE_FIELDS)
  ||!schema_object_matches(&corpus_schema["$defs"]["profile"],PROFILE_FIELDS)
  ||!schema_object_matches(&corpus_schema["$defs"]["case"]["properties"]["record"],RECORD_FIELDS)
  ||corpus_schema["$defs"]["case"]["properties"]["record"]["properties"]["profile"]["$ref"]!="#/$defs/profile"
  ||corpus_schema["$defs"]["case"]["properties"]["record"]["properties"]["observations"]["items"]["$ref"]!="#/$defs/observation"
  ||!schema_object_matches(&corpus_schema["$defs"]["dimension"],DIMENSION_FIELDS)
  ||!schema_object_matches(&corpus_schema["$defs"]["observation"],OBS_FIELDS)
 {return Err("CORPUS_SCHEMA_CONTRACT_INVALID".into());}
 Ok(())
}
fn validate_record_shape(record:&Value)->Result<(),String>{
 if !exact(record,RECORD_FIELDS){return Err("RECORD_FIELDS_INVALID".into());}
 let p=&record["profile"];
 if !exact(p,PROFILE_FIELDS){return Err("PROFILE_FIELDS_INVALID".into());}
 let dims=p["dimensions"].as_array().ok_or("DIMENSIONS_MISSING")?;
 for d in dims{if !exact(d,DIMENSION_FIELDS){return Err("DIMENSION_FIELDS_INVALID".into());}}
 let observations=record["observations"].as_array().ok_or("OBSERVATIONS_MISSING")?;
 for o in observations{
  if !exact(o,OBS_FIELDS){return Err("OBSERVATION_FIELDS_INVALID".into());}
  if !o["value"].is_number()&&!o["value"].is_string(){return Err("OBSERVATION_VALUE_INVALID".into());}
  if !o["uncertainty_value"].is_null()&&num(o,"uncertainty_value").is_none(){return Err("OBSERVATION_UNCERTAINTY_INVALID".into());}
 }
 Ok(())
}
pub fn validate_tracked()->Result<usize,String>{
 let profile_schema=parse(PROFILE_SCHEMA_JSON)?;
 let corpus_schema=parse(CORPUS_SCHEMA_JSON)?;
 validate_schema_contracts(&profile_schema,&corpus_schema)?;
 let corpus=parse(CORPUS_JSON)?;
 validate_corpus(&corpus)
}
#[cfg(test)]
mod tests{
 use super::*;
 fn case(id:&str)->Value{let c=parse(CORPUS_JSON).unwrap();c["cases"].as_array().unwrap().iter().find(|x|txt(x,"case_id")==id).unwrap().clone()}
 #[test]fn all_40_known_answers_match(){assert_eq!(validate_tracked().unwrap(),40);}

 #[test]fn categorical_vocabularies_are_frozen_not_self_defined(){let mut c=case("CCA-024");c["record"]["profile"]["dimensions"][0]["allowed_values"][0]=Value::String("false".into());assert_eq!(classify(&c["record"]),"NOT_COMPARABLE");}
 #[test]fn source_receipt_absence_is_missing_evidence(){assert_eq!(classify(&case("CCA-032")["record"]),"MISSING_REQUIRED_CONDITION");}
 #[test]fn schema_contract_checks_the_standalone_profile_schema(){let s=parse(PROFILE_SCHEMA_JSON).unwrap();assert!(schema_object_matches(&s["$defs"]["dimension"],DIMENSION_FIELDS));}
 #[test]fn unknown_nested_fields_are_rejected_even_in_negative_cases(){let mut c=case("CCA-017");c["record"]["observations"][0]["unreviewed"]=Value::Bool(true);assert!(validate_record_shape(&c["record"]).is_err());}
 #[test]fn duplicate_json_member_is_rejected(){assert!(parse(r#"{"a":1,"a":2}"#).is_err());}
 #[test]fn missing_critical_condition_never_becomes_within(){assert_eq!(classify(&case("CCA-005")["record"]),"MISSING_REQUIRED_CONDITION");}
 #[test]fn boundary_overlap_is_indeterminate(){assert_eq!(classify(&case("CCA-002")["record"]),"INDETERMINATE_BOUNDARY_OVERLAP");}
 #[test]fn entirely_outside_interval_is_outside(){assert_eq!(classify(&case("CCA-003")["record"]),"OUTSIDE_DECLARED_ENVELOPE");}
 #[test]fn source_cannot_validate_its_own_operating_condition(){assert_eq!(classify(&case("CCA-018")["record"]),"NOT_COMPARABLE");}
 #[test]fn method_revision_must_match(){assert_eq!(classify(&case("CCA-012")["record"]),"NOT_COMPARABLE");}
 #[test]fn timebase_and_clock_epoch_must_match(){assert_eq!(classify(&case("CCA-014")["record"]),"NOT_COMPARABLE");assert_eq!(classify(&case("CCA-015")["record"]),"NOT_COMPARABLE");}
 #[test]fn spatial_scope_must_match(){assert_eq!(classify(&case("CCA-016")["record"]),"NOT_COMPARABLE");}
 #[test]fn water_closed_conduit_scope_is_explicit(){assert_eq!(classify(&case("CCA-024")["record"]),"OUTSIDE_DECLARED_ENVELOPE");}
 #[test]fn electrical_power_quality_conditions_are_a_distinct_domain(){assert_eq!(classify(&case("CCA-038")["record"]),"WITHIN_DECLARED_ENVELOPE");}
 #[test]fn pm25_method_context_is_separate(){assert_eq!(classify(&case("CCA-039")["record"]),"WITHIN_DECLARED_ENVELOPE");}
 #[test]fn missing_uncertainty_is_not_zero(){assert_eq!(classify(&case("CCA-019")["record"]),"MISSING_REQUIRED_CONDITION");}
 #[test]fn revoked_rights_reject_condition_use(){assert_eq!(classify(&case("CCA-009")["record"]),"REJECTED_FOR_DECLARED_PURPOSE");}
}
