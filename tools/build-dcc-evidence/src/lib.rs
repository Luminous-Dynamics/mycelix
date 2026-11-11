//! Synthetic-only trust/applicability state machine for PTB DCC 3.3.0 reports.
//! It does not parse XML, validate XSD bytes, or implement XML Digital Signatures.

#![forbid(unsafe_code)]

use serde::de::{MapAccess,SeqAccess,Visitor};
use serde::{Deserialize,Deserializer};
use serde_json::{Map,Value};
use std::collections::BTreeSet;
use std::fmt;

pub const PROFILE_JSON:&str=include_str!("../../../docs/metrology/BUILDING_DCC_EVIDENCE_PROFILE_V0_1.json");
pub const PROFILE_SCHEMA_JSON:&str=include_str!("../../../docs/metrology/BUILDING_DCC_EVIDENCE_PROFILE_V0_1.schema.json");
pub const CORPUS_JSON:&str=include_str!("../../../docs/metrology/BUILDING_DCC_EVIDENCE_CORPUS_V0_1.json");
pub const CORPUS_SCHEMA_JSON:&str=include_str!("../../../docs/metrology/BUILDING_DCC_EVIDENCE_CORPUS_V0_1.schema.json");

const EXPECTED:&[(&str,&str)]=&[
("DCC-001","ELIGIBLE_FOR_INDEPENDENT_PASSPORT_REVIEW_ONLY"),
("DCC-002","MORE_EVIDENCE_REQUIRED"),("DCC-003","REJECTED_SIGNATURE_INVALID"),
("DCC-004","REJECTED_SIGNATURE_INVALID"),("DCC-005","UNTRUSTED_SIGNER"),
("DCC-006","REJECTED_SIGNATURE_INVALID"),("DCC-007","REJECTED_DOCUMENT"),
("DCC-008","REJECTED_DOCUMENT"),("DCC-009","REJECTED_DOCUMENT"),
("DCC-010","REJECTED_SOURCE_INTEGRITY"),("DCC-011","REJECTED_SIGNED_OBJECT_COVERAGE"),
("DCC-012","REJECTED_XML_ID_AMBIGUITY"),("DCC-013","REJECTED_EXTERNAL_REFERENCE"),
("DCC-014","REJECTED_SIGNATURE_COVERAGE"),("DCC-015","REJECTED_SIGNATURE_COVERAGE"),
("DCC-016","MORE_EVIDENCE_REQUIRED"),("DCC-017","NOT_COMPARABLE"),
("DCC-018","NOT_COMPARABLE"),("DCC-019","NOT_COMPARABLE"),
("DCC-020","NOT_COMPARABLE"),("DCC-021","NOT_COMPARABLE"),
("DCC-022","NOT_COMPARABLE"),("DCC-023","NOT_COMPARABLE"),
("DCC-024","NOT_COMPARABLE"),("DCC-025","EXPIRED_OR_INVALIDATED"),
("DCC-026","EXPIRED_OR_INVALIDATED"),("DCC-027","EXPIRED_OR_INVALIDATED"),
("DCC-028","EXPIRED_OR_INVALIDATED"),("DCC-029","EXPIRED_OR_INVALIDATED"),
("DCC-030","REJECTED_FOR_DECLARED_PURPOSE"),("DCC-031","MORE_EVIDENCE_REQUIRED"),
("DCC-032","MORE_EVIDENCE_REQUIRED"),("DCC-033","NOT_COMPARABLE"),
("DCC-034","MORE_EVIDENCE_REQUIRED"),("DCC-035","MORE_EVIDENCE_REQUIRED"),
("DCC-036","ELIGIBLE_FOR_INDEPENDENT_PASSPORT_REVIEW_ONLY")
];
const ROOT_FIELDS:&[&str]=&["artifact","document","signature","laboratory_scope","calibration_result","passport_binding","correction"];
const ARTIFACT_FIELDS:&[&str]=&["source_uri","original_bytes_digest","received_bytes_digest","byte_length","media_type","source_receipt_id","retention_state"];
const DOCUMENT_FIELDS:&[&str]=&["well_formed_state","namespace_uri","root_element","schema_version","xsd_artifact_state","xsd_digest","xsd_validation_state","id_reference_state","parser_profile_id","parser_profile_state"];
const SIGNATURE_FIELDS:&[&str]=&["presence_state","signature_count","reference_digest_state","signature_value_state","signer_certificate_state","signer_trust_state","signer_id","signer_role_state","signed_reference_uris","covered_object_ids","consumed_object_ids","external_reference_count","duplicate_id_count","transform_policy_state","wrapping_check_state"];
const LAB_FIELDS:&[&str]=&["laboratory_id","accreditation_body_id","schedule_id","schedule_digest","schedule_state","scope_id","scope_state","measurand_id","method_id","method_revision","unit","range_min","range_max","valid_from","valid_to"];
const RESULT_FIELDS:&[&str]=&["result_id","dcc_object_id","instrument_id","laboratory_id","channel_id","measurand_id","unit","method_id","method_revision","range_min","range_max","calibration_date","due_date","firmware_revision","result_state","uncertainty_value","uncertainty_unit","coverage_factor","certificate_rights_state"];
const BINDING_FIELDS:&[&str]=&["instrument_id","channel_id","measurand_id","unit","method_id","method_revision","firmware_revision","sample_observation_id","sample_digest","sample_date","sample_value"];
const CORRECTION_FIELDS:&[&str]=&["supersession_state","previous_artifact_digest","correction_notice_id","dependency_impact_state"];

#[derive(Debug)] struct UniqueJson(Value);
impl<'de> Deserialize<'de> for UniqueJson{
 fn deserialize<D>(d:D)->Result<Self,D::Error> where D:Deserializer<'de>{d.deserialize_any(UniqueVisitor)}
}
struct UniqueVisitor;
impl<'de> Visitor<'de> for UniqueVisitor{
 type Value=UniqueJson;
 fn expecting(&self,f:&mut fmt::Formatter<'_>)->fmt::Result{f.write_str("JSON without duplicate object members")}
 fn visit_bool<E>(self,v:bool)->Result<Self::Value,E> where E:serde::de::Error{Ok(UniqueJson(Value::Bool(v)))}
 fn visit_i64<E>(self,v:i64)->Result<Self::Value,E> where E:serde::de::Error{Ok(UniqueJson(Value::Number(v.into())))}
 fn visit_u64<E>(self,v:u64)->Result<Self::Value,E> where E:serde::de::Error{Ok(UniqueJson(Value::Number(v.into())))}
 fn visit_f64<E>(self,v:f64)->Result<Self::Value,E> where E:serde::de::Error{let n=serde_json::Number::from_f64(v).ok_or_else(||E::custom("non-finite number"))?;Ok(UniqueJson(Value::Number(n)))}
 fn visit_str<E>(self,v:&str)->Result<Self::Value,E> where E:serde::de::Error{Ok(UniqueJson(Value::String(v.to_owned())))}
 fn visit_string<E>(self,v:String)->Result<Self::Value,E> where E:serde::de::Error{Ok(UniqueJson(Value::String(v)))}
 fn visit_unit<E>(self)->Result<Self::Value,E> where E:serde::de::Error{Ok(UniqueJson(Value::Null))}
 fn visit_none<E>(self)->Result<Self::Value,E> where E:serde::de::Error{Ok(UniqueJson(Value::Null))}
 fn visit_seq<A>(self,mut seq:A)->Result<Self::Value,A::Error> where A:SeqAccess<'de>{let mut v=Vec::new();while let Some(x)=seq.next_element::<UniqueJson>()?{v.push(x.0);}Ok(UniqueJson(Value::Array(v)))}
 fn visit_map<A>(self,mut map:A)->Result<Self::Value,A::Error> where A:MapAccess<'de>{let mut v=Map::new();let mut seen=BTreeSet::new();while let Some(k)=map.next_key::<String>()?{if !seen.insert(k.clone()){return Err(serde::de::Error::custom(format!("duplicate member {k:?}")));}let x=map.next_value::<UniqueJson>()?;v.insert(k,x.0);}Ok(UniqueJson(Value::Object(v)))}
}
fn parse(s:&str)->Result<Value,String>{let mut d=serde_json::Deserializer::from_str(s);let v=UniqueJson::deserialize(&mut d).map_err(|e|format!("STRICT_JSON_REJECTED:{e}"))?;d.end().map_err(|e|format!("JSON_TRAILING_DATA:{e}"))?;Ok(v.0)}
fn exact(v:&Value,fields:&[&str])->bool{v.as_object().is_some_and(|o|o.len()==fields.len()&&fields.iter().all(|k|o.contains_key(*k)))}
fn text<'a>(v:&'a Value,k:&str)->&'a str{v.get(k).and_then(Value::as_str).unwrap_or("")}
fn non(v:&Value,k:&str)->bool{!text(v,k).trim().is_empty()}
fn num(v:&Value,k:&str)->Option<f64>{v.get(k).and_then(Value::as_f64).filter(|x|x.is_finite())}
fn strings(v:&Value,k:&str)->Option<BTreeSet<String>>{let a=v.get(k)?.as_array()?;let mut s=BTreeSet::new();for x in a{let t=x.as_str()?;if !s.insert(t.to_owned()){return None;}}Some(s)}


fn str_array_eq(value:&Value,expected:&[&str])->bool{
 value.as_array().is_some_and(|items|items.len()==expected.len()&&items.iter().zip(expected).all(|(v,e)|v.as_str()==Some(*e)))
}
fn json_type_matches(instance:&Value,expected:&str)->bool{
 match expected{
  "object"=>instance.is_object(),"array"=>instance.is_array(),"string"=>instance.is_string(),
  "number"=>instance.as_f64().is_some_and(f64::is_finite),
  "integer"=>instance.as_i64().is_some()||instance.as_u64().is_some()||instance.as_f64().is_some_and(|v|v.is_finite()&&v.fract()==0.0),
  "boolean"=>instance.is_boolean(),"null"=>instance.is_null(),_=>false
 }
}
fn resolve_schema_ref<'a>(root:&'a Value,reference:&str)->Option<&'a Value>{
 let mut current=root;
 let pointer=reference.strip_prefix("#/")?;
 for encoded in pointer.split('/'){
  let key=encoded.replace("~1","/").replace("~0","~");
  current=current.get(key)?;
 }
 Some(current)
}
fn dcc_pattern_matches(pattern:&str,value:&str)->bool{
 if pattern=="^DCC-[0-9]{3}$"{
  let b=value.as_bytes();
  return b.len()==7&&&b[0..4]==b"DCC-"&&b[4..7].iter().all(|x|x.is_ascii_digit());
 }
 false
}
fn validate_json_schema(instance:&Value,schema:&Value,root:&Value,path:&str,errors:&mut Vec<String>,depth:usize){
 if depth>96{errors.push(format!("{path}:SCHEMA_DEPTH_LIMIT"));return;}
 if let Some(reference)=schema.get("$ref").and_then(Value::as_str){
  let Some(target)=resolve_schema_ref(root,reference)else{errors.push(format!("{path}:SCHEMA_REF_UNRESOLVED:{reference}"));return;};
  validate_json_schema(instance,target,root,path,errors,depth+1);return;
 }
 if let Some(constant)=schema.get("const"){if instance!=constant{errors.push(format!("{path}:SCHEMA_CONST"));}}
 if let Some(allowed)=schema.get("enum").and_then(Value::as_array){
  if !allowed.iter().any(|value|value==instance){errors.push(format!("{path}:SCHEMA_ENUM"));}
 }
 if let Some(expected)=schema.get("type"){
  let valid=if let Some(name)=expected.as_str(){json_type_matches(instance,name)}
   else if let Some(names)=expected.as_array(){names.iter().any(|name|name.as_str().is_some_and(|name|json_type_matches(instance,name)))}
   else{false};
  if !valid{errors.push(format!("{path}:SCHEMA_TYPE"));return;}
 }
 if let Some(object)=instance.as_object(){
  if let Some(required)=schema.get("required").and_then(Value::as_array){
   for field in required.iter().filter_map(Value::as_str){
    if !object.contains_key(field){errors.push(format!("{path}:SCHEMA_REQUIRED:{field}"));}
   }
  }
  let properties=schema.get("properties").and_then(Value::as_object);
  for(key,value)in object{
   if let Some(property_schema)=properties.and_then(|properties|properties.get(key)){
    validate_json_schema(value,property_schema,root,&format!("{path}.{key}"),errors,depth+1);
   }else if schema.get("additionalProperties")==Some(&Value::Bool(false)){
    errors.push(format!("{path}:SCHEMA_ADDITIONAL_PROPERTY:{key}"));
   }else if let Some(additional_schema)=schema.get("additionalProperties").filter(|value|value.is_object()){
    validate_json_schema(value,additional_schema,root,&format!("{path}.{key}"),errors,depth+1);
   }
  }
 }
 if let Some(array)=instance.as_array(){
  if schema.get("minItems").and_then(Value::as_u64).is_some_and(|min|array.len()<min as usize){errors.push(format!("{path}:SCHEMA_MIN_ITEMS"));}
  if schema.get("maxItems").and_then(Value::as_u64).is_some_and(|max|array.len()>max as usize){errors.push(format!("{path}:SCHEMA_MAX_ITEMS"));}
  if schema.get("uniqueItems")==Some(&Value::Bool(true)){
   let mut seen=BTreeSet::new();
   if array.iter().any(|value|!seen.insert(value.to_string())){errors.push(format!("{path}:SCHEMA_UNIQUE_ITEMS"));}
  }
  if let Some(items)=schema.get("items"){
   for(index,value)in array.iter().enumerate(){validate_json_schema(value,items,root,&format!("{path}[{index}]"),errors,depth+1);}
  }
 }
 if let Some(value)=instance.as_str(){
  if schema.get("minLength").and_then(Value::as_u64).is_some_and(|min|value.chars().count()<min as usize){errors.push(format!("{path}:SCHEMA_MIN_LENGTH"));}
  if schema.get("maxLength").and_then(Value::as_u64).is_some_and(|max|value.chars().count()>max as usize){errors.push(format!("{path}:SCHEMA_MAX_LENGTH"));}
  if let Some(pattern)=schema.get("pattern").and_then(Value::as_str){if !dcc_pattern_matches(pattern,value){errors.push(format!("{path}:SCHEMA_PATTERN"));}}
 }
 if let Some(value)=instance.as_f64().filter(|value|value.is_finite()){
  if schema.get("minimum").and_then(Value::as_f64).is_some_and(|minimum|value<minimum){errors.push(format!("{path}:SCHEMA_MINIMUM"));}
  if schema.get("maximum").and_then(Value::as_f64).is_some_and(|maximum|value>maximum){errors.push(format!("{path}:SCHEMA_MAXIMUM"));}
 }
}
fn validate_schema_instance(instance:&Value,schema:&Value,path:&str)->Result<(),String>{
 let mut errors=Vec::new();validate_json_schema(instance,schema,schema,path,&mut errors,0);
 if errors.is_empty(){Ok(())}else{errors.sort();Err(format!("JSON_SCHEMA_INVALID:{}",errors.join("|")))}
}

pub fn classify_record(r:&Value)->&'static str{
 let a=&r["artifact"];let d=&r["document"];let s=&r["signature"];let l=&r["laboratory_scope"];let cr=&r["calibration_result"];let p=&r["passport_binding"];let c=&r["correction"];
 if !non(a,"original_bytes_digest")||text(a,"original_bytes_digest")!=text(a,"received_bytes_digest")||!non(a,"source_receipt_id")||text(a,"retention_state")!="retained_original_bytes"{return "REJECTED_SOURCE_INTEGRITY";}
 if text(d,"well_formed_state")!="well_formed"||text(d,"namespace_uri")!="https://ptb.de/dcc"||text(d,"root_element")!="digitalCalibrationCertificate"||text(d,"schema_version")!="3.3.0"{return "REJECTED_DOCUMENT";}
 if text(d,"xsd_validation_state")=="invalid"{return "REJECTED_DOCUMENT";}
 if num(s,"duplicate_id_count").is_none_or(|n|n>0.0)||matches!(text(d,"id_reference_state"),"duplicate_id"|"invalid_reference"){return "REJECTED_XML_ID_AMBIGUITY";}
 if text(d,"xsd_artifact_state")!="pinned"||!non(d,"xsd_digest")||text(d,"xsd_validation_state")!="valid"||text(d,"id_reference_state")!="valid"||text(d,"parser_profile_state")!="independently_reviewed"{return "MORE_EVIDENCE_REQUIRED";}
 if num(s,"external_reference_count").is_none_or(|n|n>0.0){return "REJECTED_EXTERNAL_REFERENCE";}
 if text(s,"transform_policy_state")!="allowlisted"||text(s,"wrapping_check_state")!="safe"{return "REJECTED_SIGNATURE_COVERAGE";}
 if text(s,"presence_state")=="absent"||num(s,"signature_count").is_none_or(|n|n<1.0){return "MORE_EVIDENCE_REQUIRED";}
 if text(s,"reference_digest_state")=="invalid"||text(s,"signature_value_state")=="invalid"{return "REJECTED_SIGNATURE_INVALID";}
 if text(s,"reference_digest_state")!="valid"||text(s,"signature_value_state")!="valid"{return "MORE_EVIDENCE_REQUIRED";}
 if matches!(text(s,"signer_certificate_state"),"revoked"|"expired"){return "REJECTED_SIGNATURE_INVALID";}
 if text(s,"signer_certificate_state")!="valid"{return "MORE_EVIDENCE_REQUIRED";}
 if text(s,"signer_trust_state")=="untrusted"||text(s,"signer_role_state")!="authorized_calibration_signer"{return "UNTRUSTED_SIGNER";}
 if text(s,"signer_trust_state")!="trusted"{return "MORE_EVIDENCE_REQUIRED";}
 let Some(covered)=strings(s,"covered_object_ids")else{return "REJECTED_SIGNED_OBJECT_COVERAGE";};
 let Some(consumed)=strings(s,"consumed_object_ids")else{return "REJECTED_SIGNED_OBJECT_COVERAGE";};
 if consumed.is_empty()||consumed.iter().any(|x|!covered.contains(x))||!covered.contains(text(cr,"dcc_object_id"))||!consumed.contains(text(cr,"dcc_object_id")){return "REJECTED_SIGNED_OBJECT_COVERAGE";}
 let Some(uris)=s["signed_reference_uris"].as_array()else{return "REJECTED_SIGNED_OBJECT_COVERAGE";};
 let expected_uri=format!("#{}",text(cr,"dcc_object_id"));
 if !uris.iter().any(|x|x.as_str()==Some(expected_uri.as_str())){return "REJECTED_SIGNED_OBJECT_COVERAGE";}
 if text(l,"schedule_state")!="authenticated_current"||!non(l,"schedule_digest")||!non(l,"schedule_id"){return "MORE_EVIDENCE_REQUIRED";}
 if text(l,"scope_state")=="unknown"{return "MORE_EVIDENCE_REQUIRED";}
 if text(l,"scope_state")!="covers_measurand_method_range"{return "NOT_COMPARABLE";}
 if text(l,"laboratory_id")!=text(cr,"laboratory_id"){return "NOT_COMPARABLE";}
 if text(l,"measurand_id")!=text(cr,"measurand_id")||text(l,"method_id")!=text(cr,"method_id")||text(l,"method_revision")!=text(cr,"method_revision")||text(l,"unit")!=text(cr,"unit"){return "NOT_COMPARABLE";}
 let (Some(lmin),Some(lmax),Some(cmin),Some(cmax))=(num(l,"range_min"),num(l,"range_max"),num(cr,"range_min"),num(cr,"range_max"))else{return "MORE_EVIDENCE_REQUIRED";};
 if lmin>cmin||lmax<cmax{return "NOT_COMPARABLE";}
 if matches!(text(c,"supersession_state"),"superseded"){return "EXPIRED_OR_INVALIDATED";}
 if text(c,"supersession_state")!="current"{return "MORE_EVIDENCE_REQUIRED";}
 if text(cr,"result_state")=="out_of_tolerance"{return "EXPIRED_OR_INVALIDATED";}
 if text(cr,"result_state")!="within_tolerance"{return "MORE_EVIDENCE_REQUIRED";}
 if text(cr,"certificate_rights_state")=="revoked"{return "REJECTED_FOR_DECLARED_PURPOSE";}
 if text(cr,"instrument_id")!=text(p,"instrument_id")||text(cr,"channel_id")!=text(p,"channel_id")||text(cr,"measurand_id")!=text(p,"measurand_id")||text(cr,"unit")!=text(p,"unit")||text(cr,"method_id")!=text(p,"method_id")||text(cr,"method_revision")!=text(p,"method_revision")||text(cr,"firmware_revision")!=text(p,"firmware_revision"){return "NOT_COMPARABLE";}
 let (Some(sample_value),Some(cr_min),Some(cr_max))=(num(p,"sample_value"),num(cr,"range_min"),num(cr,"range_max"))else{return "MORE_EVIDENCE_REQUIRED";};
 if sample_value<cr_min||sample_value>cr_max{return "EXPIRED_OR_INVALIDATED";}
 let cal_date=text(cr,"calibration_date");let due_date=text(cr,"due_date");let sample_date=text(p,"sample_date");
 if !valid_date(cal_date)||!valid_date(due_date)||!valid_date(sample_date)||cal_date>due_date{return "NOT_COMPARABLE";}
 if sample_date<cal_date||sample_date>due_date{return "EXPIRED_OR_INVALIDATED";}
 let (Some(unc),Some(factor))=(num(cr,"uncertainty_value"),num(cr,"coverage_factor"))else{return "MORE_EVIDENCE_REQUIRED";};
 if unc<0.0||factor<=0.0||text(cr,"uncertainty_unit")!=text(cr,"unit"){return "MORE_EVIDENCE_REQUIRED";}
 "ELIGIBLE_FOR_INDEPENDENT_PASSPORT_REVIEW_ONLY"
}
fn valid_date(s:&str)->bool{let b=s.as_bytes();if b.len()!=10||b[4]!=b'-'||b[7]!=b'-'||!b.iter().enumerate().all(|(i,x)|i==4||i==7||x.is_ascii_digit()){return false;}let Ok(y)=s[0..4].parse::<u32>()else{return false;};let Ok(m)=s[5..7].parse::<u32>()else{return false;};let Ok(d)=s[8..10].parse::<u32>()else{return false;};let leap=y%4==0&&(y%100!=0||y%400==0);let days=match m{1|3|5|7|8|10|12=>31,4|6|9|11=>30,2 if leap=>29,2=>28,_=>return false};d>0&&d<=days}
fn validate_record(r:&Value)->Result<(),String>{
 if !exact(r,ROOT_FIELDS)||!exact(&r["artifact"],ARTIFACT_FIELDS)||!exact(&r["document"],DOCUMENT_FIELDS)||!exact(&r["signature"],SIGNATURE_FIELDS)||!exact(&r["laboratory_scope"],LAB_FIELDS)||!exact(&r["calibration_result"],RESULT_FIELDS)||!exact(&r["passport_binding"],BINDING_FIELDS)||!exact(&r["correction"],CORRECTION_FIELDS){return Err("DCC_RECORD_FIELDS_INVALID".into());}
 Ok(())
}
fn validate_schema_contract(profile:&Value,ps:&Value,cs:&Value)->Result<(),String>{
 if !exact(profile,&["schema_version","dcc_profile_id","claim_ceiling","official_reference","trust_axes","source_boundary","non_authority"])
  ||text(profile,"schema_version")!="mycelix.building-dcc-evidence/0.1"
  ||text(profile,"dcc_profile_id")!="PTB-DCC-3.3.0-TRUST-APPLICABILITY"
  ||text(profile,"claim_ceiling")!="normalized_synthetic_evidence_axes_only"{return Err("DCC_PROFILE_IDENTITY_INVALID".into());}
 if text(&profile["official_reference"],"namespace_uri")!="https://ptb.de/dcc"
  ||text(&profile["official_reference"],"schema_version")!="3.3.0"
  ||profile["official_reference"]["signature_min_occurs"]!=0
  ||profile["official_reference"]["xsd_digest"]!=Value::Null{return Err("DCC_OFFICIAL_REFERENCE_DRIFT".into());}
 if ps["type"]!="object"||ps["additionalProperties"]!=false
  ||!str_array_eq(&ps["required"],&["schema_version","dcc_profile_id","claim_ceiling","official_reference","trust_axes","source_boundary","non_authority"])
  ||cs["type"]!="object"||cs["additionalProperties"]!=false
  ||!str_array_eq(&cs["required"],&["schema_version","corpus_id","classification","claim_ceiling","cases"])
  ||cs["properties"]["cases"]["minItems"]!=EXPECTED.len()
  ||cs["properties"]["cases"]["maxItems"]!=EXPECTED.len()
  ||cs["properties"]["cases"]["items"]["additionalProperties"]!=false
  ||!str_array_eq(&cs["properties"]["cases"]["items"]["required"],&["case_id","description","record","expected_disposition"])
  ||!str_array_eq(&cs["$defs"]["record"]["required"],ROOT_FIELDS)
  {return Err("DCC_SCHEMAS_INVALID".into());}
 let expected_dispositions:BTreeSet<&str>=EXPECTED.iter().map(|(_,disposition)|*disposition).collect();
 let schema_dispositions=cs["properties"]["cases"]["items"]["properties"]["expected_disposition"]["enum"].as_array().ok_or("DCC_DISPOSITION_ENUM_MISSING")?;
 let schema_disposition_set:BTreeSet<&str>=schema_dispositions.iter().filter_map(Value::as_str).collect();
 if expected_dispositions!=schema_disposition_set{return Err("DCC_DISPOSITION_ENUM_DRIFT".into());}
 Ok(())
}
pub fn validate_tracked()->Result<usize,String>{
 let profile=parse(PROFILE_JSON)?;let ps=parse(PROFILE_SCHEMA_JSON)?;let cs=parse(CORPUS_SCHEMA_JSON)?;validate_schema_contract(&profile,&ps,&cs)?;
 validate_schema_instance(&profile,&ps,"$profile")?;
 let c=parse(CORPUS_JSON)?;
 validate_schema_instance(&c,&cs,"$corpus")?;
 if !exact(&c,&["schema_version","corpus_id","classification","claim_ceiling","cases"])
  ||text(&c,"schema_version")!="mycelix.building-dcc-evidence-corpus/0.1"
  ||text(&c,"corpus_id")!="BUILDING-DCC-EVIDENCE-CORPUS-V0.1"
  ||text(&c,"classification")!="synthetic_dcc_evidence_state_machine_cases"
  ||text(&c,"claim_ceiling")!="normalized_synthetic_dcc_evidence_contract_only"
  ||c["cases"].as_array().is_none_or(|a|a.len()!=EXPECTED.len()){return Err("DCC_CORPUS_IDENTITY_INVALID".into());}
 for (index,(case,(id,expected))) in c["cases"].as_array().unwrap().iter().zip(EXPECTED).enumerate(){
  if !exact(case,&["case_id","description","record","expected_disposition"])
   ||text(case,"case_id")!=*id||text(case,"expected_disposition")!=*expected||text(case,"description").len()<20{return Err(format!("DCC_CASE_ID_OR_EXPECTED_DRIFT:{id}"));}
  validate_record(&case["record"])?;
  let actual=classify_record(&case["record"]);if actual!=*expected{return Err(format!("DCC_DISPOSITION_MISMATCH:{}:expected={expected}:actual={actual}",index+1));}
 }
 Ok(EXPECTED.len())
}
#[cfg(test)]
mod tests{
 use super::*;
 fn case(id:&str)->Value{let c=parse(CORPUS_JSON).unwrap();c["cases"].as_array().unwrap().iter().find(|x|text(x,"case_id")==id).unwrap()["record"].clone()}
 #[test]fn tracked_36_case_contract_matches(){assert_eq!(validate_tracked().unwrap(),36);}
 #[test]fn unsigned_xsd_valid_document_is_not_authenticated(){assert_eq!(classify_record(&case("DCC-002")),"MORE_EVIDENCE_REQUIRED");}
 #[test]fn signed_object_coverage_is_required(){assert_eq!(classify_record(&case("DCC-011")),"REJECTED_SIGNED_OBJECT_COVERAGE");}
 #[test]fn trusted_signature_does_not_replace_lab_scope(){assert_eq!(classify_record(&case("DCC-016")),"MORE_EVIDENCE_REQUIRED");}
 #[test]fn scope_method_or_range_mismatch_is_not_comparable(){assert_eq!(classify_record(&case("DCC-017")),"NOT_COMPARABLE");}
 #[test]fn original_bytes_digest_is_preserved(){assert_eq!(classify_record(&case("DCC-010")),"REJECTED_SOURCE_INTEGRITY");}
 #[test]fn certificate_supersession_invalidates_old_result(){assert_eq!(classify_record(&case("DCC-029")),"EXPIRED_OR_INVALIDATED");}
 #[test]fn signer_trust_is_distinct_from_signature_validity(){assert_eq!(classify_record(&case("DCC-005")),"UNTRUSTED_SIGNER");}
 #[test]fn dcc_schema_signature_minimum_is_zero(){let p=parse(PROFILE_JSON).unwrap();assert_eq!(p["official_reference"]["signature_min_occurs"],0);}
 #[test]fn duplicate_json_members_are_rejected(){assert!(parse(r#"{"a":1,"a":2}"#).is_err());}
 #[test]fn no_unsigned_dcc_can_reach_review_eligibility(){assert_ne!(classify_record(&case("DCC-002")),"ELIGIBLE_FOR_INDEPENDENT_PASSPORT_REVIEW_ONLY");}
 #[test]fn no_schema_valid_document_promotes_a_signer_or_lab_scope_automatically(){assert_ne!(classify_record(&case("DCC-016")),"ELIGIBLE_FOR_INDEPENDENT_PASSPORT_REVIEW_ONLY");}
 #[test]fn duplicate_xml_id_is_explicitly_rejected(){assert_eq!(classify_record(&case("DCC-012")),"REJECTED_XML_ID_AMBIGUITY");}
 #[test]fn accreditation_schedule_lab_must_match_dcc_result_lab(){assert_eq!(classify_record(&case("DCC-019")),"NOT_COMPARABLE");}
 #[test]fn calibration_after_sample_date_invalidates_applicability(){assert_eq!(classify_record(&case("DCC-026")),"EXPIRED_OR_INVALIDATED");}
 #[test]fn schema_validator_rejects_unknown_nested_members(){let p=parse(PROFILE_JSON).unwrap();let ps=parse(PROFILE_SCHEMA_JSON).unwrap();let mut bad=p.clone();bad["official_reference"]["unreviewed_field"]=Value::Bool(true);assert!(validate_schema_instance(&bad,&ps,"$profile").is_err());}
 #[test]fn passport_review_eligibility_is_never_pass_or_admission(){assert_eq!(classify_record(&case("DCC-036")),"ELIGIBLE_FOR_INDEPENDENT_PASSPORT_REVIEW_ONLY");}
}
