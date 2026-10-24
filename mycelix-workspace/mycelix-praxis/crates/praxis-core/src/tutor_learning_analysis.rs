// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Deterministic analysis for preregistered, individual-randomized learning outcomes.
//! V1 supports unadjusted difference-in-means only and fails closed on missing
//! primary outcomes, under-planned/small samples, degenerate variance, and unsupported plans.

use super::tutor_learning_outcomes::{
    validate_learning_outcome_protocol, Estimator, LearningOutcomeProtocol, MissingDataPlan,
    ProtocolStatus, TrialDesign,
};
use serde::{de::Deserializer, Deserialize, Serialize};
use std::collections::BTreeSet;

const ANALYSIS_PROTOCOL_ID: &str = "PRAXIS-LEARNING-OUTCOME-ANALYSIS-001";
const STUDY_PROTOCOL_ID: &str = "PRAXIS-LEARNING-OUTCOME-PROTOCOL-001";
const MINIMUM_INFERENTIAL_N_PER_ARM: usize = 30;
const MICRO_PER_VALUE_MILLI: f64 = 1_000_000.0;

fn required_option<'de, D, T>(deserializer: D) -> Result<Option<T>, D::Error>
where D: Deserializer<'de>, T: Deserialize<'de> {
    Option::<T>::deserialize(deserializer)
}
fn nonempty(value: &str) -> bool { !value.trim().is_empty() }
fn is_lower_hex(value: &str, len: usize) -> bool {
    value.len() == len && value.bytes().all(|b| b.is_ascii_digit() || (b'a'..=b'f').contains(&b))
}
fn is_digest(value: &str) -> bool { is_lower_hex(value, 64) }
fn is_revision(value: &str) -> bool { is_lower_hex(value, 40) }
fn exact_digest(bytes: &[u8]) -> String { blake3::hash(bytes).to_hex().to_string() }

#[derive(Clone, Copy, Debug, Deserialize, Serialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum MissingOutcomeReason {
    LearnerWithdrew, AssessmentNotCompleted, AssessmentUnavailable, TechnicalFailure, OtherPredefined,
}

#[derive(Clone, Debug, Deserialize, Serialize, PartialEq, Eq)]
#[serde(deny_unknown_fields)]
pub struct ParticipantOutcomeRow {
    /// Pseudonymous token only; never a name, email, or local identifier.
    pub participant_id: String,
    pub arm_id: String,
    /// Endpoint value multiplied by 1,000; null means missing, not zero.
    #[serde(deserialize_with = "required_option")]
    pub value_milli: Option<i32>,
    /// Required for missing values and forbidden for observed outcomes.
    #[serde(deserialize_with = "required_option")]
    pub missing_reason: Option<MissingOutcomeReason>,
}

#[derive(Clone, Debug, Deserialize, Serialize, PartialEq, Eq)]
#[serde(deny_unknown_fields)]
pub struct ParticipantOutcomeDataset {
    pub schema_version: u32,
    pub protocol_id: String,
    pub dataset_id: String,
    pub study_id: String,
    pub endpoint_id: String,
    /// Exactly one row per randomized participant for the primary endpoint.
    pub rows: Vec<ParticipantOutcomeRow>,
}

#[derive(Clone, Copy, Debug, Deserialize, Serialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum OutcomeAnalysisStatus {
    InferentialIntervalAvailable, MissingPrimaryOutcomes, InsufficientSample, DegenerateVariance,
}
#[derive(Clone, Debug, Deserialize, Serialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct OutcomeArmSummary {
    pub arm_id: String,
    pub n_randomized: u64,
    pub n_observed: u64,
    pub n_missing: u64,
    /// Mean in 1e-6 of the integer value_milli unit.
    pub observed_mean_value_milli_micros: Option<i64>,
    pub observed_sample_sd_value_milli_micros: Option<i64>,
}
#[derive(Clone, Debug, Deserialize, Serialize, PartialEq, Eq)]
#[serde(deny_unknown_fields)]
pub struct DifferenceConfidenceInterval {
    pub confidence_level_percent: u32,
    /// Endpoints in value_milli_micros; rounded outward.
    pub lower_value_milli_micros: i64,
    pub upper_value_milli_micros: i64,
    pub method: String,
}
#[derive(Clone, Debug, Deserialize, Serialize, PartialEq, Eq)]
#[serde(deny_unknown_fields)]
pub struct MissingOutcomeBounds {
    /// Worst-case bounded-outcome sensitivity interval, not a confidence interval.
    pub lower_value_milli_micros: i64,
    pub upper_value_milli_micros: i64,
    pub method: String,
}
#[derive(Clone, Debug, Deserialize, Serialize, PartialEq, Eq)]
#[serde(deny_unknown_fields)]
pub struct AnalysisFinding { pub code: String, pub detail: String }

#[derive(Clone, Debug, Deserialize, Serialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct LearningOutcomeAnalysisResult {
    pub schema_version: u32,
    pub analysis_protocol_id: String,
    pub study_id: String,
    pub dataset_id: String,
    pub endpoint_id: String,
    /// BLAKE3 digests bind exact input bytes, not canonicalized JSON.
    pub input_protocol_blake3: String,
    pub input_dataset_blake3: String,
    /// Immutable 40-lowercase-hex Git SHA of the analysis implementation.
    pub analysis_revision: String,
    pub endpoint_kind: String,
    pub estimator: String,
    pub estimand: String,
    pub status: OutcomeAnalysisStatus,
    pub confidence_level_percent: u32,
    pub intervention: OutcomeArmSummary,
    pub comparator: OutcomeArmSummary,
    /// Descriptive treatment-minus-comparator difference; withheld if any primary outcome is missing.
    pub observed_case_difference_value_milli_micros: Option<i64>,
    pub confidence_interval: Option<DifferenceConfidenceInterval>,
    pub missing_outcome_bounds: Option<MissingOutcomeBounds>,
    pub findings: Vec<AnalysisFinding>,
}

fn finding(code: &str, detail: impl Into<String>) -> AnalysisFinding {
    AnalysisFinding { code: code.to_owned(), detail: detail.into() }
}
fn floor_div(n: i128, d: i128) -> i128 {
    let q = n / d; let r = n % d;
    if r != 0 && n < 0 { q - 1 } else { q }
}
fn ceil_div(n: i128, d: i128) -> i128 {
    let q = n / d; let r = n % d;
    if r != 0 && n > 0 { q + 1 } else { q }
}
fn micro_mean(sum: i128, n: i128, round_up: bool) -> i64 {
    let scaled = sum * 1_000_000;
    (if round_up { ceil_div(scaled, n) } else { floor_div(scaled, n) }) as i64
}
fn rounded_micro(value: f64) -> i64 { (value * MICRO_PER_VALUE_MILLI).round() as i64 }
fn lower_micro(value: f64) -> i64 { (value * MICRO_PER_VALUE_MILLI).floor() as i64 }
fn upper_micro(value: f64) -> i64 { (value * MICRO_PER_VALUE_MILLI).ceil() as i64 }

#[derive(Debug)]
struct ValidatedArm {
    id: String, n_randomized: usize, values: Vec<i32>, missing_count: usize, sum_observed: i128,
}
fn validate_dataset(
    protocol: &LearningOutcomeProtocol,
    dataset: &ParticipantOutcomeDataset,
) -> Result<(ValidatedArm, ValidatedArm), Vec<AnalysisFinding>> {
    let mut findings = Vec::new();
    if dataset.schema_version != 1 { findings.push(finding("unsupported_dataset_schema", "dataset schema_version must be 1")); }
    if dataset.protocol_id != "PRAXIS-LEARNING-OUTCOME-PROTOCOL-001" {
        findings.push(finding("dataset_protocol_mismatch", "dataset protocol_id does not identify the study protocol"));
    }
    if !nonempty(&dataset.dataset_id) || !nonempty(&dataset.study_id) {
        findings.push(finding("empty_dataset_identity", "dataset_id and study_id must be nonempty"));
    }
    if dataset.study_id != protocol.study_id { findings.push(finding("study_id_mismatch", "dataset study_id differs from the registered protocol")); }
    if dataset.endpoint_id != protocol.primary_endpoint.endpoint_id {
        findings.push(finding("endpoint_id_mismatch", "dataset endpoint_id must match the prespecified primary endpoint"));
    }
    if dataset.rows.is_empty() { findings.push(finding("empty_outcome_dataset", "dataset must include randomized participants")); }
    let mut ids = BTreeSet::new();
    let mut t = ValidatedArm { id: protocol.intervention_arm_id.clone(), n_randomized: 0, values: Vec::new(), missing_count: 0, sum_observed: 0 };
    let mut c = ValidatedArm { id: protocol.comparator_arm_id.clone(), n_randomized: 0, values: Vec::new(), missing_count: 0, sum_observed: 0 };
    for row in &dataset.rows {
        if !nonempty(&row.participant_id) {
            findings.push(finding("empty_participant_id", "participant_id must be a nonempty pseudonymous token")); continue;
        }
        if !ids.insert(row.participant_id.as_str()) {
            findings.push(finding("duplicate_participant", format!("participant {} appears more than once", row.participant_id))); continue;
        }
        let arm = if row.arm_id == protocol.intervention_arm_id { &mut t }
            else if row.arm_id == protocol.comparator_arm_id { &mut c }
            else {
                findings.push(finding("unknown_arm", format!("participant {} has an arm outside the registered design", row.participant_id))); continue;
            };
        arm.n_randomized += 1;
        match (row.value_milli, row.missing_reason) {
            (Some(value), None) => {
                if value < protocol.primary_endpoint.scale_min_milli || value > protocol.primary_endpoint.scale_max_milli {
                    findings.push(finding("outcome_out_of_range", format!("participant {} outcome lies outside the registered scale", row.participant_id))); continue;
                }
                arm.values.push(value); arm.sum_observed += i128::from(value);
            }
            (None, Some(_)) => arm.missing_count += 1,
            (None, None) => findings.push(finding("missing_reason_required", format!("participant {} has a missing outcome without a reason", row.participant_id))),
            (Some(_), Some(_)) => findings.push(finding("missing_reason_on_observed_outcome", format!("participant {} has both value and missing reason", row.participant_id))),
        }
    }
    if t.n_randomized < 2 || c.n_randomized < 2 { findings.push(finding("arm_too_small", "both randomized arms must contain at least two participants")); }
    if findings.is_empty() { t.values.sort_unstable(); c.values.sort_unstable(); Ok((t, c)) } else { Err(findings) }
}
fn summarize(arm: &ValidatedArm) -> OutcomeArmSummary {
    let n = arm.values.len();
    let mean = if n == 0 { None } else { Some(micro_mean(arm.sum_observed, n as i128, false)) };
    let sd = if n < 2 { None } else {
        let m = arm.sum_observed as f64 / n as f64;
        let ss = arm.values.iter().map(|v| (f64::from(*v)-m).powi(2)).sum::<f64>();
        Some(rounded_micro((ss/(n-1) as f64).sqrt()))
    };
    OutcomeArmSummary { arm_id: arm.id.clone(), n_randomized: arm.n_randomized as u64,
        n_observed: n as u64, n_missing: arm.missing_count as u64,
        observed_mean_value_milli_micros: mean, observed_sample_sd_value_milli_micros: sd }
}
fn missing_bounds(p: &LearningOutcomeProtocol, t: &ValidatedArm, c: &ValidatedArm) -> MissingOutcomeBounds {
    let min = i128::from(p.primary_endpoint.scale_min_milli); let max = i128::from(p.primary_endpoint.scale_max_milli);
    let tn=t.n_randomized as i128; let cn=c.n_randomized as i128;
    let tm=t.missing_count as i128; let cm=c.missing_count as i128;
    let tl=i128::from(micro_mean(t.sum_observed+tm*min,tn,false));
    let th=i128::from(micro_mean(t.sum_observed+tm*max,tn,true));
    let cl=i128::from(micro_mean(c.sum_observed+cm*min,cn,false));
    let ch=i128::from(micro_mean(c.sum_observed+cm*max,cn,true));
    MissingOutcomeBounds { lower_value_milli_micros:(tl-ch) as i64, upper_value_milli_micros:(th-cl) as i64,
        method:"worst_case_bounded_outcome_bounds".to_owned() }
}
// Cornish-Fisher Student-t approximation; invoked only when n >= 30 in each arm.
fn t_critical(confidence: u32, df: f64) -> f64 {
    let z = match confidence { 90=>1.644_853_626_951_472_2, 95=>1.959_963_984_540_054, 99=>2.575_829_303_548_900_4, _=>unreachable!() };
    let z2=z*z; let z3=z2*z; let z5=z3*z2; let z7=z5*z2;
    let inv=1.0/df; let inv2=inv*inv; let inv3=inv2*inv;
    z+(z3+z)*inv/4.0+(5.0*z5+16.0*z3+3.0*z)*inv2/96.0+(3.0*z7+19.0*z5+17.0*z3-15.0*z)*inv3/384.0
}
fn welch_interval(p: &LearningOutcomeProtocol, t: &ValidatedArm, c: &ValidatedArm) -> Option<(i64,i64)> {
    let nt=t.values.len(); let nc=c.values.len();
    if nt<MINIMUM_INFERENTIAL_N_PER_ARM || nc<MINIMUM_INFERENTIAL_N_PER_ARM
        || t.n_randomized<p.planned_n_intervention as usize || c.n_randomized<p.planned_n_comparator as usize { return None; }
    let mt=t.sum_observed as f64/nt as f64; let mc=c.sum_observed as f64/nc as f64;
    let vt=t.values.iter().map(|v|(f64::from(*v)-mt).powi(2)).sum::<f64>()/(nt-1) as f64;
    let vc=c.values.iter().map(|v|(f64::from(*v)-mc).powi(2)).sum::<f64>()/(nc-1) as f64;
    let a=vt/nt as f64; let b=vc/nc as f64; let se2=a+b;
    if !se2.is_finite() || se2<=0.0 { return None; }
    let denom=a.powi(2)/(nt-1) as f64+b.powi(2)/(nc-1) as f64;
    if !denom.is_finite() || denom<=0.0 { return None; }
    let df=se2.powi(2)/denom;
    if !df.is_finite() || df<20.0 { return None; }
    let delta=mt-mc; let margin=t_critical(p.analysis_plan.confidence_level_percent,df)*se2.sqrt();
    Some((lower_micro(delta-margin),upper_micro(delta+margin)))
}
fn endpoint_name(p: &LearningOutcomeProtocol) -> &'static str {
    use super::tutor_learning_outcomes::EndpointKind::*;
    match p.primary_endpoint.kind {
        ImmediateGain=>"immediate_gain", DelayedRetention=>"delayed_retention", NearTransfer=>"near_transfer",
        FarTransfer=>"far_transfer", ErrorRecovery=>"error_recovery", Engagement=>"engagement",
        TimeOnTask=>"time_on_task", CostPerGain=>"cost_per_gain", Displacement=>"displacement", AdverseEvents=>"adverse_events",
    }
}

/// Analyze a preregistered individual-randomized primary endpoint. Exact input
/// bytes are hashed; this does not authenticate registration or assignments.
pub fn analyze_learning_outcomes_json(
    protocol_json: &str, dataset_json: &str, analysis_revision: &str,
) -> Result<LearningOutcomeAnalysisResult, Vec<AnalysisFinding>> {
    if !is_revision(analysis_revision) { return Err(vec![finding("invalid_analysis_revision", "analysis_revision must be a 40-character lowercase hex SHA")]); }
    let p: LearningOutcomeProtocol = serde_json::from_str(protocol_json).map_err(|e|vec![finding("protocol_parse_error",e.to_string())])?;
    let invalid=validate_learning_outcome_protocol(&p);
    if !invalid.is_empty() { return Err(invalid.into_iter().map(|f|finding(&f.code,f.detail)).collect()); }
    if p.status==ProtocolStatus::Draft { return Err(vec![finding("protocol_not_preregistered","inferential analysis requires preregistered or complete protocol status")]); }
    if p.design!=TrialDesign::IndividualRandomized || p.analysis_plan.estimator!=Estimator::DifferenceInMeans {
        return Err(vec![finding("unsupported_analysis_design","v1 supports individual-randomized designs and difference_in_means only")]);
    }
    if p.analysis_plan.missing_data_plan!=MissingDataPlan::ReportByArmAndSensitivity {
        return Err(vec![finding("unsupported_missing_data_plan","v1 supports report_by_arm_and_sensitivity; multiple imputation is not implemented")]);
    }
    if !is_digest(&p.analysis_plan.sample_size_justification_blake3) { return Err(vec![finding("missing_power_artifact_digest","sample-size justification digest has invalid shape")]); }
    let d: ParticipantOutcomeDataset=serde_json::from_str(dataset_json).map_err(|e|vec![finding("dataset_parse_error",e.to_string())])?;
    let (t,c)=validate_dataset(&p,&d)?;
    let ts=summarize(&t); let cs=summarize(&c); let any_missing=ts.n_missing>0||cs.n_missing>0;
    let mut findings=Vec::new();
    let status; let difference; let interval; let bounds;
    if any_missing {
        status=OutcomeAnalysisStatus::MissingPrimaryOutcomes; difference=None; interval=None;
        bounds=Some(missing_bounds(&p,&t,&c));
        findings.push(finding("inferential_effect_suppressed_for_missing_outcomes","effect and confidence interval withheld; arm counts and worst-case bounded-outcome sensitivity bounds are reported"));
    } else {
        bounds=None;
        let enough=t.values.len()>=MINIMUM_INFERENTIAL_N_PER_ARM && c.values.len()>=MINIMUM_INFERENTIAL_N_PER_ARM
            && t.n_randomized>=p.planned_n_intervention as usize && c.n_randomized>=p.planned_n_comparator as usize;
        let mt=t.sum_observed as f64/t.values.len() as f64; let mc=c.sum_observed as f64/c.values.len() as f64;
        difference=Some(rounded_micro(mt-mc));
        if !enough {
            status=OutcomeAnalysisStatus::InsufficientSample; interval=None;
            findings.push(finding("inferential_interval_suppressed_for_sample_size","descriptive difference only: require >=30 observed per arm and registered randomized sample targets"));
        } else if ts.observed_sample_sd_value_milli_micros==Some(0)&&cs.observed_sample_sd_value_milli_micros==Some(0) {
            status=OutcomeAnalysisStatus::DegenerateVariance; interval=None;
            findings.push(finding("inferential_interval_suppressed_for_degenerate_variance","both arm variances are zero"));
        } else if let Some((lo,hi))=welch_interval(&p,&t,&c) {
            status=OutcomeAnalysisStatus::InferentialIntervalAvailable;
            interval=Some(DifferenceConfidenceInterval { confidence_level_percent:p.analysis_plan.confidence_level_percent,
                lower_value_milli_micros:lo, upper_value_milli_micros:hi,
                method:"welch_t_interval_cornish_fisher_critical_approximation".to_owned() });
        } else {
            status=OutcomeAnalysisStatus::DegenerateVariance; interval=None;
            findings.push(finding("inferential_interval_unavailable","Welch standard error or degrees of freedom not finite and positive"));
        }
    }
    Ok(LearningOutcomeAnalysisResult {
        schema_version:1, analysis_protocol_id:ANALYSIS_PROTOCOL_ID.to_owned(), study_id:d.study_id,
        dataset_id:d.dataset_id, endpoint_id:d.endpoint_id, input_protocol_blake3:exact_digest(protocol_json.as_bytes()),
        input_dataset_blake3:exact_digest(dataset_json.as_bytes()), analysis_revision:analysis_revision.to_owned(),
        endpoint_kind:endpoint_name(&p).to_owned(), estimator:"difference_in_means".to_owned(), estimand:"intention_to_treat".to_owned(),
        status, confidence_level_percent:p.analysis_plan.confidence_level_percent, intervention:ts, comparator:cs,
        observed_case_difference_value_milli_micros:difference, confidence_interval:interval, missing_outcome_bounds:bounds, findings,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::{json,Value};
    const REVISION:&str="aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";
    fn registered_protocol()->String {
        let mut p:Value=serde_json::from_str(include_str!("../../../examples/tutor-history/outcomes-v1/study_protocol.synthetic.json")).unwrap();
        p["status"]=json!("preregistered");
        p["preregistration_uri"]=json!("https://registry.example.test/synthetic-only");
        p["preregistration_blake3"]=json!("bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb");
        serde_json::to_string(&p).unwrap()
    }
    fn rows(n:usize,missing:bool)->Vec<Value> {
        let mut v=Vec::new();
        for i in 0..n {
            v.push(json!({"participant_id":format!("t-{i:04}"),"arm_id":"treatment",
                "value_milli":if missing&&i==0 {Value::Null}else{json!(600+(i*37%301))},
                "missing_reason":if missing&&i==0 {json!("assessment_not_completed")}else{Value::Null}}));
            v.push(json!({"participant_id":format!("c-{i:04}"),"arm_id":"usual_practice",
                "value_milli":json!(400+(i*37%301)),"missing_reason":Value::Null}));
        }
        v
    }
    fn dataset(n:usize,missing:bool)->String {
        serde_json::to_string(&json!({"schema_version":1,"protocol_id":STUDY_PROTOCOL_ID,"dataset_id":"synthetic-test-only",
            "study_id":"synthetic-protocol-fixture","endpoint_id":"far-transfer-7d","rows":rows(n,missing)})).unwrap()
    }
    #[test] fn registered_complete_data_returns_deterministic_interval() {
        let p=registered_protocol();let d=dataset(120,false);
        let a=analyze_learning_outcomes_json(&p,&d,REVISION).unwrap();let b=analyze_learning_outcomes_json(&p,&d,REVISION).unwrap();
        assert_eq!(a,b);assert_eq!(a.status,OutcomeAnalysisStatus::InferentialIntervalAvailable);
        assert!(a.confidence_interval.is_some());assert_eq!(a.input_dataset_blake3,exact_digest(d.as_bytes()));
    }
    #[test] fn row_permutation_preserves_result_but_changes_input_digest() {
        let p=registered_protocol();let d=dataset(120,false);let a=analyze_learning_outcomes_json(&p,&d,REVISION).unwrap();
        let mut v:Value=serde_json::from_str(&d).unwrap();v["rows"].as_array_mut().unwrap().reverse();
        let q=serde_json::to_string(&v).unwrap();let b=analyze_learning_outcomes_json(&p,&q,REVISION).unwrap();
        assert_eq!(a.observed_case_difference_value_milli_micros,b.observed_case_difference_value_milli_micros);
        assert_eq!(a.confidence_interval,b.confidence_interval);assert_ne!(a.input_dataset_blake3,b.input_dataset_blake3);
    }
    #[test] fn missing_primary_outcome_suppresses_effect_and_reports_bounds() {
        let r=analyze_learning_outcomes_json(&registered_protocol(),&dataset(120,true),REVISION).unwrap();
        assert_eq!(r.status,OutcomeAnalysisStatus::MissingPrimaryOutcomes);assert!(r.observed_case_difference_value_milli_micros.is_none());
        assert!(r.confidence_interval.is_none());assert!(r.missing_outcome_bounds.is_some());
    }
    #[test] fn draft_protocol_cannot_be_used_for_inferential_analysis() {
        let p=include_str!("../../../examples/tutor-history/outcomes-v1/study_protocol.synthetic.json");
        let r=analyze_learning_outcomes_json(p,&dataset(120,false),REVISION).unwrap_err();
        assert!(r.iter().any(|f|f.code=="protocol_not_preregistered"));
    }
    #[test] fn duplicate_participant_is_rejected() {
        let p=registered_protocol();let mut d:Value=serde_json::from_str(&dataset(120,false)).unwrap();
        let duplicate=d["rows"][0]["participant_id"].clone();d["rows"][1]["participant_id"]=duplicate;
        let r=analyze_learning_outcomes_json(&p,&serde_json::to_string(&d).unwrap(),REVISION).unwrap_err();
        assert!(r.iter().any(|f|f.code=="duplicate_participant"));
    }
    #[test] fn out_of_range_outcome_is_rejected() {
        let p=registered_protocol();let mut d:Value=serde_json::from_str(&dataset(120,false)).unwrap();d["rows"][0]["value_milli"]=json!(1001);
        let r=analyze_learning_outcomes_json(&p,&serde_json::to_string(&d).unwrap(),REVISION).unwrap_err();
        assert!(r.iter().any(|f|f.code=="outcome_out_of_range"));
    }
    #[test] fn unsupported_multiple_imputation_fails_closed() {
        let mut p:Value=serde_json::from_str(&registered_protocol()).unwrap();p["analysis_plan"]["missing_data_plan"]=json!("multiple_imputation_with_sensitivity");
        let r=analyze_learning_outcomes_json(&serde_json::to_string(&p).unwrap(),&dataset(120,false),REVISION).unwrap_err();
        assert!(r.iter().any(|f|f.code=="unsupported_missing_data_plan"));
    }
    #[test] fn missing_reason_is_required_when_value_is_null() {
        let p=registered_protocol();let mut d:Value=serde_json::from_str(&dataset(120,false)).unwrap();
        d["rows"][0]["value_milli"]=Value::Null;
        let r=analyze_learning_outcomes_json(&p,&serde_json::to_string(&d).unwrap(),REVISION).unwrap_err();
        assert!(r.iter().any(|f|f.code=="missing_reason_required"));
    }
    #[test] fn missing_reason_is_forbidden_when_value_is_present() {
        let p=registered_protocol();let mut d:Value=serde_json::from_str(&dataset(120,false)).unwrap();
        d["rows"][0]["missing_reason"]=json!("assessment_not_completed");
        let r=analyze_learning_outcomes_json(&p,&serde_json::to_string(&d).unwrap(),REVISION).unwrap_err();
        assert!(r.iter().any(|f|f.code=="missing_reason_on_observed_outcome"));
    }
    #[test] fn wrong_endpoint_is_rejected() {
        let p=registered_protocol();let mut d:Value=serde_json::from_str(&dataset(120,false)).unwrap();
        d["endpoint_id"]=json!("posthoc-endpoint");
        let r=analyze_learning_outcomes_json(&p,&serde_json::to_string(&d).unwrap(),REVISION).unwrap_err();
        assert!(r.iter().any(|f|f.code=="endpoint_id_mismatch"));
    }
    #[test] fn below_planned_sample_size_withholds_interval() {
        let r=analyze_learning_outcomes_json(&registered_protocol(),&dataset(40,false),REVISION).unwrap();
        assert_eq!(r.status,OutcomeAnalysisStatus::InsufficientSample);
        assert!(r.observed_case_difference_value_milli_micros.is_some());
        assert!(r.confidence_interval.is_none());
    }
    #[test] fn malformed_dataset_json_is_rejected() {
        let r=analyze_learning_outcomes_json(&registered_protocol(),"{",REVISION).unwrap_err();
        assert!(r.iter().any(|f|f.code=="dataset_parse_error"));
    }
    #[test] fn analysis_matrix_ids_and_test_links_are_unique() {
        let matrix:Value=serde_json::from_str(include_str!("../../../examples/tutor-history/outcomes-v1/analysis_mutation_matrix.json")).unwrap();
        let controls=matrix["controls"].as_array().unwrap();
        let source=include_str!("tutor_learning_analysis.rs");
        let mut ids=BTreeSet::new();let mut tests=BTreeSet::new();
        for control in controls {
            let id=control["control_id"].as_str().unwrap();
            let name=control["rust_test"].as_str().unwrap();
            assert!(ids.insert(id),"duplicate mutation ID {id}");
            assert!(tests.insert(name),"duplicate test link {name}");
            assert!(source.contains(&format!("fn {name}(")),"missing Rust test {name}");
        }
        assert_eq!(controls.len(),13);
    }
    #[test] fn unknown_arm_is_rejected() {
        let p=registered_protocol();let mut d:Value=serde_json::from_str(&dataset(120,false)).unwrap();d["rows"][0]["arm_id"]=json!("post_hoc_arm");
        let r=analyze_learning_outcomes_json(&p,&serde_json::to_string(&d).unwrap(),REVISION).unwrap_err();
        assert!(r.iter().any(|f|f.code=="unknown_arm"));
    }
}
