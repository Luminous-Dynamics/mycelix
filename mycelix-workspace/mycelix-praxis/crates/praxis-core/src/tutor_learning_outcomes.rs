// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Pre-registered, outcome-first evaluation contract for tutoring systems.
//!
//! This validates study design and endpoint declarations; it does not analyze
//! participant data, establish causal effects, or certify educational efficacy.

use serde::{de::Deserializer, Deserialize, Serialize};
use std::collections::BTreeSet;

const PROTOCOL_ID: &str = "PRAXIS-LEARNING-OUTCOME-PROTOCOL-001";

fn required_option<'de, D, T>(deserializer: D) -> Result<Option<T>, D::Error>
where
    D: Deserializer<'de>,
    T: Deserialize<'de>,
{
    Option::<T>::deserialize(deserializer)
}

fn nonempty(value: &str) -> bool {
    !value.trim().is_empty()
}

fn is_blake3(value: &str) -> bool {
    value.len() == 64
        && value
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}

#[derive(Clone, Copy, Debug, Deserialize, Serialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum ProtocolStatus {
    Draft,
    Preregistered,
    Complete,
}

#[derive(Clone, Copy, Debug, Deserialize, Serialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum TrialDesign {
    IndividualRandomized,
    ClusterRandomized,
    ObservationalPilot,
}

#[derive(Clone, Copy, Debug, Deserialize, Serialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum AssignmentUnit {
    Learner,
    Classroom,
    School,
}

#[derive(Clone, Copy, Debug, Deserialize, Serialize, PartialEq, Eq, PartialOrd, Ord)]
#[serde(rename_all = "snake_case")]
pub enum EndpointKind {
    ImmediateGain,
    DelayedRetention,
    NearTransfer,
    FarTransfer,
    ErrorRecovery,
    Engagement,
    TimeOnTask,
    CostPerGain,
    Displacement,
    AdverseEvents,
}

#[derive(Clone, Copy, Debug, Deserialize, Serialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum EndpointTimepoint {
    Baseline,
    ImmediatePost,
    DuringIntervention,
    Delayed24h,
    Delayed7d,
    Delayed28d,
}

impl EndpointTimepoint {
    fn is_delayed(self) -> bool {
        matches!(self, Self::Delayed24h | Self::Delayed7d | Self::Delayed28d)
    }
}

#[derive(Clone, Debug, Deserialize, Serialize, PartialEq, Eq)]
#[serde(deny_unknown_fields)]
pub struct OutcomeEndpoint {
    pub endpoint_id: String,
    pub kind: EndpointKind,
    pub timepoint: EndpointTimepoint,
    /// Digest of the assessment instrument or measurement procedure.
    pub instrument_blake3: String,
    /// Digest of the independent scoring or calculation procedure.
    pub scoring_blake3: String,
    /// Scale values multiplied by 1,000, avoiding float serialization drift.
    pub scale_min_milli: i32,
    pub scale_max_milli: i32,
    pub independent_assessment: bool,
    pub assessor_blinded: bool,
    /// Must be true for transfer endpoints; "unpracticed" is defined before enrollment.
    pub uses_unpracticed_items: bool,
    /// Minimum elapsed time from intervention to measurement, in hours.
    pub minimum_delay_hours: u32,
}

#[derive(Clone, Copy, Debug, Deserialize, Serialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum Estimator {
    DifferenceInMeans,
    CovariateAdjustedDifference,
    ClusterRobustDifference,
}

#[derive(Clone, Copy, Debug, Deserialize, Serialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum MultiplicityCorrection {
    Holm,
    BenjaminiHochberg,
    HierarchicalGatekeeping,
}

#[derive(Clone, Copy, Debug, Deserialize, Serialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum MissingDataPlan {
    ReportByArmAndSensitivity,
    MultipleImputationWithSensitivity,
}

#[derive(Clone, Debug, Deserialize, Serialize, PartialEq, Eq)]
#[serde(deny_unknown_fields)]
pub struct AnalysisPlan {
    /// Version 1 requires intention-to-treat as the primary estimand for randomized studies.
    pub estimand: String,
    pub estimator: Estimator,
    pub confidence_level_percent: u32,
    /// Alpha multiplied by 1,000; 50 means 0.05.
    pub alpha_milli: u32,
    pub multiplicity_correction: MultiplicityCorrection,
    pub missing_data_plan: MissingDataPlan,
    /// Target power multiplied by 1,000; v1 requires at least 80% before enrollment.
    pub target_power_milli: u32,
    /// Predeclared standardized minimum detectable effect multiplied by 1,000.
    pub minimum_detectable_effect_milli: u32,
    /// Digest of the retained sample-size/power calculation and its assumptions.
    pub sample_size_justification_blake3: String,
}

#[derive(Clone, Debug, Deserialize, Serialize, PartialEq, Eq)]
#[serde(deny_unknown_fields)]
pub struct LearningOutcomeProtocol {
    pub schema_version: u32,
    pub protocol_id: String,
    pub study_id: String,
    pub status: ProtocolStatus,
    /// Required null in a draft; required URI and digest before preregistration/completion.
    #[serde(deserialize_with = "required_option")]
    pub preregistration_uri: Option<String>,
    #[serde(deserialize_with = "required_option")]
    pub preregistration_blake3: Option<String>,
    pub design: TrialDesign,
    pub assignment_unit: AssignmentUnit,
    pub intervention_arm_id: String,
    pub comparator_arm_id: String,
    pub planned_n_intervention: u32,
    pub planned_n_comparator: u32,
    /// Exactly one primary learning endpoint; no opaque aggregate "tutor score".
    pub primary_endpoint: OutcomeEndpoint,
    pub secondary_endpoints: Vec<OutcomeEndpoint>,
    pub analysis_plan: AnalysisPlan,
    /// Subgroups must be named before outcomes are inspected; do not add post-hoc slices here.
    pub predeclared_subgroups: Vec<String>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ProtocolFinding {
    pub code: &'static str,
    pub detail: String,
}

fn finding(code: &'static str, detail: impl Into<String>) -> ProtocolFinding {
    ProtocolFinding {
        code,
        detail: detail.into(),
    }
}

fn validate_endpoint(endpoint: &OutcomeEndpoint, is_primary: bool) -> Vec<ProtocolFinding> {
    let mut findings = Vec::new();
    if !nonempty(&endpoint.endpoint_id) {
        findings.push(finding("empty_endpoint_id", "endpoint_id must not be empty"));
    }
    if !is_blake3(&endpoint.instrument_blake3) {
        findings.push(finding(
            "invalid_instrument_digest",
            format!("{} has an invalid instrument digest", endpoint.endpoint_id),
        ));
    }
    if !is_blake3(&endpoint.scoring_blake3) {
        findings.push(finding(
            "invalid_scoring_digest",
            format!("{} has an invalid scoring digest", endpoint.endpoint_id),
        ));
    }
    if endpoint.instrument_blake3 == endpoint.scoring_blake3 {
        findings.push(finding(
            "instrument_scoring_digest_collision",
            format!("{} must separately pin measurement and scoring", endpoint.endpoint_id),
        ));
    }
    if endpoint.scale_min_milli >= endpoint.scale_max_milli {
        findings.push(finding(
            "invalid_endpoint_scale",
            format!("{} must have scale_min_milli < scale_max_milli", endpoint.endpoint_id),
        ));
    }

    let is_transfer = matches!(endpoint.kind, EndpointKind::NearTransfer | EndpointKind::FarTransfer);
    let is_delayed_learning = matches!(
        endpoint.kind,
        EndpointKind::DelayedRetention | EndpointKind::NearTransfer | EndpointKind::FarTransfer
    );
    if is_delayed_learning && (!endpoint.timepoint.is_delayed() || endpoint.minimum_delay_hours < 24) {
        findings.push(finding(
            "delayed_measurement_not_delayed",
            format!("{} requires a delayed timepoint and at least 24 hours", endpoint.endpoint_id),
        ));
    }
    if is_transfer && !endpoint.uses_unpracticed_items {
        findings.push(finding(
            "transfer_uses_practiced_items",
            format!("{} must use items declared unpracticed before enrollment", endpoint.endpoint_id),
        ));
    }
    if is_primary && (!endpoint.independent_assessment || !endpoint.assessor_blinded) {
        findings.push(finding(
            "primary_assessment_not_independent",
            "the primary learning endpoint requires independent assessment and a blinded assessor",
        ));
    }
    findings
}

/// Parse and validate a versioned study protocol. All findings are fail-closed.
/// This is a protocol validator, not a statistical analysis engine.
pub fn validate_learning_outcome_protocol_json(
    json: &str,
) -> Result<LearningOutcomeProtocol, Vec<ProtocolFinding>> {
    let protocol: LearningOutcomeProtocol = serde_json::from_str(json).map_err(|error| {
        vec![finding("json_parse", format!("learning outcome protocol: {error}"))]
    })?;
    let findings = validate_learning_outcome_protocol(&protocol);
    if findings.is_empty() {
        Ok(protocol)
    } else {
        Err(findings)
    }
}

pub fn validate_learning_outcome_protocol(
    protocol: &LearningOutcomeProtocol,
) -> Vec<ProtocolFinding> {
    let mut findings = Vec::new();

    if protocol.schema_version != 1 || protocol.protocol_id != PROTOCOL_ID {
        findings.push(finding(
            "unsupported_protocol",
            "schema_version and protocol_id must identify outcome protocol v1",
        ));
    }
    if !nonempty(&protocol.study_id) {
        findings.push(finding("empty_study_id", "study_id must not be empty"));
    }
    if !nonempty(&protocol.intervention_arm_id)
        || !nonempty(&protocol.comparator_arm_id)
        || protocol.intervention_arm_id == protocol.comparator_arm_id
    {
        findings.push(finding(
            "invalid_arms",
            "intervention and comparator arm IDs must be nonempty and distinct",
        ));
    }
    if protocol.planned_n_intervention < 2 || protocol.planned_n_comparator < 2 {
        findings.push(finding(
            "planned_sample_too_small",
            "each planned arm must contain at least two units; this is a structural floor, not a power analysis",
        ));
    }

    match protocol.status {
        ProtocolStatus::Draft => {
            if protocol.preregistration_uri.is_some() || protocol.preregistration_blake3.is_some() {
                findings.push(finding(
                    "draft_has_preregistration",
                    "draft protocols must not claim a completed preregistration",
                ));
            }
        }
        ProtocolStatus::Preregistered | ProtocolStatus::Complete => {
            if !protocol.preregistration_uri.as_deref().is_some_and(nonempty)
                || !protocol.preregistration_blake3.as_deref().is_some_and(is_blake3)
            {
                findings.push(finding(
                    "missing_preregistration_receipt",
                    "preregistered/complete status requires a nonempty URI and valid digest",
                ));
            }
        }
    }
    if protocol.preregistration_blake3.as_deref().is_some_and(|digest| !is_blake3(digest)) {
        findings.push(finding("invalid_preregistration_digest", "preregistration digest must be lowercase 64-hex BLAKE3"));
    }

    match protocol.design {
        TrialDesign::IndividualRandomized if protocol.assignment_unit != AssignmentUnit::Learner => {
            findings.push(finding(
                "randomization_unit_mismatch",
                "individual_randomized designs must assign at the learner level",
            ));
        }
        TrialDesign::ClusterRandomized => {
            if protocol.assignment_unit == AssignmentUnit::Learner {
                findings.push(finding(
                    "randomization_unit_mismatch",
                    "cluster_randomized designs must assign classrooms or schools",
                ));
            }
            if protocol.analysis_plan.estimator != Estimator::ClusterRobustDifference {
                findings.push(finding(
                    "cluster_estimator_required",
                    "cluster-randomized designs require a cluster-robust primary estimator",
                ));
            }
        }
        _ => {}
    }
    if protocol.analysis_plan.estimand != "intention_to_treat" {
        findings.push(finding(
            "non_itt_primary_estimand",
            "version 1 requires intention_to_treat as the primary estimand",
        ));
    }
    if !matches!(protocol.analysis_plan.confidence_level_percent, 90 | 95 | 99)
        || protocol.analysis_plan.alpha_milli == 0
        || protocol.analysis_plan.confidence_level_percent * 10 + protocol.analysis_plan.alpha_milli != 1000
    {
        findings.push(finding(
            "inconsistent_uncertainty_plan",
            "confidence level and alpha must be a matching 90/10, 95/5, or 99/1 pair",
        ));
    }

    if !(800..=950).contains(&protocol.analysis_plan.target_power_milli)
        || protocol.analysis_plan.minimum_detectable_effect_milli == 0
        || !is_blake3(&protocol.analysis_plan.sample_size_justification_blake3)
    {
        findings.push(finding(
            "invalid_power_plan",
            "power plan requires 80-95% target power, a positive predeclared MDE, and a valid digest of the calculation",
        ));
    }

    if !matches!(
        protocol.primary_endpoint.kind,
        EndpointKind::DelayedRetention | EndpointKind::NearTransfer | EndpointKind::FarTransfer
    ) {
        findings.push(finding(
            "primary_endpoint_not_delayed_learning",
            "the primary endpoint must be delayed retention or near/far transfer, not a response-quality or engagement score",
        ));
    }
    findings.extend(validate_endpoint(&protocol.primary_endpoint, true));

    let mut endpoint_ids = BTreeSet::new();
    let mut endpoint_kinds = BTreeSet::new();
    endpoint_ids.insert(protocol.primary_endpoint.endpoint_id.as_str());
    endpoint_kinds.insert(protocol.primary_endpoint.kind);
    for endpoint in &protocol.secondary_endpoints {
        findings.extend(validate_endpoint(endpoint, false));
        if !endpoint_ids.insert(endpoint.endpoint_id.as_str()) {
            findings.push(finding(
                "duplicate_endpoint_id",
                format!("endpoint ID {} occurs more than once", endpoint.endpoint_id),
            ));
        }
        endpoint_kinds.insert(endpoint.kind);
    }

    for required_kind in [
        EndpointKind::Engagement,
        EndpointKind::TimeOnTask,
        EndpointKind::CostPerGain,
        EndpointKind::Displacement,
        EndpointKind::AdverseEvents,
    ] {
        if !endpoint_kinds.contains(&required_kind) {
            findings.push(finding(
                "required_outcome_dimension_missing",
                format!("missing required reporting dimension: {required_kind:?}"),
            ));
        }
    }
    let mut subgroups = BTreeSet::new();
    for subgroup in &protocol.predeclared_subgroups {
        if !nonempty(subgroup) || !subgroups.insert(subgroup) {
            findings.push(finding(
                "invalid_predeclared_subgroup",
                "predeclared subgroup IDs must be nonempty and unique",
            ));
        }
    }

    findings
}

#[cfg(test)]
mod tests {
    use super::*;

    fn digest(byte: char) -> String {
        std::iter::repeat(byte).take(64).collect()
    }

    fn endpoint(id: &str, kind: EndpointKind, timepoint: EndpointTimepoint) -> OutcomeEndpoint {
        let index = id.bytes().fold(0_u32, |acc, byte| acc.wrapping_add(u32::from(byte))) % 20;
        let instrument_char = char::from_u32(u32::from(b'a') + index).unwrap();
        let scoring_char = char::from_u32(u32::from(b'a') + ((index + 7) % 26)).unwrap();
        OutcomeEndpoint {
            endpoint_id: id.to_owned(),
            kind,
            timepoint,
            instrument_blake3: digest(instrument_char),
            scoring_blake3: digest(scoring_char),
            scale_min_milli: 0,
            scale_max_milli: 1000,
            independent_assessment: true,
            assessor_blinded: true,
            uses_unpracticed_items: matches!(kind, EndpointKind::NearTransfer | EndpointKind::FarTransfer),
            minimum_delay_hours: if timepoint.is_delayed() { 168 } else { 0 },
        }
    }

    fn valid_protocol() -> LearningOutcomeProtocol {
        LearningOutcomeProtocol {
            schema_version: 1,
            protocol_id: PROTOCOL_ID.to_owned(),
            study_id: "synthetic-protocol-fixture".to_owned(),
            status: ProtocolStatus::Draft,
            preregistration_uri: None,
            preregistration_blake3: None,
            design: TrialDesign::IndividualRandomized,
            assignment_unit: AssignmentUnit::Learner,
            intervention_arm_id: "treatment".to_owned(),
            comparator_arm_id: "usual_practice".to_owned(),
            planned_n_intervention: 120,
            planned_n_comparator: 120,
            primary_endpoint: endpoint("delayed-transfer", EndpointKind::FarTransfer, EndpointTimepoint::Delayed7d),
            secondary_endpoints: vec![
                endpoint("engagement", EndpointKind::Engagement, EndpointTimepoint::DuringIntervention),
                endpoint("time-on-task", EndpointKind::TimeOnTask, EndpointTimepoint::DuringIntervention),
                endpoint("cost-per-gain", EndpointKind::CostPerGain, EndpointTimepoint::DuringIntervention),
                endpoint("displacement", EndpointKind::Displacement, EndpointTimepoint::DuringIntervention),
                endpoint("adverse-events", EndpointKind::AdverseEvents, EndpointTimepoint::DuringIntervention),
                endpoint("delayed-retention", EndpointKind::DelayedRetention, EndpointTimepoint::Delayed7d),
                endpoint("immediate-gain", EndpointKind::ImmediateGain, EndpointTimepoint::ImmediatePost),
                endpoint("error-recovery", EndpointKind::ErrorRecovery, EndpointTimepoint::DuringIntervention),
            ],
            analysis_plan: AnalysisPlan {
                estimand: "intention_to_treat".to_owned(),
                estimator: Estimator::DifferenceInMeans,
                confidence_level_percent: 95,
                alpha_milli: 50,
                multiplicity_correction: MultiplicityCorrection::HierarchicalGatekeeping,
                missing_data_plan: MissingDataPlan::ReportByArmAndSensitivity,
                target_power_milli: 800,
                minimum_detectable_effect_milli: 200,
                sample_size_justification_blake3: digest('9'),
            },
            predeclared_subgroups: vec!["prior_attainment_band".to_owned()],
        }
    }

    #[test]
    fn valid_delayed_transfer_protocol_is_accepted() {
        assert!(validate_learning_outcome_protocol(&valid_protocol()).is_empty());
    }

    #[test]
    fn immediate_gain_cannot_be_primary_endpoint() {
        let mut protocol = valid_protocol();
        protocol.primary_endpoint = endpoint("immediate", EndpointKind::ImmediateGain, EndpointTimepoint::ImmediatePost);
        assert!(validate_learning_outcome_protocol(&protocol).iter().any(|f| f.code == "primary_endpoint_not_delayed_learning"));
    }

    #[test]
    fn transfer_must_use_unpracticed_items() {
        let mut protocol = valid_protocol();
        protocol.primary_endpoint.uses_unpracticed_items = false;
        assert!(validate_learning_outcome_protocol(&protocol).iter().any(|f| f.code == "transfer_uses_practiced_items"));
    }

    #[test]
    fn transfer_must_be_delayed() {
        let mut protocol = valid_protocol();
        protocol.primary_endpoint.minimum_delay_hours = 1;
        assert!(validate_learning_outcome_protocol(&protocol).iter().any(|f| f.code == "delayed_measurement_not_delayed"));
    }

    #[test]
    fn primary_assessment_must_be_independent_and_blinded() {
        let mut protocol = valid_protocol();
        protocol.primary_endpoint.assessor_blinded = false;
        assert!(validate_learning_outcome_protocol(&protocol).iter().any(|f| f.code == "primary_assessment_not_independent"));
    }

    #[test]
    fn duplicate_endpoint_ids_are_rejected() {
        let mut protocol = valid_protocol();
        protocol.secondary_endpoints[0].endpoint_id = protocol.primary_endpoint.endpoint_id.clone();
        assert!(validate_learning_outcome_protocol(&protocol).iter().any(|f| f.code == "duplicate_endpoint_id"));
    }

    #[test]
    fn required_harm_and_operational_dimensions_cannot_be_omitted() {
        let mut protocol = valid_protocol();
        protocol.secondary_endpoints.retain(|endpoint| endpoint.kind != EndpointKind::Displacement);
        assert!(validate_learning_outcome_protocol(&protocol).iter().any(|f| f.code == "required_outcome_dimension_missing"));
    }

    #[test]
    fn preregistered_protocol_requires_receipt() {
        let mut protocol = valid_protocol();
        protocol.status = ProtocolStatus::Preregistered;
        assert!(validate_learning_outcome_protocol(&protocol).iter().any(|f| f.code == "missing_preregistration_receipt"));
    }

    #[test]
    fn invalid_digest_is_rejected() {
        let mut protocol = valid_protocol();
        protocol.primary_endpoint.instrument_blake3 = "not-a-digest".to_owned();
        assert!(validate_learning_outcome_protocol(&protocol).iter().any(|f| f.code == "invalid_instrument_digest"));
    }

    #[test]
    fn instrument_and_scoring_digests_must_be_distinct() {
        let mut protocol = valid_protocol();
        protocol.primary_endpoint.scoring_blake3 = protocol.primary_endpoint.instrument_blake3.clone();
        assert!(validate_learning_outcome_protocol(&protocol).iter().any(|f| f.code == "instrument_scoring_digest_collision"));
    }

    #[test]
    fn cluster_trial_requires_cluster_robust_estimator() {
        let mut protocol = valid_protocol();
        protocol.design = TrialDesign::ClusterRandomized;
        protocol.assignment_unit = AssignmentUnit::School;
        assert!(validate_learning_outcome_protocol(&protocol).iter().any(|f| f.code == "cluster_estimator_required"));
    }

    #[test]
    fn assignment_unit_must_match_randomization_design() {
        let mut protocol = valid_protocol();
        protocol.design = TrialDesign::IndividualRandomized;
        protocol.assignment_unit = AssignmentUnit::Classroom;
        assert!(validate_learning_outcome_protocol(&protocol).iter().any(|f| f.code == "randomization_unit_mismatch"));
    }

    #[test]
    fn invalid_primary_scale_is_rejected() {
        let mut protocol = valid_protocol();
        protocol.primary_endpoint.scale_min_milli = 1000;
        protocol.primary_endpoint.scale_max_milli = 1000;
        assert!(validate_learning_outcome_protocol(&protocol).iter().any(|f| f.code == "invalid_endpoint_scale"));
    }

    #[test]
    fn confidence_and_alpha_must_agree() {
        let mut protocol = valid_protocol();
        protocol.analysis_plan.confidence_level_percent = 95;
        protocol.analysis_plan.alpha_milli = 100;
        assert!(validate_learning_outcome_protocol(&protocol).iter().any(|f| f.code == "inconsistent_uncertainty_plan"));
    }

    #[test]
    fn underpowered_or_unpinned_power_plan_is_rejected() {
        let mut protocol = valid_protocol();
        protocol.analysis_plan.target_power_milli = 700;
        assert!(validate_learning_outcome_protocol(&protocol).iter().any(|f| f.code == "invalid_power_plan"));
    }

    #[test]
    fn malformed_json_fails_closed() {
        assert!(validate_learning_outcome_protocol_json("{").is_err());
    }

    #[test]
    fn draft_must_not_claim_preregistration() {
        let mut protocol = valid_protocol();
        protocol.preregistration_uri = Some("https://example.org/registration".to_owned());
        assert!(validate_learning_outcome_protocol(&protocol).iter().any(|f| f.code == "draft_has_preregistration"));
    }
}
