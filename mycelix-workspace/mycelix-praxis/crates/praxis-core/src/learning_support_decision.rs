// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Synthetic-first learning-support decision benchmark contracts.
//!
//! This module validates benchmark fixtures and their internal policy constraints.
//! It does not select support for a real learner, measure learning efficacy, or
//! grant mastery, credential, trust, or authorization authority.

use serde::{de::Deserializer, Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};

pub const SUPPORT_DECISION_BENCHMARK_ID: &str = "PRAXIS-LSDB-001";
pub const SUPPORT_DECISION_SCHEMA_VERSION: u32 = 1;
pub const SUPPORT_DECISION_DATASET_KIND: &str = "synthetic_policy_probes";

/// Strict top-level benchmark manifest.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SupportDecisionManifest {
    pub schema_version: u32,
    pub benchmark_id: String,
    pub dataset_kind: String,
    pub non_empirical: bool,
    pub cases: Vec<SupportDecisionCase>,
}

/// One synthetic decision probe.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SupportDecisionCase {
    pub case_id: String,
    pub counterfactual_group: String,
    /// Required on the wire; explicit null is allowed only for unpaired cases.
    #[serde(deserialize_with = "deserialize_required_option")]
    pub counterfactual_peer: Option<String>,
    pub counterfactual_relation: CounterfactualRelation,
    pub counterfactual_dimension: CounterfactualDimension,
    pub prompt: String,
    pub context: SupportDecisionContext,
    pub acceptable_actions: Vec<SupportAction>,
    pub forbidden_actions: Vec<SupportAction>,
    pub critical_invariants: Vec<CriticalInvariant>,
    pub rationale: String,
}

/// Decision-relevant context available to the policy under evaluation.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SupportDecisionContext {
    pub learning_goal: String,
    pub request: String,
    pub evidence_sufficiency: EvidenceSufficiency,
    pub evidence_summary: String,
    pub displacement_risk: DisplacementRisk,
    pub accessibility_requirements: Vec<String>,
    pub assessment_policy: AssessmentPolicy,
    pub constraints: Vec<String>,
    pub support_options: Vec<SupportOption>,
}

/// A candidate support mode and an explicit availability observation.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SupportOption {
    pub action: SupportAction,
    pub available: bool,
    pub basis: String,
}

/// Actions that a tested support-selection policy may return.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SupportAction {
    AiHintThenRetrieval,
    RetrievalPractice,
    HumanSupport,
    PeerCollaboration,
    EducatorCopilot,
    AccessibilityAdaptation,
    OfflineFallback,
    AskClarifyingQuestion,
    AbstainAndEscalate,
    NoIntervention,
}

impl SupportAction {
    /// Protocol actions do not require an external support resource to exist.
    fn is_protocol_action(self) -> bool {
        matches!(
            self,
            Self::AskClarifyingQuestion | Self::AbstainAndEscalate | Self::NoIntervention
        )
    }

    fn is_ai_action(self) -> bool {
        matches!(self, Self::AiHintThenRetrieval | Self::EducatorCopilot)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum EvidenceSufficiency {
    Sufficient,
    Partial,
    Insufficient,
    Conflicting,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DisplacementRisk {
    None,
    Low,
    Material,
    Unknown,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AssessmentPolicy {
    NotApplicable,
    PermitsScaffolding,
    ProhibitsAiAssistance,
    Unknown,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CounterfactualRelation {
    Change,
    Invariant,
    None,
}

/// The fact being varied in a paired decision probe.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CounterfactualDimension {
    DisplacementRisk,
    SupportAvailability,
    AccessibilityAvailability,
    EvidenceSufficiency,
    PeerAvailability,
    AssessmentPolicy,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CriticalInvariant {
    RespectActualAvailability,
    DoNotDisplaceHumanSupport,
    PreserveAccessibility,
    PreserveIndependentAttempt,
    DoNotInferMasteryFromAssistedAccuracy,
    ClarifyBeforeHighImpactDecision,
    RespectAssessmentPolicy,
    MinimizeSensitiveData,
    StateUncertainty,
    UseAchievableFallback,
}

/// Stable, machine-readable validation finding.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct SupportDecisionIssue {
    pub code: String,
    pub case_id: Option<String>,
    pub message: String,
}

impl SupportDecisionManifest {
    /// Validate schema-level and cross-field policy invariants without a JSON
    /// Schema runtime dependency. This is not a replacement for meta-validating
    /// the schema against the full JSON Schema 2020-12 specification.
    pub fn validate(&self) -> Vec<SupportDecisionIssue> {
        let mut issues = Vec::new();

        if self.schema_version != SUPPORT_DECISION_SCHEMA_VERSION {
            issue(&mut issues, None, "unsupported_schema_version", "schema_version must be 1");
        }
        if self.benchmark_id != SUPPORT_DECISION_BENCHMARK_ID {
            issue(&mut issues, None, "unknown_benchmark_id", "benchmark_id does not identify PRAXIS-LSDB-001");
        }
        if self.dataset_kind != SUPPORT_DECISION_DATASET_KIND {
            issue(&mut issues, None, "unknown_dataset_kind", "dataset_kind must be synthetic_policy_probes");
        }
        if !self.non_empirical {
            issue(&mut issues, None, "empirical_claim_for_synthetic_fixture", "v1 fixture must be explicitly marked non_empirical");
        }
        if self.cases.len() < 12 {
            issue(&mut issues, None, "insufficient_case_count", "v1 requires at least 12 cases");
        }

        let mut by_id: BTreeMap<&str, &SupportDecisionCase> = BTreeMap::new();
        for case in &self.cases {
            if !valid_case_id(&case.case_id) {
                issue(&mut issues, Some(&case.case_id), "invalid_case_id", "case_id must match LSDB-NNN");
            }
            if by_id.insert(&case.case_id, case).is_some() {
                issue(&mut issues, Some(&case.case_id), "duplicate_case_id", "case_id must be unique");
            }
            self.validate_case(case, &mut issues);
        }

        for case in &self.cases {
            self.validate_counterfactual(case, &by_id, &mut issues);
        }

        issues
    }

    fn validate_case(&self, case: &SupportDecisionCase, issues: &mut Vec<SupportDecisionIssue>) {
        let id = case.case_id.as_str();

        if case.counterfactual_group.trim().is_empty()
            || case.prompt.trim().is_empty()
            || case.rationale.trim().is_empty()
            || case.context.learning_goal.trim().is_empty()
            || case.context.request.trim().is_empty()
            || case.context.evidence_summary.trim().is_empty()
        {
            issue(issues, Some(id), "empty_required_text", "required descriptive text must not be blank");
        }
        if case.context.support_options.is_empty() {
            issue(issues, Some(id), "missing_support_options", "at least one support option must be described");
        }
        if case.acceptable_actions.is_empty() {
            issue(issues, Some(id), "missing_acceptable_action", "at least one acceptable action must be declared");
        }
        if case.critical_invariants.is_empty() {
            issue(issues, Some(id), "missing_critical_invariant", "at least one critical invariant must be declared");
        }

        if has_duplicates(&case.acceptable_actions)
            || has_duplicates(&case.forbidden_actions)
            || has_duplicates(&case.critical_invariants)
        {
            issue(issues, Some(id), "duplicate_decision_label", "action and invariant arrays must not contain duplicate labels");
        }

        let acceptable: BTreeSet<_> = case.acceptable_actions.iter().copied().collect();
        let forbidden: BTreeSet<_> = case.forbidden_actions.iter().copied().collect();
        if !acceptable.is_disjoint(&forbidden) {
            issue(issues, Some(id), "acceptable_forbidden_overlap", "an action cannot be both acceptable and forbidden");
        }

        let mut available = BTreeMap::new();
        for option in &case.context.support_options {
            if option.basis.trim().is_empty() {
                issue(issues, Some(id), "empty_support_basis", "each support option needs a non-empty availability basis");
            }
            if available.insert(option.action, option.available).is_some() {
                issue(issues, Some(id), "duplicate_support_option", "each support action may appear only once per case");
            }
        }

        for action in &case.acceptable_actions {
            if !action.is_protocol_action() && available.get(action) != Some(&true) {
                issue(issues, Some(id), "acceptable_action_unavailable", "acceptable support action must be explicitly declared available");
            }
        }

        if case.context.assessment_policy == AssessmentPolicy::ProhibitsAiAssistance {
            if case.acceptable_actions.iter().any(|action| action.is_ai_action())
                || case.context.support_options.iter().any(|option| option.action.is_ai_action() && option.available)
            {
                issue(issues, Some(id), "ai_action_under_prohibited_assessment", "AI assistance cannot be acceptable or available under an explicit prohibition");
            }
            if !case.critical_invariants.contains(&CriticalInvariant::RespectAssessmentPolicy) {
                issue(issues, Some(id), "missing_assessment_policy_invariant", "an explicit AI prohibition requires respect_assessment_policy");
            }
        }

        if !case.context.accessibility_requirements.is_empty() {
            if !case.critical_invariants.contains(&CriticalInvariant::PreserveAccessibility) {
                issue(issues, Some(id), "missing_accessibility_invariant", "stated accessibility requirements require preserve_accessibility");
            }

            let adapted = acceptable.contains(&SupportAction::AccessibilityAdaptation)
                && available.get(&SupportAction::AccessibilityAdaptation) == Some(&true);
            let human_path = acceptable.contains(&SupportAction::HumanSupport)
                && available.get(&SupportAction::HumanSupport) == Some(&true);
            let abstain = acceptable.contains(&SupportAction::AbstainAndEscalate);

            if !(adapted || human_path || abstain) {
                issue(issues, Some(id), "no_accessible_acceptable_path", "must accept an available accessible adaptation, human path, or abstention");
            }
            if acceptable.contains(&SupportAction::AiHintThenRetrieval) && !adapted {
                issue(issues, Some(id), "inaccessible_ai_path_acceptable", "AI-only tutoring cannot satisfy a stated accessibility requirement without an available accessible adaptation");
            }
        }

        if case.context.displacement_risk == DisplacementRisk::Material
            && case.critical_invariants.contains(&CriticalInvariant::DoNotDisplaceHumanSupport)
        {
            if acceptable.contains(&SupportAction::AiHintThenRetrieval) {
                issue(issues, Some(id), "protected_human_support_displacement", "AI-only tutoring cannot be acceptable under a material protected-support displacement risk");
            }
            if available.get(&SupportAction::HumanSupport) == Some(&true)
                && !(acceptable.contains(&SupportAction::HumanSupport)
                    || acceptable.contains(&SupportAction::EducatorCopilot))
            {
                issue(issues, Some(id), "human_support_not_preserved", "an available protected human-support path must remain acceptable");
            }
        }

        if matches!(case.context.evidence_sufficiency, EvidenceSufficiency::Insufficient | EvidenceSufficiency::Conflicting)
            && case.critical_invariants.contains(&CriticalInvariant::ClarifyBeforeHighImpactDecision)
        {
            let safe_resolution = acceptable.contains(&SupportAction::AskClarifyingQuestion)
                || acceptable.contains(&SupportAction::AbstainAndEscalate)
                || acceptable.contains(&SupportAction::HumanSupport);
            if !safe_resolution {
                issue(issues, Some(id), "missing_evidence_resolution", "insufficient or conflicting evidence requires clarification, human support, or abstention");
            }
        }
    }

    fn validate_counterfactual(
        &self,
        case: &SupportDecisionCase,
        by_id: &BTreeMap<&str, &SupportDecisionCase>,
        issues: &mut Vec<SupportDecisionIssue>,
    ) {
        let id = case.case_id.as_str();
        match (&case.counterfactual_relation, &case.counterfactual_peer) {
            (CounterfactualRelation::None, None) => return,
            (CounterfactualRelation::None, Some(_)) => {
                issue(issues, Some(id), "unexpected_counterfactual_peer", "relation none must not name a peer case");
                return;
            }
            (_, None) => {
                issue(issues, Some(id), "missing_counterfactual_peer", "change/invariant relations require a peer case");
                return;
            }
            (_, Some(peer_id)) => {
                if peer_id == &case.case_id {
                    issue(issues, Some(id), "self_referential_counterfactual", "a case cannot be its own counterfactual peer");
                    return;
                }
                let Some(peer) = by_id.get(peer_id.as_str()).copied() else {
                    issue(issues, Some(id), "unknown_counterfactual_peer", "counterfactual peer must identify another case in the manifest");
                    return;
                };

                if peer.counterfactual_peer.as_deref() != Some(id)
                    || peer.counterfactual_group != case.counterfactual_group
                    || peer.counterfactual_relation != case.counterfactual_relation
                    || peer.counterfactual_dimension != case.counterfactual_dimension
                {
                    issue(issues, Some(id), "counterfactual_pair_mismatch", "paired cases must reference each other and share group, relation, and dimension");
                    return;
                }

                let decisions_differ = !same_set(&case.acceptable_actions, &peer.acceptable_actions);
                let fact_changed = dimension_changed(case, peer);

                match case.counterfactual_relation {
                    CounterfactualRelation::Change if !fact_changed => issue(
                        issues, Some(id), "counterfactual_dimension_not_changed",
                        "a change pair must differ on its named decision-relevant dimension",
                    ),
                    CounterfactualRelation::Change if !decisions_differ => issue(
                        issues, Some(id), "counterfactual_decision_not_changed",
                        "a change pair must define different acceptable decision sets",
                    ),
                    CounterfactualRelation::Invariant if fact_changed => issue(
                        issues, Some(id), "invariant_dimension_changed",
                        "an invariant pair must keep its named dimension unchanged",
                    ),
                    CounterfactualRelation::Invariant if decisions_differ => issue(
                        issues, Some(id), "invariant_decision_changed",
                        "an invariant pair must keep its acceptable decision set unchanged",
                    ),
                    _ => {}
                }
            }
        }
    }
}

/// Parse JSON without weakening the strict typed wire contract.
pub fn parse_support_decision_manifest(input: &str) -> Result<SupportDecisionManifest, serde_json::Error> {
    serde_json::from_str(input)
}

fn deserialize_required_option<'de, D, T>(deserializer: D) -> Result<Option<T>, D::Error>
where
    D: Deserializer<'de>,
    T: Deserialize<'de>,
{
    Option::<T>::deserialize(deserializer)
}

fn valid_case_id(value: &str) -> bool {
    value.strip_prefix("LSDB-")
        .is_some_and(|suffix| suffix.len() == 3 && suffix.chars().all(|ch| ch.is_ascii_digit()))
}

fn has_duplicates<T: Copy + Ord>(values: &[T]) -> bool {
    let mut seen = BTreeSet::new();
    values.iter().any(|value| !seen.insert(*value))
}

fn same_set<T: Copy + Ord>(left: &[T], right: &[T]) -> bool {
    left.iter().copied().collect::<BTreeSet<_>>()
        == right.iter().copied().collect::<BTreeSet<_>>()
}

fn availability_for(case: &SupportDecisionCase, action: SupportAction) -> Option<bool> {
    case.context.support_options.iter()
        .find(|option| option.action == action)
        .map(|option| option.available)
}

/// A counterfactual can claim an availability change only when both cases
/// explicitly observe the same option. Missing records are not observations.
fn availability_changed_when_observed(left: Option<bool>, right: Option<bool>) -> bool {
    matches!((left, right), (Some(left), Some(right)) if left != right)
}

fn dimension_changed(left: &SupportDecisionCase, right: &SupportDecisionCase) -> bool {
    match left.counterfactual_dimension {
        CounterfactualDimension::DisplacementRisk =>
            left.context.displacement_risk != right.context.displacement_risk,
        CounterfactualDimension::SupportAvailability => {
            let actions: BTreeSet<_> = left.context.support_options.iter()
                .chain(&right.context.support_options)
                .map(|option| option.action)
                .collect();
            actions.into_iter().any(|action| {
                availability_changed_when_observed(
                    availability_for(left, action),
                    availability_for(right, action),
                )
            })
        }
        CounterfactualDimension::AccessibilityAvailability =>
            availability_changed_when_observed(
                availability_for(left, SupportAction::AccessibilityAdaptation),
                availability_for(right, SupportAction::AccessibilityAdaptation),
            ),
        CounterfactualDimension::EvidenceSufficiency =>
            left.context.evidence_sufficiency != right.context.evidence_sufficiency,
        CounterfactualDimension::PeerAvailability =>
            availability_changed_when_observed(
                availability_for(left, SupportAction::PeerCollaboration),
                availability_for(right, SupportAction::PeerCollaboration),
            ),
        CounterfactualDimension::AssessmentPolicy =>
            left.context.assessment_policy != right.context.assessment_policy,
    }
}

fn issue(issues: &mut Vec<SupportDecisionIssue>, case_id: Option<&str>, code: &str, message: &str) {
    issues.push(SupportDecisionIssue {
        code: code.to_string(),
        case_id: case_id.map(str::to_string),
        message: message.to_string(),
    });
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::Value;

    const FIXTURE: &str = include_str!("../../../examples/learning-support-decision/v1/synthetic_cases.json");

    fn fixture() -> SupportDecisionManifest {
        parse_support_decision_manifest(FIXTURE).expect("synthetic fixture must parse")
    }

    fn has_code(issues: &[SupportDecisionIssue], code: &str) -> bool {
        issues.iter().any(|issue| issue.code == code)
    }

    #[test]
    fn synthetic_fixture_satisfies_typed_and_cross_field_contract() {
        let manifest = fixture();
        let issues = manifest.validate();
        assert!(issues.is_empty(), "fixture issues: {issues:#?}");
        assert_eq!(manifest.cases.len(), 12);
        assert_eq!(
            manifest.cases.iter().map(|case| case.counterfactual_group.as_str())
                .collect::<BTreeSet<_>>().len(),
            6
        );
    }

    #[test]
    fn unknown_fields_are_rejected_instead_of_silently_ignored() {
        let mut value: Value = serde_json::from_str(FIXTURE).expect("valid JSON");
        value["unexpected_authority"] = serde_json::json!(true);
        assert!(parse_support_decision_manifest(&value.to_string()).is_err());
    }

    #[test]
    fn omitted_nullable_peer_is_not_silently_defaulted_to_none() {
        let malformed = FIXTURE.replacen("      \"counterfactual_peer\": \"LSDB-002\",\n", "", 1);
        assert!(parse_support_decision_manifest(&malformed).is_err());
    }

    #[test]
    fn unavailable_action_cannot_be_accepted() {
        let mut manifest = fixture();
        let case = manifest.cases.iter_mut()
            .find(|case| case.case_id == "LSDB-003").expect("fixture case");
        let option = case.context.support_options.iter_mut()
            .find(|option| option.action == SupportAction::AiHintThenRetrieval).expect("AI option");
        option.available = false;
        assert!(has_code(&manifest.validate(), "acceptable_action_unavailable"));
    }

    #[test]
    fn named_counterfactual_dimension_must_really_change() {
        let mut manifest = fixture();
        let case = manifest.cases.iter_mut()
            .find(|case| case.case_id == "LSDB-002").expect("fixture case");
        case.context.displacement_risk = DisplacementRisk::Material;
        assert!(has_code(&manifest.validate(), "counterfactual_dimension_not_changed"));
    }

    #[test]
    fn missing_option_is_not_treated_as_an_observed_availability_change() {
        let mut manifest = fixture();
        let case = manifest.cases.iter_mut()
            .find(|case| case.case_id == "LSDB-010").expect("fixture case");
        case.context.support_options
            .retain(|option| option.action != SupportAction::PeerCollaboration);
        assert!(has_code(
            &manifest.validate(),
            "counterfactual_dimension_not_changed"
        ));
    }

    #[test]
    fn materially_protected_human_support_cannot_be_replaced_by_ai_only() {
        let mut manifest = fixture();
        let case = manifest.cases.iter_mut()
            .find(|case| case.case_id == "LSDB-001").expect("fixture case");
        case.acceptable_actions.push(SupportAction::AiHintThenRetrieval);
        assert!(has_code(&manifest.validate(), "protected_human_support_displacement"));
    }

    #[test]
    fn prohibited_assessment_policy_cannot_mark_ai_support_available() {
        let mut manifest = fixture();
        let case = manifest.cases.iter_mut()
            .find(|case| case.case_id == "LSDB-012").expect("fixture case");
        let option = case.context.support_options.iter_mut()
            .find(|option| option.action == SupportAction::AiHintThenRetrieval).expect("AI option");
        option.available = true;
        assert!(has_code(&manifest.validate(), "ai_action_under_prohibited_assessment"));
    }

    #[test]
    fn accessibility_requirement_needs_an_accessible_acceptable_path() {
        let mut manifest = fixture();
        let case = manifest.cases.iter_mut()
            .find(|case| case.case_id == "LSDB-006").expect("fixture case");
        case.acceptable_actions = vec![SupportAction::AiHintThenRetrieval];
        assert!(has_code(&manifest.validate(), "no_accessible_acceptable_path"));
    }

    #[test]
    fn acceptable_and_forbidden_actions_must_be_disjoint() {
        let mut manifest = fixture();
        let case = manifest.cases.iter_mut()
            .find(|case| case.case_id == "LSDB-003").expect("fixture case");
        case.forbidden_actions.push(SupportAction::AiHintThenRetrieval);
        assert!(has_code(&manifest.validate(), "acceptable_forbidden_overlap"));
    }
}
