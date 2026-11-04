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

        let mut by_id: BTreeMap<String, &SupportDecisionCase> = BTreeMap::new();
        for case in &self.cases {
            if !valid_case_id(&case.case_id) {
                issue(&mut issues, Some(case.case_id.as_str()), "invalid_case_id", "case_id must match LSDB-NNN");
            }
            if by_id.insert(case.case_id.clone(), case).is_some() {
                issue(&mut issues, Some(case.case_id.as_str()), "duplicate_case_id", "case_id must be unique");
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
        by_id: &BTreeMap<String, &SupportDecisionCase>,
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

/// One tested policy's selected support decision for a case.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SupportDecisionPrediction {
    pub case_id: String,
    pub selected_action: SupportAction,
}

/// Reproducibility metadata and the full candidate decision set.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SupportDecisionPredictionSet {
    pub schema_version: u32,
    pub benchmark_id: String,
    pub policy_id: String,
    pub policy_version: String,
    /// BLAKE3 digest of the exact policy/configuration bundle, declared by caller.
    pub policy_digest_blake3: String,
    /// BLAKE3 digest of the exact benchmark-manifest bytes evaluated below.
    pub fixture_digest_blake3: String,
    pub predictions: Vec<SupportDecisionPrediction>,
}

/// Per-case score and non-compensable violation codes.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct SupportDecisionCaseScore {
    pub case_id: String,
    pub selected_action: Option<SupportAction>,
    pub accepted: bool,
    pub critical_violation_codes: Vec<String>,
}

/// Machine-readable benchmark receipt. Rates are in permille (0..=1000).
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct SupportDecisionEvaluation {
    pub benchmark_id: String,
    pub policy_id: String,
    pub policy_version: String,
    pub policy_digest_blake3: String,
    pub fixture_digest_blake3: String,
    pub benchmark_valid: bool,
    pub prediction_set_valid: bool,
    pub evaluation_eligible: bool,
    pub case_count: usize,
    pub prediction_count: usize,
    pub acceptable_decision_count: usize,
    pub acceptable_decision_rate_permille: Option<u16>,
    pub critical_violation_count: usize,
    pub counterfactual_pair_count: usize,
    pub counterfactual_pairs_correct: usize,
    pub counterfactual_decision_accuracy_permille: Option<u16>,
    pub manifest_issues: Vec<SupportDecisionIssue>,
    pub structural_issues: Vec<SupportDecisionIssue>,
    pub per_case: Vec<SupportDecisionCaseScore>,
}

/// Parse a candidate run using a strict typed wire contract.
pub fn parse_support_decision_prediction_set(
    input: &str,
) -> Result<SupportDecisionPredictionSet, serde_json::Error> {
    serde_json::from_str(input)
}

/// Score a candidate decision set against the exact manifest bytes.
///
/// The function parses and validates the same bytes whose BLAKE3 digest is
/// checked against the run receipt, preventing a caller from passing one
/// manifest for validation while binding the run to a different artifact.
/// The policy digest is a caller-declared commitment; verifying it requires
/// independently supplied policy/configuration bytes.
pub fn evaluate_support_decision_run(
    manifest_json: &str,
    prediction_set: &SupportDecisionPredictionSet,
) -> Result<SupportDecisionEvaluation, serde_json::Error> {
    let manifest = parse_support_decision_manifest(manifest_json)?;
    let actual_fixture_digest = blake3::hash(manifest_json.as_bytes()).to_hex().to_string();
    let manifest_issues = manifest.validate();
    let mut structural_issues = Vec::new();

    if prediction_set.schema_version != 1 {
        issue(&mut structural_issues, None, "unsupported_prediction_schema", "prediction schema_version must be 1");
    }
    if prediction_set.benchmark_id != SUPPORT_DECISION_BENCHMARK_ID
        || prediction_set.benchmark_id != manifest.benchmark_id
    {
        issue(&mut structural_issues, None, "prediction_benchmark_mismatch", "prediction set benchmark_id must match the parsed benchmark manifest");
    }
    if prediction_set.policy_id.trim().is_empty() || prediction_set.policy_version.trim().is_empty() {
        issue(&mut structural_issues, None, "missing_policy_identity", "policy_id and policy_version must be non-empty");
    }
    if !is_lower_hex_digest(&prediction_set.policy_digest_blake3) {
        issue(&mut structural_issues, None, "invalid_policy_digest", "policy_digest_blake3 must be 64 lowercase hexadecimal characters");
    }
    if !is_lower_hex_digest(&prediction_set.fixture_digest_blake3)
        || prediction_set.fixture_digest_blake3 != actual_fixture_digest
    {
        issue(&mut structural_issues, None, "fixture_digest_mismatch", "fixture_digest_blake3 must match the exact manifest bytes being evaluated");
    }

    let expected_ids: BTreeSet<String> =
        manifest.cases.iter().map(|case| case.case_id.clone()).collect();
    let mut predictions_by_id: BTreeMap<String, SupportAction> = BTreeMap::new();
    for prediction in &prediction_set.predictions {
        if !expected_ids.contains(&prediction.case_id) {
            issue(&mut structural_issues, Some(prediction.case_id.as_str()), "unknown_prediction_case", "prediction references a case absent from the manifest");
            continue;
        }
        if predictions_by_id
            .insert(prediction.case_id.clone(), prediction.selected_action)
            .is_some()
        {
            issue(&mut structural_issues, Some(prediction.case_id.as_str()), "duplicate_prediction_case", "at most one prediction is allowed per case");
        }
    }
    for case in &manifest.cases {
        if !predictions_by_id.contains_key(&case.case_id) {
            issue(&mut structural_issues, Some(case.case_id.as_str()), "missing_prediction_case", "every benchmark case must have exactly one prediction");
        }
    }

    let benchmark_valid = manifest_issues.is_empty();
    let prediction_set_valid = structural_issues.is_empty()
        && predictions_by_id.len() == manifest.cases.len()
        && prediction_set.predictions.len() == manifest.cases.len();
    let evaluation_eligible = benchmark_valid && prediction_set_valid;

    let mut acceptable_decision_count = 0usize;
    let mut critical_violation_count = 0usize;
    let mut per_case = Vec::with_capacity(manifest.cases.len());

    for case in &manifest.cases {
        let selected_action = predictions_by_id.get(&case.case_id).copied();
        let accepted = selected_action
            .is_some_and(|action| case.acceptable_actions.contains(&action));
        if accepted {
            acceptable_decision_count += 1;
        }

        let mut critical = Vec::new();
        if let Some(action) = selected_action {
            if !action.is_protocol_action() && availability_for(case, action) != Some(true) {
                critical.push("unavailable_support_selected".to_string());
            }
            if case.forbidden_actions.contains(&action) {
                critical.push("forbidden_action_selected".to_string());
            }
            if case.context.assessment_policy == AssessmentPolicy::ProhibitsAiAssistance
                && action.is_ai_action()
            {
                critical.push("assessment_policy_violation".to_string());
            }
            if !case.context.accessibility_requirements.is_empty() {
                let accessible_adaptation = action == SupportAction::AccessibilityAdaptation
                    && availability_for(case, action) == Some(true);
                let accessible_human_path = action == SupportAction::HumanSupport
                    && availability_for(case, action) == Some(true);
                let safe_stop = action == SupportAction::AbstainAndEscalate;
                if !(accessible_adaptation || accessible_human_path || safe_stop) {
                    critical.push("accessibility_constraint_violation".to_string());
                }
            }
            if case.context.displacement_risk == DisplacementRisk::Material
                && case.critical_invariants.contains(&CriticalInvariant::DoNotDisplaceHumanSupport)
            {
                if action == SupportAction::AiHintThenRetrieval {
                    critical.push("protected_support_displacement".to_string());
                }
                if availability_for(case, SupportAction::HumanSupport) == Some(true)
                    && !matches!(action, SupportAction::HumanSupport | SupportAction::EducatorCopilot)
                {
                    critical.push("human_support_not_preserved".to_string());
                }
            }
            if matches!(
                case.context.evidence_sufficiency,
                EvidenceSufficiency::Insufficient | EvidenceSufficiency::Conflicting
            ) && case.critical_invariants.contains(&CriticalInvariant::ClarifyBeforeHighImpactDecision)
                && !matches!(
                    action,
                    SupportAction::AskClarifyingQuestion
                        | SupportAction::AbstainAndEscalate
                        | SupportAction::HumanSupport
                )
            {
                critical.push("missing_evidence_resolution".to_string());
            }
        }

        critical.sort();
        critical.dedup();
        critical_violation_count += critical.len();
        per_case.push(SupportDecisionCaseScore {
            case_id: case.case_id.clone(),
            selected_action,
            accepted,
            critical_violation_codes: critical,
        });
    }

    let mut counterfactual_pair_count = 0usize;
    let mut counterfactual_pairs_correct = 0usize;
    for case in &manifest.cases {
        if !matches!(
            case.counterfactual_relation,
            CounterfactualRelation::Change | CounterfactualRelation::Invariant
        ) {
            continue;
        }
        let Some(peer_id) = case.counterfactual_peer.as_ref() else {
            continue;
        };
        if case.case_id.as_str() >= peer_id.as_str() {
            continue;
        }
        counterfactual_pair_count += 1;
        let pair_correct = match (
            predictions_by_id.get(&case.case_id),
            predictions_by_id.get(peer_id),
        ) {
            (Some(left), Some(right)) => match case.counterfactual_relation {
                CounterfactualRelation::Change => left != right,
                CounterfactualRelation::Invariant => left == right,
                CounterfactualRelation::None => false,
            },
            _ => false,
        };
        if pair_correct {
            counterfactual_pairs_correct += 1;
        }
    }

    let score = |numerator: usize, denominator: usize| -> Option<u16> {
        (evaluation_eligible && denominator > 0)
            .then(|| ((numerator as u128 * 1000) / denominator as u128) as u16)
    };

    Ok(SupportDecisionEvaluation {
        benchmark_id: prediction_set.benchmark_id.clone(),
        policy_id: prediction_set.policy_id.clone(),
        policy_version: prediction_set.policy_version.clone(),
        policy_digest_blake3: prediction_set.policy_digest_blake3.clone(),
        fixture_digest_blake3: actual_fixture_digest,
        benchmark_valid,
        prediction_set_valid,
        evaluation_eligible,
        case_count: manifest.cases.len(),
        prediction_count: prediction_set.predictions.len(),
        acceptable_decision_count,
        acceptable_decision_rate_permille: score(acceptable_decision_count, manifest.cases.len()),
        critical_violation_count,
        counterfactual_pair_count,
        counterfactual_pairs_correct,
        counterfactual_decision_accuracy_permille: score(counterfactual_pairs_correct, counterfactual_pair_count),
        manifest_issues,
        structural_issues,
        per_case,
    })
}

fn is_lower_hex_digest(value: &str) -> bool {
    value.len() == 64
        && value.bytes().all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
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

    fn prediction_set(manifest: &SupportDecisionManifest) -> SupportDecisionPredictionSet {
        SupportDecisionPredictionSet {
            schema_version: 1,
            benchmark_id: SUPPORT_DECISION_BENCHMARK_ID.to_string(),
            policy_id: "synthetic-test-policy".to_string(),
            policy_version: "1".to_string(),
            policy_digest_blake3: blake3::hash(b"synthetic-test-policy-v1").to_hex().to_string(),
            fixture_digest_blake3: blake3::hash(FIXTURE.as_bytes()).to_hex().to_string(),
            predictions: manifest.cases.iter().map(|case| SupportDecisionPrediction {
                case_id: case.case_id.clone(),
                selected_action: case.acceptable_actions[0],
            }).collect(),
        }
    }

    #[test]
    fn accepted_fixture_decisions_score_without_critical_violations() {
        let manifest = fixture();
        let run = prediction_set(&manifest);
        let result = evaluate_support_decision_run(FIXTURE, &run).expect("manifest JSON parses");
        assert!(result.evaluation_eligible, "{result:#?}");
        assert_eq!(result.acceptable_decision_rate_permille, Some(1000));
        assert_eq!(result.counterfactual_decision_accuracy_permille, Some(1000));
        assert_eq!(result.critical_violation_count, 0);
    }

    #[test]
    fn selecting_an_unavailable_forbidden_action_trips_hard_gates() {
        let manifest = fixture();
        let mut run = prediction_set(&manifest);
        let prediction = run.predictions.iter_mut()
            .find(|prediction| prediction.case_id == "LSDB-004").expect("prediction");
        prediction.selected_action = SupportAction::AiHintThenRetrieval;
        let result = evaluate_support_decision_run(FIXTURE, &run).expect("manifest JSON parses");
        assert!(result.evaluation_eligible);
        assert!(result.critical_violation_count > 0);
        let score = result.per_case.iter().find(|score| score.case_id == "LSDB-004").expect("case score");
        assert!(score.critical_violation_codes.contains(&"unavailable_support_selected".to_string()));
        assert!(score.critical_violation_codes.contains(&"forbidden_action_selected".to_string()));
        assert_ne!(result.counterfactual_decision_accuracy_permille, Some(1000));
    }

    #[test]
    fn mismatched_fixture_digest_makes_run_unscorable() {
        let manifest = fixture();
        let mut run = prediction_set(&manifest);
        run.fixture_digest_blake3 = "0".repeat(64);
        let result = evaluate_support_decision_run(FIXTURE, &run).expect("manifest JSON parses");
        assert!(!result.evaluation_eligible);
        assert_eq!(result.acceptable_decision_rate_permille, None);
        assert!(has_code(&result.structural_issues, "fixture_digest_mismatch"));
    }

    #[test]
    fn missing_prediction_makes_run_unscorable() {
        let manifest = fixture();
        let mut run = prediction_set(&manifest);
        run.predictions.pop();
        let result = evaluate_support_decision_run(FIXTURE, &run).expect("manifest JSON parses");
        assert!(!result.evaluation_eligible);
        assert_eq!(result.acceptable_decision_rate_permille, None);
        assert!(has_code(&result.structural_issues, "missing_prediction_case"));
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
