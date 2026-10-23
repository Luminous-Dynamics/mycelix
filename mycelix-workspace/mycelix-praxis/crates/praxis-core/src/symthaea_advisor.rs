// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! A bounded, side-effect-free wire contract for optional Symthaea advice.
//!
//! The adapter validates evidence references and local Praxis policy constraints.
//! It does not call Symthaea, infer mastery, mutate learner state, or authorize effects.

use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;

pub const SYMTHAEA_ADVISOR_CONTRACT_ID: &str = "PRAXIS-SYMTHAEA-ADVISOR-001";
pub const SYMTHAEA_ADVISOR_SCHEMA_VERSION: u32 = 1;
const SYMTHAEA_REPOSITORY: &str = "Luminous-Dynamics/symthaea";

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SymthaeaAdvisorRequest {
    pub schema_version: u32,
    pub contract_id: String,
    pub request_id: String,
    pub advisory_consent_granted: bool,
    pub policy_profile: PolicyProfile,
    pub goal: LearningGoal,
    pub evidence_snapshot: EvidenceSnapshot,
    pub evidence_sufficiency: EvidenceSufficiency,
    pub displacement_risk: DisplacementRisk,
    pub human_support_must_be_preserved: bool,
    pub accessibility_requirements: Vec<String>,
    pub assessment_policy: AssessmentPolicy,
    pub preferred_actions: Vec<AdvisorAction>,
    pub support_options: Vec<SupportOption>,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PolicyProfile {
    pub profile_id: String,
    pub profile_version: String,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct LearningGoal {
    pub target_capability_id: String,
    pub summary: String,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct EvidenceSnapshot {
    pub snapshot_id: String,
    pub snapshot_digest_blake3: String,
    pub captured_at: String,
    pub evidence_items: Vec<LearningEvidenceItem>,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct LearningEvidenceItem {
    pub evidence_id: String,
    pub kind: EvidenceKind,
    pub epistemic_status: EpistemicStatus,
    pub summary: String,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum EvidenceKind {
    LearnerGoal,
    IndependentAttempt,
    AssistedAttempt,
    ProgressSummary,
    SupportAvailability,
    CoursePolicy,
    AccessibilityRequirement,
    LearnerPreference,
    StudyContext,
    Other,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum EpistemicStatus {
    Observed,
    LearnerReported,
    Derived,
    Hypothesis,
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
pub enum AdvisorAction {
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

impl AdvisorAction {
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

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SupportOption {
    pub action: AdvisorAction,
    pub available: bool,
    pub accessible_for_context: bool,
    pub policy_permitted: bool,
    pub basis_evidence_refs: Vec<String>,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SymthaeaAdvisorResponse {
    pub schema_version: u32,
    pub contract_id: String,
    pub request_id: String,
    pub request_digest_blake3: String,
    pub advisor_build: AdvisorBuild,
    pub policy_profile: PolicyProfile,
    pub disposition: AdviceDisposition,
    pub selected_action: AdvisorAction,
    pub alternative_actions: Vec<AdvisorAction>,
    pub evidence_refs: Vec<String>,
    pub uncertainty: UncertaintyLevel,
    pub rationale: String,
    pub clarifying_questions: Vec<String>,
    pub limitations: Vec<String>,
    pub effect_mode: EffectMode,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AdvisorBuild {
    pub repository: String,
    pub source_commit: String,
    pub feature_profile: String,
    pub feature_profile_digest_blake3: String,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AdviceDisposition {
    Recommend,
    Clarify,
    Abstain,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum UncertaintyLevel {
    Low,
    Medium,
    High,
    Unknown,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum EffectMode {
    AdvisoryOnly,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct SymthaeaAdvisorIssue {
    pub code: String,
    pub field: String,
    pub message: String,
}

/// A small provider boundary. Implementations may be in-process or isolated sidecars;
/// they must return an advisory response and must not perform learning or ledger writes.
pub trait SymthaeaAdvisorPort {
    fn advise(&self, request_json: &str) -> Result<String, SymthaeaAdvisorPortError>;
}

#[derive(Clone, Debug, PartialEq, Eq, thiserror::Error)]
pub enum SymthaeaAdvisorPortError {
    #[error("Symthaea advisor is unavailable")]
    Unavailable,
    #[error("Symthaea adapter transport failed: {0}")]
    Transport(String),
}

/// Parse with a strict typed contract. Unknown fields and unknown enum values fail.
pub fn parse_symthaea_advisor_request(
    input: &str,
) -> Result<SymthaeaAdvisorRequest, serde_json::Error> {
    serde_json::from_str(input)
}

pub fn parse_symthaea_advisor_response(
    input: &str,
) -> Result<SymthaeaAdvisorResponse, serde_json::Error> {
    serde_json::from_str(input)
}

/// Validate the request before dispatching any advisory call.
pub fn validate_symthaea_advisor_request(
    request: &SymthaeaAdvisorRequest,
) -> Vec<SymthaeaAdvisorIssue> {
    let mut issues = Vec::new();

    if request.schema_version != SYMTHAEA_ADVISOR_SCHEMA_VERSION {
        issue(&mut issues, "schema_version", "unsupported_schema_version", "schema_version must be 1");
    }
    if request.contract_id != SYMTHAEA_ADVISOR_CONTRACT_ID {
        issue(&mut issues, "contract_id", "unknown_contract_id", "contract_id must identify PRAXIS-SYMTHAEA-ADVISOR-001");
    }
    if request.request_id.trim().is_empty() {
        issue(&mut issues, "request_id", "empty_request_id", "request_id must not be blank");
    }
    if !request.advisory_consent_granted {
        issue(&mut issues, "advisory_consent_granted", "advisory_consent_missing", "Symthaea advice requires affirmative, purpose-specific consent");
    }
    if request.policy_profile.profile_id.trim().is_empty() || request.policy_profile.profile_version.trim().is_empty() {
        issue(&mut issues, "policy_profile", "invalid_policy_profile", "policy profile id and version must not be blank");
    }
    if request.goal.target_capability_id.trim().is_empty() || request.goal.summary.trim().is_empty() {
        issue(&mut issues, "goal", "invalid_learning_goal", "target capability and goal summary must not be blank");
    }

    let snapshot = &request.evidence_snapshot;
    if snapshot.snapshot_id.trim().is_empty() {
        issue(&mut issues, "evidence_snapshot.snapshot_id", "empty_snapshot_id", "snapshot id must not be blank");
    }
    if !is_lower_hex_digest(&snapshot.snapshot_digest_blake3) {
        issue(&mut issues, "evidence_snapshot.snapshot_digest_blake3", "invalid_snapshot_digest", "snapshot digest must be 64 lowercase hexadecimal characters");
    }
    if chrono::DateTime::parse_from_rfc3339(&snapshot.captured_at).is_err() {
        issue(&mut issues, "evidence_snapshot.captured_at", "invalid_snapshot_timestamp", "captured_at must be RFC 3339 date-time");
    }
    if snapshot.evidence_items.is_empty() {
        issue(&mut issues, "evidence_snapshot.evidence_items", "missing_evidence_items", "at least one purpose-limited evidence item is required");
    }

    let mut evidence_ids = BTreeSet::new();
    for item in &snapshot.evidence_items {
        if item.evidence_id.trim().is_empty() || item.summary.trim().is_empty() {
            issue(&mut issues, "evidence_snapshot.evidence_items", "empty_evidence_item", "evidence id and minimized summary must not be blank");
        }
        if !evidence_ids.insert(item.evidence_id.as_str()) {
            issue(&mut issues, "evidence_snapshot.evidence_items", "duplicate_evidence_id", "evidence ids must be unique within a snapshot");
        }
    }

    if request.support_options.is_empty() {
        issue(&mut issues, "support_options", "missing_support_options", "at least one support option must be enumerated");
    }
    let mut preferred = BTreeSet::new();
    for action in &request.preferred_actions {
        if !preferred.insert(*action) {
            issue(&mut issues, "preferred_actions", "duplicate_preferred_action", "preferred actions must be unique");
        }
    }
    let mut accessibility = BTreeSet::new();
    for requirement in &request.accessibility_requirements {
        if requirement.trim().is_empty() || !accessibility.insert(requirement.as_str()) {
            issue(&mut issues, "accessibility_requirements", "invalid_accessibility_requirement", "accessibility requirements must be non-empty and unique");
        }
    }

    let mut actions = BTreeSet::new();
    for option in &request.support_options {
        if !actions.insert(option.action) {
            issue(&mut issues, "support_options", "duplicate_support_option", "each support action may appear only once");
        }
        if option.basis_evidence_refs.is_empty() {
            issue(&mut issues, "support_options.basis_evidence_refs", "missing_support_basis", "each support option must cite at least one input evidence reference");
        }
        let mut basis_references = BTreeSet::new();
        for reference in &option.basis_evidence_refs {
            if !basis_references.insert(reference.as_str()) {
                issue(&mut issues, "support_options.basis_evidence_refs", "duplicate_support_basis_reference", "support-option basis references must be unique");
            }
        }
        for reference in &option.basis_evidence_refs {
            if !evidence_ids.iter().any(|evidence_id| *evidence_id == reference.as_str()) {
                issue(&mut issues, "support_options.basis_evidence_refs", "unknown_support_basis_reference", "support-option basis must cite evidence supplied in this request");
            }
        }
        if matches!(
            request.assessment_policy,
            AssessmentPolicy::ProhibitsAiAssistance | AssessmentPolicy::Unknown
        ) && option.action.is_ai_action() && option.policy_permitted {
            issue(&mut issues, "support_options.policy_permitted", "ai_policy_conflict", "AI assistance cannot be marked policy-permitted while the assessment policy is prohibited or unresolved");
        }
    }

    issues
}

/// Validate the response against the exact request bytes and local policy.
///
/// The BLAKE3 digest is computed over request_json itself, then the same bytes
/// are parsed and checked. Evidence-snapshot digest syntax is checked here;
/// source bytes for that snapshot must be verified by the caller.
pub fn validate_symthaea_advice_for_request(
    request_json: &str,
    response_json: &str,
) -> Result<Vec<SymthaeaAdvisorIssue>, serde_json::Error> {
    let request = parse_symthaea_advisor_request(request_json)?;
    let response = parse_symthaea_advisor_response(response_json)?;
    let mut issues = validate_symthaea_advisor_request(&request);
    let request_digest = blake3::hash(request_json.as_bytes()).to_hex().to_string();

    if response.schema_version != SYMTHAEA_ADVISOR_SCHEMA_VERSION {
        issue(&mut issues, "schema_version", "unsupported_response_version", "response schema_version must be 1");
    }
    if response.contract_id != SYMTHAEA_ADVISOR_CONTRACT_ID {
        issue(&mut issues, "contract_id", "response_contract_mismatch", "response contract id must identify PRAXIS-SYMTHAEA-ADVISOR-001");
    }
    if response.request_id != request.request_id {
        issue(&mut issues, "request_id", "request_id_mismatch", "response request_id must equal the dispatched request id");
    }
    if response.request_digest_blake3 != request_digest {
        issue(&mut issues, "request_digest_blake3", "request_digest_mismatch", "response must bind the exact request bytes dispatched to Symthaea");
    }
    if response.policy_profile != request.policy_profile {
        issue(&mut issues, "policy_profile", "policy_profile_mismatch", "response policy profile must match the request");
    }
    if response.advisor_build.repository != SYMTHAEA_REPOSITORY
        || !is_lower_hex_commit(&response.advisor_build.source_commit)
        || response.advisor_build.feature_profile.trim().is_empty()
        || !is_lower_hex_digest(&response.advisor_build.feature_profile_digest_blake3)
    {
        issue(&mut issues, "advisor_build", "invalid_advisor_build_identity", "advisor build must bind repository, 40-character commit, feature profile, and its BLAKE3 digest");
    }
    if response.effect_mode != EffectMode::AdvisoryOnly {
        issue(&mut issues, "effect_mode", "effect_authority_forbidden", "the Symthaea adapter is advisory-only");
    }
    if response.rationale.trim().is_empty() {
        issue(&mut issues, "rationale", "empty_rationale", "advice rationale must not be blank");
    }

    match response.disposition {
        AdviceDisposition::Recommend => {
            if matches!(response.selected_action, AdvisorAction::AskClarifyingQuestion | AdvisorAction::AbstainAndEscalate) {
                issue(&mut issues, "selected_action", "disposition_action_mismatch", "recommend disposition cannot select clarify or abstain actions");
            }
            if !response.clarifying_questions.is_empty() {
                issue(&mut issues, "clarifying_questions", "unexpected_clarifying_questions", "recommend disposition should not include pending clarification questions");
            }
        }
        AdviceDisposition::Clarify => {
            if response.selected_action != AdvisorAction::AskClarifyingQuestion {
                issue(&mut issues, "selected_action", "disposition_action_mismatch", "clarify disposition must select ask_clarifying_question");
            }
            if response.clarifying_questions.is_empty() {
                issue(&mut issues, "clarifying_questions", "missing_clarifying_questions", "clarify disposition must include at least one specific question");
            }
        }
        AdviceDisposition::Abstain => {
            if response.selected_action != AdvisorAction::AbstainAndEscalate {
                issue(&mut issues, "selected_action", "disposition_action_mismatch", "abstain disposition must select abstain_and_escalate");
            }
        }
    }

    let supplied_evidence: BTreeSet<&str> = request.evidence_snapshot.evidence_items
        .iter().map(|item| item.evidence_id.as_str()).collect();
    if response.evidence_refs.is_empty() {
        issue(&mut issues, "evidence_refs", "missing_evidence_references", "advice must cite the input evidence references supporting its recommendation");
    }
    let mut cited = BTreeSet::new();
    for reference in &response.evidence_refs {
        if !supplied_evidence.iter().any(|evidence_id| *evidence_id == reference.as_str()) {
            issue(&mut issues, "evidence_refs", "unknown_advice_evidence_reference", "advice may cite only evidence ids supplied in the request");
        }
        if !cited.insert(reference.as_str()) {
            issue(&mut issues, "evidence_refs", "duplicate_advice_evidence_reference", "evidence references must be unique");
        }
    }

    validate_selected_action(&request, response.selected_action, "selected_action", &mut issues);
    let mut alternatives = BTreeSet::new();
    for action in &response.alternative_actions {
        if !alternatives.insert(*action) {
            issue(&mut issues, "alternative_actions", "duplicate_alternative_action", "alternative actions must be unique");
        }
        validate_selected_action(&request, *action, "alternative_actions", &mut issues);
    }
    if response.alternative_actions.contains(&response.selected_action) {
        issue(&mut issues, "alternative_actions", "selected_action_repeated_as_alternative", "selected action must not also appear as an alternative");
    }

    Ok(issues)
}

impl SymthaeaAdvisorRequest {
    fn contextually_requires_accessible_support(&self) -> bool {
        !self.accessibility_requirements.is_empty()
    }
}

fn validate_selected_action(
    request: &SymthaeaAdvisorRequest,
    action: AdvisorAction,
    field: &str,
    issues: &mut Vec<SymthaeaAdvisorIssue>,
) {
    if request.assessment_policy == AssessmentPolicy::ProhibitsAiAssistance && action.is_ai_action() {
        issue(issues, field, "prohibited_ai_support_selected", "advice cannot select or suggest AI assistance when the assessment policy explicitly prohibits it");
    }
    if request.assessment_policy == AssessmentPolicy::Unknown && action.is_ai_action() {
        issue(issues, field, "unresolved_assessment_policy_for_ai_support", "AI assistance cannot be selected or suggested until an unknown assessment policy is resolved or explicitly marked not applicable");
    }

    if matches!(request.evidence_sufficiency, EvidenceSufficiency::Insufficient | EvidenceSufficiency::Conflicting)
        && !matches!(action, AdvisorAction::AskClarifyingQuestion | AdvisorAction::AbstainAndEscalate | AdvisorAction::HumanSupport)
    {
        issue(issues, field, "insufficient_evidence_requires_clarification", "insufficient or conflicting context must lead to clarification, abstention, or human support");
    }

    if request.contextually_requires_accessible_support()
        && action == AdvisorAction::NoIntervention
    {
        issue(issues, field, "accessibility_needs_unresolved", "a stated accessibility requirement cannot be left unresolved by a no-op; clarification and safe escalation remain allowed");
    }

    if request.human_support_must_be_preserved {
        if action == AdvisorAction::AiHintThenRetrieval {
            issue(issues, field, "protected_human_support_displacement", "AI-only support cannot replace protected human support");
        }
        let human_path_available = request.support_options.iter().any(|option| {
            matches!(option.action, AdvisorAction::HumanSupport | AdvisorAction::EducatorCopilot)
                && option.available
                && option.policy_permitted
        });
        if human_path_available
            && !matches!(
                action,
                AdvisorAction::HumanSupport
                    | AdvisorAction::EducatorCopilot
                    | AdvisorAction::AskClarifyingQuestion
                    | AdvisorAction::AbstainAndEscalate
            )
        {
            issue(issues, field, "human_support_not_preserved", "action must preserve available human support, augment the educator, or safely escalate instead of replacing it");
        } else if !human_path_available
            && !matches!(action, AdvisorAction::AskClarifyingQuestion | AdvisorAction::AbstainAndEscalate)
        {
            issue(issues, field, "protected_human_support_unavailable_requires_abstention", "when preservation is required but no human/educator path is available, advice must clarify or abstain");
        }
    }

    if action.is_protocol_action() {
        return;
    }
    let Some(option) = request.support_options.iter().find(|option| option.action == action) else {
        issue(issues, field, "action_not_in_request", "advice selected an action not enumerated in the request");
        return;
    };
    if !option.available {
        issue(issues, field, "selected_support_unavailable", "advice cannot select an unavailable support option");
    }
    if !option.policy_permitted {
        issue(issues, field, "selected_support_not_policy_permitted", "advice cannot select a support option prohibited by local policy");
    }
    if request.contextually_requires_accessible_support() && !option.accessible_for_context {
        issue(issues, field, "inaccessible_support_selected", "selected or alternative support is not marked accessible for the declared context");
    }
}

fn is_lower_hex_digest(value: &str) -> bool {
    value.len() == 64
        && value.bytes().all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}

fn is_lower_hex_commit(value: &str) -> bool {
    value.len() == 40
        && value.bytes().all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}

fn issue(
    issues: &mut Vec<SymthaeaAdvisorIssue>,
    field: &str,
    code: &str,
    message: &str,
) {
    issues.push(SymthaeaAdvisorIssue {
        code: code.to_string(),
        field: field.to_string(),
        message: message.to_string(),
    });
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::Value;

    const REQUEST_FIXTURE: &str =
        include_str!("../../../examples/symthaea-advisor/v1/synthetic_request.json");

    fn valid_response_json(request_json: &str) -> String {
        let request = parse_symthaea_advisor_request(request_json).expect("request fixture parses");
        serde_json::to_string(&SymthaeaAdvisorResponse {
            schema_version: 1,
            contract_id: SYMTHAEA_ADVISOR_CONTRACT_ID.to_string(),
            request_id: request.request_id.clone(),
            request_digest_blake3: blake3::hash(request_json.as_bytes()).to_hex().to_string(),
            advisor_build: AdvisorBuild {
                repository: SYMTHAEA_REPOSITORY.to_string(),
                source_commit: "a".repeat(40),
                feature_profile: "synthetic-mock-reasoning-engine".to_string(),
                feature_profile_digest_blake3: "b".repeat(64),
            },
            policy_profile: request.policy_profile.clone(),
            disposition: AdviceDisposition::Recommend,
            selected_action: AdvisorAction::AiHintThenRetrieval,
            alternative_actions: vec![AdvisorAction::RetrievalPractice],
            evidence_refs: vec!["goal-001".to_string(), "policy-001".to_string()],
            uncertainty: UncertaintyLevel::Medium,
            rationale: "A short hint followed by an independent attempt matches the stated practice goal and permitted support options.".to_string(),
            clarifying_questions: Vec::new(),
            limitations: vec!["The advice does not establish durable mastery.".to_string()],
            effect_mode: EffectMode::AdvisoryOnly,
        }).expect("response serializes")
    }

    fn has_code(issues: &[SymthaeaAdvisorIssue], code: &str) -> bool {
        issues.iter().any(|issue| issue.code == code)
    }

    #[test]
    fn synthetic_request_is_typed_and_has_no_contract_violations() {
        let request = parse_symthaea_advisor_request(REQUEST_FIXTURE).expect("request fixture parses");
        assert!(validate_symthaea_advisor_request(&request).is_empty());
    }

    #[test]
    fn bound_advisory_response_is_accepted() {
        let response = valid_response_json(REQUEST_FIXTURE);
        let issues = validate_symthaea_advice_for_request(REQUEST_FIXTURE, &response)
            .expect("both JSON documents parse");
        assert!(issues.is_empty(), "unexpected findings: {issues:#?}");
    }

    #[test]
    fn request_digest_is_bound_to_exact_dispatched_bytes() {
        let response = valid_response_json(REQUEST_FIXTURE);
        let minified = serde_json::to_string(&parse_symthaea_advisor_request(REQUEST_FIXTURE).unwrap()).unwrap();
        let issues = validate_symthaea_advice_for_request(&minified, &response)
            .expect("both JSON documents parse");
        assert!(has_code(&issues, "request_digest_mismatch"));
    }

    #[test]
    fn unavailable_support_selection_is_rejected() {
        let mut request: Value = serde_json::from_str(REQUEST_FIXTURE).unwrap();
        request["support_options"][0]["available"] = Value::Bool(false);
        let request_json = serde_json::to_string(&request).unwrap();
        let response = valid_response_json(&request_json);
        let issues = validate_symthaea_advice_for_request(&request_json, &response).unwrap();
        assert!(has_code(&issues, "selected_support_unavailable"));
    }

    #[test]
    fn evidence_references_cannot_be_invented() {
        let mut response: Value = serde_json::from_str(&valid_response_json(REQUEST_FIXTURE)).unwrap();
        response["evidence_refs"] = serde_json::json!(["invented-evidence-999"]);
        let issues = validate_symthaea_advice_for_request(
            REQUEST_FIXTURE,
            &serde_json::to_string(&response).unwrap(),
        ).unwrap();
        assert!(has_code(&issues, "unknown_advice_evidence_reference"));
    }

    #[test]
    fn prohibited_ai_assistance_is_rejected_even_if_the_service_is_online() {
        let mut request: Value = serde_json::from_str(REQUEST_FIXTURE).unwrap();
        request["assessment_policy"] = serde_json::json!("prohibits_ai_assistance");
        request["support_options"][0]["policy_permitted"] = Value::Bool(false);
        let request_json = serde_json::to_string(&request).unwrap();
        let response = valid_response_json(&request_json);
        let issues = validate_symthaea_advice_for_request(&request_json, &response).unwrap();
        assert!(has_code(&issues, "prohibited_ai_support_selected"));
    }

    #[test]
    fn unresolved_assessment_policy_cannot_mark_ai_support_permitted() {
        let mut request: Value = serde_json::from_str(REQUEST_FIXTURE).unwrap();
        request["assessment_policy"] = serde_json::json!("unknown");
        let parsed = parse_symthaea_advisor_request(&serde_json::to_string(&request).unwrap()).unwrap();
        assert!(has_code(
            &validate_symthaea_advisor_request(&parsed),
            "ai_policy_conflict"
        ));
    }

    #[test]
    fn missing_preserved_human_path_can_abstain_instead_of_invalidating_request() {
        let mut request: Value = serde_json::from_str(REQUEST_FIXTURE).unwrap();
        request["human_support_must_be_preserved"] = Value::Bool(true);
        request["displacement_risk"] = serde_json::json!("material");
        request["support_options"] = serde_json::json!([
            {"action":"ai_hint_then_retrieval","available":true,"accessible_for_context":true,"policy_permitted":true,"basis_evidence_refs":["goal-001","availability-001","policy-001"]},
            {"action":"offline_fallback","available":true,"accessible_for_context":true,"policy_permitted":true,"basis_evidence_refs":["availability-001"]}
        ]);
        let request_json = serde_json::to_string(&request).unwrap();
        let parsed = parse_symthaea_advisor_request(&request_json).unwrap();
        assert!(validate_symthaea_advisor_request(&parsed).is_empty());
        let mut response: Value = serde_json::from_str(&valid_response_json(&request_json)).unwrap();
        response["disposition"] = serde_json::json!("abstain");
        response["selected_action"] = serde_json::json!("abstain_and_escalate");
        response["alternative_actions"] = serde_json::json!([]);
        let issues = validate_symthaea_advice_for_request(
            &request_json,
            &serde_json::to_string(&response).unwrap(),
        ).unwrap();
        assert!(issues.is_empty(), "abstention should pass: {issues:#?}");
    }

    #[test]
    fn unresolved_assessment_policy_blocks_ai_even_if_option_is_online() {
        let mut request: Value = serde_json::from_str(REQUEST_FIXTURE).unwrap();
        request["assessment_policy"] = serde_json::json!("unknown");
        request["support_options"][0]["policy_permitted"] = Value::Bool(false);
        let request_json = serde_json::to_string(&request).unwrap();
        let response = valid_response_json(&request_json);
        let issues = validate_symthaea_advice_for_request(&request_json, &response).unwrap();
        assert!(has_code(&issues, "unresolved_assessment_policy_for_ai_support"));
    }

    #[test]
    fn human_support_preservation_flag_is_authoritative_even_without_material_risk_label() {
        let mut request: Value = serde_json::from_str(REQUEST_FIXTURE).unwrap();
        request["human_support_must_be_preserved"] = Value::Bool(true);
        request["displacement_risk"] = serde_json::json!("low");
        request["support_options"][2]["available"] = Value::Bool(true);
        let request_json = serde_json::to_string(&request).unwrap();
        let response = valid_response_json(&request_json);
        let issues = validate_symthaea_advice_for_request(&request_json, &response).unwrap();
        assert!(has_code(&issues, "protected_human_support_displacement"));
        assert!(has_code(&issues, "human_support_not_preserved"));
    }

    #[test]
    fn inaccessible_support_is_rejected_when_accessibility_is_required() {
        let mut request: Value = serde_json::from_str(REQUEST_FIXTURE).unwrap();
        request["accessibility_requirements"] = serde_json::json!(["screen_reader_compatible_representation"]);
        request["support_options"][0]["accessible_for_context"] = Value::Bool(false);
        let request_json = serde_json::to_string(&request).unwrap();
        let response = valid_response_json(&request_json);
        let issues = validate_symthaea_advice_for_request(&request_json, &response).unwrap();
        assert!(has_code(&issues, "inaccessible_support_selected"));
    }

    #[test]
    fn material_protected_support_cannot_be_replaced_by_ai_only() {
        let mut request: Value = serde_json::from_str(REQUEST_FIXTURE).unwrap();
        request["displacement_risk"] = serde_json::json!("material");
        request["human_support_must_be_preserved"] = Value::Bool(true);
        request["support_options"][2]["available"] = Value::Bool(true);
        let request_json = serde_json::to_string(&request).unwrap();
        let response = valid_response_json(&request_json);
        let issues = validate_symthaea_advice_for_request(&request_json, &response).unwrap();
        assert!(has_code(&issues, "protected_human_support_displacement"));
        assert!(has_code(&issues, "human_support_not_preserved"));
    }

    #[test]
    fn insufficient_evidence_requires_clarification_or_abstention() {
        let mut request: Value = serde_json::from_str(REQUEST_FIXTURE).unwrap();
        request["evidence_sufficiency"] = serde_json::json!("insufficient");
        let request_json = serde_json::to_string(&request).unwrap();
        let response = valid_response_json(&request_json);
        let issues = validate_symthaea_advice_for_request(&request_json, &response).unwrap();
        assert!(has_code(&issues, "insufficient_evidence_requires_clarification"));
    }

    #[test]
    fn unavailable_alternative_is_rejected_too() {
        let mut response: Value = serde_json::from_str(&valid_response_json(REQUEST_FIXTURE)).unwrap();
        response["alternative_actions"] = serde_json::json!(["peer_collaboration"]);
        let issues = validate_symthaea_advice_for_request(
            REQUEST_FIXTURE,
            &serde_json::to_string(&response).unwrap(),
        ).unwrap();
        assert!(has_code(&issues, "action_not_in_request"));
    }

    #[test]
    fn advisor_can_safely_abstain_when_no_accessible_option_is_available() {
        let mut request: Value = serde_json::from_str(REQUEST_FIXTURE).unwrap();
        request["accessibility_requirements"] = serde_json::json!(["screen_reader_compatible_representation"]);
        for option in request["support_options"].as_array_mut().unwrap() {
            option["accessible_for_context"] = Value::Bool(false);
        }
        let request_json = serde_json::to_string(&request).unwrap();
        let mut response: Value = serde_json::from_str(&valid_response_json(&request_json)).unwrap();
        response["disposition"] = serde_json::json!("abstain");
        response["selected_action"] = serde_json::json!("abstain_and_escalate");
        response["alternative_actions"] = serde_json::json!([]);
        let issues = validate_symthaea_advice_for_request(
            &request_json,
            &serde_json::to_string(&response).unwrap(),
        ).unwrap();
        assert!(issues.is_empty(), "safe abstention should be accepted: {issues:#?}");
    }

    #[test]
    fn clarification_is_allowed_when_accessible_support_is_not_yet_known() {
        let mut request: Value = serde_json::from_str(REQUEST_FIXTURE).unwrap();
        request["accessibility_requirements"] = serde_json::json!(["screen_reader_compatible_representation"]);
        for option in request["support_options"].as_array_mut().unwrap() {
            option["accessible_for_context"] = Value::Bool(false);
        }
        let request_json = serde_json::to_string(&request).unwrap();
        let mut response: Value = serde_json::from_str(&valid_response_json(&request_json)).unwrap();
        response["disposition"] = serde_json::json!("clarify");
        response["selected_action"] = serde_json::json!("ask_clarifying_question");
        response["alternative_actions"] = serde_json::json!([]);
        response["clarifying_questions"] = serde_json::json!(["Which accessible format works for this activity?"]);
        let issues = validate_symthaea_advice_for_request(
            &request_json,
            &serde_json::to_string(&response).unwrap(),
        ).unwrap();
        assert!(issues.is_empty(), "clarification should be allowed: {issues:#?}");
    }

    #[test]
    fn no_op_cannot_silently_skip_accessibility_requirements() {
        let mut request: Value = serde_json::from_str(REQUEST_FIXTURE).unwrap();
        request["accessibility_requirements"] = serde_json::json!(["screen_reader_compatible_representation"]);
        let request_json = serde_json::to_string(&request).unwrap();
        let mut response: Value = serde_json::from_str(&valid_response_json(&request_json)).unwrap();
        response["selected_action"] = serde_json::json!("no_intervention");
        response["alternative_actions"] = serde_json::json!([]);
        let issues = validate_symthaea_advice_for_request(
            &request_json,
            &serde_json::to_string(&response).unwrap(),
        ).unwrap();
        assert!(has_code(&issues, "accessibility_needs_unresolved"));
    }

    #[test]
    fn unknown_fields_fail_closed() {
        let mut request: Value = serde_json::from_str(REQUEST_FIXTURE).unwrap();
        request["unexpected_authority"] = Value::Bool(true);
        assert!(parse_symthaea_advisor_request(&request.to_string()).is_err());
    }
}
