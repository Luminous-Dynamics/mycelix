// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Fail-closed contract for optional Symthaea learning advice.
//!
//! This module deliberately validates *suggestions*, not truth, mastery, or
//! educational effectiveness. An accepted value is still advisory: the caller
//! must render it as an optional choice and must not execute it automatically.

use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;

/// Version of the JSON-compatible advisory response contract.
pub const SYMTHAEA_ADVISORY_SCHEMA_VERSION: u16 = 1;
/// Bound on model-supplied evidence references in one response.
pub const MAX_SUPPORTING_CONTENT_IDS: usize = 8;
/// Bound on identifiers accepted from an advisory response.
pub const MAX_ADVISORY_IDENTIFIER_CHARS: usize = 160;
/// Hard cap on the complete untrusted JSON response, checked before parsing.
/// The finite schema fits comfortably within this bound even at identifier maxima.
pub const MAX_ADVISORY_RESPONSE_BYTES: usize = 16384;

/// A deliberately small action set. These are invitations, never commands.
///
/// No variant can change a grade, mastery estimate, credential, privacy choice,
/// or learner's curriculum enrollment.
#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum LearningSuggestion {
    OfferHint,
    OfferWorkedExample,
    OfferAlternateExplanation,
    InviteRetrievalPractice,
    InviteSpacedReview,
    AskForSelfExplanation,
    OfferTransferProblem,
    SuggestCraftProject,
    OfferBreak,
    NoSuggestion,
}

/// Finite reason codes let the UI localize explanations without trusting
/// arbitrary model-generated rationale text.
#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum AdviceReason {
    LearnerRequestedHelp,
    RepeatedIncorrectAttempts,
    ReviewDue,
    TransferNotDemonstrated,
    ReadyForApplication,
    LearnerRequestedBreak,
    InsufficientEvidence,
    NoChange,
}

/// Untrusted response received from a configured Symthaea adapter.
#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct SymthaeaAdvice {
    pub schema_version: u16,
    pub model_id: String,
    pub model_version: String,
    pub skill_id: String,
    pub curriculum_revision: String,
    /// Digest of the exact, minimized request assembled by the trusted caller.
    /// A digest is freshness binding, not proof of model identity or truth.
    pub input_digest: String,
    pub action: LearningSuggestion,
    /// Self-reported model confidence (0..=1000); not a calibrated probability.
    pub reported_confidence_permille: u16,
    pub reason: AdviceReason,
    /// Must refer only to curriculum/content identifiers supplied as allowable
    /// context by Praxis. These identifiers are not automatically citations to
    /// evidence for a factual claim.
    pub supporting_content_ids: Vec<String>,
}

/// Trusted, request-local values against which an untrusted response is checked.
pub struct AdviceValidationContext<'a> {
    pub advisor_enabled: bool,
    pub sandbox_active: bool,
    pub expected_model_id: &'a str,
    pub expected_model_version: &'a str,
    pub skill_id: &'a str,
    pub curriculum_revision: &'a str,
    pub input_digest: &'a str,
    pub allowed_content_ids: &'a BTreeSet<String>,
    /// Actions eligible under trusted Praxis policy for this exact turn.
    /// The model may only choose from this set; it cannot authorize itself.
    pub allowed_actions: &'a [LearningSuggestion],
    /// Reason codes supported by the trusted evidence/request context.
    /// This prevents the model from fabricating claims such as "learner asked".
    pub allowed_reasons: &'a [AdviceReason],
}

/// Why a candidate was rejected. Rejections are safe to surface as diagnostics.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AdviceRejection {
    AdvisorDisabled,
    SandboxActive,
    UnsupportedSchema,
    InvalidIdentifier,
    UnexpectedModel,
    UnexpectedModelVersion,
    SkillMismatch,
    CurriculumRevisionMismatch,
    StaleInput,
    InvalidInputDigest,
    ActionNotAllowed,
    ReasonNotAllowed,
    InvalidConfidence,
    TooManyContentReferences,
    DuplicateContentReference,
    UnknownContentReference,
    MissingGrounding,
    InvalidActionReasonPair,
}

/// Rejections that happen while decoding the untrusted wire response.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AdviceWireRejection {
    /// Response exceeded the hard byte limit and was not deserialized.
    ResponseTooLarge,
    /// Response was not a valid instance of the strict JSON response type.
    MalformedJson,
}

/// Parse the untrusted JSON response only after enforcing its raw byte bound.
///
/// This guard must run before JSON deserialization to limit parser work and
/// allocations attributable to an oversized response. It is not a transport
/// timeout, process authentication mechanism, or total-memory limit.
pub fn parse_symthaea_advice_response(
    response_bytes: &[u8],
) -> Result<SymthaeaAdvice, AdviceWireRejection> {
    if response_bytes.len() > MAX_ADVISORY_RESPONSE_BYTES {
        return Err(AdviceWireRejection::ResponseTooLarge);
    }

    serde_json::from_slice(response_bytes).map_err(|_| AdviceWireRejection::MalformedJson)
}

/// Contract-validated suggestion. This is not an authorization token and does
/// not prove the recommendation is correct or useful.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ValidatedAdvisorySuggestion {
    pub model_id: String,
    pub model_version: String,
    pub skill_id: String,
    pub curriculum_revision: String,
    pub input_digest: String,
    pub action: LearningSuggestion,
    pub reported_confidence_permille: u16,
    pub reason: AdviceReason,
    pub supporting_content_ids: Vec<String>,
}

/// Validate a candidate against trusted, request-local context.
///
/// The caller remains responsible for:
/// - bounding the raw response size *before deserialization*;
/// - authenticating the adapter/process that supplied the response;
/// - checking the request is still current immediately before display;
/// - keeping learner consent, mastery, credentials, and execution authority
///   outside the model;
/// - allowing the learner to accept or dismiss the suggestion.
pub fn validate_symthaea_advice(
    candidate: SymthaeaAdvice,
    context: &AdviceValidationContext<'_>,
) -> Result<ValidatedAdvisorySuggestion, AdviceRejection> {
    if !context.advisor_enabled {
        return Err(AdviceRejection::AdvisorDisabled);
    }
    if context.sandbox_active {
        return Err(AdviceRejection::SandboxActive);
    }
    if candidate.schema_version != SYMTHAEA_ADVISORY_SCHEMA_VERSION {
        return Err(AdviceRejection::UnsupportedSchema);
    }
    if !valid_identifier(&candidate.model_id)
        || !valid_identifier(&candidate.model_version)
        || !valid_identifier(&candidate.skill_id)
        || !valid_identifier(&candidate.curriculum_revision)
        || candidate
            .supporting_content_ids
            .iter()
            .any(|value| !valid_identifier(value))
    {
        return Err(AdviceRejection::InvalidIdentifier);
    }
    if !valid_blake3_digest(&candidate.input_digest) {
        return Err(AdviceRejection::InvalidInputDigest);
    }
    if candidate.model_id != context.expected_model_id {
        return Err(AdviceRejection::UnexpectedModel);
    }
    if candidate.model_version != context.expected_model_version {
        return Err(AdviceRejection::UnexpectedModelVersion);
    }
    if candidate.skill_id != context.skill_id {
        return Err(AdviceRejection::SkillMismatch);
    }
    if candidate.curriculum_revision != context.curriculum_revision {
        return Err(AdviceRejection::CurriculumRevisionMismatch);
    }
    if candidate.input_digest != context.input_digest {
        return Err(AdviceRejection::StaleInput);
    }
    if !context.allowed_actions.contains(&candidate.action) {
        return Err(AdviceRejection::ActionNotAllowed);
    }
    if !context.allowed_reasons.contains(&candidate.reason) {
        return Err(AdviceRejection::ReasonNotAllowed);
    }
    if candidate.reported_confidence_permille > 1000 {
        return Err(AdviceRejection::InvalidConfidence);
    }
    if candidate.supporting_content_ids.len() > MAX_SUPPORTING_CONTENT_IDS {
        return Err(AdviceRejection::TooManyContentReferences);
    }

    let mut seen = BTreeSet::new();
    for content_id in &candidate.supporting_content_ids {
        if !seen.insert(content_id) {
            return Err(AdviceRejection::DuplicateContentReference);
        }
        if !context.allowed_content_ids.contains(content_id) {
            return Err(AdviceRejection::UnknownContentReference);
        }
    }

    let grounding_exempt = matches!(
        candidate.action,
        LearningSuggestion::OfferBreak | LearningSuggestion::NoSuggestion
    );
    if !grounding_exempt && candidate.supporting_content_ids.is_empty() {
        return Err(AdviceRejection::MissingGrounding);
    }
    if !action_reason_pair_is_valid(candidate.action, candidate.reason) {
        return Err(AdviceRejection::InvalidActionReasonPair);
    }

    Ok(ValidatedAdvisorySuggestion {
        model_id: candidate.model_id,
        model_version: candidate.model_version,
        skill_id: candidate.skill_id,
        curriculum_revision: candidate.curriculum_revision,
        input_digest: candidate.input_digest,
        action: candidate.action,
        reported_confidence_permille: candidate.reported_confidence_permille,
        reason: candidate.reason,
        supporting_content_ids: candidate.supporting_content_ids,
    })
}

/// Bind advice to the exact request bytes sent to the advisor.
///
/// The caller must finish serialization and policy filtering *before* calling
/// this helper, then send those same bytes to the configured adapter. This
/// function hashes bytes; it does not make a non-canonical serialization
/// canonical, authenticate the model, or prove that a response is truthful.
pub fn digest_advisory_request_bytes(canonical_request_bytes: &[u8]) -> String {
    format!("blake3:{}", blake3::hash(canonical_request_bytes).to_hex())
}

fn valid_blake3_digest(value: &str) -> bool {
    let Some(hex) = value.strip_prefix("blake3:") else {
        return false;
    };
    hex.len() == 64
        && hex
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}

fn valid_identifier(value: &str) -> bool {
    !value.is_empty()
        && value.trim() == value
        && !value.chars().any(char::is_control)
        && value.chars().count() <= MAX_ADVISORY_IDENTIFIER_CHARS
}

fn action_reason_pair_is_valid(action: LearningSuggestion, reason: AdviceReason) -> bool {
    use AdviceReason as R;
    use LearningSuggestion as A;

    match action {
        A::OfferHint | A::OfferWorkedExample | A::OfferAlternateExplanation => {
            matches!(reason, R::LearnerRequestedHelp | R::RepeatedIncorrectAttempts)
        }
        A::InviteRetrievalPractice | A::InviteSpacedReview => {
            matches!(reason, R::ReviewDue | R::RepeatedIncorrectAttempts)
        }
        A::AskForSelfExplanation | A::OfferTransferProblem => {
            matches!(reason, R::TransferNotDemonstrated | R::ReadyForApplication)
        }
        A::SuggestCraftProject => reason == R::ReadyForApplication,
        // A break is only surfaced when the learner asked for one. Inferred
        // affect or a model score must never force a break.
        A::OfferBreak => reason == R::LearnerRequestedBreak,
        A::NoSuggestion => matches!(reason, R::NoChange | R::InsufficientEvidence),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn allowed_content() -> BTreeSet<String> {
        ["lesson.fractions.01".to_string()].into_iter().collect()
    }

    const DEFAULT_ALLOWED_ACTIONS: &[LearningSuggestion] = &[
        LearningSuggestion::OfferHint,
        LearningSuggestion::NoSuggestion,
    ];
    const DEFAULT_ALLOWED_REASONS: &[AdviceReason] = &[
        AdviceReason::RepeatedIncorrectAttempts,
        AdviceReason::InsufficientEvidence,
    ];
    const BREAK_ALLOWED_ACTIONS: &[LearningSuggestion] = &[
        LearningSuggestion::OfferBreak,
        LearningSuggestion::NoSuggestion,
    ];
    const BREAK_ALLOWED_REASONS: &[AdviceReason] = &[
        AdviceReason::LearnerRequestedBreak,
        AdviceReason::InsufficientEvidence,
    ];

    fn context<'a>(
        advisor_enabled: bool,
        sandbox_active: bool,
        allowed_content_ids: &'a BTreeSet<String>,
    ) -> AdviceValidationContext<'a> {
        AdviceValidationContext {
            advisor_enabled,
            sandbox_active,
            expected_model_id: "symthaea-advisor",
            expected_model_version: "0.1.0",
            skill_id: "math.fractions",
            curriculum_revision: "caps-2026.1",
            input_digest: "blake3:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
            allowed_content_ids,
            allowed_actions: DEFAULT_ALLOWED_ACTIONS,
            allowed_reasons: DEFAULT_ALLOWED_REASONS,
        }
    }

    fn context_with_break_requested<'a>(
        allowed_content_ids: &'a BTreeSet<String>,
    ) -> AdviceValidationContext<'a> {
        AdviceValidationContext {
            advisor_enabled: true,
            sandbox_active: false,
            expected_model_id: "symthaea-advisor",
            expected_model_version: "0.1.0",
            skill_id: "math.fractions",
            curriculum_revision: "caps-2026.1",
            input_digest: "blake3:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
            allowed_content_ids,
            allowed_actions: BREAK_ALLOWED_ACTIONS,
            allowed_reasons: BREAK_ALLOWED_REASONS,
        }
    }

    fn candidate() -> SymthaeaAdvice {
        SymthaeaAdvice {
            schema_version: SYMTHAEA_ADVISORY_SCHEMA_VERSION,
            model_id: "symthaea-advisor".into(),
            model_version: "0.1.0".into(),
            skill_id: "math.fractions".into(),
            curriculum_revision: "caps-2026.1".into(),
            input_digest: "blake3:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa".into(),
            action: LearningSuggestion::OfferHint,
            reported_confidence_permille: 720,
            reason: AdviceReason::RepeatedIncorrectAttempts,
            supporting_content_ids: vec!["lesson.fractions.01".into()],
        }
    }

    #[test]
    fn accepts_scoped_advisory_suggestion() {
        let ids = allowed_content();
        let result = validate_symthaea_advice(candidate(), &context(true, false, &ids));
        let accepted = result.expect("valid advisory response should be accepted");
        assert_eq!(accepted.action, LearningSuggestion::OfferHint);
        assert_eq!(accepted.skill_id, "math.fractions");
        assert_eq!(accepted.supporting_content_ids, vec!["lesson.fractions.01".to_string()]);
    }

    #[test]
    fn disabled_advisor_is_rejected() {
        let ids = allowed_content();
        assert_eq!(
            validate_symthaea_advice(candidate(), &context(false, false, &ids)),
            Err(AdviceRejection::AdvisorDisabled)
        );
    }

    #[test]
    fn sandbox_suppresses_model_adaptivity() {
        let ids = allowed_content();
        assert_eq!(
            validate_symthaea_advice(candidate(), &context(true, true, &ids)),
            Err(AdviceRejection::SandboxActive)
        );
    }

    #[test]
    fn stale_request_digest_is_rejected() {
        let ids = allowed_content();
        let mut advice = candidate();
        advice.input_digest = "blake3:bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb".into();
        assert_eq!(
            validate_symthaea_advice(advice, &context(true, false, &ids)),
            Err(AdviceRejection::StaleInput)
        );
    }

    #[test]
    fn request_digest_matches_blake3_empty_input_golden_vector() {
        assert_eq!(
            digest_advisory_request_bytes(b""),
            "blake3:af1349b9f5f9a1a6a0404dea36dcc9499bcb25c9adc112b7cc9a93cae41f3262"
        );
    }

    #[test]
    fn request_digest_matches_blake3_nonempty_golden_vector() {
        assert_eq!(
            digest_advisory_request_bytes(b"abc"),
            "blake3:6437b3ac38465133ffb63b75273a8db548c558465d79db03fd359c6cd5bd9d85"
        );
    }

    #[test]
    fn malformed_input_digests_are_rejected() {
        let ids = allowed_content();
        for digest in [
            "sha256:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
            "blake3:short",
            "blake3:AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA",
        ] {
            let mut advice = candidate();
            advice.input_digest = digest.into();
            assert_eq!(
                validate_symthaea_advice(advice, &context(true, false, &ids)),
                Err(AdviceRejection::InvalidInputDigest)
            );
        }
    }

    #[test]
    fn a_different_skill_cannot_receive_the_advice() {
        let ids = allowed_content();
        let mut advice = candidate();
        advice.skill_id = "math.algebra".into();
        assert_eq!(
            validate_symthaea_advice(advice, &context(true, false, &ids)),
            Err(AdviceRejection::SkillMismatch)
        );
    }

    #[test]
    fn untrusted_content_reference_is_rejected() {
        let ids = allowed_content();
        let mut advice = candidate();
        advice.supporting_content_ids = vec!["invented.lesson".into()];
        assert_eq!(
            validate_symthaea_advice(advice, &context(true, false, &ids)),
            Err(AdviceRejection::UnknownContentReference)
        );
    }

    #[test]
    fn unsupported_confidence_range_is_rejected() {
        let ids = allowed_content();
        let mut advice = candidate();
        advice.reported_confidence_permille = 1001;
        assert_eq!(
            validate_symthaea_advice(advice, &context(true, false, &ids)),
            Err(AdviceRejection::InvalidConfidence)
        );
    }

    #[test]
    fn suggestions_need_trusted_content_grounding() {
        let ids = allowed_content();
        let mut advice = candidate();
        advice.supporting_content_ids.clear();
        assert_eq!(
            validate_symthaea_advice(advice, &context(true, false, &ids)),
            Err(AdviceRejection::MissingGrounding)
        );
    }

    #[test]
    fn a_model_cannot_order_a_break_even_by_claiming_the_learner_requested_it() {
        let ids = allowed_content();
        let mut advice = candidate();
        advice.action = LearningSuggestion::OfferBreak;
        advice.reason = AdviceReason::LearnerRequestedBreak;
        advice.supporting_content_ids.clear();
        assert_eq!(
            validate_symthaea_advice(advice, &context(true, false, &ids)),
            Err(AdviceRejection::ActionNotAllowed)
        );
    }

    #[test]
    fn model_cannot_fabricate_a_learner_requested_reason() {
        let ids = allowed_content();
        let mut advice = candidate();
        advice.reason = AdviceReason::LearnerRequestedBreak;
        assert_eq!(
            validate_symthaea_advice(advice, &context(true, false, &ids)),
            Err(AdviceRejection::ReasonNotAllowed)
        );
    }

    #[test]
    fn break_is_allowed_only_when_trusted_context_enables_it() {
        let ids = allowed_content();
        let mut advice = candidate();
        advice.action = LearningSuggestion::OfferBreak;
        advice.reason = AdviceReason::LearnerRequestedBreak;
        advice.supporting_content_ids.clear();
        let accepted = validate_symthaea_advice(
            advice,
            &context_with_break_requested(&ids),
        )
        .expect("trusted learner-request state can enable an optional break");
        assert_eq!(accepted.action, LearningSuggestion::OfferBreak);
    }

    #[test]
    fn action_and_reason_must_be_semantically_compatible() {
        const REVIEW_DUE_ONLY: &[AdviceReason] = &[AdviceReason::ReviewDue];
        let ids = allowed_content();
        let mut advice = candidate();
        advice.reason = AdviceReason::ReviewDue;
        let mut validation = context(true, false, &ids);
        validation.allowed_reasons = REVIEW_DUE_ONLY;
        assert_eq!(
            validate_symthaea_advice(advice, &validation),
            Err(AdviceRejection::InvalidActionReasonPair)
        );
    }

    #[test]
    fn no_suggestion_is_allowed_when_evidence_is_insufficient() {
        let ids = allowed_content();
        let mut advice = candidate();
        advice.action = LearningSuggestion::NoSuggestion;
        advice.reason = AdviceReason::InsufficientEvidence;
        advice.supporting_content_ids.clear();
        let accepted = validate_symthaea_advice(advice, &context(true, false, &ids))
            .expect("no-op advice should be accepted");
        assert_eq!(accepted.action, LearningSuggestion::NoSuggestion);
    }

    #[test]
    fn rejects_unknown_schema_and_model_version() {
        let ids = allowed_content();
        let mut advice = candidate();
        advice.schema_version = 2;
        assert_eq!(
            validate_symthaea_advice(advice, &context(true, false, &ids)),
            Err(AdviceRejection::UnsupportedSchema)
        );

        let mut advice = candidate();
        advice.model_version = "0.2.0".into();
        assert_eq!(
            validate_symthaea_advice(advice, &context(true, false, &ids)),
            Err(AdviceRejection::UnexpectedModelVersion)
        );
    }

    #[test]
    fn a_different_model_cannot_submit_advice() {
        let ids = allowed_content();
        let mut advice = candidate();
        advice.model_id = "unconfigured-model".into();
        assert_eq!(
            validate_symthaea_advice(advice, &context(true, false, &ids)),
            Err(AdviceRejection::UnexpectedModel)
        );
    }

    #[test]
    fn an_old_curriculum_revision_is_rejected() {
        let ids = allowed_content();
        let mut advice = candidate();
        advice.curriculum_revision = "caps-2025.1".into();
        assert_eq!(
            validate_symthaea_advice(advice, &context(true, false, &ids)),
            Err(AdviceRejection::CurriculumRevisionMismatch)
        );
    }

    #[test]
    fn oversized_reference_lists_are_rejected_before_reference_lookup() {
        let ids = allowed_content();
        let mut advice = candidate();
        advice.supporting_content_ids = (0..=MAX_SUPPORTING_CONTENT_IDS)
            .map(|index| format!("lesson-{index}"))
            .collect();
        assert_eq!(
            validate_symthaea_advice(advice, &context(true, false, &ids)),
            Err(AdviceRejection::TooManyContentReferences)
        );
    }

    #[test]
    fn identifiers_with_control_characters_are_rejected() {
        let ids = allowed_content();
        let mut advice = candidate();
        advice.skill_id = "math.fractions\nforged-log-line".into();
        assert_eq!(
            validate_symthaea_advice(advice, &context(true, false, &ids)),
            Err(AdviceRejection::InvalidIdentifier)
        );
    }

    #[test]
    fn bounded_wire_parser_accepts_a_valid_contract_response() {
        let advice = candidate();
        let encoded = serde_json::to_vec(&advice).expect("candidate serializes");
        let parsed = parse_symthaea_advice_response(&encoded).expect("valid response parses");
        assert_eq!(parsed, advice);
    }

    #[test]
    fn bounded_wire_parser_rejects_oversized_response_before_json_parsing() {
        let encoded = vec![b' '; MAX_ADVISORY_RESPONSE_BYTES + 1];
        assert_eq!(
            parse_symthaea_advice_response(&encoded),
            Err(AdviceWireRejection::ResponseTooLarge)
        );
    }

    #[test]
    fn bounded_wire_parser_accepts_limit_length_for_parsing() {
        // Exact-limit bytes pass the size gate; JSON validity is checked next.
        let encoded = vec![b' '; MAX_ADVISORY_RESPONSE_BYTES];
        assert_eq!(
            parse_symthaea_advice_response(&encoded),
            Err(AdviceWireRejection::MalformedJson)
        );
    }

    #[test]
    fn bounded_wire_parser_rejects_malformed_json() {
        assert_eq!(
            parse_symthaea_advice_response(b"{"),
            Err(AdviceWireRejection::MalformedJson)
        );
    }

    #[test]
    fn wire_contract_rejects_unknown_fields() {
        let mut value = serde_json::to_value(candidate()).expect("candidate serializes");
        value
            .as_object_mut()
            .expect("serialized struct is object")
            .insert("can_issue_credential".into(), serde_json::Value::Bool(true));
        let encoded = serde_json::to_vec(&value).expect("value serializes");
        assert_eq!(
            parse_symthaea_advice_response(&encoded),
            Err(AdviceWireRejection::MalformedJson)
        );
    }

    #[test]
    fn duplicate_references_are_rejected_deterministically() {
        let ids = allowed_content();
        let mut advice = candidate();
        advice.supporting_content_ids = vec![
            "lesson.fractions.01".into(),
            "lesson.fractions.01".into(),
        ];
        assert_eq!(
            validate_symthaea_advice(advice, &context(true, false, &ids)),
            Err(AdviceRejection::DuplicateContentReference)
        );
    }
}
