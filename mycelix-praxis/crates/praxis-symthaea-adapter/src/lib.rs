// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Host-only, size-bounded decoding primitives for the optional Symthaea advisor.
//!
//! This crate intentionally has no Symthaea runtime or network-client dependency.
//! A host transport supplies a reader with its own authentication and timeout
//! policy. This module caps bytes read before parsing and delegates strict JSON
//! decoding to `praxis-core`; callers must still run semantic validation with
//! trusted request-local context.

#![forbid(unsafe_code)]

#[cfg(target_family = "wasm")]
compile_error!(
    "praxis-symthaea-adapter is host-only and must not be compiled into WASM guest zomes"
);

use std::{
    collections::BTreeSet,
    io::{self, Read},
};

use praxis_core::{
    AdviceReason, AdviceRejection, AdviceValidationContext, AdviceWireRejection,
    LearningSuggestion, MAX_ADVISORY_IDENTIFIER_CHARS, MAX_ADVISORY_RESPONSE_BYTES,
    MAX_SUPPORTING_CONTENT_IDS, SYMTHAEA_ADVISORY_SCHEMA_VERSION, SymthaeaAdvice,
    ValidatedAdvisorySuggestion, digest_advisory_request_bytes, parse_symthaea_advice_response,
    validate_symthaea_advice,
};
use serde::Serialize;

/// Hard cap for the exact serialized request body sent to the host adapter.
pub const MAX_ADVISORY_REQUEST_BYTES: usize = 16384;
/// Maximum number of distinct finite actions a request can allow.
pub const MAX_ALLOWED_ACTIONS: usize = 10;
/// Maximum number of distinct finite reason codes a request can allow.
pub const MAX_ALLOWED_REASONS: usize = 8;

/// Trusted inputs from which a minimized, deterministic advisor request is built.
///
/// Do not place raw answer histories, learner identity, demographics, or inferred
/// affect in this structure. The allowed reason codes are the trusted signals
/// permitted for this turn, not model-generated observations.
pub struct AdvisoryRequestPolicy<'a> {
    pub model_id: &'a str,
    pub model_version: &'a str,
    pub skill_id: &'a str,
    pub curriculum_revision: &'a str,
    pub allowed_actions: &'a [LearningSuggestion],
    pub allowed_reasons: &'a [AdviceReason],
    pub allowed_content_ids: &'a [String],
}

/// Why a trusted request could not be prepared.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RequestPreparationError {
    InvalidIdentifier,
    TooManyContentReferences,
    DuplicateContentReference,
    EmptyActionAllowlist,
    TooManyAllowedActions,
    DuplicateAllowedAction,
    EmptyReasonAllowlist,
    TooManyAllowedReasons,
    DuplicateAllowedReason,
    SerializationFailed,
    RequestTooLarge,
}

#[derive(Serialize)]
struct AdvisoryRequestWireV1<'a> {
    schema_version: u16,
    model_id: &'a str,
    model_version: &'a str,
    skill_id: &'a str,
    curriculum_revision: &'a str,
    allowed_actions: &'a [LearningSuggestion],
    allowed_reasons: &'a [AdviceReason],
    allowed_content_ids: &'a [String],
}

/// Immutable prepared body and its trusted validation scope.
///
/// The digest is computed over `body_bytes()` exactly. Send those bytes
/// unchanged as the request body, and convey `input_digest()` through
/// authenticated transport metadata (for example, a request header) rather
/// than adding a self-referential digest field to the body.
pub struct PreparedAdvisoryRequest {
    body: Vec<u8>,
    input_digest: String,
    model_id: String,
    model_version: String,
    skill_id: String,
    curriculum_revision: String,
    allowed_actions: Vec<LearningSuggestion>,
    allowed_reasons: Vec<AdviceReason>,
    allowed_content_ids: BTreeSet<String>,
}

impl PreparedAdvisoryRequest {
    /// Exact bytes that must be sent unchanged as the request body.
    pub fn body_bytes(&self) -> &[u8] {
        &self.body
    }

    /// BLAKE3 digest of the exact request body bytes, suitable for transport
    /// metadata. It is freshness binding only, not authentication.
    pub fn input_digest(&self) -> &str {
        &self.input_digest
    }

    /// Build response-validation context from the very same frozen request
    /// scope so callers do not accidentally validate against a different turn.
    pub fn validation_context(
        &self,
        advisor_enabled: bool,
        sandbox_active: bool,
    ) -> AdviceValidationContext<'_> {
        AdviceValidationContext {
            advisor_enabled,
            sandbox_active,
            expected_model_id: &self.model_id,
            expected_model_version: &self.model_version,
            skill_id: &self.skill_id,
            curriculum_revision: &self.curriculum_revision,
            input_digest: &self.input_digest,
            allowed_content_ids: &self.allowed_content_ids,
            allowed_actions: &self.allowed_actions,
            allowed_reasons: &self.allowed_reasons,
        }
    }
}

/// Prepare a deterministic, minimized request body and freeze the exact policy
/// scope used to validate its response.
///
/// Enum lists are emitted in declared protocol order and content IDs are sorted
/// lexically. Duplicate trusted inputs are rejected rather than silently
/// normalized. The request digest is computed only after final serialization.
pub fn prepare_advisory_request(
    policy: AdvisoryRequestPolicy<'_>,
) -> Result<PreparedAdvisoryRequest, RequestPreparationError> {
    for identifier in [
        policy.model_id,
        policy.model_version,
        policy.skill_id,
        policy.curriculum_revision,
    ] {
        if !valid_request_identifier(identifier) {
            return Err(RequestPreparationError::InvalidIdentifier);
        }
    }

    if policy.allowed_content_ids.len() > MAX_SUPPORTING_CONTENT_IDS {
        return Err(RequestPreparationError::TooManyContentReferences);
    }
    let mut content_ids = policy.allowed_content_ids.to_vec();
    content_ids.sort();
    if content_ids.windows(2).any(|pair| pair[0] == pair[1]) {
        return Err(RequestPreparationError::DuplicateContentReference);
    }
    if content_ids.iter().any(|value| !valid_request_identifier(value)) {
        return Err(RequestPreparationError::InvalidIdentifier);
    }

    if policy.allowed_actions.is_empty() {
        return Err(RequestPreparationError::EmptyActionAllowlist);
    }
    if policy.allowed_actions.len() > MAX_ALLOWED_ACTIONS {
        return Err(RequestPreparationError::TooManyAllowedActions);
    }
    let mut allowed_actions = policy.allowed_actions.to_vec();
    allowed_actions.sort_by_key(action_rank);
    if allowed_actions.windows(2).any(|pair| pair[0] == pair[1]) {
        return Err(RequestPreparationError::DuplicateAllowedAction);
    }

    if policy.allowed_reasons.is_empty() {
        return Err(RequestPreparationError::EmptyReasonAllowlist);
    }
    if policy.allowed_reasons.len() > MAX_ALLOWED_REASONS {
        return Err(RequestPreparationError::TooManyAllowedReasons);
    }
    let mut allowed_reasons = policy.allowed_reasons.to_vec();
    allowed_reasons.sort_by_key(reason_rank);
    if allowed_reasons.windows(2).any(|pair| pair[0] == pair[1]) {
        return Err(RequestPreparationError::DuplicateAllowedReason);
    }

    let wire = AdvisoryRequestWireV1 {
        schema_version: SYMTHAEA_ADVISORY_SCHEMA_VERSION,
        model_id: policy.model_id,
        model_version: policy.model_version,
        skill_id: policy.skill_id,
        curriculum_revision: policy.curriculum_revision,
        allowed_actions: &allowed_actions,
        allowed_reasons: &allowed_reasons,
        allowed_content_ids: &content_ids,
    };
    let body = serde_json::to_vec(&wire).map_err(|_| RequestPreparationError::SerializationFailed)?;
    if body.len() > MAX_ADVISORY_REQUEST_BYTES {
        return Err(RequestPreparationError::RequestTooLarge);
    }
    let input_digest = digest_advisory_request_bytes(&body);

    Ok(PreparedAdvisoryRequest {
        body,
        input_digest,
        model_id: policy.model_id.to_string(),
        model_version: policy.model_version.to_string(),
        skill_id: policy.skill_id.to_string(),
        curriculum_revision: policy.curriculum_revision.to_string(),
        allowed_actions,
        allowed_reasons,
        allowed_content_ids: content_ids.into_iter().collect(),
    })
}

fn valid_request_identifier(value: &str) -> bool {
    !value.is_empty()
        && value.trim() == value
        && value.chars().count() <= MAX_ADVISORY_IDENTIFIER_CHARS
        && !value.chars().any(char::is_control)
}

fn action_rank(action: &LearningSuggestion) -> u8 {
    match action {
        LearningSuggestion::OfferHint => 0,
        LearningSuggestion::OfferWorkedExample => 1,
        LearningSuggestion::OfferAlternateExplanation => 2,
        LearningSuggestion::InviteRetrievalPractice => 3,
        LearningSuggestion::InviteSpacedReview => 4,
        LearningSuggestion::AskForSelfExplanation => 5,
        LearningSuggestion::OfferTransferProblem => 6,
        LearningSuggestion::SuggestCraftProject => 7,
        LearningSuggestion::OfferBreak => 8,
        LearningSuggestion::NoSuggestion => 9,
    }
}

fn reason_rank(reason: &AdviceReason) -> u8 {
    match reason {
        AdviceReason::LearnerRequestedHelp => 0,
        AdviceReason::RepeatedIncorrectAttempts => 1,
        AdviceReason::ReviewDue => 2,
        AdviceReason::TransferNotDemonstrated => 3,
        AdviceReason::ReadyForApplication => 4,
        AdviceReason::LearnerRequestedBreak => 5,
        AdviceReason::InsufficientEvidence => 6,
        AdviceReason::NoChange => 7,
    }
}

/// Failure while reading and decoding a host-provided advisor response.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BoundedAdviceReadError {
    /// The underlying reader failed. The original payload is never accepted.
    ReaderFailed(io::ErrorKind),
    /// More than the configured maximum arrived; only max + 1 bytes were read.
    ResponseTooLarge,
    /// The bounded bytes were not a valid strict `SymthaeaAdvice` JSON object.
    MalformedJson,
    /// Strict decoding succeeded, but trusted request-local policy rejected advice.
    Rejected(AdviceRejection),
}

/// Read and parse one advisory response with a hard pre-buffering byte ceiling.
///
/// The reader is wrapped in `Read::take(MAX_ADVISORY_RESPONSE_BYTES + 1)`, so
/// the function buffers at most one byte beyond the permitted body size. That
/// extra byte is enough to distinguish an exactly-at-limit body from an
/// oversized one. It never reads the remainder of an oversized stream.
///
/// The caller is responsible for configuring transport/process authentication,
/// read timeouts, learner consent, and request freshness. The returned object
/// remains untrusted: call `praxis_core::validate_symthaea_advice` with trusted
/// per-turn context before displaying any suggestion.
fn read_bounded_response_bytes<R: Read>(
    reader: R,
) -> Result<Vec<u8>, BoundedAdviceReadError> {
    let mut bounded = reader.take((MAX_ADVISORY_RESPONSE_BYTES + 1) as u64);
    let mut bytes = Vec::with_capacity(MAX_ADVISORY_RESPONSE_BYTES + 1);
    bounded
        .read_to_end(&mut bytes)
        .map_err(|error| BoundedAdviceReadError::ReaderFailed(error.kind()))?;

    if bytes.len() > MAX_ADVISORY_RESPONSE_BYTES {
        return Err(BoundedAdviceReadError::ResponseTooLarge);
    }

    Ok(bytes)
}

fn parse_bounded_response(bytes: &[u8]) -> Result<SymthaeaAdvice, BoundedAdviceReadError> {
    parse_symthaea_advice_response(bytes).map_err(|error| match error {
        AdviceWireRejection::ResponseTooLarge => BoundedAdviceReadError::ResponseTooLarge,
        AdviceWireRejection::MalformedJson => BoundedAdviceReadError::MalformedJson,
    })
}

/// Read and strictly decode an advisor response, enforcing the streaming byte
/// limit before buffering the entire body. The returned response is still
/// untrusted until semantic validation is applied.
pub fn read_bounded_symthaea_advice<R: Read>(
    reader: R,
) -> Result<SymthaeaAdvice, BoundedAdviceReadError> {
    let bytes = read_bounded_response_bytes(reader)?;
    parse_bounded_response(&bytes)
}

/// Preferred integration path: read at most the response ceiling plus one
/// byte, decode the strict wire contract, then apply trusted request-local
/// validation before returning a value fit to be considered for display.
///
/// This does not authenticate the source or configure the reader's timeout.
pub fn read_bounded_and_validate_symthaea_advice<R: Read>(
    reader: R,
    context: &AdviceValidationContext<'_>,
) -> Result<ValidatedAdvisorySuggestion, BoundedAdviceReadError> {
    let bytes = read_bounded_response_bytes(reader)?;
    let candidate = parse_bounded_response(&bytes)?;
    validate_symthaea_advice(candidate, context).map_err(BoundedAdviceReadError::Rejected)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Cursor;
    use std::sync::{
        Arc,
        atomic::{AtomicUsize, Ordering},
    };

    fn valid_advice_json() -> String {
        serde_json::json!({
            "schema_version": 1,
            "model_id": "symthaea",
            "model_version": "0.5.0",
            "skill_id": "math.fractions",
            "curriculum_revision": "2026.1",
            "input_digest": format!("blake3:{}", "0".repeat(64)),
            "action": "offer_hint",
            "reported_confidence_permille": 500,
            "reason": "repeated_incorrect_attempts",
            "supporting_content_ids": ["lesson.fractions.01"],
        })
        .to_string()
    }

    struct CountingReader {
        remaining: usize,
        observed_bytes: Arc<AtomicUsize>,
    }

    impl Read for CountingReader {
        fn read(&mut self, buffer: &mut [u8]) -> io::Result<usize> {
            let count = buffer.len().min(self.remaining);
            buffer[..count].fill(b' ');
            self.remaining -= count;
            self.observed_bytes.fetch_add(count, Ordering::SeqCst);
            Ok(count)
        }
    }

    struct FailingReader;

    impl Read for FailingReader {
        fn read(&mut self, _buffer: &mut [u8]) -> io::Result<usize> {
            Err(io::Error::new(io::ErrorKind::BrokenPipe, "test reader failure"))
        }
    }

    fn prepare_test_request<'a>(
        allowed_content_ids: &'a [String],
        allowed_actions: &'a [LearningSuggestion],
        allowed_reasons: &'a [AdviceReason],
    ) -> PreparedAdvisoryRequest {
        prepare_advisory_request(AdvisoryRequestPolicy {
            model_id: "symthaea",
            model_version: "0.5.0",
            skill_id: "math.fractions",
            curriculum_revision: "2026.1",
            allowed_actions,
            allowed_reasons,
            allowed_content_ids,
        })
        .expect("test policy produces a valid request")
    }

    #[test]
    fn reads_a_valid_response_from_a_host_reader() {
        let encoded = valid_advice_json();
        let advice = read_bounded_symthaea_advice(Cursor::new(encoded.as_bytes()))
            .expect("valid bounded response parses");
        assert_eq!(advice.model_id, "symthaea");
        assert_eq!(advice.action, praxis_core::LearningSuggestion::OfferHint);
    }

    #[test]
    fn rejects_oversized_stream_after_reading_only_limit_plus_one() {
        let observed_bytes = Arc::new(AtomicUsize::new(0));
        let reader = CountingReader {
            remaining: MAX_ADVISORY_RESPONSE_BYTES * 4,
            observed_bytes: Arc::clone(&observed_bytes),
        };

        assert_eq!(
            read_bounded_symthaea_advice(reader),
            Err(BoundedAdviceReadError::ResponseTooLarge)
        );
        assert_eq!(
            observed_bytes.load(Ordering::SeqCst),
            MAX_ADVISORY_RESPONSE_BYTES + 1,
            "oversized streams must not be fully buffered"
        );
    }

    #[test]
    fn exact_limit_body_reaches_parser_without_crossing_the_size_gate() {
        let reader = Cursor::new(vec![b' '; MAX_ADVISORY_RESPONSE_BYTES]);
        assert_eq!(
            read_bounded_symthaea_advice(reader),
            Err(BoundedAdviceReadError::MalformedJson)
        );
    }

    #[test]
    fn malformed_json_is_rejected() {
        assert_eq!(
            read_bounded_symthaea_advice(Cursor::new(&b"{"[..])),
            Err(BoundedAdviceReadError::MalformedJson)
        );
    }

    #[test]
    fn trusted_validation_path_returns_only_a_validated_suggestion() {
        let ids = ["lesson.fractions.01".to_string()];
        let request = prepare_test_request(
            &ids,
            &[LearningSuggestion::OfferHint],
            &[AdviceReason::RepeatedIncorrectAttempts],
        );
        let context = request.validation_context(true, false);
        let encoded = valid_advice_json().replace(
            &format!("blake3:{}", "0".repeat(64)),
            request.input_digest(),
        );

        let suggestion =
            read_bounded_and_validate_symthaea_advice(Cursor::new(encoded.as_bytes()), &context)
                .expect("in-scope advice passes both wire and semantic validation");
        assert_eq!(suggestion.input_digest, request.input_digest());
        assert_eq!(suggestion.action, LearningSuggestion::OfferHint);
    }

    #[test]
    fn trusted_validation_path_rejects_an_unallowed_action() {
        let ids = ["lesson.fractions.01".to_string()];
        let request = prepare_test_request(
            &ids,
            &[LearningSuggestion::OfferHint],
            &[AdviceReason::RepeatedIncorrectAttempts],
        );
        let context = request.validation_context(true, false);
        let encoded = valid_advice_json()
            .replace("\"offer_hint\"", "\"offer_break\"")
            .replace(
                &format!("blake3:{}", "0".repeat(64)),
                request.input_digest(),
            );

        assert_eq!(
            read_bounded_and_validate_symthaea_advice(
                Cursor::new(encoded.as_bytes()),
                &context
            ),
            Err(BoundedAdviceReadError::Rejected(AdviceRejection::ActionNotAllowed))
        );
    }

    #[test]
    fn request_body_and_digest_are_deterministic_and_bound_to_exact_bytes() {
        let ids_a = ["z.content".to_string(), "a.content".to_string()];
        let ids_b = ["a.content".to_string(), "z.content".to_string()];
        let actions_a = [
            LearningSuggestion::NoSuggestion,
            LearningSuggestion::OfferHint,
        ];
        let actions_b = [
            LearningSuggestion::OfferHint,
            LearningSuggestion::NoSuggestion,
        ];
        let reasons_a = [
            AdviceReason::InsufficientEvidence,
            AdviceReason::RepeatedIncorrectAttempts,
        ];
        let reasons_b = [
            AdviceReason::RepeatedIncorrectAttempts,
            AdviceReason::InsufficientEvidence,
        ];
        let first = prepare_test_request(&ids_a, &actions_a, &reasons_a);
        let second = prepare_test_request(&ids_b, &actions_b, &reasons_b);

        assert_eq!(first.body_bytes(), second.body_bytes());
        assert_eq!(first.input_digest(), second.input_digest());
        assert_eq!(
            first.input_digest(),
            digest_advisory_request_bytes(first.body_bytes())
        );
        let body = std::str::from_utf8(first.body_bytes()).expect("JSON is UTF-8");
        assert!(
            !body.contains("input_digest"),
            "digest metadata must not make the body self-referential"
        );
        assert!(body.find("a.content").unwrap() < body.find("z.content").unwrap());
    }

    #[test]
    fn request_preparation_rejects_duplicate_content_ids() {
        let ids = ["lesson.fractions.01".to_string(), "lesson.fractions.01".to_string()];
        assert_eq!(
            prepare_advisory_request(AdvisoryRequestPolicy {
                model_id: "symthaea",
                model_version: "0.5.0",
                skill_id: "math.fractions",
                curriculum_revision: "2026.1",
                allowed_actions: &[LearningSuggestion::OfferHint],
                allowed_reasons: &[AdviceReason::RepeatedIncorrectAttempts],
                allowed_content_ids: &ids,
            })
            .err(),
            Some(RequestPreparationError::DuplicateContentReference)
        );
    }

    #[test]
    fn request_preparation_rejects_empty_action_allowlist() {
        let ids = ["lesson.fractions.01".to_string()];
        assert_eq!(
            prepare_advisory_request(AdvisoryRequestPolicy {
                model_id: "symthaea",
                model_version: "0.5.0",
                skill_id: "math.fractions",
                curriculum_revision: "2026.1",
                allowed_actions: &[],
                allowed_reasons: &[AdviceReason::RepeatedIncorrectAttempts],
                allowed_content_ids: &ids,
            })
            .err(),
            Some(RequestPreparationError::EmptyActionAllowlist)
        );
    }

    #[test]
    fn request_preparation_rejects_oversized_action_allowlist_before_copy() {
        let ids = ["lesson.fractions.01".to_string()];
        let actions = vec![LearningSuggestion::OfferHint; MAX_ALLOWED_ACTIONS + 1];
        assert_eq!(
            prepare_advisory_request(AdvisoryRequestPolicy {
                model_id: "symthaea",
                model_version: "0.5.0",
                skill_id: "math.fractions",
                curriculum_revision: "2026.1",
                allowed_actions: &actions,
                allowed_reasons: &[AdviceReason::RepeatedIncorrectAttempts],
                allowed_content_ids: &ids,
            })
            .err(),
            Some(RequestPreparationError::TooManyAllowedActions)
        );
    }

    #[test]
    fn request_preparation_rejects_oversized_content_reference_list() {
        let ids = (0..=MAX_SUPPORTING_CONTENT_IDS)
            .map(|index| format!("lesson.item.{index:02}"))
            .collect::<Vec<_>>();
        assert_eq!(
            prepare_advisory_request(AdvisoryRequestPolicy {
                model_id: "symthaea",
                model_version: "0.5.0",
                skill_id: "math.fractions",
                curriculum_revision: "2026.1",
                allowed_actions: &[LearningSuggestion::OfferHint],
                allowed_reasons: &[AdviceReason::RepeatedIncorrectAttempts],
                allowed_content_ids: &ids,
            })
            .err(),
            Some(RequestPreparationError::TooManyContentReferences)
        );
    }

    #[test]
    fn request_preparation_rejects_duplicate_actions() {
        let ids = ["lesson.fractions.01".to_string()];
        let actions = [
            LearningSuggestion::OfferHint,
            LearningSuggestion::OfferHint,
        ];
        assert_eq!(
            prepare_advisory_request(AdvisoryRequestPolicy {
                model_id: "symthaea",
                model_version: "0.5.0",
                skill_id: "math.fractions",
                curriculum_revision: "2026.1",
                allowed_actions: &actions,
                allowed_reasons: &[AdviceReason::RepeatedIncorrectAttempts],
                allowed_content_ids: &ids,
            })
            .err(),
            Some(RequestPreparationError::DuplicateAllowedAction)
        );
    }

    #[test]
    fn request_preparation_rejects_duplicate_reasons() {
        let ids = ["lesson.fractions.01".to_string()];
        let reasons = [
            AdviceReason::RepeatedIncorrectAttempts,
            AdviceReason::RepeatedIncorrectAttempts,
        ];
        assert_eq!(
            prepare_advisory_request(AdvisoryRequestPolicy {
                model_id: "symthaea",
                model_version: "0.5.0",
                skill_id: "math.fractions",
                curriculum_revision: "2026.1",
                allowed_actions: &[LearningSuggestion::OfferHint],
                allowed_reasons: &reasons,
                allowed_content_ids: &ids,
            })
            .err(),
            Some(RequestPreparationError::DuplicateAllowedReason)
        );
    }

    #[test]
    fn request_preparation_rejects_empty_reason_allowlist() {
        let ids = ["lesson.fractions.01".to_string()];
        assert_eq!(
            prepare_advisory_request(AdvisoryRequestPolicy {
                model_id: "symthaea",
                model_version: "0.5.0",
                skill_id: "math.fractions",
                curriculum_revision: "2026.1",
                allowed_actions: &[LearningSuggestion::OfferHint],
                allowed_reasons: &[],
                allowed_content_ids: &ids,
            })
            .err(),
            Some(RequestPreparationError::EmptyReasonAllowlist)
        );
    }

    #[test]
    fn request_preparation_rejects_oversized_reason_allowlist_before_copy() {
        let ids = ["lesson.fractions.01".to_string()];
        let reasons = vec![AdviceReason::RepeatedIncorrectAttempts; MAX_ALLOWED_REASONS + 1];
        assert_eq!(
            prepare_advisory_request(AdvisoryRequestPolicy {
                model_id: "symthaea",
                model_version: "0.5.0",
                skill_id: "math.fractions",
                curriculum_revision: "2026.1",
                allowed_actions: &[LearningSuggestion::OfferHint],
                allowed_reasons: &reasons,
                allowed_content_ids: &ids,
            })
            .err(),
            Some(RequestPreparationError::TooManyAllowedReasons)
        );
    }

    #[test]
    fn request_preparation_rejects_invalid_identifiers() {
        let ids = ["lesson.fractions.01".to_string()];
        assert_eq!(
            prepare_advisory_request(AdvisoryRequestPolicy {
                model_id: "symthaea\nforged",
                model_version: "0.5.0",
                skill_id: "math.fractions",
                curriculum_revision: "2026.1",
                allowed_actions: &[LearningSuggestion::OfferHint],
                allowed_reasons: &[AdviceReason::RepeatedIncorrectAttempts],
                allowed_content_ids: &ids,
            })
            .err(),
            Some(RequestPreparationError::InvalidIdentifier)
        );
    }

    #[test]
    fn reader_errors_fail_closed() {
        assert_eq!(
            read_bounded_symthaea_advice(FailingReader),
            Err(BoundedAdviceReadError::ReaderFailed(
                io::ErrorKind::BrokenPipe
            ))
        );
    }
}
