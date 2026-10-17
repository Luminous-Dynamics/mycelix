//! Deterministic lifecycle model for protected side effects.
//!
//! This module describes a durable adapter's state machine. The implementation
//! here is in-memory value logic only: it does not persist records, dispatch
//! effects, or prove the destination outcome. A production adapter must commit
//! transitions atomically, enforce a unique idempotency key, and verify
//! destination-authenticated outcome evidence before calling resolve().

const MAX_KEY_BYTES: usize = 256;
const MAX_OPERATION_ID_BYTES: usize = 256;
const MAX_REASON_BYTES: usize = 128;

/// Durable adapter lifecycle. Submitted without a definitive answer is not
/// equivalent to failure: after a crash it must become OutcomeUnknown.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AttemptPhase {
    Prepared,
    Submitted,
    OutcomeUnknown,
    Committed,
    Rejected,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SinkDisposition {
    Applied,
    DefinitivelyRejected,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum JournalError {
    InvalidIdentity,
    InvalidOperationId,
    InvalidOutcomeDigest,
    InvalidRejectionCode,
    InvalidProofSize,
    InvalidTransition,
    RevisionOverflow,
    IdentityMismatch,
    OperationIdMismatch,
    OutcomeVerificationFailed,
}

/// Stable identity for one logical operation. The idempotency key is scoped by
/// the caller's authority domain in the production uniqueness constraint.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct EffectIdentity {
    idempotency_key: String,
    target_digest: String,
    policy_binding_digest: String,
}

impl EffectIdentity {
    pub fn new(
        idempotency_key: &str,
        target_digest: &str,
        policy_binding_digest: &str,
    ) -> Result<Self, JournalError> {
        if !valid_text(idempotency_key, MAX_KEY_BYTES)
            || !valid_digest(target_digest)
            || !valid_digest(policy_binding_digest)
        {
            return Err(JournalError::InvalidIdentity);
        }
        Ok(Self {
            idempotency_key: idempotency_key.to_owned(),
            target_digest: target_digest.to_owned(),
            policy_binding_digest: policy_binding_digest.to_owned(),
        })
    }

    pub fn idempotency_key(&self) -> &str {
        &self.idempotency_key
    }

    pub fn target_digest(&self) -> &str {
        &self.target_digest
    }

    pub fn policy_binding_digest(&self) -> &str {
        &self.policy_binding_digest
    }
}

/// Raw sink evidence is untrusted until the configured verifier authenticates it
/// and confirms the disposition, operation ID, key, target, and result digest.
pub struct SinkOutcomeEvidence {
    pub idempotency_key: String,
    pub target_digest: String,
    pub operation_id: String,
    pub disposition: SinkDisposition,
    /// Digest of the sink receipt/outcome, not the raw receipt or personal data.
    pub outcome_digest: String,
    pub proof: Vec<u8>,
}

pub trait SinkOutcomeVerifier {
    /// Validate proof authenticity and authoritative destination state.
    /// The adapter must fail closed on missing/unavailable/stale proof sources.
    fn verify(&self, evidence: &SinkOutcomeEvidence) -> Result<(), ()>;
}

/// Opaque destination-evidence result. It has no public constructor and cannot
/// be built from a caller-provided boolean.
pub struct VerifiedSinkOutcome {
    idempotency_key: String,
    target_digest: String,
    operation_id: String,
    disposition: SinkDisposition,
    outcome_digest: String,
}

impl VerifiedSinkOutcome {
    pub fn verify<V: SinkOutcomeVerifier>(
        evidence: SinkOutcomeEvidence,
        expected: &EffectIdentity,
        verifier: &V,
    ) -> Result<Self, JournalError> {
        if !valid_text(&evidence.idempotency_key, MAX_KEY_BYTES)
            || !valid_digest(&evidence.target_digest)
            || !valid_text(&evidence.operation_id, MAX_OPERATION_ID_BYTES)
        {
            return Err(JournalError::InvalidIdentity);
        }
        if !valid_digest(&evidence.outcome_digest) {
            return Err(JournalError::InvalidOutcomeDigest);
        }
        if evidence.proof.is_empty() || evidence.proof.len() > 16 * 1024 {
            return Err(JournalError::InvalidProofSize);
        }
        if evidence.idempotency_key != expected.idempotency_key
            || evidence.target_digest != expected.target_digest
        {
            return Err(JournalError::IdentityMismatch);
        }
        verifier
            .verify(&evidence)
            .map_err(|()| JournalError::OutcomeVerificationFailed)?;
        Ok(Self {
            idempotency_key: evidence.idempotency_key,
            target_digest: evidence.target_digest,
            operation_id: evidence.operation_id,
            disposition: evidence.disposition,
            outcome_digest: evidence.outcome_digest,
        })
    }
}

/// One attempt record. It is intentionally not Clone: callers should load the
/// canonical durable record and apply a compare-and-swap transition, not fork
/// independent mutable copies and treat them as authoritative.
#[derive(Debug, PartialEq, Eq)]
pub struct EffectAttempt {
    identity: EffectIdentity,
    phase: AttemptPhase,
    revision: u64,
    operation_id: Option<String>,
    outcome_digest: Option<String>,
    rejection_code: Option<String>,
}

impl EffectAttempt {
    pub fn prepare(identity: EffectIdentity) -> Self {
        Self {
            identity,
            phase: AttemptPhase::Prepared,
            revision: 0,
            operation_id: None,
            outcome_digest: None,
            rejection_code: None,
        }
    }

    pub fn identity(&self) -> &EffectIdentity {
        &self.identity
    }

    pub fn phase(&self) -> AttemptPhase {
        self.phase
    }

    pub fn revision(&self) -> u64 {
        self.revision
    }

    pub fn operation_id(&self) -> Option<&str> {
        self.operation_id.as_deref()
    }

    pub fn outcome_digest(&self) -> Option<&str> {
        self.outcome_digest.as_deref()
    }

    /// Record the downstream operation ID before dispatch. The adapter should
    /// durably commit this transition before sending the request.
    pub fn mark_submitted(&mut self, operation_id: &str) -> Result<(), JournalError> {
        if self.phase != AttemptPhase::Prepared {
            return Err(JournalError::InvalidTransition);
        }
        if !valid_text(operation_id, MAX_OPERATION_ID_BYTES) {
            return Err(JournalError::InvalidOperationId);
        }
        self.advance()?;
        self.operation_id = Some(operation_id.to_owned());
        self.phase = AttemptPhase::Submitted;
        Ok(())
    }

    /// Persist this after timeout/crash uncertainty; never use it as permission
    /// to resend the downstream effect.
    pub fn mark_outcome_unknown(&mut self) -> Result<(), JournalError> {
        if self.phase != AttemptPhase::Submitted {
            return Err(JournalError::InvalidTransition);
        }
        self.advance()?;
        self.phase = AttemptPhase::OutcomeUnknown;
        Ok(())
    }

    /// Abort only before dispatch. Once Submitted, use authenticated sink
    /// evidence to resolve; an arbitrary local error is not proof of rejection.
    pub fn abort_before_dispatch(&mut self, reason_code: &str) -> Result<(), JournalError> {
        if self.phase != AttemptPhase::Prepared {
            return Err(JournalError::InvalidTransition);
        }
        if !valid_reason_code(reason_code) {
            return Err(JournalError::InvalidRejectionCode);
        }
        self.advance()?;
        self.rejection_code = Some(reason_code.to_owned());
        self.phase = AttemptPhase::Rejected;
        Ok(())
    }

    /// Resolve only with opaque, destination-verified evidence matching both
    /// the logical idempotency key and exact target. The adapter must CAS the
    /// revision in durable storage before accepting the resulting state.
    pub fn resolve(&mut self, outcome: VerifiedSinkOutcome) -> Result<(), JournalError> {
        if !matches!(self.phase, AttemptPhase::Submitted | AttemptPhase::OutcomeUnknown) {
            return Err(JournalError::InvalidTransition);
        }
        if outcome.idempotency_key != self.identity.idempotency_key
            || outcome.target_digest != self.identity.target_digest
        {
            return Err(JournalError::IdentityMismatch);
        }
        if self
            .operation_id
            .as_deref()
            .is_some_and(|expected| expected != outcome.operation_id)
        {
            return Err(JournalError::OperationIdMismatch);
        }
        self.advance()?;
        self.outcome_digest = Some(outcome.outcome_digest);
        self.phase = match outcome.disposition {
            SinkDisposition::Applied => AttemptPhase::Committed,
            SinkDisposition::DefinitivelyRejected => AttemptPhase::Rejected,
        };
        Ok(())
    }

    fn advance(&mut self) -> Result<(), JournalError> {
        self.revision = self
            .revision
            .checked_add(1)
            .ok_or(JournalError::RevisionOverflow)?;
        Ok(())
    }
}

fn valid_digest(value: &str) -> bool {
    value.len() == 64
        && value
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}

fn valid_text(value: &str, max_bytes: usize) -> bool {
    !value.trim().is_empty()
        && value.len() <= max_bytes
        && !value.chars().any(char::is_control)
}

fn valid_reason_code(value: &str) -> bool {
    !value.is_empty()
        && value.len() <= MAX_REASON_BYTES
        && value
            .bytes()
            .all(|byte| byte.is_ascii_lowercase() || byte.is_ascii_digit() || byte == b'-' || byte == b'_')
}

#[cfg(test)]
mod tests {
    use super::*;

    struct AcceptSinkProof;
    impl SinkOutcomeVerifier for AcceptSinkProof {
        fn verify(&self, evidence: &SinkOutcomeEvidence) -> Result<(), ()> {
            if evidence.proof == b"fixture-proof" {
                Ok(())
            } else {
                Err(())
            }
        }
    }

    struct RejectSinkProof;
    impl SinkOutcomeVerifier for RejectSinkProof {
        fn verify(&self, _evidence: &SinkOutcomeEvidence) -> Result<(), ()> {
            Err(())
        }
    }

    fn identity() -> EffectIdentity {
        EffectIdentity::new("idem-key-1", &"a".repeat(64), &"b".repeat(64))
            .expect("test identity is structurally valid")
    }

    fn evidence(disposition: SinkDisposition) -> SinkOutcomeEvidence {
        SinkOutcomeEvidence {
            idempotency_key: "idem-key-1".to_owned(),
            target_digest: "a".repeat(64),
            operation_id: "sink-operation-1".to_owned(),
            disposition,
            outcome_digest: "c".repeat(64),
            proof: b"fixture-proof".to_vec(),
        }
    }

    fn verified_outcome(disposition: SinkDisposition) -> VerifiedSinkOutcome {
        VerifiedSinkOutcome::verify(evidence(disposition), &identity(), &AcceptSinkProof)
            .expect("fixture verifier accepts test proof")
    }

    #[test]
    fn starts_prepared_and_revision_increases_only_on_legal_transition() {
        let mut attempt = EffectAttempt::prepare(identity());
        assert_eq!(attempt.phase(), AttemptPhase::Prepared);
        assert_eq!(attempt.revision(), 0);
        attempt.mark_submitted("sink-operation-1").expect("prepare -> submit");
        assert_eq!(attempt.phase(), AttemptPhase::Submitted);
        assert_eq!(attempt.revision(), 1);
    }

    #[test]
    fn submitted_attempt_becomes_unknown_not_retryable() {
        let mut attempt = EffectAttempt::prepare(identity());
        attempt.mark_submitted("sink-operation-1").expect("prepare -> submit");
        attempt.mark_outcome_unknown().expect("submit -> unknown");
        assert_eq!(attempt.phase(), AttemptPhase::OutcomeUnknown);
        assert_eq!(attempt.mark_submitted("sink-operation-2"), Err(JournalError::InvalidTransition));
    }

    #[test]
    fn only_authenticated_matching_outcome_resolves_unknown() {
        let mut attempt = EffectAttempt::prepare(identity());
        attempt.mark_submitted("sink-operation-1").expect("prepare -> submit");
        attempt.mark_outcome_unknown().expect("submit -> unknown");
        attempt.resolve(verified_outcome(SinkDisposition::Applied)).expect("matching proof resolves");
        assert_eq!(attempt.phase(), AttemptPhase::Committed);
        assert_eq!(attempt.outcome_digest(), Some("c".repeat(64).as_str()));
    }

    #[test]
    fn rejected_sink_outcome_is_terminal_only_when_verified() {
        let mut attempt = EffectAttempt::prepare(identity());
        attempt.mark_submitted("sink-operation-1").expect("prepare -> submit");
        attempt.resolve(verified_outcome(SinkDisposition::DefinitivelyRejected)).expect("verified rejection");
        assert_eq!(attempt.phase(), AttemptPhase::Rejected);
        assert_eq!(attempt.mark_outcome_unknown(), Err(JournalError::InvalidTransition));
    }

    #[test]
    fn wrong_idempotency_key_or_target_is_rejected_before_verification() {
        let expected = identity();
        let mut evidence = evidence(SinkDisposition::Applied);
        evidence.idempotency_key = "other-key".to_owned();
        assert!(matches!(
            VerifiedSinkOutcome::verify(evidence, &expected, &AcceptSinkProof),
            Err(JournalError::IdentityMismatch)
        ));
    }

    #[test]
    fn untrusted_or_missing_sink_proof_fails_closed() {
        let expected = identity();
        let mut bad = evidence(SinkDisposition::Applied);
        bad.proof = b"forged".to_vec();
        assert!(matches!(
            VerifiedSinkOutcome::verify(bad, &expected, &AcceptSinkProof),
            Err(JournalError::OutcomeVerificationFailed)
        ));
        let mut no_proof = evidence(SinkDisposition::Applied);
        no_proof.proof.clear();
        assert!(matches!(
            VerifiedSinkOutcome::verify(no_proof, &expected, &AcceptSinkProof),
            Err(JournalError::InvalidProofSize)
        ));
    }

    #[test]
    fn cannot_resolve_prepared_or_terminal_attempt() {
        let mut attempt = EffectAttempt::prepare(identity());
        assert_eq!(attempt.resolve(verified_outcome(SinkDisposition::Applied)), Err(JournalError::InvalidTransition));
        attempt.abort_before_dispatch("cancelled").expect("prepared can be aborted");
        assert_eq!(attempt.phase(), AttemptPhase::Rejected);
        assert_eq!(attempt.resolve(verified_outcome(SinkDisposition::Applied)), Err(JournalError::InvalidTransition));
    }

    #[test]
    fn operation_id_must_match_before_terminal_resolution() {
        let mut attempt = EffectAttempt::prepare(identity());
        attempt.mark_submitted("sink-operation-1").expect("prepare -> submit");
        let mut e = evidence(SinkDisposition::Applied);
        e.operation_id = "different-operation".to_owned();
        let verified = VerifiedSinkOutcome::verify(e, &identity(), &AcceptSinkProof)
            .expect("mock verifier authenticates this fixture");
        assert_eq!(attempt.resolve(verified), Err(JournalError::OperationIdMismatch));
        assert_eq!(attempt.phase(), AttemptPhase::Submitted);
    }

    #[test]
    fn invalid_identity_and_reason_code_are_rejected() {
        assert_eq!(
            EffectIdentity::new("", &"a".repeat(64), &"b".repeat(64)),
            Err(JournalError::InvalidIdentity)
        );
        let mut attempt = EffectAttempt::prepare(identity());
        assert_eq!(attempt.abort_before_dispatch("sensitive reason"), Err(JournalError::InvalidRejectionCode));
    }

    #[test]
    fn sink_verifier_rejection_does_not_create_verified_outcome() {
        assert!(matches!(
            VerifiedSinkOutcome::verify(evidence(SinkDisposition::Applied), &identity(), &RejectSinkProof),
            Err(JournalError::OutcomeVerificationFailed)
        ));
    }
}
