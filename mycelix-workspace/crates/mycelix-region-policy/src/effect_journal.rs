//! Deterministic lifecycle model for protected side effects.
//!
//! This module describes a durable adapter's state machine. The implementation
//! here is in-memory value logic only: it does not persist records, dispatch
//! effects, or prove the destination outcome. A production adapter must commit
//! transitions atomically, enforce a unique idempotency key, and verify
//! destination-authenticated outcome evidence before calling resolve().

const MAX_AUTHORITY_DOMAIN_BYTES: usize = 256;
const MAX_KEY_BYTES: usize = 256;
const MAX_OPERATION_ID_BYTES: usize = 256;
const MAX_REASON_BYTES: usize = 128;

/// Separate reservation, dispatch start, and external outcome. These are not
/// interchangeable facts: a Reserved attempt has not begun dispatch, while a
/// DispatchStarted attempt may already have changed the remote world.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum AttemptPhase {
    Prepared,
    Reserved,
    DispatchStarted,
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
    authority_domain: String,
    idempotency_key: String,
    target_digest: String,
    policy_binding_digest: String,
}

impl EffectIdentity {
    pub fn new(
        authority_domain: &str,
        idempotency_key: &str,
        target_digest: &str,
        policy_binding_digest: &str,
    ) -> Result<Self, JournalError> {
        if !valid_text(authority_domain, MAX_AUTHORITY_DOMAIN_BYTES)
            || !valid_text(idempotency_key, MAX_KEY_BYTES)
            || !valid_digest(target_digest)
            || !valid_digest(policy_binding_digest)
        {
            return Err(JournalError::InvalidIdentity);
        }
        Ok(Self {
            authority_domain: authority_domain.to_owned(),
            idempotency_key: idempotency_key.to_owned(),
            target_digest: target_digest.to_owned(),
            policy_binding_digest: policy_binding_digest.to_owned(),
        })
    }

    pub fn authority_domain(&self) -> &str {
        &self.authority_domain
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
    pub authority_domain: String,
    pub idempotency_key: String,
    pub target_digest: String,
    /// Digest binding the authorization decision to exact profile/agreement versions.
    pub policy_binding_digest: String,
    pub operation_id: String,
    pub disposition: SinkDisposition,
    /// Digest of the sink receipt/outcome, not the raw receipt or personal data.
    pub outcome_digest: String,
    pub proof: Vec<u8>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SinkProofError {
    InvalidSignature,
    MissingTrustRoot,
    StaleStatus,
    Unavailable,
    InvalidSemantics,
}

pub trait SinkOutcomeVerifier {
    /// Validate proof authenticity and authoritative destination state.
    /// The adapter must fail closed on missing/unavailable/stale proof sources.
    fn verify(&self, evidence: &SinkOutcomeEvidence) -> Result<(), SinkProofError>;
}

/// Opaque destination-evidence result. It has no public constructor and cannot
/// be built from a caller-provided boolean.
pub struct VerifiedSinkOutcome {
    authority_domain: String,
    idempotency_key: String,
    target_digest: String,
    policy_binding_digest: String,
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
        if !valid_text(&evidence.authority_domain, MAX_AUTHORITY_DOMAIN_BYTES)
            || !valid_text(&evidence.idempotency_key, MAX_KEY_BYTES)
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
        if evidence.authority_domain != expected.authority_domain
            || evidence.idempotency_key != expected.idempotency_key
            || evidence.target_digest != expected.target_digest
            || evidence.policy_binding_digest != expected.policy_binding_digest
        {
            return Err(JournalError::IdentityMismatch);
        }
        verifier
            .verify(&evidence)
            .map_err(|_error| JournalError::OutcomeVerificationFailed)?;
        Ok(Self {
            authority_domain: evidence.authority_domain,
            idempotency_key: evidence.idempotency_key,
            target_digest: evidence.target_digest,
            policy_binding_digest: evidence.policy_binding_digest,
            operation_id: evidence.operation_id,
            disposition: evidence.disposition,
            outcome_digest: evidence.outcome_digest,
        })
    }
}

/// One attempt record. It is intentionally not Clone: callers should load the
/// canonical durable record and apply a compare-and-swap transition, not fork
/// independent mutable copies and treat them as authoritative.
/// Owned persistence snapshot. A database adapter should serialize this record
/// using its own versioned schema, then use revision as a compare-and-swap token.
/// The snapshot contains only opaque identifiers and digests, never payload bytes.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct EffectAttemptSnapshot {
    pub identity: EffectIdentity,
    pub phase: AttemptPhase,
    pub revision: u64,
    pub dispatch_started: bool,
    pub operation_id: Option<String>,
    pub outcome_digest: Option<String>,
    pub rejection_code: Option<String>,
}

#[derive(Debug, PartialEq, Eq)]
pub struct EffectAttempt {
    identity: EffectIdentity,
    phase: AttemptPhase,
    revision: u64,
    dispatch_started: bool,
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
            dispatch_started: false,
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

    /// Export a value snapshot for persistence. The adapter must persist this
    /// with compare-and-swap on revision before performing the next external step.
    pub fn snapshot(&self) -> EffectAttemptSnapshot {
        EffectAttemptSnapshot {
            identity: self.identity.clone(),
            phase: self.phase,
            revision: self.revision,
            dispatch_started: self.dispatch_started,
            operation_id: self.operation_id.clone(),
            outcome_digest: self.outcome_digest.clone(),
            rejection_code: self.rejection_code.clone(),
        }
    }

    /// Restore a persisted snapshot only after checking its state invariants.
    /// Authenticity/rollback protection for the stored record remains the
    /// durable adapter's responsibility.
    pub fn restore(snapshot: EffectAttemptSnapshot) -> Result<Self, JournalError> {
        let attempt = Self {
            identity: snapshot.identity,
            phase: snapshot.phase,
            revision: snapshot.revision,
            dispatch_started: snapshot.dispatch_started,
            operation_id: snapshot.operation_id,
            outcome_digest: snapshot.outcome_digest,
            rejection_code: snapshot.rejection_code,
        };
        if !attempt.state_is_consistent() {
            return Err(JournalError::InvalidTransition);
        }
        Ok(attempt)
    }

    fn state_is_consistent(&self) -> bool {
        let op_valid = self.operation_id.as_ref()
            .is_some_and(|id| valid_text(id, MAX_OPERATION_ID_BYTES));
        let outcome_valid = self.outcome_digest.as_deref().is_some_and(valid_digest);
        let reason_valid = self.rejection_code.as_deref().is_some_and(valid_reason_code);
        match self.phase {
            AttemptPhase::Prepared => {
                self.revision == 0
                    && !self.dispatch_started
                    && self.operation_id.is_none()
                    && self.outcome_digest.is_none()
                    && self.rejection_code.is_none()
            }
            AttemptPhase::Reserved => {
                self.revision == 1
                    && !self.dispatch_started
                    && op_valid
                    && self.outcome_digest.is_none()
                    && self.rejection_code.is_none()
            }
            AttemptPhase::DispatchStarted => {
                self.revision == 2
                    && self.dispatch_started
                    && op_valid
                    && self.outcome_digest.is_none()
                    && self.rejection_code.is_none()
            }
            AttemptPhase::OutcomeUnknown => {
                self.revision >= 3
                    && self.dispatch_started
                    && op_valid
                    && self.outcome_digest.is_none()
                    && self.rejection_code.is_none()
            }
            AttemptPhase::Committed => {
                self.revision >= 3
                    && self.dispatch_started
                    && op_valid
                    && outcome_valid
                    && self.rejection_code.is_none()
            }
            AttemptPhase::Rejected => {
                (!self.dispatch_started
                    && self.outcome_digest.is_none()
                    && reason_valid
                    && ((self.revision == 1 && self.operation_id.is_none())
                        || (self.revision == 2 && op_valid)))
                    || (self.dispatch_started
                        && self.revision >= 3
                        && op_valid
                        && outcome_valid
                        && self.rejection_code.is_none())
            }
        }
    }

    /// Reserve a stable downstream operation identity before any dispatch.
    /// The durable adapter must persist this transition using compare-and-swap.
    pub fn reserve(&mut self, operation_id: &str) -> Result<(), JournalError> {
        if self.phase != AttemptPhase::Prepared {
            return Err(JournalError::InvalidTransition);
        }
        if !valid_text(operation_id, MAX_OPERATION_ID_BYTES) {
            return Err(JournalError::InvalidOperationId);
        }
        self.advance()?;
        self.operation_id = Some(operation_id.to_owned());
        self.phase = AttemptPhase::Reserved;
        Ok(())
    }

    /// Record that the dispatch boundary is about to be crossed. Production
    /// adapters must durably commit this before performing network I/O. From
    /// this point onward a crash/timeout may mean the remote effect occurred.
    pub fn mark_dispatch_started(&mut self) -> Result<(), JournalError> {
        if self.phase != AttemptPhase::Reserved {
            return Err(JournalError::InvalidTransition);
        }
        self.advance()?;
        self.dispatch_started = true;
        self.phase = AttemptPhase::DispatchStarted;
        Ok(())
    }

    /// Persist this after timeout/crash uncertainty; never use it as permission
    /// to resend the downstream effect.
    pub fn mark_outcome_unknown(&mut self) -> Result<(), JournalError> {
        if self.phase != AttemptPhase::DispatchStarted {
            return Err(JournalError::InvalidTransition);
        }
        self.advance()?;
        self.phase = AttemptPhase::OutcomeUnknown;
        Ok(())
    }

    /// Abort only before dispatch. Prepared and Reserved are both pre-dispatch;
    /// any state at or after DispatchStarted needs authoritative sink evidence.
    pub fn abort_before_dispatch(&mut self, reason_code: &str) -> Result<(), JournalError> {
        if !matches!(self.phase, AttemptPhase::Prepared | AttemptPhase::Reserved) {
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
        if !matches!(self.phase, AttemptPhase::DispatchStarted | AttemptPhase::OutcomeUnknown) {
            return Err(JournalError::InvalidTransition);
        }
        if outcome.authority_domain != self.identity.authority_domain
            || outcome.idempotency_key != self.identity.idempotency_key
            || outcome.target_digest != self.identity.target_digest
            || outcome.policy_binding_digest != self.identity.policy_binding_digest
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
            .all(|byte| {
                byte.is_ascii_lowercase()
                    || byte.is_ascii_digit()
                    || byte == b'-'
                    || byte == b'_'
            })
}

#[cfg(test)]
mod tests {
    use super::*;

    struct AcceptSinkProof;
    impl SinkOutcomeVerifier for AcceptSinkProof {
        fn verify(&self, evidence: &SinkOutcomeEvidence) -> Result<(), SinkProofError> {
            if evidence.proof.as_slice() == &b"fixture-proof"[..] {
                Ok(())
            } else {
                Err(SinkProofError::InvalidSignature)
            }
        }
    }

    struct RejectSinkProof;
    impl SinkOutcomeVerifier for RejectSinkProof {
        fn verify(&self, _evidence: &SinkOutcomeEvidence) -> Result<(), SinkProofError> {
            Err(SinkProofError::InvalidSignature)
        }
    }

    fn identity() -> EffectIdentity {
        EffectIdentity::new("community-a", "idem-key-1", &"a".repeat(64), &"b".repeat(64))
            .expect("test identity is structurally valid")
    }

    fn evidence(disposition: SinkDisposition) -> SinkOutcomeEvidence {
        SinkOutcomeEvidence {
            authority_domain: "community-a".to_owned(),
            idempotency_key: "idem-key-1".to_owned(),
            target_digest: "a".repeat(64),
            policy_binding_digest: "b".repeat(64),
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
        attempt.reserve("sink-operation-1").expect("prepare -> reserve");
        assert_eq!(attempt.phase(), AttemptPhase::Reserved);
        assert_eq!(attempt.revision(), 1);
        attempt.mark_dispatch_started().expect("reserve -> dispatch");
        assert_eq!(attempt.phase(), AttemptPhase::DispatchStarted);
        assert_eq!(attempt.revision(), 2);
    }

    #[test]
    fn dispatch_started_attempt_becomes_unknown_not_retryable() {
        let mut attempt = EffectAttempt::prepare(identity());
        attempt.reserve("sink-operation-1").expect("prepare -> reserve");
        attempt.mark_dispatch_started().expect("reserve -> dispatch");
        attempt.mark_outcome_unknown().expect("dispatch -> unknown");
        assert_eq!(attempt.phase(), AttemptPhase::OutcomeUnknown);
        assert_eq!(attempt.mark_dispatch_started(), Err(JournalError::InvalidTransition));
        assert_eq!(attempt.reserve("sink-operation-2"), Err(JournalError::InvalidTransition));
    }

    #[test]
    fn only_authenticated_matching_outcome_resolves_unknown() {
        let mut attempt = EffectAttempt::prepare(identity());
        attempt.reserve("sink-operation-1").expect("prepare -> reserve");
        attempt.mark_dispatch_started().expect("reserve -> dispatch");
        attempt.mark_outcome_unknown().expect("dispatch -> unknown");
        attempt
            .resolve(verified_outcome(SinkDisposition::Applied))
            .expect("matching proof resolves");
        assert_eq!(attempt.phase(), AttemptPhase::Committed);
        let expected_digest = "c".repeat(64);
        assert_eq!(attempt.outcome_digest(), Some(expected_digest.as_str()));
    }

    #[test]
    fn rejected_sink_outcome_is_terminal_only_when_verified() {
        let mut attempt = EffectAttempt::prepare(identity());
        attempt.reserve("sink-operation-1").expect("prepare -> reserve");
        attempt.mark_dispatch_started().expect("reserve -> dispatch");
        attempt
            .resolve(verified_outcome(SinkDisposition::DefinitivelyRejected))
            .expect("verified rejection");
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
        assert_eq!(
            attempt.resolve(verified_outcome(SinkDisposition::Applied)),
            Err(JournalError::InvalidTransition)
        );
        attempt.abort_before_dispatch("cancelled").expect("prepared can be aborted");
        assert_eq!(attempt.phase(), AttemptPhase::Rejected);
        assert_eq!(
            attempt.resolve(verified_outcome(SinkDisposition::Applied)),
            Err(JournalError::InvalidTransition)
        );
    }

    #[test]
    fn operation_id_must_match_before_terminal_resolution() {
        let mut attempt = EffectAttempt::prepare(identity());
        attempt.reserve("sink-operation-1").expect("prepare -> reserve");
        attempt.mark_dispatch_started().expect("reserve -> dispatch");
        let mut e = evidence(SinkDisposition::Applied);
        e.operation_id = "different-operation".to_owned();
        let verified = VerifiedSinkOutcome::verify(e, &identity(), &AcceptSinkProof)
            .expect("mock verifier authenticates this fixture");
        assert_eq!(attempt.resolve(verified), Err(JournalError::OperationIdMismatch));
        assert_eq!(attempt.phase(), AttemptPhase::DispatchStarted);
    }

    #[test]
    fn invalid_identity_and_reason_code_are_rejected() {
        assert_eq!(
            EffectIdentity::new("", "idem-key", &"a".repeat(64), &"b".repeat(64)),
            Err(JournalError::InvalidIdentity)
        );
        let mut attempt = EffectAttempt::prepare(identity());
        assert_eq!(
            attempt.abort_before_dispatch("sensitive reason"),
            Err(JournalError::InvalidRejectionCode)
        );
    }

    #[test]
    fn sink_verifier_rejection_does_not_create_verified_outcome() {
        assert!(matches!(
            VerifiedSinkOutcome::verify(
                evidence(SinkDisposition::Applied),
                &identity(),
                &RejectSinkProof
            ),
            Err(JournalError::OutcomeVerificationFailed)
        ));
    }
    #[test]
    fn snapshot_round_trips_reserved_dispatch_and_unknown_states() {
        let mut attempt = EffectAttempt::prepare(identity());
        attempt.reserve("sink-operation-1").expect("reserve");
        let restored_reserved =
            EffectAttempt::restore(attempt.snapshot()).expect("restore reserved");
        assert_eq!(restored_reserved.phase(), AttemptPhase::Reserved);
        assert_eq!(restored_reserved.operation_id(), Some("sink-operation-1"));
        let mut dispatched = restored_reserved;
        dispatched.mark_dispatch_started().expect("dispatch started");
        let restored_dispatched =
            EffectAttempt::restore(dispatched.snapshot()).expect("restore dispatched");
        assert_eq!(restored_dispatched.phase(), AttemptPhase::DispatchStarted);
        let mut unknown = restored_dispatched;
        unknown.mark_outcome_unknown().expect("unknown");
        let restored_unknown = EffectAttempt::restore(unknown.snapshot()).expect("restore unknown");
        assert_eq!(restored_unknown.phase(), AttemptPhase::OutcomeUnknown);
        assert_eq!(restored_unknown.revision(), 3);
    }

    #[test]
    fn restore_rejects_impossible_phase_field_combinations() {
        let mut snapshot = EffectAttempt::prepare(identity()).snapshot();
        snapshot.phase = AttemptPhase::Committed;
        snapshot.revision = 1;
        snapshot.outcome_digest = Some("c".repeat(64));
        assert_eq!(
            EffectAttempt::restore(snapshot),
            Err(JournalError::InvalidTransition)
        );
    }

    #[test]
    fn rejects_replayed_sink_proof_for_changed_policy_binding() {
        let expected = identity();
        let mut e = evidence(SinkDisposition::Applied);
        e.policy_binding_digest = "d".repeat(64);
        assert!(matches!(
            VerifiedSinkOutcome::verify(e, &expected, &AcceptSinkProof),
            Err(JournalError::IdentityMismatch)
        ));
    }

    #[test]
    fn rejects_sink_proof_replayed_across_authority_domains() {
        let expected = identity();
        let mut e = evidence(SinkDisposition::Applied);
        e.authority_domain = "community-b".to_owned();
        assert!(matches!(
            VerifiedSinkOutcome::verify(e, &expected, &AcceptSinkProof),
            Err(JournalError::IdentityMismatch)
        ));
    }

    #[test]
    fn reserved_attempt_can_be_aborted_before_dispatch() {
        let mut attempt = EffectAttempt::prepare(identity());
        attempt.reserve("sink-operation-1").expect("reserve");
        attempt.abort_before_dispatch("policy-revoked").expect("reserved is pre-dispatch");
        assert_eq!(attempt.phase(), AttemptPhase::Rejected);
        assert_eq!(attempt.operation_id(), Some("sink-operation-1"));
        assert_eq!(attempt.outcome_digest(), None);
        assert!(EffectAttempt::restore(attempt.snapshot()).is_ok());
    }

    #[test]
    fn dispatch_started_attempt_cannot_be_aborted_without_sink_evidence() {
        let mut attempt = EffectAttempt::prepare(identity());
        attempt.reserve("sink-operation-1").expect("reserve");
        attempt.mark_dispatch_started().expect("dispatch");
        assert_eq!(
            attempt.abort_before_dispatch("cancelled"),
            Err(JournalError::InvalidTransition)
        );
        assert_eq!(attempt.phase(), AttemptPhase::DispatchStarted);
    }

    #[test]
    fn restore_rejects_postdispatch_rejection_without_sink_evidence() {
        let mut attempt = EffectAttempt::prepare(identity());
        attempt.reserve("sink-operation-1").expect("reserve");
        attempt.mark_dispatch_started().expect("dispatch");
        let mut snapshot = attempt.snapshot();
        snapshot.phase = AttemptPhase::Rejected;
        snapshot.rejection_code = Some("local-timeout".to_owned());
        snapshot.outcome_digest = None;
        assert_eq!(
            EffectAttempt::restore(snapshot),
            Err(JournalError::InvalidTransition)
        );
    }

}
