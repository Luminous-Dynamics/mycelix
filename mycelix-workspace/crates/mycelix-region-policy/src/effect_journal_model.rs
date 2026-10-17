//! Bounded deterministic model checking for reservation races and lost replies.
//!
//! This explores a small abstract durable row and an idempotent destination. It
//! does not exercise SQLite, process scheduling, storage durability, or a real
//! provider. Passing model tests can expose state-machine counterexamples but is
//! not production concurrency or exactly-once evidence.

use crate::effect_journal::{
    AttemptPhase, EffectAttempt, EffectAttemptSnapshot, EffectIdentity, SinkDisposition,
    SinkOutcomeEvidence, SinkOutcomeVerifier, SinkProofError, VerifiedSinkOutcome,
};

const OPERATION_A: &str = "sink-operation-a";
const OPERATION_B: &str = "sink-operation-b";

#[derive(Debug, Clone, Copy)]
#[derive(Debug, Clone, Copy)]
enum Event {
    LoadA,
    ReserveA,
    LoadB,
    ReserveB,
}

#[derive(Default, Debug, Clone)]
struct WorkerObservation {
    revision: Option<u64>,
    phase: Option<AttemptPhase>,
}

#[derive(Debug, Clone)]
struct SharedRow {
    snapshot: EffectAttemptSnapshot,
}

impl SharedRow {
    fn try_reserve(
        &mut self,
        observed_revision: Option<u64>,
        observed_phase: Option<AttemptPhase>,
        operation_id: &str,
    ) -> bool {
        if observed_revision != Some(self.snapshot.revision)
            || observed_phase != Some(AttemptPhase::Prepared)
            || self.snapshot.phase != AttemptPhase::Prepared
        {
            return false;
        }
        let Ok(mut attempt) = EffectAttempt::restore(self.snapshot.clone()) else {
            return false;
        };
        if attempt.reserve(operation_id).is_err() {
            return false;
        }
        self.snapshot = attempt.snapshot();
        true
    }
}

fn interleavings() -> Vec<Vec<Event>> {
    fn extend(
        schedule: &mut Vec<Event>,
        loaded_a: bool,
        reserved_a: bool,
        loaded_b: bool,
        reserved_b: bool,
        output: &mut Vec<Vec<Event>>,
    ) {
        if reserved_a && reserved_b {
            output.push(schedule.clone());
            return;
        }
        if !loaded_a {
            schedule.push(Event::LoadA);
            extend(schedule, true, reserved_a, loaded_b, reserved_b, output);
            schedule.pop();
        } else if !reserved_a {
            schedule.push(Event::ReserveA);
            extend(schedule, loaded_a, true, loaded_b, reserved_b, output);
            schedule.pop();
        }
        if !loaded_b {
            schedule.push(Event::LoadB);
            extend(schedule, loaded_a, reserved_a, true, reserved_b, output);
            schedule.pop();
        } else if !reserved_b {
            schedule.push(Event::ReserveB);
            extend(schedule, loaded_a, reserved_a, loaded_b, true, output);
            schedule.pop();
        }
    }

    let mut output = Vec::new();
    extend(&mut Vec::new(), false, false, false, false, &mut output);
    output
}

#[derive(Debug, Clone)]
struct StoredSinkResult {
    authority_domain: String,
    idempotency_key: String,
    target_digest: String,
    policy_binding_digest: String,
    operation_id: String,
    outcome_digest: String,
    disposition: SinkDisposition,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ModelSinkError {
    Conflict,
}

#[derive(Default)]
struct IdempotentSink {
    stored: Option<StoredSinkResult>,
    applied_effects: usize,
    delivery_attempts: usize,
}

impl IdempotentSink {
    fn dispatch(
        &mut self,
        identity: &EffectIdentity,
        operation_id: &str,
    ) -> Result<SinkOutcomeEvidence, ModelSinkError> {
        self.delivery_attempts += 1;
        if let Some(stored) = &self.stored {
            if stored.authority_domain != identity.authority_domain()
                || stored.idempotency_key != identity.idempotency_key()
                || stored.target_digest != identity.target_digest()
                || stored.policy_binding_digest != identity.policy_binding_digest()
                || stored.operation_id != operation_id
            {
                return Err(ModelSinkError::Conflict);
            }
            return Ok(evidence_from_stored(stored));
        }

        self.applied_effects += 1;
        let stored = StoredSinkResult {
            authority_domain: identity.authority_domain().to_owned(),
            idempotency_key: identity.idempotency_key().to_owned(),
            target_digest: identity.target_digest().to_owned(),
            policy_binding_digest: identity.policy_binding_digest().to_owned(),
            operation_id: operation_id.to_owned(),
            outcome_digest: "c".repeat(64),
            disposition: SinkDisposition::Applied,
        };
        self.stored = Some(stored.clone());
        Ok(evidence_from_stored(&stored))
    }
}

fn evidence_from_stored(stored: &StoredSinkResult) -> SinkOutcomeEvidence {
    SinkOutcomeEvidence {
        authority_domain: stored.authority_domain.clone(),
        idempotency_key: stored.idempotency_key.clone(),
        target_digest: stored.target_digest.clone(),
        policy_binding_digest: stored.policy_binding_digest.clone(),
        operation_id: stored.operation_id.clone(),
        disposition: stored.disposition,
        outcome_digest: stored.outcome_digest.clone(),
        proof: vec![0xA5; 64],
    }
}

struct AcceptModelProof;
impl SinkOutcomeVerifier for AcceptModelProof {
    fn verify(&self, evidence: &SinkOutcomeEvidence) -> Result<(), SinkProofError> {
        if evidence.proof == vec![0xA5; 64] {
            Ok(())
        } else {
            Err(SinkProofError::InvalidSignature)
        }
    }
}

fn identity() -> EffectIdentity {
    EffectIdentity::new(
        "community-a",
        "idempotency-operation-42",
        &"a".repeat(64),
        &"b".repeat(64),
    )
    .expect("model identity should be valid")
}

fn explore_schedule(schedule: &[Event]) -> (usize, SharedRow) {
    let mut shared = SharedRow {
        snapshot: EffectAttempt::prepare(identity()).snapshot(),
    };
    let mut a = WorkerObservation::default();
    let mut b = WorkerObservation::default();
    let mut winners = 0usize;

    for event in schedule {
        match event {
            Event::LoadA => {
                a.revision = Some(shared.snapshot.revision);
                a.phase = Some(shared.snapshot.phase);
            }
            Event::LoadB => {
                b.revision = Some(shared.snapshot.revision);
                b.phase = Some(shared.snapshot.phase);
            }
            Event::ReserveA => {
                if shared.try_reserve(a.revision, a.phase, OPERATION_A) {
                    winners += 1;
                }
            }
            Event::ReserveB => {
                if shared.try_reserve(b.revision, b.phase, OPERATION_B) {
                    winners += 1;
                }
            }
        }
    }
    (winners, shared)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_two_worker_load_reserve_interleaving_has_one_winner() {
        let schedules = interleavings();
        assert_eq!(schedules.len(), 6, "all order-preserving interleavings must be enumerated");
        for schedule in &schedules {
            let (winners, shared) = explore_schedule(schedule);
            assert_eq!(winners, 1, "schedule {schedule:?}");
            assert_eq!(shared.snapshot.phase, AttemptPhase::Reserved);
            assert_eq!(shared.snapshot.revision, 1);
            assert!(shared.snapshot.operation_id.as_deref() == Some(OPERATION_A)
                || shared.snapshot.operation_id.as_deref() == Some(OPERATION_B));
        }
    }

    #[test]
    fn lost_reply_after_sink_commit_recovers_without_duplicate_effect() {
        let identity = identity();
        let mut attempt = EffectAttempt::prepare(identity.clone());
        attempt.reserve(OPERATION_A).expect("reserve before dispatch");
        attempt.mark_dispatch_started().expect("durably mark dispatch started");
        let crash_cut = attempt.snapshot();

        let mut sink = IdempotentSink::default();
        let _lost_reply = sink.dispatch(&identity, OPERATION_A).expect("first delivery applies");
        assert_eq!(sink.applied_effects, 1);

        // Simulate process restart: only the persisted snapshot survives.
        let mut recovered = EffectAttempt::restore(crash_cut).expect("restore uncertain dispatch");
        assert_eq!(recovered.phase(), AttemptPhase::DispatchStarted);

        let replayed_reply = sink.dispatch(&identity, OPERATION_A).expect("same key returns stored result");
        assert_eq!(sink.delivery_attempts, 2);
        assert_eq!(sink.applied_effects, 1, "destination idempotency prevents a duplicate effect");

        let verified = VerifiedSinkOutcome::verify(replayed_reply, &identity, &AcceptModelProof)
            .expect("destination proof verifies");
        recovered.resolve(verified).expect("verified reply closes the attempt");
        assert_eq!(recovered.phase(), AttemptPhase::Committed);
    }

    #[test]
    fn sink_idempotency_key_reuse_for_changed_target_is_a_conflict() {
        let identity = identity();
        let mut sink = IdempotentSink::default();
        sink.dispatch(&identity, OPERATION_A).expect("first effect");
        let changed_target = EffectIdentity::new(
            identity.authority_domain(),
            identity.idempotency_key(),
            &"d".repeat(64),
            identity.policy_binding_digest(),
        )
        .expect("second identity is well-formed");
        assert!(matches!(
            sink.dispatch(&changed_target, OPERATION_A),
            Err(ModelSinkError::Conflict)
        ));
        assert_eq!(sink.applied_effects, 1);
    }

    #[test]
    fn revocation_before_dispatch_aborts_reserved_attempt_without_sink_call() {
        let mut attempt = EffectAttempt::prepare(identity());
        attempt.reserve(OPERATION_A).expect("reserve");
        let mut sink = IdempotentSink::default();
        let current_authority = false; // synthetic revocation between reservation and dispatch
        if current_authority {
            attempt.mark_dispatch_started().expect("only current authority may dispatch");
            sink.dispatch(attempt.identity(), OPERATION_A).expect("dispatch");
        } else {
            attempt.abort_before_dispatch("authority-revoked").expect("reserved work is pre-dispatch");
        }
        assert_eq!(attempt.phase(), AttemptPhase::Rejected);
        assert_eq!(sink.applied_effects, 0);
        assert_eq!(sink.delivery_attempts, 0);
    }
}
