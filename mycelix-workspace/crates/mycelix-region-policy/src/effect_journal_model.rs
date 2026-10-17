//! Bounded deterministic model checking for reservation races and lost replies.
//!
//! This explores a small abstract durable row and an idempotent destination. It
//! does not exercise SQLite, process scheduling, storage durability, or a real
//! provider. Passing model tests can expose state-machine counterexamples but is
//! not production concurrency or exactly-once evidence.

use std::collections::{HashSet, VecDeque};

use crate::effect_journal::{
    AttemptPhase, EffectAttempt, EffectAttemptSnapshot, EffectIdentity, SinkDisposition,
    SinkOutcomeEvidence, SinkOutcomeVerifier, SinkProofError, VerifiedSinkOutcome,
};

const MODEL_WORKER_COUNT: usize = 4;
const OPERATION_IDS: [&str; MODEL_WORKER_COUNT] = [
    "sink-operation-a",
    "sink-operation-b",
    "sink-operation-c",
    "sink-operation-d",
];

#[derive(Debug, Clone, Copy)]
enum Event {
    Load(usize),
    Reserve(usize),
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
        next_stage: &mut [u8; MODEL_WORKER_COUNT],
        output: &mut Vec<Vec<Event>>,
    ) {
        if next_stage.iter().all(|stage| *stage == 2) {
            output.push(schedule.clone());
            return;
        }

        // Each worker preserves Load -> Reserve order; the recursive enumerator
        // covers every interleaving that preserves those per-worker constraints.
        for worker in 0..MODEL_WORKER_COUNT {
            if next_stage[worker] >= 2 {
                continue;
            }
            let event = if next_stage[worker] == 0 {
                Event::Load(worker)
            } else {
                Event::Reserve(worker)
            };
            next_stage[worker] += 1;
            schedule.push(event);
            extend(schedule, next_stage, output);
            schedule.pop();
            next_stage[worker] -= 1;
        }
    }

    let mut output = Vec::new();
    extend(&mut Vec::new(), &mut [0; MODEL_WORKER_COUNT], &mut output);
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


#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
struct AbstractState {
    phase: AttemptPhase,
    authority_current: bool,
    sink_applied: bool,
    effect_count: u8,
    dispatch_count: u8,
}

#[derive(Debug, Clone, Copy)]
enum AbstractEvent {
    Reserve,
    Revoke,
    StartDispatch,
    AbortBeforeDispatch,
    ApplyAndReply,
    ApplyAndLoseReply,
    CrashAfterDispatchBeforeApply,
    Reconcile,
    RetryAsNewOperation,
}

const ABSTRACT_EVENTS: [AbstractEvent; 9] = [
    AbstractEvent::Reserve,
    AbstractEvent::Revoke,
    AbstractEvent::StartDispatch,
    AbstractEvent::AbortBeforeDispatch,
    AbstractEvent::ApplyAndReply,
    AbstractEvent::ApplyAndLoseReply,
    AbstractEvent::CrashAfterDispatchBeforeApply,
    AbstractEvent::Reconcile,
    AbstractEvent::RetryAsNewOperation,
];

fn abstract_step(state: AbstractState, event: AbstractEvent) -> Option<AbstractState> {
    let mut next = state;
    match event {
        AbstractEvent::Reserve
            if next.phase == AttemptPhase::Prepared && next.authority_current =>
        {
            next.phase = AttemptPhase::Reserved;
            Some(next)
        }
        AbstractEvent::Revoke if next.authority_current => {
            next.authority_current = false;
            Some(next)
        }
        AbstractEvent::StartDispatch
            if next.phase == AttemptPhase::Reserved && next.authority_current =>
        {
            next.phase = AttemptPhase::DispatchStarted;
            next.dispatch_count = next.dispatch_count.saturating_add(1);
            Some(next)
        }
        AbstractEvent::AbortBeforeDispatch
            if matches!(next.phase, AttemptPhase::Prepared | AttemptPhase::Reserved) =>
        {
            next.phase = AttemptPhase::Rejected;
            Some(next)
        }
        AbstractEvent::ApplyAndReply
            if next.phase == AttemptPhase::DispatchStarted && !next.sink_applied =>
        {
            next.sink_applied = true;
            next.effect_count = next.effect_count.saturating_add(1);
            next.phase = AttemptPhase::Committed;
            Some(next)
        }
        AbstractEvent::ApplyAndLoseReply
            if next.phase == AttemptPhase::DispatchStarted && !next.sink_applied =>
        {
            next.sink_applied = true;
            next.effect_count = next.effect_count.saturating_add(1);
            next.phase = AttemptPhase::OutcomeUnknown;
            Some(next)
        }
        AbstractEvent::CrashAfterDispatchBeforeApply
            if next.phase == AttemptPhase::DispatchStarted && !next.sink_applied =>
        {
            next.phase = AttemptPhase::OutcomeUnknown;
            Some(next)
        }
        AbstractEvent::Reconcile
            if next.phase == AttemptPhase::OutcomeUnknown && next.sink_applied =>
        {
            next.phase = AttemptPhase::Committed;
            Some(next)
        }
        // No event authorizes a new logical effect from OutcomeUnknown.
        AbstractEvent::RetryAsNewOperation => None,
        _ => None,
    }
}

fn invariant_holds(state: AbstractState) -> bool {
    state.effect_count <= 1
        && state.dispatch_count <= 1
        && (state.sink_applied == (state.effect_count == 1))
        && (state.phase != AttemptPhase::Committed
            || (state.sink_applied && state.effect_count == 1 && state.dispatch_count == 1))
        && (!(state.phase == AttemptPhase::Rejected && state.dispatch_count == 0)
            || state.effect_count == 0)
        && (!(state.phase == AttemptPhase::OutcomeUnknown) || state.dispatch_count == 1)
        && (!matches!(state.phase, AttemptPhase::Prepared | AttemptPhase::Reserved)
            || (state.dispatch_count == 0 && !state.sink_applied))
}

fn explore_abstract_state_space(max_depth: usize) -> (HashSet<AbstractState>, usize) {
    let initial = AbstractState {
        phase: AttemptPhase::Prepared,
        authority_current: true,
        sink_applied: false,
        effect_count: 0,
        dispatch_count: 0,
    };
    let mut visited = HashSet::from([initial]);
    let mut queue = VecDeque::from([(initial, 0usize)]);
    let mut transitions = 0usize;

    while let Some((state, depth)) = queue.pop_front() {
        assert!(invariant_holds(state), "invariant failed at state {state:?}");
        if depth >= max_depth {
            continue;
        }
        for event in ABSTRACT_EVENTS {
            if let Some(next) = abstract_step(state, event) {
                transitions += 1;
                assert!(invariant_holds(next), "event {event:?} violated invariant at {next:?}");
                if visited.insert(next) {
                    queue.push_back((next, depth + 1));
                }
            }
        }
    }
    (visited, transitions)
}

fn explore_schedule(schedule: &[Event]) -> (usize, SharedRow) {
    let mut shared = SharedRow {
        snapshot: EffectAttempt::prepare(identity()).snapshot(),
    };
    let mut observations: [WorkerObservation; MODEL_WORKER_COUNT] =
        std::array::from_fn(|_| WorkerObservation::default());
    let mut winners = 0usize;

    for event in schedule {
        match *event {
            Event::Load(worker) => {
                observations[worker].revision = Some(shared.snapshot.revision);
                observations[worker].phase = Some(shared.snapshot.phase);
            }
            Event::Reserve(worker) => {
                let observed = &observations[worker];
                if shared.try_reserve(observed.revision, observed.phase, OPERATION_IDS[worker]) {
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
    fn every_four_worker_load_reserve_interleaving_has_one_winner() {
        let schedules = interleavings();
        assert_eq!(
            schedules.len(),
            2_520,
            "all order-preserving interleavings must be enumerated"
        );
        for schedule in &schedules {
            let (winners, shared) = explore_schedule(schedule);
            assert_eq!(winners, 1, "schedule {schedule:?}");
            assert_eq!(shared.snapshot.phase, AttemptPhase::Reserved);
            assert_eq!(shared.snapshot.revision, 1);
            assert!(OPERATION_IDS.contains(
                &shared.snapshot.operation_id.as_deref().unwrap_or_default()
            ));
        }
    }

    #[test]
    fn lost_reply_after_sink_commit_recovers_without_duplicate_effect() {
        let identity = identity();
        let mut attempt = EffectAttempt::prepare(identity.clone());
        attempt.reserve(OPERATION_IDS[0]).expect("reserve before dispatch");
        attempt.mark_dispatch_started().expect("durably mark dispatch started");
        let crash_cut = attempt.snapshot();

        let mut sink = IdempotentSink::default();
        let _lost_reply = sink
            .dispatch(&identity, OPERATION_IDS[0])
            .expect("first delivery applies");
        assert_eq!(sink.applied_effects, 1);

        // Simulate process restart: only the persisted snapshot survives.
        let mut recovered = EffectAttempt::restore(crash_cut).expect("restore uncertain dispatch");
        assert_eq!(recovered.phase(), AttemptPhase::DispatchStarted);

        let replayed_reply = sink
            .dispatch(&identity, OPERATION_IDS[0])
            .expect("same key returns stored result");
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
        sink.dispatch(&identity, OPERATION_IDS[0]).expect("first effect");
        let changed_target = EffectIdentity::new(
            identity.authority_domain(),
            identity.idempotency_key(),
            &"d".repeat(64),
            identity.policy_binding_digest(),
        )
        .expect("second identity is well-formed");
        assert!(matches!(
            sink.dispatch(&changed_target, OPERATION_IDS[0]),
            Err(ModelSinkError::Conflict)
        ));
        assert_eq!(sink.applied_effects, 1);
    }

    #[test]
    fn revocation_before_dispatch_aborts_reserved_attempt_without_sink_call() {
        let mut attempt = EffectAttempt::prepare(identity());
        attempt.reserve(OPERATION_IDS[0]).expect("reserve");
        let mut sink = IdempotentSink::default();
        let current_authority = false; // synthetic revocation between reservation and dispatch
        if current_authority {
            attempt.mark_dispatch_started().expect("only current authority may dispatch");
            sink.dispatch(attempt.identity(), OPERATION_IDS[0]).expect("dispatch");
        } else {
            attempt
                .abort_before_dispatch("authority-revoked")
                .expect("reserved work is pre-dispatch");
        }
        assert_eq!(attempt.phase(), AttemptPhase::Rejected);
        assert_eq!(sink.applied_effects, 0);
        assert_eq!(sink.delivery_attempts, 0);
    }
    #[test]
    fn bounded_state_exploration_preserves_effect_safety_invariants() {
        let (states, transitions) = explore_abstract_state_space(8);
        assert!(states.len() >= 8, "explorer should reach the meaningful lifecycle phases");
        assert!(transitions >= states.len(), "explorer should exercise distinct transitions");
        assert!(states.iter().all(|state| invariant_holds(*state)));
        assert!(states.iter().any(|state| {
            state.phase == AttemptPhase::OutcomeUnknown && state.sink_applied
        }));
        assert!(states.iter().any(|state| {
            state.phase == AttemptPhase::OutcomeUnknown && !state.sink_applied
        }));

        // Negative controls: demonstrate that the oracle is sensitive to the
        // exact duplicate-effect and false-sink-applied failures it is meant to detect.
        let duplicate_effect = AbstractState {
            phase: AttemptPhase::OutcomeUnknown,
            authority_current: true,
            sink_applied: true,
            effect_count: 2,
            dispatch_count: 1,
        };
        assert!(!invariant_holds(duplicate_effect));
        let false_commit = AbstractState {
            phase: AttemptPhase::Committed,
            authority_current: true,
            sink_applied: false,
            effect_count: 0,
            dispatch_count: 1,
        };
        assert!(!invariant_holds(false_commit));
    }

    #[test]
    fn outcome_unknown_has_no_dispatch_or_new_operation_transition() {
        let unknown = AbstractState {
            phase: AttemptPhase::OutcomeUnknown,
            authority_current: true,
            sink_applied: true,
            effect_count: 1,
            dispatch_count: 1,
        };
        assert!(abstract_step(unknown, AbstractEvent::StartDispatch).is_none());
        assert!(abstract_step(unknown, AbstractEvent::RetryAsNewOperation).is_none());
        let reconciled = abstract_step(unknown, AbstractEvent::Reconcile)
            .expect("matching sink result reconciles");
        assert_eq!(reconciled.phase, AttemptPhase::Committed);
        assert_eq!(reconciled.effect_count, 1);
    }

}
