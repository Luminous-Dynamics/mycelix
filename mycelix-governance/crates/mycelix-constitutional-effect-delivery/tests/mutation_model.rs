use mycelix_constitutional_effect_delivery::{
    DeliveryError, DeliveryRecord, DeliverySnapshot, DeliveryState, EffectBinding,
    Observation, ObservationKind, ReplayProfile,
};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
struct ModelBinding {
    instance: u64,
    contract: u64,
    payload: u64,
    sink: u64,
    epoch: u32,
}

impl EffectBinding for ModelBinding {
    type InstanceRoot = u64;
    type ContractRoot = u64;
    type PayloadCommitment = u64;
    type SinkRoot = u64;
    type SinkEpoch = u32;

    fn effect_instance_root(&self) -> &Self::InstanceRoot { &self.instance }
    fn effect_contract_root(&self) -> &Self::ContractRoot { &self.contract }
    fn payload_commitment(&self) -> &Self::PayloadCommitment { &self.payload }
    fn sink_root(&self) -> &Self::SinkRoot { &self.sink }
    fn sink_epoch(&self) -> &Self::SinkEpoch { &self.epoch }
}

fn binding() -> ModelBinding {
    ModelBinding { instance: 10, contract: 20, payload: 30, sink: 40, epoch: 1 }
}

fn drifted() -> ModelBinding {
    ModelBinding { instance: 10, contract: 20, payload: 31, sink: 40, epoch: 1 }
}

#[derive(Debug, Clone, Copy)]
enum Op {
    Start,
    StartWrongBinding,
    RetrySame,
    RetrySameWrongBinding,
    RetryNoEffect,
    RetryNoEffectWrongBinding,
    Unknown,
    UnknownWrongBinding,
    Success,
    SuccessWrongBinding,
    ReconcileNoEffect,
    ReconcileNoEffectBadTarget,
    ReconcileSuccess,
    DuplicateLast,
    Commit,
    Acknowledge,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Effect {
    Applied,
    Rejected,
    Halted,
    HistoricalCompletionPreserved,
    VolatileOnly,
}

#[derive(Debug, Clone, PartialEq, Eq)]
struct ModelObservation {
    sequence: u64,
    attempt: u64,
    kind: ObservationKind,
}

#[derive(Debug, Clone, PartialEq, Eq)]
struct Model {
    profile: ReplayProfile,
    state: DeliveryState,
    attempts: u64,
    observations: Vec<ModelObservation>,
    next_sequence: u64,
    completion: bool,
    halted: bool,
}

impl Model {
    fn new(profile: ReplayProfile) -> Self {
        Self {
            profile,
            state: DeliveryState::Pending,
            attempts: 0,
            observations: Vec::new(),
            next_sequence: 1,
            completion: false,
            halted: false,
        }
    }

    fn halt(&mut self) -> Effect {
        self.state = DeliveryState::IntegrityHalted;
        self.halted = true;
        Effect::Halted
    }

    fn observe(&mut self, attempt: u64, kind: ObservationKind) -> Effect {
        if self.halted {
            return Effect::Rejected;
        }

        if !self.observations.iter().any(|o| o.attempt == attempt) && self.attempts < attempt {
            return self.halt();
        }
        if attempt == 0 || attempt > self.attempts {
            return self.halt();
        }

        let sequence = self.next_sequence;
        self.next_sequence += 1;
        self.observations.push(ModelObservation { sequence, attempt, kind });

        self.state = match kind {
            ObservationKind::TransportAccepted | ObservationKind::ProviderAcknowledged => self.state,
            ObservationKind::OutcomeUnknown => match self.state {
                DeliveryState::AttemptStarted | DeliveryState::OutcomeUnknown => {
                    DeliveryState::OutcomeUnknown
                }
                _ => self.state,
            },
            ObservationKind::SemanticSuccess | ObservationKind::ReconciliationSuccess => {
                if self.state == DeliveryState::KnownNoEffect {
                    return self.halt();
                }
                DeliveryState::KnownSuccess
            }
            ObservationKind::ReconciliationNoEffect => {
                if self.state == DeliveryState::KnownSuccess {
                    return self.halt();
                }
                DeliveryState::KnownNoEffect
            }
        };
        Effect::Applied
    }

    fn step(&mut self, op: Op) -> Effect {
        match op {
            Op::Start => {
                if self.halted || self.state != DeliveryState::Pending || self.attempts != 0 {
                    return Effect::Rejected;
                }
                self.attempts += 1;
                self.state = DeliveryState::AttemptStarted;
                Effect::Applied
            }
            Op::StartWrongBinding => {
                if self.halted {
                    Effect::Rejected
                } else {
                    self.halt()
                }
            }
            Op::RetrySame => {
                if self.halted {
                    return Effect::Rejected;
                }
                if self.state == DeliveryState::OutcomeUnknown
                    && self.profile == ReplayProfile::IdempotentByEffectInstance
                {
                    self.attempts += 1;
                    self.state = DeliveryState::AttemptStarted;
                    Effect::Applied
                } else {
                    Effect::Rejected
                }
            }
            Op::RetrySameWrongBinding => {
                if self.halted { Effect::Rejected } else { self.halt() }
            }
            Op::RetryNoEffect => {
                if self.halted {
                    return Effect::Rejected;
                }
                if self.state == DeliveryState::KnownNoEffect
                    && self.profile == ReplayProfile::IdempotentByEffectInstance
                {
                    self.attempts += 1;
                    self.state = DeliveryState::AttemptStarted;
                    Effect::Applied
                } else {
                    Effect::Rejected
                }
            }
            Op::RetryNoEffectWrongBinding => {
                if self.halted { Effect::Rejected } else { self.halt() }
            }
            Op::Unknown => {
                let attempt = self.attempts;
                self.observe(attempt, ObservationKind::OutcomeUnknown)
            }
            Op::UnknownWrongBinding => {
                if self.halted { Effect::Rejected } else { self.halt() }
            }
            Op::Success => {
                let attempt = self.attempts;
                self.observe(attempt, ObservationKind::SemanticSuccess)
            }
            Op::SuccessWrongBinding => {
                if self.halted { Effect::Rejected } else { self.halt() }
            }
            Op::ReconcileNoEffect => {
                let attempt = self.attempts;
                if self.halted {
                    return Effect::Rejected;
                }
                let target = self.observations.last();
                let valid = matches!(
                    (self.state, target.map(|o| o.kind)),
                    (DeliveryState::OutcomeUnknown, Some(ObservationKind::OutcomeUnknown))
                        | (DeliveryState::KnownSuccess, Some(ObservationKind::SemanticSuccess))
                );
                if !valid {
                    return self.halt();
                }
                self.observe(attempt, ObservationKind::ReconciliationNoEffect)
            }
            Op::ReconcileNoEffectBadTarget => {
                if self.halted {
                    Effect::Rejected
                } else {
                    self.halt()
                }
            }
            Op::ReconcileSuccess => {
                let attempt = self.attempts;
                if self.halted {
                    return Effect::Rejected;
                }
                let target = self.observations.last();
                let valid = matches!(
                    (self.state, target.map(|o| o.kind)),
                    (DeliveryState::OutcomeUnknown, Some(ObservationKind::OutcomeUnknown))
                        | (DeliveryState::KnownNoEffect, Some(ObservationKind::ReconciliationNoEffect))
                );
                if !valid {
                    return self.halt();
                }
                self.observe(attempt, ObservationKind::ReconciliationSuccess)
            }
            Op::DuplicateLast => {
                if self.observations.is_empty() {
                    Effect::Rejected
                } else {
                    Effect::Applied
                }
            },
            Op::Commit => {
                if self.completion {
                    return Effect::HistoricalCompletionPreserved;
                }
                if self.state == DeliveryState::KnownSuccess {
                    self.completion = true;
                    Effect::Applied
                } else {
                    Effect::Rejected
                }
            }
            Op::Acknowledge => Effect::VolatileOnly,
        }
    }
}

fn record(profile: ReplayProfile) -> DeliveryRecord<ModelBinding> {
    DeliveryRecord::recover(DeliverySnapshot::from_test(binding(), profile)).unwrap()
}

trait SnapshotFixture<B: EffectBinding> {
    fn from_test(binding: B, profile: ReplayProfile) -> DeliverySnapshot<B>;
}

impl SnapshotFixture<ModelBinding> for DeliverySnapshot<ModelBinding> {
    fn from_test(binding: ModelBinding, profile: ReplayProfile) -> DeliverySnapshot<ModelBinding> {
        // Constructing through the public API is preferable; this fixture is only
        // used to obtain the empty record without depending on private fields.
        let intent = mycelix_constitutional_effect_delivery::PreparedDeliveryIntent::new(binding, profile);
        let committed = intent.apply_commit_outcome(
            mycelix_constitutional_effect_delivery::IntentCommitOutcome::Committed,
        ).unwrap();
        committed.start_record().snapshot()
    }
}

fn apply_op(record: &mut DeliveryRecord<ModelBinding>, op: Op) -> Result<(), DeliveryError> {
    match op {
        Op::Start => { record.start_initial_attempt(&binding()).map(|_| ()) }
        Op::StartWrongBinding => { record.start_initial_attempt(&drifted()).map(|_| ()) }
        Op::RetrySame => { record.retry_same_effect(&binding()).map(|_| ()) }
        Op::RetrySameWrongBinding => { record.retry_same_effect(&drifted()).map(|_| ()) }
        Op::RetryNoEffect => { record.retry_after_no_effect(&binding()).map(|_| ()) }
        Op::RetryNoEffectWrongBinding => { record.retry_after_no_effect(&drifted()).map(|_| ()) }
        Op::Unknown => {
            let id = match record.attempts().last() {
                Some(a) => a.id(),
                None => return record.start_initial_attempt(&drifted()).map(|_| ()),
            };
            record.observe(Observation::outcome_unknown(binding(), id, 100 + record.observations().len() as u64)).map(|_| ())
        }
        Op::UnknownWrongBinding => {
            let id = match record.attempts().last() {
                Some(a) => a.id(),
                None => return record.start_initial_attempt(&drifted()).map(|_| ()),
            };
            record.observe(Observation::outcome_unknown(drifted(), id, 200 + record.observations().len() as u64)).map(|_| ())
        }
        Op::Success => {
            let id = match record.attempts().last() {
                Some(a) => a.id(),
                None => return record.start_initial_attempt(&drifted()).map(|_| ()),
            };
            record.observe(Observation::semantic_success(binding(), id, 300 + record.observations().len() as u64)).map(|_| ())
        }
        Op::SuccessWrongBinding => {
            let id = match record.attempts().last() {
                Some(a) => a.id(),
                None => return record.start_initial_attempt(&drifted()).map(|_| ()),
            };
            record.observe(Observation::semantic_success(drifted(), id, 400 + record.observations().len() as u64)).map(|_| ())
        }
        Op::ReconcileNoEffect => {
            let id = match record.attempts().last() {
                Some(a) => a.id(),
                None => return record.start_initial_attempt(&drifted()).map(|_| ()),
            };
            let target = record.observations().last().map(|o| o.sequence()).unwrap_or(0);
            record.observe(Observation::reconciliation_no_effect(binding(), id, 500 + record.observations().len() as u64, target)).map(|_| ())
        }
        Op::ReconcileNoEffectBadTarget => {
            let id = match record.attempts().last() {
                Some(a) => a.id(),
                None => return record.start_initial_attempt(&drifted()).map(|_| ()),
            };
            record.observe(Observation::reconciliation_no_effect(binding(), id, 600 + record.observations().len() as u64, u64::MAX)).map(|_| ())
        }
        Op::ReconcileSuccess => {
            let id = record.attempts().last().map(|a| a.id()).unwrap_or(crate::dummy_attempt_id());
            let target = record.observations().last().map(|o| o.sequence()).unwrap_or(0);
            record.observe(Observation::reconciliation_success(binding(), id, 700 + record.observations().len() as u64, target)).map(|_| ())
        }
        Op::DuplicateLast => {
            let observation = record.observations().last().cloned();
            match observation {
                Some(o) => record.observe(o).map(|_| ()),
                None => Err(DeliveryError::WrongAttempt),
            }
        }
        Op::Commit => record.commit_completion(),
        Op::Acknowledge => {
            record.acknowledge_caller();
            Ok(())
        }
    }
}

fn assert_projection(model: &Model, record: &DeliveryRecord<ModelBinding>) {
    assert_eq!(record.state(), model.state);
    assert_eq!(record.attempts().len() as u64, model.attempts);
    assert_eq!(record.observations().len(), model.observations.len());
    assert_eq!(record.completion_committed(), model.completion);
    assert_eq!(record.halt_provenance().is_some(), model.halted);
    for (actual, expected) in record.observations().iter().zip(&model.observations) {
        assert_eq!(actual.sequence(), expected.sequence);
        assert_eq!(actual.attempt_id().raw(), expected.attempt);
        assert_eq!(actual.kind(), expected.kind);
    }
}

fn sequence(depth: usize, prefix: &mut Vec<Op>, out: &mut Vec<Vec<Op>>) {
    if depth == 0 {
        out.push(prefix.clone());
        return;
    }
    const OPS: [Op; 16] = [
        Op::Start,
        Op::StartWrongBinding,
        Op::RetrySame,
        Op::RetrySameWrongBinding,
        Op::RetryNoEffect,
        Op::RetryNoEffectWrongBinding,
        Op::Unknown,
        Op::UnknownWrongBinding,
        Op::Success,
        Op::SuccessWrongBinding,
        Op::ReconcileNoEffect,
        Op::ReconcileNoEffectBadTarget,
        Op::ReconcileSuccess,
        Op::DuplicateLast,
        Op::Commit,
        Op::Acknowledge,
    ];
    for op in OPS {
        prefix.push(op);
        sequence(depth - 1, prefix, out);
        prefix.pop();
    }
}

#[test]
fn bounded_mutation_model_checks_prefixes_and_recovery() {
    let mut cases = Vec::new();
    sequence(3, &mut Vec::new(), &mut cases);

    for profile in [ReplayProfile::NoAutomaticRetry, ReplayProfile::IdempotentByEffectInstance] {
        for operations in &cases {
            let mut model = Model::new(profile);
            let mut record = record(profile);

            for (index, op) in operations.iter().copied().enumerate() {
                let before = record.snapshot();
                let expected = model.step(op);
                let actual = apply_op(&mut record, op);

                match expected {
                    Effect::VolatileOnly => {
                        assert!(actual.is_ok(), "step {index} {op:?}");
                        assert_eq!(record.snapshot(), before, "volatile mutation leaked at {index}");
                    }
                    Effect::HistoricalCompletionPreserved => {
                        assert_eq!(actual, Err(DeliveryError::AlreadyCompleted), "step {index} {op:?}");
                    }
                    Effect::Applied => {
                        assert!(actual.is_ok(), "step {index} {op:?}: {actual:?}");
                    }
                    Effect::Rejected => {
                        assert!(actual.is_err(), "step {index} {op:?} unexpectedly succeeded");
                    }
                    Effect::Halted => {
                        assert!(actual.is_err(), "step {index} {op:?} unexpectedly succeeded");
                    }
                }

                assert_projection(&model, &record);

                let recovered = DeliveryRecord::recover(record.snapshot()).unwrap();
                assert_projection(&model, &recovered);
                assert_eq!(record.snapshot(), recovered.snapshot(), "recovery drift at {index}");

                record = recovered;
            }
        }
    }
}
