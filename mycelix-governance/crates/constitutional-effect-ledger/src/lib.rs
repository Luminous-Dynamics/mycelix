use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};
use thiserror::Error;

pub const EFFECT_LEDGER_SCHEMA_VERSION: u16 = 1;
pub const MAX_ACTIONS: usize = 256;
pub const MAX_ID_LEN: usize = 256;
pub const MAX_COMMITMENT_LEN: usize = 512;

#[derive(Debug, Error, Clone, PartialEq, Eq)]
pub enum LedgerError {
    #[error("{0}")]
    Violation(String),
}

type LedgerResult<T> = Result<T, LedgerError>;

fn violation(message: impl Into<String>) -> LedgerError {
    LedgerError::Violation(message.into())
}

fn valid_opaque(value: &str, max_len: usize) -> bool {
    !value.trim().is_empty() && value.len() <= max_len
}

fn require_opaque(label: &str, value: &str, max_len: usize) -> LedgerResult<()> {
    if valid_opaque(value, max_len) {
        Ok(())
    } else {
        Err(violation(format!(
            "{label} must be non-empty and <= {max_len} bytes"
        )))
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ReplaySafety {
    Unknown,
    StableKeyIdempotent,
    ProviderTokenIdempotent,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum OutcomeObservability {
    Unknown,
    Opaque,
    Queryable,
    ReceiptVerifiable,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum CompensationCapability {
    Unknown,
    ForwardCompensatable,
    Irreversible,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ProviderBatchAtomicity {
    NoneObserved,
    QualifiedAtomic,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct CapabilitySnapshot {
    pub action_type: String,
    pub profile_id: String,
    pub profile_revision: u32,
    pub replay_safety: ReplaySafety,
    pub outcome_observability: OutcomeObservability,
    pub compensation: CompensationCapability,
    pub provider_batch_atomicity: ProviderBatchAtomicity,
    pub safe_retry_without_reconciliation: bool,
    pub qualification_evidence_id: Option<String>,
}

impl CapabilitySnapshot {
    pub fn validate(&self) -> LedgerResult<()> {
        require_opaque("capability action_type", &self.action_type, MAX_ID_LEN)?;
        require_opaque("capability profile_id", &self.profile_id, MAX_ID_LEN)?;
        if self.profile_revision == 0 {
            return Err(violation("capability profile_revision must be > 0"));
        }

        if self.safe_retry_without_reconciliation && self.replay_safety == ReplaySafety::Unknown {
            return Err(violation(
                "safe retry cannot be granted when replay safety is Unknown",
            ));
        }

        let positive_claim = self.replay_safety != ReplaySafety::Unknown
            || matches!(
                self.outcome_observability,
                OutcomeObservability::Queryable | OutcomeObservability::ReceiptVerifiable
            )
            || self.compensation != CompensationCapability::Unknown
            || self.provider_batch_atomicity == ProviderBatchAtomicity::QualifiedAtomic
            || self.safe_retry_without_reconciliation;

        if positive_claim {
            let evidence = self.qualification_evidence_id.as_deref().ok_or_else(|| {
                violation("positive provider capabilities require qualification_evidence_id")
            })?;
            require_opaque("qualification_evidence_id", evidence, MAX_COMMITMENT_LEN)?;
        } else if let Some(evidence) = &self.qualification_evidence_id {
            require_opaque("qualification_evidence_id", evidence, MAX_COMMITMENT_LEN)?;
        }

        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ConstitutionalOperation {
    pub schema_version: u16,
    pub operation_id: String,
    pub proposal_id: String,
    pub timelock_id: String,
    pub claim_binding: String,
    pub capability_profile_id: String,
    pub capability_profile_revision: u32,
    pub ordered_action_commitments: Vec<String>,
    pub committed_at_unix_ms: u64,
}

impl ConstitutionalOperation {
    pub fn validate(&self) -> LedgerResult<()> {
        if self.schema_version != EFFECT_LEDGER_SCHEMA_VERSION {
            return Err(violation(format!(
                "unsupported effect ledger schema version {}",
                self.schema_version
            )));
        }

        require_opaque("operation_id", &self.operation_id, MAX_ID_LEN)?;
        require_opaque("proposal_id", &self.proposal_id, MAX_ID_LEN)?;
        require_opaque("timelock_id", &self.timelock_id, MAX_ID_LEN)?;
        require_opaque("claim_binding", &self.claim_binding, MAX_COMMITMENT_LEN)?;
        require_opaque(
            "capability_profile_id",
            &self.capability_profile_id,
            MAX_ID_LEN,
        )?;

        if self.capability_profile_revision == 0 {
            return Err(violation(
                "capability_profile_revision must be greater than zero",
            ));
        }

        if self.ordered_action_commitments.is_empty()
            || self.ordered_action_commitments.len() > MAX_ACTIONS
        {
            return Err(violation(format!(
                "operation must contain 1..={MAX_ACTIONS} ordered actions"
            )));
        }

        for (ordinal, commitment) in self.ordered_action_commitments.iter().enumerate() {
            require_opaque(
                &format!("ordered_action_commitments[{ordinal}]"),
                commitment,
                MAX_COMMITMENT_LEN,
            )?;
        }

        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ActionIntent {
    pub operation_id: String,
    pub action_id: String,
    pub ordinal: u32,
    pub action_commitment: String,
    pub capability: CapabilitySnapshot,
    pub idempotency_identity: Option<String>,
}

impl ActionIntent {
    pub fn validate(&self) -> LedgerResult<()> {
        require_opaque("intent operation_id", &self.operation_id, MAX_ID_LEN)?;
        require_opaque("action_id", &self.action_id, MAX_ID_LEN)?;
        require_opaque(
            "action_commitment",
            &self.action_commitment,
            MAX_COMMITMENT_LEN,
        )?;
        self.capability.validate()?;

        if let Some(identity) = &self.idempotency_identity {
            require_opaque("idempotency_identity", identity, MAX_COMMITMENT_LEN)?;
        }

        Ok(())
    }

    pub fn validate_against_operation(
        &self,
        operation: &ConstitutionalOperation,
    ) -> LedgerResult<()> {
        self.validate()?;
        if self.operation_id != operation.operation_id {
            return Err(violation("action intent operation_id mismatch"));
        }
        let ordinal = self.ordinal as usize;
        let expected = operation
            .ordered_action_commitments
            .get(ordinal)
            .ok_or_else(|| violation("action intent ordinal outside operation plan"))?;
        if expected != &self.action_commitment {
            return Err(violation(
                "action intent commitment does not match ordered operation plan",
            ));
        }
        if self.capability.profile_id != operation.capability_profile_id
            || self.capability.profile_revision != operation.capability_profile_revision
        {
            return Err(violation(
                "action capability profile does not match operation capability profile",
            ));
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum RetryAuthorization {
    Initial,
    ReplaySafe,
    ReconciledNoEffect { observation_id: String },
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ActionAttempt {
    pub operation_id: String,
    pub action_id: String,
    pub attempt_id: String,
    pub attempt_ordinal: u32,
    pub event_seq: u64,
    pub started_at_unix_ms: u64,
    pub idempotency_identity: Option<String>,
    pub retry_authorization: RetryAuthorization,
}

impl ActionAttempt {
    pub fn validate_against_intent(&self, intent: &ActionIntent) -> LedgerResult<()> {
        require_opaque("attempt operation_id", &self.operation_id, MAX_ID_LEN)?;
        require_opaque("attempt action_id", &self.action_id, MAX_ID_LEN)?;
        require_opaque("attempt_id", &self.attempt_id, MAX_ID_LEN)?;

        if self.operation_id != intent.operation_id || self.action_id != intent.action_id {
            return Err(violation("attempt identity does not match action intent"));
        }
        if self.attempt_ordinal == 0 {
            return Err(violation("attempt_ordinal must be >= 1"));
        }
        if self.event_seq == 0 {
            return Err(violation("attempt event_seq must be >= 1"));
        }
        if self.idempotency_identity != intent.idempotency_identity {
            return Err(violation(
                "attempt changed the action's stable idempotency identity",
            ));
        }

        match (&self.retry_authorization, self.attempt_ordinal) {
            (RetryAuthorization::Initial, 1) => {}
            (RetryAuthorization::Initial, _) => {
                return Err(violation(
                    "only the first attempt may use Initial authorization",
                ));
            }
            (RetryAuthorization::ReplaySafe, 1)
            | (RetryAuthorization::ReconciledNoEffect { .. }, 1) => {
                return Err(violation("first attempt must use Initial authorization"));
            }
            (RetryAuthorization::ReplaySafe, _) => {
                if !intent.capability.safe_retry_without_reconciliation
                    || intent.capability.replay_safety == ReplaySafety::Unknown
                {
                    return Err(violation(
                        "ReplaySafe retry requires qualified replay-safe capability",
                    ));
                }
            }
            (RetryAuthorization::ReconciledNoEffect { observation_id }, _) => {
                require_opaque(
                    "retry reconciliation observation_id",
                    observation_id,
                    MAX_ID_LEN,
                )?;
            }
        }

        Ok(())
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum EffectOutcome {
    KnownSuccess,
    KnownNoEffect,
    UnknownOutcome,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ObservationBasis {
    DispatchResponse,
    ProviderReceipt,
    ProviderQuery,
    Reconciliation,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct EffectObservation {
    pub operation_id: String,
    pub action_id: String,
    pub attempt_id: String,
    pub observation_id: String,
    pub event_seq: u64,
    pub observed_at_unix_ms: u64,
    pub outcome: EffectOutcome,
    pub basis: ObservationBasis,
    pub evidence_commitment: String,
    pub provider_receipt_commitment: Option<String>,
    pub reconciles_observation_id: Option<String>,
}

impl EffectObservation {
    pub fn validate_against_attempt(&self, attempt: &ActionAttempt) -> LedgerResult<()> {
        require_opaque("observation operation_id", &self.operation_id, MAX_ID_LEN)?;
        require_opaque("observation action_id", &self.action_id, MAX_ID_LEN)?;
        require_opaque("observation attempt_id", &self.attempt_id, MAX_ID_LEN)?;
        require_opaque("observation_id", &self.observation_id, MAX_ID_LEN)?;
        require_opaque(
            "evidence_commitment",
            &self.evidence_commitment,
            MAX_COMMITMENT_LEN,
        )?;

        if self.operation_id != attempt.operation_id
            || self.action_id != attempt.action_id
            || self.attempt_id != attempt.attempt_id
        {
            return Err(violation(
                "observation identity does not match referenced attempt",
            ));
        }
        if self.event_seq == 0 {
            return Err(violation("observation event_seq must be >= 1"));
        }
        if self.event_seq <= attempt.event_seq {
            return Err(violation(
                "observation must be sequenced after its action attempt",
            ));
        }

        if self.outcome == EffectOutcome::KnownNoEffect
            && !matches!(
                self.basis,
                ObservationBasis::ProviderQuery | ObservationBasis::Reconciliation
            )
        {
            return Err(violation(
                "KnownNoEffect requires provider-query or reconciliation evidence",
            ));
        }

        if self.basis == ObservationBasis::ProviderReceipt {
            let receipt = self.provider_receipt_commitment.as_deref().ok_or_else(|| {
                violation("ProviderReceipt observation requires provider_receipt_commitment")
            })?;
            require_opaque("provider_receipt_commitment", receipt, MAX_COMMITMENT_LEN)?;
        } else if let Some(receipt) = &self.provider_receipt_commitment {
            require_opaque("provider_receipt_commitment", receipt, MAX_COMMITMENT_LEN)?;
        }

        match self.basis {
            ObservationBasis::Reconciliation => {
                let prior = self.reconciles_observation_id.as_deref().ok_or_else(|| {
                    violation("Reconciliation observation must identify the unknown it resolves")
                })?;
                require_opaque("reconciles_observation_id", prior, MAX_ID_LEN)?;
            }
            _ if self.reconciles_observation_id.is_some() => {
                return Err(violation(
                    "only Reconciliation observations may resolve a prior observation",
                ));
            }
            _ => {}
        }

        Ok(())
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum OperationResolutionKind {
    Completed,
    FailedNoEffect,
    PartiallyCompleted,
    CompensationRequired,
    UnknownOutcome,
    IntegrityHalted,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct OperationResolution {
    pub operation_id: String,
    pub resolution_id: String,
    pub kind: OperationResolutionKind,
    pub successful_ordinals: Vec<u32>,
    pub known_no_effect_ordinals: Vec<u32>,
    pub unknown_ordinals: Vec<u32>,
    pub compensation_required_ordinals: Vec<u32>,
    pub evidence_commitment: String,
    pub resolved_at_unix_ms: u64,
}

impl OperationResolution {
    fn validate_shape(&self, action_count: usize) -> LedgerResult<()> {
        require_opaque("resolution operation_id", &self.operation_id, MAX_ID_LEN)?;
        require_opaque("resolution_id", &self.resolution_id, MAX_ID_LEN)?;
        require_opaque(
            "resolution evidence_commitment",
            &self.evidence_commitment,
            MAX_COMMITMENT_LEN,
        )?;

        for (label, values) in [
            ("successful_ordinals", &self.successful_ordinals),
            ("known_no_effect_ordinals", &self.known_no_effect_ordinals),
            ("unknown_ordinals", &self.unknown_ordinals),
            (
                "compensation_required_ordinals",
                &self.compensation_required_ordinals,
            ),
        ] {
            let mut seen = BTreeSet::new();
            for value in values {
                if *value as usize >= action_count {
                    return Err(violation(format!(
                        "{label} contains ordinal outside operation plan"
                    )));
                }
                if !seen.insert(*value) {
                    return Err(violation(format!("{label} contains duplicate ordinal")));
                }
            }
        }

        let success: BTreeSet<_> = self.successful_ordinals.iter().copied().collect();
        let no_effect: BTreeSet<_> = self.known_no_effect_ordinals.iter().copied().collect();
        let unknown: BTreeSet<_> = self.unknown_ordinals.iter().copied().collect();

        if !success.is_disjoint(&no_effect)
            || !success.is_disjoint(&unknown)
            || !no_effect.is_disjoint(&unknown)
        {
            return Err(violation(
                "resolution outcome sets must be pairwise disjoint",
            ));
        }

        if self
            .compensation_required_ordinals
            .iter()
            .any(|ordinal| !success.contains(ordinal))
        {
            return Err(violation(
                "compensation may only be required for an observed successful effect",
            ));
        }

        if self.kind != OperationResolutionKind::CompensationRequired
            && !self.compensation_required_ordinals.is_empty()
        {
            return Err(violation(
                "compensation ordinals require CompensationRequired resolution",
            ));
        }

        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct EvidenceBundle {
    pub operation: ConstitutionalOperation,
    pub intents: Vec<ActionIntent>,
    pub attempts: Vec<ActionAttempt>,
    pub observations: Vec<EffectObservation>,
    pub resolution: Option<OperationResolution>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DerivedActionState {
    Pending,
    KnownSuccess,
    KnownNoEffect,
    UnknownOutcome,
    IntegrityHalted,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DerivedOperationState {
    pub actions: Vec<DerivedActionState>,
    pub successful_ordinals: Vec<u32>,
    pub known_no_effect_ordinals: Vec<u32>,
    pub unknown_ordinals: Vec<u32>,
    pub pending_ordinals: Vec<u32>,
    pub integrity_halted: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum AttemptKnowledge {
    None,
    Unknown,
    Success,
    NoEffect,
    Conflict,
}

pub fn validate_bundle(bundle: &EvidenceBundle) -> LedgerResult<DerivedOperationState> {
    bundle.operation.validate()?;
    let action_count = bundle.operation.ordered_action_commitments.len();

    if bundle.intents.len() != action_count {
        return Err(violation(
            "operation must have exactly one ActionIntent for each ordered action",
        ));
    }

    let mut intents_by_ordinal = BTreeMap::<u32, &ActionIntent>::new();
    let mut intents_by_id = BTreeMap::<&str, &ActionIntent>::new();

    for intent in &bundle.intents {
        intent.validate_against_operation(&bundle.operation)?;
        if intents_by_ordinal.insert(intent.ordinal, intent).is_some() {
            return Err(violation("duplicate action intent ordinal"));
        }
        if intents_by_id
            .insert(intent.action_id.as_str(), intent)
            .is_some()
        {
            return Err(violation("duplicate action_id"));
        }
    }

    for ordinal in 0..action_count as u32 {
        if !intents_by_ordinal.contains_key(&ordinal) {
            return Err(violation(
                "action intent ordinals must be contiguous from zero",
            ));
        }
    }

    let mut used_event_seq = BTreeSet::new();
    let mut attempts_by_id = BTreeMap::<&str, &ActionAttempt>::new();
    let mut attempts_by_action = BTreeMap::<&str, Vec<&ActionAttempt>>::new();

    for attempt in &bundle.attempts {
        let intent = intents_by_id
            .get(attempt.action_id.as_str())
            .copied()
            .ok_or_else(|| violation("attempt references unknown action_id"))?;
        attempt.validate_against_intent(intent)?;
        if attempts_by_id
            .insert(attempt.attempt_id.as_str(), attempt)
            .is_some()
        {
            return Err(violation("duplicate attempt_id"));
        }
        if !used_event_seq.insert(attempt.event_seq) {
            return Err(violation("duplicate event_seq"));
        }
        attempts_by_action
            .entry(attempt.action_id.as_str())
            .or_default()
            .push(attempt);
    }

    for attempts in attempts_by_action.values_mut() {
        attempts.sort_by_key(|attempt| attempt.attempt_ordinal);
        for (index, attempt) in attempts.iter().enumerate() {
            let expected = index as u32 + 1;
            if attempt.attempt_ordinal != expected {
                return Err(violation(
                    "attempt ordinals must be contiguous and start at one",
                ));
            }
            if index > 0 && attempt.event_seq <= attempts[index - 1].event_seq {
                return Err(violation(
                    "later attempt ordinal must have a greater event_seq",
                ));
            }
        }
    }

    let mut observations_by_id = BTreeMap::<&str, &EffectObservation>::new();
    let mut observations_by_attempt = BTreeMap::<&str, Vec<&EffectObservation>>::new();

    for observation in &bundle.observations {
        let attempt = attempts_by_id
            .get(observation.attempt_id.as_str())
            .copied()
            .ok_or_else(|| violation("observation references unknown attempt_id"))?;
        observation.validate_against_attempt(attempt)?;
        if observations_by_id
            .insert(observation.observation_id.as_str(), observation)
            .is_some()
        {
            return Err(violation("duplicate observation_id"));
        }
        if !used_event_seq.insert(observation.event_seq) {
            return Err(violation("duplicate event_seq"));
        }
        observations_by_attempt
            .entry(observation.attempt_id.as_str())
            .or_default()
            .push(observation);
    }

    for observations in observations_by_attempt.values_mut() {
        observations.sort_by_key(|observation| observation.event_seq);
    }

    let mut attempt_knowledge = BTreeMap::<&str, AttemptKnowledge>::new();
    for (attempt_id, attempt) in &attempts_by_id {
        let observations = observations_by_attempt
            .get(attempt_id)
            .cloned()
            .unwrap_or_default();
        let mut knowledge = AttemptKnowledge::None;
        let mut unknown_ids = BTreeSet::<&str>::new();

        for observation in observations {
            if observation.basis == ObservationBasis::Reconciliation {
                let prior_id = observation
                    .reconciles_observation_id
                    .as_deref()
                    .ok_or_else(|| violation("reconciliation missing prior observation"))?;
                let prior = observations_by_id
                    .get(prior_id)
                    .copied()
                    .ok_or_else(|| violation("reconciliation references unknown observation"))?;
                if prior.attempt_id != attempt.attempt_id
                    || prior.outcome != EffectOutcome::UnknownOutcome
                    || prior.event_seq >= observation.event_seq
                    || !unknown_ids.contains(prior.observation_id.as_str())
                {
                    return Err(violation(
                        "reconciliation must resolve an earlier UnknownOutcome from the same attempt",
                    ));
                }
            }

            match (knowledge, observation.outcome) {
                (AttemptKnowledge::None, EffectOutcome::UnknownOutcome)
                | (AttemptKnowledge::Unknown, EffectOutcome::UnknownOutcome) => {
                    knowledge = AttemptKnowledge::Unknown;
                    unknown_ids.insert(observation.observation_id.as_str());
                }
                (AttemptKnowledge::None, EffectOutcome::KnownSuccess)
                | (AttemptKnowledge::Unknown, EffectOutcome::KnownSuccess)
                | (AttemptKnowledge::Success, EffectOutcome::KnownSuccess) => {
                    knowledge = AttemptKnowledge::Success;
                }
                (AttemptKnowledge::None, EffectOutcome::KnownNoEffect)
                | (AttemptKnowledge::Unknown, EffectOutcome::KnownNoEffect)
                | (AttemptKnowledge::NoEffect, EffectOutcome::KnownNoEffect) => {
                    knowledge = AttemptKnowledge::NoEffect;
                }
                (AttemptKnowledge::Success, EffectOutcome::KnownNoEffect)
                | (AttemptKnowledge::NoEffect, EffectOutcome::KnownSuccess)
                | (AttemptKnowledge::Success, EffectOutcome::UnknownOutcome)
                | (AttemptKnowledge::NoEffect, EffectOutcome::UnknownOutcome)
                | (AttemptKnowledge::Conflict, _) => {
                    knowledge = AttemptKnowledge::Conflict;
                }
            }
        }

        attempt_knowledge.insert(*attempt_id, knowledge);
    }

    for (action_id, attempts) in &attempts_by_action {
        let intent = intents_by_id
            .get(action_id)
            .copied()
            .ok_or_else(|| violation("attempt set references unknown action intent"))?;

        for (index, attempt) in attempts.iter().enumerate() {
            if index == 0 {
                continue;
            }

            let prior = attempts[index - 1];
            let prior_knowledge = attempt_knowledge
                .get(prior.attempt_id.as_str())
                .copied()
                .unwrap_or(AttemptKnowledge::None);

            if prior_knowledge == AttemptKnowledge::Success {
                return Err(violation("cannot retry an action after known success"));
            }
            if prior_knowledge == AttemptKnowledge::Conflict {
                return Err(violation(
                    "cannot retry an action after contradictory provider evidence",
                ));
            }

            match &attempt.retry_authorization {
                RetryAuthorization::ReplaySafe => {
                    if !intent.capability.safe_retry_without_reconciliation {
                        return Err(violation(
                            "retry claims ReplaySafe without qualified capability",
                        ));
                    }
                }
                RetryAuthorization::ReconciledNoEffect { observation_id } => {
                    let observation = observations_by_id
                        .get(observation_id.as_str())
                        .copied()
                        .ok_or_else(|| {
                            violation(
                                "retry authorization references unknown reconciliation observation",
                            )
                        })?;
                    if observation.attempt_id != prior.attempt_id
                        || observation.outcome != EffectOutcome::KnownNoEffect
                        || !matches!(
                            observation.basis,
                            ObservationBasis::ProviderQuery | ObservationBasis::Reconciliation
                        )
                        || observation.event_seq >= attempt.event_seq
                    {
                        return Err(violation(
                            "retry requires earlier authoritative KnownNoEffect evidence for the prior attempt",
                        ));
                    }
                }
                RetryAuthorization::Initial => {
                    return Err(violation("retry cannot use Initial authorization"));
                }
            }
        }
    }

    let mut derived_actions = vec![DerivedActionState::Pending; action_count];
    let mut first_attempt_seq = vec![None::<u64>; action_count];
    let mut success_seq = vec![None::<u64>; action_count];

    for ordinal in 0..action_count as u32 {
        let intent = intents_by_ordinal
            .get(&ordinal)
            .copied()
            .ok_or_else(|| violation("missing action intent ordinal"))?;
        let attempts = attempts_by_action
            .get(intent.action_id.as_str())
            .cloned()
            .unwrap_or_default();

        if let Some(first) = attempts.first() {
            first_attempt_seq[ordinal as usize] = Some(first.event_seq);
        }

        let Some(last) = attempts.last() else {
            continue;
        };

        let knowledge = attempt_knowledge
            .get(last.attempt_id.as_str())
            .copied()
            .unwrap_or(AttemptKnowledge::None);

        derived_actions[ordinal as usize] = match knowledge {
            AttemptKnowledge::None => DerivedActionState::Pending,
            AttemptKnowledge::Unknown => DerivedActionState::UnknownOutcome,
            AttemptKnowledge::Success => {
                let seq = observations_by_attempt
                    .get(last.attempt_id.as_str())
                    .into_iter()
                    .flatten()
                    .filter(|observation| observation.outcome == EffectOutcome::KnownSuccess)
                    .map(|observation| observation.event_seq)
                    .min()
                    .ok_or_else(|| violation("known success missing success observation"))?;
                success_seq[ordinal as usize] = Some(seq);
                DerivedActionState::KnownSuccess
            }
            AttemptKnowledge::NoEffect => DerivedActionState::KnownNoEffect,
            AttemptKnowledge::Conflict => DerivedActionState::IntegrityHalted,
        };
    }

    for ordinal in 1..action_count {
        if let Some(start_seq) = first_attempt_seq[ordinal] {
            let predecessor_success_seq = success_seq[ordinal - 1].ok_or_else(|| {
                violation("later action attempted before predecessor was observed successful")
            })?;
            if predecessor_success_seq >= start_seq {
                return Err(violation(
                    "later action attempt was sequenced before predecessor success evidence",
                ));
            }
        }
    }

    let mut state = DerivedOperationState {
        actions: derived_actions,
        successful_ordinals: Vec::new(),
        known_no_effect_ordinals: Vec::new(),
        unknown_ordinals: Vec::new(),
        pending_ordinals: Vec::new(),
        integrity_halted: false,
    };

    for (ordinal, action_state) in state.actions.iter().enumerate() {
        match action_state {
            DerivedActionState::Pending => state.pending_ordinals.push(ordinal as u32),
            DerivedActionState::KnownSuccess => {
                state.successful_ordinals.push(ordinal as u32);
            }
            DerivedActionState::KnownNoEffect => {
                state.known_no_effect_ordinals.push(ordinal as u32);
            }
            DerivedActionState::UnknownOutcome => {
                state.unknown_ordinals.push(ordinal as u32);
            }
            DerivedActionState::IntegrityHalted => {
                state.integrity_halted = true;
            }
        }
    }

    if let Some(resolution) = &bundle.resolution {
        validate_resolution(&bundle.operation, &state, resolution)?;
    }

    Ok(state)
}

fn validate_resolution(
    operation: &ConstitutionalOperation,
    state: &DerivedOperationState,
    resolution: &OperationResolution,
) -> LedgerResult<()> {
    let action_count = operation.ordered_action_commitments.len();
    resolution.validate_shape(action_count)?;

    if resolution.operation_id != operation.operation_id {
        return Err(violation("resolution operation_id mismatch"));
    }

    if resolution.successful_ordinals != state.successful_ordinals
        || resolution.known_no_effect_ordinals != state.known_no_effect_ordinals
        || resolution.unknown_ordinals != state.unknown_ordinals
    {
        return Err(violation(
            "resolution outcome sets do not match derived append-only evidence",
        ));
    }

    match resolution.kind {
        OperationResolutionKind::Completed => {
            if state.integrity_halted
                || !state.known_no_effect_ordinals.is_empty()
                || !state.unknown_ordinals.is_empty()
                || !state.pending_ordinals.is_empty()
                || state.successful_ordinals.len() != action_count
            {
                return Err(violation(
                    "Completed requires every action to be observed successful",
                ));
            }
        }
        OperationResolutionKind::FailedNoEffect => {
            if state.integrity_halted
                || !state.successful_ordinals.is_empty()
                || !state.unknown_ordinals.is_empty()
                || state.known_no_effect_ordinals.is_empty()
            {
                return Err(violation(
                    "FailedNoEffect requires known no-effect failure and zero successful effects",
                ));
            }
        }
        OperationResolutionKind::PartiallyCompleted => {
            if state.integrity_halted
                || state.successful_ordinals.is_empty()
                || !state.unknown_ordinals.is_empty()
                || (state.pending_ordinals.is_empty() && state.known_no_effect_ordinals.is_empty())
            {
                return Err(violation(
                    "PartiallyCompleted requires a successful prefix plus incomplete/known-no-effect remainder",
                ));
            }
        }
        OperationResolutionKind::CompensationRequired => {
            if state.integrity_halted
                || state.successful_ordinals.is_empty()
                || !state.unknown_ordinals.is_empty()
                || resolution.compensation_required_ordinals.is_empty()
            {
                return Err(violation(
                    "CompensationRequired requires successful effects, no unresolved unknown, and explicit compensation targets",
                ));
            }
        }
        OperationResolutionKind::UnknownOutcome => {
            if state.integrity_halted || state.unknown_ordinals.is_empty() {
                return Err(violation(
                    "UnknownOutcome resolution requires at least one unresolved unknown action",
                ));
            }
        }
        OperationResolutionKind::IntegrityHalted => {
            if !state.integrity_halted {
                return Err(violation(
                    "IntegrityHalted resolution requires contradictory evidence",
                ));
            }
        }
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn capability(replay_safe: bool) -> CapabilitySnapshot {
        CapabilitySnapshot {
            action_type: "TransferCredits".into(),
            profile_id: "profile-v1".into(),
            profile_revision: 1,
            replay_safety: if replay_safe {
                ReplaySafety::StableKeyIdempotent
            } else {
                ReplaySafety::Unknown
            },
            outcome_observability: if replay_safe {
                OutcomeObservability::Queryable
            } else {
                OutcomeObservability::Unknown
            },
            compensation: CompensationCapability::Unknown,
            provider_batch_atomicity: ProviderBatchAtomicity::NoneObserved,
            safe_retry_without_reconciliation: replay_safe,
            qualification_evidence_id: replay_safe.then(|| "qualified-provider-evidence".into()),
        }
    }

    fn operation(action_count: usize) -> ConstitutionalOperation {
        ConstitutionalOperation {
            schema_version: EFFECT_LEDGER_SCHEMA_VERSION,
            operation_id: "op-1".into(),
            proposal_id: "proposal-1".into(),
            timelock_id: "timelock-1".into(),
            claim_binding: "abstract-claim-binding".into(),
            capability_profile_id: "profile-v1".into(),
            capability_profile_revision: 1,
            ordered_action_commitments: (0..action_count)
                .map(|i| format!("commitment-{i}"))
                .collect(),
            committed_at_unix_ms: 1,
        }
    }

    fn intent(ordinal: u32, replay_safe: bool) -> ActionIntent {
        ActionIntent {
            operation_id: "op-1".into(),
            action_id: format!("action-{ordinal}"),
            ordinal,
            action_commitment: format!("commitment-{ordinal}"),
            capability: capability(replay_safe),
            idempotency_identity: Some(format!("idem-{ordinal}")),
        }
    }

    fn initial_attempt(ordinal: u32, seq: u64) -> ActionAttempt {
        ActionAttempt {
            operation_id: "op-1".into(),
            action_id: format!("action-{ordinal}"),
            attempt_id: format!("attempt-{ordinal}-1"),
            attempt_ordinal: 1,
            event_seq: seq,
            started_at_unix_ms: seq,
            idempotency_identity: Some(format!("idem-{ordinal}")),
            retry_authorization: RetryAuthorization::Initial,
        }
    }

    fn observation(
        ordinal: u32,
        attempt_ordinal: u32,
        id: &str,
        seq: u64,
        outcome: EffectOutcome,
        basis: ObservationBasis,
    ) -> EffectObservation {
        EffectObservation {
            operation_id: "op-1".into(),
            action_id: format!("action-{ordinal}"),
            attempt_id: format!("attempt-{ordinal}-{attempt_ordinal}"),
            observation_id: id.into(),
            event_seq: seq,
            observed_at_unix_ms: seq,
            outcome,
            basis,
            evidence_commitment: format!("evidence-{id}"),
            provider_receipt_commitment: (basis == ObservationBasis::ProviderReceipt)
                .then(|| format!("receipt-{id}")),
            reconciles_observation_id: None,
        }
    }

    fn resolution(kind: OperationResolutionKind) -> OperationResolution {
        OperationResolution {
            operation_id: "op-1".into(),
            resolution_id: "resolution-1".into(),
            kind,
            successful_ordinals: Vec::new(),
            known_no_effect_ordinals: Vec::new(),
            unknown_ordinals: Vec::new(),
            compensation_required_ordinals: Vec::new(),
            evidence_commitment: "resolution-evidence".into(),
            resolved_at_unix_ms: 100,
        }
    }

    #[test]
    fn completed_batch_requires_ordered_observed_success() {
        let mut complete = resolution(OperationResolutionKind::Completed);
        complete.successful_ordinals = vec![0, 1];

        let state = validate_bundle(&EvidenceBundle {
            operation: operation(2),
            intents: vec![intent(0, false), intent(1, false)],
            attempts: vec![initial_attempt(0, 1), initial_attempt(1, 3)],
            observations: vec![
                observation(
                    0,
                    1,
                    "obs-0-success",
                    2,
                    EffectOutcome::KnownSuccess,
                    ObservationBasis::ProviderReceipt,
                ),
                observation(
                    1,
                    1,
                    "obs-1-success",
                    4,
                    EffectOutcome::KnownSuccess,
                    ObservationBasis::ProviderReceipt,
                ),
            ],
            resolution: Some(complete),
        })
        .unwrap();

        assert_eq!(state.successful_ordinals, vec![0, 1]);
        assert!(!state.integrity_halted);
    }

    #[test]
    fn later_action_cannot_start_before_predecessor_success() {
        let error = validate_bundle(&EvidenceBundle {
            operation: operation(2),
            intents: vec![intent(0, false), intent(1, false)],
            attempts: vec![initial_attempt(0, 1), initial_attempt(1, 2)],
            observations: vec![observation(
                0,
                1,
                "obs-0-success",
                3,
                EffectOutcome::KnownSuccess,
                ObservationBasis::ProviderReceipt,
            )],
            resolution: None,
        })
        .unwrap_err();

        assert!(error.to_string().contains("before predecessor"));
    }

    #[test]
    fn non_replay_safe_retry_requires_reconciled_no_effect() {
        let mut retry = ActionAttempt {
            operation_id: "op-1".into(),
            action_id: "action-0".into(),
            attempt_id: "attempt-0-2".into(),
            attempt_ordinal: 2,
            event_seq: 4,
            started_at_unix_ms: 4,
            idempotency_identity: Some("idem-0".into()),
            retry_authorization: RetryAuthorization::ReplaySafe,
        };

        let unknown = observation(
            0,
            1,
            "obs-unknown",
            2,
            EffectOutcome::UnknownOutcome,
            ObservationBasis::DispatchResponse,
        );

        let error = validate_bundle(&EvidenceBundle {
            operation: operation(1),
            intents: vec![intent(0, false)],
            attempts: vec![initial_attempt(0, 1), retry.clone()],
            observations: vec![unknown.clone()],
            resolution: None,
        })
        .unwrap_err();

        assert!(error.to_string().contains("ReplaySafe"));

        let mut no_effect = observation(
            0,
            1,
            "obs-no-effect",
            3,
            EffectOutcome::KnownNoEffect,
            ObservationBasis::Reconciliation,
        );
        no_effect.reconciles_observation_id = Some("obs-unknown".into());
        retry.retry_authorization = RetryAuthorization::ReconciledNoEffect {
            observation_id: "obs-no-effect".into(),
        };

        let state = validate_bundle(&EvidenceBundle {
            operation: operation(1),
            intents: vec![intent(0, false)],
            attempts: vec![initial_attempt(0, 1), retry],
            observations: vec![unknown, no_effect],
            resolution: None,
        })
        .unwrap();

        assert_eq!(state.actions, vec![DerivedActionState::Pending]);
    }

    #[test]
    fn unresolved_unknown_blocks_later_action() {
        let error = validate_bundle(&EvidenceBundle {
            operation: operation(2),
            intents: vec![intent(0, false), intent(1, false)],
            attempts: vec![initial_attempt(0, 1), initial_attempt(1, 3)],
            observations: vec![observation(
                0,
                1,
                "obs-unknown",
                2,
                EffectOutcome::UnknownOutcome,
                ObservationBasis::DispatchResponse,
            )],
            resolution: None,
        })
        .unwrap_err();

        assert!(error.to_string().contains("predecessor"));
    }

    #[test]
    fn successful_prefix_cannot_be_reported_failed_no_effect() {
        let mut bad = resolution(OperationResolutionKind::FailedNoEffect);
        bad.successful_ordinals = vec![0];
        bad.known_no_effect_ordinals = vec![1];

        let error = validate_bundle(&EvidenceBundle {
            operation: operation(2),
            intents: vec![intent(0, false), intent(1, false)],
            attempts: vec![initial_attempt(0, 1), initial_attempt(1, 3)],
            observations: vec![
                observation(
                    0,
                    1,
                    "obs-success",
                    2,
                    EffectOutcome::KnownSuccess,
                    ObservationBasis::ProviderReceipt,
                ),
                observation(
                    1,
                    1,
                    "obs-no-effect",
                    4,
                    EffectOutcome::KnownNoEffect,
                    ObservationBasis::ProviderQuery,
                ),
            ],
            resolution: Some(bad),
        })
        .unwrap_err();

        assert!(error.to_string().contains("FailedNoEffect"));
    }

    #[test]
    fn contradictory_same_attempt_evidence_requires_integrity_halt() {
        let halt = resolution(OperationResolutionKind::IntegrityHalted);

        let state = validate_bundle(&EvidenceBundle {
            operation: operation(1),
            intents: vec![intent(0, false)],
            attempts: vec![initial_attempt(0, 1)],
            observations: vec![
                observation(
                    0,
                    1,
                    "obs-success",
                    2,
                    EffectOutcome::KnownSuccess,
                    ObservationBasis::ProviderReceipt,
                ),
                observation(
                    0,
                    1,
                    "obs-no-effect",
                    3,
                    EffectOutcome::KnownNoEffect,
                    ObservationBasis::ProviderQuery,
                ),
            ],
            resolution: Some(halt),
        })
        .unwrap();

        assert!(state.integrity_halted);
        assert_eq!(state.actions, vec![DerivedActionState::IntegrityHalted]);
    }
}
