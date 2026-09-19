#![deny(unsafe_code)]

use constitutional_payments_provider::{
    PaymentProviderIntent, ProviderObservation, ProviderOutcomeEvidence,
};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};
use thiserror::Error;

pub const PROVIDER_JOURNAL_SCHEMA_VERSION: u16 = 1;
pub const JOURNAL_RECORD_COMMITMENT_PREFIX: &str = "payments-provider-journal-record-v1:";
const JOURNAL_RECORD_DOMAIN: &[u8] =
    b"MYCELIX-CONSTITUTIONAL-PAYMENTS-PROVIDER-JOURNAL-RECORD\0V1\0";
const MAX_ID_LEN: usize = 256;
const MAX_REF_LEN: usize = 512;

#[derive(Debug, Error, Clone, PartialEq, Eq)]
pub enum ProviderJournalError {
    #[error("{0}")]
    Violation(String),
    #[error("payments provider journal integrity halted: {0}")]
    IntegrityHalted(String),
}

pub type ProviderJournalResult<T> = Result<T, ProviderJournalError>;

fn violation(message: impl Into<String>) -> ProviderJournalError {
    ProviderJournalError::Violation(message.into())
}

fn require_opaque(label: &str, value: &str, max_len: usize) -> ProviderJournalResult<()> {
    if value.trim().is_empty() || value.len() > max_len {
        return Err(violation(format!(
            "{label} must be non-empty and <= {max_len} bytes"
        )));
    }
    Ok(())
}

fn push_str(hasher: &mut blake3::Hasher, value: &str) {
    let bytes = value.as_bytes();
    hasher.update(&(bytes.len() as u64).to_be_bytes());
    hasher.update(bytes);
}

fn push_optional_str(hasher: &mut blake3::Hasher, value: Option<&str>) {
    match value {
        Some(value) => {
            hasher.update(&[1]);
            push_str(hasher, value);
        }
        None => hasher.update(&[0]),
    }
}

fn tagged(hash: blake3::Hash) -> String {
    format!("{JOURNAL_RECORD_COMMITMENT_PREFIX}{}", hash.to_hex())
}

fn require_record_commitment(value: &str) -> ProviderJournalResult<()> {
    let digest = value
        .strip_prefix(JOURNAL_RECORD_COMMITMENT_PREFIX)
        .ok_or_else(|| violation("journal record commitment uses unexpected domain prefix"))?;
    if digest.len() != 64
        || !digest
            .bytes()
            .all(|b| b.is_ascii_digit() || (b'a'..=b'f').contains(&b))
    {
        return Err(violation(
            "journal record commitment must contain 64 lowercase hexadecimal digits",
        ));
    }
    Ok(())
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct DispatchAttempt {
    pub attempt_id: String,
    pub attempt_ordinal: u32,
    pub execution_id: String,
    pub request_commitment: String,
}

impl DispatchAttempt {
    fn validate_against(&self, intent: &PaymentProviderIntent) -> ProviderJournalResult<()> {
        require_opaque("attempt_id", &self.attempt_id, MAX_ID_LEN)?;
        if self.attempt_ordinal == 0 {
            return Err(violation("attempt_ordinal must be >= 1"));
        }
        if self.execution_id != intent.execution_id {
            return Err(violation("dispatch attempt execution_id mismatch"));
        }
        if self.request_commitment != intent.request_commitment {
            return Err(violation("dispatch attempt request_commitment mismatch"));
        }
        Ok(())
    }
}

/// Provider-origin observation plus the dispatch horizon it claims to resolve.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct JournalObservation {
    pub observation: ProviderObservation,
    pub covers_through_attempt_ordinal: u32,
}

impl JournalObservation {
    fn validate_against(
        &self,
        intent: &PaymentProviderIntent,
        attempt_count: u32,
    ) -> ProviderJournalResult<()> {
        self.observation
            .validate_against(intent)
            .map_err(|e| violation(format!("invalid F1A provider observation: {e}")))?;
        if self.covers_through_attempt_ordinal > attempt_count {
            return Err(violation(
                "provider observation claims a dispatch attempt that does not exist",
            ));
        }
        if matches!(
            &self.observation.outcome,
            ProviderOutcomeEvidence::KnownSuccess { .. }
        ) && self.covers_through_attempt_ordinal == 0
        {
            return Err(violation(
                "KnownSuccess cannot exist before any provider dispatch attempt",
            ));
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum ProviderJournalEntry {
    OperationIntent { intent: PaymentProviderIntent },
    DispatchAttempt { attempt: DispatchAttempt },
    Observation { observation: JournalObservation },
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ProviderJournalRecord {
    pub schema_version: u16,
    pub provider_operation_key: String,
    pub sequence: u64,
    pub previous_record_commitment: Option<String>,
    pub entry: ProviderJournalEntry,
    pub record_commitment: String,
    /// Diagnostic/audit time only; excluded from semantic record identity.
    pub recorded_at_unix_ms: u64,
}

impl ProviderJournalRecord {
    fn new(
        provider_operation_key: String,
        sequence: u64,
        previous_record_commitment: Option<String>,
        entry: ProviderJournalEntry,
        recorded_at_unix_ms: u64,
    ) -> ProviderJournalResult<Self> {
        let mut out = Self {
            schema_version: PROVIDER_JOURNAL_SCHEMA_VERSION,
            provider_operation_key,
            sequence,
            previous_record_commitment,
            entry,
            record_commitment: String::new(),
            recorded_at_unix_ms,
        };
        out.record_commitment = out.compute_commitment()?;
        Ok(out)
    }

    fn compute_commitment(&self) -> ProviderJournalResult<String> {
        require_opaque(
            "provider_operation_key",
            &self.provider_operation_key,
            MAX_REF_LEN,
        )?;
        let mut h = blake3::Hasher::new();
        h.update(JOURNAL_RECORD_DOMAIN);
        h.update(&self.schema_version.to_be_bytes());
        push_str(&mut h, &self.provider_operation_key);
        h.update(&self.sequence.to_be_bytes());
        push_optional_str(&mut h, self.previous_record_commitment.as_deref());
        match &self.entry {
            ProviderJournalEntry::OperationIntent { intent } => {
                h.update(&[0]);
                intent
                    .validate()
                    .map_err(|e| violation(format!("invalid provider intent: {e}")))?;
                push_str(&mut h, &intent.intent_commitment);
            }
            ProviderJournalEntry::DispatchAttempt { attempt } => {
                h.update(&[1]);
                push_str(&mut h, &attempt.attempt_id);
                h.update(&attempt.attempt_ordinal.to_be_bytes());
                push_str(&mut h, &attempt.execution_id);
                push_str(&mut h, &attempt.request_commitment);
            }
            ProviderJournalEntry::Observation { observation } => {
                h.update(&[2]);
                push_str(&mut h, &observation.observation.observation_id);
                push_str(&mut h, &observation.observation.provider_operation_key);
                push_str(&mut h, &observation.observation.execution_id);
                push_str(&mut h, &observation.observation.request_commitment);
                h.update(&observation.covers_through_attempt_ordinal.to_be_bytes());
                match &observation.observation.outcome {
                    ProviderOutcomeEvidence::KnownSuccess {
                        payment_id,
                        provider_receipt_id,
                        provider_receipt_commitment,
                    } => {
                        h.update(&[0]);
                        push_str(&mut h, payment_id);
                        push_str(&mut h, provider_receipt_id);
                        push_str(&mut h, provider_receipt_commitment);
                    }
                    ProviderOutcomeEvidence::KnownNoEffect {
                        no_effect_evidence_id,
                    } => {
                        h.update(&[1]);
                        push_str(&mut h, no_effect_evidence_id);
                    }
                    ProviderOutcomeEvidence::UnknownOutcome {
                        provider_evidence_id,
                    } => {
                        h.update(&[2]);
                        push_str(&mut h, provider_evidence_id);
                    }
                }
            }
        }
        Ok(tagged(h.finalize()))
    }

    fn validate_basic(&self) -> ProviderJournalResult<()> {
        if self.schema_version != PROVIDER_JOURNAL_SCHEMA_VERSION {
            return Err(violation("provider journal schema version drift"));
        }
        require_record_commitment(&self.record_commitment)?;
        if self.record_commitment != self.compute_commitment()? {
            return Err(violation("provider journal record commitment mismatch"));
        }
        if self.sequence == 0 {
            if self.previous_record_commitment.is_some() {
                return Err(violation(
                    "journal genesis record must not carry a predecessor commitment",
                ));
            }
            if !matches!(&self.entry, ProviderJournalEntry::OperationIntent { .. }) {
                return Err(violation(
                    "journal sequence zero must be the provider operation intent",
                ));
            }
        } else if self.previous_record_commitment.is_none() {
            return Err(violation(
                "non-genesis journal record must commit to its predecessor",
            ));
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum JournalIntegrityFault {
    ObservationIdCollision {
        observation_id: String,
    },
    SuccessContradictsNoEffect {
        success_observation_id: String,
        no_effect_observation_id: String,
    },
    NoEffectContradictsSuccess {
        no_effect_observation_id: String,
        success_observation_id: String,
    },
    ConflictingSuccessEvidence {
        prior_observation_id: String,
        conflicting_observation_id: String,
    },
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum ProviderQueryResult {
    IntentCommitted,
    InFlight {
        attempt_id: String,
        attempt_ordinal: u32,
    },
    UnknownOutcome {
        through_attempt_ordinal: u32,
        observation_id: String,
        provider_evidence_id: String,
    },
    KnownNoEffect {
        through_attempt_ordinal: u32,
        observation_id: String,
        no_effect_evidence_id: String,
    },
    KnownSuccess {
        through_attempt_ordinal: u32,
        observation_id: String,
        payment_id: String,
        provider_receipt_id: String,
        provider_receipt_commitment: String,
    },
    IntegrityHalted {
        fault: JournalIntegrityFault,
    },
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum AppendDecision {
    Appended { record_commitment: String },
    ExistingSame { record_commitment: String },
    IntegrityHalted {
        record_commitment: String,
        fault: JournalIntegrityFault,
    },
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ProviderJournal {
    intent: PaymentProviderIntent,
    records: Vec<ProviderJournalRecord>,
}

impl ProviderJournal {
    pub fn new(
        intent: PaymentProviderIntent,
        recorded_at_unix_ms: u64,
    ) -> ProviderJournalResult<Self> {
        intent
            .validate()
            .map_err(|e| violation(format!("invalid F1A provider intent: {e}")))?;
        let first = ProviderJournalRecord::new(
            intent.provider_operation_key.clone(),
            0,
            None,
            ProviderJournalEntry::OperationIntent {
                intent: intent.clone(),
            },
            recorded_at_unix_ms,
        )?;
        let out = Self {
            intent,
            records: vec![first],
        };
        out.validate_invariants()?;
        Ok(out)
    }

    pub fn intent(&self) -> &PaymentProviderIntent {
        &self.intent
    }

    pub fn records(&self) -> &[ProviderJournalRecord] {
        &self.records
    }

    pub fn head_commitment(&self) -> &str {
        &self
            .records
            .last()
            .expect("provider journal always has genesis intent")
            .record_commitment
    }

    pub fn attempt_count(&self) -> u32 {
        u32::try_from(
            self.records
                .iter()
                .filter(|record| {
                    matches!(
                        &record.entry,
                        ProviderJournalEntry::DispatchAttempt { .. }
                    )
                })
                .count(),
        )
        .expect("journal attempt count fits u32")
    }

    fn append_entry(
        &mut self,
        entry: ProviderJournalEntry,
        recorded_at_unix_ms: u64,
    ) -> ProviderJournalResult<String> {
        let sequence = u64::try_from(self.records.len())
            .map_err(|_| violation("provider journal sequence overflow"))?;
        let record = ProviderJournalRecord::new(
            self.intent.provider_operation_key.clone(),
            sequence,
            Some(self.head_commitment().to_string()),
            entry,
            recorded_at_unix_ms,
        )?;
        let commitment = record.record_commitment.clone();
        self.records.push(record);
        Ok(commitment)
    }

    /// F1B0 deliberately has no replay-capability consumption path. `UnknownOutcome`
    /// therefore blocks redispatch. Only complete `KnownNoEffect` reconciliation can
    /// authorize the next attempt in this tranche.
    pub fn append_dispatch(
        &mut self,
        attempt_id: impl Into<String>,
        recorded_at_unix_ms: u64,
    ) -> ProviderJournalResult<AppendDecision> {
        let state = self.query()?;
        let current_attempts = self.attempt_count();
        match &state {
            ProviderQueryResult::IntentCommitted if current_attempts == 0 => {}
            ProviderQueryResult::KnownNoEffect {
                through_attempt_ordinal,
                ..
            } if *through_attempt_ordinal == current_attempts => {}
            ProviderQueryResult::UnknownOutcome { .. } => {
                return Err(violation(
                    "UnknownOutcome blocks another dispatch until qualified replay authority is consumed",
                ));
            }
            ProviderQueryResult::KnownSuccess { .. } => {
                return Err(violation("KnownSuccess is terminal for this provider operation"));
            }
            ProviderQueryResult::InFlight { .. } => {
                return Err(violation("a provider dispatch is already in flight"));
            }
            ProviderQueryResult::IntegrityHalted { fault } => {
                return Err(ProviderJournalError::IntegrityHalted(format!("{fault:?}")));
            }
            ProviderQueryResult::IntentCommitted => {
                return Err(violation("provider journal has inconsistent dispatch history"));
            }
            ProviderQueryResult::KnownNoEffect { .. } => {
                return Err(violation(
                    "KnownNoEffect retry requires evidence through the complete dispatch horizon",
                ));
            }
        }

        let attempt_ordinal = current_attempts
            .checked_add(1)
            .ok_or_else(|| violation("dispatch attempt ordinal overflow"))?;
        let attempt = DispatchAttempt {
            attempt_id: attempt_id.into(),
            attempt_ordinal,
            execution_id: self.intent.execution_id.clone(),
            request_commitment: self.intent.request_commitment.clone(),
        };
        attempt.validate_against(&self.intent)?;
        if self.records.iter().any(|record| {
            matches!(
                &record.entry,
                ProviderJournalEntry::DispatchAttempt { attempt: prior }
                    if prior.attempt_id == attempt.attempt_id
            )
        }) {
            return Err(violation("attempt_id has already been used in this provider journal"));
        }
        let commitment = self.append_entry(
            ProviderJournalEntry::DispatchAttempt { attempt },
            recorded_at_unix_ms,
        )?;
        self.validate_invariants()?;
        Ok(AppendDecision::Appended {
            record_commitment: commitment,
        })
    }

    pub fn append_observation(
        &mut self,
        observation: ProviderObservation,
        covers_through_attempt_ordinal: u32,
        recorded_at_unix_ms: u64,
    ) -> ProviderJournalResult<AppendDecision> {
        let candidate = JournalObservation {
            observation,
            covers_through_attempt_ordinal,
        };
        candidate.validate_against(&self.intent, self.attempt_count())?;

        if let Some(existing_record) = self.records.iter().find(|record| {
            matches!(
                &record.entry,
                ProviderJournalEntry::Observation { observation: prior }
                    if prior.observation.observation_id == candidate.observation.observation_id
            )
        }) {
            if let ProviderJournalEntry::Observation { observation: prior } = &existing_record.entry {
                if prior == &candidate {
                    return Ok(AppendDecision::ExistingSame {
                        record_commitment: existing_record.record_commitment.clone(),
                    });
                }
            }
        }

        let commitment = self.append_entry(
            ProviderJournalEntry::Observation {
                observation: candidate,
            },
            recorded_at_unix_ms,
        )?;
        match self.query()? {
            ProviderQueryResult::IntegrityHalted { fault } => Ok(AppendDecision::IntegrityHalted {
                record_commitment: commitment,
                fault,
            }),
            _ => {
                self.validate_invariants()?;
                Ok(AppendDecision::Appended {
                    record_commitment: commitment,
                })
            }
        }
    }

    pub fn query(&self) -> ProviderJournalResult<ProviderQueryResult> {
        self.validate_chain_only()?;
        let mut state = ProviderQueryResult::IntentCommitted;
        let mut halt: Option<JournalIntegrityFault> = None;
        let mut attempt_count = 0_u32;
        let mut attempt_ids = BTreeSet::new();
        let mut observation_ids: BTreeMap<String, JournalObservation> = BTreeMap::new();
        let mut prior_successes: Vec<JournalObservation> = Vec::new();
        let mut prior_no_effects: Vec<JournalObservation> = Vec::new();

        for record in self.records.iter().skip(1) {
            match &record.entry {
                ProviderJournalEntry::OperationIntent { .. } => {
                    return Err(violation("provider journal contains multiple intent records"));
                }
                ProviderJournalEntry::DispatchAttempt { attempt } => {
                    attempt.validate_against(&self.intent)?;
                    if halt.is_some() {
                        return Err(violation("dispatch cannot occur after journal integrity halt"));
                    }
                    let expected = attempt_count
                        .checked_add(1)
                        .ok_or_else(|| violation("dispatch attempt ordinal overflow"))?;
                    if attempt.attempt_ordinal != expected {
                        return Err(violation("dispatch attempt ordinals must be contiguous"));
                    }
                    if !attempt_ids.insert(attempt.attempt_id.clone()) {
                        return Err(violation("dispatch attempt IDs must be unique"));
                    }
                    match &state {
                        ProviderQueryResult::IntentCommitted if attempt_count == 0 => {}
                        ProviderQueryResult::KnownNoEffect {
                            through_attempt_ordinal,
                            ..
                        } if *through_attempt_ordinal == attempt_count => {}
                        _ => {
                            return Err(violation(
                                "dispatch appears without an allowed predecessor state",
                            ));
                        }
                    }
                    attempt_count = expected;
                    state = ProviderQueryResult::InFlight {
                        attempt_id: attempt.attempt_id.clone(),
                        attempt_ordinal: attempt.attempt_ordinal,
                    };
                }
                ProviderJournalEntry::Observation { observation } => {
                    observation.validate_against(&self.intent, attempt_count)?;
                    let obs_id = observation.observation.observation_id.clone();
                    if let Some(prior) = observation_ids.get(&obs_id) {
                        if prior != observation && halt.is_none() {
                            halt = Some(JournalIntegrityFault::ObservationIdCollision {
                                observation_id: obs_id,
                            });
                        }
                        continue;
                    }
                    observation_ids.insert(obs_id.clone(), observation.clone());

                    if halt.is_some() {
                        continue;
                    }

                    match &observation.observation.outcome {
                        ProviderOutcomeEvidence::KnownSuccess {
                            payment_id,
                            provider_receipt_id,
                            provider_receipt_commitment,
                        } => {
                            if let Some(prior) = prior_no_effects.iter().find(|prior| {
                                prior.covers_through_attempt_ordinal
                                    >= observation.covers_through_attempt_ordinal
                            }) {
                                halt = Some(JournalIntegrityFault::SuccessContradictsNoEffect {
                                    success_observation_id: obs_id,
                                    no_effect_observation_id: prior
                                        .observation
                                        .observation_id
                                        .clone(),
                                });
                                continue;
                            }
                            if let Some(prior) = prior_successes.iter().find(|prior| {
                                prior.observation.outcome != observation.observation.outcome
                            }) {
                                halt = Some(JournalIntegrityFault::ConflictingSuccessEvidence {
                                    prior_observation_id: prior
                                        .observation
                                        .observation_id
                                        .clone(),
                                    conflicting_observation_id: obs_id,
                                });
                                continue;
                            }
                            let stale = matches!(
                                &state,
                                ProviderQueryResult::KnownSuccess {
                                    through_attempt_ordinal,
                                    ..
                                } if *through_attempt_ordinal
                                    >= observation.covers_through_attempt_ordinal
                            );
                            prior_successes.push(observation.clone());
                            if !stale {
                                state = ProviderQueryResult::KnownSuccess {
                                    through_attempt_ordinal: observation
                                        .covers_through_attempt_ordinal,
                                    observation_id: observation
                                        .observation
                                        .observation_id
                                        .clone(),
                                    payment_id: payment_id.clone(),
                                    provider_receipt_id: provider_receipt_id.clone(),
                                    provider_receipt_commitment: provider_receipt_commitment.clone(),
                                };
                            }
                        }
                        ProviderOutcomeEvidence::KnownNoEffect {
                            no_effect_evidence_id,
                        } => {
                            if let Some(prior) = prior_successes.iter().find(|prior| {
                                prior.covers_through_attempt_ordinal
                                    <= observation.covers_through_attempt_ordinal
                            }) {
                                halt = Some(JournalIntegrityFault::NoEffectContradictsSuccess {
                                    no_effect_observation_id: obs_id,
                                    success_observation_id: prior
                                        .observation
                                        .observation_id
                                        .clone(),
                                });
                                continue;
                            }
                            prior_no_effects.push(observation.clone());
                            let stale = match &state {
                                ProviderQueryResult::KnownSuccess { .. } => true,
                                ProviderQueryResult::InFlight {
                                    attempt_ordinal, ..
                                } => *attempt_ordinal
                                    > observation.covers_through_attempt_ordinal,
                                ProviderQueryResult::KnownNoEffect {
                                    through_attempt_ordinal,
                                    ..
                                }
                                | ProviderQueryResult::UnknownOutcome {
                                    through_attempt_ordinal,
                                    ..
                                } => *through_attempt_ordinal
                                    >= observation.covers_through_attempt_ordinal,
                                _ => false,
                            };
                            if !stale {
                                state = ProviderQueryResult::KnownNoEffect {
                                    through_attempt_ordinal: observation
                                        .covers_through_attempt_ordinal,
                                    observation_id: observation
                                        .observation
                                        .observation_id
                                        .clone(),
                                    no_effect_evidence_id: no_effect_evidence_id.clone(),
                                };
                            }
                        }
                        ProviderOutcomeEvidence::UnknownOutcome {
                            provider_evidence_id,
                        } => {
                            let stale = match &state {
                                ProviderQueryResult::KnownSuccess { .. } => true,
                                ProviderQueryResult::InFlight {
                                    attempt_ordinal, ..
                                } => *attempt_ordinal
                                    > observation.covers_through_attempt_ordinal,
                                ProviderQueryResult::KnownNoEffect {
                                    through_attempt_ordinal,
                                    ..
                                }
                                | ProviderQueryResult::UnknownOutcome {
                                    through_attempt_ordinal,
                                    ..
                                } => *through_attempt_ordinal
                                    >= observation.covers_through_attempt_ordinal,
                                _ => false,
                            };
                            if !stale {
                                state = ProviderQueryResult::UnknownOutcome {
                                    through_attempt_ordinal: observation
                                        .covers_through_attempt_ordinal,
                                    observation_id: observation
                                        .observation
                                        .observation_id
                                        .clone(),
                                    provider_evidence_id: provider_evidence_id.clone(),
                                };
                            }
                        }
                    }
                }
            }
        }

        if let Some(fault) = halt {
            Ok(ProviderQueryResult::IntegrityHalted { fault })
        } else {
            Ok(state)
        }
    }

    fn validate_chain_only(&self) -> ProviderJournalResult<()> {
        self.intent
            .validate()
            .map_err(|e| violation(format!("invalid F1A provider intent: {e}")))?;
        if self.records.is_empty() {
            return Err(violation("provider journal must contain its intent genesis record"));
        }
        for (index, record) in self.records.iter().enumerate() {
            record.validate_basic()?;
            if record.provider_operation_key != self.intent.provider_operation_key {
                return Err(violation("journal record operation-key mismatch"));
            }
            let expected_sequence = u64::try_from(index)
                .map_err(|_| violation("provider journal sequence overflow"))?;
            if record.sequence != expected_sequence {
                return Err(violation("provider journal sequences must be contiguous"));
            }
            if index == 0 {
                match &record.entry {
                    ProviderJournalEntry::OperationIntent { intent } if intent == &self.intent => {}
                    ProviderJournalEntry::OperationIntent { .. } => {
                        return Err(violation("journal genesis intent differs from journal intent"));
                    }
                    _ => unreachable!("validate_basic already requires intent at sequence zero"),
                }
            } else {
                let predecessor = &self.records[index - 1].record_commitment;
                if record.previous_record_commitment.as_deref() != Some(predecessor.as_str()) {
                    return Err(violation("provider journal predecessor commitment mismatch"));
                }
            }
        }
        Ok(())
    }

    pub fn validate_invariants(&self) -> ProviderJournalResult<()> {
        self.validate_chain_only()?;
        let _ = self.query()?;
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum RegistryDecision {
    Created { provider_operation_key: String },
    ExistingSame { provider_operation_key: String },
    IntegrityConflict {
        execution_id: String,
        existing_operation_key: String,
        candidate_operation_key: String,
    },
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum ProviderQuery {
    NotFound,
    Found {
        provider_operation_key: String,
        journal_head_commitment: String,
        result: ProviderQueryResult,
    },
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ProviderJournalRegistry {
    journals: BTreeMap<String, ProviderJournal>,
    execution_index: BTreeMap<String, String>,
    integrity_fault: Option<String>,
}

impl Default for ProviderJournalRegistry {
    fn default() -> Self {
        Self::new()
    }
}

impl ProviderJournalRegistry {
    pub fn new() -> Self {
        Self {
            journals: BTreeMap::new(),
            execution_index: BTreeMap::new(),
            integrity_fault: None,
        }
    }

    pub fn submit(
        &mut self,
        intent: PaymentProviderIntent,
        recorded_at_unix_ms: u64,
    ) -> ProviderJournalResult<RegistryDecision> {
        if let Some(fault) = &self.integrity_fault {
            return Err(ProviderJournalError::IntegrityHalted(fault.clone()));
        }
        intent
            .validate()
            .map_err(|e| violation(format!("invalid F1A provider intent: {e}")))?;
        let key = intent.provider_operation_key.clone();
        if let Some(existing) = self.journals.get(&key) {
            if existing.intent() == &intent {
                return Ok(RegistryDecision::ExistingSame {
                    provider_operation_key: key,
                });
            }
            self.integrity_fault = Some(format!("provider operation key collision: {key}"));
            return Ok(RegistryDecision::IntegrityConflict {
                execution_id: intent.execution_id,
                existing_operation_key: key.clone(),
                candidate_operation_key: key,
            });
        }
        if let Some(existing_key) = self.execution_index.get(&intent.execution_id) {
            if existing_key != &key {
                let existing_operation_key = existing_key.clone();
                self.integrity_fault = Some(format!(
                    "execution identity reused across provider operations: {}",
                    intent.execution_id
                ));
                return Ok(RegistryDecision::IntegrityConflict {
                    execution_id: intent.execution_id,
                    existing_operation_key,
                    candidate_operation_key: key,
                });
            }
        }
        let journal = ProviderJournal::new(intent.clone(), recorded_at_unix_ms)?;
        self.execution_index
            .insert(intent.execution_id.clone(), key.clone());
        self.journals.insert(key.clone(), journal);
        self.validate_invariants()?;
        Ok(RegistryDecision::Created {
            provider_operation_key: key,
        })
    }

    pub fn journal(&self, provider_operation_key: &str) -> Option<&ProviderJournal> {
        self.journals.get(provider_operation_key)
    }

    pub fn append_dispatch(
        &mut self,
        provider_operation_key: &str,
        attempt_id: impl Into<String>,
        recorded_at_unix_ms: u64,
    ) -> ProviderJournalResult<AppendDecision> {
        self.journals
            .get_mut(provider_operation_key)
            .ok_or_else(|| violation("provider operation key not found"))?
            .append_dispatch(attempt_id, recorded_at_unix_ms)
    }

    pub fn append_observation(
        &mut self,
        provider_operation_key: &str,
        observation: ProviderObservation,
        covers_through_attempt_ordinal: u32,
        recorded_at_unix_ms: u64,
    ) -> ProviderJournalResult<AppendDecision> {
        self.journals
            .get_mut(provider_operation_key)
            .ok_or_else(|| violation("provider operation key not found"))?
            .append_observation(
                observation,
                covers_through_attempt_ordinal,
                recorded_at_unix_ms,
            )
    }

    pub fn query(&self, provider_operation_key: &str) -> ProviderJournalResult<ProviderQuery> {
        let Some(journal) = self.journals.get(provider_operation_key) else {
            return Ok(ProviderQuery::NotFound);
        };
        Ok(ProviderQuery::Found {
            provider_operation_key: provider_operation_key.to_string(),
            journal_head_commitment: journal.head_commitment().to_string(),
            result: journal.query()?,
        })
    }

    pub fn query_by_execution(&self, execution_id: &str) -> ProviderJournalResult<ProviderQuery> {
        let Some(key) = self.execution_index.get(execution_id) else {
            return Ok(ProviderQuery::NotFound);
        };
        self.query(key)
    }

    pub fn validate_invariants(&self) -> ProviderJournalResult<()> {
        if self.journals.len() != self.execution_index.len() {
            return Err(violation(
                "every provider journal must have exactly one execution index entry",
            ));
        }
        for (key, journal) in &self.journals {
            journal.validate_invariants()?;
            if key != &journal.intent().provider_operation_key {
                return Err(violation("provider journal registry key mismatch"));
            }
            if self.execution_index.get(&journal.intent().execution_id) != Some(key) {
                return Err(violation("execution index does not bind exact provider journal"));
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use constitutional_treasury_effect_provider::{
        EffectIntent, TreasuryEffectRequest, TreasuryEffectSubject,
    };

    fn effect(action: &str, time: u64) -> EffectIntent {
        let request = TreasuryEffectRequest::new(TreasuryEffectSubject {
            operation_id: "op-1".into(),
            action_id: action.into(),
            proposal_id: "proposal-1".into(),
            claim_binding_commitment: "claim-binding-1".into(),
            action_commitment: format!("action-commitment-{action}"),
            authorization_id: format!("authorization-{action}"),
            authorization_subject_commitment: "authorization-subject-1".into(),
            treasury_descriptor_commitment: "treasury-1".into(),
            allocation_subject_commitment: format!("allocation-{action}"),
            approval_projection_commitment: "approval-1".into(),
            capacity_allocation_commitment: Some(format!("capacity-{action}")),
            recipient_did: "did:mycelix:recipient".into(),
            recipient_commitment: "recipient-1".into(),
            value_profile_id: "pending-sap-v1".into(),
            value_authority_commitment: "value-1".into(),
            policy_revision_commitment: "policy-1".into(),
            adapter_profile_id: "payments-provider-v1".into(),
            effect_target_commitment: "target-1".into(),
        })
        .unwrap();
        EffectIntent::new(request, time).unwrap()
    }

    fn intent(action: &str, time: u64) -> PaymentProviderIntent {
        PaymentProviderIntent::from_effect(
            &effect(action, time),
            "provider-profile-commitment-1",
            time,
        )
        .unwrap()
    }

    fn observation(
        intent: &PaymentProviderIntent,
        id: &str,
        outcome: ProviderOutcomeEvidence,
    ) -> ProviderObservation {
        ProviderObservation {
            observation_id: id.into(),
            provider_operation_key: intent.provider_operation_key.clone(),
            execution_id: intent.execution_id.clone(),
            request_commitment: intent.request_commitment.clone(),
            outcome,
            observed_at_unix_ms: 99,
        }
    }

    fn no_effect(intent: &PaymentProviderIntent, id: &str) -> ProviderObservation {
        observation(
            intent,
            id,
            ProviderOutcomeEvidence::KnownNoEffect {
                no_effect_evidence_id: format!("none-{id}"),
            },
        )
    }

    fn unknown(intent: &PaymentProviderIntent, id: &str) -> ProviderObservation {
        observation(
            intent,
            id,
            ProviderOutcomeEvidence::UnknownOutcome {
                provider_evidence_id: format!("unknown-{id}"),
            },
        )
    }

    fn success(
        intent: &PaymentProviderIntent,
        id: &str,
        payment: &str,
    ) -> ProviderObservation {
        observation(
            intent,
            id,
            ProviderOutcomeEvidence::KnownSuccess {
                payment_id: payment.into(),
                provider_receipt_id: format!("receipt-{payment}"),
                provider_receipt_commitment: format!("receipt-commitment-{payment}"),
            },
        )
    }

    #[test]
    fn journal_record_identity_excludes_wall_clock() {
        let i = intent("action-1", 1);
        let a = ProviderJournal::new(i.clone(), 10).unwrap();
        let b = ProviderJournal::new(i, 999_999).unwrap();
        assert_eq!(a.head_commitment(), b.head_commitment());
    }

    #[test]
    fn lost_ack_success_remains_authoritatively_queryable() {
        let i = intent("action-1", 1);
        let key = i.provider_operation_key.clone();
        let mut registry = ProviderJournalRegistry::new();
        registry.submit(i.clone(), 1).unwrap();
        registry.append_dispatch(&key, "attempt-1", 2).unwrap();
        registry
            .append_observation(&key, success(&i, "obs-success", "payment-1"), 1, 3)
            .unwrap();
        assert!(matches!(
            registry.query(&key).unwrap(),
            ProviderQuery::Found {
                result: ProviderQueryResult::KnownSuccess { .. },
                ..
            }
        ));
    }

    #[test]
    fn unknown_outcome_blocks_blind_redispatch() {
        let i = intent("action-1", 1);
        let mut journal = ProviderJournal::new(i.clone(), 1).unwrap();
        journal.append_dispatch("attempt-1", 2).unwrap();
        journal.append_observation(unknown(&i, "obs-unknown"), 1, 3).unwrap();
        assert!(journal.append_dispatch("attempt-2", 4).is_err());
    }

    #[test]
    fn reconciled_no_effect_permits_next_attempt_same_operation() {
        let i = intent("action-1", 1);
        let mut journal = ProviderJournal::new(i.clone(), 1).unwrap();
        journal.append_dispatch("attempt-1", 2).unwrap();
        journal.append_observation(no_effect(&i, "obs-none-1"), 1, 3).unwrap();
        journal.append_dispatch("attempt-2", 4).unwrap();
        assert_eq!(journal.attempt_count(), 2);
        assert!(matches!(
            journal.query().unwrap(),
            ProviderQueryResult::InFlight {
                attempt_ordinal: 2,
                ..
            }
        ));
    }

    #[test]
    fn stale_no_effect_does_not_demote_newer_in_flight_attempt() {
        let i = intent("action-1", 1);
        let mut journal = ProviderJournal::new(i.clone(), 1).unwrap();
        journal.append_dispatch("attempt-1", 2).unwrap();
        journal.append_observation(no_effect(&i, "obs-none-1"), 1, 3).unwrap();
        journal.append_dispatch("attempt-2", 4).unwrap();
        journal.append_observation(no_effect(&i, "obs-none-stale"), 1, 5).unwrap();
        assert!(matches!(
            journal.query().unwrap(),
            ProviderQueryResult::InFlight {
                attempt_ordinal: 2,
                ..
            }
        ));
    }

    #[test]
    fn stale_unknown_does_not_demote_newer_in_flight_attempt() {
        let i = intent("action-1", 1);
        let mut journal = ProviderJournal::new(i.clone(), 1).unwrap();
        journal.append_dispatch("attempt-1", 2).unwrap();
        journal.append_observation(no_effect(&i, "obs-none-1"), 1, 3).unwrap();
        journal.append_dispatch("attempt-2", 4).unwrap();
        journal.append_observation(unknown(&i, "obs-unknown-stale"), 1, 5).unwrap();
        assert!(matches!(
            journal.query().unwrap(),
            ProviderQueryResult::InFlight {
                attempt_ordinal: 2,
                ..
            }
        ));
    }

    #[test]
    fn late_success_for_reconciled_attempt_halts() {
        let i = intent("action-1", 1);
        let mut journal = ProviderJournal::new(i.clone(), 1).unwrap();
        journal.append_dispatch("attempt-1", 2).unwrap();
        journal.append_observation(no_effect(&i, "obs-none-1"), 1, 3).unwrap();
        journal.append_dispatch("attempt-2", 4).unwrap();
        let result = journal
            .append_observation(success(&i, "obs-late-success", "payment-1"), 1, 5)
            .unwrap();
        assert!(matches!(result, AppendDecision::IntegrityHalted { .. }));
    }

    #[test]
    fn later_attempt_success_is_compatible_with_earlier_no_effect() {
        let i = intent("action-1", 1);
        let mut journal = ProviderJournal::new(i.clone(), 1).unwrap();
        journal.append_dispatch("attempt-1", 2).unwrap();
        journal.append_observation(no_effect(&i, "obs-none-1"), 1, 3).unwrap();
        journal.append_dispatch("attempt-2", 4).unwrap();
        journal
            .append_observation(success(&i, "obs-success-2", "payment-2"), 2, 5)
            .unwrap();
        assert!(matches!(
            journal.query().unwrap(),
            ProviderQueryResult::KnownSuccess {
                through_attempt_ordinal: 2,
                ..
            }
        ));
    }

    #[test]
    fn conflicting_success_receipts_halt() {
        let i = intent("action-1", 1);
        let mut journal = ProviderJournal::new(i.clone(), 1).unwrap();
        journal.append_dispatch("attempt-1", 2).unwrap();
        journal
            .append_observation(success(&i, "obs-success-1", "payment-1"), 1, 3)
            .unwrap();
        let result = journal
            .append_observation(success(&i, "obs-success-2", "payment-2"), 1, 4)
            .unwrap();
        assert!(matches!(result, AppendDecision::IntegrityHalted { .. }));
    }

    #[test]
    fn observation_id_collision_halts_and_preserves_both_facts() {
        let i = intent("action-1", 1);
        let mut journal = ProviderJournal::new(i.clone(), 1).unwrap();
        journal.append_dispatch("attempt-1", 2).unwrap();
        journal.append_observation(unknown(&i, "obs-1"), 1, 3).unwrap();
        let result = journal
            .append_observation(no_effect(&i, "obs-1"), 1, 4)
            .unwrap();
        assert!(matches!(result, AppendDecision::IntegrityHalted { .. }));
        assert_eq!(journal.records().len(), 4);
    }

    #[test]
    fn later_unknown_cannot_demote_known_success() {
        let i = intent("action-1", 1);
        let mut journal = ProviderJournal::new(i.clone(), 1).unwrap();
        journal.append_dispatch("attempt-1", 2).unwrap();
        journal
            .append_observation(success(&i, "obs-success", "payment-1"), 1, 3)
            .unwrap();
        journal.append_observation(unknown(&i, "obs-unknown-late"), 1, 4).unwrap();
        assert!(matches!(
            journal.query().unwrap(),
            ProviderQueryResult::KnownSuccess { .. }
        ));
    }

    #[test]
    fn integrity_halt_is_sticky_under_later_forensic_observation() {
        let i = intent("action-1", 1);
        let mut journal = ProviderJournal::new(i.clone(), 1).unwrap();
        journal.append_dispatch("attempt-1", 2).unwrap();
        journal.append_observation(no_effect(&i, "obs-none"), 1, 3).unwrap();
        journal
            .append_observation(success(&i, "obs-success", "payment-1"), 1, 4)
            .unwrap();
        journal.append_observation(unknown(&i, "obs-forensic"), 1, 5).unwrap();
        assert!(matches!(
            journal.query().unwrap(),
            ProviderQueryResult::IntegrityHalted { .. }
        ));
        assert!(journal.append_dispatch("attempt-2", 6).is_err());
    }

    #[test]
    fn journal_chain_detects_tampering() {
        let i = intent("action-1", 1);
        let mut journal = ProviderJournal::new(i, 1).unwrap();
        journal.append_dispatch("attempt-1", 2).unwrap();
        journal.records[1].previous_record_commitment = Some("tampered".into());
        assert!(journal.validate_invariants().is_err());
    }

    #[test]
    fn registry_is_idempotent_for_exact_same_provider_intent() {
        let i = intent("action-1", 1);
        let mut registry = ProviderJournalRegistry::new();
        assert!(matches!(
            registry.submit(i.clone(), 1).unwrap(),
            RegistryDecision::Created { .. }
        ));
        assert!(matches!(
            registry.submit(i, 999).unwrap(),
            RegistryDecision::ExistingSame { .. }
        ));
    }
}
