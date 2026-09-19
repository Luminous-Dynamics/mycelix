#![deny(unsafe_code)]

use constitutional_payments_provider::{PaymentProviderIntent, ProviderOutcomeEvidence};
use constitutional_payments_provider_journal::{
    JOURNAL_RECORD_COMMITMENT_PREFIX, PROVIDER_JOURNAL_SCHEMA_VERSION, ProviderJournal,
    ProviderJournalEntry, ProviderJournalRecord,
};
use hdi::prelude::*;
use serde::{Deserialize, Serialize};

pub const PROVIDER_HDI_REFINEMENT_SCHEMA_VERSION: u16 = 1;

/// HDI-shaped but deliberately unregistered projection of one F1B0 journal record.
///
/// This type is not present in any `#[hdk_entry_types]` enum in this tranche. The
/// `#[hdk_entry_helper]` derive proves that the frozen journal representation can
/// cross the Holochain serialization boundary without creating a live DHT surface.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct ProviderJournalEntryEnvelope {
    pub refinement_schema_version: u16,
    pub journal_schema_version: u16,
    pub provider_operation_key: String,
    pub sequence: u64,
    pub previous_record_commitment: Option<String>,
    pub journal_record_commitment: String,
    pub body: ProviderJournalBodyV1,
    pub recorded_at_unix_ms: u64,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum ProviderJournalBodyV1 {
    OperationIntent {
        provider_profile_id: String,
        provider_profile_commitment: String,
        execution_id: String,
        request_commitment: String,
        provider_operation_key: String,
        intent_commitment: String,
        committed_at_unix_ms: u64,
    },
    DispatchAttempt {
        attempt_id: String,
        attempt_ordinal: u32,
        execution_id: String,
        request_commitment: String,
    },
    Observation {
        observation_id: String,
        execution_id: String,
        request_commitment: String,
        covers_through_attempt_ordinal: u32,
        outcome: ProviderOutcomeV1,
        observed_at_unix_ms: u64,
    },
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum ProviderOutcomeV1 {
    KnownSuccess {
        payment_id: String,
        provider_receipt_id: String,
        provider_receipt_commitment: String,
    },
    KnownNoEffect {
        no_effect_evidence_id: String,
    },
    UnknownOutcome {
        provider_evidence_id: String,
    },
}

/// Pure description of the future deterministic execution-id index.
///
/// It is intentionally not registered as a Holochain link type in F1B1.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct ProviderOperationIndexProjection {
    pub execution_id: String,
    pub provider_operation_key: String,
    pub genesis_record_commitment: String,
}

fn outcome_from_provider(outcome: &ProviderOutcomeEvidence) -> ProviderOutcomeV1 {
    match outcome {
        ProviderOutcomeEvidence::KnownSuccess {
            payment_id,
            provider_receipt_id,
            provider_receipt_commitment,
        } => ProviderOutcomeV1::KnownSuccess {
            payment_id: payment_id.clone(),
            provider_receipt_id: provider_receipt_id.clone(),
            provider_receipt_commitment: provider_receipt_commitment.clone(),
        },
        ProviderOutcomeEvidence::KnownNoEffect {
            no_effect_evidence_id,
        } => ProviderOutcomeV1::KnownNoEffect {
            no_effect_evidence_id: no_effect_evidence_id.clone(),
        },
        ProviderOutcomeEvidence::UnknownOutcome {
            provider_evidence_id,
        } => ProviderOutcomeV1::UnknownOutcome {
            provider_evidence_id: provider_evidence_id.clone(),
        },
    }
}

fn body_from_record(record: &ProviderJournalRecord) -> ProviderJournalBodyV1 {
    match &record.entry {
        ProviderJournalEntry::OperationIntent { intent } => ProviderJournalBodyV1::OperationIntent {
            provider_profile_id: intent.provider_profile_id.clone(),
            provider_profile_commitment: intent.provider_profile_commitment.clone(),
            execution_id: intent.execution_id.clone(),
            request_commitment: intent.request_commitment.clone(),
            provider_operation_key: intent.provider_operation_key.clone(),
            intent_commitment: intent.intent_commitment.clone(),
            committed_at_unix_ms: intent.committed_at_unix_ms,
        },
        ProviderJournalEntry::DispatchAttempt { attempt } => ProviderJournalBodyV1::DispatchAttempt {
            attempt_id: attempt.attempt_id.clone(),
            attempt_ordinal: attempt.attempt_ordinal,
            execution_id: attempt.execution_id.clone(),
            request_commitment: attempt.request_commitment.clone(),
        },
        ProviderJournalEntry::Observation { observation } => ProviderJournalBodyV1::Observation {
            observation_id: observation.observation.observation_id.clone(),
            execution_id: observation.observation.execution_id.clone(),
            request_commitment: observation.observation.request_commitment.clone(),
            covers_through_attempt_ordinal: observation.covers_through_attempt_ordinal,
            outcome: outcome_from_provider(&observation.observation.outcome),
            observed_at_unix_ms: observation.observation.observed_at_unix_ms,
        },
    }
}

impl ProviderJournalEntryEnvelope {
    /// Exact forward projection from the already-frozen F1B0 journal representation.
    pub fn from_journal_record(record: &ProviderJournalRecord) -> Self {
        Self {
            refinement_schema_version: PROVIDER_HDI_REFINEMENT_SCHEMA_VERSION,
            journal_schema_version: record.schema_version,
            provider_operation_key: record.provider_operation_key.clone(),
            sequence: record.sequence,
            previous_record_commitment: record.previous_record_commitment.clone(),
            journal_record_commitment: record.record_commitment.clone(),
            body: body_from_record(record),
            recorded_at_unix_ms: record.recorded_at_unix_ms,
        }
    }

    /// Validate only representation-level invariants available without activating HDI.
    ///
    /// F1B1 deliberately does not duplicate F1B0's record hash/reducer algorithm.
    pub fn validate_shape(&self) -> Result<(), String> {
        if self.refinement_schema_version != PROVIDER_HDI_REFINEMENT_SCHEMA_VERSION {
            return Err("HDI refinement schema version drift".into());
        }
        if self.journal_schema_version != PROVIDER_JOURNAL_SCHEMA_VERSION {
            return Err("F1B0 journal schema version drift".into());
        }
        if self.provider_operation_key.trim().is_empty() {
            return Err("provider operation key must be non-empty".into());
        }
        let digest = self
            .journal_record_commitment
            .strip_prefix(JOURNAL_RECORD_COMMITMENT_PREFIX)
            .ok_or_else(|| "journal record commitment domain mismatch".to_string())?;
        if digest.len() != 64 || !digest.bytes().all(|b| b.is_ascii_hexdigit() && !b.is_ascii_uppercase()) {
            return Err("journal record commitment must use 64 lowercase hexadecimal digits".into());
        }

        if self.sequence == 0 {
            if self.previous_record_commitment.is_some() {
                return Err("genesis projection must not carry predecessor commitment".into());
            }
            match &self.body {
                ProviderJournalBodyV1::OperationIntent {
                    provider_operation_key,
                    execution_id,
                    request_commitment,
                    ..
                } => {
                    if provider_operation_key != &self.provider_operation_key {
                        return Err("intent body operation key mismatch".into());
                    }
                    if execution_id.trim().is_empty() || request_commitment.trim().is_empty() {
                        return Err("intent identity fields must be non-empty".into());
                    }
                }
                _ => return Err("sequence zero must project OperationIntent".into()),
            }
        } else {
            if self.previous_record_commitment.is_none() {
                return Err("non-genesis projection requires predecessor commitment".into());
            }
            if matches!(&self.body, ProviderJournalBodyV1::OperationIntent { .. }) {
                return Err("OperationIntent may appear only at sequence zero".into());
            }
        }
        Ok(())
    }

    /// Strong refinement check: the HDI-shaped envelope must be exactly the canonical
    /// projection of the supplied F1B0 journal record. There is no alternate encoder.
    pub fn validate_refines_record(&self, record: &ProviderJournalRecord) -> Result<(), String> {
        self.validate_shape()?;
        let canonical = Self::from_journal_record(record);
        if self != &canonical {
            return Err("HDI envelope does not exactly refine F1B0 journal record".into());
        }
        Ok(())
    }
}

impl ProviderOperationIndexProjection {
    pub fn from_journal(journal: &ProviderJournal) -> Result<Self, String> {
        let genesis = journal
            .records()
            .first()
            .ok_or_else(|| "provider journal is missing genesis record".to_string())?;
        Ok(Self {
            execution_id: journal.intent().execution_id.clone(),
            provider_operation_key: journal.intent().provider_operation_key.clone(),
            genesis_record_commitment: genesis.record_commitment.clone(),
        })
    }

    pub fn validate_refines_journal(&self, journal: &ProviderJournal) -> Result<(), String> {
        let canonical = Self::from_journal(journal)?;
        if self != &canonical {
            return Err("provider operation index does not exactly refine F1B0 journal".into());
        }
        Ok(())
    }
}

/// F1B1's authority boundary remains intentionally unresolved.
///
/// The eventual HDI `validate` callback must prove the author/provider authority and
/// predecessor/index relationships from DHT facts. This tranche proves only the exact
/// serialization/refinement shape and must not be treated as write authorization.
pub fn provider_authority_is_qualified() -> bool {
    false
}

#[cfg(test)]
mod tests {
    use super::*;
    use constitutional_payments_provider::{ProviderObservation, ProviderOutcomeEvidence};
    use constitutional_payments_provider_journal::ProviderJournal;
    use constitutional_treasury_effect_provider::{
        EffectIntent, TreasuryEffectRequest, TreasuryEffectSubject,
    };

    fn provider_intent() -> PaymentProviderIntent {
        let request = TreasuryEffectRequest::new(TreasuryEffectSubject {
            operation_id: "op-1".into(),
            action_id: "action-1".into(),
            proposal_id: "proposal-1".into(),
            claim_binding_commitment: "claim-binding-1".into(),
            action_commitment: "action-commitment-1".into(),
            authorization_id: "authorization-1".into(),
            authorization_subject_commitment: "authorization-subject-1".into(),
            treasury_descriptor_commitment: "treasury-1".into(),
            allocation_subject_commitment: "allocation-1".into(),
            approval_projection_commitment: "approval-1".into(),
            capacity_allocation_commitment: Some("capacity-1".into()),
            recipient_did: "did:mycelix:recipient".into(),
            recipient_commitment: "recipient-1".into(),
            value_profile_id: "pending-sap-v1".into(),
            value_authority_commitment: "value-1".into(),
            policy_revision_commitment: "policy-1".into(),
            adapter_profile_id: "payments-provider-v1".into(),
            effect_target_commitment: "target-1".into(),
        })
        .unwrap();
        let effect = EffectIntent::new(request, 1).unwrap();
        PaymentProviderIntent::from_effect(&effect, "provider-profile-commitment-1", 1).unwrap()
    }

    fn journal_with_success() -> ProviderJournal {
        let intent = provider_intent();
        let mut journal = ProviderJournal::new(intent.clone(), 1).unwrap();
        journal.append_dispatch("attempt-1", 2).unwrap();
        journal
            .append_observation(
                ProviderObservation {
                    observation_id: "obs-success".into(),
                    provider_operation_key: intent.provider_operation_key.clone(),
                    execution_id: intent.execution_id.clone(),
                    request_commitment: intent.request_commitment.clone(),
                    outcome: ProviderOutcomeEvidence::KnownSuccess {
                        payment_id: "payment-legacy-1".into(),
                        provider_receipt_id: "receipt-1".into(),
                        provider_receipt_commitment: "receipt-commitment-1".into(),
                    },
                    observed_at_unix_ms: 3,
                },
                1,
                4,
            )
            .unwrap();
        journal
    }

    #[test]
    fn every_journal_record_has_exact_hdi_projection() {
        let journal = journal_with_success();
        for record in journal.records() {
            let entry = ProviderJournalEntryEnvelope::from_journal_record(record);
            entry.validate_refines_record(record).unwrap();
        }
    }

    #[test]
    fn projection_preserves_historical_payment_id_as_payload_not_operation_key() {
        let journal = journal_with_success();
        let observation_record = journal.records().last().unwrap();
        let entry = ProviderJournalEntryEnvelope::from_journal_record(observation_record);
        match entry.body {
            ProviderJournalBodyV1::Observation {
                outcome: ProviderOutcomeV1::KnownSuccess { payment_id, .. },
                ..
            } => {
                assert_eq!(payment_id, "payment-legacy-1");
                assert_ne!(payment_id, entry.provider_operation_key);
            }
            _ => panic!("expected observation projection"),
        }
    }

    #[test]
    fn changed_projection_field_is_rejected() {
        let journal = journal_with_success();
        let record = &journal.records()[1];
        let mut entry = ProviderJournalEntryEnvelope::from_journal_record(record);
        entry.sequence += 1;
        assert!(entry.validate_refines_record(record).is_err());
    }

    #[test]
    fn index_projection_is_exact_and_separate_from_payment_id() {
        let journal = journal_with_success();
        let index = ProviderOperationIndexProjection::from_journal(&journal).unwrap();
        index.validate_refines_journal(&journal).unwrap();
        assert_eq!(index.execution_id, journal.intent().execution_id);
        assert_eq!(index.provider_operation_key, journal.intent().provider_operation_key);
    }

    #[test]
    fn provider_authority_remains_fail_closed() {
        assert!(!provider_authority_is_qualified());
    }
}
