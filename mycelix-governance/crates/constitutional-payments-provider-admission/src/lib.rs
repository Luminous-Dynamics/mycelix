#![deny(unsafe_code)]

use constitutional_payments_provider_hdi::{
    ProviderJournalEntryEnvelope, ProviderOperationIndexProjection,
};
use constitutional_payments_provider_journal::ProviderJournal;
use std::collections::BTreeMap;
use thiserror::Error;

const MAX_AUTHOR_LEN: usize = 512;
const MAX_EVIDENCE_LEN: usize = 512;

#[derive(Debug, Error, Clone, PartialEq, Eq)]
pub enum AdmissionOracleError {
    #[error("{0}")]
    Violation(String),
}

pub type AdmissionOracleResult<T> = Result<T, AdmissionOracleError>;

fn violation(message: impl Into<String>) -> AdmissionOracleError {
    AdmissionOracleError::Violation(message.into())
}

fn require_opaque(label: &str, value: &str, max_len: usize) -> AdmissionOracleResult<()> {
    if value.trim().is_empty() || value.len() > max_len {
        return Err(violation(format!(
            "{label} must be non-empty and <= {max_len} bytes"
        )));
    }
    Ok(())
}

/// Reference-model-only provider-authority input.
///
/// This type is intentionally not serializable and is not a production capability.
/// `QualifiedForModelOnly` exists solely to exercise the future-authorized path in the
/// pre-registration threat model. F1B2 does not provide a live qualification/mint path.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ReferenceProviderAuthority {
    state: ReferenceProviderAuthorityState,
}

#[derive(Debug, Clone, PartialEq, Eq)]
enum ReferenceProviderAuthorityState {
    Pending,
    QualifiedForModelOnly {
        evidence_id: String,
        provider_profile_commitment: String,
    },
}

impl ReferenceProviderAuthority {
    pub fn pending() -> Self {
        Self {
            state: ReferenceProviderAuthorityState::Pending,
        }
    }

    /// Construct qualified authority for the reference model only.
    ///
    /// This does not establish real provider authority and must never be converted into
    /// a serialized credential or Holochain write capability.
    pub fn qualified_for_model_only(
        evidence_id: impl Into<String>,
        provider_profile_commitment: impl Into<String>,
    ) -> AdmissionOracleResult<Self> {
        let evidence_id = evidence_id.into();
        let provider_profile_commitment = provider_profile_commitment.into();
        require_opaque("authority evidence id", &evidence_id, MAX_EVIDENCE_LEN)?;
        require_opaque(
            "authority provider profile commitment",
            &provider_profile_commitment,
            MAX_EVIDENCE_LEN,
        )?;
        Ok(Self {
            state: ReferenceProviderAuthorityState::QualifiedForModelOnly {
                evidence_id,
                provider_profile_commitment,
            },
        })
    }

    pub fn reference_evidence_id(&self) -> Option<&str> {
        match &self.state {
            ReferenceProviderAuthorityState::Pending => None,
            ReferenceProviderAuthorityState::QualifiedForModelOnly { evidence_id, .. } => {
                Some(evidence_id)
            }
        }
    }

    fn verify_for(&self, expected_profile_commitment: &str) -> Result<(), AdmissionRejection> {
        match &self.state {
            ReferenceProviderAuthorityState::Pending => {
                Err(AdmissionRejection::ProviderAuthorityPending)
            }
            ReferenceProviderAuthorityState::QualifiedForModelOnly {
                provider_profile_commitment,
                ..
            } if provider_profile_commitment == expected_profile_commitment => Ok(()),
            ReferenceProviderAuthorityState::QualifiedForModelOnly { .. } => {
                Err(AdmissionRejection::AuthorityProfileMismatch)
            }
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum AdmissionRejection {
    ProviderAuthorityPending,
    AuthorMismatch,
    AuthorityProfileMismatch,
    ReferenceJournalInvalid,
    ProjectionInvalid,
    RecordNotInReferenceJournal,
    MissingPredecessor,
    PredecessorMismatch,
    IndexRequiresAdmittedGenesis,
    IndexProjectionMismatch,
    MutationForbidden,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum AdmissionFault {
    SameSequenceConflict { sequence: u64 },
    ConflictingOperationIndex { execution_id: String },
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum AdmissionDecision {
    Accepted,
    ExistingSame,
    Rejected(AdmissionRejection),
    IntegrityHalted(AdmissionFault),
}

/// Stateful pre-registration oracle for the future DHT admission policy.
///
/// The oracle's canonical source is an already-validated F1B0 journal. Candidate DHT
/// facts are admitted only when they exactly refine that source and satisfy the expected
/// author/predecessor/index rules. It does not register or write any Holochain entries.
#[derive(Clone)]
pub struct ProviderDhtAdmissionOracle {
    reference_journal: ProviderJournal,
    expected_author: String,
    authority: ReferenceProviderAuthority,
    admitted_records: BTreeMap<u64, ProviderJournalEntryEnvelope>,
    admitted_index: Option<ProviderOperationIndexProjection>,
    integrity_fault: Option<AdmissionFault>,
}

impl ProviderDhtAdmissionOracle {
    pub fn new(
        reference_journal: ProviderJournal,
        expected_author: impl Into<String>,
        authority: ReferenceProviderAuthority,
    ) -> AdmissionOracleResult<Self> {
        reference_journal
            .validate_invariants()
            .map_err(|e| violation(format!("invalid F1B0 reference journal: {e}")))?;
        let expected_author = expected_author.into();
        require_opaque("expected provider author", &expected_author, MAX_AUTHOR_LEN)?;
        Ok(Self {
            reference_journal,
            expected_author,
            authority,
            admitted_records: BTreeMap::new(),
            admitted_index: None,
            integrity_fault: None,
        })
    }

    pub fn reference_journal(&self) -> &ProviderJournal {
        &self.reference_journal
    }

    pub fn admitted_record(&self, sequence: u64) -> Option<&ProviderJournalEntryEnvelope> {
        self.admitted_records.get(&sequence)
    }

    pub fn admitted_index(&self) -> Option<&ProviderOperationIndexProjection> {
        self.admitted_index.as_ref()
    }

    pub fn integrity_fault(&self) -> Option<&AdmissionFault> {
        self.integrity_fault.as_ref()
    }

    fn halted(&self) -> Option<AdmissionDecision> {
        self.integrity_fault
            .clone()
            .map(AdmissionDecision::IntegrityHalted)
    }

    fn halt(&mut self, fault: AdmissionFault) -> AdmissionDecision {
        if self.integrity_fault.is_none() {
            self.integrity_fault = Some(fault.clone());
        }
        AdmissionDecision::IntegrityHalted(
            self.integrity_fault
                .clone()
                .expect("integrity fault was just installed"),
        )
    }

    fn authorize(&self, author: &str) -> Result<(), AdmissionRejection> {
        if author != self.expected_author {
            return Err(AdmissionRejection::AuthorMismatch);
        }
        self.authority
            .verify_for(&self.reference_journal.intent().provider_profile_commitment)
    }

    pub fn admit_record_create(
        &mut self,
        author: &str,
        candidate: ProviderJournalEntryEnvelope,
    ) -> AdmissionDecision {
        if let Some(halted) = self.halted() {
            return halted;
        }
        if let Err(reason) = self.authorize(author) {
            return AdmissionDecision::Rejected(reason);
        }
        if candidate.validate_shape().is_err() {
            return AdmissionDecision::Rejected(AdmissionRejection::ProjectionInvalid);
        }

        if let Some(existing) = self.admitted_records.get(&candidate.sequence) {
            if existing == &candidate {
                return AdmissionDecision::ExistingSame;
            }
            return self.halt(AdmissionFault::SameSequenceConflict {
                sequence: candidate.sequence,
            });
        }

        let Ok(index) = usize::try_from(candidate.sequence) else {
            return AdmissionDecision::Rejected(AdmissionRejection::RecordNotInReferenceJournal);
        };
        let Some(reference_record) = self.reference_journal.records().get(index) else {
            return AdmissionDecision::Rejected(AdmissionRejection::RecordNotInReferenceJournal);
        };

        if candidate.sequence > 0 {
            let Some(predecessor) = self.admitted_records.get(&(candidate.sequence - 1)) else {
                return AdmissionDecision::Rejected(AdmissionRejection::MissingPredecessor);
            };
            if candidate.previous_record_commitment.as_deref()
                != Some(predecessor.journal_record_commitment.as_str())
            {
                return AdmissionDecision::Rejected(AdmissionRejection::PredecessorMismatch);
            }
        }

        if candidate.validate_refines_record(reference_record).is_err() {
            return AdmissionDecision::Rejected(AdmissionRejection::ProjectionInvalid);
        }

        self.admitted_records.insert(candidate.sequence, candidate);
        AdmissionDecision::Accepted
    }

    pub fn admit_index_create(
        &mut self,
        author: &str,
        candidate: ProviderOperationIndexProjection,
    ) -> AdmissionDecision {
        if let Some(halted) = self.halted() {
            return halted;
        }
        if let Err(reason) = self.authorize(author) {
            return AdmissionDecision::Rejected(reason);
        }
        if !self.admitted_records.contains_key(&0) {
            return AdmissionDecision::Rejected(AdmissionRejection::IndexRequiresAdmittedGenesis);
        }

        if let Some(existing) = &self.admitted_index {
            if existing == &candidate {
                return AdmissionDecision::ExistingSame;
            }
            return self.halt(AdmissionFault::ConflictingOperationIndex {
                execution_id: candidate.execution_id,
            });
        }

        let Ok(canonical) = ProviderOperationIndexProjection::from_journal(&self.reference_journal)
        else {
            return AdmissionDecision::Rejected(AdmissionRejection::ReferenceJournalInvalid);
        };
        if candidate != canonical {
            return AdmissionDecision::Rejected(AdmissionRejection::IndexProjectionMismatch);
        }
        self.admitted_index = Some(candidate);
        AdmissionDecision::Accepted
    }

    pub fn admit_record_update(&self) -> AdmissionDecision {
        if let Some(halted) = self.halted() {
            return halted;
        }
        AdmissionDecision::Rejected(AdmissionRejection::MutationForbidden)
    }

    pub fn admit_record_delete(&self) -> AdmissionDecision {
        if let Some(halted) = self.halted() {
            return halted;
        }
        AdmissionDecision::Rejected(AdmissionRejection::MutationForbidden)
    }

    pub fn admit_index_delete(&self) -> AdmissionDecision {
        if let Some(halted) = self.halted() {
            return halted;
        }
        AdmissionDecision::Rejected(AdmissionRejection::MutationForbidden)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use constitutional_payments_provider::{
        PaymentProviderIntent, ProviderObservation, ProviderOutcomeEvidence,
    };
    use constitutional_payments_provider_hdi::{ProviderJournalBodyV1, ProviderOutcomeV1};
    use constitutional_payments_provider_journal::ProviderJournal;
    use constitutional_treasury_effect_provider::{
        EffectIntent, TreasuryEffectRequest, TreasuryEffectSubject,
    };

    const AUTHOR: &str = "did:mycelix:payments-provider";

    fn intent() -> PaymentProviderIntent {
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

    fn journal() -> ProviderJournal {
        let intent = intent();
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

    fn qualified_authority() -> ReferenceProviderAuthority {
        ReferenceProviderAuthority::qualified_for_model_only(
            "authority-evidence-1",
            "provider-profile-commitment-1",
        )
        .unwrap()
    }

    fn oracle(authority: ReferenceProviderAuthority) -> ProviderDhtAdmissionOracle {
        ProviderDhtAdmissionOracle::new(journal(), AUTHOR, authority).unwrap()
    }

    fn envelope(journal: &ProviderJournal, sequence: usize) -> ProviderJournalEntryEnvelope {
        ProviderJournalEntryEnvelope::from_journal_record(&journal.records()[sequence])
    }

    fn admit_through(oracle: &mut ProviderDhtAdmissionOracle, sequence: usize) {
        for i in 0..=sequence {
            let candidate = envelope(oracle.reference_journal(), i);
            assert_eq!(
                oracle.admit_record_create(AUTHOR, candidate),
                AdmissionDecision::Accepted
            );
        }
    }

    #[test]
    fn pending_provider_authority_rejects_canonical_create() {
        let mut oracle = oracle(ReferenceProviderAuthority::pending());
        let candidate = envelope(oracle.reference_journal(), 0);
        assert_eq!(
            oracle.admit_record_create(AUTHOR, candidate),
            AdmissionDecision::Rejected(AdmissionRejection::ProviderAuthorityPending)
        );
    }

    #[test]
    fn reference_authority_evidence_is_auditable_but_not_serialized() {
        let authority = qualified_authority();
        assert_eq!(authority.reference_evidence_id(), Some("authority-evidence-1"));
    }

    #[test]
    fn spoofed_author_is_rejected() {
        let mut oracle = oracle(qualified_authority());
        let candidate = envelope(oracle.reference_journal(), 0);
        assert_eq!(
            oracle.admit_record_create("did:mycelix:attacker", candidate),
            AdmissionDecision::Rejected(AdmissionRejection::AuthorMismatch)
        );
    }

    #[test]
    fn exact_concurrent_genesis_is_idempotent() {
        let mut oracle = oracle(qualified_authority());
        let candidate = envelope(oracle.reference_journal(), 0);
        assert_eq!(
            oracle.admit_record_create(AUTHOR, candidate.clone()),
            AdmissionDecision::Accepted
        );
        assert_eq!(
            oracle.admit_record_create(AUTHOR, candidate),
            AdmissionDecision::ExistingSame
        );
    }

    #[test]
    fn same_sequence_changed_content_halts_and_is_sticky() {
        let mut oracle = oracle(qualified_authority());
        let canonical = envelope(oracle.reference_journal(), 0);
        assert_eq!(
            oracle.admit_record_create(AUTHOR, canonical.clone()),
            AdmissionDecision::Accepted
        );
        let mut conflicting = canonical.clone();
        conflicting.recorded_at_unix_ms += 1;
        assert!(matches!(
            oracle.admit_record_create(AUTHOR, conflicting),
            AdmissionDecision::IntegrityHalted(AdmissionFault::SameSequenceConflict {
                sequence: 0
            })
        ));
        assert!(matches!(
            oracle.admit_record_create(AUTHOR, canonical),
            AdmissionDecision::IntegrityHalted(_)
        ));
    }

    #[test]
    fn orphan_non_genesis_record_is_rejected() {
        let mut oracle = oracle(qualified_authority());
        let candidate = envelope(oracle.reference_journal(), 1);
        assert_eq!(
            oracle.admit_record_create(AUTHOR, candidate),
            AdmissionDecision::Rejected(AdmissionRejection::MissingPredecessor)
        );
    }

    #[test]
    fn forged_predecessor_is_rejected() {
        let mut oracle = oracle(qualified_authority());
        admit_through(&mut oracle, 0);
        let mut candidate = envelope(oracle.reference_journal(), 1);
        candidate.previous_record_commitment = Some("payments-provider-journal-record-v1:ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff".into());
        assert_eq!(
            oracle.admit_record_create(AUTHOR, candidate),
            AdmissionDecision::Rejected(AdmissionRejection::PredecessorMismatch)
        );
    }

    #[test]
    fn receipt_transplant_is_rejected() {
        let mut oracle = oracle(qualified_authority());
        admit_through(&mut oracle, 1);
        let mut candidate = envelope(oracle.reference_journal(), 2);
        if let ProviderJournalBodyV1::Observation { outcome, .. } = &mut candidate.body {
            if let ProviderOutcomeV1::KnownSuccess {
                provider_receipt_id,
                ..
            } = outcome
            {
                *provider_receipt_id = "receipt-from-other-operation".into();
            }
        }
        assert_eq!(
            oracle.admit_record_create(AUTHOR, candidate),
            AdmissionDecision::Rejected(AdmissionRejection::ProjectionInvalid)
        );
    }

    #[test]
    fn request_transplant_is_rejected() {
        let mut oracle = oracle(qualified_authority());
        admit_through(&mut oracle, 1);
        let mut candidate = envelope(oracle.reference_journal(), 2);
        if let ProviderJournalBodyV1::Observation {
            request_commitment,
            ..
        } = &mut candidate.body
        {
            *request_commitment = "other-request".into();
        }
        assert_eq!(
            oracle.admit_record_create(AUTHOR, candidate),
            AdmissionDecision::Rejected(AdmissionRejection::ProjectionInvalid)
        );
    }

    #[test]
    fn attempt_horizon_transplant_is_rejected() {
        let mut oracle = oracle(qualified_authority());
        admit_through(&mut oracle, 1);
        let mut candidate = envelope(oracle.reference_journal(), 2);
        if let ProviderJournalBodyV1::Observation {
            covers_through_attempt_ordinal,
            ..
        } = &mut candidate.body
        {
            *covers_through_attempt_ordinal += 1;
        }
        assert_eq!(
            oracle.admit_record_create(AUTHOR, candidate),
            AdmissionDecision::Rejected(AdmissionRejection::ProjectionInvalid)
        );
    }

    #[test]
    fn exact_index_is_idempotent_but_conflicting_index_halts() {
        let mut oracle = oracle(qualified_authority());
        admit_through(&mut oracle, 0);
        let canonical =
            ProviderOperationIndexProjection::from_journal(oracle.reference_journal()).unwrap();
        assert_eq!(
            oracle.admit_index_create(AUTHOR, canonical.clone()),
            AdmissionDecision::Accepted
        );
        assert_eq!(
            oracle.admit_index_create(AUTHOR, canonical.clone()),
            AdmissionDecision::ExistingSame
        );
        let mut conflicting = canonical;
        conflicting.provider_operation_key.push_str("-attacker");
        assert!(matches!(
            oracle.admit_index_create(AUTHOR, conflicting),
            AdmissionDecision::IntegrityHalted(AdmissionFault::ConflictingOperationIndex { .. })
        ));
    }

    #[test]
    fn index_cannot_exist_before_genesis() {
        let mut oracle = oracle(qualified_authority());
        let canonical =
            ProviderOperationIndexProjection::from_journal(oracle.reference_journal()).unwrap();
        assert_eq!(
            oracle.admit_index_create(AUTHOR, canonical),
            AdmissionDecision::Rejected(AdmissionRejection::IndexRequiresAdmittedGenesis)
        );
    }

    #[test]
    fn update_and_delete_attempts_are_forbidden() {
        let oracle = oracle(qualified_authority());
        assert_eq!(
            oracle.admit_record_update(),
            AdmissionDecision::Rejected(AdmissionRejection::MutationForbidden)
        );
        assert_eq!(
            oracle.admit_record_delete(),
            AdmissionDecision::Rejected(AdmissionRejection::MutationForbidden)
        );
        assert_eq!(
            oracle.admit_index_delete(),
            AdmissionDecision::Rejected(AdmissionRejection::MutationForbidden)
        );
    }

    #[test]
    fn historical_payment_id_never_becomes_operation_index_authority() {
        let oracle = oracle(qualified_authority());
        let canonical =
            ProviderOperationIndexProjection::from_journal(oracle.reference_journal()).unwrap();
        assert_ne!(canonical.provider_operation_key, "payment-legacy-1");
        assert_ne!(canonical.execution_id, "payment-legacy-1");
    }
}
