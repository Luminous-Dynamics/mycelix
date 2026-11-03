#![forbid(unsafe_code)]

//! Pure reference model for a journaled SAP payment operation.
//!
//! This crate is deliberately not wired into the production Finance zome.
//! It models ordering, stable per-leg effect identities, retry/reconciliation,
//! and the point at which a local ledger payment may be called Completed.
//! It does not prove Holochain DHT uniqueness, cross-agent atomicity, account
//! conservation, provider settlement, or external legal finality.

use std::collections::BTreeMap;
use std::fmt;

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct PaymentIntent {
    sender_did: String,
    recipient_did: String,
    client_idempotency_key: String,
    payload_fingerprint: String,
    amount_minor: u64,
    fee_minor: u64,
    currency: String,
}

impl PaymentIntent {
    pub fn new(
        sender_did: impl Into<String>,
        recipient_did: impl Into<String>,
        client_idempotency_key: impl Into<String>,
        payload_fingerprint: impl Into<String>,
        amount_minor: u64,
        fee_minor: u64,
        currency: impl Into<String>,
    ) -> Self {
        Self {
            sender_did: sender_did.into(),
            recipient_did: recipient_did.into(),
            client_idempotency_key: client_idempotency_key.into(),
            payload_fingerprint: payload_fingerprint.into(),
            amount_minor,
            fee_minor,
            currency: currency.into(),
        }
    }

    fn validate(&self) -> Result<(), SagaError> {
        if self.sender_did.is_empty()
            || self.recipient_did.is_empty()
            || self.client_idempotency_key.is_empty()
            || self.payload_fingerprint.is_empty()
            || self.currency.is_empty()
        {
            return Err(SagaError::InvalidIntent("required field is empty"));
        }
        if self.sender_did == self.recipient_did {
            return Err(SagaError::InvalidIntent("sender and recipient must differ"));
        }
        if self.amount_minor == 0 {
            return Err(SagaError::InvalidIntent("amount must be positive"));
        }
        self.amount_minor
            .checked_add(self.fee_minor)
            .ok_or(SagaError::AmountOverflow)?;
        Ok(())
    }

    pub fn sender_did(&self) -> &str {
        &self.sender_did
    }

    pub fn recipient_did(&self) -> &str {
        &self.recipient_did
    }

    pub fn amount_minor(&self) -> u64 {
        self.amount_minor
    }

    pub fn fee_minor(&self) -> u64 {
        self.fee_minor
    }

    pub fn currency(&self) -> &str {
        &self.currency
    }

    pub fn fingerprint(&self) -> &str {
        &self.payload_fingerprint
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd)]
pub enum PaymentLeg {
    SenderDebit,
    RecipientCredit,
    TreasuryFee,
    PaymentRecord,
    Receipt,
}

impl PaymentLeg {
    const ALL: [Self; 5] = [
        Self::SenderDebit,
        Self::RecipientCredit,
        Self::TreasuryFee,
        Self::PaymentRecord,
        Self::Receipt,
    ];

    fn suffix(self) -> &'static str {
        match self {
            Self::SenderDebit => "debit",
            Self::RecipientCredit => "credit",
            Self::TreasuryFee => "fee",
            Self::PaymentRecord => "payment-record",
            Self::Receipt => "receipt",
        }
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum EffectState {
    NotStarted,
    NotRequired,
    Prepared { effect_id: String },
    Indeterminate { effect_id: String },
    Committed {
        effect_id: String,
        evidence_digest: String,
    },
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum OperationStatus {
    Proposed,
    Authorized,
    DebitPending,
    CreditPending,
    FeePending,
    PaymentEvidencePending,
    ReceiptEvidencePending,
    Indeterminate,
    Rejected,
    Completed,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Transition {
    Prepared,
    AlreadyPrepared,
    ResumedSameEffect,
    Committed,
    AlreadyCommitted,
    NotRequired,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum SagaError {
    InvalidIntent(&'static str),
    AmountOverflow,
    IdempotencyConflict,
    NotAuthorized,
    Rejected,
    DependencyNotSatisfied(PaymentLeg),
    EffectIdentityMismatch(PaymentLeg),
    EffectEvidenceConflict(PaymentLeg),
    InvalidTransition(PaymentLeg),
    CannotRejectAfterEffectAttempt,
    EmptyEvidenceDigest,
}

impl fmt::Display for SagaError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidIntent(reason) => write!(formatter, "invalid payment intent: {reason}"),
            Self::AmountOverflow => write!(formatter, "principal plus fee overflows the amount unit"),
            Self::IdempotencyConflict => write!(formatter, "idempotency key reused with a different immutable intent"),
            Self::NotAuthorized => write!(formatter, "payment intent has not been authorized"),
            Self::Rejected => write!(formatter, "payment operation has been rejected"),
            Self::DependencyNotSatisfied(leg) => write!(formatter, "payment leg {leg:?} has unsatisfied dependencies"),
            Self::EffectIdentityMismatch(leg) => write!(formatter, "payment leg {leg:?} changed its stable effect identity"),
            Self::EffectEvidenceConflict(leg) => write!(formatter, "payment leg {leg:?} has conflicting commit evidence"),
            Self::InvalidTransition(leg) => write!(formatter, "invalid transition for payment leg {leg:?}"),
            Self::CannotRejectAfterEffectAttempt => write!(formatter, "cannot reject after an economic effect may have occurred"),
            Self::EmptyEvidenceDigest => write!(formatter, "effect evidence digest must be non-empty"),
        }
    }
}

impl std::error::Error for SagaError {}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct PaymentJournal {
    intent: PaymentIntent,
    operation_id: String,
    authorized: bool,
    rejected: bool,
    legs: BTreeMap<PaymentLeg, EffectState>,
}

impl PaymentJournal {
    fn new(intent: PaymentIntent) -> Self {
        // Length-prefix both identity components so separators inside a DID or
        // client key cannot create ambiguous operation IDs.
        let operation_id = format!(
            "sap-payment:{}:{}:{}:{}",
            intent.sender_did.len(),
            intent.sender_did,
            intent.client_idempotency_key.len(),
            intent.client_idempotency_key
        );
        let mut legs = BTreeMap::new();
        for leg in PaymentLeg::ALL {
            legs.insert(leg, EffectState::NotStarted);
        }
        if intent.fee_minor == 0 {
            legs.insert(PaymentLeg::TreasuryFee, EffectState::NotRequired);
        }
        Self {
            intent,
            operation_id,
            authorized: false,
            rejected: false,
            legs,
        }
    }

    pub fn operation_id(&self) -> &str {
        &self.operation_id
    }

    pub fn intent(&self) -> &PaymentIntent {
        &self.intent
    }

    pub fn effect_id(&self, leg: PaymentLeg) -> String {
        format!("{}:{}", self.operation_id, leg.suffix())
    }

    pub fn effect_state(&self, leg: PaymentLeg) -> &EffectState {
        // Every leg is installed at creation and never removed.
        self.legs.get(&leg).expect("all payment legs are initialized")
    }

    pub fn authorize(&mut self) -> Result<Transition, SagaError> {
        if self.rejected {
            return Err(SagaError::Rejected);
        }
        if self.authorized {
            return Ok(Transition::AlreadyPrepared);
        }
        self.authorized = true;
        Ok(Transition::Prepared)
    }

    pub fn reject_before_effects(&mut self) -> Result<(), SagaError> {
        if self.has_effect_attempted() {
            return Err(SagaError::CannotRejectAfterEffectAttempt);
        }
        if self.rejected {
            return Ok(());
        }
        self.rejected = true;
        Ok(())
    }

    fn has_effect_attempted(&self) -> bool {
        self.legs.values().any(|state| {
            matches!(
                state,
                EffectState::Prepared { .. }
                    | EffectState::Indeterminate { .. }
                    | EffectState::Committed { .. }
            )
        })
    }

    fn leg_is_satisfied(&self, leg: PaymentLeg) -> bool {
        matches!(
            self.effect_state(leg),
            EffectState::NotRequired | EffectState::Committed { .. }
        )
    }

    fn dependencies_satisfied(&self, leg: PaymentLeg) -> bool {
        match leg {
            PaymentLeg::SenderDebit => true,
            PaymentLeg::RecipientCredit => self.leg_is_satisfied(PaymentLeg::SenderDebit),
            PaymentLeg::TreasuryFee => {
                self.leg_is_satisfied(PaymentLeg::SenderDebit)
                    && self.leg_is_satisfied(PaymentLeg::RecipientCredit)
            }
            PaymentLeg::PaymentRecord => {
                self.leg_is_satisfied(PaymentLeg::SenderDebit)
                    && self.leg_is_satisfied(PaymentLeg::RecipientCredit)
                    && self.leg_is_satisfied(PaymentLeg::TreasuryFee)
            }
            PaymentLeg::Receipt => self.leg_is_satisfied(PaymentLeg::PaymentRecord),
        }
    }

    pub fn prepare_leg(
        &mut self,
        leg: PaymentLeg,
        effect_id: &str,
    ) -> Result<Transition, SagaError> {
        if self.rejected {
            return Err(SagaError::Rejected);
        }
        if !self.authorized {
            return Err(SagaError::NotAuthorized);
        }
        if effect_id != self.effect_id(leg) {
            return Err(SagaError::EffectIdentityMismatch(leg));
        }
        if leg == PaymentLeg::TreasuryFee && self.intent.fee_minor == 0 {
            return Ok(Transition::NotRequired);
        }
        if !self.dependencies_satisfied(leg) {
            return Err(SagaError::DependencyNotSatisfied(leg));
        }

        match self.effect_state(leg).clone() {
            EffectState::NotStarted => {
                self.legs.insert(
                    leg,
                    EffectState::Prepared {
                        effect_id: effect_id.to_owned(),
                    },
                );
                Ok(Transition::Prepared)
            }
            EffectState::Prepared { effect_id: current } => {
                if current == effect_id {
                    Ok(Transition::AlreadyPrepared)
                } else {
                    Err(SagaError::EffectIdentityMismatch(leg))
                }
            }
            EffectState::Indeterminate { effect_id: current } => {
                if current == effect_id {
                    self.legs.insert(
                        leg,
                        EffectState::Prepared {
                            effect_id: current,
                        },
                    );
                    Ok(Transition::ResumedSameEffect)
                } else {
                    Err(SagaError::EffectIdentityMismatch(leg))
                }
            }
            EffectState::Committed {
                effect_id: current, ..
            } => {
                if current == effect_id {
                    Ok(Transition::AlreadyCommitted)
                } else {
                    Err(SagaError::EffectIdentityMismatch(leg))
                }
            }
            EffectState::NotRequired => Ok(Transition::NotRequired),
        }
    }

    pub fn mark_indeterminate(
        &mut self,
        leg: PaymentLeg,
        effect_id: &str,
    ) -> Result<(), SagaError> {
        match self.effect_state(leg).clone() {
            EffectState::Prepared { effect_id: current }
            | EffectState::Indeterminate { effect_id: current }
                if current == effect_id =>
            {
                self.legs.insert(
                    leg,
                    EffectState::Indeterminate {
                        effect_id: current,
                    },
                );
                Ok(())
            }
            EffectState::Committed { effect_id: current, .. } if current == effect_id => {
                Ok(())
            }
            _ => Err(SagaError::InvalidTransition(leg)),
        }
    }

    pub fn commit_leg(
        &mut self,
        leg: PaymentLeg,
        effect_id: &str,
        evidence_digest: &str,
    ) -> Result<Transition, SagaError> {
        if evidence_digest.is_empty() {
            return Err(SagaError::EmptyEvidenceDigest);
        }
        if effect_id != self.effect_id(leg) {
            return Err(SagaError::EffectIdentityMismatch(leg));
        }
        if leg == PaymentLeg::TreasuryFee && self.intent.fee_minor == 0 {
            return Ok(Transition::NotRequired);
        }
        if !self.dependencies_satisfied(leg) {
            return Err(SagaError::DependencyNotSatisfied(leg));
        }

        match self.effect_state(leg).clone() {
            EffectState::Prepared { effect_id: current }
            | EffectState::Indeterminate { effect_id: current }
                if current == effect_id =>
            {
                self.legs.insert(
                    leg,
                    EffectState::Committed {
                        effect_id: current,
                        evidence_digest: evidence_digest.to_owned(),
                    },
                );
                Ok(Transition::Committed)
            }
            EffectState::Committed {
                effect_id: current,
                evidence_digest: current_digest,
            } if current == effect_id && current_digest == evidence_digest => {
                Ok(Transition::AlreadyCommitted)
            }
            EffectState::Committed {
                effect_id: current, ..
            } if current == effect_id => Err(SagaError::EffectEvidenceConflict(leg)),
            _ => Err(SagaError::InvalidTransition(leg)),
        }
    }

    pub fn status(&self) -> OperationStatus {
        if self.rejected {
            return OperationStatus::Rejected;
        }
        if self.leg_is_satisfied(PaymentLeg::Receipt) {
            return OperationStatus::Completed;
        }
        if self
            .legs
            .values()
            .any(|state| matches!(state, EffectState::Indeterminate { .. }))
        {
            return OperationStatus::Indeterminate;
        }
        if self.leg_is_satisfied(PaymentLeg::PaymentRecord) {
            return OperationStatus::ReceiptEvidencePending;
        }
        if matches!(
            self.effect_state(PaymentLeg::Receipt),
            EffectState::Prepared { .. }
        ) {
            return OperationStatus::ReceiptEvidencePending;
        }
        if matches!(
            self.effect_state(PaymentLeg::PaymentRecord),
            EffectState::Prepared { .. }
        ) {
            return OperationStatus::PaymentEvidencePending;
        }
        if !self.leg_is_satisfied(PaymentLeg::TreasuryFee) {
            return if self.leg_is_satisfied(PaymentLeg::RecipientCredit) {
                OperationStatus::FeePending
            } else if matches!(
                self.effect_state(PaymentLeg::TreasuryFee),
                EffectState::Prepared { .. }
            ) {
                OperationStatus::FeePending
            } else if self.leg_is_satisfied(PaymentLeg::SenderDebit) {
                OperationStatus::CreditPending
            } else if self.authorized {
                OperationStatus::DebitPending
            } else {
                OperationStatus::Proposed
            };
        }
        if !self.leg_is_satisfied(PaymentLeg::RecipientCredit) {
            return if self.leg_is_satisfied(PaymentLeg::SenderDebit) {
                OperationStatus::CreditPending
            } else if self.authorized {
                OperationStatus::DebitPending
            } else {
                OperationStatus::Proposed
            };
        }
        if !self.leg_is_satisfied(PaymentLeg::SenderDebit) {
            return if self.authorized {
                OperationStatus::DebitPending
            } else {
                OperationStatus::Proposed
            };
        }

        // All monetary legs are satisfied; Payment record evidence is the next
        // required step even when its preparation has not yet begun.
        if self.authorized {
            OperationStatus::PaymentEvidencePending
        } else {
            OperationStatus::Proposed
        }
    }

    pub fn is_completed(&self) -> bool {
        self.status() == OperationStatus::Completed
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CreateDisposition {
    Created,
    ExistingSameIntent,
}

#[derive(Default, Debug)]
pub struct JournalRegistry {
    operations: BTreeMap<(String, String), PaymentJournal>,
}

impl JournalRegistry {
    pub fn create_or_get(
        &mut self,
        intent: PaymentIntent,
    ) -> Result<(CreateDisposition, String), SagaError> {
        intent.validate()?;
        let key = (
            intent.sender_did.clone(),
            intent.client_idempotency_key.clone(),
        );
        if let Some(existing) = self.operations.get(&key) {
            if existing.intent != intent {
                return Err(SagaError::IdempotencyConflict);
            }
            return Ok((
                CreateDisposition::ExistingSameIntent,
                existing.operation_id.clone(),
            ));
        }
        let journal = PaymentJournal::new(intent);
        let operation_id = journal.operation_id.clone();
        self.operations.insert(key, journal);
        Ok((CreateDisposition::Created, operation_id))
    }

    pub fn get(&self, sender_did: &str, idempotency_key: &str) -> Option<&PaymentJournal> {
        self.operations
            .get(&(sender_did.to_owned(), idempotency_key.to_owned()))
    }

    pub fn get_mut(
        &mut self,
        sender_did: &str,
        idempotency_key: &str,
    ) -> Option<&mut PaymentJournal> {
        self.operations
            .get_mut(&(sender_did.to_owned(), idempotency_key.to_owned()))
    }

    pub fn len(&self) -> usize {
        self.operations.len()
    }

    pub fn is_empty(&self) -> bool {
        self.operations.is_empty()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::{Arc, Mutex};
    use std::thread;

    fn intent() -> PaymentIntent {
        PaymentIntent::new(
            "did:mycelix:alice",
            "did:mycelix:bob",
            "client-op-001",
            "sha256:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
            10_000,
            100,
            "SAP",
        )
    }

    fn ready_journal() -> PaymentJournal {
        let mut registry = JournalRegistry::default();
        registry.create_or_get(intent()).unwrap();
        let journal = registry
            .get_mut("did:mycelix:alice", "client-op-001")
            .unwrap();
        journal.authorize().unwrap();
        let debit_id = journal.effect_id(PaymentLeg::SenderDebit);
        journal
            .prepare_leg(PaymentLeg::SenderDebit, &debit_id)
            .unwrap();
        journal
            .commit_leg(PaymentLeg::SenderDebit, &debit_id, "sha256:debit-evidence")
            .unwrap();
        let credit_id = journal.effect_id(PaymentLeg::RecipientCredit);
        journal
            .prepare_leg(PaymentLeg::RecipientCredit, &credit_id)
            .unwrap();
        journal
            .commit_leg(PaymentLeg::RecipientCredit, &credit_id, "sha256:credit-evidence")
            .unwrap();
        let fee_id = journal.effect_id(PaymentLeg::TreasuryFee);
        journal
            .prepare_leg(PaymentLeg::TreasuryFee, &fee_id)
            .unwrap();
        journal
            .commit_leg(PaymentLeg::TreasuryFee, &fee_id, "sha256:fee-evidence")
            .unwrap();
        journal.clone()
    }

    #[test]
    fn exact_retry_returns_one_existing_operation() {
        let mut registry = JournalRegistry::default();
        let (first, id1) = registry.create_or_get(intent()).unwrap();
        let (second, id2) = registry.create_or_get(intent()).unwrap();
        assert_eq!(first, CreateDisposition::Created);
        assert_eq!(second, CreateDisposition::ExistingSameIntent);
        assert_eq!(id1, id2);
        assert_eq!(registry.len(), 1);
    }

    #[test]
    fn changed_payload_under_same_sender_key_conflicts() {
        let mut registry = JournalRegistry::default();
        registry.create_or_get(intent()).unwrap();
        let changed = PaymentIntent::new(
            "did:mycelix:alice",
            "did:mycelix:mallory",
            "client-op-001",
            "sha256:bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb",
            10_000,
            100,
            "SAP",
        );
        assert_eq!(
            registry.create_or_get(changed),
            Err(SagaError::IdempotencyConflict)
        );
    }

    #[test]
    fn principal_fee_overflow_and_invalid_amount_fail_before_journal_creation() {
        let mut registry = JournalRegistry::default();
        let overflow = PaymentIntent::new(
            "alice",
            "bob",
            "op-overflow",
            "fp-1",
            u64::MAX,
            1,
            "SAP",
        );
        assert_eq!(registry.create_or_get(overflow), Err(SagaError::AmountOverflow));
        let zero = PaymentIntent::new("alice", "bob", "op-zero", "fp-2", 0, 0, "SAP");
        assert!(matches!(
            registry.create_or_get(zero),
            Err(SagaError::InvalidIntent(_))
        ));
        assert!(registry.is_empty());
    }

    #[test]
    fn recipient_credit_cannot_start_before_sender_debit_commit() {
        let mut registry = JournalRegistry::default();
        registry.create_or_get(intent()).unwrap();
        let journal = registry.get_mut("did:mycelix:alice", "client-op-001").unwrap();
        journal.authorize().unwrap();
        let credit_id = journal.effect_id(PaymentLeg::RecipientCredit);
        assert_eq!(
            journal.prepare_leg(PaymentLeg::RecipientCredit, &credit_id),
            Err(SagaError::DependencyNotSatisfied(PaymentLeg::RecipientCredit))
        );
    }

    #[test]
    fn debit_ack_loss_retries_same_effect_not_a_second_debit() {
        let mut journal = ready_journal();
        let debit_id = journal.effect_id(PaymentLeg::SenderDebit);
        assert_eq!(
            journal.prepare_leg(PaymentLeg::SenderDebit, &debit_id).unwrap(),
            Transition::AlreadyCommitted
        );
        assert_eq!(
            journal
                .commit_leg(PaymentLeg::SenderDebit, &debit_id, "sha256:debit-evidence")
                .unwrap(),
            Transition::AlreadyCommitted
        );
        assert_eq!(
            journal.effect_state(PaymentLeg::SenderDebit),
            &EffectState::Committed {
                effect_id: debit_id,
                evidence_digest: "sha256:debit-evidence".into()
            }
        );
    }

    #[test]
    fn same_leg_cannot_change_effect_identity_or_commit_evidence() {
        let mut registry = JournalRegistry::default();
        registry.create_or_get(intent()).unwrap();
        let journal = registry.get_mut("did:mycelix:alice", "client-op-001").unwrap();
        journal.authorize().unwrap();
        let debit_id = journal.effect_id(PaymentLeg::SenderDebit);
        journal.prepare_leg(PaymentLeg::SenderDebit, &debit_id).unwrap();
        assert_eq!(
            journal.prepare_leg(PaymentLeg::SenderDebit, "a-new-effect-id"),
            Err(SagaError::EffectIdentityMismatch(PaymentLeg::SenderDebit))
        );
        journal.commit_leg(PaymentLeg::SenderDebit, &debit_id, "digest-a").unwrap();
        assert_eq!(
            journal.commit_leg(PaymentLeg::SenderDebit, &debit_id, "digest-b"),
            Err(SagaError::EffectEvidenceConflict(PaymentLeg::SenderDebit))
        );
    }

    #[test]
    fn lost_credit_ack_is_indeterminate_and_resumes_original_credit_id() {
        let mut journal = ready_journal();
        let credit_id = journal.effect_id(PaymentLeg::RecipientCredit);
        // Model an in-flight credit whose response was lost. The debit remains committed.
        journal
            .mark_indeterminate(PaymentLeg::RecipientCredit, &credit_id)
            .unwrap();
        assert_eq!(journal.status(), OperationStatus::Indeterminate);
        assert_eq!(
            journal.prepare_leg(PaymentLeg::RecipientCredit, &credit_id).unwrap(),
            Transition::ResumedSameEffect
        );
        journal
            .commit_leg(PaymentLeg::RecipientCredit, &credit_id, "sha256:credit-evidence")
            .unwrap();
        assert_eq!(
            journal.effect_state(PaymentLeg::SenderDebit),
            &EffectState::Committed {
                effect_id: journal.effect_id(PaymentLeg::SenderDebit),
                evidence_digest: "sha256:debit-evidence".into()
            }
        );
    }

    #[test]
    fn fee_failure_blocks_payment_record_and_completion() {
        let mut registry = JournalRegistry::default();
        registry.create_or_get(intent()).unwrap();
        let journal = registry.get_mut("did:mycelix:alice", "client-op-001").unwrap();
        journal.authorize().unwrap();
        let debit = journal.effect_id(PaymentLeg::SenderDebit);
        journal.prepare_leg(PaymentLeg::SenderDebit, &debit).unwrap();
        journal.commit_leg(PaymentLeg::SenderDebit, &debit, "debit").unwrap();
        let credit = journal.effect_id(PaymentLeg::RecipientCredit);
        journal.prepare_leg(PaymentLeg::RecipientCredit, &credit).unwrap();
        journal.commit_leg(PaymentLeg::RecipientCredit, &credit, "credit").unwrap();
        let fee = journal.effect_id(PaymentLeg::TreasuryFee);
        journal.prepare_leg(PaymentLeg::TreasuryFee, &fee).unwrap();
        journal.mark_indeterminate(PaymentLeg::TreasuryFee, &fee).unwrap();
        assert_eq!(journal.status(), OperationStatus::Indeterminate);
        let payment_record = journal.effect_id(PaymentLeg::PaymentRecord);
        assert_eq!(
            journal.prepare_leg(PaymentLeg::PaymentRecord, &payment_record),
            Err(SagaError::DependencyNotSatisfied(PaymentLeg::PaymentRecord))
        );
        assert!(!journal.is_completed());
    }

    #[test]
    fn zero_fee_is_explicitly_not_required_but_evidence_still_required() {
        let no_fee = PaymentIntent::new("alice", "bob", "op-no-fee", "fp", 10, 0, "SAP");
        let mut registry = JournalRegistry::default();
        registry.create_or_get(no_fee).unwrap();
        let journal = registry.get_mut("alice", "op-no-fee").unwrap();
        assert_eq!(
            journal.effect_state(PaymentLeg::TreasuryFee),
            &EffectState::NotRequired
        );
    }

    #[test]
    fn payment_record_and_receipt_are_required_before_completed() {
        let mut journal = ready_journal();
        assert_eq!(journal.status(), OperationStatus::PaymentEvidencePending);
        let payment_id = journal.effect_id(PaymentLeg::PaymentRecord);
        journal.prepare_leg(PaymentLeg::PaymentRecord, &payment_id).unwrap();
        assert_eq!(journal.status(), OperationStatus::PaymentEvidencePending);
        journal.commit_leg(PaymentLeg::PaymentRecord, &payment_id, "payment").unwrap();
        assert_eq!(journal.status(), OperationStatus::ReceiptEvidencePending);
        let receipt_id = journal.effect_id(PaymentLeg::Receipt);
        journal.prepare_leg(PaymentLeg::Receipt, &receipt_id).unwrap();
        journal.commit_leg(PaymentLeg::Receipt, &receipt_id, "receipt").unwrap();
        assert_eq!(journal.status(), OperationStatus::Completed);
    }

    #[test]
    fn receipt_cannot_be_created_before_payment_record() {
        let mut journal = ready_journal();
        let receipt_id = journal.effect_id(PaymentLeg::Receipt);
        assert_eq!(
            journal.prepare_leg(PaymentLeg::Receipt, &receipt_id),
            Err(SagaError::DependencyNotSatisfied(PaymentLeg::Receipt))
        );
    }

    #[test]
    fn cannot_reject_after_an_effect_might_have_occurred() {
        let mut registry = JournalRegistry::default();
        registry.create_or_get(intent()).unwrap();
        let journal = registry.get_mut("did:mycelix:alice", "client-op-001").unwrap();
        journal.authorize().unwrap();
        let debit = journal.effect_id(PaymentLeg::SenderDebit);
        journal.prepare_leg(PaymentLeg::SenderDebit, &debit).unwrap();
        assert_eq!(
            journal.reject_before_effects(),
            Err(SagaError::CannotRejectAfterEffectAttempt)
        );
    }

    #[test]
    fn concurrent_same_key_creation_is_singleton_in_the_reference_registry() {
        let registry = Arc::new(Mutex::new(JournalRegistry::default()));
        let workers: Vec<_> = (0..16)
            .map(|_| {
                let registry = Arc::clone(&registry);
                thread::spawn(move || {
                    registry
                        .lock()
                        .unwrap()
                        .create_or_get(intent())
                        .unwrap()
                })
            })
            .collect();
        let outcomes: Vec<_> = workers.into_iter().map(|worker| worker.join().unwrap()).collect();
        let ids: std::collections::BTreeSet<_> =
            outcomes.iter().map(|(_, operation_id)| operation_id).collect();
        assert_eq!(ids.len(), 1);
        assert_eq!(registry.lock().unwrap().len(), 1);
        assert_eq!(
            outcomes.iter().filter(|(disposition, _)| *disposition == CreateDisposition::Created).count(),
            1
        );
    }

    #[test]
    fn fee_and_balance_legs_are_never_replayed_when_evidence_is_pending() {
        let mut journal = ready_journal();
        let payment_id = journal.effect_id(PaymentLeg::PaymentRecord);
        journal.prepare_leg(PaymentLeg::PaymentRecord, &payment_id).unwrap();
        journal.mark_indeterminate(PaymentLeg::PaymentRecord, &payment_id).unwrap();
        assert_eq!(journal.status(), OperationStatus::Indeterminate);
        assert_eq!(
            journal.prepare_leg(PaymentLeg::PaymentRecord, &payment_id).unwrap(),
            Transition::ResumedSameEffect
        );
        assert_eq!(
            journal.effect_state(PaymentLeg::SenderDebit),
            &EffectState::Committed {
                effect_id: journal.effect_id(PaymentLeg::SenderDebit),
                evidence_digest: "sha256:debit-evidence".into()
            }
        );
    }
}
