#![forbid(unsafe_code)]

//! Effects-disabled reference model for a sender-owned SAP payment.
//!
//! The local Payment coordinator calls its debit/credit helpers directly during
//! one zome invocation. Holochain commits that invocation's source-chain write
//! set atomically. Treasury fee delivery is a separate inter-zome call and is
//! modeled as a second, stable-ID effect that can be retried after the local
//! payment transaction has committed.
//!
//! This crate does not prove Holochain DHT uniqueness, action authenticity,
//! predecessor authorization, monetary conservation in production, or legal
//!/external settlement finality.

use std::collections::{BTreeMap, btree_map::Entry};
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
            || self.currency.is_empty()
        {
            return Err(SagaError::InvalidIntent("required field is empty"));
        }
        if self.sender_did == self.recipient_did {
            return Err(SagaError::InvalidIntent("sender and recipient must differ"));
        }
        if !is_sha256_digest(&self.payload_fingerprint) {
            return Err(SagaError::InvalidIntent(
                "payload fingerprint must be sha256:<64 lowercase hex characters>",
            ));
        }
        if self.amount_minor == 0 {
            return Err(SagaError::InvalidIntent("amount must be positive"));
        }
        self.amount_minor
            .checked_add(self.fee_minor)
            .ok_or(SagaError::AmountOverflow)?;
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd)]
pub enum PaymentLeg {
    /// One atomic source-chain write set: debit, credit, Payment, Receipt and
    /// a durable fee-distribution outbox intent.
    LocalPaymentTransaction,
    /// Separate inter-zome Treasury movement, recovered by its stable ID.
    TreasuryFee,
}

impl PaymentLeg {
    const ALL: [Self; 2] = [Self::LocalPaymentTransaction, Self::TreasuryFee];

    fn suffix(self) -> &'static str {
        match self {
            Self::LocalPaymentTransaction => "local-payment-transaction",
            Self::TreasuryFee => "treasury-fee",
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MovementDirection {
    Debit,
    Credit,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PaymentRecordStatus {
    Completed,
    Pending,
    Failed,
}

/// Typed subject data supplied by a source/effect adapter.
///
/// The model checks subject equality and reference syntax. It does not prove
/// that a referenced source record exists or that its digest/signature is
/// cryptographically authentic.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PaymentRecordStatus {
    Completed,
    Pending,
    Failed,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct EffectEvidence {
    pub operation_id: String,
    pub effect_id: String,
    pub leg: PaymentLeg,
    pub sender_did: String,
    pub recipient_did: String,
    pub principal_minor: u64,
    pub fee_minor: u64,
    pub currency: String,
    pub evidence_digest: String,
    pub kind: EffectEvidenceKind,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum EffectEvidenceKind {
    LocalPaymentTransaction {
        sender_account_id: String,
        recipient_account_id: String,
        sender_debit_minor: u64,
        recipient_credit_minor: u64,
        sender_predecessor_action: String,
        sender_debit_action: String,
        recipient_predecessor_action: String,
        recipient_credit_action: String,
        payment_action_hash: String,
        payment_status: PaymentRecordStatus,
        receipt_action_hash: String,
        receipt_signer_did: String,
        signing_profile: String,
        signature_digest: String,
        treasury_fee_outbox_id: Option<String>,
    },
    TreasuryFee {
        pool_id: String,
        source_member_did: String,
        amount_minor: u64,
        payment_action_hash: String,
        idempotency_key: String,
        predecessor_action: String,
        treasury_action: String,
    },
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum EffectState {
    NotStarted,
    NotRequired,
    Prepared { effect_id: String },
    Indeterminate { effect_id: String },
    Committed {
        effect_id: String,
        evidence: EffectEvidence,
    },
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum OperationStatus {
    Proposed,
    Authorized,
    LocalPaymentPending,
    FeeDistributionPending,
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

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CreateDisposition {
    Created,
    ExistingSameIntent,
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
    InvalidEffectEvidence(PaymentLeg),
}

impl fmt::Display for SagaError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidIntent(reason) => write!(formatter, "invalid payment intent: {reason}"),
            Self::AmountOverflow => {
                write!(formatter, "principal plus fee overflows the amount unit")
            }
            Self::IdempotencyConflict => {
                write!(formatter, "sender/key reused with a different immutable intent")
            }
            Self::NotAuthorized => write!(formatter, "payment intent has not been authorized"),
            Self::Rejected => write!(formatter, "payment operation has been rejected"),
            Self::DependencyNotSatisfied(leg) => {
                write!(formatter, "payment leg {leg:?} has unsatisfied dependencies")
            }
            Self::EffectIdentityMismatch(leg) => {
                write!(formatter, "payment leg {leg:?} changed its stable effect identity")
            }
            Self::EffectEvidenceConflict(leg) => {
                write!(formatter, "payment leg {leg:?} has conflicting commit evidence")
            }
            Self::InvalidTransition(leg) => {
                write!(formatter, "invalid transition for payment leg {leg:?}")
            }
            Self::CannotRejectAfterEffectAttempt => {
                write!(formatter, "cannot reject after an economic effect may have occurred")
            }
            Self::InvalidEffectEvidence(leg) => {
                write!(formatter, "effect evidence does not match immutable intent for {leg:?}")
            }
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

    fn local_payment_committed(&self) -> bool {
        matches!(
            self.effect_state(PaymentLeg::LocalPaymentTransaction),
            EffectState::Committed { .. }
        )
    }

    fn committed_payment_action_hash(&self) -> Option<&str> {
        match self.effect_state(PaymentLeg::LocalPaymentTransaction) {
            EffectState::Committed { evidence, .. } => {
                match &evidence.kind {
                    EffectEvidenceKind::LocalPaymentTransaction {
                        payment_action_hash,
                        ..
                    } => Some(payment_action_hash),
                    _ => None,
                }
            }
            _ => None,
        }
    }

    fn fee_outbox_id(&self) -> Option<&str> {
        match self.effect_state(PaymentLeg::LocalPaymentTransaction) {
            EffectState::Committed { evidence, .. } => {
                match &evidence.kind {
                    EffectEvidenceKind::LocalPaymentTransaction {
                        treasury_fee_outbox_id,
                        ..
                    } => treasury_fee_outbox_id.as_deref(),
                    _ => None,
                }
            }
            _ => None,
        }
    }

    fn dependencies_satisfied(&self, leg: PaymentLeg) -> bool {
        match leg {
            PaymentLeg::LocalPaymentTransaction => true,
            PaymentLeg::TreasuryFee => self.local_payment_committed(),
        }
    }

    fn validate_effect_evidence(
        &self,
        leg: PaymentLeg,
        evidence: &EffectEvidence,
    ) -> Result<(), SagaError> {
        let common_valid = evidence.operation_id == self.operation_id
            && evidence.effect_id == self.effect_id(leg)
            && evidence.leg == leg
            && evidence.sender_did == self.intent.sender_did
            && evidence.recipient_did == self.intent.recipient_did
            && evidence.principal_minor == self.intent.amount_minor
            && evidence.fee_minor == self.intent.fee_minor
            && evidence.currency == self.intent.currency
            && is_sha256_digest(&evidence.evidence_digest);
        if !common_valid {
            return Err(SagaError::InvalidEffectEvidence(leg));
        }

        let total_debit = self
            .intent
            .amount_minor
            .checked_add(self.intent.fee_minor)
            .ok_or(SagaError::AmountOverflow)?;

        let valid = match (&leg, &evidence.kind) {
            (
                PaymentLeg::LocalPaymentTransaction,
                EffectEvidenceKind::LocalPaymentTransaction {
                    sender_account_id,
                    recipient_account_id,
                    sender_debit_minor,
                    recipient_credit_minor,
                    sender_predecessor_action,
                    sender_debit_action,
                    recipient_predecessor_action,
                    recipient_credit_action,
                    payment_action_hash,
                    payment_status,
                    receipt_action_hash,
                    receipt_signer_did,
                    signing_profile,
                    signature_digest,
                    treasury_fee_outbox_id,
                },
            ) => {
                let fee_outbox_valid = if self.intent.fee_minor == 0 {
                    treasury_fee_outbox_id.is_none()
                } else {
                    treasury_fee_outbox_id.as_deref()
                        == Some(self.effect_id(PaymentLeg::TreasuryFee).as_str())
                };
                sender_account_id == &self.intent.sender_did
                    && recipient_account_id == &self.intent.recipient_did
                    && *sender_debit_minor == total_debit
                    && *recipient_credit_minor == self.intent.amount_minor
                    && is_action_reference(sender_predecessor_action)
                    && is_action_reference(sender_debit_action)
                    && sender_predecessor_action != sender_debit_action
                    && is_action_reference(recipient_predecessor_action)
                    && is_action_reference(recipient_credit_action)
                    && recipient_predecessor_action != recipient_credit_action
                    && is_action_reference(payment_action_hash)
                    && *payment_status == PaymentRecordStatus::Completed
                    && is_action_reference(receipt_action_hash)
                    && receipt_action_hash != payment_action_hash
                    && receipt_signer_did == &self.intent.sender_did
                    && signing_profile == "mycelix.finance.payment-receipt.v1"
                    && is_sha256_digest(signature_digest)
                    && fee_outbox_valid
            }
            (
                PaymentLeg::TreasuryFee,
                EffectEvidenceKind::TreasuryFee {
                    pool_id,
                    source_member_did,
                    amount_minor,
                    payment_action_hash,
                    idempotency_key,
                    predecessor_action,
                    treasury_action,
                },
            ) => {
                self.intent.fee_minor > 0
                    && pool_id == "treasury:global-fee-pool"
                    && source_member_did == &self.intent.sender_did
                    && *amount_minor == self.intent.fee_minor
                    && Some(payment_action_hash.as_str())
                        == self.committed_payment_action_hash()
                    && Some(idempotency_key.as_str()) == self.fee_outbox_id()
                    && is_action_reference(payment_action_hash)
                    && is_action_reference(predecessor_action)
                    && is_action_reference(treasury_action)
                    && predecessor_action != treasury_action
            }
            _ => false,
        };

        if valid {
            Ok(())
        } else {
            Err(SagaError::InvalidEffectEvidence(leg))
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
        if !self.dependencies_satisfied(leg) {
            return Err(SagaError::DependencyNotSatisfied(leg));
        }
        if leg == PaymentLeg::TreasuryFee && self.intent.fee_minor == 0 {
            return Ok(Transition::NotRequired);
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
            EffectState::Committed { effect_id: current, .. } if current == effect_id => Ok(()),
            _ => Err(SagaError::InvalidTransition(leg)),
        }
    }

    pub fn commit_leg(
        &mut self,
        leg: PaymentLeg,
        evidence: EffectEvidence,
    ) -> Result<Transition, SagaError> {
        if self.rejected {
            return Err(SagaError::Rejected);
        }
        if !self.dependencies_satisfied(leg) {
            return Err(SagaError::DependencyNotSatisfied(leg));
        }
        if leg == PaymentLeg::TreasuryFee && self.intent.fee_minor == 0 {
            return Ok(Transition::NotRequired);
        }
        self.validate_effect_evidence(leg, &evidence)?;
        let effect_id = evidence.effect_id.clone();

        match self.effect_state(leg).clone() {
            EffectState::Prepared { effect_id: current }
            | EffectState::Indeterminate { effect_id: current }
                if current == effect_id =>
            {
                self.legs.insert(
                    leg,
                    EffectState::Committed {
                        effect_id,
                        evidence,
                    },
                );
                Ok(Transition::Committed)
            }
            EffectState::Committed {
                effect_id: current,
                evidence: current_evidence,
            } if current == effect_id && current_evidence == evidence => {
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
        if self
            .legs
            .values()
            .any(|state| matches!(state, EffectState::Indeterminate { .. }))
        {
            return OperationStatus::Indeterminate;
        }
        if !self.local_payment_committed() {
            return if self.authorized {
                OperationStatus::LocalPaymentPending
            } else {
                OperationStatus::Proposed
            };
        }
        if self.intent.fee_minor == 0
            || matches!(
                self.effect_state(PaymentLeg::TreasuryFee),
                EffectState::Committed { .. } | EffectState::NotRequired
            )
        {
            return OperationStatus::Completed;
        }
        OperationStatus::FeeDistributionPending
    }

    pub fn is_completed(&self) -> bool {
        self.status() == OperationStatus::Completed
    }
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
        match self.operations.entry(key) {
            Entry::Occupied(existing_entry) => {
                let existing = existing_entry.get();
                if existing.intent != intent {
                    return Err(SagaError::IdempotencyConflict);
                }
                Ok((
                    CreateDisposition::ExistingSameIntent,
                    existing.operation_id.clone(),
                ))
            }
            Entry::Vacant(vacant_entry) => {
                let journal = PaymentJournal::new(intent);
                let operation_id = journal.operation_id.clone();
                vacant_entry.insert(journal);
                Ok((CreateDisposition::Created, operation_id))
            }
        }
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

fn is_sha256_digest(value: &str) -> bool {
    value.strip_prefix("sha256:").is_some_and(|hex| {
        hex.len() == 64
            && hex
                .bytes()
                .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
    })
}

fn is_action_reference(value: &str) -> bool {
    value.strip_prefix("action:").is_some_and(|reference| {
        !reference.is_empty()
            && reference.bytes().all(|byte| {
                byte.is_ascii_alphanumeric() || byte == b'-' || byte == b'_'
            })
    })
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

    fn digest(seed: u64) -> String {
        format!("sha256:{seed:064x}")
    }

    fn local_payment_evidence(
        journal: &PaymentJournal,
        label: &str,
        seed: u64,
    ) -> EffectEvidence {
        let kind = EffectEvidenceKind::LocalPaymentTransaction {
            sender_account_id: journal.intent.sender_did.clone(),
            recipient_account_id: journal.intent.recipient_did.clone(),
            sender_debit_minor: journal.intent.amount_minor + journal.intent.fee_minor,
            recipient_credit_minor: journal.intent.amount_minor,
            sender_predecessor_action: format!("action:sender-before-{label}"),
            sender_debit_action: format!("action:sender-debit-{label}"),
            recipient_predecessor_action: format!("action:recipient-before-{label}"),
            recipient_credit_action: format!("action:recipient-credit-{label}"),
            payment_action_hash: format!("action:payment-{label}"),
            payment_status: PaymentRecordStatus::Completed,
            receipt_action_hash: format!("action:receipt-{label}"),
            receipt_signer_did: journal.intent.sender_did.clone(),
            signing_profile: "mycelix.finance.payment-receipt.v1".into(),
            signature_digest: digest(seed + 10_000),
            treasury_fee_outbox_id: if journal.intent.fee_minor == 0 {
                None
            } else {
                Some(journal.effect_id(PaymentLeg::TreasuryFee))
            },
        };
        EffectEvidence {
            operation_id: journal.operation_id.clone(),
            effect_id: journal.effect_id(PaymentLeg::LocalPaymentTransaction),
            leg: PaymentLeg::LocalPaymentTransaction,
            sender_did: journal.intent.sender_did.clone(),
            recipient_did: journal.intent.recipient_did.clone(),
            principal_minor: journal.intent.amount_minor,
            fee_minor: journal.intent.fee_minor,
            currency: journal.intent.currency.clone(),
            evidence_digest: digest(seed),
            kind,
        }
    }

    fn treasury_fee_evidence(
        journal: &PaymentJournal,
        label: &str,
        seed: u64,
    ) -> EffectEvidence {
        let payment_action_hash = journal
            .committed_payment_action_hash()
            .expect("local Payment transaction must be committed")
            .to_owned();
        EffectEvidence {
            operation_id: journal.operation_id.clone(),
            effect_id: journal.effect_id(PaymentLeg::TreasuryFee),
            leg: PaymentLeg::TreasuryFee,
            sender_did: journal.intent.sender_did.clone(),
            recipient_did: journal.intent.recipient_did.clone(),
            principal_minor: journal.intent.amount_minor,
            fee_minor: journal.intent.fee_minor,
            currency: journal.intent.currency.clone(),
            evidence_digest: digest(seed),
            kind: EffectEvidenceKind::TreasuryFee {
                pool_id: "treasury:global-fee-pool".into(),
                source_member_did: journal.intent.sender_did.clone(),
                amount_minor: journal.intent.fee_minor,
                payment_action_hash,
                idempotency_key: journal.effect_id(PaymentLeg::TreasuryFee),
                predecessor_action: format!("action:treasury-before-{label}"),
                treasury_action: format!("action:treasury-fee-{label}"),
            },
        }
    }

    fn commit_local(
        journal: &mut PaymentJournal,
        label: &str,
        seed: u64,
    ) -> Result<Transition, SagaError> {
        journal.commit_leg(
            PaymentLeg::LocalPaymentTransaction,
            local_payment_evidence(journal, label, seed),
        )
    }

    fn commit_fee(
        journal: &mut PaymentJournal,
        label: &str,
        seed: u64,
    ) -> Result<Transition, SagaError> {
        let evidence = treasury_fee_evidence(journal, label, seed);
        journal.commit_leg(PaymentLeg::TreasuryFee, evidence)
    }

    fn ready_local_payment() -> PaymentJournal {
        let mut registry = JournalRegistry::default();
        registry.create_or_get(intent()).unwrap();
        let journal = registry
            .get_mut("did:mycelix:alice", "client-op-001")
            .unwrap();
        journal.authorize().unwrap();
        let local_id = journal.effect_id(PaymentLeg::LocalPaymentTransaction);
        journal.prepare_leg(PaymentLeg::LocalPaymentTransaction, &local_id).unwrap();
        commit_local(journal, "local", 1).unwrap();
        journal.clone()
    }

    fn completed_payment() -> PaymentJournal {
        let mut journal = ready_local_payment();
        if journal.intent.fee_minor > 0 {
            let fee_id = journal.effect_id(PaymentLeg::TreasuryFee);
            journal.prepare_leg(PaymentLeg::TreasuryFee, &fee_id).unwrap();
            commit_fee(&mut journal, "treasury", 2).unwrap();
        }
        journal
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
            "sha256:1111111111111111111111111111111111111111111111111111111111111111",
            u64::MAX,
            1,
            "SAP",
        );
        assert_eq!(registry.create_or_get(overflow), Err(SagaError::AmountOverflow));
        let zero = PaymentIntent::new(
            "alice",
            "bob",
            "op-zero",
            "sha256:2222222222222222222222222222222222222222222222222222222222222222",
            0,
            0,
            "SAP",
        );
        assert!(matches!(
            registry.create_or_get(zero),
            Err(SagaError::InvalidIntent(_))
        ));
        assert!(registry.is_empty());
    }

    #[test]
    fn treasury_fee_cannot_prepare_before_atomic_local_payment_commit() {
        let mut registry = JournalRegistry::default();
        registry.create_or_get(intent()).unwrap();
        let journal = registry.get_mut("did:mycelix:alice", "client-op-001").unwrap();
        journal.authorize().unwrap();
        let fee_id = journal.effect_id(PaymentLeg::TreasuryFee);
        assert_eq!(
            journal.prepare_leg(PaymentLeg::TreasuryFee, &fee_id),
            Err(SagaError::DependencyNotSatisfied(PaymentLeg::TreasuryFee))
        );
    }

    #[test]
    fn local_transaction_is_one_atomic_subject_not_independent_debit_and_credit_legs() {
        let journal = completed_payment();
        assert!(journal.local_payment_committed());
        assert!(matches!(
            journal.effect_state(PaymentLeg::LocalPaymentTransaction),
            EffectState::Committed {
                evidence: EffectEvidence {
                    kind: EffectEvidenceKind::LocalPaymentTransaction { .. },
                    ..
                },
                ..
            }
        ));
        assert_eq!(
            journal.effect_state(PaymentLeg::TreasuryFee),
            &EffectState::Committed {
                effect_id: journal.effect_id(PaymentLeg::TreasuryFee),
                evidence: treasury_fee_evidence(&journal, "treasury", 2)
            }
        );
        assert!(journal.is_completed());
    }

    #[test]
    fn local_payment_transaction_enforces_both_balances_and_receipt_subject() {
        let mut journal = ready_local_payment();
        // Once committed, duplicate identical evidence is idempotent.
        assert_eq!(
            commit_local(&mut journal, "local", 1).unwrap(),
            Transition::AlreadyCommitted
        );

        let mut wrong_debit = local_payment_evidence(&journal, "wrong-debit", 3);
        if let EffectEvidenceKind::LocalPaymentTransaction { sender_debit_minor, .. } =
            &mut wrong_debit.kind
        {
            *sender_debit_minor -= 1;
        }
        assert_eq!(
            journal.validate_effect_evidence(
                PaymentLeg::LocalPaymentTransaction,
                &wrong_debit,
            ),
            Err(SagaError::InvalidEffectEvidence(PaymentLeg::LocalPaymentTransaction))
        );

        let mut wrong_credit = local_payment_evidence(&journal, "wrong-credit", 4);
        if let EffectEvidenceKind::LocalPaymentTransaction {
            recipient_credit_minor,
            ..
        } = &mut wrong_credit.kind
        {
            *recipient_credit_minor += 1;
        }
        assert_eq!(
            journal.validate_effect_evidence(
                PaymentLeg::LocalPaymentTransaction,
                &wrong_credit,
            ),
            Err(SagaError::InvalidEffectEvidence(PaymentLeg::LocalPaymentTransaction))
        );

        let mut wrong_signer = local_payment_evidence(&journal, "wrong-signer", 5);
        if let EffectEvidenceKind::LocalPaymentTransaction {
            receipt_signer_did,
            ..
        } = &mut wrong_signer.kind
        {
            *receipt_signer_did = "did:mycelix:mallory".into();
        }
        assert_eq!(
            journal.validate_effect_evidence(
                PaymentLeg::LocalPaymentTransaction,
                &wrong_signer,
            ),
            Err(SagaError::InvalidEffectEvidence(PaymentLeg::LocalPaymentTransaction))
        );
    }

    #[test]
    fn local_transaction_rejects_non_completed_payment_and_missing_outbox_intent() {
        let mut journal = ready_local_payment();

        let mut pending_payment = local_payment_evidence(&journal, "pending-payment", 6);
        if let EffectEvidenceKind::LocalPaymentTransaction { payment_status, .. } =
            &mut pending_payment.kind
        {
            *payment_status = PaymentRecordStatus::Pending;
        }
        assert_eq!(
            journal.validate_effect_evidence(
                PaymentLeg::LocalPaymentTransaction,
                &pending_payment,
            ),
            Err(SagaError::InvalidEffectEvidence(PaymentLeg::LocalPaymentTransaction))
        );

        let mut failed_payment = local_payment_evidence(&journal, "failed-payment", 8);
        if let EffectEvidenceKind::LocalPaymentTransaction { payment_status, .. } =
            &mut failed_payment.kind
        {
            *payment_status = PaymentRecordStatus::Failed;
        }
        assert_eq!(
            journal.validate_effect_evidence(
                PaymentLeg::LocalPaymentTransaction,
                &failed_payment,
            ),
            Err(SagaError::InvalidEffectEvidence(
                PaymentLeg::LocalPaymentTransaction
            ))
        );

        let mut missing_outbox = local_payment_evidence(&journal, "missing-outbox", 7);
        if let EffectEvidenceKind::LocalPaymentTransaction {
            treasury_fee_outbox_id,
            ..
        } = &mut missing_outbox.kind
        {
            *treasury_fee_outbox_id = None;
        }
        assert_eq!(
            journal.validate_effect_evidence(
                PaymentLeg::LocalPaymentTransaction,
                &missing_outbox,
            ),
            Err(SagaError::InvalidEffectEvidence(PaymentLeg::LocalPaymentTransaction))
        );
    }

    #[test]
    fn local_transaction_requires_v1_receipt_signing_profile_and_digest() {
        let journal = ready_local_payment();
        let mut wrong_profile = local_payment_evidence(&journal, "wrong-profile", 9);
        if let EffectEvidenceKind::LocalPaymentTransaction { signing_profile, .. } =
            &mut wrong_profile.kind
        {
            *signing_profile = "legacy-unversioned".into();
        }
        assert_eq!(
            journal.validate_effect_evidence(
                PaymentLeg::LocalPaymentTransaction,
                &wrong_profile,
            ),
            Err(SagaError::InvalidEffectEvidence(
                PaymentLeg::LocalPaymentTransaction
            ))
        );

        let mut bad_signature_digest = local_payment_evidence(&journal, "bad-signature-digest", 10);
        if let EffectEvidenceKind::LocalPaymentTransaction { signature_digest, .. } =
            &mut bad_signature_digest.kind
        {
            *signature_digest = "not-a-digest".into();
        }
        assert_eq!(
            journal.validate_effect_evidence(
                PaymentLeg::LocalPaymentTransaction,
                &bad_signature_digest,
            ),
            Err(SagaError::InvalidEffectEvidence(
                PaymentLeg::LocalPaymentTransaction
            ))
        );
    }

    #[test]
    fn local_transaction_ack_loss_reconciles_same_atomic_effect_identity() {
        let mut registry = JournalRegistry::default();
        registry.create_or_get(intent()).unwrap();
        let journal = registry.get_mut("did:mycelix:alice", "client-op-001").unwrap();
        journal.authorize().unwrap();
        let local_id = journal.effect_id(PaymentLeg::LocalPaymentTransaction);
        journal
            .prepare_leg(PaymentLeg::LocalPaymentTransaction, &local_id)
            .unwrap();
        journal
            .mark_indeterminate(PaymentLeg::LocalPaymentTransaction, &local_id)
            .unwrap();
        assert_eq!(journal.status(), OperationStatus::Indeterminate);
        assert_eq!(
            journal
                .prepare_leg(PaymentLeg::LocalPaymentTransaction, &local_id)
                .unwrap(),
            Transition::ResumedSameEffect
        );
        commit_local(journal, "local-transaction", 8).unwrap();
        assert_eq!(journal.status(), OperationStatus::FeeDistributionPending);
        assert_eq!(
            journal
                .prepare_leg(
                    PaymentLeg::LocalPaymentTransaction,
                    &local_id
                )
                .unwrap(),
            Transition::AlreadyCommitted
        );
    }

    #[test]
    fn treasury_fee_delivery_is_blocked_until_local_transaction_commits() {
        let mut journal = ready_local_payment();
        let fee_id = journal.effect_id(PaymentLeg::TreasuryFee);
        journal.prepare_leg(PaymentLeg::TreasuryFee, &fee_id).unwrap();

        let mut wrong_pool = treasury_fee_evidence(&journal, "wrong-pool", 10);
        if let EffectEvidenceKind::TreasuryFee { pool_id, .. } = &mut wrong_pool.kind {
            *pool_id = "treasury:other-pool".into();
        }
        assert_eq!(
            journal.validate_effect_evidence(PaymentLeg::TreasuryFee, &wrong_pool),
            Err(SagaError::InvalidEffectEvidence(PaymentLeg::TreasuryFee))
        );

        let mut wrong_amount = treasury_fee_evidence(&journal, "wrong-amount", 11);
        if let EffectEvidenceKind::TreasuryFee { amount_minor, .. } = &mut wrong_amount.kind {
            *amount_minor += 1;
        }
        assert_eq!(
            journal.validate_effect_evidence(PaymentLeg::TreasuryFee, &wrong_amount),
            Err(SagaError::InvalidEffectEvidence(PaymentLeg::TreasuryFee))
        );

        let mut wrong_payment = treasury_fee_evidence(&journal, "wrong-payment", 12);
        if let EffectEvidenceKind::TreasuryFee {
            payment_action_hash,
            ..
        } = &mut wrong_payment.kind
        {
            *payment_action_hash = "action:some-other-payment".into();
        }
        assert_eq!(
            journal.validate_effect_evidence(PaymentLeg::TreasuryFee, &wrong_payment),
            Err(SagaError::InvalidEffectEvidence(PaymentLeg::TreasuryFee))
        );

        let mut wrong_key = treasury_fee_evidence(&journal, "wrong-key", 13);
        if let EffectEvidenceKind::TreasuryFee { idempotency_key, .. } = &mut wrong_key.kind {
            *idempotency_key = "another-operation".into();
        }
        assert_eq!(
            journal.validate_effect_evidence(PaymentLeg::TreasuryFee, &wrong_key),
            Err(SagaError::InvalidEffectEvidence(PaymentLeg::TreasuryFee))
        );
    }

    #[test]
    fn treasury_fee_timeout_retries_same_outbox_id_and_completes() {
        let mut journal = ready_local_payment();
        let fee_id = journal.effect_id(PaymentLeg::TreasuryFee);
        journal.prepare_leg(PaymentLeg::TreasuryFee, &fee_id).unwrap();
        journal.mark_indeterminate(PaymentLeg::TreasuryFee, &fee_id).unwrap();
        assert_eq!(journal.status(), OperationStatus::Indeterminate);
        assert_eq!(
            journal.prepare_leg(PaymentLeg::TreasuryFee, &fee_id).unwrap(),
            Transition::ResumedSameEffect
        );
        commit_fee(&mut journal, "treasury", 14).unwrap();
        assert_eq!(journal.status(), OperationStatus::Completed);
    }

    #[test]
    fn zero_fee_completes_with_no_treasury_effect() {
        let no_fee = PaymentIntent::new(
            "alice",
            "bob",
            "op-no-fee",
            "sha256:cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc",
            10,
            0,
            "SAP",
        );
        let mut registry = JournalRegistry::default();
        registry.create_or_get(no_fee).unwrap();
        let journal = registry.get_mut("alice", "op-no-fee").unwrap();
        journal.authorize().unwrap();
        let local_id = journal.effect_id(PaymentLeg::LocalPaymentTransaction);
        journal.prepare_leg(PaymentLeg::LocalPaymentTransaction, &local_id).unwrap();
        commit_local(journal, "no-fee", 15).unwrap();
        assert_eq!(
            journal.effect_state(PaymentLeg::TreasuryFee),
            &EffectState::NotRequired
        );
        assert_eq!(journal.status(), OperationStatus::Completed);
    }

    #[test]
    fn changed_evidence_for_same_treasury_effect_conflicts() {
        let mut journal = ready_local_payment();
        let fee_id = journal.effect_id(PaymentLeg::TreasuryFee);
        journal.prepare_leg(PaymentLeg::TreasuryFee, &fee_id).unwrap();
        commit_fee(&mut journal, "treasury", 16).unwrap();
        let changed = treasury_fee_evidence(&journal, "treasury", 17);
        assert_eq!(
            journal.commit_leg(PaymentLeg::TreasuryFee, changed),
            Err(SagaError::EffectEvidenceConflict(PaymentLeg::TreasuryFee))
        );
    }

    #[test]
    fn cannot_reject_after_local_transaction_or_treasury_effect_may_have_occurred() {
        let mut registry = JournalRegistry::default();
        registry.create_or_get(intent()).unwrap();
        let journal = registry.get_mut("did:mycelix:alice", "client-op-001").unwrap();
        journal.authorize().unwrap();
        let local_id = journal.effect_id(PaymentLeg::LocalPaymentTransaction);
        journal.prepare_leg(PaymentLeg::LocalPaymentTransaction, &local_id).unwrap();
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
        let outcomes: Vec<_> = workers
            .into_iter()
            .map(|worker| worker.join().unwrap())
            .collect();
        let ids: std::collections::BTreeSet<_> = outcomes
            .iter()
            .map(|(_, operation_id)| operation_id)
            .collect();
        assert_eq!(ids.len(), 1);
        assert_eq!(registry.lock().unwrap().len(), 1);
        assert_eq!(
            outcomes
                .iter()
                .filter(|(disposition, _)| *disposition == CreateDisposition::Created)
                .count(),
            1
        );
    }

    #[test]
    fn length_prefixed_operation_ids_do_not_alias_separator_collisions() {
        let first = PaymentIntent::new(
            "sender:one",
            "bob",
            "key",
            "sha256:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
            1,
            0,
            "SAP",
        );
        let second = PaymentIntent::new(
            "sender",
            "bob",
            "one:key",
            "sha256:bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb",
            1,
            0,
            "SAP",
        );
        let mut registry = JournalRegistry::default();
        let (_, first_id) = registry.create_or_get(first).unwrap();
        let (_, second_id) = registry.create_or_get(second).unwrap();
        assert_ne!(first_id, second_id);
        assert_eq!(registry.len(), 2);
    }

    #[test]
    fn noncanonical_evidence_digest_is_rejected() {
        let mut registry = JournalRegistry::default();
        registry.create_or_get(intent()).unwrap();
        let journal = registry.get_mut("did:mycelix:alice", "client-op-001").unwrap();
        journal.authorize().unwrap();
        let local_id = journal.effect_id(PaymentLeg::LocalPaymentTransaction);
        journal.prepare_leg(PaymentLeg::LocalPaymentTransaction, &local_id).unwrap();
        let mut evidence = local_payment_evidence(journal, "bad-digest", 20);
        evidence.evidence_digest = "not-a-digest".into();
        assert_eq!(
            journal.commit_leg(PaymentLeg::LocalPaymentTransaction, evidence),
            Err(SagaError::InvalidEffectEvidence(
                PaymentLeg::LocalPaymentTransaction
            ))
        );
    }
}

fn is_sha256_digest(value: &str) -> bool {
    value.strip_prefix("sha256:").is_some_and(|hex| {
        hex.len() == 64
            && hex
                .bytes()
                .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
    })
}

fn is_action_reference(value: &str) -> bool {
    value.strip_prefix("action:").is_some_and(|reference| {
        !reference.is_empty()
            && reference.bytes().all(|byte| {
                byte.is_ascii_alphanumeric() || byte == b'-' || byte == b'_'
            })
    })
}
