#![forbid(unsafe_code)]

//! Pure reference model for a journaled SAP payment operation.
//!
//! This crate is deliberately not wired into the production Finance zome.
//! It models ordering, stable per-leg effect identities, retry/reconciliation,
//! and the point at which a local ledger payment may be called Completed.
//! It does not prove Holochain DHT uniqueness, cross-agent atomicity, account
//! conservation, provider settlement, or external legal finality.

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
            || self.payload_fingerprint.is_empty()
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

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MovementDirection {
    Debit,
    Credit,
}

/// Typed evidence for one effect. The model checks that the evidence describes
/// the exact operation, leg, parties, unit and amount before recording commit.
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
pub enum PaymentRecordStatus {
    Completed,
    Pending,
    Failed,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum EffectEvidenceKind {
    MonetaryMovement {
        account_id: String,
        counterparty_id: String,
        direction: MovementDirection,
        amount_minor: u64,
        predecessor_action_hash: String,
        action_hash: String,
    },
    PaymentRecord {
        payment_action_hash: String,
        status: PaymentRecordStatus,
    },
    Receipt {
        receipt_action_hash: String,
        payment_action_hash: String,
        signer_did: String,
        signing_profile: String,
        signature_digest: String,
    },
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
    InvalidEffectEvidence(PaymentLeg),
}

impl fmt::Display for SagaError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidIntent(reason) => write!(formatter, "invalid payment intent: {reason}"),
            Self::AmountOverflow => {
                write!(formatter, "principal plus fee overflows the amount unit")
            }
            Self::IdempotencyConflict => write!(
                formatter,
                "idempotency key reused with a different immutable intent"
            ),
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

    fn committed_payment_action_hash(&self) -> Option<&str> {
        match self.effect_state(PaymentLeg::PaymentRecord) {
            EffectState::Committed { evidence, .. } => match &evidence.kind {
                EffectEvidenceKind::PaymentRecord { payment_action_hash } => {
                    Some(payment_action_hash)
                }
                _ => None,
            },
            _ => None,
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
                PaymentLeg::SenderDebit,
                EffectEvidenceKind::MonetaryMovement {
                    account_id,
                    counterparty_id,
                    direction,
                    amount_minor,
                    predecessor_action_hash,
                    action_hash,
                },
            ) => {
                account_id == &self.intent.sender_did
                    && counterparty_id == &self.intent.recipient_did
                    && *direction == MovementDirection::Debit
                    && *amount_minor == total_debit
                    && is_action_reference(predecessor_action_hash)
                    && is_action_reference(action_hash)
                    && predecessor_action_hash != action_hash
            }
            (
                PaymentLeg::RecipientCredit,
                EffectEvidenceKind::MonetaryMovement {
                    account_id,
                    counterparty_id,
                    direction,
                    amount_minor,
                    predecessor_action_hash,
                    action_hash,
                },
            ) => {
                account_id == &self.intent.recipient_did
                    && counterparty_id == &self.intent.sender_did
                    && *direction == MovementDirection::Credit
                    && *amount_minor == self.intent.amount_minor
                    && !predecessor_action_hash.is_empty()
                    && !action_hash.is_empty()
                    && predecessor_action_hash != action_hash
            }
            (
                PaymentLeg::TreasuryFee,
                EffectEvidenceKind::MonetaryMovement {
                    account_id,
                    counterparty_id,
                    direction,
                    amount_minor,
                    predecessor_action_hash,
                    action_hash,
                },
            ) => {
                account_id == "treasury:global-fee-pool"
                    && counterparty_id == &self.intent.sender_did
                    && *direction == MovementDirection::Credit
                    && *amount_minor == self.intent.fee_minor
                    && self.intent.fee_minor > 0
                    && !predecessor_action_hash.is_empty()
                    && !action_hash.is_empty()
                    && predecessor_action_hash != action_hash
            }
            (
                PaymentLeg::PaymentRecord,
                EffectEvidenceKind::PaymentRecord {
                    payment_action_hash,
                    status,
                },
            ) => {
                is_action_reference(payment_action_hash)
                    && *status == PaymentRecordStatus::Completed
            },
            (
                PaymentLeg::Receipt,
                EffectEvidenceKind::Receipt {
                    receipt_action_hash,
                    payment_action_hash,
                    signer_did,
                    signing_profile,
                    signature_digest,
                },
            ) => {
                is_action_reference(receipt_action_hash)
                    && is_action_reference(payment_action_hash)
                    && receipt_action_hash != payment_action_hash
                    && self.committed_payment_action_hash() == Some(payment_action_hash.as_str())
                    && signer_did == &self.intent.sender_did
                    && signing_profile == "mycelix.finance.payment-receipt.v1"
                    && is_sha256_digest(signature_digest)
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
        evidence: EffectEvidence,
    ) -> Result<Transition, SagaError> {
        if self.rejected {
            return Err(SagaError::Rejected);
        }
        if leg == PaymentLeg::TreasuryFee && self.intent.fee_minor == 0 {
            return Ok(Transition::NotRequired);
        }
        if !self.dependencies_satisfied(leg) {
            return Err(SagaError::DependencyNotSatisfied(leg));
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

    fn valid_evidence(
        journal: &PaymentJournal,
        leg: PaymentLeg,
        action_label: &str,
        digest_seed: u64,
    ) -> EffectEvidence {
        let kind = match leg {
            PaymentLeg::SenderDebit => EffectEvidenceKind::MonetaryMovement {
                account_id: journal.intent.sender_did.clone(),
                counterparty_id: journal.intent.recipient_did.clone(),
                direction: MovementDirection::Debit,
                amount_minor: journal.intent.amount_minor + journal.intent.fee_minor,
                predecessor_action_hash: format!("action:sender-before-{action_label}"),
                action_hash: format!("action:sender-debit-{action_label}"),
            },
            PaymentLeg::RecipientCredit => EffectEvidenceKind::MonetaryMovement {
                account_id: journal.intent.recipient_did.clone(),
                counterparty_id: journal.intent.sender_did.clone(),
                direction: MovementDirection::Credit,
                amount_minor: journal.intent.amount_minor,
                predecessor_action_hash: format!("action:recipient-before-{action_label}"),
                action_hash: format!("action:recipient-credit-{action_label}"),
            },
            PaymentLeg::TreasuryFee => EffectEvidenceKind::MonetaryMovement {
                account_id: "treasury:global-fee-pool".into(),
                counterparty_id: journal.intent.sender_did.clone(),
                direction: MovementDirection::Credit,
                amount_minor: journal.intent.fee_minor,
                predecessor_action_hash: format!("action:treasury-before-{action_label}"),
                action_hash: format!("action:treasury-fee-{action_label}"),
            },
            PaymentLeg::PaymentRecord => EffectEvidenceKind::PaymentRecord {
                payment_action_hash: format!("action:payment-record-{action_label}"),
                status: PaymentRecordStatus::Completed,
            },
            PaymentLeg::Receipt => EffectEvidenceKind::Receipt {
                receipt_action_hash: format!("action:receipt-{action_label}"),
                payment_action_hash: journal
                    .committed_payment_action_hash()
                    .unwrap_or("action:missing-payment-record")
                    .to_owned(),
                signer_did: journal.intent.sender_did.clone(),
                signing_profile: "mycelix.finance.payment-receipt.v1".into(),
                signature_digest: digest(digest_seed + 10_000),
            },
        };
        EffectEvidence {
            operation_id: journal.operation_id.clone(),
            effect_id: journal.effect_id(leg),
            leg,
            sender_did: journal.intent.sender_did.clone(),
            recipient_did: journal.intent.recipient_did.clone(),
            principal_minor: journal.intent.amount_minor,
            fee_minor: journal.intent.fee_minor,
            currency: journal.intent.currency.clone(),
            evidence_digest: digest(digest_seed),
            kind,
        }
    }

    fn commit_for_test(
        journal: &mut PaymentJournal,
        leg: PaymentLeg,
        label: &str,
        digest_seed: u64,
    ) -> Result<Transition, SagaError> {
        let evidence = valid_evidence(journal, leg, label, digest_seed);
        journal.commit_leg(leg, evidence)
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
        commit_for_test(journal, PaymentLeg::SenderDebit, "debit", 1).unwrap();
        let credit_id = journal.effect_id(PaymentLeg::RecipientCredit);
        journal
            .prepare_leg(PaymentLeg::RecipientCredit, &credit_id)
            .unwrap();
        commit_for_test(journal, PaymentLeg::RecipientCredit, "credit", 2).unwrap();
        let fee_id = journal.effect_id(PaymentLeg::TreasuryFee);
        journal
            .prepare_leg(PaymentLeg::TreasuryFee, &fee_id)
            .unwrap();
        commit_for_test(journal, PaymentLeg::TreasuryFee, "fee", 3).unwrap();
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
            commit_for_test(&mut journal, PaymentLeg::SenderDebit, "debit", 1).unwrap(),
            Transition::AlreadyCommitted
        );
        assert_eq!(
            journal.effect_state(PaymentLeg::SenderDebit),
            &EffectState::Committed {
                effect_id: debit_id.clone(),
                evidence: valid_evidence(&journal, PaymentLeg::SenderDebit, "debit", 1)
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
        commit_for_test(journal, PaymentLeg::SenderDebit, "debit-a", 4).unwrap();
        assert_eq!(
            {
                let evidence =
                    valid_evidence(journal, PaymentLeg::SenderDebit, "debit-b", 5);
                journal.commit_leg(PaymentLeg::SenderDebit, evidence)
            },
            Err(SagaError::EffectEvidenceConflict(PaymentLeg::SenderDebit))
        );
    }

    #[test]
    fn lost_credit_ack_is_indeterminate_and_resumes_original_credit_id() {
        let mut registry = JournalRegistry::default();
        registry.create_or_get(intent()).unwrap();
        let journal = registry.get_mut("did:mycelix:alice", "client-op-001").unwrap();
        journal.authorize().unwrap();

        let debit_id = journal.effect_id(PaymentLeg::SenderDebit);
        journal.prepare_leg(PaymentLeg::SenderDebit, &debit_id).unwrap();
        commit_for_test(journal, PaymentLeg::SenderDebit, "debit", 1).unwrap();

        let credit_id = journal.effect_id(PaymentLeg::RecipientCredit);
        journal.prepare_leg(PaymentLeg::RecipientCredit, &credit_id).unwrap();
        journal.mark_indeterminate(PaymentLeg::RecipientCredit, &credit_id).unwrap();
        assert_eq!(journal.status(), OperationStatus::Indeterminate);
        assert_eq!(
            journal.prepare_leg(PaymentLeg::RecipientCredit, &credit_id).unwrap(),
            Transition::ResumedSameEffect
        );
        commit_for_test(journal, PaymentLeg::RecipientCredit, "credit", 2).unwrap();
        assert!(matches!(
            journal.effect_state(PaymentLeg::SenderDebit),
            EffectState::Committed { .. }
        ));
    }

    #[test]
    fn fee_failure_blocks_payment_record_and_completion() {
        let mut registry = JournalRegistry::default();
        registry.create_or_get(intent()).unwrap();
        let journal = registry.get_mut("did:mycelix:alice", "client-op-001").unwrap();
        journal.authorize().unwrap();
        let debit = journal.effect_id(PaymentLeg::SenderDebit);
        journal.prepare_leg(PaymentLeg::SenderDebit, &debit).unwrap();
        commit_for_test(journal, PaymentLeg::SenderDebit, "debit", 1).unwrap();
        let credit = journal.effect_id(PaymentLeg::RecipientCredit);
        journal.prepare_leg(PaymentLeg::RecipientCredit, &credit).unwrap();
        commit_for_test(journal, PaymentLeg::RecipientCredit, "credit", 2).unwrap();
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
        commit_for_test(&mut journal, PaymentLeg::PaymentRecord, "payment", 4).unwrap();
        assert_eq!(journal.status(), OperationStatus::ReceiptEvidencePending);
        let receipt_id = journal.effect_id(PaymentLeg::Receipt);
        journal.prepare_leg(PaymentLeg::Receipt, &receipt_id).unwrap();
        commit_for_test(&mut journal, PaymentLeg::Receipt, "receipt", 5).unwrap();
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
    fn fee_timeout_resumes_same_effect_and_unblocks_payment_evidence() {
        let mut registry = JournalRegistry::default();
        registry.create_or_get(intent()).unwrap();
        let journal = registry.get_mut("did:mycelix:alice", "client-op-001").unwrap();
        journal.authorize().unwrap();

        let debit = journal.effect_id(PaymentLeg::SenderDebit);
        journal.prepare_leg(PaymentLeg::SenderDebit, &debit).unwrap();
        commit_for_test(journal, PaymentLeg::SenderDebit, "debit", 1).unwrap();

        let credit = journal.effect_id(PaymentLeg::RecipientCredit);
        journal.prepare_leg(PaymentLeg::RecipientCredit, &credit).unwrap();
        commit_for_test(journal, PaymentLeg::RecipientCredit, "credit", 2).unwrap();

        let fee = journal.effect_id(PaymentLeg::TreasuryFee);
        journal.prepare_leg(PaymentLeg::TreasuryFee, &fee).unwrap();
        journal.mark_indeterminate(PaymentLeg::TreasuryFee, &fee).unwrap();
        assert_eq!(journal.status(), OperationStatus::Indeterminate);
        assert_eq!(
            journal.prepare_leg(PaymentLeg::TreasuryFee, &fee).unwrap(),
            Transition::ResumedSameEffect
        );
        commit_for_test(journal, PaymentLeg::TreasuryFee, "fee", 3).unwrap();

        let payment = journal.effect_id(PaymentLeg::PaymentRecord);
        assert_eq!(journal.status(), OperationStatus::PaymentEvidencePending);
        assert_eq!(
            journal.prepare_leg(PaymentLeg::PaymentRecord, &payment).unwrap(),
            Transition::Prepared
        );
    }

    #[test]
    fn receipt_timeout_resumes_same_evidence_leg_and_then_completes() {
        let mut journal = ready_journal();
        let payment = journal.effect_id(PaymentLeg::PaymentRecord);
        journal.prepare_leg(PaymentLeg::PaymentRecord, &payment).unwrap();
        commit_for_test(&mut journal, PaymentLeg::PaymentRecord, "payment", 4).unwrap();

        let receipt = journal.effect_id(PaymentLeg::Receipt);
        journal.prepare_leg(PaymentLeg::Receipt, &receipt).unwrap();
        journal.mark_indeterminate(PaymentLeg::Receipt, &receipt).unwrap();
        assert_eq!(journal.status(), OperationStatus::Indeterminate);
        assert_eq!(
            journal.prepare_leg(PaymentLeg::Receipt, &receipt).unwrap(),
            Transition::ResumedSameEffect
        );
        commit_for_test(&mut journal, PaymentLeg::Receipt, "receipt", 5).unwrap();
        assert_eq!(journal.status(), OperationStatus::Completed);
    }

    #[test]
    fn evidence_must_match_exact_debit_credit_and_fee_amounts() {
        let mut registry = JournalRegistry::default();
        registry.create_or_get(intent()).unwrap();
        let journal = registry.get_mut("did:mycelix:alice", "client-op-001").unwrap();
        journal.authorize().unwrap();

        let debit = journal.effect_id(PaymentLeg::SenderDebit);
        journal.prepare_leg(PaymentLeg::SenderDebit, &debit).unwrap();
        let mut wrong_debit = valid_evidence(journal, PaymentLeg::SenderDebit, "bad-debit", 10);
        if let EffectEvidenceKind::MonetaryMovement { amount_minor, .. } = &mut wrong_debit.kind {
            *amount_minor -= 1;
        }
        assert_eq!(
            journal.commit_leg(PaymentLeg::SenderDebit, wrong_debit),
            Err(SagaError::InvalidEffectEvidence(PaymentLeg::SenderDebit))
        );
        commit_for_test(journal, PaymentLeg::SenderDebit, "debit", 1).unwrap();

        let credit = journal.effect_id(PaymentLeg::RecipientCredit);
        journal.prepare_leg(PaymentLeg::RecipientCredit, &credit).unwrap();
        let mut wrong_credit =
            valid_evidence(journal, PaymentLeg::RecipientCredit, "bad-credit", 11);
        if let EffectEvidenceKind::MonetaryMovement { account_id, .. } = &mut wrong_credit.kind {
            *account_id = "did:mycelix:mallory".into();
        }
        assert_eq!(
            journal.commit_leg(PaymentLeg::RecipientCredit, wrong_credit),
            Err(SagaError::InvalidEffectEvidence(PaymentLeg::RecipientCredit))
        );
        commit_for_test(journal, PaymentLeg::RecipientCredit, "credit", 2).unwrap();

        let fee = journal.effect_id(PaymentLeg::TreasuryFee);
        journal.prepare_leg(PaymentLeg::TreasuryFee, &fee).unwrap();
        let mut wrong_fee = valid_evidence(journal, PaymentLeg::TreasuryFee, "bad-fee", 12);
        if let EffectEvidenceKind::MonetaryMovement { counterparty_id, .. } = &mut wrong_fee.kind {
            *counterparty_id = "did:mycelix:mallory".into();
        }
        assert_eq!(
            journal.commit_leg(PaymentLeg::TreasuryFee, wrong_fee),
            Err(SagaError::InvalidEffectEvidence(PaymentLeg::TreasuryFee))
        );

        let mut wrong_fee_amount =
            valid_evidence(journal, PaymentLeg::TreasuryFee, "bad-fee-amount", 13);
        if let EffectEvidenceKind::MonetaryMovement { amount_minor, .. } =
            &mut wrong_fee_amount.kind
        {
            *amount_minor += 1;
        }
        assert_eq!(
            journal.commit_leg(PaymentLeg::TreasuryFee, wrong_fee_amount),
            Err(SagaError::InvalidEffectEvidence(PaymentLeg::TreasuryFee))
        );
    }

    #[test]
    fn evidence_must_match_intent_currency_and_stable_operation_identity() {
        let mut registry = JournalRegistry::default();
        registry.create_or_get(intent()).unwrap();
        let journal = registry.get_mut("did:mycelix:alice", "client-op-001").unwrap();
        journal.authorize().unwrap();
        let debit = journal.effect_id(PaymentLeg::SenderDebit);
        journal.prepare_leg(PaymentLeg::SenderDebit, &debit).unwrap();

        let mut wrong_currency =
            valid_evidence(journal, PaymentLeg::SenderDebit, "wrong-currency", 50);
        wrong_currency.currency = "USD".into();
        assert_eq!(
            journal.commit_leg(PaymentLeg::SenderDebit, wrong_currency),
            Err(SagaError::InvalidEffectEvidence(PaymentLeg::SenderDebit))
        );

        let mut wrong_operation =
            valid_evidence(journal, PaymentLeg::SenderDebit, "wrong-operation", 51);
        wrong_operation.operation_id = "sap-payment:other-operation".into();
        assert_eq!(
            journal.commit_leg(PaymentLeg::SenderDebit, wrong_operation),
            Err(SagaError::InvalidEffectEvidence(PaymentLeg::SenderDebit))
        );

        let mut wrong_direction =
            valid_evidence(journal, PaymentLeg::SenderDebit, "wrong-direction", 52);
        if let EffectEvidenceKind::MonetaryMovement { direction, .. } =
            &mut wrong_direction.kind
        {
            *direction = MovementDirection::Credit;
        }
        assert_eq!(
            journal.commit_leg(PaymentLeg::SenderDebit, wrong_direction),
            Err(SagaError::InvalidEffectEvidence(PaymentLeg::SenderDebit))
        );
    }

    #[test]
    fn payment_record_must_be_completed_and_match_the_journaled_intent() {
        let mut journal = ready_journal();
        let payment = journal.effect_id(PaymentLeg::PaymentRecord);
        journal.prepare_leg(PaymentLeg::PaymentRecord, &payment).unwrap();

        let mut pending =
            valid_evidence(&journal, PaymentLeg::PaymentRecord, "payment-pending", 40);
        if let EffectEvidenceKind::PaymentRecord { status, .. } = &mut pending.kind {
            *status = PaymentRecordStatus::Pending;
        }
        assert_eq!(
            journal.commit_leg(PaymentLeg::PaymentRecord, pending),
            Err(SagaError::InvalidEffectEvidence(PaymentLeg::PaymentRecord))
        );

        let mut failed = valid_evidence(&journal, PaymentLeg::PaymentRecord, "payment-failed", 41);
        if let EffectEvidenceKind::PaymentRecord { status, .. } = &mut failed.kind {
            *status = PaymentRecordStatus::Failed;
        }
        assert_eq!(
            journal.commit_leg(PaymentLeg::PaymentRecord, failed),
            Err(SagaError::InvalidEffectEvidence(PaymentLeg::PaymentRecord))
        );

        commit_for_test(&mut journal, PaymentLeg::PaymentRecord, "payment", 4).unwrap();
    }

    #[test]
    fn receipt_evidence_must_bind_exact_committed_payment_action_and_v1_signature() {
        let mut journal = ready_journal();
        let payment = journal.effect_id(PaymentLeg::PaymentRecord);
        journal.prepare_leg(PaymentLeg::PaymentRecord, &payment).unwrap();
        commit_for_test(&mut journal, PaymentLeg::PaymentRecord, "payment", 4).unwrap();

        let receipt = journal.effect_id(PaymentLeg::Receipt);
        journal.prepare_leg(PaymentLeg::Receipt, &receipt).unwrap();
        let mut bad_receipt = valid_evidence(&journal, PaymentLeg::Receipt, "receipt-bad", 20);
        if let EffectEvidenceKind::Receipt { payment_action_hash, .. } = &mut bad_receipt.kind {
            *payment_action_hash = "action:some-other-payment".into();
        }
        assert_eq!(
            journal.commit_leg(PaymentLeg::Receipt, bad_receipt),
            Err(SagaError::InvalidEffectEvidence(PaymentLeg::Receipt))
        );

        let mut bad_profile =
            valid_evidence(&journal, PaymentLeg::Receipt, "receipt-bad-profile", 21);
        if let EffectEvidenceKind::Receipt { signing_profile, .. } = &mut bad_profile.kind {
            *signing_profile = "legacy-unversioned".into();
        }
        assert_eq!(
            journal.commit_leg(PaymentLeg::Receipt, bad_profile),
            Err(SagaError::InvalidEffectEvidence(PaymentLeg::Receipt))
        );
        commit_for_test(&mut journal, PaymentLeg::Receipt, "receipt", 5).unwrap();
        assert!(journal.is_completed());
    }

    #[test]
    fn receipt_evidence_rejects_wrong_signer_and_reused_payment_action() {
        let mut journal = ready_journal();
        let payment = journal.effect_id(PaymentLeg::PaymentRecord);
        journal.prepare_leg(PaymentLeg::PaymentRecord, &payment).unwrap();
        commit_for_test(&mut journal, PaymentLeg::PaymentRecord, "payment", 4).unwrap();

        let receipt = journal.effect_id(PaymentLeg::Receipt);
        journal.prepare_leg(PaymentLeg::Receipt, &receipt).unwrap();

        let mut wrong_signer =
            valid_evidence(&journal, PaymentLeg::Receipt, "wrong-signer", 60);
        if let EffectEvidenceKind::Receipt { signer_did, .. } = &mut wrong_signer.kind {
            *signer_did = "did:mycelix:mallory".into();
        }
        assert_eq!(
            journal.commit_leg(PaymentLeg::Receipt, wrong_signer),
            Err(SagaError::InvalidEffectEvidence(PaymentLeg::Receipt))
        );

        let mut reused_action =
            valid_evidence(&journal, PaymentLeg::Receipt, "reused-payment-action", 61);
        if let EffectEvidenceKind::Receipt {
            receipt_action_hash,
            payment_action_hash,
            ..
        } = &mut reused_action.kind
        {
            *receipt_action_hash = payment_action_hash.clone();
        }
        assert_eq!(
            journal.commit_leg(PaymentLeg::Receipt, reused_action),
            Err(SagaError::InvalidEffectEvidence(PaymentLeg::Receipt))
        );

        commit_for_test(&mut journal, PaymentLeg::Receipt, "receipt", 5).unwrap();
    }

    #[test]
    fn noncanonical_evidence_digest_is_rejected() {
        let mut registry = JournalRegistry::default();
        registry.create_or_get(intent()).unwrap();
        let journal = registry.get_mut("did:mycelix:alice", "client-op-001").unwrap();
        journal.authorize().unwrap();
        let debit = journal.effect_id(PaymentLeg::SenderDebit);
        journal.prepare_leg(PaymentLeg::SenderDebit, &debit).unwrap();
        let mut evidence = valid_evidence(journal, PaymentLeg::SenderDebit, "bad-digest", 30);
        evidence.evidence_digest = "not-a-digest".into();
        assert_eq!(
            journal.commit_leg(PaymentLeg::SenderDebit, evidence),
            Err(SagaError::InvalidEffectEvidence(PaymentLeg::SenderDebit))
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
        for leg in [
            PaymentLeg::SenderDebit,
            PaymentLeg::RecipientCredit,
            PaymentLeg::TreasuryFee,
        ] {
            assert!(
                matches!(journal.effect_state(leg), EffectState::Committed { .. }),
                "previous monetary leg {leg:?} must remain committed"
            );
        }
    }
}
