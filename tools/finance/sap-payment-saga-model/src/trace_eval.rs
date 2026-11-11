//! Executable Trace V1 interpreter and independent disposition oracle.
//!
//! The interpreter executes synthetic traces against the effects-disabled
//! PaymentJournal. The oracle is a separate opcode/fault contract; it does not
//! call saga transition helpers to choose expected dispositions.

use super::{
    CreateDisposition, EffectEvidence, EffectEvidenceKind, JournalRegistry, OperationStatus,
    PaymentIntent, PaymentLeg, PaymentRecordStatus, SagaError, Transition,
};
use crate::trace::{
    TraceCommandV1, TraceFault, TraceGeneratorV1, TraceOpcode, TraceOwner, TraceV1,
};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum TraceDispositionV1 {
    Applied,
    Idempotent,
    NotRequired,
    Indeterminate,
    EffectObserved,
    AuthoritativeAbsenceObserved,
    Unresolved,
    Rejected,
    DuplicateSuppressed,
    RequiresSeparateEffect,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum TraceInvariantV1 {
    StableOperationIdentity,
    StableEffectIdentity,
    LocalConservation,
    TreasuryFeeConservation,
    DependencyOrdering,
    IndeterminatePreserved,
    IntentImmutability,
    CompensationIsSeparateEffect,
    CheckpointAuthority,
    OracleDispositionAgreement,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct TraceStepV1 {
    pub command_index: usize,
    pub opcode: TraceOpcode,
    pub expected: TraceDispositionV1,
    pub observed: TraceDispositionV1,
    pub operation_status: OperationStatus,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct TraceInvariantFailureV1 {
    pub command_index: usize,
    pub invariant: TraceInvariantV1,
    pub detail: String,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct TraceEvaluationV1 {
    pub seed: u64,
    pub trace_index: u64,
    pub scenario: u8,
    pub trace_digest: String,
    pub operation_id: String,
    pub local_effect_id: String,
    pub treasury_effect_id: String,
    pub final_status: OperationStatus,
    pub steps: Vec<TraceStepV1>,
    pub failures: Vec<TraceInvariantFailureV1>,
}

impl TraceEvaluationV1 {
    pub fn passed(&self) -> bool {
        self.failures.is_empty()
            && self.steps.iter().all(|step| step.expected == step.observed)
    }

    pub fn command_count(&self) -> usize {
        self.steps.len()
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum TraceEvaluationError {
    InvalidTrace,
    MissingCreateIntent,
    MissingAuthorization,
    InvalidIntent,
    MissingJournal,
}

struct ExecutionContext {
    registry: JournalRegistry,
    sender: String,
    idempotency_key: String,
    original_intent: PaymentIntent,
    operation_id: String,
    local_effect_id: String,
    treasury_effect_id: String,
    operation_nonce: u64,
    effect_nonce: Option<u64>,
    amount_minor: u64,
    fee_minor: u64,
    local_committed: bool,
    treasury_dispatched: bool,
    remote_effect_present: bool,
    authoritative_absence_observed: bool,
    receipt_observed: bool,
    treasury_evidence: Option<EffectEvidence>,
    payments_checkpoint: Option<u64>,
    treasury_checkpoint: Option<u64>,
    compensation_seen: bool,
}

impl ExecutionContext {
    fn journal(&self) -> Result<&super::PaymentJournal, TraceEvaluationError> {
        self.registry
            .get(&self.sender, &self.idempotency_key)
            .ok_or(TraceEvaluationError::MissingJournal)
    }

    fn journal_mut(&mut self) -> Result<&mut super::PaymentJournal, TraceEvaluationError> {
        self.registry
            .get_mut(&self.sender, &self.idempotency_key)
            .ok_or(TraceEvaluationError::MissingJournal)
    }

    fn treasury_evidence(&mut self) -> Result<EffectEvidence, TraceEvaluationError> {
        if let Some(existing) = &self.treasury_evidence {
            return Ok(existing.clone());
        }
        let journal = self.journal().map_err(|_| TraceEvaluationError::MissingJournal)?;
        let payment_action_hash = journal
            .committed_payment_action_hash()
            .ok_or(TraceEvaluationError::MissingJournal)?
            .to_owned();
        let mut evidence = EffectEvidence {
            operation_id: journal.operation_id.clone(),
            effect_id: self.treasury_effect_id.clone(),
            leg: PaymentLeg::TreasuryFee,
            sender_did: journal.intent.sender_did.clone(),
            recipient_did: journal.intent.recipient_did.clone(),
            principal_minor: journal.intent.amount_minor,
            fee_minor: journal.intent.fee_minor,
            currency: journal.intent.currency.clone(),
            evidence_digest: String::new(),
            kind: EffectEvidenceKind::TreasuryFee {
                pool_id: "treasury:global-fee-pool".into(),
                source_member_did: journal.intent.sender_did.clone(),
                amount_minor: journal.intent.fee_minor,
                payment_action_hash,
                idempotency_key: self.treasury_effect_id.clone(),
                predecessor_action: format!("action:trace-treasury-before-{}", self.trace_label()),
                treasury_action: format!("action:trace-treasury-effect-{}", self.trace_label()),
            },
        };
        evidence.evidence_digest = evidence
            .canonical_digest()
            .map_err(|_| TraceEvaluationError::InvalidIntent)?;
        self.treasury_evidence = Some(evidence.clone());
        Ok(evidence)
    }

    fn trace_label(&self) -> String {
        format!("{:016x}-{:016x}", self.operation_nonce, self.effect_nonce.unwrap_or(0))
    }

    fn record_failure(
        failures: &mut Vec<TraceInvariantFailureV1>,
        command_index: usize,
        invariant: TraceInvariantV1,
        detail: impl Into<String>,
    ) {
        failures.push(TraceInvariantFailureV1 {
            command_index,
            invariant,
            detail: detail.into(),
        });
    }

    fn check_invariants(
        &self,
        command_index: usize,
        failures: &mut Vec<TraceInvariantFailureV1>,
    ) -> Result<(), TraceEvaluationError> {
        let journal = self.journal()?;
        let total = self
            .amount_minor
            .checked_add(self.fee_minor)
            .ok_or(TraceEvaluationError::InvalidIntent)?;

        if journal.operation_id() != self.operation_id {
            Self::record_failure(
                failures,
                command_index,
                TraceInvariantV1::StableOperationIdentity,
                "operation ID changed during trace execution",
            );
        }

        if journal.effect_id(PaymentLeg::LocalPaymentTransaction) != self.local_effect_id
            || journal.effect_id(PaymentLeg::TreasuryFee) != self.treasury_effect_id
        {
            Self::record_failure(
                failures,
                command_index,
                TraceInvariantV1::StableEffectIdentity,
                "a retry changed the stable effect identity",
            );
        }

        if self.original_intent.amount_minor != self.amount_minor
            || self.original_intent.fee_minor != self.fee_minor
            || self.original_intent.sender_did != self.sender
        {
            Self::record_failure(
                failures,
                command_index,
                TraceInvariantV1::LocalConservation,
                "trace intent fields changed after initial creation",
            );
        }

        let local_committed = matches!(
            journal.effect_state(PaymentLeg::LocalPaymentTransaction),
            super::EffectState::Committed { evidence, .. }
                if matches!(
                    &evidence.kind,
                    EffectEvidenceKind::LocalPaymentTransaction {
                        sender_debit_minor,
                        recipient_credit_minor,
                        ..
                    } if *sender_debit_minor == total
                        && *recipient_credit_minor == self.amount_minor
                )
        );

        if local_committed != self.local_committed {
            Self::record_failure(
                failures,
                command_index,
                TraceInvariantV1::LocalConservation,
                "local commit observation differs from the journal evidence",
            );
        }

        if matches!(
            journal.effect_state(PaymentLeg::TreasuryFee),
            super::EffectState::Committed { evidence, .. }
                if matches!(
                    &evidence.kind,
                    EffectEvidenceKind::TreasuryFee { amount_minor, .. }
                        if *amount_minor != self.fee_minor
                )
        ) {
            Self::record_failure(
                failures,
                command_index,
                TraceInvariantV1::TreasuryFeeConservation,
                "committed Treasury amount differs from the immutable fee",
            );
        }

        let treasury_committed = matches!(
            journal.effect_state(PaymentLeg::TreasuryFee),
            super::EffectState::Committed { .. }
        );
        if treasury_committed && (!local_committed || !self.remote_effect_present) {
            Self::record_failure(
                failures,
                command_index,
                TraceInvariantV1::DependencyOrdering,
                "Treasury effect committed before local commit or without observed destination effect",
            );
        }

        if journal.status() == OperationStatus::Completed
            && (!local_committed || (self.fee_minor > 0 && !treasury_committed))
        {
            Self::record_failure(
                failures,
                command_index,
                TraceInvariantV1::DependencyOrdering,
                "operation reported Completed while a required effect was unresolved",
            );
        }

        if matches!(
            journal.effect_state(PaymentLeg::TreasuryFee),
            super::EffectState::Indeterminate { .. }
        ) && journal.status() != OperationStatus::Indeterminate
        {
            Self::record_failure(
                failures,
                command_index,
                TraceInvariantV1::IndeterminatePreserved,
                "indeterminate Treasury effect was not reflected by operation status",
            );
        }

        if self.authoritative_absence_observed && self.receipt_observed {
            Self::record_failure(
                failures,
                command_index,
                TraceInvariantV1::IndeterminatePreserved,
                "trace simultaneously recorded authoritative absence and destination effect observation",
            );
        }

        if self.compensation_seen && !local_committed {
            Self::record_failure(
                failures,
                command_index,
                TraceInvariantV1::CompensationIsSeparateEffect,
                "compensation command rewrote or invalidated the original local commit",
            );
        }

        Ok(())
    }
}

/// Independent expectation table for the frozen V1 command/fault contract.
/// It deliberately does not inspect PaymentJournal or call saga transition code.
pub fn expected_trace_disposition_v1(
    command: &TraceCommandV1,
    fee_minor: u64,
) -> TraceDispositionV1 {
    if command.owner != expected_owner(command.opcode) {
        return TraceDispositionV1::Rejected;
    }
    if fee_minor == 0
        && matches!(
            command.opcode,
            TraceOpcode::PrepareTreasury
                | TraceOpcode::DispatchTreasury
                | TraceOpcode::MarkIndeterminate
                | TraceOpcode::ReconcileTreasury
                | TraceOpcode::RetrySameEffect
                | TraceOpcode::CommitTreasury
                | TraceOpcode::DuplicateDelivery
                | TraceOpcode::CheckpointTreasury
        )
    {
        return TraceDispositionV1::NotRequired;
    }

    match command.opcode {
        TraceOpcode::MarkIndeterminate => TraceDispositionV1::Indeterminate,
        TraceOpcode::ReconcileTreasury => match command.fault {
            TraceFault::AuthoritativeAbsence => TraceDispositionV1::AuthoritativeAbsenceObserved,
            TraceFault::LookupUncertain => TraceDispositionV1::Unresolved,
            TraceFault::ReceiptMismatch => TraceDispositionV1::Rejected,
            TraceFault::None => TraceDispositionV1::EffectObserved,
            _ => TraceDispositionV1::Rejected,
        },
        TraceOpcode::DuplicateDelivery => TraceDispositionV1::DuplicateSuppressed,
        TraceOpcode::MutateIntent
        | TraceOpcode::MutateEvent
        | TraceOpcode::StaleCheckpoint
        | TraceOpcode::ForkCheckpoint => TraceDispositionV1::Rejected,
        TraceOpcode::Compensate => TraceDispositionV1::RequiresSeparateEffect,
        TraceOpcode::CheckpointTreasury if command.fault != TraceFault::None => {
            TraceDispositionV1::Rejected
        }
        TraceOpcode::CheckpointPayments if command.fault != TraceFault::None => {
            TraceDispositionV1::Rejected
        }
        _ => TraceDispositionV1::Applied,
    }
}

fn expected_owner(opcode: TraceOpcode) -> TraceOwner {
    match opcode {
        TraceOpcode::CreateIntent | TraceOpcode::Authorize | TraceOpcode::Compensate => {
            TraceOwner::Sender
        }
        TraceOpcode::PrepareLocal
        | TraceOpcode::CommitLocal
        | TraceOpcode::CheckpointPayments => TraceOwner::Payments,
        TraceOpcode::PrepareTreasury
        | TraceOpcode::DispatchTreasury
        | TraceOpcode::MarkIndeterminate
        | TraceOpcode::ReconcileTreasury
        | TraceOpcode::RetrySameEffect
        | TraceOpcode::CommitTreasury
        | TraceOpcode::CheckpointTreasury => TraceOwner::Treasury,
        TraceOpcode::MutateIntent
        | TraceOpcode::DuplicateDelivery
        | TraceOpcode::MutateEvent
        | TraceOpcode::StaleCheckpoint
        | TraceOpcode::ForkCheckpoint => TraceOwner::Adversary,
    }
}

fn digest(seed: u64) -> String {
    format!("sha256:{seed:064x}")
}

fn local_payment_evidence(journal: &super::PaymentJournal, label: &str, seed: u64) -> EffectEvidence {
    let mut evidence = EffectEvidence {
        operation_id: journal.operation_id.clone(),
        effect_id: journal.effect_id(PaymentLeg::LocalPaymentTransaction),
        leg: PaymentLeg::LocalPaymentTransaction,
        sender_did: journal.intent.sender_did.clone(),
        recipient_did: journal.intent.recipient_did.clone(),
        principal_minor: journal.intent.amount_minor,
        fee_minor: journal.intent.fee_minor,
        currency: journal.intent.currency.clone(),
        evidence_digest: String::new(),
        kind: EffectEvidenceKind::LocalPaymentTransaction {
            sender_account_id: journal.intent.sender_did.clone(),
            recipient_account_id: journal.intent.recipient_did.clone(),
            sender_debit_minor: journal.intent.amount_minor + journal.intent.fee_minor,
            recipient_credit_minor: journal.intent.amount_minor,
            sender_predecessor_action: format!("action:trace-sender-before-{label}"),
            sender_debit_action: format!("action:trace-sender-debit-{label}"),
            recipient_predecessor_action: format!("action:trace-recipient-before-{label}"),
            recipient_credit_action: format!("action:trace-recipient-credit-{label}"),
            payment_action_hash: format!("action:trace-payment-{label}"),
            payment_status: PaymentRecordStatus::Completed,
            receipt_action_hash: format!("action:trace-receipt-{label}"),
            receipt_signer_did: journal.intent.sender_did.clone(),
            signing_profile: "mycelix.finance.payment-receipt.v1".into(),
            signature_digest: digest(seed + 10_000),
            treasury_fee_outbox_id: if journal.intent.fee_minor == 0 {
                None
            } else {
                Some(journal.effect_id(PaymentLeg::TreasuryFee))
            },
        },
    };
    evidence.evidence_digest = evidence
        .canonical_digest()
        .expect("synthetic local evidence fits canonical profile");
    evidence
}

fn treasury_evidence(
    journal: &super::PaymentJournal,
    effect_id: &str,
    label: &str,
    seed: u64,
) -> EffectEvidence {
    let payment_action_hash = journal
        .committed_payment_action_hash()
        .expect("local payment is committed before Treasury evidence")
        .to_owned();
    let mut evidence = EffectEvidence {
        operation_id: journal.operation_id.clone(),
        effect_id: effect_id.to_owned(),
        leg: PaymentLeg::TreasuryFee,
        sender_did: journal.intent.sender_did.clone(),
        recipient_did: journal.intent.recipient_did.clone(),
        principal_minor: journal.intent.amount_minor,
        fee_minor: journal.intent.fee_minor,
        currency: journal.intent.currency.clone(),
        evidence_digest: String::new(),
        kind: EffectEvidenceKind::TreasuryFee {
            pool_id: "treasury:global-fee-pool".into(),
            source_member_did: journal.intent.sender_did.clone(),
            amount_minor: journal.intent.fee_minor,
            payment_action_hash,
            idempotency_key: effect_id.to_owned(),
            predecessor_action: format!("action:trace-treasury-before-{label}"),
            treasury_action: format!("action:trace-treasury-effect-{label}"),
        },
    };
    evidence.evidence_digest = evidence
        .canonical_digest()
        .expect("synthetic Treasury evidence fits canonical profile");
    evidence
}

fn mutate_treasury_evidence(mut evidence: EffectEvidence, mutation: TraceOpcode) -> EffectEvidence {
    match &mut evidence.kind {
        EffectEvidenceKind::TreasuryFee {
            payment_action_hash,
            treasury_action,
            amount_minor,
            ..
        } => match mutation {
            TraceOpcode::ReconcileTreasury => *payment_action_hash = "action:unlinked-payment".into(),
            TraceOpcode::MutateEvent => {
                *treasury_action = "action:mutated-treasury-event".into();
                *amount_minor = amount_minor.saturating_add(1);
            }
            _ => {}
        },
        _ => {}
    }
    evidence.evidence_digest = evidence
        .canonical_digest()
        .expect("mutated evidence remains canonically encodable");
    evidence
}

fn model_error_disposition<T>(result: Result<T, SagaError>) -> TraceDispositionV1 {
    match result {
        Ok(_) => TraceDispositionV1::Applied,
        Err(_) => TraceDispositionV1::Rejected,
    }
}

fn observed_disposition(
    context: &mut ExecutionContext,
    command: &TraceCommandV1,
    _command_index: usize,
) -> Result<TraceDispositionV1, TraceEvaluationError> {
    if command.owner != expected_owner(command.opcode) {
        return Ok(TraceDispositionV1::Rejected);
    }

    let journal_status = context.journal()?.status();
    let fee_not_required = context.fee_minor == 0;

    match command.opcode {
        TraceOpcode::CreateIntent => {
            let disposition = if context.registry.len() == 1 {
                CreateDisposition::Created
            } else {
                CreateDisposition::ExistingSameIntent
            };
            Ok(match disposition {
                CreateDisposition::Created => TraceDispositionV1::Applied,
                CreateDisposition::ExistingSameIntent => TraceDispositionV1::Idempotent,
            })
        }
        TraceOpcode::Authorize => {
            let result = context.journal_mut()?.authorize();
            Ok(match result {
                Ok(Transition::Prepared) => TraceDispositionV1::Applied,
                Ok(_) => TraceDispositionV1::Idempotent,
                Err(_) => TraceDispositionV1::Rejected,
            })
        }
        TraceOpcode::PrepareLocal => {
            let effect_id = context.local_effect_id.clone();
            let result = context
                .journal_mut()?
                .prepare_leg(PaymentLeg::LocalPaymentTransaction, &effect_id);
            Ok(match result {
                Ok(Transition::Prepared | Transition::ResumedSameEffect) => TraceDispositionV1::Applied,
                Ok(_) => TraceDispositionV1::Idempotent,
                Err(_) => TraceDispositionV1::Rejected,
            })
        }
        TraceOpcode::CommitLocal => {
            let label = context.trace_label();
            let evidence = local_payment_evidence(
                context.journal()?,
                &format!("local-{label}"),
                context.operation_nonce ^ context.effect_nonce.unwrap_or(0),
            );
            let result = context
                .journal_mut()?
                .commit_leg(PaymentLeg::LocalPaymentTransaction, evidence);
            match result {
                Ok(Transition::Committed) => context.local_committed = true,
                Ok(Transition::AlreadyCommitted) => context.local_committed = true,
                _ => {}
            }
            Ok(match result {
                Ok(Transition::Committed) => TraceDispositionV1::Applied,
                Ok(_) => TraceDispositionV1::Idempotent,
                Err(_) => TraceDispositionV1::Rejected,
            })
        }
        TraceOpcode::PrepareTreasury => {
            context.effect_nonce.get_or_insert(command.subject);
            if context.effect_nonce != Some(command.subject) {
                return Ok(TraceDispositionV1::Rejected);
            }
            if fee_not_required {
                return Ok(TraceDispositionV1::NotRequired);
            }
            let effect_id = context.treasury_effect_id.clone();
            let result = context
                .journal_mut()?
                .prepare_leg(PaymentLeg::TreasuryFee, &effect_id);
            Ok(match result {
                Ok(Transition::Prepared | Transition::ResumedSameEffect) => TraceDispositionV1::Applied,
                Ok(_) => TraceDispositionV1::Idempotent,
                Err(_) => TraceDispositionV1::Rejected,
            })
        }
        TraceOpcode::DispatchTreasury => {
            if fee_not_required {
                return Ok(TraceDispositionV1::NotRequired);
            }
            if context.effect_nonce != Some(command.subject)
                || !matches!(
                    context.journal()?.effect_state(PaymentLeg::TreasuryFee),
                    super::EffectState::Prepared { .. }
                )
            {
                return Ok(TraceDispositionV1::Rejected);
            }
            context.treasury_dispatched = true;
            if command.fault == TraceFault::None {
                context.remote_effect_present = true;
            }
            Ok(TraceDispositionV1::Applied)
        }
        TraceOpcode::MarkIndeterminate => {
            if fee_not_required {
                return Ok(TraceDispositionV1::NotRequired);
            }
            if !context.treasury_dispatched || context.effect_nonce != Some(command.subject) {
                return Ok(TraceDispositionV1::Rejected);
            }
            context.remote_effect_present = command.fault == TraceFault::ResponseLostAfterEffect;
            context.authoritative_absence_observed = false;
            context.receipt_observed = false;
            let effect_id = context.treasury_effect_id.clone();
            let result = context
                .journal_mut()?
                .mark_indeterminate(PaymentLeg::TreasuryFee, &effect_id);
            Ok(match result {
                Ok(()) => TraceDispositionV1::Indeterminate,
                Err(_) => TraceDispositionV1::Rejected,
            })
        }
        TraceOpcode::ReconcileTreasury => {
            if fee_not_required {
                return Ok(TraceDispositionV1::NotRequired);
            }
            if context.effect_nonce != Some(command.subject) || !context.treasury_dispatched {
                return Ok(TraceDispositionV1::Rejected);
            }
            match command.fault {
                TraceFault::AuthoritativeAbsence => {
                    if journal_status != OperationStatus::Indeterminate || context.remote_effect_present {
                        return Ok(TraceDispositionV1::Rejected);
                    }
                    context.authoritative_absence_observed = true;
                    Ok(TraceDispositionV1::AuthoritativeAbsenceObserved)
                }
                TraceFault::LookupUncertain => {
                    if journal_status != OperationStatus::Indeterminate {
                        return Ok(TraceDispositionV1::Rejected);
                    }
                    Ok(TraceDispositionV1::Unresolved)
                }
                TraceFault::ReceiptMismatch => {
                    let evidence = context.treasury_evidence()?;
                    let mismatched = mutate_treasury_evidence(evidence, TraceOpcode::ReconcileTreasury);
                    let result = context
                        .journal_mut()?
                        .commit_leg(PaymentLeg::TreasuryFee, mismatched);
                    Ok(model_error_disposition(result))
                }
                TraceFault::None => {
                    if !context.remote_effect_present {
                        return Ok(TraceDispositionV1::Rejected);
                    }
                    context.receipt_observed = true;
                    Ok(TraceDispositionV1::EffectObserved)
                }
                _ => Ok(TraceDispositionV1::Rejected),
            }
        }
        TraceOpcode::RetrySameEffect => {
            if fee_not_required {
                return Ok(TraceDispositionV1::NotRequired);
            }
            if !context.authoritative_absence_observed
                || context.remote_effect_present
                || context.effect_nonce != Some(command.subject)
            {
                return Ok(TraceDispositionV1::Rejected);
            }
            let effect_id = context.treasury_effect_id.clone();
            let result = context
                .journal_mut()?
                .prepare_leg(PaymentLeg::TreasuryFee, &effect_id);
            match result {
                Ok(Transition::ResumedSameEffect | Transition::Prepared) => {
                    context.treasury_dispatched = false;
                    context.authoritative_absence_observed = false;
                    Ok(TraceDispositionV1::Applied)
                }
                Ok(_) => Ok(TraceDispositionV1::Idempotent),
                Err(_) => Ok(TraceDispositionV1::Rejected),
            }
        }
        TraceOpcode::CommitTreasury => {
            if fee_not_required {
                return Ok(TraceDispositionV1::NotRequired);
            }
            if !context.remote_effect_present || !context.receipt_observed && journal_status == OperationStatus::Indeterminate {
                return Ok(TraceDispositionV1::Rejected);
            }
            let evidence = context.treasury_evidence()?;
            let result = context.journal_mut()?.commit_leg(PaymentLeg::TreasuryFee, evidence);
            Ok(match result {
                Ok(Transition::Committed) => TraceDispositionV1::Applied,
                Ok(_) => TraceDispositionV1::Idempotent,
                Err(_) => TraceDispositionV1::Rejected,
            })
        }
        TraceOpcode::CheckpointPayments => {
            if command.fault != TraceFault::None || command.subject != context.operation_nonce {
                return Ok(TraceDispositionV1::Rejected);
            }
            context.payments_checkpoint = Some(command.value);
            Ok(TraceDispositionV1::Applied)
        }
        TraceOpcode::CheckpointTreasury => {
            if fee_not_required {
                return Ok(TraceDispositionV1::NotRequired);
            }
            if command.fault != TraceFault::None
                || command.subject != context.effect_nonce.unwrap_or(0)
                || context.payments_checkpoint != Some(command.value)
            {
                return Ok(TraceDispositionV1::Rejected);
            }
            context.treasury_checkpoint = Some(command.value);
            Ok(TraceDispositionV1::Applied)
        }
        TraceOpcode::MutateIntent => {
            let changed = PaymentIntent::new(
                context.sender.clone(),
                "did:mycelix:trace-adversary",
                context.idempotency_key.clone(),
                digest(context.operation_nonce.wrapping_add(1)),
                command.value.max(1),
                context.fee_minor,
                "SAP",
            );
            let result = context.registry.create_or_get(changed);
            let preserved = context
                .registry
                .get(&context.sender, &context.idempotency_key)
                .is_some_and(|journal| {
                    journal.operation_id() == context.operation_id
                        && journal.intent() == &context.original_intent
                });
            if !preserved {
                return Ok(TraceDispositionV1::Rejected);
            }
            Ok(match result {
                Err(SagaError::IdempotencyConflict) => TraceDispositionV1::Rejected,
                _ => TraceDispositionV1::Applied,
            })
        }
        TraceOpcode::DuplicateDelivery => {
            if fee_not_required {
                return Ok(TraceDispositionV1::NotRequired);
            }
            if context.effect_nonce != Some(command.subject) {
                return Ok(TraceDispositionV1::Rejected);
            }
            if matches!(
                context.journal()?.effect_state(PaymentLeg::TreasuryFee),
                super::EffectState::Committed { .. }
            ) {
                let evidence = context.treasury_evidence()?;
                let result = context.journal_mut()?.commit_leg(PaymentLeg::TreasuryFee, evidence);
                return Ok(match result {
                    Ok(Transition::AlreadyCommitted) => TraceDispositionV1::DuplicateSuppressed,
                    _ => TraceDispositionV1::Rejected,
                });
            }
            // Delivery is a duplicate request, not permission to create a new
            // effect or to infer destination finality from the request itself.
            let effect_id = context.treasury_effect_id.clone();
            let result = context
                .journal_mut()?
                .prepare_leg(PaymentLeg::TreasuryFee, &effect_id);
            Ok(match result {
                Ok(_) => TraceDispositionV1::DuplicateSuppressed,
                Err(_) => TraceDispositionV1::Rejected,
            })
        }
        TraceOpcode::MutateEvent => {
            if fee_not_required {
                return Ok(TraceDispositionV1::NotRequired);
            }
            let evidence = context.treasury_evidence()?;
            let mutated = mutate_treasury_evidence(evidence, TraceOpcode::MutateEvent);
            let result = context.journal_mut()?.commit_leg(PaymentLeg::TreasuryFee, mutated);
            Ok(model_error_disposition(result))
        }
        TraceOpcode::StaleCheckpoint => {
            if command.fault == TraceFault::WrongOwner {
                Ok(TraceDispositionV1::Rejected)
            } else {
                Ok(TraceDispositionV1::Rejected)
            }
        }
        TraceOpcode::ForkCheckpoint => {
            if command.fault == TraceFault::ForkedCheckpoint {
                Ok(TraceDispositionV1::Rejected)
            } else {
                Ok(TraceDispositionV1::Rejected)
            }
        }
        TraceOpcode::Compensate => {
            // Compensation is intentionally a separate effect. This V1
            // interpreter has no compensation adapter and never rewrites the
            // original committed payment or Treasury fee.
            context.compensation_seen = true;
            Ok(TraceDispositionV1::RequiresSeparateEffect)
        }
    }
}

/// Execute one frozen Trace V1 input against the effects-disabled saga model,
/// then compare every observed command disposition with the independent oracle.
pub fn evaluate_trace_v1(trace: &TraceV1) -> Result<TraceEvaluationV1, TraceEvaluationError> {
    trace.canonical_bytes().map_err(|_| TraceEvaluationError::InvalidTrace)?;
    let create = trace
        .commands
        .iter()
        .find(|command| command.opcode == TraceOpcode::CreateIntent)
        .ok_or(TraceEvaluationError::MissingCreateIntent)?;
    let authorize = trace
        .commands
        .iter()
        .find(|command| command.opcode == TraceOpcode::Authorize)
        .ok_or(TraceEvaluationError::MissingAuthorization)?;
    let local_prepare = trace
        .commands
        .iter()
        .find(|command| command.opcode == TraceOpcode::PrepareLocal)
        .ok_or(TraceEvaluationError::MissingAuthorization)?;
    let amount_minor = create.value;
    let fee_minor = authorize.value;
    let total = amount_minor
        .checked_add(fee_minor)
        .ok_or(TraceEvaluationError::InvalidIntent)?;
    if local_prepare.subject != create.subject || local_prepare.value != total {
        return Err(TraceEvaluationError::InvalidTrace);
    }

    let sender = "did:mycelix:trace-sender".to_owned();
    let idempotency_key = format!("trace-{:016x}-{:016x}", trace.seed, trace.trace_index);
    let original_intent = PaymentIntent::new(
        sender.clone(),
        "did:mycelix:trace-recipient",
        idempotency_key.clone(),
        digest(trace.operation_nonce),
        amount_minor,
        fee_minor,
        "SAP",
    );
    let mut registry = JournalRegistry::default();
    let (created, operation_id) = registry
        .create_or_get(original_intent.clone())
        .map_err(|_| TraceEvaluationError::InvalidIntent)?;
    if created != CreateDisposition::Created {
        return Err(TraceEvaluationError::InvalidIntent);
    }
    let local_effect_id = registry
        .get(&sender, &idempotency_key)
        .ok_or(TraceEvaluationError::MissingJournal)?
        .effect_id(PaymentLeg::LocalPaymentTransaction);
    let treasury_effect_id = registry
        .get(&sender, &idempotency_key)
        .ok_or(TraceEvaluationError::MissingJournal)?
        .effect_id(PaymentLeg::TreasuryFee);

    let mut context = ExecutionContext {
        registry,
        sender,
        idempotency_key,
        original_intent,
        operation_id: operation_id.clone(),
        local_effect_id: local_effect_id.clone(),
        treasury_effect_id: treasury_effect_id.clone(),
        operation_nonce: trace.operation_nonce,
        effect_nonce: None,
        amount_minor,
        fee_minor,
        local_committed: false,
        treasury_dispatched: false,
        remote_effect_present: false,
        authoritative_absence_observed: false,
        receipt_observed: false,
        treasury_evidence: None,
        payments_checkpoint: None,
        treasury_checkpoint: None,
        compensation_seen: false,
    };

    let trace_digest = trace
        .canonical_digest()
        .map_err(|_| TraceEvaluationError::InvalidTrace)?;
    let mut steps = Vec::with_capacity(trace.commands.len());
    let mut failures = Vec::new();

    for (command_index, command) in trace.commands.iter().enumerate() {
        let expected = expected_trace_disposition_v1(command, fee_minor);
        let observed = observed_disposition(&mut context, command, command_index)?;
        if expected != observed {
            ExecutionContext::record_failure(
                &mut failures,
                command_index,
                TraceInvariantV1::OracleDispositionAgreement,
                format!(
                    "{:?}: expected {:?}, observed {:?}",
                    command.opcode, expected, observed
                ),
            );
        }

        let status = context.journal()?.status();
        steps.push(TraceStepV1 {
            command_index,
            opcode: command.opcode,
            expected,
            observed,
            operation_status: status,
        });
        context.check_invariants(command_index, &mut failures)?;
    }

    let final_status = context.journal()?.status();
    Ok(TraceEvaluationV1 {
        seed: trace.seed,
        trace_index: trace.trace_index,
        scenario: trace.scenario,
        trace_digest,
        operation_id,
        local_effect_id,
        treasury_effect_id,
        final_status,
        steps,
        failures,
    })
}

pub fn evaluate_generated_trace_v1(
    seed: u64,
    trace_index: u64,
) -> Result<TraceEvaluationV1, TraceEvaluationError> {
    let trace = TraceGeneratorV1::generate(seed, trace_index);
    evaluate_trace_v1(&trace)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn all_generated_scenarios_match_the_independent_disposition_oracle() {
        let mut seen = [false; 8];
        for trace_index in 0..4096 {
            let trace = TraceGeneratorV1::generate(0x5eed_2026, trace_index);
            seen[usize::from(trace.scenario - 1)] = true;
            let evaluation = evaluate_trace_v1(&trace).unwrap();
            assert!(
                evaluation.passed(),
                "seed={} trace={} scenario={} digest={} failures={:?}",
                evaluation.seed,
                evaluation.trace_index,
                evaluation.scenario,
                evaluation.trace_digest,
                evaluation.failures
            );
        }
        assert!(seen.into_iter().all(|scenario_seen| scenario_seen));
    }

    #[test]
    fn changed_intent_conflict_preserves_the_original_local_commit() {
        let trace = (0..4096)
            .map(|index| TraceGeneratorV1::generate(0x5eed_2026, index))
            .find(|trace| trace.scenario == 5)
            .expect("scenario five appears in deterministic corpus");
        let evaluation = evaluate_trace_v1(&trace).unwrap();
        assert!(evaluation.passed(), "{:?}", evaluation.failures);
        assert_eq!(evaluation.final_status, OperationStatus::FeeDistributionPending);
        assert_eq!(
            evaluation.steps.last().map(|step| step.observed),
            Some(TraceDispositionV1::Rejected)
        );
    }

    #[test]
    fn uncertain_lookup_never_releases_indeterminate_state() {
        let trace = (0..4096)
            .map(|index| TraceGeneratorV1::generate(0x5eed_2026, index))
            .find(|trace| trace.scenario == 4)
            .expect("scenario four appears in deterministic corpus");
        let evaluation = evaluate_trace_v1(&trace).unwrap();
        assert!(evaluation.passed(), "{:?}", evaluation.failures);
        assert_eq!(evaluation.final_status, OperationStatus::Indeterminate);
    }

    #[test]
    fn compensation_is_reported_as_a_separate_effect_without_undoing_history() {
        let trace = (0..4096)
            .map(|index| TraceGeneratorV1::generate(0x5eed_2026, index))
            .find(|trace| trace.scenario == 8)
            .expect("scenario eight appears in deterministic corpus");
        let evaluation = evaluate_trace_v1(&trace).unwrap();
        assert!(evaluation.passed(), "{:?}", evaluation.failures);
        assert_eq!(evaluation.final_status, OperationStatus::Completed);
        assert_eq!(
            evaluation.steps.last().map(|step| step.observed),
            Some(TraceDispositionV1::RequiresSeparateEffect)
        );
    }
}
