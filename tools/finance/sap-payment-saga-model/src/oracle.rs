//! Independent deterministic interpreter and invariant oracle for frozen saga trace V1.
//!
//! This module does not call the payment journal or a provider. It interprets the
//! serialized command grammar, records expected dispositions, and independently
//! checks integer conservation and stable-effect constraints.

use sha2::{Digest, Sha256};

use crate::trace::{TraceCommandV1, TraceFault, TraceOpcode, TraceOwner, TraceV1};

const ORACLE_DOMAIN: &[u8] = b"MXSAP-SAGA-ORACLE-V1\0";

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum TraceDispositionV1 {
    Applied = 1,
    NoOp = 2,
    Rejected = 3,
    Indeterminate = 4,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum TraceReasonV1 {
    None = 0,
    OwnerMismatch = 1,
    WrongSubject = 2,
    WrongAmount = 3,
    MissingIntent = 4,
    IntentConflict = 5,
    MissingAuthorization = 6,
    MissingPrepare = 7,
    MissingLocalCommit = 8,
    AlreadyCommitted = 9,
    NotRequired = 10,
    MissingTreasuryPrepare = 11,
    InvalidTreasuryState = 12,
    EffectOutcomeUnknown = 13,
    LookupUncertain = 14,
    AuthoritativeAbsenceContradiction = 15,
    ReceiptMismatch = 16,
    MissingDependency = 17,
    StaleFrontier = 18,
    WrongOwner = 19,
    ForkedCheckpoint = 20,
    DuplicateEffectSuppressed = 21,
    ImmutableIntent = 22,
    MutatedEvent = 23,
    InvalidFault = 24,
    CompensationIdentity = 25,
    CompensationAmount = 26,
    CompensationUnavailable = 27,
    CheckpointConflict = 28,
    MissingDispatch = 29,
    UnknownEffect = 30,
    NotAuthoritativeAbsence = 31,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum TreasuryOracleStateV1 {
    NotPrepared = 0,
    NotRequired = 1,
    Prepared = 2,
    DispatchAuthorized = 3,
    Indeterminate = 4,
    AuthoritativelyAbsent = 5,
    ObservedUnsettled = 6,
    Committed = 7,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct TraceStepResultV1 {
    pub command_index: u8,
    pub disposition: TraceDispositionV1,
    pub reason: TraceReasonV1,
    pub actual_effect_count: u8,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct TraceEvaluationV1 {
    pub trace_digest: [u8; 32],
    pub steps: Vec<TraceStepResultV1>,
    pub principal_minor: u64,
    pub fee_minor: u64,
    pub payer_debit_minor: u64,
    pub recipient_credit_minor: u64,
    pub fee_held_minor: u64,
    pub treasury_credit_minor: u64,
    pub compensation_source_debit_minor: u64,
    pub compensation_recipient_credit_minor: u64,
    pub unresolved_treasury_exposure_minor: u64,
    pub actual_treasury_effect_count: u8,
    pub compensation_count: u8,
    pub local_committed: bool,
    pub treasury_state: TreasuryOracleStateV1,
    pub conservation_delta_minor: i128,
    pub invariant_failure_mask: u32,
}

impl TraceEvaluationV1 {
    pub fn is_safe(&self) -> bool {
        self.invariant_failure_mask == 0
            && self.conservation_delta_minor == 0
            && self.actual_treasury_effect_count <= 1
            && self.compensation_count <= 1
    }

    pub fn canonical_bytes(&self) -> Result<Vec<u8>, TraceOracleError> {
        let count = u32::try_from(self.steps.len()).map_err(|_| TraceOracleError::TooManySteps)?;
        let mut output = Vec::with_capacity(160 + self.steps.len() * 4);
        output.extend_from_slice(ORACLE_DOMAIN);
        output.extend_from_slice(&self.trace_digest);
        output.extend_from_slice(&count.to_be_bytes());
        for step in &self.steps {
            output.push(step.command_index);
            output.push(step.disposition as u8);
            output.push(step.reason as u8);
            output.push(step.actual_effect_count);
        }
        for value in [
            self.principal_minor,
            self.fee_minor,
            self.payer_debit_minor,
            self.recipient_credit_minor,
            self.fee_held_minor,
            self.treasury_credit_minor,
            self.compensation_source_debit_minor,
            self.compensation_recipient_credit_minor,
            self.unresolved_treasury_exposure_minor,
        ] {
            output.extend_from_slice(&value.to_be_bytes());
        }
        output.push(u8::from(self.local_committed));
        output.push(self.treasury_state as u8);
        output.push(self.actual_treasury_effect_count);
        output.push(self.compensation_count);
        output.extend_from_slice(&self.conservation_delta_minor.to_be_bytes());
        output.extend_from_slice(&self.invariant_failure_mask.to_be_bytes());
        Ok(output)
    }

    pub fn canonical_digest(&self) -> Result<String, TraceOracleError> {
        let digest = Sha256::digest(self.canonical_bytes()?);
        let mut output = String::with_capacity(71);
        output.push_str("sha256:");
        for byte in digest {
            use std::fmt::Write as _;
            write!(&mut output, "{byte:02x}").expect("writing to String cannot fail");
        }
        Ok(output)
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum TraceOracleError {
    InvalidTrace,
    TooManySteps,
}

impl std::fmt::Display for TraceOracleError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter.write_str(match self {
            Self::InvalidTrace => "trace failed canonical validation",
            Self::TooManySteps => "trace oracle step count exceeds the canonical bound",
        })
    }
}

impl std::error::Error for TraceOracleError {}

#[derive(Default)]
struct OracleState {
    operation_nonce: u64,
    intent_created: bool,
    authorized: bool,
    local_prepared: bool,
    local_committed: bool,
    principal_minor: u64,
    fee_minor: u64,
    total_minor: u64,
    treasury_effect_id: Option<u64>,
    treasury_state: TreasuryOracleStateV1,
    actual_treasury_effect_count: u8,
    payment_checkpoint: Option<u64>,
    treasury_checkpoint: Option<u64>,
    compensation: Option<(u64, u64)>,
}

fn trace_fault_allowed_v1(opcode: TraceOpcode, fault: TraceFault) -> bool {
    use TraceFault as F;
    use TraceOpcode as O;
    match opcode {
        O::CreateIntent
        | O::Authorize
        | O::PrepareLocal
        | O::CommitLocal
        | O::PrepareTreasury
        | O::DispatchTreasury
        | O::RetrySameEffect
        | O::CommitTreasury
        | O::Compensate => fault == F::None,
        O::MarkIndeterminate => {
            matches!(fault, F::ResponseLostBeforeEffect | F::ResponseLostAfterEffect)
        }
        O::ReconcileTreasury => matches!(
            fault,
            F::None | F::LookupUncertain | F::ReceiptMismatch | F::AuthoritativeAbsence
        ),
        O::CheckpointPayments => matches!(fault, F::None | F::MissingDependency),
        O::CheckpointTreasury => matches!(fault, F::None | F::StaleFrontier),
        O::MutateIntent | O::MutateEvent => fault == F::PayloadMutation,
        O::DuplicateDelivery => fault == F::DuplicateDelivery,
        O::StaleCheckpoint => fault == F::WrongOwner,
        O::ForkCheckpoint => fault == F::ForkedCheckpoint,
    }
}

impl Default for TreasuryOracleStateV1 {
    fn default() -> Self {
        Self::NotPrepared
    }
}

impl OracleState {
    fn new(operation_nonce: u64) -> Self {
        Self {
            operation_nonce,
            ..Self::default()
        }
    }

    fn process(&mut self, command: TraceCommandV1) -> (TraceDispositionV1, TraceReasonV1) {
        use TraceDispositionV1 as D;
        use TraceReasonV1 as R;
        use TraceOpcode as O;
        use TraceOwner as A;

        let expected_owner = match command.opcode {
            O::CreateIntent | O::Authorize | O::Compensate => A::Sender,
            O::PrepareLocal | O::CommitLocal | O::CheckpointPayments => A::Payments,
            O::PrepareTreasury
            | O::DispatchTreasury
            | O::MarkIndeterminate
            | O::ReconcileTreasury
            | O::RetrySameEffect
            | O::CommitTreasury
            | O::CheckpointTreasury => A::Treasury,
            O::MutateIntent
            | O::DuplicateDelivery
            | O::MutateEvent
            | O::StaleCheckpoint
            | O::ForkCheckpoint => A::Adversary,
        };
        if command.owner != expected_owner {
            return (D::Rejected, R::OwnerMismatch);
        }
        if !trace_fault_allowed_v1(command.opcode, command.fault) {
            return (D::Rejected, R::InvalidFault);
        }

        match command.opcode {
            O::CreateIntent => self.create_intent(command),
            O::Authorize => self.authorize(command),
            O::PrepareLocal => self.prepare_local(command),
            O::CommitLocal => self.commit_local(command),
            O::PrepareTreasury => self.prepare_treasury(command),
            O::DispatchTreasury => self.dispatch_treasury(command),
            O::MarkIndeterminate => self.mark_indeterminate(command),
            O::ReconcileTreasury => self.reconcile_treasury(command),
            O::RetrySameEffect => self.retry_same_effect(command),
            O::CommitTreasury => self.commit_treasury(command),
            O::CheckpointPayments => self.checkpoint_payments(command),
            O::CheckpointTreasury => self.checkpoint_treasury(command),
            O::MutateIntent => {
                if command.fault == TraceFault::PayloadMutation {
                    (D::Rejected, R::ImmutableIntent)
                } else {
                    (D::Rejected, R::InvalidFault)
                }
            }
            O::DuplicateDelivery => self.duplicate_delivery(command),
            O::MutateEvent => {
                if command.fault == TraceFault::PayloadMutation {
                    (D::Rejected, R::MutatedEvent)
                } else {
                    (D::Rejected, R::InvalidFault)
                }
            }
            O::StaleCheckpoint => {
                if command.fault == TraceFault::WrongOwner {
                    (D::Rejected, R::WrongOwner)
                } else {
                    (D::Rejected, R::InvalidFault)
                }
            }
            O::ForkCheckpoint => {
                if command.fault == TraceFault::ForkedCheckpoint {
                    (D::Rejected, R::ForkedCheckpoint)
                } else {
                    (D::Rejected, R::InvalidFault)
                }
            }
            O::Compensate => self.compensate(command),
        }
    }

    fn create_intent(
        &mut self,
        command: TraceCommandV1,
    ) -> (TraceDispositionV1, TraceReasonV1) {
        use TraceDispositionV1 as D;
        use TraceReasonV1 as R;
        if command.subject != self.operation_nonce {
            return (D::Rejected, R::WrongSubject);
        }
        if command.value == 0 {
            return (D::Rejected, R::WrongAmount);
        }
        if self.intent_created {
            return if self.principal_minor == command.value {
                (D::NoOp, R::AlreadyCommitted)
            } else {
                (D::Rejected, R::IntentConflict)
            };
        }
        self.intent_created = true;
        self.principal_minor = command.value;
        (D::Applied, R::None)
    }

    fn authorize(&mut self, command: TraceCommandV1) -> (TraceDispositionV1, TraceReasonV1) {
        use TraceDispositionV1 as D;
        use TraceReasonV1 as R;
        if !self.intent_created {
            return (D::Rejected, R::MissingIntent);
        }
        if command.subject != self.operation_nonce {
            return (D::Rejected, R::WrongSubject);
        }
        if self.authorized {
            return if self.fee_minor == command.value {
                (D::NoOp, R::AlreadyCommitted)
            } else {
                (D::Rejected, R::IntentConflict)
            };
        }
        self.fee_minor = command.value;
        self.authorized = true;
        if self.fee_minor == 0 {
            self.treasury_state = TreasuryOracleStateV1::NotRequired;
        }
        (D::Applied, R::None)
    }

    fn prepare_local(&mut self, command: TraceCommandV1) -> (TraceDispositionV1, TraceReasonV1) {
        use TraceDispositionV1 as D;
        use TraceReasonV1 as R;
        if !self.intent_created {
            return (D::Rejected, R::MissingIntent);
        }
        if !self.authorized {
            return (D::Rejected, R::MissingAuthorization);
        }
        if command.subject != self.operation_nonce {
            return (D::Rejected, R::WrongSubject);
        }
        let Some(total) = self.principal_minor.checked_add(self.fee_minor) else {
            return (D::Rejected, R::WrongAmount);
        };
        if command.value != total {
            return (D::Rejected, R::WrongAmount);
        }
        if self.local_prepared {
            return (D::NoOp, R::AlreadyCommitted);
        }
        self.total_minor = total;
        self.local_prepared = true;
        (D::Applied, R::None)
    }

    fn commit_local(&mut self, command: TraceCommandV1) -> (TraceDispositionV1, TraceReasonV1) {
        use TraceDispositionV1 as D;
        use TraceReasonV1 as R;
        if !self.local_prepared {
            return (D::Rejected, R::MissingPrepare);
        }
        if command.subject != self.operation_nonce {
            return (D::Rejected, R::WrongSubject);
        }
        if command.value != self.total_minor {
            return (D::Rejected, R::WrongAmount);
        }
        if self.local_committed {
            return (D::NoOp, R::AlreadyCommitted);
        }
        self.local_committed = true;
        (D::Applied, R::None)
    }

    fn prepare_treasury(
        &mut self,
        command: TraceCommandV1,
    ) -> (TraceDispositionV1, TraceReasonV1) {
        use TraceDispositionV1 as D;
        use TraceReasonV1 as R;
        if !self.local_committed {
            return (D::Rejected, R::MissingLocalCommit);
        }
        if self.fee_minor == 0 || self.treasury_state == TreasuryOracleStateV1::NotRequired {
            return (D::NoOp, R::NotRequired);
        }
        if command.value != self.fee_minor {
            return (D::Rejected, R::WrongAmount);
        }
        match self.treasury_state {
            TreasuryOracleStateV1::NotPrepared => {
                self.treasury_effect_id = Some(command.subject);
                self.treasury_state = TreasuryOracleStateV1::Prepared;
                (D::Applied, R::None)
            }
            TreasuryOracleStateV1::Prepared if self.treasury_effect_id == Some(command.subject) => {
                (D::NoOp, R::AlreadyCommitted)
            }
            _ => (D::Rejected, R::InvalidTreasuryState),
        }
    }

    fn dispatch_treasury(
        &mut self,
        command: TraceCommandV1,
    ) -> (TraceDispositionV1, TraceReasonV1) {
        use TraceDispositionV1 as D;
        use TraceReasonV1 as R;
        if self.fee_minor == 0 || self.treasury_state == TreasuryOracleStateV1::NotRequired {
            return (D::NoOp, R::NotRequired);
        }
        if self.treasury_effect_id != Some(command.subject) {
            return (D::Rejected, R::WrongSubject);
        }
        if command.value != self.fee_minor {
            return (D::Rejected, R::WrongAmount);
        }
        match self.treasury_state {
            TreasuryOracleStateV1::Prepared => {
                self.treasury_state = TreasuryOracleStateV1::DispatchAuthorized;
                (D::Applied, R::None)
            }
            TreasuryOracleStateV1::DispatchAuthorized => (D::NoOp, R::AlreadyCommitted),
            _ => (D::Rejected, R::InvalidTreasuryState),
        }
    }

    fn mark_indeterminate(
        &mut self,
        command: TraceCommandV1,
    ) -> (TraceDispositionV1, TraceReasonV1) {
        use TraceDispositionV1 as D;
        use TraceReasonV1 as R;
        if self.fee_minor == 0 || self.treasury_state == TreasuryOracleStateV1::NotRequired {
            return (D::NoOp, R::NotRequired);
        }
        if self.treasury_effect_id != Some(command.subject) {
            return (D::Rejected, R::WrongSubject);
        }
        if command.value != self.fee_minor {
            return (D::Rejected, R::WrongAmount);
        }
        if self.treasury_state == TreasuryOracleStateV1::Indeterminate {
            return (D::NoOp, R::EffectOutcomeUnknown);
        }
        if self.treasury_state != TreasuryOracleStateV1::DispatchAuthorized {
            return (D::Rejected, R::MissingDispatch);
        }
        match command.fault {
            TraceFault::ResponseLostBeforeEffect => {
                self.actual_treasury_effect_count = 0;
            }
            TraceFault::ResponseLostAfterEffect => {
                self.actual_treasury_effect_count = 1;
            }
            _ => return (D::Rejected, R::InvalidFault),
        }
        self.treasury_state = TreasuryOracleStateV1::Indeterminate;
        (D::Indeterminate, R::EffectOutcomeUnknown)
    }

    fn reconcile_treasury(
        &mut self,
        command: TraceCommandV1,
    ) -> (TraceDispositionV1, TraceReasonV1) {
        use TraceDispositionV1 as D;
        use TraceReasonV1 as R;
        if self.fee_minor == 0 || self.treasury_state == TreasuryOracleStateV1::NotRequired {
            return (D::NoOp, R::NotRequired);
        }
        if self.treasury_effect_id != Some(command.subject) {
            return (D::Rejected, R::WrongSubject);
        }
        if command.value != self.fee_minor {
            return (D::Rejected, R::WrongAmount);
        }
        match (self.treasury_state, command.fault) {
            (TreasuryOracleStateV1::Indeterminate, TraceFault::AuthoritativeAbsence) => {
                if self.actual_treasury_effect_count != 0 {
                    return (D::Rejected, R::AuthoritativeAbsenceContradiction);
                }
                self.treasury_state = TreasuryOracleStateV1::AuthoritativelyAbsent;
                (D::Applied, R::None)
            }
            (TreasuryOracleStateV1::Indeterminate, TraceFault::LookupUncertain) => {
                (D::Indeterminate, R::LookupUncertain)
            }
            (TreasuryOracleStateV1::Indeterminate, TraceFault::ReceiptMismatch) => {
                (D::Rejected, R::ReceiptMismatch)
            }
            (TreasuryOracleStateV1::Indeterminate, TraceFault::None) => {
                if self.actual_treasury_effect_count == 1 {
                    self.treasury_state = TreasuryOracleStateV1::ObservedUnsettled;
                    (D::Applied, R::None)
                } else {
                    (D::Indeterminate, R::NotAuthoritativeAbsence)
                }
            }
            (TreasuryOracleStateV1::AuthoritativelyAbsent, TraceFault::AuthoritativeAbsence) => {
                (D::NoOp, R::AlreadyCommitted)
            }
            (TreasuryOracleStateV1::ObservedUnsettled, TraceFault::None) => {
                (D::NoOp, R::AlreadyCommitted)
            }
            (TreasuryOracleStateV1::Committed, TraceFault::None) => {
                (D::NoOp, R::AlreadyCommitted)
            }
            _ => (D::Rejected, R::InvalidTreasuryState),
        }
    }

    fn retry_same_effect(
        &mut self,
        command: TraceCommandV1,
    ) -> (TraceDispositionV1, TraceReasonV1) {
        use TraceDispositionV1 as D;
        use TraceReasonV1 as R;
        if self.fee_minor == 0 || self.treasury_state == TreasuryOracleStateV1::NotRequired {
            return (D::NoOp, R::NotRequired);
        }
        if self.treasury_effect_id != Some(command.subject) {
            return (D::Rejected, R::WrongSubject);
        }
        if command.value != self.fee_minor {
            return (D::Rejected, R::WrongAmount);
        }
        match self.treasury_state {
            TreasuryOracleStateV1::AuthoritativelyAbsent => {
                self.treasury_state = TreasuryOracleStateV1::Prepared;
                (D::Applied, R::None)
            }
            TreasuryOracleStateV1::Prepared => (D::NoOp, R::AlreadyCommitted),
            _ => (D::Rejected, R::NotAuthoritativeAbsence),
        }
    }

    fn commit_treasury(
        &mut self,
        command: TraceCommandV1,
    ) -> (TraceDispositionV1, TraceReasonV1) {
        use TraceDispositionV1 as D;
        use TraceReasonV1 as R;
        if !self.local_committed {
            return (D::Rejected, R::MissingLocalCommit);
        }
        if self.fee_minor == 0 || self.treasury_state == TreasuryOracleStateV1::NotRequired {
            return (D::NoOp, R::NotRequired);
        }
        if self.treasury_effect_id != Some(command.subject) {
            return (D::Rejected, R::WrongSubject);
        }
        if command.value != self.fee_minor {
            return (D::Rejected, R::WrongAmount);
        }
        match self.treasury_state {
            TreasuryOracleStateV1::DispatchAuthorized => {
                self.actual_treasury_effect_count = 1;
                self.treasury_state = TreasuryOracleStateV1::Committed;
                (D::Applied, R::None)
            }
            TreasuryOracleStateV1::ObservedUnsettled
                if self.actual_treasury_effect_count == 1 =>
            {
                self.treasury_state = TreasuryOracleStateV1::Committed;
                (D::Applied, R::None)
            }
            TreasuryOracleStateV1::Committed => (D::NoOp, R::AlreadyCommitted),
            TreasuryOracleStateV1::Indeterminate => (D::Rejected, R::EffectOutcomeUnknown),
            TreasuryOracleStateV1::AuthoritativelyAbsent => {
                (D::Rejected, R::NotAuthoritativeAbsence)
            }
            _ => (D::Rejected, R::InvalidTreasuryState),
        }
    }

    fn checkpoint_payments(
        &mut self,
        command: TraceCommandV1,
    ) -> (TraceDispositionV1, TraceReasonV1) {
        use TraceDispositionV1 as D;
        use TraceReasonV1 as R;
        if command.subject != self.operation_nonce {
            return (D::Rejected, R::WrongSubject);
        }
        if command.fault == TraceFault::MissingDependency {
            return (D::Rejected, R::MissingDependency);
        }
        if command.fault != TraceFault::None {
            return (D::Rejected, R::InvalidFault);
        }
        if !self.local_committed {
            return (D::Rejected, R::MissingDependency);
        }
        match self.payment_checkpoint {
            Some(current) if current == command.value => (D::NoOp, R::AlreadyCommitted),
            Some(_) => (D::Rejected, R::CheckpointConflict),
            None => {
                self.payment_checkpoint = Some(command.value);
                (D::Applied, R::None)
            }
        }
    }

    fn checkpoint_treasury(
        &mut self,
        command: TraceCommandV1,
    ) -> (TraceDispositionV1, TraceReasonV1) {
        use TraceDispositionV1 as D;
        use TraceReasonV1 as R;
        if command.fault == TraceFault::StaleFrontier {
            return (D::Rejected, R::StaleFrontier);
        }
        if command.fault != TraceFault::None {
            return (D::Rejected, R::InvalidFault);
        }
        if self.fee_minor > 0 && self.treasury_effect_id != Some(command.subject) {
            return (D::Rejected, R::WrongSubject);
        }
        if self.payment_checkpoint != Some(command.value) {
            return (D::Rejected, R::MissingDependency);
        }
        if self.fee_minor > 0
            && !matches!(
                self.treasury_state,
                TreasuryOracleStateV1::Committed | TreasuryOracleStateV1::NotRequired
            )
        {
            return (D::Rejected, R::MissingDependency);
        }
        if self.fee_minor == 0 && self.treasury_state != TreasuryOracleStateV1::NotRequired {
            return (D::Rejected, R::MissingDependency);
        }
        match self.treasury_checkpoint {
            Some(current) if current == command.value => (D::NoOp, R::AlreadyCommitted),
            Some(_) => (D::Rejected, R::CheckpointConflict),
            None => {
                self.treasury_checkpoint = Some(command.value);
                (D::Applied, R::None)
            }
        }
    }

    fn duplicate_delivery(
        &mut self,
        command: TraceCommandV1,
    ) -> (TraceDispositionV1, TraceReasonV1) {
        use TraceDispositionV1 as D;
        use TraceReasonV1 as R;
        if command.fault != TraceFault::DuplicateDelivery {
            return (D::Rejected, R::InvalidFault);
        }
        if self.fee_minor == 0 || self.treasury_state == TreasuryOracleStateV1::NotRequired {
            return (D::NoOp, R::NotRequired);
        }
        if self.treasury_effect_id != Some(command.subject) {
            return (D::Rejected, R::WrongSubject);
        }
        if command.value != self.fee_minor {
            return (D::Rejected, R::WrongAmount);
        }
        match self.treasury_state {
            TreasuryOracleStateV1::DispatchAuthorized => {
                // The first delivery may have reached the destination. The
                // duplicate uses the same stable ID and cannot create a second effect.
                self.actual_treasury_effect_count = 1;
                self.treasury_state = TreasuryOracleStateV1::Indeterminate;
                (D::Indeterminate, R::DuplicateEffectSuppressed)
            }
            TreasuryOracleStateV1::Indeterminate => {
                (D::Indeterminate, R::DuplicateEffectSuppressed)
            }
            TreasuryOracleStateV1::Committed => (D::NoOp, R::AlreadyCommitted),
            _ => (D::Rejected, R::InvalidTreasuryState),
        }
    }

    fn compensate(
        &mut self,
        command: TraceCommandV1,
    ) -> (TraceDispositionV1, TraceReasonV1) {
        use TraceDispositionV1 as D;
        use TraceReasonV1 as R;
        if !self.local_committed {
            return (D::Rejected, R::CompensationUnavailable);
        }
        if self.fee_minor > 0 && self.treasury_state != TreasuryOracleStateV1::Committed {
            return (D::Rejected, R::CompensationUnavailable);
        }
        if command.subject == self.operation_nonce
            || self.treasury_effect_id == Some(command.subject)
        {
            return (D::Rejected, R::CompensationIdentity);
        }
        if command.value != self.total_minor {
            return (D::Rejected, R::CompensationAmount);
        }
        match self.compensation {
            Some((effect_id, amount)) if effect_id == command.subject && amount == command.value => {
                (D::NoOp, R::AlreadyCommitted)
            }
            Some(_) => (D::Rejected, R::CompensationIdentity),
            None => {
                self.compensation = Some((command.subject, command.value));
                (D::Applied, R::None)
            }
        }
    }
}

pub struct TraceOracleV1;

impl TraceOracleV1 {
    pub fn evaluate(trace: &TraceV1) -> Result<TraceEvaluationV1, TraceOracleError> {
        let encoded = trace.canonical_bytes().map_err(|_| TraceOracleError::InvalidTrace)?;
        let trace_digest: [u8; 32] = Sha256::digest(encoded).into();
        let mut state = OracleState::new(trace.operation_nonce);
        let mut steps = Vec::with_capacity(trace.commands.len());

        for (index, command) in trace.commands.iter().copied().enumerate() {
            let (disposition, reason) = state.process(command);
            steps.push(TraceStepResultV1 {
                command_index: u8::try_from(index).map_err(|_| TraceOracleError::TooManySteps)?,
                disposition,
                reason,
                actual_effect_count: state.actual_treasury_effect_count,
            });
        }

        let payer_debit_minor = if state.local_committed { state.total_minor } else { 0 };
        let recipient_credit_minor = if state.local_committed { state.principal_minor } else { 0 };
        let treasury_credit_minor = if state.treasury_state == TreasuryOracleStateV1::Committed {
            state.fee_minor
        } else {
            0
        };
        let fee_held_minor = if state.local_committed && treasury_credit_minor == 0 {
            state.fee_minor
        } else {
            0
        };
        let compensation_source_debit_minor =
            state.compensation.map_or(0, |(_, amount)| amount);
        let compensation_recipient_credit_minor = compensation_source_debit_minor;
        let unresolved_treasury_exposure_minor = if matches!(
            state.treasury_state,
            TreasuryOracleStateV1::Prepared
                | TreasuryOracleStateV1::DispatchAuthorized
                | TreasuryOracleStateV1::Indeterminate
                | TreasuryOracleStateV1::ObservedUnsettled
        ) {
            state.fee_minor
        } else {
            0
        };

        let conservation_delta_minor = i128::from(payer_debit_minor)
            - i128::from(recipient_credit_minor)
            - i128::from(fee_held_minor)
            - i128::from(treasury_credit_minor)
            + i128::from(compensation_source_debit_minor)
            - i128::from(compensation_recipient_credit_minor);

        let mut invariant_failure_mask = 0_u32;
        if conservation_delta_minor != 0 {
            invariant_failure_mask |= 1;
        }
        if state.actual_treasury_effect_count > 1 {
            invariant_failure_mask |= 1 << 1;
        }
        if state.compensation.is_some() && state.compensation.is_some_and(|(id, _)| id == trace.operation_nonce) {
            invariant_failure_mask |= 1 << 2;
        }
        if state.treasury_state == TreasuryOracleStateV1::Committed
            && state.fee_minor > 0
            && state.actual_treasury_effect_count != 1
        {
            invariant_failure_mask |= 1 << 3;
        }
        if state.treasury_state == TreasuryOracleStateV1::AuthoritativelyAbsent
            && state.actual_treasury_effect_count != 0
        {
            invariant_failure_mask |= 1 << 4;
        }

        Ok(TraceEvaluationV1 {
            trace_digest,
            steps,
            principal_minor: state.principal_minor,
            fee_minor: state.fee_minor,
            payer_debit_minor,
            recipient_credit_minor,
            fee_held_minor,
            treasury_credit_minor,
            compensation_source_debit_minor,
            compensation_recipient_credit_minor,
            unresolved_treasury_exposure_minor,
            actual_treasury_effect_count: state.actual_treasury_effect_count,
            compensation_count: u8::from(state.compensation.is_some()),
            local_committed: state.local_committed,
            treasury_state: state.treasury_state,
            conservation_delta_minor,
            invariant_failure_mask,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::trace::TraceGeneratorV1;

    fn find_scenario(scenario: u8) -> TraceV1 {
        (0..100_000)
            .map(|index| {
                TraceGeneratorV1::generate(0x0123_4567_89ab_cdef, index)
            })
            .find(|trace| trace.scenario == scenario)
            .expect("scenario is reachable in the frozen trace generator")
    }

    #[test]
    fn every_generated_scenario_has_expected_safe_evaluation() {
        let mut scenarios_seen = [false; 8];
        for seed in 0..4096_u64 {
            let trace = TraceGeneratorV1::generate(seed, seed.rotate_left(13));
            scenarios_seen[usize::from(trace.scenario - 1)] = true;
            let evaluation = TraceOracleV1::evaluate(&trace).unwrap();
            assert!(evaluation.is_safe(), "scenario={} seed={seed}", trace.scenario);
            assert!(evaluation.local_committed);
            assert_eq!(evaluation.conservation_delta_minor, 0);
            assert!(evaluation.actual_treasury_effect_count <= 1);
            assert!(evaluation.compensation_count <= 1);
        }
        assert!(scenarios_seen.into_iter().all(|seen| seen));
    }

    #[test]
    fn oracle_v1_matches_shared_independent_known_answers() {
        let kat = include_str!("../fixtures/saga-oracle-v1.kat");
        let mut vectors_seen = 0;
        for line in kat.lines().filter_map(|line| line.strip_prefix("vector=")) {
            let fields: Vec<_> = line.split(',').collect();
            assert_eq!(fields.len(), 5);
            let seed = u64::from_str_radix(fields[0], 16).unwrap();
            let trace_index = fields[1].parse::<u64>().unwrap();
            let scenario = fields[2].parse::<u8>().unwrap();
            let command_count = fields[3].parse::<usize>().unwrap();
            let expected_digest = format!("sha256:{}", fields[4]);

            let trace = TraceGeneratorV1::generate(seed, trace_index);
            assert_eq!(trace.scenario, scenario);
            assert_eq!(trace.commands.len(), command_count);
            let evaluation = TraceOracleV1::evaluate(&trace).unwrap();
            assert!(evaluation.is_safe(), "seed={} scenario={scenario}", fields[0]);
            assert_eq!(evaluation.canonical_digest().unwrap(), expected_digest);
            vectors_seen += 1;
        }
        assert_eq!(vectors_seen, 8);
    }

    #[test]
    fn scenario_two_requires_authoritative_absence_before_same_effect_retry() {
        let evaluation = TraceOracleV1::evaluate(&find_scenario(2)).unwrap();
        assert_eq!(evaluation.treasury_state, TreasuryOracleStateV1::Committed);
        assert_eq!(evaluation.actual_treasury_effect_count, 1);
        assert_eq!(evaluation.unresolved_treasury_exposure_minor, 0);
    }

    #[test]
    fn uncertain_lookup_never_becomes_commit_or_absence() {
        let evaluation = TraceOracleV1::evaluate(&find_scenario(4)).unwrap();
        assert_eq!(evaluation.treasury_state, TreasuryOracleStateV1::Indeterminate);
        assert_eq!(evaluation.unresolved_treasury_exposure_minor, evaluation.fee_minor);
        assert_eq!(evaluation.actual_treasury_effect_count, 1);
    }

    #[test]
    fn duplicate_delivery_never_increments_effect_count_twice() {
        let evaluation = TraceOracleV1::evaluate(&find_scenario(6)).unwrap();
        assert_eq!(evaluation.actual_treasury_effect_count, 1);
        assert_eq!(evaluation.treasury_state, TreasuryOracleStateV1::Indeterminate);
        assert!(evaluation.steps.iter().any(|step| {
            step.disposition == TraceDispositionV1::Indeterminate
                && step.reason == TraceReasonV1::DuplicateEffectSuppressed
        }));
    }

    #[test]
    fn compensation_is_a_separate_effect_and_does_not_erase_original_commit() {
        let evaluation = TraceOracleV1::evaluate(&find_scenario(8)).unwrap();
        assert!(evaluation.local_committed);
        assert_eq!(evaluation.compensation_count, 1);
        assert_eq!(
            evaluation.compensation_source_debit_minor,
            evaluation.compensation_recipient_credit_minor
        );
        assert_eq!(
            evaluation.payer_debit_minor,
            evaluation.principal_minor + evaluation.fee_minor
        );
    }

    #[test]
    fn wrong_owner_and_false_absence_are_explicitly_rejected() {
        let mut wrong_owner = find_scenario(1);
        wrong_owner.commands[0].owner = TraceOwner::Adversary;
        let evaluation = TraceOracleV1::evaluate(&wrong_owner).unwrap();
        assert_eq!(evaluation.steps[0].disposition, TraceDispositionV1::Rejected);
        assert_eq!(evaluation.steps[0].reason, TraceReasonV1::OwnerMismatch);

        let mut false_absence = find_scenario(3);
        let reconcile = false_absence
            .commands
            .iter_mut()
            .find(|command| command.opcode == TraceOpcode::ReconcileTreasury)
            .expect("scenario three has a reconciliation command");
        reconcile.fault = TraceFault::AuthoritativeAbsence;
        let evaluation = TraceOracleV1::evaluate(&false_absence).unwrap();
        let step = evaluation
            .steps
            .iter()
            .find(|step| step.reason == TraceReasonV1::AuthoritativeAbsenceContradiction)
            .expect("contradictory absence must be rejected");
        assert_eq!(step.disposition, TraceDispositionV1::Rejected);
        assert_eq!(evaluation.treasury_state, TreasuryOracleStateV1::Indeterminate);
        assert_eq!(evaluation.unresolved_treasury_exposure_minor, evaluation.fee_minor);
    }
}
