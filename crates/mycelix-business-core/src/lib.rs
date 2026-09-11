// Copyright (C) 2024-2026 Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Dependency-light contracts for the Mycelix Business Fabric.
//!
//! This crate owns no business-domain state and grants no institutional authority.
//! It exists to make cross-domain reasoning, preparation, delegation, coordination,
//! degradation, and execution bindings explicit and testable.

use std::collections::BTreeSet;

/// Stable opaque reference used to bind records owned by other domains.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct ReferenceId(String);

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ReferenceIdError {
    Empty,
    NonCanonical,
    TooLong,
    ControlCharacter,
}

impl ReferenceId {
    pub const MAX_BYTES: usize = 512;

    pub fn new(value: impl Into<String>) -> Result<Self, ReferenceIdError> {
        let value = value.into();
        if value.is_empty() {
            return Err(ReferenceIdError::Empty);
        }
        if value != value.trim() {
            return Err(ReferenceIdError::NonCanonical);
        }
        if value.len() > Self::MAX_BYTES {
            return Err(ReferenceIdError::TooLong);
        }
        if value.chars().any(char::is_control) {
            return Err(ReferenceIdError::ControlCharacter);
        }
        Ok(Self(value))
    }

    pub fn as_str(&self) -> &str {
        &self.0
    }
}

/// Canonical 32-byte digest reference. Digest semantics are defined by the owner.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Digest32(pub [u8; 32]);

impl Digest32 {
    pub const fn repeat(byte: u8) -> Self {
        Self([byte; 32])
    }
}

macro_rules! typed_reference {
    ($name:ident) => {
        #[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
        pub struct $name(pub ReferenceId);

        impl $name {
            pub fn new(value: impl Into<String>) -> Result<Self, ReferenceIdError> {
                ReferenceId::new(value).map(Self)
            }

            pub fn as_ref_id(&self) -> &ReferenceId {
                &self.0
            }
        }
    };
}

typed_reference!(ObservationRef);
typed_reference!(EstimateRef);
typed_reference!(ForecastRef);
typed_reference!(ProposalRef);
typed_reference!(AuthorizedIntentRef);
typed_reference!(ExecutionAttemptRef);
typed_reference!(ExecutionReceiptRef);
typed_reference!(ReconciliationRef);
typed_reference!(CapabilityRef);
typed_reference!(RoleRef);
typed_reference!(SubjectRef);
typed_reference!(ScopeRef);
typed_reference!(ProfileRef);
typed_reference!(ReservationId);

/// Descriptive business composition only. Presence of a capability grants no authority.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BusinessDescriptor {
    pub profile: ProfileRef,
    pub roles: BTreeSet<RoleRef>,
    pub capabilities: BTreeSet<CapabilityRef>,
}

/// Exact frontier marker from an independently authoritative domain.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FrontierRef {
    pub domain: ReferenceId,
    pub sequence: u64,
    pub digest: Digest32,
}

/// The three independent contexts that can invalidate a prepared business action.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct DecisionFrontiers {
    pub observation: Vec<FrontierRef>,
    pub policy: Vec<FrontierRef>,
    pub authority: Vec<FrontierRef>,
}

/// Versioned semantic identity of an action class.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ActionContractRef {
    pub semantic_id: ReferenceId,
    pub digest: Digest32,
}

/// Opaque reference to authority minted by Governance/Xenia, never by this crate.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct AuthorityLeaseRef {
    pub lease_id: ReferenceId,
    pub authority_epoch: u64,
    pub sequence: u64,
    pub fencing_token: u64,
    pub scope: ScopeRef,
    pub issued_at_unix_ms: u64,
    pub expires_at_unix_ms: u64,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum AuthorityLeaseError {
    ZeroAuthorityEpoch,
    ZeroSequence,
    ZeroFencingToken,
    InvalidWindow,
    NotYetValid,
    Expired,
    AuthorityEpochChanged { bound: u64, current: u64 },
}

impl AuthorityLeaseRef {
    pub fn validate(&self) -> Result<(), AuthorityLeaseError> {
        if self.authority_epoch == 0 {
            return Err(AuthorityLeaseError::ZeroAuthorityEpoch);
        }
        if self.sequence == 0 {
            return Err(AuthorityLeaseError::ZeroSequence);
        }
        if self.fencing_token == 0 {
            return Err(AuthorityLeaseError::ZeroFencingToken);
        }
        if self.issued_at_unix_ms >= self.expires_at_unix_ms {
            return Err(AuthorityLeaseError::InvalidWindow);
        }
        Ok(())
    }

    /// Strict execution-time check. A changed authority epoch requires revalidation.
    pub fn validate_at(
        &self,
        now_unix_ms: u64,
        current_authority_epoch: u64,
    ) -> Result<(), AuthorityLeaseError> {
        self.validate()?;
        if current_authority_epoch != self.authority_epoch {
            return Err(AuthorityLeaseError::AuthorityEpochChanged {
                bound: self.authority_epoch,
                current: current_authority_epoch,
            });
        }
        if now_unix_ms < self.issued_at_unix_ms {
            return Err(AuthorityLeaseError::NotYetValid);
        }
        if now_unix_ms >= self.expires_at_unix_ms {
            return Err(AuthorityLeaseError::Expired);
        }
        Ok(())
    }
}

/// Reservation remains owned by its authoritative domain.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ReservationRef {
    pub domain: ReferenceId,
    pub reservation_id: ReservationId,
    pub subject: SubjectRef,
    pub digest: Digest32,
    pub expires_at_unix_ms: u64,
}

/// Short-lived execution candidate. Preparation is not execution.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PreparedAction {
    pub action_contract: ActionContractRef,
    pub decision_capsule: ReferenceId,
    pub intent_digest: Digest32,
    pub frontiers: DecisionFrontiers,
    pub authority_lease: AuthorityLeaseRef,
    pub reservations: Vec<ReservationRef>,
    pub idempotency_key: ReferenceId,
    pub prepared_at_unix_ms: u64,
    pub expires_at_unix_ms: u64,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum PreparedActionError {
    InvalidWindow,
    OutlivesAuthority,
    ReservationExpiredAtPreparation { reservation: ReservationId },
    ReservationExpiresBeforeAction { reservation: ReservationId },
    Authority(AuthorityLeaseError),
}

impl PreparedAction {
    pub fn validate(&self) -> Result<(), PreparedActionError> {
        self.authority_lease
            .validate()
            .map_err(PreparedActionError::Authority)?;
        if self.prepared_at_unix_ms >= self.expires_at_unix_ms {
            return Err(PreparedActionError::InvalidWindow);
        }
        if self.prepared_at_unix_ms < self.authority_lease.issued_at_unix_ms
            || self.expires_at_unix_ms > self.authority_lease.expires_at_unix_ms
        {
            return Err(PreparedActionError::OutlivesAuthority);
        }
        for reservation in &self.reservations {
            if reservation.expires_at_unix_ms <= self.prepared_at_unix_ms {
                return Err(PreparedActionError::ReservationExpiredAtPreparation {
                    reservation: reservation.reservation_id.clone(),
                });
            }
            if reservation.expires_at_unix_ms < self.expires_at_unix_ms {
                return Err(PreparedActionError::ReservationExpiresBeforeAction {
                    reservation: reservation.reservation_id.clone(),
                });
            }
        }
        Ok(())
    }

    pub fn validate_for_execution(
        &self,
        now_unix_ms: u64,
        current_authority_epoch: u64,
    ) -> Result<(), PreparedActionError> {
        self.validate()?;
        if now_unix_ms < self.prepared_at_unix_ms || now_unix_ms >= self.expires_at_unix_ms {
            return Err(PreparedActionError::InvalidWindow);
        }
        self.authority_lease
            .validate_at(now_unix_ms, current_authority_epoch)
            .map_err(PreparedActionError::Authority)?;
        for reservation in &self.reservations {
            if now_unix_ms >= reservation.expires_at_unix_ms {
                return Err(PreparedActionError::ReservationExpiredAtPreparation {
                    reservation: reservation.reservation_id.clone(),
                });
            }
        }
        Ok(())
    }
}

/// Ordered from least to most consequential. A delegated child may not exceed its parent.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum RiskClass {
    ReadOnly,
    Low,
    Moderate,
    High,
    Critical,
}

/// `None` means no financial-budget authority, not unlimited authority.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BudgetLimit {
    None,
    Limited(u64),
}

impl BudgetLimit {
    fn no_broader_than(self, parent: Self) -> bool {
        match (self, parent) {
            (BudgetLimit::None, _) => true,
            (BudgetLimit::Limited(_), BudgetLimit::None) => false,
            (BudgetLimit::Limited(child), BudgetLimit::Limited(parent)) => child <= parent,
        }
    }
}

/// Authority shape used only for attenuation checks; actual grants are externally owned.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct AuthorityScope {
    pub capabilities: BTreeSet<CapabilityRef>,
    pub subjects: BTreeSet<SubjectRef>,
    pub risk_ceiling: RiskClass,
    pub budget: BudgetLimit,
    pub expires_at_unix_ms: u64,
}

impl AuthorityScope {
    pub fn is_no_broader_than(&self, parent: &Self) -> bool {
        self.capabilities.is_subset(&parent.capabilities)
            && self.subjects.is_subset(&parent.subjects)
            && self.risk_ceiling <= parent.risk_ceiling
            && self.budget.no_broader_than(parent.budget)
            && self.expires_at_unix_ms <= parent.expires_at_unix_ms
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DelegationEnvelope {
    pub parent_authority: ReferenceId,
    pub child_authority: ReferenceId,
    pub parent_scope: AuthorityScope,
    pub child_scope: AuthorityScope,
    pub depth: u16,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum DelegationError {
    ZeroDepth,
    AuthorityAmplification,
}

impl DelegationEnvelope {
    pub fn validate(&self) -> Result<(), DelegationError> {
        if self.depth == 0 {
            return Err(DelegationError::ZeroDepth);
        }
        if !self.child_scope.is_no_broader_than(&self.parent_scope) {
            return Err(DelegationError::AuthorityAmplification);
        }
        Ok(())
    }
}

/// Conserved aggregate envelope for sibling delegated budget reservations.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SharedBudgetEnvelope {
    limit: u64,
    reserved: u64,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum BudgetReservationError {
    Overflow,
    ExceedsEnvelope { requested: u64, available: u64 },
    ReleaseExceedsReserved,
}

impl SharedBudgetEnvelope {
    pub const fn new(limit: u64) -> Self {
        Self { limit, reserved: 0 }
    }

    pub const fn limit(&self) -> u64 {
        self.limit
    }

    pub const fn reserved(&self) -> u64 {
        self.reserved
    }

    pub const fn available(&self) -> u64 {
        self.limit - self.reserved
    }

    pub fn reserve(&mut self, amount: u64) -> Result<(), BudgetReservationError> {
        let next = self
            .reserved
            .checked_add(amount)
            .ok_or(BudgetReservationError::Overflow)?;
        if next > self.limit {
            return Err(BudgetReservationError::ExceedsEnvelope {
                requested: amount,
                available: self.available(),
            });
        }
        self.reserved = next;
        Ok(())
    }

    pub fn release(&mut self, amount: u64) -> Result<(), BudgetReservationError> {
        self.reserved = self
            .reserved
            .checked_sub(amount)
            .ok_or(BudgetReservationError::ReleaseExceedsReserved)?;
        Ok(())
    }
}

/// Ordered autonomy ceiling. Missing information may only move this value downward.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum AutonomyLevel {
    Observe,
    Explain,
    Recommend,
    Draft,
    DelegatedSingleAction,
    DelegatedOrchestration,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct EpistemicHealth {
    pub missing_required_sources: u16,
    pub stale_required_sources: u16,
    pub unresolved_conflicts: u16,
    pub calibration_cold: bool,
}

impl EpistemicHealth {
    /// Conservative ceiling used as an integration helper, never as institutional authorization.
    pub fn autonomy_ceiling(self) -> AutonomyLevel {
        if self.missing_required_sources > 0 {
            AutonomyLevel::Observe
        } else if self.unresolved_conflicts > 0 {
            AutonomyLevel::Recommend
        } else if self.stale_required_sources > 0 || self.calibration_cold {
            AutonomyLevel::Draft
        } else {
            AutonomyLevel::DelegatedOrchestration
        }
    }
}

/// Unknown is first-class and must not be collapsed into failure for blind retry.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ExecutionDisposition {
    ConfirmedSuccess,
    ConfirmedFailure,
    OutcomeUnknown,
}

/// Ordered from least to most autonomous.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum OperatingMode {
    ObserveOnly,
    RecommendOnly,
    ApprovalRequired,
    Autonomous,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CircuitBreaker {
    mode: OperatingMode,
    tripped: bool,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum RearmError {
    Authority(AuthorityLeaseError),
    NotTripped,
}

impl CircuitBreaker {
    pub const fn new(mode: OperatingMode) -> Self {
        Self {
            mode,
            tripped: false,
        }
    }

    pub const fn mode(&self) -> OperatingMode {
        self.mode
    }

    pub const fn is_tripped(&self) -> bool {
        self.tripped
    }

    /// A trip can only preserve or reduce autonomy.
    pub fn trip_to(&mut self, ceiling: OperatingMode) {
        if ceiling < self.mode {
            self.mode = ceiling;
        }
        self.tripped = true;
    }

    /// Clearing the external condition does not re-arm the breaker. Re-arm is explicit.
    pub fn rearm(
        &mut self,
        target: OperatingMode,
        authority: &AuthorityLeaseRef,
        now_unix_ms: u64,
        current_authority_epoch: u64,
    ) -> Result<(), RearmError> {
        if !self.tripped {
            return Err(RearmError::NotTripped);
        }
        authority
            .validate_at(now_unix_ms, current_authority_epoch)
            .map_err(RearmError::Authority)?;
        self.mode = target;
        self.tripped = false;
        Ok(())
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct InvariantId(pub &'static str);

pub const BUSINESS_INVARIANTS: [InvariantId; 27] = [
    InvariantId("BIZ-I01"),
    InvariantId("BIZ-I02"),
    InvariantId("BIZ-I03"),
    InvariantId("BIZ-I04"),
    InvariantId("BIZ-I05"),
    InvariantId("BIZ-I06"),
    InvariantId("BIZ-I07"),
    InvariantId("BIZ-I08"),
    InvariantId("BIZ-I09"),
    InvariantId("BIZ-I10"),
    InvariantId("BIZ-I11"),
    InvariantId("BIZ-I12"),
    InvariantId("BIZ-I13"),
    InvariantId("BIZ-I14"),
    InvariantId("BIZ-I15"),
    InvariantId("BIZ-I16"),
    InvariantId("BIZ-I17"),
    InvariantId("BIZ-I18"),
    InvariantId("BIZ-I19"),
    InvariantId("BIZ-I20"),
    InvariantId("BIZ-I21"),
    InvariantId("BIZ-I22"),
    InvariantId("BIZ-I23"),
    InvariantId("BIZ-I24"),
    InvariantId("BIZ-I25"),
    InvariantId("BIZ-I26"),
    InvariantId("BIZ-I27"),
];

#[cfg(test)]
mod tests {
    use super::*;

    fn id(value: &str) -> ReferenceId {
        ReferenceId::new(value).unwrap()
    }

    fn capability(value: &str) -> CapabilityRef {
        CapabilityRef(id(value))
    }

    fn subject(value: &str) -> SubjectRef {
        SubjectRef(id(value))
    }

    fn lease() -> AuthorityLeaseRef {
        AuthorityLeaseRef {
            lease_id: id("authority:lease:1"),
            authority_epoch: 7,
            sequence: 3,
            fencing_token: 9,
            scope: ScopeRef(id("scope:restaurant:a")),
            issued_at_unix_ms: 1_000,
            expires_at_unix_ms: 2_000,
        }
    }

    #[test]
    fn reference_ids_are_canonical() {
        assert!(ReferenceId::new("business:1").is_ok());
        assert_eq!(
            ReferenceId::new(" business:1"),
            Err(ReferenceIdError::NonCanonical)
        );
        assert_eq!(ReferenceId::new(""), Err(ReferenceIdError::Empty));
    }

    #[test]
    fn authority_epoch_change_requires_revalidation() {
        let lease = lease();
        assert!(lease.validate_at(1_500, 7).is_ok());
        assert_eq!(
            lease.validate_at(1_500, 8),
            Err(AuthorityLeaseError::AuthorityEpochChanged {
                bound: 7,
                current: 8,
            })
        );
    }

    #[test]
    fn prepared_action_cannot_outlive_authority_or_reservation() {
        let action = PreparedAction {
            action_contract: ActionContractRef {
                semantic_id: id("mycelix.procurement.place-order.v1"),
                digest: Digest32::repeat(1),
            },
            decision_capsule: id("decision:1"),
            intent_digest: Digest32::repeat(2),
            frontiers: DecisionFrontiers::default(),
            authority_lease: lease(),
            reservations: vec![ReservationRef {
                domain: id("finance"),
                reservation_id: ReservationId(id("reservation:finance:1")),
                subject: subject("budget:restaurant:a"),
                digest: Digest32::repeat(3),
                expires_at_unix_ms: 1_900,
            }],
            idempotency_key: id("execution:order:1"),
            prepared_at_unix_ms: 1_200,
            expires_at_unix_ms: 1_800,
        };
        assert!(action.validate_for_execution(1_500, 7).is_ok());

        let mut too_long = action.clone();
        too_long.expires_at_unix_ms = 1_950;
        assert!(matches!(
            too_long.validate(),
            Err(PreparedActionError::ReservationExpiresBeforeAction { .. })
        ));
    }

    #[test]
    fn delegation_can_only_attenuate_authority() {
        let parent = AuthorityScope {
            capabilities: BTreeSet::from([
                capability("procurement:read"),
                capability("procurement:place-order"),
            ]),
            subjects: BTreeSet::from([subject("restaurant:a")]),
            risk_ceiling: RiskClass::High,
            budget: BudgetLimit::Limited(5_000),
            expires_at_unix_ms: 10_000,
        };
        let child = AuthorityScope {
            capabilities: BTreeSet::from([capability("procurement:place-order")]),
            subjects: BTreeSet::from([subject("restaurant:a")]),
            risk_ceiling: RiskClass::Moderate,
            budget: BudgetLimit::Limited(2_000),
            expires_at_unix_ms: 9_000,
        };
        assert!(child.is_no_broader_than(&parent));

        let mut amplified = child.clone();
        amplified.budget = BudgetLimit::Limited(6_000);
        assert!(!amplified.is_no_broader_than(&parent));

        amplified = child;
        amplified
            .capabilities
            .insert(capability("treasury:borrow"));
        assert!(!amplified.is_no_broader_than(&parent));
    }

    #[test]
    fn sibling_budget_reservations_cannot_amplify_parent_envelope() {
        let mut envelope = SharedBudgetEnvelope::new(5_000);
        envelope.reserve(2_000).unwrap();
        envelope.reserve(3_000).unwrap();
        assert_eq!(envelope.available(), 0);
        assert_eq!(
            envelope.reserve(1),
            Err(BudgetReservationError::ExceedsEnvelope {
                requested: 1,
                available: 0,
            })
        );
    }

    #[test]
    fn degraded_epistemic_state_never_increases_autonomy() {
        let healthy = EpistemicHealth::default().autonomy_ceiling();
        let stale = EpistemicHealth {
            stale_required_sources: 1,
            ..EpistemicHealth::default()
        }
        .autonomy_ceiling();
        let conflicted = EpistemicHealth {
            unresolved_conflicts: 1,
            stale_required_sources: 1,
            ..EpistemicHealth::default()
        }
        .autonomy_ceiling();
        let missing = EpistemicHealth {
            missing_required_sources: 1,
            unresolved_conflicts: 1,
            stale_required_sources: 1,
            calibration_cold: true,
        }
        .autonomy_ceiling();

        assert!(stale <= healthy);
        assert!(conflicted <= stale);
        assert!(missing <= conflicted);
    }

    #[test]
    fn breaker_contracts_and_requires_explicit_rearm() {
        let mut breaker = CircuitBreaker::new(OperatingMode::Autonomous);
        breaker.trip_to(OperatingMode::RecommendOnly);
        assert_eq!(breaker.mode(), OperatingMode::RecommendOnly);
        assert!(breaker.is_tripped());

        breaker.trip_to(OperatingMode::ApprovalRequired);
        assert_eq!(breaker.mode(), OperatingMode::RecommendOnly);

        breaker.rearm(OperatingMode::ApprovalRequired, &lease(), 1_500, 7)
            .unwrap();
        assert_eq!(breaker.mode(), OperatingMode::ApprovalRequired);
        assert!(!breaker.is_tripped());
    }

    #[test]
    fn invariant_registry_is_complete_and_unique() {
        assert_eq!(BUSINESS_INVARIANTS.len(), 27);
        let unique = BUSINESS_INVARIANTS.into_iter().collect::<BTreeSet<_>>();
        assert_eq!(unique.len(), 27);
        assert!(unique.contains(&InvariantId("BIZ-I01")));
        assert!(unique.contains(&InvariantId("BIZ-I27")));
    }
}
