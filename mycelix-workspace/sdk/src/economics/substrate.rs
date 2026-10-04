// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! # Economic Substrate Integrity
//!
//! Reference model for preventing economic activity from silently consuming
//! the stocks that make future activity possible.
//!
//! The design deliberately separates measurement from judgement, state from
//! incentive, hard boundaries from soft indicators, and restoration/maintenance
//! from discretionary distribution.
//!
//! A substrate boundary is a constraint, not a score. No healthy dimension can
//! compensate for a breached hard boundary elsewhere. This avoids converting
//! ecological, social, epistemic, or institutional health into a single
//! optimizable number.
//!
//! This module is a pure-Rust reference model. It does not prescribe universal
//! boundary values, monetary prices for non-market assets, or governance policy.
//! Production implementations must bind observations to evidence and local
//! policy through the appropriate Mycelix zomes.

use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};

/// Canonical domains in which productive systems may hold substrate.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub enum SubstrateDimension {
    /// Liquid and reserve financial capacity.
    Financial,
    /// Physical infrastructure and productive capacity.
    Physical,
    /// Ecological condition and regenerative capacity.
    Ecological,
    /// Social relationships, participation, and reciprocal capacity.
    Social,
    /// Epistemic reliability, evidence quality, and knowledge integrity.
    Epistemic,
    /// Institutional legitimacy, accountability, and rule integrity.
    Institutional,
}

/// Boundary orientation for an observed substrate quantity.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum BoundaryDirection {
    /// Higher values are healthier; falling below the boundary is a breach.
    Minimum,
    /// Lower values are healthier; rising above the boundary is a breach.
    Maximum,
}

/// Health state of a single substrate account.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum SubstrateState {
    /// The account has positive safety headroom.
    Healthy,
    /// The account is inside the configured warning buffer.
    Warning,
    /// The configured boundary has been crossed.
    Breached,
}

/// A locally defined boundary for a substrate account.
///
/// boundary is expressed in the account's native units. warning_buffer is
/// measured in the same units and only controls early warning; it does not
/// change the hard boundary itself.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct SubstrateBoundary {
    /// Minimum or maximum orientation of the boundary.
    pub direction: BoundaryDirection,
    /// Boundary value in native units.
    pub boundary: i128,
    /// Early-warning distance from the boundary.
    pub warning_buffer: u128,
    /// Whether crossing the boundary has policy-level blocking effect.
    pub hard: bool,
}

impl SubstrateBoundary {
    /// Construct a lower-bound boundary.
    pub fn minimum(boundary: i128, warning_buffer: u128, hard: bool) -> Self {
        Self {
            direction: BoundaryDirection::Minimum,
            boundary,
            warning_buffer,
            hard,
        }
    }

    /// Construct an upper-bound boundary.
    pub fn maximum(boundary: i128, warning_buffer: u128, hard: bool) -> Self {
        Self {
            direction: BoundaryDirection::Maximum,
            boundary,
            warning_buffer,
            hard,
        }
    }

    /// Return signed headroom. Positive means distance remains before breach.
    pub fn headroom(&self, current: i128) -> i128 {
        match self.direction {
            BoundaryDirection::Minimum => current.saturating_sub(self.boundary),
            BoundaryDirection::Maximum => self.boundary.saturating_sub(current),
        }
    }

    /// Classify the current account state.
    pub fn classify(&self, current: i128) -> SubstrateState {
        let headroom = self.headroom(current);
        if headroom < 0 {
            SubstrateState::Breached
        } else if (headroom as u128) <= self.warning_buffer {
            SubstrateState::Warning
        } else {
            SubstrateState::Healthy
        }
    }
}

/// A substrate account tracked independently from financial balances.
///
/// The current value is updated by append-only events. Deliberate breaches
/// are recorded rather than hidden or rejected so the ledger remains an honest
/// record of system condition.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct SubstrateAccount {
    /// Dimension represented by this account.
    pub dimension: SubstrateDimension,
    /// Human-readable unit (e.g. m3, hours, index points).
    pub unit: String,
    /// Starting/reference stock used for interpretation and reporting.
    pub baseline: i128,
    /// Current stock or pressure value.
    pub current: i128,
    /// Applicable safety boundary.
    pub boundary: SubstrateBoundary,
    /// Last observation/event timestamp.
    pub updated_at: u64,
}

impl SubstrateAccount {
    /// Create a substrate account.
    pub fn new(
        dimension: SubstrateDimension,
        unit: impl Into<String>,
        baseline: i128,
        current: i128,
        boundary: SubstrateBoundary,
        updated_at: u64,
    ) -> Self {
        Self {
            dimension,
            unit: unit.into(),
            baseline,
            current,
            boundary,
            updated_at,
        }
    }

    /// Classify this account against its configured boundary.
    pub fn state(&self) -> SubstrateState {
        self.boundary.classify(self.current)
    }

    /// Return signed headroom to the boundary.
    pub fn headroom(&self) -> i128 {
        self.boundary.headroom(self.current)
    }
}

/// Why a substrate ledger entry occurred.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum SubstrateEventKind {
    /// New productive or regenerative capacity.
    Regeneration,
    /// Upkeep that preserves existing capacity.
    Maintenance,
    /// Direct consumption/depletion of substrate.
    Depletion,
    /// Cost transferred outside the actor's direct accounting boundary.
    ExternalizedImpact,
    /// Reconciliation or corrected measurement.
    Reconciliation,
}

/// Append-only substrate event.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct SubstrateEvent {
    /// Event identifier supplied by the caller.
    pub id: String,
    /// Affected dimension.
    pub dimension: SubstrateDimension,
    /// Signed change in native units.
    pub delta: i128,
    /// Semantic event kind.
    pub kind: SubstrateEventKind,
    /// Actor responsible for the event.
    pub actor: String,
    /// Event timestamp.
    pub timestamp: u64,
    /// Optional evidence/provenance reference.
    pub evidence_ref: Option<String>,
}

/// Policy scope for a proposed distribution or action.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum DistributionPurpose {
    /// Discretionary surplus extraction/distribution.
    Discretionary,
    /// Routine maintenance of productive capacity.
    Maintenance,
    /// Restoration following depletion or damage.
    Restoration,
    /// Time-bounded emergency response.
    Emergency,
}

/// Result of evaluating substrate conditions against a proposed action.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum GateDecision {
    /// No warnings or breaches are present.
    Allowed,
    /// The action may proceed, but substrate warnings must remain visible.
    AllowedWithWarning,
    /// The action is blocked because required evidence is missing.
    InsufficientEvidence,
    /// The action is blocked because a hard boundary is breached.
    Blocked,
    /// Emergency action may proceed only through explicit escalation.
    EmergencyEscalationRequired,
}

/// Aggregate substrate state. This intentionally uses worst-case logic rather
/// than averaging dimensions into a composite health score.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct SubstrateReport {
    /// Status by required dimension.
    pub dimensions: BTreeMap<SubstrateDimension, SubstrateState>,
    /// Number of hard-boundary breaches.
    pub hard_breaches: usize,
    /// Number of warnings, including soft-boundary warnings.
    pub warnings: usize,
    /// Required dimensions for which no account exists.
    pub missing: BTreeSet<SubstrateDimension>,
}

impl SubstrateReport {
    /// Return true when any hard boundary is breached.
    pub fn has_hard_breach(&self) -> bool {
        self.hard_breaches > 0
    }

    /// Return true when every required dimension is represented.
    pub fn complete(&self) -> bool {
        self.missing.is_empty()
    }
}

/// Collection of substrate accounts plus their append-only evidence ledger.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct SubstrateLedger {
    /// Current account state by dimension.
    accounts: BTreeMap<SubstrateDimension, SubstrateAccount>,
    /// Append-only economic/substrate events.
    events: Vec<SubstrateEvent>,
}

impl SubstrateLedger {
    /// Create an empty substrate ledger.
    pub fn new() -> Self {
        Self::default()
    }

    /// Register an account definition for a dimension.
    ///
    /// Existing definitions cannot be silently replaced. A boundary change is
    /// a governance event in production systems and must therefore be modeled
    /// explicitly rather than smuggled in through account registration.
    pub fn register_account(&mut self, account: SubstrateAccount) -> Result<(), String> {
        if let Some(existing) = self.accounts.get(&account.dimension) {
            if existing == &account {
                return Ok(());
            }
            return Err(format!(
                "Substrate account {:?} already exists; boundary changes require explicit versioning",
                account.dimension
            ));
        }

        self.accounts.insert(account.dimension, account);
        Ok(())
    }

    /// Read an account by dimension.
    pub fn account(&self, dimension: SubstrateDimension) -> Option<&SubstrateAccount> {
        self.accounts.get(&dimension)
    }

    /// Read the append-only event history.
    pub fn events(&self) -> &[SubstrateEvent] {
        &self.events
    }

    /// Record a substrate event.
    ///
    /// Boundary crossings are intentionally not rejected: hiding a bad
    /// measurement or depletion event would corrupt the feedback loop intended
    /// to detect corrosion.
    pub fn record_event(&mut self, event: SubstrateEvent) -> Result<(), String> {
        let account = self
            .accounts
            .get_mut(&event.dimension)
            .ok_or_else(|| format!("No substrate account for {:?}", event.dimension))?;

        if self.events.iter().any(|existing| existing.id == event.id) {
            return Err(format!("Duplicate substrate event id: {}", event.id));
        }

        account.current = account
            .current
            .checked_add(event.delta)
            .ok_or_else(|| "Substrate account overflow".to_string())?;
        account.updated_at = account.updated_at.max(event.timestamp);
        self.events.push(event);

        Ok(())
    }

    /// Evaluate all required substrate dimensions.
    pub fn report(&self, required: &[SubstrateDimension]) -> SubstrateReport {
        let mut dimensions = BTreeMap::new();
        let mut missing = BTreeSet::new();
        let mut hard_breaches = 0;
        let mut warnings = 0;

        let required: BTreeSet<SubstrateDimension> = required.iter().copied().collect();

        for dimension in required {
            match self.accounts.get(&dimension) {
                Some(account) => {
                    let state = account.state();
                    if state == SubstrateState::Warning {
                        warnings += 1;
                    }
                    if state == SubstrateState::Breached {
                        warnings += 1;
                        if account.boundary.hard {
                            hard_breaches += 1;
                        }
                    }
                    dimensions.insert(*dimension, state);
                }
                None => {
                    missing.insert(*dimension);
                }
            }
        }

        SubstrateReport {
            dimensions,
            hard_breaches,
            warnings,
            missing,
        }
    }

    /// Gate an action against the required substrate dimensions.
    ///
    /// Maintenance and restoration remain available during substrate breach:
    /// blocking repair would turn a correctable deficit into permanent decay.
    /// Emergency operations have an explicit escalation path rather than a
    /// silent bypass.
    pub fn gate(
        &self,
        required: &[SubstrateDimension],
        purpose: DistributionPurpose,
    ) -> GateDecision {
        let report = self.report(required);

        if !report.complete() {
            return GateDecision::InsufficientEvidence;
        }

        match purpose {
            DistributionPurpose::Maintenance | DistributionPurpose::Restoration => {
                if report.warnings > 0 {
                    GateDecision::AllowedWithWarning
                } else {
                    GateDecision::Allowed
                }
            }
            DistributionPurpose::Emergency => {
                if report.has_hard_breach() {
                    GateDecision::EmergencyEscalationRequired
                } else if report.warnings > 0 {
                    GateDecision::AllowedWithWarning
                } else {
                    GateDecision::Allowed
                }
            }
            DistributionPurpose::Discretionary => {
                if report.has_hard_breach() {
                    GateDecision::Blocked
                } else if report.warnings > 0 {
                    GateDecision::AllowedWithWarning
                } else {
                    GateDecision::Allowed
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn financial_account(current: i128) -> SubstrateAccount {
        SubstrateAccount::new(
            SubstrateDimension::Financial,
            "SAP",
            1_000,
            current,
            SubstrateBoundary::minimum(800, 100, true),
            1_000,
        )
    }

    fn ecological_account(current: i128) -> SubstrateAccount {
        SubstrateAccount::new(
            SubstrateDimension::Ecological,
            "condition-index",
            900,
            current,
            SubstrateBoundary::minimum(700, 100, true),
            1_000,
        )
    }

    #[test]
    fn hard_breach_is_non_compensable() {
        let mut ledger = SubstrateLedger::new();
        ledger.register_account(financial_account(1_000)).unwrap();
        ledger.register_account(ecological_account(600)).unwrap();

        let required = [
            SubstrateDimension::Financial,
            SubstrateDimension::Ecological,
        ];

        let report = ledger.report(&required);

        assert_eq!(
            report.dimensions[&SubstrateDimension::Financial],
            SubstrateState::Healthy
        );
        assert_eq!(
            report.dimensions[&SubstrateDimension::Ecological],
            SubstrateState::Breached
        );
        assert_eq!(report.hard_breaches, 1);
        assert_eq!(
            ledger.gate(&required, DistributionPurpose::Discretionary),
            GateDecision::Blocked
        );
    }

    #[test]
    fn warning_does_not_become_a_hidden_block() {
        let mut ledger = SubstrateLedger::new();
        ledger.register_account(financial_account(850)).unwrap();

        let required = [SubstrateDimension::Financial];
        assert_eq!(
            ledger.gate(&required, DistributionPurpose::Discretionary),
            GateDecision::AllowedWithWarning
        );
    }

    #[test]
    fn breached_state_is_recorded_instead_of_hidden() {
        let mut ledger = SubstrateLedger::new();
        ledger.register_account(financial_account(900)).unwrap();

        ledger
            .record_event(SubstrateEvent {
                id: "evt-1".into(),
                dimension: SubstrateDimension::Financial,
                delta: -150,
                kind: SubstrateEventKind::Depletion,
                actor: "did:example:actor".into(),
                timestamp: 2_000,
                evidence_ref: Some("evidence:evt-1".into()),
            })
            .unwrap();

        assert_eq!(
            ledger
                .account(SubstrateDimension::Financial)
                .unwrap()
                .current,
            750
        );
        assert_eq!(
            ledger
                .account(SubstrateDimension::Financial)
                .unwrap()
                .state(),
            SubstrateState::Breached
        );
        assert_eq!(ledger.events().len(), 1);
    }

    #[test]
    fn missing_required_dimension_blocks_discretionary_action() {
        let mut ledger = SubstrateLedger::new();
        ledger.register_account(financial_account(1_000)).unwrap();

        let required = [
            SubstrateDimension::Financial,
            SubstrateDimension::Ecological,
        ];

        assert_eq!(
            ledger.gate(&required, DistributionPurpose::Discretionary),
            GateDecision::InsufficientEvidence
        );
    }

    #[test]
    fn maintenance_and_restoration_remain_possible_during_breach() {
        let mut ledger = SubstrateLedger::new();
        ledger.register_account(ecological_account(600)).unwrap();

        let required = [SubstrateDimension::Ecological];

        assert_eq!(
            ledger.gate(&required, DistributionPurpose::Maintenance),
            GateDecision::AllowedWithWarning
        );
        assert_eq!(
            ledger.gate(&required, DistributionPurpose::Restoration),
            GateDecision::AllowedWithWarning
        );
    }

    #[test]
    fn emergency_requires_explicit_escalation_when_hard_boundary_breached() {
        let mut ledger = SubstrateLedger::new();
        ledger.register_account(ecological_account(600)).unwrap();

        let required = [SubstrateDimension::Ecological];

        assert_eq!(
            ledger.gate(&required, DistributionPurpose::Emergency),
            GateDecision::EmergencyEscalationRequired
        );
    }

    #[test]
    fn oversized_warning_buffer_remains_a_warning() {
        let account = SubstrateAccount::new(
            SubstrateDimension::Ecological,
            "index",
            0,
            1,
            SubstrateBoundary::minimum(0, u128::MAX, true),
            1_000,
        );

        assert_eq!(account.state(), SubstrateState::Warning);
    }

    #[test]
    fn maximum_boundary_has_correct_orientation() {
        let account = SubstrateAccount::new(
            SubstrateDimension::Ecological,
            "tons-co2e",
            100,
            90,
            SubstrateBoundary::maximum(100, 10, true),
            1_000,
        );

        assert_eq!(account.state(), SubstrateState::Warning);

        let breached = SubstrateAccount {
            current: 120,
            ..account
        };
        assert_eq!(breached.state(), SubstrateState::Breached);
    }

    #[test]
    fn required_dimensions_are_set_semantically() {
        let mut ledger = SubstrateLedger::new();
        ledger.register_account(financial_account(750)).unwrap();

        let required = [
            SubstrateDimension::Financial,
            SubstrateDimension::Financial,
        ];
        let report = ledger.report(&required);

        assert_eq!(report.warnings, 1);
        assert_eq!(report.hard_breaches, 1);
    }

    #[test]
    fn boundary_definition_cannot_be_silently_replaced() {
        let mut ledger = SubstrateLedger::new();
        ledger.register_account(financial_account(1_000)).unwrap();

        let changed = financial_account(1_000);
        assert!(ledger.register_account(changed).is_ok());

        let mut altered = financial_account(1_000);
        altered.boundary = SubstrateBoundary::minimum(500, 50, false);

        let result = ledger.register_account(altered);
        assert!(result.is_err());
        assert_eq!(
            ledger
                .account(SubstrateDimension::Financial)
                .unwrap()
                .boundary
                .boundary,
            800
        );
    }

    #[test]
    fn duplicate_event_ids_are_rejected_without_mutation() {
        let mut ledger = SubstrateLedger::new();
        ledger.register_account(financial_account(1_000)).unwrap();

        let event = SubstrateEvent {
            id: "evt-duplicate".into(),
            dimension: SubstrateDimension::Financial,
            delta: -50,
            kind: SubstrateEventKind::Depletion,
            actor: "did:example:actor".into(),
            timestamp: 2_000,
            evidence_ref: None,
        };

        ledger.record_event(event.clone()).unwrap();
        let second = ledger.record_event(event);
        assert!(second.is_err());
        assert_eq!(
            ledger
                .account(SubstrateDimension::Financial)
                .unwrap()
                .current,
            950
        );
        assert_eq!(ledger.events().len(), 1);
    }

    #[test]
    fn latest_timestamp_is_processing_order_invariant() {
        let mut first = SubstrateLedger::new();
        let mut second = SubstrateLedger::new();
        first.register_account(financial_account(1_000)).unwrap();
        second.register_account(financial_account(1_000)).unwrap();

        let earlier = SubstrateEvent {
            id: "evt-earlier".into(),
            dimension: SubstrateDimension::Financial,
            delta: -10,
            kind: SubstrateEventKind::Depletion,
            actor: "did:example:a".into(),
            timestamp: 2_000,
            evidence_ref: None,
        };
        let later = SubstrateEvent {
            id: "evt-later".into(),
            dimension: SubstrateDimension::Financial,
            delta: 20,
            kind: SubstrateEventKind::Regeneration,
            actor: "did:example:b".into(),
            timestamp: 3_000,
            evidence_ref: None,
        };

        first.record_event(earlier.clone()).unwrap();
        first.record_event(later.clone()).unwrap();

        second.record_event(later).unwrap();
        second.record_event(earlier).unwrap();

        let a = first.account(SubstrateDimension::Financial).unwrap();
        let b = second.account(SubstrateDimension::Financial).unwrap();
        assert_eq!(a.current, b.current);
        assert_eq!(a.updated_at, b.updated_at);
    }

    proptest::proptest! {
        #[test]
        fn signed_event_addition_is_order_invariant(deltas in proptest::collection::vec(-100i64..=100i64, 0..32)) {
            let mut forward = SubstrateLedger::new();
            let mut reverse = SubstrateLedger::new();
            forward.register_account(financial_account(10_000)).unwrap();
            reverse.register_account(financial_account(10_000)).unwrap();

            let events: Vec<SubstrateEvent> = deltas
                .iter()
                .enumerate()
                .map(|(i, delta)| SubstrateEvent {
                    id: format!("evt-{i}"),
                    dimension: SubstrateDimension::Financial,
                    delta: *delta as i128,
                    kind: if *delta < 0 {
                        SubstrateEventKind::Depletion
                    } else {
                        SubstrateEventKind::Regeneration
                    },
                    actor: format!("did:example:{i}"),
                    timestamp: 2_000 + i as u64,
                    evidence_ref: None,
                })
                .collect();

            for event in &events {
                forward.record_event(event.clone()).unwrap();
            }
            for event in events.iter().rev() {
                reverse.record_event(event.clone()).unwrap();
            }

            proptest::prop_assert_eq!(
                forward.account(SubstrateDimension::Financial).unwrap().current,
                reverse.account(SubstrateDimension::Financial).unwrap().current
            );
            proptest::prop_assert_eq!(
                forward.account(SubstrateDimension::Financial).unwrap().updated_at,
                reverse.account(SubstrateDimension::Financial).unwrap().updated_at
            );
        }
    }
}
