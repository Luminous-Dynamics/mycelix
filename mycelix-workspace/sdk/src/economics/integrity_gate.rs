// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! # Economic Integrity Gate
//!
//! Composition boundary for the economic substrate and impact/reciprocity
//! ledgers.
//!
//! AC-017 and AC-018 deliberately expose independent gates because they solve
//! different problems. AC-025 makes their composition explicit: an economic
//! action is evaluated against both layers, and the combined decision is never
//! more permissive than either component decision.
//!
//! The assessment preserves both component decisions so callers can audit the
//! exact reason for a block, warning, evidence failure, or escalation.
//!
//! This is a pure-Rust reference model. Production policy must still determine
//! which substrate dimensions are relevant to a given action.

use super::{
    impact::{ImpactGateDecision, ImpactLedger},
    substrate::{DistributionPurpose, GateDecision, SubstrateDimension, SubstrateLedger},
};
use serde::{Deserialize, Serialize};

/// Combined result of evaluating both economic integrity layers.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum EconomicIntegrityDecision {
    /// Both integrity layers allow the action without warning.
    Allowed,
    /// The action may proceed, but at least one integrity layer reports a
    /// warning/open-impact condition.
    AllowedWithWarning,
    /// At least one layer lacks the evidence required for a clean decision.
    InsufficientEvidence,
    /// At least one layer blocks the action.
    Blocked,
    /// At least one layer requires explicit emergency escalation.
    EmergencyEscalationRequired,
}

/// Auditable assessment retaining both component decisions.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct EconomicIntegrityAssessment {
    /// Result from the AC-017 substrate gate.
    pub substrate: GateDecision,
    /// Result from the AC-018 impact/reciprocity gate.
    pub impact: ImpactGateDecision,
    /// Monotone combined decision derived from both results.
    pub decision: EconomicIntegrityDecision,
}

impl EconomicIntegrityAssessment {
    /// Whether the combined policy permits ordinary execution.
    pub is_allowed(&self) -> bool {
        matches!(
            self.decision,
            EconomicIntegrityDecision::Allowed
                | EconomicIntegrityDecision::AllowedWithWarning
        )
    }

    /// Whether the action needs explicit escalation rather than ordinary
    /// execution.
    pub requires_escalation(&self) -> bool {
        self.decision == EconomicIntegrityDecision::EmergencyEscalationRequired
    }

    /// Whether at least one component is blocking or otherwise prevents a
    /// clean ordinary execution.
    pub is_blocked(&self) -> bool {
        matches!(
            self.decision,
            EconomicIntegrityDecision::InsufficientEvidence
                | EconomicIntegrityDecision::Blocked
                | EconomicIntegrityDecision::EmergencyEscalationRequired
        )
    }
}

/// Stateless composition gate for AC-017 and AC-018.
#[derive(Debug, Default, Clone, Copy)]
pub struct EconomicIntegrityGate;

impl EconomicIntegrityGate {
    /// Evaluate an economic action against substrate and impact reciprocity.
    ///
    /// The two ledgers are always evaluated independently first. The combined
    /// result then applies a fail-closed severity ordering:
    ///
    /// emergency escalation > block > insufficient evidence > warning > allow.
    ///
    /// This ordering never lets an independently blocking condition disappear
    /// because the other ledger is healthy.
    pub fn assess(
        substrate: &SubstrateLedger,
        impacts: &ImpactLedger,
        required_dimensions: &[SubstrateDimension],
        purpose: DistributionPurpose,
    ) -> EconomicIntegrityAssessment {
        let substrate = substrate.gate(required_dimensions, purpose);
        let impact = impacts.gate(purpose);
        let decision = Self::combine(substrate, impact);

        EconomicIntegrityAssessment {
            substrate,
            impact,
            decision,
        }
    }

    /// Combine independently evaluated decisions without losing their detail.
    pub fn combine(
        substrate: GateDecision,
        impact: ImpactGateDecision,
    ) -> EconomicIntegrityDecision {
        if substrate == GateDecision::EmergencyEscalationRequired
            || impact == ImpactGateDecision::EmergencyEscalationRequired
        {
            return EconomicIntegrityDecision::EmergencyEscalationRequired;
        }

        if substrate == GateDecision::Blocked || impact == ImpactGateDecision::BlockedByObligation {
            return EconomicIntegrityDecision::Blocked;
        }

        if substrate == GateDecision::InsufficientEvidence
            || impact == ImpactGateDecision::InsufficientAttribution
        {
            return EconomicIntegrityDecision::InsufficientEvidence;
        }

        if substrate == GateDecision::AllowedWithWarning
            || impact == ImpactGateDecision::AllowedWithOpenImpacts
        {
            return EconomicIntegrityDecision::AllowedWithWarning;
        }

        EconomicIntegrityDecision::Allowed
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::economics::impact::{
        AttributionBasis, ImpactAttribution, ImpactDirection, ImpactStatus, SubstrateImpact,
    };
    use crate::economics::substrate::{SubstrateAccount, SubstrateBoundary, SubstrateEventKind};

    fn healthy_substrate() -> SubstrateLedger {
        let mut ledger = SubstrateLedger::new();
        ledger
            .register_account(SubstrateAccount::new(
                SubstrateDimension::Financial,
                "SAP",
                1_000,
                1_000,
                SubstrateBoundary::minimum(800, 100, true),
                1_000,
            ))
            .unwrap();
        ledger
    }

    fn warning_substrate() -> SubstrateLedger {
        let mut ledger = healthy_substrate();
        ledger
            .register_account(SubstrateAccount::new(
                SubstrateDimension::Ecological,
                "condition-index",
                900,
                750,
                SubstrateBoundary::minimum(700, 100, true),
                1_000,
            ))
            .unwrap();
        ledger
    }

    fn breached_substrate() -> SubstrateLedger {
        let mut ledger = healthy_substrate();
        ledger
            .register_account(SubstrateAccount::new(
                SubstrateDimension::Ecological,
                "condition-index",
                900,
                600,
                SubstrateBoundary::minimum(700, 100, true),
                1_000,
            ))
            .unwrap();
        ledger
    }

    fn open_depletion() -> SubstrateImpact {
        SubstrateImpact {
            id: "impact-1".into(),
            action_actor: "did:example:actor".into(),
            action_ref: "action:1".into(),
            dimension: SubstrateDimension::Ecological,
            unit: "m3".into(),
            magnitude: 1_000,
            direction: ImpactDirection::Depletion,
            affected_ref: "river:1".into(),
            attributions: vec![],
            evidence_refs: vec!["evidence:1".into()],
            status: ImpactStatus::Open,
            obligation_id: None,
            recorded_at: 1_000,
        }
    }

    fn blocked_impact() -> ImpactLedger {
        let mut ledger = ImpactLedger::new();
        ledger.record_impact(open_depletion()).unwrap();
        ledger
            .attribute_impact(
                "impact-1",
                vec![ImpactAttribution {
                    actor: "did:example:actor".into(),
                    basis: AttributionBasis::Direct,
                    share_bps: 10_000,
                    evidence_ref: Some("evidence:actor".into()),
                }],
                Some("obligation-1".into()),
                Some(5_000),
            )
            .unwrap();
        ledger
    }

    #[test]
    fn healthy_layers_allow_discretionary_action() {
        let substrate = healthy_substrate();
        let impacts = ImpactLedger::new();

        let assessment = EconomicIntegrityGate::assess(
            &substrate,
            &impacts,
            &[SubstrateDimension::Financial],
            DistributionPurpose::Discretionary,
        );

        assert_eq!(assessment.substrate, GateDecision::Allowed);
        assert_eq!(assessment.impact, ImpactGateDecision::Allowed);
        assert_eq!(assessment.decision, EconomicIntegrityDecision::Allowed);
        assert!(assessment.is_allowed());
        assert!(!assessment.is_blocked());
    }

    #[test]
    fn substrate_block_cannot_be_compensated_by_clean_impact_state() {
        let substrate = breached_substrate();
        let impacts = ImpactLedger::new();

        let assessment = EconomicIntegrityGate::assess(
            &substrate,
            &impacts,
            &[
                SubstrateDimension::Financial,
                SubstrateDimension::Ecological,
            ],
            DistributionPurpose::Discretionary,
        );

        assert_eq!(assessment.substrate, GateDecision::Blocked);
        assert_eq!(assessment.impact, ImpactGateDecision::Allowed);
        assert_eq!(assessment.decision, EconomicIntegrityDecision::Blocked);
    }

    #[test]
    fn impact_obligation_cannot_be_compensated_by_healthy_substrate() {
        let substrate = healthy_substrate();
        let impacts = blocked_impact();

        let assessment = EconomicIntegrityGate::assess(
            &substrate,
            &impacts,
            &[SubstrateDimension::Financial],
            DistributionPurpose::Discretionary,
        );

        assert_eq!(assessment.substrate, GateDecision::Allowed);
        assert_eq!(assessment.impact, ImpactGateDecision::BlockedByObligation);
        assert_eq!(assessment.decision, EconomicIntegrityDecision::Blocked);
    }

    #[test]
    fn evidence_failure_cannot_be_hidden_by_healthy_other_layer() {
        let substrate = healthy_substrate();
        let mut impacts = ImpactLedger::new();
        impacts.record_impact(open_depletion()).unwrap();

        let assessment = EconomicIntegrityGate::assess(
            &substrate,
            &impacts,
            &[SubstrateDimension::Financial],
            DistributionPurpose::Discretionary,
        );

        assert_eq!(assessment.substrate, GateDecision::Allowed);
        assert_eq!(
            assessment.impact,
            ImpactGateDecision::InsufficientAttribution
        );
        assert_eq!(
            assessment.decision,
            EconomicIntegrityDecision::InsufficientEvidence
        );
    }

    #[test]
    fn warnings_are_visible_but_do_not_become_hidden_blocks() {
        let substrate = warning_substrate();
        let impacts = ImpactLedger::new();

        let assessment = EconomicIntegrityGate::assess(
            &substrate,
            &impacts,
            &[
                SubstrateDimension::Financial,
                SubstrateDimension::Ecological,
            ],
            DistributionPurpose::Discretionary,
        );

        assert_eq!(assessment.substrate, GateDecision::AllowedWithWarning);
        assert_eq!(assessment.impact, ImpactGateDecision::Allowed);
        assert_eq!(
            assessment.decision,
            EconomicIntegrityDecision::AllowedWithWarning
        );
        assert!(assessment.is_allowed());
    }

    #[test]
    fn open_impacts_produce_a_visible_warning_for_maintenance() {
        let substrate = healthy_substrate();
        let mut impacts = ImpactLedger::new();
        impacts.record_impact(open_depletion()).unwrap();

        let assessment = EconomicIntegrityGate::assess(
            &substrate,
            &impacts,
            &[SubstrateDimension::Financial],
            DistributionPurpose::Maintenance,
        );

        assert_eq!(assessment.substrate, GateDecision::Allowed);
        assert_eq!(assessment.impact, ImpactGateDecision::AllowedWithOpenImpacts);
        assert_eq!(
            assessment.decision,
            EconomicIntegrityDecision::AllowedWithWarning
        );
    }

    #[test]
    fn emergency_escalation_has_precedence_over_ordinary_blocks() {
        let substrate = breached_substrate();
        let impacts = blocked_impact();

        let assessment = EconomicIntegrityGate::assess(
            &substrate,
            &impacts,
            &[
                SubstrateDimension::Financial,
                SubstrateDimension::Ecological,
            ],
            DistributionPurpose::Emergency,
        );

        assert_eq!(
            assessment.substrate,
            GateDecision::EmergencyEscalationRequired
        );
        assert_eq!(
            assessment.impact,
            ImpactGateDecision::EmergencyEscalationRequired
        );
        assert_eq!(
            assessment.decision,
            EconomicIntegrityDecision::EmergencyEscalationRequired
        );
        assert!(assessment.requires_escalation());
    }

    #[test]
    fn combine_is_order_independent_and_fail_closed() {
        let a = EconomicIntegrityGate::combine(
            GateDecision::Blocked,
            ImpactGateDecision::InsufficientAttribution,
        );
        let b = EconomicIntegrityGate::combine(
            GateDecision::InsufficientEvidence,
            ImpactGateDecision::BlockedByObligation,
        );

        assert_eq!(a, EconomicIntegrityDecision::Blocked);
        assert_eq!(b, EconomicIntegrityDecision::Blocked);
    }

    #[test]
    fn duplicate_required_dimensions_do_not_change_composition() {
        let substrate = healthy_substrate();
        let impacts = ImpactLedger::new();

        let assessment = EconomicIntegrityGate::assess(
            &substrate,
            &impacts,
            &[
                SubstrateDimension::Financial,
                SubstrateDimension::Financial,
            ],
            DistributionPurpose::Discretionary,
        );

        assert_eq!(assessment.decision, EconomicIntegrityDecision::Allowed);
    }

    #[test]
    fn imported_event_kind_remains_available_to_reference_helpers() {
        assert_eq!(
            SubstrateEventKind::Depletion,
            SubstrateEventKind::Depletion
        );
    }
}
