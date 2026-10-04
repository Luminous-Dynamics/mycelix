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

/// Versioned declaration of the integrity scope an action claims to operate under.
///
/// Scope is an explicit policy artifact, not an implicit argument assembled at
/// each call site. The attestation reference can point to an externally signed
/// or otherwise governed record; this reference model does not prescribe a
/// particular signature system.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct EconomicActionScope {
    /// Stable scope declaration identifier.
    pub scope_id: String,
    /// Stable action/procurement/payment identifier.
    pub action_ref: String,
    /// Intended action purpose.
    pub purpose: DistributionPurpose,
    /// Required substrate dimensions, in strict canonical order.
    pub required_dimensions: Vec<SubstrateDimension>,
    /// Policy defining why these dimensions are in scope.
    pub policy_ref: String,
    /// Authority responsible for the scope declaration.
    pub authority_ref: String,
    /// External attestation/proof reference.
    pub attestation_ref: String,
    /// Evidence references supporting scope selection.
    pub evidence_refs: Vec<String>,
    /// Declaration timestamp.
    pub declared_at: u64,
}

impl EconomicActionScope {
    /// Validate the scope declaration without inventing missing policy.
    pub fn validate(&self) -> Result<(), String> {
        if self.scope_id.trim().is_empty() {
            return Err("Economic scope ID cannot be empty".into());
        }
        if self.action_ref.trim().is_empty() {
            return Err("Economic scope action reference cannot be empty".into());
        }
        if self.required_dimensions.is_empty() {
            return Err("Economic scope requires at least one substrate dimension".into());
        }
        if self
            .required_dimensions
            .windows(2)
            .any(|pair| pair[0] >= pair[1])
        {
            return Err(
                "Economic scope dimensions must be strictly sorted and unique".into()
            );
        }
        if self.policy_ref.trim().is_empty() {
            return Err("Economic scope policy reference cannot be empty".into());
        }
        if self.authority_ref.trim().is_empty() {
            return Err("Economic scope authority reference cannot be empty".into());
        }
        if self.attestation_ref.trim().is_empty() {
            return Err("Economic scope attestation reference cannot be empty".into());
        }
        if self
            .evidence_refs
            .iter()
            .any(|reference| reference.trim().is_empty())
        {
            return Err("Economic scope evidence references cannot be empty".into());
        }
        Ok(())
    }
}

/// Auditable result of evaluating a declared action scope.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ScopedEconomicIntegrityAssessment {
    /// Scope declaration identifier.
    pub scope_id: String,
    /// Action identifier covered by the scope.
    pub action_ref: String,
    /// Known impact IDs covered by the scope declaration.
    pub covered_impact_ids: Vec<String>,
    /// The underlying AC-017/AC-018 assessment.
    pub assessment: EconomicIntegrityAssessment,
}

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

    /// Evaluate an explicitly attested action scope.
    ///
    /// The scope is validated first, then checked against known impacts for the
    /// action. A known impacted dimension cannot be omitted from the declaration.
    pub fn assess_scoped(
        substrate: &SubstrateLedger,
        impacts: &ImpactLedger,
        scope: &EconomicActionScope,
    ) -> Result<ScopedEconomicIntegrityAssessment, String> {
        scope.validate()?;

        let covered_impact_ids =
            impacts.validate_action_scope(&scope.action_ref, &scope.required_dimensions)?;

        let assessment = Self::assess(
            substrate,
            impacts,
            &scope.required_dimensions,
            scope.purpose,
        );

        Ok(ScopedEconomicIntegrityAssessment {
            scope_id: scope.scope_id.clone(),
            action_ref: scope.action_ref.clone(),
            covered_impact_ids,
            assessment,
        })
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
    use crate::economics::substrate::{SubstrateAccount, SubstrateBoundary};

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

    fn valid_scope(action_ref: &str, purpose: DistributionPurpose) -> EconomicActionScope {
        EconomicActionScope {
            scope_id: "scope-1".into(),
            action_ref: action_ref.into(),
            purpose,
            required_dimensions: vec![SubstrateDimension::Financial, SubstrateDimension::Ecological],
            policy_ref: "policy:scope-v1".into(),
            authority_ref: "authority:dao-1".into(),
            attestation_ref: "attestation:scope-1".into(),
            evidence_refs: vec!["evidence:scope-1".into()],
            declared_at: 1_000,
        }
    }

    #[test]
    fn scoped_assessment_preserves_scope_and_known_impact_coverage() {
        let substrate = warning_substrate();
        let impacts = {
            let mut ledger = ImpactLedger::new();
            ledger.record_impact(open_depletion()).unwrap();
            ledger
        };
        let assessment = EconomicIntegrityGate::assess_scoped(
            &substrate,
            &impacts,
            &valid_scope("action:1", DistributionPurpose::Discretionary),
        )
        .unwrap();

        assert_eq!(assessment.scope_id, "scope-1");
        assert_eq!(assessment.action_ref, "action:1");
        assert_eq!(assessment.covered_impact_ids, vec!["impact-1"]);
        assert_eq!(
            assessment.assessment.decision,
            EconomicIntegrityDecision::InsufficientEvidence
        );
    }

    #[test]
    fn scoped_assessment_rejects_omitted_known_impact_dimension() {
        let substrate = healthy_substrate();
        let mut impacts = ImpactLedger::new();
        impacts.record_impact(open_depletion()).unwrap();

        let mut scope = valid_scope("action:1", DistributionPurpose::Discretionary);
        scope.required_dimensions = vec![SubstrateDimension::Financial];

        let result = EconomicIntegrityGate::assess_scoped(&substrate, &impacts, &scope);
        assert!(result.is_err());
        assert!(result
            .unwrap_err()
            .contains("omits known impact dimension"));
    }

    #[test]
    fn malformed_scope_is_rejected_before_assessment() {
        let substrate = healthy_substrate();
        let impacts = ImpactLedger::new();
        let mut scope = valid_scope("action:empty", DistributionPurpose::Discretionary);
        scope.attestation_ref.clear();

        let result = EconomicIntegrityGate::assess_scoped(&substrate, &impacts, &scope);
        assert!(result.is_err());
        assert!(result.unwrap_err().contains("attestation reference"));
    }

    #[test]
    fn scope_dimension_order_is_semantically_strict() {
        let mut scope = valid_scope("action:order", DistributionPurpose::Discretionary);
        scope.required_dimensions = vec![
            SubstrateDimension::Ecological,
            SubstrateDimension::Financial,
        ];

        assert!(scope.validate().is_err());

        scope.required_dimensions = vec![
            SubstrateDimension::Financial,
            SubstrateDimension::Financial,
        ];
        assert!(scope.validate().is_err());
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

    fn decision_rank(decision: EconomicIntegrityDecision) -> u8 {
        match decision {
            EconomicIntegrityDecision::Allowed => 0,
            EconomicIntegrityDecision::AllowedWithWarning => 1,
            EconomicIntegrityDecision::InsufficientEvidence => 2,
            EconomicIntegrityDecision::Blocked => 3,
            EconomicIntegrityDecision::EmergencyEscalationRequired => 4,
        }
    }

    fn substrate_rank(decision: GateDecision) -> u8 {
        match decision {
            GateDecision::Allowed => 0,
            GateDecision::AllowedWithWarning => 1,
            GateDecision::InsufficientEvidence => 2,
            GateDecision::Blocked => 3,
            GateDecision::EmergencyEscalationRequired => 4,
        }
    }

    fn impact_rank(decision: ImpactGateDecision) -> u8 {
        match decision {
            ImpactGateDecision::Allowed => 0,
            ImpactGateDecision::AllowedWithOpenImpacts => 1,
            ImpactGateDecision::InsufficientAttribution => 2,
            ImpactGateDecision::BlockedByObligation => 3,
            ImpactGateDecision::EmergencyEscalationRequired => 4,
        }
    }

    #[test]
    fn combined_decision_is_never_more_permissive_than_either_layer() {
        let substrate_decisions = [
            GateDecision::Allowed,
            GateDecision::AllowedWithWarning,
            GateDecision::InsufficientEvidence,
            GateDecision::Blocked,
            GateDecision::EmergencyEscalationRequired,
        ];
        let impact_decisions = [
            ImpactGateDecision::Allowed,
            ImpactGateDecision::AllowedWithOpenImpacts,
            ImpactGateDecision::InsufficientAttribution,
            ImpactGateDecision::BlockedByObligation,
            ImpactGateDecision::EmergencyEscalationRequired,
        ];

        for substrate in substrate_decisions {
            for impact in impact_decisions {
                let combined = EconomicIntegrityGate::combine(substrate, impact);
                let rank = decision_rank(combined);

                assert!(rank >= substrate_rank(substrate));
                assert!(rank >= impact_rank(impact));
            }
        }
    }
}
