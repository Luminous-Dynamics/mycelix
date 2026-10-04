// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! # Substrate Impact and Reciprocity
//!
//! Reference model for carrying economic impacts from action -> affected
//! substrate -> attributable actors -> restoration obligation.
//!
//! The model intentionally does not assign a universal monetary price to
//! externalities. It keeps impacts in native units and preserves uncertainty.
//!
//! Core invariant:
//!
//! unknown attribution != zero impact
//!
//! An impact may be partially attributable, challenged, or unresolved while
//! retaining its evidence history. A discretionary distribution cannot erase
//! an unresolved mandatory restoration obligation.
//!
//! This module is a pure-Rust reference model. Production implementations
//! should bind records to DKG provenance, identity, local policy, and the
//! relevant Commons/Finance resource.

use super::substrate::{SubstrateDimension, SubstrateEventKind};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};

/// Direction of a measured substrate impact.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ImpactDirection {
    /// Depletion or degradation of the substrate.
    Depletion,
    /// Regeneration or enhancement of the substrate.
    Regeneration,
}

impl ImpactDirection {
    /// Convert the semantic direction to a signed event kind.
    pub fn event_kind(self) -> SubstrateEventKind {
        match self {
            Self::Depletion => SubstrateEventKind::Depletion,
            Self::Regeneration => SubstrateEventKind::Regeneration,
        }
    }
}

/// How an impact is attributed to an actor.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum AttributionBasis {
    /// Direct causal evidence connects actor and impact.
    Direct,
    /// Contractual responsibility connects actor and impact.
    Contractual,
    /// Actor is one contributor among multiple causal actors.
    Contributory,
    /// Actor participates in a shared supply/value chain.
    SharedChain,
    /// No defensible attribution is currently established.
    Unknown,
}

/// Lifecycle state of an impact record.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ImpactStatus {
    /// Impact is recorded but attribution is incomplete.
    Open,
    /// A participant has challenged the impact or attribution.
    Challenged,
    /// Attribution is sufficiently resolved for an obligation to be opened.
    Attributed,
    /// Required restoration is actively underway.
    Remediating,
    /// Required restoration has been completed.
    Restored,
    /// Evidence is insufficient to resolve the impact conclusively.
    Inconclusive,
}

/// A non-monetary attribution share expressed in basis points.
///
/// Shares are explicit rather than inferred. A total of 10,000 bps means the
/// full measured impact is attributed. A total below 10,000 bps deliberately
/// leaves a residual unattributed portion.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ImpactAttribution {
    /// Actor DID.
    pub actor: String,
    /// Attribution basis.
    pub basis: AttributionBasis,
    /// Share of the measured impact in basis points (0..=10,000).
    pub share_bps: u16,
    /// Evidence/provenance reference supporting this attribution.
    pub evidence_ref: Option<String>,
}

impl ImpactAttribution {
    /// Validate one attribution component.
    pub fn validate(&self) -> Result<(), String> {
        if self.actor.trim().is_empty() {
            return Err("Attribution actor cannot be empty".into());
        }
        if self.share_bps > 10_000 {
            return Err("Attribution share exceeds 10,000 basis points".into());
        }
        Ok(())
    }
}

/// A measured impact linked to an action and affected substrate resource.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct SubstrateImpact {
    /// Unique impact ID.
    pub id: String,
    /// Actor whose action is being evaluated.
    pub action_actor: String,
    /// Reference to the action/procurement/payment/etc. that produced the impact.
    pub action_ref: String,
    /// Affected substrate dimension.
    pub dimension: SubstrateDimension,
    /// Native unit of the measured impact.
    pub unit: String,
    /// Absolute magnitude of the impact.
    pub magnitude: u128,
    /// Direction of the impact.
    pub direction: ImpactDirection,
    /// Affected resource or population reference.
    pub affected_ref: String,
    /// Current attribution set.
    pub attributions: Vec<ImpactAttribution>,
    /// Evidence references supporting the impact measurement.
    pub evidence_refs: Vec<String>,
    /// Impact lifecycle state.
    pub status: ImpactStatus,
    /// Optional restoration obligation ID.
    pub obligation_id: Option<String>,
    /// Creation timestamp.
    pub recorded_at: u64,
}

impl SubstrateImpact {
    /// Sum explicitly attributed share in basis points.
    pub fn attribution_bps(&self) -> u32 {
        self.attributions
            .iter()
            .map(|item| u32::from(item.share_bps))
            .sum()
    }

    /// Return the still-unattributed portion in basis points.
    pub fn unattributed_bps(&self) -> u32 {
        10_000_u32.saturating_sub(self.attribution_bps())
    }

    /// Validate structural requirements.
    pub fn validate(&self) -> Result<(), String> {
        if self.id.trim().is_empty() {
            return Err("Impact ID cannot be empty".into());
        }
        if self.action_actor.trim().is_empty() {
            return Err("Impact action actor cannot be empty".into());
        }
        if self.action_ref.trim().is_empty() {
            return Err("Impact action reference cannot be empty".into());
        }
        if self.unit.trim().is_empty() {
            return Err("Impact unit cannot be empty".into());
        }
        if self.affected_ref.trim().is_empty() {
            return Err("Affected resource reference cannot be empty".into());
        }
        if self.magnitude == 0 {
            return Err("Impact magnitude must be greater than zero".into());
        }
        if self.attributions.is_empty() && self.status == ImpactStatus::Attributed {
            return Err("Attributed impact requires attribution entries".into());
        }
        for attribution in &self.attributions {
            attribution.validate()?;
        }
        let mut seen = BTreeSet::new();
        for attribution in &self.attributions {
            if !seen.insert(attribution.actor.clone()) {
                return Err(format!(
                    "Duplicate attribution actor: {}",
                    attribution.actor
                ));
            }
        }
        if self.evidence_refs.iter().any(|reference| reference.trim().is_empty()) {
            return Err("Impact evidence references cannot be empty".into());
        }
        Ok(())
    }
}

/// Restoration obligation generated from an attributed depletion impact.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct RestorationObligation {
    /// Unique obligation ID.
    pub id: String,
    /// Impact that caused the obligation.
    pub impact_id: String,
    /// Substrate dimension being restored.
    pub dimension: SubstrateDimension,
    /// Native unit of the restoration target.
    pub unit: String,
    /// Resource/community receiving restoration.
    pub target_ref: String,
    /// Required restoration quantity in native units.
    pub required_amount: u128,
    /// Amount completed so far.
    pub completed_amount: u128,
    /// Current lifecycle state.
    pub status: ObligationStatus,
    /// Due timestamp.
    pub due_at: u64,
}

impl RestorationObligation {
    /// Amount still outstanding.
    pub fn outstanding(&self) -> u128 {
        self.required_amount.saturating_sub(self.completed_amount)
    }

    /// Whether the obligation still blocks discretionary extraction.
    pub fn blocks_discretionary(&self) -> bool {
        !matches!(self.status, ObligationStatus::Restored | ObligationStatus::Waived)
            && self.outstanding() > 0
    }
}

/// Lifecycle state of a restoration obligation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ObligationStatus {
    /// Obligation exists but restoration has not started.
    Open,
    /// Restoration work is active.
    InProgress,
    /// Restoration completed.
    Restored,
    /// Obligation formally waived by authorized governance.
    Waived,
}

/// Result of gating an action against impact obligations.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ImpactGateDecision {
    /// Action may proceed.
    Allowed,
    /// Action may proceed, but unresolved impacts must remain visible.
    AllowedWithOpenImpacts,
    /// Action cannot proceed until an obligation is resolved.
    BlockedByObligation,
    /// Attribution/evidence is insufficient to establish a clean ledger state.
    InsufficientAttribution,
    /// Emergency action needs explicit escalation.
    EmergencyEscalationRequired,
}

/// Append-only impact and obligation ledger.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct ImpactLedger {
    impacts: BTreeMap<String, SubstrateImpact>,
    obligations: BTreeMap<String, RestorationObligation>,
}

/// Snapshot of unresolved impact state.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ImpactExposure {
    /// Open impact IDs.
    pub open_impact_ids: Vec<String>,
    /// Attributed but not fully restored impact IDs.
    pub remediation_impact_ids: Vec<String>,
    /// Obligations still blocking discretionary extraction.
    pub blocking_obligation_ids: Vec<String>,
}

impl ImpactLedger {
    /// Create an empty impact ledger.
    pub fn new() -> Self {
        Self::default()
    }

    /// Read an impact by ID.
    pub fn impact(&self, impact_id: &str) -> Option<&SubstrateImpact> {
        self.impacts.get(impact_id)
    }

    /// Read an obligation by ID.
    pub fn obligation(&self, obligation_id: &str) -> Option<&RestorationObligation> {
        self.obligations.get(obligation_id)
    }

    /// Record a new impact without inventing attribution.
    pub fn record_impact(&mut self, impact: SubstrateImpact) -> Result<(), String> {
        impact.validate()?;
        if self.impacts.contains_key(&impact.id) {
            return Err(format!("Duplicate impact ID: {}", impact.id));
        }
        self.impacts.insert(impact.id.clone(), impact);
        Ok(())
    }

    /// Return impact IDs linked to a specific action in deterministic order.
    pub fn impact_ids_for_action(&self, action_ref: &str) -> Vec<String> {
        if action_ref.trim().is_empty() {
            return Vec::new();
        }

        self.impacts
            .values()
            .filter(|impact| impact.action_ref == action_ref)
            .map(|impact| impact.id.clone())
            .collect()
    }

    /// Verify that an action scope does not omit any known impact dimension.
    ///
    /// This protects the execution path against a caller declaring a narrower
    /// scope than the impacts already recorded for the action. Unknown future
    /// impacts still require ordinary evidence/discovery processes; this method
    /// only makes the known-impacts boundary explicit and fail closed.
    pub fn validate_action_scope(
        &self,
        action_ref: &str,
        required_dimensions: &[SubstrateDimension],
    ) -> Result<Vec<String>, String> {
        if action_ref.trim().is_empty() {
            return Err("Action reference cannot be empty".into());
        }

        let required: BTreeSet<SubstrateDimension> =
            required_dimensions.iter().copied().collect();
        if required.is_empty() {
            return Err("Action scope must declare at least one substrate dimension".into());
        }

        let impact_ids = self.impact_ids_for_action(action_ref);
        for impact in self.impacts.values().filter(|impact| impact.action_ref == action_ref) {
            if !required.contains(&impact.dimension) {
                return Err(format!(
                    "Action scope omits known impact dimension {:?} for impact {}",
                    impact.dimension, impact.id
                ));
            }
        }

        Ok(impact_ids)
    }

    /// Challenge an impact while preserving its evidence and history.
    pub fn challenge_impact(&mut self, impact_id: &str) -> Result<(), String> {
        let impact = self
            .impacts
            .get_mut(impact_id)
            .ok_or_else(|| format!("Impact not found: {impact_id}"))?;

        match impact.status {
            ImpactStatus::Restored | ImpactStatus::Inconclusive => {
                Err("Restored or inconclusive impacts cannot be challenged in-place".into())
            }
            _ => {
                impact.status = ImpactStatus::Challenged;
                Ok(())
            }
        }
    }

    /// Resolve an attribution set and open a restoration obligation when the
    /// impact is a depletion.
    ///
    /// Attribution must total exactly 10,000 bps before an impact can become
    /// Attributed. This prevents an unresolved residual from silently
    /// disappearing.
    pub fn attribute_impact(
        &mut self,
        impact_id: &str,
        mut attributions: Vec<ImpactAttribution>,
        obligation_id: Option<String>,
        due_at: Option<u64>,
    ) -> Result<(), String> {
        for attribution in &attributions {
            attribution.validate()?;
            if attribution.basis == AttributionBasis::Unknown {
                return Err(
                    "Unknown attribution cannot close an impact; leave the residual unresolved"
                        .into(),
                );
            }
        }

        attributions.sort_by(|left, right| left.actor.cmp(&right.actor));

        let mut seen = BTreeSet::new();
        for attribution in &attributions {
            if !seen.insert(attribution.actor.clone()) {
                return Err(format!(
                    "Duplicate attribution actor: {}",
                    attribution.actor
                ));
            }
        }

        let total_bps: u32 = attributions
            .iter()
            .map(|item| u32::from(item.share_bps))
            .sum();

        if total_bps != 10_000 {
            return Err(format!(
                "Attribution must total exactly 10,000 bps; got {total_bps}"
            ));
        }

        let impact = self
            .impacts
            .get_mut(impact_id)
            .ok_or_else(|| format!("Impact not found: {impact_id}"))?;

        if !matches!(impact.status, ImpactStatus::Open | ImpactStatus::Challenged) {
            return Err("Only open or challenged impacts may be attributed".into());
        }
        if impact.obligation_id.is_some() {
            return Err(
                "Impact already has a restoration obligation; resolve the existing obligation before re-attribution"
                    .into(),
            );
        }

        if impact.direction == ImpactDirection::Depletion {
            let obligation_id = obligation_id
                .ok_or_else(|| "Depletion attribution requires an obligation ID".to_string())?;
            let due_at = due_at.ok_or_else(|| {
                "Depletion attribution requires an obligation due timestamp".to_string()
            })?;

            if self.obligations.contains_key(&obligation_id) {
                return Err(format!("Duplicate obligation ID: {obligation_id}"));
            }

            let obligation = RestorationObligation {
                id: obligation_id.clone(),
                impact_id: impact.id.clone(),
                dimension: impact.dimension,
                unit: impact.unit.clone(),
                target_ref: impact.affected_ref.clone(),
                required_amount: impact.magnitude,
                completed_amount: 0,
                status: ObligationStatus::Open,
                due_at,
            };
            self.obligations.insert(obligation_id.clone(), obligation);
            impact.obligation_id = Some(obligation_id);
        } else if obligation_id.is_some() {
            return Err("Regeneration impacts cannot open restoration obligations".into());
        }

        impact.attributions = attributions;
        impact.status = ImpactStatus::Attributed;

        Ok(())
    }

    /// Record restoration progress for an obligation.
    pub fn record_restoration(
        &mut self,
        obligation_id: &str,
        amount: u128,
    ) -> Result<ObligationStatus, String> {
        if amount == 0 {
            return Err("Restoration amount must be greater than zero".into());
        }

        let obligation = self
            .obligations
            .get_mut(obligation_id)
            .ok_or_else(|| format!("Obligation not found: {obligation_id}"))?;

        if matches!(
            obligation.status,
            ObligationStatus::Restored | ObligationStatus::Waived
        ) {
            return Err("Obligation is already closed".into());
        }

        obligation.completed_amount = obligation
            .completed_amount
            .checked_add(amount)
            .ok_or_else(|| "Restoration amount overflow".to_string())?
            .min(obligation.required_amount);

        obligation.status = if obligation.completed_amount == obligation.required_amount {
            ObligationStatus::Restored
        } else {
            ObligationStatus::InProgress
        };

        let impact = self
            .impacts
            .get_mut(&obligation.impact_id)
            .ok_or_else(|| format!("Impact not found: {}", obligation.impact_id))?;

        impact.status = if obligation.status == ObligationStatus::Restored {
            ImpactStatus::Restored
        } else {
            ImpactStatus::Remediating
        };

        Ok(obligation.status)
    }

    /// Return unresolved exposure across the complete impact ledger.
    pub fn exposure(&self) -> ImpactExposure {
        self.exposure_matching(|_| true)
    }

    /// Return unresolved exposure belonging to one economic action.
    ///
    /// Action-level close-out must not inherit unresolved state belonging to
    /// unrelated actions in the shared impact ledger.
    pub fn exposure_for_action(&self, action_ref: &str) -> ImpactExposure {
        if action_ref.trim().is_empty() {
            return ImpactExposure {
                open_impact_ids: Vec::new(),
                remediation_impact_ids: Vec::new(),
                blocking_obligation_ids: Vec::new(),
            };
        }

        self.exposure_matching(|impact| impact.action_ref == action_ref)
    }

    fn exposure_matching<F>(&self, mut matches: F) -> ImpactExposure
    where
        F: FnMut(&SubstrateImpact) -> bool,
    {
        let mut open_impact_ids = Vec::new();
        let mut remediation_impact_ids = Vec::new();
        let mut blocking_obligation_ids = Vec::new();

        for impact in self.impacts.values().filter(|impact| matches(impact)) {
            if impact.direction != ImpactDirection::Depletion {
                continue;
            }

            match impact.status {
                ImpactStatus::Open | ImpactStatus::Challenged | ImpactStatus::Inconclusive => {
                    open_impact_ids.push(impact.id.clone());
                }
                ImpactStatus::Attributed | ImpactStatus::Remediating => {
                    remediation_impact_ids.push(impact.id.clone());
                }
                ImpactStatus::Restored => {}
            }
        }

        for obligation in self.obligations.values() {
            if !obligation.blocks_discretionary() {
                continue;
            }

            let Some(impact) = self.impacts.get(&obligation.impact_id) else {
                continue;
            };

            if matches(impact) {
                blocking_obligation_ids.push(obligation.id.clone());
            }
        }

        ImpactExposure {
            open_impact_ids,
            remediation_impact_ids,
            blocking_obligation_ids,
        }
    }

    fn gate_from_exposure(
        exposure: ImpactExposure,
        purpose: super::substrate::DistributionPurpose,
    ) -> ImpactGateDecision {
        match purpose {
            super::substrate::DistributionPurpose::Maintenance
            | super::substrate::DistributionPurpose::Restoration => {
                if exposure.open_impact_ids.is_empty()
                    && exposure.blocking_obligation_ids.is_empty()
                {
                    ImpactGateDecision::Allowed
                } else {
                    ImpactGateDecision::AllowedWithOpenImpacts
                }
            }
            super::substrate::DistributionPurpose::Discretionary => {
                if !exposure.blocking_obligation_ids.is_empty() {
                    ImpactGateDecision::BlockedByObligation
                } else if !exposure.open_impact_ids.is_empty() {
                    ImpactGateDecision::InsufficientAttribution
                } else {
                    ImpactGateDecision::Allowed
                }
            }
            super::substrate::DistributionPurpose::Emergency => {
                if !exposure.blocking_obligation_ids.is_empty() {
                    ImpactGateDecision::EmergencyEscalationRequired
                } else {
                    ImpactGateDecision::Allowed
                }
            }
        }
    }

    /// Gate economic action against the complete impact/reciprocity state.
    pub fn gate(&self, purpose: super::substrate::DistributionPurpose) -> ImpactGateDecision {
        Self::gate_from_exposure(self.exposure(), purpose)
    }

    /// Gate one economic action against only its own impact/reciprocity state.
    pub fn gate_for_action(
        &self,
        action_ref: &str,
        purpose: super::substrate::DistributionPurpose,
    ) -> ImpactGateDecision {
        Self::gate_from_exposure(self.exposure_for_action(action_ref), purpose)
    }

    /// Return the set of impacted dimensions represented by the ledger.
    pub fn dimensions(&self) -> BTreeSet<SubstrateDimension> {
        self.impacts.values().map(|impact| impact.dimension).collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::economics::substrate::DistributionPurpose;

    fn depletion() -> SubstrateImpact {
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

    fn attribution(actor: &str, share_bps: u16) -> ImpactAttribution {
        ImpactAttribution {
            actor: actor.into(),
            basis: AttributionBasis::Contributory,
            share_bps,
            evidence_ref: Some(format!("evidence:{actor}")),
        }
    }

    #[test]
    fn unknown_or_partial_attribution_cannot_close_depletion() {
        let mut ledger = ImpactLedger::new();
        ledger.record_impact(depletion()).unwrap();

        let result = ledger.attribute_impact(
            "impact-1",
            vec![attribution("did:example:a", 5_000)],
            Some("obligation-1".into()),
            Some(5_000),
        );

        assert!(result.is_err());
        assert_eq!(ledger.impact("impact-1").unwrap().status, ImpactStatus::Open);
        assert!(ledger.obligation("obligation-1").is_none());
    }

    #[test]
    fn unknown_basis_cannot_disguise_missing_attribution() {
        let mut ledger = ImpactLedger::new();
        ledger.record_impact(depletion()).unwrap();

        let result = ledger.attribute_impact(
            "impact-1",
            vec![ImpactAttribution {
                actor: "did:example:unknown".into(),
                basis: AttributionBasis::Unknown,
                share_bps: 10_000,
                evidence_ref: None,
            }],
            Some("obligation-1".into()),
            Some(5_000),
        );

        assert!(result.is_err());
        assert!(ledger.obligation("obligation-1").is_none());
        assert_eq!(ledger.impact("impact-1").unwrap().status, ImpactStatus::Open);
    }

    #[test]
    fn full_attribution_opens_restoration_obligation() {
        let mut ledger = ImpactLedger::new();
        ledger.record_impact(depletion()).unwrap();

        ledger
            .attribute_impact(
                "impact-1",
                vec![attribution("did:example:a", 7_500), attribution("did:example:b", 2_500)],
                Some("obligation-1".into()),
                Some(5_000),
            )
            .unwrap();

        assert_eq!(
            ledger.impact("impact-1").unwrap().status,
            ImpactStatus::Attributed
        );
        assert_eq!(
            ledger.obligation("obligation-1").unwrap().outstanding(),
            1_000
        );
        assert_eq!(
            ledger.gate(DistributionPurpose::Discretionary),
            ImpactGateDecision::BlockedByObligation
        );
    }

    #[test]
    fn attribution_order_is_canonical() {
        let mut left = ImpactLedger::new();
        let mut right = ImpactLedger::new();
        left.record_impact(depletion()).unwrap();
        right.record_impact(depletion()).unwrap();

        left.attribute_impact(
            "impact-1",
            vec![attribution("did:example:b", 2_500), attribution("did:example:a", 7_500)],
            Some("obligation-1".into()),
            Some(5_000),
        )
        .unwrap();
        right
            .attribute_impact(
                "impact-1",
                vec![attribution("did:example:a", 7_500), attribution("did:example:b", 2_500)],
                Some("obligation-1".into()),
                Some(5_000),
            )
            .unwrap();

        assert_eq!(
            left.impact("impact-1").unwrap(),
            right.impact("impact-1").unwrap()
        );
    }

    #[test]
    fn restoration_can_progress_without_erasing_history() {
        let mut ledger = ImpactLedger::new();
        ledger.record_impact(depletion()).unwrap();
        ledger
            .attribute_impact(
                "impact-1",
                vec![attribution("did:example:a", 10_000)],
                Some("obligation-1".into()),
                Some(5_000),
            )
            .unwrap();

        let status = ledger.record_restoration("obligation-1", 400).unwrap();
        assert_eq!(status, ObligationStatus::InProgress);
        assert_eq!(
            ledger.impact("impact-1").unwrap().status,
            ImpactStatus::Remediating
        );
        assert_eq!(ledger.obligation("obligation-1").unwrap().outstanding(), 600);

        let status = ledger.record_restoration("obligation-1", 600).unwrap();
        assert_eq!(status, ObligationStatus::Restored);
        assert_eq!(ledger.impact("impact-1").unwrap().status, ImpactStatus::Restored);
        assert_eq!(ledger.obligation("obligation-1").unwrap().outstanding(), 0);
    }

    #[test]
    fn reattribution_cannot_orphan_an_existing_obligation() {
        let mut ledger = ImpactLedger::new();
        ledger.record_impact(depletion()).unwrap();
        ledger
            .attribute_impact(
                "impact-1",
                vec![attribution("did:example:a", 10_000)],
                Some("obligation-1".into()),
                Some(5_000),
            )
            .unwrap();
        ledger.challenge_impact("impact-1").unwrap();

        let result = ledger.attribute_impact(
            "impact-1",
            vec![attribution("did:example:b", 10_000)],
            Some("obligation-2".into()),
            Some(6_000),
        );

        assert!(result.is_err());
        assert!(ledger.obligation("obligation-2").is_none());
        assert!(ledger.obligation("obligation-1").is_some());
    }

    #[test]
    fn challenge_preserves_impact_history() {
        let mut ledger = ImpactLedger::new();
        ledger.record_impact(depletion()).unwrap();
        ledger.challenge_impact("impact-1").unwrap();

        assert_eq!(
            ledger.impact("impact-1").unwrap().status,
            ImpactStatus::Challenged
        );
        assert_eq!(
            ledger.impact("impact-1").unwrap().evidence_refs,
            vec!["evidence:1".to_string()]
        );
    }

    #[test]
    fn discretionary_action_is_not_blocked_once_obligation_is_restored() {
        let mut ledger = ImpactLedger::new();
        ledger.record_impact(depletion()).unwrap();
        ledger
            .attribute_impact(
                "impact-1",
                vec![attribution("did:example:a", 10_000)],
                Some("obligation-1".into()),
                Some(5_000),
            )
            .unwrap();
        ledger.record_restoration("obligation-1", 1_000).unwrap();

        assert_eq!(
            ledger.gate(DistributionPurpose::Discretionary),
            ImpactGateDecision::Allowed
        );
    }

    #[test]
    fn scope_validation_isolated_between_actions() {
        let mut ledger = ImpactLedger::new();
        ledger.record_impact(SubstrateImpact {
            id: "impact-other-action".into(),
            action_actor: "did:example:other".into(),
            action_ref: "action:other".into(),
            dimension: SubstrateDimension::Ecological,
            unit: "m3".into(),
            magnitude: 500,
            direction: ImpactDirection::Depletion,
            affected_ref: "river:other".into(),
            attributions: vec![],
            evidence_refs: vec!["evidence:other".into()],
            status: ImpactStatus::Open,
            obligation_id: None,
            recorded_at: 1_000,
        }).unwrap();

        let result = ledger.validate_action_scope(
            "action:1",
            &[SubstrateDimension::Financial],
        );

        assert!(result.is_ok());
        assert!(result.unwrap().is_empty());
    }

    #[test]
    fn duplicate_impact_ids_are_rejected() {
        let mut ledger = ImpactLedger::new();
        ledger.record_impact(depletion()).unwrap();
        assert!(ledger.record_impact(depletion()).is_err());
    }

    #[test]
    fn zero_impact_is_rejected() {
        let mut impact = depletion();
        impact.magnitude = 0;
        assert!(ImpactLedger::new().record_impact(impact).is_err());
    }
}
