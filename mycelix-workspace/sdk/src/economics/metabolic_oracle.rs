// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! # Metabolic Oracle v2
//!
//! Autopoietic self-regulation with 365-day rolling memory,
//! counter-cyclical TEND expansion, and seasonal awareness.
//!
//! Changes from v1:
//! - Memory: 365 days rolling (was 24 hours)
//! - Removed: spore_allocation (CGC gone)
//! - Added: TendLimitTier for counter-cyclical TEND expansion
//! - Counter-cyclical: When stressed, lower fees + expand TEND limits

use serde::{Deserialize, Serialize};
use super::policy_profile::EconomicPolicyProfile;
use sha2::{Digest, Sha256};

/// Policy bounds preventing runaway self-modification
/// These are constitutional constraints that cannot be modified by the oracle
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct PolicyBounds {
    /// Minimum fee rate (0.01% floor)
    pub fee_rate_min: f64,
    /// Maximum fee rate (0.5% ceiling)
    pub fee_rate_max: f64,
    /// Minimum demurrage rate (1% annual floor)
    pub demurrage_rate_min: f64,
    /// Maximum demurrage rate (5% annual ceiling)
    pub demurrage_rate_max: f64,
    /// Emergency reserve minimum (5%)
    pub emergency_reserve_min: f64,
}

impl Default for PolicyBounds {
    fn default() -> Self {
        Self {
            fee_rate_min: 0.0001,
            fee_rate_max: 0.005,
            demurrage_rate_min: 0.01,
            demurrage_rate_max: 0.05,
            emergency_reserve_min: 0.05,
        }
    }
}

/// Network vitality measurement components
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VitalityComponents {
    /// Active SAP / Total SAP × velocity_multiplier
    pub circulation: f64,
    /// Average peer connections / max theoretical
    pub relationship: f64,
    /// Commons pool utilization + recognition flow
    pub commons: f64,
    /// Node count × geographic distribution / target
    pub resilience: f64,
}

impl VitalityComponents {
    /// Calculate composite Vitality Index
    pub fn calculate_vitality(&self) -> f64 {
        const CIRCULATION_WEIGHT: f64 = 0.40;
        const RELATIONSHIP_WEIGHT: f64 = 0.30;
        const COMMONS_WEIGHT: f64 = 0.20;
        const RESILIENCE_WEIGHT: f64 = 0.10;

        let vitality = self.circulation * CIRCULATION_WEIGHT
            + self.relationship * RELATIONSHIP_WEIGHT
            + self.commons * COMMONS_WEIGHT
            + self.resilience * RESILIENCE_WEIGHT;

        (vitality * 100.0).clamp(0.0, 100.0)
    }
}

/// Vitality Index result with state classification
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VitalityIndex {
    /// Raw vitality score (0-100)
    pub score: f64,
    /// Current metabolic state
    pub state: MetabolicState,
    /// Component breakdown
    pub components: VitalityComponents,
    /// Trend direction
    pub trend: VitalityTrend,
    /// Measurement timestamp
    pub timestamp: u64,
}

/// Metabolic state classification
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum MetabolicState {
    /// Score 80-100: Network thriving
    Thriving,
    /// Score 60-80: Normal operation
    Healthy,
    /// Score 40-60: Auto-healing activates
    Stressed,
    /// Score 20-40: Emergency response
    Critical,
    /// Score 0-20: Circuit breaker
    Failing,
}

impl MetabolicState {
    /// Determine state from vitality score.
    /// Thresholds aligned with canonical types crate (mycelix_finance_types).
    pub fn from_score(score: f64) -> Self {
        match score {
            s if s >= 80.0 => MetabolicState::Thriving,
            s if s >= 60.0 => MetabolicState::Healthy,
            s if s >= 40.0 => MetabolicState::Stressed,
            s if s >= 20.0 => MetabolicState::Critical,
            _ => MetabolicState::Failing,
        }
    }

    /// Check if automatic intervention is needed
    pub fn requires_intervention(&self) -> bool {
        matches!(
            self,
            MetabolicState::Stressed | MetabolicState::Critical | MetabolicState::Failing
        )
    }

    /// Check if circuit breaker should activate
    pub fn circuit_breaker_active(&self) -> bool {
        matches!(self, MetabolicState::Failing)
    }
}

/// TEND limit tier for counter-cyclical expansion (WIR Bank pattern)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum TendLimitTier {
    /// Healthy/Thriving: ±40 TEND
    Normal,
    /// Stressed: ±60 TEND
    Elevated,
    /// Critical: ±80 TEND
    High,
    /// Failing: ±120 TEND
    Emergency,
}

impl TendLimitTier {
    /// Get the TEND balance limit for this tier
    pub fn limit(&self) -> i32 {
        match self {
            TendLimitTier::Normal => 40,
            TendLimitTier::Elevated => 60,
            TendLimitTier::High => 80,
            TendLimitTier::Emergency => 120,
        }
    }

    /// Determine tier from metabolic state
    pub fn from_state(state: MetabolicState) -> Self {
        match state {
            MetabolicState::Thriving | MetabolicState::Healthy => TendLimitTier::Normal,
            MetabolicState::Stressed => TendLimitTier::Elevated,
            MetabolicState::Critical => TendLimitTier::High,
            MetabolicState::Failing => TendLimitTier::Emergency,
        }
    }
}

/// Vitality trend direction
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum VitalityTrend {
    /// Improving over measurement window
    Improving,
    /// Stable (±5%)
    Stable,
    /// Declining over measurement window
    Declining,
    /// Rapid decline (>10%)
    RapidDecline,
}

/// Policy adjustment recommendation from oracle
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PolicyAdjustment {
    /// Fee rate adjustment factor (1.0 = no change)
    pub fee_rate_factor: f64,
    /// Demurrage rate adjustment factor
    pub demurrage_rate_factor: f64,
    /// Velocity incentive multiplier
    pub velocity_incentive: f64,
    /// TEND limit tier for counter-cyclical expansion
    pub tend_limit_tier: TendLimitTier,
    /// Emergency liquidity release (if critical)
    pub emergency_release: Option<u64>,
    /// Reason for adjustment
    pub reason: String,
    /// Requires human approval (for large changes)
    pub requires_approval: bool,
}

/// A governed record authorizing application of a policy adjustment.
///
/// The oracle may recommend an adjustment from observed network vitality, but
/// application becomes a separate governance event with explicit provenance.
/// Observation references do not assert that the observations are sufficient
/// for any universal monetary policy; they make the evidentiary basis auditable.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GovernedPolicyAdjustment {
    /// Unique decision identifier.
    pub decision_id: String,
    /// Versioned economic-policy profile under which the decision is interpreted.
    pub policy_profile_ref: String,
    /// Content fingerprint of the exact policy profile used for the decision.
    pub policy_profile_fingerprint: String,
    /// References to the observations used in the decision.
    pub observation_refs: Vec<String>,
    /// Policy/rule reference authorizing the decision.
    pub rule_ref: String,
    /// Authority reference for the decision.
    pub authority_ref: String,
    /// Recommended adjustment being authorized.
    pub adjustment: PolicyAdjustment,
    /// Time at which the decision was authorized.
    pub decided_at: u64,
}

impl GovernedPolicyAdjustment {
    /// Validate the decision envelope before it can change policy state.
    pub fn validate(&self) -> Result<(), String> {
        if self.decision_id.trim().is_empty() {
            return Err("Policy decision ID cannot be empty".into());
        }
        if self.policy_profile_ref.trim().is_empty() {
            return Err("Policy decision policy profile reference cannot be empty".into());
        }
        if self.policy_profile_fingerprint.len() != 64
            || !self
                .policy_profile_fingerprint
                .as_bytes()
                .iter()
                .all(u8::is_ascii_hexdigit)
        {
            return Err(
                "Policy decision policy profile fingerprint must be a 64-character hexadecimal SHA-256"
                    .into(),
            );
        }
        if self.observation_refs.is_empty() {
            return Err("Policy decision requires at least one observation reference".into());
        }
        if self
            .observation_refs
            .iter()
            .any(|reference| reference.trim().is_empty())
        {
            return Err("Policy observation references cannot be empty".into());
        }
        if self.rule_ref.trim().is_empty() {
            return Err("Policy decision rule reference cannot be empty".into());
        }
        if self.authority_ref.trim().is_empty() {
            return Err("Policy decision authority reference cannot be empty".into());
        }
        if self.adjustment.reason.trim().is_empty() {
            return Err("Policy adjustment reason cannot be empty".into());
        }

        for (name, value) in [
            ("fee rate factor", self.adjustment.fee_rate_factor),
            ("demurrage rate factor", self.adjustment.demurrage_rate_factor),
            ("velocity incentive", self.adjustment.velocity_incentive),
        ] {
            if !value.is_finite() || value < 0.0 {
                return Err(format!(
                    "Policy adjustment {name} must be finite and non-negative"
                ));
            }
        }

        Ok(())
    }

    /// Return a deterministic content fingerprint for the governed decision.
    ///
    /// The fingerprint is a tamper-evident identifier, not a signature or proof
    /// that the referenced authority actually authorized the decision.
    pub fn fingerprint(&self) -> Result<String, String> {
        self.validate()?;
        let mut observation_refs = self.observation_refs.clone();
        observation_refs.sort();

        let payload = serde_json::json!({
            "version": 1,
            "decision_id": self.decision_id,
            "policy_profile_ref": self.policy_profile_ref,
            "policy_profile_fingerprint": self.policy_profile_fingerprint,
            "observation_refs": observation_refs,
            "rule_ref": self.rule_ref,
            "authority_ref": self.authority_ref,
            "adjustment": self.adjustment,
            "decided_at": self.decided_at,
        });
        let canonical = serde_json::to_vec(&payload)
            .map_err(|error| format!("Policy decision canonicalization failed: {error}"))?;
        let mut hasher = Sha256::new();
        hasher.update(b"MYCELIX-ECONOMIC-POLICY-DECISION-V1\\0");
        hasher.update(canonical);
        Ok(hex::encode(hasher.finalize()))
    }
}

/// The Metabolic Oracle for autopoietic parameter adjustment
#[derive(Debug, Clone)]
pub struct MetabolicOracle {
    /// Constitutional policy bounds
    pub bounds: PolicyBounds,
    /// Current network parameters
    pub current_params: NetworkParameters,
    /// Historical vitality readings (365-day rolling)
    pub vitality_history: Vec<VitalityIndex>,
    /// Adjustment history for audit and backwards-compatible inspection.
    pub adjustment_history: Vec<PolicyAdjustment>,
    /// Explicit governance decisions authorizing parameter changes.
    governed_decision_history: Vec<GovernedPolicyAdjustment>,
}

/// Current network economic parameters
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NetworkParameters {
    /// Current base fee rate
    pub fee_rate: f64,
    /// Current demurrage rate (annual)
    pub demurrage_rate: f64,
    /// Velocity incentive multiplier
    pub velocity_incentive: f64,
    /// Emergency reserve ratio
    pub emergency_reserve: f64,
    /// Current TEND limit tier
    pub tend_limit_tier: TendLimitTier,
}

impl Default for NetworkParameters {
    fn default() -> Self {
        Self {
            fee_rate: 0.0003,     // 0.03% (Member tier default)
            demurrage_rate: 0.02, // 2% annual
            velocity_incentive: 1.0,
            emergency_reserve: 0.10,
            tend_limit_tier: TendLimitTier::Normal,
        }
    }
}

impl MetabolicOracle {
    /// Create new oracle with default bounds
    pub fn new() -> Self {
        Self {
            bounds: PolicyBounds::default(),
            current_params: NetworkParameters::default(),
            vitality_history: Vec::new(),
            adjustment_history: Vec::new(),
            governed_decision_history: Vec::new(),
        }
    }

    /// Record new vitality measurement
    pub fn record_vitality(&mut self, components: VitalityComponents, timestamp: u64) {
        let score = components.calculate_vitality();
        let state = MetabolicState::from_score(score);
        let trend = self.calculate_trend(score);

        let vitality = VitalityIndex {
            score,
            state,
            components,
            trend,
            timestamp,
        };

        self.vitality_history.push(vitality);

        // Keep 365 days of history (assuming daily measurements)
        if self.vitality_history.len() > 365 {
            self.vitality_history.remove(0);
        }
    }

    /// Calculate trend from recent history
    fn calculate_trend(&self, current: f64) -> VitalityTrend {
        if self.vitality_history.is_empty() {
            return VitalityTrend::Stable;
        }

        let oldest = match self.vitality_history.first() {
            Some(entry) => entry.score,
            None => return VitalityTrend::Stable,
        };

        if oldest == 0.0 {
            return VitalityTrend::Stable;
        }

        let delta_pct = ((current - oldest) / oldest) * 100.0;

        match delta_pct {
            d if d > 5.0 => VitalityTrend::Improving,
            d if d < -10.0 => VitalityTrend::RapidDecline,
            d if d < -5.0 => VitalityTrend::Declining,
            _ => VitalityTrend::Stable,
        }
    }

    /// Generate policy adjustment based on current vitality
    pub fn generate_adjustment(&self) -> PolicyAdjustment {
        let current = match self.vitality_history.last() {
            Some(v) => v,
            None => {
                return PolicyAdjustment {
                    fee_rate_factor: 1.0,
                    demurrage_rate_factor: 1.0,
                    velocity_incentive: 1.0,
                    tend_limit_tier: TendLimitTier::Normal,
                    emergency_release: None,
                    reason: "No vitality data available".to_string(),
                    requires_approval: false,
                }
            }
        };

        match current.state {
            MetabolicState::Thriving => self.thriving_adjustment(),
            MetabolicState::Healthy => self.healthy_adjustment(),
            MetabolicState::Stressed => self.stressed_adjustment(),
            MetabolicState::Critical => self.critical_adjustment(),
            MetabolicState::Failing => self.failing_adjustment(),
        }
    }

    fn thriving_adjustment(&self) -> PolicyAdjustment {
        PolicyAdjustment {
            fee_rate_factor: 1.0,
            demurrage_rate_factor: 0.9,
            velocity_incentive: 1.0,
            tend_limit_tier: TendLimitTier::Normal,
            emergency_release: None,
            reason: "Thriving: Slightly reducing demurrage".to_string(),
            requires_approval: false,
        }
    }

    fn healthy_adjustment(&self) -> PolicyAdjustment {
        PolicyAdjustment {
            fee_rate_factor: 1.0,
            demurrage_rate_factor: 1.0,
            velocity_incentive: 1.0,
            tend_limit_tier: TendLimitTier::Normal,
            emergency_release: None,
            reason: "Healthy: Maintaining stable parameters".to_string(),
            requires_approval: false,
        }
    }

    fn stressed_adjustment(&self) -> PolicyAdjustment {
        // Counter-cyclical: lower fees + expand TEND limits
        PolicyAdjustment {
            fee_rate_factor: 0.8,
            demurrage_rate_factor: 1.0,
            velocity_incentive: 1.2,
            tend_limit_tier: TendLimitTier::Elevated, // ±60 TEND
            emergency_release: None,
            reason: "Stressed: Lower fees, expand TEND limits to ±60".to_string(),
            requires_approval: false,
        }
    }

    fn critical_adjustment(&self) -> PolicyAdjustment {
        PolicyAdjustment {
            fee_rate_factor: 0.5,
            demurrage_rate_factor: 0.5,
            velocity_incentive: 1.5,
            tend_limit_tier: TendLimitTier::High, // ±80 TEND
            emergency_release: Some(10_000),
            reason: "Critical: Emergency response, TEND limits ±80".to_string(),
            requires_approval: true,
        }
    }

    fn failing_adjustment(&self) -> PolicyAdjustment {
        PolicyAdjustment {
            fee_rate_factor: 0.0,
            demurrage_rate_factor: 0.0,
            velocity_incentive: 0.0,
            tend_limit_tier: TendLimitTier::Emergency, // ±120 TEND
            emergency_release: Some(50_000),
            reason: "FAILING: Circuit breaker, TEND limits ±120".to_string(),
            requires_approval: true,
        }
    }

    fn apply_adjustment_unchecked(&mut self, adjustment: &PolicyAdjustment) {
        let new_fee = self.current_params.fee_rate * adjustment.fee_rate_factor;
        self.current_params.fee_rate =
            new_fee.clamp(self.bounds.fee_rate_min, self.bounds.fee_rate_max);

        let new_demurrage = self.current_params.demurrage_rate * adjustment.demurrage_rate_factor;
        self.current_params.demurrage_rate = new_demurrage.clamp(
            self.bounds.demurrage_rate_min,
            self.bounds.demurrage_rate_max,
        );

        self.current_params.velocity_incentive = adjustment.velocity_incentive;
        self.current_params.tend_limit_tier = adjustment.tend_limit_tier;

        self.adjustment_history.push(adjustment.clone());
    }

    /// Return governance decisions in append-only insertion order.
    pub fn governed_decisions(&self) -> &[GovernedPolicyAdjustment] {
        &self.governed_decision_history
    }

    /// Apply an adjustment directly when the caller has already established
    /// governance externally. This remains useful for deterministic simulation.
    pub fn apply_adjustment(&mut self, adjustment: &PolicyAdjustment) -> Result<(), String> {
        if adjustment.requires_approval {
            return Err("Adjustment requires governance approval".to_string());
        }
        self.apply_adjustment_unchecked(adjustment);
        Ok(())
    }

    /// Apply a governed adjustment only when its policy profile matches
    /// the exact profile content supplied by the caller.
    ///
    /// This closes the distinction between "profile reference" and "the exact
    /// profile that was actually active". A human-readable profile ID is not
    /// sufficient to establish historical policy context.
    pub fn apply_governed_adjustment_for_profile(
        &mut self,
        decision: GovernedPolicyAdjustment,
        profile: &EconomicPolicyProfile,
    ) -> Result<String, String> {
        profile.validate()?;

        let profile_fingerprint = profile.fingerprint()?;
        if decision.policy_profile_ref != profile.profile_id {
            return Err("Policy decision profile reference does not match supplied profile".into());
        }
        if decision.policy_profile_fingerprint != profile_fingerprint {
            return Err(
                "Policy decision profile fingerprint does not match supplied profile content"
                    .into(),
            );
        }

        self.apply_governed_adjustment(decision)
    }

    /// Apply a policy adjustment with explicit decision provenance.
    ///
    /// This is the governance-safe application path: the recommendation is
    /// separated from the authorization event, and the authorization is kept
    /// in an append-only in-memory history for later persistence by the host.
    pub fn apply_governed_adjustment(
        &mut self,
        decision: GovernedPolicyAdjustment,
    ) -> Result<String, String> {
        decision.validate()?;

        if self
            .governed_decision_history
            .iter()
            .any(|existing| existing.decision_id == decision.decision_id)
        {
            return Err(format!(
                "Duplicate policy decision ID: {}",
                decision.decision_id
            ));
        }

        let fingerprint = decision.fingerprint()?;
        self.apply_adjustment_unchecked(&decision.adjustment);
        self.governed_decision_history.push(decision);
        Ok(fingerprint)
    }

    /// Get current vitality state
    pub fn current_state(&self) -> Option<MetabolicState> {
        self.vitality_history.last().map(|v| v.state)
    }

    /// Get current vitality score
    pub fn current_vitality(&self) -> Option<f64> {
        self.vitality_history.last().map(|v| v.score)
    }
}

impl Default for MetabolicOracle {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn healthy_components() -> VitalityComponents {
        // Target vitality ~70 (Healthy tier: 60-80)
        // 0.75*0.40 + 0.70*0.30 + 0.65*0.20 + 0.55*0.10 = 0.30+0.21+0.13+0.055 = 0.695 → 69.5
        VitalityComponents {
            circulation: 0.75,
            relationship: 0.70,
            commons: 0.65,
            resilience: 0.55,
        }
    }

    fn stressed_components() -> VitalityComponents {
        // Target vitality ~45 (Stressed tier: 40-60)
        // 0.45*0.40 + 0.50*0.30 + 0.40*0.20 + 0.35*0.10 = 0.18+0.15+0.08+0.035 = 0.445 → 44.5
        VitalityComponents {
            circulation: 0.45,
            relationship: 0.50,
            commons: 0.40,
            resilience: 0.35,
        }
    }

    #[test]
    fn test_vitality_calculation() {
        let healthy = healthy_components();
        let vitality = healthy.calculate_vitality();
        // 0.75*0.40 + 0.70*0.30 + 0.65*0.20 + 0.55*0.10 = 69.5
        assert!(
            vitality > 65.0 && vitality < 75.0,
            "Expected ~69.5, got {}",
            vitality
        );
    }

    #[test]
    fn test_state_classification() {
        // Thresholds aligned with mycelix_finance_types::MetabolicState::from_vitality
        assert_eq!(MetabolicState::from_score(85.0), MetabolicState::Thriving);
        assert_eq!(MetabolicState::from_score(80.0), MetabolicState::Thriving);
        assert_eq!(MetabolicState::from_score(79.9), MetabolicState::Healthy);
        assert_eq!(MetabolicState::from_score(60.0), MetabolicState::Healthy);
        assert_eq!(MetabolicState::from_score(55.0), MetabolicState::Stressed);
        assert_eq!(MetabolicState::from_score(40.0), MetabolicState::Stressed);
        assert_eq!(MetabolicState::from_score(30.0), MetabolicState::Critical);
        assert_eq!(MetabolicState::from_score(20.0), MetabolicState::Critical);
        assert_eq!(MetabolicState::from_score(15.0), MetabolicState::Failing);
        assert_eq!(MetabolicState::from_score(5.0), MetabolicState::Failing);
    }

    #[test]
    fn test_tend_limit_tiers() {
        assert_eq!(
            TendLimitTier::from_state(MetabolicState::Healthy).limit(),
            40
        );
        assert_eq!(
            TendLimitTier::from_state(MetabolicState::Stressed).limit(),
            60
        );
        assert_eq!(
            TendLimitTier::from_state(MetabolicState::Critical).limit(),
            80
        );
        assert_eq!(
            TendLimitTier::from_state(MetabolicState::Failing).limit(),
            120
        );
    }

    #[test]
    fn test_oracle_stressed_expands_tend() {
        let mut oracle = MetabolicOracle::new();
        oracle.record_vitality(stressed_components(), 1000);

        let adjustment = oracle.generate_adjustment();
        assert_eq!(adjustment.tend_limit_tier, TendLimitTier::Elevated);
        assert!(adjustment.fee_rate_factor < 1.0);
        assert!(adjustment.velocity_incentive > 1.0);
    }

    #[test]
    fn test_governed_policy_requires_observation_and_authority() {
        let mut oracle = MetabolicOracle::new();
        let adjustment = oracle.generate_adjustment();
        let decision = GovernedPolicyAdjustment {
            decision_id: "decision:1".into(),
            policy_profile_ref: "profile:za:reference:v1".into(),
            policy_profile_fingerprint: "a".repeat(64),
            observation_refs: Vec::new(),
            rule_ref: "rule:countercyclical:v1".into(),
            authority_ref: "authority:dao-1".into(),
            adjustment,
            decided_at: 1_000,
        };

        assert!(oracle.apply_governed_adjustment(decision).is_err());
    }

    #[test]
    fn test_governed_policy_records_authorization_and_fingerprint() {
        let mut oracle = MetabolicOracle::new();
        let adjustment = oracle.generate_adjustment();
        let decision = GovernedPolicyAdjustment {
            decision_id: "decision:1".into(),
            policy_profile_ref: "profile:za:reference:v1".into(),
            policy_profile_fingerprint: "a".repeat(64),
            observation_refs: vec!["observation:vitality:1".into()],
            rule_ref: "rule:countercyclical:v1".into(),
            authority_ref: "authority:dao-1".into(),
            adjustment,
            decided_at: 1_000,
        };

        let fingerprint = decision.fingerprint().unwrap();
        assert_eq!(fingerprint.len(), 64);
        let applied = oracle.apply_governed_adjustment(decision).unwrap();
        assert_eq!(applied.len(), 64);
        assert_eq!(oracle.governed_decisions().len(), 1);
        assert_eq!(oracle.governed_decisions()[0].authority_ref, "authority:dao-1");
    }

    #[test]
    fn test_governed_policy_fingerprint_is_order_independent_for_observations() {
        let oracle = MetabolicOracle::new();
        let adjustment = oracle.generate_adjustment();
        let left = GovernedPolicyAdjustment {
            decision_id: "decision:1".into(),
            policy_profile_ref: "profile:za:reference:v1".into(),
            policy_profile_fingerprint: "a".repeat(64),
            observation_refs: vec!["observation:b".into(), "observation:a".into()],
            rule_ref: "rule:countercyclical:v1".into(),
            authority_ref: "authority:dao-1".into(),
            adjustment: adjustment.clone(),
            decided_at: 1_000,
        };
        let right = GovernedPolicyAdjustment {
            observation_refs: vec!["observation:a".into(), "observation:b".into()],
            ..left.clone()
        };

        assert_eq!(left.fingerprint().unwrap(), right.fingerprint().unwrap());
    }

    fn policy_profile() -> EconomicPolicyProfile {
        EconomicPolicyProfile {
            profile_id: "profile:za:reference:v1".into(),
            jurisdiction_ref: "jurisdiction:ZA".into(),
            regime_ref: "regime:national-fiat".into(),
            policy_version: "1.0.0".into(),
            currency_refs: vec!["ZAR".into()],
            authority_refs: vec!["authority:dao-1".into()],
            policy_rule_refs: vec!["rule:countercyclical:v1".into()],
            interoperability_profile_refs: vec!["standard:SDMX-3.1".into()],
            effective_from: 1_000,
            effective_until: None,
            evidence_refs: vec!["evidence:profile".into()],
            supersedes_profile_ref: None,
            declared_at: 1_000,
        }
    }

    #[test]
    fn test_governed_policy_must_match_exact_profile() {
        let mut oracle = MetabolicOracle::new();
        let profile = policy_profile();
        let decision = GovernedPolicyAdjustment {
            decision_id: "decision:exact-profile".into(),
            policy_profile_ref: profile.profile_id.clone(),
            policy_profile_fingerprint: profile.fingerprint().unwrap(),
            observation_refs: vec!["observation:vitality:1".into()],
            rule_ref: "rule:countercyclical:v1".into(),
            authority_ref: "authority:dao-1".into(),
            adjustment: oracle.generate_adjustment(),
            decided_at: 1_000,
        };

        assert!(oracle
            .apply_governed_adjustment_for_profile(decision.clone(), &profile)
            .is_ok());

        let mut altered = profile.clone();
        altered.policy_version = "1.0.1".into();
        let second = GovernedPolicyAdjustment {
            decision_id: "decision:altered-profile".into(),
            ..decision
        };
        assert!(oracle
            .apply_governed_adjustment_for_profile(second, &altered)
            .is_err());
    }

    #[test]
    fn test_governed_policy_rejects_duplicate_decision() {
        let mut oracle = MetabolicOracle::new();
        let decision = GovernedPolicyAdjustment {
            decision_id: "decision:1".into(),
            policy_profile_ref: "profile:za:reference:v1".into(),
            policy_profile_fingerprint: "a".repeat(64),
            observation_refs: vec!["observation:vitality:1".into()],
            rule_ref: "rule:countercyclical:v1".into(),
            authority_ref: "authority:dao-1".into(),
            adjustment: oracle.generate_adjustment(),
            decided_at: 1_000,
        };
        let second = decision.clone();

        oracle.apply_governed_adjustment(decision).unwrap();
        assert!(oracle.apply_governed_adjustment(second).is_err());
    }

    #[test]
    fn test_governed_policy_rejects_non_finite_factor() {
        let mut oracle = MetabolicOracle::new();
        let mut adjustment = oracle.generate_adjustment();
        adjustment.fee_rate_factor = f64::NAN;
        let decision = GovernedPolicyAdjustment {
            decision_id: "decision:1".into(),
            observation_refs: vec!["observation:vitality:1".into()],
            rule_ref: "rule:countercyclical:v1".into(),
            authority_ref: "authority:dao-1".into(),
            adjustment,
            decided_at: 1_000,
        };

        assert!(oracle.apply_governed_adjustment(decision).is_err());
    }

    #[test]
    fn test_bounds_enforcement() {
        let mut oracle = MetabolicOracle::new();
        oracle.current_params.fee_rate = 0.0001; // At minimum

        let extreme = PolicyAdjustment {
            fee_rate_factor: 0.1,
            demurrage_rate_factor: 1.0,
            velocity_incentive: 1.0,
            tend_limit_tier: TendLimitTier::Normal,
            emergency_release: None,
            reason: "Test".to_string(),
            requires_approval: false,
        };

        oracle.apply_adjustment(&extreme).unwrap();
        assert!((oracle.current_params.fee_rate - 0.0001).abs() < 0.00001);
    }
}
