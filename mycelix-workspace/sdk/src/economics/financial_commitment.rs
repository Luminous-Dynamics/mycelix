// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Descriptive Minsky-style financial-commitment coverage.
//!
//! This module intentionally does not forecast cash flow, create debt, trigger
//! refinancing, or mutate economic state. It classifies an explicitly supplied
//! cash-flow measure against explicitly supplied contractual obligations.

use serde::{Deserialize, Serialize};

use super::observables::{classify_financing_regime, FinancingRegime};

/// A single-period financial-commitment coverage observation.
///
/// All three inputs are caller/model supplied. In particular, cash-flow
/// availability is never inferred from revenue, profit, liquidity, or actual
/// repayment.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct FinancialCommitmentObservation {
    pub cash_flow_available: i128,
    pub interest_due: i128,
    pub principal_due: i128,
}

impl FinancialCommitmentObservation {
    pub fn new(
        cash_flow_available: i128,
        interest_due: i128,
        principal_due: i128,
    ) -> Result<Self, String> {
        if interest_due < 0 || principal_due < 0 {
            return Err("financial commitments must be non-negative".into());
        }
        interest_due
            .checked_add(principal_due)
            .ok_or_else(|| "financial commitment total overflow".to_string())?;

        Ok(Self {
            cash_flow_available,
            interest_due,
            principal_due,
        })
    }

    pub fn total_due(&self) -> Result<i128, String> {
        self.interest_due
            .checked_add(self.principal_due)
            .ok_or_else(|| "financial commitment total overflow".into())
    }

    /// Positive amount by which available cash flow falls below total debt
    /// service; zero when total service is covered.
    pub fn service_shortfall(&self) -> Result<i128, String> {
        self.total_due()?
            .checked_sub(self.cash_flow_available)
            .map(|shortfall| shortfall.max(0))
            .ok_or_else(|| "financial service shortfall overflow".into())
    }

    /// Descriptive Minsky financing classification based solely on the
    /// explicitly supplied cash-flow and contractual obligations.
    pub fn regime(&self) -> Result<FinancingRegime, String> {
        classify_financing_regime(
            self.cash_flow_available,
            self.interest_due,
            self.principal_due,
        )
    }

    /// True exactly when cash flow covers all explicitly supplied obligations.
    pub fn covers_total_service(&self) -> Result<bool, String> {
        Ok(self.cash_flow_available >= self.total_due()?)
    }

    /// True exactly when cash flow covers interest, regardless of principal.
    pub fn covers_interest(&self) -> bool {
        self.cash_flow_available >= self.interest_due
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn commitment_coverage_maps_directly_to_minsky_regimes() {
        let hedge = FinancialCommitmentObservation::new(150, 100, 50).unwrap();
        assert_eq!(hedge.regime().unwrap(), FinancingRegime::Hedge);
        assert!(hedge.covers_total_service().unwrap());
        assert_eq!(hedge.service_shortfall().unwrap(), 0);

        let speculative = FinancialCommitmentObservation::new(120, 100, 50).unwrap();
        assert_eq!(speculative.regime().unwrap(), FinancingRegime::Speculative);
        assert!(!speculative.covers_total_service().unwrap());
        assert!(speculative.covers_interest());
        assert_eq!(speculative.service_shortfall().unwrap(), 30);

        let ponzi = FinancialCommitmentObservation::new(80, 100, 50).unwrap();
        assert_eq!(ponzi.regime().unwrap(), FinancingRegime::Ponzi);
        assert!(!ponzi.covers_interest());
        assert_eq!(ponzi.service_shortfall().unwrap(), 70);
    }

    #[test]
    fn commitment_diagnostic_rejects_invalid_obligations() {
        assert!(FinancialCommitmentObservation::new(10, -1, 2).is_err());
        assert!(FinancialCommitmentObservation::new(10, i128::MAX, 1).is_err());
    }

    #[test]
    fn commitment_diagnostic_fails_closed_on_shortfall_overflow() {
        let observation =
            FinancialCommitmentObservation::new(i128::MIN, 0, i128::MAX).unwrap();
        assert!(observation.service_shortfall().is_err());
    }
}
    #[test]
    fn commitment_boundaries_are_exact_and_non_predictive() {
        let exact_interest = FinancialCommitmentObservation::new(100, 100, 50).unwrap();
        assert_eq!(exact_interest.regime().unwrap(), FinancingRegime::Speculative);
        assert!(exact_interest.covers_interest());
        assert!(!exact_interest.covers_total_service().unwrap());

        let exact_total = FinancialCommitmentObservation::new(150, 100, 50).unwrap();
        assert_eq!(exact_total.regime().unwrap(), FinancingRegime::Hedge);
        assert!(exact_total.covers_total_service().unwrap());

        let below_interest = FinancialCommitmentObservation::new(99, 100, 50).unwrap();
        assert_eq!(below_interest.regime().unwrap(), FinancingRegime::Ponzi);

        let zero_commitment = FinancialCommitmentObservation::new(0, 0, 0).unwrap();
        assert_eq!(zero_commitment.regime().unwrap(), FinancingRegime::Hedge);
        assert!(zero_commitment.covers_total_service().unwrap());
        assert_eq!(zero_commitment.service_shortfall().unwrap(), 0);
    }

    #[test]
    fn commitment_observation_preserves_negative_available_cash_flow_as_input() {
        let observation = FinancialCommitmentObservation::new(-10, 5, 5).unwrap();
        assert_eq!(observation.cash_flow_available, -10);
        assert!(!observation.covers_interest());
        assert_eq!(observation.service_shortfall().unwrap(), 20);
        assert_eq!(observation.regime().unwrap(), FinancingRegime::Ponzi);
    }

