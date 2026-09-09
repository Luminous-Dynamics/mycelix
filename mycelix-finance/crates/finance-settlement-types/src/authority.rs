use std::collections::BTreeSet;

use finance_wire_types::Currency;
use serde::{Deserialize, Serialize};

use crate::SettlementInstruction;

pub const SETTLEMENT_EXECUTION_SCOPE: &str = "finance.settlement.execute";

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct CapabilityScope {
    pub action: String,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ThresholdPolicy {
    pub required_signers: u16,
    pub authorized_signers: Vec<String>,
}

impl ThresholdPolicy {
    pub fn validate(&self) -> Result<(), ExecutionAuthorityError> {
        if self.required_signers == 0 {
            return Err(ExecutionAuthorityError::InvalidThreshold);
        }
        let authorized: BTreeSet<&str> = self
            .authorized_signers
            .iter()
            .map(String::as_str)
            .filter(|value| !value.trim().is_empty())
            .collect();
        if authorized.len() != self.authorized_signers.len()
            || usize::from(self.required_signers) > authorized.len()
        {
            return Err(ExecutionAuthorityError::InvalidThreshold);
        }
        Ok(())
    }

    pub fn is_satisfied_by(&self, signer_ids: &[String]) -> Result<bool, ExecutionAuthorityError> {
        self.validate()?;
        let authorized: BTreeSet<&str> = self.authorized_signers.iter().map(String::as_str).collect();
        let presented: BTreeSet<&str> = signer_ids
            .iter()
            .map(String::as_str)
            .filter(|id| authorized.contains(id))
            .collect();
        Ok(presented.len() >= usize::from(self.required_signers))
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ExecutionLimits {
    pub currency: Currency,
    pub max_amount_minor: i128,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct EffectivePeriod {
    pub not_before_micros: u64,
    pub not_after_micros: Option<u64>,
}

impl EffectivePeriod {
    #[must_use]
    pub fn contains(&self, now_micros: u64) -> bool {
        now_micros >= self.not_before_micros
            && self
                .not_after_micros
                .map(|end| now_micros <= end)
                .unwrap_or(true)
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ExecutionAuthority {
    pub scope: CapabilityScope,
    pub threshold: ThresholdPolicy,
    pub limits: ExecutionLimits,
    pub effective_period: EffectivePeriod,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ExecutionAuthorityError {
    WrongScope,
    InvalidThreshold,
    InvalidLimits,
    InvalidEffectivePeriod,
    NotCurrentlyEffective,
    CurrencyNotAuthorized,
    AmountExceedsLimit,
    ThresholdNotMet,
    InvalidInstruction(String),
}

impl core::fmt::Display for ExecutionAuthorityError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::WrongScope => write!(f, "execution authority does not cover settlement execution"),
            Self::InvalidThreshold => write!(f, "threshold policy is invalid"),
            Self::InvalidLimits => write!(f, "execution limits are invalid"),
            Self::InvalidEffectivePeriod => write!(f, "effective period is invalid"),
            Self::NotCurrentlyEffective => write!(f, "execution authority is not currently effective"),
            Self::CurrencyNotAuthorized => write!(f, "settlement currency is not authorized"),
            Self::AmountExceedsLimit => write!(f, "settlement amount exceeds execution limit"),
            Self::ThresholdNotMet => write!(f, "execution signer threshold was not met"),
            Self::InvalidInstruction(error) => write!(f, "settlement instruction is invalid: {error}"),
        }
    }
}

impl std::error::Error for ExecutionAuthorityError {}

impl ExecutionAuthority {
    pub fn validate(&self) -> Result<(), ExecutionAuthorityError> {
        if self.scope.action != SETTLEMENT_EXECUTION_SCOPE {
            return Err(ExecutionAuthorityError::WrongScope);
        }
        self.threshold.validate()?;
        if self.limits.max_amount_minor <= 0 {
            return Err(ExecutionAuthorityError::InvalidLimits);
        }
        if self
            .effective_period
            .not_after_micros
            .is_some_and(|end| end < self.effective_period.not_before_micros)
        {
            return Err(ExecutionAuthorityError::InvalidEffectivePeriod);
        }
        Ok(())
    }

    /// Validate authority for one exact settlement instruction. This method
    /// grants no side effect; callers still need an execution runtime/fence and
    /// rail-specific currentness/finality handling.
    pub fn authorize(
        &self,
        instruction: &SettlementInstruction,
        signer_ids: &[String],
        now_micros: u64,
    ) -> Result<(), ExecutionAuthorityError> {
        self.validate()?;
        instruction
            .validate()
            .map_err(|error| ExecutionAuthorityError::InvalidInstruction(error.to_string()))?;
        if !self.effective_period.contains(now_micros) {
            return Err(ExecutionAuthorityError::NotCurrentlyEffective);
        }
        if instruction.amount.currency != self.limits.currency {
            return Err(ExecutionAuthorityError::CurrencyNotAuthorized);
        }
        if instruction.amount.amount_minor > self.limits.max_amount_minor {
            return Err(ExecutionAuthorityError::AmountExceedsLimit);
        }
        if !self.threshold.is_satisfied_by(signer_ids)? {
            return Err(ExecutionAuthorityError::ThresholdNotMet);
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn threshold() -> ThresholdPolicy {
        ThresholdPolicy {
            required_signers: 2,
            authorized_signers: vec!["treasury:a".into(), "treasury:b".into(), "treasury:c".into()],
        }
    }

    #[test]
    fn threshold_counts_only_unique_authorized_signers() {
        let policy = threshold();
        assert_eq!(
            policy
                .is_satisfied_by(&["treasury:a".into(), "treasury:a".into(), "outsider".into()])
                .unwrap(),
            false
        );
        assert!(policy
            .is_satisfied_by(&["treasury:a".into(), "treasury:b".into()])
            .unwrap());
    }

    #[test]
    fn malformed_thresholds_fail_closed() {
        let policy = ThresholdPolicy {
            required_signers: 2,
            authorized_signers: vec!["treasury:a".into()],
        };
        assert_eq!(policy.validate(), Err(ExecutionAuthorityError::InvalidThreshold));
    }

    #[test]
    fn effective_period_has_explicit_boundaries() {
        let period = EffectivePeriod {
            not_before_micros: 100,
            not_after_micros: Some(200),
        };
        assert!(!period.contains(99));
        assert!(period.contains(100));
        assert!(period.contains(200));
        assert!(!period.contains(201));
    }
}
