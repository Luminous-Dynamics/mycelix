use finance_wire_types::{AccountId, Money};
use serde::{Deserialize, Serialize};

const MAX_HANDLE_LEN: usize = 256;

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum SettlementTypeError {
    EmptyField(&'static str),
    InvalidRouteHandle,
    NonPositiveAmount,
}

impl core::fmt::Display for SettlementTypeError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::EmptyField(field) => write!(f, "{field} must be non-empty"),
            Self::InvalidRouteHandle => write!(f, "settlement route handle is invalid"),
            Self::NonPositiveAmount => write!(f, "settlement amount must be positive"),
        }
    }
}

impl std::error::Error for SettlementTypeError {}

#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct SettlementRouteHandle(String);

impl SettlementRouteHandle {
    pub fn new(value: impl Into<String>) -> Result<Self, SettlementTypeError> {
        let value = value.into();
        if !value.starts_with("route:")
            || value.len() > MAX_HANDLE_LEN
            || value.chars().any(char::is_whitespace)
        {
            return Err(SettlementTypeError::InvalidRouteHandle);
        }
        Ok(Self(value))
    }

    #[must_use]
    pub fn as_str(&self) -> &str {
        &self.0
    }
}

/// Economic instruction. It contains an opaque authorized route handle, never
/// a bank account, wallet secret, provider credential, or private key.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SettlementInstruction {
    pub instruction_id: String,
    pub beneficiary: AccountId,
    pub payee: AccountId,
    pub amount: Money,
    pub obligation_set_digest: String,
    pub route_authority_ref: String,
    pub route: SettlementRouteHandle,
}

impl SettlementInstruction {
    pub fn validate(&self) -> Result<(), SettlementTypeError> {
        for (label, value) in [
            ("instruction_id", self.instruction_id.as_str()),
            ("obligation_set_digest", self.obligation_set_digest.as_str()),
            ("route_authority_ref", self.route_authority_ref.as_str()),
        ] {
            if value.trim().is_empty() {
                return Err(SettlementTypeError::EmptyField(label));
            }
        }
        if self.amount.amount_minor <= 0 {
            return Err(SettlementTypeError::NonPositiveAmount);
        }
        Ok(())
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SettlementState {
    Authorized,
    Submitted,
    Accepted,
    Confirmed,
    Finalized,
    Failed,
    Rejected,
    Reversed,
    Disputed,
}

impl SettlementState {
    /// Rail state never owns the underlying debt. In particular Failed,
    /// Rejected, Reversed and Disputed require resolution/retry rather than
    /// recreation or deletion of the royalty obligation.
    #[must_use]
    pub const fn cancels_underlying_obligation(self) -> bool {
        false
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RailExecutionReceipt {
    pub instruction_id: String,
    pub rail_id: String,
    pub external_receipt_ref: String,
    pub state: SettlementState,
    pub observed_at_micros: u64,
}

impl RailExecutionReceipt {
    pub fn validate(&self) -> Result<(), SettlementTypeError> {
        for (label, value) in [
            ("instruction_id", self.instruction_id.as_str()),
            ("rail_id", self.rail_id.as_str()),
            ("external_receipt_ref", self.external_receipt_ref.as_str()),
        ] {
            if value.trim().is_empty() {
                return Err(SettlementTypeError::EmptyField(label));
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn route_handle_is_opaque_and_namespaced() {
        assert!(SettlementRouteHandle::new("route:creator-alice-primary").is_ok());
        assert!(SettlementRouteHandle::new("bank account 1234").is_err());
        assert!(SettlementRouteHandle::new("route:contains whitespace").is_err());
    }

    #[test]
    fn external_failure_never_cancels_the_obligation() {
        for state in [
            SettlementState::Failed,
            SettlementState::Rejected,
            SettlementState::Reversed,
            SettlementState::Disputed,
        ] {
            assert!(!state.cancels_underlying_obligation());
        }
    }
}
