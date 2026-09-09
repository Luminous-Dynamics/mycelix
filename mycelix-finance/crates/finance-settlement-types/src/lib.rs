//! Settlement boundary types for Mycelix Finance.
//!
//! This crate deliberately distinguishes value somebody owns from value the
//! network merely owes, holds, escrows, or cannot yet attribute. Member-balance
//! disposition policy must never become a shortcut for disposing of third-party
//! value.

use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ValueClass {
    OwnedBalance,
    ThirdPartyCustody,
    AccruedObligation,
    EscrowedValue,
    SuspenseValue,
}

impl ValueClass {
    /// Only value already owned as an ordinary balance may use ordinary
    /// member-exit/succession disposition rules.
    #[must_use]
    pub const fn permits_owned_balance_disposition(self) -> bool {
        matches!(self, Self::OwnedBalance)
    }

    /// Value which must retain an external beneficiary/authority lineage rather
    /// than being swept by ordinary balance disposition.
    #[must_use]
    pub const fn requires_external_value_lineage(self) -> bool {
        !self.permits_owned_balance_disposition()
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct IncompatibleValueDisposition {
    pub value_class: ValueClass,
}

impl core::fmt::Display for IncompatibleValueDisposition {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(
            f,
            "ordinary owned-balance disposition is not valid for {:?}",
            self.value_class
        )
    }
}

impl std::error::Error for IncompatibleValueDisposition {}

pub fn require_owned_balance_disposition(
    value_class: ValueClass,
) -> Result<(), IncompatibleValueDisposition> {
    if value_class.permits_owned_balance_disposition() {
        Ok(())
    } else {
        Err(IncompatibleValueDisposition { value_class })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn only_owned_balance_accepts_owned_balance_disposition() {
        assert!(require_owned_balance_disposition(ValueClass::OwnedBalance).is_ok());
        for class in [
            ValueClass::ThirdPartyCustody,
            ValueClass::AccruedObligation,
            ValueClass::EscrowedValue,
            ValueClass::SuspenseValue,
        ] {
            assert!(require_owned_balance_disposition(class).is_err());
            assert!(class.requires_external_value_lineage());
        }
    }
}
