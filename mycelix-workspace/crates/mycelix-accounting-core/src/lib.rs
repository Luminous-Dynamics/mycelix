#![forbid(unsafe_code)]
//! Transport-neutral accounting primitives for Mycelix.
//!
//! This crate deliberately owns no persistence, payment rail, Holochain, clock,
//! authority resolution, or external-effect capability. It models exact economic
//! records and invariants that owning domains can qualify independently.

pub mod ledger;

/// Maximum UTF-8 bytes accepted by the small opaque reference vocabulary.
pub const MAX_REF_BYTES: usize = 256;

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum RefError {
    Empty,
    TooLong { actual: usize, max: usize },
}

macro_rules! bounded_ref {
    ($name:ident) => {
        #[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
        pub struct $name(String);

        impl $name {
            pub fn new(value: impl Into<String>) -> Result<Self, RefError> {
                let value = value.into();
                if value.is_empty() {
                    return Err(RefError::Empty);
                }
                if value.len() > MAX_REF_BYTES {
                    return Err(RefError::TooLong {
                        actual: value.len(),
                        max: MAX_REF_BYTES,
                    });
                }
                Ok(Self(value))
            }

            pub fn as_str(&self) -> &str {
                &self.0
            }
        }
    };
}

bounded_ref!(AssetId);
bounded_ref!(AccountRef);
bounded_ref!(AuthorityRef);
bounded_ref!(LedgerEntryId);

/// Algorithm-qualified digest bytes are intentionally represented separately
/// from any claim that a particular algorithm has been cryptographically qualified.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Digest {
    algorithm: String,
    bytes: Vec<u8>,
}

impl Digest {
    pub fn new(algorithm: impl Into<String>, bytes: Vec<u8>) -> Result<Self, RefError> {
        let algorithm = algorithm.into();
        if algorithm.is_empty() || bytes.is_empty() {
            return Err(RefError::Empty);
        }
        if algorithm.len() > MAX_REF_BYTES {
            return Err(RefError::TooLong {
                actual: algorithm.len(),
                max: MAX_REF_BYTES,
            });
        }
        Ok(Self { algorithm, bytes })
    }

    pub fn algorithm(&self) -> &str {
        &self.algorithm
    }

    pub fn bytes(&self) -> &[u8] {
        &self.bytes
    }
}

/// Non-negative amount in the smallest unit of one exact asset/currency profile.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Money {
    asset: AssetId,
    units: u128,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum MoneyError {
    AssetMismatch,
    Overflow,
    Underflow,
}

impl Money {
    pub fn new(asset: AssetId, units: u128) -> Self {
        Self { asset, units }
    }

    pub fn zero(asset: AssetId) -> Self {
        Self { asset, units: 0 }
    }

    pub fn asset(&self) -> &AssetId {
        &self.asset
    }

    pub fn units(&self) -> u128 {
        self.units
    }

    pub fn checked_add(&self, other: &Self) -> Result<Self, MoneyError> {
        if self.asset != other.asset {
            return Err(MoneyError::AssetMismatch);
        }
        let units = self.units.checked_add(other.units).ok_or(MoneyError::Overflow)?;
        Ok(Self::new(self.asset.clone(), units))
    }

    pub fn checked_sub(&self, other: &Self) -> Result<Self, MoneyError> {
        if self.asset != other.asset {
            return Err(MoneyError::AssetMismatch);
        }
        let units = self.units.checked_sub(other.units).ok_or(MoneyError::Underflow)?;
        Ok(Self::new(self.asset.clone(), units))
    }
}
