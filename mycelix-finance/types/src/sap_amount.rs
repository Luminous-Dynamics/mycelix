// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Canonical value semantics for the SAP circulation currency.
//!
//! `SapAmount` is a pure quantity type. Constructing, deserializing, comparing,
//! or encoding one does not authorize a transfer, mint, reserve, allocation, or
//! any other economic action.

use serde::{Deserialize, Serialize};

use crate::Currency;

/// Arithmetic failures for [`SapAmount`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SapAmountArithmeticError {
    /// Addition exceeded the finite `u64` micro-SAP domain.
    Overflow,
    /// Subtraction would have produced a negative SAP quantity.
    Underflow,
}

impl core::fmt::Display for SapAmountArithmeticError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Overflow => f.write_str("SAP amount overflow"),
            Self::Underflow => f.write_str("SAP amount underflow"),
        }
    }
}

impl std::error::Error for SapAmountArithmeticError {}

/// A finite unsigned SAP quantity expressed in canonical micro-SAP base units.
///
/// The inner integer is deliberately private. V1 exposes only explicit
/// micro-SAP construction/access and checked arithmetic; it intentionally does
/// not implement implicit integer conversions or arithmetic operators.
#[derive(
    Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize,
)]
pub struct SapAmount(u64);

impl SapAmount {
    /// Zero micro-SAP. Zero is a valid quantity; operation policy decides
    /// whether zero is valid for any particular economic action.
    pub const ZERO: Self = Self(0);

    /// Number of micro-SAP base units in one SAP.
    pub const MICRO_SAP_PER_SAP: u64 = 1_000_000;

    /// Exact V1 authority-domain bytes.
    pub const AUTHORITY_DOMAIN: [u8; 26] = *b"mycelix/finance/sap-amount";

    /// V1 canonical authority-encoding version.
    pub const AUTHORITY_VERSION: u8 = 1;

    /// Fixed byte length of the V1 canonical authority representation.
    pub const AUTHORITY_BYTES_LEN: usize = 36;

    /// Construct explicitly from canonical micro-SAP base units.
    pub const fn from_micro_sap(value: u64) -> Self {
        Self(value)
    }

    /// Return the canonical micro-SAP base-unit quantity.
    pub const fn as_micro_sap(self) -> u64 {
        self.0
    }

    /// SAP amounts are permanently bound to the existing canonical SAP currency.
    pub const fn currency(self) -> Currency {
        Currency::Sap
    }

    /// Checked addition in the finite micro-SAP quantity domain.
    pub fn checked_add(self, rhs: Self) -> Result<Self, SapAmountArithmeticError> {
        self.0
            .checked_add(rhs.0)
            .map(Self)
            .ok_or(SapAmountArithmeticError::Overflow)
    }

    /// Checked subtraction in the unsigned micro-SAP quantity domain.
    pub fn checked_sub(self, rhs: Self) -> Result<Self, SapAmountArithmeticError> {
        self.0
            .checked_sub(rhs.0)
            .map(Self)
            .ok_or(SapAmountArithmeticError::Underflow)
    }

    /// Return the deterministic V1 semantic-authority representation.
    ///
    /// Layout:
    /// - bytes 0..26: `mycelix/finance/sap-amount`
    /// - byte 26: zero domain terminator
    /// - byte 27: encoding version 1
    /// - bytes 28..36: unsigned micro-SAP as big-endian `u64`
    ///
    /// These bytes are not themselves a digest, signature, authentication
    /// proof, transaction, or authorization token.
    pub const fn authority_bytes(self) -> [u8; Self::AUTHORITY_BYTES_LEN] {
        let mut out = [0_u8; Self::AUTHORITY_BYTES_LEN];
        let mut i = 0;
        while i < Self::AUTHORITY_DOMAIN.len() {
            out[i] = Self::AUTHORITY_DOMAIN[i];
            i += 1;
        }

        out[26] = 0;
        out[27] = Self::AUTHORITY_VERSION;

        let value = self.0.to_be_bytes();
        out[28] = value[0];
        out[29] = value[1];
        out[30] = value[2];
        out[31] = value[3];
        out[32] = value[4];
        out[33] = value[5];
        out[34] = value[6];
        out[35] = value[7];
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::fmt::Write as _;

    fn hex(bytes: &[u8]) -> String {
        let mut out = String::with_capacity(bytes.len() * 2);
        for byte in bytes {
            write!(&mut out, "{byte:02x}").expect("writing to String cannot fail");
        }
        out
    }

    #[test]
    fn zero_is_a_valid_quantity_without_operation_claims() {
        assert_eq!(SapAmount::ZERO.as_micro_sap(), 0);
        assert_eq!(SapAmount::ZERO.currency(), Currency::Sap);
    }

    #[test]
    fn one_sap_is_one_million_micro_sap() {
        assert_eq!(SapAmount::MICRO_SAP_PER_SAP, 1_000_000);
        assert_eq!(
            SapAmount::from_micro_sap(SapAmount::MICRO_SAP_PER_SAP).as_micro_sap(),
            1_000_000
        );
    }

    #[test]
    fn currency_identity_is_always_sap() {
        for amount in [0, 1, 1_000_000, 10_000_000_000, u64::MAX] {
            assert_eq!(SapAmount::from_micro_sap(amount).currency(), Currency::Sap);
        }
    }

    #[test]
    fn checked_arithmetic_distinguishes_overflow_and_underflow() {
        assert_eq!(
            SapAmount::from_micro_sap(u64::MAX).checked_add(SapAmount::from_micro_sap(1)),
            Err(SapAmountArithmeticError::Overflow)
        );
        assert_eq!(
            SapAmount::ZERO.checked_sub(SapAmount::from_micro_sap(1)),
            Err(SapAmountArithmeticError::Underflow)
        );
        assert_eq!(
            SapAmount::from_micro_sap(7)
                .checked_add(SapAmount::from_micro_sap(5))
                .unwrap(),
            SapAmount::from_micro_sap(12)
        );
        assert_eq!(
            SapAmount::from_micro_sap(12)
                .checked_sub(SapAmount::from_micro_sap(5))
                .unwrap(),
            SapAmount::from_micro_sap(7)
        );
    }

    #[test]
    fn ordering_tracks_micro_sap_quantity() {
        assert!(SapAmount::ZERO < SapAmount::from_micro_sap(1));
        assert!(SapAmount::from_micro_sap(1) < SapAmount::from_micro_sap(1_000_000));
        assert!(SapAmount::from_micro_sap(1_000_000) < SapAmount::from_micro_sap(u64::MAX));
    }

    #[test]
    fn canonical_authority_bytes_are_fixed_length() {
        for amount in [0, 1, 1_000_000, 10_000_000_000, u64::MAX] {
            assert_eq!(
                SapAmount::from_micro_sap(amount).authority_bytes().len(),
                SapAmount::AUTHORITY_BYTES_LEN
            );
        }
        assert_eq!(SapAmount::AUTHORITY_BYTES_LEN, 36);
        assert_eq!(SapAmount::AUTHORITY_DOMAIN.len(), 26);
        assert_eq!(SapAmount::AUTHORITY_VERSION, 1);
    }

    #[test]
    fn canonical_authority_golden_vectors_are_frozen() {
        let vectors = [
            (
                0,
                "6d7963656c69782f66696e616e63652f7361702d616d6f756e7400010000000000000000",
            ),
            (
                1,
                "6d7963656c69782f66696e616e63652f7361702d616d6f756e7400010000000000000001",
            ),
            (
                1_000_000,
                "6d7963656c69782f66696e616e63652f7361702d616d6f756e74000100000000000f4240",
            ),
            (
                10_000_000_000,
                "6d7963656c69782f66696e616e63652f7361702d616d6f756e74000100000002540be400",
            ),
            (
                u64::MAX,
                "6d7963656c69782f66696e616e63652f7361702d616d6f756e740001ffffffffffffffff",
            ),
        ];

        for (amount, expected) in vectors {
            assert_eq!(hex(&SapAmount::from_micro_sap(amount).authority_bytes()), expected);
        }
    }

    #[test]
    fn serde_roundtrip_preserves_quantity_but_is_not_authority_encoding() {
        let amount = SapAmount::from_micro_sap(42_000_001);
        let json = serde_json::to_string(&amount).unwrap();
        let parsed: SapAmount = serde_json::from_str(&json).unwrap();
        assert_eq!(parsed, amount);

        let authority = amount.authority_bytes();
        assert_ne!(json.as_bytes(), authority.as_slice());
        assert_eq!(authority.len(), 36);
    }
}
