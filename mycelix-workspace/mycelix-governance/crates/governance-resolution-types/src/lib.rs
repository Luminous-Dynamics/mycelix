// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

#![forbid(unsafe_code)]

//! Transport-neutral, versioned outcome vocabulary for authorized Governance
//! resolutions.
//!
//! This crate owns the semantic shape of a Governance resolution outcome. It has
//! no Holochain, Business, Finance, database, network, clock, or process API.
//! Runtime zomes may persist/verify these values, while downstream bridges may
//! translate them into domain-specific execution expectations without teaching
//! the downstream Business layer how to interpret Governance payloads.

use core::fmt;
use serde::{Deserialize, Deserializer, Serialize};

/// Semantic profile name for an authorized refund resolution outcome.
pub const AUTHORIZED_REFUND_RESOLUTION_PROFILE: &str =
    "governance.authorized-refund-resolution";
/// Semantic profile version for an authorized refund resolution outcome.
pub const AUTHORIZED_REFUND_RESOLUTION_VERSION: u32 = 1;
/// Semantic profile name for an authorized retained-exception resolution.
pub const AUTHORIZED_EXCEPTION_RESOLUTION_PROFILE: &str =
    "governance.authorized-exception-resolution";
/// Semantic profile version for an authorized retained-exception resolution.
pub const AUTHORIZED_EXCEPTION_RESOLUTION_VERSION: u32 = 1;

/// Static semantic-profile descriptor owned by this vocabulary version.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ResolutionSemanticProfile {
    pub name: &'static str,
    pub version: u32,
}

/// Construction / deserialization failures for resolution outcomes.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ResolutionOutcomeError {
    EmptyEffectId,
    EmptyOriginalOrderRef,
    EmptyBeneficiaryRef,
    EmptyUnit,
    ZeroRefundAmount,
    EmptyExceptionDomain,
    EmptyExceptionId,
}

impl fmt::Display for ResolutionOutcomeError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Self::EmptyEffectId => "refund effect id must not be empty",
            Self::EmptyOriginalOrderRef => "original order reference must not be empty",
            Self::EmptyBeneficiaryRef => "refund beneficiary reference must not be empty",
            Self::EmptyUnit => "refund unit/currency must not be empty",
            Self::ZeroRefundAmount => "refund amount must be greater than zero",
            Self::EmptyExceptionDomain => "exception domain must not be empty",
            Self::EmptyExceptionId => "exception id must not be empty",
        })
    }
}

impl std::error::Error for ResolutionOutcomeError {}

/// Exact material expected for a compensating refund effect.
///
/// `effect_id` is an opaque Governance-authorized identity for the expected
/// logical effect. The remaining fields are the material operation semantics
/// which a bridge may deterministically commit into an execution-domain intent.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize)]
pub struct RefundResolutionV1 {
    effect_id: String,
    original_order_ref: String,
    beneficiary_ref: String,
    unit: String,
    amount_minor: u64,
}

impl RefundResolutionV1 {
    pub fn new(
        effect_id: impl Into<String>,
        original_order_ref: impl Into<String>,
        beneficiary_ref: impl Into<String>,
        unit: impl Into<String>,
        amount_minor: u64,
    ) -> Result<Self, ResolutionOutcomeError> {
        let value = Self {
            effect_id: effect_id.into(),
            original_order_ref: original_order_ref.into(),
            beneficiary_ref: beneficiary_ref.into(),
            unit: unit.into(),
            amount_minor,
        };
        value.validate()?;
        Ok(value)
    }

    pub fn validate(&self) -> Result<(), ResolutionOutcomeError> {
        require_text(&self.effect_id, ResolutionOutcomeError::EmptyEffectId)?;
        require_text(
            &self.original_order_ref,
            ResolutionOutcomeError::EmptyOriginalOrderRef,
        )?;
        require_text(
            &self.beneficiary_ref,
            ResolutionOutcomeError::EmptyBeneficiaryRef,
        )?;
        require_text(&self.unit, ResolutionOutcomeError::EmptyUnit)?;
        if self.amount_minor == 0 {
            return Err(ResolutionOutcomeError::ZeroRefundAmount);
        }
        Ok(())
    }

    #[must_use]
    pub fn effect_id(&self) -> &str {
        &self.effect_id
    }

    #[must_use]
    pub fn original_order_ref(&self) -> &str {
        &self.original_order_ref
    }

    #[must_use]
    pub fn beneficiary_ref(&self) -> &str {
        &self.beneficiary_ref
    }

    #[must_use]
    pub fn unit(&self) -> &str {
        &self.unit
    }

    #[must_use]
    pub const fn amount_minor(&self) -> u64 {
        self.amount_minor
    }

    /// Collision-resistant textual commitment preimage for the material refund
    /// semantics under v1.
    ///
    /// Variable text fields use UTF-8 byte-length prefixes, so delimiter-bearing
    /// identifiers cannot become ambiguous by concatenation.
    #[must_use]
    pub fn canonical_material_v1(&self) -> String {
        let mut out = String::from("governance-refund-material-v1");
        push_text_field(&mut out, "order", &self.original_order_ref);
        push_text_field(&mut out, "beneficiary", &self.beneficiary_ref);
        push_text_field(&mut out, "unit", &self.unit);
        out.push_str("|amount-minor:");
        out.push_str(&self.amount_minor.to_string());
        out
    }
}

impl<'de> Deserialize<'de> for RefundResolutionV1 {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        #[derive(Deserialize)]
        struct Wire {
            effect_id: String,
            original_order_ref: String,
            beneficiary_ref: String,
            unit: String,
            amount_minor: u64,
        }

        let wire = Wire::deserialize(deserializer)?;
        Self::new(
            wire.effect_id,
            wire.original_order_ref,
            wire.beneficiary_ref,
            wire.unit,
            wire.amount_minor,
        )
        .map_err(serde::de::Error::custom)
    }
}

/// Exact retained exception identity authorized by Governance.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize)]
pub struct RetainedExceptionResolutionV1 {
    exception_domain: String,
    exception_id: String,
}

impl RetainedExceptionResolutionV1 {
    pub fn new(
        exception_domain: impl Into<String>,
        exception_id: impl Into<String>,
    ) -> Result<Self, ResolutionOutcomeError> {
        let value = Self {
            exception_domain: exception_domain.into(),
            exception_id: exception_id.into(),
        };
        value.validate()?;
        Ok(value)
    }

    pub fn validate(&self) -> Result<(), ResolutionOutcomeError> {
        require_text(
            &self.exception_domain,
            ResolutionOutcomeError::EmptyExceptionDomain,
        )?;
        require_text(&self.exception_id, ResolutionOutcomeError::EmptyExceptionId)
    }

    #[must_use]
    pub fn exception_domain(&self) -> &str {
        &self.exception_domain
    }

    #[must_use]
    pub fn exception_id(&self) -> &str {
        &self.exception_id
    }
}

impl<'de> Deserialize<'de> for RetainedExceptionResolutionV1 {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        #[derive(Deserialize)]
        struct Wire {
            exception_domain: String,
            exception_id: String,
        }

        let wire = Wire::deserialize(deserializer)?;
        Self::new(wire.exception_domain, wire.exception_id).map_err(serde::de::Error::custom)
    }
}

/// Version-1 authoritative terminal outcome of a Governance resolution.
///
/// The enum is deliberately small. New terminal semantics should be added only
/// when an owning Governance process can define and qualify them precisely.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(tag = "kind", content = "value", rename_all = "snake_case")]
pub enum AuthorizedResolutionOutcomeV1 {
    Refund(RefundResolutionV1),
    RetainException(RetainedExceptionResolutionV1),
}

impl AuthorizedResolutionOutcomeV1 {
    #[must_use]
    pub const fn semantic_profile(&self) -> ResolutionSemanticProfile {
        match self {
            Self::Refund(_) => ResolutionSemanticProfile {
                name: AUTHORIZED_REFUND_RESOLUTION_PROFILE,
                version: AUTHORIZED_REFUND_RESOLUTION_VERSION,
            },
            Self::RetainException(_) => ResolutionSemanticProfile {
                name: AUTHORIZED_EXCEPTION_RESOLUTION_PROFILE,
                version: AUTHORIZED_EXCEPTION_RESOLUTION_VERSION,
            },
        }
    }

    pub fn validate(&self) -> Result<(), ResolutionOutcomeError> {
        match self {
            Self::Refund(value) => value.validate(),
            Self::RetainException(value) => value.validate(),
        }
    }
}

fn require_text(value: &str, error: ResolutionOutcomeError) -> Result<(), ResolutionOutcomeError> {
    if value.trim().is_empty() {
        Err(error)
    } else {
        Ok(())
    }
}

fn push_text_field(out: &mut String, name: &str, value: &str) {
    out.push('|');
    out.push_str(name);
    out.push(':');
    out.push_str(&value.len().to_string());
    out.push(':');
    out.push_str(value);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn refund_profile_and_canonical_material_are_stable() {
        let refund = RefundResolutionV1::new(
            "refund:order-1:5000",
            "order-1",
            "customer-a",
            "USD",
            5_000,
        )
        .unwrap();
        let outcome = AuthorizedResolutionOutcomeV1::Refund(refund.clone());

        assert_eq!(
            outcome.semantic_profile(),
            ResolutionSemanticProfile {
                name: AUTHORIZED_REFUND_RESOLUTION_PROFILE,
                version: 1,
            }
        );
        assert_eq!(refund.effect_id(), "refund:order-1:5000");
        assert_eq!(
            refund.canonical_material_v1(),
            "governance-refund-material-v1|order:7:order-1|beneficiary:10:customer-a|unit:3:USD|amount-minor:5000"
        );
    }

    #[test]
    fn material_change_changes_canonical_expectation() {
        let fifty = RefundResolutionV1::new(
            "refund:order-1",
            "order-1",
            "customer-a",
            "USD",
            5_000,
        )
        .unwrap();
        let sixty = RefundResolutionV1::new(
            "refund:order-1",
            "order-1",
            "customer-a",
            "USD",
            6_000,
        )
        .unwrap();

        assert_ne!(fifty, sixty);
        assert_ne!(
            fifty.canonical_material_v1(),
            sixty.canonical_material_v1()
        );
    }

    #[test]
    fn length_prefixing_prevents_delimiter_ambiguity() {
        let a = RefundResolutionV1::new("effect-a", "a|b", "c:d", "unit|x", 1).unwrap();
        let b = RefundResolutionV1::new("effect-b", "a", "b|c:d", "unit|x", 1).unwrap();

        assert_ne!(a.canonical_material_v1(), b.canonical_material_v1());
    }

    #[test]
    fn invalid_refund_fields_fail_closed() {
        assert_eq!(
            RefundResolutionV1::new(" ", "order", "beneficiary", "USD", 1),
            Err(ResolutionOutcomeError::EmptyEffectId)
        );
        assert_eq!(
            RefundResolutionV1::new("effect", "order", "beneficiary", "USD", 0),
            Err(ResolutionOutcomeError::ZeroRefundAmount)
        );
    }

    #[test]
    fn retained_exception_is_exact_and_versioned() {
        let retained = RetainedExceptionResolutionV1::new("finance", "chargeback:pending:1")
            .unwrap();
        let outcome = AuthorizedResolutionOutcomeV1::RetainException(retained.clone());

        assert_eq!(retained.exception_domain(), "finance");
        assert_eq!(retained.exception_id(), "chargeback:pending:1");
        assert_eq!(
            outcome.semantic_profile(),
            ResolutionSemanticProfile {
                name: AUTHORIZED_EXCEPTION_RESOLUTION_PROFILE,
                version: 1,
            }
        );
    }

    #[test]
    fn invalid_retained_exception_fails_closed() {
        assert_eq!(
            RetainedExceptionResolutionV1::new("finance", " "),
            Err(ResolutionOutcomeError::EmptyExceptionId)
        );
    }
}
