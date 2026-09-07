// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

#![forbid(unsafe_code)]

//! Pure v0.1 cross-domain mapping from typed Governance resolution outcomes to
//! provenance-bound Business terminal expectations.
//!
//! This crate does not fetch Governance state and does not prove that a supplied
//! outcome was institutionally authorized. The owning Governance verifier must
//! establish the exact source -> typed outcome relation. This bridge proves only
//! that the typed outcome is mapped deterministically under the expected
//! Governance profile and exact Business qualification cut.

use core::fmt;

use governance_resolution_types::{
    AUTHORIZED_EXCEPTION_RESOLUTION_PROFILE, AUTHORIZED_EXCEPTION_RESOLUTION_VERSION,
    AUTHORIZED_REFUND_RESOLUTION_PROFILE, AUTHORIZED_REFUND_RESOLUTION_VERSION,
    AuthorizedResolutionOutcomeV1,
};
use mycelix_business_core::{
    CommittedIntent, DomainExceptionRef, DomainRef, DomainScopedRef, ExceptionRef, IdError,
    LogicalIntentRef, OperationCommitment, QualificationCut, QualifiedInputRef,
    SemanticProfileId,
};
use mycelix_business_terminal_expectation::{
    CompensationExpectationBinding, ExceptionExpectationBinding, TerminalExpectationBindingError,
};

/// Versioned cross-domain mapping owned by this bridge, not by Governance.
pub const GOVERNANCE_RESOLUTION_BRIDGE_PROFILE: &str =
    "business.bridge.governance-resolution-expectation";
pub const GOVERNANCE_RESOLUTION_BRIDGE_VERSION: u32 = 1;

/// Finance operation profile selected by bridge v1 for Governance refund outcomes.
pub const REFUND_OPERATION_PROFILE: &str = "finance.refund";
pub const REFUND_OPERATION_PROFILE_VERSION: u32 = 1;

/// Result of translating one typed Governance outcome into the corresponding
/// Business terminal expectation binding.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum GovernanceTerminalExpectationV1 {
    Compensation(CompensationExpectationBinding),
    Exception(ExceptionExpectationBinding),
}

/// Structural bridge failures. No variant interprets whether a Governance
/// resolution is substantively legitimate.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum GovernanceResolutionBridgeError {
    WrongSourceDomain { actual: DomainRef },
    SourceSemanticProfileMismatch {
        expected_name: &'static str,
        expected_version: u32,
        actual: SemanticProfileId,
    },
    InvalidBusinessIdentity(IdError),
    ExpectationBinding(TerminalExpectationBindingError),
}

impl fmt::Display for GovernanceResolutionBridgeError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::WrongSourceDomain { actual } => write!(
                f,
                "Governance resolution bridge requires source domain governance, got {actual}"
            ),
            Self::SourceSemanticProfileMismatch {
                expected_name,
                expected_version,
                actual,
            } => write!(
                f,
                "Governance resolution source must use {expected_name}@{expected_version}, got {actual}"
            ),
            Self::InvalidBusinessIdentity(err) => {
                write!(f, "Governance resolution could not map to Business identity: {err}")
            }
            Self::ExpectationBinding(err) => {
                write!(f, "Governance resolution expectation binding failed: {err}")
            }
        }
    }
}

impl std::error::Error for GovernanceResolutionBridgeError {}

impl From<IdError> for GovernanceResolutionBridgeError {
    fn from(value: IdError) -> Self {
        Self::InvalidBusinessIdentity(value)
    }
}

impl From<TerminalExpectationBindingError> for GovernanceResolutionBridgeError {
    fn from(value: TerminalExpectationBindingError) -> Self {
        Self::ExpectationBinding(value)
    }
}

/// Map one typed Governance resolution outcome into a provenance-bound Business
/// terminal expectation under bridge profile v1.
///
/// The caller must supply the exact Governance source from which its owning
/// verifier obtained `outcome`. This function checks source domain/profile and
/// current-cut membership, but cannot itself prove that the source record's bytes
/// decode to this outcome. That runtime verification remains Governance-owned.
pub fn map_authorized_resolution_v1(
    source: QualifiedInputRef,
    outcome: &AuthorizedResolutionOutcomeV1,
    qualification_cut: &QualificationCut,
) -> Result<GovernanceTerminalExpectationV1, GovernanceResolutionBridgeError> {
    if source.domain.as_str() != "governance" {
        return Err(GovernanceResolutionBridgeError::WrongSourceDomain {
            actual: source.domain.clone(),
        });
    }

    match outcome {
        AuthorizedResolutionOutcomeV1::Refund(refund) => {
            require_source_profile(
                &source,
                AUTHORIZED_REFUND_RESOLUTION_PROFILE,
                AUTHORIZED_REFUND_RESOLUTION_VERSION,
            )?;

            let expected_intent = CommittedIntent::new(
                LogicalIntentRef::new(refund.effect_id().to_owned())?,
                SemanticProfileId::new(
                    REFUND_OPERATION_PROFILE,
                    REFUND_OPERATION_PROFILE_VERSION,
                )?,
                OperationCommitment::new(refund.canonical_material_v1())?,
            );
            let binding = CompensationExpectationBinding::bind(
                expected_intent,
                source,
                qualification_cut,
            )?;
            Ok(GovernanceTerminalExpectationV1::Compensation(binding))
        }
        AuthorizedResolutionOutcomeV1::RetainException(retained) => {
            require_source_profile(
                &source,
                AUTHORIZED_EXCEPTION_RESOLUTION_PROFILE,
                AUTHORIZED_EXCEPTION_RESOLUTION_VERSION,
            )?;

            let expected_exception: DomainExceptionRef = DomainScopedRef::new(
                DomainRef::new(retained.exception_domain().to_owned())?,
                ExceptionRef::new(retained.exception_id().to_owned())?,
            );
            let binding = ExceptionExpectationBinding::bind(
                expected_exception,
                source,
                qualification_cut,
            )?;
            Ok(GovernanceTerminalExpectationV1::Exception(binding))
        }
    }
}

fn require_source_profile(
    source: &QualifiedInputRef,
    expected_name: &'static str,
    expected_version: u32,
) -> Result<(), GovernanceResolutionBridgeError> {
    if source.semantic_profile.name().as_str() != expected_name
        || source.semantic_profile.version() != expected_version
    {
        return Err(
            GovernanceResolutionBridgeError::SourceSemanticProfileMismatch {
                expected_name,
                expected_version,
                actual: source.semantic_profile.clone(),
            },
        );
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use governance_resolution_types::{RefundResolutionV1, RetainedExceptionResolutionV1};
    use mycelix_business_core::{
        OrganizationContextRef, RecordRef, TimestampMs, ValidityEnd, ValidityWindow,
    };

    use super::*;

    fn semantic(name: &str, version: u32) -> SemanticProfileId {
        SemanticProfileId::new(name, version).unwrap()
    }

    fn input(
        domain_name: &str,
        record_name: &str,
        profile_name: &str,
        profile_version: u32,
    ) -> QualifiedInputRef {
        QualifiedInputRef {
            domain: DomainRef::new(domain_name).unwrap(),
            record: RecordRef::new(record_name).unwrap(),
            version: 1,
            semantic_profile: semantic(profile_name, profile_version),
            generation: None,
            validity: ValidityWindow::new(
                TimestampMs::new(0),
                ValidityEnd::At(TimestampMs::new(100)),
            )
            .unwrap(),
        }
    }

    fn cut_with(source: QualifiedInputRef) -> QualificationCut {
        let mut cut = QualificationCut::new(
            OrganizationContextRef::new("org:acme").unwrap(),
            semantic("business.bridge-test", 1),
            TimestampMs::new(80),
        );
        assert!(cut.insert_input(source).unwrap());
        cut
    }

    #[test]
    fn refund_mapping_is_exact_and_deterministic() {
        let source = input(
            "governance",
            "resolution:refund:1",
            AUTHORIZED_REFUND_RESOLUTION_PROFILE,
            1,
        );
        let cut = cut_with(source.clone());
        let refund = RefundResolutionV1::new(
            "refund:order-1:5000",
            "order-1",
            "customer-a",
            "USD",
            5_000,
        )
        .unwrap();
        let outcome = AuthorizedResolutionOutcomeV1::Refund(refund.clone());

        let first = map_authorized_resolution_v1(source.clone(), &outcome, &cut).unwrap();
        let second = map_authorized_resolution_v1(source, &outcome, &cut).unwrap();
        assert_eq!(first, second);

        let GovernanceTerminalExpectationV1::Compensation(binding) = first else {
            panic!("refund must map to a compensation expectation");
        };
        assert_eq!(binding.expected_intent().intent_ref.as_str(), refund.effect_id());
        assert_eq!(
            binding.expected_intent().operation_profile,
            semantic(REFUND_OPERATION_PROFILE, REFUND_OPERATION_PROFILE_VERSION)
        );
        assert_eq!(
            binding.expected_intent().semantic_commitment.as_str(),
            refund.canonical_material_v1()
        );
    }

    #[test]
    fn refund_material_change_changes_business_expectation() {
        let source = input(
            "governance",
            "resolution:refund:1",
            AUTHORIZED_REFUND_RESOLUTION_PROFILE,
            1,
        );
        let cut = cut_with(source.clone());
        let fifty = AuthorizedResolutionOutcomeV1::Refund(
            RefundResolutionV1::new("refund:order-1", "order-1", "customer-a", "USD", 5_000)
                .unwrap(),
        );
        let sixty = AuthorizedResolutionOutcomeV1::Refund(
            RefundResolutionV1::new("refund:order-1", "order-1", "customer-a", "USD", 6_000)
                .unwrap(),
        );

        let GovernanceTerminalExpectationV1::Compensation(fifty_binding) =
            map_authorized_resolution_v1(source.clone(), &fifty, &cut).unwrap()
        else {
            panic!("refund must map to compensation");
        };
        let GovernanceTerminalExpectationV1::Compensation(sixty_binding) =
            map_authorized_resolution_v1(source, &sixty, &cut).unwrap()
        else {
            panic!("refund must map to compensation");
        };
        assert_ne!(
            fifty_binding.expected_intent(),
            sixty_binding.expected_intent()
        );
    }

    #[test]
    fn source_profile_must_match_typed_governance_outcome() {
        let source = input(
            "governance",
            "resolution:wrong-profile",
            AUTHORIZED_EXCEPTION_RESOLUTION_PROFILE,
            1,
        );
        let cut = cut_with(source.clone());
        let outcome = AuthorizedResolutionOutcomeV1::Refund(
            RefundResolutionV1::new("refund:1", "order-1", "customer-a", "USD", 5_000)
                .unwrap(),
        );

        assert!(matches!(
            map_authorized_resolution_v1(source, &outcome, &cut),
            Err(GovernanceResolutionBridgeError::SourceSemanticProfileMismatch { .. })
        ));
    }

    #[test]
    fn source_must_be_governance_owned() {
        let source = input(
            "commerce",
            "resolution:spoofed",
            AUTHORIZED_REFUND_RESOLUTION_PROFILE,
            1,
        );
        let cut = cut_with(source.clone());
        let outcome = AuthorizedResolutionOutcomeV1::Refund(
            RefundResolutionV1::new("refund:1", "order-1", "customer-a", "USD", 5_000)
                .unwrap(),
        );

        assert!(matches!(
            map_authorized_resolution_v1(source, &outcome, &cut),
            Err(GovernanceResolutionBridgeError::WrongSourceDomain { .. })
        ));
    }

    #[test]
    fn exact_governance_source_must_be_in_current_cut() {
        let source = input(
            "governance",
            "resolution:missing",
            AUTHORIZED_REFUND_RESOLUTION_PROFILE,
            1,
        );
        let cut = QualificationCut::new(
            OrganizationContextRef::new("org:acme").unwrap(),
            semantic("business.bridge-test", 1),
            TimestampMs::new(80),
        );
        let outcome = AuthorizedResolutionOutcomeV1::Refund(
            RefundResolutionV1::new("refund:1", "order-1", "customer-a", "USD", 5_000)
                .unwrap(),
        );

        assert!(matches!(
            map_authorized_resolution_v1(source, &outcome, &cut),
            Err(GovernanceResolutionBridgeError::ExpectationBinding(
                TerminalExpectationBindingError::SourceMissingFromCut { .. }
            ))
        ));
    }

    #[test]
    fn retained_exception_maps_exact_domain_and_identity() {
        let source = input(
            "governance",
            "resolution:exception:1",
            AUTHORIZED_EXCEPTION_RESOLUTION_PROFILE,
            1,
        );
        let cut = cut_with(source.clone());
        let retained = RetainedExceptionResolutionV1::new("finance", "chargeback:pending:1")
            .unwrap();
        let outcome = AuthorizedResolutionOutcomeV1::RetainException(retained.clone());

        let mapped = map_authorized_resolution_v1(source, &outcome, &cut).unwrap();
        let GovernanceTerminalExpectationV1::Exception(binding) = mapped else {
            panic!("retained exception must map to exception expectation");
        };
        assert_eq!(binding.expected_exception().domain().as_str(), "finance");
        assert_eq!(
            binding.expected_exception().local().as_str(),
            retained.exception_id()
        );
    }
}
