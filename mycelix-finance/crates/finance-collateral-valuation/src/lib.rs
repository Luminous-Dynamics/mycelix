#![forbid(unsafe_code)]
// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Typed collateral-valuation capability contract for Mycelix Finance.
//!
//! A health classifier cannot establish where a value came from. This crate
//! binds a valuation result to the exact collateral subject, provider method,
//! and protocol version that produced (or failed to produce) the observation.
//! Missing/mismatched capability is data about availability, never a numeric
//! valuation sentinel.

use finance_collateral_safety::{
    CheckedCollateralHealthSnapshot, CollateralValuationObservation,
    CollateralValuationUnavailableReason,
};
use serde::{Deserialize, Serialize};

pub const COLLATERAL_VALUATION_PROTOCOL_VERSION: u16 = 1;
pub const MAX_PROVIDER_ID_LEN: usize = 256;
pub const MAX_METHOD_LEN: usize = 128;
pub const MAX_SUBJECT_ID_LEN: usize = 1024;
pub const MAX_SOURCE_HAPP_LEN: usize = 256;

/// The exact collateral subject being valued.
///
/// `collateral_id` is Finance-local identity. `source_happ + asset_id` bind that
/// registration to the system that authoritatively owns the underlying asset.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct CollateralValuationSubject {
    pub collateral_id: String,
    pub source_happ: String,
    pub asset_id: String,
}

impl CollateralValuationSubject {
    pub fn validate(&self) -> Result<(), CollateralValuationContractError> {
        validate_nonempty_bounded(&self.collateral_id, MAX_SUBJECT_ID_LEN)
            .map_err(|_| CollateralValuationContractError::InvalidSubject)?;
        validate_nonempty_bounded(&self.source_happ, MAX_SOURCE_HAPP_LEN)
            .map_err(|_| CollateralValuationContractError::InvalidSubject)?;
        validate_nonempty_bounded(&self.asset_id, MAX_SUBJECT_ID_LEN)
            .map_err(|_| CollateralValuationContractError::InvalidSubject)?;
        Ok(())
    }
}

/// Identity of the valuation capability expected to answer a request.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct CollateralValuationCapability {
    /// Stable provider/adapter identity, not a display label.
    pub provider_id: String,
    /// Exact operation/method identity implemented by this provider.
    pub method: String,
    /// Wire/semantic protocol understood by this capability.
    pub protocol_version: u16,
}

impl CollateralValuationCapability {
    pub fn validate(&self) -> Result<(), CollateralValuationContractError> {
        validate_nonempty_bounded(&self.provider_id, MAX_PROVIDER_ID_LEN)
            .map_err(|_| CollateralValuationContractError::InvalidCapability)?;
        validate_nonempty_bounded(&self.method, MAX_METHOD_LEN)
            .map_err(|_| CollateralValuationContractError::InvalidCapability)?;
        if self.protocol_version == 0 {
            return Err(CollateralValuationContractError::InvalidCapability);
        }
        Ok(())
    }
}

/// Request binding used to reject a response for the wrong subject/provider.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct CollateralValuationRequest {
    pub subject: CollateralValuationSubject,
    pub capability: CollateralValuationCapability,
}

impl CollateralValuationRequest {
    pub fn validate(&self) -> Result<(), CollateralValuationContractError> {
        self.subject.validate()?;
        self.capability.validate()?;
        if self.capability.protocol_version != COLLATERAL_VALUATION_PROTOCOL_VERSION {
            return Err(CollateralValuationContractError::UnsupportedProtocolVersion);
        }
        Ok(())
    }
}

/// Why a provider did not produce usable valuation evidence.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum CollateralValuationFailure {
    /// The provider/adapter is currently unreachable.
    ProviderUnavailable,
    /// The configured method/capability does not exist.
    CapabilityMissing,
    /// The provider speaks a different protocol version.
    VersionMismatch,
    /// This provider does not support the requested collateral subject.
    UnsupportedSubject,
    /// A response arrived but did not satisfy the provider contract.
    MalformedResponse,
}

/// Provider outcome. Failure is never represented by a numeric value.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum CollateralValuationOutcome {
    Observed {
        value: u64,
        observed_at_micros: i64,
    },
    Unavailable {
        reason: CollateralValuationFailure,
        attempted_at_micros: i64,
    },
}

/// Fully bound provider response.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct CollateralValuationEnvelope {
    pub subject: CollateralValuationSubject,
    pub capability: CollateralValuationCapability,
    pub outcome: CollateralValuationOutcome,
}

/// Contract failures detected before a provider response can be trusted.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum CollateralValuationContractError {
    InvalidSubject,
    InvalidCapability,
    UnsupportedProtocolVersion,
    SubjectMismatch,
    ProviderMismatch,
    MethodMismatch,
    VersionMismatch,
}

impl CollateralValuationEnvelope {
    /// Validate that this response answers the exact request that selected it.
    pub fn validate_for(
        &self,
        request: &CollateralValuationRequest,
    ) -> Result<(), CollateralValuationContractError> {
        request.validate()?;
        self.subject.validate()?;
        self.capability.validate()?;

        if self.subject != request.subject {
            return Err(CollateralValuationContractError::SubjectMismatch);
        }
        if self.capability.provider_id != request.capability.provider_id {
            return Err(CollateralValuationContractError::ProviderMismatch);
        }
        if self.capability.method != request.capability.method {
            return Err(CollateralValuationContractError::MethodMismatch);
        }
        if self.capability.protocol_version != request.capability.protocol_version {
            return Err(CollateralValuationContractError::VersionMismatch);
        }
        Ok(())
    }

    /// Convert the provider outcome to the health kernel's observation type.
    ///
    /// The envelope retains the detailed capability failure reason. The health
    /// kernel only needs to know that no valid numeric observation exists.
    pub fn health_observation(&self) -> CollateralValuationObservation {
        match self.outcome {
            CollateralValuationOutcome::Observed { value, .. } => {
                CollateralValuationObservation::Observed { value }
            }
            CollateralValuationOutcome::Unavailable {
                reason: CollateralValuationFailure::ProviderUnavailable,
                ..
            } => CollateralValuationObservation::Unavailable(
                CollateralValuationUnavailableReason::OracleUnavailable,
            ),
            CollateralValuationOutcome::Unavailable { .. } => {
                CollateralValuationObservation::Unavailable(
                    CollateralValuationUnavailableReason::InvalidObservation,
                )
            }
        }
    }

    /// Produce a checked health snapshot only after exact request binding passes.
    pub fn checked_health_snapshot(
        &self,
        request: &CollateralValuationRequest,
        obligation_amount: u64,
        computed_at_micros: i64,
    ) -> Result<CheckedCollateralHealthSnapshot, CollateralValuationContractError> {
        self.validate_for(request)?;
        Ok(CheckedCollateralHealthSnapshot::new(
            self.subject.collateral_id.clone(),
            obligation_amount,
            self.health_observation(),
            computed_at_micros,
        ))
    }
}

fn validate_nonempty_bounded(value: &str, max_len: usize) -> Result<(), ()> {
    if value.is_empty() || value.len() > max_len {
        Err(())
    } else {
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use finance_collateral_safety::{
        CollateralHealthAssessment, CollateralHealthIndeterminateReason,
    };
    use mycelix_finance_types::CollateralHealthStatus;

    fn subject() -> CollateralValuationSubject {
        CollateralValuationSubject {
            collateral_id: "collateral:1".into(),
            source_happ: "mycelix-property".into(),
            asset_id: "property:lot:42".into(),
        }
    }

    fn capability() -> CollateralValuationCapability {
        CollateralValuationCapability {
            provider_id: "property-valuation-adapter".into(),
            method: "get_collateral_valuation".into(),
            protocol_version: COLLATERAL_VALUATION_PROTOCOL_VERSION,
        }
    }

    fn request() -> CollateralValuationRequest {
        CollateralValuationRequest {
            subject: subject(),
            capability: capability(),
        }
    }

    #[test]
    fn exact_subject_and_capability_binding_accepts_response() {
        let response = CollateralValuationEnvelope {
            subject: subject(),
            capability: capability(),
            outcome: CollateralValuationOutcome::Observed {
                value: 100,
                observed_at_micros: 10,
            },
        };
        assert_eq!(response.validate_for(&request()), Ok(()));
    }

    #[test]
    fn unsupported_requested_protocol_is_rejected_even_if_nonzero() {
        let mut unsupported = request();
        unsupported.capability.protocol_version = COLLATERAL_VALUATION_PROTOCOL_VERSION + 1;
        assert_eq!(
            unsupported.validate(),
            Err(CollateralValuationContractError::UnsupportedProtocolVersion)
        );
    }

    #[test]
    fn wrong_subject_is_rejected_before_health_computation() {
        let mut wrong = subject();
        wrong.asset_id = "property:lot:99".into();
        let response = CollateralValuationEnvelope {
            subject: wrong,
            capability: capability(),
            outcome: CollateralValuationOutcome::Observed {
                value: 100,
                observed_at_micros: 10,
            },
        };
        assert_eq!(
            response.validate_for(&request()),
            Err(CollateralValuationContractError::SubjectMismatch)
        );
    }

    #[test]
    fn wrong_provider_method_and_version_are_distinct_failures() {
        let base = CollateralValuationEnvelope {
            subject: subject(),
            capability: capability(),
            outcome: CollateralValuationOutcome::Observed {
                value: 100,
                observed_at_micros: 10,
            },
        };

        let mut wrong_provider = base.clone();
        wrong_provider.capability.provider_id = "other-provider".into();
        assert_eq!(
            wrong_provider.validate_for(&request()),
            Err(CollateralValuationContractError::ProviderMismatch)
        );

        let mut wrong_method = base.clone();
        wrong_method.capability.method = "some_other_method".into();
        assert_eq!(
            wrong_method.validate_for(&request()),
            Err(CollateralValuationContractError::MethodMismatch)
        );

        let mut wrong_version = base;
        wrong_version.capability.protocol_version += 1;
        assert_eq!(
            wrong_version.validate_for(&request()),
            Err(CollateralValuationContractError::VersionMismatch)
        );
    }

    #[test]
    fn provider_failure_never_becomes_a_numeric_sentinel() {
        for reason in [
            CollateralValuationFailure::ProviderUnavailable,
            CollateralValuationFailure::CapabilityMissing,
            CollateralValuationFailure::VersionMismatch,
            CollateralValuationFailure::UnsupportedSubject,
            CollateralValuationFailure::MalformedResponse,
        ] {
            let response = CollateralValuationEnvelope {
                subject: subject(),
                capability: capability(),
                outcome: CollateralValuationOutcome::Unavailable {
                    reason,
                    attempted_at_micros: 11,
                },
            };
            assert!(matches!(
                response.health_observation(),
                CollateralValuationObservation::Unavailable(_)
            ));
        }
    }

    #[test]
    fn observed_zero_remains_observed_zero_not_provider_failure() {
        let response = CollateralValuationEnvelope {
            subject: subject(),
            capability: capability(),
            outcome: CollateralValuationOutcome::Observed {
                value: 0,
                observed_at_micros: 12,
            },
        };
        assert_eq!(
            response.health_observation(),
            CollateralValuationObservation::Observed { value: 0 }
        );
    }

    #[test]
    fn unavailable_provider_produces_indeterminate_snapshot_not_liquidation() {
        let response = CollateralValuationEnvelope {
            subject: subject(),
            capability: capability(),
            outcome: CollateralValuationOutcome::Unavailable {
                reason: CollateralValuationFailure::ProviderUnavailable,
                attempted_at_micros: 13,
            },
        };
        let snapshot = response
            .checked_health_snapshot(&request(), 100, 14)
            .expect("bound unavailable response remains representable");
        assert_eq!(snapshot.ltv_ratio, None);
        assert_eq!(
            snapshot.assessment,
            CollateralHealthAssessment::Indeterminate(
                CollateralHealthIndeterminateReason::OracleUnavailable
            )
        );
        assert!(!snapshot.assessment.is_liquidation_evidence());
        assert_eq!(snapshot.validate(), Ok(()));
    }

    #[test]
    fn valid_observed_value_can_produce_known_liquidation_evidence() {
        let response = CollateralValuationEnvelope {
            subject: subject(),
            capability: capability(),
            outcome: CollateralValuationOutcome::Observed {
                value: 100,
                observed_at_micros: 15,
            },
        };
        let snapshot = response
            .checked_health_snapshot(&request(), 96, 16)
            .expect("exact binding should validate");
        assert_eq!(snapshot.ltv_ratio, Some(0.96));
        assert_eq!(
            snapshot.assessment,
            CollateralHealthAssessment::Known(CollateralHealthStatus::Liquidation)
        );
        assert!(snapshot.assessment.is_liquidation_evidence());
    }

    #[test]
    fn malformed_identity_fields_fail_closed() {
        let mut bad_request = request();
        bad_request.subject.source_happ.clear();
        assert_eq!(
            bad_request.validate(),
            Err(CollateralValuationContractError::InvalidSubject)
        );

        let mut bad_capability = capability();
        bad_capability.method.clear();
        assert_eq!(
            bad_capability.validate(),
            Err(CollateralValuationContractError::InvalidCapability)
        );
    }

    #[test]
    fn response_round_trip_preserves_failure_reason_and_binding() {
        let response = CollateralValuationEnvelope {
            subject: subject(),
            capability: capability(),
            outcome: CollateralValuationOutcome::Unavailable {
                reason: CollateralValuationFailure::CapabilityMissing,
                attempted_at_micros: 17,
            },
        };
        let encoded = serde_json::to_string(&response).expect("serialize response");
        let decoded: CollateralValuationEnvelope =
            serde_json::from_str(&encoded).expect("deserialize response");
        assert_eq!(decoded, response);
        assert_eq!(decoded.validate_for(&request()), Ok(()));
    }
}
