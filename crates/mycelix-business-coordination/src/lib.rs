// Copyright (C) 2024-2026 Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Declarative coordination contracts for the Mycelix Business Fabric.
//!
//! This crate validates references and requirements around prepared actions. It does
//! not mint authority, own scarce state, reconcile external systems, or execute work.

use std::collections::{BTreeMap, BTreeSet};

use mycelix_business_core::{
    ActionContractRef, AuthorityScope, CapabilityRef, Digest32, PreparedAction,
    PreparedActionError, ReferenceId,
};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReversibilityClass {
    FullyReversible,
    OperationallyReversible,
    Compensatable,
    PartiallyReversible,
    Irreversible,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UnknownOutcomePolicy {
    /// Retry is permitted only through a provider-supported stable idempotency identity.
    IdempotentRetryOnly,
    /// Reconciliation must establish state before any potentially duplicative retry.
    ReconcileBeforeRetry,
}

impl Default for UnknownOutcomePolicy {
    fn default() -> Self {
        Self::ReconcileBeforeRetry
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FreshnessRequirement {
    pub domain: ReferenceId,
    pub maximum_age_ms: u64,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct AggregatePolicyKey {
    pub key: ReferenceId,
    pub window_ms: u64,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ActionContract {
    pub contract_ref: ActionContractRef,
    pub required_reservation_domains: BTreeSet<ReferenceId>,
    pub required_capabilities: BTreeSet<CapabilityRef>,
    pub freshness: Vec<FreshnessRequirement>,
    pub aggregate_policy_keys: Vec<AggregatePolicyKey>,
    pub reversibility: ReversibilityClass,
    pub maximum_prepared_lifetime_ms: u64,
    pub unknown_outcome_policy: UnknownOutcomePolicy,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ActionContractError {
    ZeroPreparedLifetime,
    ZeroFreshnessWindow { domain: ReferenceId },
    DuplicateFreshnessDomain { domain: ReferenceId },
    ZeroAggregateWindow { key: ReferenceId },
    DuplicateAggregateKey { key: ReferenceId },
}

impl ActionContract {
    pub fn validate(&self) -> Result<(), ActionContractError> {
        if self.maximum_prepared_lifetime_ms == 0 {
            return Err(ActionContractError::ZeroPreparedLifetime);
        }

        let mut freshness_domains = BTreeSet::new();
        for requirement in &self.freshness {
            if requirement.maximum_age_ms == 0 {
                return Err(ActionContractError::ZeroFreshnessWindow {
                    domain: requirement.domain.clone(),
                });
            }
            if !freshness_domains.insert(requirement.domain.clone()) {
                return Err(ActionContractError::DuplicateFreshnessDomain {
                    domain: requirement.domain.clone(),
                });
            }
        }

        let mut aggregate_keys = BTreeSet::new();
        for key in &self.aggregate_policy_keys {
            if key.window_ms == 0 {
                return Err(ActionContractError::ZeroAggregateWindow {
                    key: key.key.clone(),
                });
            }
            if !aggregate_keys.insert(key.key.clone()) {
                return Err(ActionContractError::DuplicateAggregateKey {
                    key: key.key.clone(),
                });
            }
        }

        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ObservationAge {
    pub domain: ReferenceId,
    pub age_ms: u64,
}

/// Evidence that an aggregate-policy family was evaluated externally.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct AggregateCheckRef {
    pub key: ReferenceId,
    pub digest: Digest32,
}

/// Binds a prepared action to externally governed action-contract requirements.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CoordinationEnvelope {
    pub contract: ActionContract,
    pub prepared: PreparedAction,
    pub authority_scope: AuthorityScope,
    pub observation_ages: Vec<ObservationAge>,
    pub aggregate_checks: Vec<AggregateCheckRef>,
    pub coordination_digest: Digest32,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum CoordinationError {
    InvalidContract(ActionContractError),
    InvalidPreparedAction(PreparedActionError),
    ContractIdentityMismatch,
    PreparedLifetimeTooLong { actual_ms: u64, maximum_ms: u64 },
    MissingReservationDomain { domain: ReferenceId },
    MissingCapability { capability: CapabilityRef },
    MissingFreshnessObservation { domain: ReferenceId },
    StaleObservation {
        domain: ReferenceId,
        actual_age_ms: u64,
        maximum_age_ms: u64,
    },
    MissingAggregateCheck { key: ReferenceId },
}

impl CoordinationEnvelope {
    pub fn validate(&self) -> Result<(), CoordinationError> {
        self.contract
            .validate()
            .map_err(CoordinationError::InvalidContract)?;
        self.prepared
            .validate()
            .map_err(CoordinationError::InvalidPreparedAction)?;

        if self.prepared.action_contract != self.contract.contract_ref {
            return Err(CoordinationError::ContractIdentityMismatch);
        }

        let lifetime = self.prepared.expires_at_unix_ms - self.prepared.prepared_at_unix_ms;
        if lifetime > self.contract.maximum_prepared_lifetime_ms {
            return Err(CoordinationError::PreparedLifetimeTooLong {
                actual_ms: lifetime,
                maximum_ms: self.contract.maximum_prepared_lifetime_ms,
            });
        }

        let reservation_domains = self
            .prepared
            .reservations
            .iter()
            .map(|reservation| reservation.domain.clone())
            .collect::<BTreeSet<_>>();
        for required in &self.contract.required_reservation_domains {
            if !reservation_domains.contains(required) {
                return Err(CoordinationError::MissingReservationDomain {
                    domain: required.clone(),
                });
            }
        }

        for required in &self.contract.required_capabilities {
            if !self.authority_scope.capabilities.contains(required) {
                return Err(CoordinationError::MissingCapability {
                    capability: required.clone(),
                });
            }
        }

        let ages = self
            .observation_ages
            .iter()
            .map(|observation| (observation.domain.clone(), observation.age_ms))
            .collect::<BTreeMap<_, _>>();
        for requirement in &self.contract.freshness {
            let Some(actual_age_ms) = ages.get(&requirement.domain).copied() else {
                return Err(CoordinationError::MissingFreshnessObservation {
                    domain: requirement.domain.clone(),
                });
            };
            if actual_age_ms > requirement.maximum_age_ms {
                return Err(CoordinationError::StaleObservation {
                    domain: requirement.domain.clone(),
                    actual_age_ms,
                    maximum_age_ms: requirement.maximum_age_ms,
                });
            }
        }

        let checked = self
            .aggregate_checks
            .iter()
            .map(|check| check.key.clone())
            .collect::<BTreeSet<_>>();
        for required in &self.contract.aggregate_policy_keys {
            if !checked.contains(&required.key) {
                return Err(CoordinationError::MissingAggregateCheck {
                    key: required.key.clone(),
                });
            }
        }

        Ok(())
    }
}

/// Governs what is optimized without treating diagnostic metrics as the objective itself.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ObjectiveContract {
    pub outcomes: BTreeSet<ReferenceId>,
    pub diagnostic_metrics: BTreeSet<ReferenceId>,
    pub protected_constraints: BTreeSet<ReferenceId>,
    pub resilience_floors: BTreeSet<ReferenceId>,
    pub evaluation_horizon_ms: u64,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ObjectiveContractError {
    NoOutcomes,
    ZeroEvaluationHorizon,
}

impl ObjectiveContract {
    pub fn validate(&self) -> Result<(), ObjectiveContractError> {
        if self.outcomes.is_empty() {
            return Err(ObjectiveContractError::NoOutcomes);
        }
        if self.evaluation_horizon_ms == 0 {
            return Err(ObjectiveContractError::ZeroEvaluationHorizon);
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_business_core::{
        AuthorityLeaseRef, BudgetLimit, DecisionFrontiers, ReservationId, ReservationRef,
        RiskClass, ScopeRef, SubjectRef,
    };

    fn id(value: &str) -> ReferenceId {
        ReferenceId::new(value).unwrap()
    }

    fn capability(value: &str) -> CapabilityRef {
        CapabilityRef(id(value))
    }

    fn subject(value: &str) -> SubjectRef {
        SubjectRef(id(value))
    }

    fn contract_ref() -> ActionContractRef {
        ActionContractRef {
            semantic_id: id("mycelix.procurement.place-order.v1"),
            digest: Digest32::repeat(1),
        }
    }

    fn authority_scope() -> AuthorityScope {
        AuthorityScope {
            capabilities: BTreeSet::from([capability("procurement:place-order")]),
            subjects: BTreeSet::from([subject("restaurant:a")]),
            risk_ceiling: RiskClass::Moderate,
            budget: BudgetLimit::Limited(2_000),
            expires_at_unix_ms: 2_000,
        }
    }

    fn prepared() -> PreparedAction {
        PreparedAction {
            action_contract: contract_ref(),
            decision_capsule: id("decision:1"),
            intent_digest: Digest32::repeat(2),
            frontiers: DecisionFrontiers::default(),
            authority_lease: AuthorityLeaseRef {
                lease_id: id("authority:1"),
                authority_epoch: 7,
                sequence: 1,
                fencing_token: 11,
                scope: ScopeRef(id("scope:restaurant:a")),
                issued_at_unix_ms: 1_000,
                expires_at_unix_ms: 2_000,
            },
            reservations: vec![ReservationRef {
                domain: id("finance"),
                reservation_id: ReservationId(id("reservation:finance:1")),
                subject: subject("budget:restaurant:a"),
                digest: Digest32::repeat(3),
                expires_at_unix_ms: 1_900,
            }],
            idempotency_key: id("order:1"),
            prepared_at_unix_ms: 1_200,
            expires_at_unix_ms: 1_500,
        }
    }

    fn contract() -> ActionContract {
        ActionContract {
            contract_ref: contract_ref(),
            required_reservation_domains: BTreeSet::from([id("finance")]),
            required_capabilities: BTreeSet::from([capability("procurement:place-order")]),
            freshness: vec![FreshnessRequirement {
                domain: id("inventory"),
                maximum_age_ms: 30_000,
            }],
            aggregate_policy_keys: vec![AggregatePolicyKey {
                key: id("aggregate:procurement:24h"),
                window_ms: 86_400_000,
            }],
            reversibility: ReversibilityClass::OperationallyReversible,
            maximum_prepared_lifetime_ms: 600,
            unknown_outcome_policy: UnknownOutcomePolicy::ReconcileBeforeRetry,
        }
    }

    fn envelope() -> CoordinationEnvelope {
        CoordinationEnvelope {
            contract: contract(),
            prepared: prepared(),
            authority_scope: authority_scope(),
            observation_ages: vec![ObservationAge {
                domain: id("inventory"),
                age_ms: 10_000,
            }],
            aggregate_checks: vec![AggregateCheckRef {
                key: id("aggregate:procurement:24h"),
                digest: Digest32::repeat(4),
            }],
            coordination_digest: Digest32::repeat(5),
        }
    }

    #[test]
    fn complete_coordination_envelope_validates() {
        assert_eq!(envelope().validate(), Ok(()));
    }

    #[test]
    fn missing_reservation_domain_fails_closed() {
        let mut value = envelope();
        value.prepared.reservations.clear();
        assert_eq!(
            value.validate(),
            Err(CoordinationError::MissingReservationDomain {
                domain: id("finance"),
            })
        );
    }

    #[test]
    fn stale_observation_fails_closed() {
        let mut value = envelope();
        value.observation_ages[0].age_ms = 30_001;
        assert_eq!(
            value.validate(),
            Err(CoordinationError::StaleObservation {
                domain: id("inventory"),
                actual_age_ms: 30_001,
                maximum_age_ms: 30_000,
            })
        );
    }

    #[test]
    fn missing_aggregate_check_cannot_bypass_policy() {
        let mut value = envelope();
        value.aggregate_checks.clear();
        assert_eq!(
            value.validate(),
            Err(CoordinationError::MissingAggregateCheck {
                key: id("aggregate:procurement:24h"),
            })
        );
    }

    #[test]
    fn prepared_lifetime_is_contract_bounded() {
        let mut value = envelope();
        value.prepared.expires_at_unix_ms = 1_850;
        value.prepared.reservations[0].expires_at_unix_ms = 1_900;
        assert_eq!(
            value.validate(),
            Err(CoordinationError::PreparedLifetimeTooLong {
                actual_ms: 650,
                maximum_ms: 600,
            })
        );
    }

    #[test]
    fn objective_contract_requires_real_outcomes_and_horizon() {
        let empty = ObjectiveContract {
            outcomes: BTreeSet::new(),
            diagnostic_metrics: BTreeSet::from([id("metric:table-turnover")]),
            protected_constraints: BTreeSet::from([id("constraint:food-safety")]),
            resilience_floors: BTreeSet::from([id("floor:minimum-staffing")]),
            evaluation_horizon_ms: 86_400_000,
        };
        assert_eq!(empty.validate(), Err(ObjectiveContractError::NoOutcomes));

        let valid = ObjectiveContract {
            outcomes: BTreeSet::from([id("outcome:service-quality")]),
            ..empty
        };
        assert_eq!(valid.validate(), Ok(()));
    }
}
