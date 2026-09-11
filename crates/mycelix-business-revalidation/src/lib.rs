// Copyright (C) 2024-2026 Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Execution-time material revalidation for prepared Mycelix business actions.
//!
//! A successful revalidation says only that the declared execution preconditions still hold.
//! It does not mint authority, execute work, or establish an external economic outcome.

use std::collections::{BTreeMap, BTreeSet};

use mycelix_business_coordination::{CoordinationEnvelope, CoordinationError};
use mycelix_business_core::{
    ActionContractRef, Digest32, FrontierRef, PreparedActionError, ReferenceId,
};
use mycelix_business_decision::{DecisionCapsule, DecisionCapsuleError, DecisionCapsuleRef};

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum FrontierClass {
    Observation,
    Policy,
    Authority,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum RevalidationRule {
    /// Exact sequence + digest continuity. Mandatory for policy and authority frontiers.
    ExactFrontier {
        class: FrontierClass,
        expected: FrontierRef,
    },
    /// Domain-owned witness that one named dependency still holds despite unrelated changes.
    DependencyWitness {
        class: FrontierClass,
        domain: ReferenceId,
        condition: ReferenceId,
        condition_digest: Digest32,
        maximum_source_age_ms: u64,
    },
    /// Bounded observation staleness only. Never valid for policy or authority.
    FreshnessOnly {
        class: FrontierClass,
        domain: ReferenceId,
        maximum_source_age_ms: u64,
    },
}

impl RevalidationRule {
    fn identity(&self) -> (FrontierClass, ReferenceId) {
        match self {
            Self::ExactFrontier { class, expected } => (*class, expected.domain.clone()),
            Self::DependencyWitness { class, domain, .. }
            | Self::FreshnessOnly { class, domain, .. } => (*class, domain.clone()),
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ExecutionRevalidationContract {
    pub action_contract: ActionContractRef,
    pub rules: Vec<RevalidationRule>,
    /// Revalidation checks themselves must be recent at execution time.
    pub maximum_check_age_ms: u64,
    pub contract_digest: Digest32,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum RevalidationContractError {
    ZeroMaximumCheckAge,
    DuplicateRule {
        class: FrontierClass,
        domain: ReferenceId,
    },
    ZeroSourceAge {
        domain: ReferenceId,
    },
    ZeroExpectedSequence {
        domain: ReferenceId,
    },
    WeakRuleForPolicyOrAuthority {
        class: FrontierClass,
        domain: ReferenceId,
    },
}

impl ExecutionRevalidationContract {
    pub fn validate(&self) -> Result<(), RevalidationContractError> {
        if self.maximum_check_age_ms == 0 {
            return Err(RevalidationContractError::ZeroMaximumCheckAge);
        }

        let mut seen = BTreeSet::new();
        for rule in &self.rules {
            let (class, domain) = rule.identity();
            if !seen.insert((class, domain.clone())) {
                return Err(RevalidationContractError::DuplicateRule { class, domain });
            }

            match rule {
                RevalidationRule::ExactFrontier { expected, .. } => {
                    if expected.sequence == 0 {
                        return Err(RevalidationContractError::ZeroExpectedSequence {
                            domain: expected.domain.clone(),
                        });
                    }
                }
                RevalidationRule::DependencyWitness {
                    class,
                    domain,
                    maximum_source_age_ms,
                    ..
                }
                | RevalidationRule::FreshnessOnly {
                    class,
                    domain,
                    maximum_source_age_ms,
                } => {
                    if *maximum_source_age_ms == 0 {
                        return Err(RevalidationContractError::ZeroSourceAge {
                            domain: domain.clone(),
                        });
                    }
                    if *class != FrontierClass::Observation {
                        return Err(RevalidationContractError::WeakRuleForPolicyOrAuthority {
                            class: *class,
                            domain: domain.clone(),
                        });
                    }
                }
            }
        }

        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum RevalidationEvidence {
    ExactFrontier {
        class: FrontierClass,
        current: FrontierRef,
        checked_at_unix_ms: u64,
    },
    DependencyWitness {
        class: FrontierClass,
        domain: ReferenceId,
        condition: ReferenceId,
        condition_digest: Digest32,
        witness: ReferenceId,
        source_age_ms: u64,
        checked_at_unix_ms: u64,
    },
    FreshnessOnly {
        class: FrontierClass,
        domain: ReferenceId,
        source_age_ms: u64,
        checked_at_unix_ms: u64,
    },
}

impl RevalidationEvidence {
    fn identity(&self) -> (FrontierClass, ReferenceId) {
        match self {
            Self::ExactFrontier { class, current, .. } => (*class, current.domain.clone()),
            Self::DependencyWitness { class, domain, .. }
            | Self::FreshnessOnly { class, domain, .. } => (*class, domain.clone()),
        }
    }

    fn checked_at_unix_ms(&self) -> u64 {
        match self {
            Self::ExactFrontier {
                checked_at_unix_ms,
                ..
            }
            | Self::DependencyWitness {
                checked_at_unix_ms,
                ..
            }
            | Self::FreshnessOnly {
                checked_at_unix_ms,
                ..
            } => *checked_at_unix_ms,
        }
    }
}

/// Non-authorizing proof that declared execution preconditions were revalidated.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RevalidatedForAttempt {
    pub decision: DecisionCapsuleRef,
    pub action_contract: ActionContractRef,
    pub revalidation_contract_digest: Digest32,
    pub checked_at_unix_ms: u64,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum RevalidationError {
    InvalidContract(RevalidationContractError),
    InvalidCoordination(CoordinationError),
    InvalidDecision(DecisionCapsuleError),
    InvalidPreparedAction(PreparedActionError),
    ActionContractMismatch,
    MissingPreparedPolicyFrontier {
        domain: ReferenceId,
    },
    MissingPreparedAuthorityFrontier {
        domain: ReferenceId,
    },
    DuplicateEvidence {
        class: FrontierClass,
        domain: ReferenceId,
    },
    MissingEvidence {
        class: FrontierClass,
        domain: ReferenceId,
    },
    EvidenceFromFuture {
        class: FrontierClass,
        domain: ReferenceId,
    },
    StaleCheck {
        class: FrontierClass,
        domain: ReferenceId,
        actual_age_ms: u64,
        maximum_age_ms: u64,
    },
    ExactFrontierChanged {
        class: FrontierClass,
        domain: ReferenceId,
    },
    WitnessConditionMismatch {
        domain: ReferenceId,
    },
    StaleSource {
        domain: ReferenceId,
        actual_age_ms: u64,
        maximum_age_ms: u64,
    },
    EvidenceKindMismatch {
        class: FrontierClass,
        domain: ReferenceId,
    },
}

impl ExecutionRevalidationContract {
    pub fn evaluate(
        &self,
        decision: &DecisionCapsule,
        coordination: &CoordinationEnvelope,
        current_authority_epoch: u64,
        now_unix_ms: u64,
        evidence: &[RevalidationEvidence],
    ) -> Result<RevalidatedForAttempt, RevalidationError> {
        self.validate().map_err(RevalidationError::InvalidContract)?;
        coordination
            .validate()
            .map_err(RevalidationError::InvalidCoordination)?;
        decision
            .validate_against_coordination(coordination)
            .map_err(RevalidationError::InvalidDecision)?;
        coordination
            .prepared
            .validate_for_execution(now_unix_ms, current_authority_epoch)
            .map_err(RevalidationError::InvalidPreparedAction)?;

        if self.action_contract != coordination.contract.contract_ref {
            return Err(RevalidationError::ActionContractMismatch);
        }

        self.require_exact_prepared_frontiers(coordination)?;

        let mut evidence_by_identity = BTreeMap::new();
        for item in evidence {
            let identity = item.identity();
            if evidence_by_identity.insert(identity.clone(), item).is_some() {
                return Err(RevalidationError::DuplicateEvidence {
                    class: identity.0,
                    domain: identity.1,
                });
            }
        }

        for rule in &self.rules {
            let (class, domain) = rule.identity();
            let Some(item) = evidence_by_identity.get(&(class, domain.clone())) else {
                return Err(RevalidationError::MissingEvidence { class, domain });
            };

            let checked_at = item.checked_at_unix_ms();
            if checked_at > now_unix_ms {
                return Err(RevalidationError::EvidenceFromFuture { class, domain });
            }
            let check_age = now_unix_ms - checked_at;
            if check_age > self.maximum_check_age_ms {
                return Err(RevalidationError::StaleCheck {
                    class,
                    domain,
                    actual_age_ms: check_age,
                    maximum_age_ms: self.maximum_check_age_ms,
                });
            }

            match (rule, *item) {
                (
                    RevalidationRule::ExactFrontier { expected, .. },
                    RevalidationEvidence::ExactFrontier { current, .. },
                ) => {
                    if current != expected {
                        return Err(RevalidationError::ExactFrontierChanged {
                            class,
                            domain,
                        });
                    }
                }
                (
                    RevalidationRule::DependencyWitness {
                        condition,
                        condition_digest,
                        maximum_source_age_ms,
                        ..
                    },
                    RevalidationEvidence::DependencyWitness {
                        condition: actual_condition,
                        condition_digest: actual_digest,
                        source_age_ms,
                        ..
                    },
                ) => {
                    if actual_condition != condition || actual_digest != condition_digest {
                        return Err(RevalidationError::WitnessConditionMismatch { domain });
                    }
                    if *source_age_ms > *maximum_source_age_ms {
                        return Err(RevalidationError::StaleSource {
                            domain,
                            actual_age_ms: *source_age_ms,
                            maximum_age_ms: *maximum_source_age_ms,
                        });
                    }
                }
                (
                    RevalidationRule::FreshnessOnly {
                        maximum_source_age_ms,
                        ..
                    },
                    RevalidationEvidence::FreshnessOnly { source_age_ms, .. },
                ) => {
                    if *source_age_ms > *maximum_source_age_ms {
                        return Err(RevalidationError::StaleSource {
                            domain,
                            actual_age_ms: *source_age_ms,
                            maximum_age_ms: *maximum_source_age_ms,
                        });
                    }
                }
                _ => {
                    return Err(RevalidationError::EvidenceKindMismatch { class, domain });
                }
            }
        }

        Ok(RevalidatedForAttempt {
            decision: decision.capsule_ref.clone(),
            action_contract: self.action_contract.clone(),
            revalidation_contract_digest: self.contract_digest,
            checked_at_unix_ms: now_unix_ms,
        })
    }

    fn require_exact_prepared_frontiers(
        &self,
        coordination: &CoordinationEnvelope,
    ) -> Result<(), RevalidationError> {
        let exact = self
            .rules
            .iter()
            .filter_map(|rule| match rule {
                RevalidationRule::ExactFrontier { class, expected } => {
                    Some((*class, expected.clone()))
                }
                _ => None,
            })
            .collect::<Vec<_>>();

        for frontier in &coordination.prepared.frontiers.policy {
            if !exact
                .iter()
                .any(|(class, expected)| *class == FrontierClass::Policy && expected == frontier)
            {
                return Err(RevalidationError::MissingPreparedPolicyFrontier {
                    domain: frontier.domain.clone(),
                });
            }
        }

        for frontier in &coordination.prepared.frontiers.authority {
            if !exact.iter().any(|(class, expected)| {
                *class == FrontierClass::Authority && expected == frontier
            }) {
                return Err(RevalidationError::MissingPreparedAuthorityFrontier {
                    domain: frontier.domain.clone(),
                });
            }
        }

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_business_coordination::{
        ActionContract, AggregateCheckRef, AggregatePolicyKey, FreshnessRequirement,
        ObservationAge, ReversibilityClass, UnknownOutcomePolicy,
    };
    use mycelix_business_core::{
        AuthorityLeaseRef, AuthorityScope, BudgetLimit, CapabilityRef, DecisionFrontiers,
        ObservationRef, PreparedAction, ProposalRef, ReservationId, ReservationRef, RiskClass,
        ScopeRef, SubjectRef,
    };
    use mycelix_business_decision::{DecisionCapsule, HumanExplanationRef};

    fn id(value: &str) -> ReferenceId {
        ReferenceId::new(value).unwrap()
    }

    fn capability(value: &str) -> CapabilityRef {
        CapabilityRef(id(value))
    }

    fn subject(value: &str) -> SubjectRef {
        SubjectRef(id(value))
    }

    fn action_contract_ref() -> ActionContractRef {
        ActionContractRef {
            semantic_id: id("mycelix.procurement.place-order.v1"),
            digest: Digest32::repeat(1),
        }
    }

    fn policy_frontier() -> FrontierRef {
        FrontierRef {
            domain: id("policy"),
            sequence: 9,
            digest: Digest32::repeat(9),
        }
    }

    fn authority_frontier() -> FrontierRef {
        FrontierRef {
            domain: id("authority"),
            sequence: 12,
            digest: Digest32::repeat(12),
        }
    }

    fn coordination() -> CoordinationEnvelope {
        CoordinationEnvelope {
            contract: ActionContract {
                contract_ref: action_contract_ref(),
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
            },
            prepared: PreparedAction {
                action_contract: action_contract_ref(),
                decision_capsule: id("decision:1"),
                intent_digest: Digest32::repeat(2),
                frontiers: DecisionFrontiers {
                    observation: vec![FrontierRef {
                        domain: id("inventory"),
                        sequence: 100,
                        digest: Digest32::repeat(100),
                    }],
                    policy: vec![policy_frontier()],
                    authority: vec![authority_frontier()],
                },
                authority_lease: AuthorityLeaseRef {
                    lease_id: id("authority:lease:1"),
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
            },
            authority_scope: AuthorityScope {
                capabilities: BTreeSet::from([capability("procurement:place-order")]),
                subjects: BTreeSet::from([subject("restaurant:a")]),
                risk_ceiling: RiskClass::Moderate,
                budget: BudgetLimit::Limited(2_000),
                expires_at_unix_ms: 2_000,
            },
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

    fn decision() -> DecisionCapsule {
        let proposal = ProposalRef(id("proposal:order"));
        DecisionCapsule {
            capsule_ref: DecisionCapsuleRef {
                id: id("decision:1"),
                digest: Digest32::repeat(20),
            },
            subject: subject("restaurant:a"),
            action_contract: action_contract_ref(),
            frontiers: coordination().prepared.frontiers,
            observations: BTreeSet::from([ObservationRef(id("observation:inventory:1"))]),
            estimates: BTreeSet::new(),
            forecasts: BTreeSet::new(),
            assumptions: BTreeSet::new(),
            conflicts: BTreeSet::new(),
            alternatives: BTreeSet::from([proposal.clone()]),
            recommendation: proposal,
            required_capabilities: BTreeSet::from([capability("procurement:place-order")]),
            model_lineage: id("model:symthaea:v17"),
            objective_contract_digest: Digest32::repeat(21),
            human_explanation: Some(HumanExplanationRef(id("explanation:1"))),
            created_at_unix_ms: 1_100,
            valid_until_unix_ms: 1_600,
        }
    }

    fn contract() -> ExecutionRevalidationContract {
        ExecutionRevalidationContract {
            action_contract: action_contract_ref(),
            rules: vec![
                RevalidationRule::DependencyWitness {
                    class: FrontierClass::Observation,
                    domain: id("inventory"),
                    condition: id("condition:stock-at-least-20kg"),
                    condition_digest: Digest32::repeat(30),
                    maximum_source_age_ms: 30_000,
                },
                RevalidationRule::ExactFrontier {
                    class: FrontierClass::Policy,
                    expected: policy_frontier(),
                },
                RevalidationRule::ExactFrontier {
                    class: FrontierClass::Authority,
                    expected: authority_frontier(),
                },
            ],
            maximum_check_age_ms: 5_000,
            contract_digest: Digest32::repeat(40),
        }
    }

    fn evidence() -> Vec<RevalidationEvidence> {
        vec![
            RevalidationEvidence::DependencyWitness {
                class: FrontierClass::Observation,
                domain: id("inventory"),
                condition: id("condition:stock-at-least-20kg"),
                condition_digest: Digest32::repeat(30),
                witness: id("witness:inventory:1"),
                source_age_ms: 2_000,
                checked_at_unix_ms: 1_300,
            },
            RevalidationEvidence::ExactFrontier {
                class: FrontierClass::Policy,
                current: policy_frontier(),
                checked_at_unix_ms: 1_300,
            },
            RevalidationEvidence::ExactFrontier {
                class: FrontierClass::Authority,
                current: authority_frontier(),
                checked_at_unix_ms: 1_300,
            },
        ]
    }

    #[test]
    fn material_dependencies_can_survive_unrelated_observation_changes() {
        let result = contract().evaluate(&decision(), &coordination(), 7, 1_350, &evidence());
        assert!(result.is_ok());
    }

    #[test]
    fn policy_frontier_change_fails_closed() {
        let mut evidence = evidence();
        evidence[1] = RevalidationEvidence::ExactFrontier {
            class: FrontierClass::Policy,
            current: FrontierRef {
                domain: id("policy"),
                sequence: 10,
                digest: Digest32::repeat(10),
            },
            checked_at_unix_ms: 1_300,
        };
        assert_eq!(
            contract().evaluate(&decision(), &coordination(), 7, 1_350, &evidence),
            Err(RevalidationError::ExactFrontierChanged {
                class: FrontierClass::Policy,
                domain: id("policy"),
            })
        );
    }

    #[test]
    fn weak_policy_revalidation_is_rejected_by_contract() {
        let mut value = contract();
        value.rules[1] = RevalidationRule::FreshnessOnly {
            class: FrontierClass::Policy,
            domain: id("policy"),
            maximum_source_age_ms: 1_000,
        };
        assert_eq!(
            value.validate(),
            Err(RevalidationContractError::WeakRuleForPolicyOrAuthority {
                class: FrontierClass::Policy,
                domain: id("policy"),
            })
        );
    }

    #[test]
    fn missing_prepared_authority_frontier_rule_fails_closed() {
        let mut value = contract();
        value.rules.pop();
        assert_eq!(
            value.evaluate(&decision(), &coordination(), 7, 1_350, &evidence()[..2]),
            Err(RevalidationError::MissingPreparedAuthorityFrontier {
                domain: id("authority"),
            })
        );
    }

    #[test]
    fn authority_epoch_change_invalidates_prepared_action() {
        assert!(matches!(
            contract().evaluate(&decision(), &coordination(), 8, 1_350, &evidence()),
            Err(RevalidationError::InvalidPreparedAction(_))
        ));
    }

    #[test]
    fn stale_dependency_witness_fails_closed() {
        let mut evidence = evidence();
        evidence[0] = RevalidationEvidence::DependencyWitness {
            class: FrontierClass::Observation,
            domain: id("inventory"),
            condition: id("condition:stock-at-least-20kg"),
            condition_digest: Digest32::repeat(30),
            witness: id("witness:inventory:1"),
            source_age_ms: 30_001,
            checked_at_unix_ms: 1_300,
        };
        assert_eq!(
            contract().evaluate(&decision(), &coordination(), 7, 1_350, &evidence),
            Err(RevalidationError::StaleSource {
                domain: id("inventory"),
                actual_age_ms: 30_001,
                maximum_age_ms: 30_000,
            })
        );
    }
}
