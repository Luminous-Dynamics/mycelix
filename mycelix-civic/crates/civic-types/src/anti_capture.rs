// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Constitutional anti-capture invariants for Mycelix Civic.
//!
//! This module is intentionally small. It encodes structural rules that downstream
//! governance modules must not bypass; it does not decide policy, guilt, merit, or
//! political outcomes.

use serde::{Deserialize, Serialize};

/// Basis used to allocate *fundamental* civic standing.
///
/// Fundamental civic standing is deliberately narrower than bounded, delegated
/// domain authority. Expertise may justify a scoped mandate; it may not buy or
/// multiply a person's basic civic sovereignty.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum CivicPowerBasis {
    EqualPerson,
    EconomicCapital,
    TokenBalance,
    Reputation,
    ComputeOwnership,
}

/// Consequence class for a decision or automated process.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum DecisionEffect {
    Advisory,
    Administrative,
    RightsAffecting,
}

/// Role assigned to an automated or AI system.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum AutomatedDecisionRole {
    Advisory,
    FinalDecisionMaker,
}

/// Epistemic basis offered for a consequence.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum ConsequenceBasis {
    AdjudicatedFinding,
    Observation,
    CaptureSignal,
    AutomatedInference,
    ReputationScore,
}

/// Scope of a reputation assertion.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum ReputationScope {
    DomainSpecific(String),
    Universal,
}

/// A bounded grant of delegated authority.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct DelegatedAuthority {
    pub subject: String,
    pub domain: String,
    pub scope: String,
    pub purpose: String,
    pub issued_at: u64,
    pub expires_at: Option<u64>,
    pub revocable: bool,
    pub appealable: bool,
}

/// Minimum provenance required when public power is exercised.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct PublicPowerRecord {
    pub authority_ref: String,
    pub decision_reason: String,
    pub provenance_refs: Vec<String>,
}

/// Actions reviewed by the constitutional anti-capture kernel.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum ConstitutionalAction {
    ExerciseFundamentalCivicPower {
        basis: CivicPowerBasis,
    },
    DelegateAuthority(DelegatedAuthority),
    AutomatedDecision {
        role: AutomatedDecisionRole,
        effect: DecisionEffect,
    },
    ApplyConsequence {
        effect: DecisionEffect,
        basis: ConsequenceBasis,
        appealable: bool,
    },
    PublishReputation {
        scope: ReputationScope,
    },
    ExercisePublicPower(PublicPowerRecord),
    ConfigureFederation {
        exit_allowed: bool,
        provider_replaceable: bool,
    },
}

/// Fail-closed constitutional violations.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum AntiCaptureViolation {
    PurchasableFundamentalCivicPower,
    AuthorityMissingSubject,
    AuthorityMissingDomain,
    AuthorityMissingScope,
    AuthorityMissingPurpose,
    AuthorityWithoutExpiry,
    AuthorityExpiryNotAfterIssue,
    AuthorityIrrevocable,
    AuthorityWithoutAppeal,
    AiFinalRightsDecision,
    RightsConsequenceWithoutAdjudication,
    RightsConsequenceWithoutAppeal,
    UniversalReputationScore,
    PublicPowerMissingAuthority,
    PublicPowerMissingReason,
    PublicPowerMissingProvenance,
    FederationWithoutExit,
    FederationProviderLockIn,
}

/// Stateless validator for the AC-001 constitutional boundary.
#[derive(Debug, Default, Clone, Copy)]
pub struct AntiCaptureKernel;

impl AntiCaptureKernel {
    /// Validate one constitutional action.
    ///
    /// The validator is fail-closed: any structural ambiguity represented here is
    /// rejected rather than silently interpreted as legitimate authority.
    pub fn validate(action: &ConstitutionalAction) -> Result<(), Vec<AntiCaptureViolation>> {
        let mut violations = Vec::new();

        match action {
            ConstitutionalAction::ExerciseFundamentalCivicPower { basis } => {
                if *basis != CivicPowerBasis::EqualPerson {
                    violations.push(AntiCaptureViolation::PurchasableFundamentalCivicPower);
                }
            }
            ConstitutionalAction::DelegateAuthority(grant) => {
                if grant.subject.trim().is_empty() {
                    violations.push(AntiCaptureViolation::AuthorityMissingSubject);
                }
                if grant.domain.trim().is_empty() {
                    violations.push(AntiCaptureViolation::AuthorityMissingDomain);
                }
                if grant.scope.trim().is_empty() {
                    violations.push(AntiCaptureViolation::AuthorityMissingScope);
                }
                if grant.purpose.trim().is_empty() {
                    violations.push(AntiCaptureViolation::AuthorityMissingPurpose);
                }
                match grant.expires_at {
                    Some(expires_at) if expires_at > grant.issued_at => {}
                    Some(_) => violations.push(AntiCaptureViolation::AuthorityExpiryNotAfterIssue),
                    None => violations.push(AntiCaptureViolation::AuthorityWithoutExpiry),
                }
                if !grant.revocable {
                    violations.push(AntiCaptureViolation::AuthorityIrrevocable);
                }
                if !grant.appealable {
                    violations.push(AntiCaptureViolation::AuthorityWithoutAppeal);
                }
            }
            ConstitutionalAction::AutomatedDecision { role, effect } => {
                if *role == AutomatedDecisionRole::FinalDecisionMaker
                    && *effect == DecisionEffect::RightsAffecting
                {
                    violations.push(AntiCaptureViolation::AiFinalRightsDecision);
                }
            }
            ConstitutionalAction::ApplyConsequence {
                effect,
                basis,
                appealable,
            } => {
                if *effect == DecisionEffect::RightsAffecting {
                    if *basis != ConsequenceBasis::AdjudicatedFinding {
                        violations.push(
                            AntiCaptureViolation::RightsConsequenceWithoutAdjudication,
                        );
                    }
                    if !appealable {
                        violations.push(AntiCaptureViolation::RightsConsequenceWithoutAppeal);
                    }
                }
            }
            ConstitutionalAction::PublishReputation { scope } => {
                if *scope == ReputationScope::Universal {
                    violations.push(AntiCaptureViolation::UniversalReputationScore);
                }
            }
            ConstitutionalAction::ExercisePublicPower(record) => {
                if record.authority_ref.trim().is_empty() {
                    violations.push(AntiCaptureViolation::PublicPowerMissingAuthority);
                }
                if record.decision_reason.trim().is_empty() {
                    violations.push(AntiCaptureViolation::PublicPowerMissingReason);
                }
                if record.provenance_refs.is_empty()
                    || record.provenance_refs.iter().any(|item| item.trim().is_empty())
                {
                    violations.push(AntiCaptureViolation::PublicPowerMissingProvenance);
                }
            }
            ConstitutionalAction::ConfigureFederation {
                exit_allowed,
                provider_replaceable,
            } => {
                if !exit_allowed {
                    violations.push(AntiCaptureViolation::FederationWithoutExit);
                }
                if !provider_replaceable {
                    violations.push(AntiCaptureViolation::FederationProviderLockIn);
                }
            }
        }

        if violations.is_empty() {
            Ok(())
        } else {
            Err(violations)
        }
    }

    /// Validate a sequence of actions without erasing which action failed.
    pub fn validate_all<'a>(
        actions: impl IntoIterator<Item = &'a ConstitutionalAction>,
    ) -> Result<(), Vec<(usize, Vec<AntiCaptureViolation>)>> {
        let failures: Vec<_> = actions
            .into_iter()
            .enumerate()
            .filter_map(|(index, action)| Self::validate(action).err().map(|e| (index, e)))
            .collect();

        if failures.is_empty() {
            Ok(())
        } else {
            Err(failures)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn bounded_grant() -> DelegatedAuthority {
        DelegatedAuthority {
            subject: "did:example:alice".into(),
            domain: "municipal-water".into(),
            scope: "approve-maintenance".into(),
            purpose: "maintain service reliability".into(),
            issued_at: 100,
            expires_at: Some(200),
            revocable: true,
            appealable: true,
        }
    }

    #[test]
    fn equal_person_is_the_only_fundamental_civic_power_basis() {
        let equal = ConstitutionalAction::ExerciseFundamentalCivicPower {
            basis: CivicPowerBasis::EqualPerson,
        };
        assert_eq!(AntiCaptureKernel::validate(&equal), Ok(()));

        for basis in [
            CivicPowerBasis::EconomicCapital,
            CivicPowerBasis::TokenBalance,
            CivicPowerBasis::Reputation,
            CivicPowerBasis::ComputeOwnership,
        ] {
            let action = ConstitutionalAction::ExerciseFundamentalCivicPower { basis };
            assert_eq!(
                AntiCaptureKernel::validate(&action),
                Err(vec![
                    AntiCaptureViolation::PurchasableFundamentalCivicPower
                ])
            );
        }
    }

    #[test]
    fn bounded_delegated_authority_is_accepted() {
        assert_eq!(
            AntiCaptureKernel::validate(&ConstitutionalAction::DelegateAuthority(bounded_grant())),
            Ok(())
        );
    }

    #[test]
    fn permanent_irrevocable_unappealable_authority_is_rejected() {
        let mut grant = bounded_grant();
        grant.expires_at = None;
        grant.revocable = false;
        grant.appealable = false;

        let errors = AntiCaptureKernel::validate(&ConstitutionalAction::DelegateAuthority(grant))
            .expect_err("unbounded authority must fail closed");
        assert!(errors.contains(&AntiCaptureViolation::AuthorityWithoutExpiry));
        assert!(errors.contains(&AntiCaptureViolation::AuthorityIrrevocable));
        assert!(errors.contains(&AntiCaptureViolation::AuthorityWithoutAppeal));
    }

    #[test]
    fn non_forward_expiry_is_rejected() {
        let mut grant = bounded_grant();
        grant.expires_at = Some(grant.issued_at);
        assert_eq!(
            AntiCaptureKernel::validate(&ConstitutionalAction::DelegateAuthority(grant)),
            Err(vec![AntiCaptureViolation::AuthorityExpiryNotAfterIssue])
        );
    }

    #[test]
    fn ai_cannot_be_final_authority_for_rights() {
        let action = ConstitutionalAction::AutomatedDecision {
            role: AutomatedDecisionRole::FinalDecisionMaker,
            effect: DecisionEffect::RightsAffecting,
        };
        assert_eq!(
            AntiCaptureKernel::validate(&action),
            Err(vec![AntiCaptureViolation::AiFinalRightsDecision])
        );
    }

    #[test]
    fn ai_may_advise_on_rights_affecting_matters() {
        let action = ConstitutionalAction::AutomatedDecision {
            role: AutomatedDecisionRole::Advisory,
            effect: DecisionEffect::RightsAffecting,
        };
        assert_eq!(AntiCaptureKernel::validate(&action), Ok(()));
    }

    #[test]
    fn capture_signal_cannot_directly_remove_rights() {
        let action = ConstitutionalAction::ApplyConsequence {
            effect: DecisionEffect::RightsAffecting,
            basis: ConsequenceBasis::CaptureSignal,
            appealable: true,
        };
        assert_eq!(
            AntiCaptureKernel::validate(&action),
            Err(vec![
                AntiCaptureViolation::RightsConsequenceWithoutAdjudication
            ])
        );
    }

    #[test]
    fn rights_affecting_consequences_require_appeal() {
        let action = ConstitutionalAction::ApplyConsequence {
            effect: DecisionEffect::RightsAffecting,
            basis: ConsequenceBasis::AdjudicatedFinding,
            appealable: false,
        };
        assert_eq!(
            AntiCaptureKernel::validate(&action),
            Err(vec![AntiCaptureViolation::RightsConsequenceWithoutAppeal])
        );
    }

    #[test]
    fn universal_reputation_is_rejected_but_domain_reputation_is_allowed() {
        let universal = ConstitutionalAction::PublishReputation {
            scope: ReputationScope::Universal,
        };
        assert_eq!(
            AntiCaptureKernel::validate(&universal),
            Err(vec![AntiCaptureViolation::UniversalReputationScore])
        );

        let scoped = ConstitutionalAction::PublishReputation {
            scope: ReputationScope::DomainSpecific("water-systems".into()),
        };
        assert_eq!(AntiCaptureKernel::validate(&scoped), Ok(()));
    }

    #[test]
    fn exercise_of_public_power_requires_reason_and_provenance() {
        let action = ConstitutionalAction::ExercisePublicPower(PublicPowerRecord {
            authority_ref: "grant:123".into(),
            decision_reason: String::new(),
            provenance_refs: vec![],
        });
        let errors = AntiCaptureKernel::validate(&action)
            .expect_err("opaque public power must fail closed");
        assert!(errors.contains(&AntiCaptureViolation::PublicPowerMissingReason));
        assert!(errors.contains(&AntiCaptureViolation::PublicPowerMissingProvenance));
    }

    #[test]
    fn federation_requires_exit_and_provider_replaceability() {
        let action = ConstitutionalAction::ConfigureFederation {
            exit_allowed: false,
            provider_replaceable: false,
        };
        assert_eq!(
            AntiCaptureKernel::validate(&action),
            Err(vec![
                AntiCaptureViolation::FederationWithoutExit,
                AntiCaptureViolation::FederationProviderLockIn,
            ])
        );
    }

    #[test]
    fn validate_all_preserves_action_index() {
        let good = ConstitutionalAction::ExerciseFundamentalCivicPower {
            basis: CivicPowerBasis::EqualPerson,
        };
        let bad = ConstitutionalAction::PublishReputation {
            scope: ReputationScope::Universal,
        };
        let actions = [good, bad];

        assert_eq!(
            AntiCaptureKernel::validate_all(actions.iter()),
            Err(vec![(
                1,
                vec![AntiCaptureViolation::UniversalReputationScore]
            )])
        );
    }
}
