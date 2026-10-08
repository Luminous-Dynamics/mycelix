#![forbid(unsafe_code)]
// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! FIN-SAFE-013: pure economic-effect theorem kernel.
//!
//! This crate does not read Holochain state and does not choose winners.
//! It defines the minimum typed economic object needed to make "exact
//! conservation" mean something stronger than "the source had enough".
//!
//! Core distinction:
//!
//!   debit capacity:     debit >= credit
//!   exact allocation:   sum(debits) == sum(credits)
//!
//! Mint and burn are explicit typed supply-boundary mutations. They are not
//! represented as unexplained remainders in an ordinary transfer-like effect.
//!
//! The kernel is intentionally independent of HDK/HDI so it can be reused by
//! validators, coordinators, simulation, and an external qualification harness.

use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};

pub const ECONOMIC_EFFECT_V1_SCHEMA_VERSION: u16 = 1;
pub const MAX_ID_LEN: usize = 256;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum MutationClass {
    Transfer,
    Fee,
    Mint,
    Burn,
    Demurrage,
    TreasuryAllocation,
    CollateralLock,
    CollateralRelease,
    StakingReturn,
    StakingSlash,
    ReconciliationCorrection,
}

impl MutationClass {
    pub fn requires_exact_conservation(self) -> bool {
        !matches!(self, Self::Mint | Self::Burn)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum AllocationRole {
    Account,
    Treasury,
    Commons,
    BurnSink,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct EconomicAllocation {
    pub party_id: String,
    pub role: AllocationRole,
    pub amount: u64,
}

impl EconomicAllocation {
    fn validate(&self) -> Result<(), EconomicEffectError> {
        validate_id(&self.party_id, EconomicEffectError::InvalidPartyId)?;
        if self.amount == 0 {
            return Err(EconomicEffectError::ZeroAllocation);
        }
        Ok(())
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct EconomicEffectV1 {
    pub schema_version: u16,
    pub effect_id: String,
    pub predecessor_action_reference: String,
    pub cause_action_reference: String,
    pub asset: String,
    pub mutation_class: MutationClass,
    pub source_owner: String,
    pub debits: Vec<EconomicAllocation>,
    pub credits: Vec<EconomicAllocation>,
}

impl EconomicEffectV1 {
    pub fn validate(
        &self,
        context: EffectValidationContext<'_>,
    ) -> Result<ValidatedEconomicEffect<'_>, EconomicEffectError> {
        if self.schema_version != ECONOMIC_EFFECT_V1_SCHEMA_VERSION {
            return Err(EconomicEffectError::UnsupportedSchema);
        }
        validate_id(&self.effect_id, EconomicEffectError::InvalidEffectId)?;
        validate_id(
            &self.predecessor_action_reference,
            EconomicEffectError::InvalidPredecessorReference,
        )?;
        validate_id(
            &self.cause_action_reference,
            EconomicEffectError::InvalidCauseReference,
        )?;
        validate_id(&self.asset, EconomicEffectError::InvalidAsset)?;
        validate_id(
            &self.source_owner,
            EconomicEffectError::InvalidSourceOwner,
        )?;

        if context.seen_effect_ids.iter().any(|id| id == &self.effect_id) {
            return Err(EconomicEffectError::EffectReplay);
        }

        let debit_total = checked_total(&self.debits)?;
        let credit_total = checked_total(&self.credits)?;

        if self.debits.is_empty() && self.credits.is_empty() {
            return Err(EconomicEffectError::EmptyEffect);
        }

        match self.mutation_class {
            MutationClass::Transfer => {
                require_one(&self.debits, AllocationRole::Account)?;
                require_one(&self.credits, AllocationRole::Account)?;
                self.require_source_account()?;
                self.require_distinct_counterparty()?;
                require_equal_totals(debit_total, credit_total)?;
            }
            MutationClass::Fee => {
                require_one(&self.debits, AllocationRole::Account)?;
                require_one(&self.credits, AllocationRole::Treasury)?;
                self.require_source_account()?;
                require_equal_totals(debit_total, credit_total)?;
            }
            MutationClass::Mint => {
                if !self.debits.is_empty() {
                    return Err(EconomicEffectError::MintHasDebits);
                }
                if self.credits.is_empty() {
                    return Err(EconomicEffectError::MintHasNoDestination);
                }
            }
            MutationClass::Burn => {
                require_one(&self.debits, AllocationRole::Account)?;
                require_one(&self.credits, AllocationRole::BurnSink)?;
                self.require_source_account()?;
                require_equal_totals(debit_total, credit_total)?;
            }
            MutationClass::Demurrage => {
                require_one(&self.debits, AllocationRole::Account)?;
                self.require_source_account()?;
                if self.credits.is_empty() {
                    return Err(EconomicEffectError::MissingDemurrageDestination);
                }
                for credit in &self.credits {
                    if !matches!(credit.role, AllocationRole::Commons | AllocationRole::Treasury | AllocationRole::BurnSink) {
                        return Err(EconomicEffectError::InvalidDemurrageDestination);
                    }
                }
                require_equal_totals(debit_total, credit_total)?;
            }
            MutationClass::TreasuryAllocation => {
                require_one(&self.debits, AllocationRole::Treasury)?;
                if self.credits.is_empty() {
                    return Err(EconomicEffectError::MissingAllocationDestination);
                }
                require_equal_totals(debit_total, credit_total)?;
            }
            MutationClass::CollateralLock
            | MutationClass::CollateralRelease
            | MutationClass::StakingReturn
            | MutationClass::StakingSlash
            | MutationClass::ReconciliationCorrection => {
                if self.debits.is_empty() || self.credits.is_empty() {
                    return Err(EconomicEffectError::MissingConservationLeg);
                }
                require_equal_totals(debit_total, credit_total)?;
            }
        }

        Ok(ValidatedEconomicEffect {
            effect: self,
            debit_total,
            credit_total,
        })
    }

    fn require_source_account(&self) -> Result<(), EconomicEffectError> {
        if self.debits.len() == 1
            && self.debits[0].role == AllocationRole::Account
            && self.debits[0].party_id == self.source_owner
        {
            Ok(())
        } else {
            Err(EconomicEffectError::UnauthorizedSource)
        }
    }

    fn require_distinct_counterparty(&self) -> Result<(), EconomicEffectError> {
        if self.credits.len() == 1
            && self.credits[0].role == AllocationRole::Account
            && self.credits[0].party_id != self.source_owner
        {
            Ok(())
        } else {
            Err(EconomicEffectError::SelfTransferOrInvalidCounterparty)
        }
    }

    pub fn expected_total(&self) -> Result<u64, EconomicEffectError> {
        let debit_total = checked_total(&self.debits)?;
        let credit_total = checked_total(&self.credits)?;

        if self.mutation_class.requires_exact_conservation() {
            require_equal_totals(debit_total, credit_total)?;
        }
        Ok(credit_total)
    }
}

#[derive(Clone, Copy, Debug)]
pub struct EffectValidationContext<'a> {
    pub seen_effect_ids: &'a [String],
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ValidatedEconomicEffect<'a> {
    pub effect: &'a EconomicEffectV1,
    pub debit_total: u64,
    pub credit_total: u64,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct EconomicEffectObservation {
    pub predecessor_action_reference: String,
    pub successor_action_reference: String,
    pub effect_id: String,
}

impl EconomicEffectObservation {
    fn validate_shape(&self) -> Result<(), EconomicEffectError> {
        validate_id(
            &self.predecessor_action_reference,
            EconomicEffectError::InvalidPredecessorReference,
        )?;
        validate_id(
            &self.successor_action_reference,
            EconomicEffectError::InvalidSuccessorReference,
        )?;
        validate_id(&self.effect_id, EconomicEffectError::InvalidEffectId)?;
        if self.predecessor_action_reference == self.successor_action_reference {
            return Err(EconomicEffectError::SelfReferentialSuccessor);
        }
        Ok(())
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum EffectLineageStatus {
    Current {
        predecessor_action_reference: String,
    },
    Advanced {
        predecessor_action_reference: String,
        successor_action_reference: String,
    },
    Conflict {
        predecessor_action_reference: String,
        successor_action_references: Vec<String>,
    },
}

impl EffectLineageStatus {
    pub fn is_spendable(&self) -> bool {
        !matches!(self, Self::Conflict { .. })
    }
}

/// Classify observed economic successors without selecting a winner.
/// Distinct successors from one exact predecessor remain a conflict even when
/// their effect payloads would otherwise be identical.
pub fn classify_successors(
    predecessor_action_reference: &str,
    observations: &[EconomicEffectObservation],
) -> Result<EffectLineageStatus, EconomicEffectError> {
    validate_id(
        predecessor_action_reference,
        EconomicEffectError::InvalidPredecessorReference,
    )?;

    let mut unique_by_action: BTreeMap<String, String> = BTreeMap::new();
    for observation in observations {
        observation.validate_shape()?;
        if observation.predecessor_action_reference != predecessor_action_reference {
            return Err(EconomicEffectError::PredecessorReferenceMismatch);
        }
        match unique_by_action.get(&observation.successor_action_reference) {
            Some(existing) if existing == &observation.effect_id => {}
            Some(_) => return Err(EconomicEffectError::InconsistentDuplicateAction),
            None => {
                unique_by_action.insert(
                    observation.successor_action_reference.clone(),
                    observation.effect_id.clone(),
                );
            }
        }
    }

    match unique_by_action.len() {
        0 => Ok(EffectLineageStatus::Current {
            predecessor_action_reference: predecessor_action_reference.to_owned(),
        }),
        1 => Ok(EffectLineageStatus::Advanced {
            predecessor_action_reference: predecessor_action_reference.to_owned(),
            successor_action_reference: unique_by_action
                .into_keys()
                .next()
                .expect("length checked"),
        }),
        _ => Ok(EffectLineageStatus::Conflict {
            predecessor_action_reference: predecessor_action_reference.to_owned(),
            successor_action_references: unique_by_action.into_keys().collect(),
        }),
    }
}

fn checked_total(
    allocations: &[EconomicAllocation],
) -> Result<u64, EconomicEffectError> {
    let mut total = 0_u64;
    let mut seen = BTreeSet::new();
    for allocation in allocations {
        allocation.validate()?;
        let identity = (
            allocation.role as u8,
            allocation.party_id.as_str(),
        );
        if !seen.insert(identity) {
            return Err(EconomicEffectError::DuplicateAllocation);
        }
        total = total
            .checked_add(allocation.amount)
            .ok_or(EconomicEffectError::AllocationOverflow)?;
    }
    Ok(total)
}

fn require_one(
    allocations: &[EconomicAllocation],
    role: AllocationRole,
) -> Result<(), EconomicEffectError> {
    if allocations.len() == 1 && allocations[0].role == role {
        Ok(())
    } else {
        Err(EconomicEffectError::WrongLegStructure)
    }
}

fn require_equal_totals(
    debit_total: u64,
    credit_total: u64,
) -> Result<(), EconomicEffectError> {
    if debit_total == credit_total {
        Ok(())
    } else if debit_total > credit_total {
        Err(EconomicEffectError::UnaccountedDebitRemainder {
            debit_total,
            credit_total,
        })
    } else {
        Err(EconomicEffectError::InsufficientDebit {
            debit_total,
            credit_total,
        })
    }
}

fn validate_id(value: &str, error: EconomicEffectError) -> Result<(), EconomicEffectError> {
    if value.is_empty() || value.len() > MAX_ID_LEN {
        return Err(error);
    }
    Ok(())
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum EconomicEffectError {
    UnsupportedSchema,
    InvalidEffectId,
    InvalidPredecessorReference,
    InvalidCauseReference,
    InvalidSuccessorReference,
    InvalidAsset,
    InvalidSourceOwner,
    InvalidPartyId,
    ZeroAllocation,
    EmptyEffect,
    DuplicateAllocation,
    AllocationOverflow,
    InsufficientDebit { debit_total: u64, credit_total: u64 },
    UnaccountedDebitRemainder { debit_total: u64, credit_total: u64 },
    UnauthorizedSource,
    SelfTransferOrInvalidCounterparty,
    MintHasDebits,
    MintHasNoDestination,
    MissingDemurrageDestination,
    InvalidDemurrageDestination,
    MissingAllocationDestination,
    MissingConservationLeg,
    WrongLegStructure,
    EffectReplay,
    PredecessorReferenceMismatch,
    SelfReferentialSuccessor,
    InconsistentDuplicateAction,
}

#[cfg(test)]
mod tests {
    use super::*;

    fn allocation(party: &str, role: AllocationRole, amount: u64) -> EconomicAllocation {
        EconomicAllocation {
            party_id: party.into(),
            role,
            amount,
        }
    }

    fn base_transfer() -> EconomicEffectV1 {
        EconomicEffectV1 {
            schema_version: ECONOMIC_EFFECT_V1_SCHEMA_VERSION,
            effect_id: "effect-1".into(),
            predecessor_action_reference: "prev-1".into(),
            cause_action_reference: "cause-1".into(),
            asset: "SAP".into(),
            mutation_class: MutationClass::Transfer,
            source_owner: "alice".into(),
            debits: vec![allocation("alice", AllocationRole::Account, 100)],
            credits: vec![allocation("bob", AllocationRole::Account, 100)],
        }
    }

    fn validate(effect: &EconomicEffectV1) -> Result<ValidatedEconomicEffect<'_>, EconomicEffectError> {
        effect.validate(EffectValidationContext { seen_effect_ids: &[] })
    }

    #[test]
    fn exact_transfer_is_valid() {
        let effect = base_transfer();
        let validated = validate(&effect).unwrap();
        assert_eq!(validated.debit_total, 100);
        assert_eq!(validated.credit_total, 100);
        assert_eq!(effect.expected_total(), Ok(100));
    }

    #[test]
    fn under_consumption_is_rejected_not_treated_as_capacity() {
        let mut effect = base_transfer();
        effect.debits[0].amount = 100;
        effect.credits[0].amount = 50;
        assert_eq!(
            validate(&effect),
            Err(EconomicEffectError::UnaccountedDebitRemainder {
                debit_total: 100,
                credit_total: 50
            })
        );
    }

    #[test]
    fn remainder_laundering_cannot_hide_in_a_transfer() {
        let mut effect = base_transfer();
        effect.debits[0].amount = 100;
        effect.credits[0].amount = 50;
        effect.credits.push(allocation("bob-fee", AllocationRole::Treasury, 50));
        assert_eq!(
            validate(&effect),
            Err(EconomicEffectError::WrongLegStructure)
        );

        effect.mutation_class = MutationClass::Fee;
        effect.credits = vec![allocation("treasury", AllocationRole::Treasury, 100)];
        assert!(validate(&effect).is_err());
        effect.debits[0].amount = 100;
        assert!(validate(&effect).is_ok());
    }

    #[test]
    fn self_burn_requires_typed_burn_sink() {
        let mut effect = base_transfer();
        effect.mutation_class = MutationClass::Burn;
        effect.credits = vec![allocation("alice", AllocationRole::Account, 100)];
        assert_eq!(
            validate(&effect),
            Err(EconomicEffectError::WrongLegStructure)
        );
        effect.credits = vec![allocation("burn", AllocationRole::BurnSink, 100)];
        assert!(validate(&effect).is_ok());
    }

    #[test]
    fn cross_account_substitution_is_rejected() {
        let mut effect = base_transfer();
        effect.debits[0].party_id = "bob".into();
        assert_eq!(
            validate(&effect),
            Err(EconomicEffectError::UnauthorizedSource)
        );
    }

    #[test]
    fn self_transfer_is_rejected_even_when_conserved() {
        let mut effect = base_transfer();
        effect.credits[0].party_id = "alice".into();
        assert_eq!(
            validate(&effect),
            Err(EconomicEffectError::SelfTransferOrInvalidCounterparty)
        );
    }

    #[test]
    fn replay_is_rejected_by_effect_identity() {
        let effect = base_transfer();
        assert_eq!(
            effect.validate(EffectValidationContext {
                seen_effect_ids: &["effect-1".into()]
            }),
            Err(EconomicEffectError::EffectReplay)
        );
    }

    #[test]
    fn mint_is_explicitly_typed_not_an_unbalanced_transfer() {
        let mut effect = base_transfer();
        effect.mutation_class = MutationClass::Mint;
        effect.debits.clear();
        effect.credits = vec![allocation("alice", AllocationRole::Account, 100)];
        assert!(validate(&effect).is_ok());

        let mut malformed = effect.clone();
        malformed.debits = vec![allocation("external", AllocationRole::Treasury, 1)];
        assert_eq!(
            validate(&malformed),
            Err(EconomicEffectError::MintHasDebits)
        );
    }

    #[test]
    fn demurrage_requires_full_typed_routing() {
        let mut effect = base_transfer();
        effect.mutation_class = MutationClass::Demurrage;
        effect.debits = vec![allocation("alice", AllocationRole::Account, 70)];
        effect.credits = vec![
            allocation("local", AllocationRole::Commons, 49),
            allocation("regional", AllocationRole::Commons, 14),
            allocation("global", AllocationRole::Commons, 7),
        ];
        assert!(validate(&effect).is_ok());

        effect.credits.pop();
        assert!(matches!(
            validate(&effect),
            Err(EconomicEffectError::UnaccountedDebitRemainder { .. })
        ));
    }

    #[test]
    fn overflow_fails_closed() {
        let mut effect = base_transfer();
        effect.debits[0].amount = u64::MAX;
        effect.credits[0].amount = u64::MAX;
        effect.credits.push(allocation("treasury", AllocationRole::Treasury, 1));
        assert_eq!(
            validate(&effect),
            Err(EconomicEffectError::AllocationOverflow)
        );
    }

    #[test]
    fn fork_is_conflict_not_winner_selection() {
        let observations = vec![
            EconomicEffectObservation {
                predecessor_action_reference: "prev-1".into(),
                successor_action_reference: "succ-a".into(),
                effect_id: "effect-a".into(),
            },
            EconomicEffectObservation {
                predecessor_action_reference: "prev-1".into(),
                successor_action_reference: "succ-b".into(),
                effect_id: "effect-b".into(),
            },
        ];
        let status = classify_successors("prev-1", &observations).unwrap();
        assert!(!status.is_spendable());
        assert!(matches!(status, EffectLineageStatus::Conflict { .. }));
    }

    #[test]
    fn repeated_same_action_is_only_deduped_when_effect_identity_matches() {
        let observations = vec![
            EconomicEffectObservation {
                predecessor_action_reference: "prev-1".into(),
                successor_action_reference: "succ-a".into(),
                effect_id: "effect-a".into(),
            },
            EconomicEffectObservation {
                predecessor_action_reference: "prev-1".into(),
                successor_action_reference: "succ-a".into(),
                effect_id: "effect-a".into(),
            },
        ];
        assert!(matches!(
            classify_successors("prev-1", &observations).unwrap(),
            EffectLineageStatus::Advanced { .. }
        ));

        let inconsistent = vec![
            EconomicEffectObservation {
                predecessor_action_reference: "prev-1".into(),
                successor_action_reference: "succ-a".into(),
                effect_id: "effect-a".into(),
            },
            EconomicEffectObservation {
                predecessor_action_reference: "prev-1".into(),
                successor_action_reference: "succ-a".into(),
                effect_id: "effect-b".into(),
            },
        ];
        assert_eq!(
            classify_successors("prev-1", &inconsistent),
            Err(EconomicEffectError::InconsistentDuplicateAction)
        );
    }
}
