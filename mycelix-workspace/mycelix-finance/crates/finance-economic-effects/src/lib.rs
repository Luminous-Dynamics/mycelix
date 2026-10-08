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
pub const MAX_CAUSE_REFERENCES: usize = 256;

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

/// Canonical semantic identity of one economic effect.
///
/// Physical Holochain/source-chain references and the economic identity are
/// deliberately separate. The former establish provenance and lineage; the
/// latter is the idempotency key for the underlying economic event. A source
/// theorem must define that key (for collateral issuance, FIN-SAFE-014 uses
/// the canonical mint_id).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum EconomicIdentityNamespaceV1 {
    CollateralMint,
    GovernanceMint,
    Transfer,
    Fee,
    Demurrage,
    TreasuryAllocation,
    Burn,
    CollateralLock,
    CollateralRelease,
    StakingReturn,
    StakingSlash,
    ReconciliationCorrection,
}

#[derive(Clone, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct EconomicIdentityV1 {
    /// Source-defined canonical economic key. For example, collateral
    /// issuance binds this to FIN-SAFE-010's canonical mint_id; transfer
    /// binds it to the canonical transfer ID.
    pub namespace: EconomicIdentityNamespaceV1,
    pub key: String,
}

#[derive(Clone, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct EconomicEffectIdentityV1 {
    pub economic_identity: EconomicIdentityV1,
    pub predecessor_action_reference: String,
    pub cause_action_references: Vec<String>,
    pub asset: String,
    pub mutation_class: MutationClass,
    pub source_owner: String,
}

impl EconomicEffectIdentityV1 {
    fn validate(&self) -> Result<(), EconomicEffectError> {
        validate_id(
            &self.economic_identity.key,
            EconomicEffectError::InvalidEconomicIdentity,
        )?;
        match (self.economic_identity.namespace, self.mutation_class) {
            (EconomicIdentityNamespaceV1::Transfer, MutationClass::Transfer)
            | (EconomicIdentityNamespaceV1::Fee, MutationClass::Fee)
            | (EconomicIdentityNamespaceV1::Demurrage, MutationClass::Demurrage)
            | (
                EconomicIdentityNamespaceV1::TreasuryAllocation,
                MutationClass::TreasuryAllocation,
            )
            | (EconomicIdentityNamespaceV1::Burn, MutationClass::Burn)
            | (EconomicIdentityNamespaceV1::CollateralMint, MutationClass::Mint)
            | (EconomicIdentityNamespaceV1::GovernanceMint, MutationClass::Mint)
            | (EconomicIdentityNamespaceV1::CollateralLock, MutationClass::CollateralLock)
            | (EconomicIdentityNamespaceV1::CollateralRelease, MutationClass::CollateralRelease)
            | (EconomicIdentityNamespaceV1::StakingReturn, MutationClass::StakingReturn)
            | (EconomicIdentityNamespaceV1::StakingSlash, MutationClass::StakingSlash)
            | (
                EconomicIdentityNamespaceV1::ReconciliationCorrection,
                MutationClass::ReconciliationCorrection,
            ) => {}
            _ => return Err(EconomicEffectError::IdentityNamespaceMismatch),
        }
        validate_id(
            &self.predecessor_action_reference,
            EconomicEffectError::InvalidPredecessorReference,
        )?;
        if self.cause_action_references.is_empty()
            || self.cause_action_references.len() > MAX_CAUSE_REFERENCES
        {
            return Err(EconomicEffectError::InvalidCauseReferences);
        }
        let mut seen_causes = BTreeSet::new();
        for cause in &self.cause_action_references {
            validate_id(cause, EconomicEffectError::InvalidCauseReference)?;
            if !seen_causes.insert(cause.as_str()) {
                return Err(EconomicEffectError::DuplicateCauseReference);
            }
        }
        validate_id(&self.asset, EconomicEffectError::InvalidAsset)?;
        validate_id(&self.source_owner, EconomicEffectError::InvalidSourceOwner)?;
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
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
    pub identity: EconomicEffectIdentityV1,

    pub debits: Vec<EconomicAllocation>,
    pub credits: Vec<EconomicAllocation>,
}

/// Minimal mutable-account state used only to prove that one balance
/// successor contains exactly the monetary mutation described by one effect.
/// Non-monetary account metadata is deliberately required to remain unchanged.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct AccountBalanceProjectionV1 {
    pub owner: String,
    pub balance: u64,
    pub demurrage_marker: i64,
    pub protected_state: Option<String>,
}

impl AccountBalanceProjectionV1 {
    fn validate(&self) -> Result<(), EconomicEffectError> {
        validate_id(&self.owner, EconomicEffectError::InvalidSourceOwner)?;
        validate_id(
            self.protected_state.as_deref().unwrap_or("none"),
            EconomicEffectError::InvalidProtectedState,
        )?;
        Ok(())
    }
}

/// Validate one mutable account projection against one typed economic effect.
///
/// This deliberately does not calculate demurrage. It only enforces the
/// mutation boundary: demurrage metadata may change only for a Demurrage effect,
/// protected/non-monetary metadata may not be smuggled into a monetary effect,
/// and the account balance delta must match the exact typed allocation.
pub fn validate_account_balance_projection(
    effect: &EconomicEffectV1,
    predecessor: &AccountBalanceProjectionV1,
    successor: &AccountBalanceProjectionV1,
) -> Result<(), EconomicEffectError> {
    predecessor.validate()?;
    successor.validate()?;
    let validated = effect.validate(EffectValidationContext {
        seen_effect_identities: &[],
        seen_economic_identities: &[],
    })?;

    if predecessor.owner != successor.owner
        || predecessor.owner != effect.identity.source_owner
            && effect
                .debits
                .iter()
                .chain(effect.credits.iter())
                .all(|allocation| allocation.party_id != predecessor.owner)
    {
        return Err(EconomicEffectError::ProjectionOwnerMismatch);
    }

    if predecessor.protected_state != successor.protected_state {
        return Err(EconomicEffectError::HiddenNonMonetaryMutation);
    }

    match effect.identity.mutation_class {
        MutationClass::Demurrage => {
            if successor.demurrage_marker == predecessor.demurrage_marker {
                return Err(EconomicEffectError::DemurrageMarkerUnchanged);
            }
            if successor.demurrage_marker < predecessor.demurrage_marker {
                return Err(EconomicEffectError::DemurrageMarkerRegressed);
            }
        }
        _ => {
            if successor.demurrage_marker != predecessor.demurrage_marker {
                return Err(EconomicEffectError::HiddenDemurrageMutation);
            }
        }
    }

    let debit = effect.debits.iter().find(|allocation| {
        allocation.role == AllocationRole::Account && allocation.party_id == predecessor.owner
    });
    let credit = effect.credits.iter().find(|allocation| {
        allocation.role == AllocationRole::Account && allocation.party_id == predecessor.owner
    });

    if debit.is_some() && credit.is_some() {
        return Err(EconomicEffectError::ProjectionHasBothDebitAndCredit);
    }

    let expected_balance = if let Some(allocation) = debit {
        predecessor
            .balance
            .checked_sub(allocation.amount)
            .ok_or(EconomicEffectError::ProjectionUnderflow)?
    } else if let Some(allocation) = credit {
        predecessor
            .balance
            .checked_add(allocation.amount)
            .ok_or(EconomicEffectError::ProjectionOverflow)?
    } else {
        predecessor.balance
    };

    if successor.balance != expected_balance {
        return Err(EconomicEffectError::ProjectionDeltaMismatch);
    }

    let _ = validated;
    Ok(())
}

impl EconomicEffectV1 {
    /// Validate the effect against source-derived semantic facts.
    ///
    /// This closes the gap where a caller supplies the right economic identity
    /// key but changes the asset, actor, mutation class, or allocation payload.
    /// Allocation order is normalized for comparison; allocation multiplicity and
    /// duplicate roles/parties remain fail-closed via checked_total.
    pub fn validate_against_source_facts(
        &self,
        expected: &EconomicEffectSourceFactsV1,
        context: EffectValidationContext<'_>,
    ) -> Result<ValidatedEconomicEffect<'_>, EconomicEffectError> {
        expected.validate_shape()?;
        if self.identity.economic_identity != expected.economic_identity {
            return Err(EconomicEffectError::IdentityBindingMismatch);
        }
        if self.identity.predecessor_action_reference != expected.predecessor_action_reference
            || self.identity.cause_action_references != expected.cause_action_references
        {
            return Err(EconomicEffectError::ProvenanceBindingMismatch);
        }
        if self.identity.asset != expected.asset
            || self.identity.mutation_class != expected.mutation_class
            || self.identity.source_owner != expected.source_owner
            || canonical_allocations(&self.debits)? != canonical_allocations(&expected.debits)?
            || canonical_allocations(&self.credits)? != canonical_allocations(&expected.credits)?
        {
            return Err(EconomicEffectError::PayloadBindingMismatch);
        }
        self.validate(context)
    }
    /// Validate the effect and additionally bind it to an identity derived by
    /// the source-specific authority theorem.
    ///
    /// The expected identity must be constructed from authenticated domain
    /// evidence outside this generic kernel. For collateral issuance, this is
    /// the canonical FIN-SAFE-010 mint_id; for transfers, the canonical
    /// transfer ID derived by the value-note theorem.
    pub fn validate_against_economic_identity(
        &self,
        expected: &EconomicIdentityV1,
        context: EffectValidationContext<'_>,
    ) -> Result<ValidatedEconomicEffect<'_>, EconomicEffectError> {
        if &self.identity.economic_identity != expected {
            return Err(EconomicEffectError::IdentityBindingMismatch);
        }
        self.validate(context)
    }

    pub fn validate(
        &self,
        context: EffectValidationContext<'_>,
    ) -> Result<ValidatedEconomicEffect<'_>, EconomicEffectError> {
        if self.schema_version != ECONOMIC_EFFECT_V1_SCHEMA_VERSION {
            return Err(EconomicEffectError::UnsupportedSchema);
        }
        self.identity.validate()?;

        if context
            .seen_effect_identities
            .iter()
            .any(|identity| identity == &self.identity)
        {
            return Err(EconomicEffectError::EffectReplay);
        }
        if context
            .seen_economic_identities
            .iter()
            .any(|identity| identity == &self.identity.economic_identity)
        {
            return Err(EconomicEffectError::EconomicReplay);
        }

        let debit_total = checked_total(&self.debits)?;
        let credit_total = checked_total(&self.credits)?;

        if self.debits.is_empty() && self.credits.is_empty() {
            return Err(EconomicEffectError::EmptyEffect);
        }

        match self.identity.mutation_class {
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
                    if !matches!(
                        credit.role,
                        AllocationRole::Commons
                            | AllocationRole::Treasury
                            | AllocationRole::BurnSink
                    ) {
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
            && self.debits[0].party_id == self.identity.source_owner
        {
            Ok(())
        } else {
            Err(EconomicEffectError::UnauthorizedSource)
        }
    }

    fn require_distinct_counterparty(&self) -> Result<(), EconomicEffectError> {
        if self.credits.len() == 1
            && self.credits[0].role == AllocationRole::Account
            && self.credits[0].party_id != self.identity.source_owner
        {
            Ok(())
        } else {
            Err(EconomicEffectError::SelfTransferOrInvalidCounterparty)
        }
    }

    pub fn expected_total(&self) -> Result<u64, EconomicEffectError> {
        let debit_total = checked_total(&self.debits)?;
        let credit_total = checked_total(&self.credits)?;

        if self.identity.mutation_class.requires_exact_conservation() {
            require_equal_totals(debit_total, credit_total)?;
        }
        Ok(credit_total)
    }
}

/// Authenticated source facts that a domain-specific adapter may derive from
/// a validated source theorem before admitting a generic economic effect.
///
/// This is deliberately separate from physical predecessor/cause references.
/// A source adapter is responsible for constructing these facts from its own
/// authenticated evidence; the generic kernel only checks exact semantic binding.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct EconomicEffectSourceFactsV1 {
    pub economic_identity: EconomicIdentityV1,
    pub predecessor_action_reference: String,
    pub cause_action_references: Vec<String>,
    pub asset: String,
    pub mutation_class: MutationClass,
    pub source_owner: String,
    pub debits: Vec<EconomicAllocation>,
    pub credits: Vec<EconomicAllocation>,
}

impl EconomicEffectSourceFactsV1 {
    pub fn validate_shape(&self) -> Result<(), EconomicEffectError> {
        validate_id(
            &self.economic_identity.key,
            EconomicEffectError::InvalidEconomicIdentity,
        )?;
        validate_id(
            &self.predecessor_action_reference,
            EconomicEffectError::InvalidPredecessorReference,
        )?;
        if self.cause_action_references.is_empty()
            || self.cause_action_references.len() > MAX_CAUSE_REFERENCES
        {
            return Err(EconomicEffectError::InvalidCauseReferences);
        }
        let mut seen_causes = BTreeSet::new();
        for cause in &self.cause_action_references {
            validate_id(cause, EconomicEffectError::InvalidCauseReference)?;
            if !seen_causes.insert(cause.as_str()) {
                return Err(EconomicEffectError::DuplicateCauseReference);
            }
        }
        validate_id(&self.asset, EconomicEffectError::InvalidAsset)?;
        validate_id(&self.source_owner, EconomicEffectError::InvalidSourceOwner)?;
        match (self.economic_identity.namespace, self.mutation_class) {
            (EconomicIdentityNamespaceV1::Transfer, MutationClass::Transfer)
            | (EconomicIdentityNamespaceV1::Fee, MutationClass::Fee)
            | (EconomicIdentityNamespaceV1::Demurrage, MutationClass::Demurrage)
            | (
                EconomicIdentityNamespaceV1::TreasuryAllocation,
                MutationClass::TreasuryAllocation,
            )
            | (EconomicIdentityNamespaceV1::Burn, MutationClass::Burn)
            | (EconomicIdentityNamespaceV1::CollateralMint, MutationClass::Mint)
            | (EconomicIdentityNamespaceV1::GovernanceMint, MutationClass::Mint)
            | (EconomicIdentityNamespaceV1::CollateralLock, MutationClass::CollateralLock)
            | (EconomicIdentityNamespaceV1::CollateralRelease, MutationClass::CollateralRelease)
            | (EconomicIdentityNamespaceV1::StakingReturn, MutationClass::StakingReturn)
            | (EconomicIdentityNamespaceV1::StakingSlash, MutationClass::StakingSlash)
            | (
                EconomicIdentityNamespaceV1::ReconciliationCorrection,
                MutationClass::ReconciliationCorrection,
            ) => {}
            _ => return Err(EconomicEffectError::IdentityNamespaceMismatch),
        }
        checked_total(&self.debits)?;
        checked_total(&self.credits)?;
        Ok(())
    }
}
#[derive(Clone, Copy, Debug)]
pub struct EffectValidationContext<'a> {
    pub seen_effect_identities: &'a [EconomicEffectIdentityV1],
    pub seen_economic_identities: &'a [EconomicIdentityV1],
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ValidatedEconomicEffect<'a> {
    pub effect: &'a EconomicEffectV1,
    pub debit_total: u64,
    pub credit_total: u64,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct EconomicEffectObservation {
    pub successor_action_reference: String,
    pub effect_identity: EconomicEffectIdentityV1,
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
        self.effect_identity.validate()?;
        if self.effect_identity.predecessor_action_reference == self.successor_action_reference {
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

    let mut unique_by_action: BTreeMap<String, EconomicEffectIdentityV1> = BTreeMap::new();
    for observation in observations {
        observation.validate_shape()?;
        if observation.effect_identity.predecessor_action_reference != predecessor_action_reference
        {
            return Err(EconomicEffectError::PredecessorReferenceMismatch);
        }
        match unique_by_action.get(&observation.successor_action_reference) {
            Some(existing) if existing == &observation.effect_identity => {}
            Some(_) => return Err(EconomicEffectError::InconsistentDuplicateAction),
            None => {
                unique_by_action.insert(
                    observation.successor_action_reference.clone(),
                    observation.effect_identity.clone(),
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

fn canonical_allocations(
    allocations: &[EconomicAllocation],
) -> Result<Vec<EconomicAllocation>, EconomicEffectError> {
    let mut canonical = allocations.to_vec();
    for allocation in &canonical {
        allocation.validate()?;
    }
    canonical.sort_by(|left, right| {
        (left.role, left.party_id.as_str()).cmp(&(right.role, right.party_id.as_str()))
    });
    Ok(canonical)
}
fn checked_total(allocations: &[EconomicAllocation]) -> Result<u64, EconomicEffectError> {
    let mut total = 0_u64;
    let mut seen = BTreeSet::new();
    for allocation in allocations {
        allocation.validate()?;
        let identity = (allocation.role as u8, allocation.party_id.as_str());
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

fn require_equal_totals(debit_total: u64, credit_total: u64) -> Result<(), EconomicEffectError> {
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
    InvalidEconomicIdentity,
    IdentityNamespaceMismatch,
    IdentityBindingMismatch,
    InvalidPredecessorReference,
    InvalidCauseReference,
    InvalidCauseReferences,
    DuplicateCauseReference,
    InvalidSuccessorReference,
    InvalidAsset,
    InvalidSourceOwner,
    InvalidProtectedState,
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
    EconomicReplay,
    PayloadBindingMismatch,
    ProvenanceBindingMismatch,
    PredecessorReferenceMismatch,
    SelfReferentialSuccessor,
    InconsistentDuplicateAction,
    ProjectionOwnerMismatch,
    HiddenNonMonetaryMutation,
    HiddenDemurrageMutation,
    DemurrageMarkerUnchanged,
    DemurrageMarkerRegressed,
    ProjectionHasBothDebitAndCredit,
    ProjectionUnderflow,
    ProjectionOverflow,
    ProjectionDeltaMismatch,
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
            identity: EconomicEffectIdentityV1 {
                economic_identity: EconomicIdentityV1 {
                    namespace: EconomicIdentityNamespaceV1::Transfer,
                    key: "transfer-1".into(),
                },
                predecessor_action_reference: "prev-1".into(),
                cause_action_references: vec!["cause-1".into()],
                asset: "SAP".into(),
                mutation_class: MutationClass::Transfer,
                source_owner: "alice".into(),
            },
            debits: vec![allocation("alice", AllocationRole::Account, 100)],
            credits: vec![allocation("bob", AllocationRole::Account, 100)],
        }
    }

    fn validate(
        effect: &EconomicEffectV1,
    ) -> Result<ValidatedEconomicEffect<'_>, EconomicEffectError> {
        effect.validate(EffectValidationContext {
            seen_effect_identities: &[],
                        seen_economic_identities: &[],
        })
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
        effect
            .credits
            .push(allocation("bob-fee", AllocationRole::Treasury, 50));
        assert_eq!(
            validate(&effect),
            Err(EconomicEffectError::WrongLegStructure)
        );

        effect.identity.mutation_class = MutationClass::Fee;
        effect.identity.economic_identity.namespace = EconomicIdentityNamespaceV1::Fee;
        effect.credits = vec![allocation("treasury", AllocationRole::Treasury, 100)];
        effect.debits[0].amount = 50;
        assert_eq!(
            validate(&effect),
            Err(EconomicEffectError::InsufficientDebit {
                debit_total: 50,
                credit_total: 100
            })
        );
        effect.debits[0].amount = 100;
        assert!(validate(&effect).is_ok());
    }

    #[test]
    fn self_burn_requires_typed_burn_sink() {
        let mut effect = base_transfer();
        effect.identity.mutation_class = MutationClass::Burn;
        effect.identity.economic_identity.namespace = EconomicIdentityNamespaceV1::Burn;
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
                seen_effect_identities: &[effect.identity.clone()],
                seen_economic_identities: &[],
            }),
            Err(EconomicEffectError::EffectReplay)
        );
    }

    #[test]
    fn same_economic_identity_is_rejected_even_when_physical_provenance_differs() {
        let mut effect = base_transfer();
        let seen = vec![EconomicIdentityV1 {
            namespace: EconomicIdentityNamespaceV1::Transfer,
            key: "transfer-1".into(),
        }];
        effect.identity.predecessor_action_reference = "different-prev".into();
        effect.identity.cause_action_references = vec!["different-cause".into()];
        assert_eq!(
            effect.validate(EffectValidationContext {
                seen_effect_identities: &[],
                seen_economic_identities: &seen,
            }),
            Err(EconomicEffectError::EconomicReplay)
        );
    }

    #[test]
    fn same_cause_can_fan_out_when_source_allows_multiple_effects() {
        let transfer = base_transfer();
        let mut fee = base_transfer();
        fee.identity.economic_identity.namespace = EconomicIdentityNamespaceV1::Fee;
        fee.identity.economic_identity.key = "fee-1".into();
        fee.identity.mutation_class = MutationClass::Fee;
        fee.credits = vec![allocation("treasury", AllocationRole::Treasury, 100)];

        assert!(transfer
            .validate(EffectValidationContext {
                seen_effect_identities: &[],
                seen_economic_identities: &[],
            })
            .is_ok());
        assert!(fee
            .validate(EffectValidationContext {
                seen_effect_identities: &[],
                seen_economic_identities: &[],
            })
            .is_ok());
    }

    #[test]
    fn multiple_causes_are_representable_without_implying_one_to_one_causality() {
        let mut effect = base_transfer();
        effect.identity.cause_action_references =
            vec!["cause-input-a".into(), "cause-input-b".into()];
        assert!(validate(&effect).is_ok());

        effect.identity.cause_action_references = vec![
            "cause-input-a".into(),
            "cause-input-a".into(),
        ];
        assert_eq!(
            validate(&effect),
            Err(EconomicEffectError::DuplicateCauseReference)
        );

        effect.identity.cause_action_references = Vec::new();
        assert_eq!(
            validate(&effect),
            Err(EconomicEffectError::InvalidCauseReferences)
        );

        effect.identity.cause_action_references = (0..=MAX_CAUSE_REFERENCES)
            .map(|index| format!("cause-{index}"))
            .collect();
        assert_eq!(
            validate(&effect),
            Err(EconomicEffectError::InvalidCauseReferences)
        );
    }

    #[test]
    fn source_facts_binding_accepts_exact_semantic_payload() {
        let effect = base_transfer();
        let expected = EconomicEffectSourceFactsV1 {
            economic_identity: effect.identity.economic_identity.clone(),
            predecessor_action_reference: effect.identity.predecessor_action_reference.clone(),
            cause_action_references: effect.identity.cause_action_references.clone(),
            asset: effect.identity.asset.clone(),
            mutation_class: effect.identity.mutation_class,
            source_owner: effect.identity.source_owner.clone(),
            debits: effect.debits.clone(),
            credits: effect.credits.clone(),
        };
        assert!(effect
            .validate_against_source_facts(
                &expected,
                EffectValidationContext {
                    seen_effect_identities: &[],
                    seen_economic_identities: &[],
                },
            )
            .is_ok());
    }

    #[test]
    fn source_facts_binding_rejects_payload_substitution_even_with_correct_identity() {
        let effect = base_transfer();
        let mut expected = EconomicEffectSourceFactsV1 {
            economic_identity: effect.identity.economic_identity.clone(),
            predecessor_action_reference: effect.identity.predecessor_action_reference.clone(),
            cause_action_references: effect.identity.cause_action_references.clone(),
            asset: effect.identity.asset.clone(),
            mutation_class: effect.identity.mutation_class,
            source_owner: effect.identity.source_owner.clone(),
            debits: effect.debits.clone(),
            credits: effect.credits.clone(),
        };
        expected.credits[0].amount = 99;

        assert_eq!(
            effect.validate_against_source_facts(
                &expected,
                EffectValidationContext {
                    seen_effect_identities: &[],
                    seen_economic_identities: &[],
                },
            ),
            Err(EconomicEffectError::PayloadBindingMismatch)
        );
    }

    #[test]
    fn source_facts_binding_rejects_actor_asset_and_class_substitution() {
        let effect = base_transfer();
        let mut expected = EconomicEffectSourceFactsV1 {
            economic_identity: effect.identity.economic_identity.clone(),
            predecessor_action_reference: effect.identity.predecessor_action_reference.clone(),
            cause_action_references: effect.identity.cause_action_references.clone(),
            asset: effect.identity.asset.clone(),
            mutation_class: effect.identity.mutation_class,
            source_owner: effect.identity.source_owner.clone(),
            debits: effect.debits.clone(),
            credits: effect.credits.clone(),
        };

        expected.asset = "OTHER".into();
        assert_eq!(
            effect.validate_against_source_facts(
                &expected,
                EffectValidationContext {
                    seen_effect_identities: &[],
                    seen_economic_identities: &[],
                },
            ),
            Err(EconomicEffectError::PayloadBindingMismatch)
        );

        expected.asset = effect.identity.asset.clone();
        expected.source_owner = "did:mycelix:mallory".into();
        assert_eq!(
            effect.validate_against_source_facts(
                &expected,
                EffectValidationContext {
                    seen_effect_identities: &[],
                    seen_economic_identities: &[],
                },
            ),
            Err(EconomicEffectError::PayloadBindingMismatch)
        );
    }
    #[test]
    fn source_facts_binding_rejects_provenance_substitution() {
        let effect = base_transfer();
        let mut expected = EconomicEffectSourceFactsV1 {
            economic_identity: effect.identity.economic_identity.clone(),
            predecessor_action_reference: effect.identity.predecessor_action_reference.clone(),
            cause_action_references: effect.identity.cause_action_references.clone(),
            asset: effect.identity.asset.clone(),
            mutation_class: effect.identity.mutation_class,
            source_owner: effect.identity.source_owner.clone(),
            debits: effect.debits.clone(),
            credits: effect.credits.clone(),
        };

        expected.predecessor_action_reference = "prev-2".into();
        assert_eq!(
            effect.validate_against_source_facts(
                &expected,
                EffectValidationContext {
                    seen_effect_identities: &[],
                    seen_economic_identities: &[],
                },
            ),
            Err(EconomicEffectError::ProvenanceBindingMismatch)
        );

        expected.predecessor_action_reference =
            effect.identity.predecessor_action_reference.clone();
        expected.cause_action_references = vec!["cause-2".into()];
        assert_eq!(
            effect.validate_against_source_facts(
                &expected,
                EffectValidationContext {
                    seen_effect_identities: &[],
                    seen_economic_identities: &[],
                },
            ),
            Err(EconomicEffectError::ProvenanceBindingMismatch)
        );
    }

    #[test]
    fn source_derived_identity_binding_is_explicit() {
        let effect = base_transfer();
        let expected = EconomicIdentityV1 {
            namespace: EconomicIdentityNamespaceV1::Transfer,
            key: "transfer-1".into(),
        };
        assert!(effect
            .validate_against_economic_identity(
                &expected,
                EffectValidationContext {
                    seen_effect_identities: &[],
                    seen_economic_identities: &[],
                },
            )
            .is_ok());

        let wrong = EconomicIdentityV1 {
            namespace: EconomicIdentityNamespaceV1::Transfer,
            key: "transfer-2".into(),
        };
        assert_eq!(
            effect.validate_against_economic_identity(
                &wrong,
                EffectValidationContext {
                    seen_effect_identities: &[],
                    seen_economic_identities: &[],
                },
            ),
            Err(EconomicEffectError::IdentityBindingMismatch)
        );
    }

    #[test]
    fn economic_identity_namespace_prevents_cross_class_key_collision() {
        let transfer = EconomicIdentityV1 {
            namespace: EconomicIdentityNamespaceV1::Transfer,
            key: "same-key".into(),
        };
        let fee = EconomicIdentityV1 {
            namespace: EconomicIdentityNamespaceV1::Fee,
            key: "same-key".into(),
        };

        let mut effect = base_transfer();
        effect.identity.economic_identity = transfer.clone();

        assert!(effect
            .validate(EffectValidationContext {
                seen_effect_identities: &[],
                seen_economic_identities: &[fee],
            })
            .is_ok());

        assert_eq!(
            effect.validate(EffectValidationContext {
                seen_effect_identities: &[],
                seen_economic_identities: &[transfer],
            }),
            Err(EconomicEffectError::EconomicReplay)
        );
    }

    #[test]
    fn namespace_must_match_mutation_class() {
        let mut effect = base_transfer();
        effect.identity.economic_identity.namespace = EconomicIdentityNamespaceV1::Fee;
        assert_eq!(
            validate(&effect),
            Err(EconomicEffectError::IdentityNamespaceMismatch)
        );
    }

    #[test]
    fn mint_is_explicitly_typed_not_an_unbalanced_transfer() {
        let mut effect = base_transfer();
        effect.identity.mutation_class = MutationClass::Mint;
        effect.identity.economic_identity.namespace = EconomicIdentityNamespaceV1::CollateralMint;
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
        effect.identity.mutation_class = MutationClass::Demurrage;
        effect.identity.economic_identity.namespace = EconomicIdentityNamespaceV1::Demurrage;
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
    fn transfer_projection_rejects_hidden_demurrage() {
        let effect = base_transfer();
        let predecessor = AccountBalanceProjectionV1 {
            owner: "alice".into(),
            balance: 100,
            demurrage_marker: 10,
            protected_state: None,
        };
        let successor = AccountBalanceProjectionV1 {
            owner: "alice".into(),
            balance: 0,
            demurrage_marker: 11,
            protected_state: None,
        };
        assert_eq!(
            validate_account_balance_projection(&effect, &predecessor, &successor),
            Err(EconomicEffectError::HiddenDemurrageMutation)
        );
    }

    #[test]
    fn projection_rejects_hidden_protected_state_mutation() {
        let effect = base_transfer();
        let predecessor = AccountBalanceProjectionV1 {
            owner: "alice".into(),
            balance: 100,
            demurrage_marker: 10,
            protected_state: None,
        };
        let successor = AccountBalanceProjectionV1 {
            owner: "alice".into(),
            balance: 0,
            demurrage_marker: 10,
            protected_state: Some("amber".into()),
        };
        assert_eq!(
            validate_account_balance_projection(&effect, &predecessor, &successor),
            Err(EconomicEffectError::HiddenNonMonetaryMutation)
        );
    }

    #[test]
    fn exact_source_and_recipient_projections_match_transfer() {
        let effect = base_transfer();
        let source_before = AccountBalanceProjectionV1 {
            owner: "alice".into(),
            balance: 100,
            demurrage_marker: 10,
            protected_state: None,
        };
        let source_after = AccountBalanceProjectionV1 {
            owner: "alice".into(),
            balance: 0,
            demurrage_marker: 10,
            protected_state: None,
        };
        assert!(
            validate_account_balance_projection(&effect, &source_before, &source_after).is_ok()
        );

        let recipient_before = AccountBalanceProjectionV1 {
            owner: "bob".into(),
            balance: 20,
            demurrage_marker: 10,
            protected_state: None,
        };
        let recipient_after = AccountBalanceProjectionV1 {
            owner: "bob".into(),
            balance: 120,
            demurrage_marker: 10,
            protected_state: None,
        };
        assert!(
            validate_account_balance_projection(&effect, &recipient_before, &recipient_after)
                .is_ok()
        );
    }

    #[test]
    fn demurrage_projection_requires_distinct_forward_marker() {
        let mut effect = base_transfer();
        effect.identity.mutation_class = MutationClass::Demurrage;
        effect.credits = vec![allocation("commons", AllocationRole::Commons, 100)];
        assert!(validate_account_balance_projection(
            &effect,
            &AccountBalanceProjectionV1 {
                owner: "alice".into(),
                balance: 100,
                demurrage_marker: 10,
                protected_state: None,
            },
            &AccountBalanceProjectionV1 {
                owner: "alice".into(),
                balance: 0,
                demurrage_marker: 11,
                protected_state: None,
            }
        )
        .is_ok());

        let unchanged = AccountBalanceProjectionV1 {
            owner: "alice".into(),
            balance: 0,
            demurrage_marker: 10,
            protected_state: None,
        };
        assert_eq!(
            validate_account_balance_projection(
                &effect,
                &AccountBalanceProjectionV1 {
                    owner: "alice".into(),
                    balance: 100,
                    demurrage_marker: 10,
                    protected_state: None,
                },
                &unchanged,
            ),
            Err(EconomicEffectError::DemurrageMarkerUnchanged)
        );
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
                successor_action_reference: "succ-a".into(),
                effect_identity: base_transfer().identity.clone(),
            },
            EconomicEffectObservation {
                successor_action_reference: "succ-b".into(),
                effect_identity: EconomicEffectIdentityV1 {
                    economic_identity: EconomicIdentityV1 {
                        namespace: EconomicIdentityNamespaceV1::Transfer,
                        key: "transfer-b".into(),
                    },
                    predecessor_action_reference: "prev-1".into(),
                    cause_action_references: vec!["cause-b".into()],
                    asset: "SAP".into(),
                    mutation_class: MutationClass::Transfer,
                    source_owner: "alice".into(),
                },
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
                successor_action_reference: "succ-a".into(),
                effect_identity: base_transfer().identity.clone(),
            },
            EconomicEffectObservation {
                successor_action_reference: "succ-a".into(),
                effect_identity: base_transfer().identity.clone(),
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
                effect_identity: base_transfer().identity.clone(),
            },
            EconomicEffectObservation {
                successor_action_reference: "succ-a".into(),
                successor_action_reference: "succ-a".into(),
                effect_identity: EconomicEffectIdentityV1 {
                    namespace: EconomicIdentityNamespaceV1::Transfer,
                    key: "transfer-b".into(),
                    predecessor_action_reference: "prev-1".into(),
                    cause_action_references: vec!["cause-b".into()],
                    asset: "SAP".into(),
                    mutation_class: MutationClass::Transfer,
                    source_owner: "alice".into(),
                },
            },
        ];
        assert_eq!(
            classify_successors("prev-1", &inconsistent),
            Err(EconomicEffectError::InconsistentDuplicateAction)
        );
    }
}
