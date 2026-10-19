// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Reference state machine for constitutional authorization consumption/finality.
//!
//! This crate is deliberately transport-neutral. It does not perform Holochain
//! host calls, signatures, witness discovery, consensus, or real-world effects.
//! It defines the safety semantics those runtimes must preserve.

use constitutional_envelope::MatterId;
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};

#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum FinalityProfile {
    LocalIdempotent,
    DetectionOnly,
    WitnessedSingleSpend,
    StrongConsensus,
}

impl FinalityProfile {
    fn rank(self) -> u8 {
        match self {
            Self::LocalIdempotent => 0,
            Self::DetectionOnly => 1,
            Self::WitnessedSingleSpend => 2,
            Self::StrongConsensus => 3,
        }
    }
}

#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum EvidenceAvailability {
    Complete,
    Indeterminate,
}

/// Defines the revocation boundary for a finalized-but-not-yet-applied action.
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum RevocationCutoff {
    /// Finality is the irrevocable commit point. A later revocation stops future
    /// uses but does not cancel the already-finalized effect.
    Finality,
    /// Authority must still be unrevoked at the effect's authenticated logical
    /// sequence. Suitable for controls where finality reserves a use but should
    /// not force a delayed physical/administrative action after revocation.
    Effect,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct FinalityRequirement {
    pub minimum_profile: FinalityProfile,
    pub min_witnesses: u16,
    pub min_distinct_domains: u16,
    pub revocation_cutoff: RevocationCutoff,
}

impl FinalityRequirement {
    pub fn validate(&self) -> Result<(), ConsumptionError> {
        if self.min_distinct_domains > self.min_witnesses {
            return Err(ConsumptionError::InvalidFinalityRequirement);
        }
        if self.minimum_profile.rank() >= FinalityProfile::WitnessedSingleSpend.rank()
            && (self.min_witnesses == 0 || self.min_distinct_domains == 0)
        {
            return Err(ConsumptionError::InvalidFinalityRequirement);
        }
        Ok(())
    }
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct UsageBudget {
    /// Stable identity shared across parent/child delegated authority. Delegation
    /// must not mint a fresh budget unless the constitutional source explicitly
    /// created a new independent authorization.
    pub budget_id: String,
    pub max_uses: u32,
}

impl UsageBudget {
    pub fn validate(&self) -> Result<(), ConsumptionError> {
        if self.budget_id.trim().is_empty() {
            return Err(ConsumptionError::EmptyBudgetId);
        }
        if self.max_uses == 0 {
            return Err(ConsumptionError::InvalidMaxUses);
        }
        Ok(())
    }
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq, Hash)]
pub struct ConsumptionKey {
    /// Digest/reference for the exact canonical AuthorizationEnvelope.
    pub envelope_digest: String,
    pub nonce: String,
    pub use_index: u32,
    /// Runtime/finality domain in which the use is consumed. This prevents a
    /// caller from replaying the same authority into another domain unnoticed.
    pub jurisdiction: String,
}

impl ConsumptionKey {
    pub fn validate(&self) -> Result<(), ConsumptionError> {
        if self.envelope_digest.trim().is_empty() {
            return Err(ConsumptionError::EmptyEnvelopeDigest);
        }
        if self.nonce.trim().is_empty() {
            return Err(ConsumptionError::EmptyNonce);
        }
        if self.jurisdiction.trim().is_empty() {
            return Err(ConsumptionError::EmptyJurisdiction);
        }
        Ok(())
    }
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct ConsumptionClaim {
    pub claim_id: String,
    pub key: ConsumptionKey,
    pub matter: MatterId,
    /// Commitment to the exact target/resource authorized by the envelope.
    pub target_digest: String,
    /// Commitment to the exact effect payload/parameters.
    pub payload_digest: String,
    /// Must match the shared usage budget for this authority lineage.
    pub budget_id: String,
}

impl ConsumptionClaim {
    pub fn validate(&self) -> Result<(), ConsumptionError> {
        if self.claim_id.trim().is_empty() {
            return Err(ConsumptionError::EmptyClaimId);
        }
        self.key.validate()?;
        self.matter
            .validate()
            .map_err(|_| ConsumptionError::InvalidMatter)?;
        if self.target_digest.trim().is_empty() {
            return Err(ConsumptionError::EmptyTargetDigest);
        }
        if self.payload_digest.trim().is_empty() {
            return Err(ConsumptionError::EmptyPayloadDigest);
        }
        if self.budget_id.trim().is_empty() {
            return Err(ConsumptionError::EmptyBudgetId);
        }
        Ok(())
    }
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct WitnessAttestation {
    pub witness_id: String,
    /// Independence domain derived from authenticated constitutional/finality
    /// policy, not a self-asserted label supplied by the witness.
    pub domain_id: String,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct FinalityProof {
    pub proof_id: String,
    pub claim_id: String,
    pub profile: FinalityProfile,
    /// Monotonic logical/event ordering reference. This is intentionally not a
    /// wall-clock timestamp.
    pub finalized_at_seq: u64,
    pub dependency_state: EvidenceAvailability,
    pub witnesses: Vec<WitnessAttestation>,
    /// Required when the profile relies on an external/strong-consensus system.
    pub consensus_ref: Option<String>,
}

impl FinalityProof {
    pub fn validate_against(
        &self,
        claim: &ConsumptionClaim,
        requirement: &FinalityRequirement,
    ) -> Result<(), ConsumptionError> {
        if self.proof_id.trim().is_empty() {
            return Err(ConsumptionError::EmptyFinalityProofId);
        }
        if self.claim_id != claim.claim_id {
            return Err(ConsumptionError::ProofClaimMismatch);
        }
        if self.dependency_state != EvidenceAvailability::Complete {
            return Err(ConsumptionError::IndeterminateDependencies);
        }
        if self.profile.rank() < requirement.minimum_profile.rank() {
            return Err(ConsumptionError::InsufficientFinalityProfile);
        }

        let mut witness_ids = BTreeSet::new();
        let mut domains = BTreeSet::new();
        for witness in &self.witnesses {
            if witness.witness_id.trim().is_empty() || witness.domain_id.trim().is_empty() {
                return Err(ConsumptionError::InvalidWitness);
            }
            witness_ids.insert(witness.witness_id.as_str());
            domains.insert(witness.domain_id.as_str());
        }
        if witness_ids.len() < requirement.min_witnesses as usize {
            return Err(ConsumptionError::InsufficientWitnesses);
        }
        if domains.len() < requirement.min_distinct_domains as usize {
            return Err(ConsumptionError::InsufficientWitnessDomains);
        }
        if self.profile == FinalityProfile::StrongConsensus
            && self
                .consensus_ref
                .as_ref()
                .map(|v| v.trim().is_empty())
                .unwrap_or(true)
        {
            return Err(ConsumptionError::MissingConsensusReference);
        }
        Ok(())
    }
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct FinalizedUse {
    pub claim: ConsumptionClaim,
    pub proof: FinalityProof,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct EffectRecord {
    pub claim_id: String,
    pub applied_at_seq: u64,
    pub output_ref: String,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum IntegrityFault {
    /// A later-observed revocation proves an effective logical sequence at or
    /// before the commit point already accepted under the configured cutoff.
    /// For `Finality`, the commit point is finalization; for `Effect`, it is the
    /// already-applied side effect. Preserve history, halt new effects, review.
    LateEarlierRevocation {
        revocation_seq: u64,
        conflicting_use_index: u32,
        cutoff: RevocationCutoff,
        commit_at_seq: u64,
    },
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum SubmitOutcome {
    Pending,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum FinalizeOutcome {
    Finalized,
    AlreadyFinalized,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum EffectOutcome {
    Applied,
    AlreadyApplied { output_ref: String },
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum RevokeOutcome {
    Revoked,
    AlreadyRevoked,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum ConsumptionError {
    EmptyBudgetId,
    InvalidMaxUses,
    InvalidFinalityRequirement,
    EmptyEnvelopeDigest,
    EmptyNonce,
    EmptyJurisdiction,
    EmptyClaimId,
    InvalidMatter,
    EmptyTargetDigest,
    EmptyPayloadDigest,
    EmptyFinalityProofId,
    InvalidWitness,
    MissingConsensusReference,
    UseIndexOutOfRange,
    BudgetMismatch,
    DuplicateClaim,
    UnknownClaim,
    ProofClaimMismatch,
    InsufficientFinalityProfile,
    IndeterminateDependencies,
    InsufficientWitnesses,
    InsufficientWitnessDomains,
    AuthorizationRevokedBeforeFinality,
    AuthorizationRevokedBeforeEffect,
    ConflictingUseAlreadyFinalized,
    EffectBeforeFinality,
    EmptyOutputReference,
    IntegrityFaultActive,
    LateEarlierRevocationConflict,
    InvariantViolation,
}

/// Executable reference model for one authorization lineage/use budget.
///
/// Runtime implementations may distribute this state across Holochain, witness
/// services, or stronger consensus systems, but they must preserve the same
/// safety invariants.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct ConsumptionState {
    pub budget: UsageBudget,
    pub requirement: FinalityRequirement,
    /// Earliest accepted effective revocation in authenticated logical/event
    /// order. A contradictory earlier revocation raises `integrity_fault`.
    pub revoked_at_seq: Option<u64>,
    /// A detected contradiction halts future finality/effects for review.
    pub integrity_fault: Option<IntegrityFault>,
    /// Competing pending claims are intentionally retained as evidence. Safety
    /// comes from preventing more than one from finalizing per use index.
    pub pending: Vec<ConsumptionClaim>,
    pub finalized: BTreeMap<u32, FinalizedUse>,
    pub effects: BTreeMap<u32, EffectRecord>,
}

impl ConsumptionState {
    pub fn new(
        budget: UsageBudget,
        requirement: FinalityRequirement,
    ) -> Result<Self, ConsumptionError> {
        budget.validate()?;
        requirement.validate()?;
        Ok(Self {
            budget,
            requirement,
            revoked_at_seq: None,
            integrity_fault: None,
            pending: Vec::new(),
            finalized: BTreeMap::new(),
            effects: BTreeMap::new(),
        })
    }

    pub fn remaining_uses(&self) -> u32 {
        self.budget
            .max_uses
            .saturating_sub(self.finalized.len() as u32)
    }

    pub fn submit_claim(
        &mut self,
        claim: ConsumptionClaim,
    ) -> Result<SubmitOutcome, ConsumptionError> {
        if self.integrity_fault.is_some() {
            return Err(ConsumptionError::IntegrityFaultActive);
        }
        claim.validate()?;
        if claim.budget_id != self.budget.budget_id {
            return Err(ConsumptionError::BudgetMismatch);
        }
        if claim.key.use_index >= self.budget.max_uses {
            return Err(ConsumptionError::UseIndexOutOfRange);
        }
        if self.pending.iter().any(|c| c.claim_id == claim.claim_id) {
            return Err(ConsumptionError::DuplicateClaim);
        }
        if self.finalized.contains_key(&claim.key.use_index) {
            return Err(ConsumptionError::ConflictingUseAlreadyFinalized);
        }
        self.pending.push(claim);
        Ok(SubmitOutcome::Pending)
    }

    /// Record revocation in authenticated logical/event order.
    ///
    /// Contradiction is measured against the constitutional commit point chosen
    /// by `revocation_cutoff`: accepted finality for `Finality`, or an already
    /// applied effect for `Effect`. A finalized-but-not-yet-applied Effect-mode
    /// use may therefore be cancelled by newly learned earlier revocation
    /// evidence without rewriting history or raising an integrity fault.
    pub fn revoke(&mut self, effective_seq: u64) -> Result<RevokeOutcome, ConsumptionError> {
        if self.integrity_fault.is_some() {
            return Err(ConsumptionError::IntegrityFaultActive);
        }

        let conflict = match self.requirement.revocation_cutoff {
            RevocationCutoff::Finality => self.finalized.iter().find_map(|(use_index, record)| {
                (record.proof.finalized_at_seq >= effective_seq)
                    .then_some((*use_index, record.proof.finalized_at_seq))
            }),
            RevocationCutoff::Effect => self.effects.iter().find_map(|(use_index, record)| {
                (record.applied_at_seq >= effective_seq)
                    .then_some((*use_index, record.applied_at_seq))
            }),
        };

        if let Some((use_index, commit_at_seq)) = conflict {
            self.integrity_fault = Some(IntegrityFault::LateEarlierRevocation {
                revocation_seq: effective_seq,
                conflicting_use_index: use_index,
                cutoff: self.requirement.revocation_cutoff,
                commit_at_seq,
            });
            return Err(ConsumptionError::LateEarlierRevocationConflict);
        }

        match self.revoked_at_seq {
            Some(existing) if effective_seq >= existing => Ok(RevokeOutcome::AlreadyRevoked),
            Some(_) => {
                self.revoked_at_seq = Some(effective_seq);
                Ok(RevokeOutcome::Revoked)
            }
            None => {
                self.revoked_at_seq = Some(effective_seq);
                Ok(RevokeOutcome::Revoked)
            }
        }
    }

    pub fn finalize(
        &mut self,
        claim_id: &str,
        proof: FinalityProof,
    ) -> Result<FinalizeOutcome, ConsumptionError> {
        if self.integrity_fault.is_some() {
            return Err(ConsumptionError::IntegrityFaultActive);
        }

        if let Some(existing) = self
            .finalized
            .values()
            .find(|record| record.claim.claim_id == claim_id)
        {
            if existing.proof == proof {
                return Ok(FinalizeOutcome::AlreadyFinalized);
            }
        }

        let claim = self
            .pending
            .iter()
            .find(|claim| claim.claim_id == claim_id)
            .cloned()
            .ok_or(ConsumptionError::UnknownClaim)?;

        proof.validate_against(&claim, &self.requirement)?;

        if let Some(existing) = self.finalized.get(&claim.key.use_index) {
            if existing.claim.claim_id == claim.claim_id && existing.proof == proof {
                return Ok(FinalizeOutcome::AlreadyFinalized);
            }
            return Err(ConsumptionError::ConflictingUseAlreadyFinalized);
        }

        if self
            .revoked_at_seq
            .map(|revoked_at| revoked_at <= proof.finalized_at_seq)
            .unwrap_or(false)
        {
            return Err(ConsumptionError::AuthorizationRevokedBeforeFinality);
        }

        self.finalized
            .insert(claim.key.use_index, FinalizedUse { claim, proof });
        self.check_invariants()?;
        Ok(FinalizeOutcome::Finalized)
    }

    /// Apply the real-world/public side effect only after required finality.
    /// Duplicate delivery of the same finalized use is idempotent and returns
    /// the previously recorded output instead of applying the effect again.
    pub fn apply_effect(
        &mut self,
        claim_id: &str,
        applied_at_seq: u64,
        output_ref: impl Into<String>,
    ) -> Result<EffectOutcome, ConsumptionError> {
        let (use_index, _) = self
            .finalized
            .iter()
            .find(|(_, record)| record.claim.claim_id == claim_id)
            .ok_or(ConsumptionError::EffectBeforeFinality)?;
        let use_index = *use_index;

        // Historical duplicate delivery remains a safe read even if a later
        // integrity fault/revocation now blocks new side effects.
        if let Some(existing) = self.effects.get(&use_index) {
            if existing.claim_id == claim_id {
                return Ok(EffectOutcome::AlreadyApplied {
                    output_ref: existing.output_ref.clone(),
                });
            }
            return Err(ConsumptionError::InvariantViolation);
        }

        if self.integrity_fault.is_some() {
            return Err(ConsumptionError::IntegrityFaultActive);
        }

        if self.requirement.revocation_cutoff == RevocationCutoff::Effect
            && self
                .revoked_at_seq
                .map(|revoked_at| revoked_at <= applied_at_seq)
                .unwrap_or(false)
        {
            return Err(ConsumptionError::AuthorizationRevokedBeforeEffect);
        }

        let output_ref = output_ref.into();
        if output_ref.trim().is_empty() {
            return Err(ConsumptionError::EmptyOutputReference);
        }
        self.effects.insert(
            use_index,
            EffectRecord {
                claim_id: claim_id.to_owned(),
                applied_at_seq,
                output_ref,
            },
        );
        self.check_invariants()?;
        Ok(EffectOutcome::Applied)
    }

    pub fn check_invariants(&self) -> Result<(), ConsumptionError> {
        if self.finalized.len() > self.budget.max_uses as usize {
            return Err(ConsumptionError::InvariantViolation);
        }

        for (use_index, record) in &self.finalized {
            if *use_index >= self.budget.max_uses
                || record.claim.key.use_index != *use_index
                || record.claim.budget_id != self.budget.budget_id
            {
                return Err(ConsumptionError::InvariantViolation);
            }
            if self.requirement.revocation_cutoff == RevocationCutoff::Finality
                && self
                    .revoked_at_seq
                    .map(|revoked_at| revoked_at <= record.proof.finalized_at_seq)
                    .unwrap_or(false)
            {
                return Err(ConsumptionError::InvariantViolation);
            }
        }

        for (use_index, effect) in &self.effects {
            let finalized = self
                .finalized
                .get(use_index)
                .ok_or(ConsumptionError::InvariantViolation)?;
            if finalized.claim.claim_id != effect.claim_id {
                return Err(ConsumptionError::InvariantViolation);
            }
            if self.requirement.revocation_cutoff == RevocationCutoff::Effect
                && self
                    .revoked_at_seq
                    .map(|revoked_at| revoked_at <= effect.applied_at_seq)
                    .unwrap_or(false)
            {
                return Err(ConsumptionError::InvariantViolation);
            }
        }

        Ok(())
    }
}
