// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Temporal provenance and evidence-closure semantics for constitutional finality.
//!
//! This crate is intentionally transport-neutral. It does not authenticate
//! signatures, discover witnesses, query Holochain, or run consensus. Instead it
//! defines the ordering and closure invariants those runtimes must preserve.

use constitutional_consumption::{ClaimBinding, FinalityProfile};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};

#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum CrossOrderRelation {
    /// Effective and observed sequence values have been normalized into one
    /// authenticated constitutional order and may therefore be compared.
    SharedComparable,
    /// Effective order and verifier observation order are different domains.
    /// Their numeric values MUST NOT be compared to infer causality.
    Independent,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct ObservationOrderPolicy {
    /// Identity of the verifier/intake order domain.
    pub domain_id: String,
    pub relation: CrossOrderRelation,
}

impl ObservationOrderPolicy {
    pub fn shared_with(finality_domain_id: impl Into<String>) -> Self {
        Self {
            domain_id: finality_domain_id.into(),
            relation: CrossOrderRelation::SharedComparable,
        }
    }

    pub fn independent(observation_domain_id: impl Into<String>) -> Self {
        Self {
            domain_id: observation_domain_id.into(),
            relation: CrossOrderRelation::Independent,
        }
    }

    fn validate_against(&self, finality_domain_id: &str) -> Result<(), TemporalProvenanceError> {
        if self.domain_id.trim().is_empty() {
            return Err(TemporalProvenanceError::EmptyObservationDomainId);
        }
        if self.relation == CrossOrderRelation::SharedComparable
            && self.domain_id != finality_domain_id
        {
            return Err(TemporalProvenanceError::SharedOrderDomainMismatch);
        }
        Ok(())
    }
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct TemporalOrder {
    /// Finality/effective-order domain whose authenticated logical order is used.
    pub domain_id: String,
    /// Sequence at which the evidence became constitutionally effective.
    pub effective_seq: u64,
    /// Sequence at which this verifier observed / accepted the evidence. Its
    /// domain is fixed by the enclosing `ObservationOrderPolicy`.
    pub observed_seq: u64,
}

impl TemporalOrder {
    /// Shape-only validation. Whether the two numeric orders are comparable is
    /// a policy decision owned by `TemporalEvidenceState`.
    pub fn validate(&self) -> Result<(), TemporalProvenanceError> {
        if self.domain_id.trim().is_empty() {
            return Err(TemporalProvenanceError::EmptyDomainId);
        }
        if self.effective_seq == 0 || self.observed_seq == 0 {
            return Err(TemporalProvenanceError::ZeroSequence);
        }
        Ok(())
    }
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct FinalityEvidence {
    pub evidence_id: String,
    pub claim_id: String,
    pub proof_id: String,
    pub profile: FinalityProfile,
    pub order: TemporalOrder,
}

impl FinalityEvidence {
    pub fn validate(&self) -> Result<(), TemporalProvenanceError> {
        if self.evidence_id.trim().is_empty() {
            return Err(TemporalProvenanceError::EmptyEvidenceId);
        }
        if self.claim_id.trim().is_empty() {
            return Err(TemporalProvenanceError::EmptyClaimId);
        }
        if self.proof_id.trim().is_empty() {
            return Err(TemporalProvenanceError::EmptyProofReference);
        }
        self.order.validate()
    }
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct BoundFinalityEvidence {
    pub evidence: FinalityEvidence,
    pub claim_binding: ClaimBinding,
}

impl BoundFinalityEvidence {
    pub fn validate(&self) -> Result<(), TemporalProvenanceError> {
        self.evidence.validate()?;
        self.claim_binding
            .validate()
            .map_err(|_| TemporalProvenanceError::InvalidClaimBinding)?;
        if self.claim_binding.claim_id != self.evidence.claim_id {
            return Err(TemporalProvenanceError::ClaimBindingIdentityMismatch);
        }
        Ok(())
    }
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct RevocationEvidence {
    pub evidence_id: String,
    pub revocation_id: String,
    pub order: TemporalOrder,
}

impl RevocationEvidence {
    pub fn validate(&self) -> Result<(), TemporalProvenanceError> {
        if self.evidence_id.trim().is_empty() {
            return Err(TemporalProvenanceError::EmptyEvidenceId);
        }
        if self.revocation_id.trim().is_empty() {
            return Err(TemporalProvenanceError::EmptyRevocationId);
        }
        self.order.validate()
    }
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct ClosureWitness {
    pub witness_id: String,
    pub independence_domain: String,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum ClosureProof {
    LocalCheckpoint { checkpoint_ref: String },
    WitnessQuorum {
        proof_ref: String,
        witnesses: Vec<ClosureWitness>,
    },
    StrongConsensus {
        checkpoint_ref: String,
        namespace_commitment: String,
    },
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct EvidenceClosure {
    pub closure_id: String,
    pub domain_id: String,
    /// All admissible finality evidence in this effective-order domain with
    /// effective sequence <= this value is asserted complete by `proof`.
    pub closed_through_effective_seq: u64,
    /// Observation order in the enclosing verifier observation domain.
    pub observed_at_seq: u64,
    pub profile: FinalityProfile,
    pub policy_version: String,
    pub previous_closure_id: Option<String>,
    pub proof: ClosureProof,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct FinalityDomainPolicy {
    pub domain_id: String,
    pub profile: FinalityProfile,
    pub policy_version: String,
    pub min_closure_witnesses: u16,
    pub min_closure_independence_domains: u16,
}

impl FinalityDomainPolicy {
    pub fn validate(&self) -> Result<(), TemporalProvenanceError> {
        if self.domain_id.trim().is_empty() {
            return Err(TemporalProvenanceError::EmptyDomainId);
        }
        if self.policy_version.trim().is_empty() {
            return Err(TemporalProvenanceError::EmptyPolicyVersion);
        }
        if self.min_closure_independence_domains > self.min_closure_witnesses {
            return Err(TemporalProvenanceError::InvalidClosureThreshold);
        }
        if self.profile == FinalityProfile::WitnessedSingleSpend
            && (self.min_closure_witnesses == 0
                || self.min_closure_independence_domains == 0)
        {
            return Err(TemporalProvenanceError::InvalidClosureThreshold);
        }
        Ok(())
    }
}

impl EvidenceClosure {
    /// Validate profile/domain/proof shape. Cross-order comparability is checked
    /// by the enclosing `TemporalEvidenceState`, not here.
    pub fn validate_against(
        &self,
        policy: &FinalityDomainPolicy,
    ) -> Result<(), TemporalProvenanceError> {
        if self.closure_id.trim().is_empty() {
            return Err(TemporalProvenanceError::EmptyClosureId);
        }
        if self.domain_id.trim().is_empty() {
            return Err(TemporalProvenanceError::EmptyDomainId);
        }
        if self.policy_version.trim().is_empty() {
            return Err(TemporalProvenanceError::EmptyPolicyVersion);
        }
        if self.closed_through_effective_seq == 0 || self.observed_at_seq == 0 {
            return Err(TemporalProvenanceError::ZeroSequence);
        }
        if self.domain_id != policy.domain_id {
            return Err(TemporalProvenanceError::DomainMismatch);
        }
        if self.profile != policy.profile {
            return Err(TemporalProvenanceError::ProfileMismatch);
        }
        if self.policy_version != policy.policy_version {
            return Err(TemporalProvenanceError::PolicyVersionMismatch);
        }

        match (&self.profile, &self.proof) {
            (FinalityProfile::DetectionOnly, _) => {
                return Err(TemporalProvenanceError::ClosureUnsupportedForDetectionOnly)
            }
            (
                FinalityProfile::LocalIdempotent,
                ClosureProof::LocalCheckpoint { checkpoint_ref },
            ) => {
                if checkpoint_ref.trim().is_empty() {
                    return Err(TemporalProvenanceError::EmptyProofReference);
                }
            }
            (
                FinalityProfile::WitnessedSingleSpend,
                ClosureProof::WitnessQuorum {
                    proof_ref,
                    witnesses,
                },
            ) => {
                if proof_ref.trim().is_empty() {
                    return Err(TemporalProvenanceError::EmptyProofReference);
                }
                let mut witness_ids = BTreeSet::new();
                let mut domains = BTreeSet::new();
                for witness in witnesses {
                    if witness.witness_id.trim().is_empty()
                        || witness.independence_domain.trim().is_empty()
                    {
                        return Err(TemporalProvenanceError::InvalidClosureWitness);
                    }
                    witness_ids.insert(witness.witness_id.as_str());
                    domains.insert(witness.independence_domain.as_str());
                }
                if witness_ids.len() < policy.min_closure_witnesses as usize {
                    return Err(TemporalProvenanceError::InsufficientClosureWitnesses);
                }
                if domains.len() < policy.min_closure_independence_domains as usize {
                    return Err(TemporalProvenanceError::InsufficientClosureDomains);
                }
            }
            (
                FinalityProfile::StrongConsensus,
                ClosureProof::StrongConsensus {
                    checkpoint_ref,
                    namespace_commitment,
                },
            ) => {
                if checkpoint_ref.trim().is_empty() || namespace_commitment.trim().is_empty() {
                    return Err(TemporalProvenanceError::EmptyProofReference);
                }
            }
            _ => return Err(TemporalProvenanceError::ClosureProofKindMismatch),
        }
        Ok(())
    }
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum TemporalIntegrityFault {
    FinalityAfterClosedWatermark {
        evidence_id: String,
        closure_id: String,
        evidence_effective_seq: u64,
        closed_through_effective_seq: u64,
    },
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum ObserveFinalityOutcome {
    Accepted,
    AlreadyObserved,
    RejectedKnownRevocation,
    QuarantinedContradiction,
    QuarantinedAfterFault,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum ObserveRevocationOutcome {
    Accepted,
    AlreadyObserved,
    AcceptedAfterFault,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum AcceptClosureOutcome {
    Accepted,
    AlreadyAccepted,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum TemporalProvenanceError {
    EmptyDomainId,
    EmptyObservationDomainId,
    EmptyEvidenceId,
    EmptyClaimId,
    InvalidClaimBinding,
    ClaimBindingIdentityMismatch,
    DuplicateFinalityBindingConflict,
    UnboundFinalityCannotBeUpgraded,
    EmptyRevocationId,
    EmptyClosureId,
    EmptyPolicyVersion,
    EmptyProofReference,
    ZeroSequence,
    ObservationBeforeEffectiveOrder,
    ObservationOrderRegression,
    SharedOrderDomainMismatch,
    ProfileMismatch,
    PolicyVersionMismatch,
    DomainMismatch,
    ClosureUnsupportedForDetectionOnly,
    ClosureProofKindMismatch,
    InvalidClosureThreshold,
    InvalidClosureWitness,
    InsufficientClosureWitnesses,
    InsufficientClosureDomains,
    DuplicateEvidenceIdConflict,
    DuplicateRevocationIdConflict,
    DuplicateClosureIdConflict,
    ClosureRegression,
    ClosureChainMismatch,
    IntegrityFaultActive,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct TemporalEvidenceState {
    pub policy: FinalityDomainPolicy,
    pub observation_order: ObservationOrderPolicy,
    pub last_observed_seq: u64,
    pub accepted_finality: BTreeMap<String, FinalityEvidence>,
    pub rejected_finality: BTreeMap<String, FinalityEvidence>,
    pub quarantined_finality: BTreeMap<String, FinalityEvidence>,
    pub finality_bindings: BTreeMap<String, ClaimBinding>,
    pub revocations: BTreeMap<String, RevocationEvidence>,
    pub closures: BTreeMap<String, EvidenceClosure>,
    pub latest_closure_id: Option<String>,
    pub integrity_fault: Option<TemporalIntegrityFault>,
}

impl TemporalEvidenceState {
    /// Conservative default: callers assert that effective and observation
    /// sequence values share one normalized constitutional order.
    pub fn new(policy: FinalityDomainPolicy) -> Result<Self, TemporalProvenanceError> {
        let observation_order = ObservationOrderPolicy::shared_with(policy.domain_id.clone());
        Self::new_with_observation_order(policy, observation_order)
    }

    /// Explicit constructor for runtimes where effective and observed order are
    /// independent (for example raw external consensus height vs local intake).
    pub fn new_with_observation_order(
        policy: FinalityDomainPolicy,
        observation_order: ObservationOrderPolicy,
    ) -> Result<Self, TemporalProvenanceError> {
        policy.validate()?;
        observation_order.validate_against(&policy.domain_id)?;
        Ok(Self {
            policy,
            observation_order,
            last_observed_seq: 0,
            accepted_finality: BTreeMap::new(),
            rejected_finality: BTreeMap::new(),
            quarantined_finality: BTreeMap::new(),
            finality_bindings: BTreeMap::new(),
            revocations: BTreeMap::new(),
            closures: BTreeMap::new(),
            latest_closure_id: None,
            integrity_fault: None,
        })
    }

    pub fn latest_closure(&self) -> Option<&EvidenceClosure> {
        self.latest_closure_id
            .as_ref()
            .and_then(|id| self.closures.get(id))
    }

    fn existing_finality(&self, evidence_id: &str) -> Option<&FinalityEvidence> {
        self.accepted_finality
            .get(evidence_id)
            .or_else(|| self.rejected_finality.get(evidence_id))
            .or_else(|| self.quarantined_finality.get(evidence_id))
    }

    fn known_revocation_blocks(&self, effective_seq: u64) -> bool {
        self.revocations
            .values()
            .any(|revocation| revocation.order.effective_seq <= effective_seq)
    }

    fn validate_cross_order(
        &self,
        effective_seq: u64,
        observed_seq: u64,
    ) -> Result<(), TemporalProvenanceError> {
        if self.observation_order.relation == CrossOrderRelation::SharedComparable
            && observed_seq < effective_seq
        {
            return Err(TemporalProvenanceError::ObservationBeforeEffectiveOrder);
        }
        Ok(())
    }

    fn validate_evidence_order(&self, order: &TemporalOrder) -> Result<(), TemporalProvenanceError> {
        order.validate()?;
        if order.domain_id != self.policy.domain_id {
            return Err(TemporalProvenanceError::DomainMismatch);
        }
        self.validate_cross_order(order.effective_seq, order.observed_seq)
    }

    fn require_new_observation(&self, observed_seq: u64) -> Result<(), TemporalProvenanceError> {
        if observed_seq <= self.last_observed_seq {
            return Err(TemporalProvenanceError::ObservationOrderRegression);
        }
        Ok(())
    }

    pub fn observe_revocation(
        &mut self,
        evidence: RevocationEvidence,
    ) -> Result<ObserveRevocationOutcome, TemporalProvenanceError> {
        evidence.validate()?;
        self.validate_evidence_order(&evidence.order)?;
        if let Some(existing) = self.revocations.get(&evidence.evidence_id) {
            return if existing == &evidence {
                Ok(ObserveRevocationOutcome::AlreadyObserved)
            } else {
                Err(TemporalProvenanceError::DuplicateRevocationIdConflict)
            };
        }
        self.require_new_observation(evidence.order.observed_seq)?;

        self.last_observed_seq = evidence.order.observed_seq;
        let after_fault = self.integrity_fault.is_some();
        self.revocations
            .insert(evidence.evidence_id.clone(), evidence);
        Ok(if after_fault {
            ObserveRevocationOutcome::AcceptedAfterFault
        } else {
            ObserveRevocationOutcome::Accepted
        })
    }

    pub fn observe_bound_finality(
        &mut self,
        bound: BoundFinalityEvidence,
    ) -> Result<ObserveFinalityOutcome, TemporalProvenanceError> {
        bound.validate()?;
        let evidence_id = bound.evidence.evidence_id.clone();

        match self.finality_bindings.get(&evidence_id) {
            Some(existing) if existing != &bound.claim_binding => {
                return Err(TemporalProvenanceError::DuplicateFinalityBindingConflict);
            }
            Some(_) => {}
            None if self.existing_finality(&evidence_id).is_some() => {
                return Err(TemporalProvenanceError::UnboundFinalityCannotBeUpgraded);
            }
            None => {}
        }

        let binding = bound.claim_binding.clone();
        let outcome = self.observe_finality(bound.evidence)?;
        self.finality_bindings.entry(evidence_id).or_insert(binding);
        Ok(outcome)
    }

    pub fn finality_binding(&self, evidence_id: &str) -> Option<&ClaimBinding> {
        self.finality_bindings.get(evidence_id)
    }

    pub fn observe_finality(
        &mut self,
        evidence: FinalityEvidence,
    ) -> Result<ObserveFinalityOutcome, TemporalProvenanceError> {
        evidence.validate()?;
        self.validate_evidence_order(&evidence.order)?;
        if evidence.profile != self.policy.profile {
            return Err(TemporalProvenanceError::ProfileMismatch);
        }

        if let Some(existing) = self.existing_finality(&evidence.evidence_id) {
            return if existing == &evidence {
                Ok(ObserveFinalityOutcome::AlreadyObserved)
            } else {
                Err(TemporalProvenanceError::DuplicateEvidenceIdConflict)
            };
        }
        self.require_new_observation(evidence.order.observed_seq)?;

        self.last_observed_seq = evidence.order.observed_seq;

        if self.integrity_fault.is_some() {
            self.quarantined_finality
                .insert(evidence.evidence_id.clone(), evidence);
            return Ok(ObserveFinalityOutcome::QuarantinedAfterFault);
        }

        if self.known_revocation_blocks(evidence.order.effective_seq) {
            self.rejected_finality
                .insert(evidence.evidence_id.clone(), evidence);
            return Ok(ObserveFinalityOutcome::RejectedKnownRevocation);
        }

        if let Some(closure) = self.latest_closure().cloned() {
            if evidence.order.effective_seq <= closure.closed_through_effective_seq {
                self.integrity_fault = Some(
                    TemporalIntegrityFault::FinalityAfterClosedWatermark {
                        evidence_id: evidence.evidence_id.clone(),
                        closure_id: closure.closure_id,
                        evidence_effective_seq: evidence.order.effective_seq,
                        closed_through_effective_seq: closure.closed_through_effective_seq,
                    },
                );
                self.quarantined_finality
                    .insert(evidence.evidence_id.clone(), evidence);
                return Ok(ObserveFinalityOutcome::QuarantinedContradiction);
            }
        }

        self.accepted_finality
            .insert(evidence.evidence_id.clone(), evidence);
        Ok(ObserveFinalityOutcome::Accepted)
    }

    pub fn accept_closure(
        &mut self,
        closure: EvidenceClosure,
    ) -> Result<AcceptClosureOutcome, TemporalProvenanceError> {
        if self.integrity_fault.is_some() {
            return Err(TemporalProvenanceError::IntegrityFaultActive);
        }
        closure.validate_against(&self.policy)?;
        self.validate_cross_order(
            closure.closed_through_effective_seq,
            closure.observed_at_seq,
        )?;

        if let Some(existing) = self.closures.get(&closure.closure_id) {
            return if existing == &closure {
                Ok(AcceptClosureOutcome::AlreadyAccepted)
            } else {
                Err(TemporalProvenanceError::DuplicateClosureIdConflict)
            };
        }
        self.require_new_observation(closure.observed_at_seq)?;

        match self.latest_closure() {
            Some(previous) => {
                if closure.closed_through_effective_seq < previous.closed_through_effective_seq {
                    return Err(TemporalProvenanceError::ClosureRegression);
                }
                if closure.previous_closure_id.as_deref() != Some(previous.closure_id.as_str()) {
                    return Err(TemporalProvenanceError::ClosureChainMismatch);
                }
            }
            None => {
                if closure.previous_closure_id.is_some() {
                    return Err(TemporalProvenanceError::ClosureChainMismatch);
                }
            }
        }

        self.last_observed_seq = closure.observed_at_seq;
        self.latest_closure_id = Some(closure.closure_id.clone());
        self.closures.insert(closure.closure_id.clone(), closure);
        Ok(AcceptClosureOutcome::Accepted)
    }

    pub fn pre_revocation_interval_is_closed(&self, revocation_effective_seq: u64) -> bool {
        if revocation_effective_seq == 0 {
            return false;
        }
        if revocation_effective_seq == 1 {
            return true;
        }
        self.latest_closure()
            .map(|closure| closure.closed_through_effective_seq >= revocation_effective_seq - 1)
            .unwrap_or(false)
    }
}
