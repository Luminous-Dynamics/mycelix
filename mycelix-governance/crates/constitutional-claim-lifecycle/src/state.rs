use crate::{
    ClaimLifecycleError, ClaimLifecycleStatus, ClaimRecord, ClaimTransitionReason,
    ClaimTransitionReceipt, LifecycleFault, RevocationProvenance,
};
use constitutional_consumption::{
    ConsumptionClaim, ConsumptionError, ConsumptionState, FinalityProfile, FinalityRequirement,
    SubmitOutcome, UsageBudget,
};
use constitutional_temporal_provenance::TemporalEvidenceState;
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct ClaimLifecycleState {
    pub(crate) temporal: TemporalEvidenceState,
    pub(crate) consumption: ConsumptionState,
    pub(crate) active_revocation: Option<RevocationProvenance>,
    pub(crate) halt_fault: Option<LifecycleFault>,
    pub(crate) claims: BTreeMap<String, ClaimRecord>,
    pub(crate) next_transition_ordinal: u64,
}

fn profile_rank(profile: FinalityProfile) -> u8 {
    match profile {
        FinalityProfile::LocalIdempotent => 0,
        FinalityProfile::DetectionOnly => 1,
        FinalityProfile::WitnessedSingleSpend => 2,
        FinalityProfile::StrongConsensus => 3,
    }
}

impl ClaimLifecycleState {
    pub fn new(
        temporal: TemporalEvidenceState,
        budget: UsageBudget,
        requirement: FinalityRequirement,
    ) -> Result<Self, ClaimLifecycleError> {
        if temporal.policy.domain_id.trim().is_empty() {
            return Err(ClaimLifecycleError::EmptyFinalityDomainId);
        }
        if profile_rank(temporal.policy.profile) < profile_rank(requirement.minimum_profile) {
            return Err(ClaimLifecycleError::TemporalProfileTooWeak);
        }
        if !Self::temporal_is_pristine(&temporal) {
            return Err(ClaimLifecycleError::NonPristineTemporalState);
        }

        let state = Self {
            temporal,
            consumption: ConsumptionState::new(budget, requirement)?,
            active_revocation: None,
            halt_fault: None,
            claims: BTreeMap::new(),
            next_transition_ordinal: 0,
        };
        state.check_invariants()?;
        Ok(state)
    }

    fn temporal_is_pristine(temporal: &TemporalEvidenceState) -> bool {
        temporal.last_observed_seq == 0
            && temporal.accepted_finality.is_empty()
            && temporal.rejected_finality.is_empty()
            && temporal.quarantined_finality.is_empty()
            && temporal.revocations.is_empty()
            && temporal.closures.is_empty()
            && temporal.latest_closure_id.is_none()
            && temporal.integrity_fault.is_none()
    }

    pub fn temporal(&self) -> &TemporalEvidenceState {
        &self.temporal
    }

    pub fn consumption(&self) -> &ConsumptionState {
        &self.consumption
    }

    pub fn active_revocation(&self) -> Option<&RevocationProvenance> {
        self.active_revocation.as_ref()
    }

    pub fn halt_fault(&self) -> Option<&LifecycleFault> {
        self.halt_fault.as_ref()
    }

    pub fn claims(&self) -> &BTreeMap<String, ClaimRecord> {
        &self.claims
    }

    pub fn finality_domain_id(&self) -> &str {
        &self.temporal.policy.domain_id
    }

    pub fn next_transition_ordinal(&self) -> u64 {
        self.next_transition_ordinal
    }

    pub fn remaining_uses(&self) -> u32 {
        self.consumption.remaining_uses()
    }

    pub fn claim(&self, claim_id: &str) -> Option<&ClaimRecord> {
        self.claims.get(claim_id)
    }

    pub fn validate_restored(&self) -> Result<(), ClaimLifecycleError> {
        self.check_invariants()
    }

    pub(crate) fn transaction<T>(
        &mut self,
        operation: impl FnOnce(&mut Self) -> Result<T, ClaimLifecycleError>,
    ) -> Result<T, ClaimLifecycleError> {
        let mut staged = self.clone();
        let result = operation(&mut staged)?;
        staged.check_invariants()?;
        *self = staged;
        Ok(result)
    }

    pub(crate) fn ensure_not_halted(&self) -> Result<(), ClaimLifecycleError> {
        if self.halt_fault.is_some()
            || self.consumption.integrity_fault.is_some()
            || self.temporal.integrity_fault.is_some()
        {
            return Err(ClaimLifecycleError::IntegrityFaultActive);
        }
        Ok(())
    }

    pub(crate) fn transition_claim(
        &mut self,
        claim_id: &str,
        to: ClaimLifecycleStatus,
        reason: ClaimTransitionReason,
    ) -> Result<bool, ClaimLifecycleError> {
        let current = self
            .claims
            .get(claim_id)
            .ok_or(ClaimLifecycleError::UnknownClaim)?
            .status
            .clone();

        if current == to {
            return Ok(false);
        }
        if current.is_terminal() {
            return Err(ClaimLifecycleError::TerminalClaim);
        }

        let ordinal = self
            .next_transition_ordinal
            .checked_add(1)
            .ok_or(ClaimLifecycleError::TransitionOrdinalOverflow)?;

        let record = self
            .claims
            .get_mut(claim_id)
            .ok_or(ClaimLifecycleError::UnknownClaim)?;
        let receipt = ClaimTransitionReceipt {
            ordinal,
            claim_id: record.claim.claim_id.clone(),
            use_index: record.claim.key.use_index,
            matter: record.claim.matter.clone(),
            envelope_digest: record.claim.key.envelope_digest.clone(),
            target_digest: record.claim.target_digest.clone(),
            payload_digest: record.claim.payload_digest.clone(),
            from: current,
            to: to.clone(),
            reason,
        };
        record.status = to;
        record.transitions.push(receipt);
        self.next_transition_ordinal = ordinal;
        Ok(true)
    }

    pub(crate) fn live_claim_ids(&self) -> Vec<String> {
        self.claims
            .iter()
            .filter(|(_, record)| record.status.is_live())
            .map(|(id, _)| id.clone())
            .collect()
    }

    pub(crate) fn halt_live_claims(
        &mut self,
        fault: LifecycleFault,
    ) -> Result<(), ClaimLifecycleError> {
        let effective_fault = match &self.halt_fault {
            Some(existing) => existing.clone(),
            None => {
                self.halt_fault = Some(fault.clone());
                fault
            }
        };
        for claim_id in self.live_claim_ids() {
            self.transition_claim(
                &claim_id,
                ClaimLifecycleStatus::IntegrityHalted {
                    fault: effective_fault.clone(),
                },
                ClaimTransitionReason::IntegrityFault,
            )?;
        }
        Ok(())
    }

    pub(crate) fn synchronize_temporal_fault_in_place(
        &mut self,
    ) -> Result<bool, ClaimLifecycleError> {
        let Some(fault) = self.temporal.integrity_fault.clone() else {
            return Ok(false);
        };
        let lifecycle_fault = LifecycleFault::Temporal {
            finality_domain_id: self.temporal.policy.domain_id.clone(),
            policy_version: self.temporal.policy.policy_version.clone(),
            fault,
        };
        let newly_halted = self.halt_fault.is_none();
        self.halt_live_claims(lifecycle_fault)?;
        Ok(newly_halted)
    }

    pub fn synchronize_temporal_fault(&mut self) -> Result<bool, ClaimLifecycleError> {
        self.transaction(|staged| staged.synchronize_temporal_fault_in_place())
    }

    pub fn submit_claim(
        &mut self,
        claim: ConsumptionClaim,
    ) -> Result<SubmitOutcome, ClaimLifecycleError> {
        self.transaction(|staged| staged.submit_claim_in_place(claim))
    }

    fn submit_claim_in_place(
        &mut self,
        claim: ConsumptionClaim,
    ) -> Result<SubmitOutcome, ClaimLifecycleError> {
        self.synchronize_temporal_fault_in_place()?;
        self.ensure_not_halted()?;
        if claim.key.jurisdiction != self.temporal.policy.domain_id {
            return Err(ClaimLifecycleError::ClaimDomainMismatch);
        }
        if self.claims.contains_key(&claim.claim_id) {
            return Err(ClaimLifecycleError::Consumption(
                ConsumptionError::DuplicateClaim,
            ));
        }

        let outcome = self.consumption.submit_claim(claim.clone())?;
        let status = match &self.active_revocation {
            Some(revocation) => ClaimLifecycleStatus::BlockedAwaitingEvidenceClosure {
                revocation: revocation.clone(),
            },
            None => ClaimLifecycleStatus::PendingExecutable,
        };
        self.claims.insert(
            claim.claim_id.clone(),
            ClaimRecord {
                claim,
                status,
                transitions: Vec::new(),
            },
        );
        Ok(outcome)
    }
}
