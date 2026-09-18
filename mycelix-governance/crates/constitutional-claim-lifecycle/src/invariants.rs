use crate::{
    ClaimLifecycleError, ClaimLifecycleStatus, ClaimLifecycleState, LifecycleFault,
    RevocationResolutionEvidence,
};
use constitutional_consumption::FinalityProfile;
use std::collections::BTreeSet;

fn profile_rank(profile: FinalityProfile) -> u8 {
    match profile {
        FinalityProfile::LocalIdempotent => 0,
        FinalityProfile::DetectionOnly => 1,
        FinalityProfile::WitnessedSingleSpend => 2,
        FinalityProfile::StrongConsensus => 3,
    }
}

impl ClaimLifecycleState {
    pub fn check_invariants(&self) -> Result<(), ClaimLifecycleError> {
        self.consumption.check_invariants()?;

        if self.temporal.policy.domain_id.trim().is_empty() {
            return Err(ClaimLifecycleError::InvariantViolation);
        }
        if profile_rank(self.temporal.policy.profile)
            < profile_rank(self.consumption.requirement.minimum_profile)
        {
            return Err(ClaimLifecycleError::InvariantViolation);
        }

        let internal_fault_active =
            self.temporal.integrity_fault.is_some() || self.consumption.integrity_fault.is_some();
        if internal_fault_active != self.halt_fault.is_some() {
            return Err(ClaimLifecycleError::InvariantViolation);
        }

        // A lifecycle-owned temporal state is strict: every retained finality
        // record entered through the bound path and therefore has exactly one
        // matching ClaimBinding. This rejects restored/tampered states that
        // attempt to upgrade legacy unbound evidence into lifecycle authority.
        let mut observed_finality_ids = BTreeSet::new();
        for finality_map in [
            &self.temporal.accepted_finality,
            &self.temporal.rejected_finality,
            &self.temporal.quarantined_finality,
        ] {
            for (evidence_id, evidence) in finality_map {
                if !observed_finality_ids.insert(evidence_id.as_str()) {
                    return Err(ClaimLifecycleError::InvariantViolation);
                }
                let Some(binding) = self.temporal.finality_binding(evidence_id) else {
                    return Err(ClaimLifecycleError::InvariantViolation);
                };
                if binding.claim_id.as_str() != evidence.claim_id.as_str() {
                    return Err(ClaimLifecycleError::InvariantViolation);
                }
            }
        }
        if observed_finality_ids.len() != self.temporal.finality_bindings.len()
            || self
                .temporal
                .finality_bindings
                .keys()
                .any(|evidence_id| !observed_finality_ids.contains(evidence_id.as_str()))
        {
            return Err(ClaimLifecycleError::InvariantViolation);
        }

        if self.halt_fault.is_none() {
            let earliest_temporal_revocation = self
                .temporal
                .revocations
                .values()
                .min_by_key(|evidence| evidence.order.effective_seq);

            match (&self.active_revocation, earliest_temporal_revocation) {
                (None, None) => {
                    if self.consumption.revoked_at_seq.is_some() {
                        return Err(ClaimLifecycleError::InvariantViolation);
                    }
                }
                (Some(active), Some(evidence)) => {
                    if active.effective_seq != evidence.order.effective_seq {
                        return Err(ClaimLifecycleError::InvariantViolation);
                    }
                    let Some(bound) = self.temporal.revocations.get(&active.evidence_id) else {
                        return Err(ClaimLifecycleError::InvariantViolation);
                    };
                    if bound.revocation_id != active.revocation_id
                        || bound.order.effective_seq != active.effective_seq
                        || self.consumption.revoked_at_seq != Some(active.effective_seq)
                    {
                        return Err(ClaimLifecycleError::InvariantViolation);
                    }
                }
                _ => return Err(ClaimLifecycleError::InvariantViolation),
            }
        } else if let Some(active) = &self.active_revocation {
            let Some(bound) = self.temporal.revocations.get(&active.evidence_id) else {
                return Err(ClaimLifecycleError::InvariantViolation);
            };
            if bound.revocation_id != active.revocation_id
                || bound.order.effective_seq != active.effective_seq
            {
                return Err(ClaimLifecycleError::InvariantViolation);
            }
        }

        let mut finalized_use_indexes = BTreeSet::new();
        let mut transition_ordinals = BTreeSet::new();
        let mut transition_count = 0u64;

        for (claim_id, record) in &self.claims {
            if claim_id != &record.claim.claim_id
                || record.claim.key.jurisdiction != self.temporal.policy.domain_id
                || record.claim.budget_id != self.consumption.budget.budget_id
            {
                return Err(ClaimLifecycleError::InvariantViolation);
            }

            match &record.status {
                ClaimLifecycleStatus::PendingExecutable => {
                    if self.active_revocation.is_some() || self.halt_fault.is_some() {
                        return Err(ClaimLifecycleError::InvariantViolation);
                    }
                }
                ClaimLifecycleStatus::BlockedAwaitingEvidenceClosure { revocation } => {
                    let Some(active) = &self.active_revocation else {
                        return Err(ClaimLifecycleError::InvariantViolation);
                    };
                    if revocation != active || self.halt_fault.is_some() {
                        return Err(ClaimLifecycleError::InvariantViolation);
                    }
                }
                ClaimLifecycleStatus::Finalized {
                    use_index,
                    claim_binding,
                    finality_evidence_id,
                    proof_id,
                    finalized_effective_seq,
                } => {
                    if !finalized_use_indexes.insert(*use_index) {
                        return Err(ClaimLifecycleError::InvariantViolation);
                    }
                    let Some(finalized) = self.consumption.finalized.get(use_index) else {
                        return Err(ClaimLifecycleError::InvariantViolation);
                    };
                    if finalized.claim.claim_id != *claim_id
                        || finalized.proof.proof_id != *proof_id
                        || finalized.proof.finalized_at_seq != *finalized_effective_seq
                        || claim_binding != &record.claim.binding()
                    {
                        return Err(ClaimLifecycleError::InvariantViolation);
                    }
                    let Some(evidence) =
                        self.temporal.accepted_finality.get(finality_evidence_id)
                    else {
                        return Err(ClaimLifecycleError::InvariantViolation);
                    };
                    let Some(temporal_binding) =
                        self.temporal.finality_binding(finality_evidence_id)
                    else {
                        return Err(ClaimLifecycleError::InvariantViolation);
                    };
                    if evidence.claim_id != *claim_id
                        || evidence.proof_id != *proof_id
                        || evidence.profile != finalized.proof.profile
                        || evidence.order.effective_seq != *finalized_effective_seq
                        || temporal_binding != claim_binding
                    {
                        return Err(ClaimLifecycleError::InvariantViolation);
                    }
                }
                ClaimLifecycleStatus::RejectedConflict {
                    use_index,
                    winning_claim_id,
                    winning_claim_binding,
                    winning_finality_evidence_id,
                    winning_proof_id,
                } => {
                    let Some(winner) = self.consumption.finalized.get(use_index) else {
                        return Err(ClaimLifecycleError::InvariantViolation);
                    };
                    if winner.claim.claim_id != *winning_claim_id
                        || winner.proof.proof_id != *winning_proof_id
                        || winning_claim_binding != &winner.claim.binding()
                    {
                        return Err(ClaimLifecycleError::InvariantViolation);
                    }
                    let Some(evidence) = self
                        .temporal
                        .accepted_finality
                        .get(winning_finality_evidence_id)
                    else {
                        return Err(ClaimLifecycleError::InvariantViolation);
                    };
                    let Some(temporal_binding) = self
                        .temporal
                        .finality_binding(winning_finality_evidence_id)
                    else {
                        return Err(ClaimLifecycleError::InvariantViolation);
                    };
                    if evidence.claim_id != *winning_claim_id
                        || evidence.proof_id != *winning_proof_id
                        || evidence.order.effective_seq != winner.proof.finalized_at_seq
                        || temporal_binding != winning_claim_binding
                    {
                        return Err(ClaimLifecycleError::InvariantViolation);
                    }
                }
                ClaimLifecycleStatus::RevokedClosed {
                    revocation,
                    resolution,
                } => {
                    let Some(evidence) = self.temporal.revocations.get(&revocation.evidence_id) else {
                        return Err(ClaimLifecycleError::InvariantViolation);
                    };
                    if evidence.revocation_id != revocation.revocation_id
                        || evidence.order.effective_seq != revocation.effective_seq
                    {
                        return Err(ClaimLifecycleError::InvariantViolation);
                    }
                    match resolution {
                        RevocationResolutionEvidence::EmptyPreRevocationInterval {
                            finality_domain_id,
                            policy_version,
                        } => {
                            if revocation.effective_seq != 1
                                || finality_domain_id != &self.temporal.policy.domain_id
                                || policy_version != &self.temporal.policy.policy_version
                            {
                                return Err(ClaimLifecycleError::InvariantViolation);
                            }
                        }
                        RevocationResolutionEvidence::Closed {
                            finality_domain_id,
                            policy_version,
                            closure,
                        } => {
                            if finality_domain_id != &self.temporal.policy.domain_id
                                || policy_version != &self.temporal.policy.policy_version
                                || closure.domain_id != self.temporal.policy.domain_id
                                || closure.policy_version != self.temporal.policy.policy_version
                                || closure.closed_through_effective_seq
                                    < revocation.effective_seq.saturating_sub(1)
                            {
                                return Err(ClaimLifecycleError::InvariantViolation);
                            }
                            let Some(retained) = self.temporal.closures.get(&closure.closure_id)
                            else {
                                return Err(ClaimLifecycleError::InvariantViolation);
                            };
                            if retained != closure {
                                return Err(ClaimLifecycleError::InvariantViolation);
                            }
                        }
                    }
                }
                ClaimLifecycleStatus::IntegrityHalted { fault } => {
                    if self.halt_fault.as_ref() != Some(fault) {
                        return Err(ClaimLifecycleError::InvariantViolation);
                    }
                }
            }

            let mut previous_to: Option<ClaimLifecycleStatus> = None;
            for receipt in &record.transitions {
                transition_count = transition_count
                    .checked_add(1)
                    .ok_or(ClaimLifecycleError::InvariantViolation)?;
                if receipt.ordinal == 0 || !transition_ordinals.insert(receipt.ordinal) {
                    return Err(ClaimLifecycleError::InvariantViolation);
                }
                if receipt.claim_id != record.claim.claim_id
                    || receipt.use_index != record.claim.key.use_index
                    || receipt.matter != record.claim.matter
                    || receipt.envelope_digest != record.claim.key.envelope_digest
                    || receipt.target_digest != record.claim.target_digest
                    || receipt.payload_digest != record.claim.payload_digest
                    || receipt.claim_binding != record.claim.binding()
                {
                    return Err(ClaimLifecycleError::InvariantViolation);
                }
                if let Some(previous) = previous_to.as_ref() {
                    if previous != &receipt.from || previous.is_terminal() {
                        return Err(ClaimLifecycleError::InvariantViolation);
                    }
                }
                previous_to = Some(receipt.to.clone());
            }
            if let Some(last) = previous_to {
                if last != record.status {
                    return Err(ClaimLifecycleError::InvariantViolation);
                }
            } else if record.status.is_terminal() {
                return Err(ClaimLifecycleError::InvariantViolation);
            }
        }

        if transition_count != self.next_transition_ordinal {
            return Err(ClaimLifecycleError::InvariantViolation);
        }
        for expected in 1..=self.next_transition_ordinal {
            if !transition_ordinals.contains(&expected) {
                return Err(ClaimLifecycleError::InvariantViolation);
            }
        }

        for (use_index, finalized) in &self.consumption.finalized {
            let Some(record) = self.claims.get(&finalized.claim.claim_id) else {
                return Err(ClaimLifecycleError::InvariantViolation);
            };
            if !matches!(
                record.status,
                ClaimLifecycleStatus::Finalized {
                    use_index: status_use_index,
                    ..
                } if status_use_index == *use_index
            ) {
                return Err(ClaimLifecycleError::InvariantViolation);
            }

            for (other_id, other) in &self.claims {
                if other_id != &finalized.claim.claim_id
                    && other.claim.key.use_index == *use_index
                    && (other.status.is_live()
                        || matches!(other.status, ClaimLifecycleStatus::Finalized { .. }))
                {
                    return Err(ClaimLifecycleError::InvariantViolation);
                }
            }
        }

        for (use_index, effect) in &self.consumption.effects {
            let Some(finalized) = self.consumption.finalized.get(use_index) else {
                return Err(ClaimLifecycleError::InvariantViolation);
            };
            let Some(record) = self.claims.get(&effect.claim_id) else {
                return Err(ClaimLifecycleError::InvariantViolation);
            };
            if finalized.claim.claim_id != effect.claim_id
                || !matches!(record.status, ClaimLifecycleStatus::Finalized { .. })
            {
                return Err(ClaimLifecycleError::InvariantViolation);
            }
        }

        if self.halt_fault.is_some() && self.claims.values().any(|record| record.status.is_live()) {
            return Err(ClaimLifecycleError::InvariantViolation);
        }

        if let Some(LifecycleFault::Temporal { fault, .. }) = &self.halt_fault {
            if self.temporal.integrity_fault.as_ref() != Some(fault) {
                return Err(ClaimLifecycleError::InvariantViolation);
            }
        }

        Ok(())
    }
}


#[cfg(test)]
mod binding_recovery_tests {
    use super::*;
    use constitutional_consumption::{
        FinalityRequirement, RevocationCutoff, UsageBudget,
    };
    use constitutional_temporal_provenance::{
        FinalityDomainPolicy, FinalityEvidence, TemporalEvidenceState, TemporalOrder,
    };

    #[test]
    fn restored_lifecycle_rejects_unbound_finality_record() {
        let temporal = TemporalEvidenceState::new(FinalityDomainPolicy {
            domain_id: "domain-a".into(),
            profile: FinalityProfile::WitnessedSingleSpend,
            policy_version: "policy-v1".into(),
            min_closure_witnesses: 2,
            min_closure_independence_domains: 2,
        })
        .unwrap();
        let mut state = ClaimLifecycleState::new(
            temporal,
            UsageBudget {
                budget_id: "budget-a".into(),
                max_uses: 1,
            },
            FinalityRequirement {
                minimum_profile: FinalityProfile::WitnessedSingleSpend,
                min_witnesses: 2,
                min_distinct_domains: 2,
                revocation_cutoff: RevocationCutoff::Finality,
            },
        )
        .unwrap();

        state.temporal.accepted_finality.insert(
            "legacy-unbound".into(),
            FinalityEvidence {
                evidence_id: "legacy-unbound".into(),
                claim_id: "claim-a".into(),
                proof_id: "proof-a".into(),
                profile: FinalityProfile::WitnessedSingleSpend,
                order: TemporalOrder {
                    domain_id: "domain-a".into(),
                    effective_seq: 1,
                    observed_seq: 1,
                },
            },
        );

        assert_eq!(
            state.validate_restored(),
            Err(ClaimLifecycleError::InvariantViolation)
        );
    }
}
