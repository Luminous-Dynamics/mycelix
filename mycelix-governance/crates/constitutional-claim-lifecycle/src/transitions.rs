use crate::{
    ClaimLifecycleError, ClaimLifecycleStatus, ClaimLifecycleState, ClaimTransitionReason,
    CoverageResolutionOutcome, LifecycleFault, LifecycleRevokeOutcome, RevocationProvenance,
    RevocationResolutionEvidence,
};
use constitutional_closure_coverage::{assess_pre_revocation_coverage, PreRevocationCoverage};
use constitutional_consumption::{
    ConsumptionError, EffectOutcome, FinalityProof, FinalizeOutcome, RevokeOutcome,
};
use constitutional_temporal_provenance::{
    AcceptClosureOutcome, EvidenceClosure, FinalityEvidence, ObserveFinalityOutcome,
    RevocationEvidence,
};

impl ClaimLifecycleState {
    pub fn observe_finality_evidence(
        &mut self,
        evidence: FinalityEvidence,
    ) -> Result<ObserveFinalityOutcome, ClaimLifecycleError> {
        self.transaction(|staged| {
            let outcome = staged.temporal.observe_finality(evidence)?;
            staged.synchronize_temporal_fault_in_place()?;
            Ok(outcome)
        })
    }

    pub fn observe_revocation_evidence(
        &mut self,
        evidence: RevocationEvidence,
    ) -> Result<LifecycleRevokeOutcome, ClaimLifecycleError> {
        self.transaction(|staged| {
            let evidence_id = evidence.evidence_id.clone();
            staged.temporal.observe_revocation(evidence)?;
            staged.synchronize_temporal_fault_in_place()?;

            if let Some(fault) = staged.halt_fault.clone() {
                return Ok(LifecycleRevokeOutcome::IntegrityFault { fault });
            }

            staged.apply_revocation_in_place(&evidence_id)
        })
    }

    pub fn accept_closure(
        &mut self,
        closure: EvidenceClosure,
    ) -> Result<AcceptClosureOutcome, ClaimLifecycleError> {
        self.transaction(|staged| {
            staged.synchronize_temporal_fault_in_place()?;
            staged.ensure_not_halted()?;
            let outcome = staged.temporal.accept_closure(closure)?;
            Ok(outcome)
        })
    }

    pub fn finalize(
        &mut self,
        evidence_id: &str,
        claim_id: &str,
        proof: FinalityProof,
    ) -> Result<FinalizeOutcome, ClaimLifecycleError> {
        let evidence_id = evidence_id.to_owned();
        let claim_id = claim_id.to_owned();
        self.transaction(|staged| staged.finalize_in_place(&evidence_id, &claim_id, proof))
    }

    fn finalize_in_place(
        &mut self,
        evidence_id: &str,
        claim_id: &str,
        proof: FinalityProof,
    ) -> Result<FinalizeOutcome, ClaimLifecycleError> {
        self.synchronize_temporal_fault_in_place()?;
        self.ensure_not_halted()?;

        let record = self
            .claims
            .get(claim_id)
            .ok_or(ClaimLifecycleError::UnknownClaim)?;
        if matches!(
            record.status,
            ClaimLifecycleStatus::RejectedConflict { .. }
                | ClaimLifecycleStatus::RevokedClosed { .. }
                | ClaimLifecycleStatus::IntegrityHalted { .. }
        ) {
            return Err(ClaimLifecycleError::TerminalClaim);
        }

        let evidence = self
            .temporal
            .accepted_finality
            .get(evidence_id)
            .ok_or(ClaimLifecycleError::UnknownFinalityEvidence)?;
        if evidence.claim_id != claim_id
            || evidence.proof_id != proof.proof_id
            || evidence.profile != proof.profile
            || evidence.order.effective_seq != proof.finalized_at_seq
        {
            return Err(ClaimLifecycleError::FinalityEvidenceMismatch);
        }

        let use_index = record.claim.key.use_index;
        let outcome = self.consumption.finalize(claim_id, proof.clone())?;
        if outcome == FinalizeOutcome::AlreadyFinalized {
            if !matches!(
                self.claims.get(claim_id).map(|r| &r.status),
                Some(ClaimLifecycleStatus::Finalized {
                    finality_evidence_id,
                    proof_id,
                    ..
                }) if finality_evidence_id == evidence_id && proof_id == &proof.proof_id
            ) {
                return Err(ClaimLifecycleError::InvariantViolation);
            }
            return Ok(outcome);
        }

        self.transition_claim(
            claim_id,
            ClaimLifecycleStatus::Finalized {
                use_index,
                finality_evidence_id: evidence_id.to_owned(),
                proof_id: proof.proof_id.clone(),
                finalized_effective_seq: proof.finalized_at_seq,
            },
            ClaimTransitionReason::AcceptedFinality {
                finality_evidence_id: evidence_id.to_owned(),
            },
        )?;

        let competitors: Vec<String> = self
            .claims
            .iter()
            .filter(|(id, other)| {
                id.as_str() != claim_id
                    && other.claim.key.use_index == use_index
                    && other.status.is_live()
            })
            .map(|(id, _)| id.clone())
            .collect();

        for competitor in competitors {
            self.transition_claim(
                &competitor,
                ClaimLifecycleStatus::RejectedConflict {
                    use_index,
                    winning_claim_id: claim_id.to_owned(),
                    winning_finality_evidence_id: evidence_id.to_owned(),
                    winning_proof_id: proof.proof_id.clone(),
                },
                ClaimTransitionReason::CompetingFinalityWon {
                    winning_claim_id: claim_id.to_owned(),
                    winning_finality_evidence_id: evidence_id.to_owned(),
                    winning_proof_id: proof.proof_id.clone(),
                },
            )?;
        }

        Ok(outcome)
    }

    fn apply_revocation_in_place(
        &mut self,
        revocation_evidence_id: &str,
    ) -> Result<LifecycleRevokeOutcome, ClaimLifecycleError> {
        let evidence = self
            .temporal
            .revocations
            .get(revocation_evidence_id)
            .cloned()
            .ok_or(ClaimLifecycleError::UnknownRevocationEvidence)?;
        let revocation = RevocationProvenance {
            evidence_id: evidence.evidence_id,
            revocation_id: evidence.revocation_id,
            effective_seq: evidence.order.effective_seq,
        };

        let reason = match &self.active_revocation {
            Some(existing) if revocation.effective_seq < existing.effective_seq => {
                ClaimTransitionReason::EarlierRevocationObserved
            }
            _ => ClaimTransitionReason::RevocationObserved,
        };

        let should_replace = self
            .active_revocation
            .as_ref()
            .map(|existing| revocation.effective_seq < existing.effective_seq)
            .unwrap_or(true);
        if should_replace {
            self.active_revocation = Some(revocation.clone());
        }

        match self.consumption.revoke(revocation.effective_seq) {
            Ok(outcome) => {
                let active = self
                    .active_revocation
                    .clone()
                    .ok_or(ClaimLifecycleError::InvariantViolation)?;

                for claim_id in self.live_claim_ids() {
                    let should_transition = match self.claims.get(&claim_id).map(|r| &r.status) {
                        Some(ClaimLifecycleStatus::PendingExecutable) => true,
                        Some(ClaimLifecycleStatus::BlockedAwaitingEvidenceClosure {
                            revocation: existing,
                        }) => active.effective_seq < existing.effective_seq,
                        _ => false,
                    };
                    if should_transition {
                        self.transition_claim(
                            &claim_id,
                            ClaimLifecycleStatus::BlockedAwaitingEvidenceClosure {
                                revocation: active.clone(),
                            },
                            reason.clone(),
                        )?;
                    }
                }

                Ok(match outcome {
                    RevokeOutcome::Revoked => LifecycleRevokeOutcome::Revoked,
                    RevokeOutcome::AlreadyRevoked => LifecycleRevokeOutcome::AlreadyRevoked,
                })
            }
            Err(ConsumptionError::LateEarlierRevocationConflict) => {
                let fault = self
                    .consumption
                    .integrity_fault
                    .clone()
                    .ok_or(ClaimLifecycleError::InvariantViolation)?;
                let lifecycle_fault = LifecycleFault::Consumption {
                    revocation_evidence_id: revocation_evidence_id.to_owned(),
                    fault,
                };
                self.halt_live_claims(lifecycle_fault.clone())?;
                Ok(LifecycleRevokeOutcome::IntegrityFault {
                    fault: lifecycle_fault,
                })
            }
            Err(error) => Err(error.into()),
        }
    }

    pub fn resolve_revocation_coverage(
        &mut self,
    ) -> Result<CoverageResolutionOutcome, ClaimLifecycleError> {
        self.transaction(|staged| staged.resolve_revocation_coverage_in_place())
    }

    fn resolve_revocation_coverage_in_place(
        &mut self,
    ) -> Result<CoverageResolutionOutcome, ClaimLifecycleError> {
        if self.synchronize_temporal_fault_in_place()? {
            let fault = self
                .halt_fault
                .clone()
                .ok_or(ClaimLifecycleError::InvariantViolation)?;
            return Ok(CoverageResolutionOutcome::IntegrityFault { fault });
        }
        if let Some(fault) = self.halt_fault.clone() {
            return Ok(CoverageResolutionOutcome::IntegrityFault { fault });
        }

        let revocation = self
            .active_revocation
            .clone()
            .ok_or(ClaimLifecycleError::NoActiveRevocation)?;
        let coverage = assess_pre_revocation_coverage(&self.temporal, revocation.effective_seq)?;

        let resolution = match coverage {
            PreRevocationCoverage::Open { .. } => {
                return Ok(CoverageResolutionOutcome::Open)
            }
            PreRevocationCoverage::IntegrityFault { fault, .. } => {
                let lifecycle_fault = LifecycleFault::Temporal {
                    finality_domain_id: self.temporal.policy.domain_id.clone(),
                    policy_version: self.temporal.policy.policy_version.clone(),
                    fault,
                };
                self.halt_live_claims(lifecycle_fault.clone())?;
                return Ok(CoverageResolutionOutcome::IntegrityFault {
                    fault: lifecycle_fault,
                });
            }
            PreRevocationCoverage::EmptyPreRevocationInterval {
                finality_domain_id,
                policy_version,
                ..
            } => RevocationResolutionEvidence::EmptyPreRevocationInterval {
                finality_domain_id,
                policy_version,
            },
            PreRevocationCoverage::Closed {
                finality_domain_id,
                policy_version,
                closure,
                ..
            } => RevocationResolutionEvidence::Closed {
                finality_domain_id,
                policy_version,
                closure,
            },
        };

        let blocked: Vec<String> = self
            .claims
            .iter()
            .filter(|(_, record)| {
                matches!(
                    record.status,
                    ClaimLifecycleStatus::BlockedAwaitingEvidenceClosure { .. }
                )
            })
            .map(|(id, _)| id.clone())
            .collect();

        for claim_id in &blocked {
            self.transition_claim(
                claim_id,
                ClaimLifecycleStatus::RevokedClosed {
                    revocation: revocation.clone(),
                    resolution: resolution.clone(),
                },
                ClaimTransitionReason::EvidenceIntervalClosed,
            )?;
        }

        Ok(CoverageResolutionOutcome::Terminalized { claim_ids: blocked })
    }

    pub fn apply_effect(
        &mut self,
        claim_id: &str,
        applied_at_seq: u64,
        output_ref: impl Into<String>,
    ) -> Result<EffectOutcome, ClaimLifecycleError> {
        let claim_id = claim_id.to_owned();
        let output_ref = output_ref.into();
        self.transaction(|staged| staged.apply_effect_in_place(&claim_id, applied_at_seq, output_ref))
    }

    fn apply_effect_in_place(
        &mut self,
        claim_id: &str,
        applied_at_seq: u64,
        output_ref: String,
    ) -> Result<EffectOutcome, ClaimLifecycleError> {
        let record = self
            .claims
            .get(claim_id)
            .ok_or(ClaimLifecycleError::UnknownClaim)?;
        let use_index = match record.status {
            ClaimLifecycleStatus::Finalized { use_index, .. } => use_index,
            _ => return Err(ClaimLifecycleError::ClaimNotFinalized),
        };

        if let Some(existing) = self.consumption.effects.get(&use_index) {
            if existing.claim_id == claim_id {
                return Ok(EffectOutcome::AlreadyApplied {
                    output_ref: existing.output_ref.clone(),
                });
            }
            return Err(ClaimLifecycleError::InvariantViolation);
        }

        self.synchronize_temporal_fault_in_place()?;
        self.ensure_not_halted()?;
        Ok(self
            .consumption
            .apply_effect(claim_id, applied_at_seq, output_ref)?)
    }
}
