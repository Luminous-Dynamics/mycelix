use constitutional_closure_coverage::ClosureCoverageError;
use constitutional_consumption::{
    ConsumptionClaim, ConsumptionError, IntegrityFault as ConsumptionIntegrityFault,
};
use constitutional_envelope::MatterId;
use constitutional_temporal_provenance::{
    EvidenceClosure, TemporalIntegrityFault, TemporalProvenanceError,
};
use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct RevocationProvenance {
    pub evidence_id: String,
    pub revocation_id: String,
    pub effective_seq: u64,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum RevocationResolutionEvidence {
    EmptyPreRevocationInterval {
        finality_domain_id: String,
        policy_version: String,
    },
    Closed {
        finality_domain_id: String,
        policy_version: String,
        closure: EvidenceClosure,
    },
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum LifecycleFault {
    Consumption {
        revocation_evidence_id: String,
        fault: ConsumptionIntegrityFault,
    },
    Temporal {
        finality_domain_id: String,
        policy_version: String,
        fault: TemporalIntegrityFault,
    },
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum ClaimLifecycleStatus {
    PendingExecutable,
    BlockedAwaitingEvidenceClosure {
        revocation: RevocationProvenance,
    },
    Finalized {
        use_index: u32,
        finality_evidence_id: String,
        proof_id: String,
        finalized_effective_seq: u64,
    },
    RejectedConflict {
        use_index: u32,
        winning_claim_id: String,
        winning_finality_evidence_id: String,
        winning_proof_id: String,
    },
    RevokedClosed {
        revocation: RevocationProvenance,
        resolution: RevocationResolutionEvidence,
    },
    IntegrityHalted {
        fault: LifecycleFault,
    },
}

impl ClaimLifecycleStatus {
    pub fn is_terminal(&self) -> bool {
        matches!(
            self,
            Self::Finalized { .. }
                | Self::RejectedConflict { .. }
                | Self::RevokedClosed { .. }
                | Self::IntegrityHalted { .. }
        )
    }

    pub fn is_live(&self) -> bool {
        matches!(
            self,
            Self::PendingExecutable | Self::BlockedAwaitingEvidenceClosure { .. }
        )
    }
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum ClaimTransitionReason {
    RevocationObserved,
    EarlierRevocationObserved,
    AcceptedFinality {
        finality_evidence_id: String,
    },
    CompetingFinalityWon {
        winning_claim_id: String,
        winning_finality_evidence_id: String,
        winning_proof_id: String,
    },
    EvidenceIntervalClosed,
    IntegrityFault,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct ClaimTransitionReceipt {
    pub ordinal: u64,
    pub claim_id: String,
    pub use_index: u32,
    pub matter: MatterId,
    pub envelope_digest: String,
    pub target_digest: String,
    pub payload_digest: String,
    pub from: ClaimLifecycleStatus,
    pub to: ClaimLifecycleStatus,
    pub reason: ClaimTransitionReason,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct ClaimRecord {
    pub claim: ConsumptionClaim,
    pub status: ClaimLifecycleStatus,
    pub transitions: Vec<ClaimTransitionReceipt>,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum LifecycleRevokeOutcome {
    Revoked,
    AlreadyRevoked,
    IntegrityFault { fault: LifecycleFault },
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum CoverageResolutionOutcome {
    Open,
    Terminalized { claim_ids: Vec<String> },
    IntegrityFault { fault: LifecycleFault },
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum ClaimLifecycleError {
    Consumption(ConsumptionError),
    Temporal(TemporalProvenanceError),
    ClosureCoverage(ClosureCoverageError),
    EmptyFinalityDomainId,
    ClaimDomainMismatch,
    TemporalDomainMismatch,
    TemporalProfileTooWeak,
    NonPristineTemporalState,
    UnknownClaim,
    UnknownFinalityEvidence,
    UnknownRevocationEvidence,
    FinalityEvidenceMismatch,
    TerminalClaim,
    ClaimNotFinalized,
    NoActiveRevocation,
    IntegrityFaultActive,
    TransitionOrdinalOverflow,
    InvariantViolation,
}

impl From<ConsumptionError> for ClaimLifecycleError {
    fn from(value: ConsumptionError) -> Self {
        Self::Consumption(value)
    }
}

impl From<TemporalProvenanceError> for ClaimLifecycleError {
    fn from(value: TemporalProvenanceError) -> Self {
        Self::Temporal(value)
    }
}

impl From<ClosureCoverageError> for ClaimLifecycleError {
    fn from(value: ClosureCoverageError) -> Self {
        Self::ClosureCoverage(value)
    }
}
