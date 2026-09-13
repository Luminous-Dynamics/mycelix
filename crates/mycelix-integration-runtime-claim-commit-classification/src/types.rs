#[derive(Debug)]
pub enum ClassifiedClaimOutcome {
    Committed(CommittedClaimBatch),
    RecoveredCommitted(RecoveredCommittedClaimBatch),
    DefinitelyNotCommitted(DefinitelyNotCommittedClaimBatch),
    IndeterminateCommit(IndeterminateClaimBatch),
}

impl ClassifiedClaimOutcome {
    pub const fn classification(&self) -> CommitClassification {
        match self {
            Self::Committed(_) | Self::RecoveredCommitted(_) => CommitClassification::Committed,
            Self::DefinitelyNotCommitted(_) => CommitClassification::DefinitelyNotCommitted,
            Self::IndeterminateCommit(_) => CommitClassification::IndeterminateCommit,
        }
    }

    pub const fn grants_execution_authority(&self) -> bool {
        false
    }

    pub const fn ordinary_retry_authorized_here(&self) -> bool {
        false
    }
}

#[derive(Debug)]
pub struct CommittedClaimBatch {
    claims: Vec<ExecutionClaim>,
    recovery: RecoverySummary,
}

impl CommittedClaimBatch {
    pub fn claims(&self) -> &[ExecutionClaim] {
        &self.claims
    }

    pub const fn recovery_summary(&self) -> RecoverySummary {
        self.recovery
    }

    pub const fn live_claim_metadata_returned_here(&self) -> bool {
        true
    }

    pub const fn grants_execution_authority(&self) -> bool {
        false
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RecoveredAttemptIdentity {
    entry_id: i64,
    attempt_id: String,
    lease_until_ms: i64,
}

impl RecoveredAttemptIdentity {
    pub const fn entry_id(&self) -> i64 {
        self.entry_id
    }

    pub fn attempt_id(&self) -> &str {
        &self.attempt_id
    }

    pub const fn lease_until_ms(&self) -> i64 {
        self.lease_until_ms
    }
}

#[derive(Debug)]
pub struct RecoveredCommittedClaimBatch {
    attempts: Vec<RecoveredAttemptIdentity>,
    recovery: RecoverySummary,
    commit_error: ClaimCommitFailure,
}

impl RecoveredCommittedClaimBatch {
    pub fn attempts(&self) -> &[RecoveredAttemptIdentity] {
        &self.attempts
    }

    pub const fn recovery_summary(&self) -> RecoverySummary {
        self.recovery
    }

    pub fn commit_error(&self) -> &ClaimCommitFailure {
        &self.commit_error
    }

    pub const fn exact_candidate_successor_reobserved_here(&self) -> bool {
        true
    }

    pub const fn live_claims_reconstructed_here(&self) -> bool {
        false
    }

    pub const fn recovery_only_here(&self) -> bool {
        true
    }

    pub const fn ordinary_retry_authorized_here(&self) -> bool {
        false
    }

    pub const fn grants_execution_authority(&self) -> bool {
        false
    }
}

#[derive(Debug)]
pub struct DefinitelyNotCommittedClaimBatch {
    touched_entry_ids: Vec<i64>,
    commit_error: ClaimCommitFailure,
}

impl DefinitelyNotCommittedClaimBatch {
    pub fn touched_entry_ids(&self) -> &[i64] {
        &self.touched_entry_ids
    }

    pub fn commit_error(&self) -> &ClaimCommitFailure {
        &self.commit_error
    }

    pub const fn exact_predecessor_reobserved_here(&self) -> bool {
        true
    }

    pub const fn persistence_retry_eligible_here(&self) -> bool {
        true
    }

    pub const fn ordinary_retry_authorized_here(&self) -> bool {
        false
    }

    pub const fn live_claims_reconstructed_here(&self) -> bool {
        false
    }

    pub const fn grants_execution_authority(&self) -> bool {
        false
    }
}

#[derive(Debug)]
pub struct IndeterminateClaimBatch {
    touched_entry_ids: Vec<i64>,
    commit_error: ClaimCommitFailure,
    reread_error: Option<DurableClaimRereadError>,
}

impl IndeterminateClaimBatch {
    pub fn touched_entry_ids(&self) -> &[i64] {
        &self.touched_entry_ids
    }

    pub fn commit_error(&self) -> &ClaimCommitFailure {
        &self.commit_error
    }

    pub fn reread_error(&self) -> Option<&DurableClaimRereadError> {
        self.reread_error.as_ref()
    }

    pub const fn persistence_retry_eligible_here(&self) -> bool {
        false
    }

    pub const fn ordinary_retry_authorized_here(&self) -> bool {
        false
    }

    pub const fn live_claims_reconstructed_here(&self) -> bool {
        false
    }

    pub const fn recovery_only_here(&self) -> bool {
        true
    }

    pub const fn grants_execution_authority(&self) -> bool {
        false
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
struct OutboxRow {
    entry_id: i64,
    command_id: String,
    connector_instance: String,
    command_commitment_algorithm: i64,
    command_commitment_digest: Vec<u8>,
    authority_commitment_algorithm: i64,
    authority_commitment_digest: Vec<u8>,
    side_effect_class: i64,
    idempotency_key: Option<String>,
    command_bytes: Vec<u8>,
    stage: i64,
    worker_id: Option<String>,
    lease_until_ms: Option<i64>,
    attempt_count: i64,
    current_attempt_id: Option<String>,
    dispatch_started_at_ms: Option<i64>,
    outcome_json: Option<Vec<u8>>,
    reconciliation_json: Option<Vec<u8>>,
    created_at_ms: i64,
    updated_at_ms: i64,
}

#[derive(Clone, Debug, PartialEq, Eq)]
struct ObservationRow {
    observation_id: i64,
    entry_id: i64,
    attempt_id: String,
    outcome_json: Vec<u8>,
    observed_at_ms: i64,
    applied_to_current: i64,
}

#[derive(Clone, Debug, PartialEq, Eq)]
struct ObservationSummary {
    count: i64,
    max_observation_id: Option<i64>,
    max_row: Option<ObservationRow>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
struct EntrySnapshot {
    outbox: OutboxRow,
    observations: ObservationSummary,
}

#[derive(Clone, Debug, PartialEq, Eq, Default)]
struct BatchWitness {
    entries: BTreeMap<i64, EntrySnapshot>,
}

impl BatchWitness {
    fn touched_entry_ids(&self) -> Vec<i64> {
        self.entries.keys().copied().collect()
    }
}

#[derive(Debug)]
struct TransactionProduct {
    claims: Vec<ExecutionClaim>,
    recovery: RecoverySummary,
    predecessor: BatchWitness,
    candidate: BatchWitness,
}
