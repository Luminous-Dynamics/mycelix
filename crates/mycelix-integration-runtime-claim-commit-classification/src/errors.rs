#[derive(Debug, Error)]
pub enum ClaimCommitFailure {
    #[error("SQLite COMMIT returned an error: {0}")]
    Sqlite(rusqlite::Error),
    #[error("COMMIT succeeded but exact runtime-store binding no longer revalidated: {0}")]
    PostCommitStoreBinding(RuntimeStoreBindingError),
    #[cfg(test)]
    #[error("deterministic injected commit result: {0}")]
    Injected(&'static str),
}

#[derive(Debug, Error)]
pub enum DurableClaimRereadError {
    #[error(transparent)]
    StoreBinding(#[from] RuntimeStoreBindingError),
    #[error(transparent)]
    Claim(#[from] ClaimClassificationError),
    #[error(transparent)]
    Sqlite(#[from] rusqlite::Error),
}

#[derive(Debug, Error)]
pub enum ClaimClassificationError {
    #[error(transparent)]
    StoreBinding(#[from] RuntimeStoreBindingError),
    #[error(transparent)]
    Sqlite(#[from] rusqlite::Error),
    #[error(transparent)]
    Json(#[from] serde_json::Error),
    #[error("worker id is empty")]
    EmptyWorkerId,
    #[error("worker id exceeds {MAX_WORKER_ID_BYTES} bytes")]
    WorkerIdTooLong,
    #[error("worker id contains a control character")]
    WorkerIdContainsControl,
    #[error("claim limit must be between 1 and {MAX_OUTBOX_CLAIM}")]
    InvalidClaimLimit,
    #[error("lease duration must be positive")]
    InvalidLeaseDuration,
    #[error("lease deadline overflows i64 milliseconds")]
    LeaseDeadlineOverflow,
    #[error("timestamp must be non-negative milliseconds")]
    InvalidTimestamp,
    #[error("runtime recovery batch exceeds {limit} rows")]
    RecoveryBatchTooLarge { limit: usize },
    #[error("execution attempt budget for outbox entry {entry_id} reached {limit}")]
    AttemptBudgetExceeded { entry_id: i64, limit: u32 },
    #[error("execution observation history for outbox entry {entry_id} reached its v0.1 bound")]
    ExecutionObservationHistoryLimit { entry_id: i64 },
    #[error("serialized recovery outcome exceeds runtime bound")]
    OutcomeTooLarge,
    #[error("outbox entry {entry_id} was not found")]
    UnknownOutboxEntry { entry_id: i64 },
    #[error("outbox entry {entry_id} changed concurrently")]
    ConcurrentStateChange { entry_id: i64 },
    #[error("outbox entry {entry_id} is in storage stage {actual}, expected {expected}")]
    UnexpectedStage {
        entry_id: i64,
        expected: i64,
        actual: i64,
    },
    #[error("stored integration identifier violates core invariants: {0}")]
    StoredIdentifier(String),
    #[error("stored commitment has invalid length {actual}, expected 32")]
    InvalidStoredDigestLength { actual: usize },
    #[error("invalid stored numeric value for {field}: {value}")]
    InvalidStoredValue { field: &'static str, value: i64 },
    #[error("runtime schema differs from the exact v2 claim/recovery contract")]
    RuntimeSchemaMismatch,
    #[error("unsupported integration runtime schema version {actual}; expected {expected}")]
    UnsupportedRuntimeSchema { actual: i64, expected: i64 },
    #[error("runtime SQLite journal_mode is not WAL: {0}")]
    UnexpectedJournalMode(String),
    #[error("runtime SQLite synchronous mode is not FULL: {0}")]
    UnexpectedSynchronousMode(i64),
    #[error("runtime store path no longer names the exact main database")]
    StoreIdentityChanged,
    #[error("runtime store filesystem error: {0}")]
    Io(std::io::Error),
}
