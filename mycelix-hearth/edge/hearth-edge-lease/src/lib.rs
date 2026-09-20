// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Exclusive local executor ownership for physical Hearth automation.
//!
//! HTH-AUTO-004C closes the single-writer assumption left explicit by the
//! crash-safe edge journal. A physical adapter wrapper is returned only while
//! this process holds an exclusive OS lock for the canonical journal root.

use hearth_automation_types::{
    ActionSpec, CommandStatus, OutcomeExpectation, OutcomePolicy,
};
use hearth_edge::{EdgeAdapter, VerificationReport};
use hearth_edge_journal::{FileJournal, JournalError, JournaledAdapter};
use std::{
    fs::{self, File, OpenOptions, TryLockError},
    io::{Seek, SeekFrom, Write},
    path::{Path, PathBuf},
};

pub const EXECUTOR_LEASE_SCHEMA_VERSION: u16 = 1;
pub const EXECUTOR_LOCK_FILE: &str = ".hearth-edge-executor.lock";

#[derive(Debug)]
pub enum LeaseError {
    Io(String),
    AlreadyOwned { root: PathBuf },
}

impl std::fmt::Display for LeaseError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Io(message) => write!(f, "executor lease I/O error: {message}"),
            Self::AlreadyOwned { root } => write!(
                f,
                "journal root already has a live Hearth executor lease: {}",
                root.display()
            ),
        }
    }
}

impl std::error::Error for LeaseError {}

impl From<std::io::Error> for LeaseError {
    fn from(value: std::io::Error) -> Self {
        Self::Io(value.to_string())
    }
}

/// Process-lifetime ownership witness for one canonical Hearth journal root.
///
/// The persistent lockfile is only a rendezvous inode. Ownership is the live
/// exclusive lock held by `_lock_file`; stale file contents never grant or deny
/// authority by themselves.
#[derive(Debug)]
pub struct ExecutorLease {
    root: PathBuf,
    lock_path: PathBuf,
    _lock_file: File,
}

impl ExecutorLease {
    pub fn acquire(root: impl AsRef<Path>) -> Result<Self, LeaseError> {
        fs::create_dir_all(root.as_ref())?;
        let root = fs::canonicalize(root.as_ref())?;
        let lock_path = root.join(EXECUTOR_LOCK_FILE);
        let mut file = OpenOptions::new()
            .create(true)
            .read(true)
            .write(true)
            .open(&lock_path)?;

        match file.try_lock() {
            Ok(()) => {}
            Err(TryLockError::WouldBlock) => {
                return Err(LeaseError::AlreadyOwned { root });
            }
            Err(TryLockError::Error(error)) => return Err(error.into()),
        }

        // Diagnostic metadata is explicitly non-authoritative. Do not use its
        // presence, PID, or age to decide whether the lease is held.
        file.set_len(0)?;
        file.seek(SeekFrom::Start(0))?;
        writeln!(file, "schema={EXECUTOR_LEASE_SCHEMA_VERSION}")?;
        writeln!(file, "pid={}", std::process::id())?;
        file.sync_data()?;

        Ok(Self {
            root,
            lock_path,
            _lock_file: file,
        })
    }

    pub fn root(&self) -> &Path {
        &self.root
    }

    pub fn lock_path(&self) -> &Path {
        &self.lock_path
    }
}

#[derive(Debug)]
pub enum PhysicalAdapterError {
    Lease(LeaseError),
    Journal(JournalError),
}

impl std::fmt::Display for PhysicalAdapterError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Lease(error) => write!(f, "{error}"),
            Self::Journal(error) => write!(f, "{error}"),
        }
    }
}

impl std::error::Error for PhysicalAdapterError {}

impl From<LeaseError> for PhysicalAdapterError {
    fn from(value: LeaseError) -> Self {
        Self::Lease(value)
    }
}

impl From<JournalError> for PhysicalAdapterError {
    fn from(value: JournalError) -> Self {
        Self::Journal(value)
    }
}

/// Physical adapter wrapper whose lifetime is bounded by exclusive ownership of
/// the same local journal root used by the crash-safe journal.
///
/// The wrapped adapter is deliberately not returned by value. Dropping this
/// wrapper drops the journaled adapter and the ownership lease together.
pub struct ExclusivePhysicalAdapter<A>
where
    A: EdgeAdapter,
{
    lease: ExecutorLease,
    journaled: JournaledAdapter<A, FileJournal>,
}

impl<A> ExclusivePhysicalAdapter<A>
where
    A: EdgeAdapter,
{
    pub fn acquire(
        inner: A,
        journal_root: impl AsRef<Path>,
    ) -> Result<Self, PhysicalAdapterError> {
        let lease = ExecutorLease::acquire(journal_root)?;
        let journal = FileJournal::open(lease.root())?;
        let journaled = JournaledAdapter::new_physical(inner, journal)?;
        Ok(Self { lease, journaled })
    }

    pub fn lease(&self) -> &ExecutorLease {
        &self.lease
    }

    pub fn journaled(&self) -> &JournaledAdapter<A, FileJournal> {
        &self.journaled
    }
}

impl<A> EdgeAdapter for ExclusivePhysicalAdapter<A>
where
    A: EdgeAdapter,
{
    fn name(&self) -> &str {
        self.journaled.name()
    }

    fn supports_capability(&self, capability: &str) -> bool {
        self.journaled.supports_capability(capability)
    }

    fn supports(&self, action: &ActionSpec) -> bool {
        self.journaled.supports(action)
    }

    fn execute(
        &mut self,
        idempotency_key: &str,
        action: &ActionSpec,
        timeout_ms: u64,
        now_micros: i64,
    ) -> CommandStatus {
        self.journaled
            .execute(idempotency_key, action, timeout_ms, now_micros)
    }

    fn verify(
        &mut self,
        idempotency_key: &str,
        expectations: &[OutcomeExpectation],
        policy: &OutcomePolicy,
        now_micros: i64,
    ) -> VerificationReport {
        self.journaled
            .verify(idempotency_key, expectations, policy, now_micros)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use hearth_edge::SyntheticAdapter;
    use std::sync::atomic::{AtomicU64, Ordering};

    static TEST_ROOT_COUNTER: AtomicU64 = AtomicU64::new(0);

    fn test_root(label: &str) -> PathBuf {
        let serial = TEST_ROOT_COUNTER.fetch_add(1, Ordering::Relaxed);
        std::env::temp_dir().join(format!(
            "hearth-edge-lease-{}-{label}-{serial}",
            std::process::id()
        ))
    }

    fn cleanup(path: &Path) {
        let _ = fs::remove_dir_all(path);
    }

    #[test]
    fn second_live_lease_for_same_root_is_rejected() {
        let root = test_root("exclusive");
        cleanup(&root);

        let first = ExecutorLease::acquire(&root).unwrap();
        let second = ExecutorLease::acquire(&root);
        assert!(matches!(second, Err(LeaseError::AlreadyOwned { .. })));

        drop(first);
        cleanup(&root);
    }

    #[test]
    fn release_allows_reacquisition_without_deleting_lockfile() {
        let root = test_root("reacquire");
        cleanup(&root);

        let first = ExecutorLease::acquire(&root).unwrap();
        let lock_path = first.lock_path().to_path_buf();
        assert!(lock_path.exists());
        drop(first);

        assert!(lock_path.exists());
        let second = ExecutorLease::acquire(&root).unwrap();
        assert_eq!(second.lock_path(), lock_path.as_path());
        drop(second);

        cleanup(&root);
    }

    #[test]
    fn stale_lockfile_contents_are_not_ownership() {
        let root = test_root("stale-file");
        cleanup(&root);
        fs::create_dir_all(&root).unwrap();
        fs::write(
            root.join(EXECUTOR_LOCK_FILE),
            "schema=1\npid=999999\n",
        )
        .unwrap();

        let lease = ExecutorLease::acquire(&root).unwrap();
        assert_eq!(lease.root(), fs::canonicalize(&root).unwrap());
        drop(lease);

        cleanup(&root);
    }

    #[cfg(unix)]
    #[test]
    fn physical_wrapper_holds_lease_for_its_full_lifetime() {
        let root = test_root("physical-wrapper");
        cleanup(&root);

        let inner = SyntheticAdapter::new(
            "synthetic",
            ["home.appliance.control".to_string()],
        );
        let wrapper = ExclusivePhysicalAdapter::acquire(inner, &root).unwrap();

        assert!(matches!(
            ExecutorLease::acquire(&root),
            Err(LeaseError::AlreadyOwned { .. })
        ));

        drop(wrapper);
        let replacement = ExecutorLease::acquire(&root).unwrap();
        drop(replacement);

        cleanup(&root);
    }
}
