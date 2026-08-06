// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Process-independent optimistic transactions for small append-only JSON journals.
//!
//! The in-memory write lock used by a single server process is not sufficient
//! when operators accidentally start two API or migration processes against the
//! same durable files. This helper combines an exclusive create-new lock file,
//! an on-disk expected-state comparison, atomic replacement, file fsync, and
//! parent-directory fsync. A stale process therefore fails closed instead of
//! silently overwriting a successor committed by another process.
//!
//! Lock files are deliberately not reclaimed automatically. A crashed writer
//! may leave a lock behind, but guessing that a lock is stale is less safe than
//! requiring explicit operator recovery after inspecting the journal and lock
//! metadata.

use crate::{Error, Result};
use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize, de::DeserializeOwned};
use std::fs::{self, File, OpenOptions};
use std::io::Write;
use std::path::{Path, PathBuf};
use uuid::Uuid;

const MAX_TRANSACTION_LOCK_BYTES: u64 = 16 * 1024;

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct TransactionLockRecord {
    token: Uuid,
    process_id: u32,
    created_at: DateTime<Utc>,
    target: String,
}

struct TransactionLockGuard {
    path: PathBuf,
    token: Uuid,
}

impl TransactionLockGuard {
    fn acquire(target: &Path) -> Result<Self> {
        let parent = target.parent().unwrap_or_else(|| Path::new("."));
        fs::create_dir_all(parent)?;
        let file_name = target
            .file_name()
            .and_then(|name| name.to_str())
            .unwrap_or("journal");
        let path = parent.join(format!(".{file_name}.transaction.lock"));
        let token = Uuid::new_v4();
        let record = TransactionLockRecord {
            token,
            process_id: std::process::id(),
            created_at: Utc::now(),
            target: target.display().to_string(),
        };
        let bytes = serde_json::to_vec_pretty(&record)?;
        if bytes.len() as u64 > MAX_TRANSACTION_LOCK_BYTES {
            return Err(Error::Storage(
                "transaction lock metadata exceeds its size limit".to_string(),
            ));
        }
        match OpenOptions::new().write(true).create_new(true).open(&path) {
            Ok(mut file) => {
                file.write_all(&bytes)?;
                file.sync_all()?;
                File::open(parent)?.sync_all()?;
                Ok(Self { path, token })
            }
            Err(error) if error.kind() == std::io::ErrorKind::AlreadyExists => {
                let detail = read_existing_lock(&path)
                    .unwrap_or_else(|read_error| format!("unreadable lock metadata: {read_error}"));
                Err(Error::Storage(format!(
                    "durable journal is locked by another process ({detail}); inspect and remove {} only after proving no writer is active",
                    path.display()
                )))
            }
            Err(error) => Err(error.into()),
        }
    }
}

impl Drop for TransactionLockGuard {
    fn drop(&mut self) {
        let remove = (|| -> Result<()> {
            let bytes = fs::read(&self.path)?;
            let record: TransactionLockRecord = serde_json::from_slice(&bytes)?;
            if record.token != self.token {
                return Err(Error::VerificationFailed(
                    "transaction lock ownership changed before release".to_string(),
                ));
            }
            fs::remove_file(&self.path)?;
            if let Some(parent) = self.path.parent() {
                File::open(parent)?.sync_all()?;
            }
            Ok(())
        })();
        if let Err(error) = remove {
            eprintln!(
                "failed to release durable journal transaction lock {}: {error}",
                self.path.display()
            );
        }
    }
}

fn read_existing_lock(path: &Path) -> Result<String> {
    let metadata = fs::symlink_metadata(path)?;
    if metadata.file_type().is_symlink() || !metadata.is_file() {
        return Err(Error::Storage(
            "transaction lock must be a regular non-symbolic-link file".to_string(),
        ));
    }
    if metadata.len() > MAX_TRANSACTION_LOCK_BYTES {
        return Err(Error::Storage(
            "transaction lock metadata exceeds its size limit".to_string(),
        ));
    }
    let record: TransactionLockRecord = serde_json::from_slice(&fs::read(path)?)?;
    Ok(format!(
        "pid {}, created {}, token {}",
        record.process_id, record.created_at, record.token
    ))
}

/// Atomically replace a JSON array after proving the durable file still equals
/// the caller's expected in-memory prefix.
pub(crate) fn persist_json_vec_transaction<T>(
    path: &Path,
    expected: &[T],
    next: &[T],
    maximum_bytes: u64,
    label: &str,
) -> Result<()>
where
    T: Clone + PartialEq + Serialize + DeserializeOwned,
{
    let _guard = TransactionLockGuard::acquire(path)?;
    let current = read_json_vec::<T>(path, maximum_bytes, label)?;
    if current != expected {
        return Err(Error::Storage(format!(
            "{label} changed on disk after this process loaded it; refusing a stale overwrite"
        )));
    }

    let bytes = serde_json::to_vec_pretty(next)?;
    if bytes.len() as u64 > maximum_bytes {
        return Err(Error::Storage(format!(
            "{label} exceeds {maximum_bytes} bytes"
        )));
    }
    atomic_replace(path, &bytes, label)
}

fn read_json_vec<T>(path: &Path, maximum_bytes: u64, label: &str) -> Result<Vec<T>>
where
    T: DeserializeOwned,
{
    if !path.exists() {
        return Ok(Vec::new());
    }
    let metadata = fs::symlink_metadata(path)?;
    if metadata.file_type().is_symlink() || !metadata.is_file() {
        return Err(Error::Storage(format!(
            "{label} must be a regular non-symbolic-link file"
        )));
    }
    if metadata.len() > maximum_bytes {
        return Err(Error::Storage(format!(
            "{label} exceeds {maximum_bytes} bytes"
        )));
    }
    let bytes = fs::read(path)?;
    if bytes.is_empty() {
        Ok(Vec::new())
    } else {
        Ok(serde_json::from_slice(&bytes)?)
    }
}

fn atomic_replace(path: &Path, bytes: &[u8], label: &str) -> Result<()> {
    let parent = path.parent().unwrap_or_else(|| Path::new("."));
    fs::create_dir_all(parent)?;
    let temporary = parent.join(format!(
        ".{}.{}.tmp",
        path.file_name()
            .and_then(|name| name.to_str())
            .unwrap_or(label),
        Uuid::new_v4()
    ));
    let write_result = (|| -> Result<()> {
        let mut file = OpenOptions::new()
            .write(true)
            .create_new(true)
            .open(&temporary)?;
        file.write_all(bytes)?;
        file.sync_all()?;
        fs::rename(&temporary, path)?;
        File::open(parent)?.sync_all()?;
        Ok(())
    })();
    if write_result.is_err() {
        let _ = fs::remove_file(&temporary);
    }
    write_result
}

#[cfg(test)]
mod tests {
    use super::*;
    use tempfile::tempdir;

    #[test]
    fn stale_writer_is_rejected() {
        let directory = tempdir().unwrap();
        let path = directory.path().join("journal.json");
        persist_json_vec_transaction(&path, &[] as &[u64], &[1], 1024, "test journal").unwrap();
        let error = persist_json_vec_transaction(&path, &[] as &[u64], &[2], 1024, "test journal")
            .unwrap_err();
        assert!(error.to_string().contains("changed on disk"));
    }

    #[test]
    fn active_lock_blocks_second_writer() {
        let directory = tempdir().unwrap();
        let path = directory.path().join("journal.json");
        let _guard = TransactionLockGuard::acquire(&path).unwrap();
        let error = persist_json_vec_transaction(&path, &[] as &[u64], &[1], 1024, "test journal")
            .unwrap_err();
        assert!(error.to_string().contains("locked by another process"));
    }
}
