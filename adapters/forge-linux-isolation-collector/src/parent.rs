// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

use crate::{inside::namespace_snapshot, IsolationCollectorError};
use mycelix_forge_core::{Digest, DigestAlgorithm};
use mycelix_forge_linux_isolation_evidence::{NamespaceSnapshot, ParentIsolationEvidence};
use std::{fs, path::PathBuf};

/// Parent-side observation captured while the bubblewrap child is held behind
/// `--block-fd` after namespace/mount construction but before probe execution.
#[derive(Clone, Debug)]
pub struct PendingParentObservation {
    child_pid: u32,
    host_namespaces: NamespaceSnapshot,
    child_namespaces: NamespaceSnapshot,
    child_mountinfo_digest: Digest,
    start_status: Vec<u8>,
    start_status_commitment: Digest,
}

/// Parse bubblewrap JSON status output and independently inspect the reported
/// child via the host `/proc/<pid>` view.
///
/// The launcher is expected to use `--json-status-fd` and `--block-fd`; this
/// function must run before the block FD is released.
pub fn observe_parent_start(
    status_stream: &[u8],
) -> Result<PendingParentObservation, IsolationCollectorError> {
    let child_pid = find_child_pid(status_stream)?;
    let host_namespaces = namespace_snapshot(std::path::Path::new("/proc/self"))?;
    let child_root = PathBuf::from(format!("/proc/{child_pid}"));
    let child_namespaces = namespace_snapshot(&child_root)?;
    let mountinfo = fs::read(child_root.join("mountinfo"))?;
    let start_status_commitment = Digest::of_bytes(DigestAlgorithm::Sha256, status_stream);

    Ok(PendingParentObservation {
        child_pid,
        host_namespaces,
        child_namespaces,
        child_mountinfo_digest: Digest::of_bytes(DigestAlgorithm::Sha256, &mountinfo),
        start_status: status_stream.to_vec(),
        start_status_commitment,
    })
}

impl PendingParentObservation {
    /// Exact child PID reported by bubblewrap while the child is still held.
    ///
    /// The PID is diagnostic/correlation material, not a stable security
    /// identity. D3B2 must acquire a pidfd before releasing the block FD.
    pub const fn child_pid(&self) -> u32 {
        self.child_pid
    }

    /// Commitment to the exact bubblewrap status bytes that identified the
    /// blocked child before it was released.
    pub fn start_status_commitment(&self) -> &Digest {
        &self.start_status_commitment
    }

    /// Complete the parent channel after bubblewrap emits its exit status.
    pub fn finish(
        self,
        exit_status_stream: &[u8],
    ) -> Result<ParentIsolationEvidence, IsolationCollectorError> {
        let exit_code = find_exit_code(exit_status_stream)?;
        let mut status_bytes =
            Vec::with_capacity(self.start_status.len() + exit_status_stream.len());
        status_bytes.extend_from_slice(&self.start_status);
        status_bytes.extend_from_slice(exit_status_stream);
        let status_commitment = Digest::of_bytes(DigestAlgorithm::Sha256, &status_bytes);

        Ok(ParentIsolationEvidence::new(
            self.host_namespaces,
            self.child_namespaces,
            self.child_mountinfo_digest,
            status_commitment,
            exit_code,
        )?)
    }
}

fn find_child_pid(stream: &[u8]) -> Result<u32, IsolationCollectorError> {
    for value in parse_json_lines(stream)? {
        if let Some(pid) = value.get("child-pid").and_then(serde_json::Value::as_u64) {
            return u32::try_from(pid).map_err(|_| IsolationCollectorError::InvalidChildPid(pid));
        }
    }
    Err(IsolationCollectorError::MissingChildPid)
}

fn find_exit_code(stream: &[u8]) -> Result<i32, IsolationCollectorError> {
    for value in parse_json_lines(stream)?.into_iter().rev() {
        if let Some(code) = value.get("exit-code").and_then(serde_json::Value::as_i64) {
            return i32::try_from(code).map_err(|_| IsolationCollectorError::InvalidExitCode(code));
        }
    }
    Err(IsolationCollectorError::MissingExitCode)
}

fn parse_json_lines(stream: &[u8]) -> Result<Vec<serde_json::Value>, IsolationCollectorError> {
    let text = std::str::from_utf8(stream).map_err(|_| IsolationCollectorError::NonUtf8StatusStream)?;
    let mut values = Vec::new();
    for line in text.lines().filter(|line| !line.trim().is_empty()) {
        values.push(serde_json::from_str(line)?);
    }
    Ok(values)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn status_parser_finds_child_pid_among_multiple_records() {
        let stream = b"{\"foo\":1}\n{\"child-pid\":4242}\n";
        assert_eq!(find_child_pid(stream).unwrap(), 4242);
    }

    #[test]
    fn exit_parser_uses_final_exit_record() {
        let stream = b"{\"exit-code\":9}\n{\"exit-code\":0}\n";
        assert_eq!(find_exit_code(stream).unwrap(), 0);
    }

    #[test]
    fn missing_child_pid_fails_closed() {
        assert!(matches!(
            find_child_pid(b"{\"foo\":1}\n"),
            Err(IsolationCollectorError::MissingChildPid)
        ));
    }
}
