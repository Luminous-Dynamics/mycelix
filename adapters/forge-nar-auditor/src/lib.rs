// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Independent canonical-NAR producer for Forge runtime-closure evidence.
//!
//! This crate hashes the actual filesystem contents of each committed Nix
//! store root. It does not query Nix store metadata for `narHash` values.

use mycelix_forge_core::{Digest, DigestAlgorithm};
use mycelix_forge_linux_isolation::{NixClosureEntry, NixClosureManifest};
use mycelix_forge_runtime_closure_evidence::{
    qualify_runtime_closure, NarPathObservation, QualifiedRuntimeClosureEvidence,
    RuntimeClosureEvidenceError, RuntimeClosureObservation,
};
use nix_archive::nar::{hash_path, CaseHack};
use std::{collections::BTreeMap, path::Path};
use thiserror::Error;

const AUDITOR_DOMAIN_V1: &[u8] = b"mycelix-forge/nar-auditor/v1\0";
pub const AUDITOR_IMPLEMENTATION: &str = "cachix/nix-archive";
pub const AUDITOR_VERSION: &str = "0.6.0";
pub const AUDITOR_UPSTREAM_COMMIT: &str = "b4ecefa4c0c47e7ae7446cd59e901aa2d95f8414";
pub const AUDITOR_CASE_HACK: &str = "disabled";
pub const AUDITOR_NAR_HASH: &str = "sha256";

pub fn auditor_subject() -> Digest {
    let mut bytes = Vec::new();
    bytes.extend_from_slice(AUDITOR_DOMAIN_V1);
    for value in [
        AUDITOR_IMPLEMENTATION,
        AUDITOR_VERSION,
        AUDITOR_UPSTREAM_COMMIT,
        AUDITOR_NAR_HASH,
        AUDITOR_CASE_HACK,
    ] {
        let len = u16::try_from(value.len()).expect("auditor constants fit v1 length prefix");
        bytes.extend_from_slice(&len.to_be_bytes());
        bytes.extend_from_slice(value.as_bytes());
    }
    Digest::of_bytes(DigestAlgorithm::Sha256, &bytes)
}

pub fn audit_runtime_closure(
    expected: &NixClosureManifest,
) -> Result<RuntimeClosureObservation, NarAuditorError> {
    for entry in expected.entries() {
        if entry.nar_hash().algorithm() != DigestAlgorithm::Sha256 {
            return Err(NarAuditorError::UnsupportedNarAlgorithm {
                path: entry.store_path().to_owned(),
                algorithm: entry.nar_hash().algorithm(),
            });
        }
    }

    let first = audit_pass(expected.entries().iter())?;
    let second = audit_pass(expected.entries().iter().rev())?;
    ensure_stable(&first, &second)?;

    let paths = first.into_values().collect();
    Ok(RuntimeClosureObservation::new(
        expected.digest(DigestAlgorithm::Sha256)?,
        auditor_subject(),
        paths,
    )?)
}

pub fn audit_and_qualify_runtime_closure(
    expected: &NixClosureManifest,
) -> Result<QualifiedRuntimeClosureEvidence, NarAuditorError> {
    let observation = audit_runtime_closure(expected)?;
    Ok(qualify_runtime_closure(expected, &observation)?)
}

fn audit_pass<'a>(
    entries: impl Iterator<Item = &'a NixClosureEntry>,
) -> Result<BTreeMap<String, NarPathObservation>, NarAuditorError> {
    let mut observed = BTreeMap::new();
    for entry in entries {
        let observation = audit_entry_at(entry, Path::new(entry.store_path()))?;
        if observed
            .insert(entry.store_path().to_owned(), observation)
            .is_some()
        {
            return Err(NarAuditorError::DuplicateStorePath(
                entry.store_path().to_owned(),
            ));
        }
    }
    Ok(observed)
}

fn audit_entry_at(
    entry: &NixClosureEntry,
    filesystem_path: &Path,
) -> Result<NarPathObservation, NarAuditorError> {
    if entry.nar_hash().algorithm() != DigestAlgorithm::Sha256 {
        return Err(NarAuditorError::UnsupportedNarAlgorithm {
            path: entry.store_path().to_owned(),
            algorithm: entry.nar_hash().algorithm(),
        });
    }
    let hash = hash_path(filesystem_path, CaseHack::Disabled)?;
    let digest = Digest::new(DigestAlgorithm::Sha256, hash.sha256.to_vec())?;
    Ok(NarPathObservation::new(
        entry.store_path(),
        digest,
        hash.size,
    )?)
}

fn ensure_stable(
    first: &BTreeMap<String, NarPathObservation>,
    second: &BTreeMap<String, NarPathObservation>,
) -> Result<(), NarAuditorError> {
    if first.len() != second.len() {
        return Err(NarAuditorError::ClosureChangedDuringAudit);
    }
    for (path, before) in first {
        let Some(after) = second.get(path) else {
            return Err(NarAuditorError::ClosureChangedDuringAudit);
        };
        if before.nar_digest() != after.nar_digest() || before.nar_size() != after.nar_size() {
            return Err(NarAuditorError::PathChangedDuringAudit(path.clone()));
        }
    }
    Ok(())
}

#[derive(Debug, Error)]
pub enum NarAuditorError {
    #[error(transparent)]
    Core(#[from] mycelix_forge_core::ForgeCoreError),
    #[error(transparent)]
    Isolation(#[from] mycelix_forge_linux_isolation::IsolationPolicyError),
    #[error(transparent)]
    Evidence(#[from] RuntimeClosureEvidenceError),
    #[error(transparent)]
    Nar(#[from] nix_archive::nar::Error),
    #[error("NAR auditor v1 supports only SHA-256; {path} uses {algorithm}")]
    UnsupportedNarAlgorithm {
        path: String,
        algorithm: DigestAlgorithm,
    },
    #[error("duplicate store path during runtime audit: {0}")]
    DuplicateStorePath(String),
    #[error("runtime closure changed during the two-pass audit")]
    ClosureChangedDuringAudit,
    #[error("runtime store path changed during the two-pass audit: {0}")]
    PathChangedDuringAudit(String),
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::fs;
    use tempfile::tempdir;

    #[test]
    fn descriptor_is_sensitive_to_semantics() {
        let current = auditor_subject();
        let mut alternate = Vec::new();
        alternate.extend_from_slice(AUDITOR_DOMAIN_V1);
        for value in [
            AUDITOR_IMPLEMENTATION,
            AUDITOR_VERSION,
            AUDITOR_UPSTREAM_COMMIT,
            AUDITOR_NAR_HASH,
            "enabled",
        ] {
            alternate.extend_from_slice(&(value.len() as u16).to_be_bytes());
            alternate.extend_from_slice(value.as_bytes());
        }
        assert_ne!(
            current,
            Digest::of_bytes(DigestAlgorithm::Sha256, &alternate)
        );
    }

    #[test]
    fn hashes_actual_filesystem_as_canonical_nar() {
        let temp = tempdir().unwrap();
        fs::write(temp.path().join("hello"), b"world\n").unwrap();
        let direct = hash_path(temp.path(), CaseHack::Disabled).unwrap();
        let expected_digest = Digest::new(DigestAlgorithm::Sha256, direct.sha256.to_vec()).unwrap();
        let entry = NixClosureEntry::new("/nix/store/aaaa-test", expected_digest.clone()).unwrap();
        let observed = audit_entry_at(&entry, temp.path()).unwrap();
        assert_eq!(observed.nar_digest(), &expected_digest);
        assert_eq!(observed.nar_size(), direct.size);
    }

    #[test]
    fn changed_path_is_detected_between_passes() {
        let before = BTreeMap::from([(
            "/nix/store/aaaa-test".to_owned(),
            NarPathObservation::new(
                "/nix/store/aaaa-test",
                Digest::new(DigestAlgorithm::Sha256, vec![1; 32]).unwrap(),
                10,
            )
            .unwrap(),
        )]);
        let after = BTreeMap::from([(
            "/nix/store/aaaa-test".to_owned(),
            NarPathObservation::new(
                "/nix/store/aaaa-test",
                Digest::new(DigestAlgorithm::Sha256, vec![2; 32]).unwrap(),
                10,
            )
            .unwrap(),
        )]);
        assert!(matches!(
            ensure_stable(&before, &after),
            Err(NarAuditorError::PathChangedDuringAudit(_))
        ));
    }

    #[test]
    fn unsupported_nar_algorithm_fails_closed() {
        let entry = NixClosureEntry::new(
            "/nix/store/aaaa-test",
            Digest::new(DigestAlgorithm::Blake3_256, vec![3; 32]).unwrap(),
        )
        .unwrap();
        let temp = tempdir().unwrap();
        assert!(matches!(
            audit_entry_at(&entry, temp.path()),
            Err(NarAuditorError::UnsupportedNarAlgorithm { .. })
        ));
    }
}
