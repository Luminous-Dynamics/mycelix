// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Evidence contract for re-derived Nix runtime closure contents.
//!
//! FORGE-004D2B3A does not read the Nix database and does not perform NAR
//! serialization itself. It defines the exact observations a producer must
//! supply after hashing the canonical NAR stream of every committed store root.
//! Auditor provenance is deliberately a later composition claim.

use mycelix_forge_core::{Digest, DigestAlgorithm};
use mycelix_forge_linux_isolation::{IsolationPolicyError, NixClosureManifest};
use serde::{de::Error as _, Deserialize, Deserializer, Serialize};
use std::collections::{BTreeMap, BTreeSet};
use thiserror::Error;

const OBSERVATION_DOMAIN_V1: &[u8] = b"mycelix-forge/runtime-closure-observation/v1\0";
const QUALIFIED_DOMAIN_V1: &[u8] = b"mycelix-forge/runtime-closure-qualified/v1\0";
const MAX_TEXT: usize = 4096;
const MAX_ITEMS: usize = 16384;

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize)]
pub struct NarPathObservation {
    store_path: String,
    nar_digest: Digest,
    nar_size: u64,
}

impl NarPathObservation {
    pub fn new(
        store_path: impl Into<String>,
        nar_digest: Digest,
        nar_size: u64,
    ) -> Result<Self, RuntimeClosureEvidenceError> {
        let store_path = store_path.into();
        validate_store_root(&store_path)?;
        if nar_size == 0 {
            return Err(RuntimeClosureEvidenceError::EmptyNar(store_path));
        }
        Ok(Self {
            store_path,
            nar_digest,
            nar_size,
        })
    }

    pub fn store_path(&self) -> &str {
        &self.store_path
    }

    pub fn nar_digest(&self) -> &Digest {
        &self.nar_digest
    }

    pub const fn nar_size(&self) -> u64 {
        self.nar_size
    }
}

impl<'de> Deserialize<'de> for NarPathObservation {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        #[derive(Deserialize)]
        struct Wire {
            store_path: String,
            nar_digest: Digest,
            nar_size: u64,
        }
        let wire = Wire::deserialize(deserializer)?;
        Self::new(wire.store_path, wire.nar_digest, wire.nar_size).map_err(D::Error::custom)
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct RuntimeClosureObservation {
    closure_digest: Digest,
    auditor_subject: Digest,
    paths: Vec<NarPathObservation>,
}

impl RuntimeClosureObservation {
    pub fn new(
        closure_digest: Digest,
        auditor_subject: Digest,
        mut paths: Vec<NarPathObservation>,
    ) -> Result<Self, RuntimeClosureEvidenceError> {
        if paths.is_empty() {
            return Err(RuntimeClosureEvidenceError::EmptyObservation);
        }
        if paths.len() > MAX_ITEMS {
            return Err(RuntimeClosureEvidenceError::TooManyPaths(paths.len()));
        }
        paths.sort();
        for pair in paths.windows(2) {
            if pair[0].store_path == pair[1].store_path {
                return Err(RuntimeClosureEvidenceError::DuplicateStorePath(
                    pair[0].store_path.clone(),
                ));
            }
        }
        Ok(Self {
            closure_digest,
            auditor_subject,
            paths,
        })
    }

    pub fn closure_digest(&self) -> &Digest {
        &self.closure_digest
    }

    pub fn auditor_subject(&self) -> &Digest {
        &self.auditor_subject
    }

    pub fn paths(&self) -> &[NarPathObservation] {
        &self.paths
    }

    pub fn canonical_bytes(&self) -> Result<Vec<u8>, RuntimeClosureEvidenceError> {
        let mut out = Vec::new();
        out.extend_from_slice(OBSERVATION_DOMAIN_V1);
        push_digest(&mut out, &self.closure_digest)?;
        push_digest(&mut out, &self.auditor_subject)?;
        push_count(&mut out, self.paths.len(), "NAR observations")?;
        for path in &self.paths {
            push_string(&mut out, path.store_path(), "store path")?;
            push_digest(&mut out, path.nar_digest())?;
            out.extend_from_slice(&path.nar_size.to_be_bytes());
        }
        Ok(out)
    }

    pub fn digest(&self, algorithm: DigestAlgorithm) -> Result<Digest, RuntimeClosureEvidenceError> {
        Ok(Digest::of_bytes(algorithm, &self.canonical_bytes()?))
    }
}

impl<'de> Deserialize<'de> for RuntimeClosureObservation {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        #[derive(Deserialize)]
        struct Wire {
            closure_digest: Digest,
            auditor_subject: Digest,
            paths: Vec<NarPathObservation>,
        }
        let wire = Wire::deserialize(deserializer)?;
        let mut canonical = wire.paths.clone();
        canonical.sort();
        if canonical != wire.paths {
            return Err(D::Error::custom("NAR observations are not canonical"));
        }
        Self::new(wire.closure_digest, wire.auditor_subject, wire.paths).map_err(D::Error::custom)
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct QualifiedRuntimeClosureEvidence {
    closure_digest: Digest,
    auditor_subject: Digest,
    observation_digest: Digest,
    evidence_digest: Digest,
}

impl QualifiedRuntimeClosureEvidence {
    pub fn closure_digest(&self) -> &Digest {
        &self.closure_digest
    }

    pub fn auditor_subject(&self) -> &Digest {
        &self.auditor_subject
    }

    pub fn observation_digest(&self) -> &Digest {
        &self.observation_digest
    }

    pub fn evidence_digest(&self) -> &Digest {
        &self.evidence_digest
    }
}

pub fn qualify_runtime_closure(
    expected: &NixClosureManifest,
    observation: &RuntimeClosureObservation,
) -> Result<QualifiedRuntimeClosureEvidence, RuntimeClosureEvidenceError> {
    let expected_closure = expected.digest(observation.closure_digest.algorithm())?;
    if expected_closure != observation.closure_digest {
        return Err(RuntimeClosureEvidenceError::ClosureDigestMismatch);
    }
    if expected.entries().len() != observation.paths.len() {
        return Err(RuntimeClosureEvidenceError::ClosureCardinalityMismatch {
            expected: expected.entries().len(),
            observed: observation.paths.len(),
        });
    }

    let observed: BTreeMap<&str, &NarPathObservation> = observation
        .paths
        .iter()
        .map(|path| (path.store_path(), path))
        .collect();
    let expected_paths: BTreeSet<&str> = expected
        .entries()
        .iter()
        .map(|entry| entry.store_path())
        .collect();
    let observed_paths: BTreeSet<&str> = observed.keys().copied().collect();
    if expected_paths != observed_paths {
        return Err(RuntimeClosureEvidenceError::StorePathSetMismatch);
    }

    for entry in expected.entries() {
        let actual = observed
            .get(entry.store_path())
            .ok_or(RuntimeClosureEvidenceError::StorePathSetMismatch)?;
        if actual.nar_digest() != entry.nar_hash() {
            return Err(RuntimeClosureEvidenceError::NarDigestMismatch(
                entry.store_path().to_owned(),
            ));
        }
    }

    let observation_digest = observation.digest(DigestAlgorithm::Sha256)?;
    let closure_digest = expected.digest(DigestAlgorithm::Sha256)?;
    let mut evidence = Vec::new();
    evidence.extend_from_slice(QUALIFIED_DOMAIN_V1);
    push_digest(&mut evidence, &closure_digest)?;
    push_digest(&mut evidence, observation.auditor_subject())?;
    push_digest(&mut evidence, &observation_digest)?;
    let evidence_digest = Digest::of_bytes(DigestAlgorithm::Sha256, &evidence);

    Ok(QualifiedRuntimeClosureEvidence {
        closure_digest,
        auditor_subject: observation.auditor_subject().clone(),
        observation_digest,
        evidence_digest,
    })
}

fn validate_store_root(value: &str) -> Result<(), RuntimeClosureEvidenceError> {
    if value.is_empty() || value.len() > MAX_TEXT || value.contains('\0') {
        return Err(RuntimeClosureEvidenceError::InvalidStorePath(value.to_owned()));
    }
    let Some(suffix) = value.strip_prefix("/nix/store/") else {
        return Err(RuntimeClosureEvidenceError::InvalidStorePath(value.to_owned()));
    };
    if suffix.is_empty() || suffix.contains('/') || suffix == "." || suffix == ".." {
        return Err(RuntimeClosureEvidenceError::InvalidStorePath(value.to_owned()));
    }
    Ok(())
}

fn push_count(
    out: &mut Vec<u8>,
    count: usize,
    field: &'static str,
) -> Result<(), RuntimeClosureEvidenceError> {
    let count = u16::try_from(count)
        .map_err(|_| RuntimeClosureEvidenceError::CanonicalFieldTooLarge(field))?;
    out.extend_from_slice(&count.to_be_bytes());
    Ok(())
}

fn push_string(
    out: &mut Vec<u8>,
    value: &str,
    field: &'static str,
) -> Result<(), RuntimeClosureEvidenceError> {
    let len = u16::try_from(value.len())
        .map_err(|_| RuntimeClosureEvidenceError::CanonicalFieldTooLarge(field))?;
    out.extend_from_slice(&len.to_be_bytes());
    out.extend_from_slice(value.as_bytes());
    Ok(())
}

fn push_digest(out: &mut Vec<u8>, digest: &Digest) -> Result<(), RuntimeClosureEvidenceError> {
    push_string(out, digest.algorithm().id(), "digest algorithm")?;
    let len = u16::try_from(digest.as_bytes().len())
        .map_err(|_| RuntimeClosureEvidenceError::CanonicalFieldTooLarge("digest"))?;
    out.extend_from_slice(&len.to_be_bytes());
    out.extend_from_slice(digest.as_bytes());
    Ok(())
}

#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum RuntimeClosureEvidenceError {
    #[error(transparent)]
    Isolation(#[from] IsolationPolicyError),
    #[error("runtime-closure observation may not be empty")]
    EmptyObservation,
    #[error("too many NAR path observations: {0}")]
    TooManyPaths(usize),
    #[error("invalid Nix store root: {0}")]
    InvalidStorePath(String),
    #[error("empty NAR stream for {0}")]
    EmptyNar(String),
    #[error("duplicate observed Nix store path: {0}")]
    DuplicateStorePath(String),
    #[error("runtime closure digest does not match committed closure")]
    ClosureDigestMismatch,
    #[error("runtime closure cardinality mismatch: expected {expected}, observed {observed}")]
    ClosureCardinalityMismatch { expected: usize, observed: usize },
    #[error("runtime closure store-path set differs from committed closure")]
    StorePathSetMismatch,
    #[error("re-derived NAR digest mismatch for {0}")]
    NarDigestMismatch(String),
    #[error("canonical field is too large: {0}")]
    CanonicalFieldTooLarge(&'static str),
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_forge_linux_isolation::NixClosureEntry;

    fn digest(byte: u8) -> Digest {
        Digest::new(DigestAlgorithm::Sha256, vec![byte; 32]).unwrap()
    }

    fn closure() -> NixClosureManifest {
        NixClosureManifest::new(vec![
            NixClosureEntry::new("/nix/store/aaaa-auditor", digest(1)).unwrap(),
            NixClosureEntry::new("/nix/store/bbbb-gittuf", digest(2)).unwrap(),
        ])
        .unwrap()
    }

    fn observation(expected: &NixClosureManifest) -> RuntimeClosureObservation {
        RuntimeClosureObservation::new(
            expected.digest(DigestAlgorithm::Sha256).unwrap(),
            digest(9),
            vec![
                NarPathObservation::new("/nix/store/aaaa-auditor", digest(1), 1234).unwrap(),
                NarPathObservation::new("/nix/store/bbbb-gittuf", digest(2), 5678).unwrap(),
            ],
        )
        .unwrap()
    }

    #[test]
    fn exact_rederived_closure_qualifies() {
        let expected = closure();
        let qualified = qualify_runtime_closure(&expected, &observation(&expected)).unwrap();
        assert_eq!(
            qualified.closure_digest(),
            &expected.digest(DigestAlgorithm::Sha256).unwrap()
        );
    }

    #[test]
    fn one_byte_nar_digest_change_fails_closed() {
        let expected = closure();
        let mut observed = observation(&expected);
        observed.paths[1].nar_digest = digest(3);
        assert!(matches!(
            qualify_runtime_closure(&expected, &observed),
            Err(RuntimeClosureEvidenceError::NarDigestMismatch(path))
                if path == "/nix/store/bbbb-gittuf"
        ));
    }

    #[test]
    fn missing_path_fails_closed() {
        let expected = closure();
        let observed = RuntimeClosureObservation::new(
            expected.digest(DigestAlgorithm::Sha256).unwrap(),
            digest(9),
            vec![NarPathObservation::new("/nix/store/aaaa-auditor", digest(1), 1234).unwrap()],
        )
        .unwrap();
        assert!(matches!(
            qualify_runtime_closure(&expected, &observed),
            Err(RuntimeClosureEvidenceError::ClosureCardinalityMismatch { .. })
        ));
    }

    #[test]
    fn extra_path_fails_closed() {
        let expected = closure();
        let observed = RuntimeClosureObservation::new(
            expected.digest(DigestAlgorithm::Sha256).unwrap(),
            digest(9),
            vec![
                NarPathObservation::new("/nix/store/aaaa-auditor", digest(1), 1234).unwrap(),
                NarPathObservation::new("/nix/store/bbbb-gittuf", digest(2), 5678).unwrap(),
                NarPathObservation::new("/nix/store/cccc-extra", digest(3), 100).unwrap(),
            ],
        )
        .unwrap();
        assert!(matches!(
            qualify_runtime_closure(&expected, &observed),
            Err(RuntimeClosureEvidenceError::ClosureCardinalityMismatch { .. })
        ));
    }

    #[test]
    fn closure_commitment_mismatch_fails_closed() {
        let expected = closure();
        let mut observed = observation(&expected);
        observed.closure_digest = digest(8);
        assert!(matches!(
            qualify_runtime_closure(&expected, &observed),
            Err(RuntimeClosureEvidenceError::ClosureDigestMismatch)
        ));
    }

    #[test]
    fn duplicate_paths_are_rejected() {
        let expected = closure();
        let result = RuntimeClosureObservation::new(
            expected.digest(DigestAlgorithm::Sha256).unwrap(),
            digest(9),
            vec![
                NarPathObservation::new("/nix/store/aaaa-auditor", digest(1), 1).unwrap(),
                NarPathObservation::new("/nix/store/aaaa-auditor", digest(1), 1).unwrap(),
            ],
        );
        assert!(matches!(
            result,
            Err(RuntimeClosureEvidenceError::DuplicateStorePath(_))
        ));
    }

    #[test]
    fn serde_rejects_empty_nar_stream() {
        let value = serde_json::json!({
            "store_path": "/nix/store/aaaa-auditor",
            "nar_digest": digest(1),
            "nar_size": 0
        });
        assert!(serde_json::from_value::<NarPathObservation>(value).is_err());
    }

    #[test]
    fn serde_rejects_noncanonical_path_order() {
        let expected = closure();
        let observation = observation(&expected);
        let mut value = serde_json::to_value(observation).unwrap();
        value["paths"].as_array_mut().unwrap().reverse();
        assert!(serde_json::from_value::<RuntimeClosureObservation>(value).is_err());
    }
}
