// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Typed, fail-closed manifests for scientific benchmark runs.
//!
//! This module complements the canonical append-only scientific-event kernel;
//! it does not replace signed events, authority receipts, or artifact storage.
//! A validated manifest is intended to be serialized as a content-addressed
//! EvidenceArtifact and attached to a computational-reproduction attestation.
//!
//! Manifest validation checks the declared evidence contract. It does not prove
//! that a model is physically true, that a reviewer is honest, or that remote
//! artifacts match their declared hashes. Use verify_local_artifact for bytes
//! available in the local filesystem and the signed event/authority path for
//! attributable publication and review.

use crate::scientific_events::ContentHash;
use crate::{Error, Result};
use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};
use std::collections::HashSet;
use std::fs::File;
use std::io::{BufReader, Read};
use std::path::Path;

pub const SCIENTIFIC_BENCHMARK_MANIFEST_PROTOCOL: &str =
    "mycelix-desci-scientific-benchmark-manifest";
pub const SCIENTIFIC_BENCHMARK_MANIFEST_VERSION: u16 = 1;

/// Lifecycle state of a benchmark result. IndependentlyReproduced is a
/// recorded assertion whose identity and authorization must still be checked
/// through the canonical signed-event and authority-receipt system.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum BenchmarkStatus {
    Proposed,
    NotRun,
    LocallyVerified,
    IndependentlyReproduced,
    ReferenceMismatch,
    ExecutionFailed,
}

/// Purpose of an immutable input/output artifact binding.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum BenchmarkArtifactRole {
    Dataset,
    DatasetMetadata,
    Configuration,
    SourceDiff,
    Result,
    ReferenceResult,
    DiagnosticLog,
    TestReport,
    Other(String),
}

/// Content-addressed artifact reference. content_hash is the BLAKE3 digest
/// used by Mycelix-DeSci's canonical scientific-event kernel.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BenchmarkArtifactBinding {
    pub name: String,
    pub role: BenchmarkArtifactRole,
    pub locator: String,
    pub media_type: String,
    pub content_hash: ContentHash,
    pub size_bytes: u64,
}

impl BenchmarkArtifactBinding {
    fn validate(&self, label: &str) -> Result<()> {
        validate_nonempty(&self.name, &format!("{label} artifact name"))?;
        validate_nonempty(&self.locator, &format!("{label} artifact locator"))?;
        validate_nonempty(&self.media_type, &format!("{label} artifact media type"))
    }
}

/// Exact source tree used to produce the run. Verified outcomes require a
/// committed, clean tree so a branch name alone cannot stand in for provenance.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BenchmarkSourceRevision {
    pub repository_uri: String,
    pub git_commit: String,
    pub working_tree_clean: bool,
}

impl BenchmarkSourceRevision {
    fn validate(&self) -> Result<()> {
        validate_nonempty(&self.repository_uri, "source repository URI")?;
        if !matches!(self.git_commit.len(), 40 | 64)
            || !self.git_commit.bytes().all(|byte| byte.is_ascii_hexdigit())
        {
            return Err(Error::Validation(
                "source revision must be a full 40- or 64-character hexadecimal commit".into(),
            ));
        }
        Ok(())
    }
}

/// Minimal execution environment required to reproduce a scientific run.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BenchmarkEnvironment {
    pub toolchain: String,
    pub target: String,
    pub command: String,
    pub dependency_lock_digest: Option<ContentHash>,
}

impl BenchmarkEnvironment {
    fn validate(&self) -> Result<()> {
        validate_nonempty(&self.toolchain, "toolchain")?;
        validate_nonempty(&self.target, "execution target")?;
        validate_nonempty(&self.command, "reproduction command")
    }
}

/// Optional independently published reference against which outputs are
/// compared. A URL identifies the reference; the referenced artifact should
/// also be bound in input_artifacts when the run consumes its bytes.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BenchmarkReference {
    pub name: String,
    pub version: String,
    pub locator: String,
}

impl BenchmarkReference {
    fn validate(&self) -> Result<()> {
        validate_nonempty(&self.name, "reference name")?;
        validate_nonempty(&self.version, "reference version")?;
        validate_nonempty(&self.locator, "reference locator")
    }
}

/// A scalar result. When a reference value is supplied, its absolute tolerance
/// must be supplied too. The tolerance is a benchmark contract, not inferred
/// after seeing the observation.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BenchmarkMetric {
    pub name: String,
    pub observed_value: f64,
    pub unit: Option<String>,
    pub reference_value: Option<f64>,
    pub absolute_tolerance: Option<f64>,
}

impl BenchmarkMetric {
    fn validate(&self) -> Result<()> {
        validate_nonempty(&self.name, "metric name")?;
        if !self.observed_value.is_finite() {
            return Err(Error::Validation(format!(
                "metric '{}' has a non-finite observed value",
                self.name
            )));
        }
        if self.unit.as_deref().is_some_and(|unit| unit.trim().is_empty()) {
            return Err(Error::Validation(format!(
                "metric '{}' has an empty unit; omit it when dimensionless",
                self.name
            )));
        }
        match (self.reference_value, self.absolute_tolerance) {
            (None, None) => Ok(()),
            (Some(reference), Some(tolerance)) => {
                if !reference.is_finite() || !tolerance.is_finite() || tolerance < 0.0 {
                    return Err(Error::Validation(format!(
                        "metric '{}' has invalid reference value or tolerance",
                        self.name
                    )));
                }
                Ok(())
            }
            _ => Err(Error::Validation(format!(
                "metric '{}' must specify both reference_value and absolute_tolerance, or neither",
                self.name
            ))),
        }
    }

    fn is_reference_mismatch(&self) -> bool {
        match (self.reference_value, self.absolute_tolerance) {
            (Some(reference), Some(tolerance)) => {
                (self.observed_value - reference).abs() > tolerance
            }
            _ => false,
        }
    }
}

/// Producer identity recorded in the manifest. Canonical event verification
/// remains authoritative for key ownership and authorization.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BenchmarkPrincipal {
    pub actor_id: String,
    pub organization_id: String,
}

impl BenchmarkPrincipal {
    fn validate(&self, label: &str) -> Result<()> {
        validate_nonempty(&self.actor_id, &format!("{label} actor ID"))?;
        validate_nonempty(&self.organization_id, &format!("{label} organization ID"))
    }
}

/// Reproduction record. Independence is conservatively represented by a
/// different actor AND a different organization; the canonical Mycelix event
/// path must still authenticate these values rather than trusting this struct.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BenchmarkVerifier {
    pub principal: BenchmarkPrincipal,
    pub verifier_implementation: String,
    pub verifier_revision: String,
    pub method: String,
    pub verified_at: DateTime<Utc>,
}

impl BenchmarkVerifier {
    fn validate(&self) -> Result<()> {
        self.principal.validate("verifier")?;
        validate_nonempty(&self.verifier_implementation, "verifier implementation")?;
        validate_nonempty(&self.method, "verification method")?;
        if !matches!(self.verifier_revision.len(), 40 | 64)
            || !self
                .verifier_revision
                .bytes()
                .all(|byte| byte.is_ascii_hexdigit())
        {
            return Err(Error::Validation(
                "verifier revision must be a full 40- or 64-character hexadecimal commit".into(),
            ));
        }
        Ok(())
    }
}

/// Reproducibility contract for one benchmark execution.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ScientificBenchmarkManifest {
    pub protocol: String,
    pub schema_version: u16,
    pub benchmark_id: String,
    pub status: BenchmarkStatus,
    pub created_at: DateTime<Utc>,
    pub completed_at: Option<DateTime<Utc>>,
    pub producer: BenchmarkPrincipal,
    pub source_revision: BenchmarkSourceRevision,
    pub environment: BenchmarkEnvironment,
    pub input_artifacts: Vec<BenchmarkArtifactBinding>,
    pub output_artifacts: Vec<BenchmarkArtifactBinding>,
    pub reference: Option<BenchmarkReference>,
    pub metrics: Vec<BenchmarkMetric>,
    pub verifier: Option<BenchmarkVerifier>,
    pub limitations: Vec<String>,
    pub notes: Option<String>,
}

impl ScientificBenchmarkManifest {
    /// Validate protocol, provenance, numeric values, artifact uniqueness, and
    /// lifecycle invariants. Deserialized manifests must be validated again;
    /// successful deserialization is not semantic verification.
    pub fn validate(&self) -> Result<()> {
        if self.protocol != SCIENTIFIC_BENCHMARK_MANIFEST_PROTOCOL {
            return Err(Error::Validation("unsupported benchmark manifest protocol".into()));
        }
        if self.schema_version != SCIENTIFIC_BENCHMARK_MANIFEST_VERSION {
            return Err(Error::Validation(format!(
                "unsupported benchmark manifest schema version: {}",
                self.schema_version
            )));
        }
        validate_nonempty(&self.benchmark_id, "benchmark ID")?;
        self.producer.validate("producer")?;
        self.source_revision.validate()?;
        self.environment.validate()?;

        if let Some(reference) = &self.reference {
            reference.validate()?;
        }
        if let Some(notes) = &self.notes {
            validate_nonempty(notes, "notes")?;
        }
        for limitation in &self.limitations {
            validate_nonempty(limitation, "limitation")?;
        }

        validate_artifacts("input", &self.input_artifacts)?;
        validate_artifacts("output", &self.output_artifacts)?;

        let mut metric_names = HashSet::new();
        for metric in &self.metrics {
            metric.validate()?;
            if !metric_names.insert(metric.name.as_str()) {
                return Err(Error::Validation(format!(
                    "duplicate metric name: {}",
                    metric.name
                )));
            }
        }

        if let Some(completed_at) = self.completed_at.as_ref() {
            if completed_at < &self.created_at {
                return Err(Error::Validation(
                    "completion timestamp precedes manifest creation".into(),
                ));
            }
        }

        let reference_mismatch = self.metrics.iter().any(BenchmarkMetric::is_reference_mismatch);
        match self.status {
            BenchmarkStatus::Proposed | BenchmarkStatus::NotRun => {
                if self.completed_at.is_some() || self.verifier.is_some() {
                    return Err(Error::Validation(
                        "proposed/not-run manifests cannot contain completion or verifier timestamps"
                            .into(),
                    ));
                }
                if !self.output_artifacts.is_empty() || !self.metrics.is_empty() {
                    return Err(Error::Validation(
                        "proposed/not-run manifests cannot declare observed outputs or metrics"
                            .into(),
                    ));
                }
            }
            BenchmarkStatus::LocallyVerified => {
                self.require_completed_output_and_metrics()?;
                self.require_clean_source()?;
                if self.verifier.is_some() {
                    return Err(Error::Validation(
                        "locally verified status must not claim an independent verifier".into(),
                    ));
                }
                if reference_mismatch {
                    return Err(Error::Validation(
                        "metrics exceed declared reference tolerances; status must be reference_mismatch"
                            .into(),
                    ));
                }
            }
            BenchmarkStatus::IndependentlyReproduced => {
                self.require_completed_output_and_metrics()?;
                self.require_clean_source()?;
                let reference = self.reference.as_ref().ok_or_else(|| {
                    Error::Validation(
                        "independent reproduction requires an explicit reference".into(),
                    )
                })?;
                reference.validate()?;
                let verifier = self.verifier.as_ref().ok_or_else(|| {
                    Error::Validation(
                        "independent reproduction requires a verifier record".into(),
                    )
                })?;
                verifier.validate()?;
                if verifier.principal.actor_id == self.producer.actor_id
                    || verifier.principal.organization_id == self.producer.organization_id
                {
                    return Err(Error::Validation(
                        "independent reproduction requires a different actor and organization"
                            .into(),
                    ));
                }
                if self.completed_at.as_ref().is_some_and(|completed| &verifier.verified_at < completed) {
                    return Err(Error::Validation(
                        "verifier timestamp precedes benchmark completion".into(),
                    ));
                }
                if !self.metrics.iter().any(|metric| metric.reference_value.is_some()) {
                    return Err(Error::Validation(
                        "independent reproduction requires at least one metric compared with a declared reference value"
                            .into(),
                    ));
                }
                if reference_mismatch {
                    return Err(Error::Validation(
                        "independent reproduction exceeds a declared reference tolerance".into(),
                    ));
                }
            }
            BenchmarkStatus::ReferenceMismatch => {
                self.require_completed_output_and_metrics()?;
                self.require_clean_source()?;
                if self.reference.is_none() || !reference_mismatch {
                    return Err(Error::Validation(
                        "reference_mismatch requires a reference and at least one metric outside tolerance"
                            .into(),
                    ));
                }
            }
            BenchmarkStatus::ExecutionFailed => {
                if self.completed_at.is_none() || self.limitations.is_empty() {
                    return Err(Error::Validation(
                        "execution_failed requires a completion timestamp and an explanatory limitation"
                            .into(),
                    ));
                }
                if self.verifier.is_some() {
                    return Err(Error::Validation(
                        "execution_failed cannot claim an independent verifier".into(),
                    ));
                }
                self.require_clean_source()?;
            }
        }
        Ok(())
    }

    fn require_completed_output_and_metrics(&self) -> Result<()> {
        if self.completed_at.is_none() {
            return Err(Error::Validation(
                "completed benchmark status requires completed_at".into(),
            ));
        }
        if self.input_artifacts.is_empty() {
            return Err(Error::Validation(
                "completed benchmark status requires at least one input artifact".into(),
            ));
        }
        if self.output_artifacts.is_empty() {
            return Err(Error::Validation(
                "completed benchmark status requires at least one output artifact".into(),
            ));
        }
        if self.metrics.is_empty() {
            return Err(Error::Validation(
                "completed benchmark status requires at least one scalar metric".into(),
            ));
        }
        Ok(())
    }

    fn require_clean_source(&self) -> Result<()> {
        if !self.source_revision.working_tree_clean {
            return Err(Error::Validation(
                "completed benchmark outcomes require a clean, committed source tree".into(),
            ));
        }
        Ok(())
    }
}

fn validate_artifacts(label: &str, artifacts: &[BenchmarkArtifactBinding]) -> Result<()> {
    let mut names = HashSet::new();
    let mut locators = HashSet::new();
    for artifact in artifacts {
        artifact.validate(label)?;
        if !names.insert(artifact.name.as_str()) {
            return Err(Error::Validation(format!(
                "duplicate {label} artifact name: {}",
                artifact.name
            )));
        }
        if !locators.insert(artifact.locator.as_str()) {
            return Err(Error::Validation(format!(
                "duplicate {label} artifact locator: {}",
                artifact.locator
            )));
        }
    }
    Ok(())
}

fn validate_nonempty(value: &str, label: &str) -> Result<()> {
    if value.trim().is_empty() {
        return Err(Error::Validation(format!("{label} cannot be empty")));
    }
    if value.chars().any(char::is_control) {
        return Err(Error::Validation(format!("{label} cannot contain control characters")));
    }
    Ok(())
}

/// Verify both declared byte length and canonical BLAKE3 digest for a local
/// artifact. The caller chooses the local path; untrusted locators are never
/// resolved implicitly.
pub fn verify_local_artifact_binding(
    path: impl AsRef<Path>,
    binding: &BenchmarkArtifactBinding,
) -> Result<bool> {
    // Open once so the size check and digest refer to the same file handle.
    verify_open_file(
        File::open(path)?,
        binding.content_hash,
        Some(binding.size_bytes),
    )
}

/// Hash a local artifact with the canonical BLAKE3 content digest and compare
/// it to the manifest binding. The path is supplied by the caller; this method
/// never resolves a locator from an untrusted manifest or follows a URL.
pub fn verify_local_artifact(
    path: impl AsRef<Path>,
    expected_hash: ContentHash,
) -> Result<bool> {
    verify_open_file(File::open(path)?, expected_hash, None)
}

fn verify_open_file(
    file: File,
    expected_hash: ContentHash,
    expected_size: Option<u64>,
) -> Result<bool> {
    if let Some(size) = expected_size {
        if file.metadata()?.len() != size {
            return Ok(false);
        }
    }
    let mut reader = BufReader::new(file);
    let mut hasher = blake3::Hasher::new();
    let mut buffer = [0u8; 64 * 1024];
    let mut total_bytes = 0u64;
    loop {
        let bytes_read = reader.read(&mut buffer)?;
        if bytes_read == 0 {
            break;
        }
        total_bytes = total_bytes
            .checked_add(bytes_read as u64)
            .ok_or_else(|| Error::VerificationFailed("artifact byte count overflow".into()))?;
        if expected_size.is_some_and(|size| total_bytes > size) {
            return Ok(false);
        }
        hasher.update(&buffer[..bytes_read]);
    }
    if expected_size.is_some_and(|size| total_bytes != size) {
        return Ok(false);
    }
    let actual = ContentHash(*hasher.finalize().as_bytes());
    Ok(actual == expected_hash)
}

#[cfg(test)]
mod tests {
    use super::*;
    use chrono::TimeZone;
    use tempfile::tempdir;

    fn fixture() -> ScientificBenchmarkManifest {
        let created_at = Utc.timestamp_opt(1_800_000_000, 0).unwrap();
        ScientificBenchmarkManifest {
            protocol: SCIENTIFIC_BENCHMARK_MANIFEST_PROTOCOL.to_string(),
            schema_version: SCIENTIFIC_BENCHMARK_MANIFEST_VERSION,
            benchmark_id: "cosmology.background.eds.z1.v1".to_string(),
            status: BenchmarkStatus::LocallyVerified,
            created_at: created_at.clone(),
            completed_at: Some(created_at + chrono::Duration::seconds(10)),
            producer: BenchmarkPrincipal {
                actor_id: "did:key:solver-author".to_string(),
                organization_id: "luminous-dynamics".to_string(),
            },
            source_revision: BenchmarkSourceRevision {
                repository_uri: "https://github.com/Luminous-Dynamics/symthaea".to_string(),
                git_commit: "0123456789abcdef0123456789abcdef01234567".to_string(),
                working_tree_clean: true,
            },
            environment: BenchmarkEnvironment {
                toolchain: "rustc 1.96".to_string(),
                target: "x86_64-unknown-linux-gnu".to_string(),
                command: "cargo test -p symthaea-core cosmology".to_string(),
                dependency_lock_digest: Some(ContentHash::digest(b"Cargo.lock fixture")),
            },
            input_artifacts: vec![BenchmarkArtifactBinding {
                name: "benchmark-config".to_string(),
                role: BenchmarkArtifactRole::Configuration,
                locator: "artifact://config/eds-z1".to_string(),
                media_type: "application/json".to_string(),
                content_hash: ContentHash::digest(b"{\"z\":1}"),
                size_bytes: 7,
            }],
            output_artifacts: vec![BenchmarkArtifactBinding {
                name: "numerical-output".to_string(),
                role: BenchmarkArtifactRole::Result,
                locator: "artifact://result/eds-z1".to_string(),
                media_type: "application/json".to_string(),
                content_hash: ContentHash::digest(b"{\"h_z\":198.0}"),
                size_bytes: 13,
            }],
            reference: Some(BenchmarkReference {
                name: "Einstein-de-Sitter analytic solution".to_string(),
                version: "closed-form-v1".to_string(),
                locator: "https://example.invalid/eds-reference".to_string(),
            }),
            metrics: vec![BenchmarkMetric {
                name: "relative_distance_error".to_string(),
                observed_value: 1e-12,
                unit: None,
                reference_value: Some(0.0),
                absolute_tolerance: Some(1e-9),
            }],
            verifier: None,
            limitations: vec!["Background expansion only; no perturbations or CMB spectra.".to_string()],
            notes: None,
        }
    }

    #[test]
    fn locally_verified_manifest_requires_exact_provenance_and_output() {
        let manifest = fixture();
        assert!(manifest.validate().is_ok());

        let mut dirty = fixture();
        dirty.source_revision.working_tree_clean = false;
        assert!(dirty.validate().is_err());

        let mut missing_output = fixture();
        missing_output.output_artifacts.clear();
        assert!(missing_output.validate().is_err());
    }

    #[test]
    fn independent_reproduction_requires_different_actor_and_organization() {
        let mut manifest = fixture();
        manifest.status = BenchmarkStatus::IndependentlyReproduced;
        let completed_at = manifest.completed_at.as_ref().unwrap().clone();
        manifest.verifier = Some(BenchmarkVerifier {
            principal: BenchmarkPrincipal {
                actor_id: "did:key:solver-author".to_string(),
                organization_id: "external-observatory".to_string(),
            },
            verifier_implementation: "reference-runner".to_string(),
            verifier_revision: "abcdef0123456789abcdef0123456789abcdef01".to_string(),
            method: "recomputed from frozen inputs".to_string(),
            verified_at: completed_at + chrono::Duration::seconds(10),
        });
        assert!(manifest.validate().is_err());

        manifest.verifier.as_mut().unwrap().principal.actor_id =
            "did:key:independent-reviewer".to_string();
        assert!(manifest.validate().is_ok());
    }

    #[test]
    fn failed_execution_cannot_claim_independent_verification() {
        let mut manifest = fixture();
        manifest.status = BenchmarkStatus::ExecutionFailed;
        manifest.verifier = Some(BenchmarkVerifier {
            principal: BenchmarkPrincipal {
                actor_id: "did:key:reviewer".to_string(),
                organization_id: "external-lab".to_string(),
            },
            verifier_implementation: "reference-runner".to_string(),
            verifier_revision: "abcdef0123456789abcdef0123456789abcdef01".to_string(),
            method: "independent replay".to_string(),
            verified_at: manifest.created_at + chrono::Duration::seconds(20),
        });
        assert!(manifest.validate().is_err());
    }

    #[test]
    fn not_run_manifest_cannot_smuggle_in_observed_results() {
        let mut manifest = fixture();
        let result_artifact = manifest.output_artifacts[0].clone();
        manifest.status = BenchmarkStatus::NotRun;
        manifest.completed_at = None;
        manifest.output_artifacts.clear();
        manifest.metrics.clear();
        assert!(manifest.validate().is_ok());

        manifest.output_artifacts.push(result_artifact);
        assert!(manifest.validate().is_err());
    }

    #[test]
    fn reference_mismatch_is_not_silently_marked_verified() {
        let mut manifest = fixture();
        manifest.metrics[0].observed_value = 0.2;
        assert!(manifest.validate().is_err());

        manifest.status = BenchmarkStatus::ReferenceMismatch;
        assert!(manifest.validate().is_ok());
    }

    #[test]
    fn invalid_numeric_values_and_partial_tolerance_fail_closed() {
        let mut manifest = fixture();
        manifest.metrics[0].observed_value = f64::NAN;
        assert!(manifest.validate().is_err());

        let mut partial_tolerance = fixture();
        partial_tolerance.metrics[0].absolute_tolerance = None;
        assert!(partial_tolerance.validate().is_err());
    }

    #[test]
    fn local_artifact_digest_detects_mutated_bytes() {
        let directory = tempdir().unwrap();
        let artifact_path = directory.path().join("result.json");
        std::fs::write(&artifact_path, b"immutable scientific output").unwrap();
        let expected = ContentHash::digest(b"immutable scientific output");
        let binding = BenchmarkArtifactBinding {
            name: "result".to_string(),
            role: BenchmarkArtifactRole::Result,
            locator: "artifact://result".to_string(),
            media_type: "application/octet-stream".to_string(),
            content_hash: expected,
            size_bytes: b"immutable scientific output".len() as u64,
        };
        assert!(verify_local_artifact(&artifact_path, expected).unwrap());
        assert!(verify_local_artifact_binding(&artifact_path, &binding).unwrap());
        // Same byte length, different digest: this exercises hash verification
        // independently of the size check.
        std::fs::write(&artifact_path, b"immutable scientific outpuX").unwrap();
        assert!(!verify_local_artifact(&artifact_path, expected).unwrap());
        assert!(!verify_local_artifact_binding(&artifact_path, &binding).unwrap());

        // A separate length mismatch must also fail.
        std::fs::write(&artifact_path, b"short").unwrap();
        assert!(!verify_local_artifact_binding(&artifact_path, &binding).unwrap());
    }

    #[test]
    fn serde_round_trip_still_requires_semantic_validation() {
        let manifest = fixture();
        let bytes = serde_json::to_vec(&manifest).unwrap();
        let decoded: ScientificBenchmarkManifest = serde_json::from_slice(&bytes).unwrap();
        assert!(decoded.validate().is_ok());

        let mut value: serde_json::Value = serde_json::from_slice(&bytes).unwrap();
        value["schema_version"] = serde_json::json!(999);
        let decoded: ScientificBenchmarkManifest = serde_json::from_value(value).unwrap();
        assert!(decoded.validate().is_err());
    }
}
