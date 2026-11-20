// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Typed evidence records for space and civil-infrastructure engineering.
//!
//! Validation covers record shape and evidence bindings only, not physical truth.

use serde::{Deserialize, Serialize};

pub const SCHEMA: &str = "org.luminousdynamics.mycelix.space-engineering-evidence/v1";

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum EvidenceClass {
    SourceFact,
    Measurement,
    Derived,
    ModelResult,
    EngineeringAssumption,
    IndependentCheck,
    TestFixture,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum EvidenceState {
    Blocked,
    NotRun,
    Running,
    PassedAtSpecifiedFidelity,
    Inconclusive,
    Failed,
    Stale,
    Unavailable,
    Disputed,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ArtifactReference {
    /// Immutable artifact ID or locator, never a mutable "latest" alias.
    pub artifact_id: String,
    /// SHA-256 of exact artifact bytes.
    pub sha256: String,
    #[serde(default)]
    pub media_type: Option<String>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct IndependentCheck {
    pub method_id: String,
    pub tool_sha256: String,
    pub result: ArtifactReference,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SpaceEngineeringEvidence {
    pub schema: String,
    pub claim_id: String,
    pub case_id: String,
    pub class: EvidenceClass,
    pub state: EvidenceState,
    pub source_repository: String,
    pub source_revision: String,
    pub source_path: String,
    pub source_git_blob_sha: String,
    pub source_sha256: String,
    pub input_manifest_sha256: String,
    #[serde(default)]
    pub run_manifest_sha256: Option<String>,
    #[serde(default)]
    pub analysis_tool_sha256: Option<String>,
    pub result_artifacts: Vec<ArtifactReference>,
    #[serde(default)]
    pub frame_id: Option<String>,
    pub units: Vec<String>,
    pub uncertainty_statement: String,
    pub exclusions: Vec<String>,
    pub claim_ceiling: String,
    #[serde(default)]
    pub independent_check: Option<IndependentCheck>,
}

impl ArtifactReference {
    fn validate(&self, at: &str, errors: &mut Vec<String>) {
        if placeholder(&self.artifact_id) {
            errors.push(format!("{at}.artifact_id: required immutable identifier"));
        }
        if !sha256_ok(&self.sha256) {
            errors.push(format!("{at}.sha256: expected 64 hexadecimal characters"));
        }
        if self.media_type.as_deref().is_some_and(placeholder) {
            errors.push(format!("{at}.media_type: omit or specify"));
        }
    }
}

impl SpaceEngineeringEvidence {
    /// Validate schema/bindings only. This does not verify artifact bytes or science.
    pub fn validate(&self) -> Result<(), Vec<String>> {
        let mut errors = Vec::new();
        if self.schema != SCHEMA { errors.push("schema: unsupported version".into()); }
        for (name, value) in [
            ("claim_id", self.claim_id.as_str()),
            ("case_id", self.case_id.as_str()),
            ("source_repository", self.source_repository.as_str()),
            ("source_path", self.source_path.as_str()),
            ("uncertainty_statement", self.uncertainty_statement.as_str()),
            ("claim_ceiling", self.claim_ceiling.as_str()),
        ] {
            if placeholder(value) { errors.push(format!("{name}: required specified value")); }
        }
        if !git_id_ok(&self.source_revision) {
            errors.push(
                "source_revision: expected a full 40- or 64-character hexadecimal object ID".into(),
            );
        }
        if !git_id_ok(&self.source_git_blob_sha) {
            errors.push(
                "source_git_blob_sha: expected a full 40- or 64-character hexadecimal object ID".into(),
            );
        }
        if !sha256_ok(&self.source_sha256) { errors.push("source_sha256: invalid SHA-256".into()); }
        if !sha256_ok(&self.input_manifest_sha256) {
            errors.push("input_manifest_sha256: invalid SHA-256".into());
        }
        if self.run_manifest_sha256.as_deref().is_some_and(|v| !sha256_ok(v)) {
            errors.push("run_manifest_sha256: invalid SHA-256".into());
        }
        if self.analysis_tool_sha256.as_deref().is_some_and(|v| !sha256_ok(v)) {
            errors.push("analysis_tool_sha256: invalid SHA-256".into());
        }
        if self.frame_id.as_deref().is_some_and(placeholder) {
            errors.push("frame_id: omit or specify".into());
        }
        if self.units.iter().any(|v| placeholder(v)) {
            errors.push("units: omit unknown units rather than using placeholders".into());
        }
        if self.exclusions.iter().any(|v| placeholder(v)) {
            errors.push(
                "exclusions: omit inapplicable entries rather than using placeholders".into(),
            );
        }
        for (i, artifact) in self.result_artifacts.iter().enumerate() {
            artifact.validate(&format!("result_artifacts[{i}]"), &mut errors);
        }
        if let Some(check) = &self.independent_check {
            if placeholder(&check.method_id) {
                errors.push("independent_check.method_id: required".into());
            }
            if !sha256_ok(&check.tool_sha256) {
                errors.push("independent_check.tool_sha256: invalid SHA-256".into());
            }
            check.result.validate("independent_check.result", &mut errors);
            if self
                .analysis_tool_sha256
                .as_ref()
                .is_some_and(|primary| primary.eq_ignore_ascii_case(&check.tool_sha256))
            {
                errors.push("independent_check.tool_sha256: must differ from primary tool".into());
            }
        }
        if self.state == EvidenceState::PassedAtSpecifiedFidelity {
            match self.class {
                EvidenceClass::Derived | EvidenceClass::ModelResult => {
                    if self.run_manifest_sha256.is_none() {
                        errors.push("state: Derived/ModelResult requires run_manifest_sha256".into());
                    }
                    if self.analysis_tool_sha256.is_none() {
                        errors.push("state: Derived/ModelResult requires analysis_tool_sha256".into());
                    }
                    if self.result_artifacts.is_empty() {
                        errors.push("state: Derived/ModelResult requires at least one result artifact".into());
                    }
                }
                EvidenceClass::IndependentCheck => {
                    if self.independent_check.is_none() {
                        errors.push("state: IndependentCheck requires an independent_check record".into());
                    }
                }
                EvidenceClass::SourceFact
                | EvidenceClass::Measurement
                | EvidenceClass::EngineeringAssumption => {
                    if self.result_artifacts.is_empty() {
                        errors.push("state: evidence record requires at least one content-addressed artifact".into());
                    }
                }
                EvidenceClass::TestFixture => {
                    errors.push("state: TestFixture cannot be promoted as engineering evidence".into());
                }
            }
        }
        if errors.is_empty() { Ok(()) } else { Err(errors) }
    }
}

fn placeholder(value: &str) -> bool {
    matches!(value.trim().to_ascii_lowercase().as_str(),
        "" | "unknown" | "tbd" | "todo" | "unspecified" | "not specified" | "?" | "latest")
}
fn hex_id(value: &str, lengths: &[usize]) -> bool {
    lengths.contains(&value.len()) && value.bytes().all(|b| b.is_ascii_hexdigit())
}
fn sha256_ok(value: &str) -> bool { hex_id(value, &[64]) }
fn git_id_ok(value: &str) -> bool { hex_id(value, &[40, 64]) }

#[cfg(test)]
mod tests {
    use super::*;
    fn d(c: char) -> String { std::iter::repeat_n(c, 64).collect() }
    fn blocked() -> SpaceEngineeringEvidence {
        SpaceEngineeringEvidence {
            schema: SCHEMA.into(),
            claim_id: "claim:orbit:test".into(),
            case_id: "F-SKYHOOK-LOAD-STEP".into(),
            class: EvidenceClass::ModelResult, state: EvidenceState::Blocked,
            source_repository: "https://github.com/Luminous-Dynamics/symthaea".into(),
            source_revision: "a".repeat(40),
            source_path: "src/lib.rs".into(),
            source_git_blob_sha: "b".repeat(40),
            source_sha256: d('c'), input_manifest_sha256: d('d'), run_manifest_sha256: None,
            analysis_tool_sha256: None, result_artifacts: vec![], frame_id: Some("GCRF".into()),
            units: vec!["m".into(), "m/s".into()],
            uncertainty_statement: "Not evaluated: Blocked/NotRun.".into(),
            exclusions: vec!["No equilibrium solver executed.".into()],
            claim_ceiling: "Record validity only; no physical claim is established.".into(),
            independent_check: None,
        }
    }
    fn artifact() -> ArtifactReference {
        ArtifactReference {
            artifact_id: "artifact://result".into(),
            sha256: d('e'),
            media_type: Some("application/json".into()),
        }
    }

    #[test]
    fn valid_blocked_record_does_not_claim_physics() { assert!(blocked().validate().is_ok()); }

    #[test]
    fn malformed_digest_is_rejected() {
        let mut r = blocked(); r.source_sha256 = "not-a-digest".into();
        assert!(r.validate().unwrap_err().iter().any(|e| e.starts_with("source_sha256:")));
    }

    #[test]
    fn mutable_revision_alias_is_rejected() {
        let mut r = blocked(); r.source_revision = "latest".into();
        assert!(r.validate().unwrap_err().iter().any(|e| e.starts_with("source_revision:")));
    }

    #[test]
    fn test_fixture_cannot_be_promoted() {
        let mut r = blocked();
        r.class = EvidenceClass::TestFixture;
        r.state = EvidenceState::PassedAtSpecifiedFidelity;
        r.run_manifest_sha256 = Some(d('f'));
        r.analysis_tool_sha256 = Some(d('a'));
        r.result_artifacts.push(artifact());
        assert!(r
            .validate()
            .unwrap_err()
            .iter()
            .any(|e| e.contains("TestFixture cannot be promoted")));
    }

    #[test]
    fn source_fact_can_pass_without_a_solver_run() {
        let mut r = blocked();
        r.class = EvidenceClass::SourceFact;
        r.state = EvidenceState::PassedAtSpecifiedFidelity;
        r.result_artifacts.push(artifact());
        assert!(r.run_manifest_sha256.is_none());
        assert!(r.analysis_tool_sha256.is_none());
        assert!(r.validate().is_ok());
    }

    #[test]
    fn independent_check_class_requires_a_separate_check_record() {
        let mut r = blocked();
        r.class = EvidenceClass::IndependentCheck;
        r.state = EvidenceState::PassedAtSpecifiedFidelity;
        assert!(r.validate().unwrap_err().iter().any(|e| {
            e.contains("IndependentCheck requires an independent_check record")
        }));
        r.independent_check = Some(IndependentCheck {
            method_id: "independent-method".into(),
            tool_sha256: d('a'),
            result: artifact(),
        });
        assert!(r.validate().is_ok());
    }

    #[test]
    fn completed_result_requires_run_tool_and_output_bindings() {
        let mut r = blocked(); r.state = EvidenceState::PassedAtSpecifiedFidelity;
        let e = r.validate().unwrap_err();
        assert!(e.iter().any(|x| x.contains("requires run_manifest_sha256")));
        assert!(e.iter().any(|x| x.contains("requires analysis_tool_sha256")));
        assert!(e.iter().any(|x| x.contains("requires at least one result artifact")));
    }

    #[test]
    fn independent_check_requires_distinct_tool_fingerprint() {
        let mut r = blocked(); r.analysis_tool_sha256 = Some(d('a'));
        r.independent_check = Some(IndependentCheck {
            method_id: "independent-method".into(),
            tool_sha256: d('a'),
            result: artifact(),
        });
        assert!(r
            .validate()
            .unwrap_err()
            .iter()
            .any(|x| x.contains("must differ from primary tool")));
    }

    #[test]
    fn unknown_and_duplicate_json_fields_are_rejected() {
        let valid_json = serde_json::to_string(&blocked()).unwrap();

        // Keep every required field valid so the unknown-key failure cannot
        // be attributed to unrelated missing-field validation.
        let with_unknown = valid_json.replacen("{", r#"{"unexpected":true,"#, 1);
        let error = serde_json::from_str::<SpaceEngineeringEvidence>(&with_unknown)
            .expect_err("unknown field must be rejected");
        assert!(error.to_string().contains("unexpected"));

        // Duplicate a field in an otherwise valid record, so this specifically
        // exercises duplicate-field handling rather than missing fields.
        let with_duplicate = valid_json.replacen(
            r#"{"schema":"#,
            r#"{"schema":"first","schema":"#,
            1,
        );
        let error = serde_json::from_str::<SpaceEngineeringEvidence>(&with_duplicate)
            .expect_err("duplicate field must be rejected");
        assert!(error.to_string().contains("duplicate field"));
    }

    #[test]
    fn state_serialization_is_explicit() {
        let s = serde_json::to_string(&EvidenceState::PassedAtSpecifiedFidelity).unwrap();
        assert_eq!(s, r#""passed_at_specified_fidelity""#);
        assert_eq!(
            serde_json::from_str::<EvidenceState>(&s).unwrap(),
            EvidenceState::PassedAtSpecifiedFidelity
        );
    }
}
