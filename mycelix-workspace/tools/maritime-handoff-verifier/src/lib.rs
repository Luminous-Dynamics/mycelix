//! Effects-disabled validation for Maritime Commons cross-stack handoff manifests.
//!
//! This crate validates strict JSON structure, locally resolvable artifact references,
//! handoff-edge consistency, and execution-state declarations. It does not fetch or
//! authenticate referenced artifacts and does not establish maritime truth, provider
//! authority, legal recognition, physical feasibility, or external effects.

use serde::{Deserialize, Deserializer, Serialize};
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet};
use std::fmt::Write as _;

pub const MAX_MANIFEST_BYTES: usize = 8 * 1024 * 1024;
pub const CLAIM_CEILING: &str =
    "Manifest structure and local handoff-reference consistency only; referenced claims are not authenticated or validated.";

#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Disposition {
    Pass,
    NotEvaluable,
    Unknown,
    Reject,
    Conflict,
}

#[derive(Clone, Debug, Eq, PartialEq, Serialize, Deserialize)]
pub struct Check {
    pub disposition: Disposition,
    pub code: String,
    pub detail: String,
}

impl Check {
    fn new(disposition: Disposition, code: &str, detail: &str) -> Self {
        Self {
            disposition,
            code: code.to_owned(),
            detail: detail.to_owned(),
        }
    }
    fn pass(code: &str, detail: &str) -> Self {
        Self::new(Disposition::Pass, code, detail)
    }
    fn reject(code: &str, detail: &str) -> Self {
        Self::new(Disposition::Reject, code, detail)
    }
    fn conflict(code: &str, detail: &str) -> Self {
        Self::new(Disposition::Conflict, code, detail)
    }
    fn unknown(code: &str, detail: &str) -> Self {
        Self::new(Disposition::Unknown, code, detail)
    }
}

#[derive(Clone, Debug, Eq, PartialEq, Serialize, Deserialize)]
pub struct ValidationReport {
    pub manifest_id: String,
    pub profile_version: String,
    pub input_bytes: usize,
    pub input_sha256: String,
    pub artifact_count: usize,
    pub handoff_edge_count: usize,
    pub disposition: Disposition,
    pub checks: Vec<Check>,
    pub claim_ceiling: String,
}

impl ValidationReport {
    pub fn is_pass(&self) -> bool {
        self.disposition == Disposition::Pass
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum InputError {
    TooLarge,
    InvalidJsonOrSchema,
}

impl std::fmt::Display for InputError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::TooLarge => write!(formatter, "manifest exceeds the 8 MiB input limit"),
            Self::InvalidJsonOrSchema => {
                write!(formatter, "manifest is invalid JSON or violates the strict manifest schema")
            }
        }
    }
}
impl std::error::Error for InputError {}

fn deserialize_optional_non_null<'de, D, T>(deserializer: D) -> Result<Option<T>, D::Error>
where
    D: Deserializer<'de>,
    T: Deserialize<'de>,
{
    T::deserialize(deserializer).map(Some)
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Owner {
    Mycelix,
    SolAtlas,
    Symtropy,
    Symthaea,
}

impl Owner {
    fn expected_repository(self) -> &'static str {
        match self {
            Self::Mycelix => "Luminous-Dynamics/mycelix",
            Self::SolAtlas => "Luminous-Dynamics/sol-atlas-leptos",
            Self::Symtropy => "Luminous-Dynamics/symtropy",
            Self::Symthaea => "Luminous-Dynamics/symthaea",
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ManifestStatus {
    ContractTemplate,
    SyntheticFixtureNotExecuted,
    ExecutedFrozen,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ArtifactKind {
    SourceProjection,
    ProfilePin,
    ProviderOffer,
    ProviderHold,
    ProviderCommitment,
    EffectReceipt,
    ResourceConstraint,
    Scenario,
    SimulationRun,
    DecisionBrief,
    EvaluationReport,
    AuthorizationEvidence,
    HistoricalView,
    Transformation,
    Other,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Maturity {
    Production,
    Operational,
    ImplementationTesting,
    #[serde(rename = "prerelease")]
    PreRelease,
    Draft,
    Research,
    Synthetic,
    Unknown,
    Deprecated,
    Withdrawn,
}

#[derive(Clone, Debug, Eq, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ArtifactRef {
    pub artifact_ref: String,
    pub owner: Owner,
    pub repository: String,
    pub commit: String,
    pub path: String,
    pub content_digest: String,
    pub artifact_kind: ArtifactKind,
    pub profile_id: String,
    pub profile_version: String,
    pub maturity: Maturity,
    pub subject_refs: Vec<String>,
    #[serde(default, deserialize_with = "deserialize_optional_non_null")]
    pub resource_refs: Option<Vec<String>>,
    #[serde(default, deserialize_with = "deserialize_optional_non_null")]
    pub temporal_context_ref: Option<String>,
    #[serde(default, deserialize_with = "deserialize_optional_non_null")]
    pub spatial_context_ref: Option<String>,
    pub claim_ceiling: String,
    pub nonclaims: Vec<String>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum FidelityDisposition {
    Preserved,
    LossyDeclared,
    Blocked,
    Unknown,
    Conflict,
    NotEvaluated,
}

#[derive(Clone, Debug, Eq, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct HandoffEdge {
    pub edge_id: String,
    pub source_artifact_refs: Vec<String>,
    pub destination_owner: Owner,
    pub purpose: String,
    #[serde(default, deserialize_with = "deserialize_optional_non_null")]
    pub produced_artifact_ref: Option<String>,
    #[serde(default, deserialize_with = "deserialize_optional_non_null")]
    pub transformation_artifact_ref: Option<String>,
    #[serde(default, deserialize_with = "deserialize_optional_non_null")]
    pub independent_verifier_artifact_ref: Option<String>,
    pub fidelity_disposition: FidelityDisposition,
    pub declared_losses: Vec<String>,
    pub unresolved_fields: Vec<String>,
    pub claim_ceiling: String,
}

#[derive(Clone, Debug, Eq, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Execution {
    pub executed: bool,
    pub exact_head_verified: bool,
    pub independent_verification: bool,
    pub provider_effects: bool,
    pub legal_review: bool,
}

#[derive(Clone, Debug, Eq, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Manifest {
    pub manifest_id: String,
    pub profile_version: String,
    pub status: ManifestStatus,
    pub use_case: String,
    pub audience: Vec<String>,
    pub privacy_policy_refs: Vec<String>,
    pub artifact_refs: Vec<ArtifactRef>,
    pub handoff_edges: Vec<HandoffEdge>,
    pub nonclaims: Vec<String>,
    pub execution: Execution,
}

fn non_empty(value: &str) -> bool {
    !value.trim().is_empty()
}

fn valid_lower_hex(value: &str, expected_len: usize) -> bool {
    value.len() == expected_len
        && value
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}

fn valid_sha256(value: &str) -> bool {
    value
        .strip_prefix("sha256:")
        .is_some_and(|hex| valid_lower_hex(hex, 64))
}

fn hash_input(bytes: &[u8]) -> String {
    let digest = Sha256::digest(bytes);
    let mut encoded = String::with_capacity(64);
    for byte in digest {
        let _ = write!(&mut encoded, "{byte:02x}");
    }
    encoded
}

fn aggregate_disposition(checks: &[Check]) -> Disposition {
    checks
        .iter()
        .map(|check| check.disposition)
        .max()
        .unwrap_or(Disposition::NotEvaluable)
}

fn validate_artifacts(manifest: &Manifest, checks: &mut Vec<Check>) -> BTreeMap<String, Owner> {
    let mut owners = BTreeMap::new();
    let mut duplicate_refs = BTreeSet::new();

    for artifact in &manifest.artifact_refs {
        if owners.insert(artifact.artifact_ref.clone(), artifact.owner).is_some() {
            duplicate_refs.insert(artifact.artifact_ref.clone());
        }

        if !non_empty(&artifact.artifact_ref)
            || !non_empty(&artifact.path)
            || !non_empty(&artifact.profile_id)
            || !non_empty(&artifact.profile_version)
            || !non_empty(&artifact.claim_ceiling)
            || artifact.nonclaims.is_empty()
            || artifact.nonclaims.iter().any(|item| !non_empty(item))
            || artifact.subject_refs.iter().any(|item| !non_empty(item))
            || artifact.resource_refs.as_ref().is_some_and(|items| items.iter().any(|item| !non_empty(item)))
            || artifact.temporal_context_ref.as_deref().is_some_and(|item| !non_empty(item))
            || artifact.spatial_context_ref.as_deref().is_some_and(|item| !non_empty(item))
        {
            checks.push(Check::reject(
                "artifact_required_metadata_missing",
                "Artifact references require non-empty IDs, paths, profile pins, claim ceilings, subject references, and nonclaims.",
            ));
        }

        if artifact.repository != artifact.owner.expected_repository() {
            checks.push(Check::reject(
                "artifact_owner_repository_mismatch",
                "Artifact owner does not match the repository allow-list for this cross-stack profile.",
            ));
        }

        if !valid_lower_hex(&artifact.commit, 40) {
            checks.push(Check::reject(
                "artifact_commit_malformed",
                "Artifact commit must be exactly 40 lowercase hexadecimal characters.",
            ));
        }

        if !valid_sha256(&artifact.content_digest) {
            checks.push(Check::reject(
                "artifact_content_digest_malformed",
                "Artifact content digest must be sha256 followed by 64 lowercase hexadecimal characters.",
            ));
        }
    }

    if duplicate_refs.is_empty() {
        checks.push(Check::pass(
            "artifact_reference_ids_unique",
            "Artifact reference IDs are unique within the manifest.",
        ));
    } else {
        checks.push(Check::conflict(
            "artifact_reference_id_duplicate",
            "Two or more artifact records use the same artifact_ref identifier.",
        ));
    }

    owners
}

fn validate_edges(
    manifest: &Manifest,
    owners: &BTreeMap<String, Owner>,
    checks: &mut Vec<Check>,
) {
    let mut edge_ids = BTreeSet::new();
    let mut duplicate_edge_id = false;

    for edge in &manifest.handoff_edges {
        if !edge_ids.insert(edge.edge_id.as_str()) {
            duplicate_edge_id = true;
        }

        if !non_empty(&edge.edge_id)
            || !non_empty(&edge.purpose)
            || !non_empty(&edge.claim_ceiling)
            || edge.source_artifact_refs.is_empty()
            || edge.source_artifact_refs.iter().any(|item| !non_empty(item))
            || edge.declared_losses.iter().any(|item| !non_empty(item))
            || edge.unresolved_fields.iter().any(|item| !non_empty(item))
        {
            checks.push(Check::reject(
                "handoff_edge_required_metadata_missing",
                "Every handoff edge requires non-empty IDs, purpose, claim ceiling, source references, and well-formed loss/unresolved entries.",
            ));
        }

        let mut seen_sources = BTreeSet::new();
        let mut duplicate_sources = false;
        for source_ref in &edge.source_artifact_refs {
            if !seen_sources.insert(source_ref.as_str()) {
                duplicate_sources = true;
            }
            if !owners.contains_key(source_ref) {
                checks.push(Check::reject(
                    "handoff_source_reference_dangling",
                    "A handoff edge references a source artifact ID absent from artifact_refs.",
                ));
            }
        }
        if duplicate_sources {
            checks.push(Check::conflict(
                "handoff_source_reference_duplicate",
                "A handoff edge repeats one source artifact reference.",
            ));
        }

        let optional_refs = [
            (edge.produced_artifact_ref.as_deref(), "handoff_produced_reference_dangling"),
            (edge.transformation_artifact_ref.as_deref(), "handoff_transformation_reference_dangling"),
            (edge.independent_verifier_artifact_ref.as_deref(), "handoff_verifier_reference_dangling"),
        ];
        for (reference, code) in optional_refs {
            if reference.is_some_and(|reference| !owners.contains_key(reference)) {
                checks.push(Check::reject(
                    code,
                    "An optional handoff artifact reference does not resolve within artifact_refs.",
                ));
            }
        }

        if let Some(produced_ref) = edge.produced_artifact_ref.as_deref() {
            if let Some(produced_owner) = owners.get(produced_ref) {
                if *produced_owner != edge.destination_owner {
                    checks.push(Check::reject(
                        "handoff_produced_owner_mismatch",
                        "Produced artifact owner must match the handoff destination owner.",
                    ));
                }
            }
        }

        match edge.fidelity_disposition {
            FidelityDisposition::Preserved if !edge.declared_losses.is_empty() => {
                checks.push(Check::reject(
                    "handoff_preserved_with_loss",
                    "A preserved handoff cannot declare information loss.",
                ));
            }
            FidelityDisposition::LossyDeclared if edge.declared_losses.is_empty() => {
                checks.push(Check::reject(
                    "handoff_loss_not_declared",
                    "A lossy_declared handoff must name at least one information loss.",
                ));
            }
            FidelityDisposition::Blocked
            | FidelityDisposition::Unknown
            | FidelityDisposition::Conflict
            | FidelityDisposition::NotEvaluated
                if edge.unresolved_fields.is_empty() =>
            {
                checks.push(Check::unknown(
                    "handoff_unresolved_fields_missing",
                    "A blocked, unknown, conflicting, or unevaluated handoff must identify unresolved fields.",
                ));
            }
            _ => {}
        }
    }

    if duplicate_edge_id {
        checks.push(Check::conflict(
            "handoff_edge_id_duplicate",
            "Two or more handoff edges use the same edge_id.",
        ));
    } else {
        checks.push(Check::pass(
            "handoff_edge_ids_unique",
            "Handoff edge IDs are unique within the manifest.",
        ));
    }
}

fn validate_execution(manifest: &Manifest, checks: &mut Vec<Check>) {
    let execution = &manifest.execution;

    if execution.provider_effects {
        checks.push(Check::reject(
            "execution_provider_effects_forbidden",
            "This effects-disabled manifest profile requires provider_effects=false.",
        ));
    }

    match manifest.status {
        ManifestStatus::ContractTemplate => {
            if !manifest.artifact_refs.is_empty() || !manifest.handoff_edges.is_empty() {
                checks.push(Check::reject(
                    "template_contains_execution_artifacts",
                    "A contract_template must not claim artifact references or handoff edges.",
                ));
            }
            if execution.executed
                || execution.exact_head_verified
                || execution.independent_verification
                || execution.legal_review
            {
                checks.push(Check::reject(
                    "template_execution_flag_set",
                    "A contract_template must keep execution, exact-head, independent-verification, and legal-review flags false.",
                ));
            }
        }
        ManifestStatus::SyntheticFixtureNotExecuted => {
            if execution.executed {
                checks.push(Check::reject(
                    "synthetic_fixture_claims_execution",
                    "synthetic_fixture_not_executed cannot set executed=true.",
                ));
            }
            if execution.exact_head_verified || execution.independent_verification {
                checks.push(Check::reject(
                    "synthetic_fixture_claims_qualification",
                    "A non-executed synthetic fixture cannot claim exact-head or independent verification.",
                ));
            }
        }
        ManifestStatus::ExecutedFrozen => {
            if !execution.executed {
                checks.push(Check::reject(
                    "frozen_result_not_executed",
                    "executed_frozen requires executed=true.",
                ));
            }
            if !execution.exact_head_verified {
                checks.push(Check::reject(
                    "frozen_result_exact_head_missing",
                    "executed_frozen requires exact_head_verified=true.",
                ));
            }
            if !execution.independent_verification {
                checks.push(Check::reject(
                    "frozen_result_independent_verification_missing",
                    "executed_frozen requires independent_verification=true.",
                ));
            }
            if manifest.artifact_refs.is_empty() || manifest.handoff_edges.is_empty() {
                checks.push(Check::reject(
                    "frozen_result_evidence_graph_empty",
                    "executed_frozen requires at least one artifact reference and one handoff edge.",
                ));
            }
        }
    }
}

/// Validate one manifest byte sequence without network access or external effects.
/// Strict Serde structs reject unknown and duplicate fields. Optional properties are
/// absent-or-value: explicit JSON null is rejected. The input digest covers exact bytes.
pub fn validate_manifest_bytes(bytes: &[u8]) -> Result<ValidationReport, InputError> {
    if bytes.len() > MAX_MANIFEST_BYTES {
        return Err(InputError::TooLarge);
    }

    let manifest: Manifest =
        serde_json::from_slice(bytes).map_err(|_| InputError::InvalidJsonOrSchema)?;

    let mut checks = Vec::new();
    if !non_empty(&manifest.manifest_id) || !non_empty(&manifest.use_case) {
        checks.push(Check::reject(
            "manifest_identity_or_use_case_missing",
            "Manifest ID and use_case must be non-empty.",
        ));
    }
    if manifest.profile_version != "0.1.0" {
        checks.push(Check::reject(
            "manifest_profile_version_unsupported",
            "This verifier accepts only cross-stack handoff profile version 0.1.0.",
        ));
    }
    if manifest.audience.is_empty()
        || manifest.audience.iter().any(|item| !non_empty(item))
        || manifest.nonclaims.is_empty()
        || manifest.nonclaims.iter().any(|item| !non_empty(item))
    {
        checks.push(Check::reject(
            "manifest_audience_or_nonclaims_missing",
            "Manifest audience and nonclaims must both contain non-empty entries.",
        ));
    }
    if manifest.privacy_policy_refs.iter().any(|item| !non_empty(item)) {
        checks.push(Check::reject(
            "manifest_privacy_reference_empty",
            "Privacy policy references, when supplied, must be non-empty.",
        ));
    }

    let owners = validate_artifacts(&manifest, &mut checks);
    validate_edges(&manifest, &owners, &mut checks);
    validate_execution(&manifest, &mut checks);

    if checks.is_empty() {
        checks.push(Check::pass(
            "manifest_valid",
            "Manifest structure and local reference graph satisfy the declared checks.",
        ));
    }

    checks.sort_by(|left, right| {
        left.code
            .cmp(&right.code)
            .then(left.disposition.cmp(&right.disposition))
            .then(left.detail.cmp(&right.detail))
    });

    let disposition = aggregate_disposition(&checks);
    Ok(ValidationReport {
        manifest_id: manifest.manifest_id,
        profile_version: manifest.profile_version,
        input_bytes: bytes.len(),
        input_sha256: hash_input(bytes),
        artifact_count: manifest.artifact_refs.len(),
        handoff_edge_count: manifest.handoff_edges.len(),
        disposition,
        checks,
        claim_ceiling: CLAIM_CEILING.to_owned(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::{Value, json};

    const COMMIT_A: &str = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";
    const DIGEST_A: &str =
        "sha256:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";

    fn base_template() -> Value {
        json!({
            "manifest_id": "MARITIME-COMMONS-SYNTHETIC-COASTAL-STAY-TEMPLATE-001",
            "profile_version": "0.1.0",
            "status": "contract_template",
            "use_case": "Synthetic multi-provider coastal stay template.",
            "audience": ["research_review"],
            "privacy_policy_refs": [],
            "artifact_refs": [],
            "handoff_edges": [],
            "nonclaims": [
                "No provider commitment or booking has been made.",
                "No physical availability or legal clearance is established."
            ],
            "execution": {
                "executed": false,
                "exact_head_verified": false,
                "independent_verification": false,
                "provider_effects": false,
                "legal_review": false
            }
        })
    }

    fn artifact(reference: &str, owner: &str, repository: &str) -> Value {
        json!({
            "artifact_ref": reference,
            "owner": owner,
            "repository": repository,
            "commit": COMMIT_A,
            "path": "fixtures/maritime/example.json",
            "content_digest": DIGEST_A,
            "artifact_kind": "other",
            "profile_id": "fixture-profile",
            "profile_version": "0.1.0",
            "maturity": "synthetic",
            "subject_refs": ["fixture:subject:1"],
            "claim_ceiling": "Synthetic fixture only.",
            "nonclaims": ["Not physical truth."]
        })
    }

    fn valid_synthetic_manifest() -> Value {
        let mut value = base_template();
        value["status"] = json!("synthetic_fixture_not_executed");
        value["artifact_refs"] = json!([
            artifact("source", "sol_atlas", "Luminous-Dynamics/sol-atlas-leptos"),
            artifact("run", "symtropy", "Luminous-Dynamics/symtropy"),
            artifact("brief", "symthaea", "Luminous-Dynamics/symthaea"),
            artifact("stay", "mycelix", "Luminous-Dynamics/mycelix")
        ]);
        value["handoff_edges"] = json!([
            {
                "edge_id": "source-to-run",
                "source_artifact_refs": ["source"],
                "destination_owner": "symtropy",
                "purpose": "Synthetic feasibility input.",
                "produced_artifact_ref": "run",
                "fidelity_disposition": "preserved",
                "declared_losses": [],
                "unresolved_fields": [],
                "claim_ceiling": "Synthetic model feasibility only."
            },
            {
                "edge_id": "run-to-brief",
                "source_artifact_refs": ["run"],
                "purpose": "Synthetic read-only decision brief input.",
                "destination_owner": "symthaea",
                "produced_artifact_ref": "brief",
                "fidelity_disposition": "preserved",
                "declared_losses": [],
                "unresolved_fields": [],
                "claim_ceiling": "Read-only decision support only."
            },
            {
                "edge_id": "stay-to-review",
                "source_artifact_refs": ["stay", "brief"],
                "purpose": "Synthetic stay evidence composition review.",
                "destination_owner": "mycelix",
                "fidelity_disposition": "not_evaluated",
                "declared_losses": [],
                "unresolved_fields": ["provider authorization not supplied"],
                "claim_ceiling": "No provider effect."
            }
        ]);
        value
    }

    fn validate_value(value: &Value) -> ValidationReport {
        let bytes = serde_json::to_vec(value).expect("test JSON value serializes");
        validate_manifest_bytes(&bytes).expect("test fixture deserializes")
    }

    fn assert_has_code(report: &ValidationReport, code: &str) {
        assert!(
            report.checks.iter().any(|check| check.code == code),
            "expected check code {code}; got {:?}",
            report.checks
        );
    }

    #[test]
    fn empty_contract_template_is_a_valid_positive_control() {
        let report = validate_value(&base_template());
        assert_eq!(report.disposition, Disposition::Pass);
        assert_eq!(report.artifact_count, 0);
        assert_eq!(report.handoff_edge_count, 0);
    }

    #[test]
    fn synthetic_unexecuted_manifest_with_all_four_owners_passes() {
        let report = validate_value(&valid_synthetic_manifest());
        assert_eq!(report.disposition, Disposition::Pass);
        assert_eq!(report.artifact_count, 4);
        assert_eq!(report.handoff_edge_count, 3);
    }

    #[test]
    fn rejects_duplicate_artifact_reference_ids() {
        let mut value = valid_synthetic_manifest();
        let duplicate = value["artifact_refs"][0].clone();
        value["artifact_refs"].as_array_mut().unwrap().push(duplicate);
        let report = validate_value(&value);
        assert_eq!(report.disposition, Disposition::Conflict);
        assert_has_code(&report, "artifact_reference_id_duplicate");
    }

    #[test]
    fn rejects_duplicate_handoff_edge_ids() {
        let mut value = valid_synthetic_manifest();
        let duplicate = value["handoff_edges"][0].clone();
        value["handoff_edges"].as_array_mut().unwrap().push(duplicate);
        let report = validate_value(&value);
        assert_eq!(report.disposition, Disposition::Conflict);
        assert_has_code(&report, "handoff_edge_id_duplicate");
    }

    #[test]
    fn rejects_unknown_json_fields() {
        let mut value = base_template();
        value["unexpected_authority"] = json!("global");
        let bytes = serde_json::to_vec(&value).unwrap();
        assert_eq!(validate_manifest_bytes(&bytes), Err(InputError::InvalidJsonOrSchema));
    }

    #[test]
    fn rejects_duplicate_json_object_fields() {
        let bytes = br#"{
            "manifest_id":"first",
            "manifest_id":"second",
            "profile_version":"0.1.0",
            "status":"contract_template",
            "use_case":"fixture",
            "audience":["review"],
            "privacy_policy_refs":[],
            "artifact_refs":[],
            "handoff_edges":[],
            "nonclaims":["none"],
            "execution":{
                "executed":false,
                "exact_head_verified":false,
                "independent_verification":false,
                "provider_effects":false,
                "legal_review":false
            }
        }"#;
        assert_eq!(validate_manifest_bytes(bytes), Err(InputError::InvalidJsonOrSchema));
    }

    #[test]
    fn rejects_malformed_commit_sha() {
        let mut value = valid_synthetic_manifest();
        value["artifact_refs"][0]["commit"] = json!("abc123");
        let report = validate_value(&value);
        assert_eq!(report.disposition, Disposition::Reject);
        assert_has_code(&report, "artifact_commit_malformed");
    }

    #[test]
    fn rejects_uppercase_or_wrong_length_content_digest() {
        let mut value = valid_synthetic_manifest();
        value["artifact_refs"][0]["content_digest"] =
            json!("sha256:AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA");
        let report = validate_value(&value);
        assert_eq!(report.disposition, Disposition::Reject);
        assert_has_code(&report, "artifact_content_digest_malformed");
    }

    #[test]
    fn rejects_owner_repository_mismatch() {
        let mut value = valid_synthetic_manifest();
        value["artifact_refs"][0]["repository"] = json!("Luminous-Dynamics/symtropy");
        let report = validate_value(&value);
        assert_eq!(report.disposition, Disposition::Reject);
        assert_has_code(&report, "artifact_owner_repository_mismatch");
    }

    #[test]
    fn rejects_missing_claim_ceiling() {
        let mut value = valid_synthetic_manifest();
        value["artifact_refs"][0].as_object_mut().unwrap().remove("claim_ceiling");
        let bytes = serde_json::to_vec(&value).unwrap();
        assert_eq!(validate_manifest_bytes(&bytes), Err(InputError::InvalidJsonOrSchema));
    }

    #[test]
    fn rejects_missing_nonclaims() {
        let mut value = valid_synthetic_manifest();
        value["artifact_refs"][0].as_object_mut().unwrap().remove("nonclaims");
        let bytes = serde_json::to_vec(&value).unwrap();
        assert_eq!(validate_manifest_bytes(&bytes), Err(InputError::InvalidJsonOrSchema));
    }

    #[test]
    fn rejects_explicit_null_for_optional_string() {
        let mut value = valid_synthetic_manifest();
        value["handoff_edges"][0]["transformation_artifact_ref"] = Value::Null;
        let bytes = serde_json::to_vec(&value).unwrap();
        assert_eq!(validate_manifest_bytes(&bytes), Err(InputError::InvalidJsonOrSchema));
    }

    #[test]
    fn rejects_dangling_source_reference() {
        let mut value = valid_synthetic_manifest();
        value["handoff_edges"][0]["source_artifact_refs"] = json!(["missing"]);
        let report = validate_value(&value);
        assert_eq!(report.disposition, Disposition::Reject);
        assert_has_code(&report, "handoff_source_reference_dangling");
    }

    #[test]
    fn rejects_dangling_produced_reference() {
        let mut value = valid_synthetic_manifest();
        value["handoff_edges"][0]["produced_artifact_ref"] = json!("missing");
        let report = validate_value(&value);
        assert_eq!(report.disposition, Disposition::Reject);
        assert_has_code(&report, "handoff_produced_reference_dangling");
    }

    #[test]
    fn rejects_dangling_transformation_reference() {
        let mut value = valid_synthetic_manifest();
        value["handoff_edges"][0]["transformation_artifact_ref"] = json!("missing");
        let report = validate_value(&value);
        assert_eq!(report.disposition, Disposition::Reject);
        assert_has_code(&report, "handoff_transformation_reference_dangling");
    }

    #[test]
    fn rejects_dangling_independent_verifier_reference() {
        let mut value = valid_synthetic_manifest();
        value["handoff_edges"][0]["independent_verifier_artifact_ref"] = json!("missing");
        let report = validate_value(&value);
        assert_eq!(report.disposition, Disposition::Reject);
        assert_has_code(&report, "handoff_verifier_reference_dangling");
    }

    #[test]
    fn rejects_produced_artifact_owned_by_wrong_destination() {
        let mut value = valid_synthetic_manifest();
        value["handoff_edges"][0]["destination_owner"] = json!("symthaea");
        let report = validate_value(&value);
        assert_eq!(report.disposition, Disposition::Reject);
        assert_has_code(&report, "handoff_produced_owner_mismatch");
    }

    #[test]
    fn rejects_repeated_source_reference_within_edge() {
        let mut value = valid_synthetic_manifest();
        value["handoff_edges"][0]["source_artifact_refs"] = json!(["source", "source"]);
        let report = validate_value(&value);
        assert_eq!(report.disposition, Disposition::Conflict);
        assert_has_code(&report, "handoff_source_reference_duplicate");
    }

    #[test]
    fn rejects_preserved_edge_that_declares_loss() {
        let mut value = valid_synthetic_manifest();
        value["handoff_edges"][0]["declared_losses"] = json!(["timezone discarded"]);
        let report = validate_value(&value);
        assert_eq!(report.disposition, Disposition::Reject);
        assert_has_code(&report, "handoff_preserved_with_loss");
    }

    #[test]
    fn rejects_lossy_edge_without_declared_loss() {
        let mut value = valid_synthetic_manifest();
        value["handoff_edges"][0]["fidelity_disposition"] = json!("lossy_declared");
        let report = validate_value(&value);
        assert_eq!(report.disposition, Disposition::Reject);
        assert_has_code(&report, "handoff_loss_not_declared");
    }

    #[test]
    fn flags_unknown_edge_without_unresolved_fields() {
        let mut value = valid_synthetic_manifest();
        value["handoff_edges"][0]["fidelity_disposition"] = json!("unknown");
        value["handoff_edges"][0]["unresolved_fields"] = json!([]);
        let report = validate_value(&value);
        assert_eq!(report.disposition, Disposition::Unknown);
        assert_has_code(&report, "handoff_unresolved_fields_missing");
    }

    #[test]
    fn rejects_contract_template_with_artifact_references() {
        let mut value = base_template();
        value["artifact_refs"] = json!([
            artifact("source", "sol_atlas", "Luminous-Dynamics/sol-atlas-leptos")
        ]);
        let report = validate_value(&value);
        assert_eq!(report.disposition, Disposition::Reject);
        assert_has_code(&report, "template_contains_execution_artifacts");
    }

    #[test]
    fn rejects_contract_template_marked_executed() {
        let mut value = base_template();
        value["execution"]["executed"] = json!(true);
        let report = validate_value(&value);
        assert_eq!(report.disposition, Disposition::Reject);
        assert_has_code(&report, "template_execution_flag_set");
    }

    #[test]
    fn rejects_synthetic_fixture_marked_executed() {
        let mut value = valid_synthetic_manifest();
        value["execution"]["executed"] = json!(true);
        let report = validate_value(&value);
        assert_eq!(report.disposition, Disposition::Reject);
        assert_has_code(&report, "synthetic_fixture_claims_execution");
    }

    #[test]
    fn rejects_executed_frozen_without_exact_head_verification() {
        let mut value = valid_synthetic_manifest();
        value["status"] = json!("executed_frozen");
        value["execution"]["executed"] = json!(true);
        value["execution"]["independent_verification"] = json!(true);
        let report = validate_value(&value);
        assert_eq!(report.disposition, Disposition::Reject);
        assert_has_code(&report, "frozen_result_exact_head_missing");
    }

    #[test]
    fn rejects_executed_frozen_without_independent_verification() {
        let mut value = valid_synthetic_manifest();
        value["status"] = json!("executed_frozen");
        value["execution"]["executed"] = json!(true);
        value["execution"]["exact_head_verified"] = json!(true);
        let report = validate_value(&value);
        assert_eq!(report.disposition, Disposition::Reject);
        assert_has_code(&report, "frozen_result_independent_verification_missing");
    }

    #[test]
    fn rejects_provider_effects_flag() {
        let mut value = base_template();
        value["execution"]["provider_effects"] = json!(true);
        let report = validate_value(&value);
        assert_eq!(report.disposition, Disposition::Reject);
        assert_has_code(&report, "execution_provider_effects_forbidden");
    }

    #[test]
    fn byte_identical_inputs_produce_identical_reports() {
        let bytes = serde_json::to_vec(&valid_synthetic_manifest()).unwrap();
        let first = validate_manifest_bytes(&bytes).unwrap();
        let second = validate_manifest_bytes(&bytes).unwrap();
        assert_eq!(first, second);
        assert_eq!(first.input_sha256, hash_input(&bytes));
    }

    #[test]
    fn input_digest_is_over_exact_bytes_not_reformatted_json() {
        let compact = br#"{"x":1}"#;
        let spaced = br#"{ "x" : 1 }"#;
        assert_ne!(hash_input(compact), hash_input(spaced));
    }

    #[test]
    fn rejects_manifest_over_size_limit() {
        let bytes = vec![b' '; MAX_MANIFEST_BYTES + 1];
        assert_eq!(validate_manifest_bytes(&bytes), Err(InputError::TooLarge));
    }
}
