//! Content-bound receipts for external AeroCommons adapter runs.
//!
//! This crate verifies receipt structure, canonical receipt identity, declared
//! artifact digests and exact payload bytes. It does not prove that an adapter
//! executed, authenticate a publisher, or establish semantic correctness.

use aero_evidence_types::ArtifactRef;
use serde::{Deserialize, Serialize};
use serde_json::Value;
use sha2::{Digest, Sha256};
use std::collections::BTreeSet;
use thiserror::Error;

pub const RECEIPT_SCHEMA: &str = "AeroAdapterRunReceiptV1";

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum InputRole {
    Ap242ProductDefinition,
    QifInput,
    NcProgram,
    MetrologyPlan,
    AdapterConfiguration,
    Auxiliary,
}

impl InputRole {
    fn as_str(self) -> &'static str {
        match self {
            Self::Ap242ProductDefinition => "ap242_product_definition",
            Self::QifInput => "qif_input",
            Self::NcProgram => "nc_program",
            Self::MetrologyPlan => "metrology_plan",
            Self::AdapterConfiguration => "adapter_configuration",
            Self::Auxiliary => "auxiliary",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum OutputRole {
    RawArtifact,
    NormalizedAssertions,
}

impl OutputRole {
    fn as_str(self) -> &'static str {
        match self {
            Self::RawArtifact => "raw_artifact",
            Self::NormalizedAssertions => "normalized_assertions",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum RightsStatus {
    ClearedForTesting,
    ClearedForRedistribution,
    NotReviewed,
    Restricted,
}

impl RightsStatus {
    fn admitted(self) -> bool {
        matches!(self, Self::ClearedForTesting | Self::ClearedForRedistribution)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum NetworkPolicy {
    Disabled,
    Allowlisted,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ContentArtifactV1 {
    pub reference: ArtifactRef,
    pub sha256: String,
    pub byte_length: u64,
    pub media_type: String,
    pub format_id: String,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RunInputV1 {
    pub role: InputRole,
    pub artifact: ContentArtifactV1,
    pub rights_status: RightsStatus,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RunOutputV1 {
    pub role: OutputRole,
    pub artifact: ContentArtifactV1,
    pub rights_status: RightsStatus,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AdapterIdentityV1 {
    pub adapter_id: String,
    pub version: String,
    pub executable: ContentArtifactV1,
    pub build_provenance: Option<ArtifactRef>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct InvocationV1 {
    pub arguments: ContentArtifactV1,
    pub environment_profile: ContentArtifactV1,
    pub configuration_sha256: String,
    pub locale: String,
    pub timezone: String,
    pub network_policy: NetworkPolicy,
    pub declared_egress: Vec<String>,
    pub observed_egress: Vec<String>,
    pub timeout_ms: u64,
}

#[derive(Debug, Clone, PartialEq, Eq, Deserialize)]
#[serde(deny_unknown_fields)]
struct EnvironmentProfileV1 {
    locale: String,
    timezone: String,
    network_policy: NetworkPolicy,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RunStatus {
    Succeeded,
    Failed,
    TimedOut,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AdapterRunReceiptV1 {
    pub schema: String,
    pub run_id: String,
    pub benchmark_id: String,
    pub case_id: String,
    pub adapter: AdapterIdentityV1,
    pub inputs: Vec<RunInputV1>,
    pub invocation: InvocationV1,
    pub outputs: Vec<RunOutputV1>,
    pub status: RunStatus,
    pub exit_code: Option<i32>,
    pub elapsed_ms: u64,
    pub warnings: Vec<String>,
    pub unsupported_assertions: Vec<String>,
    pub receipt_sha256: String,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ReceiptDisposition {
    IntegrityChecked,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ReceiptVerificationReport {
    pub disposition: ReceiptDisposition,
    pub run_status: RunStatus,
    pub semantic_output_eligible: bool,
    pub receipt_sha256: String,
    pub checked_artifacts: usize,
}

#[derive(Debug, Error, PartialEq, Eq)]
pub enum ReceiptError {
    #[error("invalid receipt JSON: {0}")]
    InvalidJson(String),
    #[error("malformed object at {0}")]
    MalformedObject(String),
    #[error("unknown field {field} at {scope}")]
    UnknownField { scope: String, field: String },
    #[error("missing field {field} at {scope}")]
    MissingField { scope: String, field: String },
    #[error("wrong receipt schema")]
    WrongSchema,
    #[error("invalid or empty identity at {0}")]
    InvalidIdentity(String),
    #[error("invalid content artifact {0}")]
    InvalidArtifact(String),
    #[error("duplicate artifact reference {0}")]
    DuplicateArtifact(String),
    #[error("artifact kind or format does not match declared role at {0}")]
    RoleArtifactMismatch(String),
    #[error("input rights are not admitted for {0}")]
    RightsNotAdmitted(String),
    #[error("adapter configuration digest does not match its input artifact")]
    ConfigurationDigestMismatch,
    #[error("invalid invocation profile")]
    InvalidInvocation,
    #[error("environment profile bytes disagree with invocation metadata")]
    EnvironmentProfileMismatch,
    #[error("undeclared network egress: {0}")]
    UndeclaredEgress(String),
    #[error("run status and exit code disagree")]
    StatusExitCodeMismatch,
    #[error("successful run has no normalized assertion output")]
    MissingNormalizedAssertions,
    #[error("successful run has no raw output artifact")]
    MissingRawOutput,
    #[error("receipt digest is malformed")]
    InvalidReceiptDigest,
    #[error("receipt digest does not match canonical payload")]
    ReceiptDigestMismatch,
    #[error("artifact byte length mismatch for {0}")]
    ArtifactByteLengthMismatch(String),
    #[error("artifact digest mismatch for {0}")]
    ArtifactDigestMismatch(String),
    #[error("missing artifact payload {0}")]
    MissingArtifactPayload(String),
    #[error("unexpected artifact payload {0}")]
    UnexpectedArtifactPayload(String),
    #[error("duplicate artifact payload {0}")]
    DuplicateArtifactPayload(String),
    #[error("canonical receipt serialization failed")]
    CanonicalizationFailed,
}

#[derive(Serialize)]
struct CanonicalReceiptPayload {
    schema: String,
    run_id: String,
    benchmark_id: String,
    case_id: String,
    adapter: AdapterIdentityV1,
    inputs: Vec<RunInputV1>,
    invocation: InvocationV1,
    outputs: Vec<RunOutputV1>,
    status: RunStatus,
    exit_code: Option<i32>,
    elapsed_ms: u64,
    warnings: Vec<String>,
    unsupported_assertions: Vec<String>,
}

/// Compute SHA-256 over the versioned receipt body, excluding receipt_sha256.
/// Inputs, outputs, egress entries and unsupported assertion IDs are sorted.
pub fn compute_receipt_digest(
    receipt: &AdapterRunReceiptV1,
) -> Result<String, ReceiptError> {
    let mut inputs = receipt.inputs.clone();
    inputs.sort_by(|left, right| {
        (
            left.role.as_str(),
            left.artifact.reference.id.as_str(),
            left.artifact.reference.kind.as_str(),
        )
            .cmp(&(
                right.role.as_str(),
                right.artifact.reference.id.as_str(),
                right.artifact.reference.kind.as_str(),
            ))
    });

    let mut outputs = receipt.outputs.clone();
    outputs.sort_by(|left, right| {
        (
            left.role.as_str(),
            left.artifact.reference.id.as_str(),
            left.artifact.reference.kind.as_str(),
        )
            .cmp(&(
                right.role.as_str(),
                right.artifact.reference.id.as_str(),
                right.artifact.reference.kind.as_str(),
            ))
    });

    let mut invocation = receipt.invocation.clone();
    invocation.declared_egress.sort();
    invocation.observed_egress.sort();

    let mut warnings = receipt.warnings.clone();
    warnings.sort();
    let mut unsupported_assertions = receipt.unsupported_assertions.clone();
    unsupported_assertions.sort();

    let payload = CanonicalReceiptPayload {
        schema: receipt.schema.clone(),
        run_id: receipt.run_id.clone(),
        benchmark_id: receipt.benchmark_id.clone(),
        case_id: receipt.case_id.clone(),
        adapter: receipt.adapter.clone(),
        inputs,
        invocation,
        outputs,
        status: receipt.status,
        exit_code: receipt.exit_code,
        elapsed_ms: receipt.elapsed_ms,
        warnings,
        unsupported_assertions,
    };
    let bytes = serde_json::to_vec(&payload).map_err(|_| ReceiptError::CanonicalizationFailed)?;
    Ok(sha256_hex(&bytes))
}

/// Strict parser. ArtifactRef is a shared AeroEvidence type, so its nested
/// fields are checked explicitly before deserialization.
pub fn parse_receipt_json(bytes: &[u8]) -> Result<AdapterRunReceiptV1, ReceiptError> {
    let value: Value = serde_json::from_slice(bytes)
        .map_err(|error| ReceiptError::InvalidJson(error.to_string()))?;
    check_object_fields(
        &value,
        "receipt",
        &[
            "schema", "run_id", "benchmark_id", "case_id", "adapter", "inputs",
            "invocation", "outputs", "status", "exit_code", "elapsed_ms",
            "warnings", "unsupported_assertions", "receipt_sha256",
        ],
        &[
            "schema", "run_id", "benchmark_id", "case_id", "adapter", "inputs",
            "invocation", "outputs", "status", "exit_code", "elapsed_ms",
            "warnings", "unsupported_assertions", "receipt_sha256",
        ],
    )?;

    let adapter = value.get("adapter").ok_or_else(|| ReceiptError::MalformedObject("receipt.adapter".into()))?;
    check_object_fields(
        adapter,
        "receipt.adapter",
        &["adapter_id", "version", "executable", "build_provenance"],
        &["adapter_id", "version", "executable"],
    )?;
    check_content_artifact(adapter.get("executable").ok_or_else(|| ReceiptError::MissingField {
        scope: "receipt.adapter".into(), field: "executable".into()
    })?, "receipt.adapter.executable")?;
    if let Some(reference) = adapter.get("build_provenance").filter(|item| !item.is_null()) {
        check_artifact_ref(reference, "receipt.adapter.build_provenance")?;
    }

    let inputs = value.get("inputs").and_then(Value::as_array)
        .ok_or_else(|| ReceiptError::MalformedObject("receipt.inputs".into()))?;
    for (index, input) in inputs.iter().enumerate() {
        let scope = format!("receipt.inputs[{index}]");
        check_object_fields(input, &scope, &["role", "artifact", "rights_status"], &["role", "artifact", "rights_status"])?;
        check_content_artifact(input.get("artifact").ok_or_else(|| ReceiptError::MissingField {
            scope: scope.clone(), field: "artifact".into()
        })?, &format!("{scope}.artifact"))?;
    }

    let invocation = value.get("invocation").ok_or_else(|| ReceiptError::MalformedObject("receipt.invocation".into()))?;
    check_object_fields(
        invocation,
        "receipt.invocation",
        &[
            "arguments", "environment_profile", "configuration_sha256",
            "locale", "timezone", "network_policy", "declared_egress",
            "observed_egress", "timeout_ms",
        ],
        &[
            "arguments", "environment_profile", "configuration_sha256",
            "locale", "timezone", "network_policy", "declared_egress",
            "observed_egress", "timeout_ms",
        ],
    )?;
    check_content_artifact(
        invocation.get("arguments").ok_or_else(|| ReceiptError::MissingField {
            scope: "receipt.invocation".into(), field: "arguments".into()
        })?,
        "receipt.invocation.arguments",
    )?;
    check_content_artifact(
        invocation.get("environment_profile").ok_or_else(|| ReceiptError::MissingField {
            scope: "receipt.invocation".into(), field: "environment_profile".into()
        })?,
        "receipt.invocation.environment_profile",
    )?;

    let outputs = value.get("outputs").and_then(Value::as_array)
        .ok_or_else(|| ReceiptError::MalformedObject("receipt.outputs".into()))?;
    for (index, output) in outputs.iter().enumerate() {
        let scope = format!("receipt.outputs[{index}]");
        check_object_fields(output, &scope, &["role", "artifact", "rights_status"], &["role", "artifact", "rights_status"])?;
        check_content_artifact(output.get("artifact").ok_or_else(|| ReceiptError::MissingField {
            scope: scope.clone(), field: "artifact".into()
        })?, &format!("{scope}.artifact"))?;
    }

    let receipt: AdapterRunReceiptV1 = serde_json::from_value(value)
        .map_err(|error| ReceiptError::InvalidJson(error.to_string()))?;
    validate_receipt(&receipt)?;
    Ok(receipt)
}

fn check_object_fields(
    value: &Value,
    scope: &str,
    allowed: &[&str],
    required: &[&str],
) -> Result<(), ReceiptError> {
    let object = value.as_object().ok_or_else(|| ReceiptError::MalformedObject(scope.into()))?;
    for key in object.keys() {
        if !allowed.contains(&key.as_str()) {
            return Err(ReceiptError::UnknownField { scope: scope.into(), field: key.clone() });
        }
    }
    for field in required {
        if !object.contains_key(*field) {
            return Err(ReceiptError::MissingField { scope: scope.into(), field: (*field).into() });
        }
    }
    Ok(())
}

fn check_artifact_ref(value: &Value, scope: &str) -> Result<(), ReceiptError> {
    check_object_fields(value, scope, &["id", "kind"], &["id", "kind"])
}

fn check_content_artifact(value: &Value, scope: &str) -> Result<(), ReceiptError> {
    check_object_fields(
        value,
        scope,
        &["reference", "sha256", "byte_length", "media_type", "format_id"],
        &["reference", "sha256", "byte_length", "media_type", "format_id"],
    )?;
    check_artifact_ref(
        value.get("reference").ok_or_else(|| ReceiptError::MissingField {
            scope: scope.into(), field: "reference".into()
        })?,
        &format!("{scope}.reference"),
    )
}

fn sha256_hex(bytes: &[u8]) -> String {
    let digest = Sha256::digest(bytes);
    digest.iter().map(|byte| format!("{byte:02x}")).collect()
}

fn valid_sha256(value: &str) -> bool {
    value.len() == 64
        && value.bytes().all(|byte| byte.is_ascii_hexdigit() && !byte.is_ascii_uppercase())
}

fn validate_content_artifact(artifact: &ContentArtifactV1) -> Result<(), ReceiptError> {
    let reference = &artifact.reference;
    if reference.id.trim().is_empty()
        || reference.kind.trim().is_empty()
        || !valid_sha256(&artifact.sha256)
        || reference.id != format!("sha256:{}", artifact.sha256)
        || artifact.byte_length == 0
        || artifact.media_type.trim().is_empty()
        || artifact.format_id.trim().is_empty()
    {
        return Err(ReceiptError::InvalidArtifact(reference.id.clone()));
    }
    Ok(())
}

/// Validate receipt shape and canonical identity. This does not authenticate
/// the adapter, runner, publisher, rights declaration, or execution event.
pub fn validate_receipt(receipt: &AdapterRunReceiptV1) -> Result<(), ReceiptError> {
    if receipt.schema != RECEIPT_SCHEMA {
        return Err(ReceiptError::WrongSchema);
    }
    for (value, label) in [
        (receipt.run_id.as_str(), "run_id"),
        (receipt.benchmark_id.as_str(), "benchmark_id"),
        (receipt.case_id.as_str(), "case_id"),
        (receipt.adapter.adapter_id.as_str(), "adapter.adapter_id"),
        (receipt.adapter.version.as_str(), "adapter.version"),
        (receipt.invocation.locale.as_str(), "invocation.locale"),
        (receipt.invocation.timezone.as_str(), "invocation.timezone"),
    ] {
        if value.trim().is_empty() {
            return Err(ReceiptError::InvalidIdentity(label.into()));
        }
    }

    validate_content_artifact(&receipt.adapter.executable)?;
    if receipt.adapter.executable.reference.kind != "adapter_executable" {
        return Err(ReceiptError::RoleArtifactMismatch("adapter.executable".into()));
    }
    if let Some(provenance) = &receipt.adapter.build_provenance
        && (!provenance.id.strip_prefix("sha256:").is_some_and(valid_sha256)
            || provenance.kind != "slsa_build_provenance")
    {
        return Err(ReceiptError::InvalidArtifact(provenance.id.clone()));
    }
    validate_content_artifact(&receipt.invocation.arguments)?;
    validate_content_artifact(&receipt.invocation.environment_profile)?;
    if receipt.invocation.arguments.reference.kind != "adapter_arguments"
        || receipt.invocation.arguments.format_id != "AERO_ARGUMENT_VECTOR_V1"
        || receipt.invocation.environment_profile.reference.kind != "environment_profile"
        || receipt.invocation.environment_profile.format_id != "AERO_ENVIRONMENT_PROFILE_V1"
        || !valid_sha256(&receipt.invocation.configuration_sha256)
        || receipt.invocation.timeout_ms == 0
        || receipt.elapsed_ms == 0
        || receipt.elapsed_ms > receipt.invocation.timeout_ms
    {
        return Err(ReceiptError::InvalidInvocation);
    }

    let configuration_inputs: Vec<_> = receipt.inputs.iter()
        .filter(|input| input.role == InputRole::AdapterConfiguration)
        .collect();
    if configuration_inputs.len() != 1
        || configuration_inputs[0].artifact.sha256 != receipt.invocation.configuration_sha256
    {
        return Err(ReceiptError::ConfigurationDigestMismatch);
    }
    if receipt.inputs.is_empty() {
        return Err(ReceiptError::InvalidInvocation);
    }

    let mut seen = BTreeSet::new();
    let mut check_unique = |artifact: &ContentArtifactV1| -> Result<(), ReceiptError> {
        validate_content_artifact(artifact)?;
        let key = (artifact.reference.id.clone(), artifact.reference.kind.clone());
        if !seen.insert(key.clone()) {
            return Err(ReceiptError::DuplicateArtifact(key.0));
        }
        Ok(())
    };
    check_unique(&receipt.adapter.executable)?;
    check_unique(&receipt.invocation.arguments)?;
    check_unique(&receipt.invocation.environment_profile)?;
    for input in &receipt.inputs {
        check_unique(&input.artifact)?;
        let role_matches = match input.role {
            InputRole::Ap242ProductDefinition => {
                input.artifact.reference.kind == "step_file"
                    && input.artifact.format_id.starts_with("ISO_10303_242:")
            }
            InputRole::QifInput => {
                input.artifact.reference.kind == "qif_file"
                    && input.artifact.format_id.starts_with("ISO_23952:")
            }
            InputRole::NcProgram => input.artifact.reference.kind == "nc_program",
            InputRole::MetrologyPlan => input.artifact.reference.kind == "metrology_plan",
            InputRole::AdapterConfiguration => {
                input.artifact.reference.kind == "adapter_config"
                    && input.artifact.format_id == "AERO_ADAPTER_CONFIG_V1"
            }
            InputRole::Auxiliary => true,
        };
        if !role_matches {
            return Err(ReceiptError::RoleArtifactMismatch(input.artifact.reference.id.clone()));
        }
        if !input.rights_status.admitted() {
            return Err(ReceiptError::RightsNotAdmitted(input.artifact.reference.id.clone()));
        }
    }
    for output in &receipt.outputs {
        check_unique(&output.artifact)?;
        if output.role == OutputRole::NormalizedAssertions
            && (output.artifact.reference.kind != "semantic_assertions"
                || output.artifact.format_id != "AeroSemanticAssertionV1"
                || output.artifact.media_type != "application/json")
        {
            return Err(ReceiptError::RoleArtifactMismatch(output.artifact.reference.id.clone()));
        }
        if !output.rights_status.admitted() {
            return Err(ReceiptError::RightsNotAdmitted(output.artifact.reference.id.clone()));
        }
    }

    if receipt.invocation.declared_egress.iter().any(|item| item.trim().is_empty())
        || receipt.invocation.observed_egress.iter().any(|item| item.trim().is_empty())
    {
        return Err(ReceiptError::InvalidInvocation);
    }
    let declared: BTreeSet<&str> = receipt.invocation.declared_egress.iter().map(String::as_str).collect();
    let observed: BTreeSet<&str> = receipt.invocation.observed_egress.iter().map(String::as_str).collect();
    if declared.len() != receipt.invocation.declared_egress.len()
        || observed.len() != receipt.invocation.observed_egress.len()
    {
        return Err(ReceiptError::InvalidInvocation);
    }
    match receipt.invocation.network_policy {
        NetworkPolicy::Disabled if !declared.is_empty() || !observed.is_empty() => {
            return Err(ReceiptError::InvalidInvocation);
        }
        NetworkPolicy::Allowlisted if declared.is_empty() => {
            return Err(ReceiptError::InvalidInvocation);
        }
        _ => {}
    }
    for origin in &observed {
        if !declared.contains(*origin) {
            return Err(ReceiptError::UndeclaredEgress((*origin).to_owned()));
        }
    }

    let normalized_count = receipt.outputs.iter()
        .filter(|output| output.role == OutputRole::NormalizedAssertions)
        .count();
    let raw_count = receipt.outputs.iter()
        .filter(|output| output.role == OutputRole::RawArtifact)
        .count();
    match (receipt.status, receipt.exit_code) {
        (RunStatus::Succeeded, Some(0)) => {
            if normalized_count != 1 {
                return Err(ReceiptError::MissingNormalizedAssertions);
            }
            if raw_count == 0 {
                return Err(ReceiptError::MissingRawOutput);
            }
        }
        (RunStatus::Failed, Some(code)) if code != 0 => {}
        (RunStatus::TimedOut, None) => {}
        _ => return Err(ReceiptError::StatusExitCodeMismatch),
    }

    let mut unsupported = BTreeSet::new();
    for assertion in &receipt.unsupported_assertions {
        if assertion.trim().is_empty() || !unsupported.insert(assertion) {
            return Err(ReceiptError::InvalidIdentity("unsupported_assertions".into()));
        }
    }
    if receipt.warnings.iter().any(|warning| warning.trim().is_empty()) {
        return Err(ReceiptError::InvalidIdentity("warnings".into()));
    }

    if !valid_sha256(&receipt.receipt_sha256) {
        return Err(ReceiptError::InvalidReceiptDigest);
    }
    if compute_receipt_digest(receipt)? != receipt.receipt_sha256 {
        return Err(ReceiptError::ReceiptDigestMismatch);
    }
    Ok(())
}

/// Verify the exact byte payload for every declared input, output and adapter
/// executable. The caller supplies bytes; this function performs no network I/O.
pub fn verify_artifact_payloads(
    receipt: &AdapterRunReceiptV1,
    payloads: &[(&ArtifactRef, &[u8])],
) -> Result<ReceiptVerificationReport, ReceiptError> {
    validate_receipt(receipt)?;
    let mut expected = vec![
        &receipt.adapter.executable,
        &receipt.invocation.arguments,
        &receipt.invocation.environment_profile,
    ];
    expected.extend(receipt.inputs.iter().map(|input| &input.artifact));
    expected.extend(receipt.outputs.iter().map(|output| &output.artifact));

    if payloads.len() != expected.len() {
        return Err(ReceiptError::MissingArtifactPayload(
            "payload set must contain exactly the declared artifacts".into(),
        ));
    }
    let mut seen = BTreeSet::new();
    for (reference, bytes) in payloads {
        let key = (reference.id.clone(), reference.kind.clone());
        if !seen.insert(key.clone()) {
            return Err(ReceiptError::DuplicateArtifactPayload(key.0));
        }
        let Some(artifact) = expected.iter().find(|item| &item.reference == *reference) else {
            return Err(ReceiptError::UnexpectedArtifactPayload(key.0));
        };
        verify_artifact_bytes(artifact, bytes)?;
        if artifact.reference == receipt.invocation.arguments.reference {
            let _arguments: Vec<String> = serde_json::from_slice(bytes)
                .map_err(|_| ReceiptError::InvalidInvocation)?;
        }
        if artifact.reference == receipt.invocation.environment_profile.reference {
            let profile: EnvironmentProfileV1 = serde_json::from_slice(bytes)
                .map_err(|_| ReceiptError::InvalidInvocation)?;
            if profile.locale != receipt.invocation.locale
                || profile.timezone != receipt.invocation.timezone
                || profile.network_policy != receipt.invocation.network_policy
            {
                return Err(ReceiptError::EnvironmentProfileMismatch);
            }
        }
    }
    for artifact in &expected {
        let key = (artifact.reference.id.clone(), artifact.reference.kind.clone());
        if !seen.contains(&key) {
            return Err(ReceiptError::MissingArtifactPayload(key.0));
        }
    }

    Ok(ReceiptVerificationReport {
        disposition: ReceiptDisposition::IntegrityChecked,
        run_status: receipt.status,
        semantic_output_eligible: receipt.status == RunStatus::Succeeded,
        receipt_sha256: receipt.receipt_sha256.clone(),
        checked_artifacts: expected.len(),
    })
}

pub fn verify_artifact_bytes(
    artifact: &ContentArtifactV1,
    bytes: &[u8],
) -> Result<(), ReceiptError> {
    if bytes.len() as u64 != artifact.byte_length {
        return Err(ReceiptError::ArtifactByteLengthMismatch(artifact.reference.id.clone()));
    }
    let digest = sha256_hex(bytes);
    if digest != artifact.sha256 || artifact.reference.id != format!("sha256:{digest}") {
        return Err(ReceiptError::ArtifactDigestMismatch(artifact.reference.id.clone()));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fixture() -> AdapterRunReceiptV1 {
        parse_receipt_json(include_bytes!("../fixtures/adapter-run/receipt-v1.json"))
            .expect("synthetic run receipt parses and validates")
    }

    fn payloads() -> Vec<(ArtifactRef, Vec<u8>)> {
        vec![
            (ArtifactRef { id: "sha256:2c78684bbce9e4a714c337d848eb938a5ce64fe85d052f53adb7e617eeffa6c0".into(), kind: "adapter_executable".into() },
             include_bytes!("../fixtures/adapter-run/adapter.bin").to_vec()),
            (ArtifactRef { id: "sha256:6c0ef43428d609b7a9518e8f2a06bc48886d1bbc2160734ded4fc83679e1a98a".into(), kind: "adapter_arguments".into() },
             include_bytes!("../fixtures/adapter-run/arguments.json").to_vec()),
            (ArtifactRef { id: "sha256:9839f815354d2ff6f0033e1161d7fce6a74f9d85eccac1ce1bfe0e80bff21c19".into(), kind: "environment_profile".into() },
             include_bytes!("../fixtures/adapter-run/environment-profile.json").to_vec()),
            (ArtifactRef { id: "sha256:8362e44d47f7f89da4a972ecd585d851f6ca3abcf5735294bc7737d3554c1d33".into(), kind: "step_file".into() },
             include_bytes!("../fixtures/adapter-run/source.step").to_vec()),
            (ArtifactRef { id: "sha256:2118dab37365c4e792db91933d0f12c6f3c0acb1bdb734b5c0fa4a85f95902fd".into(), kind: "adapter_config".into() },
             include_bytes!("../fixtures/adapter-run/adapter-config.json").to_vec()),
            (ArtifactRef { id: "sha256:8c1ed8f48415cb83a5459ed2128ce8543b39ad158f9d71b798447670b73f8029".into(), kind: "qif_result".into() },
             include_bytes!("../fixtures/adapter-run/output.qif").to_vec()),
            (ArtifactRef { id: "sha256:7853b080f48dc021262a844a5fcb0b05844202cb0300449aac404e1afd234003".into(), kind: "semantic_assertions".into() },
             include_bytes!("../fixtures/adapter-run/normalized-assertions.json").to_vec()),
        ]
    }

    #[test]
    fn synthetic_receipt_replays_and_checks_all_artifact_bytes() {
        let receipt = fixture();
        let bytes = payloads();
        let borrowed: Vec<_> = bytes.iter().map(|(reference, payload)| (reference, payload.as_slice())).collect();
        let report = verify_artifact_payloads(&receipt, &borrowed).expect("all bytes match");
        assert_eq!(report.disposition, ReceiptDisposition::IntegrityChecked);
        assert_eq!(report.run_status, RunStatus::Succeeded);
        assert!(report.semantic_output_eligible);
        assert_eq!(report.checked_artifacts, 7);
        assert_eq!(report.receipt_sha256, receipt.receipt_sha256);
    }

    #[test]
    fn argument_vector_bytes_are_verified() {
        let receipt = fixture();
        let mut bytes = payloads();
        bytes[1].1.push(b' ');
        let borrowed: Vec<_> = bytes.iter().map(|(reference, payload)| (reference, payload.as_slice())).collect();
        assert!(matches!(verify_artifact_payloads(&receipt, &borrowed), Err(ReceiptError::ArtifactByteLengthMismatch(_))));
    }

    #[test]
    fn environment_profile_bytes_are_verified() {
        let receipt = fixture();
        let mut bytes = payloads();
        bytes[2].1[0] = b'[';
        let borrowed: Vec<_> = bytes.iter().map(|(reference, payload)| (reference, payload.as_slice())).collect();
        assert!(matches!(verify_artifact_payloads(&receipt, &borrowed), Err(ReceiptError::ArtifactDigestMismatch(_))));
    }

    #[test]
    fn input_payload_mutation_is_rejected() {
        let receipt = fixture();
        let mut bytes = payloads();
        bytes[3].1.push(b'X');
        let borrowed: Vec<_> = bytes.iter().map(|(reference, payload)| (reference, payload.as_slice())).collect();
        assert!(matches!(verify_artifact_payloads(&receipt, &borrowed), Err(ReceiptError::ArtifactByteLengthMismatch(_))));
    }

    #[test]
    fn output_payload_mutation_is_rejected() {
        let receipt = fixture();
        let mut bytes = payloads();
        bytes[5].1[0] = b'!';
        let borrowed: Vec<_> = bytes.iter().map(|(reference, payload)| (reference, payload.as_slice())).collect();
        assert!(matches!(verify_artifact_payloads(&receipt, &borrowed), Err(ReceiptError::ArtifactDigestMismatch(_))));
    }

    #[test]
    fn executable_bytes_are_part_of_the_verified_set() {
        let receipt = fixture();
        let mut bytes = payloads();
        bytes[0].1[0] = b'X';
        let borrowed: Vec<_> = bytes.iter().map(|(reference, payload)| (reference, payload.as_slice())).collect();
        assert!(matches!(verify_artifact_payloads(&receipt, &borrowed), Err(ReceiptError::ArtifactDigestMismatch(_))));
    }

    #[test]
    fn configuration_digest_is_bound_to_a_declared_input() {
        let mut receipt = fixture();
        receipt.invocation.configuration_sha256 = "0".repeat(64);
        receipt.receipt_sha256 = compute_receipt_digest(&receipt).unwrap();
        assert_eq!(validate_receipt(&receipt), Err(ReceiptError::ConfigurationDigestMismatch));
    }

    #[test]
    fn undeclared_egress_is_rejected() {
        let mut receipt = fixture();
        receipt.invocation.network_policy = NetworkPolicy::Allowlisted;
        receipt.invocation.declared_egress.clear();
        receipt.invocation.observed_egress = vec!["https://unexpected.example".into()];
        receipt.receipt_sha256 = compute_receipt_digest(&receipt).unwrap();
        assert_eq!(validate_receipt(&receipt), Err(ReceiptError::InvalidInvocation));
    }

    #[test]
    fn observed_egress_must_be_declared() {
        let mut receipt = fixture();
        receipt.invocation.network_policy = NetworkPolicy::Allowlisted;
        receipt.invocation.declared_egress = vec!["https://approved.example".into()];
        receipt.invocation.observed_egress = vec!["https://unexpected.example".into()];
        receipt.receipt_sha256 = compute_receipt_digest(&receipt).unwrap();
        assert!(matches!(validate_receipt(&receipt), Err(ReceiptError::UndeclaredEgress(_))));
    }

    #[test]
    fn environment_profile_must_match_invocation_metadata() {
        let mut receipt = fixture();
        receipt.invocation.network_policy = NetworkPolicy::Allowlisted;
        receipt.invocation.declared_egress = vec!["https://approved.example".into()];
        receipt.receipt_sha256 = compute_receipt_digest(&receipt).unwrap();
        let bytes = payloads();
        let borrowed: Vec<_> = bytes.iter().map(|(reference, payload)| (reference, payload.as_slice())).collect();
        assert_eq!(
            verify_artifact_payloads(&receipt, &borrowed),
            Err(ReceiptError::EnvironmentProfileMismatch)
        );
    }

    #[test]
    fn role_and_format_disagreement_is_rejected() {
        let mut receipt = fixture();
        receipt.inputs[0].role = InputRole::QifInput;
        receipt.receipt_sha256 = compute_receipt_digest(&receipt).unwrap();
        assert!(matches!(validate_receipt(&receipt), Err(ReceiptError::RoleArtifactMismatch(_))));
    }

    #[test]
    fn unknown_rights_do_not_admit_an_input() {
        let mut receipt = fixture();
        receipt.inputs[0].rights_status = RightsStatus::NotReviewed;
        receipt.receipt_sha256 = compute_receipt_digest(&receipt).unwrap();
        assert!(matches!(validate_receipt(&receipt), Err(ReceiptError::RightsNotAdmitted(_))));
    }

    #[test]
    fn timed_out_run_is_recordable_but_not_semantically_eligible() {
        let mut receipt = fixture();
        receipt.status = RunStatus::TimedOut;
        receipt.exit_code = None;
        receipt.outputs.clear();
        receipt.receipt_sha256 = compute_receipt_digest(&receipt).unwrap();
        assert!(validate_receipt(&receipt).is_ok());

        let mut bytes = payloads();
        bytes.truncate(5);
        let borrowed: Vec<_> = bytes.iter().map(|(reference, payload)| (reference, payload.as_slice())).collect();
        let report = verify_artifact_payloads(&receipt, &borrowed).expect("timeout receipt bytes verify");
        assert_eq!(report.run_status, RunStatus::TimedOut);
        assert!(!report.semantic_output_eligible);
        assert_eq!(report.checked_artifacts, 5);
    }

    #[test]
    fn successful_run_without_normalized_assertions_is_rejected() {
        let mut receipt = fixture();
        receipt.outputs.retain(|output| output.role != OutputRole::NormalizedAssertions);
        receipt.receipt_sha256 = compute_receipt_digest(&receipt).unwrap();
        assert_eq!(validate_receipt(&receipt), Err(ReceiptError::MissingNormalizedAssertions));
    }

    #[test]
    fn receipt_digest_is_input_order_invariant() {
        let mut receipt = fixture();
        let before = compute_receipt_digest(&receipt).unwrap();
        receipt.inputs.reverse();
        receipt.outputs.reverse();
        assert_eq!(compute_receipt_digest(&receipt).unwrap(), before);
    }

    #[test]
    fn digest_mutation_is_rejected() {
        let mut receipt = fixture();
        receipt.receipt_sha256 = "0".repeat(64);
        assert_eq!(validate_receipt(&receipt), Err(ReceiptError::ReceiptDigestMismatch));
    }

    #[test]
    fn strict_parser_rejects_unknown_nested_artifact_reference_field() {
        let mut value: Value = serde_json::from_slice(include_bytes!("../fixtures/adapter-run/receipt-v1.json")).unwrap();
        value["inputs"][0]["artifact"]["reference"]["release_status"] = serde_json::json!("approved");
        let bytes = serde_json::to_vec(&value).unwrap();
        assert!(matches!(parse_receipt_json(&bytes), Err(ReceiptError::UnknownField { .. })));
    }

    #[test]
    fn strict_parser_rejects_unknown_root_field() {
        let mut value: Value = serde_json::from_slice(include_bytes!("../fixtures/adapter-run/receipt-v1.json")).unwrap();
        value["trusted_execution"] = serde_json::json!(true);
        let bytes = serde_json::to_vec(&value).unwrap();
        assert!(matches!(parse_receipt_json(&bytes), Err(ReceiptError::UnknownField { .. })));
    }

    #[test]
    fn missing_required_nested_field_is_rejected() {
        let mut value: Value = serde_json::from_slice(include_bytes!("../fixtures/adapter-run/receipt-v1.json")).unwrap();
        value["adapter"]["executable"]["reference"].as_object_mut().unwrap().remove("kind");
        let bytes = serde_json::to_vec(&value).unwrap();
        assert!(matches!(parse_receipt_json(&bytes), Err(ReceiptError::MissingField { .. })));
    }

    #[test]
    fn nonzero_exit_cannot_claim_success() {
        let mut receipt = fixture();
        receipt.exit_code = Some(3);
        receipt.receipt_sha256 = compute_receipt_digest(&receipt).unwrap();
        assert_eq!(validate_receipt(&receipt), Err(ReceiptError::StatusExitCodeMismatch));
    }
}
