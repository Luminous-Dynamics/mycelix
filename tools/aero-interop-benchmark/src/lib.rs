//! Evidence-backed, multidimensional QIF/AP242 interoperability benchmark.
//!
//! The evaluator validates adapter receipts and exact payloads, requires a
//! complete tool × case run grid, and reports adjudicated confusion matrices
//! by claim dimension. It does not parse STEP/QIF, create oracle labels,
//! authenticate tool publishers, or make metrology/product-release decisions.

use aero_adapter_run_receipt::{
    compute_receipt_digest, validate_receipt, verify_artifact_payloads,
    AdapterRunReceiptV1, ContentArtifactV1, InputRole, NetworkPolicy, ReceiptError, RightsStatus, RunStatus,
};
use aero_evidence_types::ArtifactRef;
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet};
use thiserror::Error;

pub const BENCHMARK_SCHEMA: &str = "AeroInteropBenchmarkV1";

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ToolCategory {
    QifSchemaValidator,
    QifXsltValidator,
    StepAnalyzer,
    CadOrCmmAdapter,
    Extractor,
    TraceResolver,
    SemanticComparator,
    Other,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BenchmarkToolV1 {
    pub adapter_id: String,
    pub version: String,
    pub category: ToolCategory,
    pub executable: ContentArtifactV1,
    pub arguments: ContentArtifactV1,
    pub environment_profile: ContentArtifactV1,
    pub configuration_sha256: String,
    pub locale: String,
    pub timezone: String,
    pub network_policy: NetworkPolicy,
    pub declared_egress: Vec<String>,
    pub timeout_ms: u64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum BenchmarkDimension {
    QifStructure,
    QifNormativeChecks,
    Ap242PmiConformance,
    SourceExtraction,
    IdentityResolution,
    SemanticPreservation,
    ResultStatusRetention,
    Reproducibility,
    ExecutionSecurity,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum OracleExpectation {
    Accept,
    Reject,
    Unadjudicated,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct OracleClaimV1 {
    pub claim_id: String,
    pub dimension: BenchmarkDimension,
    pub expectation: OracleExpectation,
    pub safety_critical: bool,
    /// Stable locator in the source package or a named mutation control.
    pub source_locator: String,
    pub rationale_code: String,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BenchmarkInputV1 {
    pub role: InputRole,
    pub artifact: ContentArtifactV1,
    pub rights_status: RightsStatus,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BenchmarkCaseV1 {
    pub case_id: String,
    pub case_class: String,
    pub inputs: Vec<BenchmarkInputV1>,
    /// Independently authored/adjudicated oracle record, not tool output.
    pub oracle_artifact: ContentArtifactV1,
    pub oracle_rights_status: RightsStatus,
    pub oracle_method: String,
    pub oracle_profile_version: String,
    pub claims: Vec<OracleClaimV1>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BenchmarkManifestBodyV1 {
    pub benchmark_id: String,
    pub profile_version: String,
    pub qif_standard_version: String,
    pub ap242_edition: String,
    pub corpus_version: String,
    pub tools: Vec<BenchmarkToolV1>,
    pub cases: Vec<BenchmarkCaseV1>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BenchmarkManifestV1 {
    pub schema: String,
    pub body: BenchmarkManifestBodyV1,
    pub manifest_sha256: String,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ToolAttemptDisposition {
    Succeeded,
    Failed,
    TimedOut,
    Blocked,
    NotRun,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ClaimObservationDisposition {
    Accepted,
    Rejected,
    Unknown,
    Unsupported,
    Error,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ClaimObservationV1 {
    pub claim_id: String,
    pub disposition: ClaimObservationDisposition,
    pub reason_code: Option<String>,
}

pub struct ToolRunEvidence<'a> {
    pub tool_id: &'a str,
    pub case_id: &'a str,
    pub attempt: ToolAttemptDisposition,
    pub receipt: Option<&'a AdapterRunReceiptV1>,
    pub payloads: &'a [(&'a ArtifactRef, &'a [u8])],
    pub observed_claims: &'a [ClaimObservationV1],
    pub reason_code: Option<&'a str>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum BenchmarkDisposition {
    IntegrityCheckedAndScored,
}

#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct DimensionScorecard {
    /// Every expected claim for this dimension and tool, including failed runs.
    pub offered: u64,
    pub adjudicated: u64,
    pub true_accepts: u64,
    pub false_accepts: u64,
    pub true_rejects: u64,
    pub false_rejects: u64,
    pub oracle_unadjudicated: u64,
    pub observed_unknown: u64,
    pub observed_unsupported: u64,
    pub observed_errors: u64,
    pub not_run: u64,
    pub blocked: u64,
    pub critical_false_accepts: u64,
}

#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct ToolAttemptSummary {
    pub offered_cases: u64,
    pub succeeded: u64,
    pub failed: u64,
    pub timed_out: u64,
    pub blocked: u64,
    pub not_run: u64,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ToolScorecard {
    pub tool_id: String,
    pub version: String,
    pub category: ToolCategory,
    pub attempts: ToolAttemptSummary,
    pub dimensions: BTreeMap<BenchmarkDimension, DimensionScorecard>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct BenchmarkScorecard {
    pub disposition: BenchmarkDisposition,
    pub benchmark_id: String,
    pub manifest_sha256: String,
    pub tools: Vec<ToolScorecard>,
}

#[derive(Debug, Error, PartialEq, Eq)]
pub enum BenchmarkError {
    #[error("invalid manifest schema")]
    WrongSchema,
    #[error("empty benchmark identity at {0}")]
    InvalidIdentity(String),
    #[error("benchmark manifest digest is malformed")]
    InvalidManifestDigest,
    #[error("benchmark manifest digest mismatch")]
    ManifestDigestMismatch,
    #[error("duplicate tool {0}")]
    DuplicateTool(String),
    #[error("duplicate case {0}")]
    DuplicateCase(String),
    #[error("duplicate claim {0} in case {1}")]
    DuplicateClaim(String, String),
    #[error("case {0} has no oracle claims")]
    EmptyOracleClaims(String),
    #[error("oracle claim is incomplete: {0}")]
    InvalidOracleClaim(String),
    #[error("source/oracle artifact rights are not admitted: {0}")]
    RightsNotAdmitted(String),
    #[error("invalid content artifact {0}")]
    InvalidArtifact(String),
    #[error("benchmark corpus payload set is missing, duplicated or unexpected")]
    CorpusPayloadSetMismatch,
    #[error("duplicate corpus payload {0}")]
    DuplicateCorpusPayload(String),
    #[error("unexpected corpus payload {0}")]
    UnexpectedCorpusPayload(String),
    #[error("corpus artifact verification failed: {0}")]
    CorpusArtifactVerification(ReceiptError),
    #[error("benchmark offered {expected} tool/case slots but received {actual} run records")]
    RunGridMismatch { expected: usize, actual: usize },
    #[error("duplicate run record for tool {tool_id} case {case_id}")]
    DuplicateRun(String, String),
    #[error("missing run record for tool {tool_id} case {case_id}")]
    MissingRun(String, String),
    #[error("run references unregistered tool {0}")]
    UnknownTool(String),
    #[error("run references unknown case {0}")]
    UnknownCase(String),
    #[error("run receipt is missing or inconsistent for {tool_id}/{case_id}")]
    ReceiptMismatch { tool_id: String, case_id: String },
    #[error("run receipt does not identify the declared tool and case")]
    ReceiptIdentityMismatch,
    #[error("run receipt does not include the exact declared source inputs")]
    ReceiptInputMismatch,
    #[error("run receipt executable differs from the benchmark tool registry")]
    ExecutableMismatch,
    #[error("run invocation differs from the benchmark-pinned arguments/configuration/environment")]
    InvocationProfileMismatch,
    #[error("run attempt disposition disagrees with receipt status")]
    AttemptStatusMismatch,
    #[error("blocked/not-run attempt has unexpected receipt, payloads or observations")]
    UnexpectedUnexecutedEvidence,
    #[error("blocked/not-run attempt requires a reason code")]
    MissingAttemptReason,
    #[error("completed/failed/timed-out attempt requires a receipt")]
    MissingReceipt,
    #[error("failed or timed-out attempt cannot contain semantic claim observations")]
    ObservationsOnNonSuccess,
    #[error("successful run must observe every oracle claim exactly once")]
    ClaimObservationSetMismatch,
    #[error("duplicate claim observation {0}")]
    DuplicateClaimObservation(String),
    #[error("claim observation is missing a reason code: {0}")]
    MissingObservationReason(String),
    #[error("adapter receipt validation failed: {0}")]
    AdapterReceipt(ReceiptError),
    #[error("adapter artifact payload verification failed: {0}")]
    AdapterPayloadVerification(ReceiptError),
    #[error("canonical serialization failed")]
    CanonicalizationFailed,
}

fn valid_sha256(value: &str) -> bool {
    value.len() == 64
        && value
            .bytes()
            .all(|byte| byte.is_ascii_hexdigit() && !byte.is_ascii_uppercase())
}

fn sha256_hex(bytes: &[u8]) -> String {
    Sha256::digest(bytes)
        .iter()
        .map(|byte| format!("{byte:02x}"))
        .collect()
}

fn validate_artifact(artifact: &ContentArtifactV1) -> Result<(), BenchmarkError> {
    if artifact.reference.id.trim().is_empty()
        || artifact.reference.kind.trim().is_empty()
        || !valid_sha256(&artifact.sha256)
        || artifact.reference.id != format!("sha256:{}", artifact.sha256)
        || artifact.byte_length == 0
        || artifact.media_type.trim().is_empty()
        || artifact.format_id.trim().is_empty()
    {
        return Err(BenchmarkError::InvalidArtifact(
            artifact.reference.id.clone(),
        ));
    }
    Ok(())
}

fn canonical_manifest_body(body: &BenchmarkManifestBodyV1) -> BenchmarkManifestBodyV1 {
    let mut canonical = body.clone();
    canonical.tools.sort_by(|left, right| {
        (left.adapter_id.as_str(), left.version.as_str())
            .cmp(&(right.adapter_id.as_str(), right.version.as_str()))
    });
    canonical.cases.sort_by(|left, right| left.case_id.cmp(&right.case_id));
    for case in &mut canonical.cases {
        case.inputs.sort_by(|left, right| {
            (
                format!("{:?}", left.role),
                left.artifact.reference.id.as_str(),
                left.artifact.reference.kind.as_str(),
            )
                .cmp(&(
                    format!("{:?}", right.role),
                    right.artifact.reference.id.as_str(),
                    right.artifact.reference.kind.as_str(),
                ))
        });
        case.claims.sort_by(|left, right| {
            (left.dimension, left.claim_id.as_str())
                .cmp(&(right.dimension, right.claim_id.as_str()))
        });
    }
    canonical
}

/// Versioned JSON serialization with explicitly sorted unordered arrays.
/// This profile is not RFC 8785/JCS.
pub fn compute_benchmark_manifest_digest(
    manifest: &BenchmarkManifestV1,
) -> Result<String, BenchmarkError> {
    let canonical = canonical_manifest_body(&manifest.body);
    let bytes = serde_json::to_vec(&canonical).map_err(|_| BenchmarkError::CanonicalizationFailed)?;
    Ok(sha256_hex(&bytes))
}

pub fn validate_benchmark_manifest(
    manifest: &BenchmarkManifestV1,
) -> Result<(), BenchmarkError> {
    if manifest.schema != BENCHMARK_SCHEMA {
        return Err(BenchmarkError::WrongSchema);
    }
    for (value, label) in [
        (manifest.body.benchmark_id.as_str(), "benchmark_id"),
        (manifest.body.profile_version.as_str(), "profile_version"),
        (manifest.body.qif_standard_version.as_str(), "qif_standard_version"),
        (manifest.body.ap242_edition.as_str(), "ap242_edition"),
        (manifest.body.corpus_version.as_str(), "corpus_version"),
    ] {
        if value.trim().is_empty() {
            return Err(BenchmarkError::InvalidIdentity(label.into()));
        }
    }
    if manifest.body.tools.is_empty() || manifest.body.cases.is_empty() {
        return Err(BenchmarkError::InvalidIdentity("tools/cases".into()));
    }

    let mut tools = BTreeMap::new();
    for tool in &manifest.body.tools {
        if tool.adapter_id.trim().is_empty() || tool.version.trim().is_empty() {
            return Err(BenchmarkError::InvalidIdentity("tool".into()));
        }
        validate_artifact(&tool.executable)?;
        validate_artifact(&tool.arguments)?;
        validate_artifact(&tool.environment_profile)?;
        if tool.executable.reference.kind != "adapter_executable"
            || tool.arguments.reference.kind != "adapter_arguments"
            || tool.environment_profile.reference.kind != "environment_profile"
            || tool.arguments.format_id != "AERO_ARGUMENT_VECTOR_V1"
            || tool.environment_profile.format_id != "AERO_ENVIRONMENT_PROFILE_V1"
            || !valid_sha256(&tool.configuration_sha256)
            || tool.locale.trim().is_empty()
            || tool.timezone.trim().is_empty()
            || tool.timeout_ms == 0
            || (tool.network_policy == NetworkPolicy::Disabled && !tool.declared_egress.is_empty())
        {
            return Err(BenchmarkError::InvalidArtifact(tool.executable.reference.id.clone()));
        }
        if tools.insert(tool.adapter_id.as_str(), tool).is_some() {
            return Err(BenchmarkError::DuplicateTool(tool.adapter_id.clone()));
        }
    }

    let mut cases = BTreeSet::new();
    for case in &manifest.body.cases {
        if case.case_id.trim().is_empty() || case.case_class.trim().is_empty() {
            return Err(BenchmarkError::InvalidIdentity("case".into()));
        }
        if !cases.insert(case.case_id.as_str()) {
            return Err(BenchmarkError::DuplicateCase(case.case_id.clone()));
        }
        if case.inputs.is_empty() {
            return Err(BenchmarkError::InvalidIdentity(format!("case.inputs:{}", case.case_id)));
        }
        for input in &case.inputs {
            validate_artifact(&input.artifact)?;
            if !matches!(
                input.rights_status,
                RightsStatus::ClearedForTesting | RightsStatus::ClearedForRedistribution
            ) {
                return Err(BenchmarkError::RightsNotAdmitted(input.artifact.reference.id.clone()));
            }
        }
        validate_artifact(&case.oracle_artifact)?;
        if case.oracle_method.trim().is_empty() || case.oracle_profile_version.trim().is_empty() {
            return Err(BenchmarkError::InvalidIdentity(format!("oracle_method/profile:{}", case.case_id)));
        }
        if !matches!(
            case.oracle_rights_status,
            RightsStatus::ClearedForTesting | RightsStatus::ClearedForRedistribution
        ) {
            return Err(BenchmarkError::RightsNotAdmitted(case.oracle_artifact.reference.id.clone()));
        }
        let mut claims = BTreeSet::new();
        if case.claims.is_empty() {
            return Err(BenchmarkError::EmptyOracleClaims(case.case_id.clone()));
        }
        for claim in &case.claims {
            if claim.claim_id.trim().is_empty()
                || claim.source_locator.trim().is_empty()
                || claim.rationale_code.trim().is_empty()
            {
                return Err(BenchmarkError::InvalidOracleClaim(claim.claim_id.clone()));
            }
            if !claims.insert(claim.claim_id.as_str()) {
                return Err(BenchmarkError::DuplicateClaim(
                    claim.claim_id.clone(),
                    case.case_id.clone(),
                ));
            }
        }
    }
    if !valid_sha256(&manifest.manifest_sha256) {
        return Err(BenchmarkError::InvalidManifestDigest);
    }
    if compute_benchmark_manifest_digest(manifest)? != manifest.manifest_sha256 {
        return Err(BenchmarkError::ManifestDigestMismatch);
    }
    Ok(())
}

/// Check exact source and oracle bytes for all cases, rejecting missing,
/// duplicate and unexpected payloads.
pub fn verify_benchmark_corpus_payloads(
    manifest: &BenchmarkManifestV1,
    payloads: &[(&ArtifactRef, &[u8])],
) -> Result<usize, BenchmarkError> {
    validate_benchmark_manifest(manifest)?;
    let mut expected = BTreeMap::new();
    for case in &manifest.body.cases {
        for input in &case.inputs {
            add_expected_artifact(&mut expected, &input.artifact)?;
        }
        add_expected_artifact(&mut expected, &case.oracle_artifact)?;
    }
    if payloads.len() != expected.len() {
        return Err(BenchmarkError::CorpusPayloadSetMismatch);
    }
    let mut seen = BTreeSet::new();
    for (reference, bytes) in payloads {
        let key = (reference.id.clone(), reference.kind.clone());
        if !seen.insert(key.clone()) {
            return Err(BenchmarkError::DuplicateCorpusPayload(key.0));
        }
        let Some(artifact) = expected.get(&key) else {
            return Err(BenchmarkError::UnexpectedCorpusPayload(key.0));
        };
        aero_adapter_run_receipt::verify_artifact_bytes(artifact, bytes)
            .map_err(BenchmarkError::CorpusArtifactVerification)?;
    }
    if seen.len() != expected.len() {
        return Err(BenchmarkError::CorpusPayloadSetMismatch);
    }
    Ok(expected.len())
}

fn add_expected_artifact(
    expected: &mut BTreeMap<(String, String), ContentArtifactV1>,
    artifact: &ContentArtifactV1,
) -> Result<(), BenchmarkError> {
    validate_artifact(artifact)?;
    let key = (artifact.reference.id.clone(), artifact.reference.kind.clone());
    if let Some(existing) = expected.get(&key) {
        if existing != artifact {
            return Err(BenchmarkError::InvalidArtifact(key.0));
        }
    } else {
        expected.insert(key, artifact.clone());
    }
    Ok(())
}

fn validate_one_run(
    benchmark_id: &str,
    tool: &BenchmarkToolV1,
    case: &BenchmarkCaseV1,
    run: &ToolRunEvidence<'_>,
) -> Result<(), BenchmarkError> {
    if run.tool_id != tool.adapter_id || run.case_id != case.case_id {
        return Err(BenchmarkError::ReceiptIdentityMismatch);
    }
    match run.attempt {
        ToolAttemptDisposition::Blocked | ToolAttemptDisposition::NotRun => {
            if run.receipt.is_some()
                || !run.payloads.is_empty()
                || !run.observed_claims.is_empty()
            {
                return Err(BenchmarkError::UnexpectedUnexecutedEvidence);
            }
            if run.reason_code.is_none_or(|reason| reason.trim().is_empty()) {
                return Err(BenchmarkError::MissingAttemptReason);
            }
        }
        ToolAttemptDisposition::Succeeded
        | ToolAttemptDisposition::Failed
        | ToolAttemptDisposition::TimedOut => {
            if run.attempt != ToolAttemptDisposition::Succeeded
                && run.reason_code.is_none_or(|reason| reason.trim().is_empty())
            {
                return Err(BenchmarkError::MissingAttemptReason);
            }
            let receipt = run.receipt.ok_or(BenchmarkError::MissingReceipt)?;
            validate_receipt(receipt).map_err(BenchmarkError::AdapterReceipt)?;
            if receipt.benchmark_id != benchmark_id
                || receipt.case_id != case.case_id
                || receipt.adapter.adapter_id != tool.adapter_id
                || receipt.adapter.version != tool.version
            {
                return Err(BenchmarkError::ReceiptIdentityMismatch);
            }
            if receipt.adapter.executable != tool.executable {
                return Err(BenchmarkError::ExecutableMismatch);
            }
            let invocation = &receipt.invocation;
            let mut expected_egress = tool.declared_egress.clone();
            expected_egress.sort();
            let mut observed_declared_egress = invocation.declared_egress.clone();
            observed_declared_egress.sort();
            if invocation.arguments != tool.arguments
                || invocation.environment_profile != tool.environment_profile
                || invocation.configuration_sha256 != tool.configuration_sha256
                || invocation.locale != tool.locale
                || invocation.timezone != tool.timezone
                || invocation.network_policy != tool.network_policy
                || observed_declared_egress != expected_egress
                || invocation.timeout_ms != tool.timeout_ms
            {
                return Err(BenchmarkError::InvocationProfileMismatch);
            }
            if receipt.inputs.len() != case.inputs.len() {
                return Err(BenchmarkError::ReceiptInputMismatch);
            }
            for expected_input in &case.inputs {
                let matched = receipt.inputs.iter().any(|input| {
                    input.role == expected_input.role
                        && input.artifact == expected_input.artifact
                        && input.rights_status == expected_input.rights_status
                });
                if !matched {
                    return Err(BenchmarkError::ReceiptInputMismatch);
                }
            }
            let expected_status = match run.attempt {
                ToolAttemptDisposition::Succeeded => RunStatus::Succeeded,
                ToolAttemptDisposition::Failed => RunStatus::Failed,
                ToolAttemptDisposition::TimedOut => RunStatus::TimedOut,
                ToolAttemptDisposition::Blocked | ToolAttemptDisposition::NotRun => unreachable!(),
            };
            if receipt.status != expected_status {
                return Err(BenchmarkError::AttemptStatusMismatch);
            }
            verify_artifact_payloads(receipt, run.payloads)
                .map_err(BenchmarkError::AdapterPayloadVerification)?;
            match run.attempt {
                ToolAttemptDisposition::Succeeded => validate_claim_observations(case, run.observed_claims)?,
                ToolAttemptDisposition::Failed | ToolAttemptDisposition::TimedOut => {
                    if !run.observed_claims.is_empty() {
                        return Err(BenchmarkError::ObservationsOnNonSuccess);
                    }
                }
                ToolAttemptDisposition::Blocked | ToolAttemptDisposition::NotRun => unreachable!(),
            }
        }
    }
    Ok(())
}

fn validate_claim_observations(
    case: &BenchmarkCaseV1,
    observations: &[ClaimObservationV1],
) -> Result<(), BenchmarkError> {
    if observations.len() != case.claims.len() {
        return Err(BenchmarkError::ClaimObservationSetMismatch);
    }
    let expected: BTreeSet<_> = case.claims.iter().map(|claim| claim.claim_id.as_str()).collect();
    let mut observed = BTreeSet::new();
    for observation in observations {
        if !observed.insert(observation.claim_id.as_str()) {
            return Err(BenchmarkError::DuplicateClaimObservation(observation.claim_id.clone()));
        }
        if !expected.contains(observation.claim_id.as_str()) {
            return Err(BenchmarkError::ClaimObservationSetMismatch);
        }
        if matches!(
            observation.disposition,
            ClaimObservationDisposition::Unknown
                | ClaimObservationDisposition::Unsupported
                | ClaimObservationDisposition::Error
        ) && observation.reason_code.as_ref().is_none_or(|reason| reason.trim().is_empty())
        {
            return Err(BenchmarkError::MissingObservationReason(observation.claim_id.clone()));
        }
    }
    Ok(())
}

pub fn evaluate_benchmark(
    manifest: &BenchmarkManifestV1,
    corpus_payloads: &[(&ArtifactRef, &[u8])],
    runs: &[ToolRunEvidence<'_>],
) -> Result<BenchmarkScorecard, BenchmarkError> {
    validate_benchmark_manifest(manifest)?;
    verify_benchmark_corpus_payloads(manifest, corpus_payloads)?;
    let expected_slots = manifest.body.tools.len() * manifest.body.cases.len();
    if runs.len() != expected_slots {
        return Err(BenchmarkError::RunGridMismatch {
            expected: expected_slots,
            actual: runs.len(),
        });
    }

    let mut tool_map = BTreeMap::new();
    for tool in &manifest.body.tools {
        tool_map.insert(tool.adapter_id.as_str(), tool);
    }
    let mut case_map = BTreeMap::new();
    for case in &manifest.body.cases {
        case_map.insert(case.case_id.as_str(), case);
    }
    let mut run_map = BTreeMap::new();
    for run in runs {
        if !tool_map.contains_key(run.tool_id) {
            return Err(BenchmarkError::UnknownTool(run.tool_id.into()));
        }
        if !case_map.contains_key(run.case_id) {
            return Err(BenchmarkError::UnknownCase(run.case_id.into()));
        }
        if run_map.insert((run.tool_id, run.case_id), run).is_some() {
            return Err(BenchmarkError::DuplicateRun(run.tool_id.into(), run.case_id.into()));
        }
    }

    let mut scorecards = Vec::new();
    for tool in &manifest.body.tools {
        let mut attempts = ToolAttemptSummary::default();
        let mut dimensions: BTreeMap<BenchmarkDimension, DimensionScorecard> = BTreeMap::new();
        for case in &manifest.body.cases {
            let run = run_map.get(&(tool.adapter_id.as_str(), case.case_id.as_str()))
                .ok_or_else(|| BenchmarkError::MissingRun(tool.adapter_id.clone(), case.case_id.clone()))?;
            validate_one_run(&manifest.body.benchmark_id, tool, case, run)?;
            attempts.offered_cases += 1;
            match run.attempt {
                ToolAttemptDisposition::Succeeded => attempts.succeeded += 1,
                ToolAttemptDisposition::Failed => attempts.failed += 1,
                ToolAttemptDisposition::TimedOut => attempts.timed_out += 1,
                ToolAttemptDisposition::Blocked => attempts.blocked += 1,
                ToolAttemptDisposition::NotRun => attempts.not_run += 1,
            }

            let observations: BTreeMap<_, _> = run.observed_claims.iter()
                .map(|observation| (observation.claim_id.as_str(), observation))
                .collect();
            for claim in &case.claims {
                let metrics = dimensions.entry(claim.dimension).or_default();
                metrics.offered += 1;
                let observation = observations.get(claim.claim_id.as_str()).copied();
                let observed = match run.attempt {
                    ToolAttemptDisposition::Blocked => None,
                    ToolAttemptDisposition::NotRun
                    | ToolAttemptDisposition::Failed
                    | ToolAttemptDisposition::TimedOut => None,
                    ToolAttemptDisposition::Succeeded => observation.map(|item| item.disposition),
                };
                if claim.expectation == OracleExpectation::Unadjudicated {
                    metrics.oracle_unadjudicated += 1;
                } else {
                    metrics.adjudicated += 1;
                }
                match observed {
                    None if run.attempt == ToolAttemptDisposition::Blocked => metrics.blocked += 1,
                    None => metrics.not_run += 1,
                    Some(ClaimObservationDisposition::Unknown) => metrics.observed_unknown += 1,
                    Some(ClaimObservationDisposition::Unsupported) => metrics.observed_unsupported += 1,
                    Some(ClaimObservationDisposition::Error) => metrics.observed_errors += 1,
                    Some(ClaimObservationDisposition::Accepted)
                    | Some(ClaimObservationDisposition::Rejected)
                        if claim.expectation == OracleExpectation::Unadjudicated => {}
                    Some(ClaimObservationDisposition::Accepted) => {
                        match claim.expectation {
                            OracleExpectation::Accept => metrics.true_accepts += 1,
                            OracleExpectation::Reject => {
                                metrics.false_accepts += 1;
                                if claim.safety_critical {
                                    metrics.critical_false_accepts += 1;
                                }
                            }
                            OracleExpectation::Unadjudicated => unreachable!(),
                        }
                    }
                    Some(ClaimObservationDisposition::Rejected) => {
                        match claim.expectation {
                            OracleExpectation::Accept => metrics.false_rejects += 1,
                            OracleExpectation::Reject => metrics.true_rejects += 1,
                            OracleExpectation::Unadjudicated => unreachable!(),
                        }
                    }
                }
            }
        }
        scorecards.push(ToolScorecard {
            tool_id: tool.adapter_id.clone(),
            version: tool.version.clone(),
            category: tool.category,
            attempts,
            dimensions,
        });
    }
    scorecards.sort_by(|left, right| left.tool_id.cmp(&right.tool_id));

    Ok(BenchmarkScorecard {
        disposition: BenchmarkDisposition::IntegrityCheckedAndScored,
        benchmark_id: manifest.body.benchmark_id.clone(),
        manifest_sha256: manifest.manifest_sha256.clone(),
        tools: scorecards,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::Value;

    const ORACLE_BYTES: &[u8] = br#"{"oracle":"independent synthetic oracle v1","case":"synthetic-ap242-qif-001"}"#;

    fn artifact(kind: &str, format_id: &str, bytes: &[u8]) -> ContentArtifactV1 {
        let digest = sha256_hex(bytes);
        ContentArtifactV1 {
            reference: ArtifactRef {
                id: format!("sha256:{digest}"),
                kind: kind.into(),
            },
            sha256: digest,
            byte_length: bytes.len() as u64,
            media_type: "application/octet-stream".into(),
            format_id: format_id.into(),
        }
    }

    fn fixture() -> (BenchmarkManifestV1, AdapterRunReceiptV1, Vec<(ArtifactRef, Vec<u8>)>) {
        let mut receipt = aero_adapter_run_receipt::parse_receipt_json(include_bytes!(
            "../aero-adapter-run-receipt/fixtures/adapter-run/receipt-v1.json"
        )).unwrap();
        receipt.benchmark_id = "AERO-INTEROP-BENCH-001".into();
        receipt.adapter.adapter_id = "synthetic-step-to-qif".into();
        receipt.receipt_sha256 = compute_receipt_digest(&receipt).unwrap();

        let case = BenchmarkCaseV1 {
            case_id: receipt.case_id.clone(),
            case_class: "synthetic_positive_and_negative_controls".into(),
            inputs: receipt.inputs.iter().map(|input| BenchmarkInputV1 {
                role: input.role,
                artifact: input.artifact.clone(),
                rights_status: input.rights_status,
            }).collect(),
            oracle_artifact: artifact("oracle_record", "AERO_ORACLE_RECORD_V1", ORACLE_BYTES),
            oracle_rights_status: RightsStatus::ClearedForTesting,
            oracle_method: "independent_hand_enumeration".into(),
            oracle_profile_version: "AERO_ORACLE_PROFILE_V1".into(),
            claims: vec![
                OracleClaimV1 {
                    claim_id: "trace.characteristic_identity".into(),
                    dimension: BenchmarkDimension::IdentityResolution,
                    expectation: OracleExpectation::Accept,
                    safety_critical: true,
                    source_locator: "/QIFDocument/Results/CharacteristicMeasurement/CharacteristicItemId".into(),
                    rationale_code: "SYNTHETIC_VALID_TRACE".into(),
                },
                OracleClaimV1 {
                    claim_id: "trace.wrong_document_join".into(),
                    dimension: BenchmarkDimension::IdentityResolution,
                    expectation: OracleExpectation::Reject,
                    safety_critical: true,
                    source_locator: "mutation:external_qpid_substitution".into(),
                    rationale_code: "SYNTHETIC_WRONG_QPID".into(),
                },
                OracleClaimV1 {
                    claim_id: "pmi.semantic_pmi_preserved".into(),
                    dimension: BenchmarkDimension::SemanticPreservation,
                    expectation: OracleExpectation::Unadjudicated,
                    safety_critical: false,
                    source_locator: "oracle-gap:synthetic-pmi-property".into(),
                    rationale_code: "ORACLE_NOT_ADJUDICATED".into(),
                },
            ],
        };
        let tool = BenchmarkToolV1 {
            adapter_id: receipt.adapter.adapter_id.clone(),
            version: receipt.adapter.version.clone(),
            category: ToolCategory::CadOrCmmAdapter,
            executable: receipt.adapter.executable.clone(),
            arguments: receipt.invocation.arguments.clone(),
            environment_profile: receipt.invocation.environment_profile.clone(),
            configuration_sha256: receipt.invocation.configuration_sha256.clone(),
            locale: receipt.invocation.locale.clone(),
            timezone: receipt.invocation.timezone.clone(),
            network_policy: receipt.invocation.network_policy,
            declared_egress: receipt.invocation.declared_egress.clone(),
            timeout_ms: receipt.invocation.timeout_ms,
        };
        let body = BenchmarkManifestBodyV1 {
            benchmark_id: "AERO-INTEROP-BENCH-001".into(),
            profile_version: "AERO_QIF_AP242_TRACE_PROFILE_V1".into(),
            qif_standard_version: "3.0.0".into(),
            ap242_edition: "ISO_10303_242:2025_Edition_4".into(),
            corpus_version: "SYNTHETIC_CORPUS_V1".into(),
            tools: vec![tool],
            cases: vec![case],
        };
        let mut manifest = BenchmarkManifestV1 {
            schema: BENCHMARK_SCHEMA.into(),
            body,
            manifest_sha256: String::new(),
        };
        manifest.manifest_sha256 = compute_benchmark_manifest_digest(&manifest).unwrap();

        let payloads = vec![
            (
                ArtifactRef { id: "sha256:2c78684bbce9e4a714c337d848eb938a5ce64fe85d052f53adb7e617eeffa6c0".into(), kind: "adapter_executable".into() },
                include_bytes!("../aero-adapter-run-receipt/fixtures/adapter-run/adapter.bin").to_vec(),
            ),
            (
                ArtifactRef { id: "sha256:6c0ef43428d609b7a9518e8f2a06bc48886d1bbc2160734ded4fc83679e1a98a".into(), kind: "adapter_arguments".into() },
                include_bytes!("../aero-adapter-run-receipt/fixtures/adapter-run/arguments.json").to_vec(),
            ),
            (
                ArtifactRef { id: "sha256:9839f815354d2ff6f0033e1161d7fce6a74f9d85eccac1ce1bfe0e80bff21c19".into(), kind: "environment_profile".into() },
                include_bytes!("../aero-adapter-run-receipt/fixtures/adapter-run/environment-profile.json").to_vec(),
            ),
            (
                ArtifactRef { id: "sha256:8362e44d47f7f89da4a972ecd585d851f6ca3abcf5735294bc7737d3554c1d33".into(), kind: "step_file".into() },
                include_bytes!("../aero-adapter-run-receipt/fixtures/adapter-run/source.step").to_vec(),
            ),
            (
                ArtifactRef { id: "sha256:2118dab37365c4e792db91933d0f12c6f3c0acb1bdb734b5c0fa4a85f95902fd".into(), kind: "adapter_config".into() },
                include_bytes!("../aero-adapter-run-receipt/fixtures/adapter-run/adapter-config.json").to_vec(),
            ),
            (
                ArtifactRef { id: "sha256:8c1ed8f48415cb83a5459ed2128ce8543b39ad158f9d71b798447670b73f8029".into(), kind: "qif_result".into() },
                include_bytes!("../aero-adapter-run-receipt/fixtures/adapter-run/output.qif").to_vec(),
            ),
            (
                ArtifactRef { id: "sha256:7853b080f48dc021262a844a5fcb0b05844202cb0300449aac404e1afd234003".into(), kind: "semantic_assertions".into() },
                include_bytes!("../aero-adapter-run-receipt/fixtures/adapter-run/normalized-assertions.json").to_vec(),
            ),
        ];
        (manifest, receipt, payloads)
    }

    fn borrowed(payloads: &[(ArtifactRef, Vec<u8>)]) -> Vec<(&ArtifactRef, &[u8])> {
        payloads.iter().map(|(reference, bytes)| (reference, bytes.as_slice())).collect()
    }

    fn corpus_payloads(
        manifest: &BenchmarkManifestV1,
        run_payloads: &[(ArtifactRef, Vec<u8>)],
    ) -> Vec<(ArtifactRef, Vec<u8>)> {
        vec![
            (run_payloads[3].0.clone(), run_payloads[3].1.clone()),
            (run_payloads[4].0.clone(), run_payloads[4].1.clone()),
            (
                manifest.body.cases[0].oracle_artifact.reference.clone(),
                ORACLE_BYTES.to_vec(),
            ),
        ]
    }

    fn run<'a>(
        tool_id: &'a str,
        case_id: &'a str,
        receipt: &'a AdapterRunReceiptV1,
        payloads: &'a [(&'a ArtifactRef, &'a [u8])],
        claims: &'a [ClaimObservationV1],
    ) -> ToolRunEvidence<'a> {
        ToolRunEvidence {
            tool_id,
            case_id,
            attempt: ToolAttemptDisposition::Succeeded,
            receipt: Some(receipt),
            payloads,
            observed_claims: claims,
            reason_code: None,
        }
    }

    #[test]
    fn full_grid_verifies_receipt_and_reports_false_pass_without_aggregate_score() {
        let (manifest, receipt, payloads) = fixture();
        let borrowed = borrowed(&payloads);
        let corpus_owned = corpus_payloads(&manifest, &payloads);
        let corpus_borrowed = borrowed(&corpus_owned);
        let observations = vec![
            ClaimObservationV1 {
                claim_id: "trace.characteristic_identity".into(),
                disposition: ClaimObservationDisposition::Accepted,
                reason_code: None,
            },
            ClaimObservationV1 {
                claim_id: "trace.wrong_document_join".into(),
                disposition: ClaimObservationDisposition::Accepted,
                reason_code: None,
            },
            ClaimObservationV1 {
                claim_id: "pmi.semantic_pmi_preserved".into(),
                disposition: ClaimObservationDisposition::Unknown,
                reason_code: Some("ORACLE_NOT_ADJUDICATED".into()),
            },
        ];
        let run = run(
            "synthetic-step-to-qif",
            "synthetic-ap242-qif-001",
            &receipt,
            &borrowed,
            &observations,
        );
        let score = evaluate_benchmark(&manifest, &corpus_borrowed, &[run]).unwrap();
        let tool = &score.tools[0];
        let identity = &tool.dimensions[&BenchmarkDimension::IdentityResolution];
        assert_eq!(identity.true_accepts, 1);
        assert_eq!(identity.false_accepts, 1);
        assert_eq!(identity.critical_false_accepts, 1);
        let semantic = &tool.dimensions[&BenchmarkDimension::SemanticPreservation];
        assert_eq!(semantic.oracle_unadjudicated, 1);
        assert_eq!(semantic.adjudicated, 0);
        assert_eq!(tool.attempts.succeeded, 1);
    }

    #[test]
    fn failed_or_timed_out_attempt_requires_a_reason_code() {
        let (manifest, mut receipt, payloads) = fixture();
        receipt.status = RunStatus::Failed;
        receipt.exit_code = Some(2);
        receipt.outputs.clear();
        receipt.receipt_sha256 = compute_receipt_digest(&receipt).unwrap();
        let borrowed = borrowed(&payloads[..5]);
        let corpus_owned = corpus_payloads(&manifest, &payloads);
        let corpus_borrowed = borrowed(&corpus_owned);
        let run = ToolRunEvidence {
            tool_id: "synthetic-step-to-qif",
            case_id: "synthetic-ap242-qif-001",
            attempt: ToolAttemptDisposition::Failed,
            receipt: Some(&receipt),
            payloads: &borrowed,
            observed_claims: &[],
            reason_code: None,
        };
        assert_eq!(evaluate_benchmark(&manifest, &corpus_borrowed, &[run]), Err(BenchmarkError::MissingAttemptReason));
    }

    #[test]
    fn exact_corpus_artifacts_are_verified_against_manifest() {
        let (manifest, _, payloads) = fixture();
        let oracle = ORACLE_BYTES.to_vec();
        let oracle_ref = manifest.body.cases[0].oracle_artifact.reference.clone();
        let mut corpus_payloads = vec![
            (&payloads[3].0, payloads[3].1.as_slice()),
            (&payloads[4].0, payloads[4].1.as_slice()),
            (&oracle_ref, oracle.as_slice()),
        ];
        assert_eq!(verify_benchmark_corpus_payloads(&manifest, &corpus_payloads).unwrap(), 3);
        corpus_payloads.pop();
        assert_eq!(
            verify_benchmark_corpus_payloads(&manifest, &corpus_payloads),
            Err(BenchmarkError::CorpusPayloadSetMismatch)
        );
    }

    #[test]
    fn changed_corpus_bytes_cannot_be_scored_against_frozen_manifest() {
        let (manifest, _, payloads) = fixture();
        let mut corpus_owned = corpus_payloads(&manifest, &payloads);
        corpus_owned[0].1.push(b'X');
        let corpus_borrowed = borrowed(&corpus_owned);
        assert!(matches!(
            verify_benchmark_corpus_payloads(&manifest, &corpus_borrowed),
            Err(BenchmarkError::CorpusArtifactVerification(_))
        ));
    }

    #[test]
    fn missing_tool_case_slot_is_not_removed_from_denominator() {
        let (manifest, _, payloads) = fixture();
        let corpus_owned = corpus_payloads(&manifest, &payloads);
        let corpus_borrowed = borrowed(&corpus_owned);
        assert_eq!(
            evaluate_benchmark(&manifest, &corpus_borrowed, &[]),
            Err(BenchmarkError::RunGridMismatch { expected: 1, actual: 0 })
        );
    }

    #[test]
    fn duplicate_claim_observation_is_rejected() {
        let (manifest, receipt, payloads) = fixture();
        let borrowed = borrowed(&payloads);
        let corpus_owned = corpus_payloads(&manifest, &payloads);
        let corpus_borrowed = borrowed(&corpus_owned);
        let observations = vec![
            ClaimObservationV1 { claim_id: "trace.characteristic_identity".into(), disposition: ClaimObservationDisposition::Accepted, reason_code: None },
            ClaimObservationV1 { claim_id: "trace.characteristic_identity".into(), disposition: ClaimObservationDisposition::Accepted, reason_code: None },
            ClaimObservationV1 { claim_id: "trace.wrong_document_join".into(), disposition: ClaimObservationDisposition::Rejected, reason_code: None },
            ClaimObservationV1 { claim_id: "pmi.semantic_pmi_preserved".into(), disposition: ClaimObservationDisposition::Unknown, reason_code: Some("ORACLE_NOT_ADJUDICATED".into()) },
        ];
        let run = run("synthetic-step-to-qif", "synthetic-ap242-qif-001", &receipt, &borrowed, &observations);
        assert!(matches!(evaluate_benchmark(&manifest, &corpus_borrowed, &[run]), Err(BenchmarkError::ClaimObservationSetMismatch)));
    }

    #[test]
    fn receipt_case_or_tool_identity_mismatch_rejects() {
        let (manifest, mut receipt, payloads) = fixture();
        receipt.case_id = "other-case".into();
        receipt.receipt_sha256 = compute_receipt_digest(&receipt).unwrap();
        let borrowed = borrowed(&payloads);
        let corpus_owned = corpus_payloads(&manifest, &payloads);
        let corpus_borrowed = borrowed(&corpus_owned);
        let observations = vec![
            ClaimObservationV1 { claim_id: "trace.characteristic_identity".into(), disposition: ClaimObservationDisposition::Accepted, reason_code: None },
            ClaimObservationV1 { claim_id: "trace.wrong_document_join".into(), disposition: ClaimObservationDisposition::Rejected, reason_code: None },
            ClaimObservationV1 { claim_id: "pmi.semantic_pmi_preserved".into(), disposition: ClaimObservationDisposition::Unknown, reason_code: Some("ORACLE_NOT_ADJUDICATED".into()) },
        ];
        let run = run("synthetic-step-to-qif", "synthetic-ap242-qif-001", &receipt, &borrowed, &observations);
        assert_eq!(evaluate_benchmark(&manifest, &corpus_borrowed, &[run]), Err(BenchmarkError::ReceiptIdentityMismatch));
    }

    #[test]
    fn manifest_digest_is_order_invariant_for_tools_cases_inputs_and_claims() {
        let (mut manifest, _, _) = fixture();
        let before = compute_benchmark_manifest_digest(&manifest).unwrap();
        manifest.body.cases.reverse();
        manifest.body.tools.reverse();
        manifest.body.cases[0].claims.reverse();
        manifest.body.cases[0].inputs.reverse();
        assert_eq!(compute_benchmark_manifest_digest(&manifest).unwrap(), before);
    }

    #[test]
    fn unreviewed_corpus_rights_are_rejected() {
        let (mut manifest, _, _) = fixture();
        manifest.body.cases[0].inputs[0].rights_status = RightsStatus::NotReviewed;
        manifest.manifest_sha256 = compute_benchmark_manifest_digest(&manifest).unwrap();
        assert!(matches!(validate_benchmark_manifest(&manifest), Err(BenchmarkError::RightsNotAdmitted(_))));
    }

    #[test]
    fn oracle_unadjudicated_is_not_scored_as_accept_or_reject() {
        let (manifest, receipt, payloads) = fixture();
        let borrowed = borrowed(&payloads);
        let corpus_owned = corpus_payloads(&manifest, &payloads);
        let corpus_borrowed = borrowed(&corpus_owned);
        let observations = vec![
            ClaimObservationV1 { claim_id: "trace.characteristic_identity".into(), disposition: ClaimObservationDisposition::Accepted, reason_code: None },
            ClaimObservationV1 { claim_id: "trace.wrong_document_join".into(), disposition: ClaimObservationDisposition::Rejected, reason_code: None },
            ClaimObservationV1 { claim_id: "pmi.semantic_pmi_preserved".into(), disposition: ClaimObservationDisposition::Accepted, reason_code: None },
        ];
        let run = run("synthetic-step-to-qif", "synthetic-ap242-qif-001", &receipt, &borrowed, &observations);
        let score = evaluate_benchmark(&manifest, &corpus_borrowed, &[run]).unwrap();
        let semantic = &score.tools[0].dimensions[&BenchmarkDimension::SemanticPreservation];
        assert_eq!(semantic.oracle_unadjudicated, 1);
        assert_eq!(semantic.true_accepts + semantic.false_accepts + semantic.true_rejects + semantic.false_rejects, 0);
    }

    #[test]
    fn strict_manifest_parser_rejects_unknown_fields() {
        let (manifest, _, _) = fixture();
        let mut value: Value = serde_json::to_value(&manifest).unwrap();
        value["body"]["hidden_vendor_override"] = Value::Bool(true);
        assert!(serde_json::from_value::<BenchmarkManifestV1>(value).is_err());
    }
}
