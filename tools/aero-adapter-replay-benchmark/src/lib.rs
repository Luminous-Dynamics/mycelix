//! Pairwise replay classification for content-verified AeroCommons adapter runs.
//!
//! This crate distinguishes execution-context comparability, exact output-byte
//! reproducibility and normalized-assertion byte stability. It deliberately
//! does not infer semantic preservation from byte equality or output drift.

use aero_adapter_run_receipt::{
    AdapterRunReceiptV1, InputRole, OutputRole, ReceiptError, RunInputV1, RunOutputV1, RunStatus,
    verify_artifact_payloads,
};
use aero_evidence_types::ArtifactRef;
use thiserror::Error;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReplayDisposition {
    /// Both successful runs consumed the same declared context and every output
    /// artifact has the same exact bytes.
    ExactByteReplay,
    /// Both runs consumed the same context. Normalized assertion bytes match,
    /// but at least one raw output artifact differs byte-for-byte.
    AssertionsByteStableRawOutputDrift,
    /// Both runs consumed the same context, but normalized assertion bytes
    /// differ. Semantic interpretation belongs to an independent evaluator.
    NormalizedAssertionsByteDrift,
    /// The two receipts do not describe the same benchmark case or execution
    /// context. They are not a same-adapter replay pair.
    IncomparableContext,
    /// At least one run failed or timed out. It remains in the offered-run
    /// denominator but cannot establish a successful replay.
    RunNotEligible,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum ContextDifference {
    BenchmarkId,
    CaseId,
    AdapterId,
    AdapterVersion,
    ExecutableArtifact,
    BuildProvenance,
    ArgumentVector,
    EnvironmentProfile,
    Configuration,
    Locale,
    Timezone,
    NetworkPolicy,
    DeclaredEgress,
    ObservedEgress,
    Timeout,
    InputArtifacts,
    InputRights,
    OutputProfile,
    OutputRights,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ReplayComparison {
    pub first_run_id: String,
    pub second_run_id: String,
    pub first_status: RunStatus,
    pub second_status: RunStatus,
    pub disposition: ReplayDisposition,
    pub context_differences: Vec<ContextDifference>,
    /// None when one or both runs are not successful.
    pub all_output_bytes_identical: Option<bool>,
    /// None when one or both runs are not successful.
    pub raw_output_bytes_identical: Option<bool>,
    /// None when one or both runs are not successful.
    pub normalized_assertion_bytes_identical: Option<bool>,
    pub first_checked_artifacts: usize,
    pub second_checked_artifacts: usize,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ReplayPanelSummary {
    pub offered_pairs: usize,
    pub exact_byte_replays: usize,
    pub assertion_stable_raw_drift: usize,
    pub normalized_assertion_drift: usize,
    pub incomparable_context: usize,
    pub run_not_eligible: usize,
}

impl ReplayPanelSummary {
    /// Counts every offered pair. Nothing is silently removed from the panel.
    pub fn from_comparisons(comparisons: &[ReplayComparison]) -> Self {
        let mut summary = Self {
            offered_pairs: comparisons.len(),
            exact_byte_replays: 0,
            assertion_stable_raw_drift: 0,
            normalized_assertion_drift: 0,
            incomparable_context: 0,
            run_not_eligible: 0,
        };
        for comparison in comparisons {
            match comparison.disposition {
                ReplayDisposition::ExactByteReplay => summary.exact_byte_replays += 1,
                ReplayDisposition::AssertionsByteStableRawOutputDrift => {
                    summary.assertion_stable_raw_drift += 1
                }
                ReplayDisposition::NormalizedAssertionsByteDrift => {
                    summary.normalized_assertion_drift += 1
                }
                ReplayDisposition::IncomparableContext => summary.incomparable_context += 1,
                ReplayDisposition::RunNotEligible => summary.run_not_eligible += 1,
            }
        }
        summary
    }
}

#[derive(Debug, Error, PartialEq, Eq)]
pub enum ReplayError {
    #[error("receipt validation failed for run {run_id}: {source}")]
    ReceiptValidation {
        run_id: String,
        source: ReceiptError,
    },
    #[error("a replay pair must have distinct run IDs: {0}")]
    DuplicateRunIdentity(String),
}

/// Compare two runs only after independently verifying the exact payload set
/// declared by each receipt. A successful return is not a semantic verdict.
pub fn compare_verified_runs(
    first: &AdapterRunReceiptV1,
    first_payloads: &[(&ArtifactRef, &[u8])],
    second: &AdapterRunReceiptV1,
    second_payloads: &[(&ArtifactRef, &[u8])],
) -> Result<ReplayComparison, ReplayError> {
    let first_verified = verify_artifact_payloads(first, first_payloads).map_err(|source| {
        ReplayError::ReceiptValidation {
            run_id: first.run_id.clone(),
            source,
        }
    })?;
    let second_verified = verify_artifact_payloads(second, second_payloads).map_err(|source| {
        ReplayError::ReceiptValidation {
            run_id: second.run_id.clone(),
            source,
        }
    })?;

    if first.run_id == second.run_id {
        return Err(ReplayError::DuplicateRunIdentity(first.run_id.clone()));
    }

    let context_differences = context_differences(first, second);
    let both_successful =
        first_verified.semantic_output_eligible && second_verified.semantic_output_eligible;

    let (
        all_output_bytes_identical,
        raw_output_bytes_identical,
        normalized_assertion_bytes_identical,
    ) = if both_successful {
        let first_outputs = sorted_outputs(&first.outputs);
        let second_outputs = sorted_outputs(&second.outputs);
        let all_equal = output_bytes_equal(&first_outputs, &second_outputs);
        let raw_equal = raw_output_bytes_equal(&first_outputs, &second_outputs);
        let assertions_equal = normalized_assertion_bytes_equal(&first_outputs, &second_outputs);
        (Some(all_equal), Some(raw_equal), Some(assertions_equal))
    } else {
        (None, None, None)
    };

    let disposition = if !both_successful {
        ReplayDisposition::RunNotEligible
    } else if !context_differences.is_empty() {
        ReplayDisposition::IncomparableContext
    } else if all_output_bytes_identical == Some(true) {
        ReplayDisposition::ExactByteReplay
    } else if normalized_assertion_bytes_identical == Some(true) {
        ReplayDisposition::AssertionsByteStableRawOutputDrift
    } else {
        ReplayDisposition::NormalizedAssertionsByteDrift
    };

    Ok(ReplayComparison {
        first_run_id: first.run_id.clone(),
        second_run_id: second.run_id.clone(),
        first_status: first.status,
        second_status: second.status,
        disposition,
        context_differences,
        all_output_bytes_identical,
        raw_output_bytes_identical,
        normalized_assertion_bytes_identical,
        first_checked_artifacts: first_verified.checked_artifacts,
        second_checked_artifacts: second_verified.checked_artifacts,
    })
}

fn context_differences(
    first: &AdapterRunReceiptV1,
    second: &AdapterRunReceiptV1,
) -> Vec<ContextDifference> {
    let mut differences = Vec::new();
    if first.benchmark_id != second.benchmark_id {
        differences.push(ContextDifference::BenchmarkId);
    }
    if first.case_id != second.case_id {
        differences.push(ContextDifference::CaseId);
    }
    if first.adapter.adapter_id != second.adapter.adapter_id {
        differences.push(ContextDifference::AdapterId);
    }
    if first.adapter.version != second.adapter.version {
        differences.push(ContextDifference::AdapterVersion);
    }
    if first.adapter.executable != second.adapter.executable {
        differences.push(ContextDifference::ExecutableArtifact);
    }
    if first.adapter.build_provenance != second.adapter.build_provenance {
        differences.push(ContextDifference::BuildProvenance);
    }

    if first.invocation.arguments != second.invocation.arguments {
        differences.push(ContextDifference::ArgumentVector);
    }
    if first.invocation.environment_profile != second.invocation.environment_profile {
        differences.push(ContextDifference::EnvironmentProfile);
    }
    if first.invocation.configuration_sha256 != second.invocation.configuration_sha256 {
        differences.push(ContextDifference::Configuration);
    }
    if first.invocation.locale != second.invocation.locale {
        differences.push(ContextDifference::Locale);
    }
    if first.invocation.timezone != second.invocation.timezone {
        differences.push(ContextDifference::Timezone);
    }
    if first.invocation.network_policy != second.invocation.network_policy {
        differences.push(ContextDifference::NetworkPolicy);
    }
    if sorted_strings(&first.invocation.declared_egress)
        != sorted_strings(&second.invocation.declared_egress)
    {
        differences.push(ContextDifference::DeclaredEgress);
    }
    if sorted_strings(&first.invocation.observed_egress)
        != sorted_strings(&second.invocation.observed_egress)
    {
        differences.push(ContextDifference::ObservedEgress);
    }
    if first.invocation.timeout_ms != second.invocation.timeout_ms {
        differences.push(ContextDifference::Timeout);
    }

    let first_inputs = sorted_inputs(&first.inputs);
    let second_inputs = sorted_inputs(&second.inputs);
    if !input_artifacts_equal(&first_inputs, &second_inputs) {
        differences.push(ContextDifference::InputArtifacts);
    }
    if !input_rights_equal(&first_inputs, &second_inputs) {
        differences.push(ContextDifference::InputRights);
    }

    let first_outputs = sorted_outputs(&first.outputs);
    let second_outputs = sorted_outputs(&second.outputs);
    if !output_profiles_equal(&first_outputs, &second_outputs) {
        differences.push(ContextDifference::OutputProfile);
    }
    if !output_rights_equal(&first_outputs, &second_outputs) {
        differences.push(ContextDifference::OutputRights);
    }
    differences
}

fn sorted_strings(values: &[String]) -> Vec<&str> {
    let mut sorted: Vec<_> = values.iter().map(String::as_str).collect();
    sorted.sort_unstable();
    sorted
}

fn input_role_key(role: InputRole) -> &'static str {
    match role {
        InputRole::Ap242ProductDefinition => "ap242_product_definition",
        InputRole::QifInput => "qif_input",
        InputRole::NcProgram => "nc_program",
        InputRole::MetrologyPlan => "metrology_plan",
        InputRole::AdapterConfiguration => "adapter_configuration",
        InputRole::Auxiliary => "auxiliary",
    }
}

fn output_role_key(role: OutputRole) -> &'static str {
    match role {
        OutputRole::RawArtifact => "raw_artifact",
        OutputRole::NormalizedAssertions => "normalized_assertions",
    }
}

fn sorted_inputs(inputs: &[RunInputV1]) -> Vec<&RunInputV1> {
    let mut sorted: Vec<_> = inputs.iter().collect();
    sorted.sort_by(|left, right| {
        (
            input_role_key(left.role),
            left.artifact.reference.id.as_str(),
            left.artifact.reference.kind.as_str(),
        )
            .cmp(&(
                input_role_key(right.role),
                right.artifact.reference.id.as_str(),
                right.artifact.reference.kind.as_str(),
            ))
    });
    sorted
}

fn sorted_outputs(outputs: &[RunOutputV1]) -> Vec<&RunOutputV1> {
    let mut sorted: Vec<_> = outputs.iter().collect();
    sorted.sort_by(|left, right| {
        (
            output_role_key(left.role),
            left.artifact.reference.id.as_str(),
            left.artifact.reference.kind.as_str(),
        )
            .cmp(&(
                output_role_key(right.role),
                right.artifact.reference.id.as_str(),
                right.artifact.reference.kind.as_str(),
            ))
    });
    sorted
}

fn input_artifacts_equal(first: &[&RunInputV1], second: &[&RunInputV1]) -> bool {
    first.len() == second.len()
        && first
            .iter()
            .zip(second)
            .all(|(left, right)| left.role == right.role && left.artifact == right.artifact)
}

fn rights_status_key(status: aero_adapter_run_receipt::RightsStatus) -> &'static str {
    match status {
        aero_adapter_run_receipt::RightsStatus::ClearedForTesting => "CLEARED_FOR_TESTING",
        aero_adapter_run_receipt::RightsStatus::ClearedForRedistribution => {
            "CLEARED_FOR_REDISTRIBUTION"
        }
        aero_adapter_run_receipt::RightsStatus::NotReviewed => "NOT_REVIEWED",
        aero_adapter_run_receipt::RightsStatus::Restricted => "RESTRICTED",
    }
}

fn input_rights_equal(first: &[&RunInputV1], second: &[&RunInputV1]) -> bool {
    let mut first_rights: Vec<_> = first
        .iter()
        .map(|item| {
            (
                input_role_key(item.role),
                rights_status_key(item.rights_status),
            )
        })
        .collect();
    let mut second_rights: Vec<_> = second
        .iter()
        .map(|item| {
            (
                input_role_key(item.role),
                rights_status_key(item.rights_status),
            )
        })
        .collect();
    first_rights.sort_unstable();
    second_rights.sort_unstable();
    first_rights == second_rights
}

fn output_profiles_equal(first: &[&RunOutputV1], second: &[&RunOutputV1]) -> bool {
    first.len() == second.len()
        && first.iter().zip(second).all(|(left, right)| {
            left.role == right.role
                && left.artifact.reference.kind == right.artifact.reference.kind
                && left.artifact.media_type == right.artifact.media_type
                && left.artifact.format_id == right.artifact.format_id
        })
}

fn output_rights_equal(first: &[&RunOutputV1], second: &[&RunOutputV1]) -> bool {
    let mut first_rights: Vec<_> = first
        .iter()
        .map(|item| {
            (
                output_role_key(item.role),
                rights_status_key(item.rights_status),
            )
        })
        .collect();
    let mut second_rights: Vec<_> = second
        .iter()
        .map(|item| {
            (
                output_role_key(item.role),
                rights_status_key(item.rights_status),
            )
        })
        .collect();
    first_rights.sort_unstable();
    second_rights.sort_unstable();
    first_rights == second_rights
}

fn output_bytes_equal(first: &[&RunOutputV1], second: &[&RunOutputV1]) -> bool {
    first.len() == second.len()
        && first.iter().zip(second).all(|(left, right)| {
            left.role == right.role
                && left.artifact.sha256 == right.artifact.sha256
                && left.artifact.byte_length == right.artifact.byte_length
        })
}

fn raw_output_bytes_equal(first: &[&RunOutputV1], second: &[&RunOutputV1]) -> bool {
    let first_raw: Vec<_> = first
        .iter()
        .filter(|item| item.role == OutputRole::RawArtifact)
        .collect();
    let second_raw: Vec<_> = second
        .iter()
        .filter(|item| item.role == OutputRole::RawArtifact)
        .collect();
    first_raw.len() == second_raw.len()
        && first_raw.iter().zip(second_raw).all(|(left, right)| {
            left.artifact.sha256 == right.artifact.sha256
                && left.artifact.byte_length == right.artifact.byte_length
        })
}

fn normalized_assertion_bytes_equal(first: &[&RunOutputV1], second: &[&RunOutputV1]) -> bool {
    let first_assertions = first
        .iter()
        .find(|item| item.role == OutputRole::NormalizedAssertions);
    let second_assertions = second
        .iter()
        .find(|item| item.role == OutputRole::NormalizedAssertions);
    match (first_assertions, second_assertions) {
        (Some(left), Some(right)) => {
            left.artifact.sha256 == right.artifact.sha256
                && left.artifact.byte_length == right.artifact.byte_length
        }
        _ => false,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use aero_adapter_run_receipt::{
        AdapterIdentityV1, ContentArtifactV1, InvocationV1, NetworkPolicy, RightsStatus,
        compute_receipt_digest,
    };
    use sha2::{Digest, Sha256};

    struct Fixture {
        receipt: AdapterRunReceiptV1,
        payloads: Vec<(ArtifactRef, Vec<u8>)>,
    }

    fn content_artifact(kind: &str, format: &str, media: &str, bytes: &[u8]) -> ContentArtifactV1 {
        let digest = Sha256::digest(bytes);
        let sha256 = digest
            .iter()
            .map(|byte| format!("{byte:02x}"))
            .collect::<String>();
        ContentArtifactV1 {
            reference: ArtifactRef {
                id: format!("sha256:{sha256}"),
                kind: kind.into(),
            },
            sha256,
            byte_length: bytes.len() as u64,
            media_type: media.into(),
            format_id: format.into(),
        }
    }

    fn fixture(
        run_id: &str,
        adapter_bytes: &[u8],
        source_bytes: &[u8],
        configuration_bytes: &[u8],
        raw_output_bytes: &[u8],
        assertion_bytes: &[u8],
        rights_status: RightsStatus,
        status: RunStatus,
    ) -> Fixture {
        let executable_bytes = adapter_bytes;
        let argument_bytes = br#"["--input","source.step","--config","adapter.json"]"#;
        let environment_bytes = br#"{"locale":"C","timezone":"UTC","network_policy":"disabled"}"#;

        let executable = content_artifact(
            "adapter_executable",
            "SYNTHETIC_ADAPTER_V1",
            "application/octet-stream",
            executable_bytes,
        );
        let arguments = content_artifact(
            "adapter_arguments",
            "AERO_ARGUMENT_VECTOR_V1",
            "application/json",
            argument_bytes,
        );
        let environment = content_artifact(
            "environment_profile",
            "AERO_ENVIRONMENT_PROFILE_V1",
            "application/json",
            environment_bytes,
        );
        let source = content_artifact(
            "step_file",
            "ISO_10303_242:2025",
            "application/step",
            source_bytes,
        );
        let configuration = content_artifact(
            "adapter_config",
            "AERO_ADAPTER_CONFIG_V1",
            "application/json",
            configuration_bytes,
        );
        let raw_output = content_artifact(
            "qif_result",
            "ISO_23952:2020",
            "application/xml",
            raw_output_bytes,
        );
        let assertions = content_artifact(
            "semantic_assertions",
            "AeroSemanticAssertionV1",
            "application/json",
            assertion_bytes,
        );

        let inputs = vec![
            RunInputV1 {
                role: InputRole::Ap242ProductDefinition,
                artifact: source.clone(),
                rights_status,
            },
            RunInputV1 {
                role: InputRole::AdapterConfiguration,
                artifact: configuration.clone(),
                rights_status,
            },
        ];
        let outputs = if status == RunStatus::TimedOut {
            Vec::new()
        } else {
            vec![
                RunOutputV1 {
                    role: OutputRole::RawArtifact,
                    artifact: raw_output.clone(),
                    rights_status,
                },
                RunOutputV1 {
                    role: OutputRole::NormalizedAssertions,
                    artifact: assertions.clone(),
                    rights_status,
                },
            ]
        };

        let mut receipt = AdapterRunReceiptV1 {
            schema: "AeroAdapterRunReceiptV1".into(),
            run_id: run_id.into(),
            benchmark_id: "AERO-ADAPTER-REPLAY-001".into(),
            case_id: "case-ap242-qif-001".into(),
            adapter: AdapterIdentityV1 {
                adapter_id: "fixture-step-qif-adapter".into(),
                version: "0.1.0".into(),
                executable: executable.clone(),
                build_provenance: None,
            },
            inputs,
            invocation: InvocationV1 {
                arguments: arguments.clone(),
                environment_profile: environment.clone(),
                configuration_sha256: configuration.sha256.clone(),
                locale: "C".into(),
                timezone: "UTC".into(),
                network_policy: NetworkPolicy::Disabled,
                declared_egress: Vec::new(),
                observed_egress: Vec::new(),
                timeout_ms: 30_000,
            },
            outputs,
            status,
            exit_code: match status {
                RunStatus::Succeeded => Some(0),
                RunStatus::Failed => Some(2),
                RunStatus::TimedOut => None,
            },
            elapsed_ms: 125,
            warnings: Vec::new(),
            unsupported_assertions: Vec::new(),
            receipt_sha256: String::new(),
        };
        receipt.receipt_sha256 =
            compute_receipt_digest(&receipt).expect("fixture canonical digest");

        let mut payloads = vec![
            (executable.reference.clone(), executable_bytes.to_vec()),
            (arguments.reference.clone(), argument_bytes.to_vec()),
            (environment.reference.clone(), environment_bytes.to_vec()),
            (source.reference.clone(), source_bytes.to_vec()),
            (
                configuration.reference.clone(),
                configuration_bytes.to_vec(),
            ),
        ];
        if status != RunStatus::TimedOut {
            payloads.push((raw_output.reference.clone(), raw_output_bytes.to_vec()));
            payloads.push((assertions.reference.clone(), assertion_bytes.to_vec()));
        }

        Fixture { receipt, payloads }
    }

    fn borrowed(fixture: &Fixture) -> Vec<(&ArtifactRef, &[u8])> {
        fixture
            .payloads
            .iter()
            .map(|(reference, bytes)| (reference, bytes.as_slice()))
            .collect()
    }

    fn compare(first: &Fixture, second: &Fixture) -> Result<ReplayComparison, ReplayError> {
        compare_verified_runs(
            &first.receipt,
            &borrowed(first),
            &second.receipt,
            &borrowed(second),
        )
    }

    fn base(run_id: &str) -> Fixture {
        fixture(
            run_id,
            b"adapter-executable-v1",
            b"ISO-10303-21; synthetic product definition",
            b"{\"profile\":\"default\"}\n",
            b"<QIFDocument schemaVersion=\"3.0\"><Results/></QIFDocument>",
            br#"{"schema":"AeroSemanticAssertionV1","assertions":[]}"#,
            RightsStatus::ClearedForTesting,
            RunStatus::Succeeded,
        )
    }

    #[test]
    fn exact_replay_is_byte_identical() {
        let first = base("run-a");
        let second = base("run-b");
        let report = compare(&first, &second).unwrap();
        assert_eq!(report.disposition, ReplayDisposition::ExactByteReplay);
        assert_eq!(report.all_output_bytes_identical, Some(true));
        assert_eq!(report.raw_output_bytes_identical, Some(true));
        assert_eq!(report.normalized_assertion_bytes_identical, Some(true));
        assert!(report.context_differences.is_empty());
        assert_eq!(report.first_checked_artifacts, 7);
        assert_eq!(report.second_checked_artifacts, 7);
    }

    #[test]
    fn list_order_does_not_change_replay_context_or_output_result() {
        let first = base("run-a");
        let mut second = base("run-b");
        second.receipt.inputs.reverse();
        second.receipt.outputs.reverse();
        second.receipt.receipt_sha256 = compute_receipt_digest(&second.receipt).unwrap();
        assert_eq!(
            compare(&first, &second).unwrap().disposition,
            ReplayDisposition::ExactByteReplay
        );
    }

    #[test]
    fn raw_qif_drift_with_identical_assertions_is_separated() {
        let first = base("run-a");
        let second = fixture(
            "run-b",
            b"adapter-executable-v1",
            b"ISO-10303-21; synthetic product definition",
            b"{\"profile\":\"default\"}\n",
            b"<QIFDocument timestamp=\"different\"><Results/></QIFDocument>",
            br#"{"schema":"AeroSemanticAssertionV1","assertions":[]}"#,
            RightsStatus::ClearedForTesting,
            RunStatus::Succeeded,
        );
        let report = compare(&first, &second).unwrap();
        assert_eq!(
            report.disposition,
            ReplayDisposition::AssertionsByteStableRawOutputDrift
        );
        assert_eq!(report.raw_output_bytes_identical, Some(false));
        assert_eq!(report.normalized_assertion_bytes_identical, Some(true));
    }

    #[test]
    fn normalized_assertion_drift_is_not_called_semantic_loss() {
        let first = base("run-a");
        let second = fixture(
            "run-b",
            b"adapter-executable-v1",
            b"ISO-10303-21; synthetic product definition",
            b"{\"profile\":\"default\"}\n",
            b"<QIFDocument schemaVersion=\"3.0\"><Results/></QIFDocument>",
            br#"{"schema":"AeroSemanticAssertionV1","assertions":[{"assertion_id":"A1"}]}"#,
            RightsStatus::ClearedForTesting,
            RunStatus::Succeeded,
        );
        let report = compare(&first, &second).unwrap();
        assert_eq!(
            report.disposition,
            ReplayDisposition::NormalizedAssertionsByteDrift
        );
        assert_eq!(report.raw_output_bytes_identical, Some(true));
        assert_eq!(report.normalized_assertion_bytes_identical, Some(false));
    }

    #[test]
    fn changed_adapter_executable_is_incomparable_not_nondeterministic() {
        let first = base("run-a");
        let second = fixture(
            "run-b",
            b"adapter-executable-v2",
            b"ISO-10303-21; synthetic product definition",
            b"{\"profile\":\"default\"}\n",
            b"<QIFDocument schemaVersion=\"3.0\"><Results/></QIFDocument>",
            br#"{"schema":"AeroSemanticAssertionV1","assertions":[]}"#,
            RightsStatus::ClearedForTesting,
            RunStatus::Succeeded,
        );
        let report = compare(&first, &second).unwrap();
        assert_eq!(report.disposition, ReplayDisposition::IncomparableContext);
        assert!(
            report
                .context_differences
                .contains(&ContextDifference::ExecutableArtifact)
        );
    }

    #[test]
    fn changed_source_input_is_incomparable() {
        let first = base("run-a");
        let second = fixture(
            "run-b",
            b"adapter-executable-v1",
            b"ISO-10303-21; different source bytes",
            b"{\"profile\":\"default\"}\n",
            b"<QIFDocument schemaVersion=\"3.0\"><Results/></QIFDocument>",
            br#"{"schema":"AeroSemanticAssertionV1","assertions":[]}"#,
            RightsStatus::ClearedForTesting,
            RunStatus::Succeeded,
        );
        let report = compare(&first, &second).unwrap();
        assert_eq!(report.disposition, ReplayDisposition::IncomparableContext);
        assert!(
            report
                .context_differences
                .contains(&ContextDifference::InputArtifacts)
        );
    }

    #[test]
    fn changed_configuration_is_incomparable() {
        let first = base("run-a");
        let second = fixture(
            "run-b",
            b"adapter-executable-v1",
            b"ISO-10303-21; synthetic product definition",
            b"{\"profile\":\"changed\"}\n",
            b"<QIFDocument schemaVersion=\"3.0\"><Results/></QIFDocument>",
            br#"{"schema":"AeroSemanticAssertionV1","assertions":[]}"#,
            RightsStatus::ClearedForTesting,
            RunStatus::Succeeded,
        );
        let report = compare(&first, &second).unwrap();
        assert_eq!(report.disposition, ReplayDisposition::IncomparableContext);
        assert!(
            report
                .context_differences
                .contains(&ContextDifference::Configuration)
        );
        assert!(
            report
                .context_differences
                .contains(&ContextDifference::InputArtifacts)
        );
    }

    #[test]
    fn rights_declaration_change_is_context_change() {
        let first = base("run-a");
        let second = fixture(
            "run-b",
            b"adapter-executable-v1",
            b"ISO-10303-21; synthetic product definition",
            b"{\"profile\":\"default\"}\n",
            b"<QIFDocument schemaVersion=\"3.0\"><Results/></QIFDocument>",
            br#"{"schema":"AeroSemanticAssertionV1","assertions":[]}"#,
            RightsStatus::ClearedForRedistribution,
            RunStatus::Succeeded,
        );
        let report = compare(&first, &second).unwrap();
        assert_eq!(report.disposition, ReplayDisposition::IncomparableContext);
        assert!(
            report
                .context_differences
                .contains(&ContextDifference::InputRights)
        );
        assert!(
            report
                .context_differences
                .contains(&ContextDifference::OutputRights)
        );
    }

    #[test]
    fn changed_argument_vector_is_context_change() {
        let first = base("run-a");
        let mut second = base("run-b");
        let bytes = br#"["--different-argument"]"#;
        let artifact = content_artifact(
            "adapter_arguments",
            "AERO_ARGUMENT_VECTOR_V1",
            "application/json",
            bytes,
        );
        second.receipt.invocation.arguments = artifact.clone();
        second
            .payloads
            .retain(|(reference, _)| reference.kind != "adapter_arguments");
        second
            .payloads
            .push((artifact.reference.clone(), bytes.to_vec()));
        second.receipt.receipt_sha256 = compute_receipt_digest(&second.receipt).unwrap();
        let report = compare(&first, &second).unwrap();
        assert_eq!(report.disposition, ReplayDisposition::IncomparableContext);
        assert!(
            report
                .context_differences
                .contains(&ContextDifference::ArgumentVector)
        );
    }

    #[test]
    fn failed_run_is_preserved_but_not_eligible() {
        let first = base("run-a");
        let second = fixture(
            "run-b",
            b"adapter-executable-v1",
            b"ISO-10303-21; synthetic product definition",
            b"{\"profile\":\"default\"}\n",
            b"<QIFDocument/>",
            br#"{"schema":"AeroSemanticAssertionV1","assertions":[]}"#,
            RightsStatus::ClearedForTesting,
            RunStatus::Failed,
        );
        let report = compare(&first, &second).unwrap();
        assert_eq!(report.disposition, ReplayDisposition::RunNotEligible);
        assert_eq!(report.all_output_bytes_identical, None);
        assert_eq!(report.second_status, RunStatus::Failed);
    }

    #[test]
    fn timed_out_run_is_preserved_but_not_eligible() {
        let first = base("run-a");
        let second = fixture(
            "run-b",
            b"adapter-executable-v1",
            b"ISO-10303-21; synthetic product definition",
            b"{\"profile\":\"default\"}\n",
            b"",
            b"",
            RightsStatus::ClearedForTesting,
            RunStatus::TimedOut,
        );
        assert_eq!(
            compare(&first, &second).unwrap().disposition,
            ReplayDisposition::RunNotEligible
        );
    }

    #[test]
    fn duplicate_run_identity_is_rejected() {
        let first = base("same-run");
        let second = base("same-run");
        assert_eq!(
            compare(&first, &second),
            Err(ReplayError::DuplicateRunIdentity("same-run".into()))
        );
    }

    #[test]
    fn tampered_receipt_digest_is_rejected_before_comparison() {
        let first = base("run-a");
        let mut second = base("run-b");
        second.receipt.receipt_sha256 = "0".repeat(64);
        assert!(matches!(
            compare(&first, &second),
            Err(ReplayError::ReceiptValidation {
                source: ReceiptError::ReceiptDigestMismatch,
                ..
            })
        ));
    }

    #[test]
    fn missing_payload_is_rejected_before_comparison() {
        let first = base("run-a");
        let second = base("run-b");
        let mut payloads = borrowed(&second);
        payloads.pop();
        assert!(matches!(
            compare_verified_runs(
                &first.receipt,
                &borrowed(&first),
                &second.receipt,
                &payloads
            ),
            Err(ReplayError::ReceiptValidation {
                source: ReceiptError::MissingArtifactPayload(_),
                ..
            })
        ));
    }

    #[test]
    fn different_network_policy_cannot_be_called_a_replay() {
        let first = base("run-a");
        let mut second = base("run-b");
        let bytes = br#"{"locale":"C","timezone":"UTC","network_policy":"allowlisted"}"#;
        let environment = content_artifact(
            "environment_profile",
            "AERO_ENVIRONMENT_PROFILE_V1",
            "application/json",
            bytes,
        );
        second.receipt.invocation.environment_profile = environment.clone();
        second.receipt.invocation.network_policy = NetworkPolicy::Allowlisted;
        second.receipt.invocation.declared_egress = vec!["https://approved.example".into()];
        second.receipt.invocation.observed_egress = vec!["https://approved.example".into()];
        second
            .payloads
            .retain(|(reference, _)| reference.kind != "environment_profile");
        second
            .payloads
            .push((environment.reference.clone(), bytes.to_vec()));
        second.receipt.receipt_sha256 = compute_receipt_digest(&second.receipt).unwrap();
        let report = compare(&first, &second).unwrap();
        assert_eq!(report.disposition, ReplayDisposition::IncomparableContext);
        assert!(
            report
                .context_differences
                .contains(&ContextDifference::EnvironmentProfile)
        );
        assert!(
            report
                .context_differences
                .contains(&ContextDifference::NetworkPolicy)
        );
    }

    #[test]
    fn panel_summary_keeps_ineligible_and_incomparable_pairs() {
        let first = base("run-a");
        let exact = base("run-b");
        let timeout = fixture(
            "run-c",
            b"adapter-executable-v1",
            b"ISO-10303-21; synthetic product definition",
            b"{\"profile\":\"default\"}\n",
            b"",
            b"",
            RightsStatus::ClearedForTesting,
            RunStatus::TimedOut,
        );
        let comparisons = vec![
            compare(&first, &exact).unwrap(),
            compare(&first, &timeout).unwrap(),
        ];
        let summary = ReplayPanelSummary::from_comparisons(&comparisons);
        assert_eq!(summary.offered_pairs, 2);
        assert_eq!(summary.exact_byte_replays, 1);
        assert_eq!(summary.run_not_eligible, 1);
    }
}
