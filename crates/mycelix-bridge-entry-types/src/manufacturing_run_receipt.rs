// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Strict process-run evidence receipt for manufacturing experiments.
//!
//! This module validates a receipt's declared structure and evidence-state
//! consistency. It does not authenticate signers, read or hash referenced bytes,
//! prove a calibration record genuine, validate a physical model, or authorize
//! machine actuation / production release.

use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;

pub const PROCESS_RUN_RECEIPT_SCHEMA_VERSION: u8 = 1;
pub const MAX_ID_BYTES: usize = 256;
pub const MAX_VERSION_BYTES: usize = 128;
pub const MAX_INPUT_ARTIFACTS: usize = 64;
pub const MAX_OBSERVATIONS: usize = 256;
pub const MAX_ASSUMPTIONS: usize = 64;

/// Origin of a run's result. These origin classes are not interchangeable.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum EvidenceOrigin {
    SyntheticFixture,
    SimulationOutput,
    InstrumentObserved,
}

/// Whether this receipt claims the measurement system was calibrated for the
/// referenced observation time. This is still a claim, not calibration proof.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CalibrationClaim {
    NotClaimed,
    ClaimedCurrentAtObservation,
}

/// Independent review disposition. Pass means only that the identified
/// verifier passed its declared review; it is not safety approval or release.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ReviewDisposition {
    NotReviewed,
    Pass,
    Fail,
    Indeterminate,
}

/// Immutable reference to one modelled or instrument-observed process run.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ProcessRunReceipt {
    pub schema_version: u8,
    pub receipt_id: String,
    pub run_id: String,
    pub policy_id: String,
    /// Identity of the actor or process asserting this receipt.
    pub producer_id: String,
    pub scenario_id: String,
    pub seed: u64,
    pub created_at_unix_ms: u64,

    /// Stable canonical workload identity. The verifier checks syntax only.
    pub workload_sha256: String,
    /// Exact source revision for the model/tool, 40- or 64-character Git hex.
    pub source_commit_hex: String,
    pub tool_name: String,
    pub tool_version: String,
    /// Digest of the built executable/package where available.
    pub build_sha256: String,
    pub configuration_sha256: String,
    /// Digests of all input artifacts; contents are not embedded here.
    pub input_artifact_sha256: Vec<String>,
    /// Digest of the exact report artifact referenced by this receipt.
    pub report_sha256: String,

    pub evidence_origin: EvidenceOrigin,
    pub model_domain: String,
    pub assumption_ids: Vec<String>,
    /// Explicit out-of-domain result. It must never be hidden by a passing
    /// structural verifier or an independently reviewed report.
    pub out_of_domain: bool,

    /// Optional measurement provenance. Simulated/synthetic records may not
    /// populate instrument, calibration, or observation fields.
    pub measurement_system_id: Option<String>,
    pub instrument_id: Option<String>,
    pub observation_ids: Vec<String>,
    /// One raw-observation artifact digest per observation ID.
    pub observation_artifact_sha256: Vec<String>,
    pub observed_at_unix_ms: Option<u64>,
    pub calibration_claim: CalibrationClaim,
    pub calibration_record_sha256: Option<String>,
    pub calibration_valid_from_unix_ms: Option<u64>,
    pub calibration_valid_until_unix_ms: Option<u64>,

    pub review_disposition: ReviewDisposition,
    pub reviewer_id: Option<String>,
    pub reviewer_version: Option<String>,
    /// Must bind the review to this exact report digest.
    pub reviewed_report_sha256: Option<String>,
    pub reviewed_at_unix_ms: Option<u64>,

    /// Immutable ancestry references to prior receipts, if applicable.
    pub parent_receipt_sha256: Option<String>,
    pub supersedes_receipt_sha256: Option<String>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ReceiptValidationError {
    UnsupportedSchemaVersion(u8),
    InvalidSerializedReceipt,
    MissingOrOversizeField(&'static str),
    InvalidDigest(&'static str),
    InvalidGitCommit,
    TooManyItems(&'static str),
    DuplicateItem(&'static str),
    InconsistentEvidenceState(&'static str),
    MissingInstrumentObservation(&'static str),
    InvalidCalibrationInterval,
    ObservationOutsideCalibrationInterval,
    InvalidReviewState(&'static str),
}

impl std::fmt::Display for ReceiptValidationError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::UnsupportedSchemaVersion(v) => {
                write!(f, "unsupported process-run receipt schema version {v}")
            }
            Self::InvalidSerializedReceipt => {
                write!(f, "serialized receipt is malformed or violates strict JSON field rules")
            }
            Self::MissingOrOversizeField(field) => {
                write!(f, "required field {field} is missing, blank, or oversized")
            }
            Self::InvalidDigest(field) => {
                write!(f, "{field} must be a 64-character lowercase SHA-256 digest")
            }
            Self::InvalidGitCommit => write!(
                f,
                "source_commit_hex must be 40 or 64 lowercase hexadecimal characters"
            ),
            Self::TooManyItems(field) => write!(f, "too many entries in {field}"),
            Self::DuplicateItem(field) => write!(f, "duplicate entries in {field}"),
            Self::InconsistentEvidenceState(reason) => {
                write!(f, "inconsistent evidence state: {reason}")
            }
            Self::MissingInstrumentObservation(field) => {
                write!(f, "instrument-observed evidence requires {field}")
            }
            Self::InvalidCalibrationInterval => {
                write!(f, "calibration validity interval is incomplete or reversed")
            }
            Self::ObservationOutsideCalibrationInterval => write!(
                f,
                "observation time is outside the declared calibration interval"
            ),
            Self::InvalidReviewState(reason) => write!(f, "invalid review state: {reason}"),
        }
    }
}
impl std::error::Error for ReceiptValidationError {}

/// Decode exact JSON bytes into the strict receipt structure and validate it.
/// Struct decoding rejects unknown fields and duplicate struct fields. This
/// function does not canonicalize or hash the input bytes; callers that require
/// content identity must compute and retain a digest over their chosen exact bytes.
pub fn decode_and_validate_process_run_receipt_json(
    raw_json: &str,
) -> Result<ProcessRunReceipt, ReceiptValidationError> {
    let receipt: ProcessRunReceipt = serde_json::from_str(raw_json)
        .map_err(|_| ReceiptValidationError::InvalidSerializedReceipt)?;
    validate_process_run_receipt(&receipt)?;
    Ok(receipt)
}

/// Validate structure and declared evidence-state consistency only.
///
/// The function deliberately does not return a generic verified or approved
/// boolean. A receipt can be structurally valid while its source, instrument,
/// calibration, model, or claim remains unverified.
pub fn validate_process_run_receipt(
    receipt: &ProcessRunReceipt,
) -> Result<(), ReceiptValidationError> {
    if receipt.schema_version != PROCESS_RUN_RECEIPT_SCHEMA_VERSION {
        return Err(ReceiptValidationError::UnsupportedSchemaVersion(
            receipt.schema_version,
        ));
    }

    for (field, value, max) in [
        ("receipt_id", receipt.receipt_id.as_str(), MAX_ID_BYTES),
        ("run_id", receipt.run_id.as_str(), MAX_ID_BYTES),
        ("policy_id", receipt.policy_id.as_str(), MAX_ID_BYTES),
        ("producer_id", receipt.producer_id.as_str(), MAX_ID_BYTES),
        ("scenario_id", receipt.scenario_id.as_str(), MAX_ID_BYTES),
        ("tool_name", receipt.tool_name.as_str(), MAX_ID_BYTES),
        (
            "tool_version",
            receipt.tool_version.as_str(),
            MAX_VERSION_BYTES,
        ),
        ("model_domain", receipt.model_domain.as_str(), MAX_ID_BYTES),
    ] {
        if value.trim().is_empty() || value.len() > max {
            return Err(ReceiptValidationError::MissingOrOversizeField(field));
        }
    }

    if !is_sha256(&receipt.workload_sha256) {
        return Err(ReceiptValidationError::InvalidDigest("workload_sha256"));
    }
    if !is_sha256(&receipt.build_sha256) {
        return Err(ReceiptValidationError::InvalidDigest("build_sha256"));
    }
    if !is_sha256(&receipt.configuration_sha256) {
        return Err(ReceiptValidationError::InvalidDigest(
            "configuration_sha256",
        ));
    }
    if !is_sha256(&receipt.report_sha256) {
        return Err(ReceiptValidationError::InvalidDigest("report_sha256"));
    }
    if !is_git_commit(&receipt.source_commit_hex) {
        return Err(ReceiptValidationError::InvalidGitCommit);
    }

    if receipt.input_artifact_sha256.is_empty() {
        return Err(ReceiptValidationError::MissingOrOversizeField(
            "input_artifact_sha256",
        ));
    }
    if receipt.assumption_ids.is_empty() {
        return Err(ReceiptValidationError::MissingOrOversizeField(
            "assumption_ids",
        ));
    }
    validate_digest_list(
        "input_artifact_sha256",
        &receipt.input_artifact_sha256,
        MAX_INPUT_ARTIFACTS,
    )?;
    validate_string_list("assumption_ids", &receipt.assumption_ids, MAX_ASSUMPTIONS)?;
    validate_string_list(
        "observation_ids",
        &receipt.observation_ids,
        MAX_OBSERVATIONS,
    )?;
    validate_digest_list(
        "observation_artifact_sha256",
        &receipt.observation_artifact_sha256,
        MAX_OBSERVATIONS,
    )?;
    validate_optional_digest(
        "calibration_record_sha256",
        &receipt.calibration_record_sha256,
    )?;
    validate_optional_digest("parent_receipt_sha256", &receipt.parent_receipt_sha256)?;
    validate_optional_digest(
        "supersedes_receipt_sha256",
        &receipt.supersedes_receipt_sha256,
    )?;

    match receipt.evidence_origin {
        EvidenceOrigin::SyntheticFixture | EvidenceOrigin::SimulationOutput => {
            if receipt.measurement_system_id.is_some()
                || receipt.instrument_id.is_some()
                || !receipt.observation_ids.is_empty()
                || !receipt.observation_artifact_sha256.is_empty()
                || receipt.observed_at_unix_ms.is_some()
                || receipt.calibration_claim != CalibrationClaim::NotClaimed
                || receipt.calibration_record_sha256.is_some()
                || receipt.calibration_valid_from_unix_ms.is_some()
                || receipt.calibration_valid_until_unix_ms.is_some()
            {
                return Err(ReceiptValidationError::InconsistentEvidenceState(
                    "synthetic/simulated result cannot claim instrument or calibration evidence",
                ));
            }
        }
        EvidenceOrigin::InstrumentObserved => {
            validate_required_option_string(
                "measurement_system_id",
                &receipt.measurement_system_id,
                MAX_ID_BYTES,
            )?;
            validate_required_option_string("instrument_id", &receipt.instrument_id, MAX_ID_BYTES)?;
            if receipt.observed_at_unix_ms.is_none() {
                return Err(ReceiptValidationError::MissingInstrumentObservation(
                    "observed_at_unix_ms",
                ));
            }
            if receipt.observation_ids.is_empty() {
                return Err(ReceiptValidationError::MissingInstrumentObservation(
                    "observation_ids",
                ));
            }
            if receipt.observation_ids.len() != receipt.observation_artifact_sha256.len() {
                return Err(ReceiptValidationError::InconsistentEvidenceState(
                    "each observation ID must have exactly one raw artifact digest",
                ));
            }
        }
    }

    match receipt.calibration_claim {
        CalibrationClaim::NotClaimed => {
            if receipt.calibration_record_sha256.is_some()
                || receipt.calibration_valid_from_unix_ms.is_some()
                || receipt.calibration_valid_until_unix_ms.is_some()
            {
                return Err(ReceiptValidationError::InconsistentEvidenceState(
                    "calibration details supplied without a calibration claim",
                ));
            }
        }
        CalibrationClaim::ClaimedCurrentAtObservation => {
            if receipt.evidence_origin != EvidenceOrigin::InstrumentObserved {
                return Err(ReceiptValidationError::InconsistentEvidenceState(
                    "calibration claim requires instrument-observed evidence",
                ));
            }
            if receipt.calibration_record_sha256.is_none()
                || receipt.calibration_valid_from_unix_ms.is_none()
                || receipt.calibration_valid_until_unix_ms.is_none()
            {
                return Err(ReceiptValidationError::InvalidCalibrationInterval);
            }
            let from = receipt.calibration_valid_from_unix_ms.unwrap();
            let until = receipt.calibration_valid_until_unix_ms.unwrap();
            let observed = receipt.observed_at_unix_ms.unwrap();
            if from > until {
                return Err(ReceiptValidationError::InvalidCalibrationInterval);
            }
            if observed < from || observed > until {
                return Err(ReceiptValidationError::ObservationOutsideCalibrationInterval);
            }
        }
    }

    validate_review_state(receipt)?;
    Ok(())
}

fn validate_review_state(receipt: &ProcessRunReceipt) -> Result<(), ReceiptValidationError> {
    match receipt.review_disposition {
        ReviewDisposition::NotReviewed => {
            if receipt.reviewer_id.is_some()
                || receipt.reviewer_version.is_some()
                || receipt.reviewed_report_sha256.is_some()
                || receipt.reviewed_at_unix_ms.is_some()
            {
                return Err(ReceiptValidationError::InvalidReviewState(
                    "review fields must be absent when disposition is not_reviewed",
                ));
            }
        }
        ReviewDisposition::Pass | ReviewDisposition::Fail | ReviewDisposition::Indeterminate => {
            let reviewer = receipt.reviewer_id.as_deref().unwrap_or_default();
            let version = receipt.reviewer_version.as_deref().unwrap_or_default();
            if reviewer.trim().is_empty()
                || reviewer.len() > MAX_ID_BYTES
                || version.trim().is_empty()
                || version.len() > MAX_VERSION_BYTES
            {
                return Err(ReceiptValidationError::InvalidReviewState(
                    "reviewer identity and version must be present and bounded",
                ));
            }
            if reviewer == receipt.producer_id {
                return Err(ReceiptValidationError::InvalidReviewState(
                    "independent reviewer must differ from the receipt producer",
                ));
            }
            if receipt.reviewed_report_sha256.as_deref() != Some(receipt.report_sha256.as_str()) {
                return Err(ReceiptValidationError::InvalidReviewState(
                    "review must bind to the exact report digest",
                ));
            }
            if receipt.reviewed_at_unix_ms.is_none() {
                return Err(ReceiptValidationError::InvalidReviewState(
                    "review timestamp is required",
                ));
            }
        }
    }
    Ok(())
}

fn validate_required_option_string(
    field: &'static str,
    value: &Option<String>,
    max_bytes: usize,
) -> Result<(), ReceiptValidationError> {
    match value {
        Some(v) if !v.trim().is_empty() && v.len() <= max_bytes => Ok(()),
        _ => Err(ReceiptValidationError::MissingInstrumentObservation(field)),
    }
}

fn is_sha256(value: &str) -> bool {
    value.len() == 64
        && value
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}
fn is_git_commit(value: &str) -> bool {
    (value.len() == 40 || value.len() == 64)
        && value
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}
fn validate_optional_digest(
    field: &'static str,
    value: &Option<String>,
) -> Result<(), ReceiptValidationError> {
    if value.as_deref().is_some_and(|digest| !is_sha256(digest)) {
        return Err(ReceiptValidationError::InvalidDigest(field));
    }
    Ok(())
}
fn validate_digest_list(
    field: &'static str,
    values: &[String],
    max: usize,
) -> Result<(), ReceiptValidationError> {
    if values.len() > max {
        return Err(ReceiptValidationError::TooManyItems(field));
    }
    let mut seen = BTreeSet::new();
    for value in values {
        if !is_sha256(value) {
            return Err(ReceiptValidationError::InvalidDigest(field));
        }
        if !seen.insert(value) {
            return Err(ReceiptValidationError::DuplicateItem(field));
        }
    }
    Ok(())
}
fn validate_string_list(
    field: &'static str,
    values: &[String],
    max: usize,
) -> Result<(), ReceiptValidationError> {
    if values.len() > max {
        return Err(ReceiptValidationError::TooManyItems(field));
    }
    let mut seen = BTreeSet::new();
    for value in values {
        if value.trim().is_empty() || value.len() > MAX_ID_BYTES {
            return Err(ReceiptValidationError::MissingOrOversizeField(field));
        }
        if !seen.insert(value) {
            return Err(ReceiptValidationError::DuplicateItem(field));
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn digest(ch: char) -> String {
        std::iter::repeat_n(ch, 64).collect()
    }

    fn valid_simulation() -> ProcessRunReceipt {
        ProcessRunReceipt {
            schema_version: PROCESS_RUN_RECEIPT_SCHEMA_VERSION,
            receipt_id: "receipt-001".into(),
            run_id: "run-001".into(),
            policy_id: "condition-based-v1".into(),
            producer_id: "agent-producer".into(),
            scenario_id: "gradual-wear-seed-7".into(),
            seed: 7,
            created_at_unix_ms: 1_800_000_000_000,
            workload_sha256: digest('a'),
            source_commit_hex: "b".repeat(40),
            tool_name: "quality-drift-sim".into(),
            tool_version: "0.1.0".into(),
            build_sha256: digest('c'),
            configuration_sha256: digest('d'),
            input_artifact_sha256: vec![digest('e')],
            report_sha256: digest('f'),
            evidence_origin: EvidenceOrigin::SimulationOutput,
            model_domain: "synthetic-tool-wear".into(),
            assumption_ids: vec!["wear-model-v1".into()],
            out_of_domain: false,
            measurement_system_id: None,
            instrument_id: None,
            observation_ids: vec![],
            observation_artifact_sha256: vec![],
            observed_at_unix_ms: None,
            calibration_claim: CalibrationClaim::NotClaimed,
            calibration_record_sha256: None,
            calibration_valid_from_unix_ms: None,
            calibration_valid_until_unix_ms: None,
            review_disposition: ReviewDisposition::NotReviewed,
            reviewer_id: None,
            reviewer_version: None,
            reviewed_report_sha256: None,
            reviewed_at_unix_ms: None,
            parent_receipt_sha256: None,
            supersedes_receipt_sha256: None,
        }
    }

    fn valid_instrument_observation() -> ProcessRunReceipt {
        let mut r = valid_simulation();
        r.evidence_origin = EvidenceOrigin::InstrumentObserved;
        r.measurement_system_id = Some("ms-01".into());
        r.instrument_id = Some("instrument-01".into());
        r.observation_ids = vec!["obs-001".into()];
        r.observation_artifact_sha256 = vec![digest('9')];
        r.observed_at_unix_ms = Some(1_800_000_000_500);
        r
    }

    #[test]
    fn required_inputs_and_assumptions_cannot_be_omitted() {
        let mut r = valid_simulation();
        r.input_artifact_sha256.clear();
        assert_eq!(
            validate_process_run_receipt(&r),
            Err(ReceiptValidationError::MissingOrOversizeField(
                "input_artifact_sha256"
            ))
        );

        let mut r = valid_simulation();
        r.assumption_ids.clear();
        assert_eq!(
            validate_process_run_receipt(&r),
            Err(ReceiptValidationError::MissingOrOversizeField(
                "assumption_ids"
            ))
        );
    }

    #[test]
    fn instrument_identity_fields_are_bounded() {
        let mut r = valid_instrument_observation();
        r.instrument_id = Some("x".repeat(MAX_ID_BYTES + 1));
        assert!(matches!(
            validate_process_run_receipt(&r),
            Err(ReceiptValidationError::MissingInstrumentObservation(
                "instrument_id"
            ))
        ));
    }

    #[test]
    fn valid_simulation_is_structurally_accepted_without_physical_claims() {
        assert_eq!(validate_process_run_receipt(&valid_simulation()), Ok(()));
    }

    #[test]
    fn simulation_cannot_claim_instrument_or_calibration_evidence() {
        let mut r = valid_simulation();
        r.instrument_id = Some("instrument-01".into());
        assert!(matches!(
            validate_process_run_receipt(&r),
            Err(ReceiptValidationError::InconsistentEvidenceState(_))
        ));

        let mut r = valid_simulation();
        r.calibration_claim = CalibrationClaim::ClaimedCurrentAtObservation;
        assert!(validate_process_run_receipt(&r).is_err());
    }

    #[test]
    fn instrument_observation_requires_raw_observation_binding() {
        let mut r = valid_instrument_observation();
        r.observation_artifact_sha256.clear();
        assert!(matches!(
            validate_process_run_receipt(&r),
            Err(ReceiptValidationError::InconsistentEvidenceState(_))
        ));

        let mut r = valid_instrument_observation();
        r.observation_ids.clear();
        assert!(matches!(
            validate_process_run_receipt(&r),
            Err(ReceiptValidationError::MissingInstrumentObservation(_))
        ));
    }

    #[test]
    fn calibration_claim_requires_digest_and_observation_inside_validity() {
        let mut r = valid_instrument_observation();
        r.calibration_claim = CalibrationClaim::ClaimedCurrentAtObservation;
        r.calibration_record_sha256 = Some(digest('8'));
        r.calibration_valid_from_unix_ms = Some(1_800_000_000_000);
        r.calibration_valid_until_unix_ms = Some(1_800_000_001_000);
        assert_eq!(validate_process_run_receipt(&r), Ok(()));

        r.calibration_valid_until_unix_ms = Some(1_800_000_000_200);
        assert_eq!(
            validate_process_run_receipt(&r),
            Err(ReceiptValidationError::ObservationOutsideCalibrationInterval)
        );
    }

    #[test]
    fn calibration_claim_requires_ordered_complete_interval() {
        let mut r = valid_instrument_observation();
        r.calibration_claim = CalibrationClaim::ClaimedCurrentAtObservation;
        r.calibration_record_sha256 = Some(digest('8'));
        r.calibration_valid_from_unix_ms = Some(1_800_000_000_700);
        r.calibration_valid_until_unix_ms = Some(1_800_000_000_100);
        assert_eq!(
            validate_process_run_receipt(&r),
            Err(ReceiptValidationError::InvalidCalibrationInterval)
        );
    }

    #[test]
    fn independent_review_binds_exact_report_and_separate_actor() {
        let mut r = valid_simulation();
        r.review_disposition = ReviewDisposition::Pass;
        r.reviewer_id = Some("reviewer-agent".into());
        r.reviewer_version = Some("verifier-2".into());
        r.reviewed_report_sha256 = Some(r.report_sha256.clone());
        r.reviewed_at_unix_ms = Some(1_800_000_001_000);
        assert_eq!(validate_process_run_receipt(&r), Ok(()));

        r.reviewed_report_sha256 = Some(digest('0'));
        assert!(matches!(
            validate_process_run_receipt(&r),
            Err(ReceiptValidationError::InvalidReviewState(_))
        ));

        r.reviewed_report_sha256 = Some(r.report_sha256.clone());
        r.reviewer_id = Some(r.producer_id.clone());
        assert!(matches!(
            validate_process_run_receipt(&r),
            Err(ReceiptValidationError::InvalidReviewState(_))
        ));
    }

    #[test]
    fn report_review_does_not_clear_out_of_domain_flag() {
        let mut r = valid_simulation();
        r.out_of_domain = true;
        r.review_disposition = ReviewDisposition::Pass;
        r.reviewer_id = Some("reviewer-agent".into());
        r.reviewer_version = Some("verifier-2".into());
        r.reviewed_report_sha256 = Some(r.report_sha256.clone());
        r.reviewed_at_unix_ms = Some(1_800_000_001_000);
        assert_eq!(validate_process_run_receipt(&r), Ok(()));
        assert!(
            r.out_of_domain,
            "independent review must not erase OOD status"
        );
    }

    #[test]
    fn rejects_bad_digests_and_commit_identities() {
        let mut r = valid_simulation();
        r.report_sha256 = "not-a-digest".into();
        assert_eq!(
            validate_process_run_receipt(&r),
            Err(ReceiptValidationError::InvalidDigest("report_sha256"))
        );

        let mut r = valid_simulation();
        r.source_commit_hex = "not-a-commit".into();
        assert_eq!(
            validate_process_run_receipt(&r),
            Err(ReceiptValidationError::InvalidGitCommit)
        );
    }

    #[test]
    fn rejects_duplicate_digest_and_assumption_entries() {
        let mut r = valid_simulation();
        r.input_artifact_sha256
            .push(r.input_artifact_sha256[0].clone());
        assert_eq!(
            validate_process_run_receipt(&r),
            Err(ReceiptValidationError::DuplicateItem(
                "input_artifact_sha256"
            ))
        );

        let mut r = valid_simulation();
        r.assumption_ids.push(r.assumption_ids[0].clone());
        assert_eq!(
            validate_process_run_receipt(&r),
            Err(ReceiptValidationError::DuplicateItem("assumption_ids"))
        );
    }

    #[test]
    fn rejects_incomplete_or_unsolicited_review_fields() {
        let mut r = valid_simulation();
        r.review_disposition = ReviewDisposition::NotReviewed;
        r.reviewer_id = Some("someone".into());
        assert!(matches!(
            validate_process_run_receipt(&r),
            Err(ReceiptValidationError::InvalidReviewState(_))
        ));

        let mut r = valid_simulation();
        r.review_disposition = ReviewDisposition::Indeterminate;
        r.reviewer_id = Some("reviewer".into());
        r.reviewer_version = Some("v1".into());
        r.reviewed_report_sha256 = Some(r.report_sha256.clone());
        r.reviewed_at_unix_ms = None;
        assert!(matches!(
            validate_process_run_receipt(&r),
            Err(ReceiptValidationError::InvalidReviewState(_))
        ));
    }

    #[test]
    fn rejects_unknown_json_fields_and_unsupported_versions() {
        let receipt = valid_simulation();
        let mut value = serde_json::to_value(&receipt).unwrap();
        value
            .as_object_mut()
            .unwrap()
            .insert("unexpected".into(), serde_json::Value::Bool(true));
        assert!(serde_json::from_value::<ProcessRunReceipt>(value).is_err());

        let mut r = valid_simulation();
        r.schema_version = 2;
        assert_eq!(
            validate_process_run_receipt(&r),
            Err(ReceiptValidationError::UnsupportedSchemaVersion(2))
        );
    }

    #[test]
    fn raw_json_decoder_rejects_duplicate_keys_and_unknown_fields() {
        let receipt = valid_simulation();
        let json = serde_json::to_string(&receipt).unwrap();
        assert_eq!(
            decode_and_validate_process_run_receipt_json(&json).unwrap(),
            receipt
        );

        let duplicate = json.replacen(
            "\"schema_version\":1,",
            "\"schema_version\":1,\"schema_version\":1,",
            1,
        );
        assert_ne!(duplicate, json);
        assert_eq!(
            decode_and_validate_process_run_receipt_json(&duplicate),
            Err(ReceiptValidationError::InvalidSerializedReceipt)
        );

        let unknown = json.replacen('}', ",\"unexpected\":true}", 1);
        assert_eq!(
            decode_and_validate_process_run_receipt_json(&unknown),
            Err(ReceiptValidationError::InvalidSerializedReceipt)
        );
    }

    #[test]
    fn serialized_receipt_round_trips_deterministically() {
        let receipt = valid_simulation();
        let json1 = serde_json::to_string(&receipt).unwrap();
        let decoded = decode_and_validate_process_run_receipt_json(&json1).unwrap();
        let json2 = serde_json::to_string(&decoded).unwrap();
        assert_eq!(json1, json2);
        assert_eq!(validate_process_run_receipt(&decoded), Ok(()));
    }
}
