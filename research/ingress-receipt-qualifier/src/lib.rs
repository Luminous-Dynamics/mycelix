#![forbid(unsafe_code)]

//! Synthetic reference evaluator for qualified ingress-receipt time intervals.
//!
//! This crate does not verify signatures, clock hardware, transparency proofs,
//! source rights, or physical truth. It consumes typed verification receipts
//! from their owning components and applies deterministic cutoff semantics.

use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};

pub const CORPUS_JSON: &str = include_str!("../tests/fixtures/ingress-receipt-cases-v0.1.json");

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TimeInterval {
    /// Earliest plausible time in the declared timebase/epoch, nanoseconds.
    pub earliest_ns: u64,
    /// Latest plausible time in the declared timebase/epoch, nanoseconds.
    pub latest_ns: u64,
}

impl TimeInterval {
    pub fn width_ns(&self) -> Option<u64> {
        self.latest_ns.checked_sub(self.earliest_ns)
    }

    pub fn is_ordered(&self) -> bool {
        self.earliest_ns <= self.latest_ns
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum VerificationDisposition {
    Verified,
    Failed,
    Indeterminate,
    Unsupported,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct VerificationReceiptRef {
    pub receipt_id: String,
    pub digest_sha256: String,
    pub verifier_id: String,
    pub verifier_version: String,
    pub profile_id: String,
    /// Exact statement/checkpoint digest which this verifier receipt assessed.
    pub subject_sha256: String,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct QualificationEvidence {
    pub disposition: VerificationDisposition,
    pub verifier_receipt: Option<VerificationReceiptRef>,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ClockMappingReceipt {
    pub mapping_id: String,
    pub mapping_sha256: String,
    pub source_timebase_id: String,
    pub source_clock_epoch_id: String,
    pub target_timebase_id: String,
    pub target_clock_epoch_id: String,
    /// The mapping is valid over this source-native clock interval.
    pub source_validity: TimeInterval,
    /// Output interval after the owner-qualified mapping is applied.
    pub normalized_interval: TimeInterval,
    pub qualification: QualificationEvidence,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct StreamSequenceRef {
    pub stream_id: String,
    pub clock_epoch_id: String,
    pub sequence_number: u64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum HistoryStatus {
    Consistent,
    InclusionOnly,
    ForkObserved,
    Indeterminate,
    Unavailable,
    NotRequired,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct HistoryEvidence {
    pub status: HistoryStatus,
    pub log_id: Option<String>,
    pub checkpoint_size: Option<u64>,
    pub checkpoint_root_sha256: Option<String>,
    pub inclusion_receipt: Option<VerificationReceiptRef>,
    pub consistency_receipt: Option<VerificationReceiptRef>,
    pub witness_profile_id: Option<String>,
    /// Witness references are provenance, not an independence count.
    pub witnesses: Vec<VerificationReceiptRef>,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct IngressReceiptV1 {
    pub receipt_id: String,
    /// Commitment to the complete canonical ingress-receipt statement.
    pub receipt_statement_sha256: String,
    pub receiver_id: String,
    pub source_artifact_sha256: String,
    pub source_artifact_byte_length: u64,
    pub source_stream_id: Option<String>,
    pub source_clock_domain_id: String,
    pub source_clock_epoch_id: String,
    pub source_timebase_id: String,
    /// Original source-clock interval before any declared mapping.
    pub source_receipt_interval: TimeInterval,
    /// Interval normalized by the owner-qualified mapping, if any.
    pub comparison_timebase_id: String,
    pub comparison_clock_epoch_id: String,
    pub comparison_receipt_interval: TimeInterval,
    pub mapping_receipt: Option<ClockMappingReceipt>,
    pub sequence: Option<StreamSequenceRef>,
    pub time_qualification: QualificationEvidence,
    pub history: HistoryEvidence,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AsOfCutoffV1 {
    pub cutoff_id: String,
    pub cutoff_statement_sha256: String,
    pub timebase_id: String,
    pub clock_epoch_id: String,
    /// The cutoff is itself an interval, not an assumed zero-uncertainty point.
    pub interval: TimeInterval,
    pub qualification: QualificationEvidence,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct QualificationPolicyV1 {
    pub policy_id: String,
    pub required_time_profile_id: String,
    pub required_mapping_profile_id: String,
    pub required_history_profile_id: String,
    pub maximum_receipt_interval_width_ns: u64,
    pub maximum_cutoff_interval_width_ns: u64,
    pub require_consistent_history: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CutoffQualification {
    Verified,
    Failed,
    Indeterminate,
    Unsupported,
    Invalid,
    UncertaintyTooWide,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum HistoryAssessment {
    ConsistentForProfile,
    NotRequired,
    Unproven,
    Forked,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ReceiptDisposition {
    EligibleByCutoff,
    DefinitelyAfterCutoff,
    CutoffIndeterminate,
    ReceiptInvalid,
    CutoffUnqualified,
    TimeQualificationFailed,
    TimeQualificationIndeterminate,
    TimeQualificationUnsupported,
    TimebaseMismatch,
    ClockMappingMissing,
    ClockMappingUnqualified,
    UncertaintyTooWide,
    HistoryUnproven,
    HistoryForked,
    ReceiptIdentityConflict,
    SequenceConflict,
}

impl ReceiptDisposition {
    pub const fn id(self) -> &'static str {
        match self {
            Self::EligibleByCutoff => "eligible_by_cutoff",
            Self::DefinitelyAfterCutoff => "definitely_after_cutoff",
            Self::CutoffIndeterminate => "cutoff_indeterminate",
            Self::ReceiptInvalid => "receipt_invalid",
            Self::CutoffUnqualified => "cutoff_unqualified",
            Self::TimeQualificationFailed => "time_qualification_failed",
            Self::TimeQualificationIndeterminate => "time_qualification_indeterminate",
            Self::TimeQualificationUnsupported => "time_qualification_unsupported",
            Self::TimebaseMismatch => "timebase_mismatch",
            Self::ClockMappingMissing => "clock_mapping_missing",
            Self::ClockMappingUnqualified => "clock_mapping_unqualified",
            Self::UncertaintyTooWide => "uncertainty_too_wide",
            Self::HistoryUnproven => "history_unproven",
            Self::HistoryForked => "history_forked",
            Self::ReceiptIdentityConflict => "receipt_identity_conflict",
            Self::SequenceConflict => "sequence_conflict",
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ReceiptAssessment {
    pub receipt_id: String,
    pub source_artifact_sha256: String,
    pub receiver_id: String,
    pub disposition: ReceiptDisposition,
    pub history_assessment: HistoryAssessment,
    pub reason_codes: Vec<String>,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct QualificationReport {
    pub cutoff_id: String,
    pub policy_id: String,
    pub cutoff_qualification: CutoffQualification,
    pub assessments: Vec<ReceiptAssessment>,
    pub eligible_receipt_ids: Vec<String>,
    pub excluded_receipt_ids: Vec<String>,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ExpectedAssessment {
    pub receipt_id: String,
    pub disposition: ReceiptDisposition,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct QualificationCase {
    pub case_id: String,
    pub cutoff: AsOfCutoffV1,
    pub policy: QualificationPolicyV1,
    pub receipts: Vec<IngressReceiptV1>,
    pub expected: Vec<ExpectedAssessment>,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct QualificationCorpus {
    pub corpus_id: String,
    pub cases: Vec<QualificationCase>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum CorpusError {
    InvalidJson(String),
    EmptyCorpus,
    EmptyCaseId,
    ExpectedMismatch {
        case_id: String,
        receipt_id: String,
        expected: ReceiptDisposition,
        actual: Option<ReceiptDisposition>,
    },
    UnexpectedReceipt {
        case_id: String,
        receipt_id: String,
    },
}

fn nonblank(value: &str) -> bool {
    !value.trim().is_empty()
}

fn sha256_hex(value: &str) -> bool {
    value.len() == 64 && value.bytes().all(|b| b.is_ascii_hexdigit())
}

fn valid_ref(
    reference: &VerificationReceiptRef,
    expected_subject: &str,
    expected_profile: &str,
) -> bool {
    nonblank(&reference.receipt_id)
        && sha256_hex(&reference.digest_sha256)
        && nonblank(&reference.verifier_id)
        && nonblank(&reference.verifier_version)
        && reference.profile_id == expected_profile
        && reference.subject_sha256.eq_ignore_ascii_case(expected_subject)
}

fn qualification_failure(
    evidence: &QualificationEvidence,
    expected_subject: &str,
    expected_profile: &str,
) -> Option<ReceiptDisposition> {
    match evidence.disposition {
        VerificationDisposition::Failed => Some(ReceiptDisposition::TimeQualificationFailed),
        VerificationDisposition::Indeterminate => {
            Some(ReceiptDisposition::TimeQualificationIndeterminate)
        }
        VerificationDisposition::Unsupported => {
            Some(ReceiptDisposition::TimeQualificationUnsupported)
        }
        VerificationDisposition::Verified => {
            let Some(reference) = evidence.verifier_receipt.as_ref() else {
                return Some(ReceiptDisposition::TimeQualificationIndeterminate);
            };
            if valid_ref(reference, expected_subject, expected_profile) {
                None
            } else {
                Some(ReceiptDisposition::TimeQualificationIndeterminate)
            }
        }
    }
}

fn cutoff_qualification(
    cutoff: &AsOfCutoffV1,
    policy: &QualificationPolicyV1,
) -> CutoffQualification {
    if !nonblank(&policy.policy_id)
        || !nonblank(&policy.required_time_profile_id)
        || !nonblank(&policy.required_mapping_profile_id)
        || !nonblank(&policy.required_history_profile_id)
        || !nonblank(&cutoff.cutoff_id)
        || !sha256_hex(&cutoff.cutoff_statement_sha256)
        || !nonblank(&cutoff.timebase_id)
        || !nonblank(&cutoff.clock_epoch_id)
        || !cutoff.interval.is_ordered()
    {
        return CutoffQualification::Invalid;
    }
    if cutoff.interval.width_ns().is_none_or(|w| w > policy.maximum_cutoff_interval_width_ns) {
        return CutoffQualification::UncertaintyTooWide;
    }
    match cutoff.qualification.disposition {
        VerificationDisposition::Failed => CutoffQualification::Failed,
        VerificationDisposition::Indeterminate => CutoffQualification::Indeterminate,
        VerificationDisposition::Unsupported => CutoffQualification::Unsupported,
        VerificationDisposition::Verified => {
            let Some(reference) = cutoff.qualification.verifier_receipt.as_ref() else {
                return CutoffQualification::Indeterminate;
            };
            if valid_ref(
                reference,
                &cutoff.cutoff_statement_sha256,
                &policy.required_time_profile_id,
            ) {
                CutoffQualification::Verified
            } else {
                CutoffQualification::Indeterminate
            }
        }
    }
}

fn map_cutoff_failure(value: CutoffQualification) -> ReceiptDisposition {
    match value {
        CutoffQualification::Verified => ReceiptDisposition::EligibleByCutoff,
        CutoffQualification::Failed => ReceiptDisposition::TimeQualificationFailed,
        CutoffQualification::Indeterminate => ReceiptDisposition::TimeQualificationIndeterminate,
        CutoffQualification::Unsupported => ReceiptDisposition::TimeQualificationUnsupported,
        CutoffQualification::Invalid => ReceiptDisposition::ReceiptInvalid,
        CutoffQualification::UncertaintyTooWide => ReceiptDisposition::UncertaintyTooWide,
    }
}

fn history_assessment(
    history: &HistoryEvidence,
    receipt_statement_sha256: &str,
    policy: &QualificationPolicyV1,
) -> HistoryAssessment {
    match history.status {
        HistoryStatus::ForkObserved => HistoryAssessment::Forked,
        HistoryStatus::NotRequired => {
            if policy.require_consistent_history {
                HistoryAssessment::Unproven
            } else {
                HistoryAssessment::NotRequired
            }
        }
        HistoryStatus::Consistent => {
            let (
                Some(log_id),
                Some(checkpoint_size),
                Some(root),
                Some(inclusion),
                Some(consistency),
                Some(witness_profile),
            ) = (
                history.log_id.as_deref(),
                history.checkpoint_size,
                history.checkpoint_root_sha256.as_deref(),
                history.inclusion_receipt.as_ref(),
                history.consistency_receipt.as_ref(),
                history.witness_profile_id.as_deref(),
            )
            else {
                return HistoryAssessment::Unproven;
            };
            if !nonblank(log_id)
                || checkpoint_size == 0
                || !sha256_hex(root)
                || !nonblank(witness_profile)
                || history.witnesses.is_empty()
                || !valid_ref(inclusion, receipt_statement_sha256, policy.required_history_profile_id.as_str())
                || !valid_ref(consistency, root, policy.required_history_profile_id.as_str())
                || history.witnesses.iter().any(|w| {
                    !valid_ref(w, root, policy.required_history_profile_id.as_str())
                })
            {
                return HistoryAssessment::Unproven;
            }
            HistoryAssessment::ConsistentForProfile
        }
        HistoryStatus::InclusionOnly | HistoryStatus::Indeterminate | HistoryStatus::Unavailable => {
            HistoryAssessment::Unproven
        }
    }
}

fn receipt_invalid(record: &IngressReceiptV1) -> bool {
    !nonblank(&record.receipt_id)
        || !sha256_hex(&record.receipt_statement_sha256)
        || !nonblank(&record.receiver_id)
        || !sha256_hex(&record.source_artifact_sha256)
        || record.source_artifact_byte_length == 0
        || !nonblank(&record.source_clock_domain_id)
        || !nonblank(&record.source_clock_epoch_id)
        || !nonblank(&record.source_timebase_id)
        || !record.source_receipt_interval.is_ordered()
        || !nonblank(&record.comparison_timebase_id)
        || !nonblank(&record.comparison_clock_epoch_id)
        || !record.comparison_receipt_interval.is_ordered()
        || record.sequence.as_ref().is_some_and(|s| {
            !nonblank(&s.stream_id) || !nonblank(&s.clock_epoch_id)
        })
        || record.sequence.as_ref().is_some_and(|s| {
            s.clock_epoch_id != record.source_clock_epoch_id
                || record.source_stream_id.as_deref() != Some(s.stream_id.as_str())
        })
        || record.source_stream_id.as_ref().is_some_and(|s| !nonblank(s))
}

fn mapping_is_qualified(
    record: &IngressReceiptV1,
    policy: &QualificationPolicyV1,
) -> Result<(), ReceiptDisposition> {
    let same_basis = record.source_timebase_id == record.comparison_timebase_id
        && record.source_clock_epoch_id == record.comparison_clock_epoch_id;
    match (&record.mapping_receipt, same_basis) {
        (None, true) => {
            if record.source_receipt_interval == record.comparison_receipt_interval {
                Ok(())
            } else {
                Err(ReceiptDisposition::ClockMappingMissing)
            }
        }
        (None, false) => Err(ReceiptDisposition::ClockMappingMissing),
        (Some(mapping), _) => {
            if !nonblank(&mapping.mapping_id)
                || !sha256_hex(&mapping.mapping_sha256)
                || mapping.source_timebase_id != record.source_timebase_id
                || mapping.source_clock_epoch_id != record.source_clock_epoch_id
                || mapping.target_timebase_id != record.comparison_timebase_id
                || mapping.target_clock_epoch_id != record.comparison_clock_epoch_id
                || !mapping.source_validity.is_ordered()
                || !mapping.normalized_interval.is_ordered()
                || mapping.source_validity.earliest_ns > record.source_receipt_interval.earliest_ns
                || mapping.source_validity.latest_ns < record.source_receipt_interval.latest_ns
                || mapping.normalized_interval != record.comparison_receipt_interval
            {
                return Err(ReceiptDisposition::ClockMappingUnqualified);
            }
            if mapping.qualification.disposition != VerificationDisposition::Verified {
                return Err(ReceiptDisposition::ClockMappingUnqualified);
            }
            let Some(reference) = mapping.qualification.verifier_receipt.as_ref() else {
                return Err(ReceiptDisposition::ClockMappingUnqualified);
            };
            if !valid_ref(
                reference,
                &mapping.mapping_sha256,
                &policy.required_mapping_profile_id,
            ) {
                return Err(ReceiptDisposition::ClockMappingUnqualified);
            }
            Ok(())
        }
    }
}

fn assess_one(
    record: &IngressReceiptV1,
    cutoff: &AsOfCutoffV1,
    policy: &QualificationPolicyV1,
    cutoff_status: CutoffQualification,
) -> ReceiptAssessment {
    let mut reasons = Vec::new();
    let mut history = history_assessment(&record.history, &record.receipt_statement_sha256, policy);

    let mut disposition = if receipt_invalid(record) {
        reasons.push("receipt_identity_digest_or_interval_invalid".to_owned());
        ReceiptDisposition::ReceiptInvalid
    } else if cutoff_status != CutoffQualification::Verified {
        reasons.push("decision_cutoff_not_qualified".to_owned());
        map_cutoff_failure(cutoff_status)
    } else if let Some(failure) = qualification_failure(
        &record.time_qualification,
        &record.receipt_statement_sha256,
        &policy.required_time_profile_id,
    ) {
        reasons.push("receipt_time_verification_not_qualified".to_owned());
        failure
    } else if let Err(failure) = mapping_is_qualified(record, policy) {
        reasons.push("clock_mapping_not_qualified".to_owned());
        failure
    } else if record.comparison_timebase_id != cutoff.timebase_id
        || record.comparison_clock_epoch_id != cutoff.clock_epoch_id
    {
        reasons.push("receipt_and_cutoff_not_in_same_qualified_timebase_epoch".to_owned());
        ReceiptDisposition::TimebaseMismatch
    } else if record.comparison_receipt_interval.width_ns().is_none_or(|w| {
        w > policy.maximum_receipt_interval_width_ns
    }) {
        reasons.push("receipt_interval_exceeds_policy_uncertainty_width".to_owned());
        ReceiptDisposition::UncertaintyTooWide
    } else if record.comparison_receipt_interval.latest_ns <= cutoff.interval.earliest_ns {
        ReceiptDisposition::EligibleByCutoff
    } else if record.comparison_receipt_interval.earliest_ns > cutoff.interval.latest_ns {
        ReceiptDisposition::DefinitelyAfterCutoff
    } else {
        reasons.push("receipt_interval_overlaps_decision_cutoff_interval".to_owned());
        ReceiptDisposition::CutoffIndeterminate
    };

    if disposition == ReceiptDisposition::EligibleByCutoff {
        match history {
            HistoryAssessment::Forked => {
                reasons.push("witnessed_history_fork_observed".to_owned());
                disposition = ReceiptDisposition::HistoryForked;
            }
            HistoryAssessment::Unproven if policy.require_consistent_history => {
                reasons.push("consistent_witnessed_history_required_but_unproven".to_owned());
                disposition = ReceiptDisposition::HistoryUnproven;
            }
            _ => {}
        }
    }

    if history == HistoryAssessment::NotRequired && policy.require_consistent_history {
        history = HistoryAssessment::Unproven;
    }

    ReceiptAssessment {
        receipt_id: record.receipt_id.clone(),
        source_artifact_sha256: record.source_artifact_sha256.clone(),
        receiver_id: record.receiver_id.clone(),
        disposition,
        history_assessment: history,
        reason_codes: reasons,
    }
}

fn sequence_key(record: &IngressReceiptV1) -> Option<(String, String, u64)> {
    record.sequence.as_ref().map(|sequence| {
        (
            sequence.stream_id.clone(),
            sequence.clock_epoch_id.clone(),
            sequence.sequence_number,
        )
    })
}

/// Qualify a set of ingress receipts against an uncertain as-of cutoff.
///
/// Important: this is a reference evaluator over owner-issued verification
/// references. It validates profile/subject binding and fail-closed policy
/// composition, but does not cryptographically verify signatures, clock
/// hardware, inclusion proofs, consistency proofs, or witness independence.
pub fn qualify_receipts(
    records: &[IngressReceiptV1],
    cutoff: &AsOfCutoffV1,
    policy: &QualificationPolicyV1,
) -> QualificationReport {
    let cutoff_status = cutoff_qualification(cutoff, policy);
    let mut by_id: BTreeMap<String, Vec<&IngressReceiptV1>> = BTreeMap::new();
    for record in records {
        by_id.entry(record.receipt_id.clone()).or_default().push(record);
    }

    let mut assessments = Vec::new();
    let mut sequence_groups: BTreeMap<(String, String, u64), BTreeSet<String>> = BTreeMap::new();
    for record in records {
        if let Some(key) = sequence_key(record) {
            sequence_groups
                .entry(key)
                .or_default()
                .insert(record.source_artifact_sha256.clone());
        }
    }
    let conflicting_sequences: BTreeSet<(String, String, u64)> = sequence_groups
        .into_iter()
        .filter_map(|(key, digests)| (digests.len() > 1).then_some(key))
        .collect();

    for (receipt_id, group) in by_id {
        // Select a canonical representative so conflicting duplicate receipt IDs
        // cannot make the output depend on input array order.
        let first = group
            .iter()
            .min_by_key(|record| serde_json::to_string(*record).unwrap_or_default())
            .copied()
            .expect("receipt ID group is non-empty");
        let mut assessment = assess_one(first, cutoff, policy, cutoff_status);
        if group.iter().any(|other| *other != first) {
            assessment.disposition = ReceiptDisposition::ReceiptIdentityConflict;
            assessment.reason_codes.push("same_receipt_id_has_conflicting_content".to_owned());
        } else if group.len() > 1 {
            assessment.reason_codes.push("exact_duplicate_receipt_collapsed_idempotently".to_owned());
        }

        if let Some(key) = sequence_key(first) {
            if conflicting_sequences.contains(&key) {
                assessment.disposition = ReceiptDisposition::SequenceConflict;
                assessment.reason_codes.push("same_stream_epoch_sequence_has_different_artifact_digests".to_owned());
            }
        }
        assessment.reason_codes.sort();
        assessment.reason_codes.dedup();
        if assessment.receipt_id != receipt_id {
            assessment.disposition = ReceiptDisposition::ReceiptIdentityConflict;
        }
        assessments.push(assessment);
    }

    assessments.sort_by(|a, b| a.receipt_id.cmp(&b.receipt_id));
    let eligible_receipt_ids: Vec<String> = assessments
        .iter()
        .filter(|a| a.disposition == ReceiptDisposition::EligibleByCutoff)
        .map(|a| a.receipt_id.clone())
        .collect();
    let excluded_receipt_ids: Vec<String> = assessments
        .iter()
        .filter(|a| a.disposition != ReceiptDisposition::EligibleByCutoff)
        .map(|a| a.receipt_id.clone())
        .collect();

    QualificationReport {
        cutoff_id: cutoff.cutoff_id.clone(),
        policy_id: policy.policy_id.clone(),
        cutoff_qualification: cutoff_status,
        assessments,
        eligible_receipt_ids,
        excluded_receipt_ids,
    }
}

pub fn validate_corpus_json(json: &str) -> Result<usize, CorpusError> {
    let corpus: QualificationCorpus =
        serde_json::from_str(json).map_err(|e| CorpusError::InvalidJson(e.to_string()))?;
    if corpus.cases.is_empty() {
        return Err(CorpusError::EmptyCorpus);
    }
    let mut seen_case_ids = BTreeSet::new();
    for case in &corpus.cases {
        if !nonblank(&case.case_id) || !seen_case_ids.insert(case.case_id.as_str()) {
            return Err(CorpusError::EmptyCaseId);
        }
        let report = qualify_receipts(&case.receipts, &case.cutoff, &case.policy);
        let actual: BTreeMap<&str, ReceiptDisposition> = report
            .assessments
            .iter()
            .map(|a| (a.receipt_id.as_str(), a.disposition))
            .collect();
        let expected: BTreeMap<&str, ReceiptDisposition> = case
            .expected
            .iter()
            .map(|e| (e.receipt_id.as_str(), e.disposition))
            .collect();

        for expected_entry in &case.expected {
            match actual.get(expected_entry.receipt_id.as_str()) {
                Some(disposition) if *disposition == expected_entry.disposition => {}
                Some(disposition) => {
                    return Err(CorpusError::ExpectedMismatch {
                        case_id: case.case_id.clone(),
                        receipt_id: expected_entry.receipt_id.clone(),
                        expected: expected_entry.disposition,
                        actual: Some(*disposition),
                    });
                }
                None => {
                    return Err(CorpusError::ExpectedMismatch {
                        case_id: case.case_id.clone(),
                        receipt_id: expected_entry.receipt_id.clone(),
                        expected: expected_entry.disposition,
                        actual: None,
                    });
                }
            }
        }
        for receipt_id in actual.keys() {
            if !expected.contains_key(receipt_id) {
                return Err(CorpusError::UnexpectedReceipt {
                    case_id: case.case_id.clone(),
                    receipt_id: (*receipt_id).to_owned(),
                });
            }
        }
    }
    Ok(corpus.cases.len())
}

pub fn validate_tracked_corpus() -> Result<usize, CorpusError> {
    validate_corpus_json(CORPUS_JSON)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn frozen_ingress_receipt_corpus_matches_all_expected_dispositions() {
        assert_eq!(validate_tracked_corpus().expect("tracked corpus"), 36);
    }

    #[test]
    fn interval_comparison_uses_both_receipt_and_cutoff_uncertainty() {
        let corpus: QualificationCorpus = serde_json::from_str(CORPUS_JSON).unwrap();
        let find = |id: &str| corpus.cases.iter().find(|c| c.case_id == id).unwrap();
        for (id, expected) in [
            ("IRT-001", ReceiptDisposition::EligibleByCutoff),
            ("IRT-002", ReceiptDisposition::DefinitelyAfterCutoff),
            ("IRT-003", ReceiptDisposition::CutoffIndeterminate),
            ("IRT-004", ReceiptDisposition::EligibleByCutoff),
        ] {
            let c = find(id);
            let report = qualify_receipts(&c.receipts, &c.cutoff, &c.policy);
            assert_eq!(report.assessments[0].disposition, expected, "{id}");
        }
    }

    #[test]
    fn different_receivers_do_not_collapse_one_artifact_into_one_receipt() {
        let corpus: QualificationCorpus = serde_json::from_str(CORPUS_JSON).unwrap();
        let c = corpus.cases.iter().find(|c| c.case_id == "IRT-034").unwrap();
        let report = qualify_receipts(&c.receipts, &c.cutoff, &c.policy);
        assert_eq!(report.assessments.len(), 2);
        assert_eq!(report.eligible_receipt_ids.len(), 2);
        assert_ne!(report.assessments[0].receiver_id, report.assessments[1].receiver_id);
    }
}
