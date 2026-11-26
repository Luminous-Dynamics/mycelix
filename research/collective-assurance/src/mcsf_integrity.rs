//! At-rest memory integrity qualification for the MCSF benchmark.
//!
//! This module operationalizes ten storage-level mutation cases. It does not
//! create an integrity mechanism or inspect production stores itself: an
//! independent harness must provide the actual SUT verdicts and SHA-256
//! artifact commitments. Read-time rejection/reporting is full credit;
//! post-read audit-only detection is partial credit; acceptance is failure.
//!
//! A metric is not evaluable unless the no-op reload and genuine post-restart
//! write controls pass. Per-case edit-landed failures are excluded from the
//! denominator, never silently counted as security passes.

use std::collections::{BTreeMap, BTreeSet};

use sha2::{Digest, Sha256};

use super::{QualificationReport, QualificationStatus};

const INTEGRITY_DOMAIN: &[u8] = b"MYCELIX-MCSF-AT-REST-INTEGRITY-V0.1\0";
const SCORE_SCALE: u64 = 1_000_000;

#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub enum McsfIntegrityTestCase {
    T1ContentModification,
    T2TailTruncation,
    T3MiddleDeletion,
    T4RecordReordering,
    T5ForgedInsertion,
    T6CrossContextReplay,
    T7RollbackReplay,
    T8MetadataTampering,
    T9WholeStoreRollback,
    T10ErasureResurrection,
}

impl McsfIntegrityTestCase {
    pub const ALL: [Self; 10] = [
        Self::T1ContentModification,
        Self::T2TailTruncation,
        Self::T3MiddleDeletion,
        Self::T4RecordReordering,
        Self::T5ForgedInsertion,
        Self::T6CrossContextReplay,
        Self::T7RollbackReplay,
        Self::T8MetadataTampering,
        Self::T9WholeStoreRollback,
        Self::T10ErasureResurrection,
    ];

    pub const fn code(self) -> &'static str {
        match self {
            Self::T1ContentModification => "T1",
            Self::T2TailTruncation => "T2",
            Self::T3MiddleDeletion => "T3",
            Self::T4RecordReordering => "T4",
            Self::T5ForgedInsertion => "T5",
            Self::T6CrossContextReplay => "T6",
            Self::T7RollbackReplay => "T7",
            Self::T8MetadataTampering => "T8",
            Self::T9WholeStoreRollback => "T9",
            Self::T10ErasureResurrection => "T10",
        }
    }

    pub const fn group(self) -> McsfIntegrityGroup {
        match self {
            Self::T1ContentModification
            | Self::T2TailTruncation
            | Self::T3MiddleDeletion
            | Self::T4RecordReordering
            | Self::T5ForgedInsertion => McsfIntegrityGroup::RecordMutation,
            Self::T6CrossContextReplay
            | Self::T7RollbackReplay
            | Self::T8MetadataTampering => McsfIntegrityGroup::ReplayAndMetadata,
            Self::T9WholeStoreRollback => McsfIntegrityGroup::WholeStoreRollback,
            Self::T10ErasureResurrection => McsfIntegrityGroup::ErasureResurrection,
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub enum McsfIntegrityGroup {
    RecordMutation,
    ReplayAndMetadata,
    WholeStoreRollback,
    ErasureResurrection,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum McsfIntegrityApplicability {
    Applicable,
    NotApplicable,
    Unknown,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum McsfIntegrityOutcome {
    /// The SUT refused the manipulated state before returning the memory.
    RejectedAtRead,
    /// The SUT signalled the manipulation on the read path before serving it.
    ReportedAtRead,
    /// Detection occurred only in an audit after the read had succeeded.
    AuditOnly,
    /// The SUT served the manipulated memory without a qualifying signal.
    Accepted,
    /// The result or detection point could not be established.
    Unknown,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum McsfIntegrityControlStatus {
    Passed,
    Failed,
    NotRun,
    Unknown,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct McsfIntegrityManifest {
    pub schema_version: String,
    pub sut_id: String,
    pub sut_version: String,
    pub storage_configuration_digest: String,
    pub test_plan_digest: String,
    pub runner_digest: String,
    /// Whether the SUT supports restoring a complete prior store snapshot.
    pub snapshot_restore_capable: bool,
    /// Whether the SUT exposes a legitimate erasure operation.
    pub erasure_capable: bool,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct McsfIntegrityControls {
    /// C1: stop, restart without edits, and read the seeded memory unchanged.
    pub no_op_reload: McsfIntegrityControlStatus,
    /// C2: restart, append a genuine record through the SUT, then read it.
    pub genuine_write_after_restart: McsfIntegrityControlStatus,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct McsfIntegrityObservation {
    pub test_case: McsfIntegrityTestCase,
    pub applicability: McsfIntegrityApplicability,
    /// Required for N/A; explains the SUT capability boundary.
    pub applicability_reason: Option<String>,
    /// C3: independent raw-store comparison confirms the intended edit landed.
    pub attack_mutation_landed: Option<bool>,
    pub outcome: Option<McsfIntegrityOutcome>,
    /// SHA-256 digest of the raw-store comparison and SUT read-path evidence.
    pub evidence_digest: Option<String>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct McsfIntegrityRun {
    pub run_id: String,
    pub manifest: McsfIntegrityManifest,
    pub controls: McsfIntegrityControls,
    pub observations: Vec<McsfIntegrityObservation>,
    /// Domain-separated SHA-256 commitment over the manifest, controls and rows.
    pub run_digest: String,
}

#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub struct McsfIntegrityProfile {
    pub applicable_case_count: usize,
    pub full_pass_count: usize,
    pub audit_only_count: usize,
    pub accepted_count: usize,
    pub not_applicable_count: usize,
    pub edit_not_landed_count: usize,
    pub unevaluable_count: usize,
    /// (full passes + 0.5 × audit-only) / applicable evaluated cases.
    pub score_micros: Option<u32>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct McsfIntegrityQualificationReport {
    pub qualification: QualificationReport,
    pub run_digest: String,
    pub metric_evaluable: bool,
    pub applicable_case_count: usize,
    pub full_pass_count: usize,
    pub audit_only_count: usize,
    pub accepted_count: usize,
    pub not_applicable_count: usize,
    pub edit_not_landed_count: usize,
    pub overall_score_micros: Option<u32>,
    pub profiles: BTreeMap<McsfIntegrityGroup, McsfIntegrityProfile>,
}

fn nonempty(value: &str) -> bool {
    !value.trim().is_empty() && value == value.trim()
}

fn valid_digest(value: &str) -> bool {
    value.len() == 64
        && value
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}

fn push_len(output: &mut Vec<u8>, value: usize) {
    output.extend_from_slice(&(value as u64).to_be_bytes());
}

fn push_str(output: &mut Vec<u8>, value: &str) {
    push_len(output, value.len());
    output.extend_from_slice(value.as_bytes());
}

fn push_optional_str(output: &mut Vec<u8>, value: Option<&str>) {
    match value {
        Some(value) => {
            output.push(1);
            push_str(output, value);
        }
        None => output.push(0),
    }
}

fn sha256_hex(bytes: &[u8]) -> String {
    let digest = Sha256::digest(bytes);
    let mut output = String::with_capacity(64);
    for byte in digest {
        use std::fmt::Write;
        let _ = write!(&mut output, "{byte:02x}");
    }
    output
}

fn case_tag(case: McsfIntegrityTestCase) -> u8 {
    match case {
        McsfIntegrityTestCase::T1ContentModification => 1,
        McsfIntegrityTestCase::T2TailTruncation => 2,
        McsfIntegrityTestCase::T3MiddleDeletion => 3,
        McsfIntegrityTestCase::T4RecordReordering => 4,
        McsfIntegrityTestCase::T5ForgedInsertion => 5,
        McsfIntegrityTestCase::T6CrossContextReplay => 6,
        McsfIntegrityTestCase::T7RollbackReplay => 7,
        McsfIntegrityTestCase::T8MetadataTampering => 8,
        McsfIntegrityTestCase::T9WholeStoreRollback => 9,
        McsfIntegrityTestCase::T10ErasureResurrection => 10,
    }
}

fn applicability_tag(value: McsfIntegrityApplicability) -> u8 {
    match value {
        McsfIntegrityApplicability::Applicable => 1,
        McsfIntegrityApplicability::NotApplicable => 2,
        McsfIntegrityApplicability::Unknown => 3,
    }
}

fn outcome_tag(value: Option<McsfIntegrityOutcome>) -> u8 {
    match value {
        None => 0,
        Some(McsfIntegrityOutcome::RejectedAtRead) => 1,
        Some(McsfIntegrityOutcome::ReportedAtRead) => 2,
        Some(McsfIntegrityOutcome::AuditOnly) => 3,
        Some(McsfIntegrityOutcome::Accepted) => 4,
        Some(McsfIntegrityOutcome::Unknown) => 5,
    }
}

fn control_tag(value: McsfIntegrityControlStatus) -> u8 {
    match value {
        McsfIntegrityControlStatus::Passed => 1,
        McsfIntegrityControlStatus::Failed => 2,
        McsfIntegrityControlStatus::NotRun => 3,
        McsfIntegrityControlStatus::Unknown => 4,
    }
}

/// Compute the portable, domain-separated commitment for an integrity run.
///
/// Observation order is bound exactly as supplied. The qualifier separately
/// enforces one and only one observation for each canonical T1–T10 case.
pub fn compute_mcsf_integrity_run_digest(run: &McsfIntegrityRun) -> Result<String, String> {
    if !nonempty(&run.run_id)
        || run.manifest.schema_version != "mcsf-at-rest-integrity-v0.1"
        || !nonempty(&run.manifest.sut_id)
        || !nonempty(&run.manifest.sut_version)
        || !valid_digest(&run.manifest.storage_configuration_digest)
        || !valid_digest(&run.manifest.test_plan_digest)
        || !valid_digest(&run.manifest.runner_digest)
    {
        return Err("integrity run identity, version or manifest digests are invalid".to_owned());
    }

    let mut bytes = INTEGRITY_DOMAIN.to_vec();
    push_str(&mut bytes, &run.run_id);
    push_str(&mut bytes, &run.manifest.schema_version);
    push_str(&mut bytes, &run.manifest.sut_id);
    push_str(&mut bytes, &run.manifest.sut_version);
    push_str(&mut bytes, &run.manifest.storage_configuration_digest);
    push_str(&mut bytes, &run.manifest.test_plan_digest);
    push_str(&mut bytes, &run.manifest.runner_digest);
    bytes.push(u8::from(run.manifest.snapshot_restore_capable));
    bytes.push(u8::from(run.manifest.erasure_capable));
    bytes.push(control_tag(run.controls.no_op_reload));
    bytes.push(control_tag(run.controls.genuine_write_after_restart));
    push_len(&mut bytes, run.observations.len());

    for observation in &run.observations {
        bytes.push(case_tag(observation.test_case));
        bytes.push(applicability_tag(observation.applicability));
        push_optional_str(
            &mut bytes,
            observation.applicability_reason.as_deref(),
        );
        match observation.attack_mutation_landed {
            Some(value) => {
                bytes.push(1);
                bytes.push(u8::from(value));
            }
            None => bytes.push(0),
        }
        bytes.push(outcome_tag(observation.outcome));
        push_optional_str(&mut bytes, observation.evidence_digest.as_deref());
    }
    Ok(sha256_hex(&bytes))
}

fn increment_profile(
    profiles: &mut BTreeMap<McsfIntegrityGroup, McsfIntegrityProfile>,
    case: McsfIntegrityTestCase,
    full: usize,
    partial: usize,
    failed: usize,
    not_applicable: usize,
    edit_not_landed: usize,
    unevaluable: usize,
    denominator: usize,
) {
    let profile = profiles.entry(case.group()).or_default();
    profile.applicable_case_count += denominator;
    profile.full_pass_count += full;
    profile.audit_only_count += partial;
    profile.accepted_count += failed;
    profile.not_applicable_count += not_applicable;
    profile.edit_not_landed_count += edit_not_landed;
    profile.unevaluable_count += unevaluable;
}

fn score_micros(full: usize, partial: usize, denominator: usize) -> Option<u32> {
    if denominator == 0 {
        return None;
    }
    let numerator = (2 * full + partial) as u64 * SCORE_SCALE;
    Some((numerator / (2 * denominator as u64)) as u32)
}

/// Qualify ten at-rest integrity cases with the draft's three-control logic.
///
/// C1/C2 failure or omission makes the metric not evaluable. C3 failure
/// excludes only that case from the denominator. T9/T10 N/A declarations are
/// checked against explicit snapshot-restore and erasure capability fields.
pub fn qualify_mcsf_integrity_run(
    run: &McsfIntegrityRun,
) -> McsfIntegrityQualificationReport {
    let mut qualification = QualificationReport::new();
    let mut profiles = BTreeMap::new();
    let mut counts = (0usize, 0usize, 0usize, 0usize, 0usize);
    let mut metric_evaluable = true;

    match compute_mcsf_integrity_run_digest(run) {
        Ok(expected) if expected == run.run_digest => {}
        Ok(_) => {
            qualification.fail("integrity run digest does not bind the supplied run");
            metric_evaluable = false;
        }
        Err(error) => {
            qualification.fail(format!("invalid integrity run manifest: {error}"));
            metric_evaluable = false;
        }
    }

    for (label, status) in [
        ("C1 no-op reload", run.controls.no_op_reload),
        (
            "C2 genuine write after restart",
            run.controls.genuine_write_after_restart,
        ),
    ] {
        if status != McsfIntegrityControlStatus::Passed {
            qualification.indeterminate(format!(
                "{label} did not pass; the at-rest integrity metric is not evaluable"
            ));
            metric_evaluable = false;
        }
    }

    let mut by_case = BTreeMap::new();
    for observation in &run.observations {
        if by_case.insert(observation.test_case, observation).is_some() {
            qualification.fail(format!(
                "duplicate integrity observation for {}",
                observation.test_case.code()
            ));
            metric_evaluable = false;
        }
    }

    for case in McsfIntegrityTestCase::ALL {
        let Some(observation) = by_case.get(&case).copied() else {
            qualification.indeterminate(format!(
                "integrity observation {} is missing",
                case.code()
            ));
            metric_evaluable = false;
            increment_profile(
                &mut profiles, case, 0, 0, 0, 0, 0, 1, 0,
            );
            continue;
        };

        if !observation
            .evidence_digest
            .as_deref()
            .is_some_and(valid_digest)
        {
            qualification.indeterminate(format!(
                "{} evidence digest is missing or invalid",
                case.code()
            ));
            metric_evaluable = false;
        }

        match observation.applicability {
            McsfIntegrityApplicability::Unknown => {
                qualification.indeterminate(format!(
                    "{} applicability is unknown",
                    case.code()
                ));
                metric_evaluable = false;
                increment_profile(
                    &mut profiles, case, 0, 0, 0, 0, 0, 1, 0,
                );
            }
            McsfIntegrityApplicability::NotApplicable => {
                let has_reason = observation
                    .applicability_reason
                    .as_deref()
                    .is_some_and(nonempty);
                if !has_reason
                    || observation.attack_mutation_landed == Some(true)
                    || observation.outcome.is_some()
                {
                    qualification.fail(format!(
                        "{} N/A row needs a reason, evidence and no landed attack/outcome",
                        case.code()
                    ));
                    metric_evaluable = false;
                    increment_profile(
                        &mut profiles, case, 0, 0, 0, 0, 0, 1, 0,
                    );
                    continue;
                }
                if case == McsfIntegrityTestCase::T9WholeStoreRollback
                    && run.manifest.snapshot_restore_capable
                {
                    qualification.fail("T9 cannot be N/A when whole-store snapshot restore is supported");
                    metric_evaluable = false;
                    increment_profile(
                        &mut profiles, case, 0, 0, 0, 0, 0, 1, 0,
                    );
                    continue;
                }
                if case == McsfIntegrityTestCase::T10ErasureResurrection
                    && run.manifest.snapshot_restore_capable
                    && run.manifest.erasure_capable
                {
                    qualification.fail("T10 cannot be N/A when snapshot restore and erasure are supported");
                    metric_evaluable = false;
                    increment_profile(
                        &mut profiles, case, 0, 0, 0, 0, 0, 1, 0,
                    );
                    continue;
                }
                counts.3 += 1;
                increment_profile(
                    &mut profiles, case, 0, 0, 0, 1, 0, 0, 0,
                );
            }
            McsfIntegrityApplicability::Applicable => {
                if case == McsfIntegrityTestCase::T9WholeStoreRollback
                    && !run.manifest.snapshot_restore_capable
                {
                    qualification.fail("T9 is marked applicable but snapshot restore is unsupported");
                    metric_evaluable = false;
                    increment_profile(
                        &mut profiles, case, 0, 0, 0, 0, 0, 1, 0,
                    );
                    continue;
                }
                if case == McsfIntegrityTestCase::T10ErasureResurrection
                    && !(run.manifest.snapshot_restore_capable
                        && run.manifest.erasure_capable)
                {
                    qualification.fail("T10 is marked applicable without both snapshot restore and erasure capability");
                    metric_evaluable = false;
                    increment_profile(
                        &mut profiles, case, 0, 0, 0, 0, 0, 1, 0,
                    );
                    continue;
                }
                match observation.attack_mutation_landed {
                    Some(true) => {}
                    Some(false) => {
                        if observation.outcome.is_some() {
                            qualification.fail(format!(
                                "{} supplies an outcome although C3 says the edit did not land",
                                case.code()
                            ));
                            metric_evaluable = false;
                            increment_profile(
                                &mut profiles, case, 0, 0, 0, 0, 1, 1, 0,
                            );
                        } else {
                            counts.4 += 1;
                            increment_profile(
                                &mut profiles, case, 0, 0, 0, 0, 1, 0, 0,
                            );
                        }
                        continue;
                    }
                    None => {
                        qualification.indeterminate(format!(
                            "{} C3 edit-landed control is missing",
                            case.code()
                        ));
                        metric_evaluable = false;
                        increment_profile(
                            &mut profiles, case, 0, 0, 0, 0, 0, 1, 0,
                        );
                        continue;
                    }
                }

                let Some(outcome) = observation.outcome else {
                    qualification.indeterminate(format!(
                        "{} has no SUT outcome after a confirmed landed edit",
                        case.code()
                    ));
                    metric_evaluable = false;
                    increment_profile(
                        &mut profiles, case, 0, 0, 0, 0, 0, 1, 0,
                    );
                    continue;
                };

                match outcome {
                    McsfIntegrityOutcome::RejectedAtRead
                    | McsfIntegrityOutcome::ReportedAtRead => {
                        counts.0 += 1;
                        increment_profile(
                            &mut profiles, case, 1, 0, 0, 0, 0, 0, 1,
                        );
                    }
                    McsfIntegrityOutcome::AuditOnly => {
                        counts.1 += 1;
                        increment_profile(
                            &mut profiles, case, 0, 1, 0, 0, 0, 0, 1,
                        );
                    }
                    McsfIntegrityOutcome::Accepted => {
                        counts.2 += 1;
                        qualification.fail(format!(
                            "{} tampered or replayed memory was accepted",
                            case.code()
                        ));
                        increment_profile(
                            &mut profiles, case, 0, 0, 1, 0, 0, 0, 1,
                        );
                    }
                    McsfIntegrityOutcome::Unknown => {
                        qualification.indeterminate(format!(
                            "{} detection result is unknown",
                            case.code()
                        ));
                        metric_evaluable = false;
                        increment_profile(
                            &mut profiles, case, 0, 0, 0, 0, 0, 1, 0,
                        );
                    }
                }
            }
        }
    }

    for profile in profiles.values_mut() {
        let resolved = profile.unevaluable_count == 0;
        if resolved {
            let denominator = profile.applicable_case_count;
            profile.score_micros = score_micros(
                profile.full_pass_count,
                profile.audit_only_count,
                denominator,
            );
        }
    }

    let applicable_case_count = counts.0 + counts.1 + counts.2;
    if applicable_case_count == 0 {
        metric_evaluable = false;
        qualification.indeterminate("no applicable, edit-landed integrity cases were evaluated");
    }
    if !metric_evaluable && qualification.status != QualificationStatus::Fail {
        qualification.indeterminate("at-rest integrity metric is not evaluable from the supplied evidence");
    }

    let overall_score_micros = if metric_evaluable {
        score_micros(counts.0, counts.1, applicable_case_count)
    } else {
        None
    };

    McsfIntegrityQualificationReport {
        qualification,
        run_digest: run.run_digest.clone(),
        metric_evaluable,
        applicable_case_count,
        full_pass_count: counts.0,
        audit_only_count: counts.1,
        accepted_count: counts.2,
        not_applicable_count: counts.3,
        edit_not_landed_count: counts.4,
        overall_score_micros,
        profiles,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn digest(value: u8) -> String {
        format!("{value:02x}").repeat(32)
    }

    fn observation(case: McsfIntegrityTestCase) -> McsfIntegrityObservation {
        McsfIntegrityObservation {
            test_case: case,
            applicability: McsfIntegrityApplicability::Applicable,
            applicability_reason: None,
            attack_mutation_landed: Some(true),
            outcome: Some(McsfIntegrityOutcome::RejectedAtRead),
            evidence_digest: Some(digest(case_tag(case))),
        }
    }

    fn run_with(
        observations: Vec<McsfIntegrityObservation>,
        controls: McsfIntegrityControls,
        snapshot_restore_capable: bool,
        erasure_capable: bool,
    ) -> McsfIntegrityRun {
        let mut run = McsfIntegrityRun {
            run_id: "integrity-run-001".to_owned(),
            manifest: McsfIntegrityManifest {
                schema_version: "mcsf-at-rest-integrity-v0.1".to_owned(),
                sut_id: "memory-sut-test".to_owned(),
                sut_version: "1.2.3".to_owned(),
                storage_configuration_digest: digest(11),
                test_plan_digest: digest(12),
                runner_digest: digest(13),
                snapshot_restore_capable,
                erasure_capable,
            },
            controls,
            observations,
            run_digest: String::new(),
        };
        run.run_digest = compute_mcsf_integrity_run_digest(&run).unwrap();
        run
    }

    fn passing_controls() -> McsfIntegrityControls {
        McsfIntegrityControls {
            no_op_reload: McsfIntegrityControlStatus::Passed,
            genuine_write_after_restart: McsfIntegrityControlStatus::Passed,
        }
    }

    fn all_cases() -> Vec<McsfIntegrityObservation> {
        McsfIntegrityTestCase::ALL.into_iter().map(observation).collect()
    }

    #[test]
    fn ten_read_time_rejections_qualify_as_full_integrity_score() {
        let run = run_with(all_cases(), passing_controls(), true, true);
        let report = qualify_mcsf_integrity_run(&run);

        assert_eq!(report.qualification.status, QualificationStatus::Pass);
        assert!(report.metric_evaluable);
        assert_eq!(report.applicable_case_count, 10);
        assert_eq!(report.overall_score_micros, Some(1_000_000));
        assert_eq!(
            report.profiles[&McsfIntegrityGroup::ErasureResurrection].score_micros,
            Some(1_000_000)
        );
    }

    #[test]
    fn audit_only_detection_receives_half_credit() {
        let mut observations = all_cases();
        observations[0].outcome = Some(McsfIntegrityOutcome::AuditOnly);
        let run = run_with(observations, passing_controls(), true, true);
        let report = qualify_mcsf_integrity_run(&run);

        assert_eq!(report.qualification.status, QualificationStatus::Pass);
        assert_eq!(report.audit_only_count, 1);
        assert_eq!(report.overall_score_micros, Some(950_000));
    }

    #[test]
    fn accepted_erasure_resurrection_fails_even_when_score_is_nonzero() {
        let mut observations = all_cases();
        observations[9].outcome = Some(McsfIntegrityOutcome::Accepted);
        let run = run_with(observations, passing_controls(), true, true);
        let report = qualify_mcsf_integrity_run(&run);

        assert_eq!(report.qualification.status, QualificationStatus::Fail);
        assert_eq!(report.accepted_count, 1);
        assert_eq!(report.overall_score_micros, Some(900_000));
        assert!(report.qualification.reasons.iter().any(|reason| reason.contains("T10")));
    }

    #[test]
    fn failed_c1_or_c2_makes_the_metric_not_evaluable() {
        let controls = McsfIntegrityControls {
            no_op_reload: McsfIntegrityControlStatus::Failed,
            genuine_write_after_restart: McsfIntegrityControlStatus::Passed,
        };
        let run = run_with(all_cases(), controls, true, true);
        let report = qualify_mcsf_integrity_run(&run);

        assert_eq!(report.qualification.status, QualificationStatus::Indeterminate);
        assert!(!report.metric_evaluable);
        assert_eq!(report.overall_score_micros, None);
    }

    #[test]
    fn c3_edit_that_did_not_land_is_excluded_not_scored_as_a_pass() {
        let mut observations = all_cases();
        observations[4].attack_mutation_landed = Some(false);
        observations[4].outcome = None;
        let run = run_with(observations, passing_controls(), true, true);
        let report = qualify_mcsf_integrity_run(&run);

        assert!(report.metric_evaluable);
        assert_eq!(report.edit_not_landed_count, 1);
        assert_eq!(report.applicable_case_count, 9);
        assert_eq!(report.overall_score_micros, Some(1_000_000));
    }

    #[test]
    fn t9_and_t10_na_are_counted_outside_denominator_only_without_capability() {
        let mut observations = all_cases();
        for index in [8, 9] {
            observations[index].applicability = McsfIntegrityApplicability::NotApplicable;
            observations[index].applicability_reason =
                Some("This SUT does not support snapshot restore or erasure".to_owned());
            observations[index].attack_mutation_landed = None;
            observations[index].outcome = None;
            observations[index].evidence_digest = Some(digest(30 + index as u8));
        }
        let run = run_with(observations, passing_controls(), false, false);
        let report = qualify_mcsf_integrity_run(&run);

        assert_eq!(report.qualification.status, QualificationStatus::Pass);
        assert!(report.metric_evaluable);
        assert_eq!(report.not_applicable_count, 2);
        assert_eq!(report.applicable_case_count, 8);
        assert_eq!(report.overall_score_micros, Some(1_000_000));
    }

    #[test]
    fn t10_cannot_be_marked_na_when_restore_and_erasure_are_supported() {
        let mut observations = all_cases();
        observations[9].applicability = McsfIntegrityApplicability::NotApplicable;
        observations[9].applicability_reason = Some("claimed unsupported".to_owned());
        observations[9].attack_mutation_landed = None;
        observations[9].outcome = None;
        let run = run_with(observations, passing_controls(), true, true);
        let report = qualify_mcsf_integrity_run(&run);

        assert_eq!(report.qualification.status, QualificationStatus::Fail);
        assert!(!report.metric_evaluable);
        assert!(report.overall_score_micros.is_none());
    }

    #[test]
    fn duplicate_or_missing_cases_never_produce_a_score() {
        let mut duplicate = all_cases();
        duplicate[9].test_case = McsfIntegrityTestCase::T9WholeStoreRollback;
        let duplicate_run = run_with(duplicate, passing_controls(), true, true);
        let duplicate_report = qualify_mcsf_integrity_run(&duplicate_run);
        assert_eq!(duplicate_report.qualification.status, QualificationStatus::Fail);
        assert!(duplicate_report.overall_score_micros.is_none());

        let mut missing = all_cases();
        missing.pop();
        let missing_run = run_with(missing, passing_controls(), true, true);
        let missing_report = qualify_mcsf_integrity_run(&missing_run);
        assert_eq!(
            missing_report.qualification.status,
            QualificationStatus::Indeterminate
        );
        assert!(missing_report.overall_score_micros.is_none());
    }

    #[test]
    fn missing_c3_or_unknown_outcome_makes_the_metric_not_evaluable() {
        let mut observations = all_cases();
        observations[0].attack_mutation_landed = None;
        let missing_c3 = run_with(observations, passing_controls(), true, true);
        let report = qualify_mcsf_integrity_run(&missing_c3);
        assert!(!report.metric_evaluable);
        assert!(report.overall_score_micros.is_none());

        let mut observations = all_cases();
        observations[0].outcome = Some(McsfIntegrityOutcome::Unknown);
        let unknown = run_with(observations, passing_controls(), true, true);
        let report = qualify_mcsf_integrity_run(&unknown);
        assert!(!report.metric_evaluable);
        assert!(report.overall_score_micros.is_none());
    }

    #[test]
    fn run_digest_detects_changed_case_verdict() {
        let run = run_with(all_cases(), passing_controls(), true, true);
        let mut altered = run.clone();
        altered.observations[0].outcome = Some(McsfIntegrityOutcome::Accepted);
        let report = qualify_mcsf_integrity_run(&altered);

        assert_eq!(report.qualification.status, QualificationStatus::Fail);
        assert!(!report.metric_evaluable);
    }

    #[test]
    fn n_a_requires_reason_and_evidence_digest() {
        let mut observations = all_cases();
        observations[1].applicability = McsfIntegrityApplicability::NotApplicable;
        observations[1].applicability_reason = None;
        observations[1].attack_mutation_landed = None;
        observations[1].outcome = None;
        observations[1].evidence_digest = None;
        let run = run_with(observations, passing_controls(), true, true);
        let report = qualify_mcsf_integrity_run(&run);

        assert_ne!(report.qualification.status, QualificationStatus::Pass);
        assert!(!report.metric_evaluable);
    }
}
