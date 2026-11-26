//! Non-compensatory qualification for collaborative, role-scoped memory.
//!
//! This native Mycelix contract is designed to interoperate with COLLABMEM-style
//! matched memory-only studies. It does not claim an official COLLABMEM run:
//! official dataset, harness and split digests must be independently supplied.
//!
//! Unauthorized disclosure, access grants, routing, forged provenance, stale
//! authority, and post-revocation use are hard failures. An incomplete observer
//! yields INDETERMINATE rather than allowing absence of evidence to pass.

use std::collections::{BTreeMap, BTreeSet};

use sha2::{Digest, Sha256};

use super::{QualificationReport, QualificationStatus};

const CONTROL_DOMAIN: &[u8] = b"MYCELIX-COLLABORATIVE-MEMORY-CONTROLS-V0.1\0";
const RUN_DOMAIN: &[u8] = b"MYCELIX-COLLABORATIVE-MEMORY-RUN-V0.1\0";
const SCORE_SCALE: u64 = 1_000_000;

#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub enum CollabMemoryCompetency {
    Sharing,
    Access,
    Routing,
    Provenance,
    ConflictResolution,
    Retention,
}

impl CollabMemoryCompetency {
    pub const ALL: [Self; 6] = [
        Self::Sharing,
        Self::Access,
        Self::Routing,
        Self::Provenance,
        Self::ConflictResolution,
        Self::Retention,
    ];

    pub const fn code(self) -> &'static str {
        match self {
            Self::Sharing => "sharing",
            Self::Access => "access",
            Self::Routing => "routing",
            Self::Provenance => "provenance",
            Self::ConflictResolution => "conflict_resolution",
            Self::Retention => "retention",
        }
    }

    const fn tag(self) -> u8 {
        match self {
            Self::Sharing => 1,
            Self::Access => 2,
            Self::Routing => 3,
            Self::Provenance => 4,
            Self::ConflictResolution => 5,
            Self::Retention => 6,
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CollabMemoryEvidenceClass {
    SyntheticFixture,
    ObservedRun,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CollabMemoryManifest {
    pub schema_version: String,
    pub evidence_class: CollabMemoryEvidenceClass,
    pub suite_id: String,
    /// Dataset/scenario manifest digest, independent of the run's observations.
    pub dataset_manifest_digest: String,
    /// Frozen conversation/event trace and its ordering.
    pub trace_manifest_digest: String,
    /// Target principal/role inventory and expected role projection.
    pub role_manifest_digest: String,
    /// Answerer, judge, prompt, tools, hardware and budget are matched controls.
    pub answerer_model_digest: String,
    pub judge_digest: String,
    pub gold_event_manifest_digest: String,
    pub prompt_digest: String,
    pub toolset_digest: String,
    pub hardware_profile_digest: String,
    pub budget_profile_digest: String,
    pub decoding_config_digest: String,
    pub memory_adapter_digest: String,
    pub memory_policy_digest: String,
    pub seed: u64,
    /// These three exact artifacts are mandatory before official COLLABMEM
    /// compatibility may be claimed. Native Mycelix panels may leave them empty.
    pub official_dataset_digest: Option<String>,
    pub official_harness_digest: Option<String>,
    pub official_split_manifest_digest: Option<String>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum CollabMemoryEvidence {
    Sharing {
        observer_complete: bool,
        expected_recipients: BTreeSet<String>,
        observed_recipients: BTreeSet<String>,
    },
    Access {
        observer_complete: bool,
        principal_id: String,
        resource_scope_id: String,
        purpose_id: String,
        expected_allow: bool,
        observed_allow: bool,
    },
    Routing {
        observer_complete: bool,
        expected_role_id: String,
        observed_role_ids: BTreeSet<String>,
    },
    Provenance {
        observer_complete: bool,
        expected_source_event_digest: String,
        observed_source_event_digest: String,
        expected_writer_id: String,
        observed_writer_id: String,
        expected_authority_digest: String,
        observed_authority_digest: String,
    },
    ConflictResolution {
        observer_complete: bool,
        expected_current_value_digest: String,
        observed_current_value_digest: String,
        expected_authority_rank: u32,
        observed_authority_rank: u32,
    },
    Retention {
        observer_complete: bool,
        expected_retained: bool,
        observed_retained: bool,
        stale_memory_used: bool,
        post_revocation_used: bool,
        derived_descendant_used_after_revocation: bool,
    },
}

impl CollabMemoryEvidence {
    pub const fn competency(&self) -> CollabMemoryCompetency {
        match self {
            Self::Sharing { .. } => CollabMemoryCompetency::Sharing,
            Self::Access { .. } => CollabMemoryCompetency::Access,
            Self::Routing { .. } => CollabMemoryCompetency::Routing,
            Self::Provenance { .. } => CollabMemoryCompetency::Provenance,
            Self::ConflictResolution { .. } => CollabMemoryCompetency::ConflictResolution,
            Self::Retention { .. } => CollabMemoryCompetency::Retention,
        }
    }

    const fn observer_complete(&self) -> bool {
        match self {
            Self::Sharing { observer_complete, .. }
            | Self::Access { observer_complete, .. }
            | Self::Routing { observer_complete, .. }
            | Self::Provenance { observer_complete, .. }
            | Self::ConflictResolution { observer_complete, .. }
            | Self::Retention { observer_complete, .. } => *observer_complete,
        }
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CollabMemoryProbe {
    pub probe_id: String,
    pub evidence: CollabMemoryEvidence,
    /// Independently captured artifact digest: memory view, policy decision,
    /// provenance edge, or retention/revocation witness.
    pub evidence_digest: String,
    /// Applies across all competencies when an unauthorized principal saw content.
    pub unauthorized_content_exposure: bool,
    /// Side-channel or metadata inference prohibited by the scenario policy.
    pub forbidden_metadata_inference: bool,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CollabMemoryRun {
    pub run_id: String,
    pub manifest: CollabMemoryManifest,
    pub probes: Vec<CollabMemoryProbe>,
    pub control_digest: String,
    pub run_digest: String,
}

#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub struct CollabCompetencyScore {
    pub observed_probe_count: usize,
    pub passed_probe_count: usize,
    pub failed_probe_count: usize,
    pub incomplete_probe_count: usize,
    pub score_micros: Option<u32>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CollabMemoryQualificationReport {
    pub qualification: QualificationReport,
    pub control_digest: String,
    pub run_digest: String,
    pub metric_evaluable: bool,
    pub official_collabmem_compatible: bool,
    pub scores: BTreeMap<CollabMemoryCompetency, CollabCompetencyScore>,
    pub unauthorized_content_exposures: u64,
    pub forbidden_metadata_inferences: u64,
    pub unauthorized_access_grants: u64,
    pub unauthorized_role_routes: u64,
    pub provenance_mismatches: u64,
    pub authority_conflicts: u64,
    pub stale_memory_uses: u64,
    pub post_revocation_uses: u64,
    pub derived_descendant_uses_after_revocation: u64,
    pub reasons: Vec<String>,
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

fn push_len(bytes: &mut Vec<u8>, value: usize) {
    bytes.extend_from_slice(&(value as u64).to_be_bytes());
}

fn push_str(bytes: &mut Vec<u8>, value: &str) {
    push_len(bytes, value.len());
    bytes.extend_from_slice(value.as_bytes());
}

fn push_optional_str(bytes: &mut Vec<u8>, value: Option<&str>) {
    match value {
        Some(value) => {
            bytes.push(1);
            push_str(bytes, value);
        }
        None => bytes.push(0),
    }
}

fn push_set(bytes: &mut Vec<u8>, values: &BTreeSet<String>) {
    push_len(bytes, values.len());
    for value in values {
        push_str(bytes, value);
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

fn evidence_class_tag(value: CollabMemoryEvidenceClass) -> u8 {
    match value {
        CollabMemoryEvidenceClass::SyntheticFixture => 1,
        CollabMemoryEvidenceClass::ObservedRun => 2,
    }
}

fn encode_manifest(manifest: &CollabMemoryManifest) -> Result<Vec<u8>, String> {
    if manifest.schema_version != "mcsf-collaborative-memory-v0.1"
        || !nonempty(&manifest.suite_id)
        || !valid_digest(&manifest.dataset_manifest_digest)
        || !valid_digest(&manifest.trace_manifest_digest)
        || !valid_digest(&manifest.role_manifest_digest)
        || !valid_digest(&manifest.answerer_model_digest)
        || !valid_digest(&manifest.judge_digest)
        || !valid_digest(&manifest.gold_event_manifest_digest)
        || !valid_digest(&manifest.prompt_digest)
        || !valid_digest(&manifest.toolset_digest)
        || !valid_digest(&manifest.hardware_profile_digest)
        || !valid_digest(&manifest.budget_profile_digest)
        || !valid_digest(&manifest.decoding_config_digest)
        || !valid_digest(&manifest.memory_adapter_digest)
        || !valid_digest(&manifest.memory_policy_digest)
    {
        return Err("collaborative-memory matched controls are incomplete or malformed".to_owned());
    }

    for (label, digest) in [
        ("official dataset", manifest.official_dataset_digest.as_deref()),
        ("official harness", manifest.official_harness_digest.as_deref()),
        ("official split manifest", manifest.official_split_manifest_digest.as_deref()),
    ] {
        if digest.is_some_and(|value| !valid_digest(value)) {
            return Err(format!("{label} digest is malformed"));
        }
    }

    let mut bytes = CONTROL_DOMAIN.to_vec();
    push_str(&mut bytes, &manifest.schema_version);
    bytes.push(evidence_class_tag(manifest.evidence_class));
    push_str(&mut bytes, &manifest.suite_id);
    for digest in [
        &manifest.dataset_manifest_digest,
        &manifest.trace_manifest_digest,
        &manifest.role_manifest_digest,
        &manifest.answerer_model_digest,
        &manifest.judge_digest,
        &manifest.gold_event_manifest_digest,
        &manifest.prompt_digest,
        &manifest.toolset_digest,
        &manifest.hardware_profile_digest,
        &manifest.budget_profile_digest,
        &manifest.decoding_config_digest,
        &manifest.memory_adapter_digest,
        &manifest.memory_policy_digest,
    ] {
        push_str(&mut bytes, digest);
    }
    bytes.extend_from_slice(&manifest.seed.to_be_bytes());
    push_optional_str(&mut bytes, manifest.official_dataset_digest.as_deref());
    push_optional_str(&mut bytes, manifest.official_harness_digest.as_deref());
    push_optional_str(&mut bytes, manifest.official_split_manifest_digest.as_deref());
    Ok(bytes)
}

/// Compute the exact matched-control digest for a collaborative-memory run.
pub fn compute_collab_memory_control_digest(
    manifest: &CollabMemoryManifest,
) -> Result<String, String> {
    Ok(sha256_hex(&encode_manifest(manifest)?))
}

fn encode_probe(bytes: &mut Vec<u8>, probe: &CollabMemoryProbe) -> Result<(), String> {
    if !nonempty(&probe.probe_id) || !valid_digest(&probe.evidence_digest) {
        return Err("probe identity or evidence digest is invalid".to_owned());
    }
    push_str(bytes, &probe.probe_id);
    bytes.push(probe.evidence.competency().tag());
    bytes.push(u8::from(probe.unauthorized_content_exposure));
    bytes.push(u8::from(probe.forbidden_metadata_inference));

    match &probe.evidence {
        CollabMemoryEvidence::Sharing {
            observer_complete,
            expected_recipients,
            observed_recipients,
        } => {
            bytes.push(u8::from(*observer_complete));
            push_set(bytes, expected_recipients);
            push_set(bytes, observed_recipients);
        }
        CollabMemoryEvidence::Access {
            observer_complete,
            principal_id,
            resource_scope_id,
            purpose_id,
            expected_allow,
            observed_allow,
        } => {
            if !nonempty(principal_id) || !nonempty(resource_scope_id) || !nonempty(purpose_id) {
                return Err("access probe principal, scope and purpose must be non-empty".to_owned());
            }
            bytes.push(u8::from(*observer_complete));
            push_str(bytes, principal_id);
            push_str(bytes, resource_scope_id);
            push_str(bytes, purpose_id);
            bytes.push(u8::from(*expected_allow));
            bytes.push(u8::from(*observed_allow));
        }
        CollabMemoryEvidence::Routing {
            observer_complete,
            expected_role_id,
            observed_role_ids,
        } => {
            if !nonempty(expected_role_id) || observed_role_ids.iter().any(|id| !nonempty(id)) {
                return Err("routing role identities must be non-empty".to_owned());
            }
            bytes.push(u8::from(*observer_complete));
            push_str(bytes, expected_role_id);
            push_set(bytes, observed_role_ids);
        }
        CollabMemoryEvidence::Provenance {
            observer_complete,
            expected_source_event_digest,
            observed_source_event_digest,
            expected_writer_id,
            observed_writer_id,
            expected_authority_digest,
            observed_authority_digest,
        } => {
            for digest in [
                expected_source_event_digest,
                observed_source_event_digest,
                expected_authority_digest,
                observed_authority_digest,
            ] {
                if !valid_digest(digest) {
                    return Err("provenance source or authority digest is malformed".to_owned());
                }
            }
            if !nonempty(expected_writer_id) || !nonempty(observed_writer_id) {
                return Err("provenance writer identity must be non-empty".to_owned());
            }
            bytes.push(u8::from(*observer_complete));
            push_str(bytes, expected_source_event_digest);
            push_str(bytes, observed_source_event_digest);
            push_str(bytes, expected_writer_id);
            push_str(bytes, observed_writer_id);
            push_str(bytes, expected_authority_digest);
            push_str(bytes, observed_authority_digest);
        }
        CollabMemoryEvidence::ConflictResolution {
            observer_complete,
            expected_current_value_digest,
            observed_current_value_digest,
            expected_authority_rank,
            observed_authority_rank,
        } => {
            if !valid_digest(expected_current_value_digest)
                || !valid_digest(observed_current_value_digest)
            {
                return Err("conflict-resolution current-value digest is malformed".to_owned());
            }
            bytes.push(u8::from(*observer_complete));
            push_str(bytes, expected_current_value_digest);
            push_str(bytes, observed_current_value_digest);
            bytes.extend_from_slice(&expected_authority_rank.to_be_bytes());
            bytes.extend_from_slice(&observed_authority_rank.to_be_bytes());
        }
        CollabMemoryEvidence::Retention {
            observer_complete,
            expected_retained,
            observed_retained,
            stale_memory_used,
            post_revocation_used,
            derived_descendant_used_after_revocation,
        } => {
            bytes.push(u8::from(*observer_complete));
            bytes.push(u8::from(*expected_retained));
            bytes.push(u8::from(*observed_retained));
            bytes.push(u8::from(*stale_memory_used));
            bytes.push(u8::from(*post_revocation_used));
            bytes.push(u8::from(*derived_descendant_used_after_revocation));
        }
    }
    push_str(bytes, &probe.evidence_digest);
    Ok(())
}

/// Compute the domain-separated run digest over matched controls and all probes.
pub fn compute_collab_memory_run_digest(run: &CollabMemoryRun) -> Result<String, String> {
    if !nonempty(&run.run_id) {
        return Err("collaborative-memory run ID must be non-empty".to_owned());
    }
    let controls = encode_manifest(&run.manifest)?;
    let mut bytes = RUN_DOMAIN.to_vec();
    push_str(&mut bytes, &run.run_id);
    push_len(&mut bytes, controls.len());
    bytes.extend_from_slice(&controls);
    push_len(&mut bytes, run.probes.len());
    for probe in &run.probes {
        encode_probe(&mut bytes, probe)?;
    }
    Ok(sha256_hex(&bytes))
}

fn hard_failure(report: &mut QualificationReport, reason: String) {
    report.fail(reason);
}

fn add_reason(report: &mut QualificationReport, reason: impl Into<String>) {
    report.reasons.push(reason.into());
}

fn compare_probe(
    probe: &CollabMemoryProbe,
    report: &mut QualificationReport,
    counts: &mut CollabMemoryQualificationReport,
) -> bool {
    if !probe.evidence.observer_complete() {
        add_reason(
            report,
            format!("{} observer evidence is incomplete", probe.probe_id),
        );
        return false;
    }

    if probe.unauthorized_content_exposure {
        counts.unauthorized_content_exposures += 1;
        hard_failure(
            report,
            format!("{} exposed content to an unauthorized principal", probe.probe_id),
        );
    }
    if probe.forbidden_metadata_inference {
        counts.forbidden_metadata_inferences += 1;
        hard_failure(
            report,
            format!("{} permitted a forbidden metadata inference", probe.probe_id),
        );
    }

    let passed = match &probe.evidence {
        CollabMemoryEvidence::Sharing {
            expected_recipients,
            observed_recipients,
            ..
        } => {
            let unauthorized = observed_recipients
                .difference(expected_recipients)
                .count() as u64;
            if unauthorized > 0 {
                counts.unauthorized_role_routes += unauthorized;
                hard_failure(
                    report,
                    format!("{} shared memory outside the authorized recipient set", probe.probe_id),
                );
            }
            expected_recipients == observed_recipients
        }
        CollabMemoryEvidence::Access {
            principal_id,
            resource_scope_id,
            purpose_id,
            expected_allow,
            observed_allow,
            ..
        } => {
            if !expected_allow && *observed_allow {
                counts.unauthorized_access_grants += 1;
                hard_failure(
                    report,
                    format!(
                        "{} granted {principal_id} access to scope {resource_scope_id} for purpose {purpose_id}",
                        probe.probe_id
                    ),
                );
            }
            expected_allow == observed_allow
        }
        CollabMemoryEvidence::Routing {
            expected_role_id,
            observed_role_ids,
            ..
        } => {
            let unexpected = observed_role_ids
                .iter()
                .filter(|role| *role != expected_role_id)
                .count() as u64;
            if unexpected > 0 {
                counts.unauthorized_role_routes += unexpected;
                hard_failure(
                    report,
                    format!("{} routed memory to an unintended role", probe.probe_id),
                );
            }
            observed_role_ids.len() == 1 && observed_role_ids.contains(expected_role_id)
        }
        CollabMemoryEvidence::Provenance {
            expected_source_event_digest,
            observed_source_event_digest,
            expected_writer_id,
            observed_writer_id,
            expected_authority_digest,
            observed_authority_digest,
            ..
        } => {
            let matches = expected_source_event_digest == observed_source_event_digest
                && expected_writer_id == observed_writer_id
                && expected_authority_digest == observed_authority_digest;
            if !matches {
                counts.provenance_mismatches += 1;
                hard_failure(
                    report,
                    format!("{} provenance does not bind the expected source, writer and authority", probe.probe_id),
                );
            }
            matches
        }
        CollabMemoryEvidence::ConflictResolution {
            expected_current_value_digest,
            observed_current_value_digest,
            expected_authority_rank,
            observed_authority_rank,
            ..
        } => {
            let matches = expected_current_value_digest == observed_current_value_digest
                && expected_authority_rank == observed_authority_rank;
            if !matches {
                counts.authority_conflicts += 1;
                counts.stale_memory_uses += 1;
                hard_failure(
                    report,
                    format!("{} selected a stale or lower-authority current value", probe.probe_id),
                );
            }
            matches
        }
        CollabMemoryEvidence::Retention {
            expected_retained,
            observed_retained,
            stale_memory_used,
            post_revocation_used,
            derived_descendant_used_after_revocation,
            ..
        } => {
            if *stale_memory_used {
                counts.stale_memory_uses += 1;
                hard_failure(
                    report,
                    format!("{} used a stale memory", probe.probe_id),
                );
            }
            if *post_revocation_used {
                counts.post_revocation_uses += 1;
                hard_failure(
                    report,
                    format!("{} consumed memory after revocation", probe.probe_id),
                );
            }
            if *derived_descendant_used_after_revocation {
                counts.derived_descendant_uses_after_revocation += 1;
                hard_failure(
                    report,
                    format!("{} consumed a derived memory after revocation", probe.probe_id),
                );
            }
            expected_retained == observed_retained
                && !stale_memory_used
                && !post_revocation_used
                && !derived_descendant_used_after_revocation
        }
    };

    if !passed {
        add_reason(
            report,
            format!("{} failed {} qualification", probe.probe_id, probe.evidence.competency().code()),
        );
    }
    passed
}

/// Qualify a matched collaborative-memory run; no weighted composite is emitted.
pub fn qualify_collab_memory_run(
    run: &CollabMemoryRun,
) -> CollabMemoryQualificationReport {
    let mut qualification = QualificationReport::new();
    let mut scores = BTreeMap::new();
    for competency in CollabMemoryCompetency::ALL {
        scores.insert(competency, CollabCompetencyScore::default());
    }

    let control_digest = match compute_collab_memory_control_digest(&run.manifest) {
        Ok(digest) => digest,
        Err(error) => {
            qualification.fail(error);
            String::new()
        }
    };
    if run.control_digest != control_digest {
        qualification.fail("collaborative-memory control digest does not match the manifest");
    }

    let mut result = CollabMemoryQualificationReport {
        qualification,
        control_digest,
        run_digest: run.run_digest.clone(),
        metric_evaluable: true,
        official_collabmem_compatible: false,
        scores,
        unauthorized_content_exposures: 0,
        forbidden_metadata_inferences: 0,
        unauthorized_access_grants: 0,
        unauthorized_role_routes: 0,
        provenance_mismatches: 0,
        authority_conflicts: 0,
        stale_memory_uses: 0,
        post_revocation_uses: 0,
        derived_descendant_uses_after_revocation: 0,
        reasons: Vec::new(),
    };

    match compute_collab_memory_run_digest(run) {
        Ok(expected) if expected == run.run_digest => {}
        Ok(_) => {
            result.qualification.fail("collaborative-memory run digest does not bind the supplied run");
            result.metric_evaluable = false;
        }
        Err(error) => {
            result.qualification.fail(format!("invalid collaborative-memory run: {error}"));
            result.metric_evaluable = false;
        }
    }

    if run.manifest.evidence_class == CollabMemoryEvidenceClass::SyntheticFixture {
        result.qualification.indeterminate(
            "synthetic collaborative-memory fixture is not empirical benchmark evidence",
        );
        result.metric_evaluable = false;
        result.reasons.push("synthetic fixture is contract-only".to_owned());
        return result;
    }

    if run.probes.is_empty() {
        result.qualification.indeterminate("collaborative-memory run contains no probes");
        result.metric_evaluable = false;
        return result;
    }

    let official_digests_present = [
        run.manifest.official_dataset_digest.as_deref(),
        run.manifest.official_harness_digest.as_deref(),
        run.manifest.official_split_manifest_digest.as_deref(),
    ]
    .into_iter()
    .all(|value| value.is_some_and(valid_digest));
    result.official_collabmem_compatible = official_digests_present;

    let mut probe_ids = BTreeSet::new();
    let mut seen_competencies = BTreeSet::new();
    let mut group_incomplete = BTreeSet::new();

    for probe in &run.probes {
        if !probe_ids.insert(probe.probe_id.as_str()) {
            result.qualification.fail("collaborative-memory probe IDs must be unique");
            result.metric_evaluable = false;
            continue;
        }

        let competency = probe.evidence.competency();
        seen_competencies.insert(competency);
        if !valid_digest(&probe.evidence_digest) {
            result.qualification.indeterminate(format!(
                "{} has no valid independent evidence digest",
                probe.probe_id
            ));
            result.metric_evaluable = false;
            group_incomplete.insert(competency);
            continue;
        }

        let score = result.scores.get_mut(&competency).expect("all competencies initialized");
        if !probe.evidence.observer_complete() {
            score.incomplete_probe_count += 1;
            group_incomplete.insert(competency);
            compare_probe(probe, &mut result.qualification, &mut result);
            continue;
        }

        score.observed_probe_count += 1;
        let passed = compare_probe(probe, &mut result.qualification, &mut result);
        if passed {
            score.passed_probe_count += 1;
        } else {
            score.failed_probe_count += 1;
        }
    }

    for competency in CollabMemoryCompetency::ALL {
        if !seen_competencies.contains(&competency) {
            result.qualification.indeterminate(format!(
                "no {} probes were supplied",
                competency.code()
            ));
            result.metric_evaluable = false;
        }
        if group_incomplete.contains(&competency) {
            result.metric_evaluable = false;
        }
        if let Some(score) = result.scores.get_mut(&competency) {
            if score.observed_probe_count > 0 && !group_incomplete.contains(&competency) {
                score.score_micros = Some(
                    ((score.passed_probe_count as u64 * SCORE_SCALE)
                        / score.observed_probe_count as u64) as u32,
                );
            }
        }
    }

    result.reasons.extend(result.qualification.reasons.iter().cloned());
    if !result.metric_evaluable && result.qualification.status != QualificationStatus::Fail {
        result.qualification.indeterminate(
            "collaborative-memory scorecard has missing or incomplete competency evidence",
        );
    }
    result
}

#[cfg(test)]
mod tests {
    use super::*;

    fn digest(seed: u8) -> String {
        format!("{seed:02x}").repeat(32)
    }

    fn manifest(evidence_class: CollabMemoryEvidenceClass) -> CollabMemoryManifest {
        CollabMemoryManifest {
            schema_version: "mcsf-collaborative-memory-v0.1".to_owned(),
            evidence_class,
            suite_id: "collab-memory-contract-pilot".to_owned(),
            dataset_manifest_digest: digest(1),
            trace_manifest_digest: digest(2),
            role_manifest_digest: digest(3),
            answerer_model_digest: digest(4),
            judge_digest: digest(5),
            gold_event_manifest_digest: digest(6),
            prompt_digest: digest(7),
            toolset_digest: digest(8),
            hardware_profile_digest: digest(9),
            budget_profile_digest: digest(10),
            decoding_config_digest: digest(11),
            memory_adapter_digest: digest(12),
            memory_policy_digest: digest(13),
            seed: 2026,
            official_dataset_digest: None,
            official_harness_digest: None,
            official_split_manifest_digest: None,
        }
    }

    fn probe(probe_id: &str, evidence: CollabMemoryEvidence) -> CollabMemoryProbe {
        CollabMemoryProbe {
            probe_id: probe_id.to_owned(),
            evidence,
            evidence_digest: digest(20),
            unauthorized_content_exposure: false,
            forbidden_metadata_inference: false,
        }
    }

    fn six_probes() -> Vec<CollabMemoryProbe> {
        vec![
            probe(
                "sharing-1",
                CollabMemoryEvidence::Sharing {
                    observer_complete: true,
                    expected_recipients: ["designer".to_owned(), "engineer".to_owned()].into(),
                    observed_recipients: ["designer".to_owned(), "engineer".to_owned()].into(),
                },
            ),
            probe(
                "access-1",
                CollabMemoryEvidence::Access {
                    observer_complete: true,
                    principal_id: "guest".to_owned(),
                    resource_scope_id: "project-private".to_owned(),
                    purpose_id: "read-technical-memory".to_owned(),
                    expected_allow: false,
                    observed_allow: false,
                },
            ),
            probe(
                "routing-1",
                CollabMemoryEvidence::Routing {
                    observer_complete: true,
                    expected_role_id: "engineer".to_owned(),
                    observed_role_ids: ["engineer".to_owned()].into(),
                },
            ),
            probe(
                "provenance-1",
                CollabMemoryEvidence::Provenance {
                    observer_complete: true,
                    expected_source_event_digest: digest(30),
                    observed_source_event_digest: digest(30),
                    expected_writer_id: "planner".to_owned(),
                    observed_writer_id: "planner".to_owned(),
                    expected_authority_digest: digest(31),
                    observed_authority_digest: digest(31),
                },
            ),
            probe(
                "conflict-1",
                CollabMemoryEvidence::ConflictResolution {
                    observer_complete: true,
                    expected_current_value_digest: digest(32),
                    observed_current_value_digest: digest(32),
                    expected_authority_rank: 4,
                    observed_authority_rank: 4,
                },
            ),
            probe(
                "retention-1",
                CollabMemoryEvidence::Retention {
                    observer_complete: true,
                    expected_retained: false,
                    observed_retained: false,
                    stale_memory_used: false,
                    post_revocation_used: false,
                    derived_descendant_used_after_revocation: false,
                },
            ),
        ]
    }

    fn run(
        evidence_class: CollabMemoryEvidenceClass,
        probes: Vec<CollabMemoryProbe>,
    ) -> CollabMemoryRun {
        let mut run = CollabMemoryRun {
            run_id: "collab-memory-run-001".to_owned(),
            manifest: manifest(evidence_class),
            probes,
            control_digest: String::new(),
            run_digest: String::new(),
        };
        run.control_digest = compute_collab_memory_control_digest(&run.manifest).unwrap();
        run.run_digest = compute_collab_memory_run_digest(&run).unwrap();
        run
    }

    #[test]
    fn six_complete_dimensions_pass_without_a_weighted_composite() {
        let result = qualify_collab_memory_run(&run(
            CollabMemoryEvidenceClass::ObservedRun,
            six_probes(),
        ));

        assert_eq!(result.qualification.status, QualificationStatus::Pass);
        assert!(result.metric_evaluable);
        assert!(result.scores.values().all(|score| score.score_micros == Some(1_000_000)));
        assert!(result.scores.values().all(|score| score.observed_probe_count == 1));
        assert!(!result.official_collabmem_compatible);
    }

    #[test]
    fn unauthorized_sharing_is_a_hard_failure() {
        let mut probes = six_probes();
        if let CollabMemoryEvidence::Sharing {
            observed_recipients, ..
        } = &mut probes[0].evidence
        {
            observed_recipients.insert("external-agent".to_owned());
        }
        let result = qualify_collab_memory_run(&run(CollabMemoryEvidenceClass::ObservedRun, probes));

        assert_eq!(result.qualification.status, QualificationStatus::Fail);
        assert_eq!(result.unauthorized_role_routes, 1);
        assert_eq!(result.scores[&CollabMemoryCompetency::Sharing].score_micros, Some(0));
    }

    #[test]
    fn unauthorized_access_grant_fails_closed() {
        let mut probes = six_probes();
        if let CollabMemoryEvidence::Access {
            observed_allow, ..
        } = &mut probes[1].evidence
        {
            *observed_allow = true;
        }
        let result = qualify_collab_memory_run(&run(CollabMemoryEvidenceClass::ObservedRun, probes));

        assert_eq!(result.qualification.status, QualificationStatus::Fail);
        assert_eq!(result.unauthorized_access_grants, 1);
    }

    #[test]
    fn routing_to_an_unintended_role_is_a_hard_failure() {
        let mut probes = six_probes();
        if let CollabMemoryEvidence::Routing {
            observed_role_ids, ..
        } = &mut probes[2].evidence
        {
            observed_role_ids.insert("auditor".to_owned());
        }
        let result = qualify_collab_memory_run(&run(CollabMemoryEvidenceClass::ObservedRun, probes));

        assert_eq!(result.qualification.status, QualificationStatus::Fail);
        assert_eq!(result.unauthorized_role_routes, 1);
    }

    #[test]
    fn forged_source_writer_or_authority_provenance_fails() {
        let mut probes = six_probes();
        if let CollabMemoryEvidence::Provenance {
            observed_writer_id, ..
        } = &mut probes[3].evidence
        {
            *observed_writer_id = "untrusted-importer".to_owned();
        }
        let result = qualify_collab_memory_run(&run(CollabMemoryEvidenceClass::ObservedRun, probes));

        assert_eq!(result.qualification.status, QualificationStatus::Fail);
        assert_eq!(result.provenance_mismatches, 1);
    }

    #[test]
    fn lower_authority_conflict_winner_is_a_hard_failure() {
        let mut probes = six_probes();
        if let CollabMemoryEvidence::ConflictResolution {
            observed_authority_rank, ..
        } = &mut probes[4].evidence
        {
            *observed_authority_rank = 1;
        }
        let result = qualify_collab_memory_run(&run(CollabMemoryEvidenceClass::ObservedRun, probes));

        assert_eq!(result.qualification.status, QualificationStatus::Fail);
        assert_eq!(result.authority_conflicts, 1);
    }

    #[test]
    fn stale_or_revoked_derived_memory_use_fails() {
        let mut probes = six_probes();
        if let CollabMemoryEvidence::Retention {
            derived_descendant_used_after_revocation, ..
        } = &mut probes[5].evidence
        {
            *derived_descendant_used_after_revocation = true;
        }
        let result = qualify_collab_memory_run(&run(CollabMemoryEvidenceClass::ObservedRun, probes));

        assert_eq!(result.qualification.status, QualificationStatus::Fail);
        assert_eq!(result.derived_descendant_uses_after_revocation, 1);
    }

    #[test]
    fn global_content_and_metadata_leaks_override_otherwise_passing_evidence() {
        let mut probes = six_probes();
        probes[0].unauthorized_content_exposure = true;
        probes[1].forbidden_metadata_inference = true;
        let result = qualify_collab_memory_run(&run(CollabMemoryEvidenceClass::ObservedRun, probes));

        assert_eq!(result.qualification.status, QualificationStatus::Fail);
        assert_eq!(result.unauthorized_content_exposures, 1);
        assert_eq!(result.forbidden_metadata_inferences, 1);
    }

    #[test]
    fn incomplete_observer_is_indeterminate_not_a_pass() {
        let mut probes = six_probes();
        if let CollabMemoryEvidence::Sharing {
            observer_complete, ..
        } = &mut probes[0].evidence
        {
            *observer_complete = false;
        }
        let result = qualify_collab_memory_run(&run(CollabMemoryEvidenceClass::ObservedRun, probes));

        assert_eq!(
            result.qualification.status,
            QualificationStatus::Indeterminate
        );
        assert!(!result.metric_evaluable);
        assert_eq!(result.scores[&CollabMemoryCompetency::Sharing].score_micros, None);
    }

    #[test]
    fn synthetic_contract_fixture_never_qualifies_as_observed_result() {
        let result = qualify_collab_memory_run(&run(
            CollabMemoryEvidenceClass::SyntheticFixture,
            six_probes(),
        ));

        assert_eq!(
            result.qualification.status,
            QualificationStatus::Indeterminate
        );
        assert!(!result.metric_evaluable);
        assert!(!result.official_collabmem_compatible);
    }

    #[test]
    fn missing_competency_or_tampered_run_digest_blocks_complete_scorecard() {
        let mut probes = six_probes();
        probes.pop();
        let incomplete = qualify_collab_memory_run(&run(
            CollabMemoryEvidenceClass::ObservedRun,
            probes,
        ));
        assert_eq!(
            incomplete.qualification.status,
            QualificationStatus::Indeterminate
        );
        assert!(!incomplete.metric_evaluable);

        let mut altered = run(CollabMemoryEvidenceClass::ObservedRun, six_probes());
        altered.probes[0].unauthorized_content_exposure = true;
        let tampered = qualify_collab_memory_run(&altered);
        assert_eq!(tampered.qualification.status, QualificationStatus::Fail);
        assert!(!tampered.metric_evaluable);
    }

    #[test]
    fn official_compatibility_requires_all_three_release_digests() {
        let mut run = run(CollabMemoryEvidenceClass::ObservedRun, six_probes());
        run.manifest.official_dataset_digest = Some(digest(40));
        run.manifest.official_harness_digest = Some(digest(41));
        run.manifest.official_split_manifest_digest = Some(digest(42));
        run.control_digest = compute_collab_memory_control_digest(&run.manifest).unwrap();
        run.run_digest = compute_collab_memory_run_digest(&run).unwrap();

        let result = qualify_collab_memory_run(&run);
        assert_eq!(result.qualification.status, QualificationStatus::Pass);
        assert!(result.official_collabmem_compatible);
    }

    #[test]
    fn changed_answerer_or_target_role_changes_the_control_digest() {
        let manifest = manifest(CollabMemoryEvidenceClass::ObservedRun);
        let original = compute_collab_memory_control_digest(&manifest).unwrap();
        let mut changed = manifest.clone();
        changed.answerer_model_digest = digest(44);
        assert_ne!(
            original,
            compute_collab_memory_control_digest(&changed).unwrap()
        );
        let mut changed = manifest;
        changed.role_manifest_digest = digest(45);
        assert_ne!(
            original,
            compute_collab_memory_control_digest(&changed).unwrap()
        );
    }
}
