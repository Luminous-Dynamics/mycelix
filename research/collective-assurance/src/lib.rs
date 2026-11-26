//! Fail-closed qualification primitives for collective-capability experiments.
//! This crate validates signed AI-off claims and team-level allocation schedules.
//! It does not create a sandbox, measure a host, implement cryptography, or supply a CSPRNG.

use std::collections::{BTreeMap, BTreeSet};

mod allocation;
mod capability;
mod coordination;
mod aml_adapter;
mod mcsf;
mod comparison;
mod constraint_binding;
mod ledger;
mod response_audit;
mod support_policy;

pub use mcsf::{
    compute_mcsf_control_digest, qualify_mcsf_suite, McsfArchitecture,
    McsfArchitectureManifest, McsfArchitectureScorecard, McsfEvidenceReferences,
    McsfPolicyClass, McsfQualificationReport, McsfResourceTotals, McsfResourceUsage,
    McsfRun, McsfSuiteManifest, McsfTaskObservation, McsfTaskState, McsfTrack,
};
pub use aml_adapter::{
    compute_aml_persistence_receipt_digest, qualify_aml_run, AmlAddAttempt,
    AmlAddOperation, AmlComponentIdentity, AmlComponentKind, AmlContentPart,
    AmlDivision, AmlIngestionMode, AmlModelPolicy, AmlPersistenceReceipt,
    AmlQualificationReport, AmlReleaseContract, AmlRunEvidence, AmlRunManifest,
    AmlRunMode, AmlSearchCandidate, AmlSearchOperation, AmlTrack,
};
pub use coordination::{
    qualify_coordination_trial, CoordinationArchitecture, CoordinationDisposition,
    CoordinationQualificationReport, CoordinationRunEvidence, CoordinationStudyContract,
    CoordinationTaskFeatures, CoordinationTrial, PairedCoordinationGain,
};
pub use capability::{
    qualify_capability_outcomes, AccommodationState, CapabilityMeasurement,
    CapabilityOutcomeObservation, CapabilityStage, CapabilityStudyContract,
};
pub use allocation::{
    allocate_committed_teams, compute_design_digest, compute_roster_digest,
    create_allocation_commitment, verify_allocation_reveal, AllocationCommitment,
    AllocationSecret,
};
pub use ledger::{
    genesis_digest, validate_ledger, ClaimDisposition, ClaimReference, DecisionDisposition,
    DisclosureOutcome, DisclosurePermission, LedgerEvent, LedgerEventKind,
    LedgerValidationReport, VerificationOutcome,
};
pub use support_policy::{
    qualify_support_choice, SupportDecision, SupportMode, SupportQualificationReport,
    SupportQualificationStatus, SupportRequest, UserIntent,
};
pub use constraint_binding::{
    qualify_response_grounding, Applicability, AvailableEvidenceConstraint,
    ConstraintHandling, ConstraintQualificationReport, ConstraintQualificationStatus,
    EvidenceStatus, ResponseConstraintUse, ResponseGrounding,
};
pub use response_audit::{
    validate_independent_response_audit, AuditedClaimDisposition, AuditedEvidenceRecord,
    IndependentResponseAudit, MaterialClaimAnnotation, ResponseAuditReport, ResponseAuditStatus,
};
pub use comparison::{
    qualify_benchmark_comparison, BenchmarkRunManifest, ComparisonIntent,
    ComparisonReport, ComparisonStatus,
};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum QualificationStatus {
    Pass,
    Fail,
    Indeterminate,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct QualificationReport {
    pub status: QualificationStatus,
    pub reasons: Vec<String>,
}

impl QualificationReport {
    fn new() -> Self {
        Self {
            status: QualificationStatus::Pass,
            reasons: Vec::new(),
        }
    }
    fn fail(&mut self, reason: impl Into<String>) {
        self.status = QualificationStatus::Fail;
        self.reasons.push(reason.into());
    }
    fn indeterminate(&mut self, reason: impl Into<String>) {
        if self.status != QualificationStatus::Fail {
            self.status = QualificationStatus::Indeterminate;
        }
        self.reasons.push(reason.into());
    }
}

#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub enum ProcessClass {
    ApprovedHarness,
    CandidateAgent,
    ModelRuntime,
    Unclassified,
}

#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub enum ArtifactClass {
    TaskInput,
    HumanAuthored,
    CandidateOutput,
    AiGeneratedAnswer,
    ModelWeights,
    PromptOrSessionCache,
    Unclassified,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ArtifactObservation {
    pub artifact_id: String,
    pub class: ArtifactClass,
    pub candidate_reachable: bool,
}

#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub enum ProbeKind {
    InferenceEndpoint,
    AgentControlPlane,
    ExternalNetworkEgress,
    ModelCredentialVault,
    CacheReplayCanary,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ProbeOutcome {
    Blocked,
    Allowed,
    Unknown,
    NotRun,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ProbeObservation {
    pub kind: ProbeKind,
    pub outcome: ProbeOutcome,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum InventoryStatus {
    Complete,
    Incomplete,
    Unknown,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct AiOffClaims {
    pub schema_version: String,
    pub run_id: String,
    pub phase_id: String,
    pub challenge_nonce: String,
    pub signer_key_id: String,
    pub isolation_profile_id: String,
    pub platform_measurement_digest: String,
    pub runner_digest: String,
    pub isolation_policy_digest: String,
    pub process_inventory: InventoryStatus,
    pub processes: Vec<ProcessClass>,
    pub artifact_inventory: InventoryStatus,
    pub artifacts: Vec<ArtifactObservation>,
    pub probes: Vec<ProbeObservation>,
    pub observed_external_connections: Option<u64>,
}

impl AiOffClaims {
    /// Crate-local, length-prefixed signing encoding; not a general canonical JSON standard.
    pub fn signing_bytes(&self) -> Vec<u8> {
        let mut out = Vec::new();
        for value in [
            self.schema_version.as_str(),
            self.run_id.as_str(),
            self.phase_id.as_str(),
            self.challenge_nonce.as_str(),
            self.signer_key_id.as_str(),
            self.isolation_profile_id.as_str(),
            self.platform_measurement_digest.as_str(),
            self.runner_digest.as_str(),
            self.isolation_policy_digest.as_str(),
        ] {
            push_str(&mut out, value);
        }
        out.push(inventory_tag(self.process_inventory));
        push_len(&mut out, self.processes.len());
        for value in &self.processes {
            out.push(process_tag(*value));
        }
        out.push(inventory_tag(self.artifact_inventory));
        push_len(&mut out, self.artifacts.len());
        for artifact in &self.artifacts {
            push_str(&mut out, &artifact.artifact_id);
            out.push(artifact_tag(artifact.class));
            out.push(u8::from(artifact.candidate_reachable));
        }
        push_len(&mut out, self.probes.len());
        for probe in &self.probes {
            out.push(probe_tag(probe.kind));
            out.push(outcome_tag(probe.outcome));
        }
        match self.observed_external_connections {
            Some(count) => {
                out.push(1);
                out.extend_from_slice(&count.to_be_bytes());
            }
            None => out.push(0),
        }
        out
    }
}

fn push_len(out: &mut Vec<u8>, value: usize) {
    out.extend_from_slice(&(value as u64).to_be_bytes());
}
fn push_str(out: &mut Vec<u8>, value: &str) {
    push_len(out, value.len());
    out.extend_from_slice(value.as_bytes());
}
fn inventory_tag(v: InventoryStatus) -> u8 {
    match v {
        InventoryStatus::Complete => 1,
        InventoryStatus::Incomplete => 2,
        InventoryStatus::Unknown => 3,
    }
}
fn process_tag(v: ProcessClass) -> u8 {
    match v {
        ProcessClass::ApprovedHarness => 1,
        ProcessClass::CandidateAgent => 2,
        ProcessClass::ModelRuntime => 3,
        ProcessClass::Unclassified => 4,
    }
}
fn artifact_tag(v: ArtifactClass) -> u8 {
    match v {
        ArtifactClass::TaskInput => 1,
        ArtifactClass::HumanAuthored => 2,
        ArtifactClass::CandidateOutput => 3,
        ArtifactClass::AiGeneratedAnswer => 4,
        ArtifactClass::ModelWeights => 5,
        ArtifactClass::PromptOrSessionCache => 6,
        ArtifactClass::Unclassified => 7,
    }
}
fn probe_tag(v: ProbeKind) -> u8 {
    match v {
        ProbeKind::InferenceEndpoint => 1,
        ProbeKind::AgentControlPlane => 2,
        ProbeKind::ExternalNetworkEgress => 3,
        ProbeKind::ModelCredentialVault => 4,
        ProbeKind::CacheReplayCanary => 5,
    }
}
fn outcome_tag(v: ProbeOutcome) -> u8 {
    match v {
        ProbeOutcome::Blocked => 1,
        ProbeOutcome::Allowed => 2,
        ProbeOutcome::Unknown => 3,
        ProbeOutcome::NotRun => 4,
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ExpectedAiOffPhase {
    pub run_id: String,
    pub phase_id: String,
    pub challenge_nonce: String,
    pub trusted_signer_key_id: String,
    pub isolation_profile_id: String,
    pub platform_measurement_digest: String,
    pub runner_digest: String,
    pub isolation_policy_digest: String,
}

/// Production adapters must use a real signature algorithm and pinned trust roots.
pub trait ReceiptSignatureVerifier {
    fn verify(&self, key_id: &str, message: &[u8], signature: &[u8]) -> Result<bool, String>;
}

/// Production adapters must provide durable, atomic consume-once replay protection.
pub trait ReplayGuard {
    fn consume_once(&mut self, run: &str, phase: &str, nonce: &str) -> Result<bool, String>;
}

fn is_sha256(value: &str) -> bool {
    value.len() == 64 && value.bytes().all(|b| b.is_ascii_hexdigit())
}
fn require_match(report: &mut QualificationReport, label: &str, actual: &str, expected: &str) {
    if actual != expected {
        report.fail(format!("{label} differs from preregistered phase"));
    }
}

pub fn qualify_ai_off<V: ReceiptSignatureVerifier, R: ReplayGuard>(
    claims: &AiOffClaims,
    signature: &[u8],
    expected: &ExpectedAiOffPhase,
    verifier: &V,
    replay: &mut R,
) -> QualificationReport {
    let mut report = QualificationReport::new();
    for (label, value) in [
        ("expected run id", expected.run_id.as_str()),
        ("expected phase id", expected.phase_id.as_str()),
        (
            "expected signer key id",
            expected.trusted_signer_key_id.as_str(),
        ),
        (
            "expected isolation profile",
            expected.isolation_profile_id.as_str(),
        ),
    ] {
        if value.trim().is_empty() {
            report.fail(format!("{label} must not be empty"));
        }
    }
    if expected.challenge_nonce.len() < 32 {
        report.fail("expected challenge nonce must contain at least 32 characters");
    }
    require_match(
        &mut report,
        "schema version",
        &claims.schema_version,
        "ai-off-attestation-v0.1",
    );
    for (label, actual, expected_value) in [
        ("run id", claims.run_id.as_str(), expected.run_id.as_str()),
        (
            "phase id",
            claims.phase_id.as_str(),
            expected.phase_id.as_str(),
        ),
        (
            "challenge nonce",
            claims.challenge_nonce.as_str(),
            expected.challenge_nonce.as_str(),
        ),
        (
            "signer key id",
            claims.signer_key_id.as_str(),
            expected.trusted_signer_key_id.as_str(),
        ),
        (
            "isolation profile",
            claims.isolation_profile_id.as_str(),
            expected.isolation_profile_id.as_str(),
        ),
        (
            "platform measurement digest",
            claims.platform_measurement_digest.as_str(),
            expected.platform_measurement_digest.as_str(),
        ),
        (
            "runner digest",
            claims.runner_digest.as_str(),
            expected.runner_digest.as_str(),
        ),
        (
            "isolation policy digest",
            claims.isolation_policy_digest.as_str(),
            expected.isolation_policy_digest.as_str(),
        ),
    ] {
        require_match(&mut report, label, actual, expected_value);
    }

    for (label, value) in [
        (
            "platform measurement",
            claims.platform_measurement_digest.as_str(),
        ),
        ("runner", claims.runner_digest.as_str()),
        ("isolation policy", claims.isolation_policy_digest.as_str()),
    ] {
        if !is_sha256(value) {
            report.fail(format!("{label} digest is not 64 hex characters"));
        }
    }
    if signature.is_empty() {
        report.fail("receipt signature is empty");
    } else {
        match verifier.verify(&claims.signer_key_id, &claims.signing_bytes(), signature) {
            Ok(true) => {}
            Ok(false) => report.fail("receipt signature is invalid"),
            Err(error) => report.indeterminate(format!("signature verifier unavailable: {error}")),
        }
    }
    if report.status == QualificationStatus::Fail {
        return report;
    }

    match replay.consume_once(&claims.run_id, &claims.phase_id, &claims.challenge_nonce) {
        Ok(true) => {}
        Ok(false) => report.fail("challenge nonce was already consumed"),
        Err(error) => report.indeterminate(format!("durable replay guard unavailable: {error}")),
    }
    if !claims.processes.contains(&ProcessClass::ApprovedHarness) {
        report.indeterminate("approved qualification harness is absent from the process inventory");
    }
    match claims.process_inventory {
        InventoryStatus::Complete => {}
        InventoryStatus::Incomplete => report.indeterminate("process inventory incomplete"),
        InventoryStatus::Unknown => report.indeterminate("process inventory unknown"),
    }
    match claims.artifact_inventory {
        InventoryStatus::Complete => {}
        InventoryStatus::Incomplete => report.indeterminate("artifact inventory incomplete"),
        InventoryStatus::Unknown => report.indeterminate("artifact inventory unknown"),
    }
    for process in &claims.processes {
        match process {
            ProcessClass::ApprovedHarness => {}
            ProcessClass::CandidateAgent => report.fail("candidate agent process present"),
            ProcessClass::ModelRuntime => report.fail("model runtime process present"),
            ProcessClass::Unclassified => report.indeterminate("unclassified process present"),
        }
    }
    let mut artifact_ids = BTreeSet::new();
    for artifact in &claims.artifacts {
        if artifact.artifact_id.trim().is_empty()
            || !artifact_ids.insert(artifact.artifact_id.as_str())
        {
            report.fail("artifact identifiers must be non-empty and unique");
        }
        if artifact.candidate_reachable
            && matches!(
                artifact.class,
                ArtifactClass::AiGeneratedAnswer
                    | ArtifactClass::ModelWeights
                    | ArtifactClass::PromptOrSessionCache
            )
        {
            report.fail("candidate can reach AI answer, model weights, or prompt/session cache");
        }
        if artifact.class == ArtifactClass::Unclassified {
            report.indeterminate("unclassified artifact present");
        }
    }
    let mut probes = BTreeMap::new();
    for probe in &claims.probes {
        if probes.insert(probe.kind, probe.outcome).is_some() {
            report.fail("duplicate negative-control probe");
        }
    }
    for required in [
        ProbeKind::InferenceEndpoint,
        ProbeKind::AgentControlPlane,
        ProbeKind::ExternalNetworkEgress,
        ProbeKind::ModelCredentialVault,
        ProbeKind::CacheReplayCanary,
    ] {
        match probes.get(&required) {
            Some(ProbeOutcome::Blocked) => {}
            Some(ProbeOutcome::Allowed) => {
                report.fail(format!("negative control {required:?} allowed"))
            }
            Some(ProbeOutcome::Unknown) => {
                report.indeterminate(format!("negative control {required:?} unknown"))
            }
            Some(ProbeOutcome::NotRun) => {
                report.indeterminate(format!("negative control {required:?} not run"))
            }
            None => report.indeterminate(format!("negative control {required:?} missing")),
        }
    }
    match claims.observed_external_connections {
        Some(0) => {}
        Some(_) => report.fail("external network connections observed during AI-off phase"),
        None => report.indeterminate("external connection count unavailable"),
    }
    report
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct RandomizationUnit {
    pub unit_id: String,
    pub stratum: String,
}
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Assignment {
    pub unit_id: String,
    pub stratum: String,
    pub arm: String,
}

/// Supply an operating-system-backed cryptographically secure RNG in production.
pub trait RandomSource {
    fn next_u64(&mut self) -> u64;
}

fn uniform_below<R: RandomSource>(rng: &mut R, upper: usize) -> usize {
    assert!(upper > 0);
    let bound = upper as u64;
    let threshold = bound.wrapping_neg() % bound;
    loop {
        let value = rng.next_u64();
        if value >= threshold {
            return (value % bound) as usize;
        }
    }
}
fn shuffle<T, R: RandomSource>(items: &mut [T], rng: &mut R) {
    for index in (1..items.len()).rev() {
        let other = uniform_below(rng, index + 1);
        items.swap(index, other);
    }
}

/// Assigns whole teams within predeclared strata, one of every arm per complete block.
pub fn allocate_complete_blocks<R: RandomSource>(
    roster: &[RandomizationUnit],
    arms: &[String],
    rng: &mut R,
) -> Result<Vec<Assignment>, String> {
    if arms.len() < 2 {
        return Err("at least two arms required".into());
    }
    let mut unique_arms = BTreeSet::new();
    if arms
        .iter()
        .any(|arm| arm.trim().is_empty() || !unique_arms.insert(arm.as_str()))
    {
        return Err("arm names must be non-empty and unique".into());
    }
    let mut strata: BTreeMap<&str, Vec<&RandomizationUnit>> = BTreeMap::new();
    let mut ids = BTreeSet::new();
    for unit in roster {
        if unit.unit_id.trim().is_empty() || unit.stratum.trim().is_empty() {
            return Err("team id and stratum must be non-empty".into());
        }
        if !ids.insert(unit.unit_id.as_str()) {
            return Err("duplicate team id".into());
        }
        strata.entry(unit.stratum.as_str()).or_default().push(unit);
    }
    if roster.is_empty() {
        return Err("roster is empty".into());
    }
    let mut assignments = Vec::with_capacity(roster.len());
    for (stratum, mut teams) in strata {
        if teams.len() % arms.len() != 0 {
            return Err(format!("stratum {stratum} cannot form complete arm blocks"));
        }
        teams.sort_by(|left, right| left.unit_id.cmp(&right.unit_id));
        shuffle(&mut teams, rng);
        for block in teams.chunks_exact(arms.len()) {
            let mut block_arms = arms.to_vec();
            shuffle(&mut block_arms, rng);
            for (team, arm) in block.iter().zip(block_arms) {
                assignments.push(Assignment {
                    unit_id: team.unit_id.clone(),
                    stratum: stratum.to_owned(),
                    arm,
                });
            }
        }
    }
    assignments.sort_by(|a, b| a.unit_id.cmp(&b.unit_id));
    Ok(assignments)
}

/// Checks externally supplied assignments against the frozen roster and arm set.
pub fn validate_assignments(
    roster: &[RandomizationUnit],
    arms: &[String],
    assignments: &[Assignment],
) -> Result<(), String> {
    if arms.len() < 2 {
        return Err("at least two arms required".into());
    }
    let allowed: BTreeSet<&str> = arms.iter().map(String::as_str).collect();
    if allowed.len() != arms.len() || allowed.contains("") {
        return Err("arm names must be non-empty and unique".into());
    }
    let mut expected = BTreeMap::new();
    for unit in roster {
        if unit.unit_id.trim().is_empty()
            || unit.stratum.trim().is_empty()
            || expected
                .insert(unit.unit_id.as_str(), unit.stratum.as_str())
                .is_some()
        {
            return Err("roster contains empty or duplicate team identity".into());
        }
    }
    if expected.is_empty() || assignments.len() != expected.len() {
        return Err("assignment count differs from roster".into());
    }
    let mut seen = BTreeSet::new();
    let mut counts: BTreeMap<(&str, &str), usize> = BTreeMap::new();
    for assignment in assignments {
        let Some(stratum) = expected.get(assignment.unit_id.as_str()) else {
            return Err("assignment contains team absent from roster".into());
        };
        if !seen.insert(assignment.unit_id.as_str()) {
            return Err("team assigned more than once".into());
        }
        if assignment.stratum != *stratum {
            return Err("assignment stratum differs from roster".into());
        }
        if !allowed.contains(assignment.arm.as_str()) {
            return Err("unknown assignment arm".into());
        }
        *counts
            .entry((assignment.stratum.as_str(), assignment.arm.as_str()))
            .or_default() += 1;
    }
    let strata: BTreeSet<&str> = roster.iter().map(|unit| unit.stratum.as_str()).collect();
    for stratum in strata {
        let per_arm: Vec<usize> = arms
            .iter()
            .map(|arm| *counts.get(&(stratum, arm.as_str())).unwrap_or(&0))
            .collect();
        if per_arm.iter().any(|count| *count != per_arm[0]) {
            return Err("arm counts unbalanced within stratum".into());
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    struct TestVerifier;
    impl ReceiptSignatureVerifier for TestVerifier {
        fn verify(&self, key: &str, message: &[u8], signature: &[u8]) -> Result<bool, String> {
            Ok(key == "trusted-test-key" && !message.is_empty() && signature == b"test-only")
        }
    }
    #[derive(Default)]
    struct TestReplay(BTreeSet<String>);
    impl ReplayGuard for TestReplay {
        fn consume_once(&mut self, run: &str, phase: &str, nonce: &str) -> Result<bool, String> {
            Ok(self.0.insert(format!("{run}:{phase}:{nonce}")))
        }
    }
    #[derive(Default)]
    struct TestRng(u64);
    impl RandomSource for TestRng {
        fn next_u64(&mut self) -> u64 {
            self.0 = self.0.wrapping_add(0x9e37_79b9_7f4a_7c15);
            let mut v = self.0;
            v = (v ^ (v >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
            v = (v ^ (v >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
            v ^ (v >> 31)
        }
    }
    fn digest(c: char) -> String {
        std::iter::repeat_n(c, 64).collect()
    }
    fn claims() -> AiOffClaims {
        AiOffClaims {
            schema_version: "ai-off-attestation-v0.1".into(),
            run_id: "run-1".into(),
            phase_id: "ai-off-1".into(),
            challenge_nonce: "n".repeat(32),
            signer_key_id: "trusted-test-key".into(),
            isolation_profile_id: "microvm-v1".into(),
            platform_measurement_digest: digest('a'),
            runner_digest: digest('b'),
            isolation_policy_digest: digest('c'),
            process_inventory: InventoryStatus::Complete,
            processes: vec![ProcessClass::ApprovedHarness],
            artifact_inventory: InventoryStatus::Complete,
            artifacts: vec![ArtifactObservation {
                artifact_id: "task-1".into(),
                class: ArtifactClass::TaskInput,
                candidate_reachable: true,
            }],
            probes: vec![
                ProbeObservation {
                    kind: ProbeKind::InferenceEndpoint,
                    outcome: ProbeOutcome::Blocked,
                },
                ProbeObservation {
                    kind: ProbeKind::AgentControlPlane,
                    outcome: ProbeOutcome::Blocked,
                },
                ProbeObservation {
                    kind: ProbeKind::ExternalNetworkEgress,
                    outcome: ProbeOutcome::Blocked,
                },
                ProbeObservation {
                    kind: ProbeKind::ModelCredentialVault,
                    outcome: ProbeOutcome::Blocked,
                },
                ProbeObservation {
                    kind: ProbeKind::CacheReplayCanary,
                    outcome: ProbeOutcome::Blocked,
                },
            ],
            observed_external_connections: Some(0),
        }
    }
    fn expected() -> ExpectedAiOffPhase {
        ExpectedAiOffPhase {
            run_id: "run-1".into(),
            phase_id: "ai-off-1".into(),
            challenge_nonce: "n".repeat(32),
            trusted_signer_key_id: "trusted-test-key".into(),
            isolation_profile_id: "microvm-v1".into(),
            platform_measurement_digest: digest('a'),
            runner_digest: digest('b'),
            isolation_policy_digest: digest('c'),
        }
    }

    #[test]
    fn valid_receipt_passes_once_then_replay_fails() {
        let c = claims();
        let mut replay = TestReplay::default();
        assert_eq!(
            qualify_ai_off(&c, b"test-only", &expected(), &TestVerifier, &mut replay).status,
            QualificationStatus::Pass
        );
        assert_eq!(
            qualify_ai_off(&c, b"test-only", &expected(), &TestVerifier, &mut replay).status,
            QualificationStatus::Fail
        );
    }
    #[test]
    fn reachable_answer_cache_fails() {
        let mut c = claims();
        c.artifacts.push(ArtifactObservation {
            artifact_id: "cache".into(),
            class: ArtifactClass::PromptOrSessionCache,
            candidate_reachable: true,
        });
        let report = qualify_ai_off(
            &c,
            b"test-only",
            &expected(),
            &TestVerifier,
            &mut TestReplay::default(),
        );
        assert_eq!(report.status, QualificationStatus::Fail);
    }
    #[test]
    fn unknown_inventory_is_indeterminate() {
        let mut c = claims();
        c.process_inventory = InventoryStatus::Unknown;
        let report = qualify_ai_off(
            &c,
            b"test-only",
            &expected(),
            &TestVerifier,
            &mut TestReplay::default(),
        );
        assert_eq!(report.status, QualificationStatus::Indeterminate);
    }
    #[test]
    fn allowed_endpoint_fails() {
        let mut c = claims();
        c.probes[0].outcome = ProbeOutcome::Allowed;
        let report = qualify_ai_off(
            &c,
            b"test-only",
            &expected(),
            &TestVerifier,
            &mut TestReplay::default(),
        );
        assert_eq!(report.status, QualificationStatus::Fail);
    }
    #[test]
    fn bad_signature_fails() {
        let report = qualify_ai_off(
            &claims(),
            b"tampered",
            &expected(),
            &TestVerifier,
            &mut TestReplay::default(),
        );
        assert_eq!(report.status, QualificationStatus::Fail);
    }
    #[test]
    fn partial_allocation_block_is_rejected() {
        let roster: Vec<_> = (0..7)
            .map(|n| RandomizationUnit {
                unit_id: format!("t{n}"),
                stratum: "s".into(),
            })
            .collect();
        let arms = vec!["a".into(), "b".into(), "c".into(), "d".into()];
        assert!(allocate_complete_blocks(&roster, &arms, &mut TestRng::default()).is_err());
    }
    #[test]
    fn allocation_is_balanced_and_reproducible() {
        let roster: Vec<_> = (0..8)
            .map(|n| RandomizationUnit {
                unit_id: format!("t{n}"),
                stratum: "s".into(),
            })
            .collect();
        let arms = vec![
            "ai-first".into(),
            "independent-first".into(),
            "evidence-first".into(),
            "human-only".into(),
        ];
        let first = allocate_complete_blocks(&roster, &arms, &mut TestRng::default()).unwrap();
        let second = allocate_complete_blocks(&roster, &arms, &mut TestRng::default()).unwrap();
        assert_eq!(first, second);
        validate_assignments(&roster, &arms, &first).unwrap();
    }
    #[test]
    fn externally_supplied_imbalance_is_rejected() {
        let roster: Vec<_> = (0..8)
            .map(|n| RandomizationUnit {
                unit_id: format!("t{n}"),
                stratum: "s".into(),
            })
            .collect();
        let arms = vec!["a".into(), "b".into(), "c".into(), "d".into()];
        let assignments: Vec<_> = roster
            .iter()
            .enumerate()
            .map(|(i, u)| Assignment {
                unit_id: u.unit_id.clone(),
                stratum: u.stratum.clone(),
                arm: if i < 5 { "a" } else { "b" }.into(),
            })
            .collect();
        assert!(validate_assignments(&roster, &arms, &assignments).is_err());
    }
    #[test]
    fn duplicate_roster_team_is_rejected() {
        let roster = vec![
            RandomizationUnit {
                unit_id: "t1".into(),
                stratum: "s".into(),
            },
            RandomizationUnit {
                unit_id: "t1".into(),
                stratum: "s".into(),
            },
        ];
        assert!(
            allocate_complete_blocks(&roster, &["a".into(), "b".into()], &mut TestRng::default())
                .is_err()
        );
    }
    #[test]
    fn missing_negative_control_is_indeterminate() {
        let mut c = claims();
        c.probes.pop();
        let report = qualify_ai_off(
            &c,
            b"test-only",
            &expected(),
            &TestVerifier,
            &mut TestReplay::default(),
        );
        assert_eq!(report.status, QualificationStatus::Indeterminate);
    }

    #[test]
    fn observed_external_connection_fails() {
        let mut c = claims();
        c.observed_external_connections = Some(1);
        let report = qualify_ai_off(
            &c,
            b"test-only",
            &expected(),
            &TestVerifier,
            &mut TestReplay::default(),
        );
        assert_eq!(report.status, QualificationStatus::Fail);
    }

    #[test]
    fn candidate_agent_process_fails() {
        let mut c = claims();
        c.processes.push(ProcessClass::CandidateAgent);
        let report = qualify_ai_off(
            &c,
            b"test-only",
            &expected(),
            &TestVerifier,
            &mut TestReplay::default(),
        );
        assert_eq!(report.status, QualificationStatus::Fail);
    }

    #[test]
    fn missing_harness_is_indeterminate() {
        let mut c = claims();
        c.processes.clear();
        let report = qualify_ai_off(
            &c,
            b"test-only",
            &expected(),
            &TestVerifier,
            &mut TestReplay::default(),
        );
        assert_eq!(report.status, QualificationStatus::Indeterminate);
    }

    #[test]
    fn one_thousand_fault_injections_never_false_pass() {
        for index in 0..1_000 {
            let mut c = claims();
            let mut e = expected();
            let mut signature = b"test-only".to_vec();
            match index % 12 {
                0 => c.processes.push(ProcessClass::ModelRuntime),
                1 => c.artifacts.push(ArtifactObservation {
                    artifact_id: format!("answer-{index}"),
                    class: ArtifactClass::AiGeneratedAnswer,
                    candidate_reachable: true,
                }),
                2 => c.probes[0].outcome = ProbeOutcome::Allowed,
                3 => {
                    c.probes.pop();
                }
                4 => c.observed_external_connections = Some(1),
                5 => c.process_inventory = InventoryStatus::Unknown,
                6 => c.artifact_inventory = InventoryStatus::Incomplete,
                7 => c.runner_digest.replace_range(0..1, "f"),
                8 => signature = b"invalid".to_vec(),
                9 => c.phase_id.clear(),
                10 => {
                    let probe = c.probes[0].clone();
                    c.probes.push(probe);
                }
                11 => e.phase_id.clear(),
                _ => unreachable!(),
            }
            let report = qualify_ai_off(
                &c,
                &signature,
                &e,
                &TestVerifier,
                &mut TestReplay::default(),
            );
            assert_ne!(
                report.status,
                QualificationStatus::Pass,
                "fault injection {index} unexpectedly passed: {:?}",
                report.reasons
            );
        }
    }

    #[test]
    fn multi_stratum_allocation_is_balanced_independently() {
        let roster: Vec<_> = ["north", "south"]
            .into_iter()
            .flat_map(|stratum| {
                (0..8).map(move |n| RandomizationUnit {
                    unit_id: format!("{stratum}-{n}"),
                    stratum: stratum.into(),
                })
            })
            .collect();
        let arms = vec![
            "ai-first".into(),
            "independent-first".into(),
            "evidence-first".into(),
            "human-only".into(),
        ];
        let assignments =
            allocate_complete_blocks(&roster, &arms, &mut TestRng::default()).unwrap();
        validate_assignments(&roster, &arms, &assignments).unwrap();
    }
}
