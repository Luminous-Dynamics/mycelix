//! Independent, effects-disabled oracle for civil-awareness owner handoffs.
//!
//! This crate composes typed synthetic owner-result fixtures. It does not call,
//! reimplement or claim to qualify Mycelix, Sol-Atlas, Symtropy or Symthaea
//! production reducers. The separately run #5972 handoff verifier supplies a
//! structural manifest report; #5923 remains the sole dependency-graph owner.
//!
//! A REVIEWABLE result means only "reviewable within the declared evidence".
//! It is not an operational safety verdict, navigation clearance or action grant.

use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet};

pub const CLAIM_CEILING: &str = "Independent rule-table evaluation over synthetic civil-awareness owner-result fixtures and an empty handoff contract template only; no owner-native runtime outputs or physical sensor data are present.";

pub const CORPUS_JSON: &str = include_str!(
    "../../../docs/civil-awareness/integration/civil-awareness-cross-stack-golden-traces-v0.1.json"
);
pub const INPUTS_JSON: &str = include_str!(
    "../../../docs/civil-awareness/integration/civil-awareness-cross-stack-oracle-inputs-v0.1.json"
);
pub const HANDOFF_TEMPLATE_JSON: &str = include_str!(
    "../../../docs/civil-awareness/integration/civil-awareness-handoff-template-v0.1.json"
);

pub const MAX_INPUT_BYTES: usize = 8 * 1024 * 1024;

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Domain {
    Uas,
    Maritime,
    CivilAviation,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum EvidenceDisposition {
    ReviewableWithinDeclaredEvidence,
    ReviewWithCorrelatedSources,
    Unresolved,
    Conflict,
    Quarantined,
    Rejected,
    NotEvaluable,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum SourceIntegrity {
    Verified,
    Unverified,
    Failed,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum RightsStatus {
    Cleared,
    Pending,
    Denied,
    Unknown,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum AuthorizationStatus {
    Approved,
    Pending,
    Denied,
    Unknown,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum DependencyState {
    SharedDependency,
    UnknownDependency,
    NoDeclaredCommonDomain,
    NotEvaluated,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum ActionAuthority {
    None,
    Attempted,
    Granted,
    Unknown,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum ContextStatus {
    Current,
    Stale,
    Unknown,
    Conflict,
    Unsupported,
    NotApplicable,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum ScenarioReplayStatus {
    Passed,
    Failed,
    Unsupported,
    NotEvaluated,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum TriageStatus {
    Reviewable,
    HumanReview,
    Abstain,
    NotEvaluated,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum ContradictionStatus {
    None,
    Unresolved,
    Conflict,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum PrivacyState {
    Minimized,
    Quarantined,
    Violation,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum Freshness {
    Current,
    Stale,
    Unknown,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum ClockQuality {
    Qualified,
    Untrusted,
    Unknown,
    NotApplicable,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum CalibrationStatus {
    Current,
    Stale,
    Unknown,
    NotApplicable,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum EvidenceCompleteness {
    Complete,
    Incomplete,
    Unknown,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct MycelixOutputs {
    pub source_integrity: SourceIntegrity,
    pub rights_status: RightsStatus,
    pub authorization_status: AuthorizationStatus,
    pub dependency_state: DependencyState,
    pub duplicate_or_relayed: bool,
    pub action_authority: ActionAuthority,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct SolAtlasOutputs {
    pub context_status: ContextStatus,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct SymtropyOutputs {
    pub scenario_replay_status: ScenarioReplayStatus,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct SymthaeaOutputs {
    pub triage_status: TriageStatus,
    pub contradiction_status: ContradictionStatus,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct OwnerOutputs {
    pub mycelix: MycelixOutputs,
    pub sol_atlas: SolAtlasOutputs,
    pub symtropy: SymtropyOutputs,
    pub symthaea: SymthaeaOutputs,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct GateObservations {
    pub required_handoffs_complete: bool,
    pub schema_valid: bool,
    pub prohibited_fields_detected: bool,
    pub privacy_state: PrivacyState,
    pub source_freshness: Freshness,
    pub clock_quality: ClockQuality,
    pub calibration_status: CalibrationStatus,
    pub evidence_completeness: EvidenceCompleteness,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct OracleInputCase {
    pub case_id: String,
    pub owner_outputs: OwnerOutputs,
    pub gates: GateObservations,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct OracleInputs {
    pub corpus_id: String,
    pub profile_id: String,
    pub status: String,
    pub source_corpus_path: String,
    pub source_corpus_sha256: String,
    pub handoff_template_sha256: String,
    pub case_count: usize,
    pub claim_ceiling: String,
    pub cases: Vec<OracleInputCase>,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct GoldenCase {
    pub case_id: String,
    pub domain: Domain,
    pub scenario: String,
    pub control_kind: String,
    pub expected_disposition: EvidenceDisposition,
    pub invariant_ids: Vec<String>,
    pub hard_gates: Vec<String>,
    pub owner_artifact_requirements: Vec<ArtifactRequirement>,
    pub claim_ceiling: String,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct ArtifactRequirement {
    pub owner: String,
    pub artifact_kind: String,
    pub required: bool,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct OwnerRepositories {
    pub mycelix: String,
    pub sol_atlas: String,
    pub symtropy: String,
    pub symthaea: String,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct GoldenCorpus {
    pub corpus_id: String,
    pub profile_id: String,
    pub status: String,
    pub parent_issue: u32,
    pub scenario_count: usize,
    pub claim_ceiling: String,
    pub owner_repositories: OwnerRepositories,
    pub owner_artifact_requirements: Vec<ArtifactRequirement>,
    pub cases: Vec<GoldenCase>,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum HandoffDisposition {
    Pass,
    NotEvaluable,
    Unknown,
    Reject,
    Conflict,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct HandoffCheck {
    pub disposition: HandoffDisposition,
    pub code: String,
    pub detail: String,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct HandoffVerifierReport {
    pub manifest_id: String,
    pub profile_version: String,
    pub input_bytes: usize,
    pub input_sha256: String,
    pub artifact_count: usize,
    pub handoff_edge_count: usize,
    pub disposition: HandoffDisposition,
    pub checks: Vec<HandoffCheck>,
    pub claim_ceiling: String,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum ManifestStatus {
    ContractTemplate,
    SyntheticFixtureNotExecuted,
    ExecutedFrozen,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct HandoffExecution {
    pub executed: bool,
    pub exact_head_verified: bool,
    pub independent_verification: bool,
    pub provider_effects: bool,
    pub legal_review: bool,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct HandoffManifest {
    pub manifest_id: String,
    pub profile_version: String,
    pub status: ManifestStatus,
    pub use_case: String,
    pub audience: Vec<String>,
    pub privacy_policy_refs: Vec<String>,
    pub artifact_refs: Vec<serde_json::Value>,
    pub handoff_edges: Vec<serde_json::Value>,
    pub nonclaims: Vec<String>,
    pub execution: HandoffExecution,
}

#[derive(Clone, Debug, Serialize, Eq, PartialEq)]
pub struct CaseResult {
    pub case_id: String,
    pub domain: Domain,
    pub calculated: EvidenceDisposition,
    pub expected: EvidenceDisposition,
    pub reason_code: String,
    pub matches: bool,
}

#[derive(Clone, Debug, Serialize, Eq, PartialEq)]
pub struct OracleReport {
    pub corpus_id: String,
    pub corpus_sha256: String,
    pub oracle_inputs_sha256: String,
    pub handoff_manifest_sha256: String,
    pub handoff_verifier_disposition: HandoffDisposition,
    pub case_count: usize,
    pub pass_count: usize,
    pub mismatch_count: usize,
    pub results: Vec<CaseResult>,
    pub disposition: String,
    pub claim_ceiling: String,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum OracleError {
    InputTooLarge,
    InvalidCorpus,
    InvalidOracleInputs,
    InvalidHandoffManifest,
    InvalidHandoffReport,
    CaseSetMismatch,
}

pub fn sha256_hex(bytes: &[u8]) -> String {
    let digest = Sha256::digest(bytes);
    let mut result = String::with_capacity(64);
    for byte in digest {
        use std::fmt::Write as _;
        let _ = write!(&mut result, "{byte:02x}");
    }
    result
}

pub fn derive_disposition(
    outputs: &OwnerOutputs,
    gates: &GateObservations,
) -> (EvidenceDisposition, &'static str) {
    if !gates.schema_valid || gates.prohibited_fields_detected {
        return (EvidenceDisposition::Rejected, "schema_or_prohibited_field_gate");
    }
    if gates.privacy_state == PrivacyState::Violation {
        return (EvidenceDisposition::Rejected, "privacy_violation");
    }
    if outputs.mycelix.source_integrity == SourceIntegrity::Failed {
        return (EvidenceDisposition::Rejected, "source_integrity_failed");
    }
    if outputs.mycelix.action_authority != ActionAuthority::None {
        return (EvidenceDisposition::Rejected, "action_authority_must_be_none");
    }

    if outputs.mycelix.rights_status != RightsStatus::Cleared
        || outputs.mycelix.authorization_status != AuthorizationStatus::Approved
        || gates.privacy_state == PrivacyState::Quarantined
    {
        return (EvidenceDisposition::Quarantined, "rights_authorization_or_privacy_gate");
    }

    if !gates.required_handoffs_complete {
        return (EvidenceDisposition::NotEvaluable, "required_owner_handoff_missing");
    }

    if outputs.symthaea.contradiction_status == ContradictionStatus::Conflict
        || outputs.sol_atlas.context_status == ContextStatus::Conflict
    {
        return (EvidenceDisposition::Conflict, "cross_owner_or_source_conflict");
    }

    if outputs.symthaea.contradiction_status == ContradictionStatus::Unresolved {
        return (EvidenceDisposition::Unresolved, "contradiction_unresolved");
    }
    if outputs.mycelix.source_integrity != SourceIntegrity::Verified {
        return (EvidenceDisposition::Unresolved, "source_integrity_unverified");
    }
    if gates.source_freshness != Freshness::Current {
        return (EvidenceDisposition::Unresolved, "source_freshness_not_current");
    }
    if !matches!(gates.clock_quality, ClockQuality::Qualified | ClockQuality::NotApplicable) {
        return (EvidenceDisposition::Unresolved, "clock_quality_not_qualified");
    }
    if !matches!(
        gates.calibration_status,
        CalibrationStatus::Current | CalibrationStatus::NotApplicable
    ) {
        return (EvidenceDisposition::Unresolved, "calibration_not_current");
    }
    if gates.evidence_completeness != EvidenceCompleteness::Complete {
        return (EvidenceDisposition::Unresolved, "evidence_completeness_not_established");
    }
    if !matches!(
        outputs.sol_atlas.context_status,
        ContextStatus::Current | ContextStatus::NotApplicable
    ) {
        return (EvidenceDisposition::Unresolved, "spatial_temporal_context_not_current");
    }
    if outputs.symtropy.scenario_replay_status != ScenarioReplayStatus::Passed {
        return (EvidenceDisposition::Unresolved, "synthetic_scenario_replay_not_passed");
    }
    if matches!(
        outputs.mycelix.dependency_state,
        DependencyState::UnknownDependency | DependencyState::NotEvaluated
    ) {
        return (EvidenceDisposition::Unresolved, "dependency_state_unknown_or_not_evaluated");
    }
    if outputs.symthaea.triage_status == TriageStatus::Abstain {
        return (EvidenceDisposition::Unresolved, "symthaea_abstained");
    }
    if outputs.symthaea.triage_status == TriageStatus::NotEvaluated {
        return (EvidenceDisposition::NotEvaluable, "symthaea_triage_not_evaluated");
    }
    if outputs.mycelix.duplicate_or_relayed
        || outputs.mycelix.dependency_state == DependencyState::SharedDependency
    {
        return (
            EvidenceDisposition::ReviewWithCorrelatedSources,
            "sources_correlated_or_duplicate_not_independent",
        );
    }
    if outputs.mycelix.dependency_state == DependencyState::NoDeclaredCommonDomain
        && matches!(
            outputs.symthaea.triage_status,
            TriageStatus::Reviewable | TriageStatus::HumanReview
        )
    {
        return (
            EvidenceDisposition::ReviewableWithinDeclaredEvidence,
            "reviewable_within_declared_evidence_not_physical_independence",
        );
    }

    (
        EvidenceDisposition::Unresolved,
        "evidence_does_not_meet_reviewability_contract",
    )
}

fn digest_prefixed(bytes: &[u8]) -> String {
    format!("sha256:{}", sha256_hex(bytes))
}

pub fn evaluate_bytes(
    corpus_bytes: &[u8],
    input_bytes: &[u8],
    manifest_bytes: &[u8],
    handoff_report_bytes: &[u8],
) -> Result<OracleReport, OracleError> {
    if [
        corpus_bytes,
        input_bytes,
        manifest_bytes,
        handoff_report_bytes,
    ]
    .iter()
    .any(|bytes| bytes.len() > MAX_INPUT_BYTES)
    {
        return Err(OracleError::InputTooLarge);
    }

    let corpus: GoldenCorpus =
        serde_json::from_slice(corpus_bytes).map_err(|_| OracleError::InvalidCorpus)?;
    let inputs: OracleInputs =
        serde_json::from_slice(input_bytes).map_err(|_| OracleError::InvalidOracleInputs)?;
    let manifest: HandoffManifest =
        serde_json::from_slice(manifest_bytes).map_err(|_| OracleError::InvalidHandoffManifest)?;
    let handoff_report: HandoffVerifierReport = serde_json::from_slice(handoff_report_bytes)
        .map_err(|_| OracleError::InvalidHandoffReport)?;

    if corpus.corpus_id != "GCPN-CIVIL-CROSS-STACK-0.1"
        || corpus.profile_id != "civil-awareness-cross-stack-golden-trace-0.1"
        || corpus.status != "declarative_synthetic_not_executed"
        || corpus.parent_issue != 5998
        || corpus.scenario_count != corpus.cases.len()
        || corpus.cases.is_empty()
        || corpus.claim_ceiling.trim().is_empty()
    {
        return Err(OracleError::InvalidCorpus);
    }
    if corpus.owner_repositories.mycelix != "Luminous-Dynamics/mycelix"
        || corpus.owner_repositories.sol_atlas != "Luminous-Dynamics/sol-atlas-leptos"
        || corpus.owner_repositories.symtropy != "Luminous-Dynamics/symtropy"
        || corpus.owner_repositories.symthaea != "Luminous-Dynamics/symthaea"
    {
        return Err(OracleError::InvalidCorpus);
    }
    if inputs.corpus_id != corpus.corpus_id
        || inputs.profile_id != corpus.profile_id
        || inputs.status != "synthetic_owner_result_fixtures_not_executed"
        || inputs.source_corpus_path
            != "mycelix-workspace/docs/civil-awareness/integration/civil-awareness-cross-stack-golden-traces-v0.1.json"
        || inputs.source_corpus_sha256 != digest_prefixed(corpus_bytes)
        || inputs.handoff_template_sha256 != digest_prefixed(manifest_bytes)
        || inputs.case_count != inputs.cases.len()
        || inputs.case_count != corpus.cases.len()
        || inputs.claim_ceiling.trim().is_empty()
    {
        return Err(OracleError::InvalidOracleInputs);
    }
    if manifest.manifest_id != "GCPN-CIVIL-CROSS-STACK-CONTRACT-TEMPLATE-001"
        || manifest.profile_version != "0.1.0"
        || manifest.status != ManifestStatus::ContractTemplate
        || !manifest.artifact_refs.is_empty()
        || !manifest.handoff_edges.is_empty()
        || manifest.audience.is_empty()
        || manifest.privacy_policy_refs.is_empty()
        || manifest.nonclaims.is_empty()
        || manifest.execution.executed
        || manifest.execution.exact_head_verified
        || manifest.execution.independent_verification
        || manifest.execution.provider_effects
        || manifest.execution.legal_review
    {
        return Err(OracleError::InvalidHandoffManifest);
    }
    if handoff_report.disposition != HandoffDisposition::Pass
        || handoff_report.manifest_id != manifest.manifest_id
        || handoff_report.profile_version != manifest.profile_version
        || handoff_report.input_bytes != manifest_bytes.len()
        || handoff_report.input_sha256 != sha256_hex(manifest_bytes)
        || handoff_report.artifact_count != 0
        || handoff_report.handoff_edge_count != 0
        || handoff_report.checks.iter().any(|check| check.disposition != HandoffDisposition::Pass)
    {
        return Err(OracleError::InvalidHandoffReport);
    }

    let mut golden_by_id = BTreeMap::new();
    for golden in &corpus.cases {
        if golden_by_id.insert(golden.case_id.as_str(), golden).is_some() {
            return Err(OracleError::InvalidCorpus);
        }
    }
    let mut inputs_by_id = BTreeMap::new();
    for case in &inputs.cases {
        if inputs_by_id.insert(case.case_id.as_str(), case).is_some() {
            return Err(OracleError::InvalidOracleInputs);
        }
    }
    if golden_by_id.len() != inputs_by_id.len()
        || golden_by_id.keys().any(|case_id| !inputs_by_id.contains_key(case_id))
    {
        return Err(OracleError::CaseSetMismatch);
    }

    let mut results = Vec::with_capacity(corpus.cases.len());
    let mut pass_count = 0usize;
    for golden in &corpus.cases {
        let case = inputs_by_id
            .get(golden.case_id.as_str())
            .ok_or(OracleError::CaseSetMismatch)?;
        let calculated = derive_disposition(&case.owner_outputs, &case.gates);
        let matches = calculated.0 == golden.expected_disposition;
        if matches {
            pass_count += 1;
        }
        results.push(CaseResult {
            case_id: golden.case_id.clone(),
            domain: golden.domain,
            calculated: calculated.0,
            expected: golden.expected_disposition,
            reason_code: calculated.1.to_owned(),
            matches,
        });
    }
    let mismatch_count = results.len().saturating_sub(pass_count);
    Ok(OracleReport {
        corpus_id: corpus.corpus_id,
        corpus_sha256: sha256_hex(corpus_bytes),
        oracle_inputs_sha256: sha256_hex(input_bytes),
        handoff_manifest_sha256: sha256_hex(manifest_bytes),
        handoff_verifier_disposition: handoff_report.disposition,
        case_count: results.len(),
        pass_count,
        mismatch_count,
        results,
        disposition: if mismatch_count == 0 { "PASS".into() } else { "REJECT".into() },
        claim_ceiling: CLAIM_CEILING.into(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    fn fixtures() -> (Vec<u8>, Vec<u8>, Vec<u8>, Vec<u8>) {
        let corpus = CORPUS_JSON.as_bytes().to_vec();
        let inputs = INPUTS_JSON.as_bytes().to_vec();
        let manifest = HANDOFF_TEMPLATE_JSON.as_bytes().to_vec();
        let manifest_obj: serde_json::Value = serde_json::from_slice(&manifest).unwrap();
        let report = HandoffVerifierReport {
            manifest_id: manifest_obj["manifest_id"].as_str().unwrap().to_owned(),
            profile_version: manifest_obj["profile_version"].as_str().unwrap().to_owned(),
            input_bytes: manifest.len(),
            input_sha256: sha256_hex(&manifest),
            artifact_count: 0,
            handoff_edge_count: 0,
            disposition: HandoffDisposition::Pass,
            checks: vec![HandoffCheck {
                disposition: HandoffDisposition::Pass,
                code: "synthetic_fixture".into(),
                detail: "test-only stub report; workflow uses the real #5972 verifier".into(),
            }],
            claim_ceiling: "test stub only".into(),
        };
        (corpus, inputs, manifest, serde_json::to_vec(&report).unwrap())
    }

    #[test]
    fn all_pinned_civil_golden_traces_match() {
        let (corpus, inputs, manifest, report) = fixtures();
        let result = evaluate_bytes(&corpus, &inputs, &manifest, &report).unwrap();
        assert_eq!(result.disposition, "PASS");
        assert_eq!(result.case_count, 22);
        assert_eq!(result.pass_count, 22);
        assert_eq!(result.mismatch_count, 0);
    }

    #[test]
    fn action_authority_promotion_cannot_pass() {
        let (corpus, inputs, manifest, report) = fixtures();
        let mut value: serde_json::Value = serde_json::from_slice(&inputs).unwrap();
        value["cases"][0]["owner_outputs"]["mycelix"]["action_authority"] = json!("attempted");
        let tampered_inputs = serde_json::to_vec(&value).unwrap();
        let result = evaluate_bytes(&corpus, &tampered_inputs, &manifest, &report).unwrap();
        assert_eq!(result.disposition, "REJECT");
        assert_eq!(result.results[0].calculated, EvidenceDisposition::Rejected);
    }

    #[test]
    fn corpus_digest_mismatch_is_rejected() {
        let (corpus, inputs, manifest, report) = fixtures();
        let mut value: serde_json::Value = serde_json::from_slice(&inputs).unwrap();
        value["source_corpus_sha256"] = json!("sha256:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa");
        let tampered_inputs = serde_json::to_vec(&value).unwrap();
        assert_eq!(
            evaluate_bytes(&corpus, &tampered_inputs, &manifest, &report),
            Err(OracleError::InvalidOracleInputs)
        );
    }

    #[test]
    fn non_pass_handoff_report_is_rejected() {
        let (corpus, inputs, manifest, report) = fixtures();
        let mut value: serde_json::Value = serde_json::from_slice(&report).unwrap();
        value["disposition"] = json!("unknown");
        let tampered_report = serde_json::to_vec(&value).unwrap();
        assert_eq!(
            evaluate_bytes(&corpus, &inputs, &manifest, &tampered_report),
            Err(OracleError::InvalidHandoffReport)
        );
    }

    #[test]
    fn duplicate_corpus_object_fields_are_rejected() {
        let (corpus, inputs, manifest, report) = fixtures();
        let text = String::from_utf8(corpus).unwrap();
        let duplicate = text.replacen(
            "\"corpus_id\": \"GCPN-CIVIL-CROSS-STACK-0.1\",",
            "\"corpus_id\": \"GCPN-CIVIL-CROSS-STACK-0.1\", \"corpus_id\": \"tampered\",",
            1,
        );
        assert_eq!(
            evaluate_bytes(duplicate.as_bytes(), &inputs, &manifest, &report),
            Err(OracleError::InvalidCorpus)
        );
    }

    #[test]
    fn unknown_corpus_fields_are_rejected() {
        let (corpus, inputs, manifest, report) = fixtures();
        let mut value: serde_json::Value = serde_json::from_slice(&corpus).unwrap();
        value["unexpected_field"] = json!(true);
        let changed = serde_json::to_vec(&value).unwrap();
        assert_eq!(
            evaluate_bytes(&changed, &inputs, &manifest, &report),
            Err(OracleError::InvalidCorpus)
        );
    }

    #[test]
    fn trailing_json_documents_are_rejected() {
        let (mut corpus, inputs, manifest, report) = fixtures();
        corpus.extend_from_slice(b" {}");
        assert_eq!(
            evaluate_bytes(&corpus, &inputs, &manifest, &report),
            Err(OracleError::InvalidCorpus)
        );
    }

    #[test]
    fn oversized_input_is_rejected_before_deserialization() {
        let bytes = vec![b' '; MAX_INPUT_BYTES + 1];
        assert_eq!(
            evaluate_bytes(&bytes, b"{}", b"{}", b"{}"),
            Err(OracleError::InputTooLarge)
        );
    }
}
