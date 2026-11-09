//! Independent, effects-disabled reference oracle for Maritime Commons golden traces.
//!
//! This crate composes declared synthetic owner-result fixtures. It does not call or
//! reimplement Sol-Atlas, Symtropy, Symthaea, provider, or Mycelix production reducers.
//! The handoff-template report is consumed from the separate #5972 verifier CLI.

use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::BTreeSet;
use std::fmt::Write as _;

pub const CLAIM_CEILING: &str =
    "Cross-owner disposition rules over synthetic owner-result fixtures only; no owner implementation outputs or field evidence are present.";

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum SourceDisposition { Qualified, Stale, Unknown, Unsupported, Conflict, NotApplicable }
#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum FeasibilityDisposition { Feasible, Infeasible, Unknown, Unsupported, Conflict, NotEvaluated }
#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum DecisionDisposition { Recommend, Abstain, RequestInformation, HumanReview, NotEvaluated }
#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum CommitmentDisposition { Present, Absent, Pending, NeedsReconciliation, Conflict, NotEvaluated }
#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum DependencyDisposition { SharedDependency, UnknownDependency, NoDeclaredCommonDomain, NotEvaluated }
#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum ProviderEffectEvidence { None, ReceiptPresent, ProviderStateUnavailable, NotApplicable }
#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum ModelActionAuthority { None, Attempted, Granted, Unknown }
#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum StayDisposition { Confirmed, Rejected, Pending, Unknown, Conflict, NeedsReconciliation, NotEvaluated, Incomparable }

#[derive(Clone, Debug, Deserialize, Eq, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct OwnerOutputs {
    pub source_context: SourceDisposition,
    pub feasibility: FeasibilityDisposition,
    pub decision: DecisionDisposition,
    pub commitment: CommitmentDisposition,
    pub dependency: DependencyDisposition,
    pub provider_effect_evidence: ProviderEffectEvidence,
    pub model_action_authority: ModelActionAuthority,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct GateObservations {
    pub required_handoffs_complete: bool,
    pub hard_gate_failure: bool,
    pub provider_commitment_revoked: bool,
    pub cross_owner_conflict: bool,
    pub privacy_violation: bool,
    pub offline_partition: bool,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct OracleInputCase {
    pub case_id: String,
    pub owner_outputs: OwnerOutputs,
    pub gate_observations: GateObservations,
    pub expected_stay_disposition: StayDisposition,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct OracleInputs {
    pub corpus_id: String,
    pub profile_id: String,
    pub status: String,
    pub source_corpus_path: String,
    pub source_corpus_git_blob_sha1: String,
    pub case_count: usize,
    pub claim_ceiling: String,
    pub cases: Vec<OracleInputCase>,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct CorpusExpected {
    pub source_context: SourceDisposition,
    pub feasibility: FeasibilityDisposition,
    pub decision: DecisionDisposition,
    pub commitment: CommitmentDisposition,
    pub dependency: DependencyDisposition,
    pub stay_disposition: StayDisposition,
    pub provider_effect_evidence: ProviderEffectEvidence,
    pub model_action_authority: String,
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
pub struct GoldenCase {
    pub case_id: String,
    pub family: String,
    pub control_kind: String,
    pub scenario: String,
    pub owner_artifact_requirements: Vec<ArtifactRequirement>,
    pub expected: CorpusExpected,
    pub invariant_ids: Vec<String>,
    pub hard_gates: Vec<String>,
    pub claim_ceiling: String,
    #[serde(default)]
    pub metamorphic_group: Option<String>,
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
pub struct CorpusExecutionStatus {
    pub executed: bool,
    pub owner_artifacts_pinned: bool,
    pub independent_oracle_built: bool,
    pub mutation_campaign: bool,
    pub cross_coordinator_replay: bool,
    pub provider_sandbox_used: bool,
    pub field_evaluation: bool,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct GoldenCorpus {
    pub corpus_id: String,
    pub profile_id: String,
    pub status: String,
    pub parent_issue: u32,
    pub claim_ceiling: String,
    pub scenario_count: usize,
    pub owner_repositories: OwnerRepositories,
    pub cases: Vec<GoldenCase>,
    pub execution_status: CorpusExecutionStatus,
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
    pub disposition: String,
    pub claim_ceiling: String,
}

#[derive(Clone, Debug, Serialize, Eq, PartialEq)]
pub struct CaseResult {
    pub case_id: String,
    pub calculated: StayDisposition,
    pub expected: StayDisposition,
    pub matches: bool,
}

#[derive(Clone, Debug, Serialize, Eq, PartialEq)]
pub struct OracleReport {
    pub corpus_id: String,
    pub corpus_sha256: String,
    pub oracle_inputs_sha256: String,
    pub handoff_template_sha256: String,
    pub handoff_verifier_disposition: String,
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
    InvalidHandoffReport,
    CaseSetMismatch,
}

fn digest(bytes: &[u8]) -> String {
    let mut out = String::with_capacity(64);
    for b in Sha256::digest(bytes) { let _ = write!(&mut out, "{b:02x}"); }
    out
}

/// Independent cross-owner rule table. This deliberately does not inspect free-text
/// scenario descriptions: only typed owner outputs and explicit gate observations.
pub fn derive_stay_disposition(outputs: &OwnerOutputs, gates: &GateObservations) -> StayDisposition {
    if !gates.required_handoffs_complete {
        return StayDisposition::Incomparable;
    }
    if gates.cross_owner_conflict
        || outputs.commitment == CommitmentDisposition::Conflict
        || outputs.source_context == SourceDisposition::Conflict
        || outputs.feasibility == FeasibilityDisposition::Conflict
    {
        return StayDisposition::Conflict;
    }
    if outputs.commitment == CommitmentDisposition::NeedsReconciliation
        && outputs.provider_effect_evidence == ProviderEffectEvidence::ProviderStateUnavailable
    {
        return StayDisposition::NeedsReconciliation;
    }
    if outputs.model_action_authority != ModelActionAuthority::None
        || gates.hard_gate_failure
        || gates.provider_commitment_revoked
        || gates.privacy_violation
        || outputs.feasibility == FeasibilityDisposition::Infeasible
    {
        return StayDisposition::Rejected;
    }
    // Store-and-forward requests can remain pending during a partition, but only as
    // pending requests. This branch never permits an offline confirmation.
    if gates.offline_partition && outputs.commitment == CommitmentDisposition::Pending {
        return StayDisposition::Pending;
    }
    if outputs.source_context != SourceDisposition::Qualified {
        return StayDisposition::Unknown;
    }
    if outputs.dependency == DependencyDisposition::UnknownDependency {
        return StayDisposition::Unknown;
    }
    if outputs.feasibility != FeasibilityDisposition::Feasible {
        return StayDisposition::Unknown;
    }
    if matches!(outputs.decision, DecisionDisposition::Abstain
        | DecisionDisposition::RequestInformation
        | DecisionDisposition::NotEvaluated)
    {
        return StayDisposition::Unknown;
    }
    if outputs.dependency == DependencyDisposition::SharedDependency {
        return if outputs.commitment == CommitmentDisposition::Pending
            && outputs.decision == DecisionDisposition::HumanReview
        { StayDisposition::Pending } else { StayDisposition::Unknown };
    }
    if outputs.commitment == CommitmentDisposition::Present
        && outputs.decision == DecisionDisposition::Recommend
        && outputs.provider_effect_evidence == ProviderEffectEvidence::ReceiptPresent
        && outputs.dependency != DependencyDisposition::UnknownDependency
    {
        return StayDisposition::Confirmed;
    }
    if outputs.commitment == CommitmentDisposition::Absent
        && outputs.decision == DecisionDisposition::Recommend
    {
        return StayDisposition::Pending;
    }
    if outputs.commitment == CommitmentDisposition::Pending
        && outputs.decision == DecisionDisposition::HumanReview
    {
        return StayDisposition::Pending;
    }
    StayDisposition::Unknown
}

pub fn evaluate_bytes(
    corpus_bytes: &[u8],
    input_bytes: &[u8],
    template_bytes: &[u8],
    handoff_report_bytes: &[u8],
) -> Result<OracleReport, OracleError> {
    const LIMIT: usize = 16 * 1024 * 1024;
    if [corpus_bytes, input_bytes, template_bytes, handoff_report_bytes].iter().any(|b| b.len() > LIMIT) {
        return Err(OracleError::InputTooLarge);
    }
    let corpus: GoldenCorpus = serde_json::from_slice(corpus_bytes).map_err(|_| OracleError::InvalidCorpus)?;
    let inputs: OracleInputs = serde_json::from_slice(input_bytes).map_err(|_| OracleError::InvalidOracleInputs)?;
    let handoff: HandoffVerifierReport = serde_json::from_slice(handoff_report_bytes).map_err(|_| OracleError::InvalidHandoffReport)?;

    if corpus.corpus_id != "MCB-CROSS-STACK-0.1"
        || corpus.profile_id != "maritime-cross-stack-golden-trace-0.1"
        || corpus.status != "declarative_synthetic_not_executed"
        || corpus.parent_issue != 5974
        || corpus.scenario_count != corpus.cases.len()
        || corpus.execution_status.executed
        || corpus.execution_status.owner_artifacts_pinned
        || corpus.execution_status.independent_oracle_built
        || corpus.execution_status.mutation_campaign
        || corpus.execution_status.cross_coordinator_replay
        || corpus.execution_status.provider_sandbox_used
        || corpus.execution_status.field_evaluation
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
        || inputs.case_count != inputs.cases.len()
        || inputs.case_count != corpus.cases.len()
        || inputs.source_corpus_path != "mycelix-workspace/docs/maritime-stay/integration/cross-stack-golden-traces-v0.1.json"
        || inputs.claim_ceiling.trim().is_empty()
        || !inputs.source_corpus_git_blob_sha1.chars().all(|c| c.is_ascii_hexdigit() && !c.is_ascii_uppercase())
        || inputs.source_corpus_git_blob_sha1.len() != 40
    {
        return Err(OracleError::InvalidOracleInputs);
    }
    if handoff.disposition != "pass"
        || handoff.input_sha256 != digest(template_bytes)
        || handoff.manifest_id != "MARITIME-COMMONS-SYNTHETIC-COASTAL-STAY-TEMPLATE-001"
    {
        return Err(OracleError::InvalidHandoffReport);
    }

    let corpus_by_id: std::collections::BTreeMap<&str, &GoldenCase> =
        corpus.cases.iter().map(|c| (c.case_id.as_str(), c)).collect();
    if corpus_by_id.len() != corpus.cases.len() { return Err(OracleError::InvalidCorpus); }
    let mut seen = BTreeSet::new();
    let mut results = Vec::with_capacity(inputs.cases.len());
    let mut pass_count = 0usize;

    for fixture in &inputs.cases {
        if !seen.insert(fixture.case_id.as_str()) { return Err(OracleError::InvalidOracleInputs); }
        let Some(golden) = corpus_by_id.get(fixture.case_id.as_str()) else { return Err(OracleError::CaseSetMismatch); };
        let expected_owner = &golden.expected;
        if fixture.owner_outputs.source_context != expected_owner.source_context
            || fixture.owner_outputs.feasibility != expected_owner.feasibility
            || fixture.owner_outputs.decision != expected_owner.decision
            || fixture.owner_outputs.commitment != expected_owner.commitment
            || fixture.owner_outputs.dependency != expected_owner.dependency
            || fixture.owner_outputs.provider_effect_evidence != expected_owner.provider_effect_evidence
            || expected_owner.model_action_authority != "none"
            || fixture.owner_outputs.model_action_authority != ModelActionAuthority::None
            || fixture.expected_stay_disposition != expected_owner.stay_disposition
        {
            return Err(OracleError::InvalidOracleInputs);
        }
        let calculated = derive_stay_disposition(&fixture.owner_outputs, &fixture.gate_observations);
        let matches = calculated == golden.expected.stay_disposition;
        if matches { pass_count += 1; }
        results.push(CaseResult {
            case_id: fixture.case_id.clone(),
            calculated,
            expected: golden.expected.stay_disposition,
            matches,
        });
    }
    if seen.len() != corpus_by_id.len() { return Err(OracleError::CaseSetMismatch); }

    let mismatch_count = results.len().saturating_sub(pass_count);
    Ok(OracleReport {
        corpus_id: corpus.corpus_id,
        corpus_sha256: digest(corpus_bytes),
        oracle_inputs_sha256: digest(input_bytes),
        handoff_template_sha256: digest(template_bytes),
        handoff_verifier_disposition: handoff.disposition,
        case_count: results.len(),
        pass_count,
        mismatch_count,
        results,
        disposition: if mismatch_count == 0 { "pass".into() } else { "reject".into() },
        claim_ceiling: CLAIM_CEILING.into(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    const CORPUS: &str = include_str!("../../../docs/maritime-stay/integration/cross-stack-golden-traces-v0.1.json");
    const INPUTS: &str = include_str!("../../../docs/maritime-stay/integration/cross-stack-oracle-inputs-v0.1.json");
    const TEMPLATE: &str = include_str!("../../maritime-handoff-verifier/fixtures/contract-template-v0.1.json");

    fn report_for_template() -> String {
        let digest = digest(TEMPLATE.as_bytes());
        json!({
            "manifest_id":"MARITIME-COMMONS-SYNTHETIC-COASTAL-STAY-TEMPLATE-001",
            "profile_version":"0.1.0",
            "input_bytes":TEMPLATE.len(),
            "input_sha256":digest,
            "artifact_count":0,
            "handoff_edge_count":0,
            "disposition":"pass",
            "claim_ceiling":"manifest structure only"
        }).to_string()
    }

    #[test]
    fn all_frozen_golden_cases_match_independent_reference_rule_table() {
        let report = evaluate_bytes(CORPUS.as_bytes(), INPUTS.as_bytes(), TEMPLATE.as_bytes(), report_for_template().as_bytes()).unwrap();
        assert_eq!(report.case_count, 35);
        assert_eq!(report.pass_count, 35);
        assert_eq!(report.mismatch_count, 0);
        assert_eq!(report.disposition, "pass");
    }

    #[test]
    fn positive_trace_requires_provider_commitment_and_effect_receipt() {
        let mut value: OracleInputs = serde_json::from_str(INPUTS).unwrap();
        let c = value.cases.iter_mut().find(|c| c.case_id == "MCB-X-001").unwrap();
        c.owner_outputs.commitment = CommitmentDisposition::Absent;
        assert_eq!(derive_stay_disposition(&c.owner_outputs, &c.gate_observations), StayDisposition::Pending);
    }

    #[test]
    fn feasibility_without_provider_commitment_is_pending_not_confirmed() {
        let mut value: OracleInputs = serde_json::from_str(INPUTS).unwrap();
        let c = value.cases.iter_mut().find(|c| c.case_id == "MCB-X-009").unwrap();
        assert_eq!(derive_stay_disposition(&c.owner_outputs, &c.gate_observations), StayDisposition::Pending);
    }

    #[test]
    fn stale_source_never_becomes_confirmed() {
        let mut value: OracleInputs = serde_json::from_str(INPUTS).unwrap();
        let c = value.cases.iter_mut().find(|c| c.case_id == "MCB-X-001").unwrap();
        c.owner_outputs.source_context = SourceDisposition::Stale;
        assert_eq!(derive_stay_disposition(&c.owner_outputs, &c.gate_observations), StayDisposition::Unknown);
    }

    #[test]
    fn model_output_never_creates_action_authority() {
        let mut value: OracleInputs = serde_json::from_str(INPUTS).unwrap();
        let c = value.cases.iter_mut().find(|c| c.case_id == "MCB-X-001").unwrap();
        c.owner_outputs.model_action_authority = ModelActionAuthority::Granted;
        assert_eq!(derive_stay_disposition(&c.owner_outputs, &c.gate_observations), StayDisposition::Rejected);
    }

    #[test]
    fn hard_gate_failure_is_non_compensatory() {
        let mut value: OracleInputs = serde_json::from_str(INPUTS).unwrap();
        let c = value.cases.iter_mut().find(|c| c.case_id == "MCB-X-001").unwrap();
        c.gate_observations.hard_gate_failure = true;
        assert_eq!(derive_stay_disposition(&c.owner_outputs, &c.gate_observations), StayDisposition::Rejected);
    }

    #[test]
    fn conflict_is_not_last_write_wins() {
        let mut value: OracleInputs = serde_json::from_str(INPUTS).unwrap();
        let c = value.cases.iter_mut().find(|c| c.case_id == "MCB-X-001").unwrap();
        c.gate_observations.cross_owner_conflict = true;
        assert_eq!(derive_stay_disposition(&c.owner_outputs, &c.gate_observations), StayDisposition::Conflict);
    }

    #[test]
    fn provider_effect_loss_requires_reconciliation() {
        let mut value: OracleInputs = serde_json::from_str(INPUTS).unwrap();
        let c = value.cases.iter_mut().find(|c| c.case_id == "MCB-X-027").unwrap();
        assert_eq!(derive_stay_disposition(&c.owner_outputs, &c.gate_observations), StayDisposition::NeedsReconciliation);
    }

    #[test]
    fn incomplete_owner_handoff_is_incomparable_not_rejected_or_passed() {
        let mut value: OracleInputs = serde_json::from_str(INPUTS).unwrap();
        let c = value.cases.iter_mut().find(|c| c.case_id == "MCB-X-001").unwrap();
        c.gate_observations.required_handoffs_complete = false;
        assert_eq!(derive_stay_disposition(&c.owner_outputs, &c.gate_observations), StayDisposition::Incomparable);
    }

    #[test]
    fn offline_partition_can_only_remain_pending() {
        let mut value: OracleInputs = serde_json::from_str(INPUTS).unwrap();
        let c = value.cases.iter_mut().find(|c| c.case_id == "MCB-X-032").unwrap();
        assert_eq!(derive_stay_disposition(&c.owner_outputs, &c.gate_observations), StayDisposition::Pending);
        c.owner_outputs.commitment = CommitmentDisposition::Present;
        c.owner_outputs.provider_effect_evidence = ProviderEffectEvidence::ReceiptPresent;
        c.owner_outputs.source_context = SourceDisposition::Qualified;
        c.owner_outputs.feasibility = FeasibilityDisposition::Feasible;
        c.owner_outputs.decision = DecisionDisposition::Recommend;
        assert_eq!(derive_stay_disposition(&c.owner_outputs, &c.gate_observations), StayDisposition::Pending);
    }

    #[test]
    fn duplicate_case_id_is_rejected() {
        let mut value: serde_json::Value = serde_json::from_str(INPUTS).unwrap();
        let first = value["cases"][0]["case_id"].clone();
        value["cases"][34]["case_id"] = first;
        let bytes = serde_json::to_vec(&value).unwrap();
        let report = report_for_template();
        assert_eq!(
            evaluate_bytes(CORPUS.as_bytes(), &bytes, TEMPLATE.as_bytes(), report.as_bytes()),
            Err(OracleError::InvalidOracleInputs)
        );
    }

    #[test]
    fn handoff_report_must_bind_exact_template_bytes() {
        let mut value: serde_json::Value = serde_json::from_str(&report_for_template()).unwrap();
        value["input_sha256"] = json!("sha256:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa");
        let report = serde_json::to_vec(&value).unwrap();
        assert_eq!(
            evaluate_bytes(CORPUS.as_bytes(), INPUTS.as_bytes(), TEMPLATE.as_bytes(), &report),
            Err(OracleError::InvalidHandoffReport)
        );
    }

    #[test]
    fn owner_result_fixture_cannot_drift_from_frozen_corpus() {
        let mut value: serde_json::Value = serde_json::from_str(INPUTS).unwrap();
        value["cases"][0]["owner_outputs"]["commitment"] = json!("absent");
        let bytes = serde_json::to_vec(&value).unwrap();
        let report = report_for_template();
        assert_eq!(
            evaluate_bytes(CORPUS.as_bytes(), &bytes, TEMPLATE.as_bytes(), report.as_bytes()),
            Err(OracleError::InvalidOracleInputs)
        );
    }

    #[test]
    fn report_is_deterministic_for_identical_input_bytes() {
        let report1 = evaluate_bytes(CORPUS.as_bytes(), INPUTS.as_bytes(), TEMPLATE.as_bytes(), report_for_template().as_bytes()).unwrap();
        let report2 = evaluate_bytes(CORPUS.as_bytes(), INPUTS.as_bytes(), TEMPLATE.as_bytes(), report_for_template().as_bytes()).unwrap();
        assert_eq!(report1, report2);
    }
}
