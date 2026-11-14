//! LARP v2 institutional contract reference evaluator.
//!
//! This crate is a deterministic, owner-input reference model. It is not a
//! production authorizer, policy engine, authority registry, legal interpreter,
//! provider client, treasury, or execution system. It has no I/O or mutation
//! capabilities in its public evaluation API.
//!
//! Time is supplied by the caller. Unknown facts never silently become true.

use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Disposition {
    Permit,
    Deny,
    Hold,
    Indeterminate,
    ChallengeRequired,
}

impl Disposition {
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Permit => "permit",
            Self::Deny => "deny",
            Self::Hold => "hold",
            Self::Indeterminate => "indeterminate",
            Self::ChallengeRequired => "challenge_required",
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Decision {
    pub disposition: Disposition,
    pub reason_codes: Vec<String>,
}

impl Decision {
    fn new(disposition: Disposition, reason: &'static str) -> Self {
        Self {
            disposition,
            reason_codes: vec![reason.to_owned()],
        }
    }

    fn permit(reason: &'static str) -> Self {
        Self::new(Disposition::Permit, reason)
    }

    fn deny(reason: &'static str) -> Self {
        Self::new(Disposition::Deny, reason)
    }

    fn hold(reason: &'static str) -> Self {
        Self::new(Disposition::Hold, reason)
    }

    fn indeterminate(reason: &'static str) -> Self {
        Self::new(Disposition::Indeterminate, reason)
    }

    fn challenge(reason: &'static str) -> Self {
        Self::new(Disposition::ChallengeRequired, reason)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum EvaluationMode {
    ProtectedAction,
    ContractActivation,
    Ratification,
    EvidenceAdmission,
    HistoricalReconstruction,
    Exit,
    Remedy,
    StagedReward,
    SafeContinuity,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ExecutionOutcome {
    NotStarted,
    Unknown,
    PartiallyApplied,
    Applied,
    ConfirmedNoEffect,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AuthorityFacts {
    pub local_authorized: Option<bool>,
    pub peer_credential_only: Option<bool>,
    pub transitive_authority: Option<bool>,
    pub constitution_current: Option<bool>,
    pub local_ratification: Option<bool>,
    pub predecessor_digest_matches: Option<bool>,
    pub ai_self_ratification: Option<bool>,
    pub old_authority_stale: Option<bool>,
    pub restore_old_privileges: Option<bool>,
    pub recovery_quorum_met: Option<bool>,
    pub partner_credential_valid: Option<bool>,
    pub treasury_separately_authorized: Option<bool>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ScopeFacts {
    pub purpose_allowed: Option<bool>,
    pub exact_operation_match: Option<bool>,
    pub delegation_attenuates: Option<bool>,
    pub payout_bounded: Option<bool>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RightsFacts {
    pub rights_satisfied: Option<bool>,
    pub remedy_path_defined: Option<bool>,
    pub high_impact: Option<bool>,
    pub exit_valid: Option<bool>,
    pub challenge_upheld: Option<bool>,
    pub reviewer_independent: Option<bool>,
    pub remedy_authorized: Option<bool>,
    pub history_preserved: Option<bool>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct EvidenceFacts {
    pub sufficient: Option<bool>,
    pub independent: Option<bool>,
    pub inventory_complete: Option<bool>,
    pub known_shared_dependency: Option<bool>,
    pub historically_valid_at_time: Option<bool>,
    pub fresh_for_current_time: Option<bool>,
    pub outcome_independently_observed: Option<bool>,
    pub similarity_only_signal: Option<bool>,
    pub score_only_sanction: Option<bool>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct InstitutionFacts {
    pub capture_risk: Option<bool>,
    pub oversight_independent: Option<bool>,
    pub known_route_bypass: Option<bool>,
    pub degraded_service_allowlisted: Option<bool>,
    pub high_impact_paused: Option<bool>,
    pub high_impact_requested: Option<bool>,
    pub old_sessions_fenced: Option<bool>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ExecutionFacts {
    pub outcome: ExecutionOutcome,
    pub retry_requested: Option<bool>,
    pub retry_safety_qualified: Option<bool>,
    pub single_use_consumed: Option<bool>,
    pub duplicate_requested: Option<bool>,
    pub contract_expires_at_unix: Option<i64>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct EvaluationFacts {
    pub mode: EvaluationMode,
    pub evaluated_at_unix: i64,
    pub authority: AuthorityFacts,
    pub scope: ScopeFacts,
    pub rights: RightsFacts,
    pub evidence: EvidenceFacts,
    pub institution: InstitutionFacts,
    pub execution: ExecutionFacts,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Scenario {
    pub id: String,
    pub family: String,
    pub kind: String,
    pub title: String,
    pub given: Vec<String>,
    pub request: String,
    pub facts: EvaluationFacts,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ScenarioSuite {
    pub schema_version: String,
    pub suite_id: String,
    pub status: String,
    pub contract_version: String,
    pub case_count: usize,
    pub fixture_rules: Vec<String>,
    pub fact_schema_version: String,
    pub cases: Vec<Scenario>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct OracleRow {
    pub case_id: String,
    pub expected_disposition: String,
    pub reason_codes: Vec<String>,
    pub notes: String,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct OracleSuite {
    pub schema_version: String,
    pub suite_id: String,
    pub status: String,
    pub scenario_file: String,
    pub disposition_vocabulary: Vec<String>,
    pub case_count: usize,
    pub rows: Vec<OracleRow>,
}

/// Evaluate owner-resolved facts without I/O, side effects, or implicit time.
///
/// A returned Permit only means that this finite reference relation found no
/// mismatch in the supplied facts. It does not authorize a real protected
/// effect. Owner-native execution must re-check current authority and state.
pub fn evaluate(facts: &EvaluationFacts, now_unix: i64) -> Decision {
    // Known hard violations take precedence over incomplete or stale evidence.
    if facts.authority.peer_credential_only == Some(true) {
        return Decision::deny("PEER_CREDENTIAL_IS_NOT_LOCAL_AUTHORITY");
    }
    if facts.authority.transitive_authority == Some(true) {
        return Decision::deny("AUTHORITY_IS_NOT_TRANSITIVE");
    }
    if facts.authority.ai_self_ratification == Some(true) {
        return Decision::deny("AI_CANNOT_SELF_RATIFY");
    }
    if facts.authority.old_authority_stale == Some(true)
        && facts.authority.restore_old_privileges == Some(true)
    {
        return Decision::deny("STALE_AUTHORITY_MUST_NOT_BE_RESTORED");
    }
    if facts.scope.purpose_allowed == Some(false) {
        return Decision::deny("PURPOSE_CREEP_PROHIBITED");
    }
    if facts.scope.exact_operation_match == Some(false) {
        return Decision::deny("EXACT_OPERATION_MISMATCH");
    }
    if facts.scope.delegation_attenuates == Some(false) {
        return Decision::deny("DELEGATION_SCOPE_AMPLIFICATION");
    }
    if facts.institution.known_route_bypass == Some(true) {
        return Decision::deny("KNOWN_ENFORCEMENT_BYPASS");
    }
    if facts.rights.high_impact == Some(true)
        && facts.rights.remedy_path_defined == Some(false)
    {
        return Decision::deny("HIGH_IMPACT_CONTRACT_REQUIRES_REMEDY_PATH");
    }
    if facts.evidence.score_only_sanction == Some(true) {
        return Decision::deny("MODEL_SCORE_ALONE_CANNOT_JUSTIFY_SANCTION");
    }
    if facts.execution.single_use_consumed == Some(true)
        && facts.execution.duplicate_requested == Some(true)
    {
        return Decision::deny("CONSUMED_SINGLE_USE_AUTHORITY_CANNOT_REPLAY");
    }
    if facts.institution.high_impact_requested == Some(true)
        && facts.mode == EvaluationMode::SafeContinuity
        && facts.authority.recovery_quorum_met == Some(false)
    {
        return Decision::deny("RECOVERY_QUORUM_REQUIRED_FOR_HIGH_IMPACT_ACTION");
    }

    // Capture indicators trigger a contestable process, not an automatic verdict.
    if facts.institution.capture_risk == Some(true)
        && facts.institution.oversight_independent == Some(false)
    {
        return Decision::challenge("EFFECTIVE_CONTROL_REQUIRES_INDEPENDENT_CHALLENGE");
    }

    // Historical reconstruction uses the historical validity of the evidence;
    // current freshness must not rewrite what was known at the earlier time.
    if facts.mode == EvaluationMode::HistoricalReconstruction {
        if facts.evidence.historically_valid_at_time == Some(false) {
            return Decision::deny("EVIDENCE_WAS_NOT_VALID_AT_RECONSTRUCTION_TIME");
        }
        if facts.evidence.historically_valid_at_time.is_none()
            || facts.rights.history_preserved != Some(true)
        {
            return Decision::indeterminate("HISTORICAL_EVIDENCE_OR_HISTORY_INCOMPLETE");
        }
        return Decision::permit("HISTORICAL_VALIDITY_PRESERVED_SEPARATE_FROM_CURRENT_FRESHNESS");
    }

    // Constitutional staleness is held for local successor-epoch review.
    if facts.authority.constitution_current == Some(false) {
        return Decision::hold("CONSTITUTIONAL_EPOCH_REAUTHORIZATION_REQUIRED");
    }

    // Partial and unknown effects are never silently collapsed into success/failure.
    match facts.execution.outcome {
        ExecutionOutcome::Unknown => {
            return Decision::hold("EXTERNAL_OUTCOME_UNKNOWN_RECONCILIATION_REQUIRED");
        }
        ExecutionOutcome::PartiallyApplied => {
            return Decision::hold("PARTIAL_EFFECT_REQUIRES_RECONCILIATION");
        }
        ExecutionOutcome::Applied | ExecutionOutcome::NotStarted | ExecutionOutcome::ConfirmedNoEffect => {}
    }
    if facts.execution.retry_requested == Some(true)
        && facts.execution.outcome == ExecutionOutcome::ConfirmedNoEffect
        && facts.execution.retry_safety_qualified != Some(true)
    {
        return Decision::hold("RETRY_SAFETY_NOT_QUALIFIED");
    }
    if facts.execution.contract_expires_at_unix.is_some_and(|expiry| expiry <= now_unix)
        && facts.execution.outcome != ExecutionOutcome::NotStarted
    {
        return Decision::hold("EXPIRED_CONTRACT_WITH_UNRECONCILED_EFFECT");
    }

    // Known overlap or an incomplete inventory defeats an independence claim.
    if facts.evidence.inventory_complete == Some(false) {
        return Decision::indeterminate("DEPENDENCY_INVENTORY_INCOMPLETE");
    }
    if facts.evidence.known_shared_dependency == Some(true)
        || facts.evidence.independent == Some(false)
    {
        return Decision::indeterminate("EVIDENCE_INDEPENDENCE_NOT_ESTABLISHED");
    }

    // Stale evidence may be historically valid but is held for a current decision.
    if facts.evidence.fresh_for_current_time == Some(false) {
        return Decision::hold("EVIDENCE_STALE_FOR_CURRENT_DECISION");
    }

    match facts.mode {
        EvaluationMode::ProtectedAction | EvaluationMode::ContractActivation => {
            require_true(facts.authority.local_authorized, "LOCAL_AUTHORITY_UNKNOWN", "LOCAL_AUTHORITY_DENIED")?;
            require_true(facts.authority.constitution_current, "CONSTITUTIONAL_EPOCH_UNKNOWN", "CONSTITUTIONAL_EPOCH_STALE")?;
            require_true(facts.scope.purpose_allowed, "PURPOSE_SCOPE_UNKNOWN", "PURPOSE_SCOPE_DENIED")?;
            require_true(facts.scope.exact_operation_match, "OPERATION_BINDING_UNKNOWN", "OPERATION_BINDING_MISMATCH")?;
            require_true(facts.scope.delegation_attenuates, "DELEGATION_SCOPE_UNKNOWN", "DELEGATION_SCOPE_AMPLIFIED")?;
            require_true(facts.rights.rights_satisfied, "RIGHTS_STATUS_UNKNOWN", "RIGHTS_FLOOR_NOT_SATISFIED")?;
            require_true(facts.evidence.sufficient, "REQUIRED_EVIDENCE_UNKNOWN", "REQUIRED_EVIDENCE_INSUFFICIENT")?;
            Decision::permit("LOCAL_AUTHORITY_SCOPE_RIGHTS_AND_EVIDENCE_QUALIFIED")
        }
        EvaluationMode::Ratification => {
            require_true(facts.authority.local_authorized, "LOCAL_RATIFIER_UNKNOWN", "LOCAL_RATIFIER_UNAUTHORIZED")?;
            require_true(facts.authority.local_ratification, "LOCAL_RATIFICATION_UNKNOWN", "LOCAL_RATIFICATION_ABSENT")?;
            require_true(facts.authority.predecessor_digest_matches, "PREDECESSOR_DIGEST_UNKNOWN", "PREDECESSOR_DIGEST_MISMATCH")?;
            require_true(facts.authority.constitution_current, "CONSTITUTIONAL_EPOCH_UNKNOWN", "CONSTITUTIONAL_EPOCH_STALE")?;
            Decision::permit("SUCCESSOR_CONTRACT_RATIFIED_UNDER_CURRENT_LOCAL_CONSTITUTION")
        }
        EvaluationMode::EvidenceAdmission => {
            let local = facts.authority.local_authorized == Some(true);
            let partner = facts.authority.partner_credential_valid == Some(true);
            if !local && !partner {
                return Decision::indeterminate("NO_QUALIFIED_LOCAL_OR_ISSUER_EVIDENCE_BASIS");
            }
            require_true(facts.evidence.sufficient, "EVIDENCE_SUFFICIENCY_UNKNOWN", "EVIDENCE_INSUFFICIENT")?;
            Decision::permit("EVIDENCE_ACCEPTED_FOR_LOCAL_EVALUATION_NOT_AUTHORITY")
        }
        EvaluationMode::Exit => {
            require_true(facts.rights.exit_valid, "EXIT_RIGHT_UNKNOWN", "EXIT_NOT_AUTHORIZED")?;
            require_true(facts.rights.history_preserved, "HISTORY_PRESERVATION_UNKNOWN", "HISTORY_NOT_PRESERVED")?;
            Decision::permit("EXIT_HONORED_WITH_HISTORY_AND_REMEDIES_PRESERVED")
        }
        EvaluationMode::Remedy => {
            require_true(facts.rights.challenge_upheld, "CHALLENGE_RESULT_UNKNOWN", "CHALLENGE_NOT_UPHELD")?;
            require_true(facts.rights.reviewer_independent, "REVIEWER_INDEPENDENCE_UNKNOWN", "REVIEWER_NOT_INDEPENDENT")?;
            require_true(facts.rights.remedy_authorized, "REMEDY_AUTHORITY_UNKNOWN", "REMEDY_UNAUTHORIZED")?;
            require_true(facts.rights.history_preserved, "HISTORY_PRESERVATION_UNKNOWN", "HISTORY_NOT_PRESERVED")?;
            Decision::permit("CONTESTED_REMEDY_AUTHORIZED_AND_HISTORY_PRESERVED")
        }
        EvaluationMode::StagedReward => {
            require_true(facts.evidence.outcome_independently_observed, "OUTCOME_OBSERVATION_UNKNOWN", "OUTCOME_NOT_INDEPENDENTLY_OBSERVED")?;
            require_true(facts.scope.payout_bounded, "PAYOUT_BOUND_UNKNOWN", "PAYOUT_UNBOUNDED")?;
            require_true(facts.authority.treasury_separately_authorized, "TREASURY_AUTHORITY_UNKNOWN", "TREASURY_AUTHORITY_DENIED")?;
            Decision::permit("STAGED_REWARD_REQUEST_QUALIFIED_FOR_OWNER_TREASURY")
        }
        EvaluationMode::SafeContinuity => {
            require_true(facts.institution.degraded_service_allowlisted, "DEGRADED_SERVICE_ALLOWLIST_UNKNOWN", "SERVICE_NOT_ALLOWLISTED")?;
            require_true(facts.institution.high_impact_paused, "HIGH_IMPACT_PAUSE_UNKNOWN", "HIGH_IMPACT_NOT_PAUSED")?;
            require_true(facts.institution.old_sessions_fenced, "OLD_SESSION_FENCING_UNKNOWN", "OLD_SESSIONS_NOT_FENCED")?;
            if facts.institution.high_impact_requested == Some(true) {
                require_true(facts.authority.recovery_quorum_met, "RECOVERY_QUORUM_UNKNOWN", "RECOVERY_QUORUM_NOT_MET")?;
            }
            Decision::permit("ESSENTIAL_DEGRADED_SERVICE_ONLY_AUTHORITY_REMAINS_BOUNDED")
        }
        EvaluationMode::HistoricalReconstruction => unreachable!("handled before current-state checks"),
    }
}

/// Helper that preserves unknown as Indeterminate and known false as Deny.
fn require_true(value: Option<bool>, unknown: &'static str, denied: &'static str) -> Result<(), Decision> {
    match value {
        Some(true) => Ok(()),
        Some(false) => Err(Decision::deny(denied)),
        None => Err(Decision::indeterminate(unknown)),
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ContractState {
    Draft,
    Validated,
    Proposed,
    Ratified,
    Active,
    Challenged,
    Suspended,
    Amended,
    Expired,
    Terminated,
    Archived,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Obligation {
    pub id: String,
    pub responsible_party: String,
    pub description: String,
    pub required_evidence: Vec<String>,
    pub deadline_unix: Option<i64>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RemedyPolicy {
    pub challenge_window_seconds: u64,
    pub independent_review_required: bool,
    pub appeal_available: bool,
    pub available_remedies: Vec<String>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct InstitutionalContract {
    pub contract_id: String,
    pub version: u64,
    pub digest: String,
    pub predecessor_digest: Option<String>,
    pub constitutional_digest: String,
    pub jurisdiction: String,
    pub purpose: String,
    pub parties: Vec<String>,
    pub rights_floor: Vec<String>,
    pub obligations: Vec<Obligation>,
    pub remedy_policy: Option<RemedyPolicy>,
    pub high_impact: bool,
    pub state: ContractState,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ContractViolation {
    MissingField(&'static str),
    DuplicateParty(String),
    DuplicateObligation(String),
    HighImpactMissingIndependentRemedy,
    HighImpactMissingChallengeWindow,
    HighImpactMissingRemedyOptions,
}

pub fn validate_contract(contract: &InstitutionalContract) -> Vec<ContractViolation> {
    let mut violations = Vec::new();
    if contract.contract_id.trim().is_empty() {
        violations.push(ContractViolation::MissingField("contract_id"));
    }
    if contract.version == 0 {
        violations.push(ContractViolation::MissingField("version"));
    }
    if contract.digest.trim().is_empty() {
        violations.push(ContractViolation::MissingField("digest"));
    }
    if contract.constitutional_digest.trim().is_empty() {
        violations.push(ContractViolation::MissingField("constitutional_digest"));
    }
    if contract.jurisdiction.trim().is_empty() {
        violations.push(ContractViolation::MissingField("jurisdiction"));
    }
    if contract.purpose.trim().is_empty() {
        violations.push(ContractViolation::MissingField("purpose"));
    }
    if contract.parties.is_empty() {
        violations.push(ContractViolation::MissingField("parties"));
    }
    if contract.rights_floor.is_empty() {
        violations.push(ContractViolation::MissingField("rights_floor"));
    }
    if contract.obligations.is_empty() {
        violations.push(ContractViolation::MissingField("obligations"));
    }

    let mut parties = BTreeSet::new();
    for party in &contract.parties {
        if !parties.insert(party) {
            violations.push(ContractViolation::DuplicateParty(party.clone()));
        }
    }
    let mut obligations = BTreeSet::new();
    for obligation in &contract.obligations {
        if obligation.id.trim().is_empty() || obligation.responsible_party.trim().is_empty()
            || obligation.description.trim().is_empty()
        {
            violations.push(ContractViolation::MissingField("obligation_fields"));
        }
        if !obligations.insert(&obligation.id) {
            violations.push(ContractViolation::DuplicateObligation(obligation.id.clone()));
        }
    }

    if contract.high_impact {
        match &contract.remedy_policy {
            None => violations.push(ContractViolation::HighImpactMissingIndependentRemedy),
            Some(policy) => {
                if !policy.independent_review_required {
                    violations.push(ContractViolation::HighImpactMissingIndependentRemedy);
                }
                if policy.challenge_window_seconds == 0 {
                    violations.push(ContractViolation::HighImpactMissingChallengeWindow);
                }
                if policy.available_remedies.is_empty() {
                    violations.push(ContractViolation::HighImpactMissingRemedyOptions);
                }
            }
        }
    }
    violations
}

pub fn transition_allowed(from: ContractState, to: ContractState) -> bool {
    matches!(
        (from, to),
        (ContractState::Draft, ContractState::Validated)
            | (ContractState::Validated, ContractState::Proposed)
            | (ContractState::Proposed, ContractState::Ratified)
            | (ContractState::Ratified, ContractState::Active)
            | (ContractState::Active, ContractState::Challenged)
            | (ContractState::Active, ContractState::Suspended)
            | (ContractState::Active, ContractState::Amended)
            | (ContractState::Active, ContractState::Expired)
            | (ContractState::Active, ContractState::Terminated)
            | (ContractState::Challenged, ContractState::Active)
            | (ContractState::Challenged, ContractState::Suspended)
            | (ContractState::Challenged, ContractState::Terminated)
            | (ContractState::Suspended, ContractState::Active)
            | (ContractState::Suspended, ContractState::Terminated)
            | (ContractState::Amended, ContractState::Proposed)
            | (ContractState::Expired, ContractState::Archived)
            | (ContractState::Terminated, ContractState::Archived)
    )
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct AmendmentBinding<'a> {
    pub proposed_predecessor_digest: &'a str,
    pub active_contract_digest: &'a str,
    pub ratified_constitution_digest: &'a str,
    pub current_constitution_digest: &'a str,
}

pub fn validate_amendment(binding: &AmendmentBinding<'_>) -> Result<(), &'static str> {
    if binding.proposed_predecessor_digest != binding.active_contract_digest {
        return Err("PREDECESSOR_DIGEST_MISMATCH");
    }
    if binding.ratified_constitution_digest != binding.current_constitution_digest {
        return Err("CONSTITUTION_DIGEST_MISMATCH");
    }
    Ok(())
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DelegationScope {
    pub actions: BTreeSet<String>,
    pub resources: BTreeSet<String>,
    pub purposes: BTreeSet<String>,
    pub fields: BTreeSet<String>,
    pub valid_from_unix: i64,
    pub valid_until_unix: i64,
    pub max_amount: Option<u64>,
    pub max_operations: Option<u32>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ScopeViolation {
    ActionAmplification,
    ResourceAmplification,
    PurposeAmplification,
    FieldAmplification,
    ValidityStartsTooEarly,
    ValidityEndsTooLate,
    AmountCeilingAmplification,
    OperationCeilingAmplification,
    InvalidValidityWindow,
}

/// Check that a child grant is no broader than its immediate parent.
///
/// A parent with no amount/count ceiling is unbounded in that dimension.
/// A bounded parent cannot delegate an unbounded child.
pub fn delegation_is_attenuating(parent: &DelegationScope, child: &DelegationScope) -> Result<(), Vec<ScopeViolation>> {
    let mut violations = Vec::new();
    if child.valid_from_unix > child.valid_until_unix {
        violations.push(ScopeViolation::InvalidValidityWindow);
    }
    if !child.actions.is_subset(&parent.actions) {
        violations.push(ScopeViolation::ActionAmplification);
    }
    if !child.resources.is_subset(&parent.resources) {
        violations.push(ScopeViolation::ResourceAmplification);
    }
    if !child.purposes.is_subset(&parent.purposes) {
        violations.push(ScopeViolation::PurposeAmplification);
    }
    if !child.fields.is_subset(&parent.fields) {
        violations.push(ScopeViolation::FieldAmplification);
    }
    if child.valid_from_unix < parent.valid_from_unix {
        violations.push(ScopeViolation::ValidityStartsTooEarly);
    }
    if child.valid_until_unix > parent.valid_until_unix {
        violations.push(ScopeViolation::ValidityEndsTooLate);
    }
    if parent.max_amount.is_some_and(|limit| child.max_amount.is_none_or(|child_limit| child_limit > limit)) {
        violations.push(ScopeViolation::AmountCeilingAmplification);
    }
    if parent.max_operations.is_some_and(|limit| child.max_operations.is_none_or(|child_limit| child_limit > limit)) {
        violations.push(ScopeViolation::OperationCeilingAmplification);
    }
    if violations.is_empty() {
        Ok(())
    } else {
        Err(violations)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn scope() -> DelegationScope {
        DelegationScope {
            actions: ["read", "write"].into_iter().map(str::to_owned).collect(),
            resources: ["ledger-a"].into_iter().map(str::to_owned).collect(),
            purposes: ["reconcile"].into_iter().map(str::to_owned).collect(),
            fields: ["amount", "status"].into_iter().map(str::to_owned).collect(),
            valid_from_unix: 100,
            valid_until_unix: 500,
            max_amount: Some(100),
            max_operations: Some(5),
        }
    }

    #[test]
    fn child_scope_may_only_narrow_each_dimension() {
        let parent = scope();
        let mut child = scope();
        child.actions.remove("write");
        child.max_amount = Some(50);
        child.max_operations = Some(2);
        child.valid_from_unix = 150;
        child.valid_until_unix = 450;
        assert!(delegation_is_attenuating(&parent, &child).is_ok());
    }

    #[test]
    fn bounded_parent_rejects_unbounded_child() {
        let parent = scope();
        let mut child = scope();
        child.max_amount = None;
        assert!(delegation_is_attenuating(&parent, &child)
            .unwrap_err()
            .contains(&ScopeViolation::AmountCeilingAmplification));
    }

    #[test]
    fn child_cannot_add_a_resource_or_extend_expiry() {
        let parent = scope();
        let mut child = scope();
        child.resources.insert("ledger-b".to_owned());
        child.valid_until_unix = 501;
        let errors = delegation_is_attenuating(&parent, &child).unwrap_err();
        assert!(errors.contains(&ScopeViolation::ResourceAmplification));
        assert!(errors.contains(&ScopeViolation::ValidityEndsTooLate));
    }

    #[test]
    fn contract_state_machine_does_not_allow_draft_to_activate() {
        assert!(!transition_allowed(ContractState::Draft, ContractState::Active));
        assert!(transition_allowed(ContractState::Draft, ContractState::Validated));
        assert!(transition_allowed(ContractState::Ratified, ContractState::Active));
    }

    #[test]
    fn amendment_requires_exact_predecessor_and_constitution_digests() {
        let valid = AmendmentBinding {
            proposed_predecessor_digest: "contract-a",
            active_contract_digest: "contract-a",
            ratified_constitution_digest: "constitution-7",
            current_constitution_digest: "constitution-7",
        };
        assert!(validate_amendment(&valid).is_ok());
        let stale = AmendmentBinding { current_constitution_digest: "constitution-8", ..valid };
        assert_eq!(validate_amendment(&stale), Err("CONSTITUTION_DIGEST_MISMATCH"));
    }

    #[test]
    fn high_impact_contract_requires_independent_review_and_remedy() {
        let contract = InstitutionalContract {
            contract_id: "contract-1".into(),
            version: 1,
            digest: "digest-1".into(),
            predecessor_digest: None,
            constitutional_digest: "constitution-1".into(),
            jurisdiction: "local".into(),
            purpose: "essential service".into(),
            parties: vec!["community".into(), "operator".into()],
            rights_floor: vec!["challenge".into(), "exit".into()],
            obligations: vec![Obligation {
                id: "ob-1".into(),
                responsible_party: "operator".into(),
                description: "maintain service".into(),
                required_evidence: vec!["uptime-observation".into()],
                deadline_unix: None,
            }],
            remedy_policy: None,
            high_impact: true,
            state: ContractState::Draft,
        };
        assert!(validate_contract(&contract).contains(&ContractViolation::HighImpactMissingIndependentRemedy));
    }
}
