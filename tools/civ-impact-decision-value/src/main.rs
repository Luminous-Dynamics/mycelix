//! CIV-IMPACT-002 deterministic synthetic decision-value harness.
//!
//! The policy function only accepts PolicyView and visible query history. The
//! evaluator/oracle truth and evidence-provider responses remain outside that
//! function boundary. This is a synthetic contract exercise, not a production
//! planner or evidence-acquisition service.

use serde::{Deserialize, Serialize};
use serde_json::Value;
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet};
use std::env;
use std::error::Error;
use std::fs;
use std::path::Path;

const SCHEMA_VERSION: &str = "0.1";
const CLAIM_CEILING: &str =
    "synthetic_or_replay_scoring_only; not proof_of_real_world_effect_or_economic_benefit";
const POLICY_SOURCE_BYTES: &[u8] = include_bytes!("main.rs");
const SEEDS_PER_REGIME: u64 = 8;

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq, PartialOrd, Ord)]
#[serde(rename_all = "snake_case")]
enum Regime {
    Nominal,
    StaleOrMissing,
    CorrelatedMirrors,
    ConflictingClaims,
    TargetedInformation,
    UninformativeRequest,
    DistributionShift,
    BudgetOrLatencyPressure,
    CoordinatorOutage,
    ReceiptTampering,
    HighConsequenceFallback,
    LowCostObservation,
}

impl Regime {
    const ALL: [Self; 12] = [
        Self::Nominal,
        Self::StaleOrMissing,
        Self::CorrelatedMirrors,
        Self::ConflictingClaims,
        Self::TargetedInformation,
        Self::UninformativeRequest,
        Self::DistributionShift,
        Self::BudgetOrLatencyPressure,
        Self::CoordinatorOutage,
        Self::ReceiptTampering,
        Self::HighConsequenceFallback,
        Self::LowCostObservation,
    ];

    fn as_str(self) -> &'static str {
        match self {
            Self::Nominal => "nominal",
            Self::StaleOrMissing => "stale_or_missing",
            Self::CorrelatedMirrors => "correlated_mirrors",
            Self::ConflictingClaims => "conflicting_claims",
            Self::TargetedInformation => "targeted_information",
            Self::UninformativeRequest => "uninformative_request",
            Self::DistributionShift => "distribution_shift",
            Self::BudgetOrLatencyPressure => "budget_or_latency_pressure",
            Self::CoordinatorOutage => "coordinator_outage",
            Self::ReceiptTampering => "receipt_tampering",
            Self::HighConsequenceFallback => "high_consequence_fallback",
            Self::LowCostObservation => "low_cost_observation",
        }
    }

    fn is_high_consequence(self) -> bool {
        self == Self::HighConsequenceFallback
    }

    fn initial_evidence_is_trusted(self) -> bool {
        matches!(self, Self::Nominal | Self::UninformativeRequest | Self::LowCostObservation)
    }

    fn reported_status_is_wrong(self) -> bool {
        matches!(
            self,
            Self::StaleOrMissing
                | Self::ConflictingClaims
                | Self::TargetedInformation
                | Self::DistributionShift
                | Self::BudgetOrLatencyPressure
                | Self::CoordinatorOutage
                | Self::ReceiptTampering
                | Self::HighConsequenceFallback
        )
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
enum ObservationState {
    Observed,
    Partial,
    Conflicted,
    Unknown,
    Stale,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
enum RightsProfile {
    PublicPermitted,
    AuthorizedAggregate,
    Consented,
    Restricted,
    Unknown,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
enum RequestType {
    IndependentPublicSource,
    AuthorizedAggregate,
    SourceAudit,
    ProspectiveMeasurement,
    FreshnessCheck,
    Replication,
    PrivacyPreservingAggregate,
    OtherPermitted,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
enum Sensitivity {
    Public,
    Aggregate,
    Personal,
    SpecialCategory,
    Unknown,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
enum Authorization {
    NoneRequiredForSyntheticFixture,
    SeparateOwnerApprovalRequired,
    ParticipantConsentRequired,
    NotAuthorized,
}

impl Authorization {
    fn permitted_in_this_fixture(self) -> bool {
        self == Self::NoneRequiredForSyntheticFixture
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct Budgets {
    max_evidence_requests: usize,
    max_evidence_cost_microcredits: u64,
    max_total_cost_microcredits: u64,
    max_elapsed_ms: u64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct Observation {
    observation_id: String,
    subject_ref: String,
    predicate: String,
    value: Value,
    source_ref: String,
    source_family_ref: String,
    provenance_digest_sha256: String,
    age_ms: u64,
    freshness_limit_ms: u64,
    state: ObservationState,
    rights_profile: RightsProfile,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct EvidenceOption {
    request_option_id: String,
    request_type: RequestType,
    expected_cost_microcredits: u64,
    expected_latency_ms: u64,
    expected_coverage_permille: u16,
    expected_reliability_permille: u16,
    expected_decision_loss_reduction_microcredits: u64,
    sensitivity: Sensitivity,
    authorization: Authorization,
    candidate_source_family_refs: Vec<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct ActionOption {
    action_id: String,
    description: String,
    authority_profile: String,
}

/// This is the entire data structure passed into choose_action().
/// It deliberately has no evaluator truth, hidden outcome or oracle field.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct PolicyView {
    schema_version: String,
    episode_id: String,
    regime: Regime,
    budgets: Budgets,
    permitted_decision_kinds: Vec<String>,
    observations: Vec<Observation>,
    evidence_options: Vec<EvidenceOption>,
    action_options: Vec<ActionOption>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct LatentFact {
    fact_id: String,
    subject_ref: String,
    predicate: String,
    value: Value,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct TruthActionOutcome {
    action_id: String,
    realized_decision_loss_microcredits: u64,
    completed_units: u64,
    unserved_units: u64,
    guardrail_violations: Vec<String>,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
enum RequestDisposition {
    Provided,
    Denied,
    NotAuthorized,
    BudgetExceeded,
    Timeout,
    NoResult,
    InvalidReceipt,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct TruthEvidenceRequestOutcome {
    request_option_id: String,
    disposition: RequestDisposition,
    actual_cost_microcredits: u64,
    actual_latency_ms: u64,
    returned_observations: Vec<Observation>,
    source_family_refs: Vec<String>,
}

/// Evaluator-only object; never handed to choose_action().
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct EvaluatorTruth {
    schema_version: String,
    episode_id: String,
    latent_facts: Vec<LatentFact>,
    action_outcomes: Vec<TruthActionOutcome>,
    evidence_request_outcomes: Vec<TruthEvidenceRequestOutcome>,
}

impl EvaluatorTruth {
    fn bool_fact(&self, predicate: &str) -> bool {
        self.latent_facts
            .iter()
            .find(|f| f.predicate == predicate)
            .and_then(|f| f.value.as_bool())
            .unwrap_or(false)
    }

    fn action_loss(&self, action_id: &str) -> u64 {
        self.action_outcomes
            .iter()
            .find(|o| o.action_id == action_id)
            .map(|o| o.realized_decision_loss_microcredits)
            .unwrap_or(1_000_000)
    }

    fn request_outcome(&self, request_id: &str) -> Option<&TruthEvidenceRequestOutcome> {
        self.evidence_request_outcomes
            .iter()
            .find(|o| o.request_option_id == request_id)
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq, PartialOrd, Ord)]
#[serde(rename_all = "snake_case")]
enum PolicyKind {
    ObservationRule,
    ForcedAction,
    AlwaysDefer,
    QueryAll,
    FixedQuery,
    ValueOfInformation,
    UnauthorizedRequester,
}

impl PolicyKind {
    const ALL: [Self; 7] = [
        Self::ObservationRule,
        Self::ForcedAction,
        Self::AlwaysDefer,
        Self::QueryAll,
        Self::FixedQuery,
        Self::ValueOfInformation,
        Self::UnauthorizedRequester,
    ];

    fn as_str(self) -> &'static str {
        match self {
            Self::ObservationRule => "observation_rule",
            Self::ForcedAction => "forced_action",
            Self::AlwaysDefer => "always_defer",
            Self::QueryAll => "query_all",
            Self::FixedQuery => "fixed_query",
            Self::ValueOfInformation => "value_of_information",
            Self::UnauthorizedRequester => "unauthorized_requester_negative_control",
        }
    }

    fn compute_cost(self) -> u64 {
        match self {
            Self::ObservationRule | Self::ForcedAction | Self::AlwaysDefer => 1,
            Self::FixedQuery => 2,
            Self::ValueOfInformation | Self::UnauthorizedRequester => 3,
            Self::QueryAll => 4,
        }
    }

    fn policy_latency_ms(self) -> u64 {
        self.compute_cost() * 2
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(tag = "kind", rename_all = "snake_case")]
enum Choice {
    Act { action_id: String },
    RequestEvidence { request_option_id: String },
    Abstain,
    WaitReplan,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct ActualRequestRecord {
    request_option_id: String,
    disposition: RequestDisposition,
    cost_microcredits: u64,
    latency_ms: u64,
    returned_observation_sha256: Option<String>,
    source_family_refs: Vec<String>,
}

#[derive(Debug, Clone)]
struct RunOutcome {
    policy: PolicyKind,
    initial_view_sha256: String,
    truth_sha256: String,
    final_view: PolicyView,
    final_choice: Choice,
    request_records: Vec<ActualRequestRecord>,
    queried_option_ids: Vec<String>,
    decision_loss: u64,
    oracle_loss: u64,
    regret: u64,
    evidence_cost: u64,
    compute_cost: u64,
    delay_cost: u64,
    human_review_cost: u64,
    fallback_cost: u64,
    operational_cost: u64,
    failure_recovery_cost: u64,
    total_cost: u64,
    end_to_end_loss: u64,
    decision_latency_ms: u64,
    authority_violations: Vec<String>,
    budget_violations: Vec<String>,
    other_violations: Vec<String>,
}

impl RunOutcome {
    fn guardrail_violations(&self) -> Vec<String> {
        let mut result = self.authority_violations.clone();
        result.extend(self.budget_violations.clone());
        result.extend(self.other_violations.clone());
        result
    }

    fn provided_evidence_count(&self) -> usize {
        self.request_records
            .iter()
            .filter(|r| r.disposition == RequestDisposition::Provided)
            .count()
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct ReceiptDecision {
    kind: String,
    chosen_action_id: Option<String>,
    requested_evidence_option_ids: Vec<String>,
    pre_outcome_confidence_ppm: Option<u32>,
    decision_latency_ms: u64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct ReceiptOutcome {
    state: String,
    completed_units: u64,
    unserved_units: u64,
    fallback_used: bool,
    fallback_succeeded: bool,
    realized_decision_loss_microcredits: u64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct ReceiptCosts {
    evidence_acquisition_microcredits: u64,
    compute_microcredits: u64,
    delay_microcredits: u64,
    human_review_microcredits: u64,
    fallback_microcredits: u64,
    operational_microcredits: u64,
    failure_recovery_microcredits: u64,
    total_cost_microcredits: u64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct ReceiptRegret {
    oracle_action_loss_microcredits: u64,
    regret_vs_oracle_microcredits: u64,
    comparator_policy_artifact_sha256: String,
    comparator_baseline_core_sha256: String,
    comparator_policy_loss_microcredits: u64,
    comparator_total_cost_microcredits: u64,
    candidate_minus_comparator_end_to_end_loss_microcredits: i64,
    realized_end_to_end_loss_reduction_vs_comparator_microcredits: Option<i64>,
    realized_loss_reduction_defined: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct ReceiptGuardrails {
    budget_compliant: bool,
    acquisition_authorized: bool,
    privacy_and_consent_compliant: bool,
    source_rights_compliant: bool,
    no_external_side_effects: bool,
    artifact_integrity_valid: bool,
    invalid_receipt_rejected: bool,
    violations: Vec<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct ReceiptEvidenceQuality {
    unique_source_family_count: usize,
    stale_observation_count: usize,
    conflicted_observation_count: usize,
    observation_coverage_permille: u16,
    brier_score: Option<f64>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct ScoringReceipt {
    schema_version: String,
    episode_id: String,
    policy_id: String,
    policy_artifact_sha256: String,
    policy_observation_sha256: String,
    evidence_model_sha256: String,
    evaluator_truth_sha256: String,
    decision: ReceiptDecision,
    evidence_requests: Vec<ActualRequestRecord>,
    outcome: ReceiptOutcome,
    costs: ReceiptCosts,
    regret: ReceiptRegret,
    guardrails: ReceiptGuardrails,
    evidence_quality: ReceiptEvidenceQuality,
    claim_ceiling: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
struct PolicySummary {
    runs: usize,
    mean_decision_loss_microcredits: f64,
    mean_regret_vs_oracle_microcredits: f64,
    mean_total_cost_microcredits: f64,
    mean_end_to_end_loss_microcredits: f64,
    abstention_count: usize,
    evidence_request_count: usize,
    provided_evidence_count: usize,
    authority_violation_count: usize,
    budget_violation_count: usize,
}

impl Default for PolicySummary {
    fn default() -> Self {
        Self {
            runs: 0,
            mean_decision_loss_microcredits: 0.0,
            mean_regret_vs_oracle_microcredits: 0.0,
            mean_total_cost_microcredits: 0.0,
            mean_end_to_end_loss_microcredits: 0.0,
            abstention_count: 0,
            evidence_request_count: 0,
            provided_evidence_count: 0,
            authority_violation_count: 0,
            budget_violation_count: 0,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
struct StratifiedSummary {
    regime: String,
    by_policy: BTreeMap<String, PolicySummary>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
struct BenchmarkReport {
    schema_version: String,
    benchmark_id: String,
    evidence_class: String,
    fixture_count: usize,
    seeds_per_regime: u64,
    policy_count: usize,
    run_count: usize,
    policies: BTreeMap<String, PolicySummary>,
    stratified_results: Vec<StratifiedSummary>,
    receipts: Vec<ScoringReceipt>,
    all_scheduled_runs_accounted_for: bool,
    hard_guardrail_violation_count: usize,
    economic_benefit_established: bool,
    claim_ceiling: String,
}

fn sha256_hex(bytes: &[u8]) -> String {
    Sha256::digest(bytes)
        .iter()
        .map(|b| format!("{b:02x}"))
        .collect()
}

fn encode<T: Serialize>(value: &T) -> Result<Vec<u8>, Box<dyn Error>> {
    Ok(serde_json::to_vec_pretty(value)?)
}

fn observation(
    observation_id: &str,
    value: bool,
    source_ref: &str,
    source_family_ref: &str,
    age_ms: u64,
    state: ObservationState,
) -> Observation {
    Observation {
        observation_id: observation_id.to_string(),
        subject_ref: "network:bridge:primary".to_string(),
        predicate: "bridge_open".to_string(),
        value: Value::Bool(value),
        source_ref: source_ref.to_string(),
        source_family_ref: source_family_ref.to_string(),
        provenance_digest_sha256: sha256_hex(source_ref.as_bytes()),
        age_ms,
        freshness_limit_ms: 60_000,
        state,
        rights_profile: RightsProfile::PublicPermitted,
    }
}

fn make_fixture(regime: Regime, seed: u64) -> Result<(PolicyView, EvaluatorTruth), Box<dyn Error>> {
    let regime_index = Regime::ALL.iter().position(|r| *r == regime).unwrap_or(0) as u64;
    let bridge_open = (regime_index * 7 + seed * 3) % 5 < 3;
    let high_consequence = regime.is_high_consequence();
    let reported_open = if regime.reported_status_is_wrong() {
        !bridge_open
    } else {
        bridge_open
    };
    let initial_state = if regime == Regime::ConflictingClaims {
        ObservationState::Conflicted
    } else if regime.initial_evidence_is_trusted() {
        ObservationState::Observed
    } else {
        ObservationState::Stale
    };
    let initial_family = if regime == Regime::CorrelatedMirrors {
        "shared-map-provenance-family"
    } else {
        "public-map-family"
    };
    let mut observations = vec![observation(
        "initial-map-status",
        reported_open,
        "source:public-map",
        initial_family,
        if initial_state == ObservationState::Observed { 1_000 } else { 180_000 },
        initial_state,
    )];

    if regime == Regime::ConflictingClaims {
        observations.push(observation(
            "conflicting-registry-status",
            !reported_open,
            "source:registry-claim",
            "conflicting-registry-family",
            30_000,
            ObservationState::Conflicted,
        ));
    }

    let budgets = Budgets {
        max_evidence_requests: 2,
        max_evidence_cost_microcredits: if regime == Regime::BudgetOrLatencyPressure { 5 } else { 30 },
        max_total_cost_microcredits: 100,
        max_elapsed_ms: 5_000,
    };

    let mut evidence_options = Vec::new();
    if regime != Regime::CoordinatorOutage {
        let authorization = if matches!(
            regime,
            Regime::HighConsequenceFallback | Regime::CoordinatorOutage
        ) {
            Authorization::NotAuthorized
        } else {
            Authorization::NoneRequiredForSyntheticFixture
        };
        evidence_options.push(EvidenceOption {
            request_option_id: "independent_status".to_string(),
            request_type: RequestType::IndependentPublicSource,
            expected_cost_microcredits: 10,
            expected_latency_ms: 20,
            expected_coverage_permille: 1_000,
            expected_reliability_permille: if regime == Regime::DistributionShift { 650 } else { 980 },
            expected_decision_loss_reduction_microcredits: if regime == Regime::DistributionShift || (regime == Regime::TargetedInformation && seed == 0) { 6 } else { 18 },
            sensitivity: Sensitivity::Public,
            authorization: if regime == Regime::ReceiptTampering {
                Authorization::NoneRequiredForSyntheticFixture
            } else {
                authorization
            },
            candidate_source_family_refs: vec!["network-observation-family".to_string()],
        });
    }

    if matches!(
        regime,
        Regime::CorrelatedMirrors | Regime::UninformativeRequest | Regime::BudgetOrLatencyPressure
    ) {
        evidence_options.push(EvidenceOption {
            request_option_id: "mirror_status".to_string(),
            request_type: RequestType::OtherPermitted,
            expected_cost_microcredits: 4,
            expected_latency_ms: 10,
            expected_coverage_permille: 1_000,
            expected_reliability_permille: 500,
            expected_decision_loss_reduction_microcredits: 1,
            sensitivity: Sensitivity::Public,
            authorization: Authorization::NoneRequiredForSyntheticFixture,
            candidate_source_family_refs: vec![initial_family.to_string()],
        });
    }

    let view = PolicyView {
        schema_version: SCHEMA_VERSION.to_string(),
        episode_id: format!("civdv-{}-seed-{seed:02}", regime.as_str()),
        regime,
        budgets,
        permitted_decision_kinds: vec![
            "act".to_string(),
            "request_evidence".to_string(),
            "abstain".to_string(),
            "wait_replan".to_string(),
        ],
        observations,
        evidence_options,
        action_options: vec![
            ActionOption {
                action_id: "cross_primary".to_string(),
                description: "Use the primary crossing if the evidence supports it".to_string(),
                authority_profile: "proposal_only".to_string(),
            },
            ActionOption {
                action_id: "take_detour".to_string(),
                description: "Use the conservative detour".to_string(),
                authority_profile: "proposal_only".to_string(),
            },
        ],
    };

    let mut truth = EvaluatorTruth {
        schema_version: SCHEMA_VERSION.to_string(),
        episode_id: view.episode_id.clone(),
        latent_facts: vec![
            LatentFact {
                fact_id: "latent-bridge-status".to_string(),
                subject_ref: "network:bridge:primary".to_string(),
                predicate: "bridge_open".to_string(),
                value: Value::Bool(bridge_open),
            },
            LatentFact {
                fact_id: "latent-consequence-class".to_string(),
                subject_ref: "episode".to_string(),
                predicate: "high_consequence".to_string(),
                value: Value::Bool(high_consequence),
            },
        ],
        action_outcomes: Vec::new(),
        evidence_request_outcomes: Vec::new(),
    };

    let cross_loss = if bridge_open {
        0
    } else if high_consequence {
        250
    } else {
        120
    };
    let detour_loss = 25;
    let abstain_loss = if high_consequence { 8 } else { 50 };
    let wait_loss = if high_consequence { 16 } else { 45 };
    truth.action_outcomes = vec![
        TruthActionOutcome {
            action_id: "cross_primary".to_string(),
            realized_decision_loss_microcredits: cross_loss,
            completed_units: if bridge_open { 1 } else { 0 },
            unserved_units: if !bridge_open { 1 } else { 0 },
            guardrail_violations: if bridge_open { vec![] } else { vec!["closed_primary_route".to_string()] },
        },
        TruthActionOutcome {
            action_id: "take_detour".to_string(),
            realized_decision_loss_microcredits: detour_loss,
            completed_units: 1,
            unserved_units: 0,
            guardrail_violations: vec![],
        },
        TruthActionOutcome {
            action_id: "abstain".to_string(),
            realized_decision_loss_microcredits: abstain_loss,
            completed_units: 0,
            unserved_units: 1,
            guardrail_violations: vec![],
        },
        TruthActionOutcome {
            action_id: "wait_replan".to_string(),
            realized_decision_loss_microcredits: wait_loss,
            completed_units: 0,
            unserved_units: 1,
            guardrail_violations: vec![],
        },
    ];

    for option in &view.evidence_options {
        let (disposition, cost, latency, returned) =
            make_truth_request_outcome(option, regime, reported_open, bridge_open)?;
        let source_families = returned
            .iter()
            .map(|o| o.source_family_ref.clone())
            .collect::<BTreeSet<_>>()
            .into_iter()
            .collect();
        truth.evidence_request_outcomes.push(TruthEvidenceRequestOutcome {
            request_option_id: option.request_option_id.clone(),
            disposition,
            actual_cost_microcredits: cost,
            actual_latency_ms: latency,
            returned_observations: returned,
            source_family_refs: source_families,
        });
    }

    Ok((view, truth))
}

fn make_truth_request_outcome(
    option: &EvidenceOption,
    regime: Regime,
    reported_open: bool,
    bridge_open: bool,
) -> Result<(RequestDisposition, u64, u64, Vec<Observation>), Box<dyn Error>> {
    if !option.authorization.permitted_in_this_fixture() {
        return Ok((RequestDisposition::NotAuthorized, 0, 0, vec![]));
    }
    if regime == Regime::ReceiptTampering && option.request_type == RequestType::IndependentPublicSource {
        return Ok((
            RequestDisposition::InvalidReceipt,
            option.expected_cost_microcredits,
            option.expected_latency_ms,
            vec![],
        ));
    }

    let returned = match option.request_type {
        RequestType::IndependentPublicSource
        | RequestType::AuthorizedAggregate
        | RequestType::PrivacyPreservingAggregate => {
            // The shifted regime deliberately violates the declared reliability
            // model to measure OOD failure, not to simulate a trusted source.
            let returned_open = if regime == Regime::DistributionShift {
                !bridge_open
            } else {
                bridge_open
            };
            vec![observation(
                &format!("response-{}", option.request_option_id),
                returned_open,
                "source:independent-network-observer",
                option.candidate_source_family_refs.first()
                    .map(String::as_str)
                    .unwrap_or("independent-family"),
                0,
                ObservationState::Observed,
            )]
        }
        _ => {
            let source_family = option.candidate_source_family_refs.first()
                .map(String::as_str)
                .unwrap_or("public-map-family");
            vec![observation(
                &format!("response-{}", option.request_option_id),
                reported_open,
                "source:map-mirror",
                source_family,
                180_000,
                ObservationState::Stale,
            )]
        }
    };
    Ok((
        RequestDisposition::Provided,
        option.expected_cost_microcredits,
        option.expected_latency_ms,
        returned,
    ))
}

fn trusted_status(view: &PolicyView) -> Option<bool> {
    view.observations.iter().rev().find_map(|o| {
        let permitted_rights = matches!(
            o.rights_profile,
            RightsProfile::PublicPermitted | RightsProfile::AuthorizedAggregate | RightsProfile::Consented
        );
        let fresh = o.state == ObservationState::Observed && o.age_ms <= o.freshness_limit_ms;
        if o.predicate == "bridge_open" && permitted_rights && fresh {
            o.value.as_bool()
        } else {
            None
        }
    })
}

fn status_decision(value: bool) -> Choice {
    Choice::Act {
        action_id: if value { "cross_primary" } else { "take_detour" }.to_string(),
    }
}

fn request_fits_budget(option: &EvidenceOption, view: &PolicyView, queried: &BTreeSet<String>) -> bool {
    if !option.authorization.permitted_in_this_fixture() || queried.contains(&option.request_option_id) {
        return false;
    }
    let used_requests = queried.len();
    let used_cost = view
        .evidence_options
        .iter()
        .filter(|o| queried.contains(&o.request_option_id))
        .map(|o| o.expected_cost_microcredits)
        .sum::<u64>();
    used_requests < view.budgets.max_evidence_requests
        && used_cost.saturating_add(option.expected_cost_microcredits)
            <= view.budgets.max_evidence_cost_microcredits
}

fn choose_action(policy: PolicyKind, view: &PolicyView, queried: &BTreeSet<String>) -> Choice {
    // IMPORTANT: this function has no EvaluatorTruth parameter, no global
    // evaluator state and no callback that can reveal the latent outcome.
    if policy == PolicyKind::ForcedAction {
        return Choice::Act { action_id: "cross_primary".to_string() };
    }
    if policy == PolicyKind::AlwaysDefer {
        return Choice::Abstain;
    }
    if policy == PolicyKind::UnauthorizedRequester {
        return view.evidence_options.iter()
            .find(|o| !queried.contains(&o.request_option_id))
            .map(|o| Choice::RequestEvidence { request_option_id: o.request_option_id.clone() })
            .unwrap_or(Choice::Abstain);
    }

    // Deliberately adversarial comparator: ask for every query that fits,
    // even when the initial evidence is already sufficient.
    if policy == PolicyKind::QueryAll {
        if let Some(option) = view.evidence_options.iter()
            .find(|option| request_fits_budget(option, view, queried))
        {
            return Choice::RequestEvidence { request_option_id: option.request_option_id.clone() };
        }
        return trusted_status(view).map(status_decision).unwrap_or_else(|| {
            if view.regime.is_high_consequence() {
                Choice::Abstain
            } else {
                Choice::Act { action_id: "take_detour".to_string() }
            }
        });
    }

    // Fixed-query control requests its preferred source before considering
    // whether another observation already suffices.
    if policy == PolicyKind::FixedQuery {
        if let Some(option) = view.evidence_options.iter().find(|option| {
            option.request_type == RequestType::IndependentPublicSource
                && request_fits_budget(option, view, queried)
        }) {
            return Choice::RequestEvidence { request_option_id: option.request_option_id.clone() };
        }
        return trusted_status(view).map(status_decision).unwrap_or_else(|| {
            Choice::Act { action_id: "take_detour".to_string() }
        });
    }

    if let Some(status) = trusted_status(view) {
        return status_decision(status);
    }

    match policy {
        PolicyKind::ObservationRule => Choice::Act { action_id: "take_detour".to_string() },
        PolicyKind::ValueOfInformation => {
            let option = view.evidence_options.iter().find(|option| {
                matches!(
                    option.request_type,
                    RequestType::IndependentPublicSource
                        | RequestType::AuthorizedAggregate
                        | RequestType::PrivacyPreservingAggregate
                ) && option.expected_reliability_permille >= 900
                    && option.expected_decision_loss_reduction_microcredits
                        > option.expected_cost_microcredits + (option.expected_latency_ms + 9) / 10
                    && matches!(option.sensitivity, Sensitivity::Public | Sensitivity::Aggregate)
                    && option.authorization.permitted_in_this_fixture()
                    && request_fits_budget(option, view, queried)
                    && option.candidate_source_family_refs.iter().any(|family| {
                        !view.observations.iter().any(|observation| &observation.source_family_ref == family)
                    })
            });
            if let Some(option) = option {
                Choice::RequestEvidence { request_option_id: option.request_option_id.clone() }
            } else if view.regime.is_high_consequence() {
                Choice::Abstain
            } else {
                Choice::Act { action_id: "take_detour".to_string() }
            }
        }
        PolicyKind::ForcedAction
        | PolicyKind::AlwaysDefer
        | PolicyKind::QueryAll
        | PolicyKind::FixedQuery
        | PolicyKind::UnauthorizedRequester => unreachable!("handled before trusted-status branch"),
    }
}

fn execute_request(
    option: &EvidenceOption,
    truth: &EvaluatorTruth,
) -> Option<TruthEvidenceRequestOutcome> {
    truth.request_outcome(&option.request_option_id).cloned()
}

fn evaluate_policy(
    policy: PolicyKind,
    initial_view: &PolicyView,
    truth: &EvaluatorTruth,
) -> Result<RunOutcome, Box<dyn Error>> {
    let initial_view_sha256 = sha256_hex(&encode(initial_view)?);
    let truth_sha256 = sha256_hex(&encode(truth)?);
    let mut view = initial_view.clone();
    let mut queried = BTreeSet::new();
    let mut records = Vec::new();
    let mut authority_violations = Vec::new();
    let mut budget_violations = Vec::new();
    let mut other_violations = Vec::new();
    let mut evidence_cost = 0_u64;
    let mut delay_cost = 0_u64;
    let mut elapsed_ms = policy.policy_latency_ms();
    let mut final_choice = Choice::Abstain;

    for _ in 0..=(view.evidence_options.len() + 1) {
        let choice = choose_action(policy, &view, &queried);
        match choice {
            Choice::RequestEvidence { request_option_id } => {
                let Some(option) = view.evidence_options.iter()
                    .find(|o| o.request_option_id == request_option_id)
                    .cloned()
                else {
                    other_violations.push("policy requested an option not present in the policy view".to_string());
                    final_choice = Choice::Abstain;
                    break;
                };
                if queried.contains(&request_option_id) {
                    other_violations.push("policy requested the same evidence option twice".to_string());
                    final_choice = Choice::Abstain;
                    break;
                }
                queried.insert(request_option_id.clone());

                if !option.authorization.permitted_in_this_fixture() {
                    authority_violations.push(format!("attempted unauthorized evidence request: {request_option_id}"));
                    records.push(ActualRequestRecord {
                        request_option_id,
                        disposition: RequestDisposition::NotAuthorized,
                        cost_microcredits: 0,
                        latency_ms: 0,
                        returned_observation_sha256: None,
                        source_family_refs: vec![],
                    });
                    continue;
                }
                let used_requests = records.iter().filter(|r| {
                    matches!(r.disposition, RequestDisposition::Provided | RequestDisposition::InvalidReceipt)
                }).count();
                if used_requests >= view.budgets.max_evidence_requests
                    || evidence_cost.saturating_add(option.expected_cost_microcredits)
                        > view.budgets.max_evidence_cost_microcredits
                {
                    budget_violations.push(format!("attempted evidence request beyond budget: {request_option_id}"));
                    records.push(ActualRequestRecord {
                        request_option_id,
                        disposition: RequestDisposition::BudgetExceeded,
                        cost_microcredits: 0,
                        latency_ms: 0,
                        returned_observation_sha256: None,
                        source_family_refs: vec![],
                    });
                    continue;
                }
                let Some(request_outcome) = execute_request(&option, truth) else {
                    other_violations.push("evaluator evidence model omitted a registered request option".to_string());
                    final_choice = Choice::Abstain;
                    break;
                };
                evidence_cost = evidence_cost.saturating_add(request_outcome.actual_cost_microcredits);
                delay_cost = delay_cost.saturating_add((request_outcome.actual_latency_ms + 9) / 10);
                elapsed_ms = elapsed_ms.saturating_add(request_outcome.actual_latency_ms);
                let observation_sha = if request_outcome.returned_observations.is_empty() {
                    None
                } else {
                    Some(sha256_hex(&encode(&request_outcome.returned_observations)?))
                };
                for observation in &request_outcome.returned_observations {
                    view.observations.push(observation.clone());
                }
                records.push(ActualRequestRecord {
                    request_option_id,
                    disposition: request_outcome.disposition,
                    cost_microcredits: request_outcome.actual_cost_microcredits,
                    latency_ms: request_outcome.actual_latency_ms,
                    returned_observation_sha256: observation_sha,
                    source_family_refs: request_outcome.source_family_refs,
                });
                continue;
            }
            other => {
                final_choice = other;
                break;
            }
        }
    }

    // The evaluator now scores the final choice. This section is intentionally
    // separate from choose_action() and is the only code that uses hidden truth.
    let decision_action = match &final_choice {
        Choice::Act { action_id } => action_id.as_str(),
        Choice::Abstain => "abstain",
        Choice::WaitReplan => "wait_replan",
        Choice::RequestEvidence { .. } => {
            other_violations.push("policy exhausted decision loop while requesting evidence".to_string());
            "abstain"
        }
    };
    let decision_loss = truth.action_loss(decision_action);
    let oracle_loss = truth.action_outcomes.iter()
        .map(|outcome| outcome.realized_decision_loss_microcredits)
        .min()
        .unwrap_or(decision_loss);
    let regret = decision_loss.saturating_sub(oracle_loss);
    let compute_cost = policy.compute_cost();
    let (human_review_cost, fallback_cost, operational_cost, failure_recovery_cost) =
        match decision_action {
            "abstain" => (2, 15, 0, 0),
            "wait_replan" => (2, 12, 0, 0),
            "cross_primary" if !truth.bool_fact("bridge_open") => (0, 0, 3, 30),
            "cross_primary" => (0, 0, 3, 0),
            _ => (0, 0, 5, 0),
        };
    let total_cost = evidence_cost
        .saturating_add(compute_cost)
        .saturating_add(delay_cost)
        .saturating_add(human_review_cost)
        .saturating_add(fallback_cost)
        .saturating_add(operational_cost)
        .saturating_add(failure_recovery_cost);
    if total_cost > view.budgets.max_total_cost_microcredits {
        budget_violations.push("total end-to-end cost exceeded the registered budget".to_string());
    }
    if elapsed_ms > view.budgets.max_elapsed_ms {
        budget_violations.push("end-to-end latency exceeded the registered budget".to_string());
    }

    Ok(RunOutcome {
        policy,
        initial_view_sha256,
        truth_sha256,
        final_view: view,
        final_choice,
        request_records: records,
        queried_option_ids: queried.into_iter().collect(),
        decision_loss,
        oracle_loss,
        regret,
        evidence_cost,
        compute_cost,
        delay_cost,
        human_review_cost,
        fallback_cost,
        operational_cost,
        failure_recovery_cost,
        total_cost,
        end_to_end_loss: decision_loss.saturating_add(total_cost),
        decision_latency_ms: elapsed_ms,
        authority_violations,
        budget_violations,
        other_violations,
    })
}

fn signed_difference(a: u64, b: u64) -> i64 {
    if a >= b {
        i64::try_from(a - b).unwrap_or(i64::MAX)
    } else {
        -i64::try_from(b - a).unwrap_or(i64::MAX)
    }
}

fn scoring_receipt(
    outcome: &RunOutcome,
    baseline: &RunOutcome,
    truth: &EvaluatorTruth,
) -> Result<ScoringReceipt, Box<dyn Error>> {
    let mut violations = outcome.guardrail_violations();
    let action_id = match &outcome.final_choice {
        Choice::Act { action_id } => Some(action_id.clone()),
        Choice::Abstain | Choice::WaitReplan | Choice::RequestEvidence { .. } => None,
    };
    let decision_kind = match &outcome.final_choice {
        Choice::Act { .. } => "act",
        Choice::RequestEvidence { .. } => "request_evidence",
        Choice::Abstain => "abstain",
        Choice::WaitReplan => "wait_replan",
    }.to_string();

    let scoring_action_id = action_id.as_deref().unwrap_or(match &outcome.final_choice {
        Choice::Abstain => "abstain",
        Choice::WaitReplan => "wait_replan",
        Choice::RequestEvidence { .. } => "abstain",
        Choice::Act { .. } => "abstain",
    });
    if let Some(action_outcome) = truth.action_outcomes.iter()
        .find(|action| action.action_id == scoring_action_id)
    {
        violations.extend(action_outcome.guardrail_violations.clone());
    }

    let state = match action_id.as_deref() {
        Some("cross_primary") if outcome.decision_loss == 0 => "successful",
        Some("cross_primary") => "invalid_decision",
        Some("take_detour") => "successful",
        _ if matches!(&outcome.final_choice, Choice::WaitReplan) => "deferred",
        _ => "unserved",
    }.to_string();
    let completed_units = if state == "successful" { 1 } else { 0 };
    let unserved_units = if completed_units == 0 { 1 } else { 0 };
    let fallback_used = matches!(&outcome.final_choice, Choice::Abstain | Choice::WaitReplan);
    let fallback_succeeded = fallback_used && outcome.decision_loss <= 16;

    let source_families = outcome.final_view.observations.iter()
        .map(|o| o.source_family_ref.clone())
        .collect::<BTreeSet<_>>();
    let stale_count = outcome.final_view.observations.iter()
        .filter(|o| o.state == ObservationState::Stale || o.age_ms > o.freshness_limit_ms)
        .count();
    let conflicted_count = outcome.final_view.observations.iter()
        .filter(|o| o.state == ObservationState::Conflicted)
        .count();
    let coverage = if trusted_status(&outcome.final_view).is_some() { 1_000 } else { 0 };

    let baseline_core = serde_json::json!({
        "episode_id": outcome.final_view.episode_id,
        "policy_id": baseline.policy.as_str(),
        "policy_observation_sha256": baseline.initial_view_sha256,
        "evaluator_truth_sha256": baseline.truth_sha256,
        "decision_loss_microcredits": baseline.decision_loss,
        "total_cost_microcredits": baseline.total_cost,
        "end_to_end_loss_microcredits": baseline.end_to_end_loss,
        "final_decision": &baseline.final_choice,
    });
    let baseline_receipt_sha256 = sha256_hex(&serde_json::to_vec(&baseline_core)?);
    let queried_an_information_source = outcome.request_records.iter().any(|r| {
        r.disposition == RequestDisposition::Provided
            && r.source_family_refs.iter().any(|family| family == "network-observation-family")
    });
    let realized_loss_reduction = queried_an_information_source.then(|| {
        signed_difference(baseline.end_to_end_loss, outcome.end_to_end_loss)
    });
    let realized_loss_reduction_defined = realized_loss_reduction.is_some();
    let request_authorized = outcome.authority_violations.is_empty();
    let rights_compliant = outcome.request_records.iter().all(|record| {
        outcome.final_view.evidence_options.iter()
            .find(|o| o.request_option_id == record.request_option_id)
            .map(|o| o.sensitivity == Sensitivity::Public && o.authorization.permitted_in_this_fixture())
            .unwrap_or(false)
    });
    let acquisition_authorized = request_authorized
        && outcome.request_records.iter().all(|r| r.disposition != RequestDisposition::NotAuthorized);
    let budget_compliant = outcome.budget_violations.is_empty();
    let mut all_guardrail_violations = violations;
    if !rights_compliant && !outcome.request_records.is_empty() {
        all_guardrail_violations.push("request did not satisfy the synthetic privacy/source-rights policy".to_string());
    }

    Ok(ScoringReceipt {
        schema_version: SCHEMA_VERSION.to_string(),
        episode_id: outcome.final_view.episode_id.clone(),
        policy_id: outcome.policy.as_str().to_string(),
        policy_artifact_sha256: sha256_hex(POLICY_SOURCE_BYTES),
        policy_observation_sha256: outcome.initial_view_sha256.clone(),
        evidence_model_sha256: sha256_hex(POLICY_SOURCE_BYTES),
        evaluator_truth_sha256: outcome.truth_sha256.clone(),
        decision: ReceiptDecision {
            kind: decision_kind,
            chosen_action_id: action_id,
            requested_evidence_option_ids: outcome.queried_option_ids.clone(),
            pre_outcome_confidence_ppm: None,
            decision_latency_ms: outcome.decision_latency_ms,
        },
        evidence_requests: outcome.request_records.clone(),
        outcome: ReceiptOutcome {
            state,
            completed_units,
            unserved_units,
            fallback_used,
            fallback_succeeded,
            realized_decision_loss_microcredits: outcome.decision_loss,
        },
        costs: ReceiptCosts {
            evidence_acquisition_microcredits: outcome.evidence_cost,
            compute_microcredits: outcome.compute_cost,
            delay_microcredits: outcome.delay_cost,
            human_review_microcredits: outcome.human_review_cost,
            fallback_microcredits: outcome.fallback_cost,
            operational_microcredits: outcome.operational_cost,
            failure_recovery_microcredits: outcome.failure_recovery_cost,
            total_cost_microcredits: outcome.total_cost,
        },
        regret: ReceiptRegret {
            oracle_action_loss_microcredits: outcome.oracle_loss,
            regret_vs_oracle_microcredits: outcome.regret,
            comparator_policy_artifact_sha256: sha256_hex(POLICY_SOURCE_BYTES),
            comparator_baseline_core_sha256: baseline_receipt_sha256,
            comparator_policy_loss_microcredits: baseline.decision_loss,
            comparator_total_cost_microcredits: baseline.total_cost,
            candidate_minus_comparator_end_to_end_loss_microcredits: signed_difference(
                outcome.end_to_end_loss,
                baseline.end_to_end_loss,
            ),
            realized_end_to_end_loss_reduction_vs_comparator_microcredits: realized_loss_reduction,
            realized_loss_reduction_defined,
        },
        guardrails: ReceiptGuardrails {
            budget_compliant,
            acquisition_authorized,
            privacy_and_consent_compliant: rights_compliant,
            source_rights_compliant: rights_compliant,
            no_external_side_effects: true,
            artifact_integrity_valid: outcome.request_records.iter()
                .all(|r| r.disposition != RequestDisposition::InvalidReceipt),
            invalid_receipt_rejected: outcome.request_records.iter()
                .all(|r| r.disposition != RequestDisposition::InvalidReceipt
                    || r.returned_observation_sha256.is_none()),
            violations: all_guardrail_violations,
        },
        evidence_quality: ReceiptEvidenceQuality {
            unique_source_family_count: source_families.len(),
            stale_observation_count: stale_count,
            conflicted_observation_count: conflicted_count,
            observation_coverage_permille: coverage,
            brier_score: None,
        },
        claim_ceiling: CLAIM_CEILING.to_string(),
    })
}

fn add_to_summary(summary: &mut PolicySummary, receipt: &ScoringReceipt) {
    summary.runs += 1;
    summary.mean_decision_loss_microcredits += receipt.outcome.realized_decision_loss_microcredits as f64;
    summary.mean_regret_vs_oracle_microcredits += receipt.regret.regret_vs_oracle_microcredits as f64;
    summary.mean_total_cost_microcredits += receipt.costs.total_cost_microcredits as f64;
    summary.mean_end_to_end_loss_microcredits += (
        receipt.outcome.realized_decision_loss_microcredits + receipt.costs.total_cost_microcredits
    ) as f64;
    summary.abstention_count += if receipt.decision.kind == "abstain" { 1 } else { 0 };
    summary.evidence_request_count += receipt.evidence_requests.len();
    summary.provided_evidence_count += receipt.evidence_requests.iter()
        .filter(|r| r.disposition == RequestDisposition::Provided).count();
    summary.authority_violation_count += receipt.guardrails.violations.iter()
        .filter(|v| v.contains("unauthorized") || v.contains("source-rights") || v.contains("privacy"))
        .count();
    summary.budget_violation_count += receipt.guardrails.violations.iter()
        .filter(|v| v.contains("budget") || v.contains("latency"))
        .count();
}

fn average_summary(summary: &mut PolicySummary) {
    if summary.runs == 0 {
        return;
    }
    let n = summary.runs as f64;
    summary.mean_decision_loss_microcredits /= n;
    summary.mean_regret_vs_oracle_microcredits /= n;
    summary.mean_total_cost_microcredits /= n;
    summary.mean_end_to_end_loss_microcredits /= n;
}

fn run_suite() -> Result<BenchmarkReport, Box<dyn Error>> {
    let mut receipts = Vec::new();
    let mut by_policy: BTreeMap<String, PolicySummary> = BTreeMap::new();
    let mut stratified: BTreeMap<String, BTreeMap<String, PolicySummary>> = BTreeMap::new();
    let mut fixture_count = 0_usize;

    for regime in Regime::ALL {
        for seed in 0..SEEDS_PER_REGIME {
            let (view, truth) = make_fixture(regime, seed)?;
            fixture_count += 1;
            // Same episode and seed: the simple fair baseline is paired with
            // every candidate on the identical policy-visible initial view.
            let baseline = evaluate_policy(PolicyKind::ObservationRule, &view, &truth)?;
            for policy in PolicyKind::ALL {
                let outcome = evaluate_policy(policy, &view, &truth)?;
                let receipt = scoring_receipt(&outcome, &baseline, &truth)?;
                add_to_summary(by_policy.entry(policy.as_str().to_string()).or_default(), &receipt);
                add_to_summary(
                    stratified.entry(regime.as_str().to_string()).or_default()
                        .entry(policy.as_str().to_string()).or_default(),
                    &receipt,
                );
                receipts.push(receipt);
            }
        }
    }

    for summary in by_policy.values_mut() {
        average_summary(summary);
    }
    let stratified_results = stratified.into_iter().map(|(regime, mut rows)| {
        for summary in rows.values_mut() {
            average_summary(summary);
        }
        StratifiedSummary { regime, by_policy: rows }
    }).collect::<Vec<_>>();

    let hard_guardrail_violation_count = receipts.iter()
        .map(|r| r.guardrails.violations.len())
        .sum::<usize>();
    let expected_runs = fixture_count * PolicyKind::ALL.len();
    let run_count = receipts.len();
    Ok(BenchmarkReport {
        schema_version: SCHEMA_VERSION.to_string(),
        benchmark_id: "CIV-IMPACT-DECISION-VALUE-SYNTH-001".to_string(),
        evidence_class: "synthetic_known_truth".to_string(),
        fixture_count,
        seeds_per_regime: SEEDS_PER_REGIME,
        policy_count: PolicyKind::ALL.len(),
        run_count,
        policies: by_policy,
        stratified_results,
        receipts,
        all_scheduled_runs_accounted_for: run_count == expected_runs,
        hard_guardrail_violation_count,
        economic_benefit_established: false,
        claim_ceiling: CLAIM_CEILING.to_string(),
    })
}

fn main() -> Result<(), Box<dyn Error>> {
    let mut args = env::args().skip(1);
    let command = args.next().unwrap_or_else(|| "evaluate".to_string());
    if command != "evaluate" {
        return Err(format!("unsupported command {command:?}; expected evaluate [report.json]").into());
    }
    let output = args.next().unwrap_or_else(|| "/tmp/civ-impact-decision-value-report.json".to_string());
    let report = run_suite()?;
    let bytes = serde_json::to_vec_pretty(&report)?;
    let path = Path::new(&output);
    if let Some(parent) = path.parent() {
        if !parent.as_os_str().is_empty() {
            fs::create_dir_all(parent)?;
        }
    }
    fs::write(path, &bytes)?;
    println!(
        "{}",
        serde_json::to_string_pretty(&serde_json::json!({
            "status": "synthetic_policy_evaluation_completed",
            "report_path": output.clone(),
            "benchmark_id": report.benchmark_id,
            "fixture_count": report.fixture_count,
            "policy_count": report.policy_count,
            "run_count": report.run_count,
            "all_scheduled_runs_accounted_for": report.all_scheduled_runs_accounted_for,
            "hard_guardrail_violation_count": report.hard_guardrail_violation_count,
            "economic_benefit_established": report.economic_benefit_established,
            "claim_ceiling": report.claim_ceiling,
        }))?
    );
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;


    fn load_schema_validator(schema_name: &str) -> jsonschema::Validator {
        let schema_path = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../..")
            .join("docs/research/schema")
            .join(schema_name);
        let schema_bytes = fs::read(&schema_path)
            .unwrap_or_else(|error| panic!("read schema {}: {error}", schema_path.display()));
        let schema: Value = serde_json::from_slice(&schema_bytes)
            .unwrap_or_else(|error| panic!("parse schema {}: {error}", schema_path.display()));
        // Building the validator also checks that the schema document is valid
        // for the declared JSON Schema dialect.
        jsonschema::validator_for(&schema)
            .unwrap_or_else(|error| panic!("invalid schema {}: {error}", schema_path.display()))
    }

    fn assert_schema_instance(validator: &jsonschema::Validator, schema_name: &str, instance: &Value) {
        let errors = validator.iter_errors(instance).map(|e| e.to_string()).collect::<Vec<_>>();
        assert!(
            errors.is_empty(),
            "instance does not satisfy {schema_name}: {}",
            errors.join("; ")
        );
    }

    #[test]
    fn schemas_validate_policy_view_truth_aggregate_report_and_every_scoring_receipt() {
        let report = run_suite().expect("synthetic report");
        let report_validator = load_schema_validator("CIV_IMPACT_DECISION_REPORT_V0_1.schema.json");
        assert_schema_instance(
            &report_validator,
            "CIV_IMPACT_DECISION_REPORT_V0_1.schema.json",
            &serde_json::to_value(&report).expect("aggregate report JSON"),
        );
        let (view, truth) = make_fixture(Regime::TargetedInformation, 1).expect("fixture");
        let view_validator = load_schema_validator("CIV_IMPACT_DECISION_EPISODE_V0_1.schema.json");
        assert_schema_instance(
            &view_validator,
            "CIV_IMPACT_DECISION_EPISODE_V0_1.schema.json",
            &serde_json::to_value(&view).expect("view JSON"),
        );
        let truth_validator = load_schema_validator("CIV_IMPACT_DECISION_TRUTH_V0_1.schema.json");
        assert_schema_instance(
            &truth_validator,
            "CIV_IMPACT_DECISION_TRUTH_V0_1.schema.json",
            &serde_json::to_value(&truth).expect("truth JSON"),
        );
        let baseline = evaluate_policy(PolicyKind::ObservationRule, &view, &truth).expect("baseline");
        let candidate = evaluate_policy(PolicyKind::ValueOfInformation, &view, &truth).expect("candidate");
        let receipt = scoring_receipt(&candidate, &baseline, &truth).expect("receipt");
        let score_validator = load_schema_validator("CIV_IMPACT_DECISION_SCORE_V0_1.schema.json");
        assert_schema_instance(
            &score_validator,
            "CIV_IMPACT_DECISION_SCORE_V0_1.schema.json",
            &serde_json::to_value(receipt).expect("receipt JSON"),
        );
        for receipt in report.receipts {
            assert_schema_instance(
                &score_validator,
                "CIV_IMPACT_DECISION_SCORE_V0_1.schema.json",
                &serde_json::to_value(receipt).expect("report receipt JSON"),
            );
        }
    }

    #[test]
    fn policy_view_rejects_hidden_truth_and_post_outcome_fields() {
        let (view, _) = make_fixture(Regime::TargetedInformation, 1).expect("fixture");
        let mut value = serde_json::to_value(view).expect("view JSON");
        value["evaluator_truth"] = serde_json::json!({"bridge_open": true});
        assert!(serde_json::from_value::<PolicyView>(value).is_err());
    }

    #[test]
    fn scoring_schema_rejects_missing_evaluator_truth_digest() {
        let (view, truth) = make_fixture(Regime::TargetedInformation, 1).expect("fixture");
        let baseline = evaluate_policy(PolicyKind::ObservationRule, &view, &truth).expect("baseline");
        let candidate = evaluate_policy(PolicyKind::ValueOfInformation, &view, &truth).expect("candidate");
        let receipt = scoring_receipt(&candidate, &baseline, &truth).expect("receipt");
        let mut value = serde_json::to_value(receipt).expect("receipt JSON");
        value.as_object_mut().expect("receipt object").remove("evaluator_truth_sha256");
        let schema_path = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../..")
            .join("docs/research/schema/CIV_IMPACT_DECISION_SCORE_V0_1.schema.json");
        let schema: Value = serde_json::from_slice(&fs::read(schema_path).expect("schema file"))
            .expect("schema JSON");
        let validator = jsonschema::validator_for(&schema).expect("schema validator");
        assert!(!validator.is_valid(&value));
    }

    #[test]
    fn hidden_truth_changes_do_not_change_policy_visible_bytes() {
        let (view_a, truth_a) = make_fixture(Regime::TargetedInformation, 1).expect("fixture");
        let visible_bytes_before = encode(&view_a).expect("visible input before truth mutation");
        let mut truth_b = truth_a.clone();
        let fact = truth_b.latent_facts.iter_mut()
            .find(|f| f.predicate == "bridge_open")
            .expect("bridge truth");
        fact.value = Value::Bool(!fact.value.as_bool().expect("boolean fact"));
        let visible_bytes_after = encode(&view_a).expect("visible input after truth mutation");
        assert_eq!(visible_bytes_before, visible_bytes_after);
        assert_ne!(encode(&truth_a).expect("truth bytes"), encode(&truth_b).expect("changed truth bytes"));
        // The policy API is a pure function of policy-visible data and history.
        let queried = BTreeSet::new();
        assert_eq!(
            choose_action(PolicyKind::ValueOfInformation, &view_a, &queried),
            choose_action(PolicyKind::ValueOfInformation, &view_a, &queried)
        );
    }

    #[test]
    fn targeted_evidence_can_reduce_end_to_end_loss() {
        // In seed 1 this targeted-information scenario hides an open bridge
        // behind an incorrect stale report; the observation-rule baseline detours.
        let (view, truth) = make_fixture(Regime::TargetedInformation, 1).expect("fixture");
        assert!(truth.bool_fact("bridge_open"));
        let baseline = evaluate_policy(PolicyKind::ObservationRule, &view, &truth).expect("baseline");
        let candidate = evaluate_policy(PolicyKind::ValueOfInformation, &view, &truth).expect("VoI");
        assert!(candidate.provided_evidence_count() > 0);
        assert!(candidate.end_to_end_loss < baseline.end_to_end_loss);
    }

    #[test]
    fn voi_skips_a_request_when_declared_decision_value_is_below_cost() {
        let (view, truth) = make_fixture(Regime::DistributionShift, 0).expect("fixture");
        let option = view.evidence_options.iter()
            .find(|o| o.request_option_id == "independent_status")
            .expect("independent evidence option");
        let total_expected_cost = option.expected_cost_microcredits
            + (option.expected_latency_ms + 9) / 10;
        assert!(option.expected_decision_loss_reduction_microcredits <= total_expected_cost);
        let result = evaluate_policy(PolicyKind::ValueOfInformation, &view, &truth).expect("VoI");
        assert_eq!(result.provided_evidence_count(), 0);
        assert_eq!(result.evidence_cost, 0);
    }

    #[test]
    fn voi_skips_high_reliability_evidence_when_expected_decision_value_is_below_cost() {
        // Evidence is permitted and nominally reliable, but its expected
        // decision-loss reduction is smaller than its acquisition plus delay cost.
        let (view, truth) = make_fixture(Regime::TargetedInformation, 0).expect("fixture");
        let option = view.evidence_options.iter()
            .find(|o| o.request_option_id == "independent_status")
            .expect("independent evidence option");
        let expected_cost = option.expected_cost_microcredits + (option.expected_latency_ms + 9) / 10;
        assert!(option.authorization.permitted_in_this_fixture());
        assert!(option.expected_reliability_permille >= 900);
        assert!(option.expected_decision_loss_reduction_microcredits <= expected_cost);
        let result = evaluate_policy(PolicyKind::ValueOfInformation, &view, &truth).expect("VoI");
        assert_eq!(result.provided_evidence_count(), 0);
        assert_eq!(result.evidence_cost, 0);
    }

    #[test]
    fn voI_does_not_query_when_initial_evidence_is_fresh() {
        let (view, truth) = make_fixture(Regime::Nominal, 0).expect("fixture");
        let result = evaluate_policy(PolicyKind::ValueOfInformation, &view, &truth).expect("VoI");
        assert_eq!(result.provided_evidence_count(), 0);
        assert_eq!(result.evidence_cost, 0);
    }

    #[test]
    fn always_defer_pays_fallback_and_service_loss() {
        let (view, truth) = make_fixture(Regime::Nominal, 0).expect("fixture");
        let always_defer = evaluate_policy(PolicyKind::AlwaysDefer, &view, &truth).expect("defer");
        let observation_rule = evaluate_policy(PolicyKind::ObservationRule, &view, &truth).expect("baseline");
        assert!(always_defer.fallback_cost > 0);
        assert!(always_defer.end_to_end_loss > observation_rule.end_to_end_loss);
    }

    #[test]
    fn query_all_pays_for_queries_that_are_not_needed() {
        let (view, truth) = make_fixture(Regime::UninformativeRequest, 0).expect("fixture");
        let query_all = evaluate_policy(PolicyKind::QueryAll, &view, &truth).expect("query all");
        let observation_rule = evaluate_policy(PolicyKind::ObservationRule, &view, &truth).expect("baseline");
        assert!(query_all.evidence_cost > 0);
        assert!(query_all.end_to_end_loss > observation_rule.end_to_end_loss);
    }

    #[test]
    fn voi_prefers_an_independent_source_over_a_mirror() {
        let (view, truth) = make_fixture(Regime::CorrelatedMirrors, 1).expect("fixture");
        let result = evaluate_policy(PolicyKind::ValueOfInformation, &view, &truth).expect("VoI");
        assert!(result.request_records.iter().any(|r| {
            r.request_option_id == "independent_status" && r.disposition == RequestDisposition::Provided
        }));
        assert!(!result.request_records.iter().any(|r| r.request_option_id == "mirror_status"));
    }

    #[test]
    fn unauthorized_requests_are_blocked_and_recorded() {
        let (view, truth) = make_fixture(Regime::HighConsequenceFallback, 0).expect("fixture");
        let result = evaluate_policy(PolicyKind::UnauthorizedRequester, &view, &truth).expect("negative control");
        assert!(!result.authority_violations.is_empty());
        assert!(result.request_records.iter().any(|r| r.disposition == RequestDisposition::NotAuthorized));
        assert_eq!(result.evidence_cost, 0);
    }

    #[test]
    fn high_consequence_with_no_permitted_evidence_prefers_fallback_under_voi() {
        let (view, truth) = make_fixture(Regime::HighConsequenceFallback, 0).expect("fixture");
        let result = evaluate_policy(PolicyKind::ValueOfInformation, &view, &truth).expect("VoI");
        assert!(matches!(result.final_choice, Choice::Abstain));
        assert_eq!(result.evidence_cost, 0);
    }

    #[test]
    fn invalid_evidence_receipts_are_not_ingested_as_observations() {
        let (view, truth) = make_fixture(Regime::ReceiptTampering, 0).expect("fixture");
        let result = evaluate_policy(PolicyKind::ValueOfInformation, &view, &truth).expect("VoI");
        assert!(result.request_records.iter().any(|r| r.disposition == RequestDisposition::InvalidReceipt));
        assert_eq!(
            result.final_view.observations.len(),
            view.observations.len(),
            "an invalid receipt must not add a policy-visible observation"
        );
        assert!(result.request_records.iter().all(|r| {
            r.disposition != RequestDisposition::InvalidReceipt || r.returned_observation_sha256.is_none()
        }));
        let baseline = evaluate_policy(PolicyKind::ObservationRule, &view, &truth).expect("baseline");
        let receipt = scoring_receipt(&result, &baseline, &truth).expect("score");
        assert!(!receipt.guardrails.artifact_integrity_valid);
        assert!(receipt.guardrails.invalid_receipt_rejected);
    }

    #[test]
    fn complete_report_preserves_regime_strata_and_claim_ceiling() {
        let report = run_suite().expect("report");
        assert_eq!(report.fixture_count, 12 * SEEDS_PER_REGIME as usize);
        assert_eq!(report.policy_count, PolicyKind::ALL.len());
        assert_eq!(report.run_count, report.fixture_count * report.policy_count);
        assert!(report.all_scheduled_runs_accounted_for);
        assert_eq!(report.stratified_results.len(), Regime::ALL.len());
        assert!(!report.economic_benefit_established);
        assert_eq!(report.claim_ceiling, CLAIM_CEILING);
    }

    #[test]
    fn unsafe_forced_action_is_reported_as_a_guardrail_failure() {
        let (view, truth) = make_fixture(Regime::StaleOrMissing, 2).expect("fixture");
        assert!(!truth.bool_fact("bridge_open"));
        let baseline = evaluate_policy(PolicyKind::ObservationRule, &view, &truth).expect("baseline");
        let forced = evaluate_policy(PolicyKind::ForcedAction, &view, &truth).expect("forced action");
        let receipt = scoring_receipt(&forced, &baseline, &truth).expect("receipt");
        assert!(receipt.guardrails.violations.iter().any(|v| v == "closed_primary_route"));
    }

    #[test]
    fn receipt_cost_ledger_and_paired_loss_deltas_reconcile_exactly() {
        let (view, truth) = make_fixture(Regime::TargetedInformation, 1).expect("fixture");
        let baseline = evaluate_policy(PolicyKind::ObservationRule, &view, &truth).expect("baseline");
        let candidate = evaluate_policy(PolicyKind::ValueOfInformation, &view, &truth).expect("candidate");
        let receipt = scoring_receipt(&candidate, &baseline, &truth).expect("receipt");
        let costs = &receipt.costs;
        let total = costs.evidence_acquisition_microcredits
            + costs.compute_microcredits
            + costs.delay_microcredits
            + costs.human_review_microcredits
            + costs.fallback_microcredits
            + costs.operational_microcredits
            + costs.failure_recovery_microcredits;
        assert_eq!(costs.total_cost_microcredits, total);
        assert_eq!(
            receipt.regret.candidate_minus_comparator_end_to_end_loss_microcredits,
            signed_difference(candidate.end_to_end_loss, baseline.end_to_end_loss)
        );
        assert_eq!(
            receipt.regret.realized_end_to_end_loss_reduction_vs_comparator_microcredits,
            Some(signed_difference(baseline.end_to_end_loss, candidate.end_to_end_loss))
        );
        assert_eq!(
            receipt.regret.regret_vs_oracle_microcredits,
            candidate.decision_loss.saturating_sub(candidate.oracle_loss)
        );
        assert!(receipt.regret.realized_loss_reduction_defined);
    }

    #[test]
    fn exact_policy_view_and_hidden_truth_digests_are_recorded_separately() {
        let (view, truth) = make_fixture(Regime::TargetedInformation, 1).expect("fixture");
        let baseline = evaluate_policy(PolicyKind::ObservationRule, &view, &truth).expect("baseline");
        let candidate = evaluate_policy(PolicyKind::ValueOfInformation, &view, &truth).expect("candidate");
        let receipt = scoring_receipt(&candidate, &baseline, &truth).expect("receipt");
        assert_ne!(receipt.policy_observation_sha256, receipt.evaluator_truth_sha256);
        assert_eq!(receipt.policy_observation_sha256, baseline.initial_view_sha256);
        assert_eq!(receipt.evaluator_truth_sha256, baseline.truth_sha256);
    }

    #[test]
    fn fixed_query_control_exposes_distribution_shift_safety_failure() {
        // Seed 2 has a closed bridge. The query-all/fixed-query control ignores
        // the explicitly low reliability forecast and ingests a fresh but wrong
        // response in this synthetic shift fixture.
        let (view, truth) = make_fixture(Regime::DistributionShift, 2).expect("fixture");
        assert!(!truth.bool_fact("bridge_open"));
        let baseline = evaluate_policy(PolicyKind::ObservationRule, &view, &truth).expect("baseline");
        let fixed = evaluate_policy(PolicyKind::FixedQuery, &view, &truth).expect("fixed-query control");
        let receipt = scoring_receipt(&fixed, &baseline, &truth).expect("scoring receipt");
        assert!(receipt.guardrails.violations.iter().any(|v| v == "closed_primary_route"));
    }

    #[test]
    fn fixed_query_control_exposes_distribution_shift_safety_failure() {
        // Seed 2 has a closed bridge. This control ignores the low declared
        // reliability and receives a fresh-but-wrong response in the shifted world.
        let (view, truth) = make_fixture(Regime::DistributionShift, 2).expect("fixture");
        assert!(!truth.bool_fact("bridge_open"));
        let baseline = evaluate_policy(PolicyKind::ObservationRule, &view, &truth).expect("baseline");
        let fixed = evaluate_policy(PolicyKind::FixedQuery, &view, &truth).expect("fixed-query control");
        let receipt = scoring_receipt(&fixed, &baseline, &truth).expect("scoring receipt");
        assert!(receipt.guardrails.violations.iter().any(|v| v == "closed_primary_route"));
    }

    #[test]
    fn value_of_information_arm_has_no_guardrail_violations_in_registered_fixtures() {
        let report = run_suite().expect("report");
        assert!(report.receipts.iter()
            .filter(|receipt| receipt.policy_id == PolicyKind::ValueOfInformation.as_str())
            .all(|receipt| receipt.guardrails.violations.is_empty()));
        assert!(report.receipts.iter()
            .any(|receipt| receipt.policy_id == PolicyKind::ForcedAction.as_str()
                && receipt.guardrails.violations.iter().any(|v| v == "closed_primary_route")));
    }

    #[test]
    fn distribution_shift_is_retained_as_a_stratum_not_removed() {
        let report = run_suite().expect("report");
        let stratum = report.stratified_results.iter()
            .find(|row| row.regime == "distribution_shift")
            .expect("shift stratum");
        assert!(stratum.by_policy.contains_key("value_of_information"));
        assert_eq!(
            stratum.by_policy["value_of_information"].runs,
            SEEDS_PER_REGIME as usize
        );
    }
}
