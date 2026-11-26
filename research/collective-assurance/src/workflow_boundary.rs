//! Task-conditioned collaborative-memory boundary qualification.
//!
//! This module checks whether typed artifacts cross only their declared
//! producer→consumer boundaries and whether a matched isolation challenge is
//! rejected at an otherwise-solvable consumer. It complements RER: semantic
//! relevance does not grant task-graph handoff authority.
//!
//! It validates declared contracts and receipts; it does not prove artifact
//! truth, authenticate upstream receipt producers, or supply a workflow runner.

use std::collections::{BTreeMap, BTreeSet};

use sha2::{Digest, Sha256};

fn nonempty(value: &str) -> bool {
    !value.trim().is_empty()
}

fn valid_digest(value: &str) -> bool {
    value.len() == 64
        && value
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}

fn push_len(out: &mut Vec<u8>, value: usize) {
    out.extend_from_slice(&(value as u64).to_be_bytes());
}

fn push_str(out: &mut Vec<u8>, value: &str) {
    push_len(out, value.len());
    out.extend_from_slice(value.as_bytes());
}

fn push_u64(out: &mut Vec<u8>, value: u64) {
    out.extend_from_slice(&value.to_be_bytes());
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

const GRAPH_DOMAIN: &[u8] = b"MYCELIX-WORKFLOW-BOUNDARY-GRAPH-V0.1\0";

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct WorkflowIdentity {
    pub benchmark_id: String,
    pub benchmark_version: String,
    pub benchmark_certificate_digest: String,
    pub split_digest: String,
    pub workflow_id: String,
    pub task_digest: String,
    pub source_group_id: String,
    pub workflow_graph_digest: String,
    pub model_snapshot_digest: String,
    pub toolchain_digest: String,
    pub executor_digest: String,
    pub scorer_digest: String,
    pub evaluator_digest: String,
    pub inference_plan_digest: String,
    pub random_seed_digest: String,
    pub context_budget_tokens: u64,
    pub output_budget_tokens: u64,
    pub compute_budget_microusd: u64,
    pub wall_clock_budget_ms: u64,
    pub memory_budget_bytes: u64,
}

#[derive(Clone, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub struct WorkflowNode {
    pub node_id: String,
    pub task_digest: String,
}

#[derive(Clone, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub struct ArtifactHandoffContract {
    pub edge_id: String,
    pub producer_node_id: String,
    pub consumer_node_id: String,
    pub artifact_type: String,
    pub entity_scope_digest: String,
    pub required_version: u64,
    pub verification_required: bool,
    pub required: bool,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ArtifactLifecycle {
    Current,
    Superseded,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ArtifactVerification {
    Verified,
    Unverified,
    Rejected,
    Unknown,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ArtifactOperationState {
    NotOperation,
    Pending,
    Completed,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct WorkflowArtifact {
    pub artifact_id: String,
    pub payload_digest: String,
    pub producer_node_id: String,
    pub artifact_type: String,
    pub entity_scope_digest: String,
    pub version: u64,
    pub lifecycle: ArtifactLifecycle,
    pub verification: ArtifactVerification,
    pub verification_receipt_digest: Option<String>,
    pub operation_state: ArtifactOperationState,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ArtifactExposureDisposition {
    Consumed,
    RejectedInapplicable,
    RejectedStale,
    RejectedUnverified,
    RejectedWrongEntity,
    RejectedWrongBranch,
    RejectedCompletedAsPending,
    Unknown,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct WorkflowArtifactExposure {
    pub exposure_id: String,
    pub consumer_node_id: String,
    pub artifact_id: String,
    /// What the consumer/runtime claimed about the operation state. This
    /// catches a completed side effect represented as pending work.
    pub reported_operation_state: ArtifactOperationState,
    pub disposition: ArtifactExposureDisposition,
    pub exposure_receipt_digest: String,
    pub effect_receipt_digest: Option<String>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct WorkflowNodeOutcome {
    pub node_id: String,
    pub verified_complete: bool,
    pub node_execution_receipt_digest: String,
}

#[derive(Clone, Debug, PartialEq)]
pub struct WorkflowRun {
    pub run_id: String,
    pub identity: WorkflowIdentity,
    pub nodes: Vec<WorkflowNode>,
    pub handoff_contracts: Vec<ArtifactHandoffContract>,
    pub artifacts: Vec<WorkflowArtifact>,
    pub exposures: Vec<WorkflowArtifactExposure>,
    pub node_outcomes: Vec<WorkflowNodeOutcome>,
    pub terminal_success: bool,
    pub execution_receipt_digest: String,
    pub exposure_trace_receipt_digest: String,
    pub evaluator_receipt_digest: String,
    pub latency_ms: u64,
    pub total_cost_microusd: u64,
    pub input_tokens: u64,
    pub output_tokens: u64,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BoundaryPollutionOperator {
    SiblingCrosswire,
    IncompatiblePayload,
    UnverifiedResult,
    CompletedAsPending,
    WrongEntity,
    StaleVersion,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct BoundaryIsolationChallenge {
    pub challenge_id: String,
    pub target_consumer_node_id: String,
    pub polluted_artifact_id: String,
    pub operator: BoundaryPollutionOperator,
}

#[derive(Clone, Debug, PartialEq)]
pub struct WorkflowBoundaryPair {
    pub pair_id: String,
    pub clean: WorkflowRun,
    pub polluted: WorkflowRun,
    pub challenge: BoundaryIsolationChallenge,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum WorkflowBoundaryStatus {
    Pass,
    Fail,
    Indeterminate,
}

#[derive(Clone, Debug, PartialEq)]
pub struct WorkflowBoundaryMetrics {
    pub terminal_success: bool,
    pub node_count: usize,
    pub verified_completed_node_count: usize,
    pub verified_node_completion_rate: Option<f64>,
    pub required_handoff_count: usize,
    pub successful_required_handoff_count: usize,
    pub verified_handoff_success_rate: Option<f64>,
    pub exposure_count: usize,
    pub consumed_exposure_count: usize,
    pub rejected_exposure_count: usize,
    pub unknown_exposure_count: usize,
    pub latency_ms: u64,
    pub total_cost_microusd: u64,
    pub input_tokens: u64,
    pub output_tokens: u64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct WorkflowBoundaryReport {
    pub status: WorkflowBoundaryStatus,
    pub pair_id: String,
    pub operator: BoundaryPollutionOperator,
    pub clean_target_succeeded: bool,
    pub pollution_was_exposed: bool,
    pub pollution_was_consumed: bool,
    pub pollution_disposition: Option<ArtifactExposureDisposition>,
    pub clean_metrics: WorkflowBoundaryMetrics,
    pub polluted_metrics: WorkflowBoundaryMetrics,
    pub latency_delta_ms: i128,
    pub total_cost_delta_microusd: i128,
    pub reasons: Vec<String>,
}

fn empty_metrics(run: &WorkflowRun) -> WorkflowBoundaryMetrics {
    WorkflowBoundaryMetrics {
        terminal_success: run.terminal_success,
        node_count: run.nodes.len(),
        verified_completed_node_count: 0,
        verified_node_completion_rate: None,
        required_handoff_count: 0,
        successful_required_handoff_count: 0,
        verified_handoff_success_rate: None,
        exposure_count: run.exposures.len(),
        consumed_exposure_count: 0,
        rejected_exposure_count: 0,
        unknown_exposure_count: 0,
        latency_ms: run.latency_ms,
        total_cost_microusd: run.total_cost_microusd,
        input_tokens: run.input_tokens,
        output_tokens: run.output_tokens,
    }
}

fn report(
    status: WorkflowBoundaryStatus,
    pair: &WorkflowBoundaryPair,
    reasons: Vec<String>,
) -> WorkflowBoundaryReport {
    WorkflowBoundaryReport {
        status,
        pair_id: pair.pair_id.clone(),
        operator: pair.challenge.operator,
        clean_target_succeeded: false,
        pollution_was_exposed: false,
        pollution_was_consumed: false,
        pollution_disposition: None,
        clean_metrics: empty_metrics(&pair.clean),
        polluted_metrics: empty_metrics(&pair.polluted),
        latency_delta_ms: i128::from(pair.polluted.latency_ms)
            - i128::from(pair.clean.latency_ms),
        total_cost_delta_microusd: i128::from(pair.polluted.total_cost_microusd)
            - i128::from(pair.clean.total_cost_microusd),
        reasons,
    }
}

/// Computes a deterministic digest over sorted workflow nodes and handoff contracts.
pub fn compute_workflow_graph_digest(
    nodes: &[WorkflowNode],
    contracts: &[ArtifactHandoffContract],
) -> Result<String, String> {
    let mut sorted_nodes = nodes.iter().collect::<Vec<_>>();
    sorted_nodes.sort();
    let mut sorted_contracts = contracts.iter().collect::<Vec<_>>();
    sorted_contracts.sort();

    let mut node_ids = BTreeSet::new();
    let mut edge_ids = BTreeSet::new();
    let mut contract_keys = BTreeSet::new();
    let mut bytes = Vec::new();
    bytes.extend_from_slice(GRAPH_DOMAIN);

    push_len(&mut bytes, sorted_nodes.len());
    for node in sorted_nodes {
        if !nonempty(&node.node_id)
            || !valid_digest(&node.task_digest)
            || !node_ids.insert(node.node_id.as_str())
        {
            return Err("workflow node identity is invalid or duplicated".into());
        }
        push_str(&mut bytes, &node.node_id);
        push_str(&mut bytes, &node.task_digest);
    }

    push_len(&mut bytes, sorted_contracts.len());
    for edge in sorted_contracts {
        if !nonempty(&edge.edge_id)
            || !nonempty(&edge.producer_node_id)
            || !nonempty(&edge.consumer_node_id)
            || edge.producer_node_id == edge.consumer_node_id
            || !nonempty(&edge.artifact_type)
            || !valid_digest(&edge.entity_scope_digest)
            || !edge_ids.insert(edge.edge_id.as_str())
            || !contract_keys.insert((
                edge.producer_node_id.as_str(),
                edge.consumer_node_id.as_str(),
                edge.artifact_type.as_str(),
                edge.entity_scope_digest.as_str(),
            ))
            || !node_ids.contains(edge.producer_node_id.as_str())
            || !node_ids.contains(edge.consumer_node_id.as_str())
            || edge.required_version == 0
        {
            return Err("artifact handoff contract identity is invalid".into());
        }
        push_str(&mut bytes, &edge.edge_id);
        push_str(&mut bytes, &edge.producer_node_id);
        push_str(&mut bytes, &edge.consumer_node_id);
        push_str(&mut bytes, &edge.artifact_type);
        push_str(&mut bytes, &edge.entity_scope_digest);
        push_u64(&mut bytes, edge.required_version);
        bytes.push(u8::from(edge.verification_required));
        bytes.push(u8::from(edge.required));
    }
    Ok(sha256_hex(&bytes))
}

fn validate_identity(identity: &WorkflowIdentity) -> Result<(), String> {
    for (name, value) in [
        ("benchmark ID", identity.benchmark_id.as_str()),
        ("benchmark version", identity.benchmark_version.as_str()),
        ("workflow ID", identity.workflow_id.as_str()),
        ("source group ID", identity.source_group_id.as_str()),
    ] {
        if !nonempty(value) {
            return Err(format!("{name} must be non-empty"));
        }
    }
    for (name, value) in [
        ("benchmark certificate", identity.benchmark_certificate_digest.as_str()),
        ("split", identity.split_digest.as_str()),
        ("task", identity.task_digest.as_str()),
        ("workflow graph", identity.workflow_graph_digest.as_str()),
        ("model snapshot", identity.model_snapshot_digest.as_str()),
        ("toolchain", identity.toolchain_digest.as_str()),
        ("executor", identity.executor_digest.as_str()),
        ("scorer", identity.scorer_digest.as_str()),
        ("evaluator", identity.evaluator_digest.as_str()),
        ("inference plan", identity.inference_plan_digest.as_str()),
        ("random seed", identity.random_seed_digest.as_str()),
    ] {
        if !valid_digest(value) {
            return Err(format!("{name} digest is malformed"));
        }
    }
    if identity.context_budget_tokens == 0
        || identity.output_budget_tokens == 0
        || identity.compute_budget_microusd == 0
        || identity.wall_clock_budget_ms == 0
        || identity.memory_budget_bytes == 0
    {
        return Err("all workflow resource budgets must be positive".into());
    }
    Ok(())
}

fn validate_run(run: &WorkflowRun) -> Result<(), String> {
    if !nonempty(&run.run_id) {
        return Err("workflow run ID must be non-empty".into());
    }
    validate_identity(&run.identity)?;
    for (name, digest) in [
        ("execution", run.execution_receipt_digest.as_str()),
        ("exposure trace", run.exposure_trace_receipt_digest.as_str()),
        ("evaluator", run.evaluator_receipt_digest.as_str()),
    ] {
        if !valid_digest(digest) {
            return Err(format!("{name} receipt digest is malformed"));
        }
    }

    let graph_digest = compute_workflow_graph_digest(&run.nodes, &run.handoff_contracts)?;
    if graph_digest != run.identity.workflow_graph_digest {
        return Err("workflow graph digest does not bind the declared nodes and handoff contracts".into());
    }

    let node_ids: BTreeSet<&str> = run.nodes.iter().map(|node| node.node_id.as_str()).collect();
    let mut artifact_by_id = BTreeMap::new();
    for artifact in &run.artifacts {
        if !nonempty(&artifact.artifact_id)
            || !valid_digest(&artifact.payload_digest)
            || !nonempty(&artifact.producer_node_id)
            || !node_ids.contains(artifact.producer_node_id.as_str())
            || !nonempty(&artifact.artifact_type)
            || !valid_digest(&artifact.entity_scope_digest)
            || artifact.version == 0
        {
            return Err("workflow artifact identity or payload metadata is invalid".into());
        }
        if artifact
            .verification_receipt_digest
            .as_deref()
            .is_some_and(|digest| !valid_digest(digest))
        {
            return Err("workflow artifact verification receipt is malformed".into());
        }
        if artifact.verification == ArtifactVerification::Verified
            && artifact.verification_receipt_digest.is_none()
        {
            return Err("verified workflow artifact lacks a verification receipt".into());
        }
        if artifact_by_id.insert(artifact.artifact_id.as_str(), artifact).is_some() {
            return Err("workflow artifact IDs must be unique".into());
        }
    }

    let mut exposure_ids = BTreeSet::new();
    for exposure in &run.exposures {
        if !nonempty(&exposure.exposure_id)
            || !node_ids.contains(exposure.consumer_node_id.as_str())
            || !artifact_by_id.contains_key(exposure.artifact_id.as_str())
            || !valid_digest(&exposure.exposure_receipt_digest)
            || !exposure_ids.insert(exposure.exposure_id.as_str())
        {
            return Err("workflow artifact exposure has invalid identity, reference, receipt or duplicate ID".into());
        }
        if exposure
            .effect_receipt_digest
            .as_deref()
            .is_some_and(|digest| !valid_digest(digest))
        {
            return Err("workflow effect receipt digest is malformed".into());
        }
        if exposure.disposition == ArtifactExposureDisposition::Consumed
            && exposure.effect_receipt_digest.is_none()
        {
            return Err("consumed artifact exposure lacks an effect receipt".into());
        }
    }

    let mut outcome_nodes = BTreeSet::new();
    for outcome in &run.node_outcomes {
        if !node_ids.contains(outcome.node_id.as_str())
            || !valid_digest(&outcome.node_execution_receipt_digest)
            || !outcome_nodes.insert(outcome.node_id.as_str())
        {
            return Err("workflow node outcome has invalid or duplicate identity/receipt".into());
        }
    }
    if outcome_nodes.len() != node_ids.len() {
        return Err("workflow node outcomes do not cover every declared node".into());
    }
    Ok(())
}

fn expected_rejection(
    contracts: &[ArtifactHandoffContract],
    artifact: &WorkflowArtifact,
    consumer_node_id: &str,
    reported_operation_state: ArtifactOperationState,
) -> Option<ArtifactExposureDisposition> {
    let related_contracts = contracts
        .iter()
        .filter(|contract| {
            contract.producer_node_id == artifact.producer_node_id
                && contract.consumer_node_id == consumer_node_id
        })
        .collect::<Vec<_>>();
    if related_contracts.is_empty() {
        return Some(ArtifactExposureDisposition::RejectedWrongBranch);
    }
    let matching_type = related_contracts
        .into_iter()
        .filter(|contract| contract.artifact_type == artifact.artifact_type)
        .collect::<Vec<_>>();
    if matching_type.is_empty() {
        return Some(ArtifactExposureDisposition::RejectedInapplicable);
    }
    let matching_entity = matching_type
        .into_iter()
        .find(|contract| contract.entity_scope_digest == artifact.entity_scope_digest);
    let Some(contract) = matching_entity else {
        return Some(ArtifactExposureDisposition::RejectedWrongEntity);
    };
    if artifact.version < contract.required_version
        || artifact.lifecycle == ArtifactLifecycle::Superseded
    {
        return Some(ArtifactExposureDisposition::RejectedStale);
    }
    if contract.verification_required && artifact.verification != ArtifactVerification::Verified {
        return Some(ArtifactExposureDisposition::RejectedUnverified);
    }
    if artifact.operation_state == ArtifactOperationState::Completed
        && reported_operation_state == ArtifactOperationState::Pending
    {
        return Some(ArtifactExposureDisposition::RejectedCompletedAsPending);
    }
    None
}

fn run_metrics(run: &WorkflowRun) -> WorkflowBoundaryMetrics {
    let mut metrics = empty_metrics(run);
    metrics.verified_completed_node_count = run
        .node_outcomes
        .iter()
        .filter(|outcome| outcome.verified_complete)
        .count();
    metrics.verified_node_completion_rate = (!run.nodes.is_empty()).then(|| {
        metrics.verified_completed_node_count as f64 / run.nodes.len() as f64
    });
    metrics.required_handoff_count = run
        .handoff_contracts
        .iter()
        .filter(|contract| contract.required)
        .count();

    for exposure in &run.exposures {
        match exposure.disposition {
            ArtifactExposureDisposition::Consumed => metrics.consumed_exposure_count += 1,
            ArtifactExposureDisposition::Unknown => metrics.unknown_exposure_count += 1,
            ArtifactExposureDisposition::RejectedInapplicable
            | ArtifactExposureDisposition::RejectedStale
            | ArtifactExposureDisposition::RejectedUnverified
            | ArtifactExposureDisposition::RejectedWrongEntity
            | ArtifactExposureDisposition::RejectedWrongBranch
            | ArtifactExposureDisposition::RejectedCompletedAsPending => {
                metrics.rejected_exposure_count += 1;
            }
        }
    }

    for contract in run.handoff_contracts.iter().filter(|contract| contract.required) {
        let matching_artifacts = run.artifacts.iter().filter(|artifact| {
            artifact.producer_node_id == contract.producer_node_id
                && artifact.artifact_type == contract.artifact_type
                && artifact.entity_scope_digest == contract.entity_scope_digest
                && artifact.version >= contract.required_version
                && artifact.lifecycle == ArtifactLifecycle::Current
                && (!contract.verification_required
                    || artifact.verification == ArtifactVerification::Verified)
        });
        let successful = matching_artifacts.into_iter().any(|artifact| {
            run.exposures.iter().any(|exposure| {
                exposure.consumer_node_id == contract.consumer_node_id
                    && exposure.artifact_id == artifact.artifact_id
                    && exposure.disposition == ArtifactExposureDisposition::Consumed
            })
        });
        if successful {
            metrics.successful_required_handoff_count += 1;
        }
    }
    metrics.verified_handoff_success_rate =
        (metrics.required_handoff_count > 0).then(|| {
            metrics.successful_required_handoff_count as f64
                / metrics.required_handoff_count as f64
        });
    metrics
}

/// Qualifies a matched clean/polluted workflow pair with exactly one additional
/// artifact exposure at the challenged consumer.
///
/// The clean target node must have a verified-completion receipt; otherwise the
/// isolation denominator is not eligible and the result is INDETERMINATE.
pub fn qualify_workflow_boundary_pair(
    pair: &WorkflowBoundaryPair,
) -> WorkflowBoundaryReport {
    if !nonempty(&pair.pair_id) || !nonempty(&pair.challenge.challenge_id) {
        return report(
            WorkflowBoundaryStatus::Fail,
            pair,
            vec!["pair and challenge IDs must be non-empty".into()],
        );
    }
    if let Err(error) = validate_run(&pair.clean) {
        return report(WorkflowBoundaryStatus::Fail, pair, vec![format!("clean run: {error}")]);
    }
    if let Err(error) = validate_run(&pair.polluted) {
        return report(WorkflowBoundaryStatus::Fail, pair, vec![format!("polluted run: {error}")]);
    }
    if pair.clean.run_id == pair.polluted.run_id {
        return report(
            WorkflowBoundaryStatus::Fail,
            pair,
            vec!["clean and polluted workflow runs must have distinct run IDs".into()],
        );
    }
    if pair.clean.identity != pair.polluted.identity
        || pair.clean.nodes != pair.polluted.nodes
        || pair.clean.handoff_contracts != pair.polluted.handoff_contracts
        || pair.clean.artifacts != pair.polluted.artifacts
    {
        return report(
            WorkflowBoundaryStatus::Fail,
            pair,
            vec!["clean and polluted runs differ in workflow/model/evaluator/budget identity, graph or artifact catalog".into()],
        );
    }
    if pair.challenge.polluted_artifact_id.is_empty()
        || pair.challenge.target_consumer_node_id.is_empty()
    {
        return report(
            WorkflowBoundaryStatus::Fail,
            pair,
            vec!["pollution challenge must bind an artifact and target consumer".into()],
        );
    }

    let clean_exposure_ids: BTreeSet<&str> = pair
        .clean
        .exposures
        .iter()
        .map(|exposure| exposure.exposure_id.as_str())
        .collect();
    if pair.polluted.exposures.len() != pair.clean.exposures.len() + 1 {
        return report(
            WorkflowBoundaryStatus::Fail,
            pair,
            vec!["polluted run must add exactly one artifact exposure".into()],
        );
    }
    for clean_exposure in &pair.clean.exposures {
        let unchanged = pair.polluted.exposures.iter().any(|polluted_exposure| {
            polluted_exposure.exposure_id == clean_exposure.exposure_id
                && polluted_exposure == clean_exposure
        });
        if !unchanged {
            return report(
                WorkflowBoundaryStatus::Fail,
                pair,
                vec!["polluted run changed or removed a non-target exposure".into()],
            );
        }
    }
    let added = pair
        .polluted
        .exposures
        .iter()
        .filter(|exposure| !clean_exposure_ids.contains(exposure.exposure_id.as_str()))
        .collect::<Vec<_>>();
    if added.len() != 1 {
        return report(
            WorkflowBoundaryStatus::Fail,
            pair,
            vec!["polluted run does not contain exactly one new exposure".into()],
        );
    }
    let pollution_exposure = added[0];
    if pollution_exposure.consumer_node_id != pair.challenge.target_consumer_node_id
        || pollution_exposure.artifact_id != pair.challenge.polluted_artifact_id
    {
        return report(
            WorkflowBoundaryStatus::Fail,
            pair,
            vec!["the one additional exposure does not match the declared challenge target".into()],
        );
    }

    let artifact = pair
        .polluted
        .artifacts
        .iter()
        .find(|artifact| artifact.artifact_id == pair.challenge.polluted_artifact_id);
    let Some(artifact) = artifact else {
        return report(
            WorkflowBoundaryStatus::Fail,
            pair,
            vec!["polluted artifact is absent from the frozen artifact catalog".into()],
        );
    };
    let expected = expected_rejection(
        &pair.polluted.handoff_contracts,
        artifact,
        &pair.challenge.target_consumer_node_id,
        pollution_exposure.reported_operation_state,
    );
    let expected_operator = match pair.challenge.operator {
        BoundaryPollutionOperator::SiblingCrosswire => ArtifactExposureDisposition::RejectedWrongBranch,
        BoundaryPollutionOperator::IncompatiblePayload => ArtifactExposureDisposition::RejectedInapplicable,
        BoundaryPollutionOperator::UnverifiedResult => ArtifactExposureDisposition::RejectedUnverified,
        BoundaryPollutionOperator::CompletedAsPending => ArtifactExposureDisposition::RejectedCompletedAsPending,
        BoundaryPollutionOperator::WrongEntity => ArtifactExposureDisposition::RejectedWrongEntity,
        BoundaryPollutionOperator::StaleVersion => ArtifactExposureDisposition::RejectedStale,
    };
    if expected != Some(expected_operator) {
        return report(
            WorkflowBoundaryStatus::Fail,
            pair,
            vec!["pollution artifact does not instantiate the declared inapplicability operator for this consumer".into()],
        );
    }

    let target_succeeded = pair.clean.node_outcomes.iter().any(|outcome| {
        outcome.node_id == pair.challenge.target_consumer_node_id && outcome.verified_complete
    });
    let mut result = report(WorkflowBoundaryStatus::Indeterminate, pair, Vec::new());
    result.clean_target_succeeded = target_succeeded;
    result.pollution_was_exposed = true;
    result.pollution_was_consumed =
        pollution_exposure.disposition == ArtifactExposureDisposition::Consumed;
    result.pollution_disposition = Some(pollution_exposure.disposition);
    result.clean_metrics = run_metrics(&pair.clean);
    result.polluted_metrics = run_metrics(&pair.polluted);

    if !target_succeeded {
        result.reasons.push(
            "clean target node did not have verified completion; isolation denominator is ineligible".into(),
        );
        return result;
    }

    match pollution_exposure.disposition {
        ArtifactExposureDisposition::Consumed => {
            result.status = WorkflowBoundaryStatus::Fail;
            result.reasons.push(
                "inapplicable artifact was consumed by an otherwise-solvable target consumer".into(),
            );
        }
        ArtifactExposureDisposition::Unknown => {
            result.status = WorkflowBoundaryStatus::Indeterminate;
            result.reasons.push(
                "polluted artifact exposure has no qualified consume/reject disposition".into(),
            );
        }
        disposition if disposition == expected_operator => {
            result.status = WorkflowBoundaryStatus::Pass;
            result.reasons.push(
                "exactly one inapplicable artifact reached the eligible challenge boundary and was rejected with the expected disposition".into(),
            );
        }
        _ => {
            result.status = WorkflowBoundaryStatus::Fail;
            result.reasons.push(
                "polluted artifact was not rejected with its required task-conditioned disposition".into(),
            );
        }
    }
    result
}

#[cfg(test)]
mod tests {
    use super::*;

    fn digest(ch: char) -> String {
        std::iter::repeat_n(ch, 64).collect()
    }

    fn node(id: &str, ch: char) -> WorkflowNode {
        WorkflowNode {
            node_id: id.into(),
            task_digest: digest(ch),
        }
    }

    fn contract(
        id: &str,
        producer: &str,
        consumer: &str,
        artifact_type: &str,
        entity: char,
        version: u64,
    ) -> ArtifactHandoffContract {
        ArtifactHandoffContract {
            edge_id: id.into(),
            producer_node_id: producer.into(),
            consumer_node_id: consumer.into(),
            artifact_type: artifact_type.into(),
            entity_scope_digest: digest(entity),
            required_version: version,
            verification_required: true,
            required: true,
        }
    }

    fn artifact(
        id: &str,
        producer: &str,
        artifact_type: &str,
        entity: char,
        version: u64,
        lifecycle: ArtifactLifecycle,
        verification: ArtifactVerification,
        operation: ArtifactOperationState,
    ) -> WorkflowArtifact {
        WorkflowArtifact {
            artifact_id: id.into(),
            payload_digest: digest('a'),
            producer_node_id: producer.into(),
            artifact_type: artifact_type.into(),
            entity_scope_digest: digest(entity),
            version,
            lifecycle,
            verification,
            verification_receipt_digest: (verification == ArtifactVerification::Verified)
                .then(|| digest('b')),
            operation_state: operation,
        }
    }

    fn exposure(
        id: &str,
        consumer: &str,
        artifact: &str,
        disposition: ArtifactExposureDisposition,
        reported: ArtifactOperationState,
    ) -> WorkflowArtifactExposure {
        WorkflowArtifactExposure {
            exposure_id: id.into(),
            consumer_node_id: consumer.into(),
            artifact_id: artifact.into(),
            reported_operation_state: reported,
            disposition,
            exposure_receipt_digest: digest('c'),
            effect_receipt_digest: (disposition == ArtifactExposureDisposition::Consumed)
                .then(|| digest('d')),
        }
    }

    fn identity(graph: String) -> WorkflowIdentity {
        WorkflowIdentity {
            benchmark_id: "CoMemBench".into(),
            benchmark_version: "reproduction-v1".into(),
            benchmark_certificate_digest: digest('1'),
            split_digest: digest('2'),
            workflow_id: "workflow-001".into(),
            task_digest: digest('3'),
            source_group_id: "source-group-001".into(),
            workflow_graph_digest: graph,
            model_snapshot_digest: digest('4'),
            toolchain_digest: digest('5'),
            executor_digest: digest('6'),
            scorer_digest: digest('7'),
            evaluator_digest: digest('8'),
            inference_plan_digest: digest('9'),
            random_seed_digest: digest('a'),
            context_budget_tokens: 16000,
            output_budget_tokens: 2000,
            compute_budget_microusd: 100000,
            wall_clock_budget_ms: 60000,
            memory_budget_bytes: 1000000,
        }
    }

    fn run(
        run_id: &str,
        graph_nodes: Vec<WorkflowNode>,
        contracts: Vec<ArtifactHandoffContract>,
        artifacts: Vec<WorkflowArtifact>,
        exposures: Vec<WorkflowArtifactExposure>,
    ) -> WorkflowRun {
        let graph = compute_workflow_graph_digest(&graph_nodes, &contracts).unwrap();
        let node_outcomes = graph_nodes
            .iter()
            .map(|node| WorkflowNodeOutcome {
                node_id: node.node_id.clone(),
                verified_complete: true,
                node_execution_receipt_digest: digest('e'),
            })
            .collect();
        WorkflowRun {
            run_id: run_id.into(),
            identity: identity(graph),
            nodes: graph_nodes,
            handoff_contracts: contracts,
            artifacts,
            exposures,
            node_outcomes,
            terminal_success: true,
            execution_receipt_digest: digest('f'),
            exposure_trace_receipt_digest: digest('1'),
            evaluator_receipt_digest: digest('2'),
            latency_ms: 1000,
            total_cost_microusd: 50000,
            input_tokens: 5000,
            output_tokens: 1000,
        }
    }

    fn valid_pair(
        operator: BoundaryPollutionOperator,
        bad_artifact: WorkflowArtifact,
        reported: ArtifactOperationState,
        rejected: ArtifactExposureDisposition,
    ) -> WorkflowBoundaryPair {
        let nodes = vec![node("producer-a", '3'), node("producer-b", '4'), node("consumer", '5')];
        let good_contract = contract("edge-good", "producer-a", "consumer", "report", '6', 2);
        let contracts = vec![good_contract];
        let good_artifact = artifact(
            "good-artifact", "producer-a", "report", '6', 2,
            ArtifactLifecycle::Current, ArtifactVerification::Verified,
            ArtifactOperationState::NotOperation,
        );
        let artifacts = vec![good_artifact, bad_artifact];
        let clean_exposures = vec![exposure(
            "good-exposure", "consumer", "good-artifact",
            ArtifactExposureDisposition::Consumed, ArtifactOperationState::NotOperation,
        )];
        let polluted_exposures = clean_exposures
            .iter()
            .cloned()
            .chain(vec![exposure(
                "pollution-exposure", "consumer", "bad-artifact", rejected, reported,
            )])
            .collect();
        WorkflowBoundaryPair {
            pair_id: "pair-01".into(),
            clean: run("clean-01", nodes.clone(), contracts.clone(), artifacts.clone(), clean_exposures),
            polluted: run("polluted-01", nodes, contracts, artifacts, polluted_exposures),
            challenge: BoundaryIsolationChallenge {
                challenge_id: "challenge-01".into(),
                target_consumer_node_id: "consumer".into(),
                polluted_artifact_id: "bad-artifact".into(),
                operator,
            },
        }
    }

    #[test]
    fn graph_digest_is_order_independent() {
        let nodes = vec![node("p", '1'), node("c", '2')];
        let edges = vec![contract("e", "p", "c", "report", '3', 1)];
        let mut reversed_nodes = nodes.clone();
        reversed_nodes.reverse();
        let mut reversed_edges = edges.clone();
        reversed_edges.reverse();
        assert_eq!(
            compute_workflow_graph_digest(&nodes, &edges).unwrap(),
            compute_workflow_graph_digest(&reversed_nodes, &reversed_edges).unwrap()
        );
    }

    #[test]
    fn verified_required_handoff_and_rejected_crosswire_qualify() {
        let bad = artifact(
            "bad-artifact", "producer-b", "report", '6', 2,
            ArtifactLifecycle::Current, ArtifactVerification::Verified,
            ArtifactOperationState::NotOperation,
        );
        let pair = valid_pair(
            BoundaryPollutionOperator::SiblingCrosswire,
            bad,
            ArtifactOperationState::NotOperation,
            ArtifactExposureDisposition::RejectedWrongBranch,
        );
        let report = qualify_workflow_boundary_pair(&pair);
        assert_eq!(report.status, WorkflowBoundaryStatus::Pass);
        assert!(report.pollution_was_exposed);
        assert!(!report.pollution_was_consumed);
        assert_eq!(report.clean_metrics.successful_required_handoff_count, 1);
    }

    #[test]
    fn consuming_a_sibling_crosswire_fails_isolation() {
        let bad = artifact(
            "bad-artifact", "producer-b", "report", '6', 2,
            ArtifactLifecycle::Current, ArtifactVerification::Verified,
            ArtifactOperationState::NotOperation,
        );
        let pair = valid_pair(
            BoundaryPollutionOperator::SiblingCrosswire,
            bad,
            ArtifactOperationState::NotOperation,
            ArtifactExposureDisposition::Consumed,
        );
        assert_eq!(
            qualify_workflow_boundary_pair(&pair).status,
            WorkflowBoundaryStatus::Fail
        );
    }

    #[test]
    fn unverified_result_must_be_rejected() {
        let bad = artifact(
            "bad-artifact", "producer-a", "report", '6', 2,
            ArtifactLifecycle::Current, ArtifactVerification::Unverified,
            ArtifactOperationState::NotOperation,
        );
        let pair = valid_pair(
            BoundaryPollutionOperator::UnverifiedResult,
            bad,
            ArtifactOperationState::NotOperation,
            ArtifactExposureDisposition::RejectedUnverified,
        );
        assert_eq!(
            qualify_workflow_boundary_pair(&pair).status,
            WorkflowBoundaryStatus::Pass
        );
    }

    #[test]
    fn incompatible_payload_must_be_rejected() {
        let bad = artifact(
            "bad-artifact", "producer-a", "debug-log", '6', 2,
            ArtifactLifecycle::Current, ArtifactVerification::Verified,
            ArtifactOperationState::NotOperation,
        );
        let pair = valid_pair(
            BoundaryPollutionOperator::IncompatiblePayload,
            bad,
            ArtifactOperationState::NotOperation,
            ArtifactExposureDisposition::RejectedInapplicable,
        );
        assert_eq!(
            qualify_workflow_boundary_pair(&pair).status,
            WorkflowBoundaryStatus::Pass
        );
    }

    #[test]
    fn wrong_entity_must_be_rejected() {
        let bad = artifact(
            "bad-artifact", "producer-a", "report", '7', 2,
            ArtifactLifecycle::Current, ArtifactVerification::Verified,
            ArtifactOperationState::NotOperation,
        );
        let pair = valid_pair(
            BoundaryPollutionOperator::WrongEntity,
            bad,
            ArtifactOperationState::NotOperation,
            ArtifactExposureDisposition::RejectedWrongEntity,
        );
        assert_eq!(
            qualify_workflow_boundary_pair(&pair).status,
            WorkflowBoundaryStatus::Pass
        );
    }

    #[test]
    fn stale_version_must_be_rejected() {
        let bad = artifact(
            "bad-artifact", "producer-a", "report", '6', 1,
            ArtifactLifecycle::Superseded, ArtifactVerification::Verified,
            ArtifactOperationState::NotOperation,
        );
        let pair = valid_pair(
            BoundaryPollutionOperator::StaleVersion,
            bad,
            ArtifactOperationState::NotOperation,
            ArtifactExposureDisposition::RejectedStale,
        );
        assert_eq!(
            qualify_workflow_boundary_pair(&pair).status,
            WorkflowBoundaryStatus::Pass
        );
    }

    #[test]
    fn completed_side_effect_cannot_be_represented_as_pending() {
        let bad = artifact(
            "bad-artifact", "producer-a", "report", '6', 2,
            ArtifactLifecycle::Current, ArtifactVerification::Verified,
            ArtifactOperationState::Completed,
        );
        let pair = valid_pair(
            BoundaryPollutionOperator::CompletedAsPending,
            bad,
            ArtifactOperationState::Pending,
            ArtifactExposureDisposition::RejectedCompletedAsPending,
        );
        assert_eq!(
            qualify_workflow_boundary_pair(&pair).status,
            WorkflowBoundaryStatus::Pass
        );
    }

    #[test]
    fn changed_non_target_exposure_fails_paired_control() {
        let bad = artifact(
            "bad-artifact", "producer-b", "report", '6', 2,
            ArtifactLifecycle::Current, ArtifactVerification::Verified,
            ArtifactOperationState::NotOperation,
        );
        let mut pair = valid_pair(
            BoundaryPollutionOperator::SiblingCrosswire,
            bad,
            ArtifactOperationState::NotOperation,
            ArtifactExposureDisposition::RejectedWrongBranch,
        );
        pair.polluted.exposures[0].disposition = ArtifactExposureDisposition::Unknown;
        assert_eq!(
            qualify_workflow_boundary_pair(&pair).status,
            WorkflowBoundaryStatus::Fail
        );
    }

    #[test]
    fn no_clean_success_means_isolation_denominator_is_indeterminate() {
        let bad = artifact(
            "bad-artifact", "producer-b", "report", '6', 2,
            ArtifactLifecycle::Current, ArtifactVerification::Verified,
            ArtifactOperationState::NotOperation,
        );
        let mut pair = valid_pair(
            BoundaryPollutionOperator::SiblingCrosswire,
            bad,
            ArtifactOperationState::NotOperation,
            ArtifactExposureDisposition::RejectedWrongBranch,
        );
        pair.clean.node_outcomes[2].verified_complete = false;
        let report = qualify_workflow_boundary_pair(&pair);
        assert_eq!(report.status, WorkflowBoundaryStatus::Indeterminate);
        assert!(!report.clean_target_succeeded);
    }

    #[test]
    fn missing_effect_receipt_for_consumed_artifact_fails_integrity() {
        let bad = artifact(
            "bad-artifact", "producer-b", "report", '6', 2,
            ArtifactLifecycle::Current, ArtifactVerification::Verified,
            ArtifactOperationState::NotOperation,
        );
        let mut pair = valid_pair(
            BoundaryPollutionOperator::SiblingCrosswire,
            bad,
            ArtifactOperationState::NotOperation,
            ArtifactExposureDisposition::Consumed,
        );
        pair.polluted.exposures[1].effect_receipt_digest = None;
        assert_eq!(
            qualify_workflow_boundary_pair(&pair).status,
            WorkflowBoundaryStatus::Fail
        );
    }
}
