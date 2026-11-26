//! Paired counterfactual qualification for individual evidence-relation edges.
//!
//! The validator isolates a declared relation-graph intervention while keeping
//! the evidence pool, retrieval seeds, model, prompt, tools, evaluator, budgets,
//! and randomization fixed. It validates experiment structure; it does not prove
//! that a relation is true or estimate a population-level causal effect.

use std::collections::{BTreeMap, BTreeSet};

use sha2::{Digest, Sha256};

use crate::EvidenceRelationKind;

fn nonempty(value: &str) -> bool {
    !value.trim().is_empty()
}

fn valid_digest(value: &str) -> bool {
    value.len() == 64
        && value
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}

fn unique_nonempty(values: &[String]) -> bool {
    let mut seen = BTreeSet::new();
    values
        .iter()
        .all(|value| nonempty(value) && seen.insert(value.as_str()))
}

const RELATION_GRAPH_DOMAIN: &[u8] = b"MYCELIX-RELATION-GRAPH-V0.1\\0";

fn push_len(out: &mut Vec<u8>, value: usize) {
    out.extend_from_slice(&(value as u64).to_be_bytes());
}

fn push_str(out: &mut Vec<u8>, value: &str) {
    push_len(out, value.len());
    out.extend_from_slice(value.as_bytes());
}

fn relation_tag(value: EvidenceRelationKind) -> u8 {
    match value {
        EvidenceRelationKind::SameSubject => 1,
        EvidenceRelationKind::SameEvent => 2,
        EvidenceRelationKind::SameDecision => 3,
        EvidenceRelationKind::Supports => 4,
        EvidenceRelationKind::Contradicts => 5,
        EvidenceRelationKind::Supersedes => 6,
        EvidenceRelationKind::Updates => 7,
        EvidenceRelationKind::RelatedContext => 8,
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

fn symmetric_relation(value: EvidenceRelationKind) -> bool {
    matches!(
        value,
        EvidenceRelationKind::SameSubject
            | EvidenceRelationKind::SameEvent
            | EvidenceRelationKind::SameDecision
            | EvidenceRelationKind::Contradicts
            | EvidenceRelationKind::RelatedContext
    )
}

/// Computes a deterministic, domain-separated digest of an edge set sorted by edge ID.
pub fn compute_relation_graph_digest(edges: &[RelationEdgeSnapshot]) -> Result<String, String> {
    let mut sorted = edges.iter().collect::<Vec<_>>();
    sorted.sort_by(|left, right| left.edge_id.cmp(&right.edge_id));
    let mut seen = BTreeSet::new();
    let mut bytes = Vec::new();
    bytes.extend_from_slice(RELATION_GRAPH_DOMAIN);
    push_len(&mut bytes, sorted.len());
    for edge in sorted {
        if !nonempty(&edge.edge_id)
            || !nonempty(&edge.from_evidence_id)
            || !nonempty(&edge.to_evidence_id)
            || edge.from_evidence_id == edge.to_evidence_id
            || !valid_digest(&edge.relation_receipt_digest)
            || !seen.insert(edge.edge_id.as_str())
        {
            return Err("relation graph has invalid or duplicate edge identity".into());
        }
        push_str(&mut bytes, &edge.edge_id);
        push_str(&mut bytes, &edge.from_evidence_id);
        push_str(&mut bytes, &edge.to_evidence_id);
        bytes.push(relation_tag(edge.relation));
        push_str(&mut bytes, &edge.relation_receipt_digest);
    }
    Ok(sha256_hex(&bytes))
}

#[derive(Clone, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub struct RelationEdgeSnapshot {
    pub edge_id: String,
    pub from_evidence_id: String,
    pub to_evidence_id: String,
    pub relation: EvidenceRelationKind,
    pub relation_receipt_digest: String,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RelationInterventionOperation {
    Add,
    Remove,
    Reverse,
    Relabel,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct RelationIntervention {
    pub intervention_id: String,
    pub operation: RelationInterventionOperation,
    /// Present for Remove, Reverse, and Relabel.
    pub before: Option<RelationEdgeSnapshot>,
    /// Present for Add, Reverse, and Relabel.
    pub after: Option<RelationEdgeSnapshot>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct RelationReplayIdentity {
    pub benchmark_id: String,
    pub benchmark_version: String,
    pub benchmark_certificate_digest: String,
    pub split_digest: String,
    pub task_digest: String,
    /// The source-group is the independent cluster for uncertainty estimation.
    pub source_group_id: String,
    pub candidate_pool_digest: String,
    pub retrieval_seed_digest: String,
    pub model_snapshot_digest: String,
    pub tokenizer_digest: String,
    pub prompt_digest: String,
    pub toolchain_digest: String,
    pub decoding_digest: String,
    pub scorer_digest: String,
    pub evaluator_digest: String,
    pub inference_plan_digest: String,
    pub seed_policy_digest: String,
    pub random_seed_digest: String,
    pub context_budget_tokens: u64,
    pub output_budget_tokens: u64,
    pub compute_budget_microusd: u64,
    pub wall_clock_budget_ms: u64,
    pub memory_budget_bytes: u64,
    pub agent_count: u32,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RecordedGateOutcome {
    Pass,
    Fail,
    Indeterminate,
}

/// Outcomes are observations, not qualification gates. A failed ACB or audit
/// is a valid measured outcome and must not invalidate an otherwise sound pair.
#[derive(Clone, Debug, PartialEq)]
pub struct RelationReplayOutcome {
    pub selected_evidence_ids: Vec<String>,
    pub decision_digest: String,
    /// Independent scorer output normalized to [0, 1].
    pub decision_quality: f64,
    pub constraint_gate: RecordedGateOutcome,
    pub response_audit_gate: RecordedGateOutcome,
    pub unauthorized_exposure_count: u64,
    pub latency_ms: u64,
    pub total_cost_microusd: u64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct RelationReplayRun {
    pub run_id: String,
    pub identity: RelationReplayIdentity,
    pub candidate_evidence_ids: Vec<String>,
    pub retrieval_seed_evidence_ids: Vec<String>,
    pub relation_graph_digest: String,
    pub relation_edges: Vec<RelationEdgeSnapshot>,
    pub candidate_pool_complete: bool,
    pub relation_index_complete: bool,
    pub candidate_pool_receipt_digest: Option<String>,
    pub relation_index_receipt_digest: Option<String>,
    pub execution_receipt_digest: Option<String>,
    pub generation_exposure_receipt_digest: Option<String>,
    pub rer_bundle_receipt_digest: Option<String>,
    pub constraint_receipt_digest: Option<String>,
    pub response_audit_receipt_digest: Option<String>,
    pub evaluator_receipt_digest: Option<String>,
    pub outcome: RelationReplayOutcome,
}

#[derive(Clone, Debug, PartialEq)]
pub struct RelationCounterfactualTrial {
    pub trial_id: String,
    pub intervention: RelationIntervention,
    pub baseline: RelationReplayRun,
    pub counterfactual: RelationReplayRun,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RelationCounterfactualStatus {
    Pass,
    Fail,
    Indeterminate,
}

#[derive(Clone, Debug, PartialEq)]
pub struct RelationCounterfactualReport {
    pub status: RelationCounterfactualStatus,
    pub trial_id: String,
    pub operation: RelationInterventionOperation,
    pub decision_changed: Option<bool>,
    pub selected_evidence_changed: Option<bool>,
    pub decision_quality_delta: Option<f64>,
    pub unauthorized_exposure_delta: Option<i128>,
    pub latency_delta_ms: Option<i128>,
    pub total_cost_delta_microusd: Option<i128>,
    pub baseline_constraint_gate: Option<RecordedGateOutcome>,
    pub counterfactual_constraint_gate: Option<RecordedGateOutcome>,
    pub baseline_response_audit_gate: Option<RecordedGateOutcome>,
    pub counterfactual_response_audit_gate: Option<RecordedGateOutcome>,
    pub reasons: Vec<String>,
}

fn report(
    status: RelationCounterfactualStatus,
    trial_id: &str,
    operation: RelationInterventionOperation,
    reasons: Vec<String>,
) -> RelationCounterfactualReport {
    RelationCounterfactualReport {
        status,
        trial_id: trial_id.to_owned(),
        operation,
        decision_changed: None,
        selected_evidence_changed: None,
        decision_quality_delta: None,
        unauthorized_exposure_delta: None,
        latency_delta_ms: None,
        total_cost_delta_microusd: None,
        baseline_constraint_gate: None,
        counterfactual_constraint_gate: None,
        baseline_response_audit_gate: None,
        counterfactual_response_audit_gate: None,
        reasons,
    }
}

fn validate_edge_set(
    run: &RelationReplayRun,
) -> Result<BTreeMap<&str, &RelationEdgeSnapshot>, String> {
    let evidence_ids: BTreeSet<&str> = run
        .candidate_evidence_ids
        .iter()
        .map(String::as_str)
        .collect();
    let mut edges = BTreeMap::new();
    for edge in &run.relation_edges {
        if !nonempty(&edge.edge_id)
            || !nonempty(&edge.from_evidence_id)
            || !nonempty(&edge.to_evidence_id)
            || edge.from_evidence_id == edge.to_evidence_id
            || !valid_digest(&edge.relation_receipt_digest)
        {
            return Err("relation graph contains an edge with invalid identity, endpoints or receipt".into());
        }
        if !evidence_ids.contains(edge.from_evidence_id.as_str())
            || !evidence_ids.contains(edge.to_evidence_id.as_str())
        {
            return Err(format!(
                "relation edge {} references evidence outside the frozen candidate pool",
                edge.edge_id
            ));
        }
        if edges.insert(edge.edge_id.as_str(), edge).is_some() {
            return Err("relation graph contains duplicate edge IDs".into());
        }
    }
    Ok(edges)
}

fn validate_run(run: &RelationReplayRun, label: &str) -> Result<(), String> {
    if !nonempty(&run.run_id) {
        return Err(format!("{label} run ID must be non-empty"));
    }
    for (name, value) in [
        ("benchmark ID", run.identity.benchmark_id.as_str()),
        ("benchmark version", run.identity.benchmark_version.as_str()),
        ("source-group ID", run.identity.source_group_id.as_str()),
    ] {
        if !nonempty(value) {
            return Err(format!("{label} {name} must be non-empty"));
        }
    }
    for (name, value) in [
        ("benchmark certificate", run.identity.benchmark_certificate_digest.as_str()),
        ("split", run.identity.split_digest.as_str()),
        ("task", run.identity.task_digest.as_str()),
        ("candidate pool", run.identity.candidate_pool_digest.as_str()),
        ("retrieval seed", run.identity.retrieval_seed_digest.as_str()),
        ("model snapshot", run.identity.model_snapshot_digest.as_str()),
        ("tokenizer", run.identity.tokenizer_digest.as_str()),
        ("prompt", run.identity.prompt_digest.as_str()),
        ("toolchain", run.identity.toolchain_digest.as_str()),
        ("decoding", run.identity.decoding_digest.as_str()),
        ("scorer", run.identity.scorer_digest.as_str()),
        ("evaluator", run.identity.evaluator_digest.as_str()),
        ("inference plan", run.identity.inference_plan_digest.as_str()),
        ("seed policy", run.identity.seed_policy_digest.as_str()),
        ("random seed", run.identity.random_seed_digest.as_str()),
    ] {
        if !valid_digest(value) {
            return Err(format!("{label} {name} digest is malformed"));
        }
    }
    if run.identity.agent_count == 0
        || run.identity.context_budget_tokens == 0
        || run.identity.output_budget_tokens == 0
        || run.identity.compute_budget_microusd == 0
        || run.identity.wall_clock_budget_ms == 0
        || run.identity.memory_budget_bytes == 0
    {
        return Err(format!("{label} model/compute/resource budgets must be positive"));
    }
    for (name, value) in [
        ("candidate-pool digest", run.identity.candidate_pool_digest.as_str()),
        ("retrieval-seed digest", run.identity.retrieval_seed_digest.as_str()),
        ("relation-graph digest", run.relation_graph_digest.as_str()),
    ] {
        if !valid_digest(value) {
            return Err(format!("{label} {name} is malformed"));
        }
    }
    if !unique_nonempty(&run.candidate_evidence_ids)
        || !unique_nonempty(&run.retrieval_seed_evidence_ids)
    {
        return Err(format!("{label} candidate and seed evidence IDs must be non-empty and unique"));
    }
    let candidates: BTreeSet<&str> = run
        .candidate_evidence_ids
        .iter()
        .map(String::as_str)
        .collect();
    if run
        .retrieval_seed_evidence_ids
        .iter()
        .any(|id| !candidates.contains(id.as_str()))
    {
        return Err(format!("{label} retrieval seeds are not a subset of the candidate pool"));
    }
    if !run.outcome.decision_quality.is_finite()
        || !(0.0..=1.0).contains(&run.outcome.decision_quality)
        || !valid_digest(&run.outcome.decision_digest)
    {
        return Err(format!("{label} decision outcome is malformed"));
    }
    if !unique_nonempty(&run.outcome.selected_evidence_ids)
        || run
            .outcome
            .selected_evidence_ids
            .iter()
            .any(|id| !candidates.contains(id.as_str()))
    {
        return Err(format!("{label} selected evidence is duplicated or outside the candidate pool"));
    }
    validate_edge_set(run)?;
    if compute_relation_graph_digest(&run.relation_edges)? != run.relation_graph_digest {
        return Err(format!("{label} relation-graph digest does not bind its edge set"));
    }
    Ok(())
}

fn missing_receipts(run: &RelationReplayRun, label: &str) -> Vec<String> {
    let receipts = [
        ("candidate-pool receipt", run.candidate_pool_receipt_digest.as_deref()),
        ("relation-index receipt", run.relation_index_receipt_digest.as_deref()),
        ("execution receipt", run.execution_receipt_digest.as_deref()),
        ("generation-exposure receipt", run.generation_exposure_receipt_digest.as_deref()),
        ("RER bundle receipt", run.rer_bundle_receipt_digest.as_deref()),
        ("constraint receipt", run.constraint_receipt_digest.as_deref()),
        ("response-audit receipt", run.response_audit_receipt_digest.as_deref()),
        ("evaluator receipt", run.evaluator_receipt_digest.as_deref()),
    ];
    let mut missing = Vec::new();
    for (name, value) in receipts {
        match value {
            Some(digest) if valid_digest(digest) => {}
            Some(_) => missing.push(format!("{label} {name} is malformed")),
            None => missing.push(format!("{label} {name} is missing")),
        }
    }
    missing
}

fn validate_intervention(intervention: &RelationIntervention) -> Result<(), String> {
    if !nonempty(&intervention.intervention_id) {
        return Err("intervention ID must be non-empty".into());
    }
    match (
        intervention.operation,
        intervention.before.as_ref(),
        intervention.after.as_ref(),
    ) {
        (RelationInterventionOperation::Add, None, Some(_)) => Ok(()),
        (RelationInterventionOperation::Remove, Some(_), None) => Ok(()),
        (RelationInterventionOperation::Reverse, Some(before), Some(after)) => {
            if before.edge_id != after.edge_id
                || before.from_evidence_id != after.to_evidence_id
                || before.to_evidence_id != after.from_evidence_id
                || before.relation != after.relation
            {
                return Err("Reverse must preserve edge ID and relation while swapping endpoints".into());
            }
            if symmetric_relation(before.relation) {
                return Err("symmetric relation kinds cannot be direction-reversed".into());
            }
            if before.relation_receipt_digest == after.relation_receipt_digest {
                return Err("a reversed edge requires a new relation receipt digest".into());
            }
            Ok(())
        }
        (RelationInterventionOperation::Relabel, Some(before), Some(after)) => {
            if before.edge_id != after.edge_id
                || before.from_evidence_id != after.from_evidence_id
                || before.to_evidence_id != after.to_evidence_id
                || before.relation == after.relation
            {
                return Err("Relabel must preserve edge ID and endpoints while changing relation kind".into());
            }
            if before.relation_receipt_digest == after.relation_receipt_digest {
                return Err("a relabelled edge requires a new relation receipt digest".into());
            }
            Ok(())
        }
        _ => Err("intervention before/after shape does not match its operation".into()),
    }
}

fn apply_intervention(
    baseline: &BTreeMap<&str, &RelationEdgeSnapshot>,
    intervention: &RelationIntervention,
) -> Result<BTreeMap<String, RelationEdgeSnapshot>, String> {
    let mut expected: BTreeMap<String, RelationEdgeSnapshot> = baseline
        .iter()
        .map(|(id, edge)| ((*id).to_owned(), (*edge).clone()))
        .collect();
    match (
        intervention.operation,
        intervention.before.as_ref(),
        intervention.after.as_ref(),
    ) {
        (RelationInterventionOperation::Add, None, Some(after)) => {
            if expected.contains_key(&after.edge_id) {
                return Err("added edge ID already exists in baseline graph".into());
            }
            expected.insert(after.edge_id.clone(), after.clone());
        }
        (RelationInterventionOperation::Remove, Some(before), None) => {
            match expected.remove(&before.edge_id) {
                Some(actual) if actual == *before => {}
                Some(_) => return Err("removed edge differs from baseline edge with same ID".into()),
                None => return Err("removed edge is absent from baseline graph".into()),
            }
        }
        (RelationInterventionOperation::Reverse | RelationInterventionOperation::Relabel, Some(before), Some(after)) => {
            match expected.get(&before.edge_id) {
                Some(actual) if **actual == *before => {}
                Some(_) => return Err("intervened edge differs from baseline edge with same ID".into()),
                None => return Err("intervened edge is absent from baseline graph".into()),
            }
            expected.insert(after.edge_id.clone(), after.clone());
        }
        _ => return Err("intervention shape is invalid".into()),
    }
    Ok(expected)
}

/// Qualifies one paired replay in which exactly one relation edge changes.
///
/// A PASS means the pair is structurally matched and the graph delta is exactly
/// the declared intervention. Recorded ACB/audit failures remain valid outcomes;
/// they are exposed in the run manifests rather than turned into a false PASS.
pub fn qualify_relation_counterfactual(
    trial: &RelationCounterfactualTrial,
) -> RelationCounterfactualReport {
    let operation = trial.intervention.operation;
    if !nonempty(&trial.trial_id) {
        return report(
            RelationCounterfactualStatus::Fail,
            &trial.trial_id,
            operation,
            vec!["trial ID must be non-empty".into()],
        );
    }
    if let Err(error) = validate_intervention(&trial.intervention) {
        return report(RelationCounterfactualStatus::Fail, &trial.trial_id, operation, vec![error]);
    }
    if let Err(error) = validate_run(&trial.baseline, "baseline") {
        return report(RelationCounterfactualStatus::Fail, &trial.trial_id, operation, vec![error]);
    }
    if let Err(error) = validate_run(&trial.counterfactual, "counterfactual") {
        return report(RelationCounterfactualStatus::Fail, &trial.trial_id, operation, vec![error]);
    }
    if trial.baseline.run_id == trial.counterfactual.run_id {
        return report(
            RelationCounterfactualStatus::Fail,
            &trial.trial_id,
            operation,
            vec!["baseline and counterfactual must have distinct run IDs".into()],
        );
    }
    if trial.baseline.identity != trial.counterfactual.identity {
        return report(
            RelationCounterfactualStatus::Fail,
            &trial.trial_id,
            operation,
            vec!["benchmark, split, task, group, model, evaluator, budget, or randomization identity differs".into()],
        );
    }
    if trial.baseline.candidate_evidence_ids != trial.counterfactual.candidate_evidence_ids
        || trial.baseline.retrieval_seed_evidence_ids != trial.counterfactual.retrieval_seed_evidence_ids
    {
        return report(
            RelationCounterfactualStatus::Fail,
            &trial.trial_id,
            operation,
            vec!["candidate evidence pool or retrieval seeds changed across the pair".into()],
        );
    }

    let baseline_edges = match validate_edge_set(&trial.baseline) {
        Ok(edges) => edges,
        Err(error) => {
            return report(RelationCounterfactualStatus::Fail, &trial.trial_id, operation, vec![error]);
        }
    };
    let expected_edges = match apply_intervention(&baseline_edges, &trial.intervention) {
        Ok(edges) => edges,
        Err(error) => {
            return report(RelationCounterfactualStatus::Fail, &trial.trial_id, operation, vec![error]);
        }
    };
    let treatment_edges = match validate_edge_set(&trial.counterfactual) {
        Ok(edges) => edges
            .into_iter()
            .map(|(id, edge)| (id.to_owned(), edge.clone()))
            .collect::<BTreeMap<_, _>>(),
        Err(error) => {
            return report(RelationCounterfactualStatus::Fail, &trial.trial_id, operation, vec![error]);
        }
    };
    if expected_edges != treatment_edges {
        return report(
            RelationCounterfactualStatus::Fail,
            &trial.trial_id,
            operation,
            vec!["counterfactual relation graph contains changes beyond the declared single-edge intervention".into()],
        );
    }
    if trial.baseline.relation_graph_digest == trial.counterfactual.relation_graph_digest {
        return report(
            RelationCounterfactualStatus::Fail,
            &trial.trial_id,
            operation,
            vec!["relation graph digest did not change after the declared intervention".into()],
        );
    }

    if !trial.baseline.candidate_pool_complete
        || !trial.counterfactual.candidate_pool_complete
        || !trial.baseline.relation_index_complete
        || !trial.counterfactual.relation_index_complete
    {
        return report(
            RelationCounterfactualStatus::Indeterminate,
            &trial.trial_id,
            operation,
            vec!["candidate pool or relation index is not attested complete for both runs".into()],
        );
    }
    let mut incomplete = missing_receipts(&trial.baseline, "baseline");
    incomplete.extend(missing_receipts(&trial.counterfactual, "counterfactual"));
    if !incomplete.is_empty() {
        let malformed = incomplete.iter().any(|reason| reason.contains("malformed"));
        return report(
            if malformed {
                RelationCounterfactualStatus::Fail
            } else {
                RelationCounterfactualStatus::Indeterminate
            },
            &trial.trial_id,
            operation,
            incomplete,
        );
    }

    let quality_delta =
        trial.counterfactual.outcome.decision_quality - trial.baseline.outcome.decision_quality;
    let mut result = report(
        RelationCounterfactualStatus::Pass,
        &trial.trial_id,
        operation,
        vec![
            "paired manifests match on benchmark, task, source group, model, tools, evaluator, budgets and randomization".into(),
            "candidate evidence pool and retrieval seeds are unchanged".into(),
            "the relation graph differs by exactly the declared edge intervention".into(),
            "this is one structurally qualified paired observation, not a population-level causal estimate".into(),
        ],
    );
    result.decision_changed = Some(
        trial.baseline.outcome.decision_digest != trial.counterfactual.outcome.decision_digest,
    );
    result.selected_evidence_changed = Some(
        trial.baseline.outcome.selected_evidence_ids
            != trial.counterfactual.outcome.selected_evidence_ids,
    );
    result.decision_quality_delta = Some(quality_delta);
    result.unauthorized_exposure_delta = Some(
        i128::from(trial.counterfactual.outcome.unauthorized_exposure_count)
            - i128::from(trial.baseline.outcome.unauthorized_exposure_count),
    );
    result.latency_delta_ms = Some(
        i128::from(trial.counterfactual.outcome.latency_ms)
            - i128::from(trial.baseline.outcome.latency_ms),
    );
    result.total_cost_delta_microusd = Some(
        i128::from(trial.counterfactual.outcome.total_cost_microusd)
            - i128::from(trial.baseline.outcome.total_cost_microusd),
    );
    result.baseline_constraint_gate = Some(trial.baseline.outcome.constraint_gate);
    result.counterfactual_constraint_gate = Some(trial.counterfactual.outcome.constraint_gate);
    result.baseline_response_audit_gate = Some(trial.baseline.outcome.response_audit_gate);
    result.counterfactual_response_audit_gate = Some(trial.counterfactual.outcome.response_audit_gate);
    result
}

#[cfg(test)]
mod tests {
    use super::*;

    fn digest(ch: char) -> String {
        std::iter::repeat_n(ch, 64).collect()
    }

    fn edge(id: &str, from: &str, to: &str, relation: EvidenceRelationKind, receipt: char) -> RelationEdgeSnapshot {
        RelationEdgeSnapshot {
            edge_id: id.into(),
            from_evidence_id: from.into(),
            to_evidence_id: to.into(),
            relation,
            relation_receipt_digest: digest(receipt),
        }
    }

    fn identity() -> RelationReplayIdentity {
        RelationReplayIdentity {
            benchmark_id: "GroupAssistBench".into(),
            benchmark_version: "v1".into(),
            benchmark_certificate_digest: digest('1'),
            split_digest: digest('2'),
            task_digest: digest('3'),
            source_group_id: "group-01".into(),
            candidate_pool_digest: digest('4'),
            retrieval_seed_digest: digest('5'),
            model_snapshot_digest: digest('6'),
            tokenizer_digest: digest('7'),
            prompt_digest: digest('8'),
            toolchain_digest: digest('9'),
            decoding_digest: digest('a'),
            scorer_digest: digest('b'),
            evaluator_digest: digest('c'),
            inference_plan_digest: digest('d'),
            seed_policy_digest: digest('e'),
            random_seed_digest: digest('f'),
            context_budget_tokens: 16_000,
            output_budget_tokens: 2_000,
            compute_budget_microusd: 100_000,
            wall_clock_budget_ms: 60_000,
            memory_budget_bytes: 1_000_000,
            agent_count: 1,
        }
    }

    fn run(run_id: &str, edges: Vec<RelationEdgeSnapshot>) -> RelationReplayRun {
        let relation_graph_digest = compute_relation_graph_digest(&edges).unwrap();
        RelationReplayRun {
            run_id: run_id.into(),
            identity: identity(),
            candidate_evidence_ids: vec!["e1".into(), "e2".into(), "e3".into()],
            retrieval_seed_evidence_ids: vec!["e1".into()],
            relation_graph_digest,
            relation_edges: edges,
            candidate_pool_complete: true,
            relation_index_complete: true,
            candidate_pool_receipt_digest: Some(digest('1')),
            relation_index_receipt_digest: Some(digest('2')),
            execution_receipt_digest: Some(digest('3')),
            generation_exposure_receipt_digest: Some(digest('4')),
            rer_bundle_receipt_digest: Some(digest('5')),
            constraint_receipt_digest: Some(digest('6')),
            response_audit_receipt_digest: Some(digest('7')),
            evaluator_receipt_digest: Some(digest('8')),
            outcome: RelationReplayOutcome {
                selected_evidence_ids: vec!["e1".into(), "e2".into()],
                decision_digest: digest('9'),
                decision_quality: 0.75,
                constraint_gate: RecordedGateOutcome::Pass,
                response_audit_gate: RecordedGateOutcome::Pass,
                unauthorized_exposure_count: 0,
                latency_ms: 1_000,
                total_cost_microusd: 50_000,
            },
        }
    }

    fn trial(
        operation: RelationInterventionOperation,
        before: Option<RelationEdgeSnapshot>,
        after: Option<RelationEdgeSnapshot>,
        baseline_edges: Vec<RelationEdgeSnapshot>,
        treatment_edges: Vec<RelationEdgeSnapshot>,
    ) -> RelationCounterfactualTrial {
        RelationCounterfactualTrial {
            trial_id: "trial-01".into(),
            intervention: RelationIntervention {
                intervention_id: "edge-treatment-01".into(),
                operation,
                before,
                after,
            },
            baseline: run("baseline-01", baseline_edges),
            counterfactual: run("counterfactual-01", treatment_edges),
        }
    }

    #[test]
    fn removing_one_edge_qualifies_and_preserves_candidate_pool() {
        let relation = edge("edge-1", "e1", "e2", EvidenceRelationKind::Supports, 'a');
        let unchanged = edge("edge-2", "e2", "e3", EvidenceRelationKind::Updates, 'b');
        let result = qualify_relation_counterfactual(&trial(
            RelationInterventionOperation::Remove,
            Some(relation.clone()),
            None,
            vec![relation, unchanged.clone()],
            vec![unchanged],
        ));
        assert_eq!(result.status, RelationCounterfactualStatus::Pass);
    }

    #[test]
    fn adding_one_edge_qualifies_when_no_other_edge_changes() {
        let existing = edge("edge-1", "e1", "e2", EvidenceRelationKind::Supports, 'a');
        let added = edge("edge-2", "e2", "e3", EvidenceRelationKind::Contradicts, 'b');
        let result = qualify_relation_counterfactual(&trial(
            RelationInterventionOperation::Add,
            None,
            Some(added.clone()),
            vec![existing.clone()],
            vec![existing, added],
        ));
        assert_eq!(result.status, RelationCounterfactualStatus::Pass);
    }

    #[test]
    fn reversing_a_directed_edge_requires_a_new_receipt() {
        let before = edge("edge-1", "e1", "e2", EvidenceRelationKind::Supports, 'a');
        let after = edge("edge-1", "e2", "e1", EvidenceRelationKind::Supports, 'b');
        let result = qualify_relation_counterfactual(&trial(
            RelationInterventionOperation::Reverse,
            Some(before.clone()),
            Some(after.clone()),
            vec![before],
            vec![after],
        ));
        assert_eq!(result.status, RelationCounterfactualStatus::Pass);
    }

    #[test]
    fn symmetric_relation_cannot_be_direction_reversed() {
        let before = edge("edge-1", "e1", "e2", EvidenceRelationKind::SameEvent, 'a');
        let after = edge("edge-1", "e2", "e1", EvidenceRelationKind::SameEvent, 'b');
        let result = qualify_relation_counterfactual(&trial(
            RelationInterventionOperation::Reverse,
            Some(before.clone()),
            Some(after.clone()),
            vec![before],
            vec![after],
        ));
        assert_eq!(result.status, RelationCounterfactualStatus::Fail);
    }

    #[test]
    fn relabelling_an_edge_qualifies_with_new_relation_receipt() {
        let before = edge("edge-1", "e1", "e2", EvidenceRelationKind::Supports, 'a');
        let after = edge("edge-1", "e1", "e2", EvidenceRelationKind::Contradicts, 'b');
        let result = qualify_relation_counterfactual(&trial(
            RelationInterventionOperation::Relabel,
            Some(before.clone()),
            Some(after.clone()),
            vec![before],
            vec![after],
        ));
        assert_eq!(result.status, RelationCounterfactualStatus::Pass);
    }

    #[test]
    fn extra_unrelated_edge_change_fails_exact_difference_check() {
        let target = edge("edge-1", "e1", "e2", EvidenceRelationKind::Supports, 'a');
        let unchanged = edge("edge-2", "e2", "e3", EvidenceRelationKind::Updates, 'b');
        let mutated = edge("edge-2", "e2", "e3", EvidenceRelationKind::Updates, 'c');
        let result = qualify_relation_counterfactual(&trial(
            RelationInterventionOperation::Remove,
            Some(target.clone()),
            None,
            vec![target, unchanged],
            vec![mutated],
        ));
        assert_eq!(result.status, RelationCounterfactualStatus::Fail);
    }

    #[test]
    fn model_or_budget_mismatch_fails_matched_pair() {
        let relation = edge("edge-1", "e1", "e2", EvidenceRelationKind::Supports, 'a');
        let mut pair = trial(
            RelationInterventionOperation::Remove,
            Some(relation.clone()),
            None,
            vec![relation.clone()],
            vec![],
        );
        pair.counterfactual.identity.model_snapshot_digest = digest('0');
        let result = qualify_relation_counterfactual(&pair);
        assert_eq!(result.status, RelationCounterfactualStatus::Fail);
    }

    #[test]
    fn missing_relation_receipt_is_indeterminate() {
        let relation = edge("edge-1", "e1", "e2", EvidenceRelationKind::Supports, 'a');
        let mut pair = trial(
            RelationInterventionOperation::Remove,
            Some(relation.clone()),
            None,
            vec![relation],
            vec![],
        );
        pair.baseline.relation_index_receipt_digest = None;
        let result = qualify_relation_counterfactual(&pair);
        assert_eq!(result.status, RelationCounterfactualStatus::Indeterminate);
    }

    #[test]
    fn incomplete_candidate_pool_is_indeterminate() {
        let relation = edge("edge-1", "e1", "e2", EvidenceRelationKind::Supports, 'a');
        let mut pair = trial(
            RelationInterventionOperation::Remove,
            Some(relation.clone()),
            None,
            vec![relation],
            vec![],
        );
        pair.counterfactual.candidate_pool_complete = false;
        let result = qualify_relation_counterfactual(&pair);
        assert_eq!(result.status, RelationCounterfactualStatus::Indeterminate);
    }

    #[test]
    fn edge_with_endpoint_outside_candidate_pool_fails() {
        let relation = edge("edge-1", "e1", "external", EvidenceRelationKind::Supports, 'a');
        let result = qualify_relation_counterfactual(&trial(
            RelationInterventionOperation::Remove,
            Some(relation.clone()),
            None,
            vec![relation],
            vec![],
        ));
        assert_eq!(result.status, RelationCounterfactualStatus::Fail);
    }

    #[test]
    fn paired_report_exposes_quality_cost_and_privacy_deltas() {
        let relation = edge("edge-1", "e1", "e2", EvidenceRelationKind::Supports, 'a');
        let mut pair = trial(
            RelationInterventionOperation::Remove,
            Some(relation.clone()),
            None,
            vec![relation],
            vec![],
        );
        pair.counterfactual.outcome.decision_quality = 0.60;
        pair.counterfactual.outcome.unauthorized_exposure_count = 1;
        pair.counterfactual.outcome.latency_ms = 1_200;
        pair.counterfactual.outcome.total_cost_microusd = 60_000;
        pair.counterfactual.outcome.decision_digest = digest('0');
        pair.counterfactual.outcome.selected_evidence_ids = vec!["e1".into()];
        let result = qualify_relation_counterfactual(&pair);
        assert_eq!(result.status, RelationCounterfactualStatus::Pass);
        assert_eq!(result.decision_changed, Some(true));
        assert_eq!(result.selected_evidence_changed, Some(true));
        assert!((result.decision_quality_delta.unwrap() + 0.15).abs() < 1e-9);
        assert_eq!(result.unauthorized_exposure_delta, Some(1));
        assert_eq!(result.latency_delta_ms, Some(200));
        assert_eq!(result.total_cost_delta_microusd, Some(10_000));
        assert_eq!(result.baseline_constraint_gate, Some(RecordedGateOutcome::Pass));
        assert_eq!(result.counterfactual_response_audit_gate, Some(RecordedGateOutcome::Pass));
    }
}
