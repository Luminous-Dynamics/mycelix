// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Deterministic, inspectable learning-plan compilation.
//!
//! This module turns a learner-chosen target capability and a versioned capability
//! graph into a bounded proposal. It chooses prerequisite-first tasks from supplied
//! catalog data, binds the proposal to the exact normalized graph/request digest,
//! and fails closed when the target cannot be planned within the declared constraints.
//! Validation is structural only: the digest does not authenticate the catalogue or
//! establish that a task is pedagogically effective.
//!
//! This is not an optimizer of pedagogical efficacy. It does not infer mastery,
//! admit evidence, issue credentials, or authorize work. Previously demonstrated
//! capabilities are deliberately not accepted as bare caller-supplied skip flags:
//! recognition of prior evidence belongs to a separate, policy-authorized boundary.
//!
//! Input order is not semantically meaningful for graph nodes, prerequisites, or
//! practice-option alternatives. These are canonicalized before the input digest is
//! computed so equivalent catalogues produce the same digest and task route.

use crate::learning_evidence::CapabilityId;
use serde::{Deserialize, Serialize};
use std::cmp::Ordering;
use std::collections::{BTreeMap, BTreeSet};
use thiserror::Error;

pub const CAPABILITY_PLAN_CONTRACT_VERSION: u16 = 1;
pub const CAPABILITY_PLAN_COMPILER_ID: &str = "praxis:capability-plan-compiler";
pub const CAPABILITY_PLAN_COMPILER_VERSION: &str = "deterministic-dag-v1";
pub const MAX_CAPABILITY_NODES: usize = 4_096;
pub const MAX_PRACTICE_OPTIONS_PER_CAPABILITY: usize = 64;
pub const MAX_IDENTIFIER_BYTES: usize = 256;
pub const MAX_TITLE_BYTES: usize = 512;
pub const MAX_ESTIMATED_TASK_MINUTES: u32 = 10_000_000;

/// A delivery modality is a constraint/preferences label, not an assumption about
/// a learner's aptitude. Accessibility needs should be represented separately.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DeliveryMode {
    SelfPacedStudy,
    RetrievalPractice,
    GuidedPractice,
    Simulation,
    SupervisedPractical,
    CollaborativeProject,
}

/// One versioned task/rubric option for developing or demonstrating a capability.
/// Estimated duration is planning metadata, not a promise about an individual.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PracticeOption {
    pub task_id: String,
    pub task_version: String,
    pub rubric_id: String,
    pub rubric_version: String,
    /// Curator-authored ordering, not a prediction of learning efficacy.
    /// Lower values are preferred unless the learner chooses an eligible option.
    pub route_priority: u16,
    /// Human-readable rationale for including this practice option.
    pub rationale: String,
    pub estimated_minutes: u32,
    pub delivery_mode: DeliveryMode,
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PracticeOptionRef {
    pub task_id: String,
    pub task_version: String,
    pub rubric_id: String,
    pub rubric_version: String,
    pub delivery_mode: DeliveryMode,
}

impl From<&PracticeOption> for PracticeOptionRef {
    fn from(option: &PracticeOption) -> Self {
        Self {
            task_id: option.task_id.clone(),
            task_version: option.task_version.clone(),
            rubric_id: option.rubric_id.clone(),
            rubric_version: option.rubric_version.clone(),
            delivery_mode: option.delivery_mode.clone(),
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CapabilityNode {
    pub capability_id: CapabilityId,
    pub version: String,
    pub title: String,
    /// Capabilities that should be learned before this node's selected task.
    pub prerequisites: Vec<CapabilityId>,
    /// Alternative tasks for this capability. The compiler selects one eligible
    /// option deterministically; the learner may later request another route.
    pub practice_options: Vec<PracticeOption>,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CapabilityGraph {
    pub graph_id: String,
    pub revision: String,
    pub nodes: Vec<CapabilityNode>,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct GoalPlanConstraints {
    pub allowed_delivery_modes: BTreeSet<DeliveryMode>,
    pub max_task_minutes: u32,
    pub max_total_minutes: u32,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct GoalPlanRequest {
    pub request_id: String,
    pub target_capability: CapabilityId,
    pub constraints: GoalPlanConstraints,
    /// Learner-selected options keyed by capability. Each choice is checked against
    /// the versioned graph and all request constraints.
    pub preferred_practice: BTreeMap<CapabilityId, PracticeOptionRef>,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PracticeSelectionBasis {
    CuratorPriority,
    LearnerPreference,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct GoalPlanStep {
    pub ordinal: u32,
    pub capability_id: CapabilityId,
    pub capability_version: String,
    pub title: String,
    /// Direct prerequisites, sorted by stable capability ID.
    pub prerequisites: Vec<CapabilityId>,
    /// Proposed option, not a silently accepted learner choice.
    pub proposed_practice: PracticeOption,
    /// Other eligible options remain visible to support learner choice.
    pub eligible_alternatives: Vec<PracticeOption>,
    pub selection_basis: PracticeSelectionBasis,
}

/// An advisory route from a learner-selected goal to versioned practice tasks.
/// The plan is not proof of competence or a credential. Each task keeps its rubric
/// version visible so it cannot silently inherit new assessment semantics.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct LearningPlanProposal {
    pub request_id: String,
    pub target_capability: CapabilityId,
    pub graph_id: String,
    pub graph_revision: String,
    pub compiler_id: String,
    pub compiler_version: String,
    /// BLAKE3 of the normalized graph and request; it binds bytes/inputs, not truth.
    pub input_digest: String,
    pub generated_at: i64,
    pub estimated_total_minutes: u32,
    pub steps: Vec<GoalPlanStep>,
}

#[derive(Clone, Debug, PartialEq, Eq, Error)]
pub enum GoalPlanError {
    #[error("invalid or empty field: {field}")]
    InvalidField { field: &'static str },
    #[error("capability graph has {actual} nodes; the limit is {maximum}")]
    GraphTooLarge { actual: usize, maximum: usize },
    #[error("capability graph must contain at least one node")]
    EmptyGraph,
    #[error("duplicate capability identifier: {capability_id}")]
    DuplicateCapability { capability_id: String },
    #[error("unknown target capability: {capability_id}")]
    UnknownTarget { capability_id: String },
    #[error("unknown prerequisite {prerequisite_id} referenced by {capability_id}")]
    UnknownPrerequisite {
        capability_id: String,
        prerequisite_id: String,
    },
    #[error("capability {capability_id} lists prerequisite {prerequisite_id} more than once")]
    DuplicatePrerequisite {
        capability_id: String,
        prerequisite_id: String,
    },
    #[error("capability {capability_id} cannot depend on itself")]
    SelfDependency { capability_id: String },
    #[error("duplicate practice-option identity for capability {capability_id}: {task_id}")]
    DuplicatePracticeOption {
        capability_id: String,
        task_id: String,
    },
    #[error("capability {capability_id} has too many practice options: {actual}")]
    TooManyPracticeOptions {
        capability_id: String,
        actual: usize,
    },
    #[error("dependency cycle includes capability: {capability_id}")]
    DependencyCycle { capability_id: String },
    #[error("no practice option satisfies the constraints for capability: {capability_id}")]
    NoEligiblePracticeOption { capability_id: String },
    #[error("preferred capability {capability_id} is outside the target prerequisite closure")]
    PreferenceOutsideGoalClosure { capability_id: String },
    #[error("preferred practice option {task_id} is not eligible for capability {capability_id}")]
    PreferredPracticeNotEligible { capability_id: String, task_id: String },
    #[error("generated_at must be nonnegative: {timestamp}")]
    NegativeTimestamp { timestamp: i64 },
    #[error("estimated duration overflowed the supported integer range")]
    DurationOverflow,
    #[error("required estimated duration {required_minutes} minutes exceeds budget {budget_minutes} minutes")]
    TotalBudgetExceeded {
        required_minutes: u32,
        budget_minutes: u32,
    },
    #[error("could not serialize canonical plan inputs: {message}")]
    CanonicalEncodingFailed { message: String },
}

#[derive(Serialize)]
struct CanonicalPlanInput<'a> {
    contract_version: u16,
    graph: &'a CapabilityGraph,
    request: &'a GoalPlanRequest,
}

/// Compile a deterministic prerequisite-first plan proposal.
///
/// The compiler validates the entire graph's structural references, but cycle
/// detection is scoped to the target's prerequisite closure: an unrelated malformed
/// cycle must not change a learner's otherwise-valid route. Missing references in
/// any graph node remain a catalogue integrity error.
pub fn compile_goal_plan(
    graph: &CapabilityGraph,
    request: &GoalPlanRequest,
    generated_at: i64,
) -> Result<LearningPlanProposal, GoalPlanError> {
    if generated_at < 0 {
        return Err(GoalPlanError::NegativeTimestamp {
            timestamp: generated_at,
        });
    }
    validate_request(request)?;
    let nodes = validate_graph(graph)?;

    if !nodes.contains_key(&request.target_capability) {
        return Err(GoalPlanError::UnknownTarget {
            capability_id: request.target_capability.0.clone(),
        });
    }

    let closure = dependency_closure(&request.target_capability, &nodes);
    for capability_id in request.preferred_practice.keys() {
        if !closure.contains(capability_id) {
            return Err(GoalPlanError::PreferenceOutsideGoalClosure {
                capability_id: capability_id.0.clone(),
            });
        }
    }
    let ordered_ids = topological_order(&closure, &nodes)?;

    let mut steps = Vec::with_capacity(ordered_ids.len());
    let mut estimated_total_minutes = 0_u32;

    for (index, capability_id) in ordered_ids.iter().enumerate() {
        let node = nodes
            .get(capability_id)
            .expect("dependency closure only contains validated graph nodes");

        let mut eligible_practices: Vec<PracticeOption> = node
            .practice_options
            .iter()
            .filter(|option| {
                option.estimated_minutes <= request.constraints.max_task_minutes
                    && request
                        .constraints
                        .allowed_delivery_modes
                        .contains(&option.delivery_mode)
            })
            .cloned()
            .collect();
        eligible_practices.sort_by(practice_option_order);
        if eligible_practices.is_empty() {
            return Err(GoalPlanError::NoEligiblePracticeOption {
                capability_id: capability_id.0.clone(),
            });
        }

        let preferred = request.preferred_practice.get(capability_id);
        let proposed_practice = if let Some(preferred) = preferred {
            eligible_practices
                .iter()
                .find(|option| option_matches_ref(option, preferred))
                .cloned()
                .ok_or_else(|| GoalPlanError::PreferredPracticeNotEligible {
                    capability_id: capability_id.0.clone(),
                    task_id: preferred.task_id.clone(),
                })?
        } else {
            eligible_practices[0].clone()
        };
        let selection_basis = if preferred.is_some() {
            PracticeSelectionBasis::LearnerPreference
        } else {
            PracticeSelectionBasis::CuratorPriority
        };

        estimated_total_minutes = estimated_total_minutes
            .checked_add(proposed_practice.estimated_minutes)
            .ok_or(GoalPlanError::DurationOverflow)?;

        let mut prerequisites = node.prerequisites.clone();
        prerequisites.sort();
        let eligible_alternatives = eligible_practices
            .into_iter()
            .filter(|option| !same_practice_identity(option, &proposed_practice))
            .collect();

        steps.push(GoalPlanStep {
            ordinal: u32::try_from(index + 1).map_err(|_| GoalPlanError::DurationOverflow)?,
            capability_id: node.capability_id.clone(),
            capability_version: node.version.clone(),
            title: node.title.clone(),
            prerequisites,
            proposed_practice,
            eligible_alternatives,
            selection_basis,
        });
    }

    if estimated_total_minutes > request.constraints.max_total_minutes {
        return Err(GoalPlanError::TotalBudgetExceeded {
            required_minutes: estimated_total_minutes,
            budget_minutes: request.constraints.max_total_minutes,
        });
    }

    let canonical_graph = canonicalize_graph(graph.clone());
    let canonical_bytes = serde_json::to_vec(&CanonicalPlanInput {
        contract_version: CAPABILITY_PLAN_CONTRACT_VERSION,
        graph: &canonical_graph,
        request,
    })
    .map_err(|error| GoalPlanError::CanonicalEncodingFailed {
        message: error.to_string(),
    })?;
    let input_digest = format!("blake3:{}", blake3::hash(&canonical_bytes).to_hex());

    Ok(LearningPlanProposal {
        request_id: request.request_id.clone(),
        target_capability: request.target_capability.clone(),
        graph_id: graph.graph_id.clone(),
        graph_revision: graph.revision.clone(),
        compiler_id: CAPABILITY_PLAN_COMPILER_ID.to_string(),
        compiler_version: CAPABILITY_PLAN_COMPILER_VERSION.to_string(),
        input_digest,
        generated_at,
        estimated_total_minutes,
        steps,
    })
}

fn validate_request(request: &GoalPlanRequest) -> Result<(), GoalPlanError> {
    validate_identifier(&request.request_id, "request_id")?;
    validate_identifier(&request.target_capability.0, "target_capability")?;

    if request.constraints.allowed_delivery_modes.is_empty() {
        return Err(GoalPlanError::InvalidField {
            field: "allowed_delivery_modes",
        });
    }
    if request.constraints.max_task_minutes == 0 {
        return Err(GoalPlanError::InvalidField {
            field: "max_task_minutes",
        });
    }
    if request.constraints.max_total_minutes == 0 {
        return Err(GoalPlanError::InvalidField {
            field: "max_total_minutes",
        });
    }
    for (capability_id, option) in &request.preferred_practice {
        validate_identifier(&capability_id.0, "preferred_capability_id")?;
        validate_identifier(&option.task_id, "preferred_task_id")?;
        validate_identifier(&option.task_version, "preferred_task_version")?;
        validate_identifier(&option.rubric_id, "preferred_rubric_id")?;
        validate_identifier(&option.rubric_version, "preferred_rubric_version")?;
    }
    Ok(())
}

fn validate_graph<'a>(
    graph: &'a CapabilityGraph,
) -> Result<BTreeMap<CapabilityId, &'a CapabilityNode>, GoalPlanError> {
    validate_identifier(&graph.graph_id, "graph_id")?;
    validate_identifier(&graph.revision, "graph_revision")?;

    if graph.nodes.is_empty() {
        return Err(GoalPlanError::EmptyGraph);
    }
    if graph.nodes.len() > MAX_CAPABILITY_NODES {
        return Err(GoalPlanError::GraphTooLarge {
            actual: graph.nodes.len(),
            maximum: MAX_CAPABILITY_NODES,
        });
    }

    let mut nodes = BTreeMap::new();
    for node in &graph.nodes {
        validate_identifier(&node.capability_id.0, "capability_id")?;
        validate_identifier(&node.version, "capability_version")?;
        validate_text(&node.title, "capability_title", MAX_TITLE_BYTES)?;

        if nodes
            .insert(node.capability_id.clone(), node)
            .is_some()
        {
            return Err(GoalPlanError::DuplicateCapability {
                capability_id: node.capability_id.0.clone(),
            });
        }

        if node.practice_options.len() > MAX_PRACTICE_OPTIONS_PER_CAPABILITY {
            return Err(GoalPlanError::TooManyPracticeOptions {
                capability_id: node.capability_id.0.clone(),
                actual: node.practice_options.len(),
            });
        }

        let mut option_ids = BTreeSet::new();
        for option in &node.practice_options {
            validate_identifier(&option.task_id, "task_id")?;
            validate_identifier(&option.task_version, "task_version")?;
            validate_identifier(&option.rubric_id, "rubric_id")?;
            validate_identifier(&option.rubric_version, "rubric_version")?;
            validate_text(&option.rationale, "practice_rationale", MAX_TITLE_BYTES)?;
            if option.estimated_minutes == 0
                || option.estimated_minutes > MAX_ESTIMATED_TASK_MINUTES
            {
                return Err(GoalPlanError::InvalidField {
                    field: "estimated_minutes",
                });
            }

            // A task/rubric pair may offer multiple modalities, but the same
            // pair/version/modality must not be declared twice.
            let identity = (
                option.task_id.clone(),
                option.task_version.clone(),
                option.rubric_id.clone(),
                option.rubric_version.clone(),
                option.delivery_mode.clone(),
            );
            if !option_ids.insert(identity) {
                return Err(GoalPlanError::DuplicatePracticeOption {
                    capability_id: node.capability_id.0.clone(),
                    task_id: option.task_id.clone(),
                });
            }
        }
    }

    for node in &graph.nodes {
        let mut prerequisites = BTreeSet::new();
        for prerequisite in &node.prerequisites {
            validate_identifier(&prerequisite.0, "prerequisite_capability_id")?;
            if prerequisite == &node.capability_id {
                return Err(GoalPlanError::SelfDependency {
                    capability_id: node.capability_id.0.clone(),
                });
            }
            if !prerequisites.insert(prerequisite.clone()) {
                return Err(GoalPlanError::DuplicatePrerequisite {
                    capability_id: node.capability_id.0.clone(),
                    prerequisite_id: prerequisite.0.clone(),
                });
            }
            if !nodes.contains_key(prerequisite) {
                return Err(GoalPlanError::UnknownPrerequisite {
                    capability_id: node.capability_id.0.clone(),
                    prerequisite_id: prerequisite.0.clone(),
                });
            }
        }
    }

    Ok(nodes)
}

fn validate_identifier(value: &str, field: &'static str) -> Result<(), GoalPlanError> {
    if value.trim().is_empty()
        || value.len() > MAX_IDENTIFIER_BYTES
        || value.chars().any(char::is_control)
    {
        return Err(GoalPlanError::InvalidField { field });
    }
    Ok(())
}

fn validate_text(
    value: &str,
    field: &'static str,
    maximum_bytes: usize,
) -> Result<(), GoalPlanError> {
    if value.trim().is_empty()
        || value.len() > maximum_bytes
        || value.chars().any(char::is_control)
    {
        return Err(GoalPlanError::InvalidField { field });
    }
    Ok(())
}

fn dependency_closure<'a>(
    target: &CapabilityId,
    nodes: &BTreeMap<CapabilityId, &'a CapabilityNode>,
) -> BTreeSet<CapabilityId> {
    let mut closure = BTreeSet::new();
    let mut stack = vec![target.clone()];

    while let Some(current) = stack.pop() {
        if !closure.insert(current.clone()) {
            continue;
        }
        if let Some(node) = nodes.get(&current) {
            stack.extend(node.prerequisites.iter().cloned());
        }
    }
    closure
}

fn topological_order<'a>(
    closure: &BTreeSet<CapabilityId>,
    nodes: &BTreeMap<CapabilityId, &'a CapabilityNode>,
) -> Result<Vec<CapabilityId>, GoalPlanError> {
    let mut in_degree = BTreeMap::<CapabilityId, usize>::new();
    let mut dependents = BTreeMap::<CapabilityId, BTreeSet<CapabilityId>>::new();

    for capability_id in closure {
        let node = nodes
            .get(capability_id)
            .expect("dependency closure only contains validated graph nodes");
        let in_scope_prerequisites = node
            .prerequisites
            .iter()
            .filter(|prerequisite| closure.contains(*prerequisite))
            .count();
        in_degree.insert(capability_id.clone(), in_scope_prerequisites);
        for prerequisite in &node.prerequisites {
            if closure.contains(prerequisite) {
                dependents
                    .entry(prerequisite.clone())
                    .or_default()
                    .insert(capability_id.clone());
            }
        }
    }

    let mut ready: BTreeSet<CapabilityId> = in_degree
        .iter()
        .filter_map(|(capability_id, count)| {
            (*count == 0).then_some(capability_id.clone())
        })
        .collect();
    let mut ordered = Vec::with_capacity(closure.len());

    while let Some(next) = ready.iter().next().cloned() {
        ready.remove(&next);
        ordered.push(next.clone());

        if let Some(children) = dependents.get(&next) {
            for child in children {
                let count = in_degree
                    .get_mut(child)
                    .expect("every dependent belongs to the closure");
                *count -= 1;
                if *count == 0 {
                    ready.insert(child.clone());
                }
            }
        }
    }

    if ordered.len() != closure.len() {
        let capability_id = in_degree
            .iter()
            .find_map(|(id, degree)| (*degree > 0).then_some(id.0.clone()))
            .unwrap_or_else(|| "unknown".to_string());
        return Err(GoalPlanError::DependencyCycle { capability_id });
    }
    Ok(ordered)
}

fn canonicalize_graph(mut graph: CapabilityGraph) -> CapabilityGraph {
    graph.nodes.sort_by(|left, right| left.capability_id.cmp(&right.capability_id));
    for node in &mut graph.nodes {
        node.prerequisites.sort();
        node.practice_options
            .sort_by(|left, right| practice_option_order(left, right));
    }
    graph
}

fn practice_option_order(left: &PracticeOption, right: &PracticeOption) -> Ordering {
    left.route_priority
        .cmp(&right.route_priority)
        .then_with(|| left.task_id.cmp(&right.task_id))
        .then_with(|| left.task_version.cmp(&right.task_version))
        .then_with(|| left.rubric_id.cmp(&right.rubric_id))
        .then_with(|| left.rubric_version.cmp(&right.rubric_version))
        .then_with(|| left.delivery_mode.cmp(&right.delivery_mode))
}

fn option_matches_ref(option: &PracticeOption, preferred: &PracticeOptionRef) -> bool {
    option.task_id == preferred.task_id
        && option.task_version == preferred.task_version
        && option.rubric_id == preferred.rubric_id
        && option.rubric_version == preferred.rubric_version
        && option.delivery_mode == preferred.delivery_mode
}

fn same_practice_identity(left: &PracticeOption, right: &PracticeOption) -> bool {
    option_matches_ref(left, &PracticeOptionRef::from(right))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn id(value: &str) -> CapabilityId {
        CapabilityId(value.to_string())
    }

    fn option(
        task: &str,
        task_version: &str,
        rubric: &str,
        rubric_version: &str,
        minutes: u32,
        mode: DeliveryMode,
    ) -> PracticeOption {
        PracticeOption {
            task_id: task.to_string(),
            task_version: task_version.to_string(),
            rubric_id: rubric.to_string(),
            rubric_version: rubric_version.to_string(),
            route_priority: 100,
            rationale: format!("Fixture rationale for {task}; efficacy not measured"),
            estimated_minutes: minutes,
            delivery_mode: mode,
        }
    }

    fn node(
        capability: &str,
        title: &str,
        prerequisites: &[&str],
        options: Vec<PracticeOption>,
    ) -> CapabilityNode {
        CapabilityNode {
            capability_id: id(capability),
            version: "1.0.0".into(),
            title: title.into(),
            prerequisites: prerequisites.iter().map(|value| id(value)).collect(),
            practice_options: options,
        }
    }

    fn graph(nodes: Vec<CapabilityNode>) -> CapabilityGraph {
        CapabilityGraph {
            graph_id: "essential-capabilities".into(),
            revision: "rev-7".into(),
            nodes,
        }
    }

    fn request(target: &str, allowed_modes: &[DeliveryMode], max_task: u32, total: u32) -> GoalPlanRequest {
        GoalPlanRequest {
            request_id: "request-001".into(),
            target_capability: id(target),
            constraints: GoalPlanConstraints {
                allowed_delivery_modes: allowed_modes.iter().cloned().collect(),
                max_task_minutes: max_task,
                max_total_minutes: total,
            },
            preferred_practice: BTreeMap::new(),
        }
    }

    fn basic_graph() -> CapabilityGraph {
        graph(vec![
            node(
                "cap-net-debug",
                "Diagnose a network fault",
                &["cap-ip-basics", "cap-observation"],
                vec![
                    option("task-network-lab", "2", "rubric-network", "3", 35, DeliveryMode::Simulation),
                    option("task-network-practical", "1", "rubric-network", "3", 50, DeliveryMode::SupervisedPractical),
                ],
            ),
            node(
                "cap-observation",
                "Collect relevant signals",
                &[],
                vec![option("task-observe", "1", "rubric-observe", "1", 10, DeliveryMode::GuidedPractice)],
            ),
            node(
                "cap-ip-basics",
                "Understand IP addressing",
                &[],
                vec![option("task-ip-retrieval", "1", "rubric-ip", "1", 15, DeliveryMode::RetrievalPractice)],
            ),
        ])
    }

    #[test]
    fn prerequisites_appear_before_target() {
        let plan = compile_goal_plan(
            &basic_graph(),
            &request(
                "cap-net-debug",
                &[DeliveryMode::GuidedPractice, DeliveryMode::RetrievalPractice, DeliveryMode::Simulation],
                60,
                100,
            ),
            1_800_000_000,
        )
        .unwrap();

        let ids: Vec<&str> = plan.steps.iter().map(|step| step.capability_id.0.as_str()).collect();
        assert_eq!(ids, vec!["cap-ip-basics", "cap-observation", "cap-net-debug"]);
        assert_eq!(plan.estimated_total_minutes, 60);
        assert_eq!(plan.steps.last().unwrap().proposed_practice.task_id, "task-network-lab");
        assert!(plan.input_digest.starts_with("blake3:"));
        assert_eq!(plan.compiler_version, CAPABILITY_PLAN_COMPILER_VERSION);
    }

    #[test]
    fn equivalent_graph_order_produces_same_digest_and_route() {
        let original = basic_graph();
        let mut reordered = original.clone();
        reordered.nodes.reverse();
        for node in &mut reordered.nodes {
            node.prerequisites.reverse();
            node.practice_options.reverse();
        }
        let request = request(
            "cap-net-debug",
            &[DeliveryMode::GuidedPractice, DeliveryMode::RetrievalPractice, DeliveryMode::Simulation],
            60,
            100,
        );

        let left = compile_goal_plan(&original, &request, 100).unwrap();
        let right = compile_goal_plan(&reordered, &request, 100).unwrap();
        assert_eq!(left.input_digest, right.input_digest);
        assert_eq!(left.steps, right.steps);
        assert_eq!(left.estimated_total_minutes, right.estimated_total_minutes);
    }

    #[test]
    fn changed_rubric_version_changes_input_digest() {
        let original = basic_graph();
        let mut changed = original.clone();
        changed.nodes[0].practice_options[0].rubric_version = "4".into();
        let request = request(
            "cap-net-debug",
            &[DeliveryMode::GuidedPractice, DeliveryMode::RetrievalPractice, DeliveryMode::Simulation],
            60,
            100,
        );
        let left = compile_goal_plan(&original, &request, 100).unwrap();
        let right = compile_goal_plan(&changed, &request, 100).unwrap();
        assert_ne!(left.input_digest, right.input_digest);
    }

    #[test]
    fn disallowed_mode_is_excluded_from_the_proposal() {
        let plan = compile_goal_plan(
            &basic_graph(),
            &request(
                "cap-net-debug",
                &[
                    DeliveryMode::GuidedPractice,
                    DeliveryMode::RetrievalPractice,
                    DeliveryMode::SupervisedPractical,
                ],
                60,
                100,
            ),
            100,
        )
        .unwrap();

        assert_eq!(
            plan.steps.last().unwrap().proposed_practice.delivery_mode,
            DeliveryMode::SupervisedPractical
        );
        assert_eq!(plan.steps.last().unwrap().proposed_practice.estimated_minutes, 50);
    }

    #[test]
    fn curator_priority_is_not_a_shortest_duration_heuristic() {
        let mut graph = basic_graph();
        let target = graph
            .nodes
            .iter_mut()
            .find(|node| node.capability_id == id("cap-net-debug"))
            .unwrap();
        target.practice_options[0].route_priority = 20;
        target.practice_options[1].route_priority = 1;
        let plan = compile_goal_plan(
            &graph,
            &request(
                "cap-net-debug",
                &[
                    DeliveryMode::GuidedPractice,
                    DeliveryMode::RetrievalPractice,
                    DeliveryMode::Simulation,
                    DeliveryMode::SupervisedPractical,
                ],
                60,
                100,
            ),
            100,
        )
        .unwrap();

        let target_step = plan.steps.last().unwrap();
        assert_eq!(target_step.proposed_practice.task_id, "task-network-practical");
        assert_eq!(target_step.proposed_practice.estimated_minutes, 50);
        assert_eq!(target_step.selection_basis, PracticeSelectionBasis::CuratorPriority);
        assert_eq!(target_step.eligible_alternatives.len(), 1);
        assert_eq!(target_step.eligible_alternatives[0].task_id, "task-network-lab");
    }

    #[test]
    fn learner_can_choose_an_eligible_alternative() {
        let graph = basic_graph();
        let target = graph
            .nodes
            .iter()
            .find(|node| node.capability_id == id("cap-net-debug"))
            .unwrap();
        let mut request = request(
            "cap-net-debug",
            &[
                DeliveryMode::GuidedPractice,
                DeliveryMode::RetrievalPractice,
                DeliveryMode::Simulation,
                DeliveryMode::SupervisedPractical,
            ],
            60,
            100,
        );
        request.preferred_practice.insert(
            id("cap-net-debug"),
            PracticeOptionRef::from(&target.practice_options[1]),
        );

        let plan = compile_goal_plan(&graph, &request, 100).unwrap();
        let target_step = plan.steps.last().unwrap();
        assert_eq!(target_step.proposed_practice.task_id, "task-network-practical");
        assert_eq!(target_step.selection_basis, PracticeSelectionBasis::LearnerPreference);
        assert_eq!(target_step.eligible_alternatives.len(), 1);
        assert_eq!(target_step.eligible_alternatives[0].task_id, "task-network-lab");
    }

    #[test]
    fn preference_for_capability_outside_goal_closure_is_rejected() {
        let graph = basic_graph();
        let target = graph
            .nodes
            .iter()
            .find(|node| node.capability_id == id("cap-net-debug"))
            .unwrap();
        let mut request = request("cap-observation", &[DeliveryMode::GuidedPractice], 60, 100);
        request.preferred_practice.insert(
            id("cap-net-debug"),
            PracticeOptionRef::from(&target.practice_options[0]),
        );

        assert_eq!(
            compile_goal_plan(&graph, &request, 100),
            Err(GoalPlanError::PreferenceOutsideGoalClosure {
                capability_id: "cap-net-debug".into()
            })
        );
    }

    #[test]
    fn preferred_option_that_violates_constraints_is_rejected() {
        let graph = basic_graph();
        let target = graph
            .nodes
            .iter()
            .find(|node| node.capability_id == id("cap-net-debug"))
            .unwrap();
        let mut request = request(
            "cap-net-debug",
            &[
                DeliveryMode::GuidedPractice,
                DeliveryMode::RetrievalPractice,
                DeliveryMode::Simulation,
            ],
            60,
            100,
        );
        request.preferred_practice.insert(
            id("cap-net-debug"),
            PracticeOptionRef::from(&target.practice_options[1]),
        );

        assert_eq!(
            compile_goal_plan(&graph, &request, 100),
            Err(GoalPlanError::PreferredPracticeNotEligible {
                capability_id: "cap-net-debug".into(),
                task_id: "task-network-practical".into()
            })
        );
    }

    #[test]
    fn unknown_target_is_rejected() {
        assert_eq!(
            compile_goal_plan(
                &basic_graph(),
                &request("missing", &[DeliveryMode::Simulation], 60, 100),
                100,
            ),
            Err(GoalPlanError::UnknownTarget {
                capability_id: "missing".into()
            })
        );
    }

    #[test]
    fn unknown_prerequisite_is_rejected() {
        let graph = graph(vec![node(
            "cap-target",
            "Target",
            &["cap-missing"],
            vec![option("task-target", "1", "rubric", "1", 10, DeliveryMode::Simulation)],
        )]);
        assert_eq!(
            compile_goal_plan(&graph, &request("cap-target", &[DeliveryMode::Simulation], 60, 100), 100),
            Err(GoalPlanError::UnknownPrerequisite {
                capability_id: "cap-target".into(),
                prerequisite_id: "cap-missing".into()
            })
        );
    }

    #[test]
    fn reachable_dependency_cycle_is_rejected() {
        let graph = graph(vec![
            node("a", "A", &["b"], vec![option("ta", "1", "ra", "1", 10, DeliveryMode::Simulation)]),
            node("b", "B", &["a"], vec![option("tb", "1", "rb", "1", 10, DeliveryMode::Simulation)]),
        ]);
        assert!(matches!(
            compile_goal_plan(&graph, &request("a", &[DeliveryMode::Simulation], 60, 100), 100),
            Err(GoalPlanError::DependencyCycle { .. })
        ));
    }

    #[test]
    fn unrelated_cycle_does_not_block_target_plan() {
        let graph = graph(vec![
            node("target", "Target", &[], vec![option("t-target", "1", "r-target", "1", 10, DeliveryMode::Simulation)]),
            node("unrelated-a", "Unrelated A", &["unrelated-b"], vec![option("t-a", "1", "r-a", "1", 10, DeliveryMode::Simulation)]),
            node("unrelated-b", "Unrelated B", &["unrelated-a"], vec![option("t-b", "1", "r-b", "1", 10, DeliveryMode::Simulation)]),
        ]);
        let plan = compile_goal_plan(&graph, &request("target", &[DeliveryMode::Simulation], 60, 100), 100).unwrap();
        assert_eq!(plan.steps.len(), 1);
        assert_eq!(plan.steps[0].capability_id, id("target"));
    }

    #[test]
    fn compiles_a_512_node_prerequisite_chain_without_losing_order() {
        let count = 512_usize;
        let mut nodes = Vec::with_capacity(count);
        for index in 0..count {
            let capability_id = format!("cap-{index:04}");
            let prerequisites = if index == 0 {
                Vec::new()
            } else {
                vec![id(&format!("cap-{:04}", index - 1))]
            };
            nodes.push(CapabilityNode {
                capability_id: id(&capability_id),
                version: "1".into(),
                title: format!("Capability {index}"),
                prerequisites,
                practice_options: vec![option(
                    &format!("task-{index:04}"),
                    "1",
                    &format!("rubric-{index:04}"),
                    "1",
                    1,
                    DeliveryMode::Simulation,
                )],
            });
        }
        let graph = graph(nodes);
        let plan = compile_goal_plan(
            &graph,
            &request(
                &format!("cap-{:04}", count - 1),
                &[DeliveryMode::Simulation],
                1,
                count as u32,
            ),
            100,
        )
        .unwrap();

        assert_eq!(plan.steps.len(), count);
        assert_eq!(plan.estimated_total_minutes, count as u32);
        assert_eq!(plan.steps.first().unwrap().capability_id, id("cap-0000"));
        assert_eq!(plan.steps.last().unwrap().capability_id, id("cap-0511"));
        assert_eq!(plan.steps.last().unwrap().ordinal, count as u32);
    }

    #[test]
    fn duplicate_capability_is_rejected() {
        let duplicate = node("a", "A", &[], vec![option("t1", "1", "r1", "1", 10, DeliveryMode::Simulation)]);
        assert!(matches!(
            compile_goal_plan(
                &graph(vec![duplicate.clone(), duplicate]),
                &request("a", &[DeliveryMode::Simulation], 60, 100),
                100,
            ),
            Err(GoalPlanError::DuplicateCapability { .. })
        ));
    }

    #[test]
    fn duplicate_prerequisite_is_rejected() {
        let graph = graph(vec![
            node("base", "Base", &[], vec![option("tb", "1", "rb", "1", 10, DeliveryMode::Simulation)]),
            node("target", "Target", &["base", "base"], vec![option("tt", "1", "rt", "1", 10, DeliveryMode::Simulation)]),
        ]);
        assert!(matches!(
            compile_goal_plan(&graph, &request("target", &[DeliveryMode::Simulation], 60, 100), 100),
            Err(GoalPlanError::DuplicatePrerequisite { .. })
        ));
    }

    #[test]
    fn task_limit_failure_does_not_emit_partial_plan() {
        assert_eq!(
            compile_goal_plan(
                &basic_graph(),
                &request("cap-net-debug", &[DeliveryMode::RetrievalPractice], 10, 100),
                100,
            ),
            Err(GoalPlanError::NoEligiblePracticeOption {
                capability_id: "cap-ip-basics".into()
            })
        );
    }

    #[test]
    fn total_budget_failure_does_not_truncate_plan() {
        assert_eq!(
            compile_goal_plan(
                &basic_graph(),
                &request(
                    "cap-net-debug",
                    &[DeliveryMode::GuidedPractice, DeliveryMode::RetrievalPractice, DeliveryMode::Simulation],
                    60,
                    59,
                ),
                100,
            ),
            Err(GoalPlanError::TotalBudgetExceeded {
                required_minutes: 60,
                budget_minutes: 59
            })
        );
    }

    #[test]
    fn empty_allowed_modes_are_rejected() {
        assert_eq!(
            compile_goal_plan(&basic_graph(), &request("cap-net-debug", &[], 60, 100), 100),
            Err(GoalPlanError::InvalidField {
                field: "allowed_delivery_modes"
            })
        );
    }

    #[test]
    fn zero_task_duration_is_rejected() {
        let graph = graph(vec![node(
            "target",
            "Target",
            &[],
            vec![option("task", "1", "rubric", "1", 0, DeliveryMode::Simulation)],
        )]);
        assert_eq!(
            compile_goal_plan(&graph, &request("target", &[DeliveryMode::Simulation], 60, 100), 100),
            Err(GoalPlanError::InvalidField { field: "estimated_minutes" })
        );
    }

    #[test]
    fn duplicate_practice_option_identity_is_rejected() {
        let item = option("same-task", "1", "same-rubric", "1", 10, DeliveryMode::Simulation);
        let graph = graph(vec![node("target", "Target", &[], vec![item.clone(), item])]);
        assert!(matches!(
            compile_goal_plan(&graph, &request("target", &[DeliveryMode::Simulation], 60, 100), 100),
            Err(GoalPlanError::DuplicatePracticeOption { .. })
        ));
    }

    #[test]
    fn negative_generation_time_is_rejected() {
        assert_eq!(
            compile_goal_plan(
                &basic_graph(),
                &request(
                    "cap-net-debug",
                    &[DeliveryMode::GuidedPractice, DeliveryMode::RetrievalPractice, DeliveryMode::Simulation],
                    60,
                    100,
                ),
                -1,
            ),
            Err(GoalPlanError::NegativeTimestamp { timestamp: -1 })
        );
    }

    #[test]
    fn duplicate_independent_nodes_have_stable_order() {
        let graph = graph(vec![
            node("zeta", "Zeta", &[], vec![option("tz", "1", "rz", "1", 10, DeliveryMode::Simulation)]),
            node("alpha", "Alpha", &[], vec![option("ta", "1", "ra", "1", 10, DeliveryMode::Simulation)]),
            node("target", "Target", &["zeta", "alpha"], vec![option("tt", "1", "rt", "1", 10, DeliveryMode::Simulation)]),
        ]);
        let plan = compile_goal_plan(&graph, &request("target", &[DeliveryMode::Simulation], 60, 100), 100).unwrap();
        let ids: Vec<&str> = plan.steps.iter().map(|step| step.capability_id.0.as_str()).collect();
        assert_eq!(ids, vec!["alpha", "zeta", "target"]);
    }
}
