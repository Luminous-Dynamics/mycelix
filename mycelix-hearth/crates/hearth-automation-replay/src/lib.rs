// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Deterministic, side-effect-free replay harness for Hearth automation plans.
//!
//! HTH-AUTO-004B turns the AUTO-003 execution kernel into a qualification lab.
//! A replay scenario scripts authority snapshots, predicate decisions, command
//! responses, verification responses, clock progression, and cancellation. No
//! physical adapter is admitted.
//!
//! The core property is reproducibility:
//!
//! > Same validated scenario bytes/values -> same receipt, trace, clock, and counters.
//!
//! Historical household timelines and richer effect models can be layered on this
//! crate later without weakening the executor or policy contracts.

use hearth_automation_policy::AuthoritySnapshot;
use hearth_automation_types::{
    AutomationPlan, CommandStatus, EvidenceRef, ExecutionStatus, HouseholdMode, IntentId,
    OutcomePolicy, PlanStep, VerificationStatus,
};
use hearth_edge::{
    AuthoritySource, EdgeClock, EdgeError, EdgeExecutionResult, EdgeRuntime, ExecutionControl,
    PredicateDecision, PredicateSource, SyntheticAdapter, VerificationReport,
};
use serde::{Deserialize, Serialize};
use std::{
    cell::{Cell, RefCell},
    collections::VecDeque,
};

pub const REPLAY_SCHEMA_VERSION: u16 = 1;

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum ScriptedPredicate {
    True,
    False { reason: String },
    Unknown { reason: String },
}

impl ScriptedPredicate {
    fn into_runtime(self) -> PredicateDecision {
        match self {
            Self::True => PredicateDecision::True,
            Self::False { reason } => PredicateDecision::False { reason },
            Self::Unknown { reason } => PredicateDecision::Unknown { reason },
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ScriptedVerification {
    pub status: VerificationStatus,
    pub evidence: Vec<EvidenceRef>,
}

impl ScriptedVerification {
    pub fn verified(evidence: Vec<EvidenceRef>) -> Self {
        Self {
            status: VerificationStatus::Verified,
            evidence,
        }
    }

    pub fn failed(reason: impl Into<String>) -> Self {
        Self {
            status: VerificationStatus::Failed {
                reason: reason.into(),
            },
            evidence: Vec::new(),
        }
    }

    fn into_runtime(self) -> VerificationReport {
        VerificationReport {
            status: self.status,
            evidence: self.evidence,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ReplayScenario {
    pub schema_version: u16,
    pub id: String,
    pub plan: AutomationPlan,
    pub trigger_evidence: Vec<EvidenceRef>,
    pub start_micros: i64,
    pub adapter_name: String,
    pub adapter_capabilities: Vec<String>,
    /// Responses consumed by the synthetic adapter in invocation order.
    pub command_script: Vec<CommandStatus>,
    /// Verification responses consumed in invocation order.
    pub verification_script: Vec<ScriptedVerification>,
    /// Authority snapshots consumed in request order. Once exhausted, the last
    /// snapshot remains active. At least one snapshot is required.
    pub authority_script: Vec<AuthoritySnapshot>,
    /// Predicate results consumed in evaluation order. Exhaustion becomes
    /// `Unknown`, so incomplete simulations fail closed.
    pub predicate_script: Vec<ScriptedPredicate>,
    /// 1-based `ExecutionControl::cancellation_reason` poll on which cancellation
    /// begins. `None` means never cancel.
    pub cancel_on_poll: Option<u64>,
}

impl ReplayScenario {
    pub fn validate(&self) -> Result<(), ReplayError> {
        if self.schema_version != REPLAY_SCHEMA_VERSION {
            return Err(ReplayError::InvalidScenario(format!(
                "unsupported replay schema version {}; expected {}",
                self.schema_version, REPLAY_SCHEMA_VERSION
            )));
        }
        if self.id.trim().is_empty() {
            return Err(ReplayError::InvalidScenario(
                "scenario id must not be empty".into(),
            ));
        }
        if self.adapter_name.trim().is_empty() {
            return Err(ReplayError::InvalidScenario(
                "adapter_name must not be empty".into(),
            ));
        }
        if self.adapter_capabilities.iter().any(|value| value.trim().is_empty()) {
            return Err(ReplayError::InvalidScenario(
                "adapter capabilities must not contain empty strings".into(),
            ));
        }
        if self.authority_script.is_empty() {
            return Err(ReplayError::InvalidScenario(
                "at least one authority snapshot is required".into(),
            ));
        }
        if self.cancel_on_poll == Some(0) {
            return Err(ReplayError::InvalidScenario(
                "cancel_on_poll is 1-based and must be positive".into(),
            ));
        }
        self.plan
            .validate()
            .map_err(|error| ReplayError::InvalidScenario(error.to_string()))?;
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ReplayError {
    InvalidScenario(String),
    Edge(String),
    Diverged {
        first: Box<ReplayRun>,
        second: Box<ReplayRun>,
    },
}

impl std::fmt::Display for ReplayError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InvalidScenario(message) => write!(f, "invalid replay scenario: {message}"),
            Self::Edge(message) => write!(f, "edge replay failed: {message}"),
            Self::Diverged { .. } => write!(f, "deterministic replay diverged"),
        }
    }
}

impl std::error::Error for ReplayError {}

impl From<EdgeError> for ReplayError {
    fn from(value: EdgeError) -> Self {
        Self::Edge(format!("{value:?}"))
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ReplayRun {
    pub scenario_id: String,
    pub edge: EdgeExecutionResult,
    pub finished_clock_micros: i64,
    pub sleeps_ms: Vec<u64>,
    pub authority_requests: u64,
    pub predicate_requests: u64,
    pub cancellation_polls: u64,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DeterminismReport {
    pub scenario_id: String,
    pub run: ReplayRun,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ScenarioSummary {
    pub scenario_id: String,
    pub execution: ExecutionStatus,
    pub final_verification: VerificationStatus,
    pub step_receipts: usize,
}

#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct MatrixReport {
    pub scenarios: Vec<ScenarioSummary>,
    pub completed: usize,
    pub failed: usize,
    pub cancelled: usize,
    pub inhibited: usize,
}

impl MatrixReport {
    fn push(&mut self, run: &ReplayRun) {
        let execution = run.edge.receipt.execution.clone();
        match &execution {
            ExecutionStatus::Completed => self.completed += 1,
            ExecutionStatus::Failed { .. } => self.failed += 1,
            ExecutionStatus::Cancelled { .. } => self.cancelled += 1,
            ExecutionStatus::Inhibited(_) => self.inhibited += 1,
            ExecutionStatus::Planned | ExecutionStatus::Running => self.failed += 1,
        }
        self.scenarios.push(ScenarioSummary {
            scenario_id: run.scenario_id.clone(),
            execution,
            final_verification: run.edge.receipt.final_verification.clone(),
            step_receipts: run.edge.receipt.steps.len(),
        });
    }
}

#[derive(Debug, Clone)]
struct ReplayClock {
    now_micros: i64,
    sleeps_ms: Vec<u64>,
}

impl ReplayClock {
    fn new(start_micros: i64) -> Self {
        Self {
            now_micros: start_micros,
            sleeps_ms: Vec::new(),
        }
    }
}

impl EdgeClock for ReplayClock {
    fn now_micros(&self) -> i64 {
        self.now_micros
    }

    fn sleep_ms(&mut self, millis: u64) {
        self.sleeps_ms.push(millis);
        let delta = i64::try_from(millis)
            .unwrap_or(i64::MAX)
            .saturating_mul(1_000);
        self.now_micros = self.now_micros.saturating_add(delta);
    }
}

#[derive(Debug, Clone)]
struct ReplayAuthority {
    queue: VecDeque<AuthoritySnapshot>,
    last: AuthoritySnapshot,
    requests: u64,
}

impl ReplayAuthority {
    fn new(script: Vec<AuthoritySnapshot>) -> Self {
        let last = script
            .first()
            .expect("validated replay scenario has authority")
            .clone();
        Self {
            queue: script.into(),
            last,
            requests: 0,
        }
    }
}

impl AuthoritySource for ReplayAuthority {
    fn snapshot(
        &mut self,
        _intent_id: &IntentId,
        _step: &PlanStep,
        _now_micros: i64,
    ) -> AuthoritySnapshot {
        self.requests = self.requests.saturating_add(1);
        if let Some(next) = self.queue.pop_front() {
            self.last = next;
        }
        self.last.clone()
    }
}

#[derive(Debug)]
struct ReplayPredicates {
    queue: RefCell<VecDeque<ScriptedPredicate>>,
    requests: Cell<u64>,
}

impl ReplayPredicates {
    fn new(script: Vec<ScriptedPredicate>) -> Self {
        Self {
            queue: RefCell::new(script.into()),
            requests: Cell::new(0),
        }
    }
}

impl PredicateSource for ReplayPredicates {
    fn evaluate(
        &self,
        condition: &hearth_automation_types::Condition,
        _now_micros: i64,
    ) -> PredicateDecision {
        self.requests.set(self.requests.get().saturating_add(1));
        self.queue
            .borrow_mut()
            .pop_front()
            .map(ScriptedPredicate::into_runtime)
            .unwrap_or_else(|| PredicateDecision::Unknown {
                reason: format!("replay predicate script exhausted at {condition:?}"),
            })
    }
}

#[derive(Debug, Clone)]
struct ReplayControl {
    cancel_on_poll: Option<u64>,
    polls: u64,
}

impl ReplayControl {
    fn new(cancel_on_poll: Option<u64>) -> Self {
        Self {
            cancel_on_poll,
            polls: 0,
        }
    }
}

impl ExecutionControl for ReplayControl {
    fn cancellation_reason(&mut self) -> Option<String> {
        self.polls = self.polls.saturating_add(1);
        if self
            .cancel_on_poll
            .is_some_and(|cancel_on| self.polls >= cancel_on)
        {
            Some(format!("scripted cancellation at poll {}", self.polls))
        } else {
            None
        }
    }
}

pub fn run_once(scenario: &ReplayScenario) -> Result<ReplayRun, ReplayError> {
    scenario.validate()?;

    let verification_script = scenario
        .verification_script
        .clone()
        .into_iter()
        .map(ScriptedVerification::into_runtime)
        .collect();

    let adapter = SyntheticAdapter::new(
        scenario.adapter_name.clone(),
        scenario.adapter_capabilities.clone(),
    )
    .with_commands(scenario.command_script.clone())
    .with_verifications(verification_script);

    let mut runtime = EdgeRuntime::new();
    runtime.register_adapter(adapter);

    let mut clock = ReplayClock::new(scenario.start_micros);
    let mut authority = ReplayAuthority::new(scenario.authority_script.clone());
    let predicates = ReplayPredicates::new(scenario.predicate_script.clone());
    let mut control = ReplayControl::new(scenario.cancel_on_poll);

    let edge = runtime.execute_plan(
        &scenario.plan,
        scenario.trigger_evidence.clone(),
        &mut clock,
        &mut authority,
        &predicates,
        &mut control,
    )?;

    Ok(ReplayRun {
        scenario_id: scenario.id.clone(),
        edge,
        finished_clock_micros: clock.now_micros,
        sleeps_ms: clock.sleeps_ms,
        authority_requests: authority.requests,
        predicate_requests: predicates.requests.get(),
        cancellation_polls: control.polls,
    })
}

/// Execute a scenario twice from fresh state and require exact semantic equality
/// of all derived Rust values in `ReplayRun`.
pub fn assert_deterministic(
    scenario: &ReplayScenario,
) -> Result<DeterminismReport, ReplayError> {
    let first = run_once(scenario)?;
    let second = run_once(scenario)?;
    if first != second {
        return Err(ReplayError::Diverged {
            first: Box::new(first),
            second: Box::new(second),
        });
    }
    Ok(DeterminismReport {
        scenario_id: scenario.id.clone(),
        run: first,
    })
}

pub fn run_matrix(
    scenarios: &[ReplayScenario],
    require_determinism: bool,
) -> Result<MatrixReport, ReplayError> {
    let mut report = MatrixReport::default();
    for scenario in scenarios {
        let run = if require_determinism {
            assert_deterministic(scenario)?.run
        } else {
            run_once(scenario)?
        };
        report.push(&run);
    }
    Ok(report)
}

/// Structural replay audit layered on top of the receipt's own validation.
///
/// These checks are intentionally redundant with parts of the production
/// contracts: replay qualification should detect a future regression even if a
/// lower layer accidentally becomes permissive.
pub fn audit_run(
    scenario: &ReplayScenario,
    run: &ReplayRun,
) -> Result<(), ReplayError> {
    run.edge
        .receipt
        .validate()
        .map_err(|error| ReplayError::Edge(format!("receipt audit failed: {error}")))?;

    if run.edge.receipt.plan_id != scenario.plan.id {
        return Err(ReplayError::Edge(
            "receipt plan_id differs from replay scenario".into(),
        ));
    }
    if run.edge.receipt.intent_id != scenario.plan.intent_id {
        return Err(ReplayError::Edge(
            "receipt intent_id differs from replay scenario".into(),
        ));
    }

    if matches!(run.edge.receipt.execution, ExecutionStatus::Completed)
        && run.edge.receipt.steps.len() != scenario.plan.steps.len()
    {
        return Err(ReplayError::Edge(
            "completed replay did not produce one receipt per plan step".into(),
        ));
    }

    if matches!(run.edge.receipt.execution, ExecutionStatus::Completed)
        && !run.edge.receipt.final_verification.is_verified()
    {
        return Err(ReplayError::Edge(
            "completed replay lacks verified final outcome".into(),
        ));
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use hearth_automation_types::{
        ActionSpec, AuthorityRequirement, AutomationValue, ComparisonOp, Condition,
        ConditionExpr, ConsequenceClass, EntityRef, EvidenceRequirement, OutcomeExpectation,
        PlanId, Reversibility, RetryPolicy, StateExpectation, StepId, UnverifiedDisposition,
        AUTOMATION_SCHEMA_VERSION,
    };
    use std::collections::BTreeMap;

    fn entity(id: &str) -> EntityRef {
        EntityRef {
            kind: "appliance".into(),
            id: id.into(),
        }
    }

    fn step(id: &str, depends_on: Vec<StepId>) -> PlanStep {
        PlanStep {
            id: StepId::from(id),
            depends_on,
            preconditions: ConditionExpr::Always,
            action: ActionSpec {
                capability: "home.appliance.control".into(),
                target: entity(id),
                operation: "stop".into(),
                arguments: BTreeMap::new(),
            },
            consequence: ConsequenceClass::ReversibleAct,
            reversibility: Reversibility::Reversible,
            authority: AuthorityRequirement::Capability {
                capability: "home.appliance.control".into(),
            },
            outcome: OutcomePolicy {
                expectations: vec![OutcomeExpectation::State(StateExpectation {
                    subject: entity(id),
                    attribute: "running".into(),
                    op: ComparisonOp::Eq,
                    expected: AutomationValue::Bool(false),
                })],
                verify_within_ms: 5_000,
                evidence: EvidenceRequirement::default(),
                on_unverified: UnverifiedDisposition::StopAndNotify,
            },
            timeout_ms: 1_000,
            retry: RetryPolicy::none(),
            compensation: None,
        }
    }

    fn plan() -> AutomationPlan {
        AutomationPlan {
            schema_version: AUTOMATION_SCHEMA_VERSION,
            id: PlanId::from("plan-1"),
            intent_id: IntentId::from("intent-1"),
            generated_at_micros: 1,
            steps: vec![
                step("washer", Vec::new()),
                step("dryer", vec![StepId::from("washer")]),
            ],
        }
    }

    fn authority(mode: HouseholdMode, capabilities: Vec<String>, at: i64) -> AuthoritySnapshot {
        AuthoritySnapshot::from_legacy_profile(
            "agent-1",
            mode,
            "profile-1",
            capabilities,
            Vec::<String>::new(),
            at,
        )
    }

    fn scenario() -> ReplayScenario {
        ReplayScenario {
            schema_version: REPLAY_SCHEMA_VERSION,
            id: "baseline".into(),
            plan: plan(),
            trigger_evidence: Vec::new(),
            start_micros: 100,
            adapter_name: "synthetic".into(),
            adapter_capabilities: vec!["home.appliance.control".into()],
            command_script: vec![CommandStatus::Accepted, CommandStatus::Accepted],
            verification_script: vec![
                ScriptedVerification::verified(Vec::new()),
                ScriptedVerification::verified(Vec::new()),
            ],
            authority_script: vec![authority(
                HouseholdMode::Normal,
                vec!["home.appliance.control".into()],
                100,
            )],
            predicate_script: Vec::new(),
            cancel_on_poll: None,
        }
    }

    #[test]
    fn identical_scenario_replays_identically() {
        let report = assert_deterministic(&scenario()).unwrap();
        assert!(matches!(
            report.run.edge.receipt.execution,
            ExecutionStatus::Completed
        ));
        audit_run(&scenario(), &report.run).unwrap();
    }

    #[test]
    fn authority_revocation_between_steps_stops_plan() {
        let mut scenario = scenario();
        scenario.authority_script = vec![
            authority(
                HouseholdMode::Normal,
                vec!["home.appliance.control".into()],
                100,
            ),
            authority(
                HouseholdMode::Normal,
                vec!["home.appliance.control".into()],
                100,
            ),
            authority(HouseholdMode::Normal, Vec::new(), 100),
        ];

        let run = run_once(&scenario).unwrap();
        assert!(!matches!(run.edge.receipt.execution, ExecutionStatus::Completed));
    }

    #[test]
    fn predicate_script_exhaustion_fails_closed() {
        let mut scenario = scenario();
        scenario.plan.steps[0].preconditions = ConditionExpr::Predicate(Condition::State {
            subject: entity("washer"),
            attribute: "running".into(),
            op: ComparisonOp::Eq,
            expected: AutomationValue::Bool(true),
            evidence: EvidenceRequirement::default(),
        });
        scenario.predicate_script.clear();

        let run = run_once(&scenario).unwrap();
        assert!(matches!(
            run.edge.receipt.execution,
            ExecutionStatus::Inhibited(_)
        ));
        assert_eq!(run.predicate_requests, 1);
    }

    #[test]
    fn scripted_cancellation_is_deterministic() {
        let mut scenario = scenario();
        scenario.cancel_on_poll = Some(1);

        let report = assert_deterministic(&scenario).unwrap();
        assert!(matches!(
            report.run.edge.receipt.execution,
            ExecutionStatus::Cancelled { .. }
        ));
    }

    #[test]
    fn matrix_counts_outcomes() {
        let completed = scenario();
        let mut cancelled = scenario();
        cancelled.id = "cancelled".into();
        cancelled.cancel_on_poll = Some(1);

        let report = run_matrix(&[completed, cancelled], true).unwrap();
        assert_eq!(report.completed, 1);
        assert_eq!(report.cancelled, 1);
        assert_eq!(report.scenarios.len(), 2);
    }
}
