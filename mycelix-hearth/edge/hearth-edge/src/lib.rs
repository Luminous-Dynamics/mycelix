// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Deterministic local execution kernel for Hearth household automation.
//!
//! HTH-AUTO-003 deliberately controls no real devices. It executes validated
//! semantic plans against adapter, authority, predicate, clock, and cancellation
//! traits so execution semantics can be qualified before physical integrations.

use hearth_automation_policy::{evaluate_step, AuthoritySnapshot, PolicyDecision};
use hearth_automation_types::{
    ActionSpec, AuthorityEvidence, AutomationPlan, AutomationReceipt, CommandStatus, Condition,
    ConditionExpr, EvidenceRef, ExecutionStatus, HouseholdMode, InhibitionReason, IntentId,
    OutcomeExpectation, OutcomePolicy, PlanStep, ReceiptId, StepReceipt, UnverifiedDisposition,
    VerificationStatus, AUTOMATION_SCHEMA_VERSION,
};
use std::collections::{BTreeMap, BTreeSet, VecDeque};

pub trait EdgeClock {
    fn now_micros(&self) -> i64;
    fn sleep_ms(&mut self, millis: u64);
}

/// Authority is refreshed before every step and before every retry.
pub trait AuthoritySource {
    fn snapshot(
        &mut self,
        intent_id: &IntentId,
        step: &PlanStep,
        now_micros: i64,
    ) -> AuthoritySnapshot;
}

pub trait ExecutionControl {
    fn cancellation_reason(&mut self) -> Option<String>;
}

#[derive(Debug, Default)]
pub struct NeverCancel;

impl ExecutionControl for NeverCancel {
    fn cancellation_reason(&mut self) -> Option<String> {
        None
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum PredicateDecision {
    True,
    False { reason: String },
    Unknown { reason: String },
}

pub trait PredicateSource {
    fn evaluate(&self, condition: &Condition, now_micros: i64) -> PredicateDecision;
}

/// Safe default for an edge with no domain/state world model yet.
#[derive(Debug, Default)]
pub struct UnknownPredicates;

impl PredicateSource for UnknownPredicates {
    fn evaluate(&self, condition: &Condition, _now_micros: i64) -> PredicateDecision {
        PredicateDecision::Unknown {
            reason: format!("no predicate source for {condition:?}"),
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct VerificationReport {
    pub status: VerificationStatus,
    pub evidence: Vec<EvidenceRef>,
}

impl VerificationReport {
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
}

/// Local adapter boundary. Future implementations may wrap Matter, Home
/// Assistant, OCPP, OpenADR, OS services, or virtual household actions.
///
/// `idempotency_key` is stable for a logical plan step across retries. Adapters
/// MUST treat a repeated key as replay rather than duplicate side effect.
pub trait EdgeAdapter {
    fn name(&self) -> &str;
    fn supports_capability(&self, capability: &str) -> bool;
    fn supports(&self, action: &ActionSpec) -> bool;

    fn execute(
        &mut self,
        idempotency_key: &str,
        action: &ActionSpec,
        timeout_ms: u64,
        now_micros: i64,
    ) -> CommandStatus;

    fn verify(
        &mut self,
        idempotency_key: &str,
        expectations: &[OutcomeExpectation],
        policy: &OutcomePolicy,
        now_micros: i64,
    ) -> VerificationReport;
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum TracePhase {
    Authority,
    Preconditions,
    Command,
    Verification,
    Compensation,
    Cancellation,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TraceEvent {
    pub step_id: String,
    pub phase: TracePhase,
    pub attempt: u16,
    pub adapter: Option<String>,
    pub idempotency_key: Option<String>,
    pub at_micros: i64,
    pub detail: String,
}

#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct ExecutionTrace {
    pub events: Vec<TraceEvent>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct EdgeExecutionResult {
    pub receipt: AutomationReceipt,
    pub trace: ExecutionTrace,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum EdgeError {
    InvalidPlan(String),
    SchedulerDeadlock,
    ReceiptInvalid(String),
}

pub struct EdgeRuntime {
    adapters: Vec<Box<dyn EdgeAdapter>>,
}

impl Default for EdgeRuntime {
    fn default() -> Self {
        Self::new()
    }
}

impl EdgeRuntime {
    pub fn new() -> Self {
        Self { adapters: Vec::new() }
    }

    pub fn register_adapter(&mut self, adapter: impl EdgeAdapter + 'static) {
        self.adapters.push(Box::new(adapter));
    }

    pub fn execute_plan<C, A, P, X>(
        &mut self,
        plan: &AutomationPlan,
        trigger_evidence: Vec<EvidenceRef>,
        clock: &mut C,
        authority: &mut A,
        predicates: &P,
        control: &mut X,
    ) -> Result<EdgeExecutionResult, EdgeError>
    where
        C: EdgeClock,
        A: AuthoritySource,
        P: PredicateSource,
        X: ExecutionControl,
    {
        plan.validate()
            .map_err(|error| EdgeError::InvalidPlan(error.to_string()))?;

        let started_at = clock.now_micros();
        let mut receipt = AutomationReceipt {
            schema_version: AUTOMATION_SCHEMA_VERSION,
            id: ReceiptId::new(format!("edge:{}:{started_at}", plan.id.0)),
            intent_id: plan.intent_id.clone(),
            plan_id: plan.id.clone(),
            started_at_micros: started_at,
            finished_at_micros: None,
            execution: ExecutionStatus::Running,
            trigger_evidence,
            authority_evidence: empty_authority_evidence(started_at),
            steps: Vec::new(),
            final_verification: VerificationStatus::Pending,
        };
        let mut trace = ExecutionTrace::default();
        let mut completed = BTreeSet::new();
        let mut pending: BTreeMap<_, _> = plan
            .steps
            .iter()
            .map(|step| (step.id.clone(), step))
            .collect();

        while !pending.is_empty() {
            let ready: Vec<_> = pending
                .iter()
                .filter_map(|(id, step)| {
                    step.depends_on
                        .iter()
                        .all(|dependency| completed.contains(dependency))
                        .then_some(id.clone())
                })
                .collect();

            if ready.is_empty() {
                return Err(EdgeError::SchedulerDeadlock);
            }

            for step_id in ready {
                if let Some(reason) = control.cancellation_reason() {
                    trace.events.push(TraceEvent {
                        step_id: step_id.0.clone(),
                        phase: TracePhase::Cancellation,
                        attempt: 0,
                        adapter: None,
                        idempotency_key: None,
                        at_micros: clock.now_micros(),
                        detail: reason.clone(),
                    });
                    finish_receipt(
                        &mut receipt,
                        clock.now_micros(),
                        ExecutionStatus::Cancelled { reason },
                        VerificationStatus::Unknown {
                            reason: "execution cancelled before all steps completed".into(),
                        },
                    );
                    validate_receipt(&receipt)?;
                    return Ok(EdgeExecutionResult { receipt, trace });
                }

                let step = pending
                    .remove(&step_id)
                    .expect("ready step must still be pending");
                match self.execute_step(
                    &plan.intent_id,
                    &plan.id.0,
                    step,
                    clock,
                    authority,
                    predicates,
                    control,
                    &mut receipt.authority_evidence,
                    &mut trace,
                ) {
                    StepRun::Completed(step_receipt) => {
                        receipt.steps.push(step_receipt);
                        completed.insert(step_id);
                    }
                    StepRun::Stopped {
                        receipt: step_receipt,
                        execution,
                        final_verification,
                    } => {
                        receipt.steps.push(step_receipt);
                        finish_receipt(
                            &mut receipt,
                            clock.now_micros(),
                            execution,
                            final_verification,
                        );
                        validate_receipt(&receipt)?;
                        return Ok(EdgeExecutionResult { receipt, trace });
                    }
                }
            }
        }

        finish_receipt(
            &mut receipt,
            clock.now_micros(),
            ExecutionStatus::Completed,
            VerificationStatus::Verified,
        );
        validate_receipt(&receipt)?;
        Ok(EdgeExecutionResult { receipt, trace })
    }

    #[allow(clippy::too_many_arguments)]
    fn execute_step<C, A, P, X>(
        &mut self,
        intent_id: &IntentId,
        plan_id: &str,
        step: &PlanStep,
        clock: &mut C,
        authority: &mut A,
        predicates: &P,
        control: &mut X,
        authority_evidence: &mut AuthorityEvidence,
        trace: &mut ExecutionTrace,
    ) -> StepRun
    where
        C: EdgeClock,
        A: AuthoritySource,
        P: PredicateSource,
        X: ExecutionControl,
    {
        let started_at = clock.now_micros();
        let snapshot = authority.snapshot(intent_id, step, started_at);
        trace.events.push(TraceEvent {
            step_id: step.id.0.clone(),
            phase: TracePhase::Authority,
            attempt: 0,
            adapter: None,
            idempotency_key: None,
            at_micros: started_at,
            detail: "authority refreshed before step".into(),
        });

        match evaluate_step(intent_id, step, &snapshot, started_at) {
            PolicyDecision::Allowed { evidence } => merge_authority_evidence(authority_evidence, evidence),
            PolicyDecision::ApprovalRequired { .. } => {
                return stopped(
                    step,
                    started_at,
                    clock.now_micros(),
                    CommandStatus::NotAttempted,
                    VerificationStatus::NotAttempted,
                    Vec::new(),
                    ExecutionStatus::Inhibited(InhibitionReason::MissingAuthority),
                );
            }
            PolicyDecision::Denied(reason) => {
                return stopped(
                    step,
                    started_at,
                    clock.now_micros(),
                    CommandStatus::NotAttempted,
                    VerificationStatus::NotAttempted,
                    Vec::new(),
                    ExecutionStatus::Failed {
                        reason: format!("authority denied: {reason:?}"),
                    },
                );
            }
            PolicyDecision::Inhibited(reason) => {
                return stopped(
                    step,
                    started_at,
                    clock.now_micros(),
                    CommandStatus::NotAttempted,
                    VerificationStatus::NotAttempted,
                    Vec::new(),
                    ExecutionStatus::Inhibited(reason),
                );
            }
        }

        match self.evaluate_conditions(&step.preconditions, &snapshot, predicates, clock.now_micros()) {
            PredicateDecision::True => trace.events.push(TraceEvent {
                step_id: step.id.0.clone(),
                phase: TracePhase::Preconditions,
                attempt: 0,
                adapter: None,
                idempotency_key: None,
                at_micros: clock.now_micros(),
                detail: "preconditions satisfied".into(),
            }),
            PredicateDecision::False { reason } | PredicateDecision::Unknown { reason } => {
                trace.events.push(TraceEvent {
                    step_id: step.id.0.clone(),
                    phase: TracePhase::Preconditions,
                    attempt: 0,
                    adapter: None,
                    idempotency_key: None,
                    at_micros: clock.now_micros(),
                    detail: reason.clone(),
                });
                return stopped(
                    step,
                    started_at,
                    clock.now_micros(),
                    CommandStatus::NotAttempted,
                    VerificationStatus::NotAttempted,
                    Vec::new(),
                    ExecutionStatus::Inhibited(InhibitionReason::Policy(reason)),
                );
            }
        }

        let Some(adapter_index) = self.adapters.iter().position(|adapter| adapter.supports(&step.action)) else {
            return stopped(
                step,
                started_at,
                clock.now_micros(),
                CommandStatus::NotAttempted,
                VerificationStatus::NotAttempted,
                Vec::new(),
                ExecutionStatus::Inhibited(InhibitionReason::AdapterUnavailable),
            );
        };

        let key = format!("plan:{plan_id}:step:{}", step.id.0);
        let mut attempt = 1_u16;
        let mut last_command = CommandStatus::NotAttempted;
        let mut last_verification = VerificationStatus::NotAttempted;
        let mut observations = Vec::new();

        loop {
            if let Some(reason) = control.cancellation_reason() {
                trace.events.push(TraceEvent {
                    step_id: step.id.0.clone(),
                    phase: TracePhase::Cancellation,
                    attempt,
                    adapter: None,
                    idempotency_key: Some(key.clone()),
                    at_micros: clock.now_micros(),
                    detail: reason.clone(),
                });
                return stopped(
                    step,
                    started_at,
                    clock.now_micros(),
                    last_command,
                    last_verification,
                    observations,
                    ExecutionStatus::Cancelled { reason },
                );
            }

            let now = clock.now_micros();
            let refreshed = authority.snapshot(intent_id, step, now);
            match evaluate_step(intent_id, step, &refreshed, now) {
                PolicyDecision::Allowed { evidence } => merge_authority_evidence(authority_evidence, evidence),
                PolicyDecision::ApprovalRequired { .. } => {
                    return stopped(
                        step,
                        started_at,
                        now,
                        last_command,
                        last_verification,
                        observations,
                        ExecutionStatus::Inhibited(InhibitionReason::MissingAuthority),
                    );
                }
                PolicyDecision::Inhibited(reason) => {
                    return stopped(step, started_at, now, last_command, last_verification, observations, ExecutionStatus::Inhibited(reason));
                }
                PolicyDecision::Denied(reason) => {
                    return stopped(
                        step,
                        started_at,
                        now,
                        last_command,
                        last_verification,
                        observations,
                        ExecutionStatus::Failed { reason: format!("authority revoked/denied: {reason:?}") },
                    );
                }
            }

            let adapter_name = self.adapters[adapter_index].name().to_owned();
            trace.events.push(TraceEvent {
                step_id: step.id.0.clone(),
                phase: TracePhase::Command,
                attempt,
                adapter: Some(adapter_name.clone()),
                idempotency_key: Some(key.clone()),
                at_micros: now,
                detail: "dispatch semantic action".into(),
            });
            last_command = self.adapters[adapter_index].execute(
                &key,
                &step.action,
                step.timeout_ms,
                now,
            );

            if matches!(last_command, CommandStatus::Accepted) {
                let report = self.adapters[adapter_index].verify(
                    &key,
                    &step.outcome.expectations,
                    &step.outcome,
                    clock.now_micros(),
                );
                last_verification = report.status.clone();
                observations.extend(report.evidence);
                trace.events.push(TraceEvent {
                    step_id: step.id.0.clone(),
                    phase: TracePhase::Verification,
                    attempt,
                    adapter: Some(adapter_name),
                    idempotency_key: Some(key.clone()),
                    at_micros: clock.now_micros(),
                    detail: format!("{:?}", last_verification),
                });

                if last_verification.is_verified() {
                    return StepRun::Completed(StepReceipt {
                        step_id: step.id.clone(),
                        started_at_micros: started_at,
                        finished_at_micros: Some(clock.now_micros()),
                        command: last_command,
                        verification: last_verification,
                        observations,
                    });
                }

                if step.outcome.on_unverified == UnverifiedDisposition::RetryThenNotify
                    && attempt < step.retry.max_attempts
                {
                    clock.sleep_ms(backoff_ms(&step.retry, attempt));
                    attempt += 1;
                    continue;
                }

                if step.outcome.on_unverified == UnverifiedDisposition::CompensateThenNotify {
                    self.compensate(step, adapter_index, &key, clock, trace);
                }

                let execution = match step.outcome.on_unverified {
                    UnverifiedDisposition::EscalateForApproval => {
                        ExecutionStatus::Inhibited(InhibitionReason::MissingAuthority)
                    }
                    _ => ExecutionStatus::Failed {
                        reason: "command accepted but required outcome was not verified".into(),
                    },
                };
                return stopped(
                    step,
                    started_at,
                    clock.now_micros(),
                    last_command,
                    last_verification,
                    observations,
                    execution,
                );
            }

            if is_retryable_command(&last_command) && attempt < step.retry.max_attempts {
                clock.sleep_ms(backoff_ms(&step.retry, attempt));
                attempt += 1;
                continue;
            }

            return stopped(
                step,
                started_at,
                clock.now_micros(),
                last_command.clone(),
                VerificationStatus::NotAttempted,
                observations,
                ExecutionStatus::Failed {
                    reason: format!("command not accepted: {last_command:?}"),
                },
            );
        }
    }

    fn compensate<C: EdgeClock>(
        &mut self,
        step: &PlanStep,
        adapter_index: usize,
        key: &str,
        clock: &mut C,
        trace: &mut ExecutionTrace,
    ) {
        let Some(action) = step.compensation.as_ref() else {
            return;
        };
        let compensation_key = format!("{key}:compensation");
        let adapter = &mut self.adapters[adapter_index];
        let status = adapter.execute(
            &compensation_key,
            action,
            step.timeout_ms,
            clock.now_micros(),
        );
        trace.events.push(TraceEvent {
            step_id: step.id.0.clone(),
            phase: TracePhase::Compensation,
            attempt: 1,
            adapter: Some(adapter.name().to_owned()),
            idempotency_key: Some(compensation_key),
            at_micros: clock.now_micros(),
            detail: format!("{status:?}"),
        });
    }

    fn evaluate_conditions<P: PredicateSource>(
        &self,
        expression: &ConditionExpr,
        snapshot: &AuthoritySnapshot,
        predicates: &P,
        now_micros: i64,
    ) -> PredicateDecision {
        match expression {
            ConditionExpr::Always => PredicateDecision::True,
            ConditionExpr::Predicate(condition) => match condition {
                Condition::ModeIn(modes) => {
                    if modes.contains(&snapshot.household_mode) {
                        PredicateDecision::True
                    } else {
                        PredicateDecision::False {
                            reason: format!("household mode {:?} is not allowed", snapshot.household_mode),
                        }
                    }
                }
                Condition::CapabilityAvailable { capability } => {
                    if self.adapters.iter().any(|adapter| adapter.supports_capability(capability)) {
                        PredicateDecision::True
                    } else {
                        PredicateDecision::Unknown {
                            reason: format!("no local adapter exposes capability {capability}"),
                        }
                    }
                }
                other => predicates.evaluate(other, now_micros),
            },
            ConditionExpr::All(children) => {
                for child in children {
                    match self.evaluate_conditions(child, snapshot, predicates, now_micros) {
                        PredicateDecision::True => {}
                        other => return other,
                    }
                }
                PredicateDecision::True
            }
            ConditionExpr::Any(children) => {
                let mut unknown = None;
                for child in children {
                    match self.evaluate_conditions(child, snapshot, predicates, now_micros) {
                        PredicateDecision::True => return PredicateDecision::True,
                        PredicateDecision::Unknown { reason } => unknown = Some(reason),
                        PredicateDecision::False { .. } => {}
                    }
                }
                unknown.map_or_else(
                    || PredicateDecision::False { reason: "no alternative precondition matched".into() },
                    |reason| PredicateDecision::Unknown { reason },
                )
            }
            ConditionExpr::Not(child) => match self.evaluate_conditions(child, snapshot, predicates, now_micros) {
                PredicateDecision::True => PredicateDecision::False { reason: "negated precondition matched".into() },
                PredicateDecision::False { .. } => PredicateDecision::True,
                PredicateDecision::Unknown { reason } => PredicateDecision::Unknown { reason },
            },
        }
    }
}

enum StepRun {
    Completed(StepReceipt),
    Stopped {
        receipt: StepReceipt,
        execution: ExecutionStatus,
        final_verification: VerificationStatus,
    },
}

fn stopped(
    step: &PlanStep,
    started_at: i64,
    finished_at: i64,
    command: CommandStatus,
    verification: VerificationStatus,
    observations: Vec<EvidenceRef>,
    execution: ExecutionStatus,
) -> StepRun {
    StepRun::Stopped {
        receipt: StepReceipt {
            step_id: step.id.clone(),
            started_at_micros: started_at,
            finished_at_micros: Some(finished_at),
            command,
            verification: verification.clone(),
            observations,
        },
        execution,
        final_verification: verification,
    }
}

fn empty_authority_evidence(now_micros: i64) -> AuthorityEvidence {
    AuthorityEvidence {
        evaluated_at_micros: now_micros,
        capability_grants: Vec::new(),
        approvals: Vec::new(),
        household_decisions: Vec::new(),
    }
}

fn merge_authority_evidence(target: &mut AuthorityEvidence, source: AuthorityEvidence) {
    target.evaluated_at_micros = target.evaluated_at_micros.max(source.evaluated_at_micros);
    extend_unique(&mut target.capability_grants, source.capability_grants);
    extend_unique(&mut target.approvals, source.approvals);
    extend_unique(&mut target.household_decisions, source.household_decisions);
}

fn extend_unique(target: &mut Vec<String>, source: Vec<String>) {
    for item in source {
        if !target.contains(&item) {
            target.push(item);
        }
    }
}

fn finish_receipt(
    receipt: &mut AutomationReceipt,
    now_micros: i64,
    execution: ExecutionStatus,
    verification: VerificationStatus,
) {
    receipt.finished_at_micros = Some(now_micros);
    receipt.execution = execution;
    receipt.final_verification = verification;
}

fn validate_receipt(receipt: &AutomationReceipt) -> Result<(), EdgeError> {
    receipt
        .validate()
        .map_err(|error| EdgeError::ReceiptInvalid(error.to_string()))
}

fn backoff_ms(policy: &hearth_automation_types::RetryPolicy, attempt: u16) -> u64 {
    if policy.initial_backoff_ms == 0 {
        return 0;
    }
    let shift = u32::from(attempt.saturating_sub(1)).min(63);
    let factor = 1_u64.checked_shl(shift).unwrap_or(u64::MAX);
    policy
        .initial_backoff_ms
        .saturating_mul(factor)
        .min(policy.max_backoff_ms)
}

fn is_retryable_command(status: &CommandStatus) -> bool {
    matches!(
        status,
        CommandStatus::TimedOut
            | CommandStatus::AdapterUnavailable
            | CommandStatus::TransportError { .. }
    )
}

/// Deterministic in-memory adapter used to qualify edge semantics before any
/// physical adapter is admitted.
#[derive(Debug, Clone)]
pub struct SyntheticAdapter {
    name: String,
    capabilities: BTreeSet<String>,
    command_script: VecDeque<CommandStatus>,
    verification_script: VecDeque<VerificationReport>,
    replay_cache: BTreeMap<String, CommandStatus>,
    pub command_invocations: Vec<String>,
    pub physical_executions: Vec<String>,
}

impl SyntheticAdapter {
    pub fn new(name: impl Into<String>, capabilities: impl IntoIterator<Item = String>) -> Self {
        Self {
            name: name.into(),
            capabilities: capabilities.into_iter().collect(),
            command_script: VecDeque::new(),
            verification_script: VecDeque::new(),
            replay_cache: BTreeMap::new(),
            command_invocations: Vec::new(),
            physical_executions: Vec::new(),
        }
    }

    pub fn with_commands(mut self, script: Vec<CommandStatus>) -> Self {
        self.command_script = script.into();
        self
    }

    pub fn with_verifications(mut self, script: Vec<VerificationReport>) -> Self {
        self.verification_script = script.into();
        self
    }
}

impl EdgeAdapter for SyntheticAdapter {
    fn name(&self) -> &str {
        &self.name
    }

    fn supports_capability(&self, capability: &str) -> bool {
        self.capabilities.contains(capability)
    }

    fn supports(&self, action: &ActionSpec) -> bool {
        self.supports_capability(&action.capability)
    }

    fn execute(
        &mut self,
        idempotency_key: &str,
        _action: &ActionSpec,
        _timeout_ms: u64,
        _now_micros: i64,
    ) -> CommandStatus {
        self.command_invocations.push(idempotency_key.to_owned());
        if let Some(cached) = self.replay_cache.get(idempotency_key) {
            return cached.clone();
        }
        let status = self.command_script.pop_front().unwrap_or(CommandStatus::Accepted);
        if matches!(status, CommandStatus::Accepted) {
            self.physical_executions.push(idempotency_key.to_owned());
            self.replay_cache.insert(idempotency_key.to_owned(), status.clone());
        }
        status
    }

    fn verify(
        &mut self,
        _idempotency_key: &str,
        _expectations: &[OutcomeExpectation],
        _policy: &OutcomePolicy,
        _now_micros: i64,
    ) -> VerificationReport {
        self.verification_script
            .pop_front()
            .unwrap_or_else(|| VerificationReport::verified(Vec::new()))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use hearth_automation_policy::{LegacyCapabilityFact, RestrictionFact};
    use hearth_automation_types::{
        AuthorityRequirement, AutomationValue, ComparisonOp, ConsequenceClass, EntityRef,
        EvidenceRequirement, OutcomeExpectation, Reversibility, RetryPolicy, StateExpectation,
        StepId,
    };

    struct TestClock(i64);
    impl EdgeClock for TestClock {
        fn now_micros(&self) -> i64 { self.0 }
        fn sleep_ms(&mut self, millis: u64) {
            self.0 = self.0.saturating_add(i64::try_from(millis).unwrap_or(i64::MAX).saturating_mul(1_000));
        }
    }

    struct StaticAuthority {
        snapshot: AuthoritySnapshot,
        calls: usize,
        manual_override_after: Option<usize>,
    }

    impl AuthoritySource for StaticAuthority {
        fn snapshot(&mut self, _intent_id: &IntentId, _step: &PlanStep, now: i64) -> AuthoritySnapshot {
            self.calls += 1;
            let mut snapshot = self.snapshot.clone();
            snapshot.captured_at_micros = now;
            if self.manual_override_after.is_some_and(|limit| self.calls > limit) {
                snapshot.household_mode = HouseholdMode::ManualOverride;
            }
            snapshot
        }
    }

    fn authority(now: i64) -> StaticAuthority {
        StaticAuthority {
            snapshot: AuthoritySnapshot {
                actor_id: "agent-1".into(),
                household_mode: HouseholdMode::Normal,
                captured_at_micros: now,
                legacy_capabilities: vec![LegacyCapabilityFact {
                    profile_ref: "profile-1".into(),
                    capability: "home.lighting.control".into(),
                }],
                restrictions: Vec::<RestrictionFact>::new(),
                scoped_grants: Vec::new(),
                approvals: Vec::new(),
                household_decisions: Vec::new(),
            },
            calls: 0,
            manual_override_after: None,
        }
    }

    fn action(operation: &str) -> ActionSpec {
        ActionSpec {
            capability: "home.lighting.control".into(),
            target: EntityRef { kind: "light".into(), id: "kitchen".into() },
            operation: operation.into(),
            arguments: BTreeMap::new(),
        }
    }

    fn outcome(disposition: UnverifiedDisposition) -> OutcomePolicy {
        OutcomePolicy {
            expectations: vec![OutcomeExpectation::State(StateExpectation {
                subject: EntityRef { kind: "light".into(), id: "kitchen".into() },
                attribute: "on".into(),
                op: ComparisonOp::Eq,
                expected: AutomationValue::Bool(true),
            })],
            verify_within_ms: 5_000,
            evidence: EvidenceRequirement::default(),
            on_unverified: disposition,
        }
    }

    fn step(id: &str, deps: Vec<&str>) -> PlanStep {
        PlanStep {
            id: StepId::from(id),
            depends_on: deps.into_iter().map(StepId::from).collect(),
            preconditions: ConditionExpr::Always,
            action: action("on"),
            consequence: ConsequenceClass::ReversibleAct,
            reversibility: Reversibility::Reversible,
            authority: AuthorityRequirement::Capability { capability: "home.lighting.control".into() },
            outcome: outcome(UnverifiedDisposition::StopAndNotify),
            timeout_ms: 1_000,
            retry: RetryPolicy::none(),
            compensation: None,
        }
    }

    fn plan(steps: Vec<PlanStep>) -> AutomationPlan {
        AutomationPlan {
            schema_version: AUTOMATION_SCHEMA_VERSION,
            id: "plan-1".into(),
            intent_id: "intent-1".into(),
            generated_at_micros: 1,
            steps,
        }
    }

    #[test]
    fn command_acceptance_is_not_completion_without_verification() {
        let mut runtime = EdgeRuntime::new();
        runtime.register_adapter(
            SyntheticAdapter::new("synthetic", vec!["home.lighting.control".into()])
                .with_verifications(vec![VerificationReport::failed("sensor stayed off")]),
        );
        let mut clock = TestClock(100);
        let mut auth = authority(100);
        let result = runtime.execute_plan(
            &plan(vec![step("one", vec![])]), Vec::new(), &mut clock, &mut auth,
            &UnknownPredicates, &mut NeverCancel,
        ).unwrap();
        assert!(!matches!(result.receipt.execution, ExecutionStatus::Completed));
        assert!(!result.receipt.final_verification.is_verified());
        result.receipt.validate().unwrap();
    }

    #[test]
    fn dependency_order_is_deterministic() {
        let mut runtime = EdgeRuntime::new();
        runtime.register_adapter(SyntheticAdapter::new("synthetic", vec!["home.lighting.control".into()]));
        let mut clock = TestClock(100);
        let mut auth = authority(100);
        let result = runtime.execute_plan(
            &plan(vec![step("second", vec!["first"]), step("first", vec![])]),
            Vec::new(), &mut clock, &mut auth, &UnknownPredicates, &mut NeverCancel,
        ).unwrap();
        let ids: Vec<_> = result.receipt.steps.iter().map(|r| r.step_id.0.as_str()).collect();
        assert_eq!(ids, vec!["first", "second"]);
        assert!(matches!(result.receipt.execution, ExecutionStatus::Completed));
    }

    #[test]
    fn manual_override_stops_between_steps() {
        let mut runtime = EdgeRuntime::new();
        runtime.register_adapter(SyntheticAdapter::new("synthetic", vec!["home.lighting.control".into()]));
        let mut clock = TestClock(100);
        let mut auth = authority(100);
        auth.manual_override_after = Some(2);
        let result = runtime.execute_plan(
            &plan(vec![step("first", vec![]), step("second", vec!["first"])]),
            Vec::new(), &mut clock, &mut auth, &UnknownPredicates, &mut NeverCancel,
        ).unwrap();
        assert!(matches!(result.receipt.execution, ExecutionStatus::Inhibited(InhibitionReason::ManualOverride)));
        assert!(matches!(result.receipt.steps[1].command, CommandStatus::NotAttempted));
    }

    #[test]
    fn unknown_precondition_fails_closed() {
        let mut runtime = EdgeRuntime::new();
        runtime.register_adapter(SyntheticAdapter::new("synthetic", vec!["home.lighting.control".into()]));
        let mut guarded = step("one", vec![]);
        guarded.preconditions = ConditionExpr::Predicate(Condition::State {
            subject: EntityRef { kind: "room".into(), id: "kitchen".into() },
            attribute: "occupied".into(),
            op: ComparisonOp::Eq,
            expected: AutomationValue::Bool(true),
            evidence: EvidenceRequirement::default(),
        });
        let mut clock = TestClock(100);
        let mut auth = authority(100);
        let result = runtime.execute_plan(
            &plan(vec![guarded]), Vec::new(), &mut clock, &mut auth,
            &UnknownPredicates, &mut NeverCancel,
        ).unwrap();
        assert!(matches!(result.receipt.execution, ExecutionStatus::Inhibited(InhibitionReason::Policy(_))));
        assert!(matches!(result.receipt.steps[0].command, CommandStatus::NotAttempted));
    }

    #[test]
    fn synthetic_adapter_replays_accepted_idempotency_key_without_duplicate_side_effect() {
        let mut adapter = SyntheticAdapter::new("synthetic", vec!["home.lighting.control".into()]);
        let command = action("on");
        assert!(matches!(adapter.execute("same-key", &command, 1000, 1), CommandStatus::Accepted));
        assert!(matches!(adapter.execute("same-key", &command, 1000, 2), CommandStatus::Accepted));
        assert_eq!(adapter.command_invocations.len(), 2);
        assert_eq!(adapter.physical_executions.len(), 1);
    }

    #[test]
    fn compensation_runs_after_unverified_outcome() {
        let mut runtime = EdgeRuntime::new();
        runtime.register_adapter(
            SyntheticAdapter::new("synthetic", vec!["home.lighting.control".into()])
                .with_verifications(vec![VerificationReport::failed("unverified")]),
        );
        let mut compensated = step("one", vec![]);
        compensated.reversibility = Reversibility::Compensatable;
        compensated.compensation = Some(action("off"));
        compensated.outcome.on_unverified = UnverifiedDisposition::CompensateThenNotify;
        let mut clock = TestClock(100);
        let mut auth = authority(100);
        let result = runtime.execute_plan(
            &plan(vec![compensated]), Vec::new(), &mut clock, &mut auth,
            &UnknownPredicates, &mut NeverCancel,
        ).unwrap();
        assert!(result.trace.events.iter().any(|event| event.phase == TracePhase::Compensation));
        assert!(!matches!(result.receipt.execution, ExecutionStatus::Completed));
    }
}
