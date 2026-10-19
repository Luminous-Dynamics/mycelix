// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Pure-Rust household automation ontology for Mycelix Hearth.
//!
//! This crate deliberately has no HDI/HDK, browser, Matter, Home Assistant,
//! MQTT, or device-driver dependencies. It defines durable contracts shared by
//! the Hearth zome layer, local edge runtime, and user interfaces.
//!
//! Core invariants:
//! - durable values are integer/fixed-point; there is no `f32`/`f64` state,
//! - consequence class and authority are explicit,
//! - command acceptance is distinct from outcome verification,
//! - plans target semantic capabilities, not adapter/device APIs,
//! - high-consequence actions fail closed when authority is insufficient.

use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};
use std::fmt;

pub const AUTOMATION_SCHEMA_VERSION: u16 = 1;
pub const BASIS_POINTS_MAX: u32 = 10_000;

macro_rules! id_type {
    ($name:ident) => {
        #[derive(
            Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize,
        )]
        pub struct $name(pub String);

        impl $name {
            pub fn new(value: impl Into<String>) -> Self {
                Self(value.into())
            }

            pub fn is_empty(&self) -> bool {
                self.0.trim().is_empty()
            }
        }

        impl From<String> for $name {
            fn from(value: String) -> Self {
                Self(value)
            }
        }

        impl From<&str> for $name {
            fn from(value: &str) -> Self {
                Self(value.to_owned())
            }
        }
    };
}

id_type!(IntentId);
id_type!(PlanId);
id_type!(ReceiptId);
id_type!(StepId);
id_type!(HearthId);

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ValidationError {
    pub path: String,
    pub message: String,
}

impl ValidationError {
    fn new(path: impl Into<String>, message: impl Into<String>) -> Self {
        Self {
            path: path.into(),
            message: message.into(),
        }
    }
}

impl fmt::Display for ValidationError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}: {}", self.path, self.message)
    }
}

impl std::error::Error for ValidationError {}

fn require_nonempty(path: &str, value: &str) -> Result<(), ValidationError> {
    if value.trim().is_empty() {
        Err(ValidationError::new(path, "must not be empty"))
    } else {
        Ok(())
    }
}

/// Opaque reference to a household entity.
///
/// `kind` is semantic ("room", "vehicle", "care_task"), never an adapter name.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub struct EntityRef {
    pub kind: String,
    pub id: String,
}

impl EntityRef {
    pub fn validate(&self, path: &str) -> Result<(), ValidationError> {
        require_nonempty(&format!("{path}.kind"), &self.kind)?;
        require_nonempty(&format!("{path}.id"), &self.id)
    }
}

/// Deterministic value representation for durable automation contracts.
///
/// Physical quantities should use `Fixed` with an explicit decimal scale.
/// Example: 21.5 C can be represented as `Fixed { mantissa: 215, scale: 1 }`.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum AutomationValue {
    Bool(bool),
    Signed(i64),
    Unsigned(u64),
    BasisPoints(u32),
    Fixed { mantissa: i64, scale: u8 },
    Text(String),
    TextList(Vec<String>),
}

impl AutomationValue {
    pub fn validate(&self, path: &str) -> Result<(), ValidationError> {
        match self {
            Self::BasisPoints(value) if *value > BASIS_POINTS_MAX => Err(ValidationError::new(
                path,
                format!("basis points must be <= {BASIS_POINTS_MAX}"),
            )),
            Self::Fixed { scale, .. } if *scale > 18 => Err(ValidationError::new(
                path,
                "fixed-point scale must be <= 18",
            )),
            _ => Ok(()),
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct EvidenceSource {
    pub namespace: String,
    pub id: String,
}

impl EvidenceSource {
    pub fn validate(&self, path: &str) -> Result<(), ValidationError> {
        require_nonempty(&format!("{path}.namespace"), &self.namespace)?;
        require_nonempty(&format!("{path}.id"), &self.id)
    }
}

/// A timestamped observation about household reality.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Observation {
    pub subject: EntityRef,
    pub attribute: String,
    pub value: AutomationValue,
    pub source: EvidenceSource,
    pub observed_at_micros: i64,
    pub valid_until_micros: Option<i64>,
    pub confidence_bp: u32,
}

impl Observation {
    pub fn validate(&self, path: &str) -> Result<(), ValidationError> {
        self.subject.validate(&format!("{path}.subject"))?;
        require_nonempty(&format!("{path}.attribute"), &self.attribute)?;
        self.value.validate(&format!("{path}.value"))?;
        self.source.validate(&format!("{path}.source"))?;
        if self.confidence_bp > BASIS_POINTS_MAX {
            return Err(ValidationError::new(
                format!("{path}.confidence_bp"),
                format!("must be <= {BASIS_POINTS_MAX}"),
            ));
        }
        if let Some(valid_until) = self.valid_until_micros
            && valid_until < self.observed_at_micros
        {
            return Err(ValidationError::new(
                format!("{path}.valid_until_micros"),
                "must not precede observed_at_micros",
            ));
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct EvidenceRequirement {
    pub max_age_ms: Option<u64>,
    pub min_confidence_bp: u32,
    pub min_independent_sources: u16,
    pub required_namespaces: Vec<String>,
}

impl Default for EvidenceRequirement {
    fn default() -> Self {
        Self {
            max_age_ms: None,
            min_confidence_bp: 0,
            min_independent_sources: 1,
            required_namespaces: Vec::new(),
        }
    }
}

impl EvidenceRequirement {
    pub fn validate(&self, path: &str) -> Result<(), ValidationError> {
        if self.min_confidence_bp > BASIS_POINTS_MAX {
            return Err(ValidationError::new(
                format!("{path}.min_confidence_bp"),
                format!("must be <= {BASIS_POINTS_MAX}"),
            ));
        }
        if self.min_independent_sources == 0 {
            return Err(ValidationError::new(
                format!("{path}.min_independent_sources"),
                "must be at least 1",
            ));
        }
        if self.max_age_ms == Some(0) {
            return Err(ValidationError::new(
                format!("{path}.max_age_ms"),
                "must be positive when specified",
            ));
        }
        for (index, namespace) in self.required_namespaces.iter().enumerate() {
            require_nonempty(
                &format!("{path}.required_namespaces[{index}]"),
                namespace,
            )?;
        }
        Ok(())
    }
}

/// Household execution context. Emergency and manual override are explicit
/// because they may inhibit otherwise valid plans.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum HouseholdMode {
    Normal,
    Conservation,
    Away,
    Sleeping,
    Emergency,
    Maintenance,
    ManualOverride,
}

/// Consequence ladder. Variant order is intentionally A0 -> A5.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum ConsequenceClass {
    /// A0: observe only; no household state is changed.
    Observe,
    /// A1: organize information/tasks without external side effects.
    Organize,
    /// A2: recommend an action to a person.
    Recommend,
    /// A3: perform a reversible physical/digital action.
    ReversibleAct,
    /// A4: consequential action such as purchase/access/energy export.
    ConsequentialAct,
    /// A5: safety-critical or otherwise critical action.
    CriticalAct,
}

impl ConsequenceClass {
    pub const fn level(self) -> u8 {
        match self {
            Self::Observe => 0,
            Self::Organize => 1,
            Self::Recommend => 2,
            Self::ReversibleAct => 3,
            Self::ConsequentialAct => 4,
            Self::CriticalAct => 5,
        }
    }

    pub const fn requires_execution_authority(self) -> bool {
        self.level() >= 3
    }

    pub const fn requires_verified_outcome(self) -> bool {
        self.level() >= 3
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum Reversibility {
    Reversible,
    Compensatable,
    Irreversible,
}

/// Durable authority requirement. HTH-AUTO-002 will bind these references to
/// Hearth autonomy profiles, approvals, and decision records.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum AuthorityRequirement {
    None,
    Capability { capability: String },
    ExplicitApproval {
        capability: Option<String>,
        approvals_required: u16,
    },
    HouseholdDecision { decision_ref: String },
    Forbidden,
}

impl AuthorityRequirement {
    pub fn validate_for(
        &self,
        consequence: ConsequenceClass,
        path: &str,
    ) -> Result<(), ValidationError> {
        match self {
            Self::Capability { capability } => {
                require_nonempty(&format!("{path}.capability"), capability)?
            }
            Self::ExplicitApproval {
                capability,
                approvals_required,
            } => {
                if *approvals_required == 0 {
                    return Err(ValidationError::new(
                        format!("{path}.approvals_required"),
                        "must be at least 1",
                    ));
                }
                if let Some(capability) = capability {
                    require_nonempty(&format!("{path}.capability"), capability)?;
                }
            }
            Self::HouseholdDecision { decision_ref } => {
                require_nonempty(&format!("{path}.decision_ref"), decision_ref)?
            }
            Self::None | Self::Forbidden => {}
        }

        if consequence.requires_execution_authority() && matches!(self, Self::None) {
            return Err(ValidationError::new(
                path,
                "A3+ actions require explicit authority or an explicit prohibition",
            ));
        }

        if consequence == ConsequenceClass::CriticalAct
            && !matches!(
                self,
                Self::ExplicitApproval { .. } | Self::HouseholdDecision { .. } | Self::Forbidden
            )
        {
            return Err(ValidationError::new(
                path,
                "A5 critical actions require explicit approval, a household decision, or prohibition",
            ));
        }

        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum ScheduleSpec {
    AtMicros(i64),
    EveryMs {
        period_ms: u64,
        phase_offset_ms: Option<u64>,
    },
    /// iCalendar RRULE text interpreted by the scheduler layer.
    CalendarRule {
        rrule: String,
        timezone: String,
    },
}

impl ScheduleSpec {
    pub fn validate(&self, path: &str) -> Result<(), ValidationError> {
        match self {
            Self::EveryMs {
                period_ms,
                phase_offset_ms,
            } => {
                if *period_ms == 0 {
                    return Err(ValidationError::new(
                        format!("{path}.period_ms"),
                        "must be positive",
                    ));
                }
                if let Some(offset) = phase_offset_ms
                    && *offset >= *period_ms
                {
                    return Err(ValidationError::new(
                        format!("{path}.phase_offset_ms"),
                        "must be smaller than period_ms",
                    ));
                }
            }
            Self::CalendarRule { rrule, timezone } => {
                require_nonempty(&format!("{path}.rrule"), rrule)?;
                require_nonempty(&format!("{path}.timezone"), timezone)?;
            }
            Self::AtMicros(_) => {}
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum Trigger {
    Manual,
    Schedule(ScheduleSpec),
    DomainEvent {
        domain: String,
        event: String,
        subject: Option<EntityRef>,
    },
    StateTransition {
        subject: EntityRef,
        attribute: String,
        to: AutomationValue,
    },
    ExternalSignal {
        namespace: String,
        signal_type: String,
    },
}

impl Trigger {
    pub fn validate(&self, path: &str) -> Result<(), ValidationError> {
        match self {
            Self::Manual => Ok(()),
            Self::Schedule(spec) => spec.validate(&format!("{path}.schedule")),
            Self::DomainEvent {
                domain,
                event,
                subject,
            } => {
                require_nonempty(&format!("{path}.domain"), domain)?;
                require_nonempty(&format!("{path}.event"), event)?;
                if let Some(subject) = subject {
                    subject.validate(&format!("{path}.subject"))?;
                }
                Ok(())
            }
            Self::StateTransition {
                subject,
                attribute,
                to,
            } => {
                subject.validate(&format!("{path}.subject"))?;
                require_nonempty(&format!("{path}.attribute"), attribute)?;
                to.validate(&format!("{path}.to"))
            }
            Self::ExternalSignal {
                namespace,
                signal_type,
            } => {
                require_nonempty(&format!("{path}.namespace"), namespace)?;
                require_nonempty(&format!("{path}.signal_type"), signal_type)
            }
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ComparisonOp {
    Eq,
    Ne,
    Lt,
    Le,
    Gt,
    Ge,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum Condition {
    State {
        subject: EntityRef,
        attribute: String,
        op: ComparisonOp,
        expected: AutomationValue,
        evidence: EvidenceRequirement,
    },
    ModeIn(Vec<HouseholdMode>),
    CapabilityAvailable { capability: String },
    DomainPredicate {
        domain: String,
        predicate: String,
        parameters: BTreeMap<String, AutomationValue>,
    },
}

impl Condition {
    pub fn validate(&self, path: &str) -> Result<(), ValidationError> {
        match self {
            Self::State {
                subject,
                attribute,
                expected,
                evidence,
                ..
            } => {
                subject.validate(&format!("{path}.subject"))?;
                require_nonempty(&format!("{path}.attribute"), attribute)?;
                expected.validate(&format!("{path}.expected"))?;
                evidence.validate(&format!("{path}.evidence"))
            }
            Self::ModeIn(modes) => {
                if modes.is_empty() {
                    Err(ValidationError::new(path, "mode allowlist must not be empty"))
                } else {
                    Ok(())
                }
            }
            Self::CapabilityAvailable { capability } => {
                require_nonempty(&format!("{path}.capability"), capability)
            }
            Self::DomainPredicate {
                domain,
                predicate,
                parameters,
            } => {
                require_nonempty(&format!("{path}.domain"), domain)?;
                require_nonempty(&format!("{path}.predicate"), predicate)?;
                for (key, value) in parameters {
                    require_nonempty(&format!("{path}.parameters.key"), key)?;
                    value.validate(&format!("{path}.parameters[{key}]"))?;
                }
                Ok(())
            }
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum ConditionExpr {
    Always,
    Predicate(Condition),
    All(Vec<ConditionExpr>),
    Any(Vec<ConditionExpr>),
    Not(Box<ConditionExpr>),
}

impl ConditionExpr {
    pub fn validate(&self, path: &str) -> Result<(), ValidationError> {
        match self {
            Self::Always => Ok(()),
            Self::Predicate(condition) => condition.validate(path),
            Self::All(children) | Self::Any(children) => {
                if children.is_empty() {
                    return Err(ValidationError::new(path, "condition group must not be empty"));
                }
                for (index, child) in children.iter().enumerate() {
                    child.validate(&format!("{path}[{index}]"))?;
                }
                Ok(())
            }
            Self::Not(child) => child.validate(&format!("{path}.not")),
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct GoalSpec {
    pub kind: String,
    pub parameters: BTreeMap<String, AutomationValue>,
}

impl GoalSpec {
    pub fn validate(&self, path: &str) -> Result<(), ValidationError> {
        require_nonempty(&format!("{path}.kind"), &self.kind)?;
        for (key, value) in &self.parameters {
            require_nonempty(&format!("{path}.parameters.key"), key)?;
            value.validate(&format!("{path}.parameters[{key}]"))?;
        }
        Ok(())
    }
}

/// Semantic operation. The edge runtime resolves this into a concrete adapter.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ActionSpec {
    pub capability: String,
    pub target: EntityRef,
    pub operation: String,
    pub arguments: BTreeMap<String, AutomationValue>,
}

impl ActionSpec {
    pub fn validate(&self, path: &str) -> Result<(), ValidationError> {
        require_nonempty(&format!("{path}.capability"), &self.capability)?;
        self.target.validate(&format!("{path}.target"))?;
        require_nonempty(&format!("{path}.operation"), &self.operation)?;
        for (key, value) in &self.arguments {
            require_nonempty(&format!("{path}.arguments.key"), key)?;
            value.validate(&format!("{path}.arguments[{key}]"))?;
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct StateExpectation {
    pub subject: EntityRef,
    pub attribute: String,
    pub op: ComparisonOp,
    pub expected: AutomationValue,
}

impl StateExpectation {
    pub fn validate(&self, path: &str) -> Result<(), ValidationError> {
        self.subject.validate(&format!("{path}.subject"))?;
        require_nonempty(&format!("{path}.attribute"), &self.attribute)?;
        self.expected.validate(&format!("{path}.expected"))
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum OutcomeExpectation {
    State(StateExpectation),
    DomainEvent {
        domain: String,
        event: String,
        subject: Option<EntityRef>,
    },
}

impl OutcomeExpectation {
    pub fn validate(&self, path: &str) -> Result<(), ValidationError> {
        match self {
            Self::State(expectation) => expectation.validate(path),
            Self::DomainEvent {
                domain,
                event,
                subject,
            } => {
                require_nonempty(&format!("{path}.domain"), domain)?;
                require_nonempty(&format!("{path}.event"), event)?;
                if let Some(subject) = subject {
                    subject.validate(&format!("{path}.subject"))?;
                }
                Ok(())
            }
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum UnverifiedDisposition {
    StopAndNotify,
    RetryThenNotify,
    CompensateThenNotify,
    EscalateForApproval,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct OutcomePolicy {
    pub expectations: Vec<OutcomeExpectation>,
    pub verify_within_ms: u64,
    pub evidence: EvidenceRequirement,
    pub on_unverified: UnverifiedDisposition,
}

impl OutcomePolicy {
    pub fn validate_for(
        &self,
        consequence: ConsequenceClass,
        path: &str,
    ) -> Result<(), ValidationError> {
        self.evidence.validate(&format!("{path}.evidence"))?;
        if consequence.requires_verified_outcome() {
            if self.expectations.is_empty() {
                return Err(ValidationError::new(
                    format!("{path}.expectations"),
                    "A3+ actions require an observable outcome expectation",
                ));
            }
            if self.verify_within_ms == 0 {
                return Err(ValidationError::new(
                    format!("{path}.verify_within_ms"),
                    "A3+ actions require a positive verification window",
                ));
            }
        }
        for (index, expectation) in self.expectations.iter().enumerate() {
            expectation.validate(&format!("{path}.expectations[{index}]"))?;
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum AutomationLifecycle {
    Draft,
    Trial { expires_at_micros: i64 },
    Active,
    Suspended,
    Retired,
}

/// Human-meaningful intent. It contains no adapter/device implementation data.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct AutomationIntent {
    pub schema_version: u16,
    pub id: IntentId,
    pub hearth_id: HearthId,
    pub name: String,
    pub description: String,
    pub created_by: String,
    pub created_at_micros: i64,
    pub lifecycle: AutomationLifecycle,
    pub goal: GoalSpec,
    pub triggers: Vec<Trigger>,
    pub conditions: ConditionExpr,
    pub consequence: ConsequenceClass,
    pub reversibility: Reversibility,
    pub authority: AuthorityRequirement,
    pub outcome: OutcomePolicy,
}

impl AutomationIntent {
    pub fn validate(&self) -> Result<(), ValidationError> {
        if self.schema_version != AUTOMATION_SCHEMA_VERSION {
            return Err(ValidationError::new(
                "schema_version",
                format!(
                    "unsupported version {}; expected {}",
                    self.schema_version, AUTOMATION_SCHEMA_VERSION
                ),
            ));
        }
        if self.id.is_empty() {
            return Err(ValidationError::new("id", "must not be empty"));
        }
        if self.hearth_id.is_empty() {
            return Err(ValidationError::new("hearth_id", "must not be empty"));
        }
        require_nonempty("name", &self.name)?;
        require_nonempty("created_by", &self.created_by)?;
        self.goal.validate("goal")?;
        if self.triggers.is_empty() {
            return Err(ValidationError::new(
                "triggers",
                "an intent must declare at least one trigger, including Manual if appropriate",
            ));
        }
        for (index, trigger) in self.triggers.iter().enumerate() {
            trigger.validate(&format!("triggers[{index}]"))?;
        }
        self.conditions.validate("conditions")?;
        self.authority
            .validate_for(self.consequence, "authority")?;
        self.outcome.validate_for(self.consequence, "outcome")?;
        if let AutomationLifecycle::Trial { expires_at_micros } = self.lifecycle
            && expires_at_micros <= self.created_at_micros
        {
            return Err(ValidationError::new(
                "lifecycle.expires_at_micros",
                "trial must expire after intent creation",
            ));
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct RetryPolicy {
    /// Total attempts including the first attempt.
    pub max_attempts: u16,
    pub initial_backoff_ms: u64,
    pub max_backoff_ms: u64,
}

impl RetryPolicy {
    pub fn none() -> Self {
        Self {
            max_attempts: 1,
            initial_backoff_ms: 0,
            max_backoff_ms: 0,
        }
    }

    pub fn validate(&self, path: &str) -> Result<(), ValidationError> {
        if self.max_attempts == 0 {
            return Err(ValidationError::new(
                format!("{path}.max_attempts"),
                "must be at least 1",
            ));
        }
        if self.max_backoff_ms < self.initial_backoff_ms {
            return Err(ValidationError::new(
                format!("{path}.max_backoff_ms"),
                "must be >= initial_backoff_ms",
            ));
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct PlanStep {
    pub id: StepId,
    pub depends_on: Vec<StepId>,
    pub preconditions: ConditionExpr,
    pub action: ActionSpec,
    pub consequence: ConsequenceClass,
    pub reversibility: Reversibility,
    pub authority: AuthorityRequirement,
    pub outcome: OutcomePolicy,
    pub timeout_ms: u64,
    pub retry: RetryPolicy,
    pub compensation: Option<ActionSpec>,
}

impl PlanStep {
    pub fn validate(&self, path: &str) -> Result<(), ValidationError> {
        if self.id.is_empty() {
            return Err(ValidationError::new(
                format!("{path}.id"),
                "must not be empty",
            ));
        }
        self.preconditions
            .validate(&format!("{path}.preconditions"))?;
        self.action.validate(&format!("{path}.action"))?;
        self.authority
            .validate_for(self.consequence, &format!("{path}.authority"))?;
        self.outcome
            .validate_for(self.consequence, &format!("{path}.outcome"))?;
        if self.timeout_ms == 0 {
            return Err(ValidationError::new(
                format!("{path}.timeout_ms"),
                "must be positive",
            ));
        }
        self.retry.validate(&format!("{path}.retry"))?;
        if self.reversibility == Reversibility::Compensatable && self.compensation.is_none() {
            return Err(ValidationError::new(
                format!("{path}.compensation"),
                "compensatable steps require a compensation action",
            ));
        }
        if let Some(compensation) = &self.compensation {
            compensation.validate(&format!("{path}.compensation"))?;
        }
        Ok(())
    }
}

/// Deterministic executable plan. Adapter selection remains a local edge concern.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct AutomationPlan {
    pub schema_version: u16,
    pub id: PlanId,
    pub intent_id: IntentId,
    pub generated_at_micros: i64,
    pub steps: Vec<PlanStep>,
}

impl AutomationPlan {
    pub fn validate(&self) -> Result<(), ValidationError> {
        if self.schema_version != AUTOMATION_SCHEMA_VERSION {
            return Err(ValidationError::new(
                "schema_version",
                "unsupported plan schema version",
            ));
        }
        if self.id.is_empty() {
            return Err(ValidationError::new("id", "must not be empty"));
        }
        if self.intent_id.is_empty() {
            return Err(ValidationError::new("intent_id", "must not be empty"));
        }
        if self.steps.is_empty() {
            return Err(ValidationError::new("steps", "plan must contain at least one step"));
        }

        let ids: BTreeSet<_> = self.steps.iter().map(|step| step.id.clone()).collect();
        if ids.len() != self.steps.len() {
            return Err(ValidationError::new("steps", "step IDs must be unique"));
        }

        for (index, step) in self.steps.iter().enumerate() {
            step.validate(&format!("steps[{index}]"))?;
            for dependency in &step.depends_on {
                if dependency == &step.id {
                    return Err(ValidationError::new(
                        format!("steps[{index}].depends_on"),
                        "step may not depend on itself",
                    ));
                }
                if !ids.contains(dependency) {
                    return Err(ValidationError::new(
                        format!("steps[{index}].depends_on"),
                        format!("unknown dependency {}", dependency.0),
                    ));
                }
            }
        }

        let mut remaining: BTreeMap<StepId, BTreeSet<StepId>> = self
            .steps
            .iter()
            .map(|step| {
                (
                    step.id.clone(),
                    step.depends_on.iter().cloned().collect::<BTreeSet<_>>(),
                )
            })
            .collect();

        while !remaining.is_empty() {
            let ready: Vec<_> = remaining
                .iter()
                .filter_map(|(id, dependencies)| dependencies.is_empty().then_some(id.clone()))
                .collect();
            if ready.is_empty() {
                return Err(ValidationError::new(
                    "steps",
                    "dependency graph contains a cycle",
                ));
            }
            for id in ready {
                remaining.remove(&id);
                for dependencies in remaining.values_mut() {
                    dependencies.remove(&id);
                }
            }
        }

        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct EvidenceRef {
    pub evidence_id: String,
    pub source: EvidenceSource,
    pub observed_at_micros: i64,
    pub digest: Option<String>,
}

impl EvidenceRef {
    pub fn validate(&self, path: &str) -> Result<(), ValidationError> {
        require_nonempty(&format!("{path}.evidence_id"), &self.evidence_id)?;
        self.source.validate(&format!("{path}.source"))?;
        if let Some(digest) = &self.digest {
            require_nonempty(&format!("{path}.digest"), digest)?;
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct AuthorityEvidence {
    pub evaluated_at_micros: i64,
    pub capability_grants: Vec<String>,
    pub approvals: Vec<String>,
    pub household_decisions: Vec<String>,
}

impl AuthorityEvidence {
    pub fn is_empty(&self) -> bool {
        self.capability_grants.is_empty()
            && self.approvals.is_empty()
            && self.household_decisions.is_empty()
    }
}

/// Result of sending a command to an executor/adapter.
///
/// `Accepted` does NOT mean the requested outcome occurred.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum CommandStatus {
    NotAttempted,
    Accepted,
    Rejected { reason: String },
    TimedOut,
    AdapterUnavailable,
    TransportError { message: String },
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum VerificationStatus {
    NotAttempted,
    Pending,
    Verified,
    PartiallyVerified,
    Failed { reason: String },
    Unknown { reason: String },
}

impl VerificationStatus {
    pub fn is_verified(&self) -> bool {
        matches!(self, Self::Verified)
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum InhibitionReason {
    HouseholdMode(HouseholdMode),
    MissingAuthority,
    StaleEvidence,
    ConflictingIntent,
    ManualOverride,
    AdapterUnavailable,
    Policy(String),
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum ExecutionStatus {
    Planned,
    Running,
    Completed,
    Failed { reason: String },
    Cancelled { reason: String },
    Inhibited(InhibitionReason),
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct StepReceipt {
    pub step_id: StepId,
    pub started_at_micros: i64,
    pub finished_at_micros: Option<i64>,
    pub command: CommandStatus,
    pub verification: VerificationStatus,
    pub observations: Vec<EvidenceRef>,
}

impl StepReceipt {
    pub fn validate(&self, path: &str) -> Result<(), ValidationError> {
        if self.step_id.is_empty() {
            return Err(ValidationError::new(
                format!("{path}.step_id"),
                "must not be empty",
            ));
        }
        if let Some(finished) = self.finished_at_micros
            && finished < self.started_at_micros
        {
            return Err(ValidationError::new(
                format!("{path}.finished_at_micros"),
                "must not precede started_at_micros",
            ));
        }
        for (index, observation) in self.observations.iter().enumerate() {
            observation.validate(&format!("{path}.observations[{index}]"))?;
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct AutomationReceipt {
    pub schema_version: u16,
    pub id: ReceiptId,
    pub intent_id: IntentId,
    pub plan_id: PlanId,
    pub started_at_micros: i64,
    pub finished_at_micros: Option<i64>,
    pub execution: ExecutionStatus,
    pub trigger_evidence: Vec<EvidenceRef>,
    pub authority_evidence: AuthorityEvidence,
    pub steps: Vec<StepReceipt>,
    pub final_verification: VerificationStatus,
}

impl AutomationReceipt {
    pub fn validate(&self) -> Result<(), ValidationError> {
        if self.schema_version != AUTOMATION_SCHEMA_VERSION {
            return Err(ValidationError::new(
                "schema_version",
                "unsupported receipt schema version",
            ));
        }
        if self.id.is_empty() {
            return Err(ValidationError::new("id", "must not be empty"));
        }
        if self.intent_id.is_empty() {
            return Err(ValidationError::new("intent_id", "must not be empty"));
        }
        if self.plan_id.is_empty() {
            return Err(ValidationError::new("plan_id", "must not be empty"));
        }
        if let Some(finished) = self.finished_at_micros
            && finished < self.started_at_micros
        {
            return Err(ValidationError::new(
                "finished_at_micros",
                "must not precede started_at_micros",
            ));
        }
        for (index, evidence) in self.trigger_evidence.iter().enumerate() {
            evidence.validate(&format!("trigger_evidence[{index}]"))?;
        }
        for (index, step) in self.steps.iter().enumerate() {
            step.validate(&format!("steps[{index}]"))?;
        }
        if matches!(self.execution, ExecutionStatus::Completed)
            && !self.final_verification.is_verified()
        {
            return Err(ValidationError::new(
                "final_verification",
                "Completed receipts require a verified final outcome",
            ));
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn entity(id: &str) -> EntityRef {
        EntityRef {
            kind: "appliance".into(),
            id: id.into(),
        }
    }

    fn verified_outcome() -> OutcomePolicy {
        OutcomePolicy {
            expectations: vec![OutcomeExpectation::State(StateExpectation {
                subject: entity("washer"),
                attribute: "running".into(),
                op: ComparisonOp::Eq,
                expected: AutomationValue::Bool(false),
            })],
            verify_within_ms: 10_000,
            evidence: EvidenceRequirement::default(),
            on_unverified: UnverifiedDisposition::StopAndNotify,
        }
    }

    fn step(id: &str) -> PlanStep {
        PlanStep {
            id: StepId::from(id),
            depends_on: Vec::new(),
            preconditions: ConditionExpr::Always,
            action: ActionSpec {
                capability: "home.appliance.control".into(),
                target: entity("washer"),
                operation: "stop".into(),
                arguments: BTreeMap::new(),
            },
            consequence: ConsequenceClass::ReversibleAct,
            reversibility: Reversibility::Reversible,
            authority: AuthorityRequirement::Capability {
                capability: "home.appliance.control".into(),
            },
            outcome: verified_outcome(),
            timeout_ms: 5_000,
            retry: RetryPolicy::none(),
            compensation: None,
        }
    }

    fn intent(consequence: ConsequenceClass, authority: AuthorityRequirement) -> AutomationIntent {
        AutomationIntent {
            schema_version: AUTOMATION_SCHEMA_VERSION,
            id: IntentId::from("intent-1"),
            hearth_id: HearthId::from("hearth-1"),
            name: "Keep the appliance safe".into(),
            description: String::new(),
            created_by: "agent-1".into(),
            created_at_micros: 100,
            lifecycle: AutomationLifecycle::Active,
            goal: GoalSpec {
                kind: "home.appliance.safe_state".into(),
                parameters: BTreeMap::new(),
            },
            triggers: vec![Trigger::Manual],
            conditions: ConditionExpr::Always,
            consequence,
            reversibility: Reversibility::Reversible,
            authority,
            outcome: if consequence.requires_verified_outcome() {
                verified_outcome()
            } else {
                OutcomePolicy {
                    expectations: Vec::new(),
                    verify_within_ms: 0,
                    evidence: EvidenceRequirement::default(),
                    on_unverified: UnverifiedDisposition::StopAndNotify,
                }
            },
        }
    }

    #[test]
    fn critical_intent_rejects_capability_only_authority() {
        let intent = intent(
            ConsequenceClass::CriticalAct,
            AuthorityRequirement::Capability {
                capability: "home.emergency.act".into(),
            },
        );
        assert!(intent.validate().is_err());
    }

    #[test]
    fn critical_intent_accepts_explicit_approval() {
        let intent = intent(
            ConsequenceClass::CriticalAct,
            AuthorityRequirement::ExplicitApproval {
                capability: Some("home.emergency.act".into()),
                approvals_required: 1,
            },
        );
        assert!(intent.validate().is_ok());
    }

    #[test]
    fn a3_action_requires_observable_outcome() {
        let mut intent = intent(
            ConsequenceClass::ReversibleAct,
            AuthorityRequirement::Capability {
                capability: "home.lighting.control".into(),
            },
        );
        intent.outcome.expectations.clear();
        assert!(intent.validate().is_err());
    }

    #[test]
    fn command_acceptance_does_not_equal_verification() {
        let receipt = StepReceipt {
            step_id: StepId::from("step-1"),
            started_at_micros: 100,
            finished_at_micros: Some(101),
            command: CommandStatus::Accepted,
            verification: VerificationStatus::Pending,
            observations: Vec::new(),
        };
        assert!(matches!(receipt.command, CommandStatus::Accepted));
        assert!(!receipt.verification.is_verified());
    }

    #[test]
    fn completed_receipt_requires_final_verification() {
        let receipt = AutomationReceipt {
            schema_version: AUTOMATION_SCHEMA_VERSION,
            id: ReceiptId::from("receipt-1"),
            intent_id: IntentId::from("intent-1"),
            plan_id: PlanId::from("plan-1"),
            started_at_micros: 100,
            finished_at_micros: Some(200),
            execution: ExecutionStatus::Completed,
            trigger_evidence: Vec::new(),
            authority_evidence: AuthorityEvidence {
                evaluated_at_micros: 100,
                capability_grants: vec!["grant-1".into()],
                approvals: Vec::new(),
                household_decisions: Vec::new(),
            },
            steps: Vec::new(),
            final_verification: VerificationStatus::Pending,
        };
        assert!(receipt.validate().is_err());
    }

    #[test]
    fn plan_rejects_dependency_cycle() {
        let mut first = step("a");
        first.depends_on.push(StepId::from("b"));
        let mut second = step("b");
        second.depends_on.push(StepId::from("a"));
        let plan = AutomationPlan {
            schema_version: AUTOMATION_SCHEMA_VERSION,
            id: PlanId::from("plan-1"),
            intent_id: IntentId::from("intent-1"),
            generated_at_micros: 100,
            steps: vec![first, second],
        };
        assert!(plan.validate().is_err());
    }

    #[test]
    fn compensatable_step_requires_compensation() {
        let mut candidate = step("a");
        candidate.reversibility = Reversibility::Compensatable;
        candidate.compensation = None;
        assert!(candidate.validate("step").is_err());
    }

    #[test]
    fn observation_rejects_invalid_confidence() {
        let observation = Observation {
            subject: entity("sensor"),
            attribute: "temperature".into(),
            value: AutomationValue::Fixed {
                mantissa: 215,
                scale: 1,
            },
            source: EvidenceSource {
                namespace: "matter".into(),
                id: "sensor-1".into(),
            },
            observed_at_micros: 100,
            valid_until_micros: Some(200),
            confidence_bp: BASIS_POINTS_MAX + 1,
        };
        assert!(observation.validate("observation").is_err());
    }

    #[test]
    fn trial_must_expire_after_creation() {
        let mut candidate = intent(ConsequenceClass::Recommend, AuthorityRequirement::None);
        candidate.lifecycle = AutomationLifecycle::Trial {
            expires_at_micros: 100,
        };
        assert!(candidate.validate().is_err());
    }

    #[test]
    fn serde_roundtrip_preserves_contract() {
        let original = intent(
            ConsequenceClass::ReversibleAct,
            AuthorityRequirement::Capability {
                capability: "home.appliance.control".into(),
            },
        );
        let encoded = serde_json::to_string(&original).expect("serialize");
        let decoded: AutomationIntent = serde_json::from_str(&encoded).expect("deserialize");
        assert_eq!(decoded, original);
        assert!(decoded.validate().is_ok());
    }
}
