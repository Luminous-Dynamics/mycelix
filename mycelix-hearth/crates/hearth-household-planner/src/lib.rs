// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Deterministic Care + Rhythms household work planner.
//!
//! This crate is intentionally pure Rust/Serde. It does not call Holochain,
//! parse human-written recurrence strings, infer task duration, or execute
//! household changes. It plans only from explicit normalized facts.

use hearth_automation_types::{
    ActionSpec, AuthorityRequirement, AutomationPlan, AutomationValue, ConsequenceClass,
    EntityRef, EvidenceRequirement, IntentId, OutcomePolicy, PlanId, PlanStep, Reversibility,
    RetryPolicy, StepId, UnverifiedDisposition, AUTOMATION_SCHEMA_VERSION,
};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};

pub const PLANNER_SCHEMA_VERSION: u16 = 1;
pub const BASIS_POINTS_MAX: u32 = 10_000;
const MICROS_PER_MINUTE: i64 = 60_000_000;

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct TimeWindow {
    pub start_micros: i64,
    pub end_micros: i64,
}

impl TimeWindow {
    pub fn validate(&self, path: &str) -> Result<(), PlannerError> {
        if self.end_micros <= self.start_micros {
            return Err(PlannerError::InvalidInput(format!(
                "{path}: end_micros must be after start_micros"
            )));
        }
        Ok(())
    }

    pub fn overlaps(&self, other: &Self) -> bool {
        self.start_micros < other.end_micros && other.start_micros < self.end_micros
    }

    pub fn intersection(&self, other: &Self) -> Option<Self> {
        let start = self.start_micros.max(other.start_micros);
        let end = self.end_micros.min(other.end_micros);
        (start < end).then_some(Self {
            start_micros: start,
            end_micros: end,
        })
    }

    pub fn duration_minutes_floor(&self) -> u32 {
        let micros = self.end_micros.saturating_sub(self.start_micros);
        u32::try_from(micros / MICROS_PER_MINUTE).unwrap_or(u32::MAX)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum PlannerPresence {
    Home,
    Working,
    Away,
    Sleeping,
    DoNotDisturb,
    Unknown,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct MemberProfile {
    pub member_id: String,
    pub display_name: String,
    pub active: bool,
    pub presence: PlannerPresence,
    /// Concrete windows in which this member has agreed to be considered.
    pub availability: Vec<TimeWindow>,
    /// Maximum new + already-planned work in the planning horizon.
    pub max_planned_minutes: u32,
    /// Work already allocated before this planning run.
    pub existing_planned_minutes: u32,
    /// Optional verified historical care effort. This is not the legacy digest's
    /// one-hour-per-task estimate unless an adapter explicitly chooses to supply it.
    pub verified_recent_care_minutes: u32,
    /// Explicitly declared semantic capabilities, e.g. `care.pet.walk`.
    pub capabilities: BTreeSet<String>,
}

impl MemberProfile {
    fn validate(&self, path: &str) -> Result<(), PlannerError> {
        require_nonempty(&format!("{path}.member_id"), &self.member_id)?;
        require_nonempty(&format!("{path}.display_name"), &self.display_name)?;
        if self.existing_planned_minutes > self.max_planned_minutes {
            return Err(PlannerError::InvalidInput(format!(
                "{path}: existing_planned_minutes exceeds max_planned_minutes"
            )));
        }
        for (index, window) in self.availability.iter().enumerate() {
            window.validate(&format!("{path}.availability[{index}]"))?;
        }
        for capability in &self.capabilities {
            require_nonempty(&format!("{path}.capabilities"), capability)?;
        }
        Ok(())
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum AssignmentPolicy {
    /// Keep the current assignee; if they cannot take the occurrence, ask a human.
    Fixed,
    /// Prefer the current assignee when projected burden remains reasonably close.
    PreferCurrent,
    /// Select from all eligible members deterministically.
    Pool,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct CareOccurrence {
    pub occurrence_id: String,
    /// Durable source schedule/template reference when available.
    pub source_schedule_ref: String,
    pub title: String,
    pub category: String,
    pub current_assignee: Option<String>,
    pub assignment_policy: AssignmentPolicy,
    /// Explicit estimate. `None` is not silently treated as one hour.
    pub estimated_minutes: Option<u32>,
    /// Concrete scheduling window for this occurrence. Free-form Care/Rhythm
    /// schedule strings must be normalized before reaching this crate.
    pub window: Option<TimeWindow>,
    pub required_capabilities: BTreeSet<String>,
    /// `None` means all members passing other constraints may be considered.
    pub eligible_members: Option<BTreeSet<String>>,
    /// 0-100; larger values are planned first.
    pub priority: u8,
}

impl CareOccurrence {
    fn validate(&self, path: &str) -> Result<(), PlannerError> {
        require_nonempty(&format!("{path}.occurrence_id"), &self.occurrence_id)?;
        require_nonempty(
            &format!("{path}.source_schedule_ref"),
            &self.source_schedule_ref,
        )?;
        require_nonempty(&format!("{path}.title"), &self.title)?;
        require_nonempty(&format!("{path}.category"), &self.category)?;
        if self.priority > 100 {
            return Err(PlannerError::InvalidInput(format!(
                "{path}.priority must be <= 100"
            )));
        }
        if self.estimated_minutes == Some(0) {
            return Err(PlannerError::InvalidInput(format!(
                "{path}.estimated_minutes must be positive when specified"
            )));
        }
        if let Some(window) = &self.window {
            window.validate(&format!("{path}.window"))?;
        }
        if self.assignment_policy == AssignmentPolicy::Fixed && self.current_assignee.is_none() {
            return Err(PlannerError::InvalidInput(format!(
                "{path}: Fixed assignment requires current_assignee"
            )));
        }
        for capability in &self.required_capabilities {
            require_nonempty(&format!("{path}.required_capabilities"), capability)?;
        }
        if let Some(eligible) = &self.eligible_members {
            for member in eligible {
                require_nonempty(&format!("{path}.eligible_members"), member)?;
            }
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct RhythmCommitment {
    pub rhythm_ref: String,
    pub name: String,
    pub participants: BTreeSet<String>,
    pub window: TimeWindow,
    /// Hard commitments block care scheduling. Soft rhythms are preserved as
    /// context for a later preference-aware planner but do not block AUTO-005.
    pub hard: bool,
}

impl RhythmCommitment {
    fn validate(&self, path: &str) -> Result<(), PlannerError> {
        require_nonempty(&format!("{path}.rhythm_ref"), &self.rhythm_ref)?;
        require_nonempty(&format!("{path}.name"), &self.name)?;
        self.window.validate(&format!("{path}.window"))?;
        for participant in &self.participants {
            require_nonempty(&format!("{path}.participants"), participant)?;
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct PlanningPolicy {
    /// Search granularity. Default 15 minutes.
    pub slot_minutes: u32,
    /// Historical verified effort contributes this fraction to projected burden.
    pub recent_history_weight_bp: u32,
    /// Bias retained assignments by adding this many basis points to a different
    /// candidate's projected workload in `PreferCurrent` mode.
    pub reassignment_bias_bp: u32,
    /// When false (default), unknown presence cannot be used for automatic planning.
    pub allow_unknown_presence: bool,
}

impl Default for PlanningPolicy {
    fn default() -> Self {
        Self {
            slot_minutes: 15,
            recent_history_weight_bp: 2_500,
            reassignment_bias_bp: 400,
            allow_unknown_presence: false,
        }
    }
}

impl PlanningPolicy {
    fn validate(&self) -> Result<(), PlannerError> {
        if self.slot_minutes == 0 {
            return Err(PlannerError::InvalidInput(
                "policy.slot_minutes must be positive".into(),
            ));
        }
        if self.recent_history_weight_bp > BASIS_POINTS_MAX {
            return Err(PlannerError::InvalidInput(
                "policy.recent_history_weight_bp must be <= 10000".into(),
            ));
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct PlanningInput {
    pub schema_version: u16,
    pub hearth_id: String,
    pub horizon: TimeWindow,
    pub members: Vec<MemberProfile>,
    pub care: Vec<CareOccurrence>,
    pub rhythms: Vec<RhythmCommitment>,
    pub policy: PlanningPolicy,
}

impl PlanningInput {
    pub fn validate(&self) -> Result<(), PlannerError> {
        if self.schema_version != PLANNER_SCHEMA_VERSION {
            return Err(PlannerError::InvalidInput(format!(
                "unsupported planner schema version {}; expected {}",
                self.schema_version, PLANNER_SCHEMA_VERSION
            )));
        }
        require_nonempty("hearth_id", &self.hearth_id)?;
        self.horizon.validate("horizon")?;
        self.policy.validate()?;

        let mut member_ids = BTreeSet::new();
        for (index, member) in self.members.iter().enumerate() {
            member.validate(&format!("members[{index}]"))?;
            if !member_ids.insert(member.member_id.clone()) {
                return Err(PlannerError::InvalidInput(format!(
                    "duplicate member_id {}",
                    member.member_id
                )));
            }
        }

        let mut occurrence_ids = BTreeSet::new();
        for (index, occurrence) in self.care.iter().enumerate() {
            occurrence.validate(&format!("care[{index}]"))?;
            if !occurrence_ids.insert(occurrence.occurrence_id.clone()) {
                return Err(PlannerError::InvalidInput(format!(
                    "duplicate occurrence_id {}",
                    occurrence.occurrence_id
                )));
            }
        }

        for (index, rhythm) in self.rhythms.iter().enumerate() {
            rhythm.validate(&format!("rhythms[{index}]"))?;
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum PlanIssueKind {
    MissingEffortEstimate,
    MissingSchedulingWindow,
    CurrentAssigneeUnavailable,
    NoEligibleMember,
    NoTimeSlot,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct PlanIssue {
    pub occurrence_id: String,
    pub kind: PlanIssueKind,
    pub detail: String,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct AssignmentRationale {
    pub kept_current_assignee: bool,
    pub load_before_bp: u32,
    pub load_after_bp: u32,
    pub explicit_estimated_minutes: u32,
    pub history_component_minutes: u32,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct WorkAssignment {
    pub occurrence_id: String,
    pub source_schedule_ref: String,
    pub title: String,
    pub member_id: String,
    pub start_micros: i64,
    pub end_micros: i64,
    pub rationale: AssignmentRationale,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct MemberLoad {
    pub member_id: String,
    pub existing_planned_minutes: u32,
    pub newly_assigned_minutes: u32,
    pub verified_recent_care_minutes: u32,
    pub capacity_minutes: u32,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct FairnessSummary {
    pub member_loads: Vec<MemberLoad>,
    /// Difference between largest and smallest new known-effort allocation.
    /// This is a workload diagnostic, not a score of people.
    pub new_assignment_spread_minutes: u32,
    pub unknown_effort_occurrences: u32,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct HouseholdWorkPlan {
    pub hearth_id: String,
    pub horizon: TimeWindow,
    pub assignments: Vec<WorkAssignment>,
    pub issues: Vec<PlanIssue>,
    pub fairness: FairnessSummary,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum PlannerError {
    InvalidInput(String),
    NoRecommendations,
}

impl std::fmt::Display for PlannerError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InvalidInput(message) => write!(f, "invalid planning input: {message}"),
            Self::NoRecommendations => write!(f, "work plan contains no recommendations"),
        }
    }
}

impl std::error::Error for PlannerError {}

fn require_nonempty(path: &str, value: &str) -> Result<(), PlannerError> {
    if value.trim().is_empty() {
        Err(PlannerError::InvalidInput(format!(
            "{path} must not be empty"
        )))
    } else {
        Ok(())
    }
}

#[derive(Debug, Clone)]
struct MemberState {
    profile: MemberProfile,
    newly_assigned_minutes: u32,
    blocked: Vec<TimeWindow>,
}

impl MemberState {
    fn history_component(&self, policy: &PlanningPolicy) -> u32 {
        ((self.profile.verified_recent_care_minutes as u64
            * policy.recent_history_weight_bp as u64)
            / BASIS_POINTS_MAX as u64)
            .min(u32::MAX as u64) as u32
    }

    fn burden_minutes(&self, policy: &PlanningPolicy) -> u32 {
        self.profile
            .existing_planned_minutes
            .saturating_add(self.newly_assigned_minutes)
            .saturating_add(self.history_component(policy))
    }

    fn load_bp_with(&self, extra_minutes: u32, policy: &PlanningPolicy) -> u32 {
        if self.profile.max_planned_minutes == 0 {
            return u32::MAX;
        }
        let projected = self.burden_minutes(policy).saturating_add(extra_minutes);
        ((projected as u64 * BASIS_POINTS_MAX as u64)
            / self.profile.max_planned_minutes as u64)
            .min(u32::MAX as u64) as u32
    }
}

#[derive(Debug, Clone)]
struct Candidate {
    member_id: String,
    slot: TimeWindow,
    adjusted_load_bp: u32,
    load_before_bp: u32,
    load_after_bp: u32,
    projected_minutes: u32,
    kept_current: bool,
    history_component_minutes: u32,
}

fn presence_allows(member: &MemberProfile, policy: &PlanningPolicy) -> bool {
    match member.presence {
        PlannerPresence::Home | PlannerPresence::Working => true,
        PlannerPresence::Unknown => policy.allow_unknown_presence,
        PlannerPresence::Away | PlannerPresence::Sleeping | PlannerPresence::DoNotDisturb => false,
    }
}

fn member_is_eligible(member: &MemberProfile, occurrence: &CareOccurrence) -> bool {
    if !member.active {
        return false;
    }
    if let Some(eligible) = &occurrence.eligible_members
        && !eligible.contains(&member.member_id)
    {
        return false;
    }
    occurrence
        .required_capabilities
        .iter()
        .all(|capability| member.capabilities.contains(capability))
}

fn find_slot(
    member: &MemberState,
    requested: &TimeWindow,
    duration_minutes: u32,
    slot_minutes: u32,
) -> Option<TimeWindow> {
    let duration = i64::from(duration_minutes).saturating_mul(MICROS_PER_MINUTE);
    let step = i64::from(slot_minutes).saturating_mul(MICROS_PER_MINUTE);
    if duration <= 0 || step <= 0 {
        return None;
    }

    let mut availability = member.profile.availability.clone();
    availability.sort_by_key(|window| (window.start_micros, window.end_micros));

    for available in availability {
        let Some(intersection) = available.intersection(requested) else {
            continue;
        };
        let mut start = intersection.start_micros;
        loop {
            let end = start.saturating_add(duration);
            if end > intersection.end_micros {
                break;
            }
            let candidate = TimeWindow {
                start_micros: start,
                end_micros: end,
            };
            if member.blocked.iter().all(|blocked| !candidate.overlaps(blocked)) {
                return Some(candidate);
            }
            let next = start.saturating_add(step);
            if next <= start {
                break;
            }
            start = next;
        }
    }
    None
}

pub fn plan_household_work(input: &PlanningInput) -> Result<HouseholdWorkPlan, PlannerError> {
    input.validate()?;

    let mut states: BTreeMap<String, MemberState> = input
        .members
        .iter()
        .cloned()
        .map(|profile| {
            let member_id = profile.member_id.clone();
            (
                member_id,
                MemberState {
                    profile,
                    newly_assigned_minutes: 0,
                    blocked: Vec::new(),
                },
            )
        })
        .collect();

    for rhythm in input.rhythms.iter().filter(|rhythm| rhythm.hard) {
        let Some(window) = rhythm.window.intersection(&input.horizon) else {
            continue;
        };
        for participant in &rhythm.participants {
            if let Some(state) = states.get_mut(participant) {
                state.blocked.push(window.clone());
            }
        }
    }

    let mut ordered: Vec<&CareOccurrence> = input.care.iter().collect();
    ordered.sort_by(|left, right| {
        let left_end = left.window.as_ref().map_or(i64::MAX, |window| window.end_micros);
        let right_end = right.window.as_ref().map_or(i64::MAX, |window| window.end_micros);
        right
            .priority
            .cmp(&left.priority)
            .then_with(|| left_end.cmp(&right_end))
            .then_with(|| left.occurrence_id.cmp(&right.occurrence_id))
    });

    let mut assignments = Vec::new();
    let mut issues = Vec::new();
    let mut unknown_effort_occurrences = 0_u32;

    for occurrence in ordered {
        let Some(minutes) = occurrence.estimated_minutes else {
            unknown_effort_occurrences = unknown_effort_occurrences.saturating_add(1);
            issues.push(PlanIssue {
                occurrence_id: occurrence.occurrence_id.clone(),
                kind: PlanIssueKind::MissingEffortEstimate,
                detail: "explicit estimated_minutes is required; planner will not assume one hour"
                    .into(),
            });
            continue;
        };

        let Some(raw_window) = &occurrence.window else {
            issues.push(PlanIssue {
                occurrence_id: occurrence.occurrence_id.clone(),
                kind: PlanIssueKind::MissingSchedulingWindow,
                detail: "a concrete occurrence window is required; free-form schedule text is not parsed"
                    .into(),
            });
            continue;
        };
        let Some(window) = raw_window.intersection(&input.horizon) else {
            issues.push(PlanIssue {
                occurrence_id: occurrence.occurrence_id.clone(),
                kind: PlanIssueKind::NoTimeSlot,
                detail: "occurrence window does not intersect the planning horizon".into(),
            });
            continue;
        };

        let member_ids: Vec<String> = match occurrence.assignment_policy {
            AssignmentPolicy::Fixed => occurrence.current_assignee.iter().cloned().collect(),
            AssignmentPolicy::PreferCurrent | AssignmentPolicy::Pool => {
                states.keys().cloned().collect()
            }
        };

        let mut candidates = Vec::new();
        let mut saw_eligible_member = false;
        let mut saw_capacity_or_slot_failure = false;

        for member_id in member_ids {
            let Some(state) = states.get(&member_id) else {
                continue;
            };
            if !member_is_eligible(&state.profile, occurrence) {
                continue;
            }
            if !presence_allows(&state.profile, &input.policy) {
                continue;
            }
            saw_eligible_member = true;

            let total_after = state
                .profile
                .existing_planned_minutes
                .saturating_add(state.newly_assigned_minutes)
                .saturating_add(minutes);
            if total_after > state.profile.max_planned_minutes {
                saw_capacity_or_slot_failure = true;
                continue;
            }

            let Some(slot) = find_slot(state, &window, minutes, input.policy.slot_minutes) else {
                saw_capacity_or_slot_failure = true;
                continue;
            };

            let load_before_bp = state.load_bp_with(0, &input.policy);
            let load_after_bp = state.load_bp_with(minutes, &input.policy);
            let kept_current = occurrence.current_assignee.as_ref() == Some(&member_id);
            let reassignment_bias = if occurrence.assignment_policy == AssignmentPolicy::PreferCurrent
                && occurrence.current_assignee.is_some()
                && !kept_current
            {
                input.policy.reassignment_bias_bp
            } else {
                0
            };
            candidates.push(Candidate {
                member_id: member_id.clone(),
                slot,
                adjusted_load_bp: load_after_bp.saturating_add(reassignment_bias),
                load_before_bp,
                load_after_bp,
                projected_minutes: state
                    .profile
                    .existing_planned_minutes
                    .saturating_add(state.newly_assigned_minutes)
                    .saturating_add(minutes),
                kept_current,
                history_component_minutes: state.history_component(&input.policy),
            });
        }

        candidates.sort_by(|left, right| {
            left.adjusted_load_bp
                .cmp(&right.adjusted_load_bp)
                .then_with(|| left.projected_minutes.cmp(&right.projected_minutes))
                .then_with(|| left.slot.start_micros.cmp(&right.slot.start_micros))
                .then_with(|| left.member_id.cmp(&right.member_id))
        });

        let Some(chosen) = candidates.into_iter().next() else {
            let (kind, detail) = if occurrence.assignment_policy == AssignmentPolicy::Fixed {
                (
                    PlanIssueKind::CurrentAssigneeUnavailable,
                    "fixed assignee cannot be scheduled under current presence, capability, capacity, or time constraints",
                )
            } else if !saw_eligible_member {
                (
                    PlanIssueKind::NoEligibleMember,
                    "no active member satisfies explicit presence, eligibility, and capability constraints",
                )
            } else if saw_capacity_or_slot_failure {
                (
                    PlanIssueKind::NoTimeSlot,
                    "eligible members exist, but no member has remaining capacity and a non-conflicting explicit time slot",
                )
            } else {
                (
                    PlanIssueKind::NoEligibleMember,
                    "no deterministic assignment candidate remained",
                )
            };
            issues.push(PlanIssue {
                occurrence_id: occurrence.occurrence_id.clone(),
                kind,
                detail: detail.into(),
            });
            continue;
        };

        let state = states
            .get_mut(&chosen.member_id)
            .expect("candidate member state must exist");
        state.newly_assigned_minutes = state.newly_assigned_minutes.saturating_add(minutes);
        state.blocked.push(chosen.slot.clone());

        assignments.push(WorkAssignment {
            occurrence_id: occurrence.occurrence_id.clone(),
            source_schedule_ref: occurrence.source_schedule_ref.clone(),
            title: occurrence.title.clone(),
            member_id: chosen.member_id,
            start_micros: chosen.slot.start_micros,
            end_micros: chosen.slot.end_micros,
            rationale: AssignmentRationale {
                kept_current_assignee: chosen.kept_current,
                load_before_bp: chosen.load_before_bp,
                load_after_bp: chosen.load_after_bp,
                explicit_estimated_minutes: minutes,
                history_component_minutes: chosen.history_component_minutes,
            },
        });
    }

    let mut member_loads: Vec<_> = states
        .values()
        .map(|state| MemberLoad {
            member_id: state.profile.member_id.clone(),
            existing_planned_minutes: state.profile.existing_planned_minutes,
            newly_assigned_minutes: state.newly_assigned_minutes,
            verified_recent_care_minutes: state.profile.verified_recent_care_minutes,
            capacity_minutes: state.profile.max_planned_minutes,
        })
        .collect();
    member_loads.sort_by(|left, right| left.member_id.cmp(&right.member_id));

    let min_new = member_loads
        .iter()
        .map(|load| load.newly_assigned_minutes)
        .min()
        .unwrap_or(0);
    let max_new = member_loads
        .iter()
        .map(|load| load.newly_assigned_minutes)
        .max()
        .unwrap_or(0);

    Ok(HouseholdWorkPlan {
        hearth_id: input.hearth_id.clone(),
        horizon: input.horizon.clone(),
        assignments,
        issues,
        fairness: FairnessSummary {
            member_loads,
            new_assignment_spread_minutes: max_new.saturating_sub(min_new),
            unknown_effort_occurrences,
        },
    })
}

/// Compile planner assignments into A2 recommendations. The resulting plan does
/// not mutate Care/Rhythm state and does not require execution authority.
pub fn compile_recommendation_plan(
    work_plan: &HouseholdWorkPlan,
    intent_id: IntentId,
    plan_id: PlanId,
    generated_at_micros: i64,
) -> Result<AutomationPlan, PlannerError> {
    if work_plan.assignments.is_empty() {
        return Err(PlannerError::NoRecommendations);
    }

    let steps = work_plan
        .assignments
        .iter()
        .map(|assignment| {
            let mut arguments = BTreeMap::new();
            arguments.insert(
                "member_id".into(),
                AutomationValue::Text(assignment.member_id.clone()),
            );
            arguments.insert(
                "source_schedule_ref".into(),
                AutomationValue::Text(assignment.source_schedule_ref.clone()),
            );
            arguments.insert(
                "title".into(),
                AutomationValue::Text(assignment.title.clone()),
            );
            arguments.insert(
                "start_micros".into(),
                AutomationValue::Signed(assignment.start_micros),
            );
            arguments.insert(
                "end_micros".into(),
                AutomationValue::Signed(assignment.end_micros),
            );

            PlanStep {
                id: StepId::new(format!("recommend:{}", assignment.occurrence_id)),
                depends_on: Vec::new(),
                preconditions: hearth_automation_types::ConditionExpr::Always,
                action: ActionSpec {
                    capability: "home.care.recommend".into(),
                    target: EntityRef {
                        kind: "care_occurrence".into(),
                        id: assignment.occurrence_id.clone(),
                    },
                    operation: "suggest_assignment".into(),
                    arguments,
                },
                consequence: ConsequenceClass::Recommend,
                reversibility: Reversibility::Reversible,
                authority: AuthorityRequirement::None,
                outcome: OutcomePolicy {
                    expectations: Vec::new(),
                    verify_within_ms: 0,
                    evidence: EvidenceRequirement::default(),
                    on_unverified: UnverifiedDisposition::StopAndNotify,
                },
                timeout_ms: 1_000,
                retry: RetryPolicy::none(),
                compensation: None,
            }
        })
        .collect();

    let plan = AutomationPlan {
        schema_version: AUTOMATION_SCHEMA_VERSION,
        id: plan_id,
        intent_id,
        generated_at_micros,
        steps,
    };
    plan.validate()
        .map_err(|error| PlannerError::InvalidInput(error.to_string()))?;
    Ok(plan)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn window(start_minute: i64, end_minute: i64) -> TimeWindow {
        TimeWindow {
            start_micros: start_minute * MICROS_PER_MINUTE,
            end_micros: end_minute * MICROS_PER_MINUTE,
        }
    }

    fn member(id: &str) -> MemberProfile {
        MemberProfile {
            member_id: id.into(),
            display_name: id.into(),
            active: true,
            presence: PlannerPresence::Home,
            availability: vec![window(0, 600)],
            max_planned_minutes: 600,
            existing_planned_minutes: 0,
            verified_recent_care_minutes: 0,
            capabilities: BTreeSet::new(),
        }
    }

    fn occurrence(id: &str, minutes: Option<u32>) -> CareOccurrence {
        CareOccurrence {
            occurrence_id: id.into(),
            source_schedule_ref: format!("schedule-{id}"),
            title: id.into(),
            category: "chore".into(),
            current_assignee: None,
            assignment_policy: AssignmentPolicy::Pool,
            estimated_minutes: minutes,
            window: Some(window(0, 600)),
            required_capabilities: BTreeSet::new(),
            eligible_members: None,
            priority: 50,
        }
    }

    fn input(members: Vec<MemberProfile>, care: Vec<CareOccurrence>) -> PlanningInput {
        PlanningInput {
            schema_version: PLANNER_SCHEMA_VERSION,
            hearth_id: "hearth-1".into(),
            horizon: window(0, 600),
            members,
            care,
            rhythms: Vec::new(),
            policy: PlanningPolicy::default(),
        }
    }

    #[test]
    fn missing_effort_is_never_guessed() {
        let plan = plan_household_work(&input(vec![member("a")], vec![occurrence("x", None)]))
            .unwrap();
        assert!(plan.assignments.is_empty());
        assert_eq!(plan.fairness.unknown_effort_occurrences, 1);
        assert_eq!(plan.issues[0].kind, PlanIssueKind::MissingEffortEstimate);
    }

    #[test]
    fn stable_tie_break_uses_member_id() {
        let plan = plan_household_work(&input(
            vec![member("b"), member("a")],
            vec![occurrence("x", Some(30))],
        ))
        .unwrap();
        assert_eq!(plan.assignments[0].member_id, "a");
    }

    #[test]
    fn balances_known_effort_across_equal_members() {
        let plan = plan_household_work(&input(
            vec![member("a"), member("b")],
            vec![occurrence("x", Some(60)), occurrence("y", Some(60))],
        ))
        .unwrap();
        let assigned: BTreeSet<_> = plan
            .assignments
            .iter()
            .map(|assignment| assignment.member_id.as_str())
            .collect();
        assert_eq!(assigned, BTreeSet::from(["a", "b"]));
        assert_eq!(plan.fairness.new_assignment_spread_minutes, 0);
    }

    #[test]
    fn hard_rhythm_blocks_time() {
        let mut planning = input(vec![member("a")], vec![occurrence("x", Some(60))]);
        planning.rhythms.push(RhythmCommitment {
            rhythm_ref: "rhythm-1".into(),
            name: "breakfast".into(),
            participants: BTreeSet::from(["a".to_string()]),
            window: window(0, 90),
            hard: true,
        });
        let plan = plan_household_work(&planning).unwrap();
        assert_eq!(plan.assignments[0].start_micros, 90 * MICROS_PER_MINUTE);
    }

    #[test]
    fn away_member_is_not_auto_assigned() {
        let mut away = member("a");
        away.presence = PlannerPresence::Away;
        let plan = plan_household_work(&input(vec![away], vec![occurrence("x", Some(30))]))
            .unwrap();
        assert!(plan.assignments.is_empty());
        assert_eq!(plan.issues[0].kind, PlanIssueKind::NoEligibleMember);
    }

    #[test]
    fn fixed_unavailable_assignee_needs_human() {
        let mut fixed = occurrence("x", Some(30));
        fixed.assignment_policy = AssignmentPolicy::Fixed;
        fixed.current_assignee = Some("a".into());
        let mut away = member("a");
        away.presence = PlannerPresence::Away;
        let plan = plan_household_work(&input(vec![away, member("b")], vec![fixed])).unwrap();
        assert!(plan.assignments.is_empty());
        assert_eq!(
            plan.issues[0].kind,
            PlanIssueKind::CurrentAssigneeUnavailable
        );
    }

    #[test]
    fn recommendation_plan_is_a2_and_authority_free() {
        let work = plan_household_work(&input(
            vec![member("a")],
            vec![occurrence("x", Some(30))],
        ))
        .unwrap();
        let plan = compile_recommendation_plan(
            &work,
            IntentId::from("intent"),
            PlanId::from("plan"),
            42,
        )
        .unwrap();
        assert_eq!(plan.steps.len(), 1);
        assert_eq!(plan.steps[0].consequence, ConsequenceClass::Recommend);
        assert_eq!(plan.steps[0].authority, AuthorityRequirement::None);
    }
}
