// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Evidence-qualified history adapter for the Hearth household planner.
//!
//! Planner v1 keeps its direct integer history input for compatibility. This
//! crate defines a v2 boundary where member facts contain no history field;
//! historical effort is derived only from CareDigestV3-qualified actual work.

use hearth_care_ledger_contract::{CareDigestV3, MemberId};
use hearth_household_planner::{
    plan_household_work, CareOccurrence, HouseholdWorkPlan, MemberProfile, PlannerError,
    PlannerPresence, PlanningInput, PlanningPolicy, RhythmCommitment, TimeWindow,
    PLANNER_SCHEMA_VERSION,
};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};

pub const PLANNER_EVIDENCE_SCHEMA_VERSION: u16 = 1;
const MAX_SOURCE_REF_LEN: usize = 1024;

/// Current member facts. Historical work is intentionally absent from this type.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct MemberPlanningFacts {
    pub member_id: String,
    pub display_name: String,
    pub active: bool,
    pub presence: PlannerPresence,
    pub availability: Vec<TimeWindow>,
    pub max_planned_minutes: u32,
    pub existing_planned_minutes: u32,
    pub capabilities: BTreeSet<String>,
}

/// Evidence capsule for one historical window.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct CareHistoryEvidence {
    schema_version: u16,
    source_ref: String,
    epoch_start_micros: i64,
    epoch_end_micros: i64,
    digest: CareDigestV3,
}

impl CareHistoryEvidence {
    pub fn new(
        source_ref: String,
        epoch_start_micros: i64,
        epoch_end_micros: i64,
        digest: CareDigestV3,
    ) -> Result<Self, PlannerEvidenceError> {
        let value = Self {
            schema_version: PLANNER_EVIDENCE_SCHEMA_VERSION,
            source_ref,
            epoch_start_micros,
            epoch_end_micros,
            digest,
        };
        value.validate()?;
        Ok(value)
    }

    pub fn source_ref(&self) -> &str {
        &self.source_ref
    }

    pub fn epoch_start_micros(&self) -> i64 {
        self.epoch_start_micros
    }

    pub fn epoch_end_micros(&self) -> i64 {
        self.epoch_end_micros
    }

    pub fn digest(&self) -> &CareDigestV3 {
        &self.digest
    }

    fn validate(&self) -> Result<(), PlannerEvidenceError> {
        if self.schema_version != PLANNER_EVIDENCE_SCHEMA_VERSION {
            return Err(PlannerEvidenceError::UnsupportedSchema(self.schema_version));
        }
        if self.source_ref.trim().is_empty() {
            return Err(PlannerEvidenceError::EmptySourceRef);
        }
        if self.source_ref.len() > MAX_SOURCE_REF_LEN {
            return Err(PlannerEvidenceError::SourceRefTooLong(self.source_ref.len()));
        }
        if self.epoch_end_micros <= self.epoch_start_micros {
            return Err(PlannerEvidenceError::InvalidHistoryWindow);
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct PlanningInputV2 {
    pub schema_version: u16,
    pub hearth_id: String,
    pub horizon: TimeWindow,
    pub members: Vec<MemberPlanningFacts>,
    pub care: Vec<CareOccurrence>,
    pub rhythms: Vec<RhythmCommitment>,
    pub policy: PlanningPolicy,
    pub history: CareHistoryEvidence,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct CareHistoryProvenance {
    pub source_ref: String,
    pub epoch_start_micros: i64,
    pub epoch_end_micros: i64,
    pub qualified_occurrence_count: u32,
    pub unbound_occurrence_count: u32,
    pub qualified_completion_count: u32,
    pub unbound_completion_count: u32,
    pub excluded_or_conflicted_completion_count: u32,
    pub qualified_unknown_duration_count: u32,
    /// Historical members no longer present in the current planning set.
    pub unmatched_history_member_ids: Vec<String>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct PlanningResultV2 {
    pub plan: HouseholdWorkPlan,
    pub history_provenance: CareHistoryProvenance,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum PlannerEvidenceError {
    UnsupportedSchema(u16),
    EmptySourceRef,
    SourceRefTooLong(usize),
    InvalidHistoryWindow,
    Planner(PlannerError),
}

impl std::fmt::Display for PlannerEvidenceError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{self:?}")
    }
}

impl std::error::Error for PlannerEvidenceError {}

/// Plan household work using only Digest-v3-qualified actual historical effort.
///
/// Estimated minutes and legacy/unbound completion evidence never become
/// `verified_recent_care_minutes` on this path.
pub fn plan_household_work_v2(
    input: &PlanningInputV2,
) -> Result<PlanningResultV2, PlannerEvidenceError> {
    if input.schema_version != PLANNER_EVIDENCE_SCHEMA_VERSION {
        return Err(PlannerEvidenceError::UnsupportedSchema(input.schema_version));
    }
    input.history.validate()?;

    let current_members: BTreeSet<String> = input
        .members
        .iter()
        .map(|member| member.member_id.clone())
        .collect();

    let history_minutes: BTreeMap<String, u32> = input
        .history
        .digest
        .by_member
        .iter()
        .map(|(member, stats)| (member.0.clone(), stats.known_actual_minutes))
        .collect();

    let members = input
        .members
        .iter()
        .map(|facts| MemberProfile {
            member_id: facts.member_id.clone(),
            display_name: facts.display_name.clone(),
            active: facts.active,
            presence: facts.presence,
            availability: facts.availability.clone(),
            max_planned_minutes: facts.max_planned_minutes,
            existing_planned_minutes: facts.existing_planned_minutes,
            verified_recent_care_minutes: history_minutes
                .get(&facts.member_id)
                .copied()
                .unwrap_or(0),
            capabilities: facts.capabilities.clone(),
        })
        .collect();

    let planner_input = PlanningInput {
        schema_version: PLANNER_SCHEMA_VERSION,
        hearth_id: input.hearth_id.clone(),
        horizon: input.horizon.clone(),
        members,
        care: input.care.clone(),
        rhythms: input.rhythms.clone(),
        policy: input.policy.clone(),
    };

    let plan = plan_household_work(&planner_input).map_err(PlannerEvidenceError::Planner)?;

    let mut unmatched_history_member_ids = input
        .history
        .digest
        .by_member
        .keys()
        .filter(|member| !current_members.contains(&member.0))
        .map(|member| member.0.clone())
        .collect::<Vec<_>>();
    unmatched_history_member_ids.sort();

    let qualified_unknown_duration_count = input
        .history
        .digest
        .by_member
        .values()
        .fold(0_u32, |total, stats| {
            total.saturating_add(stats.unknown_actual_duration_count)
        });
    let excluded_or_conflicted_completion_count = input
        .history
        .digest
        .orphan_completion_count
        .saturating_add(input.history.digest.conflicted_completion_count)
        .saturating_add(input.history.digest.performer_mismatch_completion_count);

    Ok(PlanningResultV2 {
        plan,
        history_provenance: CareHistoryProvenance {
            source_ref: input.history.source_ref.clone(),
            epoch_start_micros: input.history.epoch_start_micros,
            epoch_end_micros: input.history.epoch_end_micros,
            qualified_occurrence_count: input.history.digest.qualified_occurrence_count,
            unbound_occurrence_count: input.history.digest.unbound_occurrence_count,
            qualified_completion_count: input.history.digest.qualified_completion_count,
            unbound_completion_count: input.history.digest.unbound_completion_count,
            excluded_or_conflicted_completion_count,
            qualified_unknown_duration_count,
            unmatched_history_member_ids,
        },
    })
}

/// Convenience helper for callers that need to inspect the history minutes the
/// v2 adapter will inject, without running the planner.
pub fn qualified_history_minutes(
    evidence: &CareHistoryEvidence,
) -> Result<BTreeMap<String, u32>, PlannerEvidenceError> {
    evidence.validate()?;
    Ok(evidence
        .digest
        .by_member
        .iter()
        .map(|(member, stats)| (member.0.clone(), stats.known_actual_minutes))
        .collect())
}

#[cfg(test)]
mod tests {
    use super::*;
    use hearth_care_ledger_contract::{CareDigestV2Member, CareDigestV3};
    use hearth_household_planner::{AssignmentPolicy, PlanningPolicy};

    fn window(start: i64, end: i64) -> TimeWindow {
        TimeWindow {
            start_micros: start,
            end_micros: end,
        }
    }

    fn member(id: &str) -> MemberPlanningFacts {
        MemberPlanningFacts {
            member_id: id.into(),
            display_name: id.into(),
            active: true,
            presence: PlannerPresence::Home,
            availability: vec![window(0, 600_000_000)],
            max_planned_minutes: 600,
            existing_planned_minutes: 0,
            capabilities: BTreeSet::new(),
        }
    }

    fn care() -> CareOccurrence {
        CareOccurrence {
            occurrence_id: "occ:new".into(),
            source_schedule_ref: "schedule:new".into(),
            title: "New task".into(),
            category: "chore".into(),
            current_assignee: None,
            assignment_policy: AssignmentPolicy::Pool,
            estimated_minutes: Some(30),
            window: Some(window(0, 600_000_000)),
            required_capabilities: BTreeSet::new(),
            eligible_members: None,
            priority: 50,
        }
    }

    fn digest() -> CareDigestV3 {
        let mut digest = CareDigestV3::default();
        digest.by_member.insert(
            MemberId("alex".into()),
            CareDigestV2Member {
                tasks_completed: 1,
                known_actual_minutes: 90,
                unknown_actual_duration_count: 0,
                estimated_minutes_for_completed_tasks: 100,
                completed_tasks_without_estimate: 0,
            },
        );
        digest.qualified_occurrence_count = 1;
        digest.qualified_completion_count = 1;
        digest
    }

    #[test]
    fn v2_derives_history_instead_of_accepting_member_supplied_minutes() {
        let evidence = CareHistoryEvidence::new("digest:v3".into(), 0, 10, digest()).unwrap();
        let input = PlanningInputV2 {
            schema_version: PLANNER_EVIDENCE_SCHEMA_VERSION,
            hearth_id: "hearth:1".into(),
            horizon: window(0, 600_000_000),
            members: vec![member("alex"), member("mira")],
            care: vec![care()],
            rhythms: vec![],
            policy: PlanningPolicy::default(),
            history: evidence,
        };
        let result = plan_household_work_v2(&input).unwrap();
        let alex = result
            .plan
            .fairness
            .member_loads
            .iter()
            .find(|item| item.member_id == "alex")
            .unwrap();
        assert_eq!(alex.verified_recent_care_minutes, 90);
    }

    #[test]
    fn unknown_duration_is_not_converted_into_minutes() {
        let mut digest = digest();
        digest.by_member.insert(
            MemberId("mira".into()),
            CareDigestV2Member {
                tasks_completed: 1,
                known_actual_minutes: 0,
                unknown_actual_duration_count: 1,
                estimated_minutes_for_completed_tasks: 60,
                completed_tasks_without_estimate: 0,
            },
        );
        let evidence = CareHistoryEvidence::new("digest:v3".into(), 0, 10, digest).unwrap();
        let history = qualified_history_minutes(&evidence).unwrap();
        assert_eq!(history["mira"], 0);
    }

    #[test]
    fn departed_history_member_remains_visible_but_not_plannable() {
        let mut digest = digest();
        digest.by_member.insert(
            MemberId("departed".into()),
            CareDigestV2Member {
                tasks_completed: 1,
                known_actual_minutes: 40,
                unknown_actual_duration_count: 0,
                estimated_minutes_for_completed_tasks: 40,
                completed_tasks_without_estimate: 0,
            },
        );
        let input = PlanningInputV2 {
            schema_version: PLANNER_EVIDENCE_SCHEMA_VERSION,
            hearth_id: "hearth:1".into(),
            horizon: window(0, 600_000_000),
            members: vec![member("alex"), member("mira")],
            care: vec![care()],
            rhythms: vec![],
            policy: PlanningPolicy::default(),
            history: CareHistoryEvidence::new("digest:v3".into(), 0, 10, digest).unwrap(),
        };
        let result = plan_household_work_v2(&input).unwrap();
        assert_eq!(
            result.history_provenance.unmatched_history_member_ids,
            vec!["departed".to_string()]
        );
    }

    #[test]
    fn history_window_and_source_are_mandatory() {
        assert_eq!(
            CareHistoryEvidence::new("".into(), 0, 10, CareDigestV3::default()),
            Err(PlannerEvidenceError::EmptySourceRef)
        );
        assert_eq!(
            CareHistoryEvidence::new("digest:v3".into(), 10, 10, CareDigestV3::default()),
            Err(PlannerEvidenceError::InvalidHistoryWindow)
        );
    }
}
