// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Digest-v4-qualified history adapter for the Hearth household planner.

use hearth_care_digest_v4::CareDigestV4;
use hearth_household_planner::{
    plan_household_work, CareOccurrence, HouseholdWorkPlan, MemberProfile, PlannerError,
    PlanningInput, PlanningPolicy, RhythmCommitment, TimeWindow, PLANNER_SCHEMA_VERSION,
};
use hearth_planner_evidence::MemberPlanningFacts;
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};

pub const PLANNER_EVIDENCE_V4_SCHEMA_VERSION: u16 = 1;
const MAX_SOURCE_REF_LEN: usize = 1024;

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct CareHistoryEvidenceV4 {
    schema_version: u16,
    source_ref: String,
    epoch_start_micros: i64,
    epoch_end_micros: i64,
    digest: CareDigestV4,
}

impl CareHistoryEvidenceV4 {
    pub fn new(
        source_ref: String,
        epoch_start_micros: i64,
        epoch_end_micros: i64,
        digest: CareDigestV4,
    ) -> Result<Self, PlannerEvidenceV4Error> {
        let value = Self {
            schema_version: PLANNER_EVIDENCE_V4_SCHEMA_VERSION,
            source_ref,
            epoch_start_micros,
            epoch_end_micros,
            digest,
        };
        value.validate()?;
        Ok(value)
    }

    pub fn digest(&self) -> &CareDigestV4 {
        &self.digest
    }

    fn validate(&self) -> Result<(), PlannerEvidenceV4Error> {
        if self.schema_version != PLANNER_EVIDENCE_V4_SCHEMA_VERSION {
            return Err(PlannerEvidenceV4Error::UnsupportedSchema(self.schema_version));
        }
        if self.source_ref.trim().is_empty() {
            return Err(PlannerEvidenceV4Error::EmptySourceRef);
        }
        if self.source_ref.len() > MAX_SOURCE_REF_LEN {
            return Err(PlannerEvidenceV4Error::SourceRefTooLong(self.source_ref.len()));
        }
        if self.epoch_end_micros <= self.epoch_start_micros {
            return Err(PlannerEvidenceV4Error::InvalidHistoryWindow);
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct PlanningInputV4 {
    pub schema_version: u16,
    pub hearth_id: String,
    pub horizon: TimeWindow,
    pub members: Vec<MemberPlanningFacts>,
    pub care: Vec<CareOccurrence>,
    pub rhythms: Vec<RhythmCommitment>,
    pub policy: PlanningPolicy,
    pub history: CareHistoryEvidenceV4,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct CareHistoryProvenanceV4 {
    pub source_ref: String,
    pub epoch_start_micros: i64,
    pub epoch_end_micros: i64,
    pub qualified_occurrence_count: u64,
    pub qualified_completion_count: u64,
    pub assignment_missing_count: u64,
    pub assignment_stale_count: u64,
    pub assignment_conflict_count: u64,
    pub time_missing_count: u64,
    pub time_stale_count: u64,
    pub time_conflict_count: u64,
    pub completion_missing_count: u64,
    pub completion_occurrence_unqualified_count: u64,
    pub completion_conflict_count: u64,
    pub completion_performer_mismatch_count: u64,
    pub completion_third_party_attestation_count: u64,
    pub qualified_unknown_duration_count: u64,
    pub unmatched_history_member_ids: Vec<String>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct PlanningResultV4 {
    pub plan: HouseholdWorkPlan,
    pub history_provenance: CareHistoryProvenanceV4,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum PlannerEvidenceV4Error {
    UnsupportedSchema(u16),
    EmptySourceRef,
    SourceRefTooLong(usize),
    InvalidHistoryWindow,
    HistoryMinutesOverflow { member_id: String, minutes: u64 },
    Planner(PlannerError),
}

impl std::fmt::Display for PlannerEvidenceV4Error {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{self:?}")
    }
}
impl std::error::Error for PlannerEvidenceV4Error {}

/// Plan household work using only Digest-v4-qualified actual effort.
pub fn plan_household_work_v4(
    input: &PlanningInputV4,
) -> Result<PlanningResultV4, PlannerEvidenceV4Error> {
    if input.schema_version != PLANNER_EVIDENCE_V4_SCHEMA_VERSION {
        return Err(PlannerEvidenceV4Error::UnsupportedSchema(input.schema_version));
    }
    input.history.validate()?;

    let history_minutes = qualified_history_minutes_v4(&input.history)?;
    let current_members: BTreeSet<String> = input
        .members
        .iter()
        .map(|member| member.member_id.clone())
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
    let plan = plan_household_work(&planner_input).map_err(PlannerEvidenceV4Error::Planner)?;

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
        .fold(0_u64, |total, stats| {
            total.saturating_add(stats.unknown_actual_duration_count)
        });
    let digest = &input.history.digest;

    Ok(PlanningResultV4 {
        plan,
        history_provenance: CareHistoryProvenanceV4 {
            source_ref: input.history.source_ref.clone(),
            epoch_start_micros: input.history.epoch_start_micros,
            epoch_end_micros: input.history.epoch_end_micros,
            qualified_occurrence_count: digest.qualified_occurrence_count,
            qualified_completion_count: digest.qualified_completion_count,
            assignment_missing_count: digest.assignment_missing_count,
            assignment_stale_count: digest.assignment_stale_count,
            assignment_conflict_count: digest.assignment_conflict_count,
            time_missing_count: digest.time_missing_count,
            time_stale_count: digest.time_stale_count,
            time_conflict_count: digest.time_conflict_count,
            completion_missing_count: digest.completion_missing_count,
            completion_occurrence_unqualified_count: digest.completion_occurrence_unqualified_count,
            completion_conflict_count: digest.completion_conflict_count,
            completion_performer_mismatch_count: digest.completion_performer_mismatch_count,
            completion_third_party_attestation_count: digest
                .completion_third_party_attestation_count,
            qualified_unknown_duration_count,
            unmatched_history_member_ids,
        },
    })
}

pub fn qualified_history_minutes_v4(
    evidence: &CareHistoryEvidenceV4,
) -> Result<BTreeMap<String, u32>, PlannerEvidenceV4Error> {
    evidence.validate()?;
    evidence
        .digest
        .by_member
        .iter()
        .map(|(member, stats)| {
            let minutes = u32::try_from(stats.known_actual_minutes).map_err(|_| {
                PlannerEvidenceV4Error::HistoryMinutesOverflow {
                    member_id: member.0.clone(),
                    minutes: stats.known_actual_minutes,
                }
            })?;
            Ok((member.0.clone(), minutes))
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use hearth_care_digest_v4::{CareDigestV4, CareDigestV4Member};
    use hearth_care_ledger_contract::MemberId;
    use hearth_household_planner::{AssignmentPolicy, PlannerPresence};

    fn window(start: i64, end: i64) -> TimeWindow {
        TimeWindow { start_micros: start, end_micros: end }
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

    fn digest(minutes: u64) -> CareDigestV4 {
        let mut digest = CareDigestV4::default();
        digest.by_member.insert(
            MemberId("alex".into()),
            CareDigestV4Member {
                tasks_completed: 1,
                known_actual_minutes: minutes,
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
    fn v4_history_reaches_planner_without_caller_supplied_minutes() {
        let history = CareHistoryEvidenceV4::new("digest:v4".into(), 0, 10, digest(90)).unwrap();
        let input = PlanningInputV4 {
            schema_version: PLANNER_EVIDENCE_V4_SCHEMA_VERSION,
            hearth_id: "hearth:1".into(),
            horizon: window(0, 600_000_000),
            members: vec![member("alex"), member("mira")],
            care: vec![care()],
            rhythms: vec![],
            policy: PlanningPolicy::default(),
            history,
        };
        let result = plan_household_work_v4(&input).unwrap();
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
    fn history_overflow_fails_instead_of_truncating() {
        let history = CareHistoryEvidenceV4::new(
            "digest:v4".into(),
            0,
            10,
            digest(u64::from(u32::MAX) + 1),
        )
        .unwrap();
        assert!(matches!(
            qualified_history_minutes_v4(&history),
            Err(PlannerEvidenceV4Error::HistoryMinutesOverflow { .. })
        ));
    }

    #[test]
    fn unknown_duration_contributes_zero_known_minutes() {
        let mut digest = digest(0);
        digest.by_member.get_mut(&MemberId("alex".into())).unwrap().unknown_actual_duration_count = 1;
        let history = CareHistoryEvidenceV4::new("digest:v4".into(), 0, 10, digest).unwrap();
        assert_eq!(qualified_history_minutes_v4(&history).unwrap()["alex"], 0);
    }
}
