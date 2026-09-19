// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Hearth domain state shared across pages.
//!
//! This module constructs the in-memory state. Source/provenance transitions
//! between demo and live data are owned by `crate::hearth_truth`, which clears
//! demo records before establishing any live-backed state.

use crate::mock_data;
use hearth_leptos_types::*;
use leptos::prelude::*;

/// The active Hearth context shared across pages.
#[derive(Clone)]
pub struct HearthCtx {
    pub current_hearth: RwSignal<Option<HearthView>>,
    pub members: RwSignal<Vec<MemberView>>,
    pub my_role: RwSignal<Option<MemberRole>>,
    pub bonds: RwSignal<Vec<BondView>>,
    pub care_schedules: RwSignal<Vec<CareScheduleView>>,
    pub decisions: RwSignal<Vec<DecisionView>>,
    pub votes: RwSignal<Vec<VoteView>>,
    pub gratitude: RwSignal<Vec<GratitudeExpressionView>>,
    pub stories: RwSignal<Vec<StoryView>>,
    pub rhythms: RwSignal<Vec<RhythmView>>,
    pub presence: RwSignal<Vec<PresenceView>>,
    pub emergency_alerts: RwSignal<Vec<EmergencyAlertView>>,
    pub resources: RwSignal<Vec<ResourceView>>,
    pub milestones: RwSignal<Vec<MilestoneView>>,
    pub autonomy_profiles: RwSignal<Vec<AutonomyProfileView>>,
    pub my_agent: RwSignal<String>,
}

/// Initialize Hearth in explicit demo state.
///
/// No live reads happen here. `crate::hearth_truth::provide_hearth_truth`
/// observes the shared Holochain connection and replaces/clears these values
/// before any live data is presented.
pub fn provide_hearth_context() -> HearthCtx {
    let ctx = HearthCtx {
        current_hearth: RwSignal::new(Some(mock_data::mock_hearth())),
        members: RwSignal::new(mock_data::mock_members()),
        my_role: RwSignal::new(Some(MemberRole::Adult)),
        bonds: RwSignal::new(mock_data::mock_bonds()),
        care_schedules: RwSignal::new(mock_data::mock_care_schedules()),
        decisions: RwSignal::new(mock_data::mock_decisions()),
        votes: RwSignal::new(mock_data::mock_votes()),
        gratitude: RwSignal::new(mock_data::mock_gratitude()),
        stories: RwSignal::new(mock_data::mock_stories()),
        rhythms: RwSignal::new(mock_data::mock_rhythms()),
        presence: RwSignal::new(mock_data::mock_presence()),
        emergency_alerts: RwSignal::new(mock_data::mock_emergency_alerts()),
        resources: RwSignal::new(mock_data::mock_resources()),
        milestones: RwSignal::new(mock_data::mock_milestones()),
        autonomy_profiles: RwSignal::new(mock_data::mock_autonomy_profiles()),
        my_agent: RwSignal::new("agent_rowan".into()),
    };

    provide_context(ctx.clone());
    ctx
}

pub fn use_hearth() -> HearthCtx {
    expect_context::<HearthCtx>()
}

/// Look up a member's display name by agent key.
pub fn member_name(members: &[MemberView], agent: &str) -> String {
    members
        .iter()
        .find(|m| m.agent == agent)
        .map(|m| m.display_name.clone())
        .unwrap_or_else(|| {
            if agent.len() > 8 {
                format!("{}...", &agent[..8])
            } else {
                agent.to_string()
            }
        })
}

/// Simple mock timestamp (seconds since epoch, approximate).
pub fn mock_now() -> i64 {
    1_774_934_400 // ~2026-03-30
}
