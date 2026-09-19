// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Hearth context: current hearth, member list, user's role, bonds, and domain data.
//!
//! Runtime provenance is explicit. Demo starts from the simulated household.
//! Live starts empty and hydrates only from an authorized conductor connection;
//! it never substitutes mock records after a connection or decoding failure.

use hearth_leptos_types::*;
use leptos::prelude::*;
use wasm_bindgen_futures::spawn_local;

use crate::mock_data;
use crate::record_bridge::{self, WireRecord};
use crate::runtime_mode::{HearthRuntimeMode, use_runtime_mode};
use mycelix_leptos_core::holochain_provider::{
    ConnectionStatus, HolochainCtx, use_holochain,
};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum HearthDataState {
    Demo,
    AwaitingLive,
    LoadingLive,
    Live,
    Empty,
    Degraded,
}

/// The active hearth context shared across pages.
#[derive(Clone)]
pub struct HearthCtx {
    pub data_state: RwSignal<HearthDataState>,
    /// Homeostasis is intentionally non-authoritative until care + decision
    /// counters are hydrated from live/domain-complete state in HTH-UI-003.
    pub homeostasis_authoritative: RwSignal<bool>,
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

fn demo_context() -> HearthCtx {
    HearthCtx {
        data_state: RwSignal::new(HearthDataState::Demo),
        homeostasis_authoritative: RwSignal::new(false),
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
    }
}

fn live_context() -> HearthCtx {
    HearthCtx {
        data_state: RwSignal::new(HearthDataState::AwaitingLive),
        homeostasis_authoritative: RwSignal::new(false),
        current_hearth: RwSignal::new(None),
        members: RwSignal::new(Vec::new()),
        my_role: RwSignal::new(None),
        bonds: RwSignal::new(Vec::new()),
        care_schedules: RwSignal::new(Vec::new()),
        decisions: RwSignal::new(Vec::new()),
        votes: RwSignal::new(Vec::new()),
        gratitude: RwSignal::new(Vec::new()),
        stories: RwSignal::new(Vec::new()),
        rhythms: RwSignal::new(Vec::new()),
        presence: RwSignal::new(Vec::new()),
        emergency_alerts: RwSignal::new(Vec::new()),
        resources: RwSignal::new(Vec::new()),
        milestones: RwSignal::new(Vec::new()),
        autonomy_profiles: RwSignal::new(Vec::new()),
        my_agent: RwSignal::new(String::new()),
    }
}

/// Initialize Hearth state according to the explicit runtime mode.
pub fn provide_hearth_context() -> HearthCtx {
    let mode = use_runtime_mode();
    let hc = use_holochain();
    let ctx = match mode {
        HearthRuntimeMode::Demo => demo_context(),
        HearthRuntimeMode::Live => live_context(),
    };

    provide_context(ctx.clone());

    if mode == HearthRuntimeMode::Live {
        let load_started = RwSignal::new(false);
        // Every transition that invalidates the current transport increments
        // this epoch. Async hydration is allowed to publish state only while
        // the epoch it started under is still current.
        let connection_epoch = RwSignal::new(0u64);
        let ctx_for_effect = ctx.clone();
        let hc_for_effect = hc.clone();

        Effect::new(move |_| {
            let status = hc_for_effect.status.get();
            let signing_ready = hc_for_effect.zome_call_signing_ready.get();

            match status {
                ConnectionStatus::Connected if signing_ready => {
                    if load_started.get_untracked() {
                        return;
                    }
                    load_started.set(true);
                    ctx_for_effect.data_state.set(HearthDataState::LoadingLive);

                    let expected_epoch = connection_epoch.get_untracked();
                    let ctx_for_load = ctx_for_effect.clone();
                    let hc_for_load = hc_for_effect.clone();
                    spawn_local(async move {
                        try_load_real_data(
                            ctx_for_load,
                            hc_for_load,
                            connection_epoch,
                            expected_epoch,
                        )
                        .await;
                    });
                }
                ConnectionStatus::Disconnected | ConnectionStatus::Reconnecting => {
                    invalidate_live_load(load_started, connection_epoch);
                    ctx_for_effect.data_state.set(HearthDataState::AwaitingLive);
                }
                ConnectionStatus::Mock => {
                    invalidate_live_load(load_started, connection_epoch);
                    // This should be unreachable under WebSocketRequired, but if
                    // provider semantics ever drift, fail visibly rather than
                    // accepting mock provenance inside Live mode.
                    ctx_for_effect.data_state.set(HearthDataState::Degraded);
                }
                ConnectionStatus::Connecting | ConnectionStatus::Connected => {
                    // Connected without signing authority is not a valid live
                    // hydration source. Cancel an older in-flight load if signer
                    // authority was removed while the transport stayed open.
                    invalidate_live_load(load_started, connection_epoch);
                    ctx_for_effect.data_state.set(HearthDataState::AwaitingLive);
                }
            }
        });
    }

    ctx
}

fn invalidate_live_load(load_started: RwSignal<bool>, connection_epoch: RwSignal<u64>) {
    if load_started.get_untracked() {
        connection_epoch.update(|epoch| *epoch = epoch.wrapping_add(1));
        load_started.set(false);
    }
}

fn load_is_current(connection_epoch: RwSignal<u64>, expected_epoch: u64) -> bool {
    connection_epoch.get_untracked() == expected_epoch
}

/// Load the first live Hearth and the domains currently supported by the
/// record bridge. Empty live results replace local vectors; they never leave
/// simulated values behind. Results from an obsolete connection epoch are
/// discarded instead of racing a reconnect.
async fn try_load_real_data(
    ctx: HearthCtx,
    hc: HolochainCtx,
    connection_epoch: RwSignal<u64>,
    expected_epoch: u64,
) {
    if let Some(agent) = hc.connected_agent_pub_key_b64() {
        if !load_is_current(connection_epoch, expected_epoch) {
            return;
        }
        ctx.my_agent.set(agent);
    }

    web_sys::console::log_1(&"[Hearth] Connected — loading live data...".into());

    let hearth_result = hc
        .call_zome_default::<(), Vec<WireRecord>>("hearth_kinship", "get_my_hearths", &())
        .await;
    if !load_is_current(connection_epoch, expected_epoch) {
        return;
    }

    let hearth_records = match hearth_result {
        Ok(records) => records,
        Err(error) => {
            web_sys::console::error_1(
                &format!("[Hearth] get_my_hearths failed: {error}").into(),
            );
            ctx.data_state.set(HearthDataState::Degraded);
            return;
        }
    };

    if hearth_records.is_empty() {
        ctx.current_hearth.set(None);
        ctx.members.set(Vec::new());
        ctx.my_role.set(None);
        ctx.bonds.set(Vec::new());
        ctx.gratitude.set(Vec::new());
        ctx.data_state.set(HearthDataState::Empty);
        return;
    }

    let hearths = record_bridge::records_to_hearths(&hearth_records);
    let Some(current_hearth) = hearths.first().cloned() else {
        web_sys::console::error_1(
            &"[Hearth] conductor returned Hearth records that could not be decoded".into(),
        );
        ctx.data_state.set(HearthDataState::Degraded);
        return;
    };

    let hearth_hash = current_hearth.hash.clone();
    ctx.current_hearth.set(Some(current_hearth));
    let mut degraded = false;

    let member_result = hc
        .call_zome_default::<String, Vec<WireRecord>>(
            "hearth_kinship",
            "get_hearth_members",
            &hearth_hash,
        )
        .await;
    if !load_is_current(connection_epoch, expected_epoch) {
        return;
    }
    match member_result {
        Ok(member_records) => {
            let members = record_bridge::records_to_members(&member_records);
            let my_agent = ctx.my_agent.get_untracked();
            ctx.my_role.set(
                members
                    .iter()
                    .find(|member| member.agent == my_agent)
                    .map(|member| member.role.clone()),
            );
            ctx.members.set(members);
        }
        Err(error) => {
            degraded = true;
            web_sys::console::error_1(
                &format!("[Hearth] get_hearth_members failed: {error}").into(),
            );
        }
    }

    let bond_result = hc
        .call_zome_default::<String, Vec<WireRecord>>(
            "hearth_kinship",
            "get_kinship_graph",
            &hearth_hash,
        )
        .await;
    if !load_is_current(connection_epoch, expected_epoch) {
        return;
    }
    match bond_result {
        Ok(bond_records) => ctx
            .bonds
            .set(record_bridge::records_to_bonds(&bond_records)),
        Err(error) => {
            degraded = true;
            web_sys::console::error_1(
                &format!("[Hearth] get_kinship_graph failed: {error}").into(),
            );
        }
    }

    let gratitude_result = hc
        .call_zome_default::<String, Vec<WireRecord>>(
            "hearth_gratitude",
            "get_gratitude_stream",
            &hearth_hash,
        )
        .await;
    if !load_is_current(connection_epoch, expected_epoch) {
        return;
    }
    match gratitude_result {
        Ok(gratitude_records) => ctx
            .gratitude
            .set(record_bridge::records_to_gratitude(&gratitude_records)),
        Err(error) => {
            degraded = true;
            web_sys::console::error_1(
                &format!("[Hearth] get_gratitude_stream failed: {error}").into(),
            );
        }
    }

    if load_is_current(connection_epoch, expected_epoch) {
        ctx.data_state.set(if degraded {
            HearthDataState::Degraded
        } else {
            HearthDataState::Live
        });
    }
}

pub fn use_hearth() -> HearthCtx {
    expect_context::<HearthCtx>()
}

/// Look up a member's display name by agent key.
pub fn member_name(members: &[MemberView], agent: &str) -> String {
    members
        .iter()
        .find(|member| member.agent == agent)
        .map(|member| member.display_name.clone())
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
