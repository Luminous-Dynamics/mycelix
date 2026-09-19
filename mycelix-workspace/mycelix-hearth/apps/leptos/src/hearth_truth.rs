// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Provenance boundary between Hearth demo data and source-backed Holochain data.
//!
//! A connected transport does not make every Hearth domain live-backed. This
//! module records availability independently per data family, clears demo data
//! before entering live mode, and promotes only successfully established reads.
//! `Live` in this module means a complete source-backed snapshot; the current
//! browser transport does not yet provide Hearth signal callbacks, so it must
//! not be read as a continuous-freshness claim.

use crate::hearth_context::{HearthCtx, use_hearth};
use crate::record_bridge::{self, WireRecord};
use hearth_leptos_types::*;
use leptos::prelude::*;
use mycelix_leptos_core::holochain_provider::use_holochain;
use mycelix_leptos_core::{AvailabilityStateKind, ConnectionStatus};
use std::cell::Cell;
use std::rc::Rc;
use wasm_bindgen_futures::spawn_local;

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct HearthAvailability {
    pub current_hearth: AvailabilityStateKind,
    pub caller_role: AvailabilityStateKind,
    pub members: AvailabilityStateKind,
    pub bonds: AvailabilityStateKind,
    pub care_schedules: AvailabilityStateKind,
    pub decisions: AvailabilityStateKind,
    pub votes: AvailabilityStateKind,
    pub gratitude: AvailabilityStateKind,
    pub stories: AvailabilityStateKind,
    pub rhythms: AvailabilityStateKind,
    pub presence: AvailabilityStateKind,
    pub emergency_alerts: AvailabilityStateKind,
    pub resources: AvailabilityStateKind,
    pub milestones: AvailabilityStateKind,
    pub autonomy_profiles: AvailabilityStateKind,
}

impl HearthAvailability {
    pub fn mock_mode() -> Self {
        Self::filled(AvailabilityStateKind::Mock)
    }

    pub fn live_pending() -> Self {
        Self {
            current_hearth: AvailabilityStateKind::Unknown,
            caller_role: AvailabilityStateKind::Unknown,
            members: AvailabilityStateKind::Unknown,
            bonds: AvailabilityStateKind::Unknown,
            care_schedules: AvailabilityStateKind::Unknown,
            decisions: AvailabilityStateKind::Unknown,
            votes: AvailabilityStateKind::Unknown,
            gratitude: AvailabilityStateKind::Unknown,
            stories: AvailabilityStateKind::Unavailable,
            rhythms: AvailabilityStateKind::Unknown,
            presence: AvailabilityStateKind::Unknown,
            emergency_alerts: AvailabilityStateKind::Unavailable,
            resources: AvailabilityStateKind::Unavailable,
            milestones: AvailabilityStateKind::Unavailable,
            autonomy_profiles: AvailabilityStateKind::Unavailable,
        }
    }

    fn filled(kind: AvailabilityStateKind) -> Self {
        Self {
            current_hearth: kind,
            caller_role: kind,
            members: kind,
            bonds: kind,
            care_schedules: kind,
            decisions: kind,
            votes: kind,
            gratitude: kind,
            stories: kind,
            rhythms: kind,
            presence: kind,
            emergency_alerts: kind,
            resources: kind,
            milestones: kind,
            autonomy_profiles: kind,
        }
    }

    fn mark_loaded_data_degraded(&mut self) {
        for state in [
            &mut self.current_hearth,
            &mut self.caller_role,
            &mut self.members,
            &mut self.bonds,
            &mut self.care_schedules,
            &mut self.decisions,
            &mut self.votes,
            &mut self.gratitude,
            &mut self.rhythms,
            &mut self.presence,
        ] {
            if matches!(
                *state,
                AvailabilityStateKind::Live | AvailabilityStateKind::Empty
            ) {
                *state = AvailabilityStateKind::Degraded;
            }
        }
    }
}

#[derive(Clone)]
pub struct HearthTruthState {
    pub availability: RwSignal<HearthAvailability>,
}

pub fn provide_hearth_truth() -> HearthTruthState {
    let truth = HearthTruthState {
        availability: RwSignal::new(HearthAvailability::mock_mode()),
    };
    provide_context(truth.clone());

    let hearth = use_hearth();
    let hc = use_holochain();
    let prepared_live = Rc::new(Cell::new(false));
    let load_started = Rc::new(Cell::new(false));

    let truth_for_status = truth.clone();
    let hearth_for_status = hearth.clone();
    let hc_for_status = hc.clone();
    Effect::new(move |_| {
        let status = hc_for_status.status.get();
        let signer_ready = hc_for_status.zome_call_signing_ready.get();

        match status {
            ConnectionStatus::Connected => {
                if !prepared_live.replace(true) {
                    enter_live_mode(&hearth_for_status, &hc_for_status);
                    truth_for_status
                        .availability
                        .set(HearthAvailability::live_pending());
                }

                if signer_ready && !load_started.replace(true) {
                    truth_for_status
                        .availability
                        .update(reset_live_readable_to_unknown);
                    let hearth = hearth_for_status.clone();
                    let truth = truth_for_status.clone();
                    let hc = hc_for_status.clone();
                    spawn_local(async move {
                        load_live_data(hearth, truth, hc).await;
                    });
                }
            }
            ConnectionStatus::Mock => {
                if !prepared_live.get() {
                    truth_for_status
                        .availability
                        .set(HearthAvailability::mock_mode());
                }
            }
            ConnectionStatus::Disconnected | ConnectionStatus::Reconnecting => {
                if prepared_live.get() {
                    truth_for_status
                        .availability
                        .update(HearthAvailability::mark_loaded_data_degraded);
                }
            }
            ConnectionStatus::Connecting => {}
        }
    });

    truth
}

pub fn use_hearth_truth() -> HearthTruthState {
    expect_context::<HearthTruthState>()
}

/// Install simulated family-life timers only if the provider actually resolves
/// to mock mode. Connecting/live sessions never install them.
pub fn start_mock_simulation_when_resolved() {
    let hc = use_holochain();
    let started = Rc::new(Cell::new(false));

    Effect::new(move |_| {
        if hc.status.get() == ConnectionStatus::Mock && !started.replace(true) {
            crate::simulated_life::start_simulated_life();
        }
    });
}

fn enter_live_mode(hearth: &HearthCtx, hc: &mycelix_leptos_core::HolochainCtx) {
    hearth.current_hearth.set(None);
    hearth.members.set(Vec::new());
    hearth.my_role.set(None);
    hearth.bonds.set(Vec::new());
    hearth.care_schedules.set(Vec::new());
    hearth.decisions.set(Vec::new());
    hearth.votes.set(Vec::new());
    hearth.gratitude.set(Vec::new());
    hearth.stories.set(Vec::new());
    hearth.rhythms.set(Vec::new());
    hearth.presence.set(Vec::new());
    hearth.emergency_alerts.set(Vec::new());
    hearth.resources.set(Vec::new());
    hearth.milestones.set(Vec::new());
    hearth.autonomy_profiles.set(Vec::new());
    hearth
        .my_agent
        .set(hc.connected_agent_pub_key_b64().unwrap_or_default());
}

fn reset_live_readable_to_unknown(availability: &mut HearthAvailability) {
    for state in [
        &mut availability.current_hearth,
        &mut availability.caller_role,
        &mut availability.members,
        &mut availability.bonds,
        &mut availability.care_schedules,
        &mut availability.decisions,
        &mut availability.votes,
        &mut availability.gratitude,
        &mut availability.rhythms,
        &mut availability.presence,
    ] {
        *state = AvailabilityStateKind::Unknown;
    }
}

fn mark_live_readable_unavailable(availability: &mut HearthAvailability) {
    for state in [
        &mut availability.current_hearth,
        &mut availability.caller_role,
        &mut availability.members,
        &mut availability.bonds,
        &mut availability.care_schedules,
        &mut availability.decisions,
        &mut availability.votes,
        &mut availability.gratitude,
        &mut availability.rhythms,
        &mut availability.presence,
    ] {
        *state = AvailabilityStateKind::Unavailable;
    }
}

fn mark_live_readable_empty(availability: &mut HearthAvailability) {
    for state in [
        &mut availability.current_hearth,
        &mut availability.caller_role,
        &mut availability.members,
        &mut availability.bonds,
        &mut availability.care_schedules,
        &mut availability.decisions,
        &mut availability.votes,
        &mut availability.gratitude,
        &mut availability.rhythms,
        &mut availability.presence,
    ] {
        *state = AvailabilityStateKind::Empty;
    }
}

fn record_set_availability(record_count: usize, decoded_count: usize) -> AvailabilityStateKind {
    match (record_count, decoded_count) {
        (0, 0) => AvailabilityStateKind::Empty,
        (records, decoded) if records == decoded => AvailabilityStateKind::Live,
        _ => AvailabilityStateKind::Degraded,
    }
}

#[derive(Debug, Clone, serde::Deserialize)]
struct WireHearth {
    name: String,
    description: String,
    hearth_type: HearthType,
    created_by: Vec<u8>,
    created_at: i64,
    max_members: u32,
}

fn record_to_hearth(record: &WireRecord) -> Option<HearthView> {
    let hearth: WireHearth = record.decode_entry()?;
    Some(HearthView {
        hash: record.action_hash_b64(),
        name: hearth.name,
        description: hearth.description,
        hearth_type: hearth.hearth_type,
        created_by: base64_encode(&hearth.created_by),
        created_at: hearth.created_at / 1_000_000,
        max_members: hearth.max_members,
    })
}

async fn load_live_data(
    hearth: HearthCtx,
    truth: HearthTruthState,
    hc: mycelix_leptos_core::HolochainCtx,
) {
    let hearth_records = match hc
        .call_zome_default::<(), Vec<WireRecord>>("hearth_kinship", "get_my_hearths", &())
        .await
    {
        Ok(records) => records,
        Err(error) => {
            truth
                .availability
                .update(mark_live_readable_unavailable);
            web_sys::console::log_1(
                &format!("[Hearth] get_my_hearths failed: {error}").into(),
            );
            return;
        }
    };

    let Some(first_hearth) = hearth_records.first() else {
        truth.availability.update(mark_live_readable_empty);
        return;
    };

    let hearth_hash = first_hearth.action_hash_b64();
    match record_to_hearth(first_hearth) {
        Some(view) => {
            hearth.current_hearth.set(Some(view));
            truth.availability.update(|availability| {
                availability.current_hearth = AvailabilityStateKind::Live;
            });
        }
        None => {
            truth.availability.update(|availability| {
                availability.current_hearth = AvailabilityStateKind::Degraded;
            });
        }
    }

    match hc
        .call_zome_default::<String, Option<MemberRole>>(
            "hearth_kinship",
            "get_caller_role",
            &hearth_hash,
        )
        .await
    {
        Ok(role) => {
            let state = if role.is_some() {
                AvailabilityStateKind::Live
            } else {
                AvailabilityStateKind::Empty
            };
            hearth.my_role.set(role);
            truth
                .availability
                .update(|availability| availability.caller_role = state);
        }
        Err(error) => {
            truth.availability.update(|availability| {
                availability.caller_role = AvailabilityStateKind::Unavailable;
            });
            web_sys::console::log_1(
                &format!("[Hearth] get_caller_role failed: {error}").into(),
            );
        }
    }

    match hc
        .call_zome_default::<String, Vec<WireRecord>>(
            "hearth_kinship",
            "get_hearth_members",
            &hearth_hash,
        )
        .await
    {
        Ok(records) => {
            let record_count = records.len();
            let views = record_bridge::records_to_members(&records);
            let state = record_set_availability(record_count, views.len());
            hearth.members.set(views);
            truth
                .availability
                .update(|availability| availability.members = state);
        }
        Err(error) => {
            truth
                .availability
                .update(|availability| availability.members = AvailabilityStateKind::Unavailable);
            web_sys::console::log_1(
                &format!("[Hearth] get_hearth_members failed: {error}").into(),
            );
        }
    }

    match hc
        .call_zome_default::<String, Vec<WireRecord>>(
            "hearth_kinship",
            "get_kinship_graph",
            &hearth_hash,
        )
        .await
    {
        Ok(records) => {
            let record_count = records.len();
            let views = record_bridge::records_to_bonds(&records);
            let state = record_set_availability(record_count, views.len());
            hearth.bonds.set(views);
            truth
                .availability
                .update(|availability| availability.bonds = state);
        }
        Err(error) => {
            truth
                .availability
                .update(|availability| availability.bonds = AvailabilityStateKind::Unavailable);
            web_sys::console::log_1(
                &format!("[Hearth] get_kinship_graph failed: {error}").into(),
            );
        }
    }

    match hc
        .call_zome_default::<String, Vec<WireRecord>>(
            "hearth_gratitude",
            "get_gratitude_stream",
            &hearth_hash,
        )
        .await
    {
        Ok(records) => {
            let record_count = records.len();
            let views = record_bridge::records_to_gratitude(&records);
            let state = record_set_availability(record_count, views.len());
            hearth.gratitude.set(views);
            truth
                .availability
                .update(|availability| availability.gratitude = state);
        }
        Err(error) => {
            truth.availability.update(|availability| {
                availability.gratitude = AvailabilityStateKind::Unavailable;
            });
            web_sys::console::log_1(
                &format!("[Hearth] get_gratitude_stream failed: {error}").into(),
            );
        }
    }

    match hc
        .call_zome_default::<String, Vec<WireRecord>>(
            "hearth_care",
            "get_hearth_schedule",
            &hearth_hash,
        )
        .await
    {
        Ok(records) => {
            let record_count = records.len();
            let views = record_bridge::records_to_care_schedules(&records);
            let state = record_set_availability(record_count, views.len());
            hearth.care_schedules.set(views);
            truth
                .availability
                .update(|availability| availability.care_schedules = state);
        }
        Err(error) => {
            truth.availability.update(|availability| {
                availability.care_schedules = AvailabilityStateKind::Unavailable;
            });
            web_sys::console::log_1(
                &format!("[Hearth] get_hearth_schedule failed: {error}").into(),
            );
        }
    }

    match hc
        .call_zome_default::<String, Vec<WireRecord>>(
            "hearth_rhythms",
            "get_hearth_rhythms",
            &hearth_hash,
        )
        .await
    {
        Ok(records) => {
            let record_count = records.len();
            let views = record_bridge::records_to_rhythms(&records);
            let state = record_set_availability(record_count, views.len());
            hearth.rhythms.set(views);
            truth
                .availability
                .update(|availability| availability.rhythms = state);
        }
        Err(error) => {
            truth.availability.update(|availability| {
                availability.rhythms = AvailabilityStateKind::Unavailable;
            });
            web_sys::console::log_1(
                &format!("[Hearth] get_hearth_rhythms failed: {error}").into(),
            );
        }
    }

    match hc
        .call_zome_default::<String, Vec<WireRecord>>(
            "hearth_rhythms",
            "get_hearth_presence",
            &hearth_hash,
        )
        .await
    {
        Ok(records) => {
            let record_count = records.len();
            let decoded = record_bridge::records_to_presence(&records);
            let state = record_set_availability(record_count, decoded.decoded_records);
            hearth.presence.set(decoded.views);
            truth
                .availability
                .update(|availability| availability.presence = state);
        }
        Err(error) => {
            truth.availability.update(|availability| {
                availability.presence = AvailabilityStateKind::Unavailable;
            });
            web_sys::console::log_1(
                &format!("[Hearth] get_hearth_presence failed: {error}").into(),
            );
        }
    }

    load_decisions_and_votes(&hearth, &truth, &hc, &hearth_hash).await;
}

async fn load_decisions_and_votes(
    hearth: &HearthCtx,
    truth: &HearthTruthState,
    hc: &mycelix_leptos_core::HolochainCtx,
    hearth_hash: &String,
) {
    let decision_records = match hc
        .call_zome_default::<String, Vec<WireRecord>>(
            "hearth_decisions",
            "get_hearth_decisions",
            hearth_hash,
        )
        .await
    {
        Ok(records) => records,
        Err(error) => {
            truth.availability.update(|availability| {
                availability.decisions = AvailabilityStateKind::Unavailable;
                availability.votes = AvailabilityStateKind::Unavailable;
            });
            web_sys::console::log_1(
                &format!("[Hearth] get_hearth_decisions failed: {error}").into(),
            );
            return;
        }
    };

    let decision_record_count = decision_records.len();
    let decisions = record_bridge::records_to_decisions(&decision_records);
    let decision_state = record_set_availability(decision_record_count, decisions.len());
    hearth.decisions.set(decisions.clone());
    truth
        .availability
        .update(|availability| availability.decisions = decision_state);

    if decisions.is_empty() {
        let vote_state = if decision_state == AvailabilityStateKind::Empty {
            AvailabilityStateKind::Empty
        } else {
            AvailabilityStateKind::Degraded
        };
        hearth.votes.set(Vec::new());
        truth
            .availability
            .update(|availability| availability.votes = vote_state);
        return;
    }

    let mut vote_record_count = 0usize;
    let mut decoded_vote_count = 0usize;
    let mut votes = Vec::new();
    let mut successful_queries = 0usize;
    let mut failed_queries = 0usize;

    for decision in &decisions {
        match hc
            .call_zome_default::<String, Vec<WireRecord>>(
                "hearth_decisions",
                "get_decision_votes",
                &decision.hash,
            )
            .await
        {
            Ok(records) => {
                successful_queries += 1;
                vote_record_count += records.len();
                let decoded = record_bridge::records_to_votes(&records);
                decoded_vote_count += decoded.len();
                votes.extend(decoded);
            }
            Err(error) => {
                failed_queries += 1;
                web_sys::console::log_1(
                    &format!(
                        "[Hearth] get_decision_votes failed for {}: {error}",
                        decision.hash
                    )
                    .into(),
                );
            }
        }
    }

    hearth.votes.set(votes);
    let vote_state = if successful_queries == 0 && failed_queries > 0 {
        AvailabilityStateKind::Unavailable
    } else if failed_queries > 0 || decision_state == AvailabilityStateKind::Degraded {
        AvailabilityStateKind::Degraded
    } else {
        record_set_availability(vote_record_count, decoded_vote_count)
    };
    truth
        .availability
        .update(|availability| availability.votes = vote_state);
}

#[component]
pub fn HearthDataStatus() -> impl IntoView {
    let truth = use_hearth_truth();
    let hc = use_holochain();

    view! {
        {move || {
            let status = hc.status.get();
            let signer_ready = hc.zome_call_signing_ready.get();
            let availability = truth.availability.get();

            let message = match status {
                ConnectionStatus::Connecting => Some(
                    "Connecting to Hearth. Data shown during connection setup is sample data until a source-backed snapshot is established."
                        .to_string(),
                ),
                ConnectionStatus::Connected if !signer_ready => Some(
                    "Connected to the conductor, but browser zome-call signing is unavailable. Demo records are hidden; Hearth snapshots cannot be loaded through this client yet."
                        .to_string(),
                ),
                ConnectionStatus::Connected => Some(format!(
                    "Source-backed Hearth snapshot — hearth: {}, role: {}, members: {}, bonds: {}, care: {}, decisions: {}, votes: {}, gratitude: {}, rhythms: {}, presence: {}. Real-time conductor signal callbacks are not wired yet, so this is a loaded snapshot rather than a continuous-freshness claim. Unsupported domains remain unavailable instead of falling back to demo records.",
                    availability.current_hearth.label(),
                    availability.caller_role.label(),
                    availability.members.label(),
                    availability.bonds.label(),
                    availability.care_schedules.label(),
                    availability.decisions.label(),
                    availability.votes.label(),
                    availability.gratitude.label(),
                    availability.rhythms.label(),
                    availability.presence.label(),
                )),
                ConnectionStatus::Disconnected | ConnectionStatus::Reconnecting
                    if availability.current_hearth != AvailabilityStateKind::Mock =>
                {
                    Some(
                        "Hearth's source connection is interrupted. Previously loaded source-backed records are retained only as degraded data until the connection is re-established."
                            .to_string(),
                    )
                }
                _ => None,
            };

            message.map(|message| {
                view! {
                    <div class="hearth-data-status" role="status">
                        {message}
                    </div>
                }
            })
        }}
    }
}

fn base64_encode(bytes: &[u8]) -> String {
    const CHARS: &[u8] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    let mut result = String::with_capacity(bytes.len() * 4 / 3 + 4);
    for chunk in bytes.chunks(3) {
        let b0 = chunk[0] as u32;
        let b1 = chunk.get(1).copied().unwrap_or(0) as u32;
        let b2 = chunk.get(2).copied().unwrap_or(0) as u32;
        let n = (b0 << 16) | (b1 << 8) | b2;
        result.push(CHARS[((n >> 18) & 0x3f) as usize] as char);
        result.push(CHARS[((n >> 12) & 0x3f) as usize] as char);
        if chunk.len() > 1 {
            result.push(CHARS[((n >> 6) & 0x3f) as usize] as char);
        } else {
            result.push('=');
        }
        if chunk.len() > 2 {
            result.push(CHARS[(n & 0x3f) as usize] as char);
        } else {
            result.push('=');
        }
    }
    result
}

#[cfg(test)]
mod tests {
    use super::{HearthAvailability, record_set_availability};
    use mycelix_leptos_core::AvailabilityStateKind;

    #[test]
    fn live_transition_marks_supported_reads_unknown_and_unwired_domains_unavailable() {
        let state = HearthAvailability::live_pending();
        assert_eq!(state.current_hearth, AvailabilityStateKind::Unknown);
        assert_eq!(state.caller_role, AvailabilityStateKind::Unknown);
        assert_eq!(state.care_schedules, AvailabilityStateKind::Unknown);
        assert_eq!(state.decisions, AvailabilityStateKind::Unknown);
        assert_eq!(state.presence, AvailabilityStateKind::Unknown);
        assert_eq!(state.stories, AvailabilityStateKind::Unavailable);
        assert_eq!(state.emergency_alerts, AvailabilityStateKind::Unavailable);
    }

    #[test]
    fn successful_empty_is_distinct_from_unavailable() {
        assert_eq!(
            record_set_availability(0, 0),
            AvailabilityStateKind::Empty
        );
    }

    #[test]
    fn partial_decode_is_degraded_not_live() {
        assert_eq!(
            record_set_availability(3, 2),
            AvailabilityStateKind::Degraded
        );
        assert_eq!(
            record_set_availability(2, 2),
            AvailabilityStateKind::Live
        );
    }
}
