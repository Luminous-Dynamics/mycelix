// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Provenance boundary between Hearth demo data and source-backed Holochain data.
//!
//! A connected transport does not make every Hearth domain live-backed. This
//! module records availability independently per data family, clears demo data
//! before entering live mode, and promotes only successfully established reads.
//! `Live` here means a complete source-backed snapshot; continuous freshness is
//! not claimed until conductor signal callbacks are wired.

use crate::hearth_context::{HearthCtx, use_hearth};
use crate::record_bridge::{self, WireRecord};
use hearth_leptos_types::*;
use leptos::prelude::*;
use mycelix_leptos_client::HoloHashBytes;
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

/// Shared monotonic generation gate for the primary Hearth snapshot loader.
///
/// Every accepted load receives a token. Any later load or invalidation advances
/// the generation, making all earlier tokens permanently stale. A stale async
/// task may finish its transport call, but it must not publish state or clear the
/// loading flag owned by a newer task.
#[derive(Clone, Default)]
struct LoadGeneration {
    current: Rc<Cell<u64>>,
}

impl LoadGeneration {
    fn advance(&self) -> u64 {
        let next = self.current.get().wrapping_add(1);
        self.current.set(next);
        next
    }

    fn invalidate(&self) {
        self.advance();
    }

    fn is_current(&self, token: u64) -> bool {
        self.current.get() == token
    }
}

#[derive(Clone)]
pub struct HearthTruthState {
    pub availability: RwSignal<HearthAvailability>,
    pub loading: RwSignal<bool>,
    generation: LoadGeneration,
}

impl HearthTruthState {
    fn invalidate_active_load(&self) {
        self.generation.invalidate();
        self.loading.set(false);
    }
}

pub fn provide_hearth_truth() -> HearthTruthState {
    let truth = HearthTruthState {
        availability: RwSignal::new(HearthAvailability::mock_mode()),
        loading: RwSignal::new(false),
        generation: LoadGeneration::default(),
    };
    provide_context(truth.clone());

    let hearth = use_hearth();
    let hc = use_holochain();
    let prepared_live = Rc::new(Cell::new(false));
    let loaded_for_connection = Rc::new(Cell::new(false));

    let truth_for_status = truth.clone();
    let hearth_for_status = hearth.clone();
    let hc_for_status = hc.clone();
    Effect::new(move |_| {
        let status = hc_for_status.status.get();
        let signer_ready = hc_for_status.zome_call_signing_ready.get();
        let loading = truth_for_status.loading.get();

        match status {
            ConnectionStatus::Connected => {
                if !prepared_live.replace(true) {
                    enter_live_mode(&hearth_for_status, &hc_for_status);
                    truth_for_status
                        .availability
                        .set(HearthAvailability::live_pending());
                }

                if signer_ready {
                    if !loading && !loaded_for_connection.get()
                        && start_live_load(
                            hearth_for_status.clone(),
                            truth_for_status.clone(),
                            hc_for_status.clone(),
                        )
                    {
                        loaded_for_connection.set(true);
                    }
                } else {
                    loaded_for_connection.set(false);
                    if loading {
                        truth_for_status.invalidate_active_load();
                    }
                }
            }
            ConnectionStatus::Mock => {
                loaded_for_connection.set(false);
                if loading {
                    truth_for_status.invalidate_active_load();
                }
                if !prepared_live.get() {
                    truth_for_status
                        .availability
                        .set(HearthAvailability::mock_mode());
                }
            }
            ConnectionStatus::Disconnected | ConnectionStatus::Reconnecting => {
                loaded_for_connection.set(false);
                if loading {
                    truth_for_status.invalidate_active_load();
                }
                if prepared_live.get() {
                    truth_for_status
                        .availability
                        .update(HearthAvailability::mark_loaded_data_degraded);
                }
            }
            ConnectionStatus::Connecting => {
                loaded_for_connection.set(false);
                if loading {
                    truth_for_status.invalidate_active_load();
                }
                if prepared_live.get() {
                    truth_for_status
                        .availability
                        .update(HearthAvailability::mark_loaded_data_degraded);
                }
            }
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

fn start_live_load(
    hearth: HearthCtx,
    truth: HearthTruthState,
    hc: mycelix_leptos_core::HolochainCtx,
) -> bool {
    if truth.loading.get_untracked() || !hc.zome_calls_ready_untracked() {
        return false;
    }

    let token = truth.generation.advance();
    truth.loading.set(true);

    let truth_for_load = truth.clone();
    let generation = truth.generation.clone();
    spawn_local(async move {
        let staged = load_live_snapshot(&hc, &generation, token).await;

        if generation.is_current(token) && hc.zome_calls_ready_untracked() {
            if let Some(snapshot) = staged {
                snapshot.publish(&hearth, &truth_for_load);
            }
        }

        // A stale task must never clear a newer task's loading state.
        if generation.is_current(token) {
            truth_for_load.loading.set(false);
        }
    });

    true
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
        created_by: record_bridge::agent_display(&hearth.created_by)?,
        created_at: hearth.created_at / 1_000_000,
        max_members: hearth.max_members,
    })
}

fn parse_source_hash(label: &str, raw_base64: &str) -> Result<HoloHashBytes, String> {
    HoloHashBytes::from_action_raw_base64(raw_base64)
        .map_err(|error| format!("{label} is not a valid ActionHash: {error}"))
}

/// One generation's complete supported source snapshot. Nothing in this value
/// is visible to the UI until `publish` is called after the full read sequence
/// has finished and the generation token is still current.
struct StagedHearthSnapshot {
    current_hearth: Option<HearthView>,
    my_role: Option<MemberRole>,
    members: Vec<MemberView>,
    bonds: Vec<BondView>,
    care_schedules: Vec<CareScheduleView>,
    decisions: Vec<DecisionView>,
    votes: Vec<VoteView>,
    gratitude: Vec<GratitudeExpressionView>,
    rhythms: Vec<RhythmView>,
    presence: Vec<PresenceView>,
    my_agent: String,
    availability: HearthAvailability,
}

impl StagedHearthSnapshot {
    fn pending(my_agent: String) -> Self {
        Self {
            current_hearth: None,
            my_role: None,
            members: Vec::new(),
            bonds: Vec::new(),
            care_schedules: Vec::new(),
            decisions: Vec::new(),
            votes: Vec::new(),
            gratitude: Vec::new(),
            rhythms: Vec::new(),
            presence: Vec::new(),
            my_agent,
            availability: HearthAvailability::live_pending(),
        }
    }

    fn publish(self, hearth: &HearthCtx, truth: &HearthTruthState) {
        // These writes occur synchronously after the complete generation has
        // been staged. Reactive consumers therefore never observe source-A from
        // the new generation while source-B is still awaiting transport.
        hearth.current_hearth.set(self.current_hearth);
        hearth.my_role.set(self.my_role);
        hearth.members.set(self.members);
        hearth.bonds.set(self.bonds);
        hearth.care_schedules.set(self.care_schedules);
        hearth.decisions.set(self.decisions);
        hearth.votes.set(self.votes);
        hearth.gratitude.set(self.gratitude);
        hearth.rhythms.set(self.rhythms);
        hearth.presence.set(self.presence);
        hearth.my_agent.set(self.my_agent);
        truth.availability.set(self.availability);
    }
}

struct DecisionStage {
    decisions: Vec<DecisionView>,
    votes: Vec<VoteView>,
    decision_state: AvailabilityStateKind,
    vote_state: AvailabilityStateKind,
}

async fn load_live_snapshot(
    hc: &mycelix_leptos_core::HolochainCtx,
    generation: &LoadGeneration,
    token: u64,
) -> Option<StagedHearthSnapshot> {
    let mut snapshot = StagedHearthSnapshot::pending(
        hc.connected_agent_pub_key_b64().unwrap_or_default(),
    );

    let hearth_records = match hc
        .call_zome_default::<(), Vec<WireRecord>>("hearth_kinship", "get_my_hearths", &())
        .await
    {
        Ok(records) => {
            if !generation.is_current(token) {
                return None;
            }
            records
        }
        Err(error) => {
            if !generation.is_current(token) {
                return None;
            }
            mark_live_readable_unavailable(&mut snapshot.availability);
            web_sys::console::log_1(
                &format!("[Hearth] get_my_hearths failed: {error}").into(),
            );
            return Some(snapshot);
        }
    };

    let Some(first_hearth) = hearth_records.first() else {
        mark_live_readable_empty(&mut snapshot.availability);
        return Some(snapshot);
    };

    let hearth_hash_text = first_hearth.action_hash_b64();
    let hearth_hash = match parse_source_hash("current Hearth action hash", &hearth_hash_text) {
        Ok(hash) => hash,
        Err(error) => {
            snapshot.availability.current_hearth = AvailabilityStateKind::Degraded;
            snapshot.availability.caller_role = AvailabilityStateKind::Unavailable;
            snapshot.availability.members = AvailabilityStateKind::Unavailable;
            snapshot.availability.bonds = AvailabilityStateKind::Unavailable;
            snapshot.availability.care_schedules = AvailabilityStateKind::Unavailable;
            snapshot.availability.decisions = AvailabilityStateKind::Unavailable;
            snapshot.availability.votes = AvailabilityStateKind::Unavailable;
            snapshot.availability.gratitude = AvailabilityStateKind::Unavailable;
            snapshot.availability.rhythms = AvailabilityStateKind::Unavailable;
            snapshot.availability.presence = AvailabilityStateKind::Unavailable;
            web_sys::console::log_1(&format!("[Hearth] {error}").into());
            return Some(snapshot);
        }
    };

    match record_to_hearth(first_hearth) {
        Some(view) => {
            snapshot.current_hearth = Some(view);
            snapshot.availability.current_hearth = AvailabilityStateKind::Live;
        }
        None => {
            snapshot.availability.current_hearth = AvailabilityStateKind::Degraded;
        }
    }

    match hc
        .call_zome_default::<HoloHashBytes, Option<MemberRole>>(
            "hearth_kinship",
            "get_caller_role",
            &hearth_hash,
        )
        .await
    {
        Ok(role) => {
            if !generation.is_current(token) {
                return None;
            }
            snapshot.availability.caller_role = if role.is_some() {
                AvailabilityStateKind::Live
            } else {
                AvailabilityStateKind::Empty
            };
            snapshot.my_role = role;
        }
        Err(error) => {
            if !generation.is_current(token) {
                return None;
            }
            snapshot.availability.caller_role = AvailabilityStateKind::Unavailable;
            web_sys::console::log_1(
                &format!("[Hearth] get_caller_role failed: {error}").into(),
            );
        }
    }

    match hc
        .call_zome_default::<HoloHashBytes, Vec<WireRecord>>(
            "hearth_kinship",
            "get_hearth_members",
            &hearth_hash,
        )
        .await
    {
        Ok(records) => {
            if !generation.is_current(token) {
                return None;
            }
            snapshot.members = record_bridge::records_to_members(&records);
            snapshot.availability.members =
                record_set_availability(records.len(), snapshot.members.len());
        }
        Err(error) => {
            if !generation.is_current(token) {
                return None;
            }
            snapshot.availability.members = AvailabilityStateKind::Unavailable;
            web_sys::console::log_1(
                &format!("[Hearth] get_hearth_members failed: {error}").into(),
            );
        }
    }

    match hc
        .call_zome_default::<HoloHashBytes, Vec<WireRecord>>(
            "hearth_kinship",
            "get_kinship_graph",
            &hearth_hash,
        )
        .await
    {
        Ok(records) => {
            if !generation.is_current(token) {
                return None;
            }
            snapshot.bonds = record_bridge::records_to_bonds(&records);
            snapshot.availability.bonds =
                record_set_availability(records.len(), snapshot.bonds.len());
        }
        Err(error) => {
            if !generation.is_current(token) {
                return None;
            }
            snapshot.availability.bonds = AvailabilityStateKind::Unavailable;
            web_sys::console::log_1(
                &format!("[Hearth] get_kinship_graph failed: {error}").into(),
            );
        }
    }

    match hc
        .call_zome_default::<HoloHashBytes, Vec<WireRecord>>(
            "hearth_gratitude",
            "get_gratitude_stream",
            &hearth_hash,
        )
        .await
    {
        Ok(records) => {
            if !generation.is_current(token) {
                return None;
            }
            snapshot.gratitude = record_bridge::records_to_gratitude(&records);
            snapshot.availability.gratitude =
                record_set_availability(records.len(), snapshot.gratitude.len());
        }
        Err(error) => {
            if !generation.is_current(token) {
                return None;
            }
            snapshot.availability.gratitude = AvailabilityStateKind::Unavailable;
            web_sys::console::log_1(
                &format!("[Hearth] get_gratitude_stream failed: {error}").into(),
            );
        }
    }

    match hc
        .call_zome_default::<HoloHashBytes, Vec<WireRecord>>(
            "hearth_care",
            "get_hearth_schedule",
            &hearth_hash,
        )
        .await
    {
        Ok(records) => {
            if !generation.is_current(token) {
                return None;
            }
            snapshot.care_schedules = record_bridge::records_to_care_schedules(&records);
            snapshot.availability.care_schedules =
                record_set_availability(records.len(), snapshot.care_schedules.len());
        }
        Err(error) => {
            if !generation.is_current(token) {
                return None;
            }
            snapshot.availability.care_schedules = AvailabilityStateKind::Unavailable;
            web_sys::console::log_1(
                &format!("[Hearth] get_hearth_schedule failed: {error}").into(),
            );
        }
    }

    match hc
        .call_zome_default::<HoloHashBytes, Vec<WireRecord>>(
            "hearth_rhythms",
            "get_hearth_rhythms",
            &hearth_hash,
        )
        .await
    {
        Ok(records) => {
            if !generation.is_current(token) {
                return None;
            }
            snapshot.rhythms = record_bridge::records_to_rhythms(&records);
            snapshot.availability.rhythms =
                record_set_availability(records.len(), snapshot.rhythms.len());
        }
        Err(error) => {
            if !generation.is_current(token) {
                return None;
            }
            snapshot.availability.rhythms = AvailabilityStateKind::Unavailable;
            web_sys::console::log_1(
                &format!("[Hearth] get_hearth_rhythms failed: {error}").into(),
            );
        }
    }

    match hc
        .call_zome_default::<HoloHashBytes, Vec<WireRecord>>(
            "hearth_rhythms",
            "get_hearth_presence",
            &hearth_hash,
        )
        .await
    {
        Ok(records) => {
            if !generation.is_current(token) {
                return None;
            }
            let decoded = record_bridge::records_to_presence(&records);
            snapshot.availability.presence =
                record_set_availability(records.len(), decoded.decoded_records);
            snapshot.presence = decoded.views;
        }
        Err(error) => {
            if !generation.is_current(token) {
                return None;
            }
            snapshot.availability.presence = AvailabilityStateKind::Unavailable;
            web_sys::console::log_1(
                &format!("[Hearth] get_hearth_presence failed: {error}").into(),
            );
        }
    }

    let decisions = load_decisions_and_votes(hc, &hearth_hash, generation, token).await?;
    snapshot.decisions = decisions.decisions;
    snapshot.votes = decisions.votes;
    snapshot.availability.decisions = decisions.decision_state;
    snapshot.availability.votes = decisions.vote_state;

    Some(snapshot)
}

async fn load_decisions_and_votes(
    hc: &mycelix_leptos_core::HolochainCtx,
    hearth_hash: &HoloHashBytes,
    generation: &LoadGeneration,
    token: u64,
) -> Option<DecisionStage> {
    let decision_records = match hc
        .call_zome_default::<HoloHashBytes, Vec<WireRecord>>(
            "hearth_decisions",
            "get_hearth_decisions",
            hearth_hash,
        )
        .await
    {
        Ok(records) => {
            if !generation.is_current(token) {
                return None;
            }
            records
        }
        Err(error) => {
            if !generation.is_current(token) {
                return None;
            }
            web_sys::console::log_1(
                &format!("[Hearth] get_hearth_decisions failed: {error}").into(),
            );
            return Some(DecisionStage {
                decisions: Vec::new(),
                votes: Vec::new(),
                decision_state: AvailabilityStateKind::Unavailable,
                vote_state: AvailabilityStateKind::Unavailable,
            });
        }
    };

    let decisions = record_bridge::records_to_decisions(&decision_records);
    let decision_state = record_set_availability(decision_records.len(), decisions.len());

    if decisions.is_empty() {
        let vote_state = if decision_state == AvailabilityStateKind::Empty {
            AvailabilityStateKind::Empty
        } else {
            AvailabilityStateKind::Degraded
        };
        return Some(DecisionStage {
            decisions,
            votes: Vec::new(),
            decision_state,
            vote_state,
        });
    }

    let mut vote_record_count = 0usize;
    let mut decoded_vote_count = 0usize;
    let mut votes = Vec::new();
    let mut successful_queries = 0usize;
    let mut failed_queries = 0usize;

    for decision in &decisions {
        let decision_hash = match parse_source_hash("decision action hash", &decision.hash) {
            Ok(hash) => hash,
            Err(error) => {
                failed_queries += 1;
                web_sys::console::log_1(&format!("[Hearth] {error}").into());
                continue;
            }
        };

        match hc
            .call_zome_default::<HoloHashBytes, Vec<WireRecord>>(
                "hearth_decisions",
                "get_decision_votes",
                &decision_hash,
            )
            .await
        {
            Ok(records) => {
                if !generation.is_current(token) {
                    return None;
                }
                successful_queries += 1;
                vote_record_count += records.len();
                let decoded = record_bridge::records_to_votes(&records);
                decoded_vote_count += decoded.len();
                votes.extend(decoded);
            }
            Err(error) => {
                if !generation.is_current(token) {
                    return None;
                }
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

    let vote_state = if successful_queries == 0 && failed_queries > 0 {
        AvailabilityStateKind::Unavailable
    } else if failed_queries > 0 || decision_state == AvailabilityStateKind::Degraded {
        AvailabilityStateKind::Degraded
    } else {
        record_set_availability(vote_record_count, decoded_vote_count)
    };

    Some(DecisionStage {
        decisions,
        votes,
        decision_state,
        vote_state,
    })
}

fn status_message(
    hc: &mycelix_leptos_core::HolochainCtx,
    truth: &HearthTruthState,
) -> Option<String> {
    let status = hc.status.get();
    let signer_ready = hc.zome_call_signing_ready.get();
    let loading = truth.loading.get();
    let availability = truth.availability.get();

    match status {
        ConnectionStatus::Connecting => Some(
            "Connecting to Hearth. Live data remains unavailable until a source-backed snapshot is established."
                .to_string(),
        ),
        ConnectionStatus::Connected if !signer_ready => Some(
            "Connected to the conductor, but browser zome-call signing is unavailable. Demo records are hidden; Hearth snapshots cannot be loaded through this client yet."
                .to_string(),
        ),
        ConnectionStatus::Connected if loading => Some(format!(
            "Refreshing Hearth snapshot — hearth: {}, role: {}, members: {}, bonds: {}, care: {}, decisions: {}, votes: {}, gratitude: {}, rhythms: {}, presence: {}. The currently established snapshot remains visible until this generation finishes.",
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
        ConnectionStatus::Connected => Some(format!(
            "Source-backed Hearth snapshot — hearth: {}, role: {}, members: {}, bonds: {}, care: {}, decisions: {}, votes: {}, gratitude: {}, rhythms: {}, presence: {}. Real-time conductor signal callbacks are not wired yet, so refresh/reconnect reconciliation is the freshness mechanism. Unsupported domains remain unavailable instead of falling back to demo records.",
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
                "Hearth's source connection is interrupted. Source-backed state is unavailable; any previously loaded records are retained only with degraded provenance. A fresh snapshot will be requested after the authorized connection is re-established."
                    .to_string(),
            )
        }
        _ => None,
    }
}

#[component]
pub fn HearthDataStatus() -> impl IntoView {
    let hearth = use_hearth();
    let truth = use_hearth_truth();
    let hc = use_holochain();

    let truth_for_show = truth.clone();
    let hc_for_show = hc.clone();
    let truth_for_message = truth.clone();
    let hc_for_message = hc.clone();
    let truth_for_button = truth.clone();
    let hc_for_button = hc.clone();
    let truth_for_label = truth.clone();
    let hearth_for_click = hearth.clone();
    let truth_for_click = truth.clone();
    let hc_for_click = hc.clone();

    view! {
        <Show when=move || status_message(&hc_for_show, &truth_for_show).is_some()>
            <div class="hearth-data-status" role="status">
                <span>
                    {move || status_message(&hc_for_message, &truth_for_message).unwrap_or_default()}
                </span>
                <button
                    class="hearth-refresh-snapshot"
                    type="button"
                    style=move || {
                        if hc_for_button.status.get() == ConnectionStatus::Connected
                            && hc_for_button.zome_call_signing_ready.get()
                        {
                            "display: inline-flex"
                        } else {
                            "display: none"
                        }
                    }
                    disabled=move || truth_for_button.loading.get()
                    on:click=move |_| {
                        start_live_load(
                            hearth_for_click.clone(),
                            truth_for_click.clone(),
                            hc_for_click.clone(),
                        );
                    }
                >
                    {move || {
                        if truth_for_label.loading.get() {
                            "Refreshing snapshot…"
                        } else {
                            "Refresh snapshot"
                        }
                    }}
                </button>
            </div>
        </Show>
    }
}

#[cfg(test)]
mod tests {
    use super::{
        HearthAvailability, LoadGeneration, StagedHearthSnapshot, record_set_availability,
    };
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

    #[test]
    fn newer_generation_makes_prior_load_stale() {
        let generation = LoadGeneration::default();
        let first = generation.advance();
        assert!(generation.is_current(first));

        let second = generation.advance();
        assert!(!generation.is_current(first));
        assert!(generation.is_current(second));
    }

    #[test]
    fn invalidation_rejects_in_flight_token() {
        let generation = LoadGeneration::default();
        let token = generation.advance();
        generation.invalidate();
        assert!(!generation.is_current(token));
    }

    #[test]
    fn staged_snapshot_starts_unpublished_and_source_pending() {
        let snapshot = StagedHearthSnapshot::pending(String::new());
        assert!(snapshot.current_hearth.is_none());
        assert!(snapshot.members.is_empty());
        assert!(snapshot.care_schedules.is_empty());
        assert_eq!(
            snapshot.availability.current_hearth,
            AvailabilityStateKind::Unknown
        );
        assert_eq!(
            snapshot.availability.stories,
            AvailabilityStateKind::Unavailable
        );
    }
}
