// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Source-backed personal Care attention.
//!
//! `hearth_care.get_my_care_duties` returns schedules linked to the connected
//! agent across Hearths. This module validates that source, projects only the
//! current Hearth, and independently checks it against the already-loaded
//! current-Hearth Care snapshot. It does not infer write authority or priority.

use crate::hearth_context::use_hearth;
use crate::hearth_truth::use_hearth_truth;
use crate::record_bridge::{self, WireRecord};
use hearth_leptos_types::{CareScheduleStatus, CareScheduleView};
use leptos::prelude::*;
use leptos_router::components::A;
use mycelix_leptos_client::HoloHashBytes;
use mycelix_leptos_core::holochain_provider::use_holochain;
use mycelix_leptos_core::{AvailabilityStateKind, ConnectionStatus};
use std::cell::{Cell, RefCell};
use std::collections::BTreeSet;
use std::rc::Rc;
use wasm_bindgen_futures::spawn_local;

#[derive(Clone, Debug)]
pub struct CareAttentionSnapshot {
    /// Availability of the complete `get_my_care_duties` response.
    pub availability: AvailabilityStateKind,
    /// Independent agreement with the current-Hearth Care snapshot.
    pub snapshot_alignment: AvailabilityStateKind,
    /// Valid duties belonging to the current Hearth, including non-Active ones.
    pub duties: Vec<CareScheduleView>,
}

impl CareAttentionSnapshot {
    fn unknown() -> Self {
        Self {
            availability: AvailabilityStateKind::Unknown,
            snapshot_alignment: AvailabilityStateKind::Unknown,
            duties: Vec::new(),
        }
    }
}

#[derive(Clone)]
pub struct CareAttentionState {
    pub snapshot: RwSignal<CareAttentionSnapshot>,
    pub loading: RwSignal<bool>,
}

fn schedule_fingerprint(schedule: &CareScheduleView) -> String {
    format!(
        "{}|{}|{}|{:?}|{}|{}|{:?}|{:?}|{:?}",
        schedule.hash,
        schedule.hearth_hash,
        schedule.assigned_to,
        schedule.care_type,
        schedule.title,
        schedule.description,
        schedule.recurrence,
        schedule.status,
        schedule.completed_at,
    )
}

fn validate_source_records(
    records: &[WireRecord],
    current_hearth_hash: &str,
    my_agent: &str,
) -> (AvailabilityStateKind, Vec<CareScheduleView>) {
    let decoded = record_bridge::records_to_care_schedules(records);
    let mut accepted_count = 0usize;
    let mut seen = BTreeSet::new();
    let mut current_hearth_duties = Vec::new();

    for duty in decoded {
        let record_hash_ok = HoloHashBytes::from_action_raw_base64(&duty.hash).is_ok();
        let hearth_hash_ok = HoloHashBytes::from_action_raw_base64(&duty.hearth_hash).is_ok();
        let assignee_ok = HoloHashBytes::from_agent_display(&duty.assigned_to).is_ok()
            && duty.assigned_to == my_agent;
        let unique = seen.insert(duty.hash.clone());

        if record_hash_ok && hearth_hash_ok && assignee_ok && unique {
            accepted_count += 1;
            if duty.hearth_hash == current_hearth_hash {
                current_hearth_duties.push(duty);
            }
        }
    }

    let state = match (records.len(), accepted_count) {
        (0, 0) => AvailabilityStateKind::Empty,
        (source, accepted) if source == accepted => AvailabilityStateKind::Live,
        _ => AvailabilityStateKind::Degraded,
    };

    (state, current_hearth_duties)
}

fn snapshot_alignment(
    care_state: AvailabilityStateKind,
    source_state: AvailabilityStateKind,
    current_schedules: &[CareScheduleView],
    my_agent: &str,
    duties: &[CareScheduleView],
) -> AvailabilityStateKind {
    if HoloHashBytes::from_agent_display(my_agent).is_err() {
        return AvailabilityStateKind::Degraded;
    }

    match source_state {
        AvailabilityStateKind::Degraded => return AvailabilityStateKind::Degraded,
        AvailabilityStateKind::Unavailable => return AvailabilityStateKind::Unavailable,
        AvailabilityStateKind::Unknown => return AvailabilityStateKind::Unknown,
        AvailabilityStateKind::Locked => return AvailabilityStateKind::Locked,
        AvailabilityStateKind::Mock => return AvailabilityStateKind::Mock,
        AvailabilityStateKind::Live | AvailabilityStateKind::Empty => {}
    }

    match care_state {
        AvailabilityStateKind::Degraded => return AvailabilityStateKind::Degraded,
        AvailabilityStateKind::Unavailable => return AvailabilityStateKind::Unavailable,
        AvailabilityStateKind::Unknown => return AvailabilityStateKind::Unknown,
        AvailabilityStateKind::Locked => return AvailabilityStateKind::Locked,
        AvailabilityStateKind::Mock => return AvailabilityStateKind::Mock,
        AvailabilityStateKind::Live | AvailabilityStateKind::Empty => {}
    }

    let expected = current_schedules
        .iter()
        .filter(|schedule| schedule.assigned_to == my_agent)
        .map(schedule_fingerprint)
        .collect::<BTreeSet<_>>();
    let actual = duties
        .iter()
        .map(schedule_fingerprint)
        .collect::<BTreeSet<_>>();

    if expected == actual {
        AvailabilityStateKind::Live
    } else {
        AvailabilityStateKind::Degraded
    }
}

/// Personal attention is narrower than source availability: a Live duties
/// source can legitimately contain only completed/paused work, or only duties
/// from other Hearths. Only Active duties in the current Hearth are surfaced as
/// attention after cross-source alignment is established.
pub fn care_attention_state(snapshot: &CareAttentionSnapshot) -> AvailabilityStateKind {
    match snapshot.availability {
        AvailabilityStateKind::Mock => AvailabilityStateKind::Mock,
        AvailabilityStateKind::Unknown => AvailabilityStateKind::Unknown,
        AvailabilityStateKind::Unavailable => AvailabilityStateKind::Unavailable,
        AvailabilityStateKind::Locked => AvailabilityStateKind::Locked,
        AvailabilityStateKind::Degraded => AvailabilityStateKind::Degraded,
        AvailabilityStateKind::Live | AvailabilityStateKind::Empty => {
            match snapshot.snapshot_alignment {
                AvailabilityStateKind::Live => {
                    if snapshot
                        .duties
                        .iter()
                        .any(|duty| duty.status == CareScheduleStatus::Active)
                    {
                        AvailabilityStateKind::Live
                    } else {
                        AvailabilityStateKind::Empty
                    }
                }
                AvailabilityStateKind::Unknown => AvailabilityStateKind::Unknown,
                AvailabilityStateKind::Unavailable => AvailabilityStateKind::Unavailable,
                AvailabilityStateKind::Locked => AvailabilityStateKind::Locked,
                AvailabilityStateKind::Degraded
                | AvailabilityStateKind::Empty
                | AvailabilityStateKind::Mock => AvailabilityStateKind::Degraded,
            }
        }
    }
}

pub fn care_attention_is_established_empty(snapshot: &CareAttentionSnapshot) -> bool {
    care_attention_state(snapshot) == AvailabilityStateKind::Empty
}

fn source_key(
    hearth_hash: &str,
    current_schedules: &[CareScheduleView],
    my_agent: &str,
) -> String {
    let mut keys = current_schedules
        .iter()
        .map(schedule_fingerprint)
        .collect::<Vec<_>>();
    keys.sort();
    format!("{hearth_hash}|{my_agent}|{}", keys.join(","))
}

fn invalidate(generation: &Rc<Cell<u64>>) {
    generation.set(generation.get().wrapping_add(1));
}

pub fn provide_care_attention() -> CareAttentionState {
    let state = CareAttentionState {
        snapshot: RwSignal::new(CareAttentionSnapshot::unknown()),
        loading: RwSignal::new(false),
    };
    provide_context(state.clone());

    let hearth = use_hearth();
    let truth = use_hearth_truth();
    let hc = use_holochain();
    let last_key = Rc::new(RefCell::new(None::<String>));
    let generation = Rc::new(Cell::new(0u64));

    let state_effect = state.clone();
    let hearth_effect = hearth.clone();
    let truth_effect = truth.clone();
    let hc_effect = hc.clone();
    let key_effect = last_key.clone();
    let generation_effect = generation.clone();

    Effect::new(move |_| {
        let status = hc_effect.status.get();
        let signer_ready = hc_effect.zome_call_signing_ready.get();
        let snapshot_loading = truth_effect.loading.get();
        let availability = truth_effect.availability.get();

        if status == ConnectionStatus::Mock {
            invalidate(&generation_effect);
            *key_effect.borrow_mut() = None;
            state_effect.loading.set(false);
            state_effect.snapshot.set(CareAttentionSnapshot {
                availability: AvailabilityStateKind::Mock,
                snapshot_alignment: AvailabilityStateKind::Mock,
                duties: Vec::new(),
            });
            return;
        }

        if status != ConnectionStatus::Connected || !signer_ready || snapshot_loading {
            invalidate(&generation_effect);
            *key_effect.borrow_mut() = None;
            state_effect.loading.set(false);
            let source_state = if status == ConnectionStatus::Connected && !signer_ready {
                AvailabilityStateKind::Unavailable
            } else if matches!(
                status,
                ConnectionStatus::Disconnected | ConnectionStatus::Reconnecting
            ) {
                AvailabilityStateKind::Degraded
            } else {
                AvailabilityStateKind::Unknown
            };
            state_effect.snapshot.update(|value| {
                value.availability = source_state.clone();
                value.snapshot_alignment = source_state;
            });
            return;
        }

        let Some(current_hearth) = hearth_effect.current_hearth.get() else {
            *key_effect.borrow_mut() = None;
            state_effect.snapshot.set(CareAttentionSnapshot::unknown());
            return;
        };
        if !matches!(
            availability.current_hearth,
            AvailabilityStateKind::Live | AvailabilityStateKind::Degraded
        ) {
            *key_effect.borrow_mut() = None;
            state_effect.snapshot.set(CareAttentionSnapshot::unknown());
            return;
        }

        let current_schedules = hearth_effect.care_schedules.get();
        let my_agent = hearth_effect.my_agent.get();
        if HoloHashBytes::from_action_raw_base64(&current_hearth.hash).is_err()
            || HoloHashBytes::from_agent_display(&my_agent).is_err()
        {
            invalidate(&generation_effect);
            *key_effect.borrow_mut() = None;
            state_effect.loading.set(false);
            state_effect.snapshot.set(CareAttentionSnapshot {
                availability: AvailabilityStateKind::Degraded,
                snapshot_alignment: AvailabilityStateKind::Degraded,
                duties: Vec::new(),
            });
            return;
        }

        let key = source_key(&current_hearth.hash, &current_schedules, &my_agent);
        if key_effect.borrow().as_deref() == Some(&key) {
            return;
        }
        *key_effect.borrow_mut() = Some(key);

        let current_hearth_hash = current_hearth.hash;
        let care_state = availability.care_schedules;

        invalidate(&generation_effect);
        let token = generation_effect.get();
        state_effect.loading.set(true);
        state_effect.snapshot.set(CareAttentionSnapshot::unknown());

        let state_load = state_effect.clone();
        let hc_load = hc_effect.clone();
        let generation_load = generation_effect.clone();
        spawn_local(async move {
            let result = hc_load
                .call_zome_default::<(), Vec<WireRecord>>(
                    "hearth_care",
                    "get_my_care_duties",
                    &(),
                )
                .await;

            if generation_load.get() != token {
                return;
            }

            match result {
                Ok(records) => {
                    let (source_state, duties) =
                        validate_source_records(&records, &current_hearth_hash, &my_agent);
                    let alignment = snapshot_alignment(
                        care_state,
                        source_state.clone(),
                        &current_schedules,
                        &my_agent,
                        &duties,
                    );
                    state_load.snapshot.set(CareAttentionSnapshot {
                        availability: source_state,
                        snapshot_alignment: alignment,
                        duties,
                    });
                }
                Err(error) => {
                    web_sys::console::log_1(
                        &format!("[Hearth] get_my_care_duties failed: {error}").into(),
                    );
                    state_load.snapshot.set(CareAttentionSnapshot {
                        availability: AvailabilityStateKind::Unavailable,
                        snapshot_alignment: AvailabilityStateKind::Unavailable,
                        duties: Vec::new(),
                    });
                }
            }
            state_load.loading.set(false);
        });
    });

    state
}

pub fn use_care_attention() -> CareAttentionState {
    expect_context::<CareAttentionState>()
}

#[component]
pub fn CareAttentionInbox() -> impl IntoView {
    let state = use_care_attention();
    let state_status = state.clone();
    let state_view = state.clone();

    view! {
        <section class="inbox-care-attention" aria-labelledby="inbox-care-heading">
            <h2 id="inbox-care-heading">"Care assigned to you"</h2>
            <p class="task-surface-note">
                "This source reports schedules linked to your connected AgentPubKey. It does not grant permission to complete, reassign, or otherwise mutate a care task."
            </p>
            <p class="inbox-source-state" role="status">
                {move || {
                    let value = state_status.snapshot.get();
                    format!(
                        "Source: {}{} · alignment with the loaded Care snapshot: {}",
                        value.availability.label(),
                        if state_status.loading.get() { " (loading)" } else { "" },
                        value.snapshot_alignment.label(),
                    )
                }}
            </p>
            {move || {
                let value = state_view.snapshot.get();
                match care_attention_state(&value) {
                    AvailabilityStateKind::Live => {
                        let active = value
                            .duties
                            .into_iter()
                            .filter(|duty| duty.status == CareScheduleStatus::Active)
                            .collect::<Vec<_>>();
                        view! {
                            <div class="task-directory" role="list">
                                {active.into_iter().map(|duty| view! {
                                    <A href="/care" attr:class="task-directory-item" attr:role="listitem">
                                        <strong>{duty.title}</strong>
                                        <span>"Active Care schedule assigned to you. Open Care for the source-backed task and any available controls."</span>
                                    </A>
                                }).collect_view()}
                            </div>
                        }.into_any()
                    }
                    AvailabilityStateKind::Empty => view! {
                        <p class="empty-state">"The Care-duty source and current Hearth snapshot agree that no Active care schedule is assigned to you here."</p>
                    }.into_any(),
                    AvailabilityStateKind::Degraded => view! {
                        <p class="empty-state" role="alert">"Personal Care attention is partial or disagrees with the loaded Care snapshot, so it is not being presented as complete."</p>
                    }.into_any(),
                    AvailabilityStateKind::Unavailable => view! {
                        <p class="empty-state" role="alert">"The source-backed personal Care-duty query is unavailable in this snapshot."</p>
                    }.into_any(),
                    AvailabilityStateKind::Locked => view! {
                        <p class="empty-state" role="status">"The personal Care-duty source is locked."</p>
                    }.into_any(),
                    AvailabilityStateKind::Mock => view! {
                        <p class="empty-state">"Demo mode does not fabricate a source-backed personal Care Inbox."</p>
                    }.into_any(),
                    AvailabilityStateKind::Unknown => view! {
                        <p class="empty-state">"Personal Care attention has not been established yet."</p>
                    }.into_any(),
                }
            }}
        </section>
    }
}

#[component]
pub fn CareHomeAttention() -> impl IntoView {
    let state = use_care_attention();
    view! {
        {move || {
            let value = state.snapshot.get();
            match care_attention_state(&value) {
                AvailabilityStateKind::Live => {
                    let count = value
                        .duties
                        .iter()
                        .filter(|duty| duty.status == CareScheduleStatus::Active)
                        .count();
                    view! {
                        <section class="home-attention" aria-labelledby="home-care-attention-heading">
                            <h2 id="home-care-attention-heading">"care attention"</h2>
                            <A href="/inbox" attr:class="nudge">
                                {format!(
                                    "{count} active care task{} assigned to you · inspect Inbox",
                                    if count == 1 { "" } else { "s" },
                                )}
                            </A>
                        </section>
                    }.into_any()
                }
                AvailabilityStateKind::Degraded | AvailabilityStateKind::Unavailable => view! {
                    <section class="home-attention" role="status">
                        <h2>"care attention source"</h2>
                        <p>"Personal Care attention is not fully established, so Home is not claiming your Care queue is clear."</p>
                    </section>
                }.into_any(),
                AvailabilityStateKind::Unknown | AvailabilityStateKind::Locked => view! {
                    <section class="home-attention" role="status">
                        <h2>"care attention source"</h2>
                        <p>"Personal Care attention is still being established or is not currently readable."</p>
                    </section>
                }.into_any(),
                AvailabilityStateKind::Empty | AvailabilityStateKind::Mock => {
                    view! { <></> }.into_any()
                }
            }
        }}
    }
}

#[cfg(test)]
mod tests {
    use super::{
        CareAttentionSnapshot, care_attention_is_established_empty, care_attention_state,
        snapshot_alignment,
    };
    use hearth_leptos_types::{CareScheduleStatus, CareScheduleView, CareType, Recurrence};
    use mycelix_leptos_core::AvailabilityStateKind;

    fn duty(hash: &str, assigned_to: &str, status: CareScheduleStatus) -> CareScheduleView {
        CareScheduleView {
            hash: hash.into(),
            hearth_hash: "hearth".into(),
            care_type: CareType::Chore,
            title: hash.into(),
            description: String::new(),
            assigned_to: assigned_to.into(),
            recurrence: Recurrence::Weekly,
            status,
            completed_at: None,
        }
    }

    fn snapshot(
        availability: AvailabilityStateKind,
        alignment: AvailabilityStateKind,
        duties: Vec<CareScheduleView>,
    ) -> CareAttentionSnapshot {
        CareAttentionSnapshot {
            availability,
            snapshot_alignment: alignment,
            duties,
        }
    }

    #[test]
    fn active_aligned_duty_is_attention() {
        let value = snapshot(
            AvailabilityStateKind::Live,
            AvailabilityStateKind::Live,
            vec![duty("a", "me", CareScheduleStatus::Active)],
        );
        assert_eq!(care_attention_state(&value), AvailabilityStateKind::Live);
    }

    #[test]
    fn completed_aligned_duty_is_not_active_attention() {
        let value = snapshot(
            AvailabilityStateKind::Live,
            AvailabilityStateKind::Live,
            vec![duty("a", "me", CareScheduleStatus::Completed)],
        );
        assert!(care_attention_is_established_empty(&value));
    }

    #[test]
    fn empty_source_with_bad_alignment_is_not_clear() {
        let value = snapshot(
            AvailabilityStateKind::Empty,
            AvailabilityStateKind::Degraded,
            Vec::new(),
        );
        assert_eq!(care_attention_state(&value), AvailabilityStateKind::Degraded);
    }

    #[test]
    fn current_snapshot_mismatch_degrades_alignment() {
        let local = vec![duty("a", "me", CareScheduleStatus::Active)];
        assert_eq!(
            snapshot_alignment(
                AvailabilityStateKind::Live,
                AvailabilityStateKind::Empty,
                &local,
                "me",
                &[],
            ),
            AvailabilityStateKind::Degraded
        );
    }
}
