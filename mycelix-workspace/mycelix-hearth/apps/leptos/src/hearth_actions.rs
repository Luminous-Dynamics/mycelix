// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Hearth action dispatch: the bridge between UI interactions and data.
//!
//! Mock mode mutates demo signals directly. Live mode treats the zome-returned
//! `Record` as the authoritative write result and reconciles that exact record
//! into the local snapshot. A transport/response error after a write attempt is
//! treated as an unknown outcome, not proof that the mutation did not happen.

use leptos::prelude::*;
use wasm_bindgen_futures::spawn_local;

use crate::hearth_context::{member_name, mock_now, use_hearth};
use crate::hearth_truth::use_hearth_truth;
use crate::record_bridge::{self, WireRecord};
use hearth_leptos_types::*;
use mycelix_leptos_core::holochain_provider::use_holochain;
use mycelix_leptos_core::{AvailabilityStateKind, ToastKind, use_toasts};

#[derive(Clone)]
#[allow(dead_code)]
pub struct HearthActions {
    is_mock: bool,
}

pub fn provide_hearth_actions() -> HearthActions {
    let hc = use_holochain();
    let actions = HearthActions {
        is_mock: hc.is_mock(),
    };
    provide_context(actions.clone());
    actions
}

pub fn use_hearth_actions() -> HearthActions {
    expect_context::<HearthActions>()
}

// ============================================================================
// Bond Tending
// ============================================================================

pub fn tend_bond(bond_hash: String) {
    let hc = use_holochain();
    let hearth = use_hearth();
    let truth = use_hearth_truth();
    let toasts = use_toasts();

    if hc.is_mock() {
        let members = hearth.members.get_untracked();
        hearth.bonds.update(|bonds| {
            if let Some(bond) = bonds.iter_mut().find(|bond| bond.hash == bond_hash) {
                bond.strength_bp = (bond.strength_bp + 500).min(BOND_MAX);
                bond.last_tended = mock_now();
                let name_a = member_name(&members, &bond.member_a);
                let name_b = member_name(&members, &bond.member_b);
                toasts.push(
                    format!("you tended the bond between {name_a} and {name_b}"),
                    ToastKind::Custom("bond".into()),
                );
            }
        });
        return;
    }

    if !hc.zome_calls_ready_untracked() {
        toasts.push(
            "bond tending needs a connected, authorized Hearth session",
            ToastKind::Custom("bond".into()),
        );
        return;
    }

    spawn_local(async move {
        #[derive(serde::Serialize)]
        struct TendBondInput {
            bond_hash: String,
            description: String,
            quality_bp: u32,
        }

        let original_hash = bond_hash.clone();
        match hc
            .call_zome_default::<TendBondInput, WireRecord>(
                "hearth_kinship",
                "tend_bond",
                &TendBondInput {
                    bond_hash,
                    description: "tended with care".into(),
                    quality_bp: 500,
                },
            )
            .await
        {
            Ok(record) => {
                let mut decoded = record_bridge::records_to_bonds(std::slice::from_ref(&record));
                if let Some(updated) = decoded.pop() {
                    hearth.bonds.update(|bonds| {
                        if let Some(index) = bonds.iter().position(|bond| bond.hash == original_hash) {
                            bonds[index] = updated;
                        } else {
                            bonds.push(updated);
                        }
                    });
                    toasts.push("bond tended", ToastKind::Custom("bond".into()));
                } else {
                    truth.availability.update(|availability| {
                        availability.bonds = AvailabilityStateKind::Degraded;
                    });
                    toasts.push(
                        "bond update was accepted, but the returned record could not be established",
                        ToastKind::Custom("bond".into()),
                    );
                }
            }
            Err(error) => {
                truth.availability.update(|availability| {
                    availability.bonds = AvailabilityStateKind::Unknown;
                });
                web_sys::console::log_1(&format!("tend_bond outcome unknown: {error}").into());
                toasts.push(
                    "couldn’t confirm whether the bond update committed; reconcile before retrying",
                    ToastKind::Custom("bond".into()),
                );
            }
        }
    });
}

// ============================================================================
// Express Gratitude
// ============================================================================

pub fn express_gratitude(to_agent: String, message: String) {
    let hc = use_holochain();
    let hearth = use_hearth();
    let truth = use_hearth_truth();
    let toasts = use_toasts();

    if hc.is_mock() {
        let from = hearth.my_agent.get_untracked();
        let members = hearth.members.get_untracked();
        let from_name = member_name(&members, &from);
        let to_name = member_name(&members, &to_agent);
        let msg_preview = message.clone();

        hearth.gratitude.update(|gratitude| {
            gratitude.insert(
                0,
                GratitudeExpressionView {
                    hash: format!("grat_user_{}", gratitude.len()),
                    from_agent: from,
                    to_agent: to_agent.clone(),
                    message,
                    gratitude_type: GratitudeType::Appreciation,
                    visibility: HearthVisibility::AllMembers,
                    created_at: mock_now(),
                },
            );
        });

        toasts.push(
            format!("{from_name} → {to_name}: “{msg_preview}”"),
            ToastKind::Custom("gratitude".into()),
        );
        return;
    }

    if !hc.zome_calls_ready_untracked() {
        toasts.push(
            "expressing gratitude needs a connected, authorized Hearth session",
            ToastKind::Custom("gratitude".into()),
        );
        return;
    }

    spawn_local(async move {
        #[derive(serde::Serialize)]
        struct ExpressGratitudeInput {
            hearth_hash: String,
            to_agent: String,
            message: String,
            gratitude_type: GratitudeType,
            visibility: HearthVisibility,
        }

        let hearth_hash = hearth
            .current_hearth
            .get_untracked()
            .map(|hearth| hearth.hash)
            .unwrap_or_default();

        match hc
            .call_zome_default::<ExpressGratitudeInput, WireRecord>(
                "hearth_gratitude",
                "express_gratitude",
                &ExpressGratitudeInput {
                    hearth_hash,
                    to_agent,
                    message,
                    gratitude_type: GratitudeType::Appreciation,
                    visibility: HearthVisibility::AllMembers,
                },
            )
            .await
        {
            Ok(record) => {
                let mut decoded =
                    record_bridge::records_to_gratitude(std::slice::from_ref(&record));
                if let Some(created) = decoded.pop() {
                    hearth.gratitude.update(|gratitude| {
                        gratitude.retain(|item| item.hash != created.hash);
                        gratitude.insert(0, created);
                    });
                    toasts.push(
                        "gratitude expressed",
                        ToastKind::Custom("gratitude".into()),
                    );
                } else {
                    truth.availability.update(|availability| {
                        availability.gratitude = AvailabilityStateKind::Degraded;
                    });
                    toasts.push(
                        "gratitude was accepted, but the returned record could not be established",
                        ToastKind::Custom("gratitude".into()),
                    );
                }
            }
            Err(error) => {
                truth.availability.update(|availability| {
                    availability.gratitude = AvailabilityStateKind::Unknown;
                });
                web_sys::console::log_1(
                    &format!("express_gratitude outcome unknown: {error}").into(),
                );
                toasts.push(
                    "couldn’t confirm whether gratitude was recorded; reconcile before retrying",
                    ToastKind::Custom("gratitude".into()),
                );
            }
        }
    });
}

// ============================================================================
// Cast Vote
// ============================================================================

pub fn cast_vote(decision_hash: String, choice: u32, reasoning: Option<String>, weight_bp: u32) {
    let hc = use_holochain();
    let hearth = use_hearth();
    let truth = use_hearth_truth();
    let toasts = use_toasts();

    if hc.is_mock() {
        let my_agent = hearth.my_agent.get_untracked();
        hearth.votes.update(|votes| {
            votes.retain(|vote| !(vote.decision_hash == decision_hash && vote.voter == my_agent));
            votes.push(VoteView {
                decision_hash: decision_hash.clone(),
                voter: my_agent,
                choice,
                weight_bp,
                reasoning,
                created_at: mock_now(),
            });
        });
        toasts.push("vote cast", ToastKind::Custom("decision".into()));
        return;
    }

    if !hc.zome_calls_ready_untracked() {
        toasts.push(
            "voting needs a connected, authorized Hearth session",
            ToastKind::Custom("decision".into()),
        );
        return;
    }

    spawn_local(async move {
        #[derive(serde::Serialize)]
        struct CastVoteInput {
            decision_hash: String,
            choice: u32,
            reasoning: Option<String>,
        }

        match hc
            .call_zome_default::<CastVoteInput, WireRecord>(
                "hearth_decisions",
                "cast_vote",
                &CastVoteInput {
                    decision_hash,
                    choice,
                    reasoning,
                },
            )
            .await
        {
            Ok(record) => {
                let mut decoded = record_bridge::records_to_votes(std::slice::from_ref(&record));
                if let Some(created) = decoded.pop() {
                    hearth.votes.update(|votes| {
                        votes.retain(|vote| {
                            !(vote.decision_hash == created.decision_hash
                                && vote.voter == created.voter)
                        });
                        votes.push(created);
                    });
                    toasts.push("vote cast", ToastKind::Custom("decision".into()));
                } else {
                    truth.availability.update(|availability| {
                        availability.votes = AvailabilityStateKind::Degraded;
                    });
                    toasts.push(
                        "vote was accepted, but the returned record could not be established",
                        ToastKind::Custom("decision".into()),
                    );
                }
            }
            Err(error) => {
                truth.availability.update(|availability| {
                    availability.votes = AvailabilityStateKind::Unknown;
                });
                web_sys::console::log_1(&format!("cast_vote outcome unknown: {error}").into());
                toasts.push(
                    "couldn’t confirm whether the vote was recorded; reconcile before retrying",
                    ToastKind::Custom("decision".into()),
                );
            }
        }
    });
}

// ============================================================================
// Change Presence
// ============================================================================

pub fn change_presence(new_status: PresenceStatusType) {
    let hc = use_holochain();
    let hearth = use_hearth();
    let truth = use_hearth_truth();
    let toasts = use_toasts();

    if hc.is_mock() {
        let my_agent = hearth.my_agent.get_untracked();
        hearth.presence.update(|presence| {
            if let Some(item) = presence.iter_mut().find(|item| item.agent == my_agent) {
                item.status = new_status;
                item.updated_at = mock_now();
            }
        });
        return;
    }

    if !hc.zome_calls_ready_untracked() {
        toasts.push(
            "presence changes need a connected, authorized Hearth session",
            ToastKind::Custom("presence".into()),
        );
        return;
    }

    spawn_local(async move {
        #[derive(serde::Serialize)]
        struct SetPresenceInput {
            hearth_hash: String,
            status: PresenceStatusType,
            expected_return: Option<i64>,
        }

        let hearth_hash = hearth
            .current_hearth
            .get_untracked()
            .map(|hearth| hearth.hash)
            .unwrap_or_default();

        match hc
            .call_zome_default::<SetPresenceInput, WireRecord>(
                "hearth_rhythms",
                "set_presence",
                &SetPresenceInput {
                    hearth_hash,
                    status: new_status,
                    expected_return: None,
                },
            )
            .await
        {
            Ok(record) => {
                let decoded = record_bridge::records_to_presence(std::slice::from_ref(&record));
                if let Some(updated) = decoded.views.into_iter().next() {
                    hearth.presence.update(|presence| {
                        if let Some(index) = presence
                            .iter()
                            .position(|item| item.agent == updated.agent)
                        {
                            presence[index] = updated;
                        } else {
                            presence.push(updated);
                        }
                    });
                    toasts.push(
                        "presence updated",
                        ToastKind::Custom("presence".into()),
                    );
                } else {
                    truth.availability.update(|availability| {
                        availability.presence = AvailabilityStateKind::Degraded;
                    });
                    toasts.push(
                        "presence update was accepted, but the returned record could not be established",
                        ToastKind::Custom("presence".into()),
                    );
                }
            }
            Err(error) => {
                truth.availability.update(|availability| {
                    availability.presence = AvailabilityStateKind::Unknown;
                });
                web_sys::console::log_1(
                    &format!("change_presence outcome unknown: {error}").into(),
                );
                toasts.push(
                    "couldn’t confirm whether presence changed; reconcile before retrying",
                    ToastKind::Custom("presence".into()),
                );
            }
        }
    });
}

// ============================================================================
// Complete Care Task
// ============================================================================

pub fn complete_care_task(task_hash: String) {
    let hc = use_holochain();
    let hearth = use_hearth();
    let truth = use_hearth_truth();
    let toasts = use_toasts();

    if hc.is_mock() {
        hearth.care_schedules.update(|schedules| {
            if let Some(task) = schedules.iter_mut().find(|schedule| schedule.hash == task_hash) {
                task.status = CareScheduleStatus::Completed;
                task.completed_at = Some(mock_now());
            }
        });
        toasts.push("task completed", ToastKind::Custom("care".into()));
        return;
    }

    if !hc.zome_calls_ready_untracked() {
        toasts.push(
            "completing care work needs a connected, authorized Hearth session",
            ToastKind::Custom("care".into()),
        );
        return;
    }

    spawn_local(async move {
        #[derive(serde::Serialize)]
        struct CompleteTaskInput {
            schedule_hash: String,
        }

        let original_hash = task_hash.clone();
        match hc
            .call_zome_default::<CompleteTaskInput, WireRecord>(
                "hearth_care",
                "complete_task",
                &CompleteTaskInput {
                    schedule_hash: task_hash,
                },
            )
            .await
        {
            Ok(record) => {
                let mut decoded =
                    record_bridge::records_to_care_schedules(std::slice::from_ref(&record));
                if let Some(updated) = decoded.pop() {
                    hearth.care_schedules.update(|schedules| {
                        if let Some(index) = schedules
                            .iter()
                            .position(|schedule| schedule.hash == original_hash)
                        {
                            schedules[index] = updated;
                        } else {
                            schedules.push(updated);
                        }
                    });
                    toasts.push("task completed", ToastKind::Custom("care".into()));
                } else {
                    truth.availability.update(|availability| {
                        availability.care_schedules = AvailabilityStateKind::Degraded;
                    });
                    toasts.push(
                        "care completion was accepted, but the returned record could not be established",
                        ToastKind::Custom("care".into()),
                    );
                }
            }
            Err(error) => {
                truth.availability.update(|availability| {
                    availability.care_schedules = AvailabilityStateKind::Unknown;
                });
                web_sys::console::log_1(
                    &format!("complete_care_task outcome unknown: {error}").into(),
                );
                toasts.push(
                    "couldn’t confirm whether the care task completed; reconcile before retrying",
                    ToastKind::Custom("care".into()),
                );
            }
        }
    });
}

// ============================================================================
// Invite Member
// ============================================================================

pub fn invite_member(display_name: String, role: MemberRole) {
    let hc = use_holochain();
    let hearth = use_hearth();
    let toasts = use_toasts();

    if hc.is_mock() {
        let agent_key = format!("agent_{}", display_name.to_lowercase().replace(' ', "_"));
        hearth.members.update(|members| {
            members.push(MemberView {
                agent: agent_key,
                display_name: display_name.clone(),
                role,
                status: MembershipStatus::Invited,
                joined_at: mock_now(),
            });
        });
        toasts.push(
            format!("invited {display_name} to the hearth"),
            ToastKind::Custom("presence".into()),
        );
        return;
    }

    if !hc.zome_calls_ready_untracked() {
        toasts.push(
            "inviting a member needs a connected, authorized Hearth session",
            ToastKind::Custom("presence".into()),
        );
        return;
    }

    // The zome requires a real AgentPubKey. A display name is not an identity
    // and must never be serialized as an AgentPubKey placeholder.
    toasts.push(
        format!(
            "No invitation was sent for {display_name}: live invitations require a verified agent identity"
        ),
        ToastKind::Custom("presence".into()),
    );
}
