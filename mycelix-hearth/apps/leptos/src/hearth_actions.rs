// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Hearth action dispatch: the bridge between UI interactions and data.
//!
//! When connected to a conductor, dispatches real zome calls.
//! When in mock mode, mutates local RwSignal state directly.
//! The UI doesn't know the difference — it reads from the same signals.

use leptos::prelude::*;
use wasm_bindgen_futures::spawn_local;

use crate::hearth_context::{use_hearth, mock_now};
use crate::action_feedback::{feedback_copy, ActionFeedbackKind};
use crate::action_journal_store::{prepare_live_attempt, PreparedJournalAttempt};
use crate::action_lifecycle::ActionKind;
use crate::record_bridge::{self, WireRecord};
use mycelix_leptos_core::holochain_provider::use_holochain;
use mycelix_leptos_core::{use_toasts, ToastKind};
use hearth_leptos_types::*;

/// Shared action dispatcher.
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

/// Tend a bond: increase strength, update last_tended.
pub fn tend_bond(bond_hash: String) {
    let hc = use_holochain();
    let hearth = use_hearth();
    let toasts = use_toasts();

    if hc.is_mock() {
        let mut changed = false;
        hearth.bonds.update(|bonds| {
            if let Some(bond) = bonds.iter_mut().find(|b| b.hash == bond_hash) {
                bond.strength_bp = (bond.strength_bp + 500).min(BOND_MAX);
                bond.last_tended = mock_now();
                changed = true;
            }
        });
        toasts.push(
            if changed {
                feedback_copy(ActionFeedbackKind::PreviewOnly, "the bond update")
            } else {
                "Preview only: bond not found in sample data; nothing changed.".into()
            },
            ToastKind::Custom("bond".into()),
        );
        return;
    }

    let Some(bond_hash_bytes) = record_bridge::decode_holo_hash_b64(&bond_hash) else {
        toasts.push(
            "Bond update not sent: a verified Holochain bond identity is required.",
            ToastKind::Custom("bond".into()),
        );
        return;
    };

    let Some(hearth_scope_bytes) = hearth
        .current_hearth
        .get_untracked()
        .and_then(|h| record_bridge::decode_holo_hash_b64(&h.hash))
    else {
        toasts.push(
            "Bond update not sent: the selected household identity is not verified.",
            ToastKind::Custom("bond".into()),
        );
        return;
    };

    let publication_evidence = hearth.publication_evidence;
    let publication_evidence_generation = hearth.publication_evidence_generation;
    let action_journal = hearth.action_journal;
    let hc = hc.clone();
    spawn_local(async move {
        let journal_attempt = match prepare_live_attempt(
            action_journal,
            ActionKind::BondUpdate,
            &hearth_scope_bytes,
            &bond_hash_bytes,
        )
        .await
        {
            Ok(attempt) => attempt,
            Err(_) => {
                toasts.push(
                    "Bond update not sent: the durable local action journal is unavailable or could not commit.",
                    ToastKind::Custom("bond".into()),
                );
                return;
            }
        };

        #[derive(serde::Serialize)]
        struct TendBondInput {
            bond_hash: Vec<u8>,
            description: String,
            quality_bp: u32,
        }

        match hc.call_zome_default::<TendBondInput, WireRecord>(
            "hearth_kinship",
            "tend_bond",
            &TendBondInput {
                bond_hash: bond_hash_bytes,
                description: "tended with care".into(),
                quality_bp: 500,
            },
        ).await {
            Ok(updated_record) => {
                toasts.push(
                    feedback_copy(ActionFeedbackKind::SourceChainCommitted, "Bond update"),
                    ToastKind::Custom("bond".into()),
                );
                if !journal_attempt.mark_source_chain_committed().await {
                    toasts.push(
                        "Bond update returned successfully, but its local recovery record could not be committed. Do not repeat it until reconciled.",
                        ToastKind::Custom("bond".into()),
                    );
                }
                crate::publication_evidence::check_and_store(
                    &hc,
                    publication_evidence,
                    publication_evidence_generation,
                    &updated_record,
                    "Bond update",
                ).await;
            }
            Err(e) => {
                web_sys::console::log_1(&format!("tend_bond outcome unknown: {e}").into());
                toasts.push(
                    feedback_copy(ActionFeedbackKind::OutcomeUnknown, "the bond update"),
                    ToastKind::Custom("bond".into()),
                );
                if !journal_attempt.mark_outcome_unknown().await {
                    toasts.push(
                        "The bond update remains unresolved and its local recovery record could not be committed. Do not retry it.",
                        ToastKind::Custom("bond".into()),
                    );
                }
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
    let toasts = use_toasts();

    if hc.is_mock() {
        let from = hearth.my_agent.get_untracked();
        hearth.gratitude.update(|g| {
            g.insert(0, GratitudeExpressionView {
                hash: format!("grat_user_{}", g.len()),
                from_agent: from,
                to_agent,
                message,
                gratitude_type: GratitudeType::Appreciation,
                visibility: HearthVisibility::AllMembers,
                created_at: mock_now(),
            });
        });
        toasts.push(
            feedback_copy(ActionFeedbackKind::PreviewOnly, "the gratitude expression"),
            ToastKind::Custom("gratitude".into()),
        );
        return;
    }

    let Some(hearth_hash) = hearth.current_hearth.get_untracked()
        .and_then(|h| record_bridge::decode_holo_hash_b64(&h.hash))
    else {
        toasts.push(
            "Gratitude not sent: the current household identity is not verified.",
            ToastKind::Custom("gratitude".into()),
        );
        return;
    };
    let Some(to_agent_bytes) = record_bridge::decode_holo_hash_b64(&to_agent) else {
        toasts.push(
            "Gratitude not sent: choose a verified household member.",
            ToastKind::Custom("gratitude".into()),
        );
        return;
    };

    let publication_evidence = hearth.publication_evidence;
    let publication_evidence_generation = hearth.publication_evidence_generation;
    let hc = hc.clone();
    spawn_local(async move {
        #[derive(serde::Serialize)]
        struct ExpressGratitudeInput {
            hearth_hash: Vec<u8>,
            to_agent: Vec<u8>,
            message: String,
            gratitude_type: GratitudeType,
            visibility: HearthVisibility,
        }

        match hc.call_zome_default::<ExpressGratitudeInput, WireRecord>(
            "hearth_gratitude",
            "express_gratitude",
            &ExpressGratitudeInput {
                hearth_hash,
                to_agent: to_agent_bytes,
                message,
                gratitude_type: GratitudeType::Appreciation,
                visibility: HearthVisibility::AllMembers,
            },
        ).await {
            Ok(record) => {
                toasts.push(
                    feedback_copy(ActionFeedbackKind::SourceChainCommitted, "Gratitude expression"),
                    ToastKind::Custom("gratitude".into()),
                );
                crate::publication_evidence::check_and_store(
                    &hc,
                    publication_evidence,
                    publication_evidence_generation,
                    &record,
                    "Gratitude expression",
                ).await;
            },
            Err(e) => {
                web_sys::console::log_1(&format!("express_gratitude outcome unknown: {e}").into());
                toasts.push(
                    feedback_copy(ActionFeedbackKind::OutcomeUnknown, "the gratitude expression"),
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
    let toasts = use_toasts();

    if hc.is_mock() {
        let my_agent = hearth.my_agent.get_untracked();
        hearth.votes.update(|votes| {
            votes.retain(|v| !(v.decision_hash == decision_hash && v.voter == my_agent));
            votes.push(VoteView {
                decision_hash,
                voter: my_agent,
                choice,
                weight_bp,
                reasoning,
                created_at: mock_now(),
            });
        });
        toasts.push(
            feedback_copy(ActionFeedbackKind::PreviewOnly, "the vote"),
            ToastKind::Custom("decision".into()),
        );
        return;
    }

    let Some(decision_hash_bytes) = record_bridge::decode_holo_hash_b64(&decision_hash) else {
        toasts.push(
            "Vote not sent: a verified decision identity is required.",
            ToastKind::Custom("decision".into()),
        );
        return;
    };

    let publication_evidence = hearth.publication_evidence;
    let publication_evidence_generation = hearth.publication_evidence_generation;
    let hc = hc.clone();
    spawn_local(async move {
        #[derive(serde::Serialize)]
        struct CastVoteInput {
            decision_hash: Vec<u8>,
            choice: u32,
            reasoning: Option<String>,
        }

        match hc.call_zome_default::<CastVoteInput, WireRecord>(
            "hearth_decisions",
            "cast_vote",
            &CastVoteInput { decision_hash: decision_hash_bytes, choice, reasoning },
        ).await {
            Ok(record) => {
                toasts.push(
                    feedback_copy(ActionFeedbackKind::SourceChainCommitted, "Vote submission"),
                    ToastKind::Custom("decision".into()),
                );
                crate::publication_evidence::check_and_store(
                    &hc,
                    publication_evidence,
                    publication_evidence_generation,
                    &record,
                    "Vote submission",
                ).await;
            },
            Err(e) => {
                web_sys::console::log_1(&format!("cast_vote outcome unknown: {e}").into());
                toasts.push(
                    feedback_copy(ActionFeedbackKind::OutcomeUnknown, "the vote submission"),
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
    let toasts = use_toasts();

    if hc.is_mock() {
        let my_agent = hearth.my_agent.get_untracked();
        let mut changed = false;
        hearth.presence.update(|plist| {
            if let Some(p) = plist.iter_mut().find(|p| p.agent == my_agent) {
                p.status = new_status;
                p.updated_at = mock_now();
                changed = true;
            }
        });
        toasts.push(
            if changed {
                feedback_copy(ActionFeedbackKind::PreviewOnly, "the presence update")
            } else {
                "Preview only: no matching sample presence record was found; nothing changed.".into()
            },
            ToastKind::Custom("presence".into()),
        );
        return;
    }

    let Some(hearth_hash) = hearth.current_hearth.get_untracked()
        .and_then(|h| record_bridge::decode_holo_hash_b64(&h.hash))
    else {
        toasts.push(
            "Presence update not sent: the current household identity is not verified.",
            ToastKind::Custom("presence".into()),
        );
        return;
    };

    let publication_evidence = hearth.publication_evidence;
    let publication_evidence_generation = hearth.publication_evidence_generation;
    let hc = hc.clone();
    spawn_local(async move {
        #[derive(serde::Serialize)]
        struct SetPresenceInput {
            hearth_hash: Vec<u8>,
            status: PresenceStatusType,
            expected_return: Option<i64>,
        }

        match hc.call_zome_default::<SetPresenceInput, WireRecord>(
            "hearth_rhythms",
            "set_presence",
            &SetPresenceInput {
                hearth_hash,
                status: new_status,
                expected_return: None,
            },
        ).await {
            Ok(record) => {
                toasts.push(
                    feedback_copy(ActionFeedbackKind::SourceChainCommitted, "Presence update"),
                    ToastKind::Custom("presence".into()),
                );
                crate::publication_evidence::check_and_store(
                    &hc,
                    publication_evidence,
                    publication_evidence_generation,
                    &record,
                    "Presence update",
                ).await;
            },
            Err(e) => {
                web_sys::console::log_1(&format!("set_presence outcome unknown: {e}").into());
                toasts.push(
                    feedback_copy(ActionFeedbackKind::OutcomeUnknown, "the presence update"),
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
    let toasts = use_toasts();

    if hc.is_mock() {
        let exists = hearth.care_schedules.get_untracked().iter().any(|task| task.hash == task_hash);
        if !exists {
            toasts.push(
                "Preview only: care item not found in sample data; nothing changed.",
                ToastKind::Custom("care".into()),
            );
            return;
        }

        hearth.care_schedules.update(|schedules| {
            if let Some(task) = schedules.iter_mut().find(|s| s.hash == task_hash) {
                task.status = CareScheduleStatus::Completed;
                task.completed_at = Some(mock_now());
            }
        });
        if let Some(set_care) = use_context::<WriteSignal<u32>>() {
            let active = hearth.care_schedules.get_untracked().iter()
                .filter(|c| c.status == CareScheduleStatus::Active)
                .count() as u32;
            set_care.set(active);
        }
        toasts.push(
            feedback_copy(ActionFeedbackKind::PreviewOnly, "the care task completion"),
            ToastKind::Custom("care".into()),
        );
        return;
    }

    let Some(schedule_hash_bytes) = record_bridge::decode_holo_hash_b64(&task_hash) else {
        toasts.push(
            "Care completion not sent: a verified care-schedule identity is required.",
            ToastKind::Custom("care".into()),
        );
        return;
    };

    let publication_evidence = hearth.publication_evidence;
    let publication_evidence_generation = hearth.publication_evidence_generation;
    let hc = hc.clone();
    spawn_local(async move {
        #[derive(serde::Serialize)]
        struct CompleteTaskInput { schedule_hash: Vec<u8> }

        match hc.call_zome_default::<CompleteTaskInput, WireRecord>(
            "hearth_care",
            "complete_task",
            &CompleteTaskInput { schedule_hash: schedule_hash_bytes },
        ).await {
            Ok(record) => {
                toasts.push(
                    feedback_copy(ActionFeedbackKind::SourceChainCommitted, "Care task completion"),
                    ToastKind::Custom("care".into()),
                );
                crate::publication_evidence::check_and_store(
                    &hc,
                    publication_evidence,
                    publication_evidence_generation,
                    &record,
                    "Care task completion",
                ).await;
            },
            Err(e) => {
                web_sys::console::log_1(&format!("complete_task outcome unknown: {e}").into());
                toasts.push(
                    feedback_copy(ActionFeedbackKind::OutcomeUnknown, "the care task completion"),
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
                display_name,
                role,
                status: MembershipStatus::Invited,
                joined_at: mock_now(),
            });
        });
        toasts.push(
            feedback_copy(ActionFeedbackKind::PreviewOnly, "the sample invitation"),
            ToastKind::Custom("presence".into()),
        );
    } else {
        // The current UI only supplies a display name, not a verified AgentPubKey.
        // Do not submit a human-readable name where an authenticated identity is required.
        toasts.push(
            feedback_copy(
                ActionFeedbackKind::NotSent { requirement: "a verified recipient identity" },
                "this invitation",
            ),
            ToastKind::Custom("presence".into()),
        );
    }
}
