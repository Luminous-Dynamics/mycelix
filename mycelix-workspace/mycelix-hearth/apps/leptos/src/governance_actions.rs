// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Explicit governance transitions that are distinct from first-time vote casting.
//!
//! Keeping amendment separate prevents the UI from pretending a second
//! `cast_vote` is a valid way to modify an existing vote. Live authority remains
//! entirely in the Hearth decisions/civic zomes.

use crate::hearth_context::{mock_now, use_hearth};
use crate::hearth_truth::use_hearth_truth;
use crate::record_bridge::{self, WireRecord};
use hearth_leptos_types::VoteView;
use mycelix_leptos_core::holochain_provider::use_holochain;
use mycelix_leptos_core::{AvailabilityStateKind, ToastKind, use_toasts};
use wasm_bindgen_futures::spawn_local;

pub fn amend_vote(
    decision_hash: String,
    choice: u32,
    reasoning: Option<String>,
    demo_weight_bp: u32,
) {
    let hc = use_holochain();
    let hearth = use_hearth();
    let truth = use_hearth_truth();
    let toasts = use_toasts();

    if hc.is_mock() {
        let my_agent = hearth.my_agent.get_untracked();
        hearth.votes.update(|votes| {
            votes.retain(|vote| {
                !(vote.decision_hash == decision_hash && vote.voter == my_agent)
            });
            votes.push(VoteView {
                decision_hash: decision_hash.clone(),
                voter: my_agent,
                choice,
                weight_bp: demo_weight_bp,
                reasoning,
                created_at: mock_now(),
            });
        });
        toasts.push(
            "demo vote updated",
            ToastKind::Custom("decision".into()),
        );
        return;
    }

    if !hc.zome_calls_ready_untracked() {
        toasts.push(
            "vote amendment needs a connected, authorized Hearth session",
            ToastKind::Custom("decision".into()),
        );
        return;
    }

    spawn_local(async move {
        #[derive(serde::Serialize)]
        struct AmendVoteInput {
            decision_hash: String,
            choice: u32,
            reasoning: Option<String>,
        }

        match hc
            .call_zome_default::<AmendVoteInput, WireRecord>(
                "hearth_decisions",
                "amend_vote",
                &AmendVoteInput {
                    decision_hash,
                    choice,
                    reasoning,
                },
            )
            .await
        {
            Ok(record) => {
                let mut decoded = record_bridge::records_to_votes(std::slice::from_ref(&record));
                if let Some(amended) = decoded.pop() {
                    hearth.votes.update(|votes| {
                        votes.retain(|vote| {
                            !(vote.decision_hash == amended.decision_hash
                                && vote.voter == amended.voter)
                        });
                        votes.push(amended);
                    });
                    toasts.push(
                        "vote amended",
                        ToastKind::Custom("decision".into()),
                    );
                } else {
                    truth.availability.update(|availability| {
                        availability.votes = AvailabilityStateKind::Degraded;
                    });
                    toasts.push(
                        "vote amendment was accepted, but the returned record could not be established",
                        ToastKind::Custom("decision".into()),
                    );
                }
            }
            Err(error) => {
                truth.availability.update(|availability| {
                    availability.votes = AvailabilityStateKind::Unknown;
                });
                web_sys::console::log_1(
                    &format!("amend_vote outcome unknown: {error}").into(),
                );
                toasts.push(
                    "couldn’t confirm whether the vote amendment committed; reconcile before retrying",
                    ToastKind::Custom("decision".into()),
                );
            }
        }
    });
}
