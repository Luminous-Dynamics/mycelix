// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Explicit governance transitions that are distinct from first-time vote casting.
//!
//! Browser-side validation shapes inputs and prevents malformed wire values from
//! being dispatched. Live authority remains entirely in the Hearth decisions and
//! civic zomes.

use crate::hearth_context::{mock_now, use_hearth};
use crate::hearth_truth::use_hearth_truth;
use crate::record_bridge::{self, WireRecord};
use hearth_leptos_types::{DecisionStatus, DecisionType, DecisionView, MemberRole, VoteView};
use mycelix_leptos_client::HoloHashBytes;
use mycelix_leptos_core::holochain_provider::use_holochain;
use mycelix_leptos_core::{AvailabilityStateKind, ToastKind, use_toasts};
use wasm_bindgen_futures::spawn_local;

#[derive(Clone)]
pub struct CreateDecisionDraft {
    pub title: String,
    pub description: String,
    pub decision_type: DecisionType,
    pub eligible_roles: Vec<MemberRole>,
    pub options: Vec<String>,
    pub voting_window_days: u32,
    pub quorum_bp: Option<u32>,
}

fn normalize_draft(mut draft: CreateDecisionDraft) -> CreateDecisionDraft {
    draft.title = draft.title.trim().to_string();
    draft.description = draft.description.trim().to_string();
    draft.options = draft
        .options
        .into_iter()
        .map(|option| option.trim().to_string())
        .filter(|option| !option.is_empty())
        .collect();
    draft
}

fn validate_create_decision_draft(draft: &CreateDecisionDraft) -> Result<(), &'static str> {
    if draft.title.is_empty() {
        return Err("Decision title cannot be empty");
    }
    if draft.title.len() > 256 {
        return Err("Decision title must be 256 characters or fewer");
    }
    if draft.description.len() > 4096 {
        return Err("Decision description must be 4096 characters or fewer");
    }
    if draft.options.len() < 2 {
        return Err("A decision needs at least two options");
    }
    if draft.options.len() > 20 {
        return Err("A decision can have at most 20 options");
    }
    if draft.options.iter().any(|option| option.len() > 1024) {
        return Err("Each decision option must be 1024 characters or fewer");
    }
    if draft.eligible_roles.is_empty() {
        return Err("Choose at least one eligible Hearth role");
    }
    if draft.voting_window_days == 0 {
        return Err("Voting window must be longer than zero days");
    }
    if draft.quorum_bp.is_some_and(|quorum| quorum > 10_000) {
        return Err("Quorum cannot exceed 100 percent");
    }
    Ok(())
}

fn browser_deadline_micros(voting_window_days: u32) -> Result<i64, &'static str> {
    let now_micros = (js_sys::Date::now() * 1_000.0) as i64;
    let window_micros = i64::from(voting_window_days)
        .checked_mul(86_400_000_000)
        .ok_or("Voting window is too large")?;
    now_micros
        .checked_add(window_micros)
        .ok_or("Voting deadline is outside the supported timestamp range")
}

/// Schedule a Decision creation attempt.
///
/// A `true` return means only that demo creation was accepted locally or that a
/// live zome call was scheduled. It does not claim authorization or commit
/// success; live success is reported only after the returned Record is decoded.
pub fn create_decision(draft: CreateDecisionDraft) -> bool {
    let draft = normalize_draft(draft);
    let hc = use_holochain();
    let hearth = use_hearth();
    let truth = use_hearth_truth();
    let toasts = use_toasts();

    if let Err(message) = validate_create_decision_draft(&draft) {
        toasts.push(message, ToastKind::Custom("decision".into()));
        return false;
    }

    if hc.is_mock() {
        let created_at = mock_now();
        let deadline = created_at.saturating_add(i64::from(draft.voting_window_days) * 86_400);
        let hearth_hash = hearth
            .current_hearth
            .get_untracked()
            .map(|value| value.hash)
            .unwrap_or_else(|| "demo-hearth".to_string());
        let created_by = hearth.my_agent.get_untracked();

        hearth.decisions.update(|decisions| {
            decisions.push(DecisionView {
                hash: format!("decision_user_{}", decisions.len()),
                hearth_hash,
                title: draft.title,
                description: draft.description,
                decision_type: draft.decision_type,
                eligible_roles: draft.eligible_roles,
                options: draft.options,
                deadline,
                quorum_bp: draft.quorum_bp,
                status: DecisionStatus::Open,
                created_by,
                created_at,
            });
        });
        toasts.push(
            "demo decision created",
            ToastKind::Custom("decision".into()),
        );
        return true;
    }

    if !hc.zome_calls_ready_untracked() {
        toasts.push(
            "creating a decision needs a connected, authorized Hearth session",
            ToastKind::Custom("decision".into()),
        );
        return false;
    }

    let Some(hearth_hash_text) = hearth.current_hearth.get_untracked().map(|value| value.hash) else {
        truth.availability.update(|availability| {
            availability.current_hearth = AvailabilityStateKind::Unknown;
            availability.decisions = AvailabilityStateKind::Unknown;
        });
        toasts.push(
            "current Hearth is not established; refresh before creating a decision",
            ToastKind::Custom("decision".into()),
        );
        return false;
    };

    let hearth_hash = match HoloHashBytes::from_action_raw_base64(&hearth_hash_text) {
        Ok(hash) => hash,
        Err(error) => {
            truth.availability.update(|availability| {
                availability.current_hearth = AvailabilityStateKind::Degraded;
                availability.decisions = AvailabilityStateKind::Degraded;
            });
            web_sys::console::log_1(
                &format!("create_decision blocked by invalid Hearth ActionHash: {error}").into(),
            );
            toasts.push(
                "current Hearth target is malformed or is not an ActionHash; refresh before retrying",
                ToastKind::Custom("decision".into()),
            );
            return false;
        }
    };

    let deadline = match browser_deadline_micros(draft.voting_window_days) {
        Ok(deadline) => deadline,
        Err(message) => {
            toasts.push(message, ToastKind::Custom("decision".into()));
            return false;
        }
    };

    spawn_local(async move {
        #[derive(serde::Serialize)]
        struct CreateDecisionInput {
            hearth_hash: HoloHashBytes,
            title: String,
            description: String,
            decision_type: DecisionType,
            eligible_roles: Vec<MemberRole>,
            options: Vec<String>,
            deadline: i64,
            quorum_bp: Option<u32>,
        }

        match hc
            .call_zome_default::<CreateDecisionInput, WireRecord>(
                "hearth_decisions",
                "create_decision",
                &CreateDecisionInput {
                    hearth_hash,
                    title: draft.title,
                    description: draft.description,
                    decision_type: draft.decision_type,
                    eligible_roles: draft.eligible_roles,
                    options: draft.options,
                    deadline,
                    quorum_bp: draft.quorum_bp,
                },
            )
            .await
        {
            Ok(record) => {
                let mut decoded =
                    record_bridge::records_to_decisions(std::slice::from_ref(&record));
                if let Some(created) = decoded.pop() {
                    hearth.decisions.update(|decisions| {
                        decisions.retain(|item| item.hash != created.hash);
                        decisions.push(created);
                    });
                    truth.availability.update(|availability| {
                        if availability.decisions == AvailabilityStateKind::Empty {
                            availability.decisions = AvailabilityStateKind::Live;
                        }
                    });
                    toasts.push(
                        "decision created",
                        ToastKind::Custom("decision".into()),
                    );
                } else {
                    truth.availability.update(|availability| {
                        availability.decisions = AvailabilityStateKind::Degraded;
                    });
                    toasts.push(
                        "decision was accepted, but the returned record could not be established",
                        ToastKind::Custom("decision".into()),
                    );
                }
            }
            Err(error) => {
                truth.availability.update(|availability| {
                    availability.decisions = AvailabilityStateKind::Unknown;
                });
                web_sys::console::log_1(
                    &format!("create_decision outcome unknown: {error}").into(),
                );
                toasts.push(
                    "couldn’t confirm whether the decision was created; reconcile before retrying",
                    ToastKind::Custom("decision".into()),
                );
            }
        }
    });

    true
}

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

    let decision_hash = match HoloHashBytes::from_action_raw_base64(&decision_hash) {
        Ok(hash) => hash,
        Err(error) => {
            truth.availability.update(|availability| {
                availability.decisions = AvailabilityStateKind::Degraded;
                availability.votes = AvailabilityStateKind::Degraded;
            });
            web_sys::console::log_1(
                &format!("amend_vote blocked by invalid ActionHash: {error}").into(),
            );
            toasts.push(
                "decision target is malformed or is not an ActionHash; refresh before retrying",
                ToastKind::Custom("decision".into()),
            );
            return;
        }
    };

    spawn_local(async move {
        #[derive(serde::Serialize)]
        struct AmendVoteInput {
            decision_hash: HoloHashBytes,
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

#[cfg(test)]
mod tests {
    use super::{CreateDecisionDraft, normalize_draft, validate_create_decision_draft};
    use hearth_leptos_types::{DecisionType, MemberRole};

    fn valid_draft() -> CreateDecisionDraft {
        CreateDecisionDraft {
            title: "Weekend meal".into(),
            description: String::new(),
            decision_type: DecisionType::MajorityVote,
            eligible_roles: vec![MemberRole::Adult],
            options: vec!["Pizza".into(), "Tacos".into()],
            voting_window_days: 3,
            quorum_bp: Some(5000),
        }
    }

    #[test]
    fn local_shape_validation_matches_integrity_limits() {
        assert!(validate_create_decision_draft(&valid_draft()).is_ok());

        let mut empty = valid_draft();
        empty.title.clear();
        assert!(validate_create_decision_draft(&empty).is_err());

        let mut title = valid_draft();
        title.title = "x".repeat(257);
        assert!(validate_create_decision_draft(&title).is_err());

        let mut description = valid_draft();
        description.description = "x".repeat(4097);
        assert!(validate_create_decision_draft(&description).is_err());

        let mut too_few = valid_draft();
        too_few.options = vec!["only".into()];
        assert!(validate_create_decision_draft(&too_few).is_err());

        let mut too_many = valid_draft();
        too_many.options = (0..21).map(|i| format!("Option {i}")).collect();
        assert!(validate_create_decision_draft(&too_many).is_err());

        let mut no_roles = valid_draft();
        no_roles.eligible_roles.clear();
        assert!(validate_create_decision_draft(&no_roles).is_err());

        let mut quorum = valid_draft();
        quorum.quorum_bp = Some(10_001);
        assert!(validate_create_decision_draft(&quorum).is_err());
    }

    #[test]
    fn normalization_trims_and_drops_blank_options() {
        let mut draft = valid_draft();
        draft.title = "  Weekend meal  ".into();
        draft.options = vec!["  Pizza ".into(), "".into(), "   ".into(), "Tacos".into()];
        let normalized = normalize_draft(draft);
        assert_eq!(normalized.title, "Weekend meal");
        assert_eq!(normalized.options, vec!["Pizza", "Tacos"]);
    }
}
