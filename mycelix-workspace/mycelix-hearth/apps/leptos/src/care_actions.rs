// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Care-specific creation transitions.
//!
//! The browser validates only presentation/input-shape constraints. Live
//! authorization remains entirely in `hearth_care.create_care_schedule`, which
//! independently applies civic gating and Hearth-membership requirements.

use crate::hearth_context::{mock_now, use_hearth};
use crate::hearth_truth::use_hearth_truth;
use crate::record_bridge::{self, WireRecord};
use hearth_leptos_types::{CareScheduleStatus, CareScheduleView, CareType, Recurrence};
use mycelix_leptos_client::HoloHashBytes;
use mycelix_leptos_core::holochain_provider::use_holochain;
use mycelix_leptos_core::{AvailabilityStateKind, ToastKind, use_toasts};
use wasm_bindgen_futures::spawn_local;

#[derive(Clone)]
pub struct CreateCareDraft {
    pub care_type: CareType,
    pub title: String,
    pub description: String,
    pub assigned_to: String,
    pub recurrence: Recurrence,
    pub notes: String,
}

fn validate_draft(draft: &CreateCareDraft) -> Result<(), &'static str> {
    if draft.title.is_empty() {
        return Err("Care task title cannot be empty");
    }
    if draft.title.len() > 256 {
        return Err("Care task title must be 256 characters or fewer");
    }
    if draft.description.len() > 4096 {
        return Err("Care task description must be 4096 characters or fewer");
    }
    if draft.notes.len() > 4096 {
        return Err("Care task notes must be 4096 characters or fewer");
    }
    if draft.assigned_to.is_empty() {
        return Err("Choose a Hearth member for this care task");
    }
    Ok(())
}

/// Schedule a care-task creation attempt.
///
/// Returns `true` only when the draft is accepted for local demo creation or a
/// live zome call is actually scheduled. A live `true` is **not** a success
/// claim; authoritative success is reported only after the returned Record is
/// established through the Record→View bridge.
pub fn create_care_schedule(mut draft: CreateCareDraft) -> bool {
    draft.title = draft.title.trim().to_string();
    draft.description = draft.description.trim().to_string();
    draft.notes = draft.notes.trim().to_string();

    let hc = use_holochain();
    let hearth = use_hearth();
    let truth = use_hearth_truth();
    let toasts = use_toasts();

    if let Err(message) = validate_draft(&draft) {
        toasts.push(message, ToastKind::Custom("care".into()));
        return false;
    }

    if hc.is_mock() {
        let hearth_hash = hearth
            .current_hearth
            .get_untracked()
            .map(|value| value.hash)
            .unwrap_or_else(|| "demo-hearth".to_string());
        hearth.care_schedules.update(|schedules| {
            schedules.push(CareScheduleView {
                hash: format!("care_user_{}", schedules.len()),
                hearth_hash,
                care_type: draft.care_type,
                title: draft.title,
                description: draft.description,
                assigned_to: draft.assigned_to,
                recurrence: draft.recurrence,
                status: CareScheduleStatus::Active,
                completed_at: None,
            });
        });
        toasts.push(
            "demo care task created",
            ToastKind::Custom("care".into()),
        );
        return true;
    }

    if !hc.zome_calls_ready_untracked() {
        toasts.push(
            "creating care work needs a connected, authorized Hearth session",
            ToastKind::Custom("care".into()),
        );
        return false;
    }

    let Some(hearth_hash_text) = hearth.current_hearth.get_untracked().map(|value| value.hash) else {
        truth.availability.update(|availability| {
            availability.current_hearth = AvailabilityStateKind::Unknown;
            availability.care_schedules = AvailabilityStateKind::Unknown;
        });
        toasts.push(
            "current Hearth is not established; refresh before creating care work",
            ToastKind::Custom("care".into()),
        );
        return false;
    };

    let hearth_hash = match HoloHashBytes::from_action_raw_base64(&hearth_hash_text) {
        Ok(hash) => hash,
        Err(error) => {
            truth.availability.update(|availability| {
                availability.current_hearth = AvailabilityStateKind::Degraded;
                availability.care_schedules = AvailabilityStateKind::Degraded;
            });
            web_sys::console::log_1(
                &format!("create_care_schedule blocked by invalid Hearth ActionHash: {error}").into(),
            );
            toasts.push(
                "current Hearth target is malformed or is not an ActionHash; refresh before retrying",
                ToastKind::Custom("care".into()),
            );
            return false;
        }
    };

    let assigned_to = match HoloHashBytes::from_agent_display(&draft.assigned_to) {
        Ok(agent) => agent,
        Err(error) => {
            truth.availability.update(|availability| {
                availability.members = AvailabilityStateKind::Degraded;
                availability.care_schedules = AvailabilityStateKind::Degraded;
            });
            web_sys::console::log_1(
                &format!("create_care_schedule blocked by invalid assignee AgentPubKey: {error}").into(),
            );
            toasts.push(
                "selected member identity is malformed or is not an AgentPubKey; refresh before retrying",
                ToastKind::Custom("care".into()),
            );
            return false;
        }
    };

    spawn_local(async move {
        #[derive(serde::Serialize)]
        struct CreateCareScheduleInput {
            hearth_hash: HoloHashBytes,
            care_type: CareType,
            title: String,
            description: String,
            assigned_to: HoloHashBytes,
            recurrence: Recurrence,
            notes: String,
        }

        match hc
            .call_zome_default::<CreateCareScheduleInput, WireRecord>(
                "hearth_care",
                "create_care_schedule",
                &CreateCareScheduleInput {
                    hearth_hash,
                    care_type: draft.care_type,
                    title: draft.title,
                    description: draft.description,
                    assigned_to,
                    recurrence: draft.recurrence,
                    notes: draft.notes,
                },
            )
            .await
        {
            Ok(record) => {
                let mut decoded =
                    record_bridge::records_to_care_schedules(std::slice::from_ref(&record));
                if let Some(created) = decoded.pop() {
                    hearth.care_schedules.update(|schedules| {
                        schedules.retain(|item| item.hash != created.hash);
                        schedules.push(created);
                    });
                    truth.availability.update(|availability| {
                        if availability.care_schedules == AvailabilityStateKind::Empty {
                            availability.care_schedules = AvailabilityStateKind::Live;
                        }
                    });
                    toasts.push(
                        "care task created",
                        ToastKind::Custom("care".into()),
                    );
                } else {
                    truth.availability.update(|availability| {
                        availability.care_schedules = AvailabilityStateKind::Degraded;
                    });
                    toasts.push(
                        "care task was accepted, but the returned record could not be established",
                        ToastKind::Custom("care".into()),
                    );
                }
            }
            Err(error) => {
                truth.availability.update(|availability| {
                    availability.care_schedules = AvailabilityStateKind::Unknown;
                });
                web_sys::console::log_1(
                    &format!("create_care_schedule outcome unknown: {error}").into(),
                );
                toasts.push(
                    "couldn’t confirm whether the care task was created; reconcile before retrying",
                    ToastKind::Custom("care".into()),
                );
            }
        }
    });

    true
}

#[cfg(test)]
mod tests {
    use super::{CreateCareDraft, validate_draft};
    use hearth_leptos_types::{CareType, Recurrence};

    fn valid_draft() -> CreateCareDraft {
        CreateCareDraft {
            care_type: CareType::Chore,
            title: "Take bins out".into(),
            description: String::new(),
            assigned_to: "uhCAk-example".into(),
            recurrence: Recurrence::Weekly,
            notes: String::new(),
        }
    }

    #[test]
    fn local_shape_validation_matches_integrity_limits() {
        assert!(validate_draft(&valid_draft()).is_ok());

        let mut empty = valid_draft();
        empty.title.clear();
        assert!(validate_draft(&empty).is_err());

        let mut title = valid_draft();
        title.title = "x".repeat(257);
        assert!(validate_draft(&title).is_err());

        let mut description = valid_draft();
        description.description = "x".repeat(4097);
        assert!(validate_draft(&description).is_err());

        let mut notes = valid_draft();
        notes.notes = "x".repeat(4097);
        assert!(validate_draft(&notes).is_err());
    }
}
