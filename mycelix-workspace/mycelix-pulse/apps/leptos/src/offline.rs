// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Offline action queue (#4).
//!
//! When conductor is unavailable, queues actions in localStorage.
//! Flushes queue automatically when connection is restored.

use crate::holochain::use_holochain;
use leptos::prelude::*;
use mail_leptos_types::OfflineAction;
use wasm_bindgen_futures::spawn_local;

const QUEUE_KEY: &str = "mycelix_mail_offline_queue";

#[derive(Clone, Copy)]
pub struct OfflineState {
    pub queue_size: RwSignal<usize>,
}

pub fn provide_offline_context() {
    let queue = load_queue();
    let state = OfflineState {
        queue_size: RwSignal::new(queue.len()),
    };
    provide_context(state);
}

pub fn use_offline() -> OfflineState {
    expect_context::<OfflineState>()
}

/// Enqueue an action for later execution.
pub fn enqueue_action(action: OfflineAction) {
    let mut queue = load_queue();
    queue.push(action);
    save_queue(&queue);
    let state = use_offline();
    state.queue_size.set(queue.len());
}

/// Attempt to flush all queued actions to the conductor.
pub fn flush_queue() {
    let hc = use_holochain();
    if hc.is_mock() {
        return;
    }

    let queue = load_queue();
    if queue.is_empty() {
        return;
    }

    let state = use_offline();

    spawn_local(async move {
        let mut remaining = Vec::new();

        for action in queue {
            let result = execute_action(&hc, &action).await;
            if result.is_err() {
                remaining.push(action);
            }
        }

        save_queue(&remaining);
        state.queue_size.set(remaining.len());

        if remaining.is_empty() {
            web_sys::console::log_1(&"[Mail] Offline queue flushed successfully".into());
        } else {
            web_sys::console::warn_1(
                &format!("[Mail] {} actions remain in offline queue", remaining.len()).into(),
            );
        }
    });
}

async fn execute_action(
    hc: &crate::holochain::HolochainCtx,
    action: &OfflineAction,
) -> Result<(), String> {
    match action {
        OfflineAction::ToggleStar { hash } => hc
            .call_zome::<serde_json::Value, serde_json::Value>(
                "mail_messages",
                "update_email_state",
                &serde_json::json!([hash, { "is_starred": true }]),
            )
            .await
            .map(|_| ()),
        OfflineAction::ToggleRead { hash } => hc
            .call_zome::<serde_json::Value, serde_json::Value>(
                "mail_messages",
                "mark_as_read",
                &serde_json::json!([hash, false]),
            )
            .await
            .map(|_| ()),
        OfflineAction::Archive { hash } => hc
            .call_zome::<serde_json::Value, serde_json::Value>(
                "mail_messages",
                "update_email_state",
                &serde_json::json!([hash, { "is_archived": true }]),
            )
            .await
            .map(|_| ()),
        OfflineAction::Delete { hash } => hc
            .call_zome::<serde_json::Value, serde_json::Value>(
                "mail_messages",
                "update_email_state",
                &serde_json::json!([hash, { "is_trashed": true }]),
            )
            .await
            .map(|_| ()),
        OfflineAction::MoveToFolder { hash, folder_hash } => hc
            .call_zome::<serde_json::Value, serde_json::Value>(
                "mail_messages",
                "move_to_folder",
                &serde_json::json!([hash, folder_hash]),
            )
            .await
            .map(|_| ()),
        OfflineAction::Send { .. } => Err(
            "Offline sending is disabled for the working alpha; reconnect and send again".into(),
        ),
    }
}

fn load_queue() -> Vec<OfflineAction> {
    web_sys::window()
        .and_then(|w| w.local_storage().ok().flatten())
        .and_then(|s| s.get_item(QUEUE_KEY).ok().flatten())
        .and_then(|json| serde_json::from_str(&json).ok())
        .unwrap_or_default()
}

fn save_queue(queue: &[OfflineAction]) {
    if let Some(storage) = web_sys::window().and_then(|w| w.local_storage().ok().flatten()) {
        if queue.is_empty() {
            let _ = storage.remove_item(QUEUE_KEY);
        } else if let Ok(json) = serde_json::to_string(queue) {
            let _ = storage.set_item(QUEUE_KEY, &json);
        }
    }
}
