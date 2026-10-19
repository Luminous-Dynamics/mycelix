// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Honest connection/data wording for the resident-facing Hearth UI.
//!
//! A successful WebSocket connection is transport evidence, not proof that every
//! visible collection is live, current, or bound to the selected active Hearth.

use leptos::prelude::*;
use mycelix_leptos_core::holochain_provider::{use_holochain, ConnectionStatus};

/// User-facing copy for a connection state.
///
/// Keep this distinction explicit: "connected" must never be rendered as
/// equivalent to "complete and current household snapshot".
fn data_trust_copy(
    status: ConnectionStatus,
    signing_ready: bool,
) -> (&'static str, &'static str, &'static str) {
    match status {
        ConnectionStatus::Connected if !signing_ready => (
            "Connection established · signing unavailable",
            "The conductor is reachable, but signed access is not ready. This interface has not established a complete, current Hearth snapshot; some sections may still show sample content.",
            "data-trust--unverified",
        ),
        ConnectionStatus::Connected => (
            "Connected · household data not fully verified",
            "This interface currently loads only a subset of Hearth domains. Other sections can still contain sample records. A working connection does not mean this view is live or current; verify the active Hearth and source state before relying on it.",
            "data-trust--unverified",
        ),
        ConnectionStatus::Connecting => (
            "Connecting to Hearth",
            "This view starts with sample content so it can render quickly. Treat it as a preview until the connection, active Hearth, and data sources are established.",
            "data-trust--connecting",
        ),
        ConnectionStatus::Reconnecting => (
            "Connection recovering",
            "Previously displayed content may be stale or sample-derived. Recheck the active Hearth and source state after the connection returns.",
            "data-trust--stale",
        ),
        ConnectionStatus::Disconnected => (
            "Hearth connection unavailable",
            "Visible data has not been verified against a current household snapshot. It may be sample-derived or stale; do not treat it as current.",
            "data-trust--unavailable",
        ),
        ConnectionStatus::Mock => (
            "Preview mode · sample data only",
            "Interactions in this mode change demo state locally and do not sync to a household.",
            "data-trust--preview",
        ),
    }
}

#[component]
pub fn DataTrustNotice() -> impl IntoView {
    let hc = use_holochain();
    // ReadSignal handles are Copy, so each reactive closure observes the same
    // provider state without moving the provider context into one closure.
    let status = hc.status;
    let signing_ready = hc.zome_call_signing_ready;

    // The shared provider already displays an unmissable alert when a live
    // strategy degrades into Mock. Avoid duplicating that alert here.
    view! {
        <Show when=move || status.get() != ConnectionStatus::Mock>
            <aside
                class=move || {
                    let (_, _, modifier) =
                        data_trust_copy(status.get(), signing_ready.get());
                    format!("data-trust-notice {modifier}")
                }
                role="status"
                aria-live="polite"
                aria-atomic="true"
                aria-label="household data status"
            >
                <span class="data-trust-mark" aria-hidden="true">"i"</span>
                <div class="data-trust-copy">
                    <strong>
                        {move || data_trust_copy(status.get(), signing_ready.get()).0}
                    </strong>
                    <p>
                        {move || data_trust_copy(status.get(), signing_ready.get()).1}
                    </p>
                </div>
            </aside>
        </Show>
    }
}

#[cfg(test)]
mod tests {
    use super::data_trust_copy;
    use mycelix_leptos_core::holochain_provider::ConnectionStatus;

    #[test]
    fn every_connection_state_has_nonempty_resident_facing_copy() {
        for status in [
            ConnectionStatus::Disconnected,
            ConnectionStatus::Connecting,
            ConnectionStatus::Connected,
            ConnectionStatus::Reconnecting,
            ConnectionStatus::Mock,
        ] {
            let (heading, body, class) = data_trust_copy(status, false);
            assert!(!heading.trim().is_empty(), "missing heading for {status:?}");
            assert!(!body.trim().is_empty(), "missing body for {status:?}");
            assert!(class.starts_with("data-trust--"), "missing state class for {status:?}");
        }
    }

    #[test]
    fn connected_state_never_promises_a_complete_live_snapshot() {
        for signing_ready in [false, true] {
            let (heading, body, _) =
                data_trust_copy(ConnectionStatus::Connected, signing_ready);
            let copy = format!("{heading} {body}").to_lowercase();
            assert!(
                copy.contains("not") || copy.contains("subset") || copy.contains("has not"),
                "connected copy must explicitly qualify data trust: {copy}"
            );
            assert!(!copy.contains("everything is live"));
            assert!(!copy.contains("fully live"));
        }
    }

    #[test]
    fn preview_copy_says_interactions_are_local_only() {
        let (_, body, _) = data_trust_copy(ConnectionStatus::Mock, false);
        assert!(body.contains("locally"));
        assert!(body.contains("do not sync"));
    }
}
