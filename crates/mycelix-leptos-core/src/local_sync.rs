// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Truth-preserving local durability and remote synchronization disclosure.
//!
//! Local durability and remote synchronization are independent facts. This
//! module does not infer federation, counterparty receipt, consensus,
//! confirmation, execution, settlement, or authorization from either axis.

use leptos::prelude::*;

/// What is currently established about durability on this device.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LocalDurability {
    /// The application cannot currently establish local durability.
    Unknown,
    /// The change exists in the current UI/session but is not established as
    /// durably persisted on this device.
    Volatile,
    /// The change is established as durably saved on this device.
    Durable,
}

impl LocalDurability {
    pub fn label(self) -> &'static str {
        match self {
            Self::Unknown => "Local save unknown",
            Self::Volatile => "Not saved locally",
            Self::Durable => "Saved locally",
        }
    }

    pub fn css_class(self) -> &'static str {
        match self {
            Self::Unknown => "local-durability-unknown",
            Self::Volatile => "local-durability-volatile",
            Self::Durable => "local-durability-durable",
        }
    }
}

/// What is currently established about synchronization beyond this device.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RemoteSyncState {
    /// No remote synchronization has been requested for this state.
    NotRequested,
    /// Synchronization has been requested but has not begun/completed.
    Queued,
    /// Synchronization is actively in progress.
    Synchronizing,
    /// The supplied authoritative sync source reports synchronization complete.
    /// This is deliberately not called Federated, Confirmed, or Settled.
    Synchronized,
    /// Concurrent/divergent state requires reconciliation.
    Conflict,
    /// The remote side or authoritative sync layer rejected the change.
    Rejected,
    /// The synchronization mechanism cannot presently be reached/used.
    Unavailable,
    /// The application cannot currently establish the remote sync state.
    Unknown,
}

impl RemoteSyncState {
    pub fn label(self) -> &'static str {
        match self {
            Self::NotRequested => "Sync not requested",
            Self::Queued => "Waiting to sync",
            Self::Synchronizing => "Synchronizing",
            Self::Synchronized => "Synchronized",
            Self::Conflict => "Sync conflict",
            Self::Rejected => "Sync rejected",
            Self::Unavailable => "Sync unavailable",
            Self::Unknown => "Sync status unknown",
        }
    }

    pub fn css_class(self) -> &'static str {
        match self {
            Self::NotRequested => "remote-sync-not-requested",
            Self::Queued => "remote-sync-queued",
            Self::Synchronizing => "remote-sync-synchronizing",
            Self::Synchronized => "remote-sync-synchronized",
            Self::Conflict => "remote-sync-conflict",
            Self::Rejected => "remote-sync-rejected",
            Self::Unavailable => "remote-sync-unavailable",
            Self::Unknown => "remote-sync-unknown",
        }
    }
}

/// Independent local + remote state supplied by the owning domain/runtime.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LocalSyncState {
    pub local: LocalDurability,
    pub remote: RemoteSyncState,
}

impl LocalSyncState {
    pub const fn new(local: LocalDurability, remote: RemoteSyncState) -> Self {
        Self { local, remote }
    }

    pub fn accessibility_label(self) -> String {
        format!("{}. {}.", self.local.label(), self.remote.label())
    }
}

/// Compact disclosure for local durability + remote synchronization truth.
///
/// The component presents only supplied state. It performs no persistence,
/// replication, federation, consensus, receipt, authorization, or settlement
/// checks and must not be used as a substitute for those domain results.
#[component]
pub fn LocalSyncDisclosure(state: LocalSyncState) -> impl IntoView {
    let aria_label = state.accessibility_label();

    view! {
        <div class="local-sync-disclosure" aria-label=aria_label>
            <span class=format!("status-pill {}", state.local.css_class())>
                {state.local.label()}
            </span>
            <span class=format!("status-pill {}", state.remote.css_class())>
                {state.remote.label()}
            </span>
        </div>
    }
}

#[cfg(test)]
mod tests {
    use super::{LocalDurability, LocalSyncState, RemoteSyncState};

    #[test]
    fn durable_local_work_can_still_be_waiting_for_remote_sync() {
        let state = LocalSyncState::new(LocalDurability::Durable, RemoteSyncState::Queued);
        assert_eq!(state.local.label(), "Saved locally");
        assert_eq!(state.remote.label(), "Waiting to sync");
        assert!(state.accessibility_label().contains("Saved locally"));
        assert!(state.accessibility_label().contains("Waiting to sync"));
    }

    #[test]
    fn synchronized_does_not_claim_confirmation_or_settlement() {
        let label = RemoteSyncState::Synchronized.label();
        assert_eq!(label, "Synchronized");
        assert!(!label.contains("Confirmed"));
        assert!(!label.contains("Settled"));
        assert!(!label.contains("Federated"));
    }

    #[test]
    fn unknown_local_and_remote_states_remain_explicit() {
        let state = LocalSyncState::new(LocalDurability::Unknown, RemoteSyncState::Unknown);
        assert!(state.accessibility_label().contains("unknown"));
    }
}
