// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Truth-preserving projection of existing local identity capabilities into Me.
//!
//! This adapter intentionally does **not** call `local_did()` or
//! `ensure_keypair()`: opening the Me surface must not mint a new identity or
//! panic merely to inspect security/recovery status. It reads only existing
//! local status flags and projects implementation capabilities that are already
//! established by the current code.

use crate::me::{MeBatch, MeFactState, MeItem, MeSection};

const PROVIDER_ID: &str = "frontend-identity";

/// Side-effect-free input for the pure Me projection.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LocalIdentityMeSnapshot {
    /// Whether encrypted passphrase-wrapped secret-key storage is configured.
    pub passphrase_protected: bool,
    /// Whether this browser has a local historical record that browser-key
    /// anchoring previously succeeded. This is not a live DHT verification.
    pub browser_anchor_recorded: bool,
}

/// Read currently available local status without generating/unlocking identity
/// material, then project it into the shared Me envelope.
pub fn current_local_identity_me_batch() -> MeBatch {
    project_local_identity_me(LocalIdentityMeSnapshot {
        passphrase_protected: crate::local_identity::is_passphrase_protected(),
        browser_anchor_recorded: crate::did_registry::is_browser_key_anchored(),
    })
}

/// Pure projection used both by the browser adapter and unit tests.
pub fn project_local_identity_me(snapshot: LocalIdentityMeSnapshot) -> MeBatch {
    let mut protection = MeItem::known(
        PROVIDER_ID,
        "local-key-protection",
        MeSection::SecurityRecovery,
        "Private key storage on this device",
        if snapshot.passphrase_protected {
            "Passphrase protected"
        } else {
            "Not passphrase protected"
        },
    );
    protection.detail = Some(if snapshot.passphrase_protected {
        "The persisted local Ed25519 secret is stored in passphrase-wrapped encrypted form. This does not by itself establish whether the identity is unlocked in the current session."
            .into()
    } else {
        "The current implementation stores the local Ed25519 secret seed without passphrase protection in browser storage until protection is explicitly enabled."
            .into()
    });

    let mut anchor = if snapshot.browser_anchor_recorded {
        MeItem::known(
            PROVIDER_ID,
            "browser-key-association-history",
            MeSection::Identity,
            "Device key association",
            "Previously recorded from this device",
        )
    } else {
        MeItem::unknown(
            PROVIDER_ID,
            "browser-key-association-history",
            MeSection::Identity,
            "Device key association",
        )
    };
    anchor.detail = Some(if snapshot.browser_anchor_recorded {
        "This browser remembers a prior successful request to add its key as a verification method. Current remote presence and active authentication status have not been re-verified here."
            .into()
    } else {
        "This browser has no local record of a successful key-association request. That does not establish that the verification method is absent remotely."
            .into()
    });

    let mut manual_recovery = MeItem::known(
        PROVIDER_ID,
        "manual-seed-recovery-capability",
        MeSection::SecurityRecovery,
        "Manual recovery",
        "Recovery phrase export/import available",
    );
    manual_recovery.detail = Some(
        "The frontend can export/import the local Ed25519 seed as a standards-correct BIP-39 phrase. This does not establish that recovery material has been saved, retained, or tested by the user."
            .into(),
    );

    let mut guardian_recovery = MeItem::known(
        PROVIDER_ID,
        "guardian-recovery-capability",
        MeSection::SecurityRecovery,
        "Guardian recovery",
        "Not available yet",
    );
    guardian_recovery.detail = Some(
        "The recovery zome exists in the broader identity roadmap, but the shared frontend does not yet provide a completed guardian/trustee recovery flow."
            .into(),
    );

    let mut device_pairing = MeItem::known(
        PROVIDER_ID,
        "device-pairing-capability",
        MeSection::Devices,
        "Add another device",
        "Not available yet",
    );
    device_pairing.detail = Some(
        "Multi-device pairing through delegated/device authorization is not implemented in this shared frontend yet."
            .into(),
    );

    MeBatch::ready(
        PROVIDER_ID,
        vec![
            protection,
            anchor,
            manual_recovery,
            guardian_recovery,
            device_pairing,
        ],
    )
}

#[cfg(test)]
mod tests {
    use super::{LocalIdentityMeSnapshot, project_local_identity_me};
    use crate::me::{MeFactState, MeSection};

    #[test]
    fn unprotected_storage_is_described_without_inventing_unlock_or_recovery_state() {
        let batch = project_local_identity_me(LocalIdentityMeSnapshot {
            passphrase_protected: false,
            browser_anchor_recorded: false,
        });
        let protection = batch
            .items
            .iter()
            .find(|item| item.id == "local-key-protection")
            .expect("protection fact");

        assert_eq!(protection.section, MeSection::SecurityRecovery);
        assert_eq!(protection.state.label(), "Not passphrase protected");
        assert!(protection.target.is_none());
    }

    #[test]
    fn absent_local_anchor_record_remains_unknown_not_remote_absent() {
        let batch = project_local_identity_me(LocalIdentityMeSnapshot {
            passphrase_protected: true,
            browser_anchor_recorded: false,
        });
        let anchor = batch
            .items
            .iter()
            .find(|item| item.id == "browser-key-association-history")
            .expect("anchor fact");

        assert_eq!(anchor.state, MeFactState::Unknown);
        assert!(
            anchor
                .detail
                .as_deref()
                .is_some_and(|detail| detail.contains("does not establish"))
        );
    }

    #[test]
    fn recorded_anchor_is_historical_not_live_verified() {
        let batch = project_local_identity_me(LocalIdentityMeSnapshot {
            passphrase_protected: true,
            browser_anchor_recorded: true,
        });
        let anchor = batch
            .items
            .iter()
            .find(|item| item.id == "browser-key-association-history")
            .expect("anchor fact");

        assert_eq!(anchor.state.label(), "Previously recorded from this device");
        assert!(
            anchor
                .detail
                .as_deref()
                .is_some_and(|detail| detail.contains("not been re-verified"))
        );
    }

    #[test]
    fn roadmap_capabilities_are_not_rendered_as_enabled() {
        let batch = project_local_identity_me(LocalIdentityMeSnapshot {
            passphrase_protected: true,
            browser_anchor_recorded: true,
        });

        for id in ["guardian-recovery-capability", "device-pairing-capability"] {
            let item = batch
                .items
                .iter()
                .find(|item| item.id == id)
                .expect("roadmap fact");
            assert_eq!(item.state.label(), "Not available yet");
        }
    }
}
