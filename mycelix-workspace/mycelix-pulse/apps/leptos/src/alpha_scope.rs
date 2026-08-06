// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! What the restricted alpha DNA (`holochain/dna-alpha/dna.yaml`) actually
//! bundles, so pages can show an honest "not available in this alpha build"
//! state instead of silently failing or leaking a raw zome error to the UI.
//!
//! Mirrors `dna.yaml`'s `alpha_scope: "profile+hybrid-keys+encrypted-messages"`
//! property and its 5 coordinator zomes, all on the single `"main"` role.
//! Update this list in lockstep with `holochain/dna-alpha/dna.yaml` and
//! `holochain/happ.yaml` — there is no runtime capability query yet
//! (`hc.available_zomes()`-style), so this is a static mirror, not a live
//! introspection of the connected conductor.

/// Coordinator zomes actually bundled in the alpha DNA, on the `"main"` role.
const AVAILABLE_ZOMES: &[&str] = &[
    "mail_profiles",
    "mail_trust",
    "mail_keys",
    "mail_messages",
    "mail_capabilities",
];

/// Whether `zome` is bundled in the alpha DNA on the default `"main"` role.
pub fn zome_available(zome: &str) -> bool {
    AVAILABLE_ZOMES.contains(&zome)
}

/// Whether a role is bundled by the restricted alpha hApp.
pub fn role_available(role: &str) -> bool {
    role == "main"
}

/// The alpha hApp bundles no separate `"identity"` role (DID registry, MFA).
pub fn identity_role_available() -> bool {
    role_available("identity")
}

/// Consistent "not available in this alpha build" notice, for use in place
/// of UI that depends on a zome/role this build doesn't bundle — instead of
/// a silent no-op or a raw zome error surfacing to the user.
pub fn scope_notice(message: &str) -> impl leptos::prelude::IntoView {
    use leptos::prelude::{ElementChild, StyleAttribute};

    let message = message.to_string();
    leptos::prelude::view! {
        <p style="background:#1f2333;border:1px solid #3a3f54;border-radius:8px;padding:10px 12px;font-size:0.8rem;color:#8890a8;line-height:1.5">
            {message}
        </p>
    }
}


#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn restricted_alpha_surface_is_explicit() {
        assert!(role_available("main"));
        assert!(!role_available("identity"));
        assert!(zome_available("mail_messages"));
        assert!(!zome_available("mail_contacts"));
        assert!(!zome_available("mail_sync"));
        assert!(!zome_available("mail_search"));
    }
}
