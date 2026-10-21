// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Shared identity-posture bootstrap boundary for Mycelix Leptos apps.
//!
//! Identity-dependent children mount only after an exact posture read reports
//! `ProtectedUnlockedInSession`. The form uses the core state/action policy,
//! while storage operations independently enforce their preconditions.

use leptos::ev::SubmitEvent;
use leptos::prelude::*;
use wasm_bindgen_futures::spawn_local;
use zeroize::Zeroize;

use crate::local_identity::{
    IdentityBootstrapAction, LocalIdentityStoragePosture, allowed_identity_bootstrap_actions,
    create_protected_identity, identity_bootstrap_action_allowed, inspect_local_identity_posture,
    protect_existing_identity, restore_protected_identity_from_seed_phrase,
    unlock_with_passphrase,
};

/// Hold identity-dependent UI behind the shared protected-identity gate.
///
/// First-use creation, explicit protection of an existing key, unlock, and
/// restore from a 24-word backup are available from the gate. Legacy markers,
/// unavailable storage and corrupt/inconsistent data are read-only blockers.
#[component]
pub fn IdentityBootstrapGate(children: Children) -> impl IntoView {
    let posture = RwSignal::new(inspect_local_identity_posture());
    view! {
        <Show
            when=move || {
                posture.get() == LocalIdentityStoragePosture::ProtectedUnlockedInSession
            }
            fallback=move || view! { <IdentityBootstrapForm posture=posture /> }
        >
            {children()}
        </Show>
    }
}

fn identity_bootstrap_is_actionable(posture: LocalIdentityStoragePosture) -> bool {
    !allowed_identity_bootstrap_actions(posture).is_empty()
}

fn identity_bootstrap_heading(
    posture: LocalIdentityStoragePosture,
    restore_mode: bool,
) -> &'static str {
    match posture {
        LocalIdentityStoragePosture::NoIdentity if restore_mode => "Restore a protected identity",
        LocalIdentityStoragePosture::NoIdentity => "Create a protected identity",
        LocalIdentityStoragePosture::UnprotectedIdentity => "Protect your existing identity",
        LocalIdentityStoragePosture::ProtectedLocked => "Unlock your protected identity",
        LocalIdentityStoragePosture::LegacyIdentityNeedsMigration => {
            "Legacy identity needs explicit migration"
        }
        LocalIdentityStoragePosture::StorageUnavailable => "Identity storage unavailable",
        LocalIdentityStoragePosture::ProtectedUnlockedInSession => "Protected identity is unlocked",
        LocalIdentityStoragePosture::CorruptOrInconsistent => "Identity storage is inconsistent",
    }
}

fn identity_bootstrap_description(posture: LocalIdentityStoragePosture) -> &'static str {
    match posture {
        LocalIdentityStoragePosture::NoIdentity => {
            "No local identity was found. Create a new key directly into protected storage, or restore a valid 24-word backup. Existing identities are never overwritten by this flow."
        }
        LocalIdentityStoragePosture::UnprotectedIdentity => {
            "An existing key is stored without passphrase protection. Protect it in place; the key and DID should remain the same. Plaintext is removed only after the encrypted envelope is read back and verified."
        }
        LocalIdentityStoragePosture::ProtectedLocked => {
            "A protected identity was found. Enter its passphrase to unlock it. There is no passphrase reset that can recover a lost key; do not clear browser storage."
        }
        LocalIdentityStoragePosture::LegacyIdentityNeedsMigration => {
            "Only a legacy random-label identity is present. It is not cryptographic proof of control and this screen will not silently discard it or generate a replacement. Explicit migration/recovery is required."
        }
        LocalIdentityStoragePosture::StorageUnavailable => {
            "Browser storage could not be inspected. This does not mean there is no identity. No key will be generated or overwritten."
        }
        LocalIdentityStoragePosture::ProtectedUnlockedInSession => {
            "The protected identity is unlocked in this session."
        }
        LocalIdentityStoragePosture::CorruptOrInconsistent => {
            "Stored key material is malformed or contradictory. No replacement key will be generated. Preserve the browser profile and use the reviewed recovery procedure."
        }
    }
}

#[component]
fn IdentityBootstrapForm(posture: RwSignal<LocalIdentityStoragePosture>) -> impl IntoView {
    let passphrase = RwSignal::new(String::new());
    let confirmation = RwSignal::new(String::new());
    let recovery_phrase = RwSignal::new(String::new());
    let restore_mode = RwSignal::new(false);
    let busy = RwSignal::new(false);
    let status_message = RwSignal::new(String::new());

    let submit = move |ev: SubmitEvent| {
        ev.prevent_default();
        if busy.get_untracked() {
            return;
        }

        let starting_posture = posture.get_untracked();
        let is_restore = starting_posture == LocalIdentityStoragePosture::NoIdentity
            && restore_mode.get_untracked();
        let mut passphrase_value = passphrase.get_untracked();
        let mut confirmation_value = confirmation.get_untracked();
        let mut recovery_phrase_value = recovery_phrase.get_untracked();

        let action = match starting_posture {
            LocalIdentityStoragePosture::NoIdentity if is_restore => {
                IdentityBootstrapAction::RestoreProtectedIdentityFromSeedPhrase
            }
            LocalIdentityStoragePosture::NoIdentity => {
                IdentityBootstrapAction::CreateProtectedIdentity
            }
            LocalIdentityStoragePosture::UnprotectedIdentity => {
                IdentityBootstrapAction::ProtectExistingIdentity
            }
            LocalIdentityStoragePosture::ProtectedLocked => {
                IdentityBootstrapAction::UnlockProtectedIdentity
            }
            _ => {
                passphrase_value.zeroize();
                confirmation_value.zeroize();
                recovery_phrase_value.zeroize();
                status_message
                    .set("This identity state does not permit automatic mutation.".to_string());
                return;
            }
        };
        if !identity_bootstrap_action_allowed(starting_posture, action) {
            passphrase_value.zeroize();
            confirmation_value.zeroize();
            recovery_phrase_value.zeroize();
            status_message.set(
                "The requested action is not permitted for the current identity posture."
                    .to_string(),
            );
            return;
        }
        if passphrase_value.trim().is_empty() {
            status_message.set("Enter a passphrase.".to_string());
            return;
        }
        if matches!(
            starting_posture,
            LocalIdentityStoragePosture::NoIdentity
                | LocalIdentityStoragePosture::UnprotectedIdentity
        ) && passphrase_value != confirmation_value
        {
            status_message.set("The passphrases do not match.".to_string());
            passphrase_value.zeroize();
            confirmation_value.zeroize();
            recovery_phrase_value.zeroize();
            return;
        }
        if is_restore && recovery_phrase_value.trim().is_empty() {
            status_message.set("Enter your 24-word recovery phrase.".to_string());
            passphrase_value.zeroize();
            confirmation_value.zeroize();
            recovery_phrase_value.zeroize();
            return;
        }

        passphrase.set(String::new());
        confirmation.set(String::new());
        if is_restore {
            recovery_phrase.set(String::new());
        }
        status_message.set(String::new());
        busy.set(true);

        let posture_signal = posture;
        let passphrase_signal = passphrase;
        let confirmation_signal = confirmation;
        let recovery_phrase_signal = recovery_phrase;
        let busy_signal = busy;
        let status_signal = status_message;
        spawn_local(async move {
            let operation: Result<LocalIdentityStoragePosture, String> = match action {
                IdentityBootstrapAction::RestoreProtectedIdentityFromSeedPhrase => {
                    restore_protected_identity_from_seed_phrase(
                        &recovery_phrase_value,
                        &passphrase_value,
                    )
                    .await
                }
                IdentityBootstrapAction::CreateProtectedIdentity => {
                    create_protected_identity(&passphrase_value).await
                }
                IdentityBootstrapAction::ProtectExistingIdentity => {
                    protect_existing_identity(&passphrase_value).await
                }
                IdentityBootstrapAction::UnlockProtectedIdentity => {
                    unlock_with_passphrase(&passphrase_value)
                        .await
                        .map(|_| inspect_local_identity_posture())
                }
                IdentityBootstrapAction::RotatePassphrase => Err(
                    "Passphrase rotation is available only from the unlocked identity settings."
                        .into(),
                ),
            };

            let verified_posture = inspect_local_identity_posture();
            match operation {
                Ok(_)
                    if verified_posture
                        == LocalIdentityStoragePosture::ProtectedUnlockedInSession =>
                {
                    posture_signal.set(LocalIdentityStoragePosture::ProtectedUnlockedInSession);
                    status_signal
                        .set("Protected identity state verified for this session.".to_string());
                }
                Ok(_) => {
                    posture_signal.set(verified_posture);
                    status_signal.set(
                        "The operation returned, but protected unlocked state could not be verified. The application remains gated."
                            .to_string(),
                    );
                }
                Err(error) => {
                    posture_signal.set(verified_posture);
                    status_signal.set(error);
                }
            }

            passphrase_value.zeroize();
            confirmation_value.zeroize();
            recovery_phrase_value.zeroize();
            passphrase_signal.set(String::new());
            confirmation_signal.set(String::new());
            recovery_phrase_signal.set(String::new());
            busy_signal.set(false);
        });
    };

    view! {
        <div class="identity-bootstrap-shell">
            <main class="vault-card identity-bootstrap-card" aria-labelledby="identity-bootstrap-title">
                <span class="page-eyebrow">"MYCELIX / IDENTITY SAFETY"</span>
                <h1 id="identity-bootstrap-title">
                    {move || identity_bootstrap_heading(posture.get(), restore_mode.get())}
                </h1>
                <p class="supporting-copy">
                    {move || identity_bootstrap_description(posture.get())}
                </p>

                <Show when=move || identity_bootstrap_is_actionable(posture.get())>
                    <form class="identity-bootstrap-form" on:submit=submit>
                        <div class="identity-bootstrap-field">
                            <label for="identity-bootstrap-passphrase">"Passphrase"</label>
                            <input
                                id="identity-bootstrap-passphrase"
                                class="identity-bootstrap-input"
                                type="password"
                                required=true
                                autocomplete=move || {
                                    if posture.get() == LocalIdentityStoragePosture::ProtectedLocked {
                                        "current-password"
                                    } else {
                                        "new-password"
                                    }
                                }
                                prop:value=move || passphrase.get()
                                on:input=move |ev| passphrase.set(event_target_value(&ev))
                            />
                        </div>

                        <Show when=move || {
                            matches!(
                                posture.get(),
                                LocalIdentityStoragePosture::NoIdentity
                                    | LocalIdentityStoragePosture::UnprotectedIdentity
                            )
                        }>
                            <div class="identity-bootstrap-field">
                                <label for="identity-bootstrap-confirm">"Confirm passphrase"</label>
                                <input
                                    id="identity-bootstrap-confirm"
                                    class="identity-bootstrap-input"
                                    type="password"
                                    required=true
                                    autocomplete="new-password"
                                    prop:value=move || confirmation.get()
                                    on:input=move |ev| confirmation.set(event_target_value(&ev))
                                />
                            </div>
                        </Show>

                        <Show when=move || posture.get() == LocalIdentityStoragePosture::NoIdentity>
                            <button
                                type="button"
                                class="identity-bootstrap-secondary"
                                on:click=move |_| {
                                    restore_mode.update(|mode| *mode = !*mode);
                                    recovery_phrase.set(String::new());
                                    status_message.set(String::new());
                                }
                            >
                                {move || {
                                    if restore_mode.get() {
                                        "Use a new protected identity instead"
                                    } else {
                                        "Restore from a 24-word backup"
                                    }
                                }}
                            </button>
                        </Show>

                        <Show when=move || {
                            posture.get() == LocalIdentityStoragePosture::NoIdentity
                                && restore_mode.get()
                        }>
                            <div class="identity-bootstrap-field">
                                <label for="identity-bootstrap-recovery-phrase">"24-word recovery phrase"</label>
                                <textarea
                                    id="identity-bootstrap-recovery-phrase"
                                    class="identity-bootstrap-input identity-bootstrap-recovery-phrase"
                                    rows="4"
                                    required=true
                                    autocomplete="off"
                                    spellcheck="false"
                                    prop:value=move || recovery_phrase.get()
                                    on:input=move |ev| recovery_phrase.set(event_target_value(&ev))
                                />
                                <p class="supporting-copy">
                                    "Use this only in a private browser session. The phrase is not copied, logged, or stored by this form; it is cleared after the attempt."
                                </p>
                            </div>
                        </Show>

                        <p class="identity-bootstrap-warning">
                            "A strong passphrase protects the key at rest; it does not protect against active malware or a compromised browser session. Keep a verified offline backup."
                        </p>
                        <button
                            class="btn btn-primary identity-bootstrap-submit"
                            type="submit"
                            disabled=move || busy.get()
                        >
                            {move || {
                                if busy.get() {
                                    "Processing…"
                                } else if posture.get() == LocalIdentityStoragePosture::ProtectedLocked {
                                    "Unlock identity"
                                } else if posture.get() == LocalIdentityStoragePosture::UnprotectedIdentity {
                                    "Protect existing identity"
                                } else if restore_mode.get() {
                                    "Restore and protect identity"
                                } else {
                                    "Create protected identity"
                                }
                            }}
                        </button>
                    </form>
                </Show>

                <Show when=move || !identity_bootstrap_is_actionable(posture.get())>
                    <section class="identity-bootstrap-blocker" role="note">
                        <strong>"No automatic changes will be made."</strong>
                        <p>
                            "Do not clear browser storage or attempt to create another identity from this screen. Preserve available backups and use an explicit recovery/migration procedure."
                        </p>
                    </section>
                </Show>

                <p class="identity-bootstrap-status" role="status" aria-live="polite">
                    {move || status_message.get()}
                </p>
            </main>
        </div>
    }
}


