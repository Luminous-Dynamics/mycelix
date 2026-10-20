// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Shared no-silent-replacement identity bootstrap gate for Leptos app shells.
//!
//! The gate is deliberately separate from identity-dependent providers and
//! routes. Mount it as the fallback while posture is not
//! ProtectedUnlockedInSession. This keeps each hApp on the same policy and
//! create/protect/unlock/restore implementation.

use leptos::ev::SubmitEvent;
use leptos::prelude::*;
use wasm_bindgen_futures::spawn_local;
use zeroize::Zeroize;

use crate::local_identity::{
    IdentityBootstrapAction, LocalIdentityStoragePosture, create_protected_identity,
    identity_bootstrap_action_allowed,
    inspect_local_identity_posture, protect_existing_identity,
    restore_protected_identity_from_seed_phrase, unlock_with_passphrase,
};

fn bootstrap_action_for_posture(
    posture: LocalIdentityStoragePosture,
    restore_mode: bool,
) -> Option<IdentityBootstrapAction> {
    match posture {
        LocalIdentityStoragePosture::NoIdentity if restore_mode => {
            Some(IdentityBootstrapAction::RestoreProtectedIdentityFromSeedPhrase)
        }
        LocalIdentityStoragePosture::NoIdentity => {
            Some(IdentityBootstrapAction::CreateProtectedIdentity)
        }
        LocalIdentityStoragePosture::UnprotectedIdentity => {
            Some(IdentityBootstrapAction::ProtectExistingIdentity)
        }
        LocalIdentityStoragePosture::ProtectedLocked => {
            Some(IdentityBootstrapAction::UnlockProtectedIdentity)
        }
        LocalIdentityStoragePosture::ProtectedUnlockedInSession
        | LocalIdentityStoragePosture::LegacyIdentityNeedsMigration
        | LocalIdentityStoragePosture::StorageUnavailable
        | LocalIdentityStoragePosture::CorruptOrInconsistent => None,
    }
}

fn bootstrap_title(
    app_name: &str,
    posture: LocalIdentityStoragePosture,
    restore_mode: bool,
) -> String {
    let action = match posture {
        LocalIdentityStoragePosture::NoIdentity if restore_mode => "Restore a protected identity",
        LocalIdentityStoragePosture::NoIdentity => "Create a protected identity",
        LocalIdentityStoragePosture::UnprotectedIdentity => "Protect your existing identity",
        LocalIdentityStoragePosture::ProtectedLocked => "Unlock your protected identity",
        LocalIdentityStoragePosture::LegacyIdentityNeedsMigration => {
            "Legacy identity needs explicit migration"
        }
        LocalIdentityStoragePosture::StorageUnavailable => "Identity storage unavailable",
        LocalIdentityStoragePosture::ProtectedUnlockedInSession => {
            "Protected identity is unlocked"
        }
        LocalIdentityStoragePosture::CorruptOrInconsistent => {
            "Identity storage is inconsistent"
        }
    };
    format!("{app_name}: {action}")
}

fn bootstrap_description(posture: LocalIdentityStoragePosture) -> &'static str {
    match posture {
        LocalIdentityStoragePosture::NoIdentity => {
            "No local identity was found. Create a new key directly into protected storage or restore a valid 24-word backup. Existing identities are never overwritten by this flow."
        }
        LocalIdentityStoragePosture::UnprotectedIdentity => {
            "An existing key is stored without passphrase protection. Protect it in place; the public key and DID should remain the same. Plaintext is removed only after the encrypted envelope is read back and verified."
        }
        LocalIdentityStoragePosture::ProtectedLocked => {
            "A protected identity was found. Enter its passphrase to unlock it. A passphrase reset cannot recover a lost key; do not clear browser storage."
        }
        LocalIdentityStoragePosture::LegacyIdentityNeedsMigration => {
            "Only a legacy random-label identity is present. This is not cryptographic proof of control. No key will be generated or overwritten; explicit migration or recovery is required."
        }
        LocalIdentityStoragePosture::StorageUnavailable => {
            "Browser storage could not be inspected. This does not mean there is no identity. No key will be generated or overwritten."
        }
        LocalIdentityStoragePosture::ProtectedUnlockedInSession => {
            "The protected identity is unlocked in this session."
        }
        LocalIdentityStoragePosture::CorruptOrInconsistent => {
            "Stored key material is malformed or contradictory. No replacement key will be generated. Preserve this browser profile and use the reviewed recovery procedure."
        }
    }
}

fn bootstrap_is_mutable(posture: LocalIdentityStoragePosture) -> bool {
    bootstrap_action_for_posture(posture, false).is_some()
}

/// Shared identity setup gate for Personal, Finance, Climate and other hApps.
///
/// Parent app shells should mount identity-dependent routes only when the
/// supplied posture is ProtectedUnlockedInSession. The component mutates that
/// signal only after the storage inspector verifies the operation's final
/// state; create/restore is refused if an identity already exists.
#[component]
pub fn IdentityBootstrapGate(
    app_name: &'static str,
    posture: RwSignal<LocalIdentityStoragePosture>,
) -> impl IntoView {
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
        let action = bootstrap_action_for_posture(starting_posture, restore_mode.get_untracked());
        let mut passphrase_value = passphrase.get_untracked();
        let mut confirmation_value = confirmation.get_untracked();
        let mut recovery_phrase_value = recovery_phrase.get_untracked();

        if !bootstrap_is_mutable(starting_posture)
            || action.is_none_or(|action| {
                !identity_bootstrap_action_allowed(starting_posture, action)
            })
        {
            status_message.set(
                "This identity state does not permit automatic mutation. Existing data was not changed."
                    .to_string(),
            );
            passphrase_value.zeroize();
            confirmation_value.zeroize();
            recovery_phrase_value.zeroize();
            passphrase.set(String::new());
            confirmation.set(String::new());
            recovery_phrase.set(String::new());
            return;
        }

        if passphrase_value.trim().is_empty() {
            status_message.set("Enter a non-empty passphrase.".to_string());
            passphrase_value.zeroize();
            confirmation_value.zeroize();
            recovery_phrase_value.zeroize();
            return;
        }

        let requires_confirmation = matches!(
            starting_posture,
            LocalIdentityStoragePosture::NoIdentity
                | LocalIdentityStoragePosture::UnprotectedIdentity
        );
        if requires_confirmation && passphrase_value != confirmation_value {
            status_message.set("The passphrases do not match.".to_string());
            passphrase_value.zeroize();
            confirmation_value.zeroize();
            recovery_phrase_value.zeroize();
            passphrase.set(String::new());
            confirmation.set(String::new());
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
        recovery_phrase.set(String::new());
        status_message.set(String::new());
        busy.set(true);

        let posture_signal = posture;
        let passphrase_signal = passphrase;
        let confirmation_signal = confirmation;
        let recovery_phrase_signal = recovery_phrase;
        let busy_signal = busy;
        let status_signal = status_message;

        spawn_local(async move {
            let operation: Result<LocalIdentityStoragePosture, String> = match action.unwrap() {
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
                IdentityBootstrapAction::RestoreProtectedIdentityFromSeedPhrase => {
                    restore_protected_identity_from_seed_phrase(
                        &recovery_phrase_value,
                        &passphrase_value,
                    )
                    .await
                }
                IdentityBootstrapAction::RotatePassphrase => Err(
                    "Passphrase rotation belongs in the already-unlocked identity settings."
                        .into(),
                ),
            };

            let verified_posture = inspect_local_identity_posture();
            match operation {
                Ok(_) if verified_posture == LocalIdentityStoragePosture::ProtectedUnlockedInSession => {
                    posture_signal.set(LocalIdentityStoragePosture::ProtectedUnlockedInSession);
                    status_signal.set(
                        "Protected identity storage was verified for this session.".to_string(),
                    );
                }
                Ok(_) => {
                    posture_signal.set(verified_posture);
                    status_signal.set(
                        "The operation returned, but protected unlocked state could not be verified. The app remains gated."
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
        <main
            class="mycelix-identity-bootstrap"
            aria-labelledby="mycelix-identity-bootstrap-title"
            style="min-height:100vh;display:grid;place-items:center;padding:1.25rem;"
        >
            <section
                class="mycelix-identity-bootstrap-card"
                style="width:min(100%,42rem);display:grid;gap:1rem;padding:clamp(1.25rem,4vw,2.25rem);border:1px solid var(--border-visible,#9ca3af);border-radius:1rem;background:var(--bg-surface,#fff);color:var(--text-primary,#111827);"
            >
                <p style="margin:0;font-size:.75rem;letter-spacing:.12em;text-transform:uppercase;">
                    "MYCELIX / IDENTITY SAFETY"
                </p>
                <h1 id="mycelix-identity-bootstrap-title" style="margin:0;font-size:clamp(1.5rem,4vw,2.25rem);">
                    {move || bootstrap_title(app_name, posture.get(), restore_mode.get())}
                </h1>
                <p style="margin:0;line-height:1.6;">
                    {move || bootstrap_description(posture.get())}
                </p>

                <Show when=move || bootstrap_is_mutable(posture.get())>
                    <form
                        on:submit=submit
                        style="display:grid;gap:1rem;"
                    >
                        <div style="display:grid;gap:.4rem;">
                            <label for="mycelix-identity-passphrase">"Passphrase"</label>
                            <input
                                id="mycelix-identity-passphrase"
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
                                style="width:100%;min-height:44px;box-sizing:border-box;padding:.75rem;border:1px solid var(--border-visible,#9ca3af);border-radius:.5rem;background:var(--bg-surface,#fff);color:inherit;font:inherit;"
                            />
                        </div>

                        <Show when=move || {
                            matches!(
                                posture.get(),
                                LocalIdentityStoragePosture::NoIdentity
                                    | LocalIdentityStoragePosture::UnprotectedIdentity
                            )
                        }>
                            <div style="display:grid;gap:.4rem;">
                                <label for="mycelix-identity-passphrase-confirm">"Confirm passphrase"</label>
                                <input
                                    id="mycelix-identity-passphrase-confirm"
                                    type="password"
                                    required=true
                                    autocomplete="new-password"
                                    prop:value=move || confirmation.get()
                                    on:input=move |ev| confirmation.set(event_target_value(&ev))
                                    style="width:100%;min-height:44px;box-sizing:border-box;padding:.75rem;border:1px solid var(--border-visible,#9ca3af);border-radius:.5rem;background:var(--bg-surface,#fff);color:inherit;font:inherit;"
                                />
                            </div>
                        </Show>

                        <Show when=move || posture.get() == LocalIdentityStoragePosture::NoIdentity>
                            <button
                                type="button"
                                on:click=move |_| {
                                    restore_mode.update(|mode| *mode = !*mode);
                                    recovery_phrase.set(String::new());
                                    status_message.set(String::new());
                                }
                                style="min-height:44px;justify-self:start;padding:.5rem .75rem;border:1px solid var(--border-visible,#9ca3af);border-radius:.5rem;background:transparent;color:inherit;font:inherit;"
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
                            <div style="display:grid;gap:.4rem;">
                                <label for="mycelix-identity-recovery-phrase">"24-word recovery phrase"</label>
                                <textarea
                                    id="mycelix-identity-recovery-phrase"
                                    rows="4"
                                    required=true
                                    autocomplete="off"
                                    spellcheck="false"
                                    prop:value=move || recovery_phrase.get()
                                    on:input=move |ev| recovery_phrase.set(event_target_value(&ev))
                                    style="width:100%;min-height:7rem;box-sizing:border-box;padding:.75rem;border:1px solid var(--border-visible,#9ca3af);border-radius:.5rem;background:var(--bg-surface,#fff);color:inherit;font:inherit;resize:vertical;"
                                />
                                <p style="margin:0;color:var(--text-secondary,#4b5563);font-size:.85rem;line-height:1.5;">
                                    "Only enter this phrase in a private browser session. It is not added to a URL or intentionally persisted by this form and is cleared after the attempt."
                                </p>
                            </div>
                        </Show>

                        <p style="margin:0;padding:.75rem 1rem;border-inline-start:3px solid var(--accent,#b7791f);background:rgba(183,121,31,.08);font-size:.85rem;line-height:1.5;">
                            "Passphrase protection secures the key at rest; it does not defend against active XSS, malicious extensions or a compromised browser session. Keep a verified offline backup."
                        </p>
                        <button
                            type="submit"
                            disabled=move || busy.get()
                            style="min-height:44px;padding:.75rem 1rem;border:0;border-radius:.5rem;background:var(--primary,#1d4ed8);color:var(--primary-foreground,#fff);font:inherit;font-weight:650;"
                        >
                            {move || {
                                if busy.get() {
                                    "Processing…"
                                } else {
                                    match bootstrap_action_for_posture(posture.get(), restore_mode.get()) {
                                        Some(IdentityBootstrapAction::CreateProtectedIdentity) => {
                                            "Create protected identity"
                                        }
                                        Some(IdentityBootstrapAction::ProtectExistingIdentity) => {
                                            "Protect existing identity"
                                        }
                                        Some(IdentityBootstrapAction::UnlockProtectedIdentity) => {
                                            "Unlock identity"
                                        }
                                        Some(IdentityBootstrapAction::RestoreProtectedIdentityFromSeedPhrase) => {
                                            "Restore and protect identity"
                                        }
                                        Some(IdentityBootstrapAction::RotatePassphrase) | None => {
                                            "Identity action unavailable"
                                        }
                                    }
                                }
                            }}
                        </button>
                    </form>
                </Show>

                <Show when=move || !bootstrap_is_mutable(posture.get())>
                    <section
                        role="note"
                        style="padding:1rem;border:1px solid var(--border-visible,#9ca3af);border-radius:.5rem;"
                    >
                        <strong>"No automatic changes will be made."</strong>
                        <p style="margin:.5rem 0 0;line-height:1.5;color:var(--text-secondary,#4b5563);">
                            "Do not clear browser storage or create another identity from this screen. Preserve available backups and use an explicit recovery/migration procedure."
                        </p>
                    </section>
                </Show>

                <p role="status" aria-live="polite" style="min-height:1.2em;margin:0;overflow-wrap:anywhere;">
                    {move || status_message.get()}
                </p>
            </section>
        </main>
    }
}

#[cfg(test)]
mod tests {
    use super::{bootstrap_action_for_posture, bootstrap_is_mutable};
    use crate::local_identity::{IdentityBootstrapAction as Action, LocalIdentityStoragePosture as Posture};

    #[test]
    fn every_mutable_posture_has_one_explicit_supported_action() {
        assert_eq!(
            bootstrap_action_for_posture(Posture::NoIdentity, false),
            Some(Action::CreateProtectedIdentity)
        );
        assert_eq!(
            bootstrap_action_for_posture(Posture::NoIdentity, true),
            Some(Action::RestoreProtectedIdentityFromSeedPhrase)
        );
        assert_eq!(
            bootstrap_action_for_posture(Posture::UnprotectedIdentity, false),
            Some(Action::ProtectExistingIdentity)
        );
        assert_eq!(
            bootstrap_action_for_posture(Posture::ProtectedLocked, false),
            Some(Action::UnlockProtectedIdentity)
        );
    }

    #[test]
    fn uncertain_or_already_unlocked_postures_do_not_show_bootstrap_mutations() {
        for posture in [
            Posture::ProtectedUnlockedInSession,
            Posture::LegacyIdentityNeedsMigration,
            Posture::StorageUnavailable,
            Posture::CorruptOrInconsistent,
        ] {
            assert_eq!(bootstrap_action_for_posture(posture, false), None);
            assert!(!bootstrap_is_mutable(posture));
        }
    }
}
