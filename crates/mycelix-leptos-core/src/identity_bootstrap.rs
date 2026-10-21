// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Shared identity-posture bootstrap boundary for Mycelix Leptos apps.
//!
//! Identity-dependent children mount only after the identity is protected,
 //! unlocked in this session, and the current recovery-backup challenge has
 //! been completed. Storage transitions still enforce their own preconditions.

use leptos::ev::SubmitEvent;
use leptos::prelude::*;
use wasm_bindgen_futures::spawn_local;
use zeroize::Zeroize;

use crate::local_identity::{
    IdentityBootstrapAction, LocalIdentityStoragePosture, allowed_identity_bootstrap_actions,
    create_protected_identity, export_unlocked_identity_seed_phrase,
    identity_bootstrap_action_allowed, inspect_local_identity_posture, mark_recovery_backup_confirmed,
    protect_existing_identity, recovery_backup_confirmation_needed,
    recovery_phrase_challenge_positions, restore_protected_identity_from_seed_phrase,
    unlock_with_passphrase,
};

/// Hold identity-dependent UI behind the shared protected-identity gate.
///
/// First-use creation, explicit protection of an existing key, unlock, and
/// restore from a 24-word backup are available from the gate. Legacy markers,
/// unavailable storage and corrupt/inconsistent data are read-only blockers.
fn identity_ready_subtree_allowed(
    posture: LocalIdentityStoragePosture,
    backup_confirmation_needed: bool,
) -> bool {
    posture == LocalIdentityStoragePosture::ProtectedUnlockedInSession
        && !backup_confirmation_needed
}

/// Hold identity-dependent UI behind the shared protected-identity gate and
/// the recovery-backup challenge. This does not prove a physical backup lasts
/// forever; it verifies the user can transcribe and later reproduce key words.
#[component]
pub fn IdentityBootstrapGate(children: Children) -> impl IntoView {
    let posture = RwSignal::new(inspect_local_identity_posture());
    let backup_confirmation_needed = RwSignal::new(recovery_backup_confirmation_needed());
    view! {
        <Show
            when=move || posture.get() == LocalIdentityStoragePosture::ProtectedUnlockedInSession
            fallback=move || view! { <IdentityBootstrapForm posture=posture /> }
        >
            <Show
                when=move || {
                    identity_ready_subtree_allowed(
                        posture.get(),
                        backup_confirmation_needed.get(),
                    )
                }
                fallback=move || view! {
                    <RecoveryBackupGate needed=backup_confirmation_needed />
                }
            >
                {children()}
            </Show>
        </Show>
    }
}

/// An explicit, user-initiated recovery-phrase display and three-word challenge.
/// The phrase remains transient UI state only and is zeroized/hidden after
/// each attempt; the persisted marker contains the public key, never the phrase.
#[component]
fn RecoveryBackupGate(needed: RwSignal<bool>) -> impl IntoView {
    let phrase = RwSignal::new(String::new());
    let positions = RwSignal::new([1usize, 2usize, 3usize]);
    let visible = RwSignal::new(false);
    let word_one = RwSignal::new(String::new());
    let word_two = RwSignal::new(String::new());
    let word_three = RwSignal::new(String::new());
    let status_message = RwSignal::new(String::new());

    let hide_phrase = move |_| {
        clear_bootstrap_secret_signal(phrase);
        clear_bootstrap_secret_signal(word_one);
        clear_bootstrap_secret_signal(word_two);
        clear_bootstrap_secret_signal(word_three);
        visible.set(false);
    };

    let show_phrase = move |_| {
        clear_bootstrap_secret_signal(word_one);
        clear_bootstrap_secret_signal(word_two);
        clear_bootstrap_secret_signal(word_three);
        match export_unlocked_identity_seed_phrase() {
            Ok(mut exported_phrase) => match recovery_phrase_challenge_positions() {
                Ok(challenge_positions) => {
                    clear_bootstrap_secret_signal(phrase);
                    phrase.set(exported_phrase);
                    positions.set(challenge_positions);
                    visible.set(true);
                    status_message.set(String::new());
                }
                Err(error) => {
                    exported_phrase.zeroize();
                    clear_bootstrap_secret_signal(phrase);
                    visible.set(false);
                    status_message.set(error);
                }
            },
            Err(error) => {
                clear_bootstrap_secret_signal(phrase);
                visible.set(false);
                status_message.set(error);
            }
        }
    };

    let verify_backup = move |ev: SubmitEvent| {
        ev.prevent_default();
        if !visible.get_untracked() {
            status_message.set("Show the recovery phrase before answering the challenge.".into());
            return;
        }

        let phrase_positions = positions.get_untracked();
        let mut phrase_value = phrase.get_untracked();
        let mut answer_one = word_one.get_untracked();
        let mut answer_two = word_two.get_untracked();
        let mut answer_three = word_three.get_untracked();

        let mut expected_one = phrase_value
            .split_whitespace()
            .nth(phrase_positions[0].saturating_sub(1))
            .unwrap_or("")
            .to_ascii_lowercase();
        let mut expected_two = phrase_value
            .split_whitespace()
            .nth(phrase_positions[1].saturating_sub(1))
            .unwrap_or("")
            .to_ascii_lowercase();
        let mut expected_three = phrase_value
            .split_whitespace()
            .nth(phrase_positions[2].saturating_sub(1))
            .unwrap_or("")
            .to_ascii_lowercase();

        let matches = !expected_one.is_empty()
            && !expected_two.is_empty()
            && !expected_three.is_empty()
            && answer_one.trim().eq_ignore_ascii_case(&expected_one)
            && answer_two.trim().eq_ignore_ascii_case(&expected_two)
            && answer_three.trim().eq_ignore_ascii_case(&expected_three);

        phrase_value.zeroize();
        answer_one.zeroize();
        answer_two.zeroize();
        answer_three.zeroize();
        expected_one.zeroize();
        expected_two.zeroize();
        expected_three.zeroize();
        clear_bootstrap_secret_signal(phrase);
        clear_bootstrap_secret_signal(word_one);
        clear_bootstrap_secret_signal(word_two);
        clear_bootstrap_secret_signal(word_three);
        visible.set(false);

        if !matches {
            status_message.set(
                "The recovery words did not match. The phrase has been hidden; show it again to retry."
                    .into(),
            );
            return;
        }

        match mark_recovery_backup_confirmed() {
            Ok(()) if !recovery_backup_confirmation_needed() => {
                needed.set(false);
                status_message.set(
                    "Recovery-word challenge passed. Keep the backup offline and private."
                        .into(),
                );
            }
            Ok(()) => {
                needed.set(true);
                status_message.set(
                    "Confirmation could not be re-verified against current identity storage."
                        .into(),
                );
            }
            Err(error) => {
                needed.set(true);
                status_message.set(error);
            }
        }
    };

    view! {
        <div class="identity-bootstrap-shell">
            <main class="vault-card identity-bootstrap-card" aria-labelledby="recovery-backup-title">
                <span class="page-eyebrow">"MYCELIX / RECOVERY READINESS"</span>
                <h1 id="recovery-backup-title">"Verify your offline recovery backup"</h1>
                <p class="supporting-copy">
                    "Your identity is protected and unlocked, but the normal app remains gated until you demonstrate that you can reproduce selected recovery words."
                </p>
                <section class="identity-bootstrap-blocker" role="note">
                    <strong>"Keep this phrase private."</strong>
                    <p>
                        "Write the 24 words on a durable offline backup and store it securely. Do not photograph it, paste it into another website, send it to support, or put it in cloud notes. This challenge tests transcription; it cannot prove the physical backup will survive."
                    </p>
                </section>
                <Show
                    when=move || !visible.get()
                    fallback=move || view! {
                        <section class="recovery-phrase-panel" aria-label="Recovery phrase">
                            <p class="supporting-copy">
                                "This phrase is secret key material. It is shown only after your explicit action and will be hidden after verification or when you hide it."
                            </p>
                            <ol class="recovery-phrase-grid">
                                {move || {
                                    phrase.get()
                                        .split_whitespace()
                                        .enumerate()
                                        .map(|(index, word)| {
                                            view! {
                                                <li>
                                                    <span class="recovery-word-index">{index + 1}</span>
                                                    <code>{word.to_string()}</code>
                                                </li>
                                            }
                                        })
                                        .collect_view()
                                }}
                            </ol>
                            <form class="identity-bootstrap-form" on:submit=verify_backup>
                                <p class="supporting-copy">
                                    "Enter the word at each requested position to confirm you copied the phrase."
                                </p>
                                <div class="identity-bootstrap-field">
                                    <label for="recovery-word-one">
                                        {move || format!("Word {} (1 of 3)", positions.get()[0])}
                                    </label>
                                    <input
                                        id="recovery-word-one"
                                        class="identity-bootstrap-input"
                                        type="password"
                                        autocomplete="off"
                                        spellcheck="false"
                                        required=true
                                        prop:value=move || word_one.get()
                                        on:input=move |ev| word_one.set(event_target_value(&ev))
                                    />
                                </div>
                                <div class="identity-bootstrap-field">
                                    <label for="recovery-word-two">
                                        {move || format!("Word {} (2 of 3)", positions.get()[1])}
                                    </label>
                                    <input
                                        id="recovery-word-two"
                                        class="identity-bootstrap-input"
                                        type="password"
                                        autocomplete="off"
                                        spellcheck="false"
                                        required=true
                                        prop:value=move || word_two.get()
                                        on:input=move |ev| word_two.set(event_target_value(&ev))
                                    />
                                </div>
                                <div class="identity-bootstrap-field">
                                    <label for="recovery-word-three">
                                        {move || format!("Word {} (3 of 3)", positions.get()[2])}
                                    </label>
                                    <input
                                        id="recovery-word-three"
                                        class="identity-bootstrap-input"
                                        type="password"
                                        autocomplete="off"
                                        spellcheck="false"
                                        required=true
                                        prop:value=move || word_three.get()
                                        on:input=move |ev| word_three.set(event_target_value(&ev))
                                    />
                                </div>
                                <button class="btn btn-primary identity-bootstrap-submit" type="submit">
                                    "Verify recovery words"
                                </button>
                                <button class="identity-bootstrap-secondary" type="button" on:click=hide_phrase>
                                    "Hide phrase"
                                </button>
                            </form>
                        </section>
                    }
                >
                    <button
                        class="btn btn-primary identity-bootstrap-submit"
                        type="button"
                        on:click=show_phrase
                    >
                        "Show 24-word recovery phrase"
                    </button>
                </Show>
                <p class="identity-bootstrap-status" role="status" aria-live="polite">
                    {move || status_message.get()}
                </p>
            </main>
        </div>
    }
}

fn clear_bootstrap_secret_signal(signal: RwSignal<String>) {
    signal.update(|value| value.zeroize());
}

fn clear_bootstrap_secrets(
    passphrase: RwSignal<String>,
    confirmation: RwSignal<String>,
    recovery_phrase: RwSignal<String>,
) {
    clear_bootstrap_secret_signal(passphrase);
    clear_bootstrap_secret_signal(confirmation);
    clear_bootstrap_secret_signal(recovery_phrase);
}


#[cfg(test)]
mod tests {
    use super::identity_ready_subtree_allowed;
    use crate::local_identity::LocalIdentityStoragePosture as Posture;

    #[test]
    fn ready_subtree_requires_protected_unlock_and_backup_confirmation() {
        let postures = [
            Posture::StorageUnavailable,
            Posture::NoIdentity,
            Posture::LegacyIdentityNeedsMigration,
            Posture::UnprotectedIdentity,
            Posture::ProtectedLocked,
            Posture::ProtectedUnlockedInSession,
            Posture::CorruptOrInconsistent,
        ];
        for posture in postures {
            assert!(
                !identity_ready_subtree_allowed(posture, true),
                "backup-needed state must gate {posture:?}"
            );
            assert_eq!(
                identity_ready_subtree_allowed(posture, false),
                posture == Posture::ProtectedUnlockedInSession,
                "only protected/unlocked may enter the ready subtree: {posture:?}"
            );
        }
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
                clear_bootstrap_secrets(passphrase, confirmation, recovery_phrase);
                status_message
                    .set("This identity state does not permit automatic mutation.".to_string());
                return;
            }
        };
        if !identity_bootstrap_action_allowed(starting_posture, action) {
            passphrase_value.zeroize();
            confirmation_value.zeroize();
            recovery_phrase_value.zeroize();
            clear_bootstrap_secrets(passphrase, confirmation, recovery_phrase);
            status_message.set(
                "The requested action is not permitted for the current identity posture."
                    .to_string(),
            );
            return;
        }
        if passphrase_value.trim().is_empty() {
            passphrase_value.zeroize();
            confirmation_value.zeroize();
            recovery_phrase_value.zeroize();
            clear_bootstrap_secrets(passphrase, confirmation, recovery_phrase);
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
            clear_bootstrap_secrets(passphrase, confirmation, recovery_phrase);
            return;
        }
        if is_restore && recovery_phrase_value.trim().is_empty() {
            status_message.set("Enter your 24-word recovery phrase.".to_string());
            passphrase_value.zeroize();
            confirmation_value.zeroize();
            recovery_phrase_value.zeroize();
            clear_bootstrap_secrets(passphrase, confirmation, recovery_phrase);
            return;
        }

        clear_bootstrap_secrets(passphrase, confirmation, recovery_phrase);
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


