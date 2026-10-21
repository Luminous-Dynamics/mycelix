// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Shared, fail-closed bootstrap UX for every Leptos app that requires a
//! protected local Mycelix identity. The policy and mutation implementation
//! remain in `local_identity`; this module shares the accessible workflow.

use leptos::ev::SubmitEvent;
use leptos::prelude::*;
use wasm_bindgen_futures::spawn_local;
use zeroize::Zeroize;

use crate::local_identity::{
    IdentityBootstrapAction, LocalIdentityStoragePosture, allowed_identity_bootstrap_actions,
    create_protected_identity, export_unlocked_identity_seed_phrase,
    identity_bootstrap_action_allowed, inspect_local_identity_posture,
    mark_recovery_backup_confirmed, protect_existing_identity, recovery_backup_confirmation_needed,
    recovery_phrase_challenge_positions, restore_protected_identity_from_seed_phrase,
    unlock_with_passphrase,
};

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

fn clear_bootstrap_secret_signals(
    passphrase: RwSignal<String>,
    confirmation: RwSignal<String>,
    recovery_phrase: RwSignal<String>,
) {
    passphrase.update(|value| value.zeroize());
    confirmation.update(|value| value.zeroize());
    recovery_phrase.update(|value| value.zeroize());
}

fn prepare_recovery_backup() -> Result<(Vec<String>, [usize; 3]), String> {
    let mut phrase = export_unlocked_identity_seed_phrase()?;
    let mut words: Vec<String> = phrase.split_whitespace().map(str::to_owned).collect();
    phrase.zeroize();

    if words.len() != 24 {
        zeroize_recovery_words(&mut words);
        return Err("The current identity did not produce a 24-word recovery phrase.".into());
    }
    let positions = match recovery_phrase_challenge_positions() {
        Ok(positions) => positions,
        Err(error) => {
            zeroize_recovery_words(&mut words);
            return Err(error);
        }
    };
    Ok((words, positions))
}

fn zeroize_recovery_words(words: &mut Vec<String>) {
    for word in words.iter_mut() {
        word.zeroize();
    }
    words.clear();
}

fn recovery_confirmation_matches(
    words: &[String],
    positions: [usize; 3],
    answers: &[String; 3],
) -> bool {
    if words.len() != 24 {
        return false;
    }

    let mut seen = [false; 24];
    for slot in 0..3 {
        let Some(index) = positions[slot].checked_sub(1) else {
            return false;
        };
        if index >= words.len() || seen[index] {
            return false;
        }
        seen[index] = true;
        if !answers[slot]
            .trim()
            .eq_ignore_ascii_case(words[index].as_str())
        {
            return false;
        }
    }
    true
}

#[cfg(test)]
mod identity_bootstrap_backup_tests {
    use super::recovery_confirmation_matches;

    fn words() -> Vec<String> {
        (1..=24).map(|n| format!("word{n}")).collect()
    }

    #[test]
    fn backup_challenge_accepts_the_three_selected_words_case_insensitively() {
        let words = words();
        let positions = [2, 11, 24];
        let answers = [
            "WORD2".to_string(),
            " word11 ".to_string(),
            "word24".to_string(),
        ];
        assert!(recovery_confirmation_matches(&words, positions, &answers));
    }

    #[test]
    fn backup_challenge_rejects_a_wrong_word() {
        let words = words();
        let positions = [2, 11, 24];
        let answers = [
            "word2".to_string(),
            "wrong".to_string(),
            "word24".to_string(),
        ];
        assert!(!recovery_confirmation_matches(&words, positions, &answers));
    }

    #[test]
    fn backup_challenge_rejects_duplicate_or_out_of_range_positions() {
        let words = words();
        let answers = [
            "word1".to_string(),
            "word1".to_string(),
            "word24".to_string(),
        ];
        assert!(!recovery_confirmation_matches(&words, [1, 1, 24], &answers));
        assert!(!recovery_confirmation_matches(
            &words,
            [0, 11, 24],
            &answers
        ));
        assert!(!recovery_confirmation_matches(
            &words,
            [1, 11, 25],
            &answers
        ));
    }

    #[test]
    fn backup_challenge_rejects_non_24_word_material() {
        let words = vec!["word1".to_string(); 12];
        let answers = [
            "word1".to_string(),
            "word1".to_string(),
            "word1".to_string(),
        ];
        assert!(!recovery_confirmation_matches(&words, [1, 5, 12], &answers));
    }
}

#[component]
pub fn IdentityBootstrapGate(
    posture: RwSignal<LocalIdentityStoragePosture>,
    backup_pending: RwSignal<bool>,
) -> impl IntoView {
    let passphrase = RwSignal::new(String::new());
    let confirmation = RwSignal::new(String::new());
    let recovery_phrase = RwSignal::new(String::new());
    let restore_mode = RwSignal::new(false);
    let busy = RwSignal::new(false);
    let status_message = RwSignal::new(String::new());
    let backup_phrase_words = RwSignal::new(Vec::<String>::new());
    let backup_challenge_positions = RwSignal::new(None::<[usize; 3]>);
    let backup_answers = RwSignal::new([String::new(), String::new(), String::new()]);
    let backup_acknowledged = RwSignal::new(false);

    if backup_pending.get_untracked() {
        match prepare_recovery_backup() {
            Ok((words, positions)) => {
                backup_phrase_words.set(words);
                backup_challenge_positions.set(Some(positions));
            }
            Err(error) => status_message.set(format!(
                "Recovery backup needs setup before entering Personal: {error}"
            )),
        }
    }

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
                clear_bootstrap_secret_signals(passphrase, confirmation, recovery_phrase);
                status_message
                    .set("This identity state does not permit automatic mutation.".to_string());
                return;
            }
        };
        if !identity_bootstrap_action_allowed(starting_posture, action) {
            passphrase_value.zeroize();
            confirmation_value.zeroize();
            recovery_phrase_value.zeroize();
            clear_bootstrap_secret_signals(passphrase, confirmation, recovery_phrase);
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
            clear_bootstrap_secret_signals(passphrase, confirmation, recovery_phrase);
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
            clear_bootstrap_secret_signals(passphrase, confirmation, recovery_phrase);
            return;
        }
        if is_restore && recovery_phrase_value.trim().is_empty() {
            status_message.set("Enter your 24-word recovery phrase.".to_string());
            passphrase_value.zeroize();
            confirmation_value.zeroize();
            recovery_phrase_value.zeroize();
            clear_bootstrap_secret_signals(passphrase, confirmation, recovery_phrase);
            return;
        }

        clear_bootstrap_secret_signals(passphrase, confirmation, recovery_phrase);
        status_message.set(String::new());
        busy.set(true);

        let posture_signal = posture;
        let passphrase_signal = passphrase;
        let confirmation_signal = confirmation;
        let recovery_phrase_signal = recovery_phrase;
        let busy_signal = busy;
        let status_signal = status_message;
        let backup_pending_signal = backup_pending;
        let backup_words_signal = backup_phrase_words;
        let backup_positions_signal = backup_challenge_positions;
        let backup_answers_signal = backup_answers;
        let backup_ack_signal = backup_acknowledged;
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
                IdentityBootstrapAction::ExportRecoveryPhrase => Err(
                    "Recovery export is handled by the recovery backup confirmation step.".into(),
                ),
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
                    if recovery_backup_confirmation_needed() {
                        backup_pending_signal.set(true);
                        match prepare_recovery_backup() {
                            Ok((words, positions)) => {
                                backup_words_signal.update(zeroize_recovery_words);
                                backup_words_signal.set(words);
                                backup_positions_signal.set(Some(positions));
                                backup_answers_signal.update(|answers| {
                                    for answer in answers.iter_mut() {
                                        answer.zeroize();
                                    }
                                });
                                backup_answers_signal.set([
                                    String::new(),
                                    String::new(),
                                    String::new(),
                                ]);
                                backup_ack_signal.set(false);
                                status_signal.set(
                                    "Identity is protected. Save the recovery phrase and confirm three words before continuing."
                                        .to_string(),
                                );
                            }
                            Err(error) => {
                                backup_words_signal.update(zeroize_recovery_words);
                                backup_words_signal.set(Vec::new());
                                backup_positions_signal.set(None);
                                status_signal.set(format!(
                                    "Backup setup must be completed before entering Personal: {error}"
                                ));
                            }
                        }
                    } else {
                        status_signal
                            .set("Protected identity and recovery backup state verified for this session.".to_string());
                    }
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
            clear_bootstrap_secret_signals(
                passphrase_signal,
                confirmation_signal,
                recovery_phrase_signal,
            );
            busy_signal.set(false);
        });
    };

    let retry_backup = move |_| match prepare_recovery_backup() {
        Ok((words, positions)) => {
            backup_phrase_words.update(zeroize_recovery_words);
            backup_phrase_words.set(words);
            backup_challenge_positions.set(Some(positions));
            backup_answers.update(|answers| {
                for answer in answers.iter_mut() {
                    answer.zeroize();
                }
            });
            backup_answers.set([String::new(), String::new(), String::new()]);
            backup_acknowledged.set(false);
            status_message.set(
                "Recovery phrase ready. Store it offline, then confirm the requested words."
                    .to_string(),
            );
        }
        Err(error) => status_message.set(format!("Backup setup is still unavailable: {error}")),
    };

    let backup_word_entries = move || {
        backup_phrase_words
            .get()
            .into_iter()
            .enumerate()
            .collect::<Vec<(usize, String)>>()
    };

    let confirm_backup = move |ev: SubmitEvent| {
        ev.prevent_default();
        let mut words = backup_phrase_words.get_untracked();
        let positions = backup_challenge_positions.get_untracked();
        let mut answers = backup_answers.get_untracked();
        let acknowledged = backup_acknowledged.get_untracked();

        let Some(positions) = positions else {
            zeroize_recovery_words(&mut words);
            for answer in answers.iter_mut() {
                answer.zeroize();
            }
            backup_answers.set([String::new(), String::new(), String::new()]);
            status_message
                .set("Recovery challenge is not ready. Retry the backup setup.".to_string());
            return;
        };

        let words_match = recovery_confirmation_matches(&words, positions, &answers);
        if !acknowledged || !words_match {
            for answer in answers.iter_mut() {
                answer.zeroize();
            }
            backup_answers.set([String::new(), String::new(), String::new()]);
            status_message.set(if !acknowledged {
                "Confirm that you stored the recovery phrase in a place you control, then retry."
                    .to_string()
            } else {
                "The requested words did not match. The phrase remains visible; check your offline copy and try again."
                    .to_string()
            });
            zeroize_recovery_words(&mut words);
            return;
        }

        if let Err(error) = mark_recovery_backup_confirmed() {
            for answer in answers.iter_mut() {
                answer.zeroize();
            }
            backup_answers.update(|stored_answers| {
                for answer in stored_answers.iter_mut() {
                    answer.zeroize();
                }
            });
            backup_answers.set([String::new(), String::new(), String::new()]);
            zeroize_recovery_words(&mut words);
            status_message.set(format!(
                "The word check passed, but recovery confirmation could not be verified: {error}. Personal remains gated; retry the check."
            ));
            return;
        }

        zeroize_recovery_words(&mut words);
        for answer in answers.iter_mut() {
            answer.zeroize();
        }
        backup_phrase_words.update(zeroize_recovery_words);
        backup_phrase_words.set(Vec::new());
        backup_challenge_positions.set(None);
        backup_answers.update(|stored_answers| {
            for answer in stored_answers.iter_mut() {
                answer.zeroize();
            }
        });
        backup_answers.set([String::new(), String::new(), String::new()]);
        backup_acknowledged.set(false);
        backup_pending.set(false);
        status_message.set(
            "Recovery-word challenge passed. The phrase has been cleared from this form."
                .to_string(),
        );
    };

    view! {
        <div class="identity-bootstrap-shell">
            <main class="vault-card identity-bootstrap-card" aria-labelledby="identity-bootstrap-title">
                <span class="page-eyebrow">"MYCELIX / IDENTITY SAFETY"</span>
                <h1 id="identity-bootstrap-title">
                    {move || {
                        if backup_pending.get() {
                            "Verify your recovery backup"
                        } else {
                            identity_bootstrap_heading(posture.get(), restore_mode.get())
                        }
                    }}
                </h1>
                <p class="supporting-copy">
                    {move || {
                        if backup_pending.get() {
                            "Your identity is protected, but the application stays gated until you record the 24-word recovery phrase and correctly answer a short word-position check. Mycelix does not store the phrase."
                        } else {
                            identity_bootstrap_description(posture.get())
                        }
                    }}
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
                                    recovery_phrase.update(|value| value.zeroize());
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

                <Show when=move || {
                    !backup_pending.get() && !identity_bootstrap_is_actionable(posture.get())
                }>
                    <section class="identity-bootstrap-blocker" role="note">
                        <strong>"No automatic changes will be made."</strong>
                        <p>
                            "Do not clear browser storage or attempt to create another identity from this screen. Preserve available backups and use an explicit recovery/migration procedure."
                        </p>
                    </section>
                </Show>

                <Show when=move || backup_pending.get()>
                    <section class="vault-card identity-backup-card" aria-labelledby="identity-backup-title">
                        <h2 id="identity-backup-title">"Save your recovery phrase before continuing"</h2>
                        <p class="supporting-copy">
                            "These 24 words recover the identity key. Anyone who obtains them can take control of the identity. Write them down or store them in a trusted offline backup location. Do not send them by email, paste them into chat, or take a screenshot. This form never persists them."
                        </p>

                        <Show when=move || backup_phrase_words.get().len() == 24>
                            <ol class="identity-recovery-word-grid">
                                <For
                                    each=backup_word_entries
                                    key=|(index, _word)| *index
                                    children=move |(index, word)| view! {
                                        <li>
                                            <span>{index + 1}</span>
                                            <strong>{word}</strong>
                                        </li>
                                    }
                                />
                            </ol>

                            <form class="identity-bootstrap-form" on:submit=confirm_backup>
                                <h3>"Confirm three words from your saved copy"</h3>
                                <p class="supporting-copy">
                                    "Enter the requested words exactly as written. This checks that you copied part of the phrase; it cannot prove the backup will remain available in the future."
                                </p>
                                <div class="identity-bootstrap-field">
                                    <label for="identity-backup-answer-1">
                                        {move || format!(
                                            "Word #{}",
                                            backup_challenge_positions.get().map(|p| p[0]).unwrap_or(0)
                                        )}
                                    </label>
                                    <input
                                        id="identity-backup-answer-1"
                                        class="identity-bootstrap-input"
                                        type="text"
                                        autocomplete="off"
                                        autocapitalize="none"
                                        spellcheck="false"
                                        prop:value=move || backup_answers.get()[0].clone()
                                        on:input=move |ev| backup_answers.update(|answers| {
                                            answers[0] = event_target_value(&ev);
                                        })
                                    />
                                </div>
                                <div class="identity-bootstrap-field">
                                    <label for="identity-backup-answer-2">
                                        {move || format!(
                                            "Word #{}",
                                            backup_challenge_positions.get().map(|p| p[1]).unwrap_or(0)
                                        )}
                                    </label>
                                    <input
                                        id="identity-backup-answer-2"
                                        class="identity-bootstrap-input"
                                        type="text"
                                        autocomplete="off"
                                        autocapitalize="none"
                                        spellcheck="false"
                                        prop:value=move || backup_answers.get()[1].clone()
                                        on:input=move |ev| backup_answers.update(|answers| {
                                            answers[1] = event_target_value(&ev);
                                        })
                                    />
                                </div>
                                <div class="identity-bootstrap-field">
                                    <label for="identity-backup-answer-3">
                                        {move || format!(
                                            "Word #{}",
                                            backup_challenge_positions.get().map(|p| p[2]).unwrap_or(0)
                                        )}
                                    </label>
                                    <input
                                        id="identity-backup-answer-3"
                                        class="identity-bootstrap-input"
                                        type="text"
                                        autocomplete="off"
                                        autocapitalize="none"
                                        spellcheck="false"
                                        prop:value=move || backup_answers.get()[2].clone()
                                        on:input=move |ev| backup_answers.update(|answers| {
                                            answers[2] = event_target_value(&ev);
                                        })
                                    />
                                </div>

                                <label class="identity-backup-acknowledgement">
                                    <input
                                        type="checkbox"
                                        prop:checked=move || backup_acknowledged.get()
                                        on:change=move |ev| backup_acknowledged.set(event_target_checked(&ev))
                                    />
                                    <span>"I have saved these words in a place I control, separate from this browser."</span>
                                </label>
                                <button
                                    class="btn btn-primary identity-bootstrap-submit"
                                    type="submit"
                                >
                                    "Verify backup and continue"
                                </button>
                            </form>
                        </Show>

                        <Show when=move || backup_phrase_words.get().len() != 24>
                            <p class="supporting-copy">
                                "Backup setup could not be initialized. Your key remains protected and Personal remains gated. Retry; do not clear browser storage."
                            </p>
                            <button
                                type="button"
                                class="identity-bootstrap-secondary"
                                on:click=retry_backup
                            >
                                "Retry recovery phrase setup"
                            </button>
                        </Show>
                    </section>
                </Show>

                <p class="identity-bootstrap-status" role="status" aria-live="polite">
                    {move || status_message.get()}
                </p>
            </main>
        </div>
    }
}

