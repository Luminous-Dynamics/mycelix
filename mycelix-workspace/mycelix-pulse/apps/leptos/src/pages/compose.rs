// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Compose page with reply/forward prefill (#3).

use crate::components::RichEditor;
use crate::mail_context::use_mail;
use crate::toasts::use_toasts;
use leptos::prelude::*;
use mail_leptos_types::ComposeMode;
use mail_leptos_types::protocol::{
    AuthenticatedMetadataV1, ENVELOPE_V2_HYBRID_PQC, EncryptedEnvelopeV2HybridPqc, EncryptionKeyId,
    MessageId, MessagePlaintextV2, SUITE_X25519_MLKEM768_AES_256_GCM_AGENT_MLDSA65,
};
use mycelix_crypto::CryptoError;
use mycelix_crypto::pulse_v2::{RecipientPublicKeys, seal_with_aad};
use serde::Deserialize;

#[derive(Clone, Debug, Deserialize)]
struct HybridBundleWireV2 {
    key_id: [u8; 32],
    x25519_public_key: [u8; 32],
    ml_kem_768_public_key: Vec<u8>,
    ml_dsa_65_public_key: Vec<u8>,
}

#[derive(Clone, Debug, Deserialize)]
struct HybridSendContextWireV2 {
    sender_agent_raw: Vec<u8>,
    recipient_agent_raw: Vec<u8>,
    sender_bundle: HybridBundleWireV2,
    sender_bundle_hash: Vec<u8>,
    recipient_bundle: HybridBundleWireV2,
    recipient_bundle_hash: Vec<u8>,
}

#[component]
pub fn ComposePage() -> impl IntoView {
    let mail = use_mail();
    let toasts = use_toasts();

    // Initialize from compose_mode (reply/forward prefill)
    let mode = mail.compose_mode.get_untracked();
    let (initial_to, initial_subject, initial_body, mode_label, reply_hash, reply_thread) =
        match &mode {
            ComposeMode::New => (
                String::new(),
                String::new(),
                String::new(),
                "Compose",
                None,
                None,
            ),
            ComposeMode::Reply {
                email_hash,
                sender,
                sender_name,
                subject,
                body,
                thread_id,
            } => {
                let quoted = format!(
                    "\n\n--- On previous message, {} wrote ---\n{}",
                    sender_name, body
                );
                let subj = if subject.starts_with("Re: ") {
                    subject.clone()
                } else {
                    format!("Re: {subject}")
                };
                (
                    sender.clone(),
                    subj,
                    quoted,
                    "Reply",
                    Some(email_hash.clone()),
                    thread_id.clone().or_else(|| Some(email_hash.clone())),
                )
            }
            ComposeMode::Forward { subject, body } => {
                let quoted = format!("\n\n--- Forwarded message ---\n{}", body);
                (
                    String::new(),
                    subject.clone(),
                    quoted,
                    "Forward",
                    None,
                    None,
                )
            }
        };
    // Reset compose mode after reading
    mail.compose_mode.set(ComposeMode::New);

    // Load draft from localStorage if no reply/forward mode
    let draft_key = "mycelix_pulse_draft";
    let (draft_to, draft_subj, draft_body) = if mode_label == "Compose" {
        web_sys::window()
            .and_then(|w| w.local_storage().ok().flatten())
            .and_then(|s| s.get_item(draft_key).ok().flatten())
            .and_then(|json| serde_json::from_str::<(String, String, String)>(&json).ok())
            .unwrap_or_else(|| {
                (
                    initial_to.clone(),
                    initial_subject.clone(),
                    initial_body.clone(),
                )
            })
    } else {
        (
            initial_to.clone(),
            initial_subject.clone(),
            initial_body.clone(),
        )
    };

    let to_field = RwSignal::new(draft_to);
    let cc_field = RwSignal::new(String::new());
    let subject_field = RwSignal::new(draft_subj);
    let body_field = RwSignal::new(draft_body);

    // Auto-save draft every 30 seconds
    {
        let dk = draft_key;
        Effect::new(move |_| {
            let to = to_field.get();
            let subj = subject_field.get();
            let body = body_field.get();
            if !to.is_empty() || !subj.is_empty() || !body.is_empty() {
                if let Some(s) = web_sys::window().and_then(|w| w.local_storage().ok().flatten()) {
                    let _ = s.set_item(
                        dk,
                        &serde_json::to_string(&(to, subj, body)).unwrap_or_default(),
                    );
                }
            }
        });
    }
    let show_cc = RwSignal::new(false);
    let sending = RwSignal::new(false);
    let show_templates = RwSignal::new(false);

    let reply_thread_send = RwSignal::new(reply_thread.clone());
    let reply_hash_send = RwSignal::new(reply_hash.clone());
    let toasts_send = toasts.clone();
    let on_send = move |_| {
        let to = to_field.get();
        let subject = subject_field.get();
        let body = body_field.get();

        if to.trim().is_empty() {
            toasts_send.push("Please enter a recipient", "error");
            return;
        }
        if subject.trim().is_empty() && body.trim().is_empty() {
            toasts_send.push("Cannot send an empty message", "error");
            return;
        }

        // Duplicate send detection — check if we sent to this person about the same subject recently
        let recent_dup = mail.sent.get_untracked().iter().any(|e| {
            let same_recipient =
                e.sender_name.as_deref() == Some(to.trim()) || e.hash.contains(to.trim());
            let same_subject =
                e.subject.as_deref().map(|s| s.to_lowercase()) == Some(subject.to_lowercase());
            let now = js_sys::Date::now() as u64 / 1000;
            let recent = now.saturating_sub(e.timestamp) < 3600; // within 1 hour
            same_recipient && same_subject && recent
        });
        if recent_dup {
            // Use browser confirm dialog
            let confirmed = web_sys::window()
                .and_then(|w| w.confirm_with_message(
                    "You sent a similar message to this recipient less than 1 hour ago. Send anyway?"
                ).ok())
                .unwrap_or(true);
            if !confirmed {
                return;
            }
        }

        sending.set(true);
        let crypto_label = "Hybrid PQC V2 (X25519 + ML-KEM-768 + ML-DSA-65)";

        // Expand template variables
        let now = js_sys::Date::new_0();
        let today = format!(
            "{}-{:02}-{:02}",
            now.get_full_year(),
            now.get_month() + 1,
            now.get_date()
        );
        let time_now = format!("{:02}:{:02}", now.get_hours(), now.get_minutes());
        let body = body
            .replace("{{name}}", to.trim())
            .replace("{{date}}", &today)
            .replace("{{time}}", &time_now)
            .replace("{{sender}}", "You");
        let subject = subject
            .replace("{{name}}", to.trim())
            .replace("{{date}}", &today)
            .replace("{{time}}", &time_now)
            .replace("{{sender}}", "You");

        // Undo send: 5-second delay before actually sending
        let undo = RwSignal::new(false);
        let toasts_undo = toasts_send.clone();
        toasts_send.push(
            format!(
                "Sending to {} via {}... (undo available for 5s)",
                to.trim(),
                crypto_label
            ),
            "info",
        );

        // Clear form + draft immediately (feels responsive)
        to_field.set(String::new());
        cc_field.set(String::new());
        subject_field.set(String::new());
        body_field.set(String::new());
        sending.set(false);
        // Clear saved draft since we're sending
        if let Some(s) = web_sys::window().and_then(|w| w.local_storage().ok().flatten()) {
            let _ = s.remove_item(draft_key);
        }

        // Capture values before moving into async block
        let send_to = to.trim().to_string();
        let send_subject = subject.clone();
        let send_body = body.clone();

        // Delayed send — 5s undo window, then actual zome call
        wasm_bindgen_futures::spawn_local(async move {
            gloo_timers::future::sleep(std::time::Duration::from_secs(5)).await;
            if undo.get_untracked() {
                toasts_undo.push("Send cancelled", "info");
                return;
            }

            let hc = crate::holochain::use_holochain();
            if hc.is_mock() {
                // Demo mode: add the email to the local inbox so the demo feels alive
                let now = (js_sys::Date::now() / 1000.0) as u64;
                mail.inbox.update(|emails| {
                    emails.insert(
                        0,
                        mail_leptos_types::EmailListItem {
                            hash: format!("sent-{now}"),
                            sender: "uhCAk_self_mock".into(),
                            sender_name: Some("You".into()),
                            encrypted_subject: vec![],
                            subject: Some(send_subject.clone()),
                            snippet: Some(if send_body.len() > 100 {
                                format!("{}...", &send_body[..100])
                            } else {
                                send_body.clone()
                            }),
                            timestamp: now,
                            priority: mail_leptos_types::EmailPriority::Normal,
                            is_read: true,
                            is_starred: false,
                            star_type: None,
                            is_pinned: false,
                            is_muted: false,
                            is_snoozed: false,
                            snooze_until: None,
                            has_attachments: false,
                            labels: vec![],
                            thread_id: reply_thread_send.get_untracked(),
                            crypto_suite: mail_leptos_types::CryptoSuiteView {
                                key_exchange: "x25519+ml-kem-768".into(),
                                symmetric: "aes-256-gcm".into(),
                                signature: "agent-ed25519+ml-dsa-65".into(),
                            },
                        },
                    );
                });
                toasts_undo.push("Message sent (demo mode — added to inbox)", "success");
                return;
            }

            let context = match hc
                .call_zome::<serde_json::Value, Option<HybridSendContextWireV2>>(
                    "mail_keys",
                    "resolve_hybrid_send_context_v2",
                    &serde_json::json!(send_to),
                )
                .await
            {
                Ok(Some(context)) => context,
                Ok(None) => {
                    toasts_undo.push("Recipient has no active hybrid-PQC V2 key bundle", "error");
                    return;
                }
                Err(e) => {
                    toasts_undo.push(format!("Could not fetch recipient keys: {e}"), "error");
                    return;
                }
            };

            let identity = match crate::device_keystore::load_hybrid_identity().await {
                Ok(identity) => identity,
                Err(e) => {
                    toasts_undo.push(format!("Device identity unavailable: {e}"), "error");
                    return;
                }
            };
            let local_public = identity.public_bundle();
            if local_public.x25519_public_key != context.sender_bundle.x25519_public_key
                || local_public.ml_kem_768_public_key != context.sender_bundle.ml_kem_768_public_key
                || local_public.ml_dsa_65_public_key != context.sender_bundle.ml_dsa_65_public_key
            {
                toasts_undo.push("Device keys do not match the active sender bundle; refusing to replace or downgrade", "error");
                return;
            }

            let message_id: [u8; 32] = crate::crypto::generate_nonce(32)
                .try_into()
                .expect("secure RNG returned the requested message ID length");
            let timestamp_micros = (js_sys::Date::now() * 1000.0) as i64;
            let plaintext = MessagePlaintextV2 {
                subject: send_subject,
                body: send_body,
                content_type: "text/plain; charset=utf-8".into(),
            };
            let plaintext = match plaintext.canonical_bytes() {
                Ok(bytes) => bytes,
                Err(error) => {
                    toasts_undo.push(format!("Message encoding failed: {error:?}"), "error");
                    return;
                }
            };
            let recipient_keys = RecipientPublicKeys {
                x25519: context.recipient_bundle.x25519_public_key,
                ml_kem_768: context.recipient_bundle.ml_kem_768_public_key.clone(),
            };
            let sealed =
                match seal_with_aad(&recipient_keys, &plaintext, |ephemeral, kem, nonce| {
                    EncryptedEnvelopeV2HybridPqc {
                        version: ENVELOPE_V2_HYBRID_PQC,
                        cipher_suite: SUITE_X25519_MLKEM768_AES_256_GCM_AGENT_MLDSA65.into(),
                        message_id: MessageId(message_id),
                        sender_agent: context.sender_agent_raw.clone(),
                        recipient_agent: context.recipient_agent_raw.clone(),
                        sender_mldsa_key_id: EncryptionKeyId(context.sender_bundle.key_id),
                        sender_mldsa_bundle_hash: context.sender_bundle_hash.clone(),
                        recipient_bundle_hash: context.recipient_bundle_hash.clone(),
                        recipient_hybrid_key_id: EncryptionKeyId(context.recipient_bundle.key_id),
                        x25519_ephemeral_public_key: *ephemeral,
                        ml_kem_ciphertext: kem.to_vec(),
                        nonce: *nonce,
                        ciphertext: Vec::new(),
                        metadata: AuthenticatedMetadataV1 {
                            in_reply_to: None,
                            thread_id: None,
                        },
                        created_at_micros: timestamp_micros,
                        agent_signature: Vec::new(),
                        ml_dsa_signature: Vec::new(),
                    }
                    .canonical_aad()
                    .map_err(|error| {
                        CryptoError::Validation(format!("invalid canonical V2 AAD: {error:?}"))
                    })
                }) {
                    Ok(sealed) => sealed,
                    Err(error) => {
                        toasts_undo.push(format!("Hybrid-PQC encryption failed: {error}"), "error");
                        return;
                    }
                };
            let mut envelope = EncryptedEnvelopeV2HybridPqc {
                version: ENVELOPE_V2_HYBRID_PQC,
                cipher_suite: SUITE_X25519_MLKEM768_AES_256_GCM_AGENT_MLDSA65.into(),
                message_id: MessageId(message_id),
                sender_agent: context.sender_agent_raw,
                recipient_agent: context.recipient_agent_raw,
                sender_mldsa_key_id: EncryptionKeyId(context.sender_bundle.key_id),
                sender_mldsa_bundle_hash: context.sender_bundle_hash,
                recipient_bundle_hash: context.recipient_bundle_hash,
                recipient_hybrid_key_id: EncryptionKeyId(context.recipient_bundle.key_id),
                x25519_ephemeral_public_key: sealed.x25519_ephemeral_public,
                ml_kem_ciphertext: sealed.ml_kem_ciphertext,
                nonce: sealed.nonce,
                ciphertext: sealed.ciphertext,
                metadata: AuthenticatedMetadataV1 {
                    in_reply_to: None,
                    thread_id: None,
                },
                created_at_micros: timestamp_micros,
                agent_signature: Vec::new(),
                ml_dsa_signature: Vec::new(),
            };
            let transcript = match envelope.canonical_signing_bytes() {
                Ok(bytes) => bytes,
                Err(error) => {
                    toasts_undo.push(format!("V2 transcript failed: {error:?}"), "error");
                    return;
                }
            };
            envelope.ml_dsa_signature = identity.ml_dsa.sign(&transcript);

            let payload = serde_json::json!({
                "recipient": send_to,
                "message_id": message_id,
                "sender_mldsa_key_id": envelope.sender_mldsa_key_id.0,
                "sender_mldsa_bundle_hash": envelope.sender_mldsa_bundle_hash,
                "recipient_bundle_hash": envelope.recipient_bundle_hash,
                "recipient_hybrid_key_id": envelope.recipient_hybrid_key_id.0,
                "x25519_ephemeral_public_key": envelope.x25519_ephemeral_public_key,
                "ml_kem_ciphertext": envelope.ml_kem_ciphertext,
                "nonce": envelope.nonce,
                "ciphertext": envelope.ciphertext,
                "in_reply_to": serde_json::Value::Null,
                "thread_id": serde_json::Value::Null,
                "created_at_micros": timestamp_micros,
                "ml_dsa_signature": envelope.ml_dsa_signature,
            });

            // Decoded as `Vec<u8>`, not `serde_json::Value`: `send_email_v2`
            // returns a real `ActionHash` (the Sweettest suite's
            // `send_email_v2(pre-restart)`/`(post-restart)` type-annotates
            // it directly, so the zome signature can't just become `()`),
            // whose raw-byte msgpack encoding `serde_json::Value` cannot
            // represent ("invalid type: byte array, expected any valid
            // JSON value") — this caller only logs the value for
            // debugging, never reads it structurally.
            match hc
                .call_zome::<serde_json::Value, Vec<u8>>("mail_messages", "send_email_v2", &payload)
                .await
            {
                Ok(response) => {
                    web_sys::console::log_1(
                        &format!("[Mail] send_email_v2 response: {:?}", response).into(),
                    );
                    toasts_undo.push("Message sent and committed to DHT", "success");
                    mail.versions.inbox.update(|v| *v += 1);
                }
                Err(e) => {
                    web_sys::console::warn_1(&format!("[Mail] send_email_v2 failed: {e}").into());
                    toasts_undo.push(
                        format!("Send failed: {e}. Reconnect and try again."),
                        "error",
                    );
                }
            }
        });
    };

    let toasts_draft = toasts.clone();
    let toasts_template = toasts.clone();
    let on_save_draft = move |_| {
        toasts_draft.push("Draft saved", "info");
        mail.versions.drafts.update(|v| *v += 1);
    };

    // Contact discovery: local first, then DHT search
    let dht_suggestions = RwSignal::new(Vec::<mail_leptos_types::ContactView>::new());
    let last_dht_query = RwSignal::new(String::new());

    let contact_suggestions = move || {
        let query = to_field.get().to_lowercase();
        if query.len() < 2 {
            return vec![];
        }
        // Local contacts first
        let mut results: Vec<_> = mail
            .contacts
            .get()
            .into_iter()
            .filter(|c| {
                c.display_name.to_lowercase().contains(&query)
                    || c.email
                        .as_deref()
                        .unwrap_or("")
                        .to_lowercase()
                        .contains(&query)
            })
            .take(5)
            .collect();

        // Append DHT search results (deduped by agent key)
        let dht = dht_suggestions.get();
        for c in dht {
            if !results.iter().any(|r| r.agent_pub_key == c.agent_pub_key) {
                results.push(c);
            }
        }

        // Trigger async DHT search if query changed and we have < 3 local results
        if results.len() < 3 && query.len() >= 3 && query != last_dht_query.get_untracked() {
            last_dht_query.set(query.clone());
            let hc = crate::holochain::use_holochain();
            if !hc.is_mock() && crate::alpha_scope::zome_available("mail_contacts") {
                let q = query.clone();
                wasm_bindgen_futures::spawn_local(async move {
                    if let Ok(contact) = hc
                        .call_zome::<serde_json::Value, serde_json::Value>(
                            "mail_contacts",
                            "get_contact_by_email",
                            &serde_json::json!(q),
                        )
                        .await
                    {
                        if let Some(contact) = crate::zome_adapter::adapt_contact_value(contact) {
                            dht_suggestions.set(vec![contact]);
                        }
                    }
                });
            }
        }
        results.truncate(8);
        results
    };

    view! {
        <div class="page page-compose">
            <div class="page-header">
                <h1>{mode_label}</h1>
                <div class="compose-actions">
                    <button class="btn btn-secondary" on:click=on_save_draft disabled=move || sending.get()>
                        "Save Draft"
                    </button>
                    <button class="btn btn-primary" on:click=on_send disabled=move || sending.get()>
                        {move || if sending.get() { "Sending..." } else { "Send" }}
                    </button>
                </div>
            </div>

            // Keyboard shortcut: Ctrl+Enter sends while live.
            <div class="compose-form"
                 on:keydown=move |ev: web_sys::KeyboardEvent| {
                     if ev.ctrl_key() && ev.key() == "Enter" {
                         ev.prevent_default();
                         // Trigger send button click
                         let _ = js_sys::eval("document.querySelector('.page-compose .btn-primary')?.click()");
                     }
                 }>
                <div class="compose-shortcuts-hint">
                    <kbd>"Ctrl+Enter"</kbd>" send  "
                    <kbd>"Tab"</kbd>" complete contact"
                </div>
                // Template picker
                <div class="template-bar">
                    <button class="btn btn-sm btn-secondary" on:click=move |_| show_templates.update(|v| *v = !*v)>
                        {move || if show_templates.get() { "\u{25BC} Templates" } else { "\u{25B6} Templates" }}
                    </button>
                    <button class="btn btn-sm btn-secondary" style=move || if show_templates.get() { "" } else { "display:none" }
                        on:click=move |_| {
                            // Save current compose content as a new template
                            let subj = subject_field.get_untracked();
                            let body = body_field.get_untracked();
                            if subj.is_empty() && body.is_empty() {
                                toasts.push("Write something first to save as template", "error");
                                return;
                            }
                            let name = if !subj.is_empty() { subj.clone() } else { "Untitled Template".into() };
                            mail.templates.update(|tpls| {
                                tpls.push(mail_leptos_types::EmailTemplate {
                                    id: format!("tpl-{}", js_sys::Date::now() as u64),
                                    name,
                                    subject: subj,
                                    body,
                                    use_pqc: true,
                                });
                            });
                            toasts.push("Template saved", "success");
                        }
                    >
                        "+ Save as Template"
                    </button>
                    <div class="template-list" style=move || if show_templates.get() { "" } else { "display:none" }>
                        {move || mail.templates.get().iter().map(|tpl| {
                            let name = tpl.name.clone();
                            let subj = tpl.subject.clone();
                            let body = tpl.body.clone();
                            let t = toasts_template.clone();
                            let name_toast = name.clone();
                            view! {
                                <button class="template-item" on:click=move |_| {
                                    subject_field.set(subj.clone());
                                    body_field.set(body.clone());
                                    show_templates.set(false);
                                    t.push(format!("Template: {}", name_toast), "info");
                                }>
                                    <span class="template-name">{name.clone()}</span>
                                    <span class="template-preview">{subj.clone()}</span>
                                </button>
                            }
                        }).collect::<Vec<_>>()}
                    </div>
                </div>

                <div class="form-field">
                    <label for="to">"To"</label>
                    <input
                        id="to"
                        type="text"
                        placeholder="Recipient agent key or contact name"
                        prop:value=move || to_field.get()
                        on:input=move |ev| to_field.set(event_target_value(&ev))
                        on:keydown=move |ev: web_sys::KeyboardEvent| {
                            if ev.key() == "Tab" {
                                let suggestions = contact_suggestions();
                                if let Some(first) = suggestions.first() {
                                    ev.prevent_default();
                                    to_field.set(first.agent_pub_key.clone().unwrap_or_default());
                                }
                            }
                        }
                    />
                    {move || {
                        let suggestions = contact_suggestions();
                        (!suggestions.is_empty()).then(|| view! {
                            <div class="suggestions">
                                {suggestions.into_iter().map(|c| {
                                    let name = c.display_name.clone();
                                    let email = c.email.clone().unwrap_or_default();
                                    let agent = c.agent_pub_key.clone().unwrap_or_default();
                                    view! {
                                        <button
                                            class="suggestion-item"
                                            on:click=move |_| to_field.set(agent.clone())
                                        >
                                            <span class="suggestion-name">{name.clone()}</span>
                                            <span class="suggestion-email">{email.clone()}</span>
                                        </button>
                                    }
                                }).collect::<Vec<_>>()}
                            </div>
                        })
                    }}
                </div>

                <button class="toggle-cc" on:click=move |_| show_cc.update(|v| *v = !*v)>
                    {move || if show_cc.get() { "Hide CC" } else { "Show CC/BCC" }}
                </button>

                {move || show_cc.get().then(|| view! {
                    <div class="form-field">
                        <label for="cc">"CC"</label>
                        <input
                            id="cc"
                            type="text"
                            placeholder="CC recipients"
                            prop:value=move || cc_field.get()
                            on:input=move |ev| cc_field.set(event_target_value(&ev))
                        />
                    </div>
                })}

                <div class="form-field">
                    <label for="subject">"Subject"</label>
                    <input
                        id="subject"
                        type="text"
                        placeholder="Subject"
                        prop:value=move || subject_field.get()
                        on:input=move |ev| subject_field.set(event_target_value(&ev))
                    />
                </div>

                <div class="form-field body-field">
                    <label>"Message"</label>
                    <RichEditor value=body_field placeholder="Write your message..." />
                    <crate::components::ToneIndicator text=body_field />
                </div>

                <div class="compose-options">
                    <span class="option-label">"\u{1F512} X25519 + AES-256-GCM + agent Ed25519 signature"</span>
                    // Template variable hint
                    <div class="template-vars-hint">
                        <details>
                            <summary class="vars-summary">"\u{1F4DD} Template variables"</summary>
                            <div class="vars-list">
                                <code>"{{name}}"</code>" — recipient name  "
                                <code>"{{date}}"</code>" — today's date  "
                                <code>"{{time}}"</code>" — current time  "
                                <code>"{{sender}}"</code>" — your name"
                                <p class="vars-note">"Variables are expanded when the message is sent."</p>
                            </div>
                        </details>
                    </div>
                </div>
            </div>
        </div>
    }
}

fn event_target_value(ev: &leptos::ev::Event) -> String {
    use wasm_bindgen::JsCast;
    ev.target()
        .and_then(|t| t.dyn_into::<web_sys::HtmlInputElement>().ok())
        .map(|el| el.value())
        .or_else(|| {
            ev.target()
                .and_then(|t| t.dyn_into::<web_sys::HtmlTextAreaElement>().ok())
                .map(|el| el.value())
        })
        .unwrap_or_default()
}
