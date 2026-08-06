// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Mandatory profile setup for production.
//!
//! On first load with a live conductor, checks `mail_profiles.get_my_profile`.
//! If no profile exists, shows a blocking modal that MUST be completed.
//! No skip button — profile creation is required to use the app.

use leptos::prelude::*;
use wasm_bindgen_futures::spawn_local;

use crate::holochain::{ConnectionStatus, use_holochain};
use crate::toasts::use_toasts;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum SetupStep {
    Checking,
    Connecting, // Visible: shows "Connecting to network..."
    NameEntry,
    KeyGen,
    Complete,
    HasProfile,
}

fn attempt_is_current(attempt: RwSignal<u64>, expected: u64) -> bool {
    attempt.get_untracked() == expected
}

fn log_setup_stage(stage: &str, started_ms: f64, success: bool) {
    let elapsed_ms = (js_sys::Date::now() - started_ms).max(0.0).round() as u64;
    let outcome = if success { "ok" } else { "error" };
    web_sys::console::log_1(
        &format!("[Mail][onboarding] stage={stage} outcome={outcome} elapsed_ms={elapsed_ms}")
            .into(),
    );
}

#[component]
pub fn ProfileSetup() -> impl IntoView {
    let hc = use_holochain();
    let toasts = use_toasts();
    let step = RwSignal::new(SetupStep::Checking);
    let display_name = RwSignal::new(String::new());
    let bio = RwSignal::new(String::new());
    let key_status = RwSignal::new(String::new());
    let error_msg = RwSignal::new(String::new());
    // Incrementing this generation invalidates late completions from an older
    // onboarding attempt without pretending an in-flight DHT write was undone.
    let setup_attempt = RwSignal::new(0u64);

    // On production, show connecting state after 3 seconds if still checking
    if !crate::mail_context::is_demo_mode() {
        spawn_local(async move {
            gloo_timers::future::sleep(std::time::Duration::from_secs(3)).await;
            if step.get_untracked() == SetupStep::Checking {
                step.set(SetupStep::Connecting);
            }
        });
    }

    // Check if profile exists whenever connection status changes
    let hc_check = hc.clone();
    Effect::new(move |_| {
        let status = hc_check.status.get();
        let current_step = step.get_untracked();

        // Only check when waiting for connection (not during setup flow)
        if status == ConnectionStatus::Connected
            && (current_step == SetupStep::Checking || current_step == SetupStep::Connecting)
        {
            let hc = hc_check.clone();
            spawn_local(async move {
                // Small delay to let JS bridge initialize
                gloo_timers::future::sleep(std::time::Duration::from_millis(500)).await;

                match hc
                    .call_zome::<(), serde_json::Value>("mail_profiles", "get_my_profile", &())
                    .await
                {
                    Ok(val) if !val.is_null() => {
                        web_sys::console::log_1(&"[Mail] Profile exists".into());
                        step.set(SetupStep::HasProfile);
                    }
                    Ok(_) => {
                        web_sys::console::log_1(&"[Mail] No profile — setup required".into());
                        step.set(SetupStep::NameEntry);
                    }
                    Err(error) => {
                        web_sys::console::warn_1(
                            &format!("[Mail] get_my_profile failed closed: {error}").into(),
                        );
                        error_msg.set(format!(
                            "Could not determine whether a profile already exists: {error}"
                        ));
                        // Never interpret a read failure as profile absence.
                        step.set(SetupStep::Connecting);
                    }
                }
            });
        } else if crate::mail_context::is_demo_mode() {
            // Demo mode — no profile needed
            step.set(SetupStep::HasProfile);
        } else if status == ConnectionStatus::Demo
            && (current_step == SetupStep::Checking || current_step == SetupStep::Connecting)
        {
            // Explicit Demo mode does not require a DHT profile.
            step.set(SetupStep::HasProfile);
        }
    });

    // Profile creation handler
    let do_create = {
        let hc_create = hc.clone();
        let toasts_create = toasts.clone();
        std::rc::Rc::new(move || {
            let name = display_name.get_untracked();
            if name.trim().is_empty() {
                return;
            }
            let bio_val = bio.get_untracked();
            setup_attempt.update(|attempt| *attempt = attempt.wrapping_add(1));
            let attempt_id = setup_attempt.get_untracked();
            step.set(SetupStep::KeyGen);
            error_msg.set(String::new());
            key_status.set("Saving profile to DHT...".into());

            let hc = hc_create.clone();
            let toasts = toasts_create.clone();
            spawn_local(async move {
                let profile = serde_json::json!({
                    "name": name.trim(),
                    "email": "",
                    "avatar_url": "",
                    "bio": bio_val.trim(),
                });
                let started = js_sys::Date::now();
                let profile_result = hc
                    .call_zome::<serde_json::Value, serde_json::Value>(
                        "mail_profiles",
                        "set_profile",
                        &profile,
                    )
                    .await;
                log_setup_stage("set_profile", started, profile_result.is_ok());
                if !attempt_is_current(setup_attempt, attempt_id) {
                    return;
                }
                match profile_result {
                    Ok(_) => key_status.set("Profile created!".into()),
                    Err(error) => {
                        web_sys::console::warn_1(
                            &format!("[Mail] set_profile error: {error}").into(),
                        );
                        error_msg.set(format!("Could not save profile: {error}"));
                        key_status.set(String::new());
                        step.set(SetupStep::NameEntry);
                        return;
                    }
                }

                // The restricted alpha has no identity role. Do not send a
                // call that is guaranteed to fail; keep the profile/key slice
                // truthful and defer DSID registry onboarding to a later hApp.
                if crate::alpha_scope::identity_role_available() {
                    key_status.set("Creating your DSID...".into());
                    let started = js_sys::Date::now();
                    let did_result = hc
                        .call_zome_on_role::<(), serde_json::Value>(
                            "identity",
                            "did_registry",
                            "create_did",
                            &(),
                        )
                        .await;
                    log_setup_stage("create_did", started, did_result.is_ok());
                    if !attempt_is_current(setup_attempt, attempt_id) {
                        return;
                    }
                    if let Err(error) = did_result {
                        web_sys::console::warn_1(
                            &format!("[Mail] create_did skipped: {error}").into(),
                        );
                    }
                } else {
                    web_sys::console::log_1(
                        &"[Mail] Identity role not bundled; continuing with alpha profile + V2 keys"
                            .into(),
                    );
                }

                key_status.set("Checking your published hybrid PQC identity...".into());
                let started = js_sys::Date::now();
                let bundle_result = hc
                    .call_zome::<(), serde_json::Value>(
                        "mail_keys",
                        "get_my_hybrid_key_bundle_v2",
                        &(),
                    )
                    .await;
                log_setup_stage(
                    "get_my_hybrid_key_bundle_v2",
                    started,
                    bundle_result.is_ok(),
                );
                if !attempt_is_current(setup_attempt, attempt_id) {
                    return;
                }
                let existing = match bundle_result {
                    Ok(value) => (!value.is_null()).then_some(value),
                    Err(error) => {
                        error_msg.set(format!(
                            "Could not inspect the existing V2 identity: {error}"
                        ));
                        key_status.set(String::new());
                        step.set(SetupStep::NameEntry);
                        return;
                    }
                };

                if existing.is_some() {
                    // Never silently replace a published identity when its
                    // device-local private half is missing.
                    let started = js_sys::Date::now();
                    let load_result = crate::device_keystore::load_hybrid_identity().await;
                    log_setup_stage("load_device_identity", started, load_result.is_ok());
                    if !attempt_is_current(setup_attempt, attempt_id) {
                        return;
                    }
                    if let Err(error) = load_result {
                        let message =
                            format!("Cannot recover this device's encryption keys: {error}");
                        error_msg.set(message.clone());
                        key_status.set(String::new());
                        step.set(SetupStep::NameEntry);
                        toasts.push(message, "error");
                        return;
                    }
                } else {
                    // A previous publish may have timed out after committing.
                    // Reuse any already-wrapped local identity instead of
                    // silently replacing it during a retry.
                    let started = js_sys::Date::now();
                    let has_local_result = crate::device_keystore::has_hybrid_identity().await;
                    log_setup_stage("has_device_identity", started, has_local_result.is_ok());
                    if !attempt_is_current(setup_attempt, attempt_id) {
                        return;
                    }
                    let has_local = match has_local_result {
                        Ok(value) => value,
                        Err(error) => {
                            error_msg.set(error.clone());
                            key_status.set(String::new());
                            step.set(SetupStep::NameEntry);
                            toasts.push(error, "error");
                            return;
                        }
                    };

                    let public = if has_local {
                        key_status.set("Reusing your device-local hybrid PQC identity...".into());
                        let started = js_sys::Date::now();
                        let load_result = crate::device_keystore::load_hybrid_identity().await;
                        log_setup_stage(
                            "load_unpublished_device_identity",
                            started,
                            load_result.is_ok(),
                        );
                        if !attempt_is_current(setup_attempt, attempt_id) {
                            return;
                        }
                        match load_result {
                            Ok(identity) => identity.public_bundle(),
                            Err(error) => {
                                error_msg.set(error.clone());
                                key_status.set(String::new());
                                step.set(SetupStep::NameEntry);
                                toasts.push(error, "error");
                                return;
                            }
                        }
                    } else {
                        key_status.set("Creating a device-local hybrid PQC identity...".into());
                        let started = js_sys::Date::now();
                        let create_result =
                            crate::device_keystore::create_and_store_hybrid_identity().await;
                        log_setup_stage("create_device_identity", started, create_result.is_ok());
                        if !attempt_is_current(setup_attempt, attempt_id) {
                            return;
                        }
                        match create_result {
                            Ok(public) => public,
                            Err(error) => {
                                error_msg.set(error.clone());
                                key_status.set(String::new());
                                step.set(SetupStep::NameEntry);
                                toasts.push(error, "error");
                                return;
                            }
                        }
                    };
                    let created_at_us = (js_sys::Date::now() as u64) * 1000;
                    let bundle = serde_json::json!({
                        "version": 2u16,
                        "suite": mail_leptos_types::protocol::SUITE_X25519_MLKEM768_AES_256_GCM_AGENT_MLDSA65,
                        // The coordinator derives this content-addressed ID and
                        // signs the final transcript with the active agent key.
                        "key_id": vec![0u8; 32],
                        "x25519_public_key": public.x25519_public_key,
                        "ml_kem_768_public_key": public.ml_kem_768_public_key,
                        "ml_dsa_65_public_key": public.ml_dsa_65_public_key,
                        "state": "active",
                        "created_at": created_at_us,
                        "expires_at": created_at_us + (30 * 24 * 3600 * 1_000_000u64),
                        "agent_signature": vec![0u8; 64],
                    });
                    key_status.set("Publishing your hybrid PQC identity...".into());
                    let started = js_sys::Date::now();
                    // Decoded as `Vec<u8>`, not `serde_json::Value`: this
                    // zome returns a real `ActionHash` (needed by native
                    // callers, e.g. the Sweettest suite's
                    // `sender_mldsa_bundle_hash`/`recipient_bundle_hash`
                    // pointer plumbing — its return type can't just become
                    // `()`), whose raw-byte msgpack encoding
                    // `serde_json::Value` cannot represent ("invalid type:
                    // byte array, expected any valid JSON value"), even
                    // though this caller only checks Ok/Err and never reads
                    // the value.
                    let publish_result = hc
                        .call_zome::<serde_json::Value, Vec<u8>>(
                            "mail_keys",
                            "publish_hybrid_key_bundle_v2",
                            &bundle,
                        )
                        .await;
                    log_setup_stage(
                        "publish_hybrid_key_bundle_v2",
                        started,
                        publish_result.is_ok(),
                    );
                    if !attempt_is_current(setup_attempt, attempt_id) {
                        return;
                    }
                    if let Err(error) = publish_result {
                        let message = format!("Could not publish V2 keys: {error}");
                        error_msg.set(message.clone());
                        key_status.set(String::new());
                        step.set(SetupStep::NameEntry);
                        toasts.push(message, "error");
                        return;
                    }
                }

                if !attempt_is_current(setup_attempt, attempt_id) {
                    return;
                }
                toasts.push("Welcome to Mycelix Pulse!", "success");
                step.set(SetupStep::Complete);
                gloo_timers::future::sleep(std::time::Duration::from_millis(2000)).await;
                if attempt_is_current(setup_attempt, attempt_id) {
                    step.set(SetupStep::HasProfile);
                }
            });
        })
    };

    let do_create_enter = do_create.clone();
    let do_create_click = do_create.clone();

    let visible = move || {
        let s = step.get();
        s != SetupStep::HasProfile && s != SetupStep::Checking
    };
    let is_connecting = move || step.get() == SetupStep::Connecting;
    let is_name_entry = move || step.get() == SetupStep::NameEntry;
    let is_keygen = move || step.get() == SetupStep::KeyGen;
    let is_complete = move || step.get() == SetupStep::Complete;

    view! {
        <div style=move || if visible() {
            "position:fixed;inset:0;z-index:99990;background:rgba(8,8,12,0.95);display:flex;align-items:center;justify-content:center"
        } else { "display:none" }>
            <div style="position:relative;background:#1a1d2e;border-radius:16px;padding:28px;max-width:420px;width:92%;box-shadow:0 20px 60px rgba(0,0,0,0.6)">
                <div style="text-align:center;margin-bottom:20px">
                    <div style="font-size:2.5rem;margin-bottom:8px">"✉"</div>
                    <h2 style="font-size:1.4rem;font-weight:700;margin:0;color:#e2e6f0">"Welcome to Mycelix Pulse"</h2>
                    <p style="color:#8890a8;font-size:0.85rem;margin-top:6px">"Create your encrypted Pulse profile"</p>
                </div>

                // Connecting state — waiting for conductor
                <div style=move || if is_connecting() { "text-align:center;padding:20px 0" } else { "display:none" }>
                    <div class="loading-spinner" style="margin:0 auto 16px"></div>
                    <p style="color:#06D6C8;font-size:0.95rem;font-weight:500">"Connecting to the Holochain network..."</p>
                    <p style="color:#5c6380;font-size:0.75rem;margin-top:8px;line-height:1.5">
                        "Setting up encrypted connection to the conductor."<br/>
                        "This may take a few seconds on first load."
                    </p>
                    // Retry button after showing for a while
                    <button style="margin-top:16px;padding:8px 16px;background:transparent;border:1px solid #3a3f54;border-radius:8px;color:#8890a8;font-size:0.8rem;cursor:pointer"
                        on:click=move |_| {
                            // Force retry by reloading the page
                            let _ = web_sys::window().and_then(|w| w.location().reload().ok());
                        }>
                        "Retry Connection"
                    </button>
                </div>

                // Error message
                <div style=move || if error_msg.get().is_empty() { "display:none" } else {
                    "background:#2d1b1b;border:1px solid #5c2020;border-radius:8px;padding:8px 12px;margin-bottom:12px;font-size:0.8rem;color:#f87171"
                }>
                    {move || error_msg.get()}
                </div>

                // Name entry form
                <form style=move || if is_name_entry() { "" } else { "display:none" }
                    on:submit=move |e: web_sys::SubmitEvent| {
                        e.prevent_default();
                        do_create_enter();
                    }>
                    <div style="margin-bottom:14px">
                        <label for="setup-name" style="display:block;font-size:0.8rem;color:#8890a8;margin-bottom:4px;font-weight:500">"Display Name"</label>
                        <input id="setup-name" type="text"
                            style="width:100%;padding:12px 14px;background:#252838;border:1px solid #3a3f54;border-radius:8px;color:#e2e6f0;font-size:16px;box-sizing:border-box;font-family:inherit;outline:none"
                            placeholder="How others will see you"
                            autocomplete="name"
                            enterkeyhint="send"
                            required=true
                            prop:value=move || display_name.get()
                            on:input=move |e| display_name.set(event_target_value(&e))
                        />
                    </div>
                    <div style="margin-bottom:14px">
                        <label for="setup-bio" style="display:block;font-size:0.8rem;color:#8890a8;margin-bottom:4px;font-weight:500">"Bio (optional)"</label>
                        <input id="setup-bio" type="text"
                            style="width:100%;padding:12px 14px;background:#252838;border:1px solid #3a3f54;border-radius:8px;color:#e2e6f0;font-size:16px;box-sizing:border-box;font-family:inherit;outline:none"
                            placeholder="A short bio..."
                            prop:value=move || bio.get()
                            on:input=move |e| bio.set(event_target_value(&e))
                        />
                    </div>
                    <input type="submit"
                        style="width:100%;padding:14px;background:#06D6C8;border:none;border-radius:8px;color:#000;font-weight:700;font-size:1rem;cursor:pointer;margin-top:4px"
                        disabled=move || display_name.get().trim().is_empty()
                        value="Create Profile & Generate Keys"
                    />
                    <p style="font-size:0.7rem;color:#5c6380;text-align:center;margin-top:14px;line-height:1.5">
                        "Your profile and hybrid public keys are anchored to the Holochain DHT."<br/>
                        "The separate DSID registry is not bundled in this alpha. Recovery is same-browser only."
                    </p>
                </form>

                // Key generation progress
                <div style=move || if is_keygen() { "text-align:center;padding:20px 0" } else { "display:none" }>
                    <div class="loading-spinner" style="margin:0 auto 16px"></div>
                    <p style="color:#06D6C8;font-size:0.95rem;font-weight:500">{move || key_status.get()}</p>
                    <p style="color:#5c6380;font-size:0.72rem;margin-top:8px;line-height:1.5">
                        "Network operations are bounded and will fail rather than wait forever."
                    </p>
                    <button style="margin-top:14px;padding:8px 16px;background:transparent;border:1px solid #3a3f54;border-radius:8px;color:#8890a8;font-size:0.8rem;cursor:pointer"
                        on:click=move |_| {
                            setup_attempt.update(|attempt| *attempt = attempt.wrapping_add(1));
                            key_status.set(String::new());
                            error_msg.set(
                                "Stopped waiting. A DHT write that already completed may still exist; retrying is safe."
                                    .into(),
                            );
                            step.set(SetupStep::NameEntry);
                        }>
                        "Stop Waiting & Retry"
                    </button>
                </div>

                // Complete
                <div style=move || if is_complete() { "text-align:center;padding:20px 0" } else { "display:none" }>
                    <div style="font-size:3rem;margin-bottom:8px;color:#06D6C8">"✓"</div>
                    <p style="color:#06D6C8;font-weight:600;font-size:1.1rem">"You're all set!"</p>
                    <p style="color:#8890a8;font-size:0.85rem;margin-top:4px">"Your encrypted profile and V2 public keys are live on the network."</p>
                </div>
            </div>
        </div>
    }
}
