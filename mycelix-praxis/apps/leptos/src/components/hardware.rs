// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Hardware Integration Components — WebBluetooth and Physical Presence.

use leptos::prelude::*;

#[component]
pub fn HardwareScanner(
    device_type: String,
    on_linked: impl Fn(String) + Send + 'static,
) -> impl IntoView {
    let (status, set_status) = signal("Ready to Connect".to_string());
    let (is_scanning, set_is_scanning) = signal(false);
    let on_linked = std::sync::Arc::new(on_linked);

    let device_type_for_view = device_type.clone();
    let start_handshake = move |_| {
        set_is_scanning.set(true);
        set_status.set(format!("Scanning for {}...", device_type));

        // SIMULATED: WebBluetooth API Handshake
        let on_linked = on_linked.clone();
        wasm_bindgen_futures::spawn_local(async move {
            gloo_timers::future::sleep(std::time::Duration::from_millis(1500)).await;
            set_status.set("\u{1F7E2} LINK ESTABLISHED".to_string());
            set_is_scanning.set(false);
            on_linked("hw-grant-99b3".to_string());
        });
    };

    view! {
        <div class="hardware-scanner-ui" style="margin-top: 1rem">
            <div style="display: flex; justify-content: space-between; align-items: center; padding: 0.75rem; background: var(--surface-low); border-radius: 8px">
                <div style="font-size: 0.8rem">
                    <strong>{device_type_for_view}</strong>
                    <div style="color: var(--text-tertiary); font-size: 0.7rem">{move || status.get()}</div>
                </div>
                <button
                    class="btn-sm btn-primary"
                    on:click=start_handshake
                    disabled=move || is_scanning.get()
                >
                    "Connect"
                </button>
            </div>
        </div>
    }
}

#[component]
pub fn PresenceValidator(
    node_id: String,
    on_verified: impl Fn() + Send + Sync + 'static,
) -> impl IntoView {
    let (show_camera, set_show_camera) = signal(false);
    let on_verified = std::sync::Arc::new(on_verified);

    view! {
        <div class="presence-validator" style="margin-top: 1rem">
            {move || {
                let on_verified = on_verified.clone();
                if !show_camera.get() {
                view! {
                    <button class="btn-sm btn-outline" style="width: 100%" on:click=move |_| set_show_camera.set(true)>
                        "\u{1F4F7} Scan Workbench QR"
                    </button>
                }.into_any()
            } else {
                view! {
                    <div class="camera-preview" style="width: 100%; height: 200px; background: #000; border-radius: 8px; display: flex; align-items: center; justify-content: center; position: relative">
                        <span style="color: #fff; font-size: 0.7rem">"Scanning for verification hash..."</span>
                        <button
                            style="position: absolute; top: 5px; right: 5px; background: none; border: none; color: white; cursor: pointer"
                            on:click=move |_| set_show_camera.set(false)
                        >"\u{00D7}"</button>
                        // Simulated QR Success
                        <div
                            style="width: 80%; height: 2px; background: var(--accent); position: absolute; animation: scan 2s infinite"
                            on:animationiteration=move |_| {
                                set_show_camera.set(false);
                                on_verified();
                            }
                        ></div>
                    </div>
                }.into_any()
            }
            }}
        </div>
    }
}

/// Physical hardware must never be unlocked by a browser-local progress toggle.
///
/// This pre-alpha has no controller-verified safety credential, trusted hardware
/// interlock, or remote authorization receipt. Keep the demo fail-closed until
/// that complete path exists and is independently tested.
const VERIFIED_HARDWARE_AUTHORIZATION_CONNECTED: bool = false;

fn hardware_access_is_authorized() -> bool {
    VERIFIED_HARDWARE_AUTHORIZATION_CONNECTED
}

#[component]
pub fn SafetyGuard(children: ChildrenFn) -> impl IntoView {
    if hardware_access_is_authorized() {
        children().into_any()
    } else {
        // Do not render the guarded controls in this demo. A self-selected
        // "Mastered" status and simulated QR/handshake are not authorization.
        let _unused_guarded_controls = children;
        view! {
            <div class="safety-lockout" role="alert" style="padding: 2rem; background: var(--error-low); border: 2px solid var(--error); border-radius: 12px; text-align: center">
                <div style="font-size: 2rem; margin-bottom: 1rem">"\u{26A0}\u{FE0F}"</div>
                <h4 style="color: var(--error)">"PHYSICAL HARDWARE ACCESS DISABLED"</h4>
                <p style="font-size: 0.85rem">
                    "This preview has no verified hardware authorization path. Local progress, self-reported mastery, and simulated scans are not safety credentials."
                </p>
                <p style="font-size: 0.85rem">
                    "Access can only be enabled after controller-verified authorization and the required physical interlock are implemented and tested."
                </p>
            </div>
        }.into_any()
    }
}

#[cfg(test)]
mod safety_guard_tests {
    use super::hardware_access_is_authorized;

    #[test]
    fn hardware_access_fails_closed_until_verified_authorization_exists() {
        assert!(
            !hardware_access_is_authorized(),
            "browser-local lesson progress must not authorize physical hardware"
        );
    }
}
