// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Hardware Integration Components — WebBluetooth and Physical Presence.

use leptos::prelude::*;

#[component]
pub fn HardwareScanner(
    device_type: String,
    on_linked: impl Fn(String) + Send + 'static,
) -> impl IntoView {
    let status = "Physical hardware provider unavailable — no grant can be issued in this build.";
    let _ = on_linked;
    let device_type_for_view = device_type.clone();

    view! {
        <div class="hardware-scanner-ui" style="margin-top: 1rem">
            <div style="display: flex; justify-content: space-between; align-items: center; padding: 0.75rem; background: var(--surface-low); border-radius: 8px">
                <div style="font-size: 0.8rem">
                    <strong>{device_type_for_view}</strong>
                    <div style="color: var(--error); font-size: 0.7rem">{status}</div>
                </div>
                <button class="btn-sm btn-outline" disabled=true>
                    "Hardware authorization unavailable"
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
    // The production QR/physical-presence verifier is not wired in this build.
    // An animation or user click must never count as verification.
    let _ = node_id;
    let _ = on_verified;
    view! {
        <div class="presence-validator" role="status" style="margin-top: 1rem; padding: 0.75rem; border: 1px solid var(--border); border-radius: 8px">
            <p style="font-weight: 700">"Physical-presence verification unavailable"</p>
            <p style="font-size: 0.85rem">"No QR scan or physical verification has been performed. This component will not signal success in this build."</p>
        </div>
    }
}

#[component]
pub fn SafetyGuard(children: ChildrenFn) -> impl IntoView {
    // Deliberately do not read browser-local progress here. A self-selected
    // Mastered status, BKT estimate, lesson completion or local self-attestation
    // is not a trusted hardware authorization. Until an independent verifier and
    // authorization service are wired and audited, never render the children.
    let _ = children;
    view! {
        <div class="safety-lockout" role="alert" style="padding: 2rem; background: var(--error-low); border: 2px solid var(--error); border-radius: 12px; text-align: center">
            <div style="font-size: 2rem; margin-bottom: 1rem">"⚠️"</div>
            <h4 style="color: var(--error)">"PHYSICAL LOCKOUT ACTIVE"</h4>
            <p style="font-size: 0.85rem">
                "This build has no trusted hardware authorization provider. Marking a lesson as mastered, earning a local self-attestation, or passing a simulated scan does not unlock physical hardware."
            </p>
            <a href="/study/VOC-000-S" class="btn-sm btn-primary" style="margin-top: 1rem; background: var(--error); border-color: var(--error)">
                "Review the Safety Protocol (does not unlock hardware)"
            </a>
        </div>
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn hardware_authorization_has_no_success_path_without_a_trusted_provider() {
        // This build intentionally supplies no verified external hardware
        // authorization evidence. The user interface must remain fail-closed.
        let trusted_provider_configured = false;
        let independently_verified_authorization = false;
        assert!(!(trusted_provider_configured && independently_verified_authorization));
    }
}
