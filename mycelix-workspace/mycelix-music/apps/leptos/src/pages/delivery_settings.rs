// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

use leptos::prelude::*;

use crate::app::{DeliveryPreferences, MEDIA_GATEWAY_PREFERENCE_KEY};
use crate::delivery::parse_gateway_origins;

/// Per-browser media gateway selection. No public gateway is added implicitly.
#[component]
pub fn DeliverySettingsPage() -> impl IntoView {
    let preferences = expect_context::<DeliveryPreferences>();
    let origins_input = RwSignal::new(String::new());
    let validation_message = RwSignal::new(String::new());
    let status_message = RwSignal::new(String::new());

    // Reflect an existing browser preference when this route is mounted.
    Effect::new(move |_| {
        origins_input.set(
            preferences
                .gateway_origins
                .get()
                .unwrap_or_default(),
        );
    });

    let save_preference = move |_| {
        let value = origins_input.get_untracked();
        if value.trim().is_empty() {
            // An explicit empty preference means no media origins; it does not
            // mean "fall back to public gateways".
            preferences.gateway_origins.set(Some(String::new()));
            let persisted = web_sys::window()
                .and_then(|window| window.local_storage().ok().flatten())
                .is_some_and(|storage| {
                    storage
                        .set_item(MEDIA_GATEWAY_PREFERENCE_KEY, "")
                        .is_ok()
                });
            validation_message.set(String::new());
            status_message.set(if persisted {
                "Saved: media delivery is disabled for this browser until an origin is added.".into()
            } else {
                "Applied for this session, but browser storage is unavailable; the preference may not persist after reload.".into()
            });
            return;
        }

        match parse_gateway_origins(&value) {
            Ok(origins) => {
                let count = origins.len();
                preferences.gateway_origins.set(Some(value.clone()));
                let persisted = web_sys::window()
                    .and_then(|window| window.local_storage().ok().flatten())
                    .is_some_and(|storage| {
                        storage
                            .set_item(MEDIA_GATEWAY_PREFERENCE_KEY, &value)
                            .is_ok()
                    });
                validation_message.set(String::new());
                status_message.set(if persisted {
                    format!("Saved {count} unique gateway origin(s) in this browser.")
                } else {
                    format!("Applied {count} unique gateway origin(s) for this session, but browser storage is unavailable.")
                });
            }
            Err(error) => {
                validation_message.set(format!(
                    "Gateway list rejected: {error}. Use up to eight comma-separated origins. HTTPS is required except for localhost, 127.0.0.0/8, or [::1]."
                ));
                status_message.set(String::new());
            }
        }
    };

    let use_deployment_defaults = move |_| {
        preferences.gateway_origins.set(None);
        let removed = web_sys::window()
            .and_then(|window| window.local_storage().ok().flatten())
            .is_some_and(|storage| {
                storage
                    .remove_item(MEDIA_GATEWAY_PREFERENCE_KEY)
                    .is_ok()
            });
        origins_input.set(String::new());
        validation_message.set(String::new());
        status_message.set(if removed {
            "Using this deployment's configured gateway pool. Public origins are used only if the operator explicitly configured them.".into()
        } else {
            "Deployment defaults applied for this session; browser storage could not be updated.".into()
        });
    };

    view! {
        <div class="page delivery-settings-page">
            <h1>"Media Delivery Preferences"</h1>
            <p class="page-subtitle">
                "Choose which gateways this browser may contact for music and cover art."
            </p>

            <section class="upload-form" aria-labelledby="gateway-pool-title">
                <h2 id="gateway-pool-title">"Gateway origin pool"</h2>
                <p>
                    "Enter up to eight comma-separated origins in preference order. "
                    "Your browser tries these routes in order. A gateway choice is not proof that the returned bytes match the IPFS CID."
                </p>
                <div class="dependency-gate" role="note">
                    <strong>"Privacy matters."</strong>
                    <span>
                        " A gateway operator can observe your network address and the requested content identifier. "
                        "Use a local IPFS gateway if you have one, or list only operators you trust. "
                        "Public gateways are never added automatically."
                    </span>
                </div>

                <div class="form-group">
                    <label for="media-gateway-origins">"Ordered gateway origins"</label>
                    <textarea
                        id="media-gateway-origins"
                        rows="3"
                        placeholder="http://127.0.0.1:8080, https://media.example.org"
                        prop:value=move || origins_input.get()
                        on:input=move |ev| origins_input.set(event_target_value(&ev))
                        aria-describedby="gateway-origin-help"
                    />
                    <span id="gateway-origin-help" class="help-text">
                        "Example local Kubo gateway: http://127.0.0.1:8080. Non-loopback origins must use HTTPS. Paths, credentials, query strings, and fragments are rejected."
                    </span>
                </div>

                <div class="form-group">
                    <button class="btn btn-primary" on:click=save_preference>
                        "Save browser preference"
                    </button>
                    <button class="btn btn-secondary" on:click=use_deployment_defaults>
                        "Use deployment defaults"
                    </button>
                </div>

                <p role="alert" class="status-message">
                    {move || validation_message.get()}
                </p>
                <p role="status" class="status-message">
                    {move || status_message.get()}
                </p>
            </section>

            <p class="help-text">
                "Preferences are stored in this browser only; they are not published to Holochain or synchronized to other devices. "
                "An empty saved list deliberately disables media routes. Clearing the preference uses the deployment's explicitly configured pool."
            </p>
        </div>
    }
}
