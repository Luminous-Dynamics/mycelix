// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! PWA install prompt — captures `beforeinstallprompt` without evaluated code.

use std::cell::RefCell;

use leptos::prelude::*;
use wasm_bindgen::{JsCast, JsValue, closure::Closure};

thread_local! {
    static DEFERRED_INSTALL_PROMPT: RefCell<Option<JsValue>> = RefCell::new(None);
}

#[component]
pub fn PwaInstallPrompt() -> impl IntoView {
    let can_install = RwSignal::new(false);
    let dismissed = RwSignal::new(
        web_sys::window()
            .and_then(|w| w.local_storage().ok().flatten())
            .and_then(|s| s.get_item("mycelix_pwa_dismissed").ok().flatten())
            .is_some(),
    );

    if let Some(window) = web_sys::window() {
        let install_available = can_install;
        let before_install =
            Closure::<dyn FnMut(web_sys::Event)>::new(move |event: web_sys::Event| {
                event.prevent_default();
                DEFERRED_INSTALL_PROMPT.with(|slot| {
                    *slot.borrow_mut() = Some(event.into());
                });
                install_available.set(true);
            });
        let _ = window.add_event_listener_with_callback(
            "beforeinstallprompt",
            before_install.as_ref().unchecked_ref(),
        );
        before_install.forget();

        let install_cleared = can_install;
        let app_installed = Closure::<dyn FnMut(web_sys::Event)>::new(move |_| {
            DEFERRED_INSTALL_PROMPT.with(|slot| {
                slot.borrow_mut().take();
            });
            install_cleared.set(false);
        });
        let _ = window.add_event_listener_with_callback(
            "appinstalled",
            app_installed.as_ref().unchecked_ref(),
        );
        app_installed.forget();
    }

    let on_install = move |_| {
        DEFERRED_INSTALL_PROMPT.with(|slot| {
            if let Some(event) = slot.borrow_mut().take() {
                if let Ok(prompt) = js_sys::Reflect::get(&event, &JsValue::from_str("prompt"))
                    .and_then(|value| value.dyn_into::<js_sys::Function>())
                {
                    let _ = prompt.call0(&event);
                }
            }
        });
        can_install.set(false);
    };

    let on_dismiss = move |_| {
        dismissed.set(true);
        can_install.set(false);
        DEFERRED_INSTALL_PROMPT.with(|slot| {
            slot.borrow_mut().take();
        });
        if let Some(storage) = web_sys::window().and_then(|w| w.local_storage().ok().flatten()) {
            let _ = storage.set_item("mycelix_pwa_dismissed", "1");
        }
    };

    view! {
        {move || {
            if !can_install.get() || dismissed.get() { return None; }
            Some(view! {
                <div class="pwa-install-banner">
                    <span class="pwa-install-icon">"📱"</span>
                    <div class="pwa-install-text">
                        <strong>"Install Mycelix Pulse"</strong>
                        <span>" — faster access, offline support"</span>
                    </div>
                    <button class="btn btn-small btn-primary" on:click=on_install>"Install"</button>
                    <button class="btn btn-small btn-ghost pwa-dismiss" on:click=on_dismiss>"✕"</button>
                </div>
            })
        }}
    }
}
