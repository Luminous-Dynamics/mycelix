// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Notification sounds + favicon badge (#5).

use crate::mail_context::use_mail;
use leptos::prelude::*;
use wasm_bindgen::JsCast;
use wasm_bindgen::prelude::*;

#[derive(Clone, Copy)]
pub struct NotificationState {
    pub tab_visible: RwSignal<bool>,
    pub sound_enabled: RwSignal<bool>,
}

pub fn provide_notification_context() {
    let state = NotificationState {
        tab_visible: RwSignal::new(true),
        sound_enabled: RwSignal::new(true),
    };
    provide_context(state);

    // Track tab visibility
    let visible = state.tab_visible;
    let closure = Closure::<dyn Fn()>::new(move || {
        let doc = web_sys::window().unwrap().document().unwrap();
        let hidden = js_sys::Reflect::get(&doc, &JsValue::from_str("hidden"))
            .unwrap_or(JsValue::TRUE)
            .as_bool()
            .unwrap_or(true);
        visible.set(!hidden);
    });
    let doc = web_sys::window().unwrap().document().unwrap();
    let _ =
        doc.add_event_listener_with_callback("visibilitychange", closure.as_ref().unchecked_ref());
    closure.forget();

    // Reactive title update based on unread count
    let mail = use_mail();
    Effect::new(move |_| {
        let unread = mail.inbox.get().iter().filter(|e| !e.is_read).count();
        let doc = web_sys::window().unwrap().document().unwrap();
        if unread > 0 {
            doc.set_title(&format!("Mycelix Pulse ({unread})"));
        } else {
            doc.set_title("Mycelix Pulse");
        }
    });
}

pub fn use_notifications() -> NotificationState {
    expect_context::<NotificationState>()
}

fn window_constructor(name: &str) -> Option<js_sys::Function> {
    let window = web_sys::window()?;
    js_sys::Reflect::get(window.as_ref(), &JsValue::from_str(name))
        .ok()?
        .dyn_into::<js_sys::Function>()
        .ok()
}

fn call_method0(target: &JsValue, name: &str) -> Result<JsValue, JsValue> {
    let method = js_sys::Reflect::get(target, &JsValue::from_str(name))?
        .dyn_into::<js_sys::Function>()?;
    method.call0(target)
}

fn call_method1(target: &JsValue, name: &str, arg: &JsValue) -> Result<JsValue, JsValue> {
    let method = js_sys::Reflect::get(target, &JsValue::from_str(name))?
        .dyn_into::<js_sys::Function>()?;
    method.call1(target, arg)
}

/// Return the browser notification permission, or `None` when unsupported.
pub fn notification_permission() -> Option<String> {
    let constructor = window_constructor("Notification")?;
    js_sys::Reflect::get(constructor.as_ref(), &JsValue::from_str("permission"))
        .ok()?
        .as_string()
}

/// Request notification permission on first user interaction.
pub fn request_notification_permission() {
    if notification_permission().as_deref() != Some("default") {
        return;
    }
    if let Some(constructor) = window_constructor("Notification") {
        if let Ok(method) = js_sys::Reflect::get(
            constructor.as_ref(),
            &JsValue::from_str("requestPermission"),
        )
        .and_then(|value| value.dyn_into::<js_sys::Function>())
        {
            let _ = method.call0(constructor.as_ref());
        }
    }
}

/// Show a desktop notification for a new email.
pub fn show_desktop_notification(sender: &str, subject: &str) {
    if notification_permission().as_deref() != Some("granted") {
        return;
    }
    let Some(constructor) = window_constructor("Notification") else {
        return;
    };

    let options = js_sys::Object::new();
    let _ = js_sys::Reflect::set(
        &options,
        &JsValue::from_str("body"),
        &JsValue::from_str(subject),
    );
    let _ = js_sys::Reflect::set(
        &options,
        &JsValue::from_str("icon"),
        &JsValue::from_str("/icons/icon-96.png"),
    );
    let _ = js_sys::Reflect::set(
        &options,
        &JsValue::from_str("tag"),
        &JsValue::from_str("mycelix-pulse"),
    );

    let args = js_sys::Array::new();
    args.push(&JsValue::from_str(sender));
    args.push(&options);
    let _ = js_sys::Reflect::construct(&constructor, &args);
}

/// Play a short notification beep using the Web Audio API.
pub fn play_notification_sound() {
    let state = use_notifications();
    if !state.sound_enabled.get_untracked() || state.tab_visible.get_untracked() {
        return;
    }

    let constructor = window_constructor("AudioContext")
        .or_else(|| window_constructor("webkitAudioContext"));
    let Some(constructor) = constructor else {
        return;
    };
    let Ok(context) = js_sys::Reflect::construct(&constructor, &js_sys::Array::new()) else {
        return;
    };
    let Ok(oscillator) = call_method0(&context, "createOscillator") else {
        return;
    };
    let Ok(gain) = call_method0(&context, "createGain") else {
        return;
    };

    if let Ok(frequency) = js_sys::Reflect::get(&oscillator, &JsValue::from_str("frequency")) {
        let _ = js_sys::Reflect::set(
            &frequency,
            &JsValue::from_str("value"),
            &JsValue::from_f64(880.0),
        );
    }
    if let Ok(gain_param) = js_sys::Reflect::get(&gain, &JsValue::from_str("gain")) {
        let _ = js_sys::Reflect::set(
            &gain_param,
            &JsValue::from_str("value"),
            &JsValue::from_f64(0.08),
        );
    }
    let _ = call_method1(&oscillator, "connect", &gain);
    if let Ok(destination) = js_sys::Reflect::get(&context, &JsValue::from_str("destination")) {
        let _ = call_method1(&gain, "connect", &destination);
    }
    let _ = call_method0(&oscillator, "start");
    let stop_at = js_sys::Reflect::get(&context, &JsValue::from_str("currentTime"))
        .ok()
        .and_then(|value| value.as_f64())
        .unwrap_or(0.0)
        + 0.1;
    let _ = call_method1(&oscillator, "stop", &JsValue::from_f64(stop_at));
}
