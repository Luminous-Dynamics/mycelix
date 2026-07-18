// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Explicit application data modes.

use leptos::prelude::*;
use serde::{Deserialize, Serialize};

use crate::persistence;

const APP_MODE_KEY: &str = "praxis_app_mode";

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AppMode {
    /// Representative sample data. Never presented as learner or network data.
    Demo,
    /// Learner-owned browser data with no conductor connection attempt.
    #[default]
    Local,
    /// A conductor is required; transport or zome failures remain visible.
    Live,
}

impl AppMode {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Demo => "demo",
            Self::Local => "local",
            Self::Live => "live",
        }
    }

    fn from_str(value: &str) -> Option<Self> {
        match value {
            "demo" => Some(Self::Demo),
            "local" => Some(Self::Local),
            "live" => Some(Self::Live),
            _ => None,
        }
    }
}

pub fn provide_app_mode() -> (ReadSignal<AppMode>, WriteSignal<AppMode>) {
    let initial = persistence::load::<AppMode>(APP_MODE_KEY).unwrap_or_default();
    let (mode, set_mode) = signal(initial);
    provide_context(mode);
    provide_context(set_mode);
    (mode, set_mode)
}

pub fn use_app_mode() -> ReadSignal<AppMode> {
    expect_context::<ReadSignal<AppMode>>()
}

#[component]
pub fn ModeSelector() -> impl IntoView {
    let mode = use_app_mode();
    let set_mode = expect_context::<WriteSignal<AppMode>>();

    view! {
        <label class="mode-selector">
            <span>"Data mode"</span>
            <select
                aria-label="Praxis data mode"
                prop:value=move || mode.get().as_str()
                on:change=move |event| {
                    let value = event_target_value(&event);
                    let Some(next_mode) = AppMode::from_str(&value) else {
                        return;
                    };
                    if next_mode == mode.get_untracked() {
                        return;
                    }
                    persistence::save(APP_MODE_KEY, &next_mode);
                    set_mode.set(next_mode);
                    if let Some(window) = web_sys::window() {
                        let _ = window.location().reload();
                    }
                }
            >
                <option value="demo">"Demo"</option>
                <option value="local">"Local"</option>
                <option value="live">"Live"</option>
            </select>
        </label>
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn local_is_the_safe_default() {
        assert_eq!(AppMode::default(), AppMode::Local);
    }

    #[test]
    fn mode_values_round_trip() {
        for mode in [AppMode::Demo, AppMode::Local, AppMode::Live] {
            assert_eq!(AppMode::from_str(mode.as_str()), Some(mode));
        }
    }
}
