// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Explicit Hearth runtime mode.
//!
//! Live is fail-closed: conductor failure must never silently turn a household
//! session into simulated data. Demo is deliberate and permanently identified
//! in the UI.

use leptos::prelude::*;
use mycelix_leptos_core::ConnectStrategy;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum HearthRuntimeMode {
    #[default]
    Live,
    Demo,
}

impl HearthRuntimeMode {
    pub fn connect_strategy(self) -> ConnectStrategy {
        match self {
            Self::Live => ConnectStrategy::WebSocketRequired,
            Self::Demo => ConnectStrategy::MockOnly,
        }
    }

    pub fn is_demo(self) -> bool {
        matches!(self, Self::Demo)
    }
}

/// Resolve runtime mode from an optional host override and URL query string.
///
/// The host override wins so native/launcher shells can pin the expected mode.
/// Unknown values fail closed to Live. Browser users can explicitly request a
/// demo with `?mode=demo` (or `?mode=mock`).
fn from_sources(host_override: Option<&str>, query: &str) -> HearthRuntimeMode {
    if let Some(value) = host_override {
        return match value.trim().to_ascii_lowercase().as_str() {
            "demo" | "mock" => HearthRuntimeMode::Demo,
            "live" => HearthRuntimeMode::Live,
            _ => HearthRuntimeMode::Live,
        };
    }

    let requested_demo = query
        .trim_start_matches('?')
        .split('&')
        .filter_map(|pair| pair.split_once('='))
        .any(|(key, value)| {
            key.eq_ignore_ascii_case("mode")
                && matches!(value.to_ascii_lowercase().as_str(), "demo" | "mock")
        });

    if requested_demo {
        HearthRuntimeMode::Demo
    } else {
        HearthRuntimeMode::Live
    }
}

pub fn detect_runtime_mode() -> HearthRuntimeMode {
    let Some(window) = web_sys::window() else {
        return HearthRuntimeMode::Live;
    };

    let host_override = js_sys::Reflect::get(&window, &"__MYCELIX_HEARTH_MODE".into())
        .ok()
        .and_then(|value| value.as_string());
    let query = window.location().search().unwrap_or_default();

    from_sources(host_override.as_deref(), &query)
}

pub fn provide_runtime_mode(mode: HearthRuntimeMode) {
    provide_context(mode);
}

pub fn use_runtime_mode() -> HearthRuntimeMode {
    expect_context::<HearthRuntimeMode>()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn defaults_to_live() {
        assert_eq!(from_sources(None, ""), HearthRuntimeMode::Live);
    }

    #[test]
    fn query_can_explicitly_select_demo() {
        assert_eq!(
            from_sources(None, "?mode=demo"),
            HearthRuntimeMode::Demo
        );
        assert_eq!(
            from_sources(None, "?foo=1&mode=mock"),
            HearthRuntimeMode::Demo
        );
    }

    #[test]
    fn host_override_has_precedence() {
        assert_eq!(
            from_sources(Some("live"), "?mode=demo"),
            HearthRuntimeMode::Live
        );
        assert_eq!(
            from_sources(Some("demo"), ""),
            HearthRuntimeMode::Demo
        );
    }

    #[test]
    fn unknown_override_fails_closed() {
        assert_eq!(
            from_sources(Some("surprise"), "?mode=demo"),
            HearthRuntimeMode::Live
        );
    }
}
