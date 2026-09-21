// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Browser-local Hearth presentation preferences.
//!
//! These values are intentionally **not** Holochain records and carry no
//! membership, role, selection, governance, or mutation authority. They are
//! convenience/presentation state for this browser only.

use leptos::prelude::*;
use serde::{Deserialize, Serialize};

const STORAGE_KEY: &str = "mycelix-hearth-prefs-v1";
pub const MAX_MOTTO_CHARS: usize = 160;

#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
struct PersistedHearthPrefs {
    #[serde(default)]
    motto: String,
}

#[derive(Clone)]
pub struct HearthPrefsState {
    pub motto: RwSignal<String>,
}

impl HearthPrefsState {
    pub fn set_motto(&self, value: impl AsRef<str>) {
        self.motto.set(normalize_motto(value.as_ref()));
    }

    pub fn clear_motto(&self) {
        self.motto.set(String::new());
    }
}

fn normalize_motto(value: &str) -> String {
    value.trim().chars().take(MAX_MOTTO_CHARS).collect()
}

fn load_persisted() -> PersistedHearthPrefs {
    web_sys::window()
        .and_then(|window| window.local_storage().ok().flatten())
        .and_then(|storage| storage.get_item(STORAGE_KEY).ok().flatten())
        .and_then(|raw| serde_json::from_str::<PersistedHearthPrefs>(&raw).ok())
        .map(|mut prefs| {
            prefs.motto = normalize_motto(&prefs.motto);
            prefs
        })
        .unwrap_or_default()
}

pub fn provide_hearth_prefs() -> HearthPrefsState {
    let persisted = load_persisted();
    let state = HearthPrefsState {
        motto: RwSignal::new(persisted.motto),
    };

    let motto = state.motto;
    Effect::new(move |_| {
        let prefs = PersistedHearthPrefs {
            motto: normalize_motto(&motto.get()),
        };

        let Some(storage) = web_sys::window()
            .and_then(|window| window.local_storage().ok().flatten())
        else {
            return;
        };

        if let Ok(serialized) = serde_json::to_string(&prefs) {
            let _ = storage.set_item(STORAGE_KEY, &serialized);
        }
    });

    provide_context(state.clone());
    state
}

pub fn use_hearth_prefs() -> HearthPrefsState {
    expect_context::<HearthPrefsState>()
}

#[cfg(test)]
mod tests {
    use super::{MAX_MOTTO_CHARS, PersistedHearthPrefs, normalize_motto};

    #[test]
    fn motto_is_trimmed() {
        assert_eq!(normalize_motto("  keep the fire  "), "keep the fire");
    }

    #[test]
    fn motto_is_bounded_by_unicode_characters_not_bytes() {
        let raw = "🔥".repeat(MAX_MOTTO_CHARS + 10);
        let normalized = normalize_motto(&raw);
        assert_eq!(normalized.chars().count(), MAX_MOTTO_CHARS);
        assert!(normalized.is_char_boundary(normalized.len()));
    }

    #[test]
    fn old_or_missing_fields_fail_softly_to_defaults() {
        let empty: PersistedHearthPrefs = serde_json::from_str("{}").unwrap();
        assert!(empty.motto.is_empty());
        assert!(serde_json::from_str::<PersistedHearthPrefs>("not-json").is_err());
    }
}
