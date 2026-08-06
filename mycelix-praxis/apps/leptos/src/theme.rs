// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Somatic Theme Engine — UI Adaptation based on Nervous System Regulation.
//! Also hosts the plain light/dark/system UI theme toggle (an independent,
//! additive concern from the somatic engine below).

use crate::curriculum::{ProgressStatus, use_progress};
use crate::persistence;
use leptos::prelude::*;
use serde::{Deserialize, Serialize};

const THEME_KEY: &str = "praxis_theme";

/// Plain light/dark/system UI theme (distinct from `SomaticState` below,
/// which drives ambient nervous-system-based CSS, not user-chosen palette).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, Default)]
pub enum Theme {
    Light,
    #[default]
    Dark,
    System,
}

impl Theme {
    /// Cycle Light -> Dark -> System -> Light.
    pub fn next(&self) -> Theme {
        match self {
            Theme::Light => Theme::Dark,
            Theme::Dark => Theme::System,
            Theme::System => Theme::Light,
        }
    }

    pub fn label(&self) -> &'static str {
        match self {
            Theme::Light => "Light",
            Theme::Dark => "Dark",
            Theme::System => "System",
        }
    }

    pub fn icon(&self) -> &'static str {
        match self {
            Theme::Light => "\u{2600}\u{FE0F}",   // ☀️
            Theme::Dark => "\u{1F319}",           // 🌙
            Theme::System => "\u{1F5A5}\u{FE0F}", // 🖥️
        }
    }
}

/// Provide the selected theme as a context signal, following
/// `role::provide_role_context`'s exact template. Persists to localStorage
/// on every change, restores from it on startup.
pub fn provide_theme_context() -> (ReadSignal<Theme>, WriteSignal<Theme>) {
    let initial: Theme = persistence::load(THEME_KEY).unwrap_or_default();
    let (theme, set_theme) = signal(initial);

    Effect::new(move |_| {
        persistence::save(THEME_KEY, &theme.get());
    });

    provide_context(theme);
    provide_context(set_theme);
    (theme, set_theme)
}

/// Read the current theme from context (panics if not provided).
pub fn use_theme() -> ReadSignal<Theme> {
    expect_context::<ReadSignal<Theme>>()
}

/// Get the setter for the current theme from context.
pub fn use_set_theme() -> WriteSignal<Theme> {
    expect_context::<WriteSignal<Theme>>()
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum SomaticState {
    Regulated,
    HyperVigilant,
    Exhausted,
}

#[component]
pub fn SomaticThemeHandler(children: Children) -> impl IntoView {
    let progress = use_progress();

    // Derived: Infer somatic state from recent activity and somatic milestones
    let somatic_state = Memo::new(move |_| {
        let p = progress.get();
        let mastered_somatic = p
            .nodes
            .values()
            .filter(|n| n.status == ProgressStatus::Mastered && n.mastery_permille > 900)
            .count();

        if mastered_somatic > 5 {
            SomaticState::Regulated
        } else {
            SomaticState::HyperVigilant
        }
    });

    let theme_class = move || match somatic_state.get() {
        SomaticState::Regulated => "soma-zen",
        SomaticState::HyperVigilant => "soma-alert",
        SomaticState::Exhausted => "soma-recovery",
    };

    view! {
        <div class=theme_class>
            {children()}
        </div>
    }
}
