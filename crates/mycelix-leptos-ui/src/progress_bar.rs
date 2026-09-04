// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Progress bar component.

use leptos::prelude::*;

/// A horizontal progress bar.
///
/// The component exposes semantic class names and CSS custom properties so a
/// domain stylesheet can take over presentation. Inline values are deliberately
/// limited to theme-aware fallbacks and the dynamic progress width, preserving
/// correct rendering for consumers that have not imported the shared stylesheet.
#[component]
pub fn ProgressBar(
    value: f64,
    #[prop(optional)] label: Option<String>,
    #[prop(optional)] color: Option<String>,
) -> impl IntoView {
    let clamped = value.clamp(0.0, 1.0);
    let pct = clamped * 100.0;
    let aria_label = label.clone().unwrap_or_else(|| "Progress".to_string());
    let fill_style = match color {
        Some(color) => format!(
            "width: {pct:.1}%; --progress-color: {color}; \
             height: 100%; background: var(--progress-color, var(--primary, var(--mycelix-cyan, #2563eb))); \
             border-radius: inherit; transition: width var(--duration-interact, 0.3s) var(--ease-interact, ease);"
        ),
        None => format!(
            "width: {pct:.1}%; height: 100%; \
             background: var(--progress-color, var(--primary, var(--mycelix-cyan, #2563eb))); \
             border-radius: inherit; transition: width var(--duration-interact, 0.3s) var(--ease-interact, ease);"
        ),
    };

    view! {
        <div
            class="progress-bar"
            style="position: relative; width: 100%; height: 20px; \
                   background: var(--progress-track, var(--bg-surface, #e5e7eb)); \
                   border-radius: var(--radius-pill, 9999px); overflow: hidden; \
                   font-family: var(--font-sans, system-ui, sans-serif);"
            role="progressbar"
            aria-label=aria_label
            aria-valuenow=pct
            aria-valuemin=0.0
            aria-valuemax=100.0
        >
            <div class="progress-bar-fill" style=fill_style></div>
            {label.map(|text| {
                view! {
                    <span
                        class="progress-bar-label"
                        aria-hidden="true"
                        style="position: absolute; inset: 0; display: flex; \
                               align-items: center; justify-content: center; \
                               font-size: var(--text-xs, 0.75rem); font-weight: 600; \
                               color: var(--progress-label, var(--text-primary, #1f2937));"
                    >
                        {text}
                    </span>
                }
            })}
        </div>
    }
}
