// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Progress bar component.

use leptos::prelude::*;

/// A horizontal progress bar.
///
/// Visual policy lives in the shared Mycelix stylesheet. The optional `color`
/// prop remains as an escape hatch and is expressed through the
/// `--progress-color` custom property rather than hard-coding the component's
/// normal theme.
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
        Some(color) => format!("width: {pct:.1}%; --progress-color: {color};"),
        None => format!("width: {pct:.1}%;"),
    };

    view! {
        <div
            class="progress-bar"
            role="progressbar"
            aria-label=aria_label
            aria-valuenow=pct
            aria-valuemin=0.0
            aria-valuemax=100.0
        >
            <div class="progress-bar-fill" style=fill_style></div>
            {label.map(|text| {
                view! { <span class="progress-bar-label" aria-hidden="true">{text}</span> }
            })}
        </div>
    }
}
