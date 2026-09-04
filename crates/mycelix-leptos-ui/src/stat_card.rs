// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Statistic card component.

use leptos::prelude::*;

/// A card displaying a labeled statistic.
///
/// Layout and color are intentionally expressed through semantic class names
/// so domain themes can restyle the card without replacing this component.
#[component]
pub fn StatCard(
    label: &'static str,
    value: String,
    #[prop(optional)] subtitle: Option<String>,
    #[prop(optional)] icon: Option<&'static str>,
) -> impl IntoView {
    view! {
        <div class="stat-card">
            <div class="stat-card-header">
                {icon.map(|i| view! {
                    <span class="stat-card-icon" aria-hidden="true">{i}</span>
                })}
                <span class="stat-card-label">{label}</span>
            </div>
            <div class="stat-card-value">{value}</div>
            {subtitle.map(|sub| {
                view! { <div class="stat-card-subtitle">{sub}</div> }
            })}
        </div>
    }
}
