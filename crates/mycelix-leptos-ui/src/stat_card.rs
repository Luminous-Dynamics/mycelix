// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Statistic card component.

use leptos::prelude::*;

/// A card displaying a labeled statistic.
///
/// Semantic class names are the primary styling contract. Theme-aware inline
/// fallbacks keep the component usable for consumers that have not imported the
/// shared stylesheet, without forcing a light theme or fixed palette.
#[component]
pub fn StatCard(
    label: &'static str,
    value: String,
    #[prop(optional)] subtitle: Option<String>,
    #[prop(optional)] icon: Option<&'static str>,
) -> impl IntoView {
    view! {
        <div
            class="stat-card"
            style="padding: var(--space-lg, 1rem) var(--space-xl, 1.5rem); \
                   border-radius: var(--radius-md, 14px); \
                   border: 1px solid var(--border-visible, #e5e7eb); \
                   background: var(--bg-raised, white); \
                   color: var(--text-primary, #111827); \
                   font-family: var(--font-sans, system-ui, sans-serif); min-width: 140px;"
        >
            <div
                class="stat-card-header"
                style="display: flex; align-items: center; gap: var(--space-sm, 0.5rem); \
                       margin-bottom: var(--space-xs, 0.25rem);"
            >
                {icon.map(|i| view! {
                    <span class="stat-card-icon" aria-hidden="true" style="font-size: 1rem;">{i}</span>
                })}
                <span
                    class="stat-card-label"
                    style="font-size: var(--text-xs, 0.75rem); font-weight: 500; \
                           color: var(--text-secondary, #6b7280); text-transform: uppercase; \
                           letter-spacing: 0.05em;"
                >
                    {label}
                </span>
            </div>
            <div
                class="stat-card-value"
                style="font-size: var(--text-xl, 1.5rem); font-weight: 700; \
                       color: var(--text-primary, #111827); line-height: var(--leading-tight, 1.2);"
            >
                {value}
            </div>
            {subtitle.map(|sub| {
                view! {
                    <div
                        class="stat-card-subtitle"
                        style="font-size: var(--text-xs, 0.75rem); color: var(--text-muted, #9ca3af); \
                               margin-top: var(--space-xs, 0.25rem);"
                    >
                        {sub}
                    </div>
                }
            })}
        </div>
    }
}
