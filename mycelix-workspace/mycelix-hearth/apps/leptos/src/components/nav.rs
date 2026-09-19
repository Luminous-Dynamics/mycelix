// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

use crate::themes::use_theme;
use leptos::prelude::*;
use leptos_router::components::A;
use mycelix_leptos_core::{ConnectionBadge, TaskShellRoutes};

const HEARTH_AREAS: [(&str, &str); 12] = [
    ("/kinship", "Bonds"),
    ("/care", "Care"),
    ("/gratitude", "Gratitude"),
    ("/stories", "Stories"),
    ("/milestones", "Milestones"),
    ("/rhythms", "Rhythms"),
    ("/decisions", "Decisions"),
    ("/emergency", "Emergency"),
    ("/resources", "Resources"),
    ("/autonomy", "Autonomy"),
    ("/found", "Founding"),
    ("/settings", "Settings"),
];

pub const fn hearth_task_routes() -> TaskShellRoutes {
    TaskShellRoutes::new("/", "/find", "/create", "/inbox", "/personal/profile")
}

#[component]
pub fn Nav() -> impl IntoView {
    let _theme_state = use_theme();
    let task_links = hearth_task_routes().desktop_nav();

    view! {
        <nav class="navbar" role="navigation" aria-label="main navigation">
            <A href="/" attr:class="logo" attr:aria-label="hearth home">"hearth"</A>

            <div class="nav-links">
                {task_links.into_iter().map(|link| view! {
                    <A href=link.href>{link.label}</A>
                }).collect_view()}

                <details class="hearth-area-disclosure">
                    <summary>"Explore Hearth"</summary>
                    <nav class="hearth-area-links" aria-label="Hearth areas">
                        {HEARTH_AREAS.into_iter().map(|(href, label)| view! {
                            <A href=href>{label}</A>
                        }).collect_view()}
                    </nav>
                </details>
            </div>

            <div class="nav-actions">
                <FontSizeToggle />
                <SoundToggle />
                <ThemeSwitcher />
                <ConnectionBadge />
            </div>
        </nav>
    }
}

/// Font size toggle for accessibility (Sage persona).
#[component]
fn FontSizeToggle() -> impl IntoView {
    let (size_level, set_size_level) = signal(0u8); // 0=normal, 1=large, 2=xlarge

    let cycle = move |_| {
        let next = (size_level.get() + 1) % 3;
        set_size_level.set(next);
        // Apply to <html> element
        use wasm_bindgen::JsCast;
        if let Some(root) = web_sys::window()
            .and_then(|w| w.document())
            .and_then(|d| d.document_element())
            .and_then(|e| e.dyn_ref::<web_sys::HtmlElement>().cloned())
        {
            let size = match next {
                1 => "18px",
                2 => "20px",
                _ => "16px",
            };
            let _ = root.style().set_property("font-size", size);
        }
    };

    view! {
        <button
            class="font-size-toggle"
            title="change text size"
            aria-label="change text size"
            on:click=cycle
        >
            {move || match size_level.get() {
                1 => "A+",
                2 => "A++",
                _ => "A",
            }}
        </button>
    }
}

/// Ambient sound toggle.
#[component]
fn SoundToggle() -> impl IntoView {
    let sound = crate::ambient_sound::use_ambient_sound();

    view! {
        <button
            class="sound-toggle"
            title=move || if sound.enabled.get() { "mute ambient" } else { "ambient sound" }
            aria-label=move || if sound.enabled.get() { "mute ambient sound" } else { "enable ambient sound" }
            on:click=move |_| sound.enabled.set(!sound.enabled.get())
        >
            <span aria-hidden="true">
                {move || if sound.enabled.get() { "\u{266b}" } else { "\u{266a}" }}
            </span>
        </button>
    }
}

/// Single button that cycles through themes on click.
#[component]
fn ThemeSwitcher() -> impl IntoView {
    let theme_state = use_theme();

    view! {
        <button
            class="theme-cycle-btn"
            title=move || {
                let t = theme_state.current.get();
                format!("{} — click to change", t.description())
            }
            aria-label=move || {
                let t = theme_state.current.get();
                format!("Theme: {}. Change theme", t.label())
            }
            on:click=move |_| {
                let next = theme_state.current.get().next();
                theme_state.current.set(next);
            }
        >
            <span class="theme-cycle-dot" aria-hidden="true"></span>
            <span class="theme-cycle-label">{move || theme_state.current.get().label()}</span>
        </button>
    }
}

#[cfg(test)]
mod tests {
    use super::{hearth_task_routes, HEARTH_AREAS};

    #[test]
    fn hearth_binds_all_five_task_surfaces_explicitly() {
        let routes = hearth_task_routes();
        let labels = routes
            .desktop_nav()
            .into_iter()
            .map(|link| link.label)
            .collect::<Vec<_>>();

        assert_eq!(labels, vec!["Home", "Find", "Create", "Inbox", "Me"]);
        assert_eq!(routes.me, "/personal/profile");
    }

    #[test]
    fn domain_discoverability_is_not_conditioned_on_trust_tier() {
        let areas = HEARTH_AREAS.into_iter().collect::<Vec<_>>();
        assert!(areas.contains(&("/decisions", "Decisions")));
        assert!(areas.contains(&("/emergency", "Emergency")));
    }
}
