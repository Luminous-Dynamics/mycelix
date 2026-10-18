// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Shared, dependency-light Leptos 0.8 components for public Mycelix frontends.
//!
//! Components favor native semantics, visible keyboard focus, text labels for
//! status, and an honest unavailable state instead of synthetic profile data.
//! Domain state and Holochain authority remain in the consuming application.

use leptos::prelude::*;

pub mod badge {
    use leptos::prelude::*;

    #[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
    pub enum BadgeVariant {
        #[default]
        Neutral,
        Primary,
        Success,
        Warning,
        Danger,
        Info,
        Outline,
    }

    impl BadgeVariant {
        fn class(self) -> &'static str {
            match self {
                Self::Neutral => "mycelix-badge-neutral",
                Self::Primary => "mycelix-badge-primary",
                Self::Success => "mycelix-badge-success",
                Self::Warning => "mycelix-badge-warning",
                Self::Danger => "mycelix-badge-danger",
                Self::Info => "mycelix-badge-info",
                Self::Outline => "mycelix-badge-outline",
            }
        }
    }

    /// A compact, text-bearing status or category label.
    #[component]
    pub fn Badge(
        #[prop(into)] label: String,
        #[prop(optional)] variant: BadgeVariant,
    ) -> impl IntoView {
        view! {
            <span class=move || format!("mycelix-badge {}", variant.class())>
                {label}
            </span>
        }
    }

    /// Status indicator that does not rely on color alone.
    #[component]
    pub fn StatusDot(
        #[prop(into)] status: String,
        #[prop(optional, into)] label: Option<String>,
    ) -> impl IntoView {
        let normalized = status.to_ascii_lowercase();
        let class = match normalized.as_str() {
            "online" | "connected" | "ready" | "success" => "mycelix-status-dot-online",
            "warning" | "degraded" | "pending" => "mycelix-status-dot-warning",
            "offline" | "error" | "failed" => "mycelix-status-dot-error",
            _ => "mycelix-status-dot-unknown",
        };
        view! {
            <span class="mycelix-status" data-status=normalized>
                <span class=move || format!("mycelix-status-dot {class}") aria-hidden="true"></span>
                <span>{label.unwrap_or(status)}</span>
            </span>
        }
    }
}

pub mod data_table {
    use leptos::prelude::*;

    #[derive(Clone, Debug, PartialEq, Eq)]
    pub struct Column {
        pub key: String,
        pub label: String,
    }

    impl Column {
        pub fn new(key: impl Into<String>, label: impl Into<String>) -> Self {
            Self { key: key.into(), label: label.into() }
        }
    }

    /// Controlled pagination state. Pages are one-based; an empty collection
    /// has zero pages and is not reported as page one of zero.
    #[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
    pub struct Pagination {
        pub page: u32,
        pub page_size: u32,
        pub total_items: u64,
    }

    impl Pagination {
        pub fn page_count(self) -> u32 {
            if self.page_size == 0 || self.total_items == 0 {
                return 0;
            }
            let size = u64::from(self.page_size);
            let pages = self.total_items / size + u64::from(self.total_items % size != 0);
            u32::try_from(pages).unwrap_or(u32::MAX)
        }

        pub fn normalized_page(self) -> u32 {
            self.page.clamp(1, self.page_count().max(1))
        }
    }

    /// Accessible read-only table. Cell data is already rendered/display-safe
    /// text; domain code must not pass untrusted HTML here.
    #[component]
    pub fn DataTable(
        columns: Vec<Column>,
        rows: Vec<Vec<String>>,
        #[prop(optional)] caption: Option<String>,
        #[prop(optional)] pagination: Option<Pagination>,
        #[prop(optional)] on_page_change: Option<Callback<u32>>,
    ) -> impl IntoView {
        let column_count = columns.len();
        let header_columns = columns.clone();
        let current_pagination = pagination;
        let can_go_previous = current_pagination
            .map(|value| value.page > 1 && value.page_count() > 0)
            .unwrap_or(false);
        let can_go_next = current_pagination
            .map(|value| value.page < value.page_count())
            .unwrap_or(false);

        view! {
            <div class="mycelix-table-region">
                <table class="mycelix-table">
                    {caption.map(|text| view! { <caption>{text}</caption> })}
                    <thead>
                        <tr>
                            {header_columns.into_iter().map(|column| view! {
                                <th scope="col" data-column=column.key>{column.label}</th>
                            }).collect_view()}
                        </tr>
                    </thead>
                    <tbody>
                        {rows.into_iter().map(|row| {
                            let values = (0..column_count).map(|index| {
                                row.get(index).cloned().unwrap_or_else(|| "—".to_owned())
                            }).collect::<Vec<_>>();
                            view! {
                                <tr>
                                    {values.into_iter().map(|value| view! { <td>{value}</td> }).collect_view()}
                                </tr>
                            }
                        }).collect_view()}
                    </tbody>
                </table>
                {current_pagination.map(|page| {
                    let previous = on_page_change;
                    let next = on_page_change;
                    let page_count = page.page_count();
                    let current_page = page.normalized_page();
                    view! {
                        <nav class="mycelix-pagination" aria-label="Table pagination">
                            <span class="mycelix-pagination-summary">
                                {format!("Page {current_page} of {page_count} · {} items", page.total_items)}
                            </span>
                            <button
                                type="button"
                                class="btn mycelix-pagination-button"
                                disabled=!(can_go_previous && previous.is_some())
                                on:click=move |_| {
                                    if let Some(callback) = previous {
                                        callback.run(current_page.saturating_sub(1).max(1));
                                    }
                                }
                            >"Previous"</button>
                            <button
                                type="button"
                                class="btn mycelix-pagination-button"
                                disabled=!(can_go_next && next.is_some())
                                on:click=move |_| {
                                    if let Some(callback) = next {
                                        callback.run(current_page.saturating_add(1).min(page_count));
                                    }
                                }
                            >"Next"</button>
                        </nav>
                    }
                })}
            </div>
        }
    }
}

pub mod empty_state {
    use leptos::prelude::*;

    /// A calm empty state with optional explanation and an explicit next action.
    #[component]
    pub fn EmptyState(
        #[prop(into)] title: String,
        #[prop(optional, into)] icon: String,
        #[prop(optional, into)] description: String,
        #[prop(optional)] action: Option<AnyView>,
    ) -> impl IntoView {
        let icon = if icon.trim().is_empty() { "∅".to_owned() } else { icon };
        view! {
            <section class="mycelix-empty-state" aria-live="polite">
                <div class="mycelix-empty-state-icon" aria-hidden="true">{icon}</div>
                <h2>{title}</h2>
                {(!description.trim().is_empty()).then(|| view! {
                    <p class="mycelix-empty-state-description">{description}</p>
                })}
                {action}
            </section>
        }
    }
}

pub mod loading {
    use leptos::prelude::*;

    /// A small animated loading indicator with a non-color status label.
    #[component]
    pub fn Spinner(
        #[prop(optional, into)] label: String,
    ) -> impl IntoView {
        let label = if label.trim().is_empty() { "Loading".to_owned() } else { label };
        view! {
            <span class="mycelix-spinner-wrap" role="status">
                <span class="mycelix-spinner" aria-hidden="true"></span>
                <span class="mycelix-sr-only">{label}</span>
            </span>
        }
    }

    /// Skeleton placeholder. The requested row count is bounded to avoid a
    /// malformed caller creating an unbounded DOM.
    #[component]
    pub fn LoadingSkeleton(
        #[prop(optional)] lines: Option<usize>,
        #[prop(optional, into)] label: String,
    ) -> impl IntoView {
        let count = lines.unwrap_or(3).clamp(1, 12);
        let label = if label.trim().is_empty() { "Loading content".to_owned() } else { label };
        view! {
            <div class="mycelix-skeleton-group" role="status" aria-label=label>
                {(0..count).map(|index| {
                    let width = if index + 1 == count { "62%" } else { "100%" };
                    view! {
                        <span
                            class="mycelix-skeleton-line"
                            style:width=width
                            aria-hidden="true"
                        ></span>
                    }
                }).collect_view()}
                <span class="mycelix-sr-only">{label}</span>
            </div>
        }
    }
}

pub mod progress_bar {
    use leptos::prelude::*;

    pub fn clamp_progress(value: f64, max: f64) -> f64 {
        if !value.is_finite() || !max.is_finite() || max <= 0.0 {
            return 0.0;
        }
        (value / max * 100.0).clamp(0.0, 100.0)
    }

    /// Accessible progress indicator. Invalid numeric values fail closed to 0%.
    #[component]
    pub fn ProgressBar(
        value: f64,
        #[prop(optional)] max: Option<f64>,
        #[prop(optional, into)] label: String,
    ) -> impl IntoView {
        let maximum = max.unwrap_or(100.0);
        let percent = clamp_progress(value, maximum);
        let text = if label.trim().is_empty() { "Progress".to_owned() } else { label };
        view! {
            <div class="mycelix-progress-group">
                <div class="mycelix-progress-label">
                    <span>{text.clone()}</span>
                    <span>{format!("{percent:.0}%")}</span>
                </div>
                <div
                    class="mycelix-progress-track"
                    role="progressbar"
                    aria-label=text
                    aria-valuemin="0"
                    aria-valuemax="100"
                    aria-valuenow=percent
                >
                    <span class="mycelix-progress-fill" style:width=format!("{percent}%")></span>
                </div>
            </div>
        }
    }
}

pub mod sovereign_radar {
    use leptos::prelude::*;

    #[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
    pub enum SovereignRadarSize {
        Small,
        #[default]
        Medium,
        Large,
    }

    impl SovereignRadarSize {
        fn pixels(self) -> u16 {
            match self {
                Self::Small => 176,
                Self::Medium => 240,
                Self::Large => 320,
            }
        }
    }

    fn radar_points(scores: &[f64]) -> Option<String> {
        if scores.len() != 8 || scores.iter().any(|value| !value.is_finite()) {
            return None;
        }
        let centre = 100.0_f64;
        let radius = 72.0_f64;
        Some(scores.iter().enumerate().map(|(index, score)| {
            let angle = -std::f64::consts::FRAC_PI_2
                + index as f64 * std::f64::consts::TAU / 8.0;
            let magnitude = score.clamp(0.0, 1.0) * radius;
            let x = centre + angle.cos() * magnitude;
            let y = centre + angle.sin() * magnitude;
            format!("{x:.2},{y:.2}")
        }).collect::<Vec<_>>().join(" "))
    }

    /// Eight-axis profile radar. No scores are fabricated: without exactly
    /// eight finite source values the component renders the grid and an
    /// explicit unavailable label rather than a synthetic profile polygon.
    #[component]
    pub fn SovereignRadar(
        #[prop(optional)] size: SovereignRadarSize,
        #[prop(optional)] scores: Option<Vec<f64>>,
        #[prop(optional, into)] label: String,
    ) -> impl IntoView {
        let size_px = size.pixels();
        let accessible_label = if label.trim().is_empty() {
            "Eight-dimension sovereign profile".to_owned()
        } else {
            label
        };
        let polygon = scores.as_deref().and_then(radar_points);
        let has_profile = polygon.is_some();
        let axes = [
            ("Identity", 100.0, 15.0),
            ("Care", 154.0, 37.0),
            ("Trust", 174.0, 100.0),
            ("Contribution", 154.0, 161.0),
            ("Learning", 100.0, 184.0),
            ("Stewardship", 46.0, 161.0),
            ("Autonomy", 26.0, 100.0),
            ("Consensus", 46.0, 37.0),
        ];
        view! {
            <figure class="mycelix-radar">
                <svg
                    width=size_px
                    height=size_px
                    viewBox="0 0 200 200"
                    role="img"
                    aria-label=accessible_label
                >
                    <polygon points="100,28 151,49 172,100 151,151 100,172 49,151 28,100 49,49"
                        fill="none" stroke="var(--border-visible, #52665b)" stroke-width="1"/>
                    <polygon points="100,46 138,62 154,100 138,138 100,154 62,138 46,100 62,62"
                        fill="none" stroke="var(--border-visible, #52665b)" stroke-width="1"/>
                    <polygon points="100,64 125,75 136,100 125,125 100,136 75,125 64,100 75,75"
                        fill="none" stroke="var(--border-visible, #52665b)" stroke-width="1"/>
                    {(0..8).map(|index| {
                        let angle = -std::f64::consts::FRAC_PI_2
                            + index as f64 * std::f64::consts::TAU / 8.0;
                        let x = 100.0 + angle.cos() * 72.0;
                        let y = 100.0 + angle.sin() * 72.0;
                        view! {
                            <line x1="100" y1="100" x2=x y2=y stroke="var(--border-visible, #52665b)" stroke-width="1"/>
                        }
                    }).collect_view()}
                    {polygon.map(|points| view! {
                        <polygon
                            points=points
                            fill="var(--mycelix-cyan, #4ecdc4)"
                            fill-opacity="0.24"
                            stroke="var(--mycelix-cyan, #4ecdc4)"
                            stroke-width="2"
                        />
                    })}
                    {axes.into_iter().map(|(text, x, y)| view! {
                        <text x=x y=y fill="var(--text-secondary, #9aab94)" font-size="7" text-anchor="middle">{text}</text>
                    }).collect_view()}
                </svg>
                {(!has_profile).then(|| view! {
                    <figcaption class="mycelix-radar-unavailable">
                        "Profile values unavailable — no scores supplied"
                    </figcaption>
                })}
            </figure>
        }
    }

    #[cfg(test)]
    mod tests {
        use super::*;

        #[test]
        fn accepts_only_eight_finite_scores() {
            assert!(radar_points(&[0.0; 8]).is_some());
            assert!(radar_points(&[0.0; 7]).is_none());
            assert!(radar_points(&[f64::NAN; 8]).is_none());
        }
    }
}

pub mod stat_card {
    use leptos::prelude::*;

    #[component]
    pub fn StatCard(
        #[prop(into)] label: String,
        #[prop(into)] value: String,
        #[prop(optional, into)] subtitle: String,
    ) -> impl IntoView {
        view! {
            <article class="mycelix-stat-card">
                <p class="mycelix-stat-label">{label}</p>
                <p class="mycelix-stat-value">{value}</p>
                {(!subtitle.trim().is_empty()).then(|| view! {
                    <p class="mycelix-stat-subtitle">{subtitle}</p>
                })}
            </article>
        }
    }
}

pub mod tabs {
    use leptos::prelude::*;

    #[derive(Clone, Debug, PartialEq, Eq)]
    pub struct TabDefinition {
        pub id: String,
        pub label: String,
    }

    impl TabDefinition {
        pub fn new(id: impl Into<String>, label: impl Into<String>) -> Self {
            Self { id: id.into(), label: label.into() }
        }
    }

    /// Controlled ARIA tab list. The parent owns the selected tab state.
    #[component]
    pub fn Tabs(
        tabs: Vec<TabDefinition>,
        active: RwSignal<String>,
        #[prop(optional, into)] label: String,
    ) -> impl IntoView {
        let aria_label = if label.trim().is_empty() { "Section tabs".to_owned() } else { label };
        view! {
            <div class="mycelix-tabs">
                <div class="mycelix-tab-list" role="tablist" aria-label=aria_label>
                    {tabs.into_iter().map(|tab| {
                        let id = tab.id.clone();
                        let id_for_click = tab.id.clone();
                        let selected = move || active.get() == id;
                        view! {
                            <button
                                type="button"
                                role="tab"
                                id=format!("tab-{id_for_click}")
                                aria-selected=selected
                                class="mycelix-tab"
                                on:click=move |_| active.set(tab.id.clone())
                            >{tab.label}</button>
                        }
                    }).collect_view()}
                </div>
            </div>
        }
    }

    /// A panel belongs to a selected-tab signal supplied by the parent.
    #[component]
    pub fn TabPanel(
        id: String,
        active: RwSignal<String>,
        children: Children,
    ) -> impl IntoView {
        let content = children();
        view! {
            <section
                class="mycelix-tab-panel"
                role="tabpanel"
                id=format!("tabpanel-{id}")
                aria-labelledby=format!("tab-{id}")
                hidden=move || active.get() != id
            >
                {content}
            </section>
        }
    }
}

pub mod toasts {
    use leptos::prelude::*;

    #[derive(Clone, Debug, PartialEq, Eq)]
    pub enum ToastKind {
        Success,
        Error,
        Info,
        Warning,
        Custom(String),
    }

    impl ToastKind {
        fn class(&self) -> &str {
            match self {
                Self::Success => "success",
                Self::Error => "error",
                Self::Info => "info",
                Self::Warning => "warning",
                Self::Custom(name) => name.as_str(),
            }
        }

        fn role(&self) -> &'static str {
            match self {
                Self::Error => "alert",
                _ => "status",
            }
        }
    }

    #[derive(Clone, Debug, PartialEq, Eq)]
    pub struct Toast {
        pub id: u64,
        pub message: String,
        pub kind: ToastKind,
    }

    #[derive(Clone, Debug)]
    pub struct ToastState {
        toasts: RwSignal<Vec<Toast>>,
        next_id: RwSignal<u64>,
    }

    impl Default for ToastState {
        fn default() -> Self {
            Self {
                toasts: RwSignal::new(Vec::new()),
                next_id: RwSignal::new(1),
            }
        }
    }

    impl ToastState {
        pub fn push(&self, message: impl Into<String>, kind: ToastKind) {
            let id = self.next_id.get_untracked();
            self.next_id.set(id.saturating_add(1));
            self.toasts.update(|items| {
                items.push(Toast { id, message: message.into(), kind });
                if items.len() > 8 {
                    let overflow = items.len() - 8;
                    items.drain(0..overflow);
                }
            });
        }

        pub fn success(&self, message: impl Into<String>) {
            self.push(message, ToastKind::Success);
        }

        pub fn error(&self, message: impl Into<String>) {
            self.push(message, ToastKind::Error);
        }

        pub fn info(&self, message: impl Into<String>) {
            self.push(message, ToastKind::Info);
        }

        pub fn warning(&self, message: impl Into<String>) {
            self.push(message, ToastKind::Warning);
        }

        pub fn dismiss(&self, id: u64) {
            self.toasts.update(|items| items.retain(|toast| toast.id != id));
        }

        pub fn clear(&self) {
            self.toasts.set(Vec::new());
        }

        pub fn snapshot(&self) -> Vec<Toast> {
            self.toasts.get_untracked()
        }
    }

    pub fn provide_toast_context() {
        provide_context(ToastState::default());
    }

    pub fn use_toasts() -> ToastState {
        expect_context::<ToastState>()
    }

    /// Persistent until dismissed, bounded to eight entries, and announced
    /// with a polite status or assertive alert depending on severity.
    #[component]
    pub fn ToastContainer() -> impl IntoView {
        let state = use_toasts();
        view! {
            <div class="mycelix-toast-region" aria-label="Notifications">
                <For
                    each=move || state.toasts.get()
                    key=|toast| toast.id
                    children=move |toast| {
                        let id = toast.id;
                        let dismiss_state = state.clone();
                        let class = format!("mycelix-toast mycelix-toast-{}", toast.kind.class());
                        view! {
                            <div class=class role=toast.kind.role()>
                                <span class="mycelix-toast-message">{toast.message}</span>
                                <button
                                    type="button"
                                    class="mycelix-toast-dismiss"
                                    aria-label="Dismiss notification"
                                    on:click=move |_| dismiss_state.dismiss(id)
                                >"Dismiss"</button>
                            </div>
                        }
                    }
                />
            </div>
        }
    }

    #[cfg(test)]
    mod tests {
        use super::*;

        #[test]
        fn toast_kinds_keep_error_announcement_distinct() {
            assert_eq!(ToastKind::Error.role(), "alert");
            assert_eq!(ToastKind::Success.role(), "status");
            assert_eq!(ToastKind::Custom("governance".into()).class(), "governance");
        }
    }
}

pub mod app_shell {
    use leptos::prelude::*;
    use leptos_router::components::A;

    #[derive(Clone, Debug, PartialEq, Eq)]
    pub struct NavLink {
        pub href: &'static str,
        pub label: &'static str,
        pub icon: Option<&'static str>,
    }

    #[derive(Clone, Debug, PartialEq, Eq)]
    pub struct NavTab {
        pub href: &'static str,
        pub icon: &'static str,
        pub label: &'static str,
    }

    #[component]
    pub fn AppNav(nav_links: Vec<NavLink>) -> impl IntoView {
        view! {
            <nav class="mycelix-app-nav" aria-label="Primary navigation">
                {nav_links.into_iter().map(|link| view! {
                    <A href=link.href attr:class="mycelix-app-nav-link">
                        {link.icon.map(|icon| view! {
                            <span class="mycelix-app-nav-icon" aria-hidden="true">{icon}</span>
                        })}
                        <span>{link.label}</span>
                    </A>
                }).collect_view()}
            </nav>
        }
    }

    #[component]
    pub fn MobileBottomNav(tabs: Vec<NavTab>) -> impl IntoView {
        view! {
            <nav class="mycelix-mobile-nav" aria-label="Mobile navigation">
                {tabs.into_iter().map(|tab| view! {
                    <A href=tab.href attr:class="mycelix-mobile-nav-link">
                        <span class="mycelix-mobile-nav-icon" aria-hidden="true">{tab.icon}</span>
                        <span>{tab.label}</span>
                    </A>
                }).collect_view()}
            </nav>
        }
    }

    #[component]
    pub fn AppShell(
        #[prop(into)] brand_name: String,
        #[prop(into)] brand_icon: String,
        nav_links: Vec<NavLink>,
        #[prop(optional)] mobile_tabs: Option<Vec<NavTab>>,
        #[prop(optional)] meta_view: Option<AnyView>,
        children: Children,
    ) -> impl IntoView {
        let desktop_links = nav_links;
        view! {
            <div class="mycelix-app-shell">
                <a class="mycelix-skip-link" href="#mycelix-main-content">"Skip to content"</a>
                <header class="mycelix-app-header">
                    <div class="mycelix-app-brand">
                        <span class="mycelix-app-brand-icon" aria-hidden="true">{brand_icon}</span>
                        <span class="mycelix-app-brand-name">{brand_name}</span>
                    </div>
                    <AppNav nav_links=desktop_links />
                    <div class="mycelix-app-meta">{meta_view}</div>
                </header>
                <main id="mycelix-main-content" class="mycelix-app-main">
                    {children()}
                </main>
                {mobile_tabs.map(|tabs| view! { <MobileBottomNav tabs=tabs /> })}
            </div>
        }
    }
}

pub use app_shell::{AppNav, AppShell, MobileBottomNav, NavLink, NavTab};
pub use badge::{Badge, BadgeVariant, StatusDot};
pub use data_table::{Column, DataTable, Pagination};
pub use empty_state::EmptyState;
pub use loading::{LoadingSkeleton, Spinner};
pub use progress_bar::ProgressBar;
pub use sovereign_radar::{SovereignRadar, SovereignRadarSize};
pub use stat_card::StatCard;
pub use tabs::{TabPanel, Tabs};
pub use toasts::{Toast, ToastContainer, ToastKind, ToastState, provide_toast_context, use_toasts};

#[cfg(test)]
mod tests {
    use super::data_table::Pagination;
    use super::progress_bar::clamp_progress;

    #[test]
    fn pagination_page_count_is_integer_and_bounded() {
        assert_eq!(Pagination { page: 1, page_size: 10, total_items: 0 }.page_count(), 0);
        assert_eq!(Pagination { page: 1, page_size: 10, total_items: 11 }.page_count(), 2);
        assert_eq!(Pagination { page: 40, page_size: 10, total_items: 11 }.normalized_page(), 2);
        assert_eq!(Pagination { page: 1, page_size: 0, total_items: 11 }.page_count(), 0);
    }

    #[test]
    fn progress_rejects_non_finite_values_and_clamps_range() {
        assert_eq!(clamp_progress(f64::NAN, 100.0), 0.0);
        assert_eq!(clamp_progress(300.0, 100.0), 100.0);
        assert_eq!(clamp_progress(-10.0, 100.0), 0.0);
        assert_eq!(clamp_progress(50.0, f64::INFINITY), 0.0);
    }
}
