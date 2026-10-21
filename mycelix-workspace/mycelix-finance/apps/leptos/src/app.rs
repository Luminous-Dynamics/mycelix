// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

use leptos::prelude::*;
use leptos_router::components::{Route, Router, Routes};
use leptos_router::path;

use mycelix_leptos_core::{
    init_consciousness_ui, provide_consciousness_context, provide_homeostasis_context,
    try_provide_local_identity, provide_theme_context, provide_thermodynamic_context,
    provide_toast_context, AppShell, EmptyState, HolochainProviderAuto, HolochainProviderConfig,
    NavLink, ToastContainer,
};
use mycelix_leptos_i18n::provide_i18n;

use crate::context::provide_finance_context;
use crate::finance_config::provide_finance_runtime_config;
use crate::pages;

// ---------------------------------------------------------------------------
// Theme
// ---------------------------------------------------------------------------

#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum AppThemeVariant {
    Dark,
    Light,
}

impl mycelix_leptos_core::AppTheme for AppThemeVariant {
    fn label(&self) -> &'static str {
        match self {
            AppThemeVariant::Dark => "dark",
            AppThemeVariant::Light => "light",
        }
    }
    fn all() -> &'static [Self] {
        &[AppThemeVariant::Dark, AppThemeVariant::Light]
    }
    fn next(&self) -> Self {
        match self {
            AppThemeVariant::Dark => AppThemeVariant::Light,
            AppThemeVariant::Light => AppThemeVariant::Dark,
        }
    }
    fn is_light(&self) -> bool {
        matches!(self, AppThemeVariant::Light)
    }
}

// ---------------------------------------------------------------------------
// App
// ---------------------------------------------------------------------------

#[component]
pub fn App() -> impl IntoView {
    let config = HolochainProviderConfig {
        app_id: "mycelix-unified".into(),
        default_role: Some("finance".into()),
        log_prefix: "[Finance]",
        connect_strategy: mycelix_leptos_core::ConnectStrategy::WebSocket,
        status_labels: None,
    };

    view! {
        <HolochainProviderAuto config=config>
            <AppInner />
        </HolochainProviderAuto>
    }
}

#[component]
fn AppInner() -> impl IntoView {
    // Provider stack (order matters)
    provide_theme_context("finance-theme", AppThemeVariant::Dark);
    provide_thermodynamic_context();
    provide_consciousness_context();
    provide_toast_context();
    provide_homeostasis_context(2, "--homeostasis");
    if let Err(error) = try_provide_local_identity() {
        return view! {
            <main class="stack-page identity-access-blocked" role="status" aria-live="polite">
                <section class="vault-card">
                    <h1>"Finance requires a ready identity"</h1>
                    <p class="supporting-copy">{error.to_string()}</p>
                    <p class="supporting-copy">
                        "Open Mycelix Personal to create, protect, unlock or recover the identity and confirm its offline backup. No identity was created or replaced by Finance."
                    </p>
                </section>
            </main>
        }
        .into_any();
    }
    provide_finance_runtime_config();
    provide_i18n("finance-lang");
    provide_finance_context();
    init_consciousness_ui();

    let nav_links = vec![
        NavLink {
            href: "/",
            label: "Home",
            icon: None,
        },
        NavLink {
            href: "/tend",
            label: "TEND",
            icon: Some("🤝"),
        },
        NavLink {
            href: "/sap",
            label: "SAP",
            icon: Some("💧"),
        },
        NavLink {
            href: "/mycel",
            label: "MYCEL",
            icon: Some("🍄"),
        },
        NavLink {
            href: "/treasury",
            label: "Treasury",
            icon: Some("🏛"),
        },
        NavLink {
            href: "/staking",
            label: "Staking",
            icon: Some("🔒"),
        },
        NavLink {
            href: "/oracle",
            label: "Oracle",
            icon: Some("🔮"),
        },
        NavLink {
            href: "/profile",
            label: "Profile",
            icon: Some("👤"),
        },
    ];

    view! {
        <Router>
            <AppShell
                brand_name="Finance"
                brand_icon="💰"
                nav_links=nav_links
            >
                <Routes fallback=|| view! {
                    <EmptyState icon="?" title="Page not found" />
                }>
                    <Route path=path!("/") view=pages::HomePage />
                    <Route path=path!("/tend") view=pages::TendPage />
                    <Route path=path!("/sap") view=pages::SapPage />
                    <Route path=path!("/mycel") view=pages::MycelPage />
                    <Route path=path!("/recognition") view=pages::RecognitionPage />
                    <Route path=path!("/treasury") view=pages::TreasuryPage />
                    <Route path=path!("/staking") view=pages::StakingPage />
                    <Route path=path!("/oracle") view=pages::OraclePage />
                    <Route path=path!("/profile") view=pages::ProfilePage />
                </Routes>
            </AppShell>
            <ToastContainer />
            <mycelix_leptos_core::ClusterLauncher current="Finance" />
        </Router>
    }
    .into_any()
}
