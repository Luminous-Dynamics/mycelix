// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

use leptos::prelude::*;
use leptos_router::{
    components::{Route, Router, Routes},
    path,
};

use crate::components::Nav;
use crate::contexts::finance_context::provide_finance_context;
use crate::contexts::governance_context::provide_governance_context;
use crate::pages::*;

use mycelix_leptos_core::{
    ConnectStrategy, HolochainProviderConfig, HomeostasisConfig, MycelixApplication,
    MycelixApplicationConfig,
};

#[component]
pub fn App() -> impl IntoView {
    let holochain = HolochainProviderConfig {
        app_id: "mycelix-unified".into(),
        default_role: None, // Multi-role: governance + finance
        log_prefix: "[Civic]",
        connect_strategy: ConnectStrategy::WebSocket,
        status_labels: None,
    };
    let config = MycelixApplicationConfig::new(holochain)
        .with_homeostasis(HomeostasisConfig::new(2, "--homeostasis"));

    view! {
        <MycelixApplication config=config>
            <GovernanceApplication />
        </MycelixApplication>
    }
}

/// Governance-owned application composition.
///
/// Cross-domain runtime providers live in `MycelixApplication`. Everything
/// below this boundary remains domain-local: civic theme, governance/finance
/// contexts, civic actions, navigation, routes, and authority semantics.
#[component]
fn GovernanceApplication() -> impl IntoView {
    crate::themes::provide_theme_context();
    provide_governance_context();
    provide_finance_context();

    // Action dispatch remains domain-local: the shared application boundary
    // must not become an authority or business-action layer.
    crate::contexts::civic_actions::provide_civic_actions();

    view! {
        <Router>
            <a href="#main-content" class="skip-to-content">"skip to content"</a>
            <Nav />
            <main id="main-content" class="main-content civic-pulse" role="main" aria-label="civic content">
                <Routes fallback=|| view! { <p class="not-found">"you have wandered beyond the commons"</p> }>
                    <Route path=path!("/") view=HomePage />
                    <Route path=path!("/proposals") view=ProposalListPage />
                    <Route path=path!("/proposals/new") view=CreateProposalPage />
                    <Route path=path!("/proposals/:id") view=ProposalDetailPage />
                    <Route path=path!("/voting") view=VotingPage />
                    <Route path=path!("/councils") view=CouncilsPage />
                    <Route path=path!("/tend") view=TendPage />
                    <Route path=path!("/payments") view=PaymentsPage />
                    <Route path=path!("/recognition") view=RecognitionPage />
                    <Route path=path!("/treasury") view=TreasuryPage />
                    <Route path=path!("/budgeting") view=BudgetingPage />
                    <Route path=path!("/constitution") view=ConstitutionPage />
                    <Route path=path!("/execution") view=ExecutionPage />
                    <Route path=path!("/staking") view=StakingPage />
                    <Route path=path!("/oracle") view=OraclePage />
                    <Route path=path!("/profile") view=ProfilePage />
                </Routes>
            </main>
        </Router>
    }
}
