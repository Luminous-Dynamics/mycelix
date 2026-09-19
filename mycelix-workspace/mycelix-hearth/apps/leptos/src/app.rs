// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

use leptos::prelude::*;
use leptos_router::{
    components::{Route, Router, Routes},
    path,
};

use crate::care_attention::provide_care_attention;
use crate::components::{DevPanel, Nav};
use crate::decision_outcomes::{DecisionOutcomeSummary, provide_decision_outcomes};
use crate::hearth_boundary::{HearthDataBoundary, HearthDataDomain};
use crate::hearth_context::provide_hearth_context;
use crate::hearth_truth::{
    HearthDataStatus, provide_hearth_truth, start_mock_simulation_when_resolved,
};
use crate::pages::personal;
use crate::pages::*;
use crate::pending_votes::provide_unvoted_decisions;
use crate::vote_history::{VoteHistorySummary, provide_vote_history};
use mycelix_leptos_core::{
    ConnectStrategy, HolochainProviderAuto, HolochainProviderConfig, ToastContainer,
    init_consciousness_ui, provide_consciousness_context, provide_homeostasis_context,
    provide_thermodynamic_context, provide_toast_context,
};

#[component]
pub fn App() -> impl IntoView {
    let config = HolochainProviderConfig {
        app_id: "mycelix-unified".into(),
        default_role: Some("hearth".into()),
        log_prefix: "[Hearth]",
        connect_strategy: ConnectStrategy::WebSocket,
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
    crate::themes::provide_theme_context();
    crate::circadian::provide_circadian_context();
    crate::ambient_sound::provide_ambient_sound_context();
    provide_thermodynamic_context();
    provide_consciousness_context();
    provide_toast_context();
    provide_homeostasis_context(2, "--homeostasis");
    provide_hearth_context();
    provide_hearth_truth();
    provide_decision_outcomes();
    provide_vote_history();
    provide_unvoted_decisions();
    provide_care_attention();
    crate::hearth_prefs::provide_hearth_prefs();

    crate::hearth_actions::provide_hearth_actions();
    init_consciousness_ui();
    crate::signal_listener::start_signal_listener();
    start_mock_simulation_when_resolved();

    view! {
        <Router>
            <a href="#main-content" class="skip-to-content">"skip to content"</a>
            <Nav />
            <HearthDataStatus />
            <DevPanel />
            <crate::components::HearthFlame mode=crate::components::FlameMode::Ambient />
            <main id="main-content" class="main-content heartbeat" role="main" aria-label="hearth content">
                <Routes fallback=|| view! { <p class="not-found">"you’ve wandered off the path"</p> }>
                    // Task-first primary shell
                    <Route path=path!("/") view=HomeRoute />
                    <Route path=path!("/find") view=FindPage />
                    <Route path=path!("/create") view=CreatePage />
                    <Route path=path!("/inbox") view=InboxPage />

                    // Existing Hearth/domain routes remain stable and directly linkable.
                    <Route path=path!("/found") view=crate::pages::found::FoundingCeremony />
                    <Route path=path!("/settings") view=crate::pages::settings::SettingsPage />
                    <Route path=path!("/kinship") view=KinshipRoute />
                    <Route path=path!("/care") view=CareRoute />
                    <Route path=path!("/decisions") view=DecisionsRoute />
                    <Route path=path!("/gratitude") view=GratitudeRoute />
                    <Route path=path!("/stories") view=StoriesRoute />
                    <Route path=path!("/milestones") view=MilestonesRoute />
                    <Route path=path!("/rhythms") view=RhythmsRoute />
                    <Route path=path!("/emergency") view=EmergencyRoute />
                    <Route path=path!("/resources") view=ResourcesRoute />
                    <Route path=path!("/autonomy") view=AutonomyRoute />
                    <Route path=path!("/personal/profile") view=personal::ProfilePage />
                    <Route path=path!("/personal/health") view=personal::HealthPage />
                    <Route path=path!("/personal/credentials") view=personal::CredentialsPage />
                    <Route path=path!("/personal/disclosure") view=personal::DisclosurePage />
                </Routes>
            </main>
            <ToastContainer />
            <crate::onboarding::OnboardingOverlay />
        </Router>
    }
}

#[component]
fn HomeRoute() -> impl IntoView {
    view! {
        <HearthDataBoundary domain=HearthDataDomain::Home title="Home">
            <HomePage />
        </HearthDataBoundary>
    }
}

#[component]
fn KinshipRoute() -> impl IntoView {
    view! {
        <HearthDataBoundary domain=HearthDataDomain::Kinship title="Bonds">
            <KinshipPage />
        </HearthDataBoundary>
    }
}

#[component]
fn CareRoute() -> impl IntoView {
    view! {
        <HearthDataBoundary domain=HearthDataDomain::Care title="Care">
            <CarePage />
        </HearthDataBoundary>
    }
}

#[component]
fn DecisionsRoute() -> impl IntoView {
    view! {
        <HearthDataBoundary domain=HearthDataDomain::Decisions title="Decisions">
            <DecisionsPage />
            <DecisionOutcomeSummary />
            <VoteHistorySummary />
        </HearthDataBoundary>
    }
}

#[component]
fn GratitudeRoute() -> impl IntoView {
    view! {
        <HearthDataBoundary domain=HearthDataDomain::Gratitude title="Gratitude">
            <GratitudePage />
        </HearthDataBoundary>
    }
}

#[component]
fn StoriesRoute() -> impl IntoView {
    view! {
        <HearthDataBoundary domain=HearthDataDomain::Stories title="Stories">
            <StoriesPage />
        </HearthDataBoundary>
    }
}

#[component]
fn MilestonesRoute() -> impl IntoView {
    view! {
        <HearthDataBoundary domain=HearthDataDomain::Milestones title="Milestones">
            <MilestonesPage />
        </HearthDataBoundary>
    }
}

#[component]
fn RhythmsRoute() -> impl IntoView {
    view! {
        <HearthDataBoundary domain=HearthDataDomain::Rhythms title="Rhythms">
            <RhythmsPage />
        </HearthDataBoundary>
    }
}

#[component]
fn EmergencyRoute() -> impl IntoView {
    view! {
        <HearthDataBoundary domain=HearthDataDomain::Emergency title="Emergency">
            <EmergencyPage />
        </HearthDataBoundary>
    }
}

#[component]
fn ResourcesRoute() -> impl IntoView {
    view! {
        <HearthDataBoundary domain=HearthDataDomain::Resources title="Resources">
            <ResourcesPage />
        </HearthDataBoundary>
    }
}

#[component]
fn AutonomyRoute() -> impl IntoView {
    view! {
        <HearthDataBoundary domain=HearthDataDomain::Autonomy title="Autonomy">
            <AutonomyPage />
        </HearthDataBoundary>
    }
}
