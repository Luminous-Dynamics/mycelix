// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

use leptos::prelude::*;
use leptos_router::{
    components::{Route, Router, Routes},
    path,
};

use crate::components::{DevPanel, Nav};
use crate::hearth_context::{HearthDataState, provide_hearth_context, use_hearth};
use crate::pages::personal;
use crate::pages::*;
use crate::runtime_mode::{
    HearthRuntimeMode, detect_runtime_mode, provide_runtime_mode, use_runtime_mode,
};
use mycelix_leptos_core::{
    AvailabilityState, AvailabilityStateKind, ConnectionStatus, HolochainProviderAuto,
    HolochainProviderConfig, ToastContainer, init_consciousness_ui,
    provide_consciousness_context, provide_homeostasis_context, provide_thermodynamic_context,
    provide_toast_context,
};

#[component]
pub fn App() -> impl IntoView {
    let runtime_mode = detect_runtime_mode();
    let config = HolochainProviderConfig {
        app_id: "mycelix-unified".into(),
        default_role: Some("hearth".into()),
        log_prefix: "[Hearth]",
        connect_strategy: runtime_mode.connect_strategy(),
        status_labels: None,
    };
    view! {
        <HolochainProviderAuto config=config>
            <AppInner runtime_mode />
        </HolochainProviderAuto>
    }
}

#[component]
fn AppInner(runtime_mode: HearthRuntimeMode) -> impl IntoView {
    // Runtime provenance must exist before any domain state is initialized.
    provide_runtime_mode(runtime_mode);

    // Initialize providers in dependency order.
    crate::themes::provide_theme_context();
    crate::circadian::provide_circadian_context();
    crate::ambient_sound::provide_ambient_sound_context();
    provide_thermodynamic_context();
    provide_consciousness_context();
    provide_toast_context();
    provide_homeostasis_context(2, "--homeostasis");
    provide_hearth_context();
    crate::hearth_prefs::provide_hearth_prefs();

    // Action dispatch layer (real zome calls in Live, local mutation in Demo).
    crate::hearth_actions::provide_hearth_actions();

    // Wire consciousness + circadian → CSS custom properties.
    init_consciousness_ui();

    // Real-time signals from conductor (when connected).
    crate::signal_listener::start_signal_listener();

    // Simulated family life is permitted only in an explicitly selected Demo.
    if runtime_mode.is_demo() {
        crate::simulated_life::start_simulated_life();
    }

    view! {
        <Router>
            <a href="#main-content" class="skip-to-content">"skip to content"</a>
            <Nav />
            <RuntimeStatus />
            <DevPanel />
            // Ambient campfire — persistent warmth from the founding ceremony
            <crate::components::HearthFlame mode=crate::components::FlameMode::Ambient />
            <main id="main-content" class="main-content heartbeat" role="main" aria-label="hearth content">
                <Routes fallback=|| view! { <p class="not-found">"you’ve wandered off the path"</p> }>
                    <Route path=path!("/") view=HomePage />
                    <Route path=path!("/found") view=crate::pages::found::FoundingCeremony />
                    <Route path=path!("/settings") view=crate::pages::settings::SettingsPage />
                    <Route path=path!("/kinship") view=KinshipPage />
                    <Route path=path!("/care") view=CarePage />
                    <Route path=path!("/decisions") view=DecisionsPage />
                    <Route path=path!("/gratitude") view=GratitudePage />
                    <Route path=path!("/stories") view=StoriesPage />
                    <Route path=path!("/milestones") view=MilestonesPage />
                    <Route path=path!("/rhythms") view=RhythmsPage />
                    <Route path=path!("/emergency") view=EmergencyPage />
                    <Route path=path!("/resources") view=ResourcesPage />
                    <Route path=path!("/autonomy") view=AutonomyPage />
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
fn RuntimeStatus() -> impl IntoView {
    let mode = use_runtime_mode();
    let hearth = use_hearth();
    let hc = mycelix_leptos_core::holochain_provider::use_holochain();

    view! {
        <div class="runtime-status-wrap">
            {move || {
                if mode.is_demo() {
                    return Some(view! {
                        <AvailabilityState
                            kind=AvailabilityStateKind::Mock
                            title="Demo data"
                            description="This Hearth is simulated. Nothing shown here is live household data."
                            action=Option::<AnyView>::None
                        />
                    }.into_any());
                }

                match hearth.data_state.get() {
                    HearthDataState::Live => None,
                    HearthDataState::Empty => Some(view! {
                        <AvailabilityState
                            kind=AvailabilityStateKind::Empty
                            title="No Hearth yet"
                            description="The live conductor is connected, but this identity does not currently belong to a Hearth."
                            action=Option::<AnyView>::None
                        />
                    }.into_any()),
                    HearthDataState::Degraded => {
                        let detail = hc.last_error.get().unwrap_or_else(|| {
                            "Live Hearth data could not be fully synchronized. No demo data has been substituted.".into()
                        });
                        Some(view! {
                            <AvailabilityState
                                kind=AvailabilityStateKind::Degraded
                                title="Live data degraded"
                                description=detail
                                action=Option::<AnyView>::None
                            />
                        }.into_any())
                    }
                    HearthDataState::LoadingLive => Some(view! {
                        <div class="runtime-progress" role="status" aria-live="polite">
                            <span class="runtime-progress-dot"></span>
                            <span>"loading live Hearth data…"</span>
                        </div>
                    }.into_any()),
                    HearthDataState::AwaitingLive => {
                        match hc.status.get() {
                            ConnectionStatus::Connected if !hc.zome_call_signing_ready.get() => Some(view! {
                                <AvailabilityState
                                    kind=AvailabilityStateKind::Locked
                                    title="Signer required"
                                    description="The conductor is connected, but Live writes and reads requiring authorization are locked until a zome-call signer is available."
                                    action=Option::<AnyView>::None
                                />
                            }.into_any()),
                            ConnectionStatus::Reconnecting => Some(view! {
                                <AvailabilityState
                                    kind=AvailabilityStateKind::Degraded
                                    title="Reconnecting"
                                    description="The live conductor connection was interrupted. Hearth will resynchronize after reconnection; demo data is not being shown."
                                    action=Option::<AnyView>::None
                                />
                            }.into_any()),
                            ConnectionStatus::Disconnected => Some(view! {
                                <AvailabilityState
                                    kind=AvailabilityStateKind::Unavailable
                                    title="Live Hearth unavailable"
                                    description="No conductor connection is available. Start or connect to your Hearth conductor, or explicitly open Demo mode with ?mode=demo."
                                    action=Option::<AnyView>::None
                                />
                            }.into_any()),
                            ConnectionStatus::Mock => Some(view! {
                                <AvailabilityState
                                    kind=AvailabilityStateKind::Degraded
                                    title="Unexpected mock transport"
                                    description="Live mode will not accept mock fallback. Reload in explicit Demo mode if simulation is intended."
                                    action=Option::<AnyView>::None
                                />
                            }.into_any()),
                            _ => Some(view! {
                                <div class="runtime-progress" role="status" aria-live="polite">
                                    <span class="runtime-progress-dot"></span>
                                    <span>"connecting to live Hearth…"</span>
                                </div>
                            }.into_any()),
                        }
                    }
                    HearthDataState::Demo => unreachable!("Demo state is handled by runtime mode"),
                }
            }}
        </div>
    }
}
