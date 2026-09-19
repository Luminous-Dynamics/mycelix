// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Canonical cross-domain application composition for Mycelix Leptos frontends.
//!
//! This module owns shared runtime composition only. It deliberately does not
//! own routes, domain state, domain actions, domain themes, capability or
//! authorization semantics, business rules, or lifecycle truth. Those remain
//! with each domain application.
//!
//! The composition boundary is therefore infrastructure, not authority.

use leptos::prelude::*;

use crate::{
    HolochainProviderAuto, HolochainProviderConfig, ToastContainer, init_consciousness_ui,
    provide_consciousness_context, provide_homeostasis_context, provide_thermodynamic_context,
    provide_toast_context,
};

/// Optional homeostasis configuration for a Mycelix application.
///
/// The CSS variable name is static because the underlying homeostasis provider
/// installs a reactive effect that retains it for the application lifetime.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct HomeostasisConfig {
    pub counters: usize,
    pub css_var_name: &'static str,
}

impl HomeostasisConfig {
    pub const fn new(counters: usize, css_var_name: &'static str) -> Self {
        Self {
            counters,
            css_var_name,
        }
    }
}

/// Cross-domain runtime configuration for [`MycelixApplication`].
///
/// Domain configuration does not belong here. In particular, this type must
/// not become a second source of truth for routes, authorization, governance,
/// finance, learning, messaging, commerce, or other business state.
#[derive(Clone, Debug)]
pub struct MycelixApplicationConfig {
    pub holochain: HolochainProviderConfig,
    pub homeostasis: Option<HomeostasisConfig>,
}

impl MycelixApplicationConfig {
    pub fn new(holochain: HolochainProviderConfig) -> Self {
        Self {
            holochain,
            homeostasis: None,
        }
    }

    pub fn with_homeostasis(mut self, homeostasis: HomeostasisConfig) -> Self {
        self.homeostasis = Some(homeostasis);
        self
    }
}

/// Canonical shared runtime composition for Mycelix applications.
///
/// The Holochain provider is established first. Common experiential and toast
/// contexts are then initialized inside that boundary. Domain providers and
/// routing remain children so they may consume shared runtime contexts without
/// moving domain authority into the shared application shell.
///
/// This component does not infer readiness or authority from connectivity; the
/// underlying Holochain context preserves transport and signing readiness as
/// separate states.
#[component]
pub fn MycelixApplication(
    config: MycelixApplicationConfig,
    children: Children,
) -> impl IntoView {
    let MycelixApplicationConfig {
        holochain,
        homeostasis,
    } = config;

    view! {
        <HolochainProviderAuto config=holochain>
            <MycelixSharedProviders homeostasis=homeostasis>
                {children()}
            </MycelixSharedProviders>
        </HolochainProviderAuto>
    }
}

/// Shared providers that must be initialized inside the transport boundary.
#[component]
fn MycelixSharedProviders(
    homeostasis: Option<HomeostasisConfig>,
    children: Children,
) -> impl IntoView {
    // Initialization order is intentional. Domain providers belong below this
    // component and must not be pulled upward merely for convenience.
    provide_thermodynamic_context();
    provide_consciousness_context();
    provide_toast_context();

    if let Some(homeostasis) = homeostasis {
        provide_homeostasis_context(homeostasis.counters, homeostasis.css_var_name);
    }

    // This bridge requires consciousness + thermodynamic contexts above.
    init_consciousness_ui();

    view! {
        {children()}
        <ToastContainer />
    }
}

#[cfg(test)]
mod tests {
    use super::HomeostasisConfig;

    #[test]
    fn homeostasis_config_preserves_counter_and_css_contract() {
        let config = HomeostasisConfig::new(2, "--homeostasis");
        assert_eq!(config.counters, 2);
        assert_eq!(config.css_var_name, "--homeostasis");
    }
}
