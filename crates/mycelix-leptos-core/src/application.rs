// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Canonical shared application composition for Mycelix Leptos frontends.
//!
//! This module intentionally owns only cross-domain runtime providers.
//! Domain-specific themes, contexts, routes, actions, and authorization
//! semantics remain in each application.

use leptos::prelude::*;

use crate::{
    HolochainProviderAuto, HolochainProviderConfig, ToastContainer, init_consciousness_ui,
    provide_consciousness_context, provide_homeostasis_context, provide_thermodynamic_context,
    provide_toast_context,
};

/// Optional homeostasis configuration for a Mycelix application.
///
/// The CSS variable name is static because the underlying homeostasis provider
/// installs a reactive effect that retains the name for the lifetime of the app.
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

/// Cross-domain configuration for [`MycelixApplication`].
///
/// Domain configuration does not belong here. In particular, this type must
/// not become a second source of truth for routes, domain authorization, or
/// business state.
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
/// This component establishes the Holochain provider first, then initializes
/// common experiential and toast contexts inside that boundary. Domain
/// providers and routing remain children of this component so they can consume
/// the shared contexts without moving domain authority into this crate.
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

/// Common providers that must be initialized inside the Holochain boundary.
#[component]
fn MycelixSharedProviders(
    homeostasis: Option<HomeostasisConfig>,
    children: Children,
) -> impl IntoView {
    // Shared provider initialization order is intentional. Domain providers
    // should be initialized by the domain app after this boundary is present.
    provide_thermodynamic_context();
    provide_consciousness_context();
    provide_toast_context();

    if let Some(homeostasis) = homeostasis {
        provide_homeostasis_context(homeostasis.counters, homeostasis.css_var_name);
    }

    // Wire shared experiential state to CSS custom properties once the
    // consciousness and thermodynamic providers above exist.
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
