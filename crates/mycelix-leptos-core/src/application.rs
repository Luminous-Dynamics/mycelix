// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Canonical shared application composition for Mycelix Leptos frontends.
//!
//! This module intentionally owns only cross-domain providers. Domain-specific
//! contexts, routes, and authorization semantics remain in each application.

use leptos::prelude::*;

use crate::{
    init_consciousness_ui, provide_consciousness_context, provide_homeostasis_context,
    provide_thermodynamic_context, provide_toast_context, HolochainProviderAuto,
    HolochainProviderConfig, ToastContainer,
};

/// Shared runtime composition for Mycelix applications.
///
/// The component establishes the common Holochain/runtime providers and leaves
/// domain providers and routing to `children`. This keeps the application root
/// consistent without turning `mycelix-leptos-core` into a source of domain
/// authority.
#[component]
pub fn MycelixApplication(
    config: HolochainProviderConfig,
    #[prop(optional, default = 2)] homeostasis_capacity: usize,
    #[prop(optional, into, default = "--homeostasis".into())] homeostasis_css_var: String,
    children: Children,
) -> impl IntoView {
    view! {
        <HolochainProviderAuto config=config>
            <MycelixApplicationInner
                homeostasis_capacity=homeostasis_capacity
                homeostasis_css_var=homeostasis_css_var
            >
                {children()}
            </MycelixApplicationInner>
        </HolochainProviderAuto>
    }
}

#[component]
fn MycelixApplicationInner(
    homeostasis_capacity: usize,
    homeostasis_css_var: String,
    children: Children,
) -> impl IntoView {
    // Shared provider initialization order is intentional. Domain providers
    // should be initialized by the domain app after this boundary is present.
    provide_thermodynamic_context();
    provide_consciousness_context();
    provide_toast_context();
    provide_homeostasis_context(homeostasis_capacity, &homeostasis_css_var);

    // Wire shared experiential state to CSS custom properties once the
    // providers above exist.
    init_consciousness_ui();

    view! {
        {children()}
        <ToastContainer />
    }
}
