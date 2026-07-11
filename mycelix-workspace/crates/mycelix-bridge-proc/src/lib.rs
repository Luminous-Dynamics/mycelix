// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

use proc_macro::TokenStream;
use quote::quote;
use syn::{ItemFn, parse_macro_input};

/// Marks a zome function for type-safe bridge generation.
///
/// This macro:
/// 1. Passes through the original `#[hdk_extern]` or `pub fn` definition.
/// 2. Appends metadata about the function (name, inputs, outputs) to a
///    build-time registry for the codegen to consume.
///
/// # Attributes
/// * `role` - The hApp role this zome belongs to (e.g., "civic", "finance").
/// * `zome` - The name of the zome (optional, defaults to crate name).
#[proc_macro_attribute]
pub fn mycelix_zome_fn(attr: TokenStream, item: TokenStream) -> TokenStream {
    let input_fn = parse_macro_input!(item as ItemFn);
    let fn_name = &input_fn.sig.ident;

    // In a full implementation, this would append to a file in OUT_DIR or
    // a project-local .mycelix/bridge-registry.json.
    // For now, we pass through and emit a dummy trait for metadata discovery.

    let r#gen = quote! {
        #input_fn

        /// Metadata marker for #fn_name
        #[allow(non_upper_case_globals)]
        const #fn_name: () = ();
    };

    r#gen.into()
}

/// Arguments for `#[sovereign_gated(tier, "bridge_zome_name")]`.
///
/// `bridge_zome_name` must be the actual local bridge zome's name for the
/// crate this macro is used in (e.g. "energy_bridge", "governance_bridge") --
/// `require_civic`/`gate_civic` dispatches via `CallTargetCell::Local`, which
/// can only reach a zome bundled in the SAME DNA as the caller, not another
/// cluster's DNA. A hardcoded literal here previously always targeted
/// "civic_bridge" regardless of caller, which broke every non-civic user of
/// this macro (found 2026-07-08 by actually running mycelix-energy's and
/// mycelix-governance's long-`#[ignore]`d sweettests for the first time --
/// see feedback_sovereign_gated_hardcoded_zome_bug memory).
struct SovereignGatedArgs {
    tier: syn::Ident,
    zome: syn::LitStr,
}

impl syn::parse::Parse for SovereignGatedArgs {
    fn parse(input: syn::parse::ParseStream) -> syn::Result<Self> {
        let tier: syn::Ident = input.parse()?;
        input.parse::<syn::Token![,]>()?;
        let zome: syn::LitStr = input.parse()?;
        Ok(SovereignGatedArgs { tier, zome })
    }
}

/// Marks a zome function as requiring a specific 8D Sovereign Profile threshold.
///
/// Automatically injects the `require_civic` call into the function body.
/// Usage: `#[sovereign_gated(basic, "energy_bridge")]` -- the second argument
/// is this crate's OWN local bridge zome name (see `SovereignGatedArgs` doc).
#[proc_macro_attribute]
pub fn sovereign_gated(attr: TokenStream, item: TokenStream) -> TokenStream {
    let mut input_fn = parse_macro_input!(item as ItemFn);
    let args = parse_macro_input!(attr as SovereignGatedArgs);
    let tier_str = args.tier.to_string();
    let zome_lit = &args.zome;

    let requirement = match tier_str.as_str() {
        "basic" => quote! { mycelix_bridge_common::civic_requirement_basic() },
        "proposal" => quote! { mycelix_bridge_common::civic_requirement_proposal() },
        "steward" => quote! { mycelix_bridge_common::civic_requirement_steward() },
        _ => quote! { mycelix_bridge_common::civic_requirement_basic() },
    };

    // Inject validation at start of function
    let block = &input_fn.block;
    let fn_name_str = input_fn.sig.ident.to_string();

    input_fn.block = Box::new(syn::parse_quote! {
        {
            // `require_civic`/`gate_civic` returns Ok(GovernanceEligibility)
            // even when the agent DOESN'T meet the tier -- `eligible: false`
            // is a normal, successful result, not an Err. `?` alone only
            // catches a hard failure (identity unreachable, decode error);
            // it does NOT enforce the tier requirement. Previously this
            // eligibility result was discarded entirely (bound to nothing),
            // so #[sovereign_gated(...)] never actually blocked an
            // ineligible caller -- found 2026-07-08 alongside the
            // hardcoded-zome-name bug; see
            // feedback_sovereign_gated_hardcoded_zome_bug memory.
            let __sovereign_gate_result =
                mycelix_zome_helpers::require_civic(#zome_lit, &#requirement, #fn_name_str)?;
            if !__sovereign_gate_result.eligible {
                return Err(::hdk::prelude::wasm_error!(::hdk::prelude::WasmErrorInner::Guest(
                    format!(
                        "Sovereign gate denied for '{}': tier {:?} does not meet requirement ({:?})",
                        #fn_name_str,
                        __sovereign_gate_result.tier,
                        __sovereign_gate_result.reasons
                    )
                )));
            }
            #block
        }
    });

    quote! {
        #input_fn
    }
    .into()
}
