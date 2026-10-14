// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Coordinator-only runtime gates for Mycelix Space.
//!
//! This crate owns host-function calls that are unavailable to integrity zomes.
//! Shared data types must not depend on this crate or on HDK.

use mycelix_space_shared::CivicRequirement;

/// Best-effort civic gate for space operations.
///
/// Attempts a cross-role call to the identity cluster to fetch a
/// `SovereignCredential` (8D). Falls back to bootstrap mode if the identity
/// cluster is unavailable (standalone space hApp or network bootstrap).
pub fn gate_space_operation(
    requirement: &CivicRequirement,
    action_name: &str,
) -> hdk::prelude::ExternResult<()> {
    use hdk::prelude::*;
    use mycelix_bridge_common::sovereign_gate::{DimensionWeights, SovereignCredential};

    let agent = agent_info()?.agent_initial_pubkey;
    let did = format!("did:mycelix:{}", agent);

    // Try cross-role call to identity cluster's bridge zome
    match call(
        CallTargetCell::OtherRole("identity".into()),
        ZomeName::new("identity_bridge"),
        FunctionName::new("issue_sovereign_credential"),
        None,
        did,
    ) {
        Ok(ZomeCallResponse::Ok(extern_io)) => {
            match extern_io.decode::<SovereignCredential>() {
                Ok(cred) => {
                    let weights = DimensionWeights::governance();
                    if !cred.profile.meets_requirement(requirement, &weights) {
                        let tier = cred.profile.tier(&weights);
                        return Err(wasm_error!(WasmErrorInner::Guest(format!(
                            "Civic gate denied for action '{}': requires {:?} tier, \
                             agent has {:?}",
                            action_name, requirement.min_tier, tier
                        ))));
                    }
                    Ok(())
                }
                Err(_) => {
                    // Credential decode failed — bootstrap mode
                    debug!(
                        "Space civic gate: credential decode failed, \
                         bootstrap mode for: {}",
                        action_name
                    );
                    Ok(())
                }
            }
        }
        Ok(ZomeCallResponse::NetworkError(e)) => {
            // Identity cluster unreachable — allow in bootstrap mode
            debug!(
                "Space civic gate: identity cluster unreachable ({}), \
                 bootstrap mode for: {}",
                e, action_name
            );
            Ok(())
        }
        Ok(_) => {
            // Unexpected response variant — allow in bootstrap mode
            debug!(
                "Space civic gate: unexpected response, \
                 bootstrap mode for: {}",
                action_name
            );
            Ok(())
        }
        Err(_) => {
            // Zome not found — standalone space hApp without identity cluster
            debug!(
                "Space consciousness gate: no identity cluster, \
                 bootstrap mode for: {}",
                action_name
            );
            Ok(())
        }
    }
}
