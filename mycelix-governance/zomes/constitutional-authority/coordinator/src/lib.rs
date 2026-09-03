// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Read-only access to the DNA-bound constitutional authority root.

use constitutional_authority_integrity::GovernanceDnaProperties;
use hdk::prelude::*;
use mycelix_constitution_genesis::ConstitutionalGenesisManifest;

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct VerifiedConstitutionalRoot {
    pub dna_hash: String,
    pub network_seed: String,
    pub manifest: ConstitutionalGenesisManifest,
}

/// Return the constitutional root only after re-validating the exact DNA
/// properties in the running cell. This function never consults mutable DHT data.
#[hdk_extern]
pub fn get_verified_constitutional_root(_: ()) -> ExternResult<VerifiedConstitutionalRoot> {
    let properties = GovernanceDnaProperties::try_from_dna_properties().map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "Constitutional DNA properties are missing or undecodable: {error}"
        )))
    })?;
    properties.validate().map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(error))
    })?;

    let info = dna_info()?;
    Ok(VerifiedConstitutionalRoot {
        dna_hash: info.hash.to_string(),
        network_seed: info.modifiers.network_seed,
        manifest: properties.constitutional_genesis,
    })
}
