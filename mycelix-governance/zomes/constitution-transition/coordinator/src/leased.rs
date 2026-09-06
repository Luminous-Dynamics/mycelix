// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Leased binding-constitution currentness for the genesis-only authority mode.
//!
//! While constitutional amendments are disabled by the DNA-bound genesis
//! authority, current constitutional truth is the immutable genesis statement.
//! This endpoint therefore does not infer currentness from absence/order of DHT
//! transition candidates. Once amendments are enabled, it fails closed until a
//! separate amendment-currentness theorem transports complete transition state
//! and bounded verifier horizons.

use super::*;

pub const LEASED_CURRENT_CONSTITUTION_PROTOCOL: &str =
    "mycelix-governance-current-constitution-leased-v0.1";
pub const GENESIS_CURRENTNESS_LEASE_BASIS: &str =
    "dna-immutable-genesis-amendments-disabled-local-reuse-v1";
pub const GENESIS_CURRENTNESS_REUSE_MS: u64 = 30_000;

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct LeasedVerifiedCurrentConstitution {
    pub protocol: String,
    pub dna_hash: String,
    pub statement: ConstitutionStatement,
    pub statement_digest: Digest32,
    pub verified_transition_count: u64,
    pub legacy_constitution_authoritative: bool,
    pub lease_basis: String,
    pub verification_ref: String,
    pub verified_at_ms: u64,
    pub valid_until_ms: u64,
    pub genesis_currentness_by_amendments_disabled: bool,
    pub transition_currentness_supported: bool,
    pub candidate_discovery_used_for_positive_currentness: bool,
}

fn now_ms() -> ExternResult<u64> {
    let micros = sys_time()?.as_micros();
    if micros <= 0 {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "constitutional currentness time must be positive".into(),
        )));
    }
    Ok(micros as u64 / 1_000)
}

/// Return a bounded reusable current-constitution projection only when the
/// authoritative constitution mode is still DNA-bound genesis with amendments
/// disabled. No DHT candidate discovery participates in this positive result.
#[hdk_extern]
pub fn get_verified_current_constitution_leased(
    _: (),
) -> ExternResult<LeasedVerifiedCurrentConstitution> {
    let genesis = load_genesis()?;
    if genesis.amendments_enabled {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "leased amendment currentness is not implemented; fail closed until complete transition state and bounded verifier leases are qualified"
                .into(),
        )));
    }

    let statement = genesis.statement;
    let statement_digest = statement.digest().map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "cannot digest DNA-bound current constitution: {error}"
        )))
    })?;
    if statement_digest != genesis.statement_digest {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "DNA-bound genesis statement identity changed during leased currentness projection"
                .into(),
        )));
    }

    let verified_at_ms = now_ms()?;
    let valid_until_ms = verified_at_ms
        .checked_add(GENESIS_CURRENTNESS_REUSE_MS)
        .ok_or_else(|| {
            wasm_error!(WasmErrorInner::Guest(
                "genesis currentness reuse window overflow".into(),
            ))
        })?;
    let verification_ref = format!(
        "constitution-current-genesis:{STATEMENT_PROFILE}:{}",
        statement_digest.to_hex()
    );

    Ok(LeasedVerifiedCurrentConstitution {
        protocol: LEASED_CURRENT_CONSTITUTION_PROTOCOL.into(),
        dna_hash: genesis.dna_hash,
        statement,
        statement_digest,
        verified_transition_count: 0,
        legacy_constitution_authoritative: false,
        lease_basis: GENESIS_CURRENTNESS_LEASE_BASIS.into(),
        verification_ref,
        verified_at_ms,
        valid_until_ms,
        genesis_currentness_by_amendments_disabled: true,
        transition_currentness_supported: false,
        candidate_discovery_used_for_positive_currentness: false,
    })
}
