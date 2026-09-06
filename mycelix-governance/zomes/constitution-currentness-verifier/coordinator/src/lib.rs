// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Fail-closed leased currentness for binding constitutional authority.
//!
//! This verifier intentionally supports only the DNA-bound genesis mode while
//! amendments are disabled. It does not inspect DHT transition candidates and it
//! does not infer currentness from absence, ordering, timestamps, authorship, or
//! local DHT visibility. Amendment currentness remains unavailable until a later
//! theorem proves complete transition state and bounded verifier horizons.

use hdk::prelude::*;
use mycelix_governance_constitution::{
    ConstitutionGenesisManifest, ConstitutionStatement, Digest32, GENESIS_MANIFEST_PROFILE,
    STATEMENT_PROFILE,
};
use serde::de::DeserializeOwned;
use serde::{Deserialize, Serialize};

pub const PROTOCOL_VERSION: &str = "mycelix-governance-current-constitution-leased-v0.1";
pub const GENESIS_CURRENTNESS_LEASE_BASIS: &str =
    "dna-immutable-genesis-amendments-disabled-local-reuse-v1";
pub const GENESIS_CURRENTNESS_REUSE_MS: u64 = 30_000;

const CONSTITUTION_AUTHORITY_ZOME: &str = "constitution_authority";
const CONSTITUTION_GENESIS_FUNCTION: &str = "get_verified_constitution_genesis";
const MAX_DNA_HASH_BYTES: usize = 1024;

#[derive(Serialize, Deserialize, Debug, Clone)]
struct VerifiedConstitutionGenesisMirror {
    dna_hash: String,
    manifest_digest: Digest32,
    manifest_digest_profile: String,
    statement_digest: Digest32,
    statement_digest_profile: String,
    statement: ConstitutionStatement,
    amendments_enabled: bool,
}

/// Transport projection of one bounded current-constitution verification.
///
/// This is evidence for local consumers, not authority merely because it can be
/// deserialized. Consumers must call this designated verifier directly.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
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

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct ConstitutionCurrentnessVerifierStatus {
    pub protocol: String,
    pub genesis_currentness_supported: bool,
    pub amendment_currentness_supported: bool,
    pub candidate_discovery_grants_authority: bool,
    pub absence_of_transition_record_grants_authority: bool,
    pub unbounded_transition_verifier_receipts_accepted: bool,
    pub local_reuse_cap_is_authority_expiry: bool,
}

fn call_local<I, O>(zome: &str, function: &str, input: I) -> ExternResult<O>
where
    I: Serialize,
    O: DeserializeOwned,
{
    let response = call(
        CallTargetCell::Local,
        ZomeName::from(zome),
        FunctionName::from(function),
        None,
        ExternIO::encode(input)
            .map_err(|error| wasm_error!(WasmErrorInner::Guest(error.to_string())))?,
    )?;
    let io = match response {
        ZomeCallResponse::Ok(io) => io,
        other => {
            return Err(wasm_error!(WasmErrorInner::Guest(format!(
                "{zome}::{function} unavailable; constitutional currentness fails closed: {other:?}"
            ))));
        }
    };
    io.decode::<O>().map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "cannot decode {zome}::{function} response: {error}"
        )))
    })
}

fn validate_dna_hash(value: &str) -> ExternResult<()> {
    let bytes = value.as_bytes();
    if bytes.is_empty()
        || bytes.len() > MAX_DNA_HASH_BYTES
        || bytes.iter().any(|byte| byte.is_ascii_whitespace())
    {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "constitution authority returned invalid DNA identity".into(),
        )));
    }
    Ok(())
}

fn manifest_from_genesis(
    statement: &ConstitutionStatement,
) -> ExternResult<ConstitutionGenesisManifest> {
    if statement.version != 1 || statement.parent_statement_digest.is_some() {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "constitution authority did not return a genesis statement".into(),
        )));
    }
    let manifest = ConstitutionGenesisManifest {
        protocol_version: statement.protocol_version.clone(),
        network_id: statement.network_id.clone(),
        institution_id: statement.institution_id.clone(),
        constitution_id: statement.constitution_id.clone(),
        rulebook_id: statement.rulebook_id.clone(),
        rulebook_version: statement.rulebook_version.clone(),
        rulebook: statement.rulebook.clone(),
        charter: statement.charter.clone(),
        parameters: statement.parameters.clone(),
        amendment_policy: statement.amendment_policy.clone(),
        binding_vote_profile: statement.binding_vote_profile.clone(),
        threshold_authority_profile: statement.threshold_authority_profile.clone(),
        effective_from_ms: statement.effective_from_ms,
    };
    manifest.verify_genesis_statement(statement).map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "verified genesis cannot be reconstructed as a manifest: {error}"
        )))
    })?;
    Ok(manifest)
}

fn resolve_verified_genesis() -> ExternResult<VerifiedConstitutionGenesisMirror> {
    let genesis: VerifiedConstitutionGenesisMirror = call_local(
        CONSTITUTION_AUTHORITY_ZOME,
        CONSTITUTION_GENESIS_FUNCTION,
        (),
    )?;
    validate_dna_hash(&genesis.dna_hash)?;
    if genesis.amendments_enabled {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "amendments are enabled but bounded amendment currentness is not implemented; fail closed"
                .into(),
        )));
    }
    if genesis.manifest_digest_profile != GENESIS_MANIFEST_PROFILE
        || genesis.statement_digest_profile != STATEMENT_PROFILE
    {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "constitution authority returned unexpected canonical digest profile".into(),
        )));
    }
    genesis.statement.validate().map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "constitution authority returned invalid genesis statement: {error}"
        )))
    })?;
    let statement_digest = genesis.statement.digest().map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "cannot digest DNA-bound genesis statement: {error}"
        )))
    })?;
    if statement_digest != genesis.statement_digest {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "constitution authority returned inconsistent genesis statement identity".into(),
        )));
    }
    let manifest = manifest_from_genesis(&genesis.statement)?;
    let manifest_digest = manifest.digest().map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "cannot digest reconstructed DNA genesis manifest: {error}"
        )))
    })?;
    if manifest_digest != genesis.manifest_digest {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "constitution authority returned inconsistent genesis manifest identity".into(),
        )));
    }
    Ok(genesis)
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

#[hdk_extern]
pub fn get_leased_current_constitution(_: ()) -> ExternResult<LeasedVerifiedCurrentConstitution> {
    let genesis = resolve_verified_genesis()?;
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
        genesis.statement_digest.to_hex()
    );

    Ok(LeasedVerifiedCurrentConstitution {
        protocol: PROTOCOL_VERSION.into(),
        dna_hash: genesis.dna_hash,
        statement: genesis.statement,
        statement_digest: genesis.statement_digest,
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

#[hdk_extern]
pub fn constitution_currentness_verifier_status(
    _: (),
) -> ExternResult<ConstitutionCurrentnessVerifierStatus> {
    Ok(ConstitutionCurrentnessVerifierStatus {
        protocol: PROTOCOL_VERSION.into(),
        genesis_currentness_supported: true,
        amendment_currentness_supported: false,
        candidate_discovery_grants_authority: false,
        absence_of_transition_record_grants_authority: false,
        unbounded_transition_verifier_receipts_accepted: false,
        local_reuse_cap_is_authority_expiry: false,
    })
}
