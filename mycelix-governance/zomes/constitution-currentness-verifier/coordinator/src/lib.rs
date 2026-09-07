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
use mycelix_governance_constitution_currentness::{
    LeasedVerifiedCurrentConstitution, GENESIS_CURRENTNESS_REUSE_MS, PROTOCOL_VERSION,
};
use serde::de::DeserializeOwned;
use serde::{Deserialize, Serialize};

const CONSTITUTION_AUTHORITY_ZOME: &str = "constitution_authority";
const CONSTITUTION_GENESIS_FUNCTION: &str = "get_verified_constitution_genesis";

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

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct ConstitutionCurrentnessVerifierStatus {
    pub protocol: String,
    pub genesis_currentness_supported: bool,
    pub amendment_currentness_supported: bool,
    pub currentness_evidence_identity_explicit: bool,
    pub shared_currentness_contract_consumed: bool,
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
    let current = LeasedVerifiedCurrentConstitution::new_genesis(
        genesis.dna_hash,
        genesis.statement,
        verified_at_ms,
        valid_until_ms,
    )
    .map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "shared constitutional-currentness contract denied genesis evidence: {error}"
        )))
    })?;
    if current.statement_digest != genesis.statement_digest {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "shared constitutional-currentness contract changed the authoritative statement identity"
                .into(),
        )));
    }
    Ok(current)
}

#[hdk_extern]
pub fn constitution_currentness_verifier_status(
    _: (),
) -> ExternResult<ConstitutionCurrentnessVerifierStatus> {
    Ok(ConstitutionCurrentnessVerifierStatus {
        protocol: PROTOCOL_VERSION.into(),
        genesis_currentness_supported: true,
        amendment_currentness_supported: false,
        currentness_evidence_identity_explicit: true,
        shared_currentness_contract_consumed: true,
        candidate_discovery_grants_authority: false,
        absence_of_transition_record_grants_authority: false,
        unbounded_transition_verifier_receipts_accepted: false,
        local_reuse_cap_is_authority_expiry: false,
    })
}
