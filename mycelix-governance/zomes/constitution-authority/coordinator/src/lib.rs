// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Coordinator for DNA-bound constitutional genesis.
//!
//! This zome intentionally exposes no amendment API in v0.1. Binding governance
//! can read a verified genesis constitution; any later constitutional mutation
//! remains unavailable until the binding-vote + threshold verifier path is wired.

use constitution_authority_integrity::{
    load_constitution_manifest, ConstitutionGenesisRecord, EntryTypes,
};
use hdk::prelude::*;
use mycelix_governance_constitution::{
    ConstitutionStatement, Digest32, GENESIS_MANIFEST_PROFILE, STATEMENT_PROFILE,
};

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct VerifiedConstitutionGenesis {
    /// DNA hash identifies the exact governance network whose properties were
    /// used for this verification.
    pub dna_hash: String,
    pub manifest_digest: Digest32,
    pub manifest_digest_profile: String,
    pub statement_digest: Digest32,
    pub statement_digest_profile: String,
    pub statement: ConstitutionStatement,
    /// v0.1 is intentionally genesis-only until transition verification lands.
    pub amendments_enabled: bool,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ConstitutionAuthorityStatus {
    pub protocol: String,
    pub dna_hash: String,
    pub genesis_bound: bool,
    pub amendments_enabled: bool,
    pub legacy_constitution_authoritative: bool,
    pub note: String,
}

fn verified_genesis() -> ExternResult<VerifiedConstitutionGenesis> {
    let manifest = load_constitution_manifest()?;
    let manifest_digest = manifest.digest().map_err(|e| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "Cannot digest constitutional genesis manifest: {e}"
        )))
    })?;
    let statement = manifest.genesis_statement().map_err(|e| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "Cannot derive constitutional genesis statement: {e}"
        )))
    })?;
    let statement_digest = statement.digest().map_err(|e| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "Cannot digest constitutional genesis statement: {e}"
        )))
    })?;
    let dna_hash = dna_info()?.hash.to_string();

    Ok(VerifiedConstitutionGenesis {
        dna_hash,
        manifest_digest,
        manifest_digest_profile: GENESIS_MANIFEST_PROFILE.into(),
        statement_digest,
        statement_digest_profile: STATEMENT_PROFILE.into(),
        statement,
        amendments_enabled: false,
    })
}

/// Returns constitutional authority derived directly from the immutable DNA
/// properties. No DHT write is required for authority to exist.
#[hdk_extern]
pub fn get_verified_constitution_genesis(_: ()) -> ExternResult<VerifiedConstitutionGenesis> {
    verified_genesis()
}

/// Publish the exact DNA-bound genesis record for audit/discovery.
///
/// Anyone may publish the identical record. The author gains no authority; the
/// integrity zome independently checks that the bytes match the DNA manifest.
#[hdk_extern]
pub fn publish_constitution_genesis(_: ()) -> ExternResult<Record> {
    let manifest = load_constitution_manifest()?;
    let record = ConstitutionGenesisRecord::from_manifest(&manifest).map_err(|e| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "Cannot derive constitutional genesis record: {e}"
        )))
    })?;
    let action_hash = create_entry(&EntryTypes::ConstitutionGenesis(record))?;
    get(action_hash, GetOptions::default())?.ok_or_else(|| {
        wasm_error!(WasmErrorInner::Guest(
            "Could not retrieve published constitutional genesis".into(),
        ))
    })
}

/// Resolve a previously published byte-identical genesis record by deterministic
/// entry hash. Absence is not an authority failure because DNA properties remain
/// the source of truth; this endpoint is for audit/discovery only.
#[hdk_extern]
pub fn get_published_constitution_genesis(_: ()) -> ExternResult<Option<Record>> {
    let manifest = load_constitution_manifest()?;
    let record = ConstitutionGenesisRecord::from_manifest(&manifest).map_err(|e| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "Cannot derive constitutional genesis record: {e}"
        )))
    })?;
    let entry_hash = hash_entry(&EntryTypes::ConstitutionGenesis(record))?;
    get(entry_hash, GetOptions::default())
}

/// Explicit capability/status surface so clients cannot mistake the legacy
/// mutable constitution zome for the binding constitutional authority path.
#[hdk_extern]
pub fn get_constitution_authority_status(_: ()) -> ExternResult<ConstitutionAuthorityStatus> {
    let genesis = verified_genesis()?;
    Ok(ConstitutionAuthorityStatus {
        protocol: "mycelix-governance-constitution-authority-v0.1".into(),
        dna_hash: genesis.dna_hash,
        genesis_bound: true,
        amendments_enabled: false,
        legacy_constitution_authoritative: false,
        note: "DNA-bound genesis is authoritative. Constitutional amendments remain fail-closed until binding-vote and threshold transition verification is wired.".into(),
    })
}
