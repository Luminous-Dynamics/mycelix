// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! DNA-bound constitutional authority integrity layer.
//!
//! v0.1 intentionally authorizes only constitutional genesis. Amendment
//! transitions are NOT accepted here until binding-vote and threshold proofs can
//! be verified end-to-end. This avoids replacing first-writer-wins with a new
//! "first transition-shaped record wins" failure mode.

use hdi::prelude::*;
use mycelix_governance_constitution::{
    ConstitutionGenesisManifest, ConstitutionStatement, Digest32,
};

/// Governance network properties committed into the DNA hash.
///
/// The manifest deliberately contains no privileged genesis writer. The DNA
/// itself commits the exact genesis constitution.
#[dna_properties]
pub struct GovernanceDnaProperties {
    pub governance_constitution: Option<ConstitutionGenesisManifest>,
}

/// Discoverable copy of the exact DNA-bound genesis statement.
///
/// The author is not authority-bearing. Multiple agents may publish the same
/// byte-identical record; every valid record must resolve to the exact manifest
/// committed by this DNA.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct ConstitutionGenesisRecord {
    pub manifest_digest: Digest32,
    pub statement: ConstitutionStatement,
}

impl ConstitutionGenesisRecord {
    pub fn from_manifest(
        manifest: &ConstitutionGenesisManifest,
    ) -> Result<Self, mycelix_governance_constitution::ConstitutionError> {
        Ok(Self {
            manifest_digest: manifest.digest()?,
            statement: manifest.genesis_statement()?,
        })
    }

    pub fn validate_against_manifest(
        &self,
        manifest: &ConstitutionGenesisManifest,
    ) -> Result<(), String> {
        manifest
            .validate()
            .map_err(|e| format!("Invalid DNA constitutional manifest: {e}"))?;
        let expected_digest = manifest
            .digest()
            .map_err(|e| format!("Cannot digest DNA constitutional manifest: {e}"))?;
        if self.manifest_digest != expected_digest {
            return Err("Genesis record manifest digest differs from DNA properties".into());
        }
        manifest
            .verify_genesis_statement(&self.statement)
            .map_err(|e| format!("Genesis statement differs from DNA properties: {e}"))
    }
}

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    ConstitutionGenesis(ConstitutionGenesisRecord),
}

/// v0.1 defines no application links. A future amendment lineage will add
/// dedicated, integrity-checked indexes only after transition proof semantics are
/// available.
#[hdk_link_types]
pub enum LinkTypes {
    Reserved,
}

pub fn load_constitution_manifest() -> ExternResult<ConstitutionGenesisManifest> {
    let properties = GovernanceDnaProperties::try_from_dna_properties().map_err(|e| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "Cannot decode governance DNA properties: {e}"
        )))
    })?;
    let manifest = properties.governance_constitution.ok_or_else(|| {
        wasm_error!(WasmErrorInner::Guest(
            "Binding governance disabled: DNA has no governance_constitution manifest".into(),
        ))
    })?;
    manifest.validate().map_err(|e| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "Binding governance disabled: invalid governance_constitution manifest: {e}"
        )))
    })?;
    Ok(manifest)
}

/// Refuse to instantiate this constitutional-authority cell without an exact,
/// valid DNA-bound manifest. This is intentionally stricter than silently
/// falling back to the legacy mutable constitution zome.
#[hdk_extern]
pub fn genesis_self_check(_: GenesisSelfCheckData) -> ExternResult<ValidateCallbackResult> {
    match load_constitution_manifest() {
        Ok(_) => Ok(ValidateCallbackResult::Valid),
        Err(error) => Ok(ValidateCallbackResult::Invalid(format!(
            "Constitutional genesis unavailable: {error}"
        ))),
    }
}

fn validate_create_genesis(
    entry: ConstitutionGenesisRecord,
) -> ExternResult<ValidateCallbackResult> {
    let manifest = load_constitution_manifest()?;
    match entry.validate_against_manifest(&manifest) {
        Ok(()) => Ok(ValidateCallbackResult::Valid),
        Err(message) => Ok(ValidateCallbackResult::Invalid(message)),
    }
}

#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(store_entry) => match store_entry {
            OpEntry::CreateEntry { app_entry, .. } => match app_entry {
                EntryTypes::ConstitutionGenesis(entry) => validate_create_genesis(entry),
            },
            OpEntry::UpdateEntry { .. } => Ok(ValidateCallbackResult::Invalid(
                "Constitutional genesis is immutable; updates are forbidden".into(),
            )),
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterCreateLink { .. } => Ok(ValidateCallbackResult::Invalid(
            "constitution-authority v0.1 defines no application links".into(),
        )),
        FlatOp::RegisterDeleteLink { .. } => Ok(ValidateCallbackResult::Invalid(
            "constitution-authority v0.1 defines no deletable links".into(),
        )),
        FlatOp::RegisterDelete(_) => Ok(ValidateCallbackResult::Invalid(
            "Constitutional genesis is immutable; deletion is forbidden".into(),
        )),
        FlatOp::StoreRecord(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterAgentActivity(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterUpdate(_) => Ok(ValidateCallbackResult::Valid),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_governance_constitution::{
        ConstitutionId, InstitutionId, NetworkId, ProfiledDigest, RulebookId, PROTOCOL_VERSION,
    };

    fn digest(byte: u8) -> Digest32 {
        Digest32([byte; 32])
    }

    fn pd(byte: u8, profile: &str) -> ProfiledDigest {
        ProfiledDigest {
            digest: digest(byte),
            profile: profile.into(),
        }
    }

    fn manifest() -> ConstitutionGenesisManifest {
        ConstitutionGenesisManifest {
            protocol_version: PROTOCOL_VERSION.into(),
            network_id: NetworkId::new("network:test").unwrap(),
            institution_id: InstitutionId::new("institution:test").unwrap(),
            constitution_id: ConstitutionId::new("constitution:test").unwrap(),
            rulebook_id: RulebookId::new("rulebook:test").unwrap(),
            rulebook_version: "1".into(),
            rulebook: pd(1, "mycelix-rulebook-v1"),
            charter: pd(2, "mycelix-charter-v1"),
            parameters: pd(3, "mycelix-parameters-v1"),
            amendment_policy: pd(4, "mycelix-amendment-policy-v1"),
            binding_vote_profile: "mycelix-binding-vote-v2".into(),
            threshold_authority_profile: "mycelix-threshold-authority-v1".into(),
            effective_from_ms: 1_000,
        }
    }

    #[test]
    fn exact_genesis_record_matches_manifest() {
        let manifest = manifest();
        let record = ConstitutionGenesisRecord::from_manifest(&manifest).unwrap();
        record.validate_against_manifest(&manifest).unwrap();
    }

    #[test]
    fn writer_cannot_substitute_another_charter() {
        let manifest = manifest();
        let mut record = ConstitutionGenesisRecord::from_manifest(&manifest).unwrap();
        record.statement.charter = pd(99, "mycelix-charter-v1");
        assert!(record.validate_against_manifest(&manifest).is_err());
    }

    #[test]
    fn manifest_digest_prevents_cross_network_rebinding() {
        let manifest = manifest();
        let record = ConstitutionGenesisRecord::from_manifest(&manifest).unwrap();
        let mut other = manifest.clone();
        other.network_id = NetworkId::new("network:other").unwrap();
        assert!(record.validate_against_manifest(&other).is_err());
    }
}
