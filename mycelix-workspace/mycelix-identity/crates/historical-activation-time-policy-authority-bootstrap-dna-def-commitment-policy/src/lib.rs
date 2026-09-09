// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Exact Holochain `DnaHash` commitment for a prepared historical time-policy authority bootstrap
//! DNA definition.
//!
//! #444 owns exact construction of `DnaModifiers` from #431's opaque prepared constitutional
//! property. This theorem consumes those exact modifiers, replaces the modifiers of one
//! caller-supplied `DnaDef`, and computes the resulting Holochain `DnaHash`.
//!
//! The commitment boundary follows Holochain's own DNA-hash semantics: DNA name, modifiers, and
//! integrity-zome topology are identity-bearing; coordinator zomes are deliberately outside the
//! `DnaHash`. Success therefore does not authenticate or freeze coordinator code.
//!
//! Success means only: "these hash-relevant DNA-definition fields, including these exact
//! constitutional modifiers, produce this exact DNA identity". It does not establish bundle
//! validity, installability, installation, runtime provenance, deployment acceptance/currentness,
//! constitutional legitimacy, signer authority, policy currentness, or trusted time.

#![forbid(unsafe_code)]

use holo_hash::{DnaHash, HashableContentExtSync};
use holochain_zome_types::prelude::DnaDef;
use mycelix_historical_activation_time_policy_authority_bootstrap_dna_modifiers_policy::PreparedHistoricalActivationTimePolicyAuthorityBootstrapDnaModifiersV2;

#[derive(Debug)]
pub struct ExactHistoricalActivationTimePolicyAuthorityBootstrapDnaDefCommitmentV2 {
    dna_def: DnaDef,
    dna_hash: DnaHash,
}

impl ExactHistoricalActivationTimePolicyAuthorityBootstrapDnaDefCommitmentV2 {
    pub fn dna_name(&self) -> &str {
        &self.dna_def.name
    }

    pub fn network_seed(&self) -> &str {
        &self.dna_def.modifiers.network_seed
    }

    pub fn property_bytes(&self) -> &[u8] {
        self.dna_def.modifiers.properties.bytes()
    }

    pub fn dna_hash(&self) -> &DnaHash {
        &self.dna_hash
    }

    pub fn dna_hash_raw_39(&self) -> &[u8] {
        self.dna_hash.get_raw_39()
    }

    pub fn as_dna_def(&self) -> &DnaDef {
        &self.dna_def
    }

    pub fn into_dna_def(self) -> DnaDef {
        self.dna_def
    }
}

/// Replace a caller-supplied DNA definition's modifiers with the exact prepared constitutional
/// modifiers and compute the resulting Holochain DNA identity.
///
/// The caller remains responsible for the DNA name and zome topology. Holochain's hash semantics
/// bind integrity zomes but intentionally exclude coordinator zomes; this theorem preserves that
/// distinction rather than promoting the hash into a coordinator-code attestation.
pub fn commit_historical_activation_time_policy_authority_bootstrap_dna_def_v2(
    mut dna_def: DnaDef,
    modifiers: PreparedHistoricalActivationTimePolicyAuthorityBootstrapDnaModifiersV2,
) -> ExactHistoricalActivationTimePolicyAuthorityBootstrapDnaDefCommitmentV2 {
    dna_def.modifiers = modifiers.into_dna_modifiers();
    let dna_hash = dna_def.to_hash();
    ExactHistoricalActivationTimePolicyAuthorityBootstrapDnaDefCommitmentV2 { dna_def, dna_hash }
}

#[cfg(test)]
mod tests {
    use super::*;
    use holo_hash::WasmHash;
    use holochain_zome_types::prelude::{CoordinatorZomeDef, IntegrityZomeDef, ZomeName};
    use mycelix_crypto::AlgorithmId;
    use mycelix_historical_activation_time_policy_authority_bootstrap_declaration_policy::{
        canonicalize_historical_activation_time_policy_authority_bootstrap_declaration_v2,
        HistoricalActivationTimePolicyAuthorityBootstrapDeclarationV2,
        BOOTSTRAP_DECLARATION_FORMAT_VERSION_V2,
    };
    use mycelix_historical_activation_time_policy_authority_bootstrap_dna_modifiers_policy::prepare_historical_activation_time_policy_authority_bootstrap_dna_modifiers_v2;
    use mycelix_historical_activation_time_policy_authority_bootstrap_property_serialized_bytes_policy::prepare_historical_activation_time_policy_authority_bootstrap_properties_v2;
    use mycelix_historical_activation_time_policy_authority_bootstrap_subject_policy::HistoricalActivationTimePolicyAuthorityScopeV2;
    use sha2::{Digest, Sha256};

    fn full_scope() -> HistoricalActivationTimePolicyAuthorityScopeV2 {
        HistoricalActivationTimePolicyAuthorityScopeV2::from_permissions(true, true, true, true)
            .unwrap()
    }

    fn prepared_modifiers(
        network_seed: &str,
        key_byte: u8,
    ) -> PreparedHistoricalActivationTimePolicyAuthorityBootstrapDnaModifiersV2 {
        let public_key = [key_byte; 32];
        let declaration =
            canonicalize_historical_activation_time_policy_authority_bootstrap_declaration_v2(
                HistoricalActivationTimePolicyAuthorityBootstrapDeclarationV2 {
                    format_version: BOOTSTRAP_DECLARATION_FORMAT_VERSION_V2,
                    constitution_id: "mycelix-identity-v2-policy-authority",
                    constitution_version: 1,
                    policy_authority_id: "identity:policy-authority:bootstrap-v2",
                    policy_authority_key_id: "identity:policy-authority:bootstrap-v2#ed25519-1",
                    algorithm: AlgorithmId::Ed25519,
                    public_key_bytes: &public_key,
                    key_generation: 1,
                    scope: full_scope(),
                },
            )
            .unwrap();
        let properties =
            prepare_historical_activation_time_policy_authority_bootstrap_properties_v2(
                &declaration,
            )
            .unwrap();
        prepare_historical_activation_time_policy_authority_bootstrap_dna_modifiers_v2(
            network_seed.to_string(),
            properties,
        )
    }

    fn skeleton(name: &str) -> DnaDef {
        DnaDef {
            name: name.to_string(),
            modifiers: prepared_modifiers("placeholder-network", 0x55).into_dna_modifiers(),
            integrity_zomes: Vec::new(),
            coordinator_zomes: Vec::new(),
        }
    }

    fn commitment(
        name: &str,
        network_seed: &str,
        key_byte: u8,
    ) -> ExactHistoricalActivationTimePolicyAuthorityBootstrapDnaDefCommitmentV2 {
        commit_historical_activation_time_policy_authority_bootstrap_dna_def_v2(
            skeleton(name),
            prepared_modifiers(network_seed, key_byte),
        )
    }

    fn wasm_hash(byte: u8) -> WasmHash {
        WasmHash::from_raw_36(vec![byte; 36])
    }

    #[test]
    fn exact_modifiers_are_committed_into_dna_definition_identity() {
        let committed = commitment(
            "mycelix-identity-v2",
            "mycelix-identity-v2-prod-a",
            0x66,
        );

        assert_eq!(committed.dna_name(), "mycelix-identity-v2");
        assert_eq!(committed.network_seed(), "mycelix-identity-v2-prod-a");
        assert_eq!(committed.property_bytes().len(), 210);
        assert_eq!(committed.dna_hash_raw_39().len(), 39);
        assert_eq!(
            Sha256::digest(committed.property_bytes()).as_slice(),
            &[
                0xac, 0x5a, 0x2d, 0x63, 0xe6, 0xb9, 0xeb, 0x53, 0x0a, 0xe3, 0x2f, 0x7d,
                0xc2, 0x7d, 0x39, 0x79, 0xa1, 0xaf, 0x0f, 0x87, 0x52, 0x3f, 0xd5, 0x6b,
                0xc4, 0x15, 0xb6, 0xfb, 0x69, 0x12, 0x4b, 0xc7,
            ]
        );
    }

    #[test]
    fn same_hash_relevant_definition_is_hash_stable() {
        let first = commitment(
            "mycelix-identity-v2",
            "mycelix-identity-v2-prod-a",
            0x66,
        );
        let second = commitment(
            "mycelix-identity-v2",
            "mycelix-identity-v2-prod-a",
            0x66,
        );
        assert_eq!(first.dna_hash_raw_39(), second.dna_hash_raw_39());
    }

    #[test]
    fn network_seed_substitution_changes_dna_identity_without_rewriting_property() {
        let first = commitment("mycelix-identity-v2", "network-a", 0x66);
        let second = commitment("mycelix-identity-v2", "network-b", 0x66);
        assert_eq!(first.property_bytes(), second.property_bytes());
        assert_ne!(first.dna_hash_raw_39(), second.dna_hash_raw_39());
    }

    #[test]
    fn property_material_substitution_changes_dna_identity() {
        let first = commitment("mycelix-identity-v2", "network-a", 0x66);
        let second = commitment("mycelix-identity-v2", "network-a", 0x67);
        assert_ne!(first.property_bytes(), second.property_bytes());
        assert_ne!(first.dna_hash_raw_39(), second.dna_hash_raw_39());
    }

    #[test]
    fn dna_name_substitution_changes_dna_identity() {
        let first = commitment("mycelix-identity-v2", "network-a", 0x66);
        let second = commitment("mycelix-identity-v2-shadow", "network-a", 0x66);
        assert_eq!(first.property_bytes(), second.property_bytes());
        assert_ne!(first.dna_hash_raw_39(), second.dna_hash_raw_39());
    }

    #[test]
    fn coordinator_zome_substitution_is_outside_dna_hash_commitment() {
        let original = commitment("mycelix-identity-v2", "network-a", 0x66);
        let mut modified = original.as_dna_def().clone();
        modified.coordinator_zomes.push((
            ZomeName::from("coordinator-shadow"),
            CoordinatorZomeDef::from_hash(wasm_hash(0x31)),
        ));
        let modified_hash = modified.to_hash();

        assert_eq!(original.dna_hash_raw_39(), modified_hash.get_raw_39());
    }

    #[test]
    fn integrity_zome_substitution_changes_dna_hash_commitment() {
        let original = commitment("mycelix-identity-v2", "network-a", 0x66);
        let mut modified = original.as_dna_def().clone();
        modified.integrity_zomes.push((
            ZomeName::from("integrity-shadow"),
            IntegrityZomeDef::from_hash(wasm_hash(0x32)),
        ));
        let modified_hash = modified.to_hash();

        assert_ne!(original.dna_hash_raw_39(), modified_hash.get_raw_39());
    }

    #[test]
    fn commitment_wrapper_is_opaque_and_has_no_runtime_authority_shortcut() {
        let source = include_str!("lib.rs");
        let start = source
            .index("pub struct ExactHistoricalActivationTimePolicyAuthorityBootstrapDnaDefCommitmentV2")
            .unwrap();
        let end = source[start..]
            .index("impl ExactHistoricalActivationTimePolicyAuthorityBootstrapDnaDefCommitmentV2")
            .unwrap()
            + start;
        let body = &source[start..end];
        assert!(!body.contains("pub dna_def:"));
        assert!(!body.contains("pub dna_hash:"));

        let production = &source[..source.index("#[cfg(test)]").unwrap()];
        for forbidden in [
            "UnsafeBytes",
            "SerializedBytes::",
            "dna_info(",
            "DnaFile",
            "DnaBundle",
            "verify_hash(",
            "is_authorized",
            "from_trusted_configuration",
        ] {
            assert!(!production.contains(forbidden));
        }
    }
}
