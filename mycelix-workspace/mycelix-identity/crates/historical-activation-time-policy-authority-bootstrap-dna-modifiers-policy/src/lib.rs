// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Exact `DnaModifiers` construction for the historical time-policy authority bootstrap property.
//!
//! #431 owns the only permitted custom-byte property conversion. This theorem consumes that
//! opaque prepared value and places it directly into Holochain's `DnaModifiers.properties` field
//! alongside the caller-supplied network seed. It does not re-encode, deserialize, or reinterpret
//! the constitutional property bytes.
//!
//! Success means only: "these exact prepared property bytes occupy this exact modifier property
//! slot with this exact network seed". It does not establish a DNA hash, a bundled or installed
//! DNA, deployment acceptance/currentness, root legitimacy, signer authority, policy currentness,
//! or trusted time.

#![forbid(unsafe_code)]

use holochain_zome_types::prelude::DnaModifiers;
use mycelix_historical_activation_time_policy_authority_bootstrap_property_serialized_bytes_policy::PreparedHistoricalActivationTimePolicyAuthorityBootstrapPropertiesV2;

#[derive(Debug)]
pub struct PreparedHistoricalActivationTimePolicyAuthorityBootstrapDnaModifiersV2 {
    modifiers: DnaModifiers,
}

impl PreparedHistoricalActivationTimePolicyAuthorityBootstrapDnaModifiersV2 {
    pub fn network_seed(&self) -> &str {
        &self.modifiers.network_seed
    }

    pub fn property_bytes(&self) -> &[u8] {
        self.modifiers.properties.bytes()
    }

    pub fn as_dna_modifiers(&self) -> &DnaModifiers {
        &self.modifiers
    }

    pub fn into_dna_modifiers(self) -> DnaModifiers {
        self.modifiers
    }
}

pub fn prepare_historical_activation_time_policy_authority_bootstrap_dna_modifiers_v2(
    network_seed: String,
    properties: PreparedHistoricalActivationTimePolicyAuthorityBootstrapPropertiesV2,
) -> PreparedHistoricalActivationTimePolicyAuthorityBootstrapDnaModifiersV2 {
    PreparedHistoricalActivationTimePolicyAuthorityBootstrapDnaModifiersV2 {
        modifiers: DnaModifiers {
            network_seed,
            properties: properties.into_serialized_bytes(),
        },
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_crypto::AlgorithmId;
    use mycelix_historical_activation_time_policy_authority_bootstrap_declaration_policy::{
        canonicalize_historical_activation_time_policy_authority_bootstrap_declaration_v2,
        HistoricalActivationTimePolicyAuthorityBootstrapDeclarationV2,
        BOOTSTRAP_DECLARATION_FORMAT_VERSION_V2,
    };
    use mycelix_historical_activation_time_policy_authority_bootstrap_property_serialized_bytes_policy::prepare_historical_activation_time_policy_authority_bootstrap_properties_v2;
    use mycelix_historical_activation_time_policy_authority_bootstrap_subject_policy::HistoricalActivationTimePolicyAuthorityScopeV2;
    use sha2::{Digest, Sha256};

    static PUBLIC_KEY: [u8; 32] = [0x66; 32];

    fn full_scope() -> HistoricalActivationTimePolicyAuthorityScopeV2 {
        HistoricalActivationTimePolicyAuthorityScopeV2::from_permissions(true, true, true, true)
            .unwrap()
    }

    fn prepared_properties() -> PreparedHistoricalActivationTimePolicyAuthorityBootstrapPropertiesV2 {
        let declaration =
            canonicalize_historical_activation_time_policy_authority_bootstrap_declaration_v2(
                HistoricalActivationTimePolicyAuthorityBootstrapDeclarationV2 {
                    format_version: BOOTSTRAP_DECLARATION_FORMAT_VERSION_V2,
                    constitution_id: "mycelix-identity-v2-policy-authority",
                    constitution_version: 1,
                    policy_authority_id: "identity:policy-authority:bootstrap-v2",
                    policy_authority_key_id: "identity:policy-authority:bootstrap-v2#ed25519-1",
                    algorithm: AlgorithmId::Ed25519,
                    public_key_bytes: &PUBLIC_KEY,
                    key_generation: 1,
                    scope: full_scope(),
                },
            )
            .unwrap();
        prepare_historical_activation_time_policy_authority_bootstrap_properties_v2(&declaration)
            .unwrap()
    }

    #[test]
    fn exact_prepared_property_bytes_enter_dna_modifiers_unchanged() {
        let prepared = prepare_historical_activation_time_policy_authority_bootstrap_dna_modifiers_v2(
            "mycelix-identity-v2-prod-a".to_string(),
            prepared_properties(),
        );

        assert_eq!(prepared.network_seed(), "mycelix-identity-v2-prod-a");
        assert_eq!(prepared.property_bytes().len(), 210);
        assert_eq!(
            Sha256::digest(prepared.property_bytes()).as_slice(),
            &[
                0xac, 0x5a, 0x2d, 0x63, 0xe6, 0xb9, 0xeb, 0x53, 0x0a, 0xe3, 0x2f, 0x7d,
                0xc2, 0x7d, 0x39, 0x79, 0xa1, 0xaf, 0x0f, 0x87, 0x52, 0x3f, 0xd5, 0x6b,
                0xc4, 0x15, 0xb6, 0xfb, 0x69, 0x12, 0x4b, 0xc7,
            ]
        );
        assert_eq!(
            prepared.as_dna_modifiers().properties.bytes(),
            prepared.property_bytes()
        );
    }

    #[test]
    fn network_seed_is_preserved_without_rewriting_property_bytes() {
        let first = prepare_historical_activation_time_policy_authority_bootstrap_dna_modifiers_v2(
            "network-a".to_string(),
            prepared_properties(),
        );
        let second = prepare_historical_activation_time_policy_authority_bootstrap_dna_modifiers_v2(
            "network-b".to_string(),
            prepared_properties(),
        );

        assert_ne!(first.network_seed(), second.network_seed());
        assert_eq!(first.property_bytes(), second.property_bytes());
    }

    #[test]
    fn modifier_wrapper_is_opaque_and_cannot_reconstruct_properties() {
        let source = include_str!("lib.rs");
        let start = source
            .index("pub struct PreparedHistoricalActivationTimePolicyAuthorityBootstrapDnaModifiersV2")
            .unwrap();
        let end = source[start..]
            .index("impl PreparedHistoricalActivationTimePolicyAuthorityBootstrapDnaModifiersV2")
            .unwrap()
            + start;
        assert!(!source[start..end].contains("pub modifiers:"));

        let production = &source[..source.index("#[cfg(test)]").unwrap()];
        assert!(!production.contains("SerializedBytes"));
        assert!(!production.contains("UnsafeBytes"));
        assert!(!production.contains("encode_historical_activation_time_policy_authority_bootstrap_property_v2"));
        assert!(!production.contains("decode_historical_activation_time_policy_authority_bootstrap_property_v2"));
        assert!(!production.contains("dna_info("));
        assert!(!production.contains("DnaHash"));
    }
}
