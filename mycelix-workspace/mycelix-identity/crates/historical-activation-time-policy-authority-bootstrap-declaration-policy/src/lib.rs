// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Domain-independent constitutional bootstrap declaration for historical time-policy authority.
//!
//! DNA properties participate in the effective Holochain DNA hash. The #403 key-generation
//! identity, by contrast, is scoped to the #369 authority-domain digest derived from that final
//! DNA hash. Therefore DNA properties must not embed #403/#407 domain-scoped digests: doing so
//! would make the DNA hash depend on a value that itself depends on the DNA hash.
//!
//! This theorem freezes only the domain-independent declaration that may be committed by DNA
//! properties. A later HDK adapter may read those committed properties, obtain the final running
//! DNA hash, and then derive #369 -> #403 -> #407. This crate establishes no domain acceptance,
//! bootstrap legitimacy, currentness, or signer authority.

#![forbid(unsafe_code)]

use mycelix_crypto::AlgorithmId;
use mycelix_historical_activation_time_policy_authority_bootstrap_subject_policy::HistoricalActivationTimePolicyAuthorityScopeV2;
use sha2::{Digest, Sha256};

pub const SHA256_DIGEST_LEN_V2: usize = 32;
pub const BOOTSTRAP_DECLARATION_FORMAT_VERSION_V2: u16 = 1;
pub const CONSTITUTION_ID_MAX_LEN_V2: usize = 128;
pub const POLICY_AUTHORITY_ID_MAX_LEN_V2: usize = 256;
pub const POLICY_AUTHORITY_KEY_ID_MAX_LEN_V2: usize = 512;
pub const HISTORICAL_ACTIVATION_TIME_POLICY_AUTHORITY_BOOTSTRAP_DECLARATION_DOMAIN_V2: &[u8] =
    b"mycelix:identity:historical-activation-time-policy-authority-bootstrap-declaration:v2\0";

#[derive(Debug, Clone, Copy)]
pub struct HistoricalActivationTimePolicyAuthorityBootstrapDeclarationV2<'a> {
    pub format_version: u16,
    pub constitution_id: &'a str,
    pub constitution_version: u32,
    pub policy_authority_id: &'a str,
    pub policy_authority_key_id: &'a str,
    pub algorithm: AlgorithmId,
    pub public_key_bytes: &'a [u8],
    pub key_generation: u64,
    pub scope: HistoricalActivationTimePolicyAuthorityScopeV2,
}

#[derive(Debug)]
pub struct CanonicalHistoricalActivationTimePolicyAuthorityBootstrapDeclarationV2 {
    constitution_id: String,
    constitution_version: u32,
    policy_authority_id: String,
    policy_authority_key_id: String,
    algorithm: AlgorithmId,
    public_key_bytes: Vec<u8>,
    key_generation: u64,
    scope: HistoricalActivationTimePolicyAuthorityScopeV2,
    declaration_digest_sha256: [u8; SHA256_DIGEST_LEN_V2],
}

impl CanonicalHistoricalActivationTimePolicyAuthorityBootstrapDeclarationV2 {
    pub fn constitution_id(&self) -> &str {
        &self.constitution_id
    }

    pub const fn constitution_version(&self) -> u32 {
        self.constitution_version
    }

    pub fn policy_authority_id(&self) -> &str {
        &self.policy_authority_id
    }

    pub fn policy_authority_key_id(&self) -> &str {
        &self.policy_authority_key_id
    }

    pub const fn algorithm(&self) -> AlgorithmId {
        self.algorithm
    }

    pub fn public_key_bytes(&self) -> &[u8] {
        &self.public_key_bytes
    }

    pub const fn key_generation(&self) -> u64 {
        self.key_generation
    }

    pub const fn scope(&self) -> HistoricalActivationTimePolicyAuthorityScopeV2 {
        self.scope
    }

    pub fn declaration_digest_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V2] {
        &self.declaration_digest_sha256
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HistoricalActivationTimePolicyAuthorityBootstrapDeclarationErrorV2 {
    UnsupportedFormatVersion,
    ConstitutionIdInvalid,
    ConstitutionVersionInvalid,
    PolicyAuthorityIdInvalid,
    PolicyAuthorityKeyIdInvalid,
    NonSignatureAlgorithm,
    PublicKeyLengthInvalid,
    PublicKeyAllZero,
    BootstrapGenerationMustBeOne,
}

fn valid_canonical_id(value: &str, max_len: usize) -> bool {
    if value.is_empty() || value.len() > max_len || !value.is_ascii() {
        return false;
    }
    let bytes = value.as_bytes();
    let is_alnum = |byte: u8| byte.is_ascii_lowercase() || byte.is_ascii_digit();
    if !is_alnum(bytes[0]) || !is_alnum(bytes[bytes.len() - 1]) {
        return false;
    }
    bytes
        .iter()
        .all(|byte| is_alnum(*byte) || matches!(*byte, b'.' | b'_' | b':' | b'-'))
}

fn valid_visible_ascii_identifier(value: &str, max_len: usize) -> bool {
    !value.is_empty()
        && value.len() <= max_len
        && value.is_ascii()
        && value
            .as_bytes()
            .iter()
            .all(|byte| (0x21..=0x7e).contains(byte))
}

fn canonical_scope_mask_v2(scope: HistoricalActivationTimePolicyAuthorityScopeV2) -> u8 {
    let mut mask = 0u8;
    if scope.allows_adopt() {
        mask |= 1 << 0;
    }
    if scope.allows_supersede() {
        mask |= 1 << 1;
    }
    if scope.allows_revoke() {
        mask |= 1 << 2;
    }
    if scope.allows_signer_rotation() {
        mask |= 1 << 3;
    }
    mask
}

fn update_len_prefixed_u16_v2(hasher: &mut Sha256, tag: u8, value: &[u8]) {
    hasher.update([tag]);
    hasher.update((value.len() as u16).to_be_bytes());
    hasher.update(value);
}

fn update_len_prefixed_u32_v2(hasher: &mut Sha256, tag: u8, value: &[u8]) {
    hasher.update([tag]);
    hasher.update((value.len() as u32).to_be_bytes());
    hasher.update(value);
}

pub fn canonicalize_historical_activation_time_policy_authority_bootstrap_declaration_v2(
    declaration: HistoricalActivationTimePolicyAuthorityBootstrapDeclarationV2<'_>,
) -> Result<
    CanonicalHistoricalActivationTimePolicyAuthorityBootstrapDeclarationV2,
    HistoricalActivationTimePolicyAuthorityBootstrapDeclarationErrorV2,
> {
    if declaration.format_version != BOOTSTRAP_DECLARATION_FORMAT_VERSION_V2 {
        return Err(
            HistoricalActivationTimePolicyAuthorityBootstrapDeclarationErrorV2::UnsupportedFormatVersion,
        );
    }
    if !valid_canonical_id(declaration.constitution_id, CONSTITUTION_ID_MAX_LEN_V2) {
        return Err(
            HistoricalActivationTimePolicyAuthorityBootstrapDeclarationErrorV2::ConstitutionIdInvalid,
        );
    }
    if declaration.constitution_version == 0 {
        return Err(
            HistoricalActivationTimePolicyAuthorityBootstrapDeclarationErrorV2::ConstitutionVersionInvalid,
        );
    }
    if !valid_visible_ascii_identifier(
        declaration.policy_authority_id,
        POLICY_AUTHORITY_ID_MAX_LEN_V2,
    ) {
        return Err(
            HistoricalActivationTimePolicyAuthorityBootstrapDeclarationErrorV2::PolicyAuthorityIdInvalid,
        );
    }
    if !valid_visible_ascii_identifier(
        declaration.policy_authority_key_id,
        POLICY_AUTHORITY_KEY_ID_MAX_LEN_V2,
    ) {
        return Err(
            HistoricalActivationTimePolicyAuthorityBootstrapDeclarationErrorV2::PolicyAuthorityKeyIdInvalid,
        );
    }
    if !declaration.algorithm.is_signature_algorithm() {
        return Err(
            HistoricalActivationTimePolicyAuthorityBootstrapDeclarationErrorV2::NonSignatureAlgorithm,
        );
    }
    if declaration.public_key_bytes.len() != declaration.algorithm.public_key_size() {
        return Err(
            HistoricalActivationTimePolicyAuthorityBootstrapDeclarationErrorV2::PublicKeyLengthInvalid,
        );
    }
    if declaration.public_key_bytes.iter().all(|byte| *byte == 0) {
        return Err(
            HistoricalActivationTimePolicyAuthorityBootstrapDeclarationErrorV2::PublicKeyAllZero,
        );
    }
    if declaration.key_generation != 1 {
        return Err(
            HistoricalActivationTimePolicyAuthorityBootstrapDeclarationErrorV2::BootstrapGenerationMustBeOne,
        );
    }

    let mut hasher = Sha256::new();
    hasher.update(HISTORICAL_ACTIVATION_TIME_POLICY_AUTHORITY_BOOTSTRAP_DECLARATION_DOMAIN_V2);
    hasher.update([0x01]);
    hasher.update(declaration.format_version.to_be_bytes());
    update_len_prefixed_u16_v2(&mut hasher, 0x02, declaration.constitution_id.as_bytes());
    hasher.update([0x03]);
    hasher.update(declaration.constitution_version.to_be_bytes());
    update_len_prefixed_u16_v2(&mut hasher, 0x04, declaration.policy_authority_id.as_bytes());
    update_len_prefixed_u16_v2(
        &mut hasher,
        0x05,
        declaration.policy_authority_key_id.as_bytes(),
    );
    hasher.update([0x06]);
    hasher.update(declaration.algorithm.as_u16().to_be_bytes());
    update_len_prefixed_u32_v2(&mut hasher, 0x07, declaration.public_key_bytes);
    hasher.update([0x08]);
    hasher.update(declaration.key_generation.to_be_bytes());
    hasher.update([0x09]);
    hasher.update([canonical_scope_mask_v2(declaration.scope)]);
    let declaration_digest_sha256 = hasher.finalize().into();

    Ok(CanonicalHistoricalActivationTimePolicyAuthorityBootstrapDeclarationV2 {
        constitution_id: declaration.constitution_id.to_string(),
        constitution_version: declaration.constitution_version,
        policy_authority_id: declaration.policy_authority_id.to_string(),
        policy_authority_key_id: declaration.policy_authority_key_id.to_string(),
        algorithm: declaration.algorithm,
        public_key_bytes: declaration.public_key_bytes.to_vec(),
        key_generation: declaration.key_generation,
        scope: declaration.scope,
        declaration_digest_sha256,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    static PUBLIC_KEY: [u8; 32] = [0x66; 32];

    fn full_scope() -> HistoricalActivationTimePolicyAuthorityScopeV2 {
        HistoricalActivationTimePolicyAuthorityScopeV2::from_permissions(true, true, true, true)
            .unwrap()
    }

    fn declaration<'a>(
        public_key_bytes: &'a [u8],
        scope: HistoricalActivationTimePolicyAuthorityScopeV2,
    ) -> HistoricalActivationTimePolicyAuthorityBootstrapDeclarationV2<'a> {
        HistoricalActivationTimePolicyAuthorityBootstrapDeclarationV2 {
            format_version: BOOTSTRAP_DECLARATION_FORMAT_VERSION_V2,
            constitution_id: "mycelix-identity-v2-policy-authority",
            constitution_version: 1,
            policy_authority_id: "identity:policy-authority:bootstrap-v2",
            policy_authority_key_id: "identity:policy-authority:bootstrap-v2#ed25519-1",
            algorithm: AlgorithmId::Ed25519,
            public_key_bytes,
            key_generation: 1,
            scope,
        }
    }

    #[test]
    fn frozen_full_scope_declaration_digest_is_stable() {
        let canonical =
            canonicalize_historical_activation_time_policy_authority_bootstrap_declaration_v2(
                declaration(&PUBLIC_KEY, full_scope()),
            )
            .unwrap();
        assert_eq!(
            canonical.declaration_digest_sha256(),
            &[
                0xee, 0x70, 0xfb, 0x06, 0xfe, 0x2a, 0xb1, 0x5c, 0xa4, 0x4f, 0x3b, 0xc3,
                0x68, 0xc7, 0x39, 0x9c, 0x20, 0x33, 0xae, 0x66, 0x22, 0x65, 0x82, 0xa3,
                0xed, 0x01, 0x2c, 0x63, 0x7f, 0x01, 0x25, 0x04,
            ]
        );
    }

    #[test]
    fn unsupported_format_fails_before_semantic_fields() {
        let invalid = HistoricalActivationTimePolicyAuthorityBootstrapDeclarationV2 {
            format_version: BOOTSTRAP_DECLARATION_FORMAT_VERSION_V2 + 1,
            constitution_id: "INVALID UPPERCASE AND SPACE",
            constitution_version: 0,
            policy_authority_id: "",
            policy_authority_key_id: "",
            algorithm: AlgorithmId::MlKem768,
            public_key_bytes: &[],
            key_generation: 99,
            scope: full_scope(),
        };
        assert_eq!(
            canonicalize_historical_activation_time_policy_authority_bootstrap_declaration_v2(
                invalid,
            )
            .unwrap_err(),
            HistoricalActivationTimePolicyAuthorityBootstrapDeclarationErrorV2::UnsupportedFormatVersion
        );
    }

    #[test]
    fn public_key_substitution_changes_declaration_identity() {
        let first =
            canonicalize_historical_activation_time_policy_authority_bootstrap_declaration_v2(
                declaration(&PUBLIC_KEY, full_scope()),
            )
            .unwrap();
        let mut changed_key = PUBLIC_KEY;
        changed_key[0] ^= 1;
        let second =
            canonicalize_historical_activation_time_policy_authority_bootstrap_declaration_v2(
                declaration(&changed_key, full_scope()),
            )
            .unwrap();
        assert_ne!(
            first.declaration_digest_sha256(),
            second.declaration_digest_sha256()
        );
    }

    #[test]
    fn scope_change_changes_declaration_identity() {
        let full =
            canonicalize_historical_activation_time_policy_authority_bootstrap_declaration_v2(
                declaration(&PUBLIC_KEY, full_scope()),
            )
            .unwrap();
        let restricted_scope = HistoricalActivationTimePolicyAuthorityScopeV2::from_permissions(
            true, false, true, false,
        )
        .unwrap();
        let restricted =
            canonicalize_historical_activation_time_policy_authority_bootstrap_declaration_v2(
                declaration(&PUBLIC_KEY, restricted_scope),
            )
            .unwrap();
        assert_ne!(
            full.declaration_digest_sha256(),
            restricted.declaration_digest_sha256()
        );
    }

    #[test]
    fn malformed_key_and_nonroot_generation_fail_closed() {
        let short_key = [0x66; 31];
        assert_eq!(
            canonicalize_historical_activation_time_policy_authority_bootstrap_declaration_v2(
                declaration(&short_key, full_scope()),
            )
            .unwrap_err(),
            HistoricalActivationTimePolicyAuthorityBootstrapDeclarationErrorV2::PublicKeyLengthInvalid
        );

        let zero_key = [0u8; 32];
        assert_eq!(
            canonicalize_historical_activation_time_policy_authority_bootstrap_declaration_v2(
                declaration(&zero_key, full_scope()),
            )
            .unwrap_err(),
            HistoricalActivationTimePolicyAuthorityBootstrapDeclarationErrorV2::PublicKeyAllZero
        );

        let nonroot = HistoricalActivationTimePolicyAuthorityBootstrapDeclarationV2 {
            key_generation: 2,
            ..declaration(&PUBLIC_KEY, full_scope())
        };
        assert_eq!(
            canonicalize_historical_activation_time_policy_authority_bootstrap_declaration_v2(
                nonroot,
            )
            .unwrap_err(),
            HistoricalActivationTimePolicyAuthorityBootstrapDeclarationErrorV2::BootstrapGenerationMustBeOne
        );
    }

    #[test]
    fn constitution_identifier_is_canonical() {
        for invalid in ["", "Mycelix-Identity", "mycelix identity", "-mycelix", "mycelix-"] {
            let invalid_declaration =
                HistoricalActivationTimePolicyAuthorityBootstrapDeclarationV2 {
                    constitution_id: invalid,
                    ..declaration(&PUBLIC_KEY, full_scope())
                };
            assert_eq!(
                canonicalize_historical_activation_time_policy_authority_bootstrap_declaration_v2(
                    invalid_declaration,
                )
                .unwrap_err(),
                HistoricalActivationTimePolicyAuthorityBootstrapDeclarationErrorV2::ConstitutionIdInvalid
            );
        }
    }

    #[test]
    fn canonical_declaration_is_domain_independent_and_fields_are_private() {
        let source = include_str!("lib.rs");
        let production = &source[..source.index("#[cfg(test)]").unwrap()];
        assert!(!production.contains("QualifiedIdentityAuthorityDomain"));
        assert!(!production.contains("authority_domain_sha256"));
        assert!(!production.contains("ExactHistoricalActivationTimePolicyAuthorityKeyGenerationV2"));
        assert!(!production.contains("ExactHistoricalActivationTimePolicyAuthorityBootstrapSubjectV2"));

        let start = source
            .index("pub struct CanonicalHistoricalActivationTimePolicyAuthorityBootstrapDeclarationV2")
            .unwrap();
        let end = source[start..]
            .index("impl CanonicalHistoricalActivationTimePolicyAuthorityBootstrapDeclarationV2")
            .unwrap()
            + start;
        let body = &source[start..end];
        for field in [
            "pub public_key_bytes:",
            "pub scope:",
            "pub declaration_digest_sha256:",
        ] {
            assert!(!body.contains(field));
        }
    }
}
