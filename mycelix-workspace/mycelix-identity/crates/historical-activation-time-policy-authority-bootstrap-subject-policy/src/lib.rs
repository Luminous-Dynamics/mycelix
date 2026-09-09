// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Non-authoritative bootstrap-root subject for historical time-policy authority.
//!
//! This pure theorem binds one exact #403 policy-authority key generation to one explicit
//! transition/rotation scope. It creates a stable subject that a later independent
//! constitution/root-provenance theorem may authorize.
//!
//! Deriving this subject does not establish root legitimacy, currentness, signer authority,
//! transition authenticity, or accepted policy. There is no configuration-to-authority
//! constructor and no caller boolean that grants authority.

#![forbid(unsafe_code)]

use mycelix_historical_activation_time_policy_authority_key_generation_policy::ExactHistoricalActivationTimePolicyAuthorityKeyGenerationV2;
use sha2::{Digest, Sha256};

pub const SHA256_DIGEST_LEN_V2: usize = 32;
pub const HISTORICAL_ACTIVATION_TIME_POLICY_AUTHORITY_BOOTSTRAP_SUBJECT_DOMAIN_V2: &[u8] =
    b"mycelix:identity:historical-activation-time-policy-authority-bootstrap-subject:v2\0";

const SCOPE_ADOPT_V2: u8 = 1 << 0;
const SCOPE_SUPERSEDE_V2: u8 = 1 << 1;
const SCOPE_REVOKE_V2: u8 = 1 << 2;
const SCOPE_ROTATE_SIGNER_V2: u8 = 1 << 3;
const SCOPE_KNOWN_MASK_V2: u8 =
    SCOPE_ADOPT_V2 | SCOPE_SUPERSEDE_V2 | SCOPE_REVOKE_V2 | SCOPE_ROTATE_SIGNER_V2;

/// Exact capability scope named by a bootstrap-root subject.
///
/// The scope is not authority by itself. It is a bounded set of powers that a later
/// provenance theorem may authorize, and later signer rotation must only attenuate.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct HistoricalActivationTimePolicyAuthorityScopeV2 {
    mask: u8,
}

impl HistoricalActivationTimePolicyAuthorityScopeV2 {
    pub fn from_permissions(
        adopt: bool,
        supersede: bool,
        revoke: bool,
        rotate_signer: bool,
    ) -> Result<Self, HistoricalActivationTimePolicyAuthorityBootstrapSubjectErrorV2> {
        let mut mask = 0u8;
        if adopt {
            mask |= SCOPE_ADOPT_V2;
        }
        if supersede {
            mask |= SCOPE_SUPERSEDE_V2;
        }
        if revoke {
            mask |= SCOPE_REVOKE_V2;
        }
        if rotate_signer {
            mask |= SCOPE_ROTATE_SIGNER_V2;
        }
        if mask == 0 {
            return Err(
                HistoricalActivationTimePolicyAuthorityBootstrapSubjectErrorV2::EmptyScope,
            );
        }
        Ok(Self { mask })
    }

    pub const fn allows_adopt(self) -> bool {
        self.mask & SCOPE_ADOPT_V2 != 0
    }

    pub const fn allows_supersede(self) -> bool {
        self.mask & SCOPE_SUPERSEDE_V2 != 0
    }

    pub const fn allows_revoke(self) -> bool {
        self.mask & SCOPE_REVOKE_V2 != 0
    }

    pub const fn allows_signer_rotation(self) -> bool {
        self.mask & SCOPE_ROTATE_SIGNER_V2 != 0
    }

    pub const fn is_within(self, parent: Self) -> bool {
        (self.mask & !parent.mask) == 0
    }

    pub(crate) const fn canonical_mask(self) -> u8 {
        self.mask & SCOPE_KNOWN_MASK_V2
    }
}

/// Exact, non-authoritative subject naming one candidate bootstrap generation and scope.
#[derive(Debug)]
pub struct ExactHistoricalActivationTimePolicyAuthorityBootstrapSubjectV2 {
    authority_domain_sha256: [u8; SHA256_DIGEST_LEN_V2],
    bootstrap_generation_sha256: [u8; SHA256_DIGEST_LEN_V2],
    scope: HistoricalActivationTimePolicyAuthorityScopeV2,
    subject_digest_sha256: [u8; SHA256_DIGEST_LEN_V2],
}

impl ExactHistoricalActivationTimePolicyAuthorityBootstrapSubjectV2 {
    pub fn authority_domain_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V2] {
        &self.authority_domain_sha256
    }

    pub fn bootstrap_generation_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V2] {
        &self.bootstrap_generation_sha256
    }

    pub const fn scope(&self) -> HistoricalActivationTimePolicyAuthorityScopeV2 {
        self.scope
    }

    pub fn subject_digest_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V2] {
        &self.subject_digest_sha256
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HistoricalActivationTimePolicyAuthorityBootstrapSubjectErrorV2 {
    EmptyScope,
    BootstrapGenerationMustBeOne,
}

/// Derive one exact bootstrap-root subject from already-exact key material plus scope.
///
/// Generation 1 is required so this subject cannot silently re-label an arbitrary later
/// key generation as the beginning of the V2 authority lineage.
pub fn derive_exact_historical_activation_time_policy_authority_bootstrap_subject_v2(
    generation: &ExactHistoricalActivationTimePolicyAuthorityKeyGenerationV2,
    scope: HistoricalActivationTimePolicyAuthorityScopeV2,
) -> Result<
    ExactHistoricalActivationTimePolicyAuthorityBootstrapSubjectV2,
    HistoricalActivationTimePolicyAuthorityBootstrapSubjectErrorV2,
> {
    if generation.key_generation() != 1 {
        return Err(
            HistoricalActivationTimePolicyAuthorityBootstrapSubjectErrorV2::BootstrapGenerationMustBeOne,
        );
    }

    let mut hasher = Sha256::new();
    hasher.update(HISTORICAL_ACTIVATION_TIME_POLICY_AUTHORITY_BOOTSTRAP_SUBJECT_DOMAIN_V2);
    hasher.update([0x01]);
    hasher.update(generation.authority_domain_sha256());
    hasher.update([0x02]);
    hasher.update(generation.generation_digest_sha256());
    hasher.update([0x03]);
    hasher.update([scope.canonical_mask()]);
    let subject_digest_sha256 = hasher.finalize().into();

    Ok(ExactHistoricalActivationTimePolicyAuthorityBootstrapSubjectV2 {
        authority_domain_sha256: *generation.authority_domain_sha256(),
        bootstrap_generation_sha256: *generation.generation_digest_sha256(),
        scope,
        subject_digest_sha256,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_crypto::AlgorithmId;
    use mycelix_historical_activation_time_policy_authority_key_generation_policy::{
        derive_exact_historical_activation_time_policy_authority_key_generation_v2,
        HistoricalActivationTimePolicyAuthorityKeyGenerationV2,
    };
    use mycelix_identity_authority_domain_policy::{
        qualify_identity_authority_domain_v2, IdentityAuthorityDomainStatementV2,
        QualifiedIdentityAuthorityDomainV2,
    };

    static DNA: [u8; 39] = [
        0x84, 0x2d, 0x24, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09,
        0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16,
        0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f, 0x01, 0x02, 0x03, 0x04,
    ];
    static PUBLIC_KEY: [u8; 32] = [0x66; 32];

    fn domain() -> QualifiedIdentityAuthorityDomainV2 {
        qualify_identity_authority_domain_v2(IdentityAuthorityDomainStatementV2 {
            authority_domain_id: "mycelix-identity-v2",
            authority_domain_epoch: 1,
            dna_hash_raw_39: &DNA,
        })
        .unwrap()
    }

    fn generation(
        key_generation: u64,
    ) -> ExactHistoricalActivationTimePolicyAuthorityKeyGenerationV2 {
        derive_exact_historical_activation_time_policy_authority_key_generation_v2(
            &domain(),
            HistoricalActivationTimePolicyAuthorityKeyGenerationV2 {
                policy_authority_id: "identity:policy-authority:bootstrap-v2",
                policy_authority_key_id: "identity:policy-authority:bootstrap-v2#ed25519-1",
                algorithm: AlgorithmId::Ed25519,
                public_key_bytes: &PUBLIC_KEY,
                key_generation,
            },
        )
        .unwrap()
    }

    fn full_scope() -> HistoricalActivationTimePolicyAuthorityScopeV2 {
        HistoricalActivationTimePolicyAuthorityScopeV2::from_permissions(true, true, true, true)
            .unwrap()
    }

    #[test]
    fn frozen_full_scope_bootstrap_subject_digest_is_stable() {
        let generation = generation(1);
        let subject = derive_exact_historical_activation_time_policy_authority_bootstrap_subject_v2(
            &generation,
            full_scope(),
        )
        .unwrap();
        assert_eq!(
            subject.subject_digest_sha256(),
            &[
                0xeb, 0x88, 0xc1, 0x6f, 0x49, 0xea, 0x8e, 0xad, 0xa2, 0xd7, 0x77, 0xef,
                0x57, 0x78, 0x92, 0x56, 0x87, 0x44, 0xe1, 0xb5, 0x84, 0x4a, 0x6f, 0x67,
                0x25, 0xdb, 0x3d, 0x21, 0x39, 0x78, 0xb4, 0xa3,
            ]
        );
        assert_eq!(
            subject.bootstrap_generation_sha256(),
            generation.generation_digest_sha256()
        );
    }

    #[test]
    fn arbitrary_later_generation_cannot_be_relabelled_as_bootstrap() {
        assert_eq!(
            derive_exact_historical_activation_time_policy_authority_bootstrap_subject_v2(
                &generation(2),
                full_scope(),
            )
            .unwrap_err(),
            HistoricalActivationTimePolicyAuthorityBootstrapSubjectErrorV2::BootstrapGenerationMustBeOne
        );
    }

    #[test]
    fn empty_scope_fails_closed() {
        assert_eq!(
            HistoricalActivationTimePolicyAuthorityScopeV2::from_permissions(
                false, false, false, false,
            )
            .unwrap_err(),
            HistoricalActivationTimePolicyAuthorityBootstrapSubjectErrorV2::EmptyScope
        );
    }

    #[test]
    fn scope_attenuation_is_exact_subset_not_score() {
        let parent = full_scope();
        let restricted = HistoricalActivationTimePolicyAuthorityScopeV2::from_permissions(
            true, false, true, false,
        )
        .unwrap();
        let rotate_only = HistoricalActivationTimePolicyAuthorityScopeV2::from_permissions(
            false, false, false, true,
        )
        .unwrap();
        assert!(restricted.is_within(parent));
        assert!(rotate_only.is_within(parent));
        assert!(!parent.is_within(restricted));
        assert!(!rotate_only.is_within(restricted));
    }

    #[test]
    fn scope_change_changes_subject_identity() {
        let generation = generation(1);
        let full = derive_exact_historical_activation_time_policy_authority_bootstrap_subject_v2(
            &generation,
            full_scope(),
        )
        .unwrap();
        let restricted_scope = HistoricalActivationTimePolicyAuthorityScopeV2::from_permissions(
            true, false, true, false,
        )
        .unwrap();
        let restricted =
            derive_exact_historical_activation_time_policy_authority_bootstrap_subject_v2(
                &generation,
                restricted_scope,
            )
            .unwrap();
        assert_ne!(full.subject_digest_sha256(), restricted.subject_digest_sha256());
    }

    #[test]
    fn subject_fields_are_private_and_no_authority_constructor_exists() {
        let source = include_str!("lib.rs");
        let start = source
            .index("pub struct ExactHistoricalActivationTimePolicyAuthorityBootstrapSubjectV2")
            .unwrap();
        let end = source[start..]
            .index("impl ExactHistoricalActivationTimePolicyAuthorityBootstrapSubjectV2")
            .unwrap()
            + start;
        let body = &source[start..end];
        for field in [
            "pub authority_domain_sha256:",
            "pub bootstrap_generation_sha256:",
            "pub scope:",
            "pub subject_digest_sha256:",
        ] {
            assert!(!body.contains(field));
        }

        let production = &source[..source.index("#[cfg(test)]").unwrap()];
        let trusted_constructor = ["from", "trusted", "configuration"].join("_");
        let authorization_boolean = ["is", "authorized"].join("_");
        assert!(!production.contains(&trusted_constructor));
        assert!(!production.contains(&authorization_boolean));
    }
}
