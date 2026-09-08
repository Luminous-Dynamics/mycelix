// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Runtime binding for the pure #379 authority-scoped pre-crypto context.
//!
//! This layer does not acquire DNA identity itself. #382 is the sole owner of the
//! running-cell DNA host lookup. The only public qualification path accepts one opaque
//! #379 context, invokes #382 internally, requires exact authority-domain digest equality,
//! and returns an opaque runtime-bound pre-crypto capability.
//!
//! Success still does not establish exact Holochain Record provenance, observed DID/key
//! history, trusted activation, signature authenticity, policy currentness, or positive
//! evidence.

#![forbid(unsafe_code)]

use hdk::prelude::*;
use mycelix_authority_scoped_precrypto_context_policy::{
    QualifiedAuthorityScopedPrecryptoContextV2, SHA256_DIGEST_LEN_V2,
};
use mycelix_identity_runtime_authority_domain_adapter::{
    qualify_running_identity_authority_domain_v2,
};
use sha2::{Digest, Sha256};

pub const RUNTIME_BOUND_PRECRYPTO_CONTEXT_DOMAIN_V2: &[u8] =
    b"mycelix:identity:runtime-bound-precrypto-context:v2\0";

#[derive(Debug)]
pub struct RuntimeBoundAuthorityScopedPrecryptoContextV2 {
    authority_domain_digest_sha256: [u8; SHA256_DIGEST_LEN_V2],
    precrypto_context_digest_sha256: [u8; SHA256_DIGEST_LEN_V2],
    runtime_bound_context_digest_sha256: [u8; SHA256_DIGEST_LEN_V2],
}

impl RuntimeBoundAuthorityScopedPrecryptoContextV2 {
    pub fn authority_domain_digest_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V2] {
        &self.authority_domain_digest_sha256
    }

    pub fn precrypto_context_digest_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V2] {
        &self.precrypto_context_digest_sha256
    }

    pub fn runtime_bound_context_digest_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V2] {
        &self.runtime_bound_context_digest_sha256
    }
}

fn runtime_domain_matches_precrypto_context_v2(
    runtime_domain_digest_sha256: &[u8; SHA256_DIGEST_LEN_V2],
    precrypto_domain_digest_sha256: &[u8; SHA256_DIGEST_LEN_V2],
) -> bool {
    runtime_domain_digest_sha256 == precrypto_domain_digest_sha256
}

fn derive_runtime_bound_precrypto_context_digest_v2(
    authority_domain_digest_sha256: &[u8; SHA256_DIGEST_LEN_V2],
    precrypto_context_digest_sha256: &[u8; SHA256_DIGEST_LEN_V2],
) -> [u8; SHA256_DIGEST_LEN_V2] {
    let mut hasher = Sha256::new();
    hasher.update(RUNTIME_BOUND_PRECRYPTO_CONTEXT_DOMAIN_V2);
    hasher.update([0x01]);
    hasher.update(authority_domain_digest_sha256);
    hasher.update([0x02]);
    hasher.update(precrypto_context_digest_sha256);
    hasher.finalize().into()
}

/// Upgrade one pure #379 context with the executing cell's #382 runtime-domain proof.
///
/// The caller supplies no DNA identity and cannot substitute a runtime-domain capability.
/// The running-cell proof is acquired internally from #382 on every qualification.
pub fn qualify_running_authority_scoped_precrypto_context_v2(
    precrypto_context: &QualifiedAuthorityScopedPrecryptoContextV2,
) -> ExternResult<RuntimeBoundAuthorityScopedPrecryptoContextV2> {
    let runtime_domain = qualify_running_identity_authority_domain_v2()?;

    if !runtime_domain_matches_precrypto_context_v2(
        runtime_domain.authority_domain_digest_sha256(),
        precrypto_context.authority_domain_digest_sha256(),
    ) {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "running Identity DNA authority domain does not match the qualified pre-crypto context"
                .into()
        )));
    }

    let authority_domain_digest_sha256 = *runtime_domain.authority_domain_digest_sha256();
    let precrypto_context_digest_sha256 = *precrypto_context.precrypto_context_digest_sha256();
    let runtime_bound_context_digest_sha256 = derive_runtime_bound_precrypto_context_digest_v2(
        &authority_domain_digest_sha256,
        &precrypto_context_digest_sha256,
    );

    Ok(RuntimeBoundAuthorityScopedPrecryptoContextV2 {
        authority_domain_digest_sha256,
        precrypto_context_digest_sha256,
        runtime_bound_context_digest_sha256,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    const AUTHORITY_DOMAIN: [u8; SHA256_DIGEST_LEN_V2] = [
        0xf5, 0x04, 0xb4, 0x2c, 0xc8, 0x07, 0xe1, 0xc5, 0xd3, 0xd3, 0x78, 0x3a, 0xd5,
        0x41, 0xdb, 0x10, 0x36, 0x3b, 0x1c, 0x33, 0xab, 0xa5, 0xb1, 0xa6, 0xbd, 0x30,
        0xdc, 0x44, 0x62, 0xec, 0x1c, 0xd7,
    ];
    const PRECRYPTO_CONTEXT: [u8; SHA256_DIGEST_LEN_V2] = [
        0x8f, 0x5c, 0x42, 0x2f, 0x95, 0xde, 0xe5, 0xb6, 0x2b, 0xd5, 0x80, 0xba, 0x33,
        0xd1, 0xa2, 0xfb, 0xb8, 0x29, 0x16, 0x8f, 0xb2, 0x42, 0x2e, 0x1d, 0x54, 0xac,
        0xb5, 0x9a, 0x35, 0x09, 0x56, 0xad,
    ];
    const RUNTIME_BOUND_CONTEXT: [u8; SHA256_DIGEST_LEN_V2] = [
        0xb1, 0x0f, 0x10, 0x00, 0x51, 0xa5, 0x0a, 0xfa, 0xfd, 0xf6, 0x3b, 0x98, 0x27,
        0xda, 0x92, 0x3f, 0x88, 0xc5, 0xa9, 0x4d, 0x27, 0xfc, 0xb4, 0xfd, 0x1f, 0x06,
        0x92, 0x58, 0x12, 0x9d, 0x67, 0x6e,
    ];

    #[test]
    fn runtime_bound_context_digest_vector_is_frozen() {
        assert_eq!(
            derive_runtime_bound_precrypto_context_digest_v2(
                &AUTHORITY_DOMAIN,
                &PRECRYPTO_CONTEXT,
            ),
            RUNTIME_BOUND_CONTEXT
        );
    }

    #[test]
    fn runtime_domain_equality_fails_closed() {
        assert!(runtime_domain_matches_precrypto_context_v2(
            &AUTHORITY_DOMAIN,
            &AUTHORITY_DOMAIN,
        ));
        let mut other = AUTHORITY_DOMAIN;
        other[8] ^= 0x80;
        assert!(!runtime_domain_matches_precrypto_context_v2(
            &AUTHORITY_DOMAIN,
            &other,
        ));
    }

    #[test]
    fn runtime_bound_capability_fields_are_private() {
        let source = include_str!("lib.rs");
        let start = source
            .index("pub struct RuntimeBoundAuthorityScopedPrecryptoContextV2")
            .unwrap();
        let end = source[start..]
            .index("impl RuntimeBoundAuthorityScopedPrecryptoContextV2")
            .unwrap()
            + start;
        let body = &source[start..end];
        for field in [
            "pub authority_domain_digest_sha256:",
            "pub precrypto_context_digest_sha256:",
            "pub runtime_bound_context_digest_sha256:",
        ] {
            assert!(!body.contains(field));
        }
    }

    #[test]
    fn public_qualification_accepts_only_precrypto_context() {
        let source = include_str!("lib.rs");
        let start = source
            .index("pub fn qualify_running_authority_scoped_precrypto_context_v2(")
            .unwrap();
        let end = source[start..].index(") -> ExternResult").unwrap() + start;
        let signature = &source[start..end];
        assert!(signature.contains(
            "precrypto_context: &QualifiedAuthorityScopedPrecryptoContextV2"
        ));
        assert!(!signature.contains("DnaHash"));
        assert!(!signature.contains("RuntimeQualifiedIdentityAuthorityDomainV2"));
        assert!(!signature.contains("authority_domain_digest_sha256:"));
    }
}
