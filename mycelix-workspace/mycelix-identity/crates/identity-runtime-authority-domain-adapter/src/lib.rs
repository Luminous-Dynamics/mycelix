// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! One-way runtime provenance adapter for the Identity V2 authority domain.
//!
//! This is the first HDK-bearing layer in the authority-domain chain. It obtains the
//! running cell's effective DNA hash from `dna_info()` internally and binds those exact
//! bytes to the canonical #369 authority-domain theorem. No caller can supply replacement
//! DNA bytes, a network seed, a domain digest, or an already-qualified pure domain.
//!
//! Success means only: "this authority-domain digest was derived from this running
//! cell's effective DNA hash under the baked Identity V2 logical domain/epoch".
//! It does not establish domain acceptance/currentness, record provenance, DID/key
//! history, trusted activation, signature authenticity, policy authority, or positive
//! evidence.

#![forbid(unsafe_code)]

use hdk::prelude::*;
use mycelix_identity_authority_domain_policy::{
    qualify_identity_authority_domain_v2, IdentityAuthorityDomainStatementV2,
    IDENTITY_AUTHORITY_DOMAIN_DIGEST_LEN_V2,
};

pub const RUNTIME_IDENTITY_AUTHORITY_DOMAIN_ID_V2: &str = "mycelix-identity-v2";
pub const RUNTIME_IDENTITY_AUTHORITY_DOMAIN_EPOCH_V2: u32 = 1;

/// Opaque proof that one #369 authority-domain digest was derived from the effective
/// DNA hash of the cell currently executing this adapter.
///
/// The underlying raw DNA bytes are deliberately not retained or exposed. Downstream
/// code receives only the canonical #369 digest needed for equality with pure contexts.
#[derive(Debug)]
pub struct RuntimeQualifiedIdentityAuthorityDomainV2 {
    authority_domain_digest_sha256: [u8; IDENTITY_AUTHORITY_DOMAIN_DIGEST_LEN_V2],
}

impl RuntimeQualifiedIdentityAuthorityDomainV2 {
    pub fn authority_domain_digest_sha256(
        &self,
    ) -> &[u8; IDENTITY_AUTHORITY_DOMAIN_DIGEST_LEN_V2] {
        &self.authority_domain_digest_sha256
    }
}

/// Bind the running cell's actual effective DNA hash to the canonical Identity V2
/// logical authority-domain identity.
///
/// There is intentionally no input parameter. The only concrete DNA identity admitted
/// here comes from Holochain's `dna_info()` host call for the executing cell.
pub fn qualify_running_identity_authority_domain_v2(
) -> ExternResult<RuntimeQualifiedIdentityAuthorityDomainV2> {
    let info = dna_info()?;
    let qualified = qualify_identity_authority_domain_v2(IdentityAuthorityDomainStatementV2 {
        authority_domain_id: RUNTIME_IDENTITY_AUTHORITY_DOMAIN_ID_V2,
        authority_domain_epoch: RUNTIME_IDENTITY_AUTHORITY_DOMAIN_EPOCH_V2,
        dna_hash_raw_39: info.hash.get_raw_39(),
    })
    .map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "running Identity V2 authority-domain qualification failed: {error:?}"
        )))
    })?;

    Ok(RuntimeQualifiedIdentityAuthorityDomainV2 {
        authority_domain_digest_sha256: *qualified.digest_sha256(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn deployment_identity_is_explicit_and_versioned() {
        assert_eq!(
            RUNTIME_IDENTITY_AUTHORITY_DOMAIN_ID_V2,
            "mycelix-identity-v2"
        );
        assert_eq!(RUNTIME_IDENTITY_AUTHORITY_DOMAIN_EPOCH_V2, 1);
    }

    #[test]
    fn runtime_capability_does_not_expose_raw_dna_or_public_fields() {
        let source = include_str!("lib.rs");
        let start = source
            .index("pub struct RuntimeQualifiedIdentityAuthorityDomainV2")
            .unwrap();
        let end = source[start..]
            .index("impl RuntimeQualifiedIdentityAuthorityDomainV2")
            .unwrap()
            + start;
        let body = &source[start..end];
        assert!(!body.contains("pub authority_domain_digest_sha256:"));

        let impl_start = end;
        let function_start = source[impl_start..]
            .index("pub fn qualify_running_identity_authority_domain_v2")
            .unwrap()
            + impl_start;
        let impl_body = &source[impl_start..function_start];
        assert!(!impl_body.contains("dna_hash_raw_39"));
        assert!(!impl_body.contains("qualified_domain"));
    }

    #[test]
    fn runtime_qualification_has_no_caller_input() {
        let source = include_str!("lib.rs");
        assert!(source.contains(
            "pub fn qualify_running_identity_authority_domain_v2(\n) -> ExternResult<RuntimeQualifiedIdentityAuthorityDomainV2>"
        ));
        assert!(source.contains("let info = dna_info()?;"));
        assert!(source.contains("dna_hash_raw_39: info.hash.get_raw_39(),"));
    }
}
