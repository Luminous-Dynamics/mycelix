// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Canonical Identity V2 authority-domain identity theorem.
//!
//! This crate freezes the distinction between a logical authority epoch and one concrete
//! Holochain DNA execution identity. It is intentionally Holochain-free: callers supply
//! the raw 39-byte DNA hash identity, while a later HDK adapter must prove those bytes
//! came from the running cell's actual `DnaHash::get_raw_39()`.
//!
//! Success is identity evidence only. It does not establish that the DNA is accepted,
//! current, a valid successor, or that any DID/key/signature inside it is authoritative.

#![forbid(unsafe_code)]

use sha2::{Digest, Sha256};

pub const AUTHORITY_DOMAIN_ID_MAX_LEN_V2: usize = 128;
pub const HOLOCHAIN_DNA_HASH_RAW_LEN_V2: usize = 39;
pub const IDENTITY_AUTHORITY_DOMAIN_DIGEST_LEN_V2: usize = 32;
pub const IDENTITY_AUTHORITY_DOMAIN_DIGEST_DOMAIN_V2: &[u8] =
    b"mycelix:identity:authority-domain:v2\0";

#[derive(Debug, Clone, Copy)]
pub struct IdentityAuthorityDomainStatementV2<'a> {
    pub authority_domain_id: &'a str,
    pub authority_domain_epoch: u32,
    /// Exact full Holochain DNA hash bytes: type prefix + core hash + DHT location.
    /// This pure theorem validates shape only. Runtime provenance is a later HDK proof.
    pub dna_hash_raw_39: &'a [u8],
}

/// Opaque canonical authority-domain identity capability.
#[derive(Debug)]
pub struct QualifiedIdentityAuthorityDomainV2 {
    authority_domain_id: String,
    authority_domain_epoch: u32,
    dna_hash_raw_39: [u8; HOLOCHAIN_DNA_HASH_RAW_LEN_V2],
    digest_sha256: [u8; IDENTITY_AUTHORITY_DOMAIN_DIGEST_LEN_V2],
}

impl QualifiedIdentityAuthorityDomainV2 {
    pub fn authority_domain_id(&self) -> &str {
        &self.authority_domain_id
    }

    pub fn authority_domain_epoch(&self) -> u32 {
        self.authority_domain_epoch
    }

    pub fn dna_hash_raw_39(&self) -> &[u8; HOLOCHAIN_DNA_HASH_RAW_LEN_V2] {
        &self.dna_hash_raw_39
    }

    pub fn digest_sha256(&self) -> &[u8; IDENTITY_AUTHORITY_DOMAIN_DIGEST_LEN_V2] {
        &self.digest_sha256
    }

    pub fn same_exact_domain(&self, other: &Self) -> bool {
        self.authority_domain_id == other.authority_domain_id
            && self.authority_domain_epoch == other.authority_domain_epoch
            && self.dna_hash_raw_39 == other.dna_hash_raw_39
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IdentityAuthorityDomainErrorV2 {
    AuthorityDomainIdInvalid,
    AuthorityDomainEpochInvalid,
    DnaHashLengthInvalid,
    DnaHashAllZero,
}

fn valid_domain_id(value: &str) -> bool {
    if value.is_empty() || value.len() > AUTHORITY_DOMAIN_ID_MAX_LEN_V2 || !value.is_ascii() {
        return false;
    }
    let bytes = value.as_bytes();
    let is_alnum = |byte: u8| byte.is_ascii_lowercase() || byte.is_ascii_digit();
    if !is_alnum(bytes[0]) || !is_alnum(bytes[bytes.len() - 1]) {
        return false;
    }
    bytes.iter().all(|byte| {
        is_alnum(*byte) || matches!(*byte, b'.' | b'_' | b':' | b'-')
    })
}

fn validate_statement_v2(
    statement: IdentityAuthorityDomainStatementV2<'_>,
) -> Result<[u8; HOLOCHAIN_DNA_HASH_RAW_LEN_V2], IdentityAuthorityDomainErrorV2> {
    if !valid_domain_id(statement.authority_domain_id) {
        return Err(IdentityAuthorityDomainErrorV2::AuthorityDomainIdInvalid);
    }
    if statement.authority_domain_epoch == 0 {
        return Err(IdentityAuthorityDomainErrorV2::AuthorityDomainEpochInvalid);
    }
    let dna_hash: [u8; HOLOCHAIN_DNA_HASH_RAW_LEN_V2] = statement
        .dna_hash_raw_39
        .try_into()
        .map_err(|_| IdentityAuthorityDomainErrorV2::DnaHashLengthInvalid)?;
    if dna_hash.iter().all(|byte| *byte == 0) {
        return Err(IdentityAuthorityDomainErrorV2::DnaHashAllZero);
    }
    Ok(dna_hash)
}

/// Derive the canonical domain-separated digest for one logical authority epoch and one
/// concrete DNA execution identity.
pub fn derive_identity_authority_domain_digest_v2(
    statement: IdentityAuthorityDomainStatementV2<'_>,
) -> Result<[u8; IDENTITY_AUTHORITY_DOMAIN_DIGEST_LEN_V2], IdentityAuthorityDomainErrorV2> {
    let dna_hash = validate_statement_v2(statement)?;

    let mut hasher = Sha256::new();
    hasher.update(IDENTITY_AUTHORITY_DOMAIN_DIGEST_DOMAIN_V2);
    hasher.update((statement.authority_domain_id.len() as u16).to_be_bytes());
    hasher.update(statement.authority_domain_id.as_bytes());
    hasher.update(statement.authority_domain_epoch.to_be_bytes());
    hasher.update(dna_hash);
    Ok(hasher.finalize().into())
}

/// Qualify one exact Identity V2 authority-domain identity.
///
/// This does not prove runtime DNA provenance. The #356 HDK adapter must construct this
/// statement internally from the actual running `DnaHash` and never accept caller-supplied
/// DNA bytes as equivalent runtime evidence.
pub fn qualify_identity_authority_domain_v2(
    statement: IdentityAuthorityDomainStatementV2<'_>,
) -> Result<QualifiedIdentityAuthorityDomainV2, IdentityAuthorityDomainErrorV2> {
    let dna_hash_raw_39 = validate_statement_v2(statement)?;
    let digest_sha256 = derive_identity_authority_domain_digest_v2(statement)?;
    Ok(QualifiedIdentityAuthorityDomainV2 {
        authority_domain_id: statement.authority_domain_id.to_string(),
        authority_domain_epoch: statement.authority_domain_epoch,
        dna_hash_raw_39,
        digest_sha256,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    const DOMAIN_ID: &str = "mycelix-identity-v2";
    const DNA: [u8; 39] = [
        0x84, 0x2d, 0x24, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09,
        0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16,
        0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f, 0x01, 0x02, 0x03, 0x04,
    ];

    fn statement<'a>(dna: &'a [u8]) -> IdentityAuthorityDomainStatementV2<'a> {
        IdentityAuthorityDomainStatementV2 {
            authority_domain_id: DOMAIN_ID,
            authority_domain_epoch: 1,
            dna_hash_raw_39: dna,
        }
    }

    #[test]
    fn frozen_domain_digest_is_stable() {
        assert_eq!(
            derive_identity_authority_domain_digest_v2(statement(&DNA)).unwrap(),
            [
                0xf5, 0x04, 0xb4, 0x2c, 0xc8, 0x07, 0xe1, 0xc5, 0xd3, 0xd3, 0x78, 0x3a,
                0xd5, 0x41, 0xdb, 0x10, 0x36, 0x3b, 0x1c, 0x33, 0xab, 0xa5, 0xb1, 0xa6,
                0xbd, 0x30, 0xdc, 0x44, 0x62, 0xec, 0x1c, 0xd7,
            ]
        );
    }

    #[test]
    fn same_logical_id_under_different_dna_is_different_domain_identity() {
        let mut other_dna = DNA;
        other_dna[20] ^= 0x55;
        let first = qualify_identity_authority_domain_v2(statement(&DNA)).unwrap();
        let second = qualify_identity_authority_domain_v2(statement(&other_dna)).unwrap();
        assert!(!first.same_exact_domain(&second));
        assert_ne!(first.digest_sha256(), second.digest_sha256());
    }

    #[test]
    fn epoch_is_part_of_identity() {
        let first = qualify_identity_authority_domain_v2(statement(&DNA)).unwrap();
        let second = qualify_identity_authority_domain_v2(IdentityAuthorityDomainStatementV2 {
            authority_domain_id: DOMAIN_ID,
            authority_domain_epoch: 2,
            dna_hash_raw_39: &DNA,
        })
        .unwrap();
        assert!(!first.same_exact_domain(&second));
        assert_ne!(first.digest_sha256(), second.digest_sha256());
    }

    #[test]
    fn domain_id_is_ascii_canonical_and_bounded() {
        for invalid in [
            "",
            "Mycelix-Identity-V2",
            "mycelix identity v2",
            "-mycelix",
            "mycelix-",
            "mycélix",
        ] {
            assert_eq!(
                qualify_identity_authority_domain_v2(IdentityAuthorityDomainStatementV2 {
                    authority_domain_id: invalid,
                    authority_domain_epoch: 1,
                    dna_hash_raw_39: &DNA,
                })
                .unwrap_err(),
                IdentityAuthorityDomainErrorV2::AuthorityDomainIdInvalid
            );
        }
    }

    #[test]
    fn zero_epoch_and_invalid_dna_shape_fail_closed() {
        assert_eq!(
            qualify_identity_authority_domain_v2(IdentityAuthorityDomainStatementV2 {
                authority_domain_id: DOMAIN_ID,
                authority_domain_epoch: 0,
                dna_hash_raw_39: &DNA,
            })
            .unwrap_err(),
            IdentityAuthorityDomainErrorV2::AuthorityDomainEpochInvalid
        );
        assert_eq!(
            qualify_identity_authority_domain_v2(statement(&DNA[..38])).unwrap_err(),
            IdentityAuthorityDomainErrorV2::DnaHashLengthInvalid
        );
        let zeros = [0u8; 39];
        assert_eq!(
            qualify_identity_authority_domain_v2(statement(&zeros)).unwrap_err(),
            IdentityAuthorityDomainErrorV2::DnaHashAllZero
        );
    }

    #[test]
    fn qualified_domain_fields_are_verifier_owned() {
        let source = include_str!("lib.rs");
        let start = source
            .index("pub struct QualifiedIdentityAuthorityDomainV2")
            .unwrap();
        let end = source[start..]
            .index("impl QualifiedIdentityAuthorityDomainV2")
            .unwrap()
            + start;
        let body = &source[start..end];
        for field in [
            "pub authority_domain_id:",
            "pub authority_domain_epoch:",
            "pub dna_hash_raw_39:",
            "pub digest_sha256:",
        ] {
            assert!(!body.contains(field));
        }
    }
}
