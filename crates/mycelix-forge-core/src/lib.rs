// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Portable, forge-independent primitives for Mycelix Forge.
//!
//! This crate intentionally has no dependency on Holochain, GitHub, Radicle,
//! gittuf, Xenia, Spore, or SLSA. It defines protocol subjects that adapters
//! may bind to those systems without making any of them a root of trust.

use serde::{de::Error as _, Deserialize, Deserializer, Serialize};
use sha2::{Digest as _, Sha256};
use std::{fmt, str::FromStr};
use thiserror::Error;

/// Domain separator for the v1 project-identity derivation.
const PROJECT_ID_DOMAIN_V1: &[u8] = b"mycelix-forge/project-identity/v1\0";

/// Size, in bytes, of the externally supplied project genesis nonce.
pub const GENESIS_NONCE_LEN: usize = 32;

/// Current Mycelix Forge core protocol version.
pub const CURRENT_PROTOCOL_VERSION: u16 = 1;

/// Errors produced by Forge core validation and canonicalization.
#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum ForgeCoreError {
    /// The caller named a digest algorithm this implementation does not support.
    #[error("unsupported digest algorithm: {0}")]
    UnsupportedDigestAlgorithm(String),

    /// A digest had the wrong byte length for its declared algorithm.
    #[error(
        "invalid digest length for {algorithm}: expected {expected} bytes, got {actual}"
    )]
    InvalidDigestLength {
        algorithm: DigestAlgorithm,
        expected: usize,
        actual: usize,
    },

    /// The protocol version is not understood by this implementation.
    #[error("unsupported Forge protocol version: {0}")]
    UnsupportedProtocolVersion(u16),

    /// A length-prefixed canonical field exceeded the v1 encoding bound.
    #[error("canonical field {field} is too large: {len} bytes exceeds {max}")]
    CanonicalFieldTooLarge {
        field: &'static str,
        len: usize,
        max: usize,
    },
}

/// A protocol version that has been validated as supported by this crate.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize)]
#[serde(transparent)]
pub struct ProtocolVersion(u16);

impl ProtocolVersion {
    /// The currently supported protocol version.
    pub const CURRENT: Self = Self(CURRENT_PROTOCOL_VERSION);

    /// Validate and construct a supported protocol version.
    pub fn new(value: u16) -> Result<Self, ForgeCoreError> {
        match value {
            CURRENT_PROTOCOL_VERSION => Ok(Self(value)),
            other => Err(ForgeCoreError::UnsupportedProtocolVersion(other)),
        }
    }

    /// Numeric version value.
    pub const fn get(self) -> u16 {
        self.0
    }
}

impl<'de> Deserialize<'de> for ProtocolVersion {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        let value = u16::deserialize(deserializer)?;
        Self::new(value).map_err(D::Error::custom)
    }
}

/// Digest suites supported by Forge protocol v1.
///
/// The algorithm identifier is part of the identity. Adding a future digest
/// suite is therefore an explicit protocol extension rather than an implicit
/// reinterpretation of existing bytes.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub enum DigestAlgorithm {
    #[serde(rename = "sha256")]
    Sha256,
    #[serde(rename = "blake3-256")]
    Blake3_256,
}

impl DigestAlgorithm {
    /// Stable protocol identifier for this digest suite.
    pub const fn id(self) -> &'static str {
        match self {
            Self::Sha256 => "sha256",
            Self::Blake3_256 => "blake3-256",
        }
    }

    /// Digest length in bytes for this protocol version.
    pub const fn digest_len(self) -> usize {
        match self {
            Self::Sha256 | Self::Blake3_256 => 32,
        }
    }

    fn hash(self, input: &[u8]) -> Digest {
        let bytes = match self {
            Self::Sha256 => Sha256::digest(input).to_vec(),
            Self::Blake3_256 => blake3::hash(input).as_bytes().to_vec(),
        };

        // All internally produced values have a statically known valid size.
        Digest {
            algorithm: self,
            bytes,
        }
    }
}

impl fmt::Display for DigestAlgorithm {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.id())
    }
}

impl FromStr for DigestAlgorithm {
    type Err = ForgeCoreError;

    fn from_str(value: &str) -> Result<Self, Self::Err> {
        match value {
            "sha256" => Ok(Self::Sha256),
            "blake3-256" => Ok(Self::Blake3_256),
            other => Err(ForgeCoreError::UnsupportedDigestAlgorithm(other.to_owned())),
        }
    }
}

/// An algorithm-qualified, validated digest.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize)]
pub struct Digest {
    algorithm: DigestAlgorithm,
    bytes: Vec<u8>,
}

impl Digest {
    /// Construct a digest after validating its length for the declared suite.
    pub fn new(algorithm: DigestAlgorithm, bytes: Vec<u8>) -> Result<Self, ForgeCoreError> {
        let expected = algorithm.digest_len();
        let actual = bytes.len();
        if actual != expected {
            return Err(ForgeCoreError::InvalidDigestLength {
                algorithm,
                expected,
                actual,
            });
        }

        Ok(Self { algorithm, bytes })
    }

    /// Hash bytes with a supported digest suite.
    pub fn of_bytes(algorithm: DigestAlgorithm, input: &[u8]) -> Self {
        algorithm.hash(input)
    }

    /// Declared digest suite.
    pub const fn algorithm(&self) -> DigestAlgorithm {
        self.algorithm
    }

    /// Raw digest bytes.
    pub fn as_bytes(&self) -> &[u8] {
        &self.bytes
    }

    /// Lower-case hexadecimal digest bytes.
    pub fn to_hex(&self) -> String {
        hex::encode(&self.bytes)
    }
}

impl fmt::Display for Digest {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}:{}", self.algorithm, self.to_hex())
    }
}

impl<'de> Deserialize<'de> for Digest {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        #[derive(Deserialize)]
        struct WireDigest {
            algorithm: DigestAlgorithm,
            bytes: Vec<u8>,
        }

        let wire = WireDigest::deserialize(deserializer)?;
        Self::new(wire.algorithm, wire.bytes).map_err(D::Error::custom)
    }
}

/// Genesis material from which a stable project identity is derived.
///
/// Neither a project name nor a hosting location participates in the
/// derivation. `genesis_nonce` is supplied by the caller so this portable core
/// crate does not need an RNG or platform-specific entropy source.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ProjectIdentitySeed {
    version: ProtocolVersion,
    genesis_nonce: [u8; GENESIS_NONCE_LEN],
    root_authority_commitment: Digest,
}

impl ProjectIdentitySeed {
    /// Create a v1 project-identity seed.
    pub fn new(
        genesis_nonce: [u8; GENESIS_NONCE_LEN],
        root_authority_commitment: Digest,
    ) -> Self {
        Self {
            version: ProtocolVersion::CURRENT,
            genesis_nonce,
            root_authority_commitment,
        }
    }

    /// Protocol version controlling canonicalization semantics.
    pub const fn version(&self) -> ProtocolVersion {
        self.version
    }

    /// Caller-supplied project genesis nonce.
    pub const fn genesis_nonce(&self) -> &[u8; GENESIS_NONCE_LEN] {
        &self.genesis_nonce
    }

    /// Commitment to the project's initial root authority material.
    pub fn root_authority_commitment(&self) -> &Digest {
        &self.root_authority_commitment
    }

    /// Deterministic v1 identity preimage.
    ///
    /// The encoding is deliberately fixed-width/length-prefixed rather than
    /// relying on a generic serializer whose map ordering or representation
    /// might vary between implementations.
    pub fn canonical_bytes(&self) -> Result<Vec<u8>, ForgeCoreError> {
        match self.version.get() {
            CURRENT_PROTOCOL_VERSION => self.canonical_bytes_v1(),
            other => Err(ForgeCoreError::UnsupportedProtocolVersion(other)),
        }
    }

    fn canonical_bytes_v1(&self) -> Result<Vec<u8>, ForgeCoreError> {
        let algorithm = self.root_authority_commitment.algorithm().id().as_bytes();
        let digest = self.root_authority_commitment.as_bytes();

        let algorithm_len = u16::try_from(algorithm.len()).map_err(|_| {
            ForgeCoreError::CanonicalFieldTooLarge {
                field: "root_authority_algorithm",
                len: algorithm.len(),
                max: u16::MAX as usize,
            }
        })?;
        let digest_len = u32::try_from(digest.len()).map_err(|_| {
            ForgeCoreError::CanonicalFieldTooLarge {
                field: "root_authority_digest",
                len: digest.len(),
                max: u32::MAX as usize,
            }
        })?;

        let mut out = Vec::with_capacity(
            PROJECT_ID_DOMAIN_V1.len()
                + 2
                + GENESIS_NONCE_LEN
                + 2
                + algorithm.len()
                + 4
                + digest.len(),
        );
        out.extend_from_slice(PROJECT_ID_DOMAIN_V1);
        out.extend_from_slice(&self.version.get().to_be_bytes());
        out.extend_from_slice(&self.genesis_nonce);
        out.extend_from_slice(&algorithm_len.to_be_bytes());
        out.extend_from_slice(algorithm);
        out.extend_from_slice(&digest_len.to_be_bytes());
        out.extend_from_slice(digest);
        Ok(out)
    }
}

/// Stable, hosting-independent project identity.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct ProjectIdentity {
    version: ProtocolVersion,
    digest: Digest,
}

impl ProjectIdentity {
    /// Derive a stable project identity from genesis material.
    pub fn derive(
        seed: &ProjectIdentitySeed,
        algorithm: DigestAlgorithm,
    ) -> Result<Self, ForgeCoreError> {
        let canonical = seed.canonical_bytes()?;
        Ok(Self {
            version: seed.version(),
            digest: Digest::of_bytes(algorithm, &canonical),
        })
    }

    /// Protocol version governing identity derivation.
    pub const fn version(&self) -> ProtocolVersion {
        self.version
    }

    /// Algorithm-qualified project identity digest.
    pub fn digest(&self) -> &Digest {
        &self.digest
    }
}

impl fmt::Display for ProjectIdentity {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "mycelix:forge:project:v{}:{}",
            self.version.get(),
            self.digest
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn root_commitment(byte: u8) -> Digest {
        Digest::new(DigestAlgorithm::Sha256, vec![byte; 32]).unwrap()
    }

    fn seed() -> ProjectIdentitySeed {
        ProjectIdentitySeed::new([0x11; GENESIS_NONCE_LEN], root_commitment(0x22))
    }

    #[test]
    fn project_identity_has_frozen_v1_test_vector() {
        let seed = seed();
        let canonical = seed.canonical_bytes().unwrap();
        assert_eq!(
            hex::encode(canonical),
            "6d7963656c69782d666f7267652f70726f6a6563742d6964656e746974792f763100000111111111111111111111111111111111111111111111111111111111111111110006736861323536000000202222222222222222222222222222222222222222222222222222222222222222"
        );

        let identity = ProjectIdentity::derive(&seed, DigestAlgorithm::Sha256).unwrap();
        assert_eq!(
            identity.to_string(),
            "mycelix:forge:project:v1:sha256:ad2950ed9cadd60b6feb70051495c1577d229d93cc58e600cb7fe0dae46ea625"
        );
    }

    #[test]
    fn one_byte_genesis_mutation_changes_identity() {
        let original = seed();
        let mut changed_nonce = *original.genesis_nonce();
        changed_nonce[31] ^= 0x01;
        let mutated = ProjectIdentitySeed::new(
            changed_nonce,
            original.root_authority_commitment().clone(),
        );

        assert_ne!(
            ProjectIdentity::derive(&original, DigestAlgorithm::Sha256).unwrap(),
            ProjectIdentity::derive(&mutated, DigestAlgorithm::Sha256).unwrap()
        );
    }

    #[test]
    fn root_authority_change_changes_identity() {
        let original = seed();
        let mutated = ProjectIdentitySeed::new(
            *original.genesis_nonce(),
            root_commitment(0x23),
        );

        assert_ne!(
            ProjectIdentity::derive(&original, DigestAlgorithm::Sha256).unwrap(),
            ProjectIdentity::derive(&mutated, DigestAlgorithm::Sha256).unwrap()
        );
    }

    #[test]
    fn digest_algorithm_is_part_of_identity() {
        let seed = seed();
        let sha = ProjectIdentity::derive(&seed, DigestAlgorithm::Sha256).unwrap();
        let blake3 = ProjectIdentity::derive(&seed, DigestAlgorithm::Blake3_256).unwrap();

        assert_ne!(sha, blake3);
        assert_ne!(sha.to_string(), blake3.to_string());
    }

    #[test]
    fn hosting_location_is_not_identity_material() {
        let seed = seed();
        let before = ProjectIdentity::derive(&seed, DigestAlgorithm::Sha256).unwrap();

        let _old_remote = "https://github.com/Luminous-Dynamics/mycelix";
        let _new_remote = "rad://example-project-id";

        let after = ProjectIdentity::derive(&seed, DigestAlgorithm::Sha256).unwrap();
        assert_eq!(before, after);
    }

    #[test]
    fn invalid_digest_lengths_fail_closed() {
        let error = Digest::new(DigestAlgorithm::Sha256, vec![0; 31]).unwrap_err();
        assert_eq!(
            error,
            ForgeCoreError::InvalidDigestLength {
                algorithm: DigestAlgorithm::Sha256,
                expected: 32,
                actual: 31,
            }
        );
    }

    #[test]
    fn unknown_digest_algorithms_fail_closed() {
        let error = "sha512".parse::<DigestAlgorithm>().unwrap_err();
        assert_eq!(
            error,
            ForgeCoreError::UnsupportedDigestAlgorithm("sha512".to_owned())
        );
    }

    #[test]
    fn unsupported_protocol_versions_fail_closed() {
        assert_eq!(
            ProtocolVersion::new(2).unwrap_err(),
            ForgeCoreError::UnsupportedProtocolVersion(2)
        );
    }

    #[test]
    fn serde_round_trip_preserves_validated_identity() {
        let identity = ProjectIdentity::derive(&seed(), DigestAlgorithm::Sha256).unwrap();
        let encoded = serde_json::to_string(&identity).unwrap();
        let decoded: ProjectIdentity = serde_json::from_str(&encoded).unwrap();
        assert_eq!(identity, decoded);
    }

    #[test]
    fn serde_rejects_malformed_digest_length() {
        let malformed = r#"{"algorithm":"sha256","bytes":[0,1,2]}"#;
        let error = serde_json::from_str::<Digest>(malformed).unwrap_err();
        assert!(error.to_string().contains("invalid digest length"));
    }
}
