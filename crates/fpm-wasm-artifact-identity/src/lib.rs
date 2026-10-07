//! Cross-bind one exact WASM byte string to its two independent identities.
//!
//! SHA-256 remains the supply-chain artifact subject. Holochain 0.7 WasmHash
//! remains the native typed identity exposed by the Holochain substrate.
//! Positive derivation delegates HoloHash construction to holo_hash 0.7.0.

use bytes::Bytes;
use holo_hash::{
    hash_type, HashableContent, HashableContentBytes, HashableContentExt, WasmHash,
};
use holochain_serialized_bytes::prelude::{SerializedBytes, UnsafeBytes};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};

pub const SCHEMA_VERSION: &str = "fpm.verifier-artifact-identity.v1";
pub const HOLOCHAIN_PROFILE: &str = "holochain-0.7.0-wasmhash-v1";
pub const SHA256_HEX_LEN: usize = 64;
pub const HOLOHASH_RAW_39_LEN: usize = 39;
pub const MAX_ARTIFACT_BYTES: usize = 64 * 1024 * 1024;

#[derive(Clone, Debug, Eq, PartialEq, Serialize, Deserialize)]
pub struct FpmWasmArtifactIdentity {
    pub schema_version: String,
    pub holochain_profile: String,
    pub artifact_sha256: String,
    pub wasm_hash_raw_39: Vec<u8>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct FpmWasmArtifact {
    code: Bytes,
}

impl FpmWasmArtifact {
    pub fn new(code: Vec<u8>) -> Self {
        Self { code: code.into() }
    }
}

impl HashableContent for FpmWasmArtifact {
    type HashType = hash_type::Wasm;

    fn hash_type(&self) -> Self::HashType {
        hash_type::Wasm
    }

    fn hashable_content(&self) -> HashableContentBytes {
        HashableContentBytes::Content(SerializedBytes::from(UnsafeBytes::from(
            self.code.clone().to_vec(),
        )))
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum FpmArtifactIdentityError {
    EmptyArtifact,
    ArtifactTooLarge,
    InvalidSha256,
    InvalidWasmHashLength,
    InvalidWasmHashType,
    Sha256Mismatch,
    WasmHashMismatch,
}

pub fn sha256_hex(code: &[u8]) -> String {
    let mut hasher = Sha256::new();
    hasher.update(code);
    hasher
        .finalize()
        .iter()
        .map(|byte| format!("{byte:02x}"))
        .collect()
}

pub fn validate_identity_shape(
    identity: &FpmWasmArtifactIdentity,
) -> Result<(), FpmArtifactIdentityError> {
    if identity.schema_version != SCHEMA_VERSION
        || identity.holochain_profile != HOLOCHAIN_PROFILE
        || !is_canonical_sha256(&identity.artifact_sha256)
    {
        return Err(FpmArtifactIdentityError::InvalidSha256);
    }

    if identity.wasm_hash_raw_39.len() != HOLOHASH_RAW_39_LEN {
        return Err(FpmArtifactIdentityError::InvalidWasmHashLength);
    }

    let wasm_hash = WasmHash::try_from_raw_39(identity.wasm_hash_raw_39.clone())
        .map_err(|_| FpmArtifactIdentityError::InvalidWasmHashType)?;

    if *wasm_hash.hash_type() != hash_type::Wasm {
        return Err(FpmArtifactIdentityError::InvalidWasmHashType);
    }

    Ok(())
}

pub async fn derive_identity(
    code: Vec<u8>,
) -> Result<FpmWasmArtifactIdentity, FpmArtifactIdentityError> {
    if code.is_empty() {
        return Err(FpmArtifactIdentityError::EmptyArtifact);
    }
    if code.len() > MAX_ARTIFACT_BYTES {
        return Err(FpmArtifactIdentityError::ArtifactTooLarge);
    }

    let wasm_hash = WasmHash::with_data(&FpmWasmArtifact::new(code.clone())).await;
    let identity = FpmWasmArtifactIdentity {
        schema_version: SCHEMA_VERSION.into(),
        holochain_profile: HOLOCHAIN_PROFILE.into(),
        artifact_sha256: sha256_hex(&code),
        wasm_hash_raw_39: wasm_hash.get_raw_39().to_vec(),
    };

    validate_identity_shape(&identity)?;
    Ok(identity)
}

pub async fn verify_approved_artifact_against_observed_wasm_hash(
    identity: &FpmWasmArtifactIdentity,
    artifact_code: &[u8],
    observed_wasm_hash_raw_39: &[u8],
) -> Result<(), FpmArtifactIdentityError> {
    validate_identity_shape(identity)?;

    if artifact_code.is_empty() {
        return Err(FpmArtifactIdentityError::EmptyArtifact);
    }
    if artifact_code.len() > MAX_ARTIFACT_BYTES {
        return Err(FpmArtifactIdentityError::ArtifactTooLarge);
    }

    if sha256_hex(artifact_code) != identity.artifact_sha256 {
        return Err(FpmArtifactIdentityError::Sha256Mismatch);
    }

    if observed_wasm_hash_raw_39.len() != HOLOHASH_RAW_39_LEN {
        return Err(FpmArtifactIdentityError::InvalidWasmHashLength);
    }

    let observed = WasmHash::try_from_raw_39(observed_wasm_hash_raw_39.to_vec())
        .map_err(|_| FpmArtifactIdentityError::InvalidWasmHashType)?;
    let derived = WasmHash::with_data(&FpmWasmArtifact::new(artifact_code.to_vec())).await;

    if observed.get_raw_39() != derived.get_raw_39() {
        return Err(FpmArtifactIdentityError::WasmHashMismatch);
    }

    Ok(())
}

fn is_canonical_sha256(value: &str) -> bool {
    value.len() == SHA256_HEX_LEN
        && value
            .bytes()
            .all(|byte| matches!(byte, b'0'..=b'9' | b'a'..=b'f'))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn artifact() -> Vec<u8> {
        b"(module (func (export \"verify\")))".to_vec()
    }

    #[tokio::test]
    async fn exact_identity_is_derived_from_exact_bytes() {
        let code = artifact();
        let identity = derive_identity(code.clone()).await.unwrap();

        assert_eq!(identity.artifact_sha256, sha256_hex(&code));
        assert_eq!(identity.wasm_hash_raw_39.len(), HOLOHASH_RAW_39_LEN);
        assert!(validate_identity_shape(&identity).is_ok());
    }

    #[tokio::test]
    async fn one_byte_mutation_changes_both_identity_domains() {
        let code = artifact();
        let identity = derive_identity(code.clone()).await.unwrap();

        let mut mutated = code;
        mutated[10] ^= 0x01;
        let other = derive_identity(mutated).await.unwrap();

        assert_ne!(identity.artifact_sha256, other.artifact_sha256);
        assert_ne!(identity.wasm_hash_raw_39, other.wasm_hash_raw_39);
    }

    #[tokio::test]
    async fn exact_observed_hash_qualifies() {
        let code = artifact();
        let identity = derive_identity(code.clone()).await.unwrap();

        verify_approved_artifact_against_observed_wasm_hash(
            &identity,
            &code,
            &identity.wasm_hash_raw_39,
        )
        .await
        .unwrap();
    }

    #[tokio::test]
    async fn substituted_sha256_denies_even_with_correct_wasm_hash() {
        let code = artifact();
        let identity = derive_identity(code.clone()).await.unwrap();

        let mut changed = identity.clone();
        changed.artifact_sha256 = sha256_hex(b"different artifact");

        assert_eq!(
            verify_approved_artifact_against_observed_wasm_hash(
                &changed,
                &code,
                &identity.wasm_hash_raw_39,
            )
            .await
            .unwrap_err(),
            FpmArtifactIdentityError::Sha256Mismatch
        );
    }

    #[tokio::test]
    async fn substituted_wasm_hash_denies_even_with_correct_sha256() {
        let code = artifact();
        let identity = derive_identity(code.clone()).await.unwrap();
        let other = derive_identity(b"(module (func (export \"other\")))".to_vec())
            .await
            .unwrap();

        assert_eq!(
            verify_approved_artifact_against_observed_wasm_hash(
                &identity,
                &code,
                &other.wasm_hash_raw_39,
            )
            .await
            .unwrap_err(),
            FpmArtifactIdentityError::WasmHashMismatch
        );
    }

    #[tokio::test]
    async fn malformed_wasm_hash_denies() {
        let code = artifact();
        let identity = derive_identity(code.clone()).await.unwrap();

        let mut bad = identity.wasm_hash_raw_39.clone();
        bad.pop();

        assert_eq!(
            verify_approved_artifact_against_observed_wasm_hash(&identity, &code, &bad)
                .await
                .unwrap_err(),
            FpmArtifactIdentityError::InvalidWasmHashLength
        );
    }

    #[tokio::test]
    async fn mismatched_pair_cannot_qualify() {
        let left = b"(module (func (export \"left\")))".to_vec();
        let right = b"(module (func (export \"right\")))".to_vec();
        let left_identity = derive_identity(left.clone()).await.unwrap();
        let right_identity = derive_identity(right).await.unwrap();

        assert_eq!(
            verify_approved_artifact_against_observed_wasm_hash(
                &left_identity,
                &left,
                &right_identity.wasm_hash_raw_39,
            )
            .await
            .unwrap_err(),
            FpmArtifactIdentityError::WasmHashMismatch
        );
    }

    #[test]
    fn malformed_sha256_is_rejected() {
        let identity = FpmWasmArtifactIdentity {
            schema_version: SCHEMA_VERSION.into(),
            holochain_profile: HOLOCHAIN_PROFILE.into(),
            artifact_sha256: "not-a-sha".into(),
            wasm_hash_raw_39: vec![0; HOLOHASH_RAW_39_LEN],
        };
        assert_eq!(
            validate_identity_shape(&identity).unwrap_err(),
            FpmArtifactIdentityError::InvalidSha256
        );
    }
}
