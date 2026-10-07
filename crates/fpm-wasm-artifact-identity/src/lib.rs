//! Cross-bind one exact WASM byte string to its two independent identities.
//!
//! SHA-256 remains the supply-chain artifact subject. Holochain 0.7 WasmHash
//! remains the native typed identity exposed by the Holochain substrate.
//! Positive derivation uses holo_hash's canonical typed hashing API.

use holo_hash::{hash_type, HashableContentExtSync, WasmHash};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};

pub const SCHEMA_VERSION: &str = "fpm.verifier-artifact-identity.v1";
pub const HOLOCHAIN_PROFILE: &str = "holochain-0.7.0-wasmhash-v1";
pub const SHA256_HEX_LEN: usize = 64;
pub const HOLOHASH_RAW_39_LEN: usize = 39;
/// HoloHash 0.7 synchronous hashing rejects larger content; keep this explicit so
/// the verifier fails closed before calling the constructor.
pub const MAX_ARTIFACT_BYTES: usize = holo_hash::MAX_HASHABLE_CONTENT_LEN;

#[derive(Clone, Debug, Eq, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct FpmWasmArtifactIdentity {
    pub schema_version: String,
    pub holochain_profile: String,
    pub artifact_sha256: String,
    pub wasm_hash_raw_39: Vec<u8>,
}

#[derive(Clone, Debug, Eq, PartialEq, Serialize)]
pub struct MatchedFpmWasmArtifactIdentity {
    schema_version: String,
    holochain_profile: String,
    artifact_sha256: String,
    wasm_hash_raw_39: Vec<u8>,
}

impl MatchedFpmWasmArtifactIdentity {
    pub fn schema_version(&self) -> &str {
        &self.schema_version
    }

    pub fn holochain_profile(&self) -> &str {
        &self.holochain_profile
    }

    pub fn artifact_sha256(&self) -> &str {
        &self.artifact_sha256
    }

    pub fn wasm_hash_raw_39(&self) -> &[u8] {
        &self.wasm_hash_raw_39
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
    ApprovedWasmHashMismatch,
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

pub fn derive_identity(
    code: &[u8],
) -> Result<FpmWasmArtifactIdentity, FpmArtifactIdentityError> {
    validate_artifact_size(code)?;

    let wasm_hash = WasmHash::with_data_sync(code.to_vec());
    let identity = FpmWasmArtifactIdentity {
        schema_version: SCHEMA_VERSION.into(),
        holochain_profile: HOLOCHAIN_PROFILE.into(),
        artifact_sha256: sha256_hex(code),
        wasm_hash_raw_39: wasm_hash.get_raw_39().to_vec(),
    };

    validate_identity_shape(&identity)?;
    Ok(identity)
}

pub fn verify_approved_artifact_against_observed_wasm_hash(
    identity: &FpmWasmArtifactIdentity,
    artifact_code: &[u8],
    observed_wasm_hash_raw_39: &[u8],
) -> Result<MatchedFpmWasmArtifactIdentity, FpmArtifactIdentityError> {
    validate_identity_shape(identity)?;
    validate_artifact_size(artifact_code)?;

    if sha256_hex(artifact_code) != identity.artifact_sha256 {
        return Err(FpmArtifactIdentityError::Sha256Mismatch);
    }

    let derived = WasmHash::with_data_sync(artifact_code.to_vec());
    if identity.wasm_hash_raw_39.as_slice() != derived.get_raw_39() {
        return Err(FpmArtifactIdentityError::ApprovedWasmHashMismatch);
    }

    if observed_wasm_hash_raw_39.len() != HOLOHASH_RAW_39_LEN {
        return Err(FpmArtifactIdentityError::InvalidWasmHashLength);
    }

    let observed = WasmHash::try_from_raw_39(observed_wasm_hash_raw_39.to_vec())
        .map_err(|_| FpmArtifactIdentityError::InvalidWasmHashType)?;

    if observed.get_raw_39() != derived.get_raw_39() {
        return Err(FpmArtifactIdentityError::WasmHashMismatch);
    }

    Ok(MatchedFpmWasmArtifactIdentity {
        schema_version: identity.schema_version.clone(),
        holochain_profile: identity.holochain_profile.clone(),
        artifact_sha256: identity.artifact_sha256.clone(),
        wasm_hash_raw_39: observed.get_raw_39().to_vec(),
    })
}

fn validate_artifact_size(code: &[u8]) -> Result<(), FpmArtifactIdentityError> {
    if code.is_empty() {
        Err(FpmArtifactIdentityError::EmptyArtifact)
    } else if code.len() > MAX_ARTIFACT_BYTES {
        Err(FpmArtifactIdentityError::ArtifactTooLarge)
    } else {
        Ok(())
    }
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

    #[test]
    fn holochain_raw_byte_derivation_matches_canonical_dna_wasm() {
        use holochain_types::dna::wasm::DnaWasm;

        let code = artifact();
        let canonical = WasmHash::with_data_sync(&DnaWasm::from(code.clone()));
        let raw_byte_derivation = WasmHash::with_data_sync(code);

        assert_eq!(
            canonical.get_raw_39(),
            raw_byte_derivation.get_raw_39(),
            "FPM byte-domain derivation must equal Holochain 0.7 DnaWasm derivation",
        );
    }

    #[test]
    fn unknown_identity_fields_are_rejected() {
        let identity = derive_identity(&artifact()).unwrap();
        let mut json = serde_json::to_string(&identity).unwrap();
        json.insert_str(json.len() - 1, ",\"future_field\":true");

        assert!(serde_json::from_str::<FpmWasmArtifactIdentity>(&json).is_err());
    }

    #[test]
    fn exact_identity_is_derived_from_exact_bytes() {
        let code = artifact();
        let identity = derive_identity(&code).unwrap();

        assert_eq!(identity.artifact_sha256, sha256_hex(&code));
        assert_eq!(identity.wasm_hash_raw_39.len(), HOLOHASH_RAW_39_LEN);
        assert!(validate_identity_shape(&identity).is_ok());
    }

    #[test]
    fn one_byte_mutation_changes_both_identity_domains() {
        let code = artifact();
        let identity = derive_identity(&code).unwrap();

        let mut mutated = code;
        mutated[10] ^= 0x01;
        let other = derive_identity(&mutated).unwrap();

        assert_ne!(identity.artifact_sha256, other.artifact_sha256);
        assert_ne!(identity.wasm_hash_raw_39, other.wasm_hash_raw_39);
    }

    #[test]
    fn exact_observed_hash_qualifies() {
        let code = artifact();
        let identity = derive_identity(&code).unwrap();

        let matched = verify_approved_artifact_against_observed_wasm_hash(
            &identity,
            &code,
            &identity.wasm_hash_raw_39,
        )
        .unwrap();
        assert_eq!(matched.artifact_sha256(), identity.artifact_sha256);
        assert_eq!(matched.wasm_hash_raw_39(), identity.wasm_hash_raw_39);
    }

    #[test]
    fn inconsistent_approved_pair_denies_even_with_correct_observation() {
        let code = artifact();
        let identity = derive_identity(&code).unwrap();
        let other = derive_identity(b"(module (func (export \"other\")))").unwrap();
        let mut inconsistent = identity.clone();
        inconsistent.wasm_hash_raw_39 = other.wasm_hash_raw_39;

        assert_eq!(
            verify_approved_artifact_against_observed_wasm_hash(
                &inconsistent,
                &code,
                &identity.wasm_hash_raw_39,
            )
            .unwrap_err(),
            FpmArtifactIdentityError::ApprovedWasmHashMismatch
        );
    }

    #[test]
    fn substituted_sha256_denies_even_with_correct_wasm_hash() {
        let code = artifact();
        let identity = derive_identity(&code).unwrap();
        let mut changed = identity.clone();
        changed.artifact_sha256 = sha256_hex(b"different artifact");

        assert_eq!(
            verify_approved_artifact_against_observed_wasm_hash(
                &changed,
                &code,
                &identity.wasm_hash_raw_39,
            )
            .unwrap_err(),
            FpmArtifactIdentityError::Sha256Mismatch
        );
    }

    #[test]
    fn substituted_wasm_hash_denies_even_with_correct_sha256() {
        let code = artifact();
        let identity = derive_identity(&code).unwrap();
        let other = derive_identity(b"(module (func (export \"other\")))").unwrap();

        assert_eq!(
            verify_approved_artifact_against_observed_wasm_hash(
                &identity,
                &code,
                &other.wasm_hash_raw_39,
            )
            .unwrap_err(),
            FpmArtifactIdentityError::WasmHashMismatch
        );
    }

    #[test]
    fn malformed_wasm_hash_denies() {
        let code = artifact();
        let identity = derive_identity(&code).unwrap();

        let mut bad = identity.wasm_hash_raw_39.clone();
        bad.pop();

        assert_eq!(
            verify_approved_artifact_against_observed_wasm_hash(&identity, &code, &bad)
                .unwrap_err(),
            FpmArtifactIdentityError::InvalidWasmHashLength
        );
    }

    #[test]
    fn mismatched_pair_cannot_qualify() {
        let left = b"(module (func (export \"left\")))".to_vec();
        let right = b"(module (func (export \"right\")))".to_vec();
        let left_identity = derive_identity(&left).unwrap();
        let right_identity = derive_identity(&right).unwrap();

        assert_eq!(
            verify_approved_artifact_against_observed_wasm_hash(
                &left_identity,
                &left,
                &right_identity.wasm_hash_raw_39,
            )
            .unwrap_err(),
            FpmArtifactIdentityError::WasmHashMismatch
        );
    }

    #[test]
    fn empty_and_oversized_artifacts_are_rejected() {
        assert_eq!(
            derive_identity(&[]).unwrap_err(),
            FpmArtifactIdentityError::EmptyArtifact
        );
        let oversized = vec![0u8; MAX_ARTIFACT_BYTES + 1];
        assert_eq!(
            derive_identity(&oversized).unwrap_err(),
            FpmArtifactIdentityError::ArtifactTooLarge
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
