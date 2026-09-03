// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Additive profile-bound release credentials for high-assurance SIF disclosure.
//!
//! The v1 Mycelix release credential remains valid historical evidence and continues
//! to identify the witnessed accountability execution/result. This crate wraps that
//! exact canonical v1 statement with one additional authorization requirement: the
//! exact SIF protected-transfer profile Xenia must negotiate before release.
//!
//! The required profile is part of both the v2 credential identifier and every
//! authority signature. Xenia must therefore never substitute its local current
//! profile for a missing value, and changing the required profile creates a distinct
//! credential even when the witnessed accountability evidence is otherwise identical.

use ed25519_dalek::{Signature, Signer, SigningKey, Verifier, VerifyingKey};
use mycelix_accountability_bundle::PreDisclosureVerifiedBundle;
use mycelix_accountability_core::Commitment32;
use mycelix_accountability_credential::{
    ReleaseCredentialError as ReleaseCredentialV1Error, SIF_RELEASE_CREDENTIAL_ED25519,
    SIF_RELEASE_CREDENTIAL_SCHEMA, SifReleaseCredentialSignature, SifReleaseCredentialStatement,
    credential_from_verified_bundle, release_authority_key_id, release_credential_message,
};
use serde::{Deserialize, Serialize};
use thiserror::Error;

/// Stable schema for the profile-bound release credential.
pub const SIF_RELEASE_CREDENTIAL_V2_SCHEMA: &str = "sif-release-credential-v2";
/// Canonical byte profile for v2 authority signatures.
pub const SIF_RELEASE_CREDENTIAL_V2_CODEC: &str = "sif-release-credential-canonical-v2";

const RELEASE_CREDENTIAL_V2_MESSAGE_DOMAIN: &[u8] = b"sif:release-credential:statement:v2";
const RELEASE_CREDENTIAL_V2_ID_DOMAIN: &[u8] = b"sif:release-credential:id:v2";

/// Canonical commitment-only authorization statement for one profile-bound release.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq)]
pub struct SifReleaseCredentialStatementV2 {
    /// Stable v2 schema label.
    pub schema: String,
    /// Derived identity of the exact v1 statement + required SIF profile.
    pub credential_id: Commitment32,
    /// Complete historical v1 release-credential statement.
    pub v1: SifReleaseCredentialStatement,
    /// Exact SIF protected-transfer profile required by Mycelix authorization.
    pub required_sif_profile_digest: Commitment32,
}

impl SifReleaseCredentialStatementV2 {
    /// Validate schema, nested v1 identity and profile-bound credential identity.
    pub fn validate(&self) -> Result<(), ReleaseCredentialV2Error> {
        if self.schema != SIF_RELEASE_CREDENTIAL_V2_SCHEMA {
            return Err(ReleaseCredentialV2Error::UnsupportedSchema);
        }
        if self.v1.schema != SIF_RELEASE_CREDENTIAL_SCHEMA {
            return Err(ReleaseCredentialV2Error::NestedV1SchemaMismatch);
        }
        if self.required_sif_profile_digest.is_zero() {
            return Err(ReleaseCredentialV2Error::ZeroRequiredSifProfile);
        }
        let expected = profile_bound_credential_id(&self.v1, self.required_sif_profile_digest);
        if self.credential_id != expected {
            return Err(ReleaseCredentialV2Error::CredentialIdMismatch);
        }
        Ok(())
    }
}

/// Portable multi-authority v2 credential.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq)]
pub struct SifReleaseCredentialV2 {
    /// Canonical profile-bound statement.
    pub statement: SifReleaseCredentialStatementV2,
    /// Independent authority signatures over the exact v2 statement.
    pub signatures: Vec<SifReleaseCredentialSignature>,
}

/// Profile-bound credential construction/signature failures.
#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum ReleaseCredentialV2Error {
    /// Historical v1 credential derivation failed.
    #[error(transparent)]
    V1(#[from] ReleaseCredentialV1Error),
    /// Outer schema is not the exact v2 schema.
    #[error("unsupported profile-bound release credential schema")]
    UnsupportedSchema,
    /// Nested historical statement is not a v1 release credential.
    #[error("profile-bound release credential does not contain a v1 statement")]
    NestedV1SchemaMismatch,
    /// High-assurance authorization requires an explicit non-zero SIF profile.
    #[error("required SIF profile digest must not be all-zero")]
    ZeroRequiredSifProfile,
    /// Stored v2 credential ID did not match the canonical v1 statement/profile pair.
    #[error("profile-bound release credential ID mismatch")]
    CredentialIdMismatch,
    /// The same authority key may sign the exact v2 statement only once.
    #[error("profile-bound release credential already contains this authority signature")]
    DuplicateAuthoritySignature,
    /// Signature algorithm is not the v2 Ed25519 baseline.
    #[error("unsupported profile-bound release credential signature algorithm")]
    UnsupportedSignatureAlgorithm,
    /// Signature did not carry exactly 64 Ed25519 bytes.
    #[error("invalid Ed25519 profile-bound release credential signature length")]
    InvalidSignatureLength,
    /// Supplied trusted key does not match the signature's authority identifier.
    #[error("profile-bound release credential authority key identifier mismatch")]
    AuthorityKeyMismatch,
    /// Signature verification failed.
    #[error("profile-bound release credential signature verification failed")]
    InvalidSignature,
}

/// Derive a v2 credential only from the existing witnessed v1 construction boundary.
///
/// `v1_lineage_credential_id` retains the existing v1 retry-lineage identity. The v2
/// `credential_id` is then derived from the complete canonical v1 statement and exact
/// required SIF profile so changing either produces a distinct v2 authorization.
pub fn credential_v2_from_verified_bundle(
    bundle: &PreDisclosureVerifiedBundle,
    v1_lineage_credential_id: Commitment32,
    execution_proof_digest: Commitment32,
    required_sif_profile_digest: Commitment32,
) -> Result<SifReleaseCredentialV2, ReleaseCredentialV2Error> {
    if required_sif_profile_digest.is_zero() {
        return Err(ReleaseCredentialV2Error::ZeroRequiredSifProfile);
    }
    let v1 = credential_from_verified_bundle(
        bundle,
        v1_lineage_credential_id,
        execution_proof_digest,
    )?;
    let credential_id = profile_bound_credential_id(&v1.statement, required_sif_profile_digest);
    let statement = SifReleaseCredentialStatementV2 {
        schema: SIF_RELEASE_CREDENTIAL_V2_SCHEMA.to_string(),
        credential_id,
        v1: v1.statement,
        required_sif_profile_digest,
    };
    statement.validate()?;
    Ok(SifReleaseCredentialV2 {
        statement,
        signatures: Vec::new(),
    })
}

/// Derive the stable v2 credential identity from canonical witnessed authority + profile.
pub fn profile_bound_credential_id(
    v1: &SifReleaseCredentialStatement,
    required_sif_profile_digest: Commitment32,
) -> Commitment32 {
    let v1_message = release_credential_message(v1);
    let mut hasher = blake3::Hasher::new();
    hasher.update(RELEASE_CREDENTIAL_V2_ID_DOMAIN);
    hasher.update(&[0]);
    hasher.update(&(v1_message.len() as u64).to_be_bytes());
    hasher.update(&v1_message);
    hasher.update(required_sif_profile_digest.as_bytes());
    Commitment32(*hasher.finalize().as_bytes())
}

/// Exact language-neutral bytes every v2 release authority signs.
pub fn release_credential_v2_message(
    statement: &SifReleaseCredentialStatementV2,
) -> Result<Vec<u8>, ReleaseCredentialV2Error> {
    statement.validate()?;
    let v1_message = release_credential_message(&statement.v1);
    let mut out = Vec::with_capacity(v1_message.len() + 160);
    out.extend_from_slice(RELEASE_CREDENTIAL_V2_MESSAGE_DOMAIN);
    out.push(0);
    out.extend_from_slice(SIF_RELEASE_CREDENTIAL_V2_SCHEMA.as_bytes());
    out.push(0);
    out.extend_from_slice(SIF_RELEASE_CREDENTIAL_V2_CODEC.as_bytes());
    out.push(0);
    out.extend_from_slice(statement.credential_id.as_bytes());
    out.extend_from_slice(&(v1_message.len() as u64).to_be_bytes());
    out.extend_from_slice(&v1_message);
    out.extend_from_slice(statement.required_sif_profile_digest.as_bytes());
    Ok(out)
}

/// Append one Ed25519 authority signature over the exact v2 statement.
pub fn sign_release_credential_v2(
    credential: &mut SifReleaseCredentialV2,
    signing_key: &SigningKey,
) -> Result<(), ReleaseCredentialV2Error> {
    let public_key = signing_key.verifying_key().to_bytes();
    let signer_key_id = release_authority_key_id(&public_key);
    if credential
        .signatures
        .iter()
        .any(|signature| signature.signer_key_id == signer_key_id)
    {
        return Err(ReleaseCredentialV2Error::DuplicateAuthoritySignature);
    }
    let signature = signing_key
        .sign(&release_credential_v2_message(&credential.statement)?)
        .to_bytes();
    credential.signatures.push(SifReleaseCredentialSignature {
        algorithm: SIF_RELEASE_CREDENTIAL_ED25519.to_string(),
        signer_key_id,
        signature: signature.to_vec(),
    });
    Ok(())
}

/// Verify one v2 authority signature against an explicitly trusted Ed25519 key.
pub fn verify_release_credential_v2_signature(
    statement: &SifReleaseCredentialStatementV2,
    signature: &SifReleaseCredentialSignature,
    public_key: &[u8; 32],
) -> Result<(), ReleaseCredentialV2Error> {
    if signature.algorithm != SIF_RELEASE_CREDENTIAL_ED25519 {
        return Err(ReleaseCredentialV2Error::UnsupportedSignatureAlgorithm);
    }
    if signature.signer_key_id != release_authority_key_id(public_key) {
        return Err(ReleaseCredentialV2Error::AuthorityKeyMismatch);
    }
    let signature_bytes: [u8; 64] = signature
        .signature
        .as_slice()
        .try_into()
        .map_err(|_| ReleaseCredentialV2Error::InvalidSignatureLength)?;
    let verifying_key = VerifyingKey::from_bytes(public_key)
        .map_err(|_| ReleaseCredentialV2Error::InvalidSignature)?;
    verifying_key
        .verify(
            &release_credential_v2_message(statement)?,
            &Signature::from_bytes(&signature_bytes),
        )
        .map_err(|_| ReleaseCredentialV2Error::InvalidSignature)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn c(value: u8) -> Commitment32 {
        Commitment32([value; 32])
    }

    fn v1_statement() -> SifReleaseCredentialStatement {
        SifReleaseCredentialStatement {
            schema: SIF_RELEASE_CREDENTIAL_SCHEMA.to_string(),
            credential_id: c(1),
            receipt_statement_digest: c(2),
            pre_witness_bundle_digest: c(3),
            finalized_evidence_bundle_digest: c(4),
            accountability_policy_digest: c(5),
            non_witness_trust_policy_digest: c(6),
            witness_policy_digest: c(7),
            execution_proof_digest: c(8),
            execution_verifier_id: c(9),
            execution_trust_domain_id: c(10),
            result_digest: Some(c(11)),
        }
    }

    fn statement(profile: Commitment32) -> SifReleaseCredentialStatementV2 {
        let v1 = v1_statement();
        SifReleaseCredentialStatementV2 {
            schema: SIF_RELEASE_CREDENTIAL_V2_SCHEMA.to_string(),
            credential_id: profile_bound_credential_id(&v1, profile),
            v1,
            required_sif_profile_digest: profile,
        }
    }

    #[test]
    fn changing_only_profile_changes_message_and_credential_id() {
        let p1 = statement(c(20));
        let p2 = statement(c(21));
        assert_ne!(p1.credential_id, p2.credential_id);
        assert_ne!(
            release_credential_v2_message(&p1).unwrap(),
            release_credential_v2_message(&p2).unwrap()
        );
    }

    #[test]
    fn zero_profile_fails_closed() {
        let v1 = v1_statement();
        let statement = SifReleaseCredentialStatementV2 {
            schema: SIF_RELEASE_CREDENTIAL_V2_SCHEMA.to_string(),
            credential_id: profile_bound_credential_id(&v1, c(0)),
            v1,
            required_sif_profile_digest: c(0),
        };
        assert_eq!(
            statement.validate(),
            Err(ReleaseCredentialV2Error::ZeroRequiredSifProfile)
        );
    }

    #[test]
    fn v2_signature_is_profile_bound() {
        let key = SigningKey::from_bytes(&[42u8; 32]);
        let mut credential = SifReleaseCredentialV2 {
            statement: statement(c(20)),
            signatures: Vec::new(),
        };
        sign_release_credential_v2(&mut credential, &key).unwrap();
        verify_release_credential_v2_signature(
            &credential.statement,
            &credential.signatures[0],
            &key.verifying_key().to_bytes(),
        )
        .unwrap();

        let wrong_profile = statement(c(21));
        assert_eq!(
            verify_release_credential_v2_signature(
                &wrong_profile,
                &credential.signatures[0],
                &key.verifying_key().to_bytes(),
            ),
            Err(ReleaseCredentialV2Error::InvalidSignature)
        );
    }

    #[test]
    fn valid_v1_signature_does_not_verify_as_v2() {
        let key = SigningKey::from_bytes(&[42u8; 32]);
        let v1 = v1_statement();
        let v1_signature = key.sign(&release_credential_message(&v1)).to_bytes();
        let signature = SifReleaseCredentialSignature {
            algorithm: SIF_RELEASE_CREDENTIAL_ED25519.to_string(),
            signer_key_id: release_authority_key_id(&key.verifying_key().to_bytes()),
            signature: v1_signature.to_vec(),
        };
        assert_eq!(
            verify_release_credential_v2_signature(
                &statement(c(20)),
                &signature,
                &key.verifying_key().to_bytes(),
            ),
            Err(ReleaseCredentialV2Error::InvalidSignature)
        );
    }

    #[test]
    fn tampered_derived_id_is_rejected_before_signature_verification() {
        let mut statement = statement(c(20));
        statement.credential_id = c(99);
        assert_eq!(
            statement.validate(),
            Err(ReleaseCredentialV2Error::CredentialIdMismatch)
        );
    }
}
