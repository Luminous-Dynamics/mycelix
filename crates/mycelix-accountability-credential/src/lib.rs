// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Portable release credentials for crossing the Mycelix -> Xenia trust boundary.
//!
//! Xenia deliberately does not parse Mycelix's full receipt/proof graph. Passing it
//! only a raw 32-byte bundle digest, however, would let an untrusted caller invent a
//! digest and ask Xenia to release against it. This crate turns a
//! [`PreDisclosureVerifiedBundle`] into a small canonical statement that one or more
//! configured release authorities can sign. Xenia can verify those signatures and a
//! local threshold/trust-domain policy before preparing a disclosure permit.
//!
//! The credential contains commitments only: no subject identifier, case/matter ID,
//! purpose text, query text, or protected record bytes.

use ed25519_dalek::{Signature, Signer, SigningKey, Verifier, VerifyingKey};
use mycelix_accountability_bundle::PreDisclosureVerifiedBundle;
use mycelix_accountability_core::{AttestationRole, Commitment32};
use serde::{Deserialize, Serialize};
use thiserror::Error;

/// Stable release-credential schema shared with Xenia.
pub const SIF_RELEASE_CREDENTIAL_SCHEMA: &str = "sif-release-credential-v1";
/// Canonical byte profile for the signed statement.
pub const SIF_RELEASE_CREDENTIAL_CODEC: &str = "sif-release-credential-canonical-v1";
/// Signature suite implemented by the v1 helper.
pub const SIF_RELEASE_CREDENTIAL_ED25519: &str = "ed25519-rfc8032";

const RELEASE_CREDENTIAL_DOMAIN: &[u8] = b"sif:release-credential:statement:v1";
const RELEASE_AUTHORITY_KEY_DOMAIN: &[u8] = b"sif:release-credential:authority-key:v1";

/// Commitment-only statement authorizing one Xenia release lineage.
///
/// `credential_id` is intentionally independent of the receipt/bundle digests. Xenia
/// should allow it to seed one initial release plus explicit retry descendants, but
/// reject a second unrelated release lineage under the same credential.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq)]
pub struct SifReleaseCredentialStatement {
    /// Stable schema label.
    pub schema: String,
    /// Unique non-zero release-lineage credential identifier.
    pub credential_id: Commitment32,
    /// Frozen semantic Mycelix receipt statement.
    pub receipt_statement_digest: Commitment32,
    /// Exact provider-proof bundle externally witnessed before release.
    pub pre_witness_bundle_digest: Commitment32,
    /// Final archival evidence-bundle digest including resolved witnesses.
    pub finalized_evidence_bundle_digest: Commitment32,
    /// Exact semantic accountability policy qualified by the bundle.
    pub accountability_policy_digest: Commitment32,
    /// Exact non-witness trust-composition policy qualified by the bundle.
    pub non_witness_trust_policy_digest: Commitment32,
    /// Exact external-witness policy qualified by the bundle.
    pub witness_policy_digest: Commitment32,
    /// Selected trust-qualified Xenia `ExecutionBinding` proof identifier.
    pub execution_proof_digest: Commitment32,
    /// Cryptographic verifier/key root that qualified the selected execution proof.
    pub execution_verifier_id: Commitment32,
    /// Administrative trust domain that qualified the selected execution proof.
    pub execution_trust_domain_id: Commitment32,
    /// Minimum-necessary result commitment, when the lookup produced a result.
    pub result_digest: Option<Commitment32>,
}

/// One signature over a canonical release-credential statement.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq)]
pub struct SifReleaseCredentialSignature {
    /// Stable signature algorithm label.
    pub algorithm: String,
    /// Domain-separated identifier of the signing public key.
    pub signer_key_id: Commitment32,
    /// Raw signature bytes. Ed25519 v1 requires exactly 64 bytes.
    pub signature: Vec<u8>,
}

/// Portable credential. Multi-authority deployments append independent signatures
/// over the exact same statement; Xenia decides the required threshold/trust domains.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq)]
pub struct SifReleaseCredential {
    /// Canonical statement derived from a witnessed bundle.
    pub statement: SifReleaseCredentialStatement,
    /// Independent release-authority signatures.
    pub signatures: Vec<SifReleaseCredentialSignature>,
}

/// Credential construction/signature failures.
#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum ReleaseCredentialError {
    /// Credential IDs may not use the all-zero placeholder.
    #[error("release credential ID must not be all-zero")]
    ZeroCredentialId,
    /// Requested Xenia execution proof is absent from the trust-qualified base set.
    #[error("selected execution proof is not a trust-qualified ExecutionBinding")]
    ExecutionProofNotQualified,
    /// The same authority key may sign a credential only once.
    #[error("release credential already contains a signature from this authority key")]
    DuplicateAuthoritySignature,
    /// Signature record algorithm is not the v1 Ed25519 profile.
    #[error("unsupported release credential signature algorithm")]
    UnsupportedSignatureAlgorithm,
    /// Signature did not carry exactly 64 Ed25519 bytes.
    #[error("invalid Ed25519 release credential signature length")]
    InvalidSignatureLength,
    /// Supplied public key does not match the signature's committed authority ID.
    #[error("release credential authority key identifier mismatch")]
    AuthorityKeyMismatch,
    /// Ed25519 signature verification failed.
    #[error("release credential signature verification failed")]
    InvalidSignature,
}

/// Derive a release credential only from a bundle that has already passed Mycelix's
/// non-witness trust composition and external-witness verification boundary.
pub fn credential_from_verified_bundle(
    bundle: &PreDisclosureVerifiedBundle,
    credential_id: Commitment32,
    execution_proof_digest: Commitment32,
) -> Result<SifReleaseCredential, ReleaseCredentialError> {
    if credential_id.is_zero() {
        return Err(ReleaseCredentialError::ZeroCredentialId);
    }

    let execution = bundle
        .base()
        .evidence()
        .iter()
        .find(|evidence| {
            evidence.role == AttestationRole::ExecutionBinding
                && evidence.proof_digest == execution_proof_digest
        })
        .ok_or(ReleaseCredentialError::ExecutionProofNotQualified)?;

    let context = bundle.base().verification_context();
    Ok(SifReleaseCredential {
        statement: SifReleaseCredentialStatement {
            schema: SIF_RELEASE_CREDENTIAL_SCHEMA.to_string(),
            credential_id,
            receipt_statement_digest: context.statement_digest,
            pre_witness_bundle_digest: bundle.pre_witness_bundle_digest(),
            finalized_evidence_bundle_digest: bundle.finalized_evidence_bundle_digest(),
            accountability_policy_digest: bundle.base().accountability_policy_digest(),
            non_witness_trust_policy_digest: bundle.base().trust_policy_digest(),
            witness_policy_digest: bundle.witness_policy_digest(),
            execution_proof_digest,
            execution_verifier_id: execution.verifier_id,
            execution_trust_domain_id: execution.trust_domain_id,
            result_digest: context.result_digest,
        },
        signatures: Vec::new(),
    })
}

/// Exact language-neutral bytes signed by every release authority.
pub fn release_credential_message(statement: &SifReleaseCredentialStatement) -> Vec<u8> {
    let mut out = Vec::with_capacity(420);
    out.extend_from_slice(RELEASE_CREDENTIAL_DOMAIN);
    out.push(0);
    out.extend_from_slice(SIF_RELEASE_CREDENTIAL_SCHEMA.as_bytes());
    out.push(0);
    out.extend_from_slice(SIF_RELEASE_CREDENTIAL_CODEC.as_bytes());
    out.push(0);
    push_commitment(&mut out, statement.credential_id);
    push_commitment(&mut out, statement.receipt_statement_digest);
    push_commitment(&mut out, statement.pre_witness_bundle_digest);
    push_commitment(&mut out, statement.finalized_evidence_bundle_digest);
    push_commitment(&mut out, statement.accountability_policy_digest);
    push_commitment(&mut out, statement.non_witness_trust_policy_digest);
    push_commitment(&mut out, statement.witness_policy_digest);
    push_commitment(&mut out, statement.execution_proof_digest);
    push_commitment(&mut out, statement.execution_verifier_id);
    push_commitment(&mut out, statement.execution_trust_domain_id);
    match statement.result_digest {
        Some(result) => {
            out.push(1);
            push_commitment(&mut out, result);
        }
        None => out.push(0),
    }
    out
}

/// Stable BLAKE3 identifier for a release-authority Ed25519 public key.
pub fn release_authority_key_id(public_key: &[u8; 32]) -> Commitment32 {
    let mut hasher = blake3::Hasher::new();
    hasher.update(RELEASE_AUTHORITY_KEY_DOMAIN);
    hasher.update(&[0]);
    hasher.update(public_key);
    Commitment32(*hasher.finalize().as_bytes())
}

/// Append one Ed25519 release-authority signature. Duplicate authority roots are
/// rejected so signature count cannot be inflated by repeating the same signer.
pub fn sign_release_credential(
    credential: &mut SifReleaseCredential,
    signing_key: &SigningKey,
) -> Result<(), ReleaseCredentialError> {
    let public_key = signing_key.verifying_key().to_bytes();
    let signer_key_id = release_authority_key_id(&public_key);
    if credential
        .signatures
        .iter()
        .any(|signature| signature.signer_key_id == signer_key_id)
    {
        return Err(ReleaseCredentialError::DuplicateAuthoritySignature);
    }
    let signature = signing_key
        .sign(&release_credential_message(&credential.statement))
        .to_bytes();
    credential.signatures.push(SifReleaseCredentialSignature {
        algorithm: SIF_RELEASE_CREDENTIAL_ED25519.to_string(),
        signer_key_id,
        signature: signature.to_vec(),
    });
    Ok(())
}

/// Verify one signature against an explicitly trusted public key. Xenia adds the
/// deployment-specific threshold and trust-domain policy on top of this primitive.
pub fn verify_release_credential_signature(
    statement: &SifReleaseCredentialStatement,
    signature: &SifReleaseCredentialSignature,
    public_key: &[u8; 32],
) -> Result<(), ReleaseCredentialError> {
    if signature.algorithm != SIF_RELEASE_CREDENTIAL_ED25519 {
        return Err(ReleaseCredentialError::UnsupportedSignatureAlgorithm);
    }
    if signature.signer_key_id != release_authority_key_id(public_key) {
        return Err(ReleaseCredentialError::AuthorityKeyMismatch);
    }
    let signature_bytes: [u8; 64] = signature
        .signature
        .as_slice()
        .try_into()
        .map_err(|_| ReleaseCredentialError::InvalidSignatureLength)?;
    let verifying_key =
        VerifyingKey::from_bytes(public_key).map_err(|_| ReleaseCredentialError::InvalidSignature)?;
    verifying_key
        .verify(
            &release_credential_message(statement),
            &Signature::from_bytes(&signature_bytes),
        )
        .map_err(|_| ReleaseCredentialError::InvalidSignature)
}

fn push_commitment(out: &mut Vec<u8>, commitment: Commitment32) {
    out.extend_from_slice(commitment.as_bytes());
}

#[cfg(test)]
mod tests {
    use super::*;

    fn c(value: u8) -> Commitment32 {
        Commitment32([value; 32])
    }

    fn statement() -> SifReleaseCredentialStatement {
        SifReleaseCredentialStatement {
            schema: SIF_RELEASE_CREDENTIAL_SCHEMA.into(),
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

    #[test]
    fn canonical_message_changes_with_every_security_binding() {
        let base = statement();
        let expected = release_credential_message(&base);
        for index in 0..10 {
            let mut changed = base.clone();
            let value = c(20 + index);
            match index {
                0 => changed.credential_id = value,
                1 => changed.receipt_statement_digest = value,
                2 => changed.pre_witness_bundle_digest = value,
                3 => changed.finalized_evidence_bundle_digest = value,
                4 => changed.accountability_policy_digest = value,
                5 => changed.non_witness_trust_policy_digest = value,
                6 => changed.witness_policy_digest = value,
                7 => changed.execution_proof_digest = value,
                8 => changed.execution_verifier_id = value,
                9 => changed.execution_trust_domain_id = value,
                _ => unreachable!(),
            }
            assert_ne!(expected, release_credential_message(&changed));
        }
        let mut changed_result = base;
        changed_result.result_digest = Some(c(99));
        assert_ne!(expected, release_credential_message(&changed_result));
    }

    #[test]
    fn signatures_verify_and_duplicate_authority_is_rejected() {
        let key = SigningKey::from_bytes(&[42u8; 32]);
        let mut credential = SifReleaseCredential {
            statement: statement(),
            signatures: Vec::new(),
        };
        sign_release_credential(&mut credential, &key).unwrap();
        verify_release_credential_signature(
            &credential.statement,
            &credential.signatures[0],
            &key.verifying_key().to_bytes(),
        )
        .unwrap();
        assert_eq!(
            sign_release_credential(&mut credential, &key),
            Err(ReleaseCredentialError::DuplicateAuthoritySignature)
        );
    }

    #[test]
    fn tampering_after_signature_is_rejected() {
        let key = SigningKey::from_bytes(&[42u8; 32]);
        let mut credential = SifReleaseCredential {
            statement: statement(),
            signatures: Vec::new(),
        };
        sign_release_credential(&mut credential, &key).unwrap();
        credential.statement.finalized_evidence_bundle_digest = c(99);
        assert_eq!(
            verify_release_credential_signature(
                &credential.statement,
                &credential.signatures[0],
                &key.verifying_key().to_bytes(),
            ),
            Err(ReleaseCredentialError::InvalidSignature)
        );
    }
}
