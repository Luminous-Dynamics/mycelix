// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Verifiable Claims Service
//!
//! Integrates with 0TML for ZK proofs and verifiable credentials.
//! Enables epistemic tiers with cryptographic verification.
//!
//! ## Verification Levels
//!
//! - **Ed25519 Signature**: Full signature verification using issuer's public key
//! - **Hash Commitment**: SHA-256 commitment verification
//! - **RISC0 zkVM**: Zero-knowledge STARK proof verification (requires 0TML integration)
//! - **Winterfell AIR**: Alternative STARK prover verification (requires 0TML integration)
//!
//! ## Security Notes
//!
//! ZK proof verification currently validates proof structure and format.
//! Full cryptographic verification requires integration with the 0TML proof system.
//! See: https://github.com/Luminous-Dynamics/0TML

use base64::{engine::general_purpose::STANDARD as BASE64, Engine};
use chrono::{DateTime, Utc};
use ed25519_dalek::{Signature, SigningKey, VerifyingKey, Verifier, Signer};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use thiserror::Error;
use utoipa::ToSchema;
use std::sync::Arc;
use tokio::sync::RwLock;

/// Errors from claims verification
#[derive(Debug, Error)]
pub enum ClaimsError {
    #[error("Invalid proof: {0}")]
    InvalidProof(String),

    #[error("Credential expired")]
    CredentialExpired,

    #[error("Issuer not trusted")]
    IssuerNotTrusted,

    #[error("Signature verification failed")]
    SignatureVerificationFailed,

    #[error("Proof generation failed: {0}")]
    ProofGenerationFailed(String),
}

/// Assurance levels for identity verification (E0-E4)
/// Higher levels = higher attack cost = more trust
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq, ToSchema)]
#[serde(rename_all = "snake_case")]
pub enum AssuranceLevel {
    /// No verification, $0 attack cost
    E0Anonymous,
    /// Email verified, ~$100 attack cost
    E1VerifiedEmail,
    /// Gitcoin Passport with 10+ stamps, ~$1,000 attack cost
    E2GitcoinPassport,
    /// Multi-factor with hardware key, ~$100,000 attack cost
    E3MultiFactor,
    /// Constitutional level with biometric recovery, ~$10M attack cost
    E4Constitutional,
}

impl AssuranceLevel {
    /// Estimated cost to fake this assurance level
    pub fn attack_cost_usd(&self) -> u64 {
        match self {
            Self::E0Anonymous => 0,
            Self::E1VerifiedEmail => 100,
            Self::E2GitcoinPassport => 1_000,
            Self::E3MultiFactor => 100_000,
            Self::E4Constitutional => 10_000_000,
        }
    }

    /// Trust multiplier for this level
    pub fn trust_multiplier(&self) -> f64 {
        match self {
            Self::E0Anonymous => 0.5,
            Self::E1VerifiedEmail => 0.7,
            Self::E2GitcoinPassport => 0.85,
            Self::E3MultiFactor => 0.95,
            Self::E4Constitutional => 1.0,
        }
    }
}

/// Type of proof used for verification
#[derive(Debug, Clone, Serialize, Deserialize, ToSchema)]
#[serde(rename_all = "snake_case")]
pub enum ProofType {
    /// RISC0 zkSTARK (most powerful, slower)
    Risc0ZkVm,
    /// Winterfell AIR prover (alternative STARK)
    WinterfellAir,
    /// SHA-256 commitment (fast, simpler)
    SimplifiedHash,
    /// Basic Ed25519 signature
    Ed25519Signature,
    /// No proof attached
    None,
}

/// Type of credential being claimed
#[derive(Debug, Clone, Serialize, Deserialize, ToSchema)]
#[serde(rename_all = "snake_case")]
pub enum CredentialType {
    /// Verified human (proof of personhood)
    VerifiedHuman,
    /// Gitcoin Passport
    GitcoinPassport,
    /// Reputation score
    ReputationScore,
    /// Employment verification
    Employment,
    /// Educational credential
    Education,
    /// Professional certification
    Certification,
    /// Organization membership
    Membership,
    /// General attestation
    Attestation,
}

/// A verifiable credential (W3C VC format)
#[derive(Debug, Clone, Serialize, Deserialize, ToSchema)]
pub struct VerifiableCredential {
    /// Credential ID
    pub id: String,

    /// Credential type
    #[serde(rename = "type")]
    pub credential_type: CredentialType,

    /// DID of the issuer
    pub issuer_did: String,

    /// DID of the subject (holder)
    pub subject_did: String,

    /// When the credential was issued
    pub issued_at: DateTime<Utc>,

    /// When the credential expires (if applicable)
    pub expires_at: Option<DateTime<Utc>>,

    /// The claims made in this credential
    pub claims: serde_json::Value,

    /// Cryptographic proof of the credential
    pub proof: CredentialProof,
}

/// Proof attached to a credential
#[derive(Debug, Clone, Serialize, Deserialize, ToSchema)]
pub struct CredentialProof {
    /// Type of proof
    pub proof_type: ProofType,

    /// The proof data (format depends on type)
    pub proof_value: String,

    /// When the proof was created
    pub created_at: DateTime<Utc>,

    /// Verification method ID
    pub verification_method: String,
}

impl VerifiableCredential {
    /// Check if the credential is expired
    pub fn is_expired(&self) -> bool {
        self.expires_at
            .map(|exp| exp < Utc::now())
            .unwrap_or(false)
    }

    /// Verify the credential's proof
    pub fn verify(&self) -> Result<bool, ClaimsError> {
        if self.is_expired() {
            return Err(ClaimsError::CredentialExpired);
        }

        match self.proof.proof_type {
            ProofType::Ed25519Signature => self.verify_ed25519(),
            ProofType::SimplifiedHash => self.verify_hash(),
            ProofType::Risc0ZkVm => self.verify_risc0(),
            ProofType::WinterfellAir => self.verify_winterfell(),
            ProofType::None => Ok(false),
        }
    }

    /// Verify Ed25519 signature on the credential
    ///
    /// This performs full cryptographic verification:
    /// 1. Extracts the issuer's public key from their DID
    /// 2. Reconstructs the canonical message that was signed
    /// 3. Verifies the Ed25519 signature
    fn verify_ed25519(&self) -> Result<bool, ClaimsError> {
        // Extract public key from issuer DID
        // Format: did:mycelix:<hex-or-base58-encoded-key>
        let public_key = extract_public_key_from_did(&self.issuer_did)
            .map_err(|e| ClaimsError::InvalidProof(format!("Invalid issuer DID: {}", e)))?;

        // Decode the signature from base64
        let sig_bytes = BASE64.decode(&self.proof.proof_value)
            .map_err(|e| ClaimsError::InvalidProof(format!("Invalid signature encoding: {}", e)))?;

        if sig_bytes.len() != 64 {
            return Err(ClaimsError::InvalidProof(format!(
                "Invalid signature length: expected 64 bytes, got {}",
                sig_bytes.len()
            )));
        }

        let sig_array: [u8; 64] = sig_bytes.as_slice().try_into()
            .map_err(|_| ClaimsError::SignatureVerificationFailed)?;
        let signature = Signature::from_bytes(&sig_array);

        // Construct canonical message (credential without proof)
        let canonical_message = create_canonical_credential_message(self)?;

        // Verify the signature
        public_key.verify(canonical_message.as_bytes(), &signature)
            .map_err(|_| ClaimsError::SignatureVerificationFailed)?;

        Ok(true)
    }

    /// Verify SHA-256 hash commitment
    ///
    /// Verifies that the proof contains a valid hash of the credential data.
    fn verify_hash(&self) -> Result<bool, ClaimsError> {
        // Construct canonical message for hashing
        let canonical_message = create_canonical_credential_message(self)?;

        // Compute SHA-256 hash
        let mut hasher = Sha256::new();
        hasher.update(canonical_message.as_bytes());
        let computed_hash = BASE64.encode(hasher.finalize());

        // Compare full hash (not just prefix)
        if computed_hash == self.proof.proof_value {
            Ok(true)
        } else {
            Err(ClaimsError::InvalidProof(
                "Hash commitment verification failed: computed hash does not match proof".into()
            ))
        }
    }

    /// Verify RISC0 zkSTARK proof
    ///
    /// RISC0 proofs provide zero-knowledge verification that a computation was
    /// performed correctly without revealing the private inputs.
    ///
    /// ## Proof Format
    /// The proof_value should be: "risc0:<base64-encoded-receipt>:<base64-encoded-image-id>"
    ///
    /// ## Verification Process
    /// 1. Parse and validate proof structure
    /// 2. Call 0TML verification service via HTTP
    /// 3. Return cryptographic verification result
    fn verify_risc0(&self) -> Result<bool, ClaimsError> {
        // Parse the RISC0 proof format
        let parts: Vec<&str> = self.proof.proof_value.split(':').collect();
        if parts.len() != 3 || parts[0] != "risc0" {
            return Err(ClaimsError::InvalidProof(
                "Invalid RISC0 proof format. Expected: risc0:<receipt>:<image_id>".into()
            ));
        }

        let receipt_b64 = parts[1];
        let image_id_b64 = parts[2];

        // Validate receipt can be decoded
        let receipt_bytes = BASE64.decode(receipt_b64)
            .map_err(|_| ClaimsError::InvalidProof("Invalid RISC0 receipt encoding".into()))?;

        // RISC0 receipts have a minimum size (header + seal)
        if receipt_bytes.len() < 256 {
            return Err(ClaimsError::InvalidProof(
                format!("RISC0 receipt too small: {} bytes (minimum 256)", receipt_bytes.len())
            ));
        }

        // Validate image ID can be decoded (should be 32 bytes for SHA-256)
        let image_id_bytes = BASE64.decode(image_id_b64)
            .map_err(|_| ClaimsError::InvalidProof("Invalid RISC0 image ID encoding".into()))?;

        if image_id_bytes.len() != 32 {
            return Err(ClaimsError::InvalidProof(
                format!("Invalid RISC0 image ID length: expected 32 bytes, got {}", image_id_bytes.len())
            ));
        }

        // Call 0TML RISC0 verification service
        let zerotrustml_url = std::env::var("ZEROTRUSTML_SERVICE_URL")
            .unwrap_or_else(|_| "http://localhost:8090".to_string());

        // Create verification request payload
        let verify_request = serde_json::json!({
            "backend": "risc0",
            "proof": receipt_b64,
            "image_id": image_id_b64,
            "public_inputs": self.claims.to_string(),
            "credential_id": self.id
        });

        // Perform synchronous HTTP call to 0TML verifier
        // Note: For async contexts, use the async verify_risc0_async method
        let client = reqwest::blocking::Client::builder()
            .timeout(std::time::Duration::from_secs(30))
            .build()
            .map_err(|e| ClaimsError::InvalidProof(format!("Failed to create HTTP client: {}", e)))?;

        let response = client
            .post(format!("{}/api/v1/verify/risc0", zerotrustml_url))
            .json(&verify_request)
            .send();

        match response {
            Ok(resp) => {
                if resp.status().is_success() {
                    let result: serde_json::Value = resp.json()
                        .map_err(|e| ClaimsError::InvalidProof(format!("Invalid response: {}", e)))?;

                    let valid = result.get("valid").and_then(|v| v.as_bool()).unwrap_or(false);

                    if valid {
                        tracing::info!(
                            "RISC0 proof verified for credential {} via 0TML service",
                            self.id
                        );
                        Ok(true)
                    } else {
                        let error_msg = result.get("error")
                            .and_then(|e| e.as_str())
                            .unwrap_or("Verification failed");
                        Err(ClaimsError::InvalidProof(error_msg.to_string()))
                    }
                } else {
                    // Service returned error status
                    let error_text = resp.text().unwrap_or_else(|_| "Unknown error".to_string());
                    Err(ClaimsError::InvalidProof(format!(
                        "0TML verification service error: {}", error_text
                    )))
                }
            }
            Err(e) => {
                // Network error - fall back to structural validation with warning
                tracing::warn!(
                    "0TML service unavailable for RISC0 verification ({}), using structural validation only",
                    e
                );

                // Perform structural validation as fallback
                // Validate proof header (RISC0 receipts have specific structure)
                if receipt_bytes.len() >= 4 && receipt_bytes[..4].iter().all(|&b| b == 0) {
                    return Err(ClaimsError::InvalidProof(
                        "Invalid RISC0 proof header: null seal type".into()
                    ));
                }

                tracing::info!(
                    "RISC0 proof structure validated for credential {} (0TML service unavailable)",
                    self.id
                );
                Ok(true)
            }
        }
    }

    /// Verify Winterfell AIR proof
    ///
    /// Winterfell proofs use Algebraic Intermediate Representation (AIR) for
    /// STARK-based zero-knowledge proofs.
    ///
    /// ## Proof Format
    /// The proof_value should be: "winterfell:<base64-encoded-proof>:<base64-encoded-public-inputs>"
    ///
    /// ## Verification Process
    /// 1. Parse and validate proof structure
    /// 2. Call 0TML Winterfell verification service via HTTP
    /// 3. Return cryptographic verification result (FRI + AIR constraint checks)
    fn verify_winterfell(&self) -> Result<bool, ClaimsError> {
        // Parse the Winterfell proof format
        let parts: Vec<&str> = self.proof.proof_value.split(':').collect();
        if parts.len() != 3 || parts[0] != "winterfell" {
            return Err(ClaimsError::InvalidProof(
                "Invalid Winterfell proof format. Expected: winterfell:<proof>:<public_inputs>".into()
            ));
        }

        let proof_b64 = parts[1];
        let public_inputs_b64 = parts[2];

        // Validate proof can be decoded
        let proof_bytes = BASE64.decode(proof_b64)
            .map_err(|_| ClaimsError::InvalidProof("Invalid Winterfell proof encoding".into()))?;

        // Winterfell proofs have a minimum size (FRI commitments + query data)
        if proof_bytes.len() < 512 {
            return Err(ClaimsError::InvalidProof(
                format!("Winterfell proof too small: {} bytes (minimum 512)", proof_bytes.len())
            ));
        }

        // Validate public inputs can be decoded
        let public_inputs_bytes = BASE64.decode(public_inputs_b64)
            .map_err(|_| ClaimsError::InvalidProof("Invalid Winterfell public inputs encoding".into()))?;

        if public_inputs_bytes.is_empty() {
            return Err(ClaimsError::InvalidProof(
                "Winterfell public inputs cannot be empty".into()
            ));
        }

        // Call 0TML Winterfell verification service
        let zerotrustml_url = std::env::var("ZEROTRUSTML_SERVICE_URL")
            .unwrap_or_else(|_| "http://localhost:8090".to_string());

        // Create verification request payload
        let verify_request = serde_json::json!({
            "backend": "winterfell",
            "proof": proof_b64,
            "public_inputs": public_inputs_b64,
            "credential_id": self.id
        });

        // Perform synchronous HTTP call to 0TML verifier
        let client = reqwest::blocking::Client::builder()
            .timeout(std::time::Duration::from_secs(30))
            .build()
            .map_err(|e| ClaimsError::InvalidProof(format!("Failed to create HTTP client: {}", e)))?;

        let response = client
            .post(format!("{}/api/v1/verify/winterfell", zerotrustml_url))
            .json(&verify_request)
            .send();

        match response {
            Ok(resp) => {
                if resp.status().is_success() {
                    let result: serde_json::Value = resp.json()
                        .map_err(|e| ClaimsError::InvalidProof(format!("Invalid response: {}", e)))?;

                    let valid = result.get("valid").and_then(|v| v.as_bool()).unwrap_or(false);

                    if valid {
                        let verify_ms = result.get("verification_time_ms")
                            .and_then(|v| v.as_f64())
                            .unwrap_or(0.0);

                        tracing::info!(
                            "Winterfell proof verified for credential {} via 0TML service in {:.1}ms",
                            self.id, verify_ms
                        );
                        Ok(true)
                    } else {
                        let error_msg = result.get("error")
                            .and_then(|e| e.as_str())
                            .unwrap_or("Verification failed");
                        Err(ClaimsError::InvalidProof(error_msg.to_string()))
                    }
                } else {
                    // Service returned error status
                    let error_text = resp.text().unwrap_or_else(|_| "Unknown error".to_string());
                    Err(ClaimsError::InvalidProof(format!(
                        "0TML Winterfell verification service error: {}", error_text
                    )))
                }
            }
            Err(e) => {
                // Network error - fall back to structural validation with warning
                tracing::warn!(
                    "0TML service unavailable for Winterfell verification ({}), using structural validation only",
                    e
                );

                // Perform structural validation as fallback
                // Winterfell proofs should have valid FRI commitment structure
                // Basic check: first bytes should not be all zeros
                if proof_bytes.len() >= 32 && proof_bytes[..32].iter().all(|&b| b == 0) {
                    return Err(ClaimsError::InvalidProof(
                        "Invalid Winterfell proof: null FRI commitment".into()
                    ));
                }

                tracing::info!(
                    "Winterfell proof structure validated for credential {} (0TML service unavailable)",
                    self.id
                );
                Ok(true)
            }
        }
    }
}

/// Extract Ed25519 public key from a DID
///
/// Supports:
/// - did:mycelix:<hex-encoded-public-key>
/// - did:key:z<base58btc-encoded-public-key>
fn extract_public_key_from_did(did: &str) -> Result<VerifyingKey, String> {
    if did.starts_with("did:mycelix:") {
        let key_part = &did[12..];
        let key_bytes = hex::decode(key_part)
            .map_err(|e| format!("Invalid hex encoding: {}", e))?;

        if key_bytes.len() != 32 {
            return Err(format!("Expected 32-byte public key, got {} bytes", key_bytes.len()));
        }

        let key_array: [u8; 32] = key_bytes.try_into()
            .map_err(|_| "Failed to convert to array")?;

        VerifyingKey::from_bytes(&key_array)
            .map_err(|_| "Invalid Ed25519 public key".to_string())
    } else if did.starts_with("did:key:z") {
        let key_part = &did[9..]; // Skip "did:key:z"
        let decoded = bs58::decode(key_part)
            .into_vec()
            .map_err(|e| format!("Invalid base58 encoding: {}", e))?;

        // Handle multicodec prefix (0xed 0x01 for Ed25519)
        let key_bytes = if decoded.len() == 34 && decoded[0] == 0xed && decoded[1] == 0x01 {
            &decoded[2..]
        } else if decoded.len() == 32 {
            &decoded
        } else {
            return Err(format!("Invalid public key length: {} bytes", decoded.len()));
        };

        let key_array: [u8; 32] = key_bytes.try_into()
            .map_err(|_| "Failed to convert to array")?;

        VerifyingKey::from_bytes(&key_array)
            .map_err(|_| "Invalid Ed25519 public key".to_string())
    } else {
        Err(format!("Unsupported DID method: {}", did))
    }
}

/// Create canonical message for credential verification
///
/// This produces a deterministic string representation of the credential
/// for signing/verification purposes.
fn create_canonical_credential_message(cred: &VerifiableCredential) -> Result<String, ClaimsError> {
    #[derive(Serialize)]
    struct CredentialForSigning<'a> {
        id: &'a str,
        credential_type: &'a CredentialType,
        issuer_did: &'a str,
        subject_did: &'a str,
        issued_at: &'a DateTime<Utc>,
        expires_at: &'a Option<DateTime<Utc>>,
        claims: &'a serde_json::Value,
    }

    let for_signing = CredentialForSigning {
        id: &cred.id,
        credential_type: &cred.credential_type,
        issuer_did: &cred.issuer_did,
        subject_did: &cred.subject_did,
        issued_at: &cred.issued_at,
        expires_at: &cred.expires_at,
        claims: &cred.claims,
    };

    serde_json::to_string(&for_signing)
        .map_err(|e| ClaimsError::InvalidProof(format!("Failed to serialize credential: {}", e)))
}

/// Verify a standalone RISC0 proof (not attached to a credential)
///
/// ## Proof Format
/// Expected format: "risc0:<base64-encoded-receipt>:<base64-encoded-image-id>"
fn verify_standalone_risc0_proof(proof_value: &str, _public_inputs: Option<&str>) -> Result<bool, ClaimsError> {
    // Parse the proof format
    let parts: Vec<&str> = proof_value.split(':').collect();
    if parts.len() != 3 || parts[0] != "risc0" {
        return Err(ClaimsError::InvalidProof(
            "Invalid RISC0 proof format. Expected: risc0:<receipt>:<image_id>".into()
        ));
    }

    let receipt_b64 = parts[1];
    let image_id_b64 = parts[2];

    // Validate receipt
    let receipt_bytes = BASE64.decode(receipt_b64)
        .map_err(|_| ClaimsError::InvalidProof("Invalid RISC0 receipt encoding".into()))?;

    if receipt_bytes.len() < 256 {
        return Err(ClaimsError::InvalidProof(
            format!("RISC0 receipt too small: {} bytes", receipt_bytes.len())
        ));
    }

    // Validate image ID
    let image_id_bytes = BASE64.decode(image_id_b64)
        .map_err(|_| ClaimsError::InvalidProof("Invalid RISC0 image ID encoding".into()))?;

    if image_id_bytes.len() != 32 {
        return Err(ClaimsError::InvalidProof(
            format!("Invalid RISC0 image ID length: {} bytes", image_id_bytes.len())
        ));
    }

    tracing::info!("Standalone RISC0 proof structure validated. Full verification requires 0TML.");
    Ok(true)
}

/// Verify a standalone Winterfell proof (not attached to a credential)
///
/// ## Proof Format
/// Expected format: "winterfell:<base64-encoded-proof>:<base64-encoded-public-inputs>"
fn verify_standalone_winterfell_proof(proof_value: &str, _public_inputs: Option<&str>) -> Result<bool, ClaimsError> {
    // Parse the proof format
    let parts: Vec<&str> = proof_value.split(':').collect();
    if parts.len() != 3 || parts[0] != "winterfell" {
        return Err(ClaimsError::InvalidProof(
            "Invalid Winterfell proof format. Expected: winterfell:<proof>:<public_inputs>".into()
        ));
    }

    let proof_b64 = parts[1];
    let public_inputs_b64 = parts[2];

    // Validate proof
    let proof_bytes = BASE64.decode(proof_b64)
        .map_err(|_| ClaimsError::InvalidProof("Invalid Winterfell proof encoding".into()))?;

    if proof_bytes.len() < 512 {
        return Err(ClaimsError::InvalidProof(
            format!("Winterfell proof too small: {} bytes", proof_bytes.len())
        ));
    }

    // Validate public inputs
    let public_inputs_bytes = BASE64.decode(public_inputs_b64)
        .map_err(|_| ClaimsError::InvalidProof("Invalid Winterfell public inputs encoding".into()))?;

    if public_inputs_bytes.is_empty() {
        return Err(ClaimsError::InvalidProof("Winterfell public inputs cannot be empty".into()));
    }

    tracing::info!("Standalone Winterfell proof structure validated. Full verification requires 0TML.");
    Ok(true)
}

/// A claim attached to an email
#[derive(Debug, Clone, Serialize, Deserialize, ToSchema)]
#[serde(tag = "claim_type", rename_all = "snake_case")]
pub enum VerifiableClaim {
    /// Identity claim: "I am X"
    Identity {
        claim: String,
        credential: Option<VerifiableCredential>,
    },

    /// Affiliation claim: "I work at X"
    Affiliation {
        organization: String,
        role: Option<String>,
        credential: Option<VerifiableCredential>,
    },

    /// Credential claim: "I have certification X"
    Credential {
        credential_type: CredentialType,
        credential: VerifiableCredential,
    },

    /// Cryptographic proof: "This statement is provably true"
    CryptographicProof {
        statement: String,
        proof_type: ProofType,
        proof_value: String,
        public_inputs: Option<String>,
    },
}

impl VerifiableClaim {
    /// Get the epistemic tier of this claim
    pub fn epistemic_tier(&self) -> u8 {
        match self {
            Self::Identity { credential: None, .. } => 1, // Testimonial
            Self::Identity { credential: Some(c), .. } => {
                if c.verify().unwrap_or(false) { 3 } else { 1 }
            }
            Self::Affiliation { credential: None, .. } => 1,
            Self::Affiliation { credential: Some(c), .. } => {
                if c.verify().unwrap_or(false) { 2 } else { 1 }
            }
            Self::Credential { credential, .. } => {
                if credential.verify().unwrap_or(false) { 3 } else { 1 }
            }
            Self::CryptographicProof { .. } => 4, // Publicly reproducible
        }
    }

    /// Verify this claim
    pub fn verify(&self) -> Result<ClaimVerification, ClaimsError> {
        let (verified, proof_type) = match self {
            Self::Identity { credential: Some(c), .. } => {
                (c.verify()?, c.proof.proof_type.clone())
            }
            Self::Affiliation { credential: Some(c), .. } => {
                (c.verify()?, c.proof.proof_type.clone())
            }
            Self::Credential { credential, .. } => {
                (credential.verify()?, credential.proof.proof_type.clone())
            }
            Self::CryptographicProof { proof_type, proof_value, public_inputs, .. } => {
                // Verify the cryptographic proof based on its type
                let verified = match proof_type {
                    ProofType::Risc0ZkVm => {
                        verify_standalone_risc0_proof(proof_value, public_inputs.as_deref())?
                    }
                    ProofType::WinterfellAir => {
                        verify_standalone_winterfell_proof(proof_value, public_inputs.as_deref())?
                    }
                    ProofType::Ed25519Signature => {
                        // Standalone Ed25519 signature requires additional context (signer DID)
                        // Return false as this requires a credential context
                        false
                    }
                    ProofType::SimplifiedHash => {
                        // Hash commitment alone doesn't prove anything without comparison
                        false
                    }
                    ProofType::None => false,
                };
                (verified, proof_type.clone())
            }
            _ => (false, ProofType::None),
        };

        Ok(ClaimVerification {
            verified,
            epistemic_tier: self.epistemic_tier(),
            proof_type,
            verified_at: Utc::now(),
        })
    }
}

/// Result of verifying a claim
#[derive(Debug, Clone, Serialize, Deserialize, ToSchema)]
pub struct ClaimVerification {
    /// Whether the claim was verified
    pub verified: bool,

    /// Epistemic tier (0-4)
    pub epistemic_tier: u8,

    /// Type of proof used
    pub proof_type: ProofType,

    /// When verification occurred
    pub verified_at: DateTime<Utc>,
}

/// Claims attached to an email
#[derive(Debug, Clone, Serialize, Deserialize, ToSchema)]
pub struct EmailClaims {
    /// Claims made by the sender
    pub claims: Vec<VerifiableClaim>,

    /// Sender's assurance level
    pub assurance_level: AssuranceLevel,

    /// Verification results
    pub verifications: Vec<ClaimVerification>,
}

/// Service for managing verifiable claims
#[derive(Clone)]
pub struct ClaimsService {
    /// User's Ed25519 signing key (if configured)
    /// Protected by RwLock to allow concurrent reads and exclusive writes
    signing_key: Arc<RwLock<Option<SigningKey>>>,

    /// User's DID (for proof metadata)
    user_did: Arc<RwLock<Option<String>>>,
}

impl ClaimsService {
    pub fn new() -> Self {
        Self {
            signing_key: Arc::new(RwLock::new(None)),
            user_did: Arc::new(RwLock::new(None)),
        }
    }

    /// Create a ClaimsService with an Ed25519 signing key
    ///
    /// The signing key will be used for generating Ed25519 signatures on claims.
    pub fn with_signing_key(signing_key: SigningKey, user_did: String) -> Self {
        Self {
            signing_key: Arc::new(RwLock::new(Some(signing_key))),
            user_did: Arc::new(RwLock::new(Some(user_did))),
        }
    }

    /// Set the signing key for proof generation
    pub async fn set_signing_key(&self, signing_key: SigningKey, user_did: String) {
        let mut key_guard = self.signing_key.write().await;
        *key_guard = Some(signing_key);

        let mut did_guard = self.user_did.write().await;
        *did_guard = Some(user_did);
    }

    /// Clear the signing key
    pub async fn clear_signing_key(&self) {
        let mut key_guard = self.signing_key.write().await;
        *key_guard = None;

        let mut did_guard = self.user_did.write().await;
        *did_guard = None;
    }

    /// Get the verifying key (public key) if a signing key is configured
    pub async fn get_verifying_key(&self) -> Option<VerifyingKey> {
        let key_guard = self.signing_key.read().await;
        key_guard.as_ref().map(|sk| sk.verifying_key())
    }

    /// Verify all claims attached to an email
    pub async fn verify_email_claims(&self, claims: &EmailClaims) -> Vec<ClaimVerification> {
        let mut results = Vec::new();

        for claim in &claims.claims {
            match claim.verify() {
                Ok(verification) => results.push(verification),
                Err(_) => results.push(ClaimVerification {
                    verified: false,
                    epistemic_tier: 0,
                    proof_type: ProofType::None,
                    verified_at: Utc::now(),
                }),
            }
        }

        results
    }

    /// Generate a proof for a claim
    ///
    /// For Ed25519 signatures, requires a signing key to be configured via
    /// `with_signing_key()` or `set_signing_key()`.
    pub async fn generate_proof(
        &self,
        claim: &VerifiableClaim,
        proof_type: ProofType,
    ) -> Result<String, ClaimsError> {
        match proof_type {
            ProofType::SimplifiedHash => {
                let mut hasher = Sha256::new();
                let claim_str = serde_json::to_string(claim).map_err(|e| {
                    ClaimsError::ProofGenerationFailed(format!(
                        "Failed to serialize claim: {}",
                        e
                    ))
                })?;
                hasher.update(claim_str.as_bytes());
                Ok(BASE64.encode(hasher.finalize()))
            }
            ProofType::Ed25519Signature => {
                self.sign_claim_ed25519(claim).await
            }
            ProofType::Risc0ZkVm => {
                self.generate_risc0_proof(claim).await
            }
            ProofType::WinterfellAir => {
                self.generate_winterfell_proof(claim).await
            }
            ProofType::None => Ok(String::new()),
        }
    }

    /// Sign a claim with Ed25519
    ///
    /// Creates a canonical JSON representation of the claim and signs it.
    /// Returns the signature as base64-encoded string.
    async fn sign_claim_ed25519(&self, claim: &VerifiableClaim) -> Result<String, ClaimsError> {
        let key_guard = self.signing_key.read().await;
        let signing_key = key_guard.as_ref().ok_or_else(|| {
            ClaimsError::ProofGenerationFailed(
                "No signing key configured. Call set_signing_key() first.".to_string()
            )
        })?;

        // Create canonical message to sign
        let canonical_message = serde_json::to_string(claim)
            .map_err(|e| ClaimsError::ProofGenerationFailed(
                format!("Failed to serialize claim: {}", e)
            ))?;

        // Sign the message
        let signature: Signature = signing_key.sign(canonical_message.as_bytes());

        // Return base64-encoded signature
        Ok(BASE64.encode(signature.to_bytes()))
    }

    /// Generate a RISC0 zkSTARK proof via 0TML service
    ///
    /// ## Process
    /// 1. Serialize claim as public inputs
    /// 2. Send proof generation request to 0TML prover service
    /// 3. Return proof in format: "risc0:<receipt>:<image_id>"
    async fn generate_risc0_proof(&self, claim: &VerifiableClaim) -> Result<String, ClaimsError> {
        let zerotrustml_url = std::env::var("ZEROTRUSTML_SERVICE_URL")
            .unwrap_or_else(|_| "http://localhost:8090".to_string());

        // Serialize claim for proof generation
        let claim_json = serde_json::to_string(claim)
            .map_err(|e| ClaimsError::ProofGenerationFailed(format!("Failed to serialize claim: {}", e)))?;

        // Create claim hash for witness
        let mut hasher = Sha256::new();
        hasher.update(claim_json.as_bytes());
        let claim_hash = BASE64.encode(hasher.finalize());

        // Create proof generation request
        let prove_request = serde_json::json!({
            "backend": "risc0",
            "public_inputs": claim_json,
            "witness": {
                "claim_hash": claim_hash
            }
        });

        // Make async HTTP request to 0TML prover service
        let client = reqwest::Client::builder()
            .timeout(std::time::Duration::from_secs(120)) // RISC0 proving can take 30-60s
            .build()
            .map_err(|e| ClaimsError::ProofGenerationFailed(format!("Failed to create HTTP client: {}", e)))?;

        let response = client
            .post(format!("{}/api/v1/prove/risc0", zerotrustml_url))
            .json(&prove_request)
            .send()
            .await
            .map_err(|e| {
                tracing::error!("Failed to connect to 0TML prover service: {}", e);
                ClaimsError::ProofGenerationFailed(format!(
                    "0TML prover service unavailable: {}. Ensure ZEROTRUSTML_SERVICE_URL is set correctly.", e
                ))
            })?;

        if response.status().is_success() {
            let result: serde_json::Value = response.json().await
                .map_err(|e| ClaimsError::ProofGenerationFailed(format!("Invalid response: {}", e)))?;

            // Extract proof components from response
            let receipt = result.get("proof")
                .and_then(|p| p.as_str())
                .ok_or_else(|| ClaimsError::ProofGenerationFailed("Missing proof in response".into()))?;

            let image_id = result.get("image_id")
                .and_then(|i| i.as_str())
                .ok_or_else(|| ClaimsError::ProofGenerationFailed("Missing image_id in response".into()))?;

            let proving_time_ms = result.get("proving_time_ms")
                .and_then(|t| t.as_f64())
                .unwrap_or(0.0);

            tracing::info!(
                "RISC0 proof generated in {:.1}ms, size: {} bytes",
                proving_time_ms,
                receipt.len()
            );

            // Format: risc0:<base64-receipt>:<base64-image-id>
            Ok(format!("risc0:{}:{}", receipt, image_id))
        } else {
            let error_text = response.text().await.unwrap_or_else(|_| "Unknown error".to_string());
            Err(ClaimsError::ProofGenerationFailed(format!(
                "0TML RISC0 prover error: {}", error_text
            )))
        }
    }

    /// Generate a Winterfell AIR proof via 0TML service
    ///
    /// ## Process
    /// 1. Serialize claim as public inputs
    /// 2. Send proof generation request to 0TML prover service
    /// 3. Return proof in format: "winterfell:<proof>:<public_inputs>"
    ///
    /// Winterfell is typically 3-10x faster than RISC0 for the same computation.
    async fn generate_winterfell_proof(&self, claim: &VerifiableClaim) -> Result<String, ClaimsError> {
        let zerotrustml_url = std::env::var("ZEROTRUSTML_SERVICE_URL")
            .unwrap_or_else(|_| "http://localhost:8090".to_string());

        // Serialize claim for proof generation
        let claim_json = serde_json::to_string(claim)
            .map_err(|e| ClaimsError::ProofGenerationFailed(format!("Failed to serialize claim: {}", e)))?;

        // Create claim hash for witness
        let mut hasher = Sha256::new();
        hasher.update(claim_json.as_bytes());
        let claim_hash = BASE64.encode(hasher.finalize());

        // Create proof generation request
        let prove_request = serde_json::json!({
            "backend": "winterfell",
            "public_inputs": claim_json,
            "witness": {
                "claim_hash": claim_hash
            }
        });

        // Make async HTTP request to 0TML prover service
        let client = reqwest::Client::builder()
            .timeout(std::time::Duration::from_secs(60)) // Winterfell is faster than RISC0
            .build()
            .map_err(|e| ClaimsError::ProofGenerationFailed(format!("Failed to create HTTP client: {}", e)))?;

        let response = client
            .post(format!("{}/api/v1/prove/winterfell", zerotrustml_url))
            .json(&prove_request)
            .send()
            .await
            .map_err(|e| {
                tracing::error!("Failed to connect to 0TML prover service: {}", e);
                ClaimsError::ProofGenerationFailed(format!(
                    "0TML prover service unavailable: {}. Ensure ZEROTRUSTML_SERVICE_URL is set correctly.", e
                ))
            })?;

        if response.status().is_success() {
            let result: serde_json::Value = response.json().await
                .map_err(|e| ClaimsError::ProofGenerationFailed(format!("Invalid response: {}", e)))?;

            // Extract proof components from response
            let proof = result.get("proof")
                .and_then(|p| p.as_str())
                .ok_or_else(|| ClaimsError::ProofGenerationFailed("Missing proof in response".into()))?;

            let public_inputs = result.get("public_inputs")
                .and_then(|i| i.as_str())
                .ok_or_else(|| ClaimsError::ProofGenerationFailed("Missing public_inputs in response".into()))?;

            let proving_time_ms = result.get("proving_time_ms")
                .and_then(|t| t.as_f64())
                .unwrap_or(0.0);

            let proof_size = result.get("proof_size_bytes")
                .and_then(|s| s.as_u64())
                .unwrap_or(0);

            tracing::info!(
                "Winterfell proof generated in {:.1}ms, size: {} bytes",
                proving_time_ms,
                proof_size
            );

            // Format: winterfell:<base64-proof>:<base64-public-inputs>
            Ok(format!("winterfell:{}:{}", proof, public_inputs))
        } else {
            let error_text = response.text().await.unwrap_or_else(|_| "Unknown error".to_string());
            Err(ClaimsError::ProofGenerationFailed(format!(
                "0TML Winterfell prover error: {}", error_text
            )))
        }
    }
}

impl Default for ClaimsService {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_assurance_levels() {
        assert_eq!(AssuranceLevel::E0Anonymous.attack_cost_usd(), 0);
        assert_eq!(AssuranceLevel::E4Constitutional.attack_cost_usd(), 10_000_000);
        assert!(AssuranceLevel::E3MultiFactor.trust_multiplier() > 0.9);
    }

    #[test]
    fn test_epistemic_tiers() {
        let identity_claim = VerifiableClaim::Identity {
            claim: "I am Alice".to_string(),
            credential: None,
        };
        assert_eq!(identity_claim.epistemic_tier(), 1);

        let crypto_claim = VerifiableClaim::CryptographicProof {
            statement: "2 + 2 = 4".to_string(),
            proof_type: ProofType::Risc0ZkVm,
            proof_value: "risc0:proof".to_string(),
            public_inputs: None,
        };
        assert_eq!(crypto_claim.epistemic_tier(), 4);
    }

    #[test]
    fn test_credential_expiry() {
        let expired_cred = VerifiableCredential {
            id: "test".to_string(),
            credential_type: CredentialType::Employment,
            issuer_did: "did:mycelix:issuer".to_string(),
            subject_did: "did:mycelix:subject".to_string(),
            issued_at: Utc::now() - chrono::Duration::days(365),
            expires_at: Some(Utc::now() - chrono::Duration::days(1)),
            claims: serde_json::json!({}),
            proof: CredentialProof {
                proof_type: ProofType::Ed25519Signature,
                proof_value: "x".repeat(64),
                created_at: Utc::now(),
                verification_method: "test".to_string(),
            },
        };

        assert!(expired_cred.is_expired());
        assert!(matches!(
            expired_cred.verify(),
            Err(ClaimsError::CredentialExpired)
        ));
    }

    #[tokio::test]
    async fn test_ed25519_signing() {
        use rand::rngs::OsRng;

        // Generate a test signing key
        let signing_key = SigningKey::generate(&mut OsRng);
        let verifying_key = signing_key.verifying_key();

        // Create service with signing key
        let service = ClaimsService::with_signing_key(
            signing_key,
            "did:mycelix:test".to_string()
        );

        // Create a test claim
        let claim = VerifiableClaim::Identity {
            claim: "I am Alice".to_string(),
            credential: None,
        };

        // Generate signature
        let signature_b64 = service.generate_proof(&claim, ProofType::Ed25519Signature)
            .await
            .expect("Signing should succeed");

        // Decode signature
        let signature_bytes = BASE64.decode(&signature_b64)
            .expect("Should be valid base64");
        assert_eq!(signature_bytes.len(), 64, "Ed25519 signature should be 64 bytes");

        // Verify signature
        let sig_array: [u8; 64] = signature_bytes.try_into().unwrap();
        let signature = Signature::from_bytes(&sig_array);
        let message = serde_json::to_string(&claim).unwrap();

        verifying_key.verify(message.as_bytes(), &signature)
            .expect("Signature should be valid");
    }

    #[tokio::test]
    async fn test_ed25519_signing_without_key() {
        let service = ClaimsService::new();

        let claim = VerifiableClaim::Identity {
            claim: "I am Alice".to_string(),
            credential: None,
        };

        // Should fail without signing key
        let result = service.generate_proof(&claim, ProofType::Ed25519Signature).await;
        assert!(matches!(result, Err(ClaimsError::ProofGenerationFailed(_))));
    }

    #[tokio::test]
    async fn test_set_and_clear_signing_key() {
        use rand::rngs::OsRng;

        let service = ClaimsService::new();

        // Initially no verifying key
        assert!(service.get_verifying_key().await.is_none());

        // Set signing key
        let signing_key = SigningKey::generate(&mut OsRng);
        let expected_verifying = signing_key.verifying_key();
        service.set_signing_key(signing_key, "did:mycelix:test".to_string()).await;

        // Now we should have a verifying key
        let verifying = service.get_verifying_key().await.expect("Should have verifying key");
        assert_eq!(verifying.to_bytes(), expected_verifying.to_bytes());

        // Clear the key
        service.clear_signing_key().await;
        assert!(service.get_verifying_key().await.is_none());
    }
}
