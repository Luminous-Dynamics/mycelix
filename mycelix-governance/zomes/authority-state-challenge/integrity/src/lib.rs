// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Integrity for authority-state freshness challenges.
//!
//! Raw host entropy is private source-chain data. Public challenge records expose
//! only the entropy action reference and its domain-separated digest. Neither
//! entry becomes authority merely because it exists; the coordinator must
//! re-resolve the current policy context and re-verify the private entropy proof.

use hdi::prelude::*;
use mycelix_authority_freshness::AuthoritySubjectRef;
use mycelix_authority_state_coverage_context::{
    CoverageChallenge, CONTEXT_POLICY_PROFILE,
};
use mycelix_institutional_core::Digest32;

pub const CHALLENGE_RUNTIME_PROTOCOL: &str = "mycelix-authority-state-challenge-runtime-v0.1";
pub const ENTROPY_DIGEST_PROFILE: &str =
    "mycelix-authority-state-challenge-entropy-v1-blake3-framed";
const DOMAIN_ENTROPY: &[u8] = b"mycelix/authority-state/challenge-entropy/v1";
const ENTROPY_BYTES: usize = 32;
const MAX_REF_BYTES: usize = 2048;

/// Private-by-construction entropy provenance. These bytes MUST NOT be published
/// to the DHT. The create action hash becomes the public randomness-proof ref.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct ChallengeEntropyRecord {
    pub protocol_version: String,
    pub subject: AuthoritySubjectRef,
    pub context_policy_digest: Digest32,
    pub context_policy_profile: String,
    pub coverage_policy_digest: Digest32,
    pub coverage_policy_profile: String,
    pub entropy: Vec<u8>,
    pub nonce_digest: Digest32,
    pub issued_by: String,
}

impl ChallengeEntropyRecord {
    pub fn validate_structure(&self) -> Result<(), String> {
        if self.protocol_version != CHALLENGE_RUNTIME_PROTOCOL {
            return Err("wrong challenge entropy protocol".into());
        }
        self.subject
            .validate()
            .map_err(|_| "invalid challenge entropy subject".to_string())?;
        if self.context_policy_digest.is_zero() || self.coverage_policy_digest.is_zero() {
            return Err("challenge entropy policy digests must be non-zero".into());
        }
        if self.context_policy_profile != CONTEXT_POLICY_PROFILE {
            return Err("challenge entropy context profile mismatch".into());
        }
        validate_ref(&self.coverage_policy_profile, "coverage policy profile")?;
        if self.entropy.len() != ENTROPY_BYTES {
            return Err(format!("challenge entropy must be exactly {ENTROPY_BYTES} bytes"));
        }
        if self.nonce_digest != entropy_nonce_digest(&self.entropy) {
            return Err("challenge entropy digest does not recompute exactly".into());
        }
        require_mycelix_did(&self.issued_by, "challenge entropy issuer")
    }
}

/// Public challenge provenance. The private entropy is referenced by action hash,
/// never copied into this entry.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct AuthorityStateChallengeRecord {
    pub protocol_version: String,
    pub challenge: CoverageChallenge,
    pub entropy_action: ActionHash,
    pub context_verification_ref: String,
}

impl AuthorityStateChallengeRecord {
    pub fn validate_structure(&self) -> Result<(), String> {
        if self.protocol_version != CHALLENGE_RUNTIME_PROTOCOL {
            return Err("wrong public challenge runtime protocol".into());
        }
        self.challenge
            .validate()
            .map_err(|error| format!("invalid coverage challenge: {error}"))?;
        validate_ref(&self.context_verification_ref, "context verification ref")?;
        if self.challenge.randomness_proof_ref != self.entropy_action.to_string() {
            return Err("challenge randomness proof must equal private entropy action ref".into());
        }
        require_mycelix_did(&self.challenge.challenge_issuer_ref, "challenge issuer")
    }
}

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    #[entry_type(visibility = "private")]
    ChallengeEntropy(ChallengeEntropyRecord),
    AuthorityStateChallenge(AuthorityStateChallengeRecord),
}

#[hdk_link_types]
pub enum LinkTypes {
    IssuerToChallenge,
}

pub fn entropy_nonce_digest(bytes: &[u8]) -> Digest32 {
    let mut hasher = blake3::Hasher::new();
    hasher.update(DOMAIN_ENTROPY);
    hasher.update(&(ENTROPY_DIGEST_PROFILE.len() as u64).to_le_bytes());
    hasher.update(ENTROPY_DIGEST_PROFILE.as_bytes());
    hasher.update(&(bytes.len() as u64).to_le_bytes());
    hasher.update(bytes);
    Digest32(*hasher.finalize().as_bytes())
}

fn validate_create_entropy(
    action: Create,
    entry: ChallengeEntropyRecord,
) -> ExternResult<ValidateCallbackResult> {
    if let Err(error) = entry.validate_structure() {
        return Ok(ValidateCallbackResult::Invalid(error));
    }
    let expected = format!("did:mycelix:{}", action.author);
    if entry.issued_by != expected {
        return Ok(ValidateCallbackResult::Invalid(
            "private challenge entropy must be issued by the committing agent".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_challenge(
    action: Create,
    entry: AuthorityStateChallengeRecord,
) -> ExternResult<ValidateCallbackResult> {
    if let Err(error) = entry.validate_structure() {
        return Ok(ValidateCallbackResult::Invalid(error));
    }
    let expected = format!("did:mycelix:{}", action.author);
    if entry.challenge.challenge_issuer_ref != expected {
        return Ok(ValidateCallbackResult::Invalid(
            "public challenge issuer must equal the committing agent".into(),
        ));
    }
    let action_ms = timestamp_ms(action.timestamp, "challenge create action")?;
    if entry.challenge.issued_at_ms > action_ms || action_ms >= entry.challenge.expires_at_ms {
        return Ok(ValidateCallbackResult::Invalid(
            "public challenge must already be issued and remain live at commit time".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(store_entry) => match store_entry {
            OpEntry::CreateEntry { app_entry, action } => match app_entry {
                EntryTypes::ChallengeEntropy(entry) => validate_create_entropy(action, entry),
                EntryTypes::AuthorityStateChallenge(entry) => validate_create_challenge(action, entry),
            },
            OpEntry::UpdateEntry { .. } => Ok(ValidateCallbackResult::Invalid(
                "authority-state challenge records are immutable".into(),
            )),
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterCreateLink { .. } => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterDeleteLink { .. } => Ok(ValidateCallbackResult::Invalid(
            "authority-state challenge links are append-only".into(),
        )),
        FlatOp::RegisterDelete(_) => Ok(ValidateCallbackResult::Invalid(
            "authority-state challenge records are append-only".into(),
        )),
        FlatOp::RegisterUpdate(_) => Ok(ValidateCallbackResult::Invalid(
            "authority-state challenge records cannot be updated".into(),
        )),
        FlatOp::StoreRecord(_) | FlatOp::RegisterAgentActivity(_) => {
            Ok(ValidateCallbackResult::Valid)
        }
    }
}

fn validate_ref(value: &str, field: &str) -> Result<(), String> {
    if value.trim().is_empty() || value.len() > MAX_REF_BYTES {
        Err(format!("{field} must be 1-{MAX_REF_BYTES} bytes"))
    } else {
        Ok(())
    }
}

fn require_mycelix_did(value: &str, field: &str) -> Result<(), String> {
    validate_ref(value, field)?;
    if !value.starts_with("did:mycelix:") {
        return Err(format!("{field} must be a did:mycelix identifier"));
    }
    Ok(())
}

fn timestamp_ms(timestamp: Timestamp, field: &str) -> Result<u64, String> {
    let micros = timestamp.as_micros();
    if micros <= 0 {
        return Err(format!("{field} must be positive"));
    }
    Ok(micros as u64 / 1_000)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn entropy_digest_is_domain_separated_and_content_sensitive() {
        let a = entropy_nonce_digest(&[1; ENTROPY_BYTES]);
        let b = entropy_nonce_digest(&[2; ENTROPY_BYTES]);
        assert_ne!(a, b);
        assert!(!a.is_zero());
    }
}
