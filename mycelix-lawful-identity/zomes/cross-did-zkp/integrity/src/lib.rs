#![deny(unsafe_code)]
// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//
// cross-did-zkp integrity — the bridge between primary and legal DIDs.
// The primary DID can prove "I control a legal DID that holds credential X
// from issuer Y" without revealing which legal DID. See
// ../../docs/THREAT_MODEL.md vectors 1 and 2.
//
// ============================================================================
// HONESTY NOTE (added after a 2026-07 audit, do not remove or weaken):
// This crate does NOT implement or run any zero-knowledge proof system.
// `CrossDidProof::proof_value` is an opaque, cryptographically UNVERIFIED
// blob. "STARK" appears below only as the name of the scheme this design
// *intends* callers to eventually use off-chain — it is not implemented
// here, there are no cryptographic dependencies in this crate's Cargo.toml,
// and `validate_cross_did_proof` performs no proof verification whatsoever
// (see its doc comment). Any code, documentation, or API description that
// reads as "on-chain STARK/ZKP verification happens" for this zome is
// wrong. Do not treat a stored `CrossDidProof` as a cryptographically
// proven claim until real proof verification is implemented and wired in.
// ============================================================================

use hdi::prelude::*;

/// A verifier's nonce request. The verifier publishes a fresh nonce; the prover
/// uses it exactly once. Reuse detection happens at verifier side (LRU window
/// of 65,536 most-recent nonces per verifier) — this zome stores the state the
/// verifier needs to detect reuse.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct NonceRequest {
    /// Verifier DID (may be primary, legal, or external).
    pub verifier_did: String,
    /// 32-byte random nonce, base64-encoded.
    pub nonce_b64: String,
    /// ISO 8601 creation timestamp. Nonces older than 5 minutes SHOULD be rejected.
    pub created_at: String,
}

/// The cross-DID proof as presented to the verifier. Crucially, this structure
/// contains ONLY the fields the verifier needs — no legal DID string, no
/// pubkey of the legal DID, no deterministic hash of it.
///
/// **Design intent, not current implementation**: the public inputs to a
/// *future* STARK verification would be exactly
/// `{ issuer_public_key_hash, claim_predicate_hash, nonce_hash }`. Today,
/// no STARK (or any other ZKP) is generated or verified anywhere in this
/// crate — see `proof_value` below and the module-level HONESTY NOTE above.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct CrossDidProof {
    /// Hash of the issuer's public key (permits verifier to look up issuer trust tier).
    pub issuer_pk_hash: String,
    /// Hash of the claim predicate being proven (e.g., "age>=21", "nationality=USA").
    pub claim_predicate_hash: String,
    /// Hash of the verifier-supplied nonce (prevents replay).
    pub nonce_hash: String,
    /// Opaque, multibase-encoded proof bytes — scheme tag `unverified-opaque-v1`.
    ///
    /// **NOT cryptographically verified anywhere on-chain.** The name
    /// "STARK proof bytes" describes only the *intended future* format;
    /// this crate stores whatever bytes the caller supplies and never
    /// checks that they constitute a valid proof of anything. Treat this
    /// field as an unverified, caller-asserted claim, not as proof.
    pub proof_value: String,
    /// ISO 8601 generation timestamp.
    pub generated_at: String,
}

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    NonceRequest(NonceRequest),
    CrossDidProof(CrossDidProof),
}

#[hdk_link_types]
pub enum LinkTypes {
    /// Verifier anchor → nonces they've issued (for reuse detection).
    VerifierToNonce,
    /// Nonce-hash anchor → proofs that consumed that nonce (for replay detection).
    NonceHashToProof,
}

// ============================================================================
// Validation
// ============================================================================

#[hdk_extern]
pub fn genesis_self_check(_data: GenesisSelfCheckData) -> ExternResult<ValidateCallbackResult> {
    Ok(ValidateCallbackResult::Valid)
}

/// **P0 author-binding pass, 2026-07-09**: neither entry type carries an
/// agent field, and that's deliberate -- the whole point of this zome is
/// unlinkability (see the module doc comment above: proofs expose only
/// hashes, never a legal DID or its pubkey). No coordinator function
/// calls `update_entry` for either type (confirmed via grep -- both are
/// create-only: a nonce is requested once, a proof is generated once).
/// Closes the wide-open RegisterUpdate/RegisterDelete bug that
/// previously routed both through the unconditional `_ => Valid`
/// catch-all.
#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(OpEntry::CreateEntry { app_entry, .. }) => match app_entry {
            EntryTypes::NonceRequest(n) => validate_nonce_request(&n),
            EntryTypes::CrossDidProof(p) => validate_cross_did_proof(&p),
        },
        FlatOp::StoreEntry(OpEntry::UpdateEntry { .. }) => Ok(ValidateCallbackResult::Invalid(
            "Cross-DID ZKP records are immutable".into(),
        )),
        FlatOp::RegisterUpdate(_) => Ok(ValidateCallbackResult::Invalid(
            "Cross-DID ZKP records are immutable".into(),
        )),
        _ => Ok(ValidateCallbackResult::Valid),
    }
}

fn validate_nonce_request(entry: &NonceRequest) -> ExternResult<ValidateCallbackResult> {
    if entry.verifier_did.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "verifier_did empty".to_string(),
        ));
    }
    // 32 raw bytes base64-encoded → 44 chars (with padding) or 43 (without).
    if entry.nonce_b64.len() < 43 || entry.nonce_b64.len() > 48 {
        return Ok(ValidateCallbackResult::Invalid(format!(
            "nonce_b64 length {} outside expected 43-48",
            entry.nonce_b64.len()
        )));
    }
    if entry.created_at.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "created_at empty".to_string(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

/// **NO CRYPTOGRAPHIC VERIFICATION HAPPENS HERE.** This function checks only
/// that the entry's fields are structurally present (non-empty strings). It
/// does NOT parse, decode, or verify `proof_value` as a STARK proof, a
/// signature, or any other cryptographic object — this crate has zero
/// cryptographic dependencies (see `Cargo.toml`) and cannot do so. A
/// `CrossDidProof` accepted by this validator is, today, nothing more than
/// an unverified assertion by whoever submitted it. Downstream consumers
/// (verifiers, UIs, other zomes) MUST NOT treat a stored `CrossDidProof` as
/// a cryptographically proven claim until real proof verification is
/// designed and implemented (tracked as future work; see the module-level
/// HONESTY NOTE at the top of this file).
fn validate_cross_did_proof(entry: &CrossDidProof) -> ExternResult<ValidateCallbackResult> {
    if entry.issuer_pk_hash.is_empty()
        || entry.claim_predicate_hash.is_empty()
        || entry.nonce_hash.is_empty()
    {
        return Ok(ValidateCallbackResult::Invalid(
            "proof hashes must all be non-empty".to_string(),
        ));
    }
    if entry.proof_value.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "proof_value empty".to_string(),
        ));
    }
    if entry.generated_at.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "generated_at empty".to_string(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}
