// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Step 4 of [`local_identity`](crate::local_identity)'s recoverable-identity
//! roadmap: anchoring the browser-generated Ed25519 key on the real
//! `did_registry` Holochain zome, instead of `local_did()` being a
//! client-only label that every frontend independently trusts at face value.
//!
//! # What this does — and what it deliberately doesn't
//!
//! `did_registry::create_did()` takes **no input**: it always anchors a new
//! DID document to the Holochain conductor's own `AgentPubKey` (read via
//! `agent_info()` server-side), not to whatever public key a caller might
//! want to supply. That's not a limitation this module works around — it's
//! how the zome is designed, and rightly so: `controller` is validated
//! against the entry's real author, so it can't be an arbitrary value.
//!
//! So there is no way to make the browser key *the* anchor. What this module
//! does instead: call [`ensure_did_anchored`] to create the on-DHT DID
//! document (idempotent — a no-op if one already exists for this agent),
//! then register the browser's Ed25519 public key as an *additional*
//! verification method on that document via `add_verification_method`. The
//! DID's `id`/`controller` stay pinned to the conductor's `AgentPubKey`
//! either way; what changes is that the document now honestly records "this
//! browser-held key is associated with this identity" instead of nothing.
//!
//! ## Why this doesn't also mark the key as an active `authentication`
//! method (yet)
//!
//! Doing that requires a read-modify-write of the DID document's
//! `authentication` list via `update_did_document` — which means correctly
//! deserializing the zome's `ExternResult<Record>` response client-side
//! (Holochain's `Record` envelope, not a flat JSON shape). Nothing in this
//! codebase does that today for `did_registry`
//! (`mycelix-identity/apps/leptos/src/identity_context.rs`'s existing
//! `serde_json::from_value::<DidDocumentView>(record)` almost certainly
//! doesn't work either, for the same reason — `DidDocumentView`'s flat shape
//! doesn't match a `Record`'s actual envelope). Getting that right needs a
//! real round-trip test (e.g. a `sweettest` integration test against an
//! ephemeral conductor) before being worth shipping — not guesswork against
//! a shared, already-live conductor. `add_verification_method` doesn't have
//! this problem: it's append-only server-side and returns nothing this
//! module needs to parse.
//!
//! ## Verification status
//!
//! The zome-call *shapes* here (struct fields, `#[serde(rename)]` attributes)
//! were checked directly against
//! `mycelix-workspace/mycelix-identity/zomes/did_registry/{integrity,coordinator}/src/lib.rs`
//! and match exactly. What has **not** been verified is an actual live
//! round-trip against a running conductor — this is the first real
//! zome-*mutation* call from any browser Leptos client anywhere in this
//! codebase (every existing example, including `did_registry` itself, is
//! either a read or a Rust-native `sweettest` harness call). Treat this as
//! implemented-and-type-checked, not proven-in-production, until it's been
//! exercised against a live or `sweettest` conductor.

use crate::holochain_provider::HolochainCtx;
use ed25519_dalek::SigningKey;
use mycelix_crypto::{AlgorithmId, TaggedPublicKey};
use serde::{Deserialize, Serialize};

/// Set once [`ensure_browser_key_anchored`] has succeeded, so repeat app
/// loads don't re-submit `add_verification_method` — it has no dedup guard
/// server-side, so calling it again would add a second, identical method.
const DHT_ANCHOR_STATUS_KEY: &str = "mycelix_identity_dht_anchor_status_v1";

/// Mirrors `did_registry_integrity::VerificationMethod` field-for-field
/// (including the `#[serde(rename)]`s) — this is the wire shape the zome's
/// own integrity validation expects, not a shape this module invented.
#[derive(Debug, Clone, Serialize, Deserialize)]
struct VerificationMethod {
    id: String,
    #[serde(rename = "type")]
    type_: String,
    controller: String,
    #[serde(rename = "publicKeyMultibase")]
    public_key_multibase: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    algorithm: Option<u16>,
}

/// Ensure an on-DHT DID document exists for the connected agent, and return
/// its `did:mycelix:<agent_pub_key>` identifier.
///
/// Idempotent: `create_did()` has its own server-side guard ("Agent already
/// has a DID document") — this treats that specific failure as success
/// rather than an error, so it's safe to call on every app load.
///
/// # Errors
///
/// Returns an error if not connected to a real conductor (mock mode), or if
/// `create_did` fails for any reason other than already existing.
pub async fn ensure_did_anchored(hc: &HolochainCtx) -> Result<String, String> {
    let did_id = hc
        .connected_agent_did()
        .ok_or_else(|| "Not connected to a real conductor (mock mode?)".to_string())?;

    match hc
        .call_zome_default::<(), serde_json::Value>("did_registry", "create_did", &())
        .await
    {
        Ok(_) => Ok(did_id),
        Err(e) if e.contains("already has a DID document") => Ok(did_id),
        Err(e) => Err(e),
    }
}

/// Register the local browser identity's Ed25519 public key as a
/// verification method on the connected agent's on-DHT DID document.
///
/// Calls [`ensure_did_anchored`] first (idempotent), then
/// `add_verification_method` with the browser key, multibase-encoded via
/// [`mycelix_crypto::TaggedPublicKey`] — the exact encoding the zome's own
/// `validate_multibase_key` expects, not a hand-rolled equivalent.
///
/// Guarded by a `localStorage` flag ([`is_browser_key_anchored`]) so it only
/// ever submits `add_verification_method` once per browser — that zome
/// function has no dedup guard of its own, so calling it again would add a
/// second, identical verification method to the document.
///
/// # Errors
///
/// Returns an error if [`ensure_did_anchored`] fails, or if
/// `add_verification_method` is rejected by the zome (e.g. malformed
/// multibase key — shouldn't happen given `TaggedPublicKey` constructs it,
/// but the zome's own validation is the final word).
pub async fn ensure_browser_key_anchored(
    hc: &HolochainCtx,
    signing_key: &SigningKey,
) -> Result<(), String> {
    if is_browser_key_anchored() {
        return Ok(());
    }

    let did_id = ensure_did_anchored(hc).await?;

    let public_key_multibase = TaggedPublicKey::new(
        AlgorithmId::Ed25519,
        signing_key.verifying_key().to_bytes().to_vec(),
    )
    .map_err(|e| format!("Failed to encode browser public key: {e}"))?
    .to_multibase();

    let method = VerificationMethod {
        id: format!("{did_id}#browser-key-1"),
        type_: AlgorithmId::Ed25519
            .did_verification_method_type()
            .to_string(),
        controller: did_id,
        public_key_multibase,
        algorithm: Some(AlgorithmId::Ed25519.as_u16()),
    };

    hc.call_zome_default::<VerificationMethod, serde_json::Value>(
        "did_registry",
        "add_verification_method",
        &method,
    )
    .await?;

    crate::local_identity::save_string(DHT_ANCHOR_STATUS_KEY, "anchored");
    Ok(())
}

/// Has [`ensure_browser_key_anchored`] already succeeded on this browser?
///
/// A client-side-only guard — it does not check whether the verification
/// method is actually still present in the on-DHT document (that would
/// require the Record-parsing this module explicitly avoids; see module
/// docs). Good enough to prevent this browser from spamming duplicate
/// verification methods on every app load, not a source of truth about
/// on-DHT state.
pub fn is_browser_key_anchored() -> bool {
    crate::local_identity::load_string(DHT_ANCHOR_STATUS_KEY).is_some()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The multibase encoding must round-trip through the same
    /// `TaggedPublicKey` machinery `did_registry`'s own
    /// `validate_multibase_key` uses server-side — this is the one part of
    /// this module verifiable without a live conductor, and it's the part
    /// most likely to silently produce a well-formed-looking-but-wrong
    /// string if hand-rolled instead.
    #[test]
    fn multibase_key_round_trips_through_tagged_public_key() {
        let key_bytes = [7u8; 32];
        let tagged = TaggedPublicKey::new(AlgorithmId::Ed25519, key_bytes.to_vec())
            .expect("32 bytes is a valid Ed25519 key length");
        let multibase = tagged.to_multibase();

        assert!(
            multibase.starts_with('z'),
            "must use the 'z' (base58btc) prefix"
        );

        let decoded =
            TaggedPublicKey::from_multibase(&multibase).expect("must decode what we encoded");
        assert_eq!(decoded.algorithm, AlgorithmId::Ed25519);
        assert_eq!(decoded.key_bytes, key_bytes.to_vec());
    }

    /// The wire shape (field names, casing) must match
    /// `did_registry_integrity::VerificationMethod` exactly, or the zome's
    /// `#[serde(rename = "...")]`-annotated deserializer will reject the
    /// call with a confusing "unknown field" error instead of a validation
    /// error.
    #[test]
    fn verification_method_serializes_with_the_zomes_expected_field_names() {
        let method = VerificationMethod {
            id: "did:mycelix:abc#browser-key-1".to_string(),
            type_: "Ed25519VerificationKey2020".to_string(),
            controller: "did:mycelix:abc".to_string(),
            public_key_multibase: "zSomeBase58String".to_string(),
            algorithm: Some(0xed01),
        };

        let json = serde_json::to_value(&method).unwrap();
        assert_eq!(json["id"], "did:mycelix:abc#browser-key-1");
        assert_eq!(json["type"], "Ed25519VerificationKey2020");
        assert_eq!(json["controller"], "did:mycelix:abc");
        assert_eq!(json["publicKeyMultibase"], "zSomeBase58String");
        assert_eq!(json["algorithm"], 0xed01);
        // Field must not appear under its Rust name — the zome only
        // recognizes the `#[serde(rename)]`'d wire name.
        assert!(json.get("type_").is_none());
        assert!(json.get("public_key_multibase").is_none());
    }

    /// `algorithm: None` must be omitted entirely (not serialized as
    /// `null`) — the zome's field is `Option<u16>` with
    /// `#[serde(skip_serializing_if = "Option::is_none")]` semantics on the
    /// *deserializing* side too via `#[serde(default)]`, but omitting is
    /// the more conservative, guaranteed-compatible choice.
    #[test]
    fn verification_method_omits_absent_algorithm_rather_than_nulling_it() {
        let method = VerificationMethod {
            id: "x".to_string(),
            type_: "x".to_string(),
            controller: "x".to_string(),
            public_key_multibase: "x".to_string(),
            algorithm: None,
        };
        let json = serde_json::to_value(&method).unwrap();
        assert!(!json.as_object().unwrap().contains_key("algorithm"));
    }
}
