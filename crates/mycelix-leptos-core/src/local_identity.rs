// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Local agent identity — per-visitor cryptographic DID stored in localStorage.
//!
//! # Step 1 of the recoverable-identity roadmap
//!
//! This module used to mint a `did:mycelix:<random>` label from
//! `js_sys::Math::random()` — **not** a CSPRNG, and not backed by any real
//! key material. That made "identity" here nothing more than a per-browser
//! nonce: it could not sign anything, could not be recovered, and had no
//! cryptographic binding to the DID string it produced.
//!
//! This pass fixes that: `local_did()` now generates a genuine Ed25519
//! keypair in-browser (seeded from the Web Crypto CSPRNG,
//! `crypto.getRandomValues`, NOT `Math.random()`) and derives the DID
//! deterministically from the public key.
//!
//! Full trajectory (this pass is only step 1):
//!
//! 1. **DONE (this pass)**: real Ed25519 keypair; DID derived from the
//!    public key instead of random noise.
//! 2. Passphrase-wrapped encryption of the private key at rest (WebCrypto
//!    AES-GCM + a real KDF such as PBKDF2/Argon2), instead of storing raw
//!    key bytes in `localStorage`. See the "Storage" note below — this is
//!    the biggest remaining gap versus a real wallet.
//! 3. Standards-correct BIP-39 seed phrase export/import for manual/offline
//!    backup (with a proper wordlist + checksum, not an ad hoc scheme).
//! 4. Wire the local public key into a real
//!    `mycelix-identity/zomes/did_registry::create_did` zome call, so a
//!    browser-generated DID becomes the *anchor* key of an actual on-DHT DID
//!    document (with rotation support) instead of a client-only label that
//!    four frontends independently trust at face value.
//! 5. Wire `mycelix-identity/zomes/recovery` (guardian/trustee threshold
//!    voting + progressive self-recovery verification anchors) into a
//!    recovery UI, so a lost or reset device can re-establish control of the
//!    same DID instead of silently minting a new one.
//! 6. Multi-device pairing (QR / link-code handoff of an *authorization*,
//!    never of the raw private key).
//!
//! None of steps 2–6 are implemented here. `mycelix-crypto`'s `wasm` feature
//! (see `mycelix-identity/crates/mycelix-crypto/`) was evaluated for this
//! but only exposes types/validation under `wasm32` — all signing/keygen
//! there is `#[cfg(feature = "native")]` — so it cannot back in-browser
//! keygen as-is. `ed25519-dalek` is used directly instead, matching the
//! pattern already proven in `mycelix-pulse/apps/leptos/src/crypto.rs`.
//!
//! ## Storage (interim state — matches today's baseline risk, not worse)
//!
//! The Ed25519 secret key is currently persisted as base64 in
//! `localStorage`, **unencrypted**. This carries the same at-rest risk
//! profile the old code already had for its (fake) identity token — anything
//! in `localStorage` is readable by any script with page access — so this
//! pass has not made storage-at-rest *worse*. What changed is that the key
//! material is now real instead of nonexistent. Step 2 above is the fix for
//! this gap and should slot in via [`store_signing_key`] /
//! [`load_signing_key`] without touching the public API below.
//!
//! ## Migration from the old insecure DID
//!
//! Browsers that already have a legacy `did:mycelix:<12-random-bytes-hex>`
//! string (24 hex chars, minted from `Math.random()`) stored under
//! [`LEGACY_DID_KEY`] are **not** trusted or reused. The first call to
//! [`local_did`] on such a browser generates a brand-new real keypair (and
//! removes the legacy value), so every visitor gets upgraded to a real
//! identity on next load. This does mean legacy per-browser DIDs change
//! once — acceptable since they never had real cryptographic backing to
//! begin with.

use ed25519_dalek::{Signature, Signer, SigningKey, VerifyingKey};
use leptos::prelude::*;

/// Legacy key: pre-2026-07 builds stored a `did:mycelix:<12-random-bytes-hex>`
/// string here directly, generated from `js_sys::Math::random()` (not a
/// CSPRNG). It is no longer written. `local_did()` ignores any value found
/// here and mints a real keypair on next load; the stale entry is removed
/// once that happens.
const LEGACY_DID_KEY: &str = "mycelix_local_did";

/// Base64-encoded 32-byte Ed25519 secret key seed.
const IDENTITY_SECRET_KEY: &str = "mycelix_identity_secret_ed25519_b64";
/// Base64-encoded 32-byte Ed25519 public key (cached alongside the secret;
/// derivable from it, stored for cheap reads).
const IDENTITY_PUBLIC_KEY: &str = "mycelix_identity_public_ed25519_b64";

/// Generate or retrieve the local agent DID.
///
/// Backed by a real Ed25519 keypair (see module docs) — no longer a random
/// label. Idempotent: once a keypair has been generated and persisted,
/// repeat calls return the same DID.
pub fn local_did() -> String {
    did_from_public_key(&ensure_keypair().verifying_key())
}

/// Ensure a local Ed25519 keypair exists, generating one via the Web Crypto
/// CSPRNG on first use, and return it.
///
/// Exposed (not just used internally) so future callers — e.g. a
/// `did_registry::create_did` zome call, or a recovery flow — can get at
/// the real key material without re-deriving storage logic.
pub fn ensure_keypair() -> SigningKey {
    if let Some(signing_key) = load_signing_key() {
        return signing_key;
    }
    let signing_key = generate_signing_key();
    store_signing_key(&signing_key);
    signing_key
}

/// Sign an arbitrary message with the local identity's private key.
///
/// Generates the keypair on first use if one does not already exist. This
/// is what a future DID-registry `create_did`/`rotate_key` call, or any
/// other proof-of-control flow, would use to prove ownership of
/// [`local_did`].
pub fn sign_with_local_identity(message: &[u8]) -> Signature {
    ensure_keypair().sign(message)
}

/// Provide local identity as Leptos context.
pub fn provide_local_identity() {
    let did = local_did();
    provide_context(LocalIdentity { did });
}

/// Retrieve local identity from context.
pub fn use_local_identity() -> LocalIdentity {
    expect_context::<LocalIdentity>()
}

/// Local identity state.
#[derive(Clone, Debug)]
pub struct LocalIdentity {
    pub did: String,
}

impl LocalIdentity {
    /// Short display name (last 8 chars of DID).
    pub fn short_name(&self) -> String {
        let suffix = self.did.rsplit(':').next().unwrap_or(&self.did);
        if suffix.len() > 8 {
            format!("{}...", &suffix[..8])
        } else {
            suffix.to_string()
        }
    }
}

/// Derive a `did:mycelix:<hex-pubkey>` string from an Ed25519 public key.
///
/// Pure function (no browser APIs touched) — safe to unit test on any
/// target, wasm32 or native.
pub fn did_from_public_key(public_key: &VerifyingKey) -> String {
    let hex = hex_encode(public_key.as_bytes());
    format!("did:mycelix:{hex}")
}

/// Generate a fresh Ed25519 keypair, seeded from the Web Crypto CSPRNG
/// (`crypto.getRandomValues`) — NOT `Math.random()`.
fn generate_signing_key() -> SigningKey {
    let mut seed = [0u8; 32];
    let crypto = web_sys::window()
        .expect("no global `window` (not running in a browser)")
        .crypto()
        .expect("Web Crypto API unavailable");
    crypto
        .get_random_values_with_u8_array(&mut seed)
        .expect("crypto.getRandomValues failed");
    SigningKey::from_bytes(&seed)
}

// --- localStorage helpers (private key persistence) ---

fn store_signing_key(signing_key: &SigningKey) {
    save_string(IDENTITY_SECRET_KEY, &base64_encode(signing_key.as_bytes()));
    save_string(
        IDENTITY_PUBLIC_KEY,
        &base64_encode(signing_key.verifying_key().as_bytes()),
    );
    // Clean up any pre-existing insecure (Math.random()-derived) identity —
    // it never had real cryptographic backing, so there's nothing to migrate.
    remove_item(LEGACY_DID_KEY);
}

fn load_signing_key() -> Option<SigningKey> {
    let secret_b64 = load_string(IDENTITY_SECRET_KEY)?;
    let secret_bytes = base64_decode(&secret_b64)?;
    let secret_array: [u8; 32] = secret_bytes.try_into().ok()?;
    Some(SigningKey::from_bytes(&secret_array))
}

fn base64_encode(bytes: &[u8]) -> String {
    use base64::{engine::general_purpose::STANDARD, Engine as _};
    STANDARD.encode(bytes)
}

fn base64_decode(s: &str) -> Option<Vec<u8>> {
    use base64::{engine::general_purpose::STANDARD, Engine as _};
    STANDARD.decode(s).ok()
}

fn hex_encode(bytes: &[u8]) -> String {
    bytes.iter().map(|b| format!("{b:02x}")).collect()
}

fn remove_item(key: &str) {
    if let Some(Ok(Some(storage))) = web_sys::window().map(|w| w.local_storage()) {
        let _ = storage.remove_item(key);
    }
}

// --- localStorage helpers (generic) ---

pub fn load_string(key: &str) -> Option<String> {
    web_sys::window()?
        .local_storage()
        .ok()??
        .get_item(key)
        .ok()?
}

pub fn save_string(key: &str, value: &str) {
    if let Some(Ok(Some(storage))) = web_sys::window().map(|w| w.local_storage()) {
        let _ = storage.set_item(key, value);
    }
}

/// Load a JSON-serializable value from localStorage.
pub fn load_json<T: serde::de::DeserializeOwned>(key: &str) -> Option<T> {
    let s = load_string(key)?;
    serde_json::from_str(&s).ok()
}

/// Save a JSON-serializable value to localStorage.
pub fn save_json<T: serde::Serialize>(key: &str, value: &T) {
    if let Ok(s) = serde_json::to_string(value) {
        save_string(key, &s);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// `did_from_public_key` is pure and needs no browser: it should work
    /// identically on native and wasm32 targets.
    #[test]
    fn did_derivation_matches_pubkey_hex() {
        let seed = [7u8; 32];
        let signing_key = SigningKey::from_bytes(&seed);
        let did = did_from_public_key(&signing_key.verifying_key());

        assert!(did.starts_with("did:mycelix:"));
        let hex_part = did.strip_prefix("did:mycelix:").unwrap();
        // Ed25519 public keys are 32 bytes -> 64 hex chars, not the old
        // scheme's 24 (12 random bytes).
        assert_eq!(hex_part.len(), 64);
        assert_eq!(hex_part, hex_encode(signing_key.verifying_key().as_bytes()));
    }

    /// Same seed -> same keypair -> same DID. This is what makes the DID a
    /// real identity rather than a fresh random label every regeneration.
    #[test]
    fn did_derivation_is_deterministic_for_a_given_key() {
        let seed = [42u8; 32];
        let key_a = SigningKey::from_bytes(&seed);
        let key_b = SigningKey::from_bytes(&seed);
        assert_eq!(
            did_from_public_key(&key_a.verifying_key()),
            did_from_public_key(&key_b.verifying_key())
        );
    }

    /// Different keys must not collide.
    #[test]
    fn did_derivation_differs_across_keys() {
        let key_a = SigningKey::from_bytes(&[1u8; 32]);
        let key_b = SigningKey::from_bytes(&[2u8; 32]);
        assert_ne!(
            did_from_public_key(&key_a.verifying_key()),
            did_from_public_key(&key_b.verifying_key())
        );
    }

    /// The DID must be derived from *this* keypair's actual public key, not
    /// an unrelated one — i.e. it's a real binding, not decoration.
    #[test]
    fn did_is_bound_to_the_signing_key_that_produced_it() {
        let signing_key = SigningKey::from_bytes(&[9u8; 32]);
        let unrelated_key = SigningKey::from_bytes(&[99u8; 32]);
        let did = did_from_public_key(&signing_key.verifying_key());
        assert_ne!(did, did_from_public_key(&unrelated_key.verifying_key()));
    }

    /// Signatures produced by the local identity must actually verify
    /// against its own public key (round-trip proof-of-control check —
    /// this is the primitive step 4/5 of the roadmap will build on).
    #[test]
    fn signed_message_verifies_against_the_same_public_key() {
        use ed25519_dalek::Verifier;

        let signing_key = SigningKey::from_bytes(&[13u8; 32]);
        let message = b"prove control of this DID";
        let signature = signing_key.sign(message);

        assert!(signing_key
            .verifying_key()
            .verify(message, &signature)
            .is_ok());
    }

    /// `short_name()` must still work with the new (longer, 64-hex-char)
    /// pubkey-derived DIDs the same way it did for the old 24-char ones.
    #[test]
    fn short_name_truncates_pubkey_derived_did() {
        let signing_key = SigningKey::from_bytes(&[5u8; 32]);
        let identity = LocalIdentity {
            did: did_from_public_key(&signing_key.verifying_key()),
        };
        let short = identity.short_name();
        assert!(short.ends_with("..."));
        assert_eq!(short.len(), 11); // 8 chars + "..."
    }
}
