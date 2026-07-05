// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Local agent identity — per-visitor cryptographic DID stored in localStorage.
//!
//! # Steps 1–3 of the recoverable-identity roadmap
//!
//! This module used to mint a `did:mycelix:<random>` label from
//! `js_sys::Math::random()` — **not** a CSPRNG, and not backed by any real
//! key material. That made "identity" here nothing more than a per-browser
//! nonce: it could not sign anything, could not be recovered, and had no
//! cryptographic binding to the DID string it produced.
//!
//! Step 1 fixed that: `local_did()` generates a genuine Ed25519 keypair
//! in-browser (seeded from the Web Crypto CSPRNG, `crypto.getRandomValues`,
//! NOT `Math.random()`) and derives the DID deterministically from the
//! public key.
//!
//! Full trajectory:
//!
//! 1. **DONE**: real Ed25519 keypair; DID derived from the public key
//!    instead of random noise.
//! 2. **DONE (this pass)**: opt-in passphrase-wrapped encryption of the
//!    private key at rest ([`protect_with_passphrase`] /
//!    [`unlock_with_passphrase`] / [`remove_passphrase_protection`], backed
//!    by [`crate::identity_crypto`]'s WebCrypto AES-256-GCM + PBKDF2). See
//!    "Storage" below for exactly what this does and doesn't change.
//! 3. **DONE (this pass)**: standards-correct BIP-39 seed phrase
//!    export/import for manual/offline backup
//!    ([`export_seed_phrase`] / [`signing_key_from_seed_phrase`] /
//!    [`restore_from_seed_phrase`]) — the real `bip39` crate's English
//!    wordlist + checksum, not an ad hoc scheme. The 32-byte Ed25519 secret
//!    key seed maps to exactly 24 words with no padding.
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
//! None of steps 4–6 are implemented here. `mycelix-crypto`'s `wasm` feature
//! (see `mycelix-identity/crates/mycelix-crypto/`) was evaluated for this
//! but only exposes types/validation under `wasm32` — all signing/keygen
//! there is `#[cfg(feature = "native")]` — so it cannot back in-browser
//! keygen as-is. `ed25519-dalek` is used directly instead, matching the
//! pattern already proven in `mycelix-pulse/apps/leptos/src/crypto.rs`.
//!
//! ## Storage (opt-in protection, not a breaking change)
//!
//! By default, the Ed25519 secret key is still persisted as base64 in
//! `localStorage`, **unencrypted** — identical to step 1's behavior, and to
//! the old (fake) identity token's storage before that. Nothing about the
//! synchronous [`ensure_keypair`]/[`local_did`]/[`sign_with_local_identity`]/
//! [`provide_local_identity`] API changed, and none of the four apps
//! consuming it need to change anything to keep working exactly as before.
//!
//! Protection is opt-in and asynchronous (WebCrypto's `SubtleCrypto` is
//! Promise-based, so it can't be bolted onto the existing sync API without
//! breaking it): an app adds a "protect your identity" UI action that calls
//! [`protect_with_passphrase`], and on every later visit calls
//! [`unlock_with_passphrase`] once (e.g. behind a passphrase prompt at
//! startup) before touching the sync API. Once unlocked, the decrypted key
//! is cached in memory for the rest of the session — subsequent
//! [`ensure_keypair`] calls hit the cache, not `SubtleCrypto`, again.
//!
//! If an identity is protected but [`ensure_keypair`] is called before
//! [`unlock_with_passphrase`] has run this session, it panics rather than
//! silently minting (and thereby orphaning) a brand-new keypair over the
//! real one — see [`ensure_keypair`]'s docs.
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

use crate::identity_crypto::{self, EncryptedBlob};
use ed25519_dalek::{Signature, Signer, SigningKey, VerifyingKey};
use leptos::prelude::*;
use std::cell::RefCell;

/// Legacy key: pre-2026-07 builds stored a `did:mycelix:<12-random-bytes-hex>`
/// string here directly, generated from `js_sys::Math::random()` (not a
/// CSPRNG). It is no longer written. `local_did()` ignores any value found
/// here and mints a real keypair on next load; the stale entry is removed
/// once that happens.
const LEGACY_DID_KEY: &str = "mycelix_local_did";

/// Base64-encoded 32-byte Ed25519 secret key seed. Absent when the identity
/// is passphrase-protected (see [`IDENTITY_SECRET_ENCRYPTED_KEY`]).
const IDENTITY_SECRET_KEY: &str = "mycelix_identity_secret_ed25519_b64";
/// Base64-encoded 32-byte Ed25519 public key (cached alongside the secret;
/// derivable from it, stored for cheap reads).
const IDENTITY_PUBLIC_KEY: &str = "mycelix_identity_public_ed25519_b64";
/// JSON-encoded [`EncryptedBlobWire`] — present only once
/// [`protect_with_passphrase`] has been called. Its presence is what
/// [`is_passphrase_protected`] checks.
const IDENTITY_SECRET_ENCRYPTED_KEY: &str = "mycelix_identity_secret_ed25519_encrypted_v1";

thread_local! {
    // WASM is single-threaded, so a thread-local is a plain per-session
    // cache: populated by `ensure_keypair` (unprotected path) or
    // `unlock_with_passphrase` (protected path), cleared only on page
    // reload. Never persisted — the whole point is that the decrypted key
    // lives in memory only, not back in `localStorage`.
    static UNLOCKED_KEY: RefCell<Option<SigningKey>> = const { RefCell::new(None) };
}

#[derive(serde::Serialize, serde::Deserialize)]
struct EncryptedBlobWire {
    salt_b64: String,
    iv_b64: String,
    ciphertext_b64: String,
}

impl From<&EncryptedBlob> for EncryptedBlobWire {
    fn from(blob: &EncryptedBlob) -> Self {
        Self {
            salt_b64: base64_encode(&blob.salt),
            iv_b64: base64_encode(&blob.iv),
            ciphertext_b64: base64_encode(&blob.ciphertext),
        }
    }
}

impl TryFrom<&EncryptedBlobWire> for EncryptedBlob {
    type Error = String;

    fn try_from(wire: &EncryptedBlobWire) -> Result<Self, String> {
        Ok(EncryptedBlob {
            salt: base64_decode(&wire.salt_b64).ok_or("corrupt salt")?,
            iv: base64_decode(&wire.iv_b64).ok_or("corrupt iv")?,
            ciphertext: base64_decode(&wire.ciphertext_b64).ok_or("corrupt ciphertext")?,
        })
    }
}

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
///
/// # Panics
///
/// Panics if the identity is passphrase-protected (see
/// [`protect_with_passphrase`]) and [`unlock_with_passphrase`] hasn't
/// succeeded yet this session. This is deliberate: the alternative would be
/// silently minting a fresh keypair over a protected-but-locked identity,
/// which permanently orphans the real one. Since protection is opt-in and
/// nothing in this crate calls [`protect_with_passphrase`] on its own, this
/// can only happen if a consuming app added passphrase protection without
/// also gating its startup on an unlock prompt.
pub fn ensure_keypair() -> SigningKey {
    if let Some(signing_key) = UNLOCKED_KEY.with(|cell| cell.borrow().clone()) {
        return signing_key;
    }
    if let Some(signing_key) = load_signing_key() {
        UNLOCKED_KEY.with(|cell| *cell.borrow_mut() = Some(signing_key.clone()));
        return signing_key;
    }
    if is_passphrase_protected() {
        panic!(
            "Local identity is passphrase-protected but locked this session. \
             Call `unlock_with_passphrase()` before `ensure_keypair()` \
             (directly or via `local_did()`/`sign_with_local_identity()`/ \
             `provide_local_identity()`, which all call it)."
        );
    }
    let signing_key = generate_signing_key();
    store_signing_key(&signing_key);
    UNLOCKED_KEY.with(|cell| *cell.borrow_mut() = Some(signing_key.clone()));
    signing_key
}

/// Is the local identity currently passphrase-protected?
///
/// True once [`protect_with_passphrase`] has succeeded, until
/// [`remove_passphrase_protection`] is called. Does not indicate whether
/// it's currently *unlocked* this session — check that by whether
/// [`ensure_keypair`] would panic, or track it in the caller's own UI state
/// after a successful [`unlock_with_passphrase`].
pub fn is_passphrase_protected() -> bool {
    load_string(IDENTITY_SECRET_ENCRYPTED_KEY).is_some()
}

/// Encrypt the local identity's secret key with `passphrase` and switch
/// storage over to the encrypted form.
///
/// Generates the keypair first (via [`ensure_keypair`]) if none exists yet
/// — so this can be the very first thing a fresh visitor does, going
/// straight from "no identity" to "protected identity" with no unprotected
/// window from the caller's perspective (there's a brief one internally,
/// same as any "generate, then protect" flow).
///
/// Idempotent-ish: calling again with a new passphrase re-encrypts under
/// the new one (effectively a passphrase change), since it always starts
/// from the in-memory key, not the stored ciphertext.
///
/// # Errors
///
/// Returns an error if `passphrase` is empty, or if the underlying
/// WebCrypto calls fail (see [`identity_crypto::encrypt`]).
pub async fn protect_with_passphrase(passphrase: &str) -> Result<(), String> {
    if passphrase.is_empty() {
        return Err("Passphrase must not be empty".to_string());
    }
    let signing_key = ensure_keypair();
    let blob = identity_crypto::encrypt(passphrase, signing_key.as_bytes()).await?;
    save_json(
        IDENTITY_SECRET_ENCRYPTED_KEY,
        &EncryptedBlobWire::from(&blob),
    );
    remove_item(IDENTITY_SECRET_KEY); // no more plaintext secret at rest
    UNLOCKED_KEY.with(|cell| *cell.borrow_mut() = Some(signing_key));
    Ok(())
}

/// Decrypt the passphrase-protected local identity and cache it in memory
/// for the rest of the session.
///
/// Call this once at app startup (e.g. behind a passphrase prompt gated on
/// [`is_passphrase_protected`]) before any code path that reaches
/// [`ensure_keypair`].
///
/// # Errors
///
/// Returns an error if there's no protected identity to unlock, if the
/// passphrase is wrong, or if the stored ciphertext is corrupt — a wrong
/// passphrase and corrupt ciphertext are deliberately indistinguishable
/// (see [`identity_crypto`] module docs).
pub async fn unlock_with_passphrase(passphrase: &str) -> Result<SigningKey, String> {
    let wire: EncryptedBlobWire = load_json(IDENTITY_SECRET_ENCRYPTED_KEY)
        .ok_or_else(|| "No passphrase-protected identity found".to_string())?;
    let blob = EncryptedBlob::try_from(&wire)?;
    let secret_bytes = identity_crypto::decrypt(passphrase, &blob).await?;
    let secret_array: [u8; 32] = secret_bytes
        .try_into()
        .map_err(|_| "Corrupt key material (wrong length after decrypt)".to_string())?;
    let signing_key = SigningKey::from_bytes(&secret_array);
    UNLOCKED_KEY.with(|cell| *cell.borrow_mut() = Some(signing_key.clone()));
    Ok(signing_key)
}

/// Reverse [`protect_with_passphrase`]: decrypt with `passphrase`, restore
/// plaintext storage, and drop the encrypted blob.
///
/// # Errors
///
/// Same failure modes as [`unlock_with_passphrase`] (this calls it
/// internally) — wrong passphrase, no protected identity, or corrupt blob.
pub async fn remove_passphrase_protection(passphrase: &str) -> Result<(), String> {
    let signing_key = unlock_with_passphrase(passphrase).await?;
    store_signing_key(&signing_key);
    remove_item(IDENTITY_SECRET_ENCRYPTED_KEY);
    Ok(())
}

/// Export the local identity's secret key as a standards-correct 24-word
/// BIP-39 mnemonic (English wordlist), for manual/offline backup.
///
/// Takes the key explicitly rather than calling [`ensure_keypair`] itself,
/// so this works the same way whether the identity is passphrase-protected
/// or not — the caller (which already has to have unlocked it, if
/// protected, to get a [`SigningKey`] at all) decides when export happens.
///
/// The 32-byte Ed25519 secret key seed is exactly one of BIP-39's defined
/// entropy lengths (256 bits), so this always produces 24 words with no
/// padding or truncation.
pub fn export_seed_phrase(signing_key: &SigningKey) -> String {
    bip39::Mnemonic::from_entropy(signing_key.as_bytes())
        .expect("a 32-byte Ed25519 secret key is always valid BIP-39 entropy")
        .to_string()
}

/// Validate and decode a BIP-39 seed phrase back into the [`SigningKey`] it
/// encodes, without touching storage.
///
/// # Errors
///
/// Returns an error if the phrase has an invalid word count, contains a
/// word not in the wordlist, fails its checksum (the most common case: a
/// typo or a word transposed while copying it down), or doesn't decode to
/// exactly 32 bytes (e.g. a valid-but-foreign 12-word BIP-39 phrase from
/// something else entirely — this crate only ever mints 24-word phrases).
pub fn signing_key_from_seed_phrase(phrase: &str) -> Result<SigningKey, String> {
    let mnemonic = bip39::Mnemonic::parse(phrase).map_err(|e| e.to_string())?;
    let entropy = mnemonic.to_entropy();
    let secret_array: [u8; 32] = entropy.try_into().map_err(|_| {
        "Seed phrase does not encode a 32-byte key (wrong word count for a Mycelix identity?)"
            .to_string()
    })?;
    Ok(SigningKey::from_bytes(&secret_array))
}

/// Restore the local identity from a BIP-39 backup phrase — the "I'm on a
/// new or reset device/browser, but I wrote down my backup phrase" path.
///
/// Validates and decodes `phrase`, then persists the recovered key as this
/// browser's local identity (unencrypted — call [`protect_with_passphrase`]
/// afterward to re-enable protection). This **replaces** whatever identity
/// (if any) is currently stored here; there's no merge or confirmation step
/// at this layer; callers should confirm with the user before calling this,
/// since it's effectively "log in as a different identity."
///
/// # Errors
///
/// Same failure modes as [`signing_key_from_seed_phrase`] (this calls it
/// internally) — invalid word count, unknown word, bad checksum, or wrong
/// decoded length.
pub fn restore_from_seed_phrase(phrase: &str) -> Result<SigningKey, String> {
    let signing_key = signing_key_from_seed_phrase(phrase)?;
    store_signing_key(&signing_key);
    // A freshly restored identity starts unprotected, same as a freshly
    // generated one — any previous protection was on the *replaced*
    // identity's ciphertext, which is now meaningless.
    remove_item(IDENTITY_SECRET_ENCRYPTED_KEY);
    UNLOCKED_KEY.with(|cell| *cell.borrow_mut() = Some(signing_key.clone()));
    Ok(signing_key)
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
    use base64::{Engine as _, engine::general_purpose::STANDARD};
    STANDARD.encode(bytes)
}

fn base64_decode(s: &str) -> Option<Vec<u8>> {
    use base64::{Engine as _, engine::general_purpose::STANDARD};
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

        assert!(
            signing_key
                .verifying_key()
                .verify(message, &signature)
                .is_ok()
        );
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

    /// Exporting then re-parsing a seed phrase must recover the exact same
    /// key — this is the core round-trip property a backup mechanism has
    /// to guarantee, or it isn't a backup mechanism.
    #[test]
    fn seed_phrase_round_trips_the_signing_key() {
        let signing_key = SigningKey::from_bytes(&[21u8; 32]);
        let phrase = export_seed_phrase(&signing_key);

        let recovered = signing_key_from_seed_phrase(&phrase).expect("valid phrase must parse");

        assert_eq!(recovered.to_bytes(), signing_key.to_bytes());
    }

    /// A 32-byte Ed25519 seed is exactly 256 bits of BIP-39 entropy, which
    /// standard says encodes to 24 words — not 12, 15, 18, or 21. If this
    /// ever changed it would mean the key size assumption broke somewhere.
    #[test]
    fn exported_phrase_is_24_words() {
        let signing_key = SigningKey::from_bytes(&[33u8; 32]);
        let phrase = export_seed_phrase(&signing_key);
        assert_eq!(phrase.split_whitespace().count(), 24);
    }

    /// A single mistyped word must fail the BIP-39 checksum rather than
    /// silently decoding to a different (wrong) key — that would turn a
    /// typo into silent identity corruption instead of a caught error.
    #[test]
    fn corrupted_seed_phrase_is_rejected() {
        let signing_key = SigningKey::from_bytes(&[44u8; 32]);
        let phrase = export_seed_phrase(&signing_key);

        // Replace the first word with another valid wordlist word, which
        // will almost certainly break the checksum (and very likely change
        // the decoded entropy too).
        let mut words: Vec<&str> = phrase.split_whitespace().collect();
        words[0] = if words[0] == "abandon" {
            "ability"
        } else {
            "abandon"
        };
        let corrupted = words.join(" ");

        // Either the checksum catches it, or (extremely unlikely) it still
        // parses but must then decode to a *different* key than the
        // original — never silently succeed with the same key.
        match signing_key_from_seed_phrase(&corrupted) {
            Err(_) => {}
            Ok(recovered) => {
                assert_ne!(recovered.to_bytes(), signing_key.to_bytes());
            }
        }
    }

    /// Two different keys must never export to the same phrase — otherwise
    /// the "backup" would be meaningless.
    #[test]
    fn different_keys_export_to_different_phrases() {
        let phrase_a = export_seed_phrase(&SigningKey::from_bytes(&[1u8; 32]));
        let phrase_b = export_seed_phrase(&SigningKey::from_bytes(&[2u8; 32]));
        assert_ne!(phrase_a, phrase_b);
    }
}
