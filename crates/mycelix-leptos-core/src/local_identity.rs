// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Local agent identity — per-visitor cryptographic DID stored in localStorage.
//!
//! # Steps 1–4 of the recoverable-identity roadmap
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
//! 4. **DONE (this pass)**: register the browser Ed25519 key on the real
//!    on-DHT DID document via [`crate::did_registry`]
//!    ([`crate::did_registry::ensure_did_anchored`] /
//!    [`crate::did_registry::ensure_browser_key_anchored`]). Corrected
//!    framing versus the original roadmap text below: `create_did()` always
//!    anchors the DID to the Holochain conductor's own `AgentPubKey` (it
//!    takes no input), so the browser key can't become *the* anchor —
//!    instead it's registered as an *additional* verification method on the
//!    document. See [`crate::did_registry`]'s module docs for exactly what
//!    is and isn't verified end-to-end so far.
//! 5. Wire `mycelix-identity/zomes/recovery` (guardian/trustee threshold
//!    voting + progressive self-recovery verification anchors) into a
//!    recovery UI, so a lost or reset device can re-establish control of the
//!    same DID instead of silently minting a new one.
//! 6. Multi-device pairing (QR / link-code handoff of an *authorization*,
//!    never of the raw private key).
//!
//! None of steps 5–6 are implemented here. `mycelix-crypto`'s `wasm` feature
//! (see `mycelix-identity/crates/mycelix-crypto/`) was evaluated for local
//! keygen (steps 1-3) but only exposes types/validation under `wasm32` —
//! all signing/keygen there is `#[cfg(feature = "native")]` — so it cannot
//! back in-browser keygen as-is (it's used for step 4 instead, purely for
//! its multibase key-encoding types, which don't require the native
//! feature). `ed25519-dalek` is used directly for keygen, matching the
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
use std::future::Future;
#[cfg(target_arch = "wasm32")]
use std::rc::Rc;
use zeroize::{Zeroize, Zeroizing};

#[cfg(target_arch = "wasm32")]
use js_sys::{Function, Promise, Reflect};
#[cfg(target_arch = "wasm32")]
use wasm_bindgen::{closure::Closure, JsCast, JsValue};
#[cfg(target_arch = "wasm32")]
use wasm_bindgen_futures::{future_to_promise, JsFuture};

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

/// Non-secret, versioned UX marker. The stored value is the current public key,
/// so an old confirmation cannot accidentally bless a different identity.
/// This marker is not an authorization boundary; same-origin JavaScript can
/// modify browser storage. Its purpose is to prevent accidental bypass after
/// reload, not to replace proof of key possession.
const IDENTITY_BACKUP_CONFIRMED_PUBLIC_KEY: &str =
    "mycelix_identity_recovery_backup_confirmed_public_ed25519_b64_v1";

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

/// Read-only summary of identity storage posture.
///
/// This type intentionally carries no key material, passphrase, mnemonic, or
/// raw storage values. A posture is not a claim that the identity has been
/// recovered, that its backup works, or that browser storage is tamper-proof.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LocalIdentityStoragePosture {
    /// Browser storage could not be inspected. This is not the same as no identity.
    StorageUnavailable,
    /// No legacy, plaintext, public-key, or encrypted-key value was found.
    NoIdentity,
    /// Only a pre-cryptographic legacy DID label is present and needs migration.
    LegacyIdentityNeedsMigration,
    /// A structurally consistent plaintext Ed25519 keypair is stored at rest.
    UnprotectedIdentity,
    /// An encrypted key container and public key exist; this session has not unlocked it.
    ProtectedLocked,
    /// An encrypted container is present and a matching key exists in this session.
    /// This does not authenticate the currently persisted ciphertext.
    ProtectedUnlockedInSession,
    /// Stored key fields are malformed, incomplete, or contradictory.
    CorruptOrInconsistent,
}

/// A security-sensitive action that can be performed during identity bootstrap.
///
/// Keep this transition matrix pure and independent from the UI. Adding a new
/// action requires a state-authorization test; an empty action set means that
/// state is blocked from mutation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IdentityBootstrapAction {
    CreateProtectedIdentity,
    ProtectExistingIdentity,
    UnlockProtectedIdentity,
    RestoreProtectedIdentityFromSeedPhrase,
    ExportRecoveryPhrase,
    RotatePassphrase,
}

/// Return the only bootstrap mutations permitted for the observed local posture.
///
/// This is a policy decision, not proof that a storage operation has succeeded.
/// The operation implementation must still recheck state immediately before
/// writes and verify its read-back.
pub fn allowed_identity_bootstrap_actions(
    posture: LocalIdentityStoragePosture,
) -> &'static [IdentityBootstrapAction] {
    use IdentityBootstrapAction as Action;
    match posture {
        LocalIdentityStoragePosture::NoIdentity => &[
            Action::CreateProtectedIdentity,
            Action::RestoreProtectedIdentityFromSeedPhrase,
        ],
        LocalIdentityStoragePosture::UnprotectedIdentity => &[Action::ProtectExistingIdentity],
        LocalIdentityStoragePosture::ProtectedLocked => &[Action::UnlockProtectedIdentity],
        LocalIdentityStoragePosture::ProtectedUnlockedInSession => {
            &[Action::ExportRecoveryPhrase, Action::RotatePassphrase]
        }
        LocalIdentityStoragePosture::LegacyIdentityNeedsMigration
        | LocalIdentityStoragePosture::StorageUnavailable
        | LocalIdentityStoragePosture::CorruptOrInconsistent => &[],
    }
}

pub fn identity_bootstrap_action_allowed(
    posture: LocalIdentityStoragePosture,
    action: IdentityBootstrapAction,
) -> bool {
    allowed_identity_bootstrap_actions(posture).contains(&action)
}

#[derive(Default)]
struct IdentityStorageSnapshot {
    storage_available: bool,
    legacy_did: Option<String>,
    secret_b64: Option<String>,
    public_b64: Option<String>,
    encrypted_wire: Option<String>,
}

/// Inspect browser identity storage without generating, unlocking, replacing,
/// or writing any key material.
///
/// This must run before UI bootstrap code invokes provide_local_identity,
/// local_did or ensure_keypair. Storage access errors fail closed as
/// StorageUnavailable, never as no identity. For an encrypted identity, this
/// checks the persisted envelope shape only; the ciphertext's authenticated
/// integrity is verified when the user unlocks.
pub fn inspect_local_identity_posture() -> LocalIdentityStoragePosture {
    let mut snapshot = match read_identity_storage_snapshot() {
        Ok(snapshot) => snapshot,
        Err(()) => {
            return LocalIdentityStoragePosture::StorageUnavailable;
        }
    };
    let posture = classify_identity_storage_snapshot(&snapshot);
    let result = if posture == LocalIdentityStoragePosture::ProtectedLocked
        && snapshot
            .public_b64
            .as_deref()
            .is_some_and(unlocked_cache_matches_public_key)
    {
        LocalIdentityStoragePosture::ProtectedUnlockedInSession
    } else {
        posture
    };
    if let Some(secret_b64) = snapshot.secret_b64.as_mut() {
        secret_b64.zeroize();
    }
    result
}

fn read_identity_storage_snapshot() -> Result<IdentityStorageSnapshot, ()> {
    let window = web_sys::window().ok_or(())?;
    let storage = window.local_storage().map_err(|_| ())?.ok_or(())?;
    read_identity_storage_snapshot_from(&storage)
}

fn read_identity_storage_snapshot_from(
    storage: &web_sys::Storage,
) -> Result<IdentityStorageSnapshot, ()> {
    let legacy_did = storage.get_item(LEGACY_DID_KEY).map_err(|_| ())?;
    let mut secret_b64 = storage.get_item(IDENTITY_SECRET_KEY).map_err(|_| ())?;
    let public_b64 = match storage.get_item(IDENTITY_PUBLIC_KEY) {
        Ok(value) => value,
        Err(_) => {
            if let Some(secret) = secret_b64.as_mut() {
                secret.zeroize();
            }
            return Err(());
        }
    };
    let encrypted_wire = match storage.get_item(IDENTITY_SECRET_ENCRYPTED_KEY) {
        Ok(value) => value,
        Err(_) => {
            if let Some(secret) = secret_b64.as_mut() {
                secret.zeroize();
            }
            return Err(());
        }
    };

    Ok(IdentityStorageSnapshot {
        storage_available: true,
        legacy_did,
        secret_b64,
        public_b64,
        encrypted_wire,
    })
}

fn classify_identity_storage_snapshot(
    snapshot: &IdentityStorageSnapshot,
) -> LocalIdentityStoragePosture {
    if !snapshot.storage_available {
        return LocalIdentityStoragePosture::StorageUnavailable;
    }

    if let Some(encrypted_wire) = snapshot.encrypted_wire.as_deref() {
        // Protected storage must not leave a second plaintext copy of the key.
        if snapshot.secret_b64.is_some() {
            return LocalIdentityStoragePosture::CorruptOrInconsistent;
        }
        let public_key_is_well_formed = snapshot
            .public_b64
            .as_deref()
            .and_then(base64_decode)
            .is_some_and(|bytes| bytes.len() == 32);
        if public_key_is_well_formed && encrypted_blob_is_structurally_valid(encrypted_wire) {
            return LocalIdentityStoragePosture::ProtectedLocked;
        }
        return LocalIdentityStoragePosture::CorruptOrInconsistent;
    }

    match (
        snapshot.secret_b64.as_deref(),
        snapshot.public_b64.as_deref(),
    ) {
        (Some(secret_b64), Some(public_b64)) => {
            if public_key_matches_secret(secret_b64, public_b64) {
                LocalIdentityStoragePosture::UnprotectedIdentity
            } else {
                LocalIdentityStoragePosture::CorruptOrInconsistent
            }
        }
        (None, None) => {
            if snapshot
                .legacy_did
                .as_deref()
                .is_some_and(|did| !did.trim().is_empty())
            {
                LocalIdentityStoragePosture::LegacyIdentityNeedsMigration
            } else {
                LocalIdentityStoragePosture::NoIdentity
            }
        }
        // A lone public key or plaintext secret is not a usable identity.
        _ => LocalIdentityStoragePosture::CorruptOrInconsistent,
    }
}

fn unlocked_cache_matches_public_key(public_b64: &str) -> bool {
    let Some(public_bytes) = base64_decode(public_b64) else {
        return false;
    };
    if public_bytes.len() != 32 {
        return false;
    }
    UNLOCKED_KEY.with(|cell| {
        cell.borrow().as_ref().is_some_and(|signing_key| {
            public_bytes.as_slice() == signing_key.verifying_key().as_bytes()
        })
    })
}

fn public_key_matches_signing_key(public_b64: &str, signing_key: &SigningKey) -> bool {
    let Some(public_bytes) = base64_decode(public_b64) else {
        return false;
    };
    public_bytes.as_slice() == signing_key.verifying_key().as_bytes()
}

fn signing_key_from_secret_b64(secret_b64: &str) -> Result<SigningKey, String> {
    let mut secret_bytes =
        base64_decode(secret_b64).ok_or_else(|| "Stored secret key is malformed.".to_string())?;
    let secret_array_result: Result<[u8; 32], _> = secret_bytes.as_slice().try_into();
    secret_bytes.zeroize();
    let mut secret_array =
        secret_array_result.map_err(|_| "Stored secret key has the wrong length.".to_string())?;
    let signing_key = SigningKey::from_bytes(&secret_array);
    secret_array.zeroize();
    Ok(signing_key)
}

fn public_key_matches_secret(secret_b64: &str, public_b64: &str) -> bool {
    let Ok(signing_key) = signing_key_from_secret_b64(secret_b64) else {
        return false;
    };
    public_key_matches_signing_key(public_b64, &signing_key)
}

fn encrypted_blob_is_structurally_valid(encoded: &str) -> bool {
    let Ok(wire) = serde_json::from_str::<EncryptedBlobWire>(encoded) else {
        return false;
    };
    let Ok(blob) = EncryptedBlob::try_from(&wire) else {
        return false;
    };
    blob.salt.len() == identity_crypto::SALT_LEN
        && blob.iv.len() == identity_crypto::IV_LEN
        && blob.ciphertext.len() == 48
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
        let posture = inspect_local_identity_posture();
        let cache_matches_storage = match posture {
            LocalIdentityStoragePosture::ProtectedUnlockedInSession => true,
            LocalIdentityStoragePosture::UnprotectedIdentity => load_signing_key()
                .is_some_and(|stored| stored.verifying_key() == signing_key.verifying_key()),
            _ => false,
        };
        if cache_matches_storage {
            return signing_key;
        }
        UNLOCKED_KEY.with(|cell| *cell.borrow_mut() = None);
        panic!(
            "Cached local identity no longer matches a recognized persisted identity posture. \
             No key was replaced. Inspect storage and use the explicit recovery path."
        );
    }

    if let Some(signing_key) = load_signing_key() {
        if inspect_local_identity_posture() != LocalIdentityStoragePosture::UnprotectedIdentity {
            panic!(
                "Stored key material exists, but identity posture is not a consistent unprotected keypair. \
                 No replacement identity was created."
            );
        }
        UNLOCKED_KEY.with(|cell| *cell.borrow_mut() = Some(signing_key.clone()));
        return signing_key;
    }

    match inspect_local_identity_posture() {
        LocalIdentityStoragePosture::NoIdentity => {
            // Legacy compatibility: some non-Personal callers still rely on
            // first-use key creation. This path is intentionally isolated to
            // a positively identified empty storage state. It remains
            // unprotected and is tracked for removal by SEC-IDENTITY-003.
            let signing_key = generate_signing_key();
            store_signing_key(&signing_key);
            if inspect_local_identity_posture() != LocalIdentityStoragePosture::UnprotectedIdentity
                || !load_signing_key()
                    .is_some_and(|stored| stored.verifying_key() == signing_key.verifying_key())
            {
                UNLOCKED_KEY.with(|cell| *cell.borrow_mut() = None);
                panic!(
                    "Legacy identity creation could not be verified after storage write. \
                     No key was treated as persisted."
                );
            }
            UNLOCKED_KEY.with(|cell| *cell.borrow_mut() = Some(signing_key.clone()));
            signing_key
        }
        LocalIdentityStoragePosture::ProtectedLocked => panic!(
            "Local identity is passphrase-protected but locked this session. \
             Call `unlock_with_passphrase()` before accessing it. No replacement identity was created."
        ),
        LocalIdentityStoragePosture::LegacyIdentityNeedsMigration => panic!(
            "A legacy identity marker exists and requires explicit migration. No replacement identity was created."
        ),
        LocalIdentityStoragePosture::StorageUnavailable => {
            panic!("Local identity storage is unavailable. No replacement identity was created.")
        }
        LocalIdentityStoragePosture::CorruptOrInconsistent => panic!(
            "Local identity storage is corrupt or inconsistent. Preserve it and use the explicit recovery path."
        ),
        LocalIdentityStoragePosture::UnprotectedIdentity => panic!(
            "A keypair appears to exist but could not be loaded consistently. No replacement identity was created."
        ),
        LocalIdentityStoragePosture::ProtectedUnlockedInSession => panic!(
            "The protected identity posture is unlocked, but no valid in-memory key is available."
        ),
    }
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

/// Create a new identity directly into passphrase-protected storage.
///
/// Unlike `ensure_keypair` followed by `protect_with_passphrase`, this path
/// never calls `store_signing_key` and never writes the plaintext secret to
/// localStorage. It refuses to overwrite any recognized existing identity
/// posture. Writes are checked by reading the encrypted envelope and public
/// key back before the in-memory key is marked available.
///
/// This function still depends on browser storage semantics and cannot promise
/// confidentiality from a compromised active page, malicious extensions, or
/// a compromised browser profile.
/// Generate three distinct, one-based word positions for backup confirmation.
///
/// Positions are selected with the browser CSPRNG, are kept in caller memory
/// only, and contain no secret material. This is a usability check that the
/// person copied the phrase, not cryptographic proof that an offline backup
/// will remain available later.
pub fn recovery_phrase_challenge_positions() -> Result<[usize; 3], String> {
    let window =
        web_sys::window().ok_or_else(|| "Browser Web Crypto is unavailable.".to_string())?;
    let crypto = window
        .crypto()
        .map_err(|_| "Browser Web Crypto is unavailable.".to_string())?;
    let mut positions = [0usize; 3];

    for slot in 0..positions.len() {
        let mut selected = None;
        for _ in 0..64 {
            let mut sample = [0u8; 1];
            if crypto.get_random_values_with_u8_array(&mut sample).is_err() {
                sample.zeroize();
                return Err("Could not generate recovery confirmation positions.".into());
            }
            let value = sample[0];
            sample.zeroize();
            // 240 is the largest multiple of 24 below 256; rejection avoids
            // modulo bias in the 1..=24 position range.
            if value >= 240 {
                continue;
            }
            let candidate = usize::from(value % 24) + 1;
            if !positions[..slot].contains(&candidate) {
                selected = Some(candidate);
                break;
            }
        }
        positions[slot] = selected.ok_or_else(|| {
            "Could not generate distinct recovery confirmation positions.".to_string()
        })?;
    }

    positions.sort_unstable();
    Ok(positions)
}

/// Return true when the currently stored protected identity has not passed
/// the local recovery-backup confirmation step.
///
/// Storage errors, malformed posture and a missing/mismatched marker fail
/// closed. The marker contains only the public key and is a UX continuity
/// hint, not a security boundary against same-origin script.
pub fn recovery_backup_confirmation_needed() -> bool {
    let Ok(window) = web_sys::window().ok_or(()) else {
        return true;
    };
    let Ok(Some(storage)) = window.local_storage() else {
        return true;
    };
    let Ok(mut snapshot) = read_identity_storage_snapshot_from(&storage) else {
        return true;
    };
    let posture = classify_identity_storage_snapshot(&snapshot);
    let public_b64 = snapshot.public_b64.clone();
    if let Some(secret_b64) = snapshot.secret_b64.as_mut() {
        secret_b64.zeroize();
    }
    if posture != LocalIdentityStoragePosture::ProtectedLocked {
        return true;
    }
    let Some(public_b64) = public_b64 else {
        return true;
    };
    match storage.get_item(IDENTITY_BACKUP_CONFIRMED_PUBLIC_KEY) {
        Ok(Some(confirmed_public_b64)) => confirmed_public_b64 != public_b64,
        Ok(None) | Err(_) => true,
    }
}

/// Persist a non-secret confirmation marker only after the user has completed
/// the recovery-word challenge. Bind it to the public key in current storage
/// and verify the marker is still bound to the same protected envelope.
pub fn mark_recovery_backup_confirmed() -> Result<(), String> {
    let signing_key = UNLOCKED_KEY
        .with(|cell| cell.borrow().clone())
        .ok_or_else(|| "Protected identity is not unlocked in this session.".to_string())?;
    if !identity_bootstrap_action_allowed(
        LocalIdentityStoragePosture::ProtectedUnlockedInSession,
        IdentityBootstrapAction::ExportRecoveryPhrase,
    ) {
        return Err("Recovery backup confirmation is not permitted in this state.".into());
    }

    let window =
        web_sys::window().ok_or_else(|| "Browser identity storage is unavailable.".to_string())?;
    let storage = window
        .local_storage()
        .map_err(|_| "Browser identity storage is unavailable.".to_string())?
        .ok_or_else(|| "Browser identity storage is unavailable.".to_string())?;
    let mut snapshot = read_identity_storage_snapshot_from(&storage)
        .map_err(|_| "Protected identity storage could not be inspected.".to_string())?;
    let posture = classify_identity_storage_snapshot(&snapshot);
    let public_b64 = snapshot.public_b64.clone();
    let envelope = snapshot.encrypted_wire.clone();
    if let Some(secret_b64) = snapshot.secret_b64.as_mut() {
        secret_b64.zeroize();
    }
    let Some(public_b64) = public_b64 else {
        return Err("Protected identity public-key metadata is missing.".into());
    };
    if posture != LocalIdentityStoragePosture::ProtectedLocked
        || !public_key_matches_signing_key(&public_b64, &signing_key)
        || envelope
            .as_deref()
            .is_none_or(|value| !encrypted_blob_is_structurally_valid(value))
    {
        return Err(
            "Protected identity changed or is inconsistent; backup confirmation was not saved."
                .into(),
        );
    }

    storage
        .set_item(IDENTITY_BACKUP_CONFIRMED_PUBLIC_KEY, &public_b64)
        .map_err(|_| "Could not persist recovery-backup confirmation.".to_string())?;
    let marker = storage
        .get_item(IDENTITY_BACKUP_CONFIRMED_PUBLIC_KEY)
        .map_err(|_| "Could not verify recovery-backup confirmation.".to_string())?;
    let mut current = read_identity_storage_snapshot_from(&storage).map_err(|_| {
        "Could not re-check protected identity after backup confirmation.".to_string()
    })?;
    let unchanged = current.secret_b64.is_none()
        && current.public_b64.as_deref() == Some(public_b64.as_str())
        && current.encrypted_wire.as_deref() == envelope.as_deref()
        && public_key_matches_signing_key(&public_b64, &signing_key)
        && classify_identity_storage_snapshot(&current)
            == LocalIdentityStoragePosture::ProtectedLocked;
    if let Some(secret_b64) = current.secret_b64.as_mut() {
        secret_b64.zeroize();
    }
    if marker.as_deref() != Some(public_b64.as_str()) || !unchanged {
        return Err(
            "Protected identity changed while backup confirmation was being recorded.".into(),
        );
    }
    Ok(())
}

/// Export a recovery phrase for the currently unlocked protected identity.
///
/// This function is read-only: it never calls `ensure_keypair`, never mutates
/// storage, and refuses to export from plaintext, locked, legacy, unavailable
/// or inconsistent identity state. The returned phrase is secret material;
/// callers must display it only in an explicit user-initiated recovery flow,
/// never put it into telemetry/URLs/logs or persist it back into web storage.
pub fn export_unlocked_identity_seed_phrase() -> Result<String, String> {
    let signing_key = UNLOCKED_KEY
        .with(|cell| cell.borrow().clone())
        .ok_or_else(|| "Protected identity is not unlocked in this session.".to_string())?;
    let mut snapshot = read_identity_storage_snapshot().map_err(|_| {
        "Identity storage is unavailable; no recovery phrase was exported.".to_string()
    })?;
    let posture = classify_identity_storage_snapshot(&snapshot);
    let public_b64 = snapshot.public_b64.clone();
    let envelope = snapshot.encrypted_wire.clone();
    if let Some(secret_b64) = snapshot.secret_b64.as_mut() {
        secret_b64.zeroize();
    }

    if posture != LocalIdentityStoragePosture::ProtectedLocked
        || envelope
            .as_deref()
            .is_none_or(|value| !encrypted_blob_is_structurally_valid(value))
        || public_b64
            .as_deref()
            .is_none_or(|value| !public_key_matches_signing_key(value, &signing_key))
    {
        return Err(
            "Recovery export requires a consistent protected identity unlocked in this session."
                .into(),
        );
    }
    if !identity_bootstrap_action_allowed(
        LocalIdentityStoragePosture::ProtectedUnlockedInSession,
        IdentityBootstrapAction::ExportRecoveryPhrase,
    ) {
        return Err(
            "Recovery phrase export is not permitted in the current identity state.".into(),
        );
    }

    Ok(export_seed_phrase(&signing_key))
}

pub async fn create_protected_identity(
    passphrase: &str,
) -> Result<LocalIdentityStoragePosture, String> {
    let passphrase = Zeroizing::new(passphrase.to_owned());
    with_identity_operation_lock(move || async move {
        create_protected_identity_unlocked(&passphrase).await
    })
    .await
}

async fn create_protected_identity_unlocked(
    passphrase: &str,
) -> Result<LocalIdentityStoragePosture, String> {
    validate_new_identity_passphrase(passphrase)?;
    ensure_empty_identity_storage()?;
    let signing_key = try_generate_signing_key()?;
    persist_protected_signing_key(signing_key, passphrase).await
}

/// Restore a BIP-39 identity directly into protected storage on an empty device.
/// Restore a BIP-39 identity directly into protected storage on an empty device.
///
/// This intentionally refuses to replace any recognized existing local
/// identity. Replacement of an existing identity needs a separate, explicit
/// confirmation flow with an expected-current-identity check.
pub async fn restore_protected_identity_from_seed_phrase(
    phrase: &str,
    passphrase: &str,
) -> Result<LocalIdentityStoragePosture, String> {
    let phrase = Zeroizing::new(phrase.to_owned());
    let passphrase = Zeroizing::new(passphrase.to_owned());
    with_identity_operation_lock(move || async move {
        restore_protected_identity_unlocked(&phrase, &passphrase).await
    })
    .await
}

async fn restore_protected_identity_unlocked(
    phrase: &str,
    passphrase: &str,
) -> Result<LocalIdentityStoragePosture, String> {
    validate_new_identity_passphrase(passphrase)?;
    ensure_empty_identity_storage()?;
    let signing_key = signing_key_from_seed_phrase(phrase)?;
    persist_protected_signing_key(signing_key, passphrase).await
}

const IDENTITY_LIFECYCLE_LOCK_NAME: &str = "mycelix.identity.lifecycle.v1";

/// Serialize asynchronous protected-identity transitions across same-origin
/// tabs and workers. The lock name is origin-scoped, not identity-scoped,
/// because the local identity may not yet be known when creation begins.
///
/// This is cooperative serialization rather than a storage transaction:
/// every caller still rechecks state and verifies postconditions. Synchronous
/// legacy writers remain outside this lock and are tracked separately.
#[cfg(target_arch = "wasm32")]
async fn with_identity_operation_lock<T, F, Fut>(operation: F) -> Result<T, String>
where
    T: 'static,
    F: FnOnce() -> Fut + 'static,
    Fut: Future<Output = Result<T, String>> + 'static,
{
    let window = web_sys::window().ok_or_else(|| {
        "Browser window is unavailable; identity operation was not started.".to_string()
    })?;
    let navigator: JsValue = window.navigator().into();
    let locks = Reflect::get(&navigator, &JsValue::from_str("locks"))
        .map_err(|error| format!("Web Locks API inspection failed: {}", js_value_message(error)))?;
    if locks.is_null() || locks.is_undefined() {
        return Err(
            "This browser does not expose the Web Locks API; no identity change was attempted."
                .into(),
        );
    }
    let request = Reflect::get(&locks, &JsValue::from_str("request"))
        .map_err(|error| format!("Web Lock request is unavailable: {}", js_value_message(error)))?
        .dyn_into::<Function>()
        .map_err(|_| "Web Lock request is not callable; no identity change was attempted.".to_string())?;

    let output = Rc::new(RefCell::new(None::<T>));
    let operation_error = Rc::new(RefCell::new(None::<String>));
    let output_for_callback = output.clone();
    let error_for_callback = operation_error.clone();
    let mut operation = Some(operation);
    let callback = Closure::wrap(Box::new(move |_lock: JsValue| -> Promise {
        let Some(operation) = operation.take() else {
            return Promise::reject(&JsValue::from_str(
                "Identity lock callback unexpectedly ran more than once.",
            ));
        };
        let output = output_for_callback.clone();
        let operation_error = error_for_callback.clone();
        future_to_promise(async move {
            match operation().await {
                Ok(value) => {
                    *output.borrow_mut() = Some(value);
                    Ok(JsValue::UNDEFINED)
                }
                Err(error) => {
                    *operation_error.borrow_mut() = Some(error.clone());
                    Err(JsValue::from_str(&error))
                }
            }
        })
    }) as Box<dyn FnMut(JsValue) -> Promise>);

    let promise = request
        .call2(
            &locks,
            &JsValue::from_str(IDENTITY_LIFECYCLE_LOCK_NAME),
            callback.as_ref(),
        )
        .map_err(|error| format!("Could not acquire identity lifecycle lock: {}", js_value_message(error)))?
        .dyn_into::<Promise>()
        .map_err(|_| "Web Lock request did not return a Promise.".to_string())?;
    let completion = JsFuture::from(promise).await;
    drop(callback);

    match completion {
        Ok(_) => output
            .borrow_mut()
            .take()
            .ok_or_else(|| "Identity lock completed without an operation result.".to_string()),
        Err(error) => Err(operation_error
            .borrow_mut()
            .take()
            .unwrap_or_else(|| format!("Identity lifecycle lock failed: {}", js_value_message(error)))),
    }
}

#[cfg(target_arch = "wasm32")]
fn js_value_message(value: JsValue) -> String {
    value
        .as_string()
        .unwrap_or_else(|| format!("{value:?}"))
}

/// Native state tests use the same transition body without a browser lock.
#[cfg(not(target_arch = "wasm32"))]
async fn with_identity_operation_lock<T, F, Fut>(operation: F) -> Result<T, String>
where
    F: FnOnce() -> Fut,
    Fut: Future<Output = Result<T, String>>,
{
    operation().await
}

fn validate_new_identity_passphrase(passphrase: &str) -> Result<(), String> {
    if passphrase.trim().is_empty() {
        return Err("Passphrase must not be empty".into());
    }
    Ok(())
}

fn clear_recovery_backup_confirmation(storage: &web_sys::Storage) -> Result<(), String> {
    if storage
        .remove_item(IDENTITY_BACKUP_CONFIRMED_PUBLIC_KEY)
        .is_err()
    {
        return Err(
            "Could not reset recovery-backup confirmation; no identity change was made.".into(),
        );
    }
    match storage.get_item(IDENTITY_BACKUP_CONFIRMED_PUBLIC_KEY) {
        Ok(None) => Ok(()),
        Ok(Some(_)) | Err(_) => Err(
            "Could not verify reset of recovery-backup confirmation; no identity change was made."
                .into(),
        ),
    }
}

fn ensure_empty_identity_storage() -> Result<(), String> {
    if UNLOCKED_KEY.with(|cell| cell.borrow().is_some()) {
        return Err(
            "An in-memory identity key already exists; inspect identity posture before creating or restoring."
                .into(),
        );
    }
    let mut snapshot = read_identity_storage_snapshot()
        .map_err(|_| "Browser identity storage could not be inspected.".to_string())?;
    let posture = classify_identity_storage_snapshot(&snapshot);
    if let Some(secret_b64) = snapshot.secret_b64.as_mut() {
        secret_b64.zeroize();
    }
    if posture != LocalIdentityStoragePosture::NoIdentity {
        return Err(
            "Identity storage is not empty. Inspect or recover the existing identity before creating another."
                .into(),
        );
    }
    Ok(())
}

async fn persist_protected_signing_key(
    signing_key: SigningKey,
    passphrase: &str,
) -> Result<LocalIdentityStoragePosture, String> {
    validate_new_identity_passphrase(passphrase)?;

    let window = web_sys::window()
        .ok_or_else(|| "Browser window is unavailable; no identity was created.".to_string())?;
    let storage = window
        .local_storage()
        .map_err(|_| "Browser identity storage is unavailable.".to_string())?
        .ok_or_else(|| "Browser identity storage is unavailable.".to_string())?;
    let mut snapshot = read_identity_storage_snapshot_from(&storage)
        .map_err(|_| "Browser identity storage could not be inspected.".to_string())?;
    let initial_posture = classify_identity_storage_snapshot(&snapshot);
    if let Some(secret_b64) = snapshot.secret_b64.as_mut() {
        secret_b64.zeroize();
    }
    if initial_posture != LocalIdentityStoragePosture::NoIdentity {
        return Err(
            "Identity storage is not empty. Inspect or recover the existing identity before creating another."
                .into(),
        );
    }

    let blob = identity_crypto::encrypt(passphrase, signing_key.as_bytes()).await?;
    let encrypted_wire = serde_json::to_string(&EncryptedBlobWire::from(&blob))
        .map_err(|_| "Could not serialize the protected identity envelope.".to_string())?;
    let public_b64 = base64_encode(signing_key.verifying_key().as_bytes());

    // Encryption yields to the browser event loop. Recheck before any write so
    // a second tab cannot silently overwrite a newly created identity.
    let mut current = read_identity_storage_snapshot_from(&storage)
        .map_err(|_| "Browser identity storage became unavailable.".to_string())?;
    let storage_still_empty = classify_identity_storage_snapshot(&current)
        == LocalIdentityStoragePosture::NoIdentity
        && !UNLOCKED_KEY.with(|cell| cell.borrow().is_some());
    if let Some(secret_b64) = current.secret_b64.as_mut() {
        secret_b64.zeroize();
    }
    if !storage_still_empty {
        return Err(
            "Identity storage changed during creation or recovery. No new identity was written."
                .into(),
        );
    }
    clear_recovery_backup_confirmation(&storage)?;

    if storage
        .set_item(IDENTITY_SECRET_ENCRYPTED_KEY, &encrypted_wire)
        .is_err()
    {
        // Storage.setItem is atomic by contract, but verify/cleanup fail closed
        // rather than relying on a reported write result as proof of state.
        if remove_storage_item_if_matches(&storage, IDENTITY_SECRET_ENCRYPTED_KEY, &encrypted_wire)
            .is_err()
        {
            return Err(
                "Protected identity write failed and cleanup could not be confirmed; inspect posture before retrying."
                    .into(),
            );
        }
        return Err("Could not persist the protected identity envelope.".into());
    }
    if storage.set_item(IDENTITY_PUBLIC_KEY, &public_b64).is_err() {
        let encrypted_cleanup = remove_storage_item_if_matches(
            &storage,
            IDENTITY_SECRET_ENCRYPTED_KEY,
            &encrypted_wire,
        );
        let public_cleanup =
            remove_storage_item_if_matches(&storage, IDENTITY_PUBLIC_KEY, &public_b64);
        if encrypted_cleanup.is_err() || public_cleanup.is_err() {
            return Err(
                "Identity creation was interrupted; concurrent values were preserved where detectable. Inspect posture before retrying."
                    .into(),
            );
        }
        return Err("Could not persist public-key metadata; no identity was activated.".into());
    }

    let mut persisted = read_identity_storage_snapshot_from(&storage);
    let verified = persisted.as_ref().is_ok_and(|stored| {
        stored.secret_b64.is_none()
            && stored.public_b64.as_deref() == Some(public_b64.as_str())
            && stored.encrypted_wire.as_deref() == Some(encrypted_wire.as_str())
            && classify_identity_storage_snapshot(stored)
                == LocalIdentityStoragePosture::ProtectedLocked
    });
    if let Ok(stored) = persisted.as_mut() {
        if let Some(secret_b64) = stored.secret_b64.as_mut() {
            secret_b64.zeroize();
        }
    }
    if !verified {
        let encrypted_cleanup = remove_storage_item_if_matches(
            &storage,
            IDENTITY_SECRET_ENCRYPTED_KEY,
            &encrypted_wire,
        );
        let public_cleanup =
            remove_storage_item_if_matches(&storage, IDENTITY_PUBLIC_KEY, &public_b64);
        UNLOCKED_KEY.with(|cell| *cell.borrow_mut() = None);
        if encrypted_cleanup.is_err() || public_cleanup.is_err() {
            return Err(
                "Identity storage verification failed; different concurrent values were preserved where detectable. Inspect posture before retrying."
                    .into(),
            );
        }
        return Err(
            "Identity storage verification failed; the new identity was not activated.".into(),
        );
    }

    UNLOCKED_KEY.with(|cell| *cell.borrow_mut() = Some(signing_key));
    Ok(LocalIdentityStoragePosture::ProtectedUnlockedInSession)
}

/// Protect an existing, structurally consistent plaintext identity in place.
///
/// This is separate from the legacy protect_with_passphrase API because the
/// latter uses best-effort storage helpers. This path checks every write and
/// read-back, only removes the prior plaintext secret after the encrypted
/// envelope has been read back successfully, and attempts to restore the
/// unprotected state if the final protected-state verification fails.
pub async fn protect_existing_identity(
    passphrase: &str,
) -> Result<LocalIdentityStoragePosture, String> {
    let passphrase = Zeroizing::new(passphrase.to_owned());
    with_identity_operation_lock(move || async move {
        protect_existing_identity_unlocked(&passphrase).await
    })
    .await
}

async fn protect_existing_identity_unlocked(
    passphrase: &str,
) -> Result<LocalIdentityStoragePosture, String> {
    validate_new_identity_passphrase(passphrase)?;

    let window = web_sys::window()
        .ok_or_else(|| "Browser window is unavailable; identity was not changed.".to_string())?;
    let storage = window
        .local_storage()
        .map_err(|_| "Browser identity storage is unavailable.".to_string())?
        .ok_or_else(|| "Browser identity storage is unavailable.".to_string())?;
    let mut snapshot = read_identity_storage_snapshot_from(&storage)
        .map_err(|_| "Browser identity storage could not be inspected.".to_string())?;
    let initial_posture = classify_identity_storage_snapshot(&snapshot);
    if initial_posture != LocalIdentityStoragePosture::UnprotectedIdentity {
        if let Some(secret_b64) = snapshot.secret_b64.as_mut() {
            secret_b64.zeroize();
        }
        return Err(
            "Only a consistent unprotected identity can be protected by this operation; no storage was changed."
                .into(),
        );
    }

    let public_b64 = snapshot.public_b64.clone();
    let mut prior_secret_b64 = snapshot
        .secret_b64
        .take()
        .ok_or_else(|| "Existing plaintext secret is missing.".to_string())?;
    let signing_key_result = signing_key_from_secret_b64(&prior_secret_b64);
    prior_secret_b64.zeroize();
    let signing_key = signing_key_result?;
    let public_b64 =
        public_b64.ok_or_else(|| "Existing public-key metadata is missing.".to_string())?;

    if !public_key_matches_signing_key(&public_b64, &signing_key) {
        return Err("Existing identity secret/public key binding is inconsistent.".into());
    }
    if UNLOCKED_KEY.with(|cell| {
        cell.borrow()
            .as_ref()
            .is_some_and(|cached| cached.verifying_key() != signing_key.verifying_key())
    }) {
        return Err(
            "An in-session key does not match the stored identity; no storage was changed.".into(),
        );
    }

    let blob = identity_crypto::encrypt(passphrase, signing_key.as_bytes()).await?;
    let encrypted_wire = serde_json::to_string(&EncryptedBlobWire::from(&blob))
        .map_err(|_| "Could not serialize the protected identity envelope.".to_string())?;

    // Encryption yields to the browser event loop; recheck that this is still
    // the same unprotected key before writing any new state.
    let mut current = read_identity_storage_snapshot_from(&storage)
        .map_err(|_| "Browser identity storage became unavailable.".to_string())?;
    let same_unprotected_identity = classify_identity_storage_snapshot(&current)
        == LocalIdentityStoragePosture::UnprotectedIdentity
        && current.public_b64.as_deref() == Some(public_b64.as_str())
        && current
            .secret_b64
            .as_deref()
            .is_some_and(|secret| public_key_matches_secret(secret, &public_b64));
    if let Some(secret_b64) = current.secret_b64.as_mut() {
        secret_b64.zeroize();
    }
    if !same_unprotected_identity {
        return Err(
            "Identity storage changed during protection. No storage was modified by this operation."
                .into(),
        );
    }
    clear_recovery_backup_confirmation(&storage)?;

    if storage
        .set_item(IDENTITY_SECRET_ENCRYPTED_KEY, &encrypted_wire)
        .is_err()
    {
        if remove_storage_item_if_matches(&storage, IDENTITY_SECRET_ENCRYPTED_KEY, &encrypted_wire)
            .is_err()
        {
            return Err(
                "Protected write failed and cleanup could not be confirmed; inspect posture before retrying."
                    .into(),
            );
        }
        return Err("Could not persist the protected identity envelope.".into());
    }

    let mut stored_with_both = read_identity_storage_snapshot_from(&storage);
    let envelope_verified = stored_with_both.as_ref().is_ok_and(|stored| {
        stored.secret_b64.is_some()
            && stored.public_b64.as_deref() == Some(public_b64.as_str())
            && stored.encrypted_wire.as_deref() == Some(encrypted_wire.as_str())
            && public_key_matches_signing_key(&public_b64, &signing_key)
            && encrypted_blob_is_structurally_valid(&encrypted_wire)
    });
    if let Ok(stored) = stored_with_both.as_mut() {
        if let Some(secret_b64) = stored.secret_b64.as_mut() {
            secret_b64.zeroize();
        }
    }
    if !envelope_verified {
        if remove_storage_item_if_matches(&storage, IDENTITY_SECRET_ENCRYPTED_KEY, &encrypted_wire)
            .is_err()
        {
            return Err(
                "Envelope verification failed and cleanup was incomplete; inspect posture before retrying."
                    .into(),
            );
        }
        return Err(
            "Envelope verification failed; the original unprotected identity remains in storage."
                .into(),
        );
    }

    // Keep a short-lived rollback value in memory only until the final state
    // has been checked. It is zeroized on either exit path.
    let mut plaintext_rollback = base64_encode(signing_key.as_bytes());
    if storage.remove_item(IDENTITY_SECRET_KEY).is_err() {
        let cleanup = remove_storage_item_if_matches(
            &storage,
            IDENTITY_SECRET_ENCRYPTED_KEY,
            &encrypted_wire,
        );
        plaintext_rollback.zeroize();
        if cleanup.is_err() {
            return Err(
                "Plaintext removal failed; different concurrent envelope values were preserved where detectable. Inspect posture before retrying."
                    .into(),
            );
        }
        return Err(
            "Could not remove the plaintext key; the protected transition was rolled back.".into(),
        );
    }

    let mut final_snapshot = read_identity_storage_snapshot_from(&storage);
    let protected_verified = final_snapshot.as_ref().is_ok_and(|stored| {
        stored.secret_b64.is_none()
            && stored.public_b64.as_deref() == Some(public_b64.as_str())
            && stored.encrypted_wire.as_deref() == Some(encrypted_wire.as_str())
            && classify_identity_storage_snapshot(stored)
                == LocalIdentityStoragePosture::ProtectedLocked
    });
    if let Ok(stored) = final_snapshot.as_mut() {
        if let Some(secret_b64) = stored.secret_b64.as_mut() {
            secret_b64.zeroize();
        }
    }
    if !protected_verified {
        let restore_plaintext = storage.set_item(IDENTITY_SECRET_KEY, &plaintext_rollback);
        plaintext_rollback.zeroize();
        UNLOCKED_KEY.with(|cell| *cell.borrow_mut() = None);
        if restore_plaintext.is_err() {
            // Keep the already-verified encrypted envelope as the only possible
            // recovery copy; deleting it too could orphan the identity.
            return Err(
                "Protected-state verification failed and plaintext rollback was unavailable; encrypted recovery material was retained. Inspect posture before retrying."
                    .into(),
            );
        }
        let remove_envelope = remove_storage_item_if_matches(
            &storage,
            IDENTITY_SECRET_ENCRYPTED_KEY,
            &encrypted_wire,
        );
        if remove_envelope.is_err() {
            return Err(
                "Protected-state verification failed; plaintext was restored but a different/uncertain encrypted envelope was preserved. Inspect posture before retrying."
                    .into(),
            );
        }
        return Err(
            "Protected-state verification failed; the prior unprotected key was restored.".into(),
        );
    }
    plaintext_rollback.zeroize();

    UNLOCKED_KEY.with(|cell| *cell.borrow_mut() = Some(signing_key));
    Ok(LocalIdentityStoragePosture::ProtectedUnlockedInSession)
}

/// Create, protect, or rotate the passphrase for the local identity.
/// Create, protect, or rotate the passphrase for the local identity.
///
/// This entry point routes to the checked storage-transition APIs. New
/// identities are created directly in encrypted storage; an existing
/// unprotected identity is encrypted and read back before the plaintext
/// secret is removed; an already protected identity can rotate its
/// passphrase only while its key is unlocked in this session.
///
/// A protected-but-locked, legacy-only, unavailable, or inconsistent state
/// is rejected rather than triggering implicit key generation.
///
/// # Errors
///
/// Returns an error for empty passphrases, unavailable/corrupt storage,
/// unsuccessful persistence/read-back verification, or attempts to operate
/// on an identity that is locked or inconsistent.
pub async fn protect_with_passphrase(passphrase: &str) -> Result<(), String> {
    let passphrase = Zeroizing::new(passphrase.to_owned());
    with_identity_operation_lock(move || async move {
        protect_with_passphrase_unlocked(&passphrase).await
    })
    .await
}

async fn protect_with_passphrase_unlocked(passphrase: &str) -> Result<(), String> {
    validate_new_identity_passphrase(passphrase)?;
    match inspect_local_identity_posture() {
        LocalIdentityStoragePosture::NoIdentity => {
            create_protected_identity_unlocked(passphrase).await.map(|_| ())
        }
        LocalIdentityStoragePosture::UnprotectedIdentity => {
            protect_existing_identity_unlocked(passphrase).await.map(|_| ())
        }
        LocalIdentityStoragePosture::ProtectedUnlockedInSession => {
            rewrap_protected_identity(passphrase).await
        }
        LocalIdentityStoragePosture::ProtectedLocked => Err(
            "Identity is protected but locked; unlock it before changing the passphrase.".into(),
        ),
        LocalIdentityStoragePosture::LegacyIdentityNeedsMigration => Err(
            "A legacy identity label needs explicit migration or recovery; no key was generated."
                .into(),
        ),
        LocalIdentityStoragePosture::StorageUnavailable => {
            Err("Identity storage is unavailable; no identity was created or changed.".into())
        }
        LocalIdentityStoragePosture::CorruptOrInconsistent => Err(
            "Identity storage is inconsistent; inspect or recover it before changing protection."
                .into(),
        ),
    }
}

async fn rewrap_protected_identity(passphrase: &str) -> Result<(), String> {
    validate_new_identity_passphrase(passphrase)?;
    let signing_key = UNLOCKED_KEY
        .with(|cell| cell.borrow().clone())
        .ok_or_else(|| "Protected identity is not unlocked in this session.".to_string())?;

    let window = web_sys::window()
        .ok_or_else(|| "Browser window is unavailable; passphrase was not changed.".to_string())?;
    let storage = window
        .local_storage()
        .map_err(|_| "Browser identity storage is unavailable.".to_string())?
        .ok_or_else(|| "Browser identity storage is unavailable.".to_string())?;
    let mut snapshot = read_identity_storage_snapshot_from(&storage)
        .map_err(|_| "Browser identity storage could not be inspected.".to_string())?;
    let public_b64_candidate = snapshot.public_b64.clone();
    let previous_wire_candidate = snapshot.encrypted_wire.clone();
    let valid_previous_state = classify_identity_storage_snapshot(&snapshot)
        == LocalIdentityStoragePosture::ProtectedLocked
        && snapshot.secret_b64.is_none()
        && public_b64_candidate
            .as_deref()
            .is_some_and(|public| public_key_matches_signing_key(public, &signing_key))
        && previous_wire_candidate
            .as_deref()
            .is_some_and(|wire| encrypted_blob_is_structurally_valid(wire));
    if let Some(secret_b64) = snapshot.secret_b64.as_mut() {
        secret_b64.zeroize();
    }
    if !valid_previous_state {
        return Err(
            "Protected identity storage is inconsistent; passphrase was not changed.".into(),
        );
    }
    let public_b64 = public_b64_candidate
        .ok_or_else(|| "Protected identity public-key metadata is missing.".to_string())?;
    let previous_wire = previous_wire_candidate
        .ok_or_else(|| "Protected identity envelope is missing.".to_string())?;

    let blob = identity_crypto::encrypt(passphrase, signing_key.as_bytes()).await?;
    let next_wire = serde_json::to_string(&EncryptedBlobWire::from(&blob))
        .map_err(|_| "Could not serialize the updated protected envelope.".to_string())?;

    let mut current = read_identity_storage_snapshot_from(&storage)
        .map_err(|_| "Browser identity storage became unavailable.".to_string())?;
    let unchanged = current.secret_b64.is_none()
        && current.public_b64.as_deref() == Some(public_b64.as_str())
        && current.encrypted_wire.as_deref() == Some(previous_wire.as_str())
        && public_key_matches_signing_key(&public_b64, &signing_key);
    if let Some(secret_b64) = current.secret_b64.as_mut() {
        secret_b64.zeroize();
    }
    if !unchanged {
        return Err(
            "Identity storage changed during passphrase rotation; no new envelope was written."
                .into(),
        );
    }

    let write_succeeded = storage
        .set_item(IDENTITY_SECRET_ENCRYPTED_KEY, &next_wire)
        .is_ok();
    let readback = storage.get_item(IDENTITY_SECRET_ENCRYPTED_KEY);
    if write_succeeded
        && readback
            .as_ref()
            .is_ok_and(|value| value.as_deref() == Some(next_wire.as_str()))
        && encrypted_blob_is_structurally_valid(&next_wire)
    {
        return Ok(());
    }

    // Restore only if the observed value is either the new envelope from
    // this operation or the previously-authoritative envelope. Never overwrite
    // a different value that may have been committed by another tab.
    let observed = storage.get_item(IDENTITY_SECRET_ENCRYPTED_KEY);
    match observed.as_ref().ok().and_then(|value| value.as_deref()) {
        Some(value) if value == previous_wire => Err(
            "Passphrase rotation failed verification; the previous passphrase remains active."
                .into(),
        ),
        Some(value) if value == next_wire => {
            if storage
                .set_item(IDENTITY_SECRET_ENCRYPTED_KEY, &previous_wire)
                .is_err()
            {
                return Err(
                    "Passphrase rotation could not be verified and rollback failed; inspect identity posture before retrying."
                        .into(),
                );
            }
            let restored = storage.get_item(IDENTITY_SECRET_ENCRYPTED_KEY);
            if restored
                .as_ref()
                .is_ok_and(|stored| stored.as_deref() == Some(previous_wire.as_str()))
            {
                Err(
                    "Passphrase rotation failed verification; the previous passphrase remains active."
                        .into(),
                )
            } else {
                Err(
                    "Passphrase rotation rollback could not be verified; inspect identity posture before retrying."
                        .into(),
                )
            }
        }
        Some(_) => Err(
            "A different encrypted envelope was observed during passphrase rotation; it was preserved. Inspect identity posture before retrying."
                .into(),
        ),
        None => Err(
            "The encrypted envelope could not be read after rotation; no competing value was overwritten. Inspect identity posture before retrying."
                .into(),
        ),
    }
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
    let passphrase = Zeroizing::new(passphrase.to_owned());
    with_identity_operation_lock(move || async move {
        unlock_with_passphrase_unlocked(&passphrase).await
    })
    .await
}

async fn unlock_with_passphrase_unlocked(passphrase: &str) -> Result<SigningKey, String> {
    // A failed unlock leaves the identity locked rather than reusing a prior
    // in-memory key against storage whose integrity may have changed.
    UNLOCKED_KEY.with(|cell| *cell.borrow_mut() = None);
    let wire: EncryptedBlobWire = load_json(IDENTITY_SECRET_ENCRYPTED_KEY)
        .ok_or_else(|| "No passphrase-protected identity found".to_string())?;
    let blob = EncryptedBlob::try_from(&wire)?;
    let mut secret_bytes = identity_crypto::decrypt(passphrase, &blob).await?;
    let secret_array_result: Result<[u8; 32], _> = secret_bytes.as_slice().try_into();
    secret_bytes.zeroize();
    let mut secret_array = secret_array_result
        .map_err(|_| "Corrupt key material (wrong length after decrypt)".to_string())?;
    let signing_key = SigningKey::from_bytes(&secret_array);
    secret_array.zeroize();
    let public_b64 = load_string(IDENTITY_PUBLIC_KEY)
        .ok_or_else(|| "Protected identity public-key metadata is missing.".to_string())?;
    if !public_key_matches_signing_key(&public_b64, &signing_key) {
        return Err("Protected identity key material is inconsistent.".into());
    }
    UNLOCKED_KEY.with(|cell| *cell.borrow_mut() = Some(signing_key.clone()));
    Ok(signing_key)
}

/// Reverse [`protect_with_passphrase`]: decrypt with `passphrase`, restore
/// Reverse [`protect_with_passphrase`]: decrypt with `passphrase`, restore
/// plaintext storage, and drop the encrypted blob.
///
/// # Errors
///
/// Same failure modes as [`unlock_with_passphrase`] (this calls it
/// internally) — wrong passphrase, no protected identity, or corrupt blob.
pub async fn remove_passphrase_protection(passphrase: &str) -> Result<(), String> {
    let passphrase = Zeroizing::new(passphrase.to_owned());
    with_identity_operation_lock(move || async move {
        remove_passphrase_protection_unlocked(&passphrase).await
    })
    .await
}

async fn remove_passphrase_protection_unlocked(passphrase: &str) -> Result<(), String> {
    let signing_key = unlock_with_passphrase_unlocked(passphrase).await?;
    store_signing_key(&signing_key);
    remove_item(IDENTITY_SECRET_ENCRYPTED_KEY);
    Ok(())
}

/// Export the local identity's secret key as a standards-correct 24-word
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
    let mut entropy = mnemonic.to_entropy();
    let secret_array_result: Result<[u8; 32], _> = entropy.as_slice().try_into();
    entropy.zeroize();
    let mut secret_array = secret_array_result.map_err(|_| {
        "Seed phrase does not encode a 32-byte key (wrong word count for a Mycelix identity?)"
            .to_string()
    })?;
    let signing_key = SigningKey::from_bytes(&secret_array);
    secret_array.zeroize();
    Ok(signing_key)
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
#[deprecated(
    note = "Synchronous plaintext restore is disabled; use restore_protected_identity_from_seed_phrase."
)]
pub fn restore_from_seed_phrase(_phrase: &str) -> Result<SigningKey, String> {
    Err(
        "Synchronous seed-phrase restore is disabled because it writes plaintext and can replace an existing identity. Use restore_protected_identity_from_seed_phrase with an explicit passphrase on an empty device."
            .into(),
    )
}

/// Sign an arbitrary message with the local identity's private key./// Sign an arbitrary message with the local identity's private key.
///
/// Generates the keypair on first use if one does not already exist. This
/// is what a future DID-registry `create_did`/`rotate_key` call, or any
/// other proof-of-control flow, would use to prove ownership of
/// [`local_did`].
pub fn sign_with_local_identity(message: &[u8]) -> Signature {
    ensure_keypair().sign(message)
}

/// Gate an identity-dependent app shell before it provides a local identity context.
///
/// This component deliberately does not create, restore, migrate or unlock keys.
/// Users must complete the explicit identity lifecycle in the appropriate app
/// for this browser origin, then recheck or reload the dependent app.
#[component]
pub fn ProtectedIdentityAccessGate(children: ChildrenFn) -> impl IntoView {
    let posture = RwSignal::new(inspect_local_identity_posture());

    view! {
        <Show
            when=move || {
                posture.get() == LocalIdentityStoragePosture::ProtectedUnlockedInSession
            }
            fallback=move || view! {
                <main class="identity-access-gate-shell" aria-labelledby="identity-access-gate-title">
                    <section class="identity-access-gate-card" aria-describedby="identity-access-gate-description">
                        <span class="identity-access-gate-eyebrow">"MYCELIX / IDENTITY SAFETY"</span>
                        <h1 id="identity-access-gate-title">"Identity access is gated"</h1>
                        <p id="identity-access-gate-description">
                            {move || protected_identity_access_message(posture.get())}
                        </p>
                        <p>
                            "This app will not create, replace, or unlock a key during startup. Browser identity storage is origin-scoped and is not automatically shared between different origins."
                        </p>
                        <button
                            type="button"
                            class="identity-access-gate-recheck"
                            on:click=move |_| posture.set(inspect_local_identity_posture())
                        >
                            "Recheck identity"
                        </button>
                        <p class="identity-access-gate-status" role="status" aria-live="polite">
                            {move || format!("Current identity posture: {:?}", posture.get())}
                        </p>
                    </section>
                </main>
            }
        >
            <ProtectedIdentityAccessReady>
                {children()}
            </ProtectedIdentityAccessReady>
        </Show>
    }
}

fn protected_identity_access_message(posture: LocalIdentityStoragePosture) -> &'static str {
    match posture {
        LocalIdentityStoragePosture::StorageUnavailable => {
            "Browser storage could not be inspected. This is not proof that you have no identity. Restore storage access before proceeding."
        }
        LocalIdentityStoragePosture::NoIdentity => {
            "No local identity was found. Open the Mycelix identity setup experience for this browser origin to create a protected identity or restore a verified backup."
        }
        LocalIdentityStoragePosture::LegacyIdentityNeedsMigration => {
            "A legacy identity marker needs explicit migration. This app will not replace it or create a new key."
        }
        LocalIdentityStoragePosture::UnprotectedIdentity => {
            "An existing key is not protected at rest. Use the identity setup experience for this browser origin to protect it in place, then recheck here."
        }
        LocalIdentityStoragePosture::ProtectedLocked => {
            "A protected identity exists but is locked in this session. Unlock it in the identity setup experience for this browser origin, then recheck here."
        }
        LocalIdentityStoragePosture::ProtectedUnlockedInSession => {
            "The protected identity is unlocked in this session."
        }
        LocalIdentityStoragePosture::CorruptOrInconsistent => {
            "Stored identity evidence is corrupt or contradictory. Preserve the browser profile and use an explicit recovery procedure; no replacement key will be generated."
        }
    }
}

#[component]
fn ProtectedIdentityAccessReady(children: ChildrenFn) -> impl IntoView {
    provide_local_identity();
    children()
}

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
    try_generate_signing_key().expect("Web Crypto CSPRNG unavailable for local identity")
}

fn try_generate_signing_key() -> Result<SigningKey, String> {
    let window = web_sys::window().ok_or_else(|| {
        "Browser window is unavailable; identity key generation requires Web Crypto.".to_string()
    })?;
    let crypto = window
        .crypto()
        .map_err(|_| "Web Crypto API unavailable.".to_string())?;
    let mut seed = [0u8; 32];
    if crypto.get_random_values_with_u8_array(&mut seed).is_err() {
        seed.zeroize();
        return Err("crypto.getRandomValues failed.".to_string());
    }
    let signing_key = SigningKey::from_bytes(&seed);
    seed.zeroize();
    Ok(signing_key)
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
    let mut secret_b64 = load_string(IDENTITY_SECRET_KEY)?;
    let public_b64 = load_string(IDENTITY_PUBLIC_KEY);
    let signing_key_result = signing_key_from_secret_b64(&secret_b64);
    secret_b64.zeroize();
    let signing_key = signing_key_result.ok()?;
    let public_b64 = public_b64?;
    public_key_matches_signing_key(&public_b64, &signing_key).then_some(signing_key)
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

/// Best-effort ownership-aware cleanup for split localStorage transitions.
///
/// This prevents the common failure mode where a losing operation blindly
/// removes a different value written by another tab. The read/check/remove
/// sequence is not atomic across tabs; full serialization/transactional storage
/// remains tracked in SEC-IDENTITY-004.
fn remove_storage_item_if_matches(
    storage: &web_sys::Storage,
    key: &str,
    expected_value: &str,
) -> Result<(), String> {
    let current = storage
        .get_item(key)
        .map_err(|_| "Storage could not be read during rollback.".to_string())?;
    match current.as_deref() {
        None => Ok(()),
        Some(value) if value == expected_value => {
            storage
                .remove_item(key)
                .map_err(|_| "The operation's storage value could not be removed.".to_string())?;
            let after = storage
                .get_item(key)
                .map_err(|_| "Storage cleanup could not be verified.".to_string())?;
            if after.is_none() {
                Ok(())
            } else {
                Err("Storage cleanup could not be verified.".into())
            }
        }
        Some(_) => Err(
            "A different concurrent storage value was preserved; inspect identity posture.".into(),
        ),
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

    fn snapshot(
        legacy_did: Option<String>,
        secret_b64: Option<String>,
        public_b64: Option<String>,
        encrypted_wire: Option<String>,
    ) -> IdentityStorageSnapshot {
        IdentityStorageSnapshot {
            storage_available: true,
            legacy_did,
            secret_b64,
            public_b64,
            encrypted_wire,
        }
    }

    fn valid_encrypted_wire() -> String {
        let wire = EncryptedBlobWire {
            salt_b64: base64_encode(&[1u8; identity_crypto::SALT_LEN]),
            iv_b64: base64_encode(&[2u8; identity_crypto::IV_LEN]),
            ciphertext_b64: base64_encode(&[3u8; 48]),
        };
        serde_json::to_string(&wire).unwrap()
    }

    #[test]
    fn protected_unlocked_posture_requires_the_cached_key_to_match_public_key() {
        let cached = SigningKey::from_bytes(&[31u8; 32]);
        let other = SigningKey::from_bytes(&[32u8; 32]);
        UNLOCKED_KEY.with(|cell| *cell.borrow_mut() = Some(cached.clone()));

        assert!(unlocked_cache_matches_public_key(&base64_encode(
            cached.verifying_key().as_bytes()
        )));
        assert!(!unlocked_cache_matches_public_key(&base64_encode(
            other.verifying_key().as_bytes()
        )));

        UNLOCKED_KEY.with(|cell| *cell.borrow_mut() = None);
    }

    #[test]
    fn unlocked_signing_key_must_match_stored_public_key() {
        let expected = SigningKey::from_bytes(&[71u8; 32]);
        let other = SigningKey::from_bytes(&[72u8; 32]);
        assert!(public_key_matches_signing_key(
            &base64_encode(expected.verifying_key().as_bytes()),
            &expected
        ));
        assert!(!public_key_matches_signing_key(
            &base64_encode(other.verifying_key().as_bytes()),
            &expected
        ));
    }

    #[test]
    fn bootstrap_policy_allows_only_the_state_specific_action_set() {
        use IdentityBootstrapAction as Action;
        use LocalIdentityStoragePosture as Posture;

        assert_eq!(
            allowed_identity_bootstrap_actions(Posture::NoIdentity),
            &[
                Action::CreateProtectedIdentity,
                Action::RestoreProtectedIdentityFromSeedPhrase,
            ]
        );
        assert_eq!(
            allowed_identity_bootstrap_actions(Posture::UnprotectedIdentity),
            &[Action::ProtectExistingIdentity]
        );
        assert_eq!(
            allowed_identity_bootstrap_actions(Posture::ProtectedLocked),
            &[Action::UnlockProtectedIdentity]
        );
        assert_eq!(
            allowed_identity_bootstrap_actions(Posture::ProtectedUnlockedInSession),
            &[Action::ExportRecoveryPhrase, Action::RotatePassphrase]
        );
        assert!(
            allowed_identity_bootstrap_actions(Posture::LegacyIdentityNeedsMigration).is_empty()
        );
        assert!(allowed_identity_bootstrap_actions(Posture::StorageUnavailable).is_empty());
        assert!(allowed_identity_bootstrap_actions(Posture::CorruptOrInconsistent).is_empty());
    }

    #[test]
    fn bootstrap_policy_denies_cross_state_identity_replacement() {
        use IdentityBootstrapAction as Action;
        use LocalIdentityStoragePosture as Posture;

        let protected_or_unknown = [
            Posture::UnprotectedIdentity,
            Posture::ProtectedLocked,
            Posture::ProtectedUnlockedInSession,
            Posture::LegacyIdentityNeedsMigration,
            Posture::StorageUnavailable,
            Posture::CorruptOrInconsistent,
        ];
        for posture in protected_or_unknown {
            assert!(!identity_bootstrap_action_allowed(
                posture,
                Action::CreateProtectedIdentity
            ));
            assert!(!identity_bootstrap_action_allowed(
                posture,
                Action::RestoreProtectedIdentityFromSeedPhrase
            ));
        }
    }

    #[test]
    fn bootstrap_policy_denies_protection_and_unlock_in_wrong_states() {
        use IdentityBootstrapAction as Action;
        use LocalIdentityStoragePosture as Posture;

        assert!(!identity_bootstrap_action_allowed(
            Posture::NoIdentity,
            Action::UnlockProtectedIdentity
        ));
        assert!(!identity_bootstrap_action_allowed(
            Posture::ProtectedLocked,
            Action::ProtectExistingIdentity
        ));
        assert!(!identity_bootstrap_action_allowed(
            Posture::UnprotectedIdentity,
            Action::UnlockProtectedIdentity
        ));
        assert!(!identity_bootstrap_action_allowed(
            Posture::StorageUnavailable,
            Action::RotatePassphrase
        ));
    }

    #[test]
    fn bootstrap_action_policy_matches_every_identity_posture() {
        use IdentityBootstrapAction as Action;
        use LocalIdentityStoragePosture as Posture;

        let cases: &[(Posture, &[Action])] = &[
            (Posture::StorageUnavailable, &[]),
            (
                Posture::NoIdentity,
                &[
                    Action::CreateProtectedIdentity,
                    Action::RestoreProtectedIdentityFromSeedPhrase,
                ],
            ),
            (Posture::LegacyIdentityNeedsMigration, &[]),
            (
                Posture::UnprotectedIdentity,
                &[Action::ProtectExistingIdentity],
            ),
            (Posture::ProtectedLocked, &[Action::UnlockProtectedIdentity]),
            (
                Posture::ProtectedUnlockedInSession,
                &[Action::ExportRecoveryPhrase, Action::RotatePassphrase],
            ),
            (Posture::CorruptOrInconsistent, &[]),
        ];

        for (posture, expected) in cases {
            assert_eq!(allowed_identity_bootstrap_actions(*posture), *expected);
            for action in [
                Action::CreateProtectedIdentity,
                Action::ProtectExistingIdentity,
                Action::UnlockProtectedIdentity,
                Action::RestoreProtectedIdentityFromSeedPhrase,
                Action::ExportRecoveryPhrase,
                Action::RotatePassphrase,
            ] {
                assert_eq!(
                    identity_bootstrap_action_allowed(*posture, action),
                    expected.contains(&action),
                    "unexpected policy for {posture:?} / {action:?}"
                );
            }
        }
    }

    #[test]
    fn recovery_export_is_never_allowed_before_verified_protected_unlock() {
        use IdentityBootstrapAction::ExportRecoveryPhrase as Export;
        use LocalIdentityStoragePosture as Posture;

        for posture in [
            Posture::StorageUnavailable,
            Posture::NoIdentity,
            Posture::LegacyIdentityNeedsMigration,
            Posture::UnprotectedIdentity,
            Posture::ProtectedLocked,
            Posture::CorruptOrInconsistent,
        ] {
            assert!(!identity_bootstrap_action_allowed(posture, Export));
        }
        assert!(identity_bootstrap_action_allowed(
            Posture::ProtectedUnlockedInSession,
            Export
        ));
    }

    #[test]
    fn unavailable_storage_is_not_classified_as_no_identity() {
        let snapshot = IdentityStorageSnapshot::default();
        assert_eq!(
            classify_identity_storage_snapshot(&snapshot),
            LocalIdentityStoragePosture::StorageUnavailable
        );
    }

    #[test]
    fn empty_storage_and_legacy_identity_have_distinct_postures() {
        assert_eq!(
            classify_identity_storage_snapshot(&snapshot(None, None, None, None)),
            LocalIdentityStoragePosture::NoIdentity
        );
        assert_eq!(
            classify_identity_storage_snapshot(&snapshot(
                Some("did:mycelix:legacy-demo".into()),
                None,
                None,
                None,
            )),
            LocalIdentityStoragePosture::LegacyIdentityNeedsMigration
        );
    }

    #[test]
    fn consistent_plaintext_keypair_is_reported_as_unprotected() {
        let key = SigningKey::from_bytes(&[7u8; 32]);
        let posture = classify_identity_storage_snapshot(&snapshot(
            None,
            Some(base64_encode(key.as_bytes())),
            Some(base64_encode(key.verifying_key().as_bytes())),
            None,
        ));
        assert_eq!(posture, LocalIdentityStoragePosture::UnprotectedIdentity);
    }

    #[test]
    fn mismatched_or_partial_plaintext_key_material_is_corrupt() {
        let first = SigningKey::from_bytes(&[7u8; 32]);
        let second = SigningKey::from_bytes(&[8u8; 32]);
        assert_eq!(
            classify_identity_storage_snapshot(&snapshot(
                None,
                Some(base64_encode(first.as_bytes())),
                Some(base64_encode(second.verifying_key().as_bytes())),
                None,
            )),
            LocalIdentityStoragePosture::CorruptOrInconsistent
        );
        assert_eq!(
            classify_identity_storage_snapshot(&snapshot(
                None,
                Some(base64_encode(first.as_bytes())),
                None,
                None,
            )),
            LocalIdentityStoragePosture::CorruptOrInconsistent
        );
    }

    #[test]
    fn well_formed_encrypted_container_is_protected_but_not_yet_authenticated() {
        let public_key = SigningKey::from_bytes(&[9u8; 32]).verifying_key();
        assert_eq!(
            classify_identity_storage_snapshot(&snapshot(
                None,
                None,
                Some(base64_encode(public_key.as_bytes())),
                Some(valid_encrypted_wire()),
            )),
            LocalIdentityStoragePosture::ProtectedLocked
        );
    }

    #[test]
    fn encrypted_container_with_wrong_ciphertext_length_is_corrupt() {
        let key = SigningKey::from_bytes(&[11u8; 32]);
        let malformed_wire = serde_json::to_string(&EncryptedBlobWire {
            salt_b64: base64_encode(&[1u8; identity_crypto::SALT_LEN]),
            iv_b64: base64_encode(&[2u8; identity_crypto::IV_LEN]),
            ciphertext_b64: base64_encode(&[3u8; 47]),
        })
        .unwrap();
        assert_eq!(
            classify_identity_storage_snapshot(&snapshot(
                None,
                None,
                Some(base64_encode(key.verifying_key().as_bytes())),
                Some(malformed_wire),
            )),
            LocalIdentityStoragePosture::CorruptOrInconsistent
        );
    }

    #[test]
    fn plaintext_secret_next_to_encrypted_container_is_inconsistent() {
        let key = SigningKey::from_bytes(&[10u8; 32]);
        assert_eq!(
            classify_identity_storage_snapshot(&snapshot(
                None,
                Some(base64_encode(key.as_bytes())),
                Some(base64_encode(key.verifying_key().as_bytes())),
                Some(valid_encrypted_wire()),
            )),
            LocalIdentityStoragePosture::CorruptOrInconsistent
        );
    }

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

#[cfg(all(test, target_arch = "wasm32"))]
mod browser_storage_tests {
    use super::*;
    use wasm_bindgen_test::*;

    // identity_crypto's browser test module configures the crate's runner mode.

    struct IdentityStorageBackup {
        storage: web_sys::Storage,
        values: Vec<(&'static str, Option<String>)>,
        unlocked_key: Option<SigningKey>,
    }

    impl IdentityStorageBackup {
        fn new_and_clear() -> Self {
            let storage = web_sys::window()
                .expect("browser window is available in this test")
                .local_storage()
                .expect("localStorage API is available")
                .expect("localStorage is available");
            let keys = [
                LEGACY_DID_KEY,
                IDENTITY_SECRET_KEY,
                IDENTITY_PUBLIC_KEY,
                IDENTITY_SECRET_ENCRYPTED_KEY,
                IDENTITY_BACKUP_CONFIRMED_PUBLIC_KEY,
            ];
            let values = keys
                .into_iter()
                .map(|key| (key, storage.get_item(key).expect("storage read succeeds")))
                .collect();
            let unlocked_key = UNLOCKED_KEY.with(|cell| cell.borrow().clone());
            let backup = Self {
                storage,
                values,
                unlocked_key,
            };
            backup.clear_identity_keys();
            UNLOCKED_KEY.with(|cell| *cell.borrow_mut() = None);
            backup
        }

        fn clear_identity_keys(&self) {
            for key in [
                LEGACY_DID_KEY,
                IDENTITY_SECRET_KEY,
                IDENTITY_PUBLIC_KEY,
                IDENTITY_SECRET_ENCRYPTED_KEY,
                IDENTITY_BACKUP_CONFIRMED_PUBLIC_KEY,
            ] {
                self.storage
                    .remove_item(key)
                    .expect("test storage cleanup succeeds");
            }
        }
    }

    impl Drop for IdentityStorageBackup {
        fn drop(&mut self) {
            self.clear_identity_keys();
            for (key, value) in &self.values {
                match value {
                    Some(value) => {
                        let _ = self.storage.set_item(key, value);
                    }
                    None => {
                        let _ = self.storage.remove_item(key);
                    }
                }
            }
            for (_, value) in &mut self.values {
                if let Some(value) = value.as_mut() {
                    value.zeroize();
                }
            }
            UNLOCKED_KEY.with(|cell| *cell.borrow_mut() = self.unlocked_key.clone());
        }
    }

    #[wasm_bindgen_test]
    async fn web_lock_serializes_contending_identity_lifecycle_operations() {
        use std::rc::Rc;
        use std::cell::RefCell;

        let events = Rc::new(RefCell::new(Vec::<&'static str>::new()));
        let first_events = events.clone();
        let first = with_identity_operation_lock(move || async move {
            first_events.borrow_mut().push("first-start");
            gloo_timers::future::TimeoutFuture::new(50).await;
            first_events.borrow_mut().push("first-end");
            Ok(())
        });
        let second_events = events.clone();
        let second = with_identity_operation_lock(move || async move {
            second_events.borrow_mut().push("second-start");
            second_events.borrow_mut().push("second-end");
            Ok(())
        });

        let (first_result, second_result) = futures_util::future::join(first, second).await;
        assert!(first_result.is_ok(), "first lock holder succeeds");
        assert!(second_result.is_ok(), "queued lock holder succeeds");
        assert_eq!(
            *events.borrow(),
            vec!["first-start", "first-end", "second-start", "second-end"],
            "the second operation must not enter the critical section until the first Promise settles"
        );
    }

    #[wasm_bindgen_test]
    async fn protected_creation_writes_no_plaintext_secret_and_unlocks_same_key() {
        let _backup = IdentityStorageBackup::new_and_clear();
        let posture = create_protected_identity("synthetic browser test passphrase")
            .await
            .expect("protected identity creation succeeds in isolated browser storage");
        assert_eq!(
            posture,
            LocalIdentityStoragePosture::ProtectedUnlockedInSession
        );

        let storage = web_sys::window().unwrap().local_storage().unwrap().unwrap();
        assert!(storage.get_item(IDENTITY_SECRET_KEY).unwrap().is_none());
        let public_b64 = storage
            .get_item(IDENTITY_PUBLIC_KEY)
            .unwrap()
            .expect("public-key metadata was persisted");
        assert!(
            storage
                .get_item(IDENTITY_SECRET_ENCRYPTED_KEY)
                .unwrap()
                .is_some()
        );

        UNLOCKED_KEY.with(|cell| *cell.borrow_mut() = None);
        let signing_key = unlock_with_passphrase("synthetic browser test passphrase")
            .await
            .expect("correct passphrase unlocks the protected identity");
        assert!(public_key_matches_signing_key(&public_b64, &signing_key));
        assert!(storage.get_item(IDENTITY_SECRET_KEY).unwrap().is_none());
    }

    #[wasm_bindgen_test]
    async fn mnemonic_restore_writes_only_protected_key_material() {
        let _backup = IdentityStorageBackup::new_and_clear();
        let source_key = SigningKey::from_bytes(&[91u8; 32]);
        let expected_public = base64_encode(source_key.verifying_key().as_bytes());
        let phrase = export_seed_phrase(&source_key);

        let posture = restore_protected_identity_from_seed_phrase(
            &phrase,
            "synthetic browser test passphrase",
        )
        .await
        .expect("valid mnemonic restores into protected storage on an empty device");
        assert_eq!(
            posture,
            LocalIdentityStoragePosture::ProtectedUnlockedInSession
        );

        let storage = web_sys::window().unwrap().local_storage().unwrap().unwrap();
        assert!(storage.get_item(IDENTITY_SECRET_KEY).unwrap().is_none());
        assert_eq!(
            storage.get_item(IDENTITY_PUBLIC_KEY).unwrap().as_deref(),
            Some(expected_public.as_str())
        );
        UNLOCKED_KEY.with(|cell| *cell.borrow_mut() = None);
        let restored = unlock_with_passphrase("synthetic browser test passphrase")
            .await
            .expect("protected restored identity unlocks");
        assert!(public_key_matches_signing_key(&expected_public, &restored));
    }

    #[wasm_bindgen_test]
    async fn backup_confirmation_marker_is_cleared_and_bound_to_current_public_key() {
        let _backup = IdentityStorageBackup::new_and_clear();
        let storage = web_sys::window().unwrap().local_storage().unwrap().unwrap();
        storage
            .set_item(IDENTITY_BACKUP_CONFIRMED_PUBLIC_KEY, "stale-public-key")
            .expect("test can seed a stale non-secret marker");

        create_protected_identity("synthetic backup-marker passphrase")
            .await
            .expect("creation succeeds with a stale marker but clears it");
        assert!(
            recovery_backup_confirmation_needed(),
            "a new identity must require its own recovery-backup confirmation"
        );
        assert!(
            storage
                .get_item(IDENTITY_BACKUP_CONFIRMED_PUBLIC_KEY)
                .unwrap()
                .is_none(),
            "stale marker must be cleared before identity activation"
        );

        UNLOCKED_KEY.with(|cell| *cell.borrow_mut() = None);
        assert!(
            mark_recovery_backup_confirmed().is_err(),
            "backup confirmation cannot be recorded while the key is locked"
        );
        assert!(
            storage
                .get_item(IDENTITY_BACKUP_CONFIRMED_PUBLIC_KEY)
                .unwrap()
                .is_none()
        );
        unlock_with_passphrase("synthetic backup-marker passphrase")
            .await
            .expect("the protected identity unlocks again before marking backup");
        mark_recovery_backup_confirmed()
            .expect("unlocked identity can record verified backup confirmation");
        assert!(
            !recovery_backup_confirmation_needed(),
            "current public-key marker should satisfy the gate"
        );

        // Simulate a full-page reload: the in-memory signing key disappears,
        // but the non-secret marker remains bound to the same public key.
        UNLOCKED_KEY.with(|cell| *cell.borrow_mut() = None);
        assert!(
            !recovery_backup_confirmation_needed(),
            "verified backup state should survive a reload for the same public key"
        );

        let wrong_key = SigningKey::from_bytes(&[139u8; 32]);
        storage
            .set_item(
                IDENTITY_PUBLIC_KEY,
                &base64_encode(wrong_key.verifying_key().as_bytes()),
            )
            .expect("test can tamper with public metadata");
        assert!(
            recovery_backup_confirmation_needed(),
            "a marker for the old public key must not bless changed identity metadata"
        );
    }

    #[wasm_bindgen_test]
    async fn recovery_confirmation_positions_are_distinct_and_in_range() {
        for _ in 0..100 {
            let positions = recovery_phrase_challenge_positions()
                .expect("browser CSPRNG can choose recovery positions");
            assert!(positions.iter().all(|position| (1..=24).contains(position)));
            assert_ne!(positions[0], positions[1]);
            assert_ne!(positions[0], positions[2]);
            assert_ne!(positions[1], positions[2]);
        }
    }

    #[wasm_bindgen_test]
    async fn recovery_export_is_read_only_and_matches_the_unlocked_identity() {
        let _backup = IdentityStorageBackup::new_and_clear();
        create_protected_identity("synthetic recovery export passphrase")
            .await
            .expect("protected identity creation succeeds");
        let storage = web_sys::window().unwrap().local_storage().unwrap().unwrap();
        let public_before = storage
            .get_item(IDENTITY_PUBLIC_KEY)
            .unwrap()
            .expect("public metadata exists");
        let envelope_before = storage
            .get_item(IDENTITY_SECRET_ENCRYPTED_KEY)
            .unwrap()
            .expect("protected envelope exists");

        let phrase = export_unlocked_identity_seed_phrase()
            .expect("unlocked protected identity can export its recovery phrase");
        let derived =
            signing_key_from_seed_phrase(&phrase).expect("exported recovery phrase is valid");
        assert!(public_key_matches_signing_key(&public_before, &derived));
        assert!(storage.get_item(IDENTITY_SECRET_KEY).unwrap().is_none());
        assert_eq!(
            storage.get_item(IDENTITY_PUBLIC_KEY).unwrap().as_deref(),
            Some(public_before.as_str())
        );
        assert_eq!(
            storage
                .get_item(IDENTITY_SECRET_ENCRYPTED_KEY)
                .unwrap()
                .as_deref(),
            Some(envelope_before.as_str())
        );

        UNLOCKED_KEY.with(|cell| *cell.borrow_mut() = None);
        assert!(export_unlocked_identity_seed_phrase().is_err());
    }

    #[wasm_bindgen_test]
    async fn protecting_existing_identity_removes_plaintext_after_verification() {
        let _backup = IdentityStorageBackup::new_and_clear();
        let original = ensure_keypair();
        let expected_public = base64_encode(original.verifying_key().as_bytes());
        assert!(
            web_sys::window()
                .unwrap()
                .local_storage()
                .unwrap()
                .unwrap()
                .get_item(IDENTITY_SECRET_KEY)
                .unwrap()
                .is_some()
        );

        let storage_before_protection =
            web_sys::window().unwrap().local_storage().unwrap().unwrap();
        storage_before_protection
            .set_item(IDENTITY_BACKUP_CONFIRMED_PUBLIC_KEY, &expected_public)
            .expect("test can seed a stale confirmation from the prior storage posture");
        protect_with_passphrase("synthetic browser test passphrase")
            .await
            .expect("public protection API safely protects the existing identity");
        assert_eq!(
            inspect_local_identity_posture(),
            LocalIdentityStoragePosture::ProtectedUnlockedInSession
        );
        assert!(
            recovery_backup_confirmation_needed(),
            "protecting an unprotected identity must reset old backup confirmation"
        );

        let storage = web_sys::window().unwrap().local_storage().unwrap().unwrap();
        assert!(storage.get_item(IDENTITY_SECRET_KEY).unwrap().is_none());
        assert_eq!(
            storage.get_item(IDENTITY_PUBLIC_KEY).unwrap().as_deref(),
            Some(expected_public.as_str())
        );
        UNLOCKED_KEY.with(|cell| *cell.borrow_mut() = None);
        let protected_key = unlock_with_passphrase("synthetic browser test passphrase")
            .await
            .expect("migrated protected identity unlocks");
        assert!(public_key_matches_signing_key(
            &expected_public,
            &protected_key
        ));
    }

    #[wasm_bindgen_test]
    async fn public_protection_api_rotates_passphrase_without_plaintext_storage() {
        let _backup = IdentityStorageBackup::new_and_clear();
        create_protected_identity("synthetic initial passphrase")
            .await
            .expect("initial protected identity creation succeeds");
        let storage = web_sys::window().unwrap().local_storage().unwrap().unwrap();
        let original_public = storage
            .get_item(IDENTITY_PUBLIC_KEY)
            .unwrap()
            .expect("public key is present");

        protect_with_passphrase("synthetic rotated passphrase")
            .await
            .expect("public protection API rotates a currently unlocked identity");
        assert!(storage.get_item(IDENTITY_SECRET_KEY).unwrap().is_none());
        UNLOCKED_KEY.with(|cell| *cell.borrow_mut() = None);

        assert!(
            unlock_with_passphrase("synthetic initial passphrase")
                .await
                .is_err()
        );
        assert!(UNLOCKED_KEY.with(|cell| cell.borrow().is_none()));
        let rotated = unlock_with_passphrase("synthetic rotated passphrase")
            .await
            .expect("rotated passphrase unlocks the same identity");
        assert!(public_key_matches_signing_key(&original_public, &rotated));
    }

    #[wasm_bindgen_test]
    async fn empty_passphrase_does_not_create_or_mutate_identity() {
        let _backup = IdentityStorageBackup::new_and_clear();
        assert!(create_protected_identity("   ").await.is_err());
        let storage = web_sys::window().unwrap().local_storage().unwrap().unwrap();
        assert!(storage.get_item(IDENTITY_SECRET_KEY).unwrap().is_none());
        assert!(storage.get_item(IDENTITY_PUBLIC_KEY).unwrap().is_none());
        assert!(
            storage
                .get_item(IDENTITY_SECRET_ENCRYPTED_KEY)
                .unwrap()
                .is_none()
        );
        assert!(UNLOCKED_KEY.with(|cell| cell.borrow().is_none()));
    }

    #[wasm_bindgen_test]
    async fn protected_creation_refuses_to_overwrite_existing_plaintext_identity() {
        let _backup = IdentityStorageBackup::new_and_clear();
        let existing = ensure_keypair();
        let existing_secret = load_string(IDENTITY_SECRET_KEY).expect("legacy secret is present");
        let existing_public = load_string(IDENTITY_PUBLIC_KEY).expect("public key is present");

        assert!(
            create_protected_identity("synthetic passphrase")
                .await
                .is_err()
        );

        let storage = web_sys::window().unwrap().local_storage().unwrap().unwrap();
        assert_eq!(
            storage.get_item(IDENTITY_SECRET_KEY).unwrap().as_deref(),
            Some(existing_secret.as_str())
        );
        assert_eq!(
            storage.get_item(IDENTITY_PUBLIC_KEY).unwrap().as_deref(),
            Some(existing_public.as_str())
        );
        assert!(
            storage
                .get_item(IDENTITY_SECRET_ENCRYPTED_KEY)
                .unwrap()
                .is_none()
        );
        assert!(public_key_matches_signing_key(&existing_public, &existing));
    }

    #[wasm_bindgen_test]
    async fn protected_restore_refuses_to_overwrite_an_existing_identity() {
        let _backup = IdentityStorageBackup::new_and_clear();
        let existing = ensure_keypair();
        let existing_public = base64_encode(existing.verifying_key().as_bytes());
        let unrelated = SigningKey::from_bytes(&[101u8; 32]);
        let phrase = export_seed_phrase(&unrelated);

        assert!(
            restore_protected_identity_from_seed_phrase(&phrase, "synthetic passphrase")
                .await
                .is_err()
        );

        let storage = web_sys::window().unwrap().local_storage().unwrap().unwrap();
        assert_eq!(
            storage.get_item(IDENTITY_PUBLIC_KEY).unwrap().as_deref(),
            Some(existing_public.as_str())
        );
        assert!(
            storage
                .get_item(IDENTITY_SECRET_ENCRYPTED_KEY)
                .unwrap()
                .is_none()
        );
    }

    #[wasm_bindgen_test]
    async fn rollback_cleanup_preserves_a_nonmatching_concurrent_value() {
        let _backup = IdentityStorageBackup::new_and_clear();
        let storage = web_sys::window().unwrap().local_storage().unwrap().unwrap();
        storage
            .set_item(IDENTITY_SECRET_ENCRYPTED_KEY, "another-tab-envelope")
            .expect("test can seed the synthetic competing value");

        assert!(
            remove_storage_item_if_matches(
                &storage,
                IDENTITY_SECRET_ENCRYPTED_KEY,
                "this-operation-envelope",
            )
            .is_err()
        );
        assert_eq!(
            storage
                .get_item(IDENTITY_SECRET_ENCRYPTED_KEY)
                .unwrap()
                .as_deref(),
            Some("another-tab-envelope")
        );

        remove_storage_item_if_matches(
            &storage,
            IDENTITY_SECRET_ENCRYPTED_KEY,
            "another-tab-envelope",
        )
        .expect("matching value can be removed and verified");
        assert!(
            storage
                .get_item(IDENTITY_SECRET_ENCRYPTED_KEY)
                .unwrap()
                .is_none()
        );
    }

    #[wasm_bindgen_test]
    async fn plaintext_loader_rejects_a_secret_public_key_mismatch() {
        let _backup = IdentityStorageBackup::new_and_clear();
        let first = SigningKey::from_bytes(&[121u8; 32]);
        let second = SigningKey::from_bytes(&[122u8; 32]);
        let storage = web_sys::window().unwrap().local_storage().unwrap().unwrap();
        storage
            .set_item(IDENTITY_SECRET_KEY, &base64_encode(first.as_bytes()))
            .expect("test can seed the synthetic secret");
        storage
            .set_item(
                IDENTITY_PUBLIC_KEY,
                &base64_encode(second.verifying_key().as_bytes()),
            )
            .expect("test can seed the mismatching public key");

        assert!(load_signing_key().is_none());
        assert_eq!(
            inspect_local_identity_posture(),
            LocalIdentityStoragePosture::CorruptOrInconsistent
        );
        assert!(UNLOCKED_KEY.with(|cell| cell.borrow().is_none()));
    }

    #[wasm_bindgen_test]
    async fn unlock_rejects_tampered_public_key_without_caching_decrypted_key() {
        let _backup = IdentityStorageBackup::new_and_clear();
        create_protected_identity("synthetic browser test passphrase")
            .await
            .expect("protected identity creation succeeds in isolated browser storage");

        let storage = web_sys::window().unwrap().local_storage().unwrap().unwrap();
        let wrong_key = SigningKey::from_bytes(&[117u8; 32]);
        storage
            .set_item(
                IDENTITY_PUBLIC_KEY,
                &base64_encode(wrong_key.verifying_key().as_bytes()),
            )
            .expect("test can tamper with the isolated public-key metadata");
        UNLOCKED_KEY.with(|cell| *cell.borrow_mut() = None);

        assert!(
            unlock_with_passphrase("synthetic browser test passphrase")
                .await
                .is_err()
        );
        assert!(UNLOCKED_KEY.with(|cell| cell.borrow().is_none()));
        assert!(storage.get_item(IDENTITY_SECRET_KEY).unwrap().is_none());
    }
}
