// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Authenticated hybrid encryption: one call that both hybrid-seals a payload
//! to a recipient AND signs it, so the recipient gets confidentiality *and*
//! sender authenticity. This is the API a mail/vault client actually calls;
//! it composes [`crate::hybrid_kem`] and [`crate::hybrid_sig`] and fixes the
//! **encrypt-then-sign** composition order so each consumer does not reinvent
//! it (signing the ciphertext, not the plaintext, is the AEAD-safe choice).
//!
//! # Stability
//! EXPERIMENTAL — feature `hybrid-rc`, off by default. Pending crypto audit.

use crate::error::CryptoError;
use crate::hybrid_kem::{HybridCiphertext, HybridKemKeyPair, HybridPublicKeys, seal};
use crate::hybrid_sig::{HybridSignature, HybridSigner, HybridVerifyingKeys, verify as verify_sig};

/// An authenticated, hybrid-encrypted message: a hybrid ciphertext plus a
/// hybrid signature over that ciphertext (encrypt-then-sign).
#[derive(Clone, Debug)]
pub struct SignedSealed {
    /// Hybrid X25519+ML-KEM-768 ciphertext.
    pub ciphertext: HybridCiphertext,
    /// Hybrid Ed25519+ML-DSA-65 signature over the ciphertext transcript.
    pub signature: HybridSignature,
}

/// Seal `plaintext` to `recipient`'s public keys and sign it as `sender`.
pub fn seal_signed(
    recipient: &HybridPublicKeys,
    sender: &HybridSigner,
    plaintext: &[u8],
) -> Result<SignedSealed, CryptoError> {
    let ciphertext = seal(recipient, plaintext)?;
    let signature = sender.sign(&ciphertext_transcript(&ciphertext));
    Ok(SignedSealed {
        ciphertext,
        signature,
    })
}

/// Verify the sender's signature over the ciphertext, then decrypt.
///
/// Authenticity is checked *before* decryption: a signature that does not
/// verify against `sender_keys` is rejected without touching the AEAD.
pub fn open_verified(
    recipient: &HybridKemKeyPair,
    sender_keys: &HybridVerifyingKeys,
    sealed: &SignedSealed,
) -> Result<Vec<u8>, CryptoError> {
    verify_sig(
        sender_keys,
        &ciphertext_transcript(&sealed.ciphertext),
        &sealed.signature,
    )?;
    recipient.open(&sealed.ciphertext)
}

/// Deterministic byte encoding of a hybrid ciphertext, used as the signed
/// transcript (encrypt-then-sign). Length-prefixing the variable-length fields
/// keeps the encoding unambiguous.
fn ciphertext_transcript(ct: &HybridCiphertext) -> Vec<u8> {
    let mut t =
        Vec::with_capacity(32 + ct.ml_kem_ct.len() + ct.nonce.len() + ct.ciphertext.len() + 16);
    t.extend_from_slice(&ct.x25519_eph_pk);
    append_len_prefixed(&mut t, &ct.ml_kem_ct);
    append_len_prefixed(&mut t, &ct.nonce);
    append_len_prefixed(&mut t, &ct.ciphertext);
    t
}

fn append_len_prefixed(buf: &mut Vec<u8>, bytes: &[u8]) {
    let len = u32::try_from(bytes.len()).expect("ciphertext component length fits in u32");
    buf.extend_from_slice(&len.to_le_bytes());
    buf.extend_from_slice(bytes);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn seal_signed_open_verified_round_trip() {
        let recipient = HybridKemKeyPair::generate();
        let sender = HybridSigner::generate();
        let msg = b"a health record payload, encrypted at rest";

        let sealed = seal_signed(&recipient.public_keys(), &sender, msg).expect("seal_signed");
        let out = open_verified(&recipient, &sender.verifying_keys(), &sealed).expect("open");
        assert_eq!(out, msg);
    }

    #[test]
    fn wrong_sender_signature_is_rejected_before_decrypt() {
        let recipient = HybridKemKeyPair::generate();
        let real_sender = HybridSigner::generate();
        let impostor = HybridSigner::generate();

        let sealed =
            seal_signed(&recipient.public_keys(), &real_sender, b"trust me").expect("seal");
        // Verifying against the impostor's keys must fail (authenticity).
        assert!(open_verified(&recipient, &impostor.verifying_keys(), &sealed).is_err());
    }

    #[test]
    fn tampered_ciphertext_fails_signature() {
        let recipient = HybridKemKeyPair::generate();
        let sender = HybridSigner::generate();
        let mut sealed = seal_signed(&recipient.public_keys(), &sender, b"secret").expect("seal");
        // Flip a ciphertext byte: the signature transcript changes, so verify fails
        // (we never reach AEAD decryption).
        sealed.ciphertext.ciphertext[0] ^= 0x01;
        assert!(open_verified(&recipient, &sender.verifying_keys(), &sealed).is_err());
    }

    #[test]
    fn wrong_recipient_cannot_open_even_with_valid_signature() {
        let recipient = HybridKemKeyPair::generate();
        let other = HybridKemKeyPair::generate();
        let sender = HybridSigner::generate();
        let sealed =
            seal_signed(&recipient.public_keys(), &sender, b"for recipient").expect("seal");
        // Signature is valid, but the wrong KEM keypair can't recover the AEAD key.
        assert!(open_verified(&other, &sender.verifying_keys(), &sealed).is_err());
    }
}
