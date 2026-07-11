// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! String-field encryption: store an authenticated hybrid ciphertext inside an
//! existing plaintext `String` DHT field (e.g. `HealthRecord.data`) with **zero
//! schema change**. This is the recommended zero-risk Phase-4 integration — a
//! client encrypts, then marks + bs58-encodes into the field it already writes;
//! pre-encryption values simply lack the marker and are still valid plaintext.
//!
//! The client is responsible for holding the recipient [`HybridKemKeyPair`] and
//! sender [`HybridSigner`] (for a personal vault these are the same agent —
//! self-encryption). See `PQC_ROADMAP_2026-07-07.md`.
//!
//! # Stability
//! EXPERIMENTAL — feature `hybrid-rc`, off by default. Pending crypto audit.

use crate::error::CryptoError;
use crate::hybrid_kem::{HybridCiphertext, HybridKemKeyPair, HybridPublicKeys};
use crate::hybrid_sig::{HybridSignature, HybridSigner, HybridVerifyingKeys};
use crate::sealed_box::{SignedSealed, open_verified, seal_signed};

/// Marker prefix identifying a mycelix-crypto v1 encrypted field. Chosen to be
/// vanishingly unlikely to collide with legitimate plaintext record data.
pub const FIELD_MARKER: &str = "mxc1:";

/// Whether a field value is a mycelix-crypto encrypted field.
pub fn is_encrypted_field(value: &str) -> bool {
    value.starts_with(FIELD_MARKER)
}

/// Encrypt `plaintext` and encode it as a marked string safe to store in a
/// plaintext `String` field. Old (unmarked) values remain valid plaintext.
pub fn encrypt_field(
    recipient: &HybridPublicKeys,
    sender: &HybridSigner,
    plaintext: &[u8],
) -> Result<String, CryptoError> {
    let sealed = seal_signed(recipient, sender, plaintext)?;
    let bytes = encode_sealed(&sealed);
    Ok(format!(
        "{FIELD_MARKER}{}",
        bs58::encode(bytes).into_string()
    ))
}

/// Decrypt a marked field. Errors if the value is not a marked encrypted field,
/// or if signature verification or decryption fails.
pub fn decrypt_field(
    recipient: &HybridKemKeyPair,
    sender_keys: &HybridVerifyingKeys,
    value: &str,
) -> Result<Vec<u8>, CryptoError> {
    let b58 = value
        .strip_prefix(FIELD_MARKER)
        .ok_or_else(|| CryptoError::Validation("value is not a mycelix encrypted field".into()))?;
    let bytes = bs58::decode(b58)
        .into_vec()
        .map_err(|_| CryptoError::Validation("invalid bs58 in encrypted field".into()))?;
    let sealed = decode_sealed(&bytes)?;
    open_verified(recipient, sender_keys, &sealed)
}

// --- compact length-prefixed codec for SignedSealed -------------------------
// Layout: x25519_eph_pk[32] | u32 len + ml_kem_ct | nonce[24]
//         | u32 len + ciphertext | ed25519_sig[64] | u32 len + ml_dsa_sig

fn encode_sealed(s: &SignedSealed) -> Vec<u8> {
    let mut out = Vec::new();
    out.extend_from_slice(&s.ciphertext.x25519_eph_pk);
    put_bytes(&mut out, &s.ciphertext.ml_kem_ct);
    out.extend_from_slice(&s.ciphertext.nonce);
    put_bytes(&mut out, &s.ciphertext.ciphertext);
    out.extend_from_slice(&s.signature.ed25519);
    put_bytes(&mut out, &s.signature.ml_dsa);
    out
}

fn decode_sealed(bytes: &[u8]) -> Result<SignedSealed, CryptoError> {
    let mut c = Cursor { bytes, pos: 0 };
    let x25519_eph_pk = c.take_array::<32>()?;
    let ml_kem_ct = c.take_bytes()?;
    let nonce = c.take_array::<24>()?;
    let ciphertext = c.take_bytes()?;
    let ed25519 = c.take_array::<64>()?;
    let ml_dsa = c.take_bytes()?;
    if c.pos != bytes.len() {
        return Err(CryptoError::Validation(
            "trailing bytes in encrypted field".into(),
        ));
    }
    Ok(SignedSealed {
        ciphertext: HybridCiphertext {
            x25519_eph_pk,
            ml_kem_ct,
            nonce,
            ciphertext,
        },
        signature: HybridSignature { ed25519, ml_dsa },
    })
}

fn put_bytes(out: &mut Vec<u8>, b: &[u8]) {
    let len = u32::try_from(b.len()).expect("field component length fits in u32");
    out.extend_from_slice(&len.to_le_bytes());
    out.extend_from_slice(b);
}

struct Cursor<'a> {
    bytes: &'a [u8],
    pos: usize,
}

impl Cursor<'_> {
    fn take(&mut self, n: usize) -> Result<&[u8], CryptoError> {
        let end = self
            .pos
            .checked_add(n)
            .filter(|e| *e <= self.bytes.len())
            .ok_or_else(|| CryptoError::Validation("truncated encrypted field".into()))?;
        let slice = &self.bytes[self.pos..end];
        self.pos = end;
        Ok(slice)
    }

    fn take_array<const N: usize>(&mut self) -> Result<[u8; N], CryptoError> {
        let s = self.take(N)?;
        let mut a = [0u8; N];
        a.copy_from_slice(s);
        Ok(a)
    }

    fn take_bytes(&mut self) -> Result<Vec<u8>, CryptoError> {
        let len_bytes = self.take(4)?;
        let len = u32::from_le_bytes([len_bytes[0], len_bytes[1], len_bytes[2], len_bytes[3]]);
        Ok(self.take(len as usize)?.to_vec())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn encrypt_field_round_trip() {
        let patient = HybridKemKeyPair::generate();
        let writer = HybridSigner::generate();
        let phi = b"{\"condition\":\"confidential\"}";

        let field = encrypt_field(&patient.public_keys(), &writer, phi).expect("encrypt");
        assert!(is_encrypted_field(&field));
        assert!(
            !field.contains("confidential"),
            "PHI must not appear in the field"
        );

        let out = decrypt_field(&patient, &writer.verifying_keys(), &field).expect("decrypt");
        assert_eq!(out, phi);
    }

    #[test]
    fn plaintext_is_not_treated_as_encrypted() {
        assert!(!is_encrypted_field("{\"allergy\":\"peanuts\"}"));
        let patient = HybridKemKeyPair::generate();
        let writer = HybridSigner::generate();
        // Decrypting a plaintext (unmarked) value must error, not misparse.
        assert!(decrypt_field(&patient, &writer.verifying_keys(), "plain data").is_err());
    }

    #[test]
    fn corrupted_field_is_rejected() {
        let patient = HybridKemKeyPair::generate();
        let writer = HybridSigner::generate();
        let field = encrypt_field(&patient.public_keys(), &writer, b"secret").expect("encrypt");
        // Truncate the encoded body: decode/verify must fail cleanly, never panic.
        let corrupted = &field[..field.len() - 4];
        assert!(decrypt_field(&patient, &writer.verifying_keys(), corrupted).is_err());
    }
}
