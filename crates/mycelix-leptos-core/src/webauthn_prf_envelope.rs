// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Isolated WebAuthn PRF-to-envelope-key derivation contract.
//!
//! This module deliberately does **not** perform a WebAuthn ceremony, read or
//! write browser storage, encrypt the identity seed, or claim a credential is
//! trusted/hardware-backed. It accepts bytes only after a higher-level caller
//! has completed a user-consented ceremony, verified the expected RP ID and
//! credential ID, and obtained a 32-byte PRF extension result.
//!
//! The same 32-byte `envelope_salt` must be saved as envelope metadata and
//! supplied as the PRF evaluation input during subsequent assertions. It is
//! public metadata, not a secret. A new envelope must use a fresh salt from a
//! cryptographically secure random source.
//!
//! The derived key is scoped by HKDF-SHA-256 domain separation to the Mycelix
//! profile/version, canonical relying-party ID, credential ID, and stored
//! Ed25519 public key. A protocol/serialization caller must still bind and
//! authenticate those metadata fields in the actual AEAD envelope (for
//! example as authenticated additional data). Derivation alone is not an
//! authenticated envelope and is not enabled in production identity flows.

use hkdf::Hkdf;
use sha2::Sha256;
use std::fmt;
use zeroize::{Zeroize, ZeroizeOnDrop};

pub const PRF_OUTPUT_LEN: usize = 32;
pub const ENVELOPE_SALT_LEN: usize = 32;
pub const IDENTITY_PUBLIC_KEY_LEN: usize = 32;
pub const MAX_CREDENTIAL_ID_LEN: usize = 1_024;

const DOMAIN_SEPARATOR: &[u8] = b"LUMINOUS-DYNAMICS/MYCELIX/WEBAUTHN-PRF-ENVELOPE/V1\0";
const ENVELOPE_PURPOSE: &[u8] = b"ed25519-seed-wrap/aes-256-gcm";

/// Inputs that bind one PRF-derived envelope key to one credential and identity.
#[derive(Debug, Clone, Copy)]
pub struct WebAuthnPrfEnvelopeContext<'a> {
    /// Canonical lower-case ASCII RP ID (a hostname/A-label, not an origin URL).
    pub relying_party_id: &'a str,
    /// Opaque credential ID returned by the WebAuthn authenticator.
    pub credential_id: &'a [u8],
    /// Fresh 32-byte value used both as the WebAuthn PRF evaluation input and HKDF salt.
    pub envelope_salt: &'a [u8],
    /// Stable Mycelix Ed25519 public-key metadata for the private key being wrapped.
    pub identity_public_key: &'a [u8],
}

/// Safe-to-store wrapper for a derived envelope encryption key.
///
/// The key bytes are not printed by Debug and are zeroized when this value is
/// dropped. Callers should expose them only briefly to a vetted AEAD key import
/// and must not store them, serialize them, or put them in logs/telemetry.
#[derive(Zeroize, ZeroizeOnDrop)]
pub struct PrfEnvelopeKey([u8; 32]);

impl PrfEnvelopeKey {
    /// Borrow key bytes for a cryptographic API import. Do not retain the slice.
    pub fn expose_for_crypto_import(&self) -> &[u8; 32] {
        &self.0
    }
}

impl fmt::Debug for PrfEnvelopeKey {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str("PrfEnvelopeKey([REDACTED])")
    }
}

/// Non-sensitive failure category. Error values contain no PRF output/key bytes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PrfEnvelopeDerivationError {
    InvalidPrfOutputLength,
    InvalidEnvelopeSaltLength,
    InvalidRelyingPartyId,
    EmptyCredentialId,
    CredentialIdTooLong,
    InvalidIdentityPublicKeyLength,
    ContextLengthOverflow,
    HkdfExpansionFailed,
}

impl fmt::Display for PrfEnvelopeDerivationError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let message = match self {
            Self::InvalidPrfOutputLength => "WebAuthn PRF output must be exactly 32 bytes",
            Self::InvalidEnvelopeSaltLength => "Envelope salt must be exactly 32 bytes",
            Self::InvalidRelyingPartyId => "RP ID must be a canonical lower-case ASCII hostname",
            Self::EmptyCredentialId => "Credential ID must not be empty",
            Self::CredentialIdTooLong => "Credential ID exceeds the supported size bound",
            Self::InvalidIdentityPublicKeyLength => "Identity public key must be exactly 32 bytes",
            Self::ContextLengthOverflow => "Key-derivation context length is not representable",
            Self::HkdfExpansionFailed => "HKDF expansion failed",
        };
        f.write_str(message)
    }
}

impl std::error::Error for PrfEnvelopeDerivationError {}

/// Derive a 256-bit envelope key from verified WebAuthn PRF output.
///
/// The PRF output and salt must each be exactly 32 bytes. The RP ID must be a
/// canonical lower-case ASCII hostname (IDNs should be passed as their
/// canonical A-label form); credential IDs are opaque and bounded to 1024
/// bytes; the bound identity public key must be exactly 32 bytes.
///
/// The caller must verify the WebAuthn origin/RP, user verification policy,
/// credential ID, challenge freshness and PRF extension result before calling
/// this function. A successful derivation only means the inputs satisfy this
/// local contract. It does not establish that a credential was authorized,
/// that a key is hardware-backed, or that the resulting envelope is authentic.
pub fn derive_prf_envelope_key(
    prf_output: &[u8],
    context: &WebAuthnPrfEnvelopeContext<'_>,
) -> Result<PrfEnvelopeKey, PrfEnvelopeDerivationError> {
    if prf_output.len() != PRF_OUTPUT_LEN {
        return Err(PrfEnvelopeDerivationError::InvalidPrfOutputLength);
    }
    if context.envelope_salt.len() != ENVELOPE_SALT_LEN {
        return Err(PrfEnvelopeDerivationError::InvalidEnvelopeSaltLength);
    }
    if !is_canonical_rp_id(context.relying_party_id) {
        return Err(PrfEnvelopeDerivationError::InvalidRelyingPartyId);
    }
    if context.credential_id.is_empty() {
        return Err(PrfEnvelopeDerivationError::EmptyCredentialId);
    }
    if context.credential_id.len() > MAX_CREDENTIAL_ID_LEN {
        return Err(PrfEnvelopeDerivationError::CredentialIdTooLong);
    }
    if context.identity_public_key.len() != IDENTITY_PUBLIC_KEY_LEN {
        return Err(PrfEnvelopeDerivationError::InvalidIdentityPublicKeyLength);
    }

    // Length-prefix every variable field to avoid ambiguous concatenations.
    // Fields are small and individually bounded above; checked conversions are
    // retained so a future schema expansion cannot silently truncate lengths.
    let mut info = Vec::with_capacity(
        DOMAIN_SEPARATOR.len()
            + context.relying_party_id.len()
            + context.credential_id.len()
            + context.identity_public_key.len()
            + ENVELOPE_PURPOSE.len()
            + 16,
    );
    info.extend_from_slice(DOMAIN_SEPARATOR);
    append_length_prefixed(&mut info, context.relying_party_id.as_bytes())?;
    append_length_prefixed(&mut info, context.credential_id)?;
    append_length_prefixed(&mut info, context.identity_public_key)?;
    append_length_prefixed(&mut info, ENVELOPE_PURPOSE)?;

    let hkdf = Hkdf::<Sha256>::new(Some(context.envelope_salt), prf_output);
    let mut key_bytes = [0u8; 32];
    if hkdf.expand(&info, &mut key_bytes).is_err() {
        key_bytes.zeroize();
        return Err(PrfEnvelopeDerivationError::HkdfExpansionFailed);
    }
    Ok(PrfEnvelopeKey(key_bytes))
}

fn append_length_prefixed(
    target: &mut Vec<u8>,
    value: &[u8],
) -> Result<(), PrfEnvelopeDerivationError> {
    let length = u32::try_from(value.len())
        .map_err(|_| PrfEnvelopeDerivationError::ContextLengthOverflow)?;
    target.extend_from_slice(&length.to_be_bytes());
    target.extend_from_slice(value);
    Ok(())
}

fn is_canonical_rp_id(rp_id: &str) -> bool {
    if rp_id.is_empty()
        || rp_id.len() > 253
        || !rp_id.is_ascii()
        || rp_id.bytes().any(|byte| byte.is_ascii_uppercase())
        || rp_id.ends_with('.')
    {
        return false;
    }
    rp_id.split('.').all(|label| {
        !label.is_empty()
            && label.len() <= 63
            && !label.starts_with('-')
            && !label.ends_with('-')
            && label
                .bytes()
                .all(|byte| byte.is_ascii_lowercase() || byte.is_ascii_digit() || byte == b'-')
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn context<'a>(
        rp_id: &'a str,
        credential_id: &'a [u8],
        salt: &'a [u8],
        public_key: &'a [u8],
    ) -> WebAuthnPrfEnvelopeContext<'a> {
        WebAuthnPrfEnvelopeContext {
            relying_party_id: rp_id,
            credential_id,
            envelope_salt: salt,
            identity_public_key: public_key,
        }
    }

    fn fixed_inputs() -> ([u8; 32], [u8; 32], [u8; 32], [u8; 4]) {
        let prf_output = core::array::from_fn(|index| index as u8);
        let salt = core::array::from_fn(|index| (index + 32) as u8);
        let public_key = core::array::from_fn(|index| (index + 64) as u8);
        (prf_output, salt, public_key, [0, 1, 2, 255])
    }

    #[test]
    fn hkdf_derivation_matches_frozen_test_vector() {
        let (prf_output, salt, public_key, credential_id) = fixed_inputs();
        let key = derive_prf_envelope_key(
            &prf_output,
            &context("mycelix.example", &credential_id, &salt, &public_key),
        )
        .unwrap();
        assert_eq!(
            key.expose_for_crypto_import(),
            &[
                0x87, 0xed, 0x01, 0x3b, 0xdc, 0x86, 0xee, 0x9c,
                0xaa, 0x3a, 0x25, 0xe0, 0x73, 0xea, 0x40, 0x98,
                0x87, 0xfd, 0xe3, 0xda, 0x27, 0x68, 0xa0, 0xee,
                0xe1, 0xf2, 0x98, 0xed, 0x06, 0xf7, 0x42, 0xf7,
            ]
        );
    }

    #[test]
    fn derivation_is_deterministic_for_identical_context() {
        let (prf_output, salt, public_key, credential_id) = fixed_inputs();
        let first = derive_prf_envelope_key(
            &prf_output,
            &context("mycelix.example", &credential_id, &salt, &public_key),
        )
        .unwrap();
        let second = derive_prf_envelope_key(
            &prf_output,
            &context("mycelix.example", &credential_id, &salt, &public_key),
        )
        .unwrap();
        assert_eq!(
            first.expose_for_crypto_import(),
            second.expose_for_crypto_import()
        );
    }

    #[test]
    fn rp_credential_salt_and_identity_key_are_domain_separated() {
        let (prf_output, salt, public_key, credential_id) = fixed_inputs();
        let base = derive_prf_envelope_key(
            &prf_output,
            &context("mycelix.example", &credential_id, &salt, &public_key),
        )
        .unwrap();

        let other_rp = derive_prf_envelope_key(
            &prf_output,
            &context("other.example", &credential_id, &salt, &public_key),
        )
        .unwrap();
        let other_credential_id = [0, 1, 2, 254];
        let other_credential = derive_prf_envelope_key(
            &prf_output,
            &context("mycelix.example", &other_credential_id, &salt, &public_key),
        )
        .unwrap();
        let other_salt = [9u8; 32];
        let other_salt_key = derive_prf_envelope_key(
            &prf_output,
            &context("mycelix.example", &credential_id, &other_salt, &public_key),
        )
        .unwrap();
        let other_public = [10u8; 32];
        let other_identity = derive_prf_envelope_key(
            &prf_output,
            &context("mycelix.example", &credential_id, &salt, &other_public),
        )
        .unwrap();

        assert_ne!(base.expose_for_crypto_import(), other_rp.expose_for_crypto_import());
        assert_ne!(base.expose_for_crypto_import(), other_credential.expose_for_crypto_import());
        assert_ne!(base.expose_for_crypto_import(), other_salt_key.expose_for_crypto_import());
        assert_ne!(base.expose_for_crypto_import(), other_identity.expose_for_crypto_import());
    }

    #[test]
    fn invalid_prf_output_salt_and_public_key_lengths_fail_closed() {
        let (_, salt, public_key, credential_id) = fixed_inputs();
        assert_eq!(
            derive_prf_envelope_key(
                &[0u8; 31],
                &context("mycelix.example", &credential_id, &salt, &public_key)
            )
            .unwrap_err(),
            PrfEnvelopeDerivationError::InvalidPrfOutputLength
        );
        assert_eq!(
            derive_prf_envelope_key(
                &[0u8; 32],
                &context("mycelix.example", &credential_id, &[0u8; 31], &public_key)
            )
            .unwrap_err(),
            PrfEnvelopeDerivationError::InvalidEnvelopeSaltLength
        );
        assert_eq!(
            derive_prf_envelope_key(
                &[0u8; 32],
                &context("mycelix.example", &credential_id, &salt, &[0u8; 31])
            )
            .unwrap_err(),
            PrfEnvelopeDerivationError::InvalidIdentityPublicKeyLength
        );
    }

    #[test]
    fn invalid_rp_id_and_empty_or_oversized_credentials_fail_closed() {
        let (prf_output, salt, public_key, credential_id) = fixed_inputs();
        for invalid_rp in [
            "",
            "Mycelix.example",
            "https://mycelix.example",
            "mycelix.example/",
            "-mycelix.example",
            "mycelix..example",
            "mycelix.example.",
            "mýcelix.example",
        ] {
            assert_eq!(
                derive_prf_envelope_key(
                    &prf_output,
                    &context(invalid_rp, &credential_id, &salt, &public_key)
                )
                .unwrap_err(),
                PrfEnvelopeDerivationError::InvalidRelyingPartyId
            );
        }
        assert_eq!(
            derive_prf_envelope_key(
                &prf_output,
                &context("mycelix.example", &[], &salt, &public_key)
            )
            .unwrap_err(),
            PrfEnvelopeDerivationError::EmptyCredentialId
        );
        let oversized_credential = vec![0u8; MAX_CREDENTIAL_ID_LEN + 1];
        assert_eq!(
            derive_prf_envelope_key(
                &prf_output,
                &context("mycelix.example", &oversized_credential, &salt, &public_key)
            )
            .unwrap_err(),
            PrfEnvelopeDerivationError::CredentialIdTooLong
        );
    }

    #[test]
    fn length_prefixes_prevent_ambiguous_context_concatenation() {
        let (prf_output, salt, public_key, _) = fixed_inputs();
        let first = derive_prf_envelope_key(
            &prf_output,
            &context("mycelix.example", &[0x41], &salt, &[0x42; 32]),
        )
        .unwrap();
        let second = derive_prf_envelope_key(
            &prf_output,
            &context("mycelix.example", &[0x41, 0x42], &salt, &[0x42; 32]),
        )
        .unwrap();
        assert_ne!(
            first.expose_for_crypto_import(),
            second.expose_for_crypto_import()
        );
        assert_eq!(public_key.len(), IDENTITY_PUBLIC_KEY_LEN);
    }

    #[test]
    fn debug_output_redacts_derived_key_bytes() {
        let (prf_output, salt, public_key, credential_id) = fixed_inputs();
        let key = derive_prf_envelope_key(
            &prf_output,
            &context("mycelix.example", &credential_id, &salt, &public_key),
        )
        .unwrap();
        let debug = format!("{key:?}");
        assert_eq!(debug, "PrfEnvelopeKey([REDACTED])");
        assert!(!debug.contains("87ed013b"));
    }
}
