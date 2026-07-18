// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Pulse V2 hybrid-PQC cryptographic primitives.
//!
//! This module deliberately does not define the wire envelope. The caller
//! constructs the normative Pulse transcript/AAD and passes it here. It uses
//! X25519 + ML-KEM-768, a domain-separated HKDF-SHA256 combiner,
//! AES-256-GCM, and an application-level ML-DSA-65 signature. The separate
//! Holochain agent signature is produced and verified by the conductor.

use crate::error::CryptoError;
use aes_gcm::aead::{Aead, Payload};
use aes_gcm::{Aes256Gcm, KeyInit, Nonce};
use hkdf::Hkdf;
use ml_dsa::signature::{Keypair as _, Signer as _, Verifier as _};
use ml_dsa::{
    EncodedSignature, EncodedVerifyingKey, Generate as _, KeyExport as _, KeyInit as _, MlDsa65,
    Signature, SigningKey, VerifyingKey,
};
use ml_kem::kem::{Decapsulate, Encapsulate, Seed};
use ml_kem::ml_kem_768::{DecapsulationKey, EncapsulationKey};
use ml_kem::{Kem, MlKem768, TryKeyInit};
use rand::RngCore;
use rand::rngs::OsRng;
use sha2::Sha256;
use x25519_dalek::{EphemeralSecret, PublicKey, StaticSecret};
use zeroize::Zeroize;

pub const ML_KEM_768_PUBLIC_KEY_BYTES: usize = 1184;
pub const ML_KEM_768_CIPHERTEXT_BYTES: usize = 1088;
pub const ML_DSA_65_PUBLIC_KEY_BYTES: usize = 1952;
pub const ML_DSA_65_SIGNATURE_BYTES: usize = 3309;
pub const AES_GCM_NONCE_BYTES: usize = 12;

const HKDF_INFO: &[u8] = b"mycelix-pulse/v2/hybrid-kem/aes-256-gcm\0";
const HKDF_SALT_DOMAIN: &[u8] = b"mycelix-pulse/v2/hybrid-kem/salt\0";

pub struct RecipientKeyPair {
    x25519_secret: StaticSecret,
    ml_kem_secret: DecapsulationKey,
    ml_kem_public: EncapsulationKey,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RecipientPublicKeys {
    pub x25519: [u8; 32],
    pub ml_kem_768: Vec<u8>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SealedPayload {
    pub x25519_ephemeral_public: [u8; 32],
    pub ml_kem_ciphertext: Vec<u8>,
    pub nonce: [u8; AES_GCM_NONCE_BYTES],
    pub ciphertext: Vec<u8>,
}

impl RecipientKeyPair {
    pub fn generate() -> Self {
        let x25519_secret = StaticSecret::random_from_rng(OsRng);
        let (ml_kem_secret, ml_kem_public) = MlKem768::generate_keypair();
        Self {
            x25519_secret,
            ml_kem_secret,
            ml_kem_public,
        }
    }

    pub fn public_keys(&self) -> RecipientPublicKeys {
        RecipientPublicKeys {
            x25519: PublicKey::from(&self.x25519_secret).to_bytes(),
            ml_kem_768: self.ml_kem_public.to_bytes().to_vec(),
        }
    }

    /// Compact device-local secret representation. It must be wrapped before
    /// persistence and is never a portable Pulse export format.
    pub fn to_device_secret(&self) -> Result<Vec<u8>, CryptoError> {
        let seed = self
            .ml_kem_secret
            .to_seed()
            .ok_or_else(|| CryptoError::Validation("ML-KEM key has no recoverable seed".into()))?;
        let mut bytes = Vec::with_capacity(32 + seed.as_slice().len());
        bytes.extend_from_slice(&self.x25519_secret.to_bytes());
        bytes.extend_from_slice(seed.as_slice());
        Ok(bytes)
    }

    pub fn from_device_secret(bytes: &[u8]) -> Result<Self, CryptoError> {
        if bytes.len() != 96 {
            return Err(CryptoError::Validation(
                "invalid Pulse V2 recipient secret length".into(),
            ));
        }
        let x25519_secret = StaticSecret::from(
            <[u8; 32]>::try_from(&bytes[..32])
                .map_err(|_| CryptoError::Validation("invalid X25519 secret".into()))?,
        );
        let seed = Seed::<MlKem768>::try_from(&bytes[32..])
            .map_err(|_| CryptoError::Validation("invalid ML-KEM seed".into()))?;
        let ml_kem_secret = DecapsulationKey::from(seed);
        let ml_kem_public = ml_kem_secret.encapsulation_key().clone();
        Ok(Self {
            x25519_secret,
            ml_kem_secret,
            ml_kem_public,
        })
    }

    pub fn open(&self, sealed: &SealedPayload, aad: &[u8]) -> Result<Vec<u8>, CryptoError> {
        validate_sealed(sealed)?;
        let peer = PublicKey::from(sealed.x25519_ephemeral_public);
        let classical = self.x25519_secret.diffie_hellman(&peer);
        let post_quantum = self
            .ml_kem_secret
            .decapsulate_slice(&sealed.ml_kem_ciphertext)
            .map_err(|_| CryptoError::Validation("ML-KEM decapsulation failed".into()))?;
        let mut key = combine_shared_secrets(
            &sealed.x25519_ephemeral_public,
            &sealed.ml_kem_ciphertext,
            classical.as_bytes(),
            post_quantum.as_slice(),
        );
        let cipher = Aes256Gcm::new_from_slice(&key)
            .map_err(|_| CryptoError::Validation("invalid AES key".into()))?;
        let result = cipher
            .decrypt(
                Nonce::from_slice(&sealed.nonce),
                Payload {
                    msg: &sealed.ciphertext,
                    aad,
                },
            )
            .map_err(|_| CryptoError::Validation("AES-GCM authentication failed".into()));
        key.zeroize();
        result
    }
}

pub fn seal(
    recipient: &RecipientPublicKeys,
    plaintext: &[u8],
    aad: &[u8],
) -> Result<SealedPayload, CryptoError> {
    seal_with_aad(recipient, plaintext, |_, _, _| Ok(aad.to_vec()))
}

/// Seal while constructing AAD from the freshly generated KEM transcript and
/// nonce. Pulse uses this form so its canonical envelope header binds all
/// three values without predicting or later mutating them.
pub fn seal_with_aad<F>(
    recipient: &RecipientPublicKeys,
    plaintext: &[u8],
    aad_for: F,
) -> Result<SealedPayload, CryptoError>
where
    F: FnOnce(&[u8; 32], &[u8], &[u8; AES_GCM_NONCE_BYTES]) -> Result<Vec<u8>, CryptoError>,
{
    if recipient.ml_kem_768.len() != ML_KEM_768_PUBLIC_KEY_BYTES {
        return Err(CryptoError::Validation(
            "invalid ML-KEM-768 public key length".into(),
        ));
    }
    let ephemeral_secret = EphemeralSecret::random_from_rng(OsRng);
    let ephemeral_public = PublicKey::from(&ephemeral_secret).to_bytes();
    let classical = ephemeral_secret.diffie_hellman(&PublicKey::from(recipient.x25519));
    let encapsulation_key = EncapsulationKey::new_from_slice(&recipient.ml_kem_768)
        .map_err(|_| CryptoError::Validation("invalid ML-KEM-768 public key".into()))?;
    let (ml_kem_ciphertext, post_quantum) = encapsulation_key.encapsulate();
    let ml_kem_ciphertext = ml_kem_ciphertext.as_slice().to_vec();
    let mut key = combine_shared_secrets(
        &ephemeral_public,
        &ml_kem_ciphertext,
        classical.as_bytes(),
        post_quantum.as_slice(),
    );
    let mut nonce = [0u8; AES_GCM_NONCE_BYTES];
    OsRng.fill_bytes(&mut nonce);
    let aad = aad_for(&ephemeral_public, &ml_kem_ciphertext, &nonce)?;
    let cipher = Aes256Gcm::new_from_slice(&key)
        .map_err(|_| CryptoError::Validation("invalid AES key".into()))?;
    let result = cipher
        .encrypt(
            Nonce::from_slice(&nonce),
            Payload {
                msg: plaintext,
                aad: &aad,
            },
        )
        .map_err(|_| CryptoError::Validation("AES-GCM encryption failed".into()));
    key.zeroize();
    Ok(SealedPayload {
        x25519_ephemeral_public: ephemeral_public,
        ml_kem_ciphertext,
        nonce,
        ciphertext: result?,
    })
}

fn validate_sealed(sealed: &SealedPayload) -> Result<(), CryptoError> {
    if sealed.ml_kem_ciphertext.len() != ML_KEM_768_CIPHERTEXT_BYTES {
        return Err(CryptoError::Validation(
            "invalid ML-KEM-768 ciphertext length".into(),
        ));
    }
    if sealed.ciphertext.len() < 16 {
        return Err(CryptoError::Validation(
            "AES-GCM ciphertext is shorter than its tag".into(),
        ));
    }
    Ok(())
}

fn combine_shared_secrets(
    ephemeral_public: &[u8; 32],
    ml_kem_ciphertext: &[u8],
    classical: &[u8],
    post_quantum: &[u8],
) -> [u8; 32] {
    let mut salt = Vec::with_capacity(HKDF_SALT_DOMAIN.len() + 32 + ml_kem_ciphertext.len());
    salt.extend_from_slice(HKDF_SALT_DOMAIN);
    salt.extend_from_slice(ephemeral_public);
    salt.extend_from_slice(ml_kem_ciphertext);
    let mut ikm = Vec::with_capacity(8 + classical.len() + post_quantum.len());
    ikm.extend_from_slice(&(classical.len() as u32).to_be_bytes());
    ikm.extend_from_slice(classical);
    ikm.extend_from_slice(&(post_quantum.len() as u32).to_be_bytes());
    ikm.extend_from_slice(post_quantum);
    let hkdf = Hkdf::<Sha256>::new(Some(&salt), &ikm);
    let mut key = [0u8; 32];
    hkdf.expand(HKDF_INFO, &mut key)
        .expect("32-byte HKDF output is valid");
    ikm.zeroize();
    key
}

pub struct MlDsaSigner(SigningKey<MlDsa65>);

impl MlDsaSigner {
    pub fn generate() -> Self {
        Self(SigningKey::<MlDsa65>::generate())
    }
    pub fn public_key(&self) -> Vec<u8> {
        self.0.verifying_key().encode().as_slice().to_vec()
    }
    pub fn sign(&self, transcript: &[u8]) -> Vec<u8> {
        self.0.sign(transcript).encode().as_slice().to_vec()
    }
    pub fn to_device_secret(&self) -> Vec<u8> {
        self.0.to_bytes().as_slice().to_vec()
    }
    pub fn from_device_secret(bytes: &[u8]) -> Result<Self, CryptoError> {
        SigningKey::<MlDsa65>::new_from_slice(bytes)
            .map(Self)
            .map_err(|_| CryptoError::Validation("invalid ML-DSA-65 signing seed".into()))
    }
}

pub fn verify_ml_dsa(
    public_key: &[u8],
    transcript: &[u8],
    signature: &[u8],
) -> Result<(), CryptoError> {
    if public_key.len() != ML_DSA_65_PUBLIC_KEY_BYTES
        || signature.len() != ML_DSA_65_SIGNATURE_BYTES
    {
        return Err(CryptoError::Validation(
            "invalid ML-DSA-65 component length".into(),
        ));
    }
    let encoded_key = EncodedVerifyingKey::<MlDsa65>::try_from(public_key)
        .map_err(|_| CryptoError::Validation("invalid ML-DSA-65 public key".into()))?;
    let key = VerifyingKey::<MlDsa65>::decode(&encoded_key);
    let encoded_signature = EncodedSignature::<MlDsa65>::try_from(signature)
        .map_err(|_| CryptoError::Validation("invalid ML-DSA-65 signature".into()))?;
    let signature = Signature::<MlDsa65>::decode(&encoded_signature)
        .ok_or_else(|| CryptoError::Validation("undecodable ML-DSA-65 signature".into()))?;
    key.verify(transcript, &signature)
        .map_err(|_| CryptoError::Validation("ML-DSA-65 verification failed".into()))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn seal_open_binds_aad() {
        let recipient = RecipientKeyPair::generate();
        let sealed = seal(
            &recipient.public_keys(),
            b"one atomic payload",
            b"canonical aad",
        )
        .unwrap();
        assert_eq!(
            recipient.open(&sealed, b"canonical aad").unwrap(),
            b"one atomic payload"
        );
        assert!(recipient.open(&sealed, b"substituted aad").is_err());
    }

    #[test]
    fn aad_builder_observes_exact_wire_components() {
        let recipient = RecipientKeyPair::generate();
        let sealed = seal_with_aad(&recipient.public_keys(), b"payload", |eph, kem, nonce| {
            let mut aad = Vec::new();
            aad.extend_from_slice(eph);
            aad.extend_from_slice(kem);
            aad.extend_from_slice(nonce);
            Ok(aad)
        })
        .unwrap();
        let mut aad = Vec::new();
        aad.extend_from_slice(&sealed.x25519_ephemeral_public);
        aad.extend_from_slice(&sealed.ml_kem_ciphertext);
        aad.extend_from_slice(&sealed.nonce);
        assert_eq!(recipient.open(&sealed, &aad).unwrap(), b"payload");
    }

    #[test]
    fn both_kem_contributions_change_the_key() {
        let eph = [1; 32];
        let ct = vec![2; ML_KEM_768_CIPHERTEXT_BYTES];
        let base = combine_shared_secrets(&eph, &ct, &[3; 32], &[4; 32]);
        assert_ne!(base, combine_shared_secrets(&eph, &ct, &[5; 32], &[4; 32]));
        assert_ne!(base, combine_shared_secrets(&eph, &ct, &[3; 32], &[6; 32]));
    }

    #[test]
    fn kem_secret_round_trip_and_wrong_recipient_failure() {
        let recipient = RecipientKeyPair::generate();
        let restored =
            RecipientKeyPair::from_device_secret(&recipient.to_device_secret().unwrap()).unwrap();
        let sealed = seal(&recipient.public_keys(), b"recoverable here", b"aad").unwrap();
        assert_eq!(restored.open(&sealed, b"aad").unwrap(), b"recoverable here");
        assert!(RecipientKeyPair::generate().open(&sealed, b"aad").is_err());
    }

    #[test]
    fn ml_dsa_round_trip_and_tampering_failure() {
        let signer = MlDsaSigner::generate();
        let restored = MlDsaSigner::from_device_secret(&signer.to_device_secret()).unwrap();
        let signature = restored.sign(b"canonical transcript");
        assert!(verify_ml_dsa(&signer.public_key(), b"canonical transcript", &signature).is_ok());
        assert!(verify_ml_dsa(&signer.public_key(), b"changed transcript", &signature).is_err());
    }

    #[test]
    fn malformed_components_fail_closed() {
        let recipient = RecipientKeyPair::generate();
        let mut sealed = seal(&recipient.public_keys(), b"payload", b"aad").unwrap();
        sealed.ml_kem_ciphertext.pop();
        assert!(recipient.open(&sealed, b"aad").is_err());
        let signer = MlDsaSigner::generate();
        assert!(verify_ml_dsa(&signer.public_key(), b"x", &[0; 12]).is_err());
    }
}
