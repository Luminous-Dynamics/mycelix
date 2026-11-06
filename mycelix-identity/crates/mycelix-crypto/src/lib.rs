// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! # mycelix-crypto
//!
//! Crypto-agile types for the Mycelix Identity system.
//!
//! This crate provides algorithm-tagged wrappers for public keys, signatures,
//! and encrypted envelopes that enable the identity hApp to work with multiple
//! cryptographic algorithms (Ed25519 today, ML-DSA/hybrid/ML-KEM in the future)
//! without hardcoding any single algorithm.
//!
//! ## Feature flags
//!
//! - **`wasm`** — Types and validation only; suitable for `wasm32-unknown-unknown`
//!   targets (Holochain WASM zomes). No actual cryptographic operations.
//!
//! - **`native`** — Adds trait implementations using legacy `pqcrypto-*` /
//!   PQClean C-FFI wrappers. This is a compatibility/research path, not a
//!   qualified final-FIPS-204/205 provider. The wrappers are not independently
//!   audited and may fail the final-profile signature-length contract. It
//!   remains off by default; passing a size guard is not standards conformance.

pub mod algorithm;
pub mod envelope;
pub mod error;

#[cfg(feature = "native")]
pub mod traits;

#[cfg(feature = "native")]
pub mod pqc;

/// Wasm-capable hybrid X25519+ML-KEM-768 KEM (RustCrypto). Feature `hybrid-rc`.
#[cfg(feature = "hybrid-rc")]
pub mod hybrid_kem;

/// Wasm-capable hybrid Ed25519+ML-DSA-65 signatures (RustCrypto). Feature `hybrid-rc`.
#[cfg(feature = "hybrid-rc")]
pub mod hybrid_sig;

/// Pulse V2 hybrid-PQC primitives with caller-supplied canonical AAD.
/// Feature `hybrid-rc`; experimental until the Pulse implementation-evidence
/// and independent-review gates pass.
#[cfg(feature = "hybrid-rc")]
pub mod pulse_v2;

/// Authenticated hybrid encryption (seal-then-sign), composing `hybrid_kem` +
/// `hybrid_sig`. The one-call API for clients. Feature `hybrid-rc`.
#[cfg(feature = "hybrid-rc")]
pub mod sealed_box;

/// Encrypt/decrypt a plaintext `String` DHT field with zero schema change
/// (marked + bs58-encoded ciphertext). Feature `hybrid-rc`.
#[cfg(feature = "hybrid-rc")]
pub mod field_crypto;

pub use algorithm::AlgorithmId;
pub use envelope::{EncryptedEnvelope, TaggedPublicKey, TaggedSignature};
pub use error::CryptoError;

#[cfg(feature = "native")]
pub use traits::{Encryptor, KeyEncapsulator, Signer, Verifier};
