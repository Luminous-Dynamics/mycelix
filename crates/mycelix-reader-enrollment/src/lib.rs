// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Explicit hybrid-key reader enrollment for Content Fabric.
//!
//! This crate is deliberately **not an authenticator**. It defines a stable,
//! canonical policy mapping from an exact Xenia-style Ed25519 + ML-DSA-65
//! public-key pair to a Mycelix `PartyIdV1` and zero or more groups.
//!
//! A later live transport adapter must independently prove that the peer
//! authenticated the same hybrid key pair and that the request belongs to the
//! same live connection generation before using an enrollment as reader facts.

mod error;
mod model;
mod reconstruct;

pub use error::ReaderEnrollmentErrorV1;
pub use model::{
    ReaderEnrollmentRegistryV1, ReaderEnrollmentV1, XENIA_ED25519_PUBLIC_KEY_LEN_V1,
    XENIA_ML_DSA_65_PUBLIC_KEY_LEN_V1, XeniaHybridReaderCredentialV1,
};
pub use reconstruct::reconstruct_reader_enrollment_registry_v1;
