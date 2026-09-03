// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Stock Nix binary-cache compatibility over verified Content Fabric CAS bytes.
//!
//! This crate is deliberately a delivery facade, not a software trust root.
//! It exposes only catalog-admitted raw NARs, preserves supplied Nix signatures,
//! never holds a cache signing key, and verifies the exact SHA-256 NAR bytes
//! through CF-03 before serving either `.narinfo` metadata or the NAR itself.

mod model;
mod nixbase32;
mod server;

pub use model::{
    NixCacheCatalogV1, NixCacheEntryV1, NixCacheErrorV1, NixContentAddressV1,
    NixSignatureV1, NixStoreHashV1, NixStorePathBaseNameV1, NixStorePathV1,
    NIX_STORE_DIR_V1,
};
pub use nixbase32::encode_nix_base32;
pub use server::{router, serve, NixCacheConfigV1};
