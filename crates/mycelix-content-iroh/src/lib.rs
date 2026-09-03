// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Iroh transport adapter for Mycelix Content Fabric.
//!
//! CF-04 deliberately treats Iroh as transport only. Global content identity
//! remains `ContentDigestV1`, and v1 transfers complete blobs so every client
//! can verify the same whole-object digest regardless of digest algorithm.

mod client;
mod error;
mod provider;
mod wire;

pub use client::{ContentClientConfigV1, ContentClientV1, VerifiedDownloadV1};
pub use error::{TransportErrorV1, WireErrorV1};
pub use provider::{
    AllowAllReadsV1, ContentProviderConfigV1, ContentProviderV1, DenyAllReadsV1,
    ReadAuthorizerV1,
};
pub use wire::{
    ContentRequestV1, ContentResponseHeaderV1, ContentResponseStatusV1, CONTENT_ALPN_V1,
    REQUEST_LEN_V1, RESPONSE_HEADER_LEN_V1,
};
