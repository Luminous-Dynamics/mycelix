// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! CF-07C2: bind Xenia authenticated application-channel evidence to an exact
//! pinned Mycelix reader-enrollment snapshot.
//!
//! This crate does not authenticate peers itself and does not parse Content
//! Fabric request semantics. Its public binding function accepts only Xenia's
//! sealed `OpenedPeerApplicationPayloadV1`, then maps the exact authenticated
//! Ed25519 + ML-DSA-65 pair through CF-07C1 before constructing CF-07A
//! `RemoteReaderV1` facts.

mod bind;
mod error;

pub use bind::{
    CONTENT_FABRIC_READER_PAYLOAD_TYPE_V1, EnrollmentBoundOpenedPayloadV1,
    bind_xenia_opened_reader_v1,
};
pub use error::XeniaReaderBridgeErrorV1;
