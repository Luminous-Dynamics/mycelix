// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! CF-07C2/CF-07C3/CF-07C4: bind Xenia authenticated application-channel
//! evidence to an exact pinned Mycelix reader enrollment, consume that sealed
//! binding through one bounded Nix request and current CF-07A serving snapshot,
//! then prepare only the exact authorized representation through CF-03 verified
//! local CAS reads.
//!
//! This crate does not authenticate peers itself. Its production identity API
//! accepts only Xenia's sealed `OpenedPeerApplicationPayloadV1`, maps the exact
//! authenticated Ed25519 + ML-DSA-65 pair through CF-07C1, keeps the resulting
//! `RemoteReaderV1` non-public through request authorization, and never accepts
//! a second arbitrary digest/path after `AuthorizedNixReadV1` exists.

mod bind;
mod error;
mod request;
mod serve;

pub use bind::{
    CONTENT_FABRIC_READER_PAYLOAD_TYPE_V1, EnrollmentBoundOpenedPayloadV1,
    bind_xenia_opened_reader_v1,
};
pub use error::XeniaReaderBridgeErrorV1;
pub use request::{
    AuthorizedNixReadV1, CONTENT_FABRIC_READER_REQUEST_LEN_V1,
    CONTENT_FABRIC_READER_REQUEST_SCHEMA_V1, NixReaderOperationV1,
    NixReaderRequestErrorV1, authorize_bound_nix_read_v1, encode_nix_reader_request_v1,
};
pub use serve::{
    AuthorizedNarInfoV1, AuthorizedNarReaderV1, PreparedNixReadAuditV1,
    PreparedNixReadV1, VerifiedReadServeErrorV1, prepare_authorized_nix_read_v1,
};
