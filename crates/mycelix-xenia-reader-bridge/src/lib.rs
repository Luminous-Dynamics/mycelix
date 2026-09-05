// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! CF-07C2 through CF-07C6 authenticated Content Fabric reader chain.
//!
//! The crate binds Xenia authenticated application-channel evidence to explicit
//! Mycelix reader enrollment, parses one bounded resource request, evaluates
//! current remote-exposure authority, prepares only the exact authorized bytes
//! through CF-03 verified CAS, carries server responses over the exact matching
//! authenticated Xenia generation, and validates the reciprocal serial response
//! stream under one deadline-owned client round trip.

mod bind;
mod delivery;
mod error;
mod request;
mod response;
mod response_receive;
mod serve;

pub use bind::{
    CONTENT_FABRIC_READER_PAYLOAD_TYPE_V1, EnrollmentBoundOpenedPayloadV1,
    bind_xenia_opened_reader_v1,
};
pub use delivery::{
    XeniaNixResponseDeliveryErrorV1, send_authorized_nix_read_over_xenia_v1,
};
pub use error::XeniaReaderBridgeErrorV1;
pub use request::{
    AuthorizedNixReadV1, CONTENT_FABRIC_READER_REQUEST_LEN_V1,
    CONTENT_FABRIC_READER_REQUEST_SCHEMA_V1, NixReaderOperationV1,
    NixReaderRequestErrorV1, authorize_bound_nix_read_v1, encode_nix_reader_request_v1,
};
pub use response::{
    CONTENT_FABRIC_RESPONSE_DATA_MAX_V1, CONTENT_FABRIC_RESPONSE_FRAME_HEADER_LEN_V1,
    CONTENT_FABRIC_RESPONSE_FRAME_SCHEMA_V1, XeniaNixResponseStreamErrorV1,
};
pub use response_receive::{
    CompletedXeniaNixResponseV1, MAX_NARINFO_RESPONSE_BYTES_V1,
    ReceivedXeniaNixResponseChunkV1, XeniaNixResponseEventV1,
    XeniaNixResponseReceiveErrorV1, XeniaNixResponseReceiverV1,
    send_nix_nar_request_over_xenia_v1, send_nix_narinfo_request_over_xenia_v1,
};
pub use serve::{
    AuthorizedNarInfoV1, AuthorizedNarReaderV1, PreparedNixReadAuditV1,
    PreparedNixReadV1, VerifiedReadServeErrorV1, prepare_authorized_nix_read_v1,
};
