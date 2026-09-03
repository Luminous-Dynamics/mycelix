// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! External planner protocol for Content Fabric.
//!
//! JSON is an interoperability encoding only. Stable request/acceptance IDs use
//! explicit domain-separated field framing and never hash arbitrary JSON bytes.

mod model;
mod protocol;

pub use model::*;
pub use protocol::{
    accept_external_recommendation_v1, build_external_planner_request_v1,
    decode_recommendation_json_v1, decode_request_json_v1, encode_recommendation_json_v1,
    encode_request_json_v1,
};
