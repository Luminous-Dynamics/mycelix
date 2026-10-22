// Copyright (C) 2026 Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Isolated tests for the actual landscape policy-gate source modules.
//! This harness avoids compiling unrelated Mycelix core code so a missing
//! workspace source file elsewhere cannot mask these policy-gate tests.

#[rustfmt::skip]
#[path = "../../src/core/src/landscape_stewardship.rs"]
mod landscape_stewardship;

#[rustfmt::skip]
#[path = "../../src/core/src/landscape_measurement_authorization.rs"]
mod landscape_measurement_authorization;
