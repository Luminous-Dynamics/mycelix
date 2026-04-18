// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Holochain Integration for Federated Learning
//!
//! Provides WebSocket-based client for Holochain conductor operations:
//! - Immutable gradient audit trail
//! - Credit system for node incentives
//! - Reputation tracking
//! - Byzantine event logging
//!
//! ## Quick Start
//!
//! ```rust,ignore
//! use fl_aggregator::holochain::{HolochainClient, HolochainConfig};
//!
//! let config = HolochainConfig::default();
//! let mut client = HolochainClient::new(config);
//!
//! // Connect to conductor
//! client.connect().await?;
//!
//! // Store gradient
//! let hash = client.store_gradient(gradient_record).await?;
//!
//! // Issue credits
//! client.issue_credit("node_1", 10, CreditReason::GradientQuality).await?;
//! ```

pub mod client;
pub mod types;
pub mod listener;
pub mod credits;

pub use client::{HolochainClient, HolochainConfig};
pub use types::*;
pub use listener::{HolochainListener, ListenerBuilder, SignalStats};
pub use credits::{CreditManager, CreditReason};
