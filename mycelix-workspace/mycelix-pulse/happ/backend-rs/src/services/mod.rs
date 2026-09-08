// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Backend services

pub mod ai;
pub mod bridge;
pub mod claims;
pub mod crypto;
pub mod holochain;
pub mod persistence;
pub mod storage;
pub mod trust_cache;
pub mod trust_graph;

pub use ai::LocalAIService;
pub use bridge::{BridgeClient, CrossHappReputation, CrossHappIdentity, HappReputationScore, HappId};
pub use claims::ClaimsService;
pub use crypto::{encrypt_for_recipient, decrypt_envelope, EncryptedEnvelope, KeyPair};
pub use holochain::{
    HolochainService, ConnectionInfo,
    BridgeRecordReputationInput, BridgeReputationRecord, BridgeAggregateReputation, BridgeHappScore,
};
pub use persistence::PersistenceService;
pub use storage::{StorageService, StoredContent, store_encrypted, retrieve_decrypted};
pub use trust_cache::TrustCacheService;
pub use trust_graph::TrustGraphService;
