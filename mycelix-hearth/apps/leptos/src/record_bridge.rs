// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Record-to-View translation layer.
//!
//! Holochain zome calls return lightweight Record wire values. This module
//! decodes the application entries without pulling HDI/HDK into the browser.

use hearth_leptos_types::*;
use serde::{Deserialize, Serialize};

// ============================================================================
// Lightweight Record wire types (mirror of Holochain's Record, no hdk dep)
// ============================================================================

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WireRecord {
    pub signed_action: WireSignedAction,
    pub entry: WireRecordEntry,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WireSignedAction {
    pub hashed: WireHashedAction,
    #[serde(default)]
    pub signature: serde_json::Value,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WireHashedAction {
    pub hash: Vec<u8>,
    pub content: serde_json::Value,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(untagged)]
pub enum WireRecordEntry {
    Present {
        #[serde(rename = "Present")]
        present: WireEntry,
    },
    Other(serde_json::Value),
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WireEntry {
    pub entry_type: serde_json::Value,
    pub entry: Vec<u8>,
}

impl WireRecord {
    pub fn action_hash_hex(&self) -> String {
        hex_encode(self.hashed_hash())
    }

    pub fn action_hash_b64(&self) -> String {
        base64_encode(self.hashed_hash())
    }

    fn hashed_hash(&self) -> &[u8] {
        &self.signed_action.hashed.hash
    }

    pub fn author_b64(&self) -> Option<String> {
        self.signed_action
            .hashed
            .content
            .get("author")
            .and_then(|value| value.as_array())
            .map(|bytes| {
                let raw: Vec<u8> = bytes
                    .iter()
                    .filter_map(|byte| byte.as_u64().map(|number| number as u8))
                    .collect();
                base64_encode(&raw)
            })
    }

    pub fn timestamp_micros(&self) -> Option<i64> {
        self.signed_action
            .hashed
            .content
            .get("timestamp")
            .and_then(|value| value.as_i64())
    }

    pub fn decode_entry<T: serde::de::DeserializeOwned>(&self) -> Option<T> {
        match &self.entry {
            WireRecordEntry::Present { present } => rmp_serde::from_slice(&present.entry).ok(),
            _ => None,
        }
    }
}

// ============================================================================
// Domain-specific conversions: Record → View
// ============================================================================

#[derive(Debug, Clone, Deserialize)]
pub struct WireHearth {
    pub name: String,
    pub description: String,
    pub hearth_type: HearthType,
    pub created_by: Vec<u8>,
    pub created_at: i64,
    pub max_members: u32,
}

#[derive(Debug, Clone, Deserialize)]
pub struct WireMembership {
    pub hearth_hash: Vec<u8>,
    pub agent: Vec<u8>,
    pub role: MemberRole,
    pub status: MembershipStatus,
    pub display_name: String,
    pub joined_at: i64,
}

#[derive(Debug, Clone, Deserialize)]
pub struct WireBond {
    pub hearth_hash: Vec<u8>,
    pub member_a: Vec<u8>,
    pub member_b: Vec<u8>,
    pub bond_type: BondType,
    pub strength_bp: u32,
    pub last_tended: i64,
    pub created_at: i64,
}

#[derive(Debug, Clone, Deserialize)]
pub struct WireGratitude {
    pub hearth_hash: Vec<u8>,
    pub from_agent: Vec<u8>,
    pub to_agent: Vec<u8>,
    pub message: String,
    pub gratitude_type: GratitudeType,
    pub visibility: HearthVisibility,
    pub created_at: i64,
}

pub fn records_to_hearths(records: &[WireRecord]) -> Vec<HearthView> {
    records
        .iter()
        .filter_map(|record| {
            let hearth: WireHearth = record.decode_entry()?;
            Some(HearthView {
                hash: record.action_hash_b64(),
                name: hearth.name,
                description: hearth.description,
                hearth_type: hearth.hearth_type,
                created_by: base64_encode(&hearth.created_by),
                created_at: hearth.created_at / 1_000_000,
                max_members: hearth.max_members,
            })
        })
        .collect()
}

pub fn records_to_members(records: &[WireRecord]) -> Vec<MemberView> {
    records
        .iter()
        .filter_map(|record| {
            let membership: WireMembership = record.decode_entry()?;
            Some(MemberView {
                agent: base64_encode(&membership.agent),
                display_name: membership.display_name,
                role: membership.role,
                status: membership.status,
                joined_at: membership.joined_at / 1_000_000,
            })
        })
        .collect()
}

pub fn records_to_bonds(records: &[WireRecord]) -> Vec<BondView> {
    records
        .iter()
        .filter_map(|record| {
            let bond: WireBond = record.decode_entry()?;
            Some(BondView {
                hash: record.action_hash_b64(),
                member_a: base64_encode(&bond.member_a),
                member_b: base64_encode(&bond.member_b),
                bond_type: bond.bond_type,
                strength_bp: bond.strength_bp,
                last_tended: bond.last_tended / 1_000_000,
                created_at: bond.created_at / 1_000_000,
            })
        })
        .collect()
}

pub fn records_to_gratitude(records: &[WireRecord]) -> Vec<GratitudeExpressionView> {
    records
        .iter()
        .filter_map(|record| {
            let gratitude: WireGratitude = record.decode_entry()?;
            Some(GratitudeExpressionView {
                hash: record.action_hash_b64(),
                from_agent: base64_encode(&gratitude.from_agent),
                to_agent: base64_encode(&gratitude.to_agent),
                message: gratitude.message,
                gratitude_type: gratitude.gratitude_type,
                visibility: gratitude.visibility,
                created_at: gratitude.created_at / 1_000_000,
            })
        })
        .collect()
}

// ============================================================================
// Helpers
// ============================================================================

fn base64_encode(bytes: &[u8]) -> String {
    const CHARS: &[u8] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    let mut result = String::with_capacity(bytes.len() * 4 / 3 + 4);
    for chunk in bytes.chunks(3) {
        let b0 = chunk[0] as u32;
        let b1 = chunk.get(1).copied().unwrap_or(0) as u32;
        let b2 = chunk.get(2).copied().unwrap_or(0) as u32;
        let number = (b0 << 16) | (b1 << 8) | b2;
        result.push(CHARS[((number >> 18) & 0x3F) as usize] as char);
        result.push(CHARS[((number >> 12) & 0x3F) as usize] as char);
        if chunk.len() > 1 {
            result.push(CHARS[((number >> 6) & 0x3F) as usize] as char);
        } else {
            result.push('=');
        }
        if chunk.len() > 2 {
            result.push(CHARS[(number & 0x3F) as usize] as char);
        } else {
            result.push('=');
        }
    }
    result
}

fn hex_encode(bytes: &[u8]) -> String {
    bytes.iter().map(|byte| format!("{byte:02x}")).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn base64_encodes_correctly() {
        assert_eq!(base64_encode(b"hello"), "aGVsbG8=");
        assert_eq!(base64_encode(b""), "");
        assert_eq!(base64_encode(b"a"), "YQ==");
    }
}
