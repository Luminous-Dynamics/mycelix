// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Record-to-View translation layer.
//!
//! Holochain zome calls return `Vec<Record>` values. This module keeps HDK/HDI
//! types out of the browser build by decoding the entry bytes into small
//! WASM-safe mirror structs, then translating those into Hearth view types.
//! A caller can compare the source record count with the number of successfully
//! decoded records to distinguish complete, empty, and degraded snapshots.

use hearth_leptos_types::*;
use mycelix_leptos_client::{HoloHashBytes, HoloHashKind};
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;

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

    /// Internal reversible carrier for an ActionHash's exact 39 wire bytes.
    pub fn action_hash_b64(&self) -> String {
        base64_encode(self.hashed_hash())
    }

    fn hashed_hash(&self) -> &[u8] {
        &self.signed_action.hashed.hash
    }

    /// Action author in Holochain's canonical `u...` AgentPubKey display form.
    pub fn author_display(&self) -> Option<String> {
        self.signed_action
            .hashed
            .content
            .get("author")
            .and_then(|value| value.as_array())
            .and_then(|bytes| {
                let raw = bytes
                    .iter()
                    .filter_map(|byte| byte.as_u64().map(|value| value as u8))
                    .collect::<Vec<_>>();
                agent_display(&raw)
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
            WireRecordEntry::Other(_) => None,
        }
    }
}

// ============================================================================
// Domain wire mirrors
// ============================================================================

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

#[derive(Debug, Clone, Deserialize)]
pub struct WireCareSchedule {
    pub hearth_hash: Vec<u8>,
    pub care_type: CareType,
    pub title: String,
    pub description: String,
    pub assigned_to: Vec<u8>,
    pub recurrence: Recurrence,
    pub notes: String,
    pub status: CareScheduleStatus,
    pub completed_at: Option<i64>,
}

#[derive(Debug, Clone, Deserialize)]
pub struct WireDecision {
    pub hearth_hash: Vec<u8>,
    pub title: String,
    pub description: String,
    pub decision_type: DecisionType,
    pub eligible_roles: Vec<MemberRole>,
    pub options: Vec<String>,
    pub deadline: i64,
    pub quorum_bp: Option<u32>,
    pub status: DecisionStatus,
    pub created_by: Vec<u8>,
    pub created_at: i64,
}

#[derive(Debug, Clone, Deserialize)]
pub struct WireVote {
    pub decision_hash: Vec<u8>,
    pub voter: Vec<u8>,
    pub choice: u32,
    pub weight_bp: u32,
    pub reasoning: Option<String>,
    pub created_at: i64,
}

#[derive(Debug, Clone, Deserialize)]
pub struct WireRhythm {
    pub hearth_hash: Vec<u8>,
    pub name: String,
    pub rhythm_type: RhythmType,
    pub schedule: String,
    pub participants: Vec<Vec<u8>>,
    pub description: String,
    pub created_at: i64,
}

#[derive(Debug, Clone, Deserialize)]
pub struct WirePresence {
    pub hearth_hash: Vec<u8>,
    pub agent: Vec<u8>,
    pub status: PresenceStatusType,
    pub expected_return: Option<i64>,
    pub updated_at: i64,
}

// ============================================================================
// Record -> view conversions
// ============================================================================

pub fn records_to_members(records: &[WireRecord]) -> Vec<MemberView> {
    records
        .iter()
        .filter_map(|record| {
            let member: WireMembership = record.decode_entry()?;
            Some(MemberView {
                agent: agent_display(&member.agent)?,
                display_name: member.display_name,
                role: member.role,
                status: member.status,
                joined_at: member.joined_at / 1_000_000,
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
                member_a: agent_display(&bond.member_a)?,
                member_b: agent_display(&bond.member_b)?,
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
                from_agent: agent_display(&gratitude.from_agent)?,
                to_agent: agent_display(&gratitude.to_agent)?,
                message: gratitude.message,
                gratitude_type: gratitude.gratitude_type,
                visibility: gratitude.visibility,
                created_at: gratitude.created_at / 1_000_000,
            })
        })
        .collect()
}

pub fn records_to_care_schedules(records: &[WireRecord]) -> Vec<CareScheduleView> {
    records
        .iter()
        .filter_map(|record| {
            let schedule: WireCareSchedule = record.decode_entry()?;
            Some(CareScheduleView {
                hash: record.action_hash_b64(),
                hearth_hash: base64_encode(&schedule.hearth_hash),
                care_type: schedule.care_type,
                title: schedule.title,
                description: schedule.description,
                assigned_to: agent_display(&schedule.assigned_to)?,
                recurrence: schedule.recurrence,
                status: schedule.status,
                completed_at: schedule.completed_at.map(|value| value / 1_000_000),
            })
        })
        .collect()
}

pub fn records_to_decisions(records: &[WireRecord]) -> Vec<DecisionView> {
    records
        .iter()
        .filter_map(|record| {
            let decision: WireDecision = record.decode_entry()?;
            Some(DecisionView {
                hash: record.action_hash_b64(),
                hearth_hash: base64_encode(&decision.hearth_hash),
                title: decision.title,
                description: decision.description,
                decision_type: decision.decision_type,
                eligible_roles: decision.eligible_roles,
                options: decision.options,
                deadline: decision.deadline / 1_000_000,
                quorum_bp: decision.quorum_bp,
                status: decision.status,
                created_by: agent_display(&decision.created_by)?,
                created_at: decision.created_at / 1_000_000,
            })
        })
        .collect()
}

pub fn records_to_votes(records: &[WireRecord]) -> Vec<VoteView> {
    records
        .iter()
        .filter_map(|record| {
            let vote: WireVote = record.decode_entry()?;
            Some(VoteView {
                decision_hash: base64_encode(&vote.decision_hash),
                voter: agent_display(&vote.voter)?,
                choice: vote.choice,
                weight_bp: vote.weight_bp,
                reasoning: vote.reasoning,
                created_at: vote.created_at / 1_000_000,
            })
        })
        .collect()
}

pub fn records_to_rhythms(records: &[WireRecord]) -> Vec<RhythmView> {
    records
        .iter()
        .filter_map(|record| {
            let rhythm: WireRhythm = record.decode_entry()?;
            let participants = rhythm
                .participants
                .iter()
                .map(|agent| agent_display(agent))
                .collect::<Option<Vec<_>>>()?;
            Some(RhythmView {
                hash: record.action_hash_b64(),
                hearth_hash: base64_encode(&rhythm.hearth_hash),
                name: rhythm.name,
                rhythm_type: rhythm.rhythm_type,
                description: rhythm.description,
                participants,
            })
        })
        .collect()
}

/// Presence zome reads can contain multiple historical status records per
/// agent. `decoded_records` counts every successfully decoded source record;
/// `views` contains one deterministic latest status per agent.
#[derive(Debug, Clone)]
pub struct PresenceDecode {
    pub views: Vec<PresenceView>,
    pub decoded_records: usize,
}

pub fn records_to_presence(records: &[WireRecord]) -> PresenceDecode {
    let mut decoded_records = 0usize;
    let mut latest = BTreeMap::<String, (i64, String, PresenceView)>::new();

    for record in records {
        let Some(presence) = record.decode_entry::<WirePresence>() else {
            continue;
        };
        let Some(agent) = agent_display(&presence.agent) else {
            continue;
        };
        decoded_records += 1;

        let action_hash = record.action_hash_b64();
        let view = PresenceView {
            agent: agent.clone(),
            status: presence.status,
            expected_return: presence.expected_return.map(|value| value / 1_000_000),
            updated_at: presence.updated_at / 1_000_000,
        };

        let replace = match latest.get(&agent) {
            Some((current_updated_at, current_hash, _)) => {
                presence.updated_at > *current_updated_at
                    || (presence.updated_at == *current_updated_at
                        && action_hash > *current_hash)
            }
            None => true,
        };

        if replace {
            latest.insert(agent, (presence.updated_at, action_hash, view));
        }
    }

    PresenceDecode {
        views: latest.into_values().map(|(_, _, view)| view).collect(),
        decoded_records,
    }
}

// ============================================================================
// Helpers
// ============================================================================

pub(crate) fn agent_display(bytes: &[u8]) -> Option<String> {
    HoloHashBytes::from_raw_39(bytes.to_vec())
        .ok()?
        .require_kind(HoloHashKind::Agent)
        .ok()
        .map(|hash| hash.to_holochain_display())
}

fn base64_encode(bytes: &[u8]) -> String {
    const CHARS: &[u8] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    let mut result = String::with_capacity(bytes.len() * 4 / 3 + 4);
    for chunk in bytes.chunks(3) {
        let b0 = chunk[0] as u32;
        let b1 = chunk.get(1).copied().unwrap_or(0) as u32;
        let b2 = chunk.get(2).copied().unwrap_or(0) as u32;
        let n = (b0 << 16) | (b1 << 8) | b2;
        result.push(CHARS[((n >> 18) & 0x3f) as usize] as char);
        result.push(CHARS[((n >> 12) & 0x3f) as usize] as char);
        if chunk.len() > 1 {
            result.push(CHARS[((n >> 6) & 0x3f) as usize] as char);
        } else {
            result.push('=');
        }
        if chunk.len() > 2 {
            result.push(CHARS[(n & 0x3f) as usize] as char);
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
    use super::{agent_display, base64_encode};
    use mycelix_leptos_client::{HoloHashBytes, HoloHashKind};

    #[test]
    fn base64_encodes_correctly() {
        assert_eq!(base64_encode(b"hello"), "aGVsbG8=");
        assert_eq!(base64_encode(b""), "");
        assert_eq!(base64_encode(b"a"), "YQ==");
    }

    #[test]
    fn agent_display_requires_agent_prefix() {
        assert!(agent_display(&[0u8; 38]).is_none());

        let mut agent = vec![0u8; 39];
        agent[..3].copy_from_slice(&HoloHashKind::Agent.prefix());
        let display = agent_display(&agent).unwrap();
        assert!(display.starts_with("uhCAk"));

        let mut action = vec![0u8; 39];
        action[..3].copy_from_slice(&HoloHashKind::Action.prefix());
        assert!(agent_display(&action).is_none());
        assert!(HoloHashBytes::from_raw_39(action).is_ok());
    }
}
