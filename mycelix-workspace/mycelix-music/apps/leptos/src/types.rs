// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Mirror types from the music DNA zomes.
//! These must be serde-compatible with the Holochain entry types.

use serde::{Deserialize, Serialize};

mod binary_bytes {
    use serde::de::{SeqAccess, Visitor};
    use serde::{Deserializer, Serializer};
    use std::fmt;

    pub fn serialize<S>(bytes: &[u8], serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        serializer.serialize_bytes(bytes)
    }

    pub fn deserialize<'de, D>(deserializer: D) -> Result<Vec<u8>, D::Error>
    where
        D: Deserializer<'de>,
    {
        struct BytesVisitor;

        impl<'de> Visitor<'de> for BytesVisitor {
            type Value = Vec<u8>;

            fn expecting(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
                formatter.write_str("a MessagePack binary value")
            }

            fn visit_bytes<E>(self, value: &[u8]) -> Result<Self::Value, E> {
                Ok(value.to_vec())
            }

            fn visit_byte_buf<E>(self, value: Vec<u8>) -> Result<Self::Value, E> {
                Ok(value)
            }

            fn visit_seq<A>(self, mut sequence: A) -> Result<Self::Value, A::Error>
            where
                A: SeqAccess<'de>,
            {
                let mut bytes = Vec::with_capacity(sequence.size_hint().unwrap_or(0));
                while let Some(byte) = sequence.next_element()? {
                    bytes.push(byte);
                }
                Ok(bytes)
            }
        }

        deserializer.deserialize_any(BytesVisitor)
    }
}

/// Binary AgentPubKey representation used by the Holochain wire protocol.
#[derive(Clone, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(transparent)]
pub struct AgentPubKey(#[serde(with = "binary_bytes")] pub Vec<u8>);

/// Binary ActionHash representation used by the Holochain wire protocol.
#[derive(Clone, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(transparent)]
pub struct ActionHash(#[serde(with = "binary_bytes")] pub Vec<u8>);

/// Microseconds since the Unix epoch, matching Holochain's Timestamp newtype.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(transparent)]
pub struct Timestamp(pub i64);

// --- Catalog ---

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Song {
    pub song_hash: String,
    pub title: String,
    pub artist: AgentPubKey,
    pub ipfs_cid: String,
    pub cover_cid: Option<String>,
    pub duration_seconds: u32,
    pub genres: Vec<String>,
    pub strategy_id: String,
    pub released_at: Timestamp,
    pub metadata: String,
}

impl Song {
    /// Construct an IPFS gateway URL for the audio file.
    pub fn audio_url(&self) -> String {
        format!("https://ipfs.io/ipfs/{}", self.ipfs_cid)
    }

    /// Construct an IPFS gateway URL for the cover art.
    pub fn cover_url(&self) -> Option<String> {
        self.cover_cid
            .as_ref()
            .map(|cid| format!("https://ipfs.io/ipfs/{}", cid))
    }

    /// Format duration as mm:ss.
    pub fn duration_display(&self) -> String {
        let mins = self.duration_seconds / 60;
        let secs = self.duration_seconds % 60;
        format!("{mins}:{secs:02}")
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Album {
    pub title: String,
    pub artist: AgentPubKey,
    pub cover_cid: String,
    pub released_at: Timestamp,
    pub song_hashes: Vec<ActionHash>,
    pub metadata: String,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ArtistProfile {
    pub name: String,
    pub bio: String,
    pub avatar_cid: Option<String>,
    pub payment_address: String,
    pub social_links: String,
    pub verified: bool,
}

// --- Plays ---

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct RecordPlayInput {
    pub song_hash: ActionHash,
    pub duration_listened: u32,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct PlayRecord {
    pub song_hash: ActionHash,
    pub artist: AgentPubKey,
    pub played_at: Timestamp,
    pub duration_listened: u32,
    pub song_duration: u32,
    pub strategy_id: String,
    pub amount_owed: u64,
    pub settled: bool,
    pub settlement_hash: Option<ActionHash>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SongStats {
    pub total_plays: u64,
    pub total_earnings: u64,
    pub unique_listeners: u64,
    pub avg_completion: f64,
}

// --- Balances ---

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ListenerAccount {
    pub owner: AgentPubKey,
    pub eth_address: String,
    pub balance: u64,
    pub total_deposited: u64,
    pub total_spent: u64,
    pub created_at: Timestamp,
    pub updated_at: Timestamp,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ArtistAccount {
    pub owner: AgentPubKey,
    pub eth_address: String,
    pub pending_balance: u64,
    pub total_earned: u64,
    pub total_cashed_out: u64,
    pub created_at: Timestamp,
    pub updated_at: Timestamp,
}

// --- Trust ---

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct VerificationStatus {
    pub artist: AgentPubKey,
    pub trust_score: u32,
    pub tier: VerificationTier,
    pub vouch_count: u32,
    pub computed_at: Timestamp,
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub enum VerificationTier {
    Unverified,
    CommunityVerified,
    Trusted,
    PlatformVerified,
    FoundingArtist,
}

impl VerificationTier {
    pub fn label(&self) -> &'static str {
        match self {
            Self::Unverified => "Unverified",
            Self::CommunityVerified => "Community Verified",
            Self::Trusted => "Trusted",
            Self::PlatformVerified => "Platform Verified",
            Self::FoundingArtist => "Founding Artist",
        }
    }

    pub fn color_class(&self) -> &'static str {
        match self {
            Self::Unverified => "tier-unverified",
            Self::CommunityVerified => "tier-community",
            Self::Trusted => "tier-trusted",
            Self::PlatformVerified => "tier-platform",
            Self::FoundingArtist => "tier-founding",
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub enum RepeatMode {
    None,
    One,
    All,
}
impl RepeatMode {
    pub fn icon(&self) -> &'static str {
        match self {
            Self::None => "\u{1f501}",
            Self::One => "\u{1f502}",
            Self::All => "\u{1f501}",
        }
    }
    pub fn next(&self) -> Self {
        match self {
            Self::None => Self::All,
            Self::All => Self::One,
            Self::One => Self::None,
        }
    }
}

fn fixture_agent(seed: u8) -> AgentPubKey {
    let mut bytes = vec![132, 32, 36];
    bytes.extend(std::iter::repeat_n(seed, 32));
    bytes.extend([0, 0, 0, seed]);
    AgentPubKey(bytes)
}

// --- Explicit development fixtures (`--features fixtures`) ---

pub fn mock_songs() -> Vec<Song> {
    vec![
        Song {
            song_hash: "mock-1".into(),
            title: "Decentralized Dreams".into(),
            artist: fixture_agent(1),
            ipfs_cid: "QmDemo1".into(),
            cover_cid: None,
            duration_seconds: 234,
            genres: vec!["Electronic".into(), "Ambient".into()],
            strategy_id: "pay_per_stream".into(),
            released_at: Timestamp(0),
            metadata: "{}".into(),
        },
        Song {
            song_hash: "mock-2".into(),
            title: "Zero-Cost Serenade".into(),
            artist: fixture_agent(2),
            ipfs_cid: "QmDemo2".into(),
            cover_cid: None,
            duration_seconds: 187,
            genres: vec!["Indie".into(), "Folk".into()],
            strategy_id: "gift".into(),
            released_at: Timestamp(0),
            metadata: "{}".into(),
        },
        Song {
            song_hash: "mock-3".into(),
            title: "Mycelium Network".into(),
            artist: fixture_agent(1),
            ipfs_cid: "QmDemo3".into(),
            cover_cid: None,
            duration_seconds: 312,
            genres: vec!["Rock".into()],
            strategy_id: "patronage".into(),
            released_at: Timestamp(0),
            metadata: "{}".into(),
        },
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn holo_hashes_use_messagepack_binary_encoding() {
        let agent = AgentPubKey(vec![7; 39]);
        let encoded = rmp_serde::to_vec_named(&agent).unwrap();
        assert_eq!(encoded[0], 0xc4); // bin8
        assert_eq!(encoded[1], 39);

        let decoded: AgentPubKey = rmp_serde::from_slice(&encoded).unwrap();
        assert_eq!(decoded, agent);
    }

    #[test]
    fn play_input_roundtrips_with_typed_hashes() {
        let input = RecordPlayInput {
            song_hash: ActionHash(vec![1; 39]),
            duration_listened: 90,
        };

        let encoded = rmp_serde::to_vec_named(&input).unwrap();
        let decoded: RecordPlayInput = rmp_serde::from_slice(&encoded).unwrap();
        assert_eq!(decoded.song_hash, input.song_hash);
        assert_eq!(decoded.duration_listened, 90);
    }
}
