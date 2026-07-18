// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Plays Integrity Zome
//!
//! Defines validation rules for play records.
//! CRITICAL: Plays are recorded on the listener's source chain - ZERO COST.
//! Only aggregated settlements touch the blockchain.

use catalog_integrity::Song;
use hdi::prelude::*;
use std::collections::HashSet;

/// Play record - stored on listener's source chain (FREE!)
/// This is the magic of Holochain - each play is just a local entry.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct PlayRecord {
    /// Song action hash (reference to catalog)
    pub song_hash: ActionHash,
    /// Artist's agent public key (for aggregation)
    pub artist: AgentPubKey,
    /// Timestamp of the play
    pub played_at: Timestamp,
    /// Duration listened (seconds) - for partial play tracking
    pub duration_listened: u32,
    /// Total song duration (for completion percentage)
    pub song_duration: u32,
    /// Strategy that was active at play time
    pub strategy_id: String,
    /// Calculated micro-payment amount (in wei equivalent)
    pub amount_owed: u64,
    /// Whether this play has been included in a settlement batch
    pub settled: bool,
    /// Settlement batch hash (if settled)
    pub settlement_hash: Option<ActionHash>,
}

/// Play attestation authored by the same agent that authored the play.
/// The Holochain action signature is the attestation signature.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct PlayAttestation {
    /// The play record this attests to
    pub play_hash: ActionHash,
    /// Song hash for quick lookup
    pub song_hash: ActionHash,
    /// Artist who should receive payment
    pub artist: AgentPubKey,
    /// Amount owed for this play
    pub amount_owed: u64,
}

/// Settlement batch - aggregates many plays for efficient on-chain settlement
/// This is what actually touches the blockchain - batched for efficiency
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct SettlementBatch {
    /// Artist receiving payment
    pub artist: AgentPubKey,
    /// Total plays in this batch
    pub play_count: u64,
    /// Total amount to settle (in wei)
    pub total_amount: u64,
    /// Play record hashes included
    pub play_hashes: Vec<ActionHash>,
    /// Merkle root of play hashes (for efficient verification)
    pub merkle_root: Vec<u8>,
    /// When this batch was created
    pub created_at: Timestamp,
    /// On-chain settlement status
    pub status: SettlementStatus,
    /// Transaction hash if settled on-chain
    pub tx_hash: Option<String>,
}

/// Settlement status
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub enum SettlementStatus {
    /// Batch created, awaiting settlement
    Pending,
    /// Settlement submitted to blockchain
    Submitted,
    /// Settlement confirmed on-chain
    Confirmed,
    /// Settlement failed (will retry)
    Failed,
}

/// Link types for plays
#[hdk_link_types]
pub enum LinkTypes {
    /// Listener -> Their play records
    ListenerToPlays,
    /// Song -> Play records
    SongToPlays,
    /// Artist -> Settlement batches
    ArtistToSettlements,
    /// Play -> Settlement batch
    PlayToSettlement,
}

/// Entry types
#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    PlayRecord(PlayRecord),
    PlayAttestation(PlayAttestation),
    SettlementBatch(SettlementBatch),
}

/// Validation
#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(store_entry) => match store_entry {
            OpEntry::CreateEntry { app_entry, action } => match app_entry {
                EntryTypes::PlayRecord(play) => validate_create_play(play, action),
                EntryTypes::PlayAttestation(attestation) => {
                    validate_create_attestation(attestation, action)
                }
                EntryTypes::SettlementBatch(batch) => validate_create_settlement(batch, action),
            },
            OpEntry::UpdateEntry { .. } => Ok(ValidateCallbackResult::Invalid(
                "Play and settlement entries are immutable; lifecycle changes require a versioned settlement protocol"
                    .to_string(),
            )),
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterUpdate(_) => Ok(ValidateCallbackResult::Invalid(
            "Play and settlement entry updates are disabled".to_string(),
        )),
        FlatOp::RegisterDelete(_) => Ok(ValidateCallbackResult::Invalid(
            "Play and settlement entries cannot be deleted".to_string(),
        )),
        FlatOp::RegisterCreateLink {
            link_type,
            base_address,
            target_address,
            tag: _,
            action,
        } => validate_create_link(
            link_type,
            base_address,
            target_address,
            action,
        ),
        FlatOp::RegisterDeleteLink { .. } => Ok(ValidateCallbackResult::Invalid(
            "Play and settlement links cannot be deleted".to_string(),
        )),
        _ => Ok(ValidateCallbackResult::Valid),
    }
}

fn invalid(message: &str) -> ExternResult<ValidateCallbackResult> {
    Ok(ValidateCallbackResult::Invalid(message.to_string()))
}

fn action_hash(
    address: AnyLinkableHash,
    label: &str,
) -> Result<ActionHash, ValidateCallbackResult> {
    address
        .into_action_hash()
        .ok_or_else(|| ValidateCallbackResult::Invalid(format!("{label} must be an action hash")))
}

fn validate_create_link(
    link_type: LinkTypes,
    base_address: AnyLinkableHash,
    target_address: AnyLinkableHash,
    action: CreateLink,
) -> ExternResult<ValidateCallbackResult> {
    let is_song_index = matches!(&link_type, LinkTypes::SongToPlays);
    match link_type {
        LinkTypes::ListenerToPlays | LinkTypes::SongToPlays => {
            let target_hash = match action_hash(target_address, "Play link target") {
                Ok(hash) => hash,
                Err(result) => return Ok(result),
            };
            let record = must_get_valid_record(target_hash)?;
            if record.action().author() != &action.author {
                return invalid("Only a play author may index that play");
            }
            let Some(play) = record
                .entry()
                .to_app_option::<PlayRecord>()
                .map_err(|error| wasm_error!(WasmErrorInner::Guest(error.to_string())))?
            else {
                return invalid("Play index must target a PlayRecord");
            };
            if is_song_index {
                let base_hash = match action_hash(base_address, "Song link base") {
                    Ok(hash) => hash,
                    Err(result) => return Ok(result),
                };
                if base_hash != play.song_hash {
                    return invalid("Song index base must match the play's song");
                }
            }
            Ok(ValidateCallbackResult::Valid)
        }
        LinkTypes::ArtistToSettlements => {
            let target_hash = match action_hash(target_address, "Settlement link target") {
                Ok(hash) => hash,
                Err(result) => return Ok(result),
            };
            let record = must_get_valid_record(target_hash)?;
            if record.action().author() != &action.author {
                return invalid("Only a settlement author may index that settlement");
            }
            let batch = record
                .entry()
                .to_app_option::<SettlementBatch>()
                .map_err(|error| wasm_error!(WasmErrorInner::Guest(error.to_string())))?;
            if batch.is_none() {
                return invalid("Settlement index must target a SettlementBatch");
            }
            Ok(ValidateCallbackResult::Valid)
        }
        LinkTypes::PlayToSettlement => {
            let play_hash = match action_hash(base_address, "Settlement link base") {
                Ok(hash) => hash,
                Err(result) => return Ok(result),
            };
            let batch_hash = match action_hash(target_address, "Settlement link target") {
                Ok(hash) => hash,
                Err(result) => return Ok(result),
            };
            let play_record = must_get_valid_record(play_hash.clone())?;
            let batch_record = must_get_valid_record(batch_hash)?;
            if play_record.action().author() != &action.author
                || batch_record.action().author() != &action.author
            {
                return invalid("Only the play and settlement author may link them");
            }
            let Some(batch) = batch_record
                .entry()
                .to_app_option::<SettlementBatch>()
                .map_err(|error| wasm_error!(WasmErrorInner::Guest(error.to_string())))?
            else {
                return invalid("Settlement link must target a SettlementBatch");
            };
            if !batch.play_hashes.contains(&play_hash) {
                return invalid("Settlement link base must be included in the target batch");
            }
            Ok(ValidateCallbackResult::Valid)
        }
    }
}

/// Deterministic payment calculation shared with the coordinator.
pub fn calculate_play_amount(strategy_id: &str, duration_listened: u32, song_duration: u32) -> u64 {
    const BASE_RATE: u128 = 400_000_000_000_000;

    if song_duration == 0 || duration_listened > song_duration {
        return 0;
    }
    let qualifies =
        duration_listened >= 30 || u64::from(duration_listened) * 2 >= u64::from(song_duration);
    if !qualifies {
        return 0;
    }

    let pro_rata = BASE_RATE * u128::from(duration_listened) / u128::from(song_duration);
    let amount = match strategy_id {
        "premium" => pro_rata.saturating_mul(2),
        "patronage" => pro_rata.saturating_mul(3) / 2,
        "gift" => 0,
        "pay_per_stream" => pro_rata,
        _ => 0,
    };
    amount.min(u128::from(u64::MAX)) as u64
}

/// Compute a BLAKE2b-256 Merkle root from ordered action hashes.
pub fn compute_merkle_root(hashes: &[ActionHash]) -> Vec<u8> {
    use hdi::prelude::holo_hash::blake2b_256;

    if hashes.is_empty() {
        return vec![0u8; 32];
    }

    let mut current: Vec<Vec<u8>> = hashes
        .iter()
        .map(|hash| hash.get_raw_39().to_vec())
        .collect();

    while current.len() > 1 {
        let mut next = Vec::new();
        for chunk in current.chunks(2) {
            let right = if chunk.len() == 2 {
                &chunk[1]
            } else {
                &chunk[0]
            };
            let combined = [chunk[0].as_slice(), right.as_slice()].concat();
            next.push(blake2b_256(&combined).to_vec());
        }
        current = next;
    }

    current.pop().unwrap_or_else(|| vec![0u8; 32])
}

fn validate_create_play(play: PlayRecord, _action: Create) -> ExternResult<ValidateCallbackResult> {
    if play.duration_listened == 0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Duration listened must be greater than zero".to_string(),
        ));
    }

    // Duration listened cannot exceed song duration
    if play.duration_listened > play.song_duration {
        return Ok(ValidateCallbackResult::Invalid(
            "Duration listened cannot exceed song duration".to_string(),
        ));
    }

    // Plays cannot be pre-settled
    if play.settled {
        return Ok(ValidateCallbackResult::Invalid(
            "New plays must have settled=false".to_string(),
        ));
    }
    if play.settlement_hash.is_some() {
        return Ok(ValidateCallbackResult::Invalid(
            "New plays cannot name a settlement".to_string(),
        ));
    }

    // Play must reference a song
    if play.song_hash.as_ref().is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Play must reference a song".to_string(),
        ));
    }

    let song_record = must_get_valid_record(play.song_hash.clone())?;
    let Some(song) = song_record
        .entry()
        .to_app_option::<Song>()
        .map_err(|error| wasm_error!(WasmErrorInner::Guest(error.to_string())))?
    else {
        return Ok(ValidateCallbackResult::Invalid(
            "Play must reference a catalog Song entry".to_string(),
        ));
    };

    if play.artist != song.artist
        || play.song_duration != song.duration_seconds
        || play.strategy_id != song.strategy_id
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Play artist, duration, and strategy must match the referenced song".to_string(),
        ));
    }

    let expected_amount = calculate_play_amount(
        &song.strategy_id,
        play.duration_listened,
        song.duration_seconds,
    );
    if play.amount_owed != expected_amount {
        return Ok(ValidateCallbackResult::Invalid(
            "Play amount must be derived from the referenced song terms".to_string(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_attestation(
    attestation: PlayAttestation,
    action: Create,
) -> ExternResult<ValidateCallbackResult> {
    // Verify the attestation references a real play. The action itself is
    // already signed, so the attestation must be authored by the listener who
    // authored that play rather than carrying an unchecked signature blob.
    let play_record = must_get_valid_record(attestation.play_hash.clone())?;

    if play_record.action().author() != &action.author {
        return Ok(ValidateCallbackResult::Invalid(
            "Only the play author may create its attestation".to_string(),
        ));
    }

    // Verify that the referenced play matches attestation fields
    let Some(play) = play_record
        .entry()
        .to_app_option::<PlayRecord>()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
    else {
        return Ok(ValidateCallbackResult::Invalid(
            "Attestation must reference a PlayRecord".to_string(),
        ));
    };
    if play.song_hash != attestation.song_hash {
        return Ok(ValidateCallbackResult::Invalid(
            "Attestation song_hash does not match play record".to_string(),
        ));
    }
    if play.artist != attestation.artist {
        return Ok(ValidateCallbackResult::Invalid(
            "Attestation artist does not match play record".to_string(),
        ));
    }
    if play.amount_owed != attestation.amount_owed {
        return Ok(ValidateCallbackResult::Invalid(
            "Attestation amount_owed does not match play record".to_string(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_settlement(
    batch: SettlementBatch,
    action: Create,
) -> ExternResult<ValidateCallbackResult> {
    // Settlement must have plays
    if batch.play_count == 0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Settlement batch must contain at least one play".to_string(),
        ));
    }

    // Play count must match hashes
    if batch.play_count as usize != batch.play_hashes.len() {
        return Ok(ValidateCallbackResult::Invalid(
            "Play count must match number of play hashes".to_string(),
        ));
    }

    // New settlements must be pending
    if batch.status != SettlementStatus::Pending {
        return Ok(ValidateCallbackResult::Invalid(
            "New settlements must have Pending status".to_string(),
        ));
    }
    if batch.tx_hash.is_some() {
        return Ok(ValidateCallbackResult::Invalid(
            "New pending settlements cannot have a transaction hash".to_string(),
        ));
    }

    let unique_hashes: HashSet<_> = batch.play_hashes.iter().collect();
    if unique_hashes.len() != batch.play_hashes.len() {
        return Ok(ValidateCallbackResult::Invalid(
            "Settlement batch cannot contain duplicate play hashes".to_string(),
        ));
    }

    let mut expected_total = 0u64;
    for play_hash in &batch.play_hashes {
        let record = must_get_valid_record(play_hash.clone())?;
        if record.action().author() != &action.author {
            return Ok(ValidateCallbackResult::Invalid(
                "Settlement may include only plays authored by the batch author".to_string(),
            ));
        }
        let Some(play) = record
            .entry()
            .to_app_option::<PlayRecord>()
            .map_err(|error| wasm_error!(WasmErrorInner::Guest(error.to_string())))?
        else {
            return Ok(ValidateCallbackResult::Invalid(
                "Settlement contains a non-play entry".to_string(),
            ));
        };
        if play.artist != batch.artist || play.settled {
            return Ok(ValidateCallbackResult::Invalid(
                "Settlement plays must be unsettled and belong to the named artist".to_string(),
            ));
        }
        expected_total = match expected_total.checked_add(play.amount_owed) {
            Some(total) => total,
            None => {
                return Ok(ValidateCallbackResult::Invalid(
                    "Settlement amount overflow".to_string(),
                ));
            }
        };
    }

    if batch.total_amount != expected_total {
        return Ok(ValidateCallbackResult::Invalid(
            "Settlement total must equal the sum of its plays".to_string(),
        ));
    }

    if batch.merkle_root != compute_merkle_root(&batch.play_hashes) {
        return Ok(ValidateCallbackResult::Invalid(
            "Settlement Merkle root does not match its ordered play hashes".to_string(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

#[cfg(test)]
mod tests {
    use super::calculate_play_amount;

    #[test]
    fn payment_terms_are_deterministic_and_fail_closed() {
        assert_eq!(
            calculate_play_amount("pay_per_stream", 180, 180),
            400_000_000_000_000
        );
        assert_eq!(
            calculate_play_amount("premium", 180, 180),
            800_000_000_000_000
        );
        assert_eq!(
            calculate_play_amount("patronage", 180, 180),
            600_000_000_000_000
        );
        assert_eq!(calculate_play_amount("gift", 180, 180), 0);
        assert_eq!(calculate_play_amount("unknown", 180, 180), 0);
        assert_eq!(calculate_play_amount("pay_per_stream", 10, 180), 0);
        assert_eq!(calculate_play_amount("pay_per_stream", 181, 180), 0);
    }
}
