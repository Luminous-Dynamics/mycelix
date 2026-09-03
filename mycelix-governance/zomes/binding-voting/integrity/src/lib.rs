// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Integrity layer for score-independent binding governance ballots.
//!
//! This zome is deliberately additive. Legacy `Vote`, Phi-weighted, quadratic,
//! and score/ZK voting entries remain readable in their existing zome but are
//! not binding inputs here.

use hdi::prelude::*;
use mycelix_governance_electorate::{
    BallotQualification, ElectorateMembershipEvidence, ElectorateSnapshot, SnapshotBoundTally,
};
use mycelix_governance_rights::{BindingBallot, BindingTallyPolicy, RightPermit};
use mycelix_institutional_core::Digest32;
use std::collections::BTreeSet;

const MAX_ID_BYTES: usize = 512;
const MAX_REF_BYTES: usize = 2048;
const MAX_PROFILE_BYTES: usize = 128;
const MAX_BALLOT_REFS: usize = 1_000_000;

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Anchor(pub String);

/// Immutable election configuration for one binding proposal ballot.
///
/// The electorate snapshot supplies denominator/provenance; the tally policy
/// supplies the counting rule; and the ballot-finality policy supplies the rule
/// for deciding when a DHT-observed ballot set is complete enough to freeze.
/// All three are proposal/rulebook-bound at the coordinator verification layer.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct ElectionConfiguration {
    pub id: String,
    pub proposal_id: String,
    pub proposal_authority_binding: ActionHash,
    pub snapshot: ElectorateSnapshot,
    pub tally_policy: BindingTallyPolicy,
    pub tally_policy_profile: String,
    pub ballot_finality_policy_digest: Digest32,
    pub ballot_finality_policy_profile: String,
    pub ballot_finality_policy_ref: String,
    pub verifier_receipt_ref: String,
    pub created_by: String,
    pub created_at: Timestamp,
}

impl ElectionConfiguration {
    pub fn validate_structure(&self) -> Result<(), String> {
        validate_text(&self.id, "election configuration id", MAX_ID_BYTES)?;
        validate_text(&self.proposal_id, "proposal id", MAX_ID_BYTES)?;
        if !self.proposal_id.starts_with("MIP-") {
            return Err("Binding election proposal id must start with MIP-".into());
        }
        if self.snapshot.proposal.as_str() != self.proposal_id {
            return Err("Electorate snapshot targets a different proposal".into());
        }
        self.snapshot
            .validate()
            .map_err(|e| format!("Invalid electorate snapshot: {e}"))?;
        self.tally_policy
            .validate()
            .map_err(|e| format!("Invalid binding tally policy: {e}"))?;
        validate_profile(&self.tally_policy_profile, "tally policy profile")?;
        require_digest(
            self.ballot_finality_policy_digest,
            "ballot finality policy digest",
        )?;
        validate_profile(
            &self.ballot_finality_policy_profile,
            "ballot finality policy profile",
        )?;
        validate_text(
            &self.ballot_finality_policy_ref,
            "ballot finality policy reference",
            MAX_REF_BYTES,
        )?;
        validate_text(
            &self.verifier_receipt_ref,
            "election configuration verifier receipt",
            MAX_REF_BYTES,
        )?;
        require_mycelix_did(&self.created_by, "election configuration creator")?;
        let created_ms = timestamp_ms(self.created_at, "election configuration created_at")?;
        if created_ms != self.snapshot.captured_at_ms {
            return Err("Election configuration timestamp must equal snapshot capture time".into());
        }
        Ok(())
    }
}

/// Append-only record for one binding ballot cast by one principal.
///
/// Re-voting creates another record. Runtime projection chooses the latest valid
/// source-chain action per voter before asking the configured finality verifier
/// to freeze the ballot set.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct BindingBallotRecord {
    pub id: String,
    pub proposal_id: String,
    pub proposal_authority_binding: ActionHash,
    pub election_configuration: ActionHash,
    pub ballot: BindingBallot,
    pub permit: RightPermit,
    pub membership: ElectorateMembershipEvidence,
    pub verifier_receipt_ref: String,
    pub committed_at: Timestamp,
}

impl BindingBallotRecord {
    pub fn qualification(&self) -> BallotQualification {
        BallotQualification {
            ballot: self.ballot.clone(),
            permit: self.permit.clone(),
            membership: self.membership.clone(),
        }
    }

    pub fn validate_structure(&self) -> Result<(), String> {
        validate_text(&self.id, "binding ballot record id", MAX_ID_BYTES)?;
        validate_text(&self.proposal_id, "proposal id", MAX_ID_BYTES)?;
        if self.ballot.proposal.as_str() != self.proposal_id {
            return Err("Binding ballot targets a different proposal".into());
        }
        self.ballot
            .validate_against_permit(&self.permit)
            .map_err(|e| format!("Invalid ballot/permit binding: {e}"))?;
        self.membership
            .validate()
            .map_err(|e| format!("Invalid electorate membership evidence: {e}"))?;
        if self.membership.principal != self.ballot.voter
            || self.membership.principal != self.permit.principal
        {
            return Err("Ballot, right permit, and electorate membership principal differ".into());
        }
        if self.membership.eligibility_grant_id != self.ballot.eligibility_grant_id
            || self.membership.eligibility_grant_id != self.permit.eligibility_grant_id
        {
            return Err("Ballot, permit, and membership eligibility grant differ".into());
        }
        validate_text(
            &self.verifier_receipt_ref,
            "ballot verifier receipt",
            MAX_REF_BYTES,
        )?;
        let committed_ms = timestamp_ms(self.committed_at, "binding ballot committed_at")?;
        if committed_ms != self.ballot.cast_at_ms {
            return Err("Binding ballot timestamp must equal record commit timestamp".into());
        }
        Ok(())
    }
}

/// Immutable checkpoint freezing the exact ballot action set accepted by a
/// rulebook-bound finality policy.
///
/// Time alone is not finality in an eventually consistent DHT. The coordinator
/// first projects the latest valid source-chain ballot per voter, then requires
/// a verifier receipt proving that this exact action set satisfies the configured
/// observation/replication/finality policy.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct BallotSetCheckpoint {
    pub id: String,
    pub proposal_id: String,
    pub proposal_authority_binding: ActionHash,
    pub election_configuration: ActionHash,
    pub selected_ballots: Vec<ActionHash>,
    pub superseded_ballots: Vec<ActionHash>,
    pub observed_valid_ballots: u64,
    pub finality_policy_digest: Digest32,
    pub finality_policy_profile: String,
    pub finality_policy_ref: String,
    pub observer_count: u32,
    pub required_observer_count: u32,
    pub verifier_receipt_ref: String,
    pub finalized_by: String,
    pub finalized_at: Timestamp,
}

impl BallotSetCheckpoint {
    pub fn validate_structure(&self) -> Result<(), String> {
        validate_text(&self.id, "ballot-set checkpoint id", MAX_ID_BYTES)?;
        validate_text(&self.proposal_id, "proposal id", MAX_ID_BYTES)?;
        if self.selected_ballots.len() > MAX_BALLOT_REFS
            || self.superseded_ballots.len() > MAX_BALLOT_REFS
        {
            return Err("Ballot-set checkpoint references too many ballot actions".into());
        }
        ensure_unique_hashes(&self.selected_ballots, "selected ballot")?;
        ensure_unique_hashes(&self.superseded_ballots, "superseded ballot")?;
        let selected: BTreeSet<String> = self
            .selected_ballots
            .iter()
            .map(ToString::to_string)
            .collect();
        if self
            .superseded_ballots
            .iter()
            .any(|hash| selected.contains(&hash.to_string()))
        {
            return Err("A ballot cannot be both selected and superseded".into());
        }
        if self.observed_valid_ballots
            != (self.selected_ballots.len() + self.superseded_ballots.len()) as u64
        {
            return Err("Observed valid ballot count does not match checkpoint action set".into());
        }
        require_digest(self.finality_policy_digest, "finality policy digest")?;
        validate_profile(&self.finality_policy_profile, "finality policy profile")?;
        validate_text(
            &self.finality_policy_ref,
            "finality policy reference",
            MAX_REF_BYTES,
        )?;
        if self.required_observer_count == 0 || self.observer_count < self.required_observer_count {
            return Err("Ballot-set checkpoint does not meet observer threshold".into());
        }
        validate_text(
            &self.verifier_receipt_ref,
            "ballot-set finality receipt",
            MAX_REF_BYTES,
        )?;
        require_mycelix_did(&self.finalized_by, "ballot-set checkpoint author")?;
        timestamp_ms(self.finalized_at, "ballot-set checkpoint finalized_at")?;
        Ok(())
    }
}

/// Audit record for a recomputed binding tally over one finalized ballot set.
/// Stored tallies are never trusted by existence alone; consumers must use the
/// verified coordinator endpoint, which recomputes the tally from the checkpoint.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct BindingTallyRecord {
    pub id: String,
    pub proposal_id: String,
    pub proposal_authority_binding: ActionHash,
    pub election_configuration: ActionHash,
    pub ballot_set_checkpoint: ActionHash,
    pub tally: SnapshotBoundTally,
    pub computed_by: String,
    pub computed_at: Timestamp,
}

impl BindingTallyRecord {
    pub fn validate_structure(&self) -> Result<(), String> {
        validate_text(&self.id, "binding tally record id", MAX_ID_BYTES)?;
        validate_text(&self.proposal_id, "proposal id", MAX_ID_BYTES)?;
        if self.tally.tally.proposal.as_str() != self.proposal_id {
            return Err("Binding tally targets a different proposal".into());
        }
        require_mycelix_did(&self.computed_by, "binding tally author")?;
        timestamp_ms(self.computed_at, "binding tally computed_at")?;
        let tally = &self.tally.tally;
        if tally.votes_for + tally.votes_against + tally.abstentions != tally.unique_voters {
            return Err("Binding tally vote counts do not equal unique voter count".into());
        }
        if tally.unique_voters > tally.eligible_voters {
            return Err("Binding tally has more voters than the electorate".into());
        }
        if tally.approved && !tally.quorum_reached {
            return Err("Binding tally cannot be approved without quorum".into());
        }
        Ok(())
    }
}

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    Anchor(Anchor),
    ElectionConfiguration(ElectionConfiguration),
    BindingBallot(BindingBallotRecord),
    BallotSetCheckpoint(BallotSetCheckpoint),
    BindingTally(BindingTallyRecord),
}

#[hdk_link_types]
pub enum LinkTypes {
    ProposalToElectionConfiguration,
    ElectionConfigurationToBallot,
    ElectionConfigurationToCheckpoint,
    CheckpointToTally,
    VoterToBindingBallot,
}

fn validate_create_election(
    action: Create,
    entry: ElectionConfiguration,
) -> ExternResult<ValidateCallbackResult> {
    if let Err(error) = entry.validate_structure() {
        return Ok(ValidateCallbackResult::Invalid(error));
    }
    let author_did = format!("did:mycelix:{}", action.author);
    if entry.created_by != author_did {
        return Ok(ValidateCallbackResult::Invalid(
            "Election configuration created_by must equal the committing agent".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_ballot(
    action: Create,
    entry: BindingBallotRecord,
) -> ExternResult<ValidateCallbackResult> {
    if let Err(error) = entry.validate_structure() {
        return Ok(ValidateCallbackResult::Invalid(error));
    }
    let author_did = format!("did:mycelix:{}", action.author);
    if entry.ballot.voter.as_str() != author_did {
        return Ok(ValidateCallbackResult::Invalid(
            "Binding ballot voter must equal the committing agent".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_checkpoint(
    action: Create,
    entry: BallotSetCheckpoint,
) -> ExternResult<ValidateCallbackResult> {
    if let Err(error) = entry.validate_structure() {
        return Ok(ValidateCallbackResult::Invalid(error));
    }
    let author_did = format!("did:mycelix:{}", action.author);
    if entry.finalized_by != author_did {
        return Ok(ValidateCallbackResult::Invalid(
            "Ballot-set checkpoint finalized_by must equal the committing agent".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_tally(
    action: Create,
    entry: BindingTallyRecord,
) -> ExternResult<ValidateCallbackResult> {
    if let Err(error) = entry.validate_structure() {
        return Ok(ValidateCallbackResult::Invalid(error));
    }
    let author_did = format!("did:mycelix:{}", action.author);
    if entry.computed_by != author_did {
        return Ok(ValidateCallbackResult::Invalid(
            "Binding tally computed_by must equal the committing agent".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(store_entry) => match store_entry {
            OpEntry::CreateEntry { app_entry, action } => match app_entry {
                EntryTypes::Anchor(_) => Ok(ValidateCallbackResult::Valid),
                EntryTypes::ElectionConfiguration(entry) => {
                    validate_create_election(action, entry)
                }
                EntryTypes::BindingBallot(entry) => validate_create_ballot(action, entry),
                EntryTypes::BallotSetCheckpoint(entry) => {
                    validate_create_checkpoint(action, entry)
                }
                EntryTypes::BindingTally(entry) => validate_create_tally(action, entry),
            },
            OpEntry::UpdateEntry { app_entry, .. } => match app_entry {
                EntryTypes::Anchor(_) => Ok(ValidateCallbackResult::Valid),
                EntryTypes::ElectionConfiguration(_)
                | EntryTypes::BindingBallot(_)
                | EntryTypes::BallotSetCheckpoint(_)
                | EntryTypes::BindingTally(_) => Ok(ValidateCallbackResult::Invalid(
                    "Binding governance election, ballot, checkpoint, and tally entries are append-only"
                        .into(),
                )),
            },
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterCreateLink { .. } => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterDeleteLink { .. } => Ok(ValidateCallbackResult::Invalid(
            "Binding governance links are append-only".into(),
        )),
        FlatOp::RegisterDelete(_) => Ok(ValidateCallbackResult::Invalid(
            "Binding governance entries are append-only".into(),
        )),
        FlatOp::StoreRecord(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterAgentActivity(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterUpdate(_) => Ok(ValidateCallbackResult::Valid),
    }
}

fn validate_text(value: &str, field: &str, max_bytes: usize) -> Result<(), String> {
    if value.trim().is_empty() {
        return Err(format!("{field} must not be empty"));
    }
    if value.len() > max_bytes {
        return Err(format!("{field} exceeds {max_bytes} bytes"));
    }
    Ok(())
}

fn validate_profile(value: &str, field: &str) -> Result<(), String> {
    let bytes = value.as_bytes();
    if bytes.is_empty()
        || bytes.len() > MAX_PROFILE_BYTES
        || !bytes.iter().all(|byte| {
            byte.is_ascii_lowercase()
                || byte.is_ascii_digit()
                || matches!(*byte, b'.' | b'_' | b'/' | b'-' | b':')
        })
    {
        return Err(format!("{field} is not a canonical profile token"));
    }
    Ok(())
}

fn require_mycelix_did(value: &str, field: &str) -> Result<(), String> {
    validate_text(value, field, MAX_ID_BYTES)?;
    if !value.starts_with("did:mycelix:") {
        return Err(format!("{field} must be a did:mycelix identifier"));
    }
    Ok(())
}

fn timestamp_ms(timestamp: Timestamp, field: &str) -> Result<u64, String> {
    let micros = timestamp.as_micros();
    if micros <= 0 {
        return Err(format!("{field} must be positive"));
    }
    Ok(micros as u64 / 1_000)
}

fn ensure_unique_hashes(hashes: &[ActionHash], field: &str) -> Result<(), String> {
    let unique: BTreeSet<String> = hashes.iter().map(ToString::to_string).collect();
    if unique.len() != hashes.len() {
        return Err(format!("Duplicate {field} action hash"));
    }
    Ok(())
}

fn require_digest(digest: Digest32, field: &str) -> Result<(), String> {
    if digest.is_zero() {
        Err(format!("{field} must not be the zero digest"))
    } else {
        Ok(())
    }
}
