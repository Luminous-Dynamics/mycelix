// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root//! Housing Governance Integrity Zome
//! Entry types and validation for board meetings, resolutions, bylaws, and elections.

use hdi::prelude::*;

/// Anchor entry for deterministic link bases
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Anchor(pub String);

/// Type of board meeting
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum MeetingType {
    Regular,
    Special,
    Annual,
    Emergency,
}

/// A board meeting
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct BoardMeeting {
    pub cooperative_hash: Option<ActionHash>,
    pub title: String,
    pub agenda: Vec<String>,
    pub scheduled_at: Timestamp,
    pub location: String,
    pub meeting_type: MeetingType,
    pub minutes: Option<String>,
    pub attendees: Vec<AgentPubKey>,
}

/// Category of a resolution
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum ResolutionCategory {
    Budget,
    Maintenance,
    Membership,
    Rules,
    Assessment,
    Improvement,
    Emergency,
    Other(String),
}

/// A resolution proposed or adopted at a meeting
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Resolution {
    pub meeting_hash: Option<ActionHash>,
    pub title: String,
    pub description: String,
    pub proposed_by: AgentPubKey,
    pub category: ResolutionCategory,
    pub votes_for: u32,
    pub votes_against: u32,
    pub votes_abstain: u32,
    pub quorum_met: bool,
    pub passed: bool,
    pub effective_date: Option<Timestamp>,
}

/// A bylaw of the cooperative
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct ByLaw {
    pub id: String,
    pub title: String,
    pub content: String,
    pub version: u32,
    pub adopted_at: Timestamp,
    pub amended_at: Option<Timestamp>,
    pub supersedes: Option<ActionHash>,
}

/// A candidate entry in an election
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct CandidateEntry {
    pub agent: AgentPubKey,
    pub position: String,
    pub statement: String,
}

/// Result of an election for a single position
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct ElectionResult {
    pub position: String,
    pub winner: AgentPubKey,
    pub votes_received: u32,
}

/// An election for cooperative positions
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Election {
    pub title: String,
    pub positions: Vec<String>,
    pub candidates: Vec<CandidateEntry>,
    pub voting_opens: Timestamp,
    pub voting_closes: Timestamp,
    pub results: Option<Vec<ElectionResult>>,
}

/// A ballot cast in an election
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Ballot {
    pub election_hash: ActionHash,
    pub voter: AgentPubKey,
    pub votes: Vec<BallotVote>,
    pub cast_at: Timestamp,
}

/// A single vote within a ballot
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct BallotVote {
    pub position: String,
    pub candidate: AgentPubKey,
}

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    Anchor(Anchor),
    BoardMeeting(BoardMeeting),
    Resolution(Resolution),
    ByLaw(ByLaw),
    Election(Election),
    Ballot(Ballot),
}

#[hdk_link_types]
pub enum LinkTypes {
    /// All meetings anchor
    AllMeetings,
    /// Meeting to resolutions
    MeetingToResolution,
    /// All bylaws anchor
    AllByLaws,
    /// ByLaw supersession chain
    ByLawSupersedes,
    /// All elections anchor
    AllElections,
    /// Election to ballots
    ElectionToBallot,
    /// Voter to their ballots
    VoterToBallot,
    /// Proposer to their resolutions
    ProposerToResolution,
}

#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(store_entry) => match store_entry {
            OpEntry::CreateEntry { app_entry, action } => match app_entry {
                EntryTypes::Anchor(_) => Ok(ValidateCallbackResult::Valid),
                EntryTypes::BoardMeeting(meeting) => validate_create_meeting(action, meeting),
                EntryTypes::Resolution(resolution) => {
                    validate_create_resolution(action, resolution)
                }
                EntryTypes::ByLaw(bylaw) => validate_create_bylaw(action, bylaw),
                EntryTypes::Election(election) => validate_create_election(action, election),
                EntryTypes::Ballot(ballot) => validate_create_ballot(action, ballot),
            },
            OpEntry::UpdateEntry {
                app_entry,
                action,
                original_action_hash: _,
                original_entry_hash: _,
            } => validate_update_entry_type(action, app_entry),
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterCreateLink {
            link_type,
            base_address: _,
            target_address: _,
            tag: _,
            action: _,
        } => match link_type {
            LinkTypes::AllMeetings => Ok(ValidateCallbackResult::Valid),
            LinkTypes::MeetingToResolution => Ok(ValidateCallbackResult::Valid),
            LinkTypes::AllByLaws => Ok(ValidateCallbackResult::Valid),
            LinkTypes::ByLawSupersedes => Ok(ValidateCallbackResult::Valid),
            LinkTypes::AllElections => Ok(ValidateCallbackResult::Valid),
            LinkTypes::ElectionToBallot => Ok(ValidateCallbackResult::Valid),
            LinkTypes::VoterToBallot => Ok(ValidateCallbackResult::Valid),
            LinkTypes::ProposerToResolution => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterDeleteLink {
            link_type: _,
            original_action: _,
            base_address: _,
            target_address: _,
            tag: _,
            action: _,
        } => Ok(ValidateCallbackResult::Valid),
        FlatOp::StoreRecord(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterAgentActivity(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterUpdate(op_update) => match op_update {
            // This DHT op was previously left fully permissive (`Ok(Valid)`
            // unconditionally) -- the 23rd confirmed instance of this exact
            // bug pattern this pass. Found + fixed 2026-07-09 during the P0
            // author-binding pass. Route through the same per-type
            // validators as the StoreEntry perspective.
            OpUpdate::Entry { app_entry, action } => validate_update_entry_type(action, app_entry),
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterDelete(OpDelete { action }) => {
            // Also previously fully permissive. The coordinator never
            // calls delete_entry here, so this is pure hardening, zero
            // functional impact.
            let original = must_get_action(action.deletes_address.clone())?;
            if action.author != *original.action().author() {
                return Ok(ValidateCallbackResult::Invalid(
                    "Only the original entry author can delete an entry".into(),
                ));
            }
            Ok(ValidateCallbackResult::Valid)
        }
    }
}

/// Shared per-entry-type update validation, called from BOTH the
/// StoreEntry (OpEntry::UpdateEntry) and RegisterUpdate DHT-op
/// perspectives so they agree.
fn validate_update_entry_type(
    action: Update,
    app_entry: EntryTypes,
) -> ExternResult<ValidateCallbackResult> {
    match app_entry {
        EntryTypes::Anchor(_) => Ok(ValidateCallbackResult::Valid),
        EntryTypes::BoardMeeting(meeting) => validate_update_meeting(action, meeting),
        EntryTypes::Resolution(resolution) => validate_update_resolution(action, resolution),
        // amend_bylaw creates a NEW ByLaw entry with `supersedes` set
        // rather than calling update_entry (confirmed via grep) -- dead
        // update path, made explicitly immutable.
        EntryTypes::ByLaw(_) => Ok(ValidateCallbackResult::Invalid(
            "ByLaws are immutable; amendments create a new superseding entry".into(),
        )),
        EntryTypes::Election(election) => validate_update_election(action, election),
        EntryTypes::Ballot(_) => Ok(ValidateCallbackResult::Invalid(
            "Ballots cannot be modified after casting".into(),
        )),
    }
}

/// No author requirement: record_minutes has zero caller-identity check
/// (no secretary/board-officer role concept exists in this zome to bind
/// against) -- case (c). Content restricted to minutes/attendees -- this
/// closes the wide-open bug that previously let title/agenda/
/// scheduled_at/location/meeting_type change unconditionally on update
/// too.
fn validate_update_meeting(
    action: Update,
    meeting: BoardMeeting,
) -> ExternResult<ValidateCallbackResult> {
    let original_record = must_get_valid_record(action.original_action_address.clone())?;
    let original: BoardMeeting = original_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Original meeting not found".into()
        )))?;

    if meeting.cooperative_hash != original.cooperative_hash
        || meeting.title != original.title
        || meeting.agenda != original.agenda
        || meeting.scheduled_at != original.scheduled_at
        || meeting.location != original.location
        || meeting.meeting_type != original.meeting_type
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Only minutes/attendees can change on a meeting update".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

/// **Disclosed, NOT-fixed vote-integrity gap**: unlike Election/Ballot
/// (which have a real per-voter Ballot entry, duplicate-vote prevention,
/// and tallies computed from actual cast ballots), Resolution voting has
/// NO underlying per-voter record at all -- vote_on_resolution just
/// accepts caller-asserted vote tallies directly and sets `passed`
/// accordingly, with zero verification against any real vote count.
/// Content is still restricted here (votes_for/votes_against/
/// votes_abstain/quorum_met/passed/effective_date only; title/
/// description/proposed_by/category/meeting_hash immutable), which
/// prevents unrelated-field tampering, but does NOT and cannot fix the
/// deeper issue: the vote tallies themselves are entirely unverifiable
/// at the DHT level as currently architected. A real fix needs a
/// per-voter ballot/tally system for resolutions mirroring Election's,
/// which is a larger feature, not an author-binding fix. Flagged
/// 2026-07-09 during the P0 author-binding pass for dedicated follow-up.
fn validate_update_resolution(
    action: Update,
    resolution: Resolution,
) -> ExternResult<ValidateCallbackResult> {
    let original_record = must_get_valid_record(action.original_action_address.clone())?;
    let original: Resolution = original_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Original resolution not found".into()
        )))?;

    if resolution.meeting_hash != original.meeting_hash
        || resolution.title != original.title
        || resolution.description != original.description
        || resolution.proposed_by != original.proposed_by
        || resolution.category != original.category
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Only vote/quorum/passed/effective_date fields can change on a resolution update"
                .into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

/// No author requirement: tally_election has zero caller-identity check
/// (no election-official role concept exists) -- case (c). Its own
/// business logic (voting must be closed, can't re-tally) is the real
/// safeguard for this operation and is unaffected by this fix. Content
/// restricted to results only.
fn validate_update_election(
    action: Update,
    election: Election,
) -> ExternResult<ValidateCallbackResult> {
    let original_record = must_get_valid_record(action.original_action_address.clone())?;
    let original: Election = original_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Original election not found".into()
        )))?;

    if election.title != original.title
        || election.positions != original.positions
        || election.candidates != original.candidates
        || election.voting_opens != original.voting_opens
        || election.voting_closes != original.voting_closes
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Only results can change on an election update".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_meeting(
    _action: Create,
    meeting: BoardMeeting,
) -> ExternResult<ValidateCallbackResult> {
    if meeting.title.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Meeting title cannot be empty".into(),
        ));
    }
    if meeting.title.len() > 256 {
        return Ok(ValidateCallbackResult::Invalid(
            "Meeting title must be at most 256 characters".into(),
        ));
    }
    if meeting.agenda.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Meeting must have at least one agenda item".into(),
        ));
    }
    if meeting.location.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Meeting location cannot be empty".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_resolution(
    action: Create,
    resolution: Resolution,
) -> ExternResult<ValidateCallbackResult> {
    // Author-binding: the coordinator's propose_resolution previously
    // took the FULL struct straight from caller input with ZERO
    // derivation from agent_info() -- any agent could forge a respected
    // board member as proposer for false legitimacy. Found + fixed
    // 2026-07-09 during the P0 author-binding pass (coordinator-side fix
    // applied alongside this).
    if resolution.proposed_by != action.author {
        return Ok(ValidateCallbackResult::Invalid(
            "Resolution proposed_by must correspond to the committing agent".into(),
        ));
    }

    if resolution.title.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Resolution title cannot be empty".into(),
        ));
    }
    if resolution.description.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Resolution description cannot be empty".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_bylaw(_action: Create, bylaw: ByLaw) -> ExternResult<ValidateCallbackResult> {
    if bylaw.id.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "ByLaw ID cannot be empty".into(),
        ));
    }
    if bylaw.title.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "ByLaw title cannot be empty".into(),
        ));
    }
    if bylaw.content.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "ByLaw content cannot be empty".into(),
        ));
    }
    if bylaw.version == 0 {
        return Ok(ValidateCallbackResult::Invalid(
            "ByLaw version must be at least 1".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_election(
    _action: Create,
    election: Election,
) -> ExternResult<ValidateCallbackResult> {
    if election.title.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Election title cannot be empty".into(),
        ));
    }
    if election.positions.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Election must have at least one position".into(),
        ));
    }
    if election.voting_closes <= election.voting_opens {
        return Ok(ValidateCallbackResult::Invalid(
            "Voting close must be after voting open".into(),
        ));
    }
    if election.results.is_some() {
        return Ok(ValidateCallbackResult::Invalid(
            "New elections cannot have results".into(),
        ));
    }
    // Verify all candidates are for valid positions
    for candidate in &election.candidates {
        if !election.positions.contains(&candidate.position) {
            return Ok(ValidateCallbackResult::Invalid(format!(
                "Candidate position '{}' is not in the election positions list",
                candidate.position
            )));
        }
        if candidate.statement.is_empty() {
            return Ok(ValidateCallbackResult::Invalid(
                "Candidate statement cannot be empty".into(),
            ));
        }
    }
    Ok(ValidateCallbackResult::Valid)
}

/// **Real vote-forgery fix**: the coordinator's cast_ballot previously
/// took the FULL Ballot struct straight from caller input with ZERO
/// derivation from agent_info() -- `voter` was entirely caller-supplied.
/// Since cast_ballot's own "has this voter already cast a ballot" check
/// and tally_election's per-candidate vote count BOTH key off
/// `ballot.voter`, a modified coordinator could previously forge
/// `voter` as an arbitrary victim agent to cast additional ballots under
/// someone else's identity (classic vote-stuffing/ballot-forgery — the
/// exact class of bug this P0 pass exists to close). Found + fixed
/// 2026-07-09 during the P0 author-binding pass (coordinator-side fix
/// applied alongside this).
fn validate_create_ballot(action: Create, ballot: Ballot) -> ExternResult<ValidateCallbackResult> {
    if ballot.voter != action.author {
        return Ok(ValidateCallbackResult::Invalid(
            "Ballot voter must correspond to the committing agent".into(),
        ));
    }

    if ballot.votes.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Ballot must contain at least one vote".into(),
        ));
    }
    // Check for duplicate position votes
    let mut seen_positions = std::collections::HashSet::new();
    for vote in &ballot.votes {
        if !seen_positions.insert(vote.position.clone()) {
            return Ok(ValidateCallbackResult::Invalid(format!(
                "Duplicate vote for position '{}'",
                vote.position
            )));
        }
    }
    Ok(ValidateCallbackResult::Valid)
}

#[cfg(test)]
mod author_binding_tests {
    use super::*;

    fn create_action(author: AgentPubKey) -> Create {
        Create {
            author,
            timestamp: Timestamp::from_micros(0),
            action_seq: 0,
            prev_action: ActionHash::from_raw_36(vec![0u8; 36]),
            entry_type: EntryType::App(AppEntryDef::new(
                EntryDefIndex::from(0),
                0.into(),
                EntryVisibility::Public,
            )),
            entry_hash: EntryHash::from_raw_36(vec![0u8; 36]),
            weight: Default::default(),
        }
    }

    fn update_action(author: AgentPubKey) -> Update {
        Update {
            author,
            timestamp: Timestamp::from_micros(1),
            action_seq: 1,
            prev_action: ActionHash::from_raw_36(vec![0u8; 36]),
            original_action_address: ActionHash::from_raw_36(vec![9u8; 36]),
            original_entry_address: EntryHash::from_raw_36(vec![0u8; 36]),
            entry_type: EntryType::App(AppEntryDef::new(
                EntryDefIndex::from(0),
                0.into(),
                EntryVisibility::Public,
            )),
            entry_hash: EntryHash::from_raw_36(vec![0u8; 36]),
            weight: Default::default(),
        }
    }

    fn me() -> AgentPubKey {
        AgentPubKey::from_raw_36(vec![0u8; 36])
    }

    fn other_agent() -> AgentPubKey {
        AgentPubKey::from_raw_36(vec![1u8; 36])
    }

    fn valid_resolution(proposed_by: AgentPubKey) -> Resolution {
        Resolution {
            meeting_hash: None,
            title: "Approve new roof".into(),
            description: "Replace the roof on Building A".into(),
            proposed_by,
            category: ResolutionCategory::Maintenance,
            votes_for: 0,
            votes_against: 0,
            votes_abstain: 0,
            quorum_met: false,
            passed: false,
            effective_date: None,
        }
    }

    #[test]
    fn create_resolution_valid_when_proposer_matches_committer() {
        let r = valid_resolution(me());
        let result = validate_create_resolution(create_action(me()), r).unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn create_resolution_forgery_rejected() {
        let r = valid_resolution(me());
        let result = validate_create_resolution(create_action(other_agent()), r).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    fn valid_ballot(voter: AgentPubKey) -> Ballot {
        Ballot {
            election_hash: ActionHash::from_raw_36(vec![2u8; 36]),
            voter,
            votes: vec![BallotVote {
                position: "Treasurer".into(),
                candidate: AgentPubKey::from_raw_36(vec![3u8; 36]),
            }],
            cast_at: Timestamp::from_micros(0),
        }
    }

    #[test]
    fn create_ballot_valid_when_voter_matches_committer() {
        let b = valid_ballot(me());
        let result = validate_create_ballot(create_action(me()), b).unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn create_ballot_forgery_rejected() {
        let b = valid_ballot(me());
        let result = validate_create_ballot(create_action(other_agent()), b).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn update_entry_type_rejects_bylaw_update() {
        let bylaw = ByLaw {
            id: "b-1".into(),
            title: "Noise policy".into(),
            content: "Quiet hours 10pm-8am".into(),
            version: 1,
            adopted_at: Timestamp::from_micros(0),
            amended_at: None,
            supersedes: None,
        };
        let result =
            validate_update_entry_type(update_action(me()), EntryTypes::ByLaw(bylaw)).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn update_entry_type_rejects_ballot_update() {
        let b = valid_ballot(me());
        let result =
            validate_update_entry_type(update_action(me()), EntryTypes::Ballot(b)).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }
}
