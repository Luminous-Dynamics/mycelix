// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
use hdi::prelude::*;

const MAX_ARBITRATORS: usize = 5;
const MAX_REVISION_DEPTH: usize = 32;
const RESULT_THRESHOLD: f64 = 0.66;
const FLOAT_EPSILON: f64 = 0.000_001;

/// Dispute entry - represents a disputed transaction.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Dispute {
    /// Stable transaction identity: the original Transaction Create action.
    pub transaction_hash: ActionHash,

    /// Exact transaction revision that was Disputed when this case was filed.
    /// Conflict-bound disputes use the stable transaction root here and bind the
    /// exact unsafe leaves in `conflict_heads`.
    pub transaction_revision_hash: ActionHash,

    /// Exact unsafe transaction leaves under arbitration. Empty for an ordinary
    /// lifecycle dispute filed from one authored Disputed revision.
    #[serde(default)]
    pub conflict_heads: Vec<ActionHash>,

    /// Agent who filed the dispute.
    pub filed_by: AgentPubKey,

    /// Buyer in the transaction.
    pub buyer: AgentPubKey,

    /// Seller in the transaction.
    pub seller: AgentPubKey,

    /// Reason for dispute.
    pub reason: String,

    /// IPFS CIDs of evidence (photos, documents, etc.).
    pub evidence_cids: Vec<String>,

    /// Current status.
    pub status: DisputeStatus,

    /// Assigned arbitrators.
    pub arbitrators: Vec<AgentPubKey>,

    /// Result bound to a resolved dispute revision.
    pub result_hash: Option<ActionHash>,

    /// Creation timestamp.
    pub created_at: Timestamp,

    /// Last update timestamp.
    pub updated_at: Timestamp,
}

/// Dispute status lifecycle.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
#[serde(rename_all = "lowercase")]
pub enum DisputeStatus {
    Filed,
    UnderReview,
    Voting,
    ResolvedBuyer,
    ResolvedSeller,
    Withdrawn,
}

/// Arbitration vote - an arbitrator's decision.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct ArbitrationVote {
    /// Stable dispute identity: the original Dispute Create action.
    pub dispute_hash: ActionHash,

    /// Exact UnderReview/Voting revision used to authorize this vote.
    pub dispute_revision_hash: ActionHash,

    /// Arbitrator who voted.
    pub arbitrator: AgentPubKey,

    /// Decision (true = favor buyer, false = favor seller).
    pub favor_buyer: bool,

    /// Reasoning for decision.
    pub reasoning: String,

    /// Reserved compatibility field. Until reputation snapshots are
    /// cryptographically bound, every accepted vote has weight 1.0.
    pub arbitrator_matl_score: f64,

    /// Timestamp.
    pub voted_at: Timestamp,
}

/// Arbitration result - deterministic outcome derived from bound votes.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct ArbitrationResult {
    /// Stable dispute identity.
    pub dispute_hash: ActionHash,

    /// Exact Voting revision that authorized finalization.
    pub dispute_revision_hash: ActionHash,

    /// Every vote used to derive this result.
    pub vote_hashes: Vec<ActionHash>,

    /// Winner (buyer or seller).
    pub winner: AgentPubKey,

    /// Loser.
    pub loser: AgentPubKey,

    /// Equal-weight buyer vote ratio in the current guarded implementation.
    pub weighted_vote: f64,

    /// Total votes cast.
    pub total_votes: u32,

    /// Recommended compensation amount (in cents). Settlement is external.
    pub compensation_cents: Option<u64>,

    /// Resolution summary.
    pub summary: String,

    /// Timestamp.
    pub finalized_at: Timestamp,
}

#[hdk_link_types]
pub enum LinkTypes {
    TransactionToDispute,
    AgentToFiledDisputes,
    AgentToArbitrationOpportunities,
    DisputeToVotes,
    DisputeToResult,
    AllDisputes,
    AllArbitrators,
}

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    Dispute(Dispute),
    ArbitrationVote(ArbitrationVote),
    ArbitrationResult(ArbitrationResult),
}

/// Minimal transaction shape needed to bind a dispute and compensation to
/// authoritative marketplace terms. MessagePack map decoding ignores fields
/// that are not relevant to arbitration validation.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
struct TransactionSnapshot {
    buyer: AgentPubKey,
    seller: AgentPubKey,
    total_price_cents: u64,
    status: TransactionStatusSnapshot,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
#[serde(rename_all = "lowercase")]
enum TransactionStatusSnapshot {
    Pending,
    Confirmed,
    Shipped,
    Delivered,
    Completed,
    Disputed,
    Cancelled,
}

#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(store_entry) => match store_entry {
            OpEntry::CreateEntry { app_entry, action } => match app_entry {
                EntryTypes::Dispute(dispute) => validate_create_dispute(&dispute, &action),
                EntryTypes::ArbitrationVote(vote) => validate_create_vote(&vote, &action),
                EntryTypes::ArbitrationResult(result) => validate_create_result(&result, &action),
            },
            OpEntry::UpdateEntry {
                app_entry, action, ..
            } => match app_entry {
                EntryTypes::Dispute(dispute) => validate_update_dispute(&dispute, &action),
                EntryTypes::ArbitrationVote(_) => Ok(ValidateCallbackResult::Invalid(
                    "Arbitration votes are immutable".into(),
                )),
                EntryTypes::ArbitrationResult(_) => Ok(ValidateCallbackResult::Invalid(
                    "Arbitration results are immutable".into(),
                )),
            },
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterUpdate(update_entry) => match update_entry {
            OpUpdate::Entry { app_entry, action } => match app_entry {
                EntryTypes::Dispute(dispute) => validate_update_dispute(&dispute, &action),
                EntryTypes::ArbitrationVote(_) => Ok(ValidateCallbackResult::Invalid(
                    "Arbitration votes are immutable".into(),
                )),
                EntryTypes::ArbitrationResult(_) => Ok(ValidateCallbackResult::Invalid(
                    "Arbitration results are immutable".into(),
                )),
            },
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterDelete(_) => Ok(ValidateCallbackResult::Invalid(
            "Disputes, votes, and results are permanent audit records".into(),
        )),
        _ => Ok(ValidateCallbackResult::Valid),
    }
}

fn validate_create_dispute(
    dispute: &Dispute,
    action: &Create,
) -> ExternResult<ValidateCallbackResult> {
    if let Err(reason) = validate_dispute_data(dispute) {
        return Ok(ValidateCallbackResult::Invalid(reason));
    }
    if action.author != dispute.filed_by {
        return Ok(ValidateCallbackResult::Invalid(
            "Dispute creation must be authored by filed_by".into(),
        ));
    }
    if dispute.status != DisputeStatus::Filed {
        return Ok(ValidateCallbackResult::Invalid(
            "New disputes must start in Filed status".into(),
        ));
    }
    if !dispute.arbitrators.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "New disputes cannot pre-assign arbitrators".into(),
        ));
    }
    if dispute.result_hash.is_some() {
        return Ok(ValidateCallbackResult::Invalid(
            "New disputes cannot contain an arbitration result".into(),
        ));
    }
    if dispute.created_at != dispute.updated_at {
        return Ok(ValidateCallbackResult::Invalid(
            "New disputes must have matching creation and update timestamps".into(),
        ));
    }

    if dispute.conflict_heads.is_empty() {
        let transaction_root = find_action_root(dispute.transaction_revision_hash.clone())?;
        if transaction_root != dispute.transaction_hash {
            return Ok(ValidateCallbackResult::Invalid(
                "Dispute transaction revision does not belong to the declared transaction root".into(),
            ));
        }
        let transaction: TransactionSnapshot =
            decode_record(&must_get_valid_record(dispute.transaction_revision_hash.clone())?)?;
        if transaction.status != TransactionStatusSnapshot::Disputed {
            return Ok(ValidateCallbackResult::Invalid(
                "A dispute may only be filed against a Disputed transaction revision".into(),
            ));
        }
        if transaction.buyer != dispute.buyer || transaction.seller != dispute.seller {
            return Ok(ValidateCallbackResult::Invalid(
                "Dispute parties do not match the transaction".into(),
            ));
        }
    } else if let Err(reason) = validate_conflict_dispute_binding(dispute)? {
        return Ok(ValidateCallbackResult::Invalid(reason));
    }

    Ok(ValidateCallbackResult::Valid)
}

fn validate_update_dispute(
    updated: &Dispute,
    action: &Update,
) -> ExternResult<ValidateCallbackResult> {
    if let Err(reason) = validate_dispute_data(updated) {
        return Ok(ValidateCallbackResult::Invalid(reason));
    }

    let original: Dispute =
        decode_record(&must_get_valid_record(action.original_action_address.clone())?)?;

    if updated.transaction_hash != original.transaction_hash
        || updated.transaction_revision_hash != original.transaction_revision_hash
        || updated.conflict_heads != original.conflict_heads
        || updated.filed_by != original.filed_by
        || updated.buyer != original.buyer
        || updated.seller != original.seller
        || updated.reason != original.reason
        || updated.evidence_cids != original.evidence_cids
        || updated.created_at != original.created_at
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Dispute transaction binding, parties, evidence, and creation time are immutable".into(),
        ));
    }
    if updated.updated_at < original.updated_at {
        return Ok(ValidateCallbackResult::Invalid(
            "Dispute update timestamp cannot move backwards".into(),
        ));
    }

    if let Err(reason) = validate_dispute_transition(&original, updated, &action.author) {
        return Ok(ValidateCallbackResult::Invalid(reason));
    }

    if matches!(
        updated.status,
        DisputeStatus::ResolvedBuyer | DisputeStatus::ResolvedSeller
    ) {
        let result_hash = updated.result_hash.clone().ok_or_else(|| {
            wasm_error!(WasmErrorInner::Guest(
                "Resolved dispute omitted its result hash".into(),
            ))
        })?;
        let result: ArbitrationResult = decode_record(&must_get_valid_record(result_hash)?)?;
        let root = find_action_root(action.original_action_address.clone())?;
        if result.dispute_hash != root {
            return Ok(ValidateCallbackResult::Invalid(
                "Bound arbitration result belongs to a different dispute".into(),
            ));
        }
        let expected_status = if result.winner == original.buyer {
            DisputeStatus::ResolvedBuyer
        } else if result.winner == original.seller {
            DisputeStatus::ResolvedSeller
        } else {
            return Ok(ValidateCallbackResult::Invalid(
                "Arbitration winner must be a transaction party".into(),
            ));
        };
        if updated.status != expected_status {
            return Ok(ValidateCallbackResult::Invalid(
                "Resolved dispute status does not match the bound result winner".into(),
            ));
        }
    }

    Ok(ValidateCallbackResult::Valid)
}

fn validate_dispute_transition(
    original: &Dispute,
    updated: &Dispute,
    author: &AgentPubKey,
) -> Result<(), String> {
    use DisputeStatus::*;

    match (&original.status, &updated.status) {
        (Filed, UnderReview) => {
            if author != &original.filed_by {
                return Err("Only the filer may publish the initial arbitrator assignment".into());
            }
            validate_arbitrator_set(updated)?;
            if updated.result_hash.is_some() {
                return Err("Under-review disputes cannot contain a result".into());
            }
        }
        (Filed, Withdrawn) => {
            if author != &original.filed_by {
                return Err("Only the filer may withdraw a dispute".into());
            }
            if updated.arbitrators != original.arbitrators || updated.result_hash.is_some() {
                return Err("Withdrawal cannot change arbitrators or attach a result".into());
            }
        }
        (UnderReview, Voting) => {
            if !original.arbitrators.contains(author) {
                return Err("Only an assigned arbitrator may close voting".into());
            }
            if updated.arbitrators != original.arbitrators || updated.result_hash.is_some() {
                return Err("Voting transition cannot change arbitrators or attach a result".into());
            }
        }
        (UnderReview, Withdrawn) => {
            if author != &original.filed_by {
                return Err("Only the filer may withdraw a dispute".into());
            }
            if updated.arbitrators != original.arbitrators || updated.result_hash.is_some() {
                return Err("Withdrawal cannot change arbitrators or attach a result".into());
            }
        }
        (Voting, ResolvedBuyer | ResolvedSeller) => {
            if !original.arbitrators.contains(author) {
                return Err("Only an assigned arbitrator may publish the deterministic result".into());
            }
            if updated.arbitrators != original.arbitrators {
                return Err("Resolved disputes cannot change the arbitrator set".into());
            }
            if updated.result_hash.is_none() {
                return Err("Resolved disputes must bind an arbitration result".into());
            }
        }
        _ => return Err(format!(
            "Illegal dispute transition: {:?} -> {:?}",
            original.status, updated.status
        )),
    }

    Ok(())
}

fn validate_create_vote(
    vote: &ArbitrationVote,
    action: &Create,
) -> ExternResult<ValidateCallbackResult> {
    if let Err(reason) = validate_vote_data(vote) {
        return Ok(ValidateCallbackResult::Invalid(reason));
    }
    if action.author != vote.arbitrator {
        return Ok(ValidateCallbackResult::Invalid(
            "Vote creation must be authored by the arbitrator".into(),
        ));
    }

    let dispute_root = find_action_root(vote.dispute_revision_hash.clone())?;
    if dispute_root != vote.dispute_hash {
        return Ok(ValidateCallbackResult::Invalid(
            "Vote dispute revision does not belong to the declared dispute root".into(),
        ));
    }
    let dispute: Dispute =
        decode_record(&must_get_valid_record(vote.dispute_revision_hash.clone())?)?;
    if !matches!(
        dispute.status,
        DisputeStatus::UnderReview | DisputeStatus::Voting
    ) {
        return Ok(ValidateCallbackResult::Invalid(
            "Votes require an UnderReview or Voting dispute revision".into(),
        ));
    }
    if !dispute.arbitrators.contains(&vote.arbitrator) {
        return Ok(ValidateCallbackResult::Invalid(
            "Vote author is not assigned to this dispute".into(),
        ));
    }
    if vote.arbitrator == dispute.buyer
        || vote.arbitrator == dispute.seller
        || vote.arbitrator == dispute.filed_by
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Arbitrator cannot be a party to the dispute".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_result(
    result: &ArbitrationResult,
    action: &Create,
) -> ExternResult<ValidateCallbackResult> {
    if let Err(reason) = validate_result_data(result) {
        return Ok(ValidateCallbackResult::Invalid(reason));
    }

    let dispute_root = find_action_root(result.dispute_revision_hash.clone())?;
    if dispute_root != result.dispute_hash {
        return Ok(ValidateCallbackResult::Invalid(
            "Result dispute revision does not belong to the declared dispute root".into(),
        ));
    }
    let dispute: Dispute =
        decode_record(&must_get_valid_record(result.dispute_revision_hash.clone())?)?;
    if dispute.status != DisputeStatus::Voting {
        return Ok(ValidateCallbackResult::Invalid(
            "Results require a Voting dispute revision".into(),
        ));
    }
    if !dispute.arbitrators.contains(&action.author) {
        return Ok(ValidateCallbackResult::Invalid(
            "Only an assigned arbitrator may create the result".into(),
        ));
    }

    let mut seen_arbitrators = Vec::new();
    let mut buyer_votes = 0_u32;
    for vote_hash in &result.vote_hashes {
        let vote: ArbitrationVote = decode_record(&must_get_valid_record(vote_hash.clone())?)?;
        if vote.dispute_hash != result.dispute_hash {
            return Ok(ValidateCallbackResult::Invalid(
                "Result includes a vote from another dispute".into(),
            ));
        }
        if !dispute.arbitrators.contains(&vote.arbitrator) {
            return Ok(ValidateCallbackResult::Invalid(
                "Result includes a vote from an unassigned arbitrator".into(),
            ));
        }
        if seen_arbitrators.contains(&vote.arbitrator) {
            return Ok(ValidateCallbackResult::Invalid(
                "Result includes more than one vote from an arbitrator".into(),
            ));
        }
        seen_arbitrators.push(vote.arbitrator.clone());
        if vote.favor_buyer {
            buyer_votes = buyer_votes.saturating_add(1);
        }
    }

    if seen_arbitrators.len() != dispute.arbitrators.len() {
        return Ok(ValidateCallbackResult::Invalid(
            "Result must include exactly one vote from every assigned arbitrator".into(),
        ));
    }

    let expected_ratio = f64::from(buyer_votes) / result.vote_hashes.len() as f64;
    if (result.weighted_vote - expected_ratio).abs() > FLOAT_EPSILON {
        return Ok(ValidateCallbackResult::Invalid(
            "Result vote ratio does not match its bound votes".into(),
        ));
    }

    let buyer_wins = expected_ratio > RESULT_THRESHOLD;
    let (expected_winner, expected_loser) = if buyer_wins {
        (&dispute.buyer, &dispute.seller)
    } else {
        (&dispute.seller, &dispute.buyer)
    };
    if &result.winner != expected_winner || &result.loser != expected_loser {
        return Ok(ValidateCallbackResult::Invalid(
            "Result winner and loser do not match the bound vote ratio".into(),
        ));
    }

    let transaction: TransactionSnapshot =
        decode_record(&must_get_valid_record(dispute.transaction_hash.clone())?)?;
    let expected_compensation = recommended_compensation(
        transaction.total_price_cents,
        expected_ratio,
    );
    if result.compensation_cents != Some(expected_compensation) {
        return Ok(ValidateCallbackResult::Invalid(
            "Result compensation does not match the deterministic recommendation".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

fn validate_dispute_data(dispute: &Dispute) -> Result<(), String> {
    if dispute.reason.trim().is_empty() {
        return Err("Dispute reason cannot be empty".into());
    }
    if dispute.reason.len() > 5000 {
        return Err("Dispute reason too long (max 5000 characters)".into());
    }
    if dispute.filed_by != dispute.buyer && dispute.filed_by != dispute.seller {
        return Err("Only buyer or seller can file dispute".into());
    }
    if dispute.buyer == dispute.seller {
        return Err("Buyer and seller must be different".into());
    }
    if dispute.conflict_heads.len() > 16 {
        return Err("Conflict disputes may bind at most 16 transaction heads".into());
    }
    if !dispute.conflict_heads.is_empty() && !is_sorted_unique_hashes(&dispute.conflict_heads) {
        return Err("Conflict dispute heads must be sorted and duplicate-free".into());
    }
    if dispute.evidence_cids.len() > 32 {
        return Err("Disputes may attach at most 32 evidence CIDs".into());
    }
    for cid in &dispute.evidence_cids {
        if !is_valid_ipfs_cid(cid) {
            return Err(format!("Invalid IPFS CID: {cid}"));
        }
    }
    Ok(())
}


fn validate_conflict_dispute_binding(dispute: &Dispute) -> ExternResult<Result<(), String>> {
    if dispute.conflict_heads.len() < 2 {
        return Ok(Err("Conflict dispute must bind at least two transaction heads".into()));
    }
    if dispute.transaction_revision_hash != dispute.transaction_hash {
        return Ok(Err(
            "Conflict disputes use the stable transaction root as transaction_revision_hash"
                .into(),
        ));
    }

    let root_record = must_get_valid_record(dispute.transaction_hash.clone())?;
    if !matches!(root_record.action(), Action::Create(_)) {
        return Ok(Err(
            "Conflict dispute transaction_hash must be the original Create action".into(),
        ));
    }
    let root: TransactionSnapshot = decode_record(&root_record)?;
    if root.buyer != dispute.buyer || root.seller != dispute.seller {
        return Ok(Err(
            "Conflict dispute parties do not match the transaction root".into(),
        ));
    }

    for head_hash in &dispute.conflict_heads {
        if find_action_root(head_hash.clone())? != dispute.transaction_hash {
            return Ok(Err(
                "Every conflict-dispute head must belong to the declared transaction root"
                    .into(),
            ));
        }
        let head: TransactionSnapshot =
            decode_record(&must_get_valid_record(head_hash.clone())?)?;
        if head.buyer != root.buyer
            || head.seller != root.seller
            || head.total_price_cents != root.total_price_cents
        {
            return Ok(Err(
                "Conflict-dispute heads must share the authoritative transaction terms".into(),
            ));
        }
    }

    Ok(Ok(()))
}

fn is_sorted_unique_hashes(hashes: &[ActionHash]) -> bool {
    hashes
        .windows(2)
        .all(|pair| pair[0].to_string() < pair[1].to_string())
}

fn validate_arbitrator_set(dispute: &Dispute) -> Result<(), String> {
    if dispute.arbitrators.is_empty() || dispute.arbitrators.len() > MAX_ARBITRATORS {
        return Err(format!(
            "Under-review disputes require 1-{MAX_ARBITRATORS} arbitrators"
        ));
    }
    let mut unique = Vec::new();
    for arbitrator in &dispute.arbitrators {
        if arbitrator == &dispute.buyer
            || arbitrator == &dispute.seller
            || arbitrator == &dispute.filed_by
        {
            return Err("Transaction parties cannot arbitrate their own dispute".into());
        }
        if unique.contains(arbitrator) {
            return Err("Arbitrator assignments must be unique".into());
        }
        unique.push(arbitrator.clone());
    }
    Ok(())
}

fn validate_vote_data(vote: &ArbitrationVote) -> Result<(), String> {
    if vote.reasoning.trim().is_empty() {
        return Err("Vote reasoning cannot be empty".into());
    }
    if vote.reasoning.len() > 2000 {
        return Err("Vote reasoning too long (max 2000 characters)".into());
    }
    if (vote.arbitrator_matl_score - 1.0).abs() > FLOAT_EPSILON {
        return Err(
            "Votes must use equal weight 1.0 until reputation snapshots are verifiably bound"
                .into(),
        );
    }
    Ok(())
}

fn validate_result_data(result: &ArbitrationResult) -> Result<(), String> {
    if !(0.0..=1.0).contains(&result.weighted_vote) {
        return Err("Invalid vote ratio (must be 0.0-1.0)".into());
    }
    if result.vote_hashes.is_empty() {
        return Err("Result must bind at least one vote".into());
    }
    if result.vote_hashes.len() > MAX_ARBITRATORS {
        return Err("Result contains more votes than the maximum arbitrator set".into());
    }
    if result.total_votes as usize != result.vote_hashes.len() {
        return Err("Result total_votes must equal the number of bound vote hashes".into());
    }
    let mut unique = Vec::new();
    for hash in &result.vote_hashes {
        if unique.contains(hash) {
            return Err("Result vote hashes must be unique".into());
        }
        unique.push(hash.clone());
    }
    if result.winner == result.loser {
        return Err("Arbitration winner and loser must be different".into());
    }
    if result.summary.trim().is_empty() || result.summary.len() > 2000 {
        return Err("Result summary must be 1-2000 characters".into());
    }
    Ok(())
}

fn recommended_compensation(transaction_value_cents: u64, buyer_vote_ratio: f64) -> u64 {
    let consensus_strength = if buyer_vote_ratio > 0.5 {
        buyer_vote_ratio
    } else {
        1.0 - buyer_vote_ratio
    };
    let basis_points = if consensus_strength >= 0.85 {
        10_000_u64
    } else if consensus_strength >= 0.75 {
        7_500_u64
    } else {
        5_000_u64
    };
    transaction_value_cents.saturating_mul(basis_points) / 10_000
}

fn decode_record<T>(record: &Record) -> ExternResult<T>
where
    T: TryFrom<SerializedBytes, Error = SerializedBytesError>,
{
    record
        .entry()
        .to_app_option()
        .map_err(|error| wasm_error!(error))?
        .ok_or_else(|| wasm_error!(WasmErrorInner::Guest("Application entry missing".into())))
}

fn find_action_root(mut cursor: ActionHash) -> ExternResult<ActionHash> {
    let mut visited = Vec::new();
    for _ in 0..=MAX_REVISION_DEPTH {
        if visited.contains(&cursor) {
            return Err(wasm_error!(WasmErrorInner::Guest(
                "Update ancestry contains a cycle".into()
            )));
        }
        visited.push(cursor.clone());
        let record = must_get_valid_record(cursor.clone())?;
        match record.action() {
            Action::Create(_) => return Ok(cursor),
            Action::Update(update) => cursor = update.original_action_address.clone(),
            _ => {
                return Err(wasm_error!(WasmErrorInner::Guest(
                    "Bound hash must reference a Create or Update action".into()
                )))
            }
        }
    }
    Err(wasm_error!(WasmErrorInner::Guest(
        "Update ancestry exceeds the depth limit".into()
    )))
}

/// Validate IPFS CID format (bounded syntactic check; content verification is external).
fn is_valid_ipfs_cid(cid: &str) -> bool {
    (cid.starts_with("Qm") && cid.len() == 46)
        || (cid.starts_with('b') && cid.len() >= 50 && cid.len() <= 100)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn mock_agent(byte: u8) -> AgentPubKey {
        AgentPubKey::from_raw_36(vec![byte; 36])
    }

    fn valid_dispute() -> Dispute {
        Dispute {
            transaction_hash: ActionHash::from_raw_36(vec![1; 36]),
            transaction_revision_hash: ActionHash::from_raw_36(vec![2; 36]),
            conflict_heads: Vec::new(),
            filed_by: mock_agent(3),
            buyer: mock_agent(3),
            seller: mock_agent(4),
            reason: "Item not as described".into(),
            evidence_cids: Vec::new(),
            status: DisputeStatus::Filed,
            arbitrators: Vec::new(),
            result_hash: None,
            created_at: Timestamp::from_micros(1_000_000),
            updated_at: Timestamp::from_micros(1_000_000),
        }
    }

    #[test]
    fn dispute_data_rejects_empty_reason_and_self_transaction() {
        let mut dispute = valid_dispute();
        dispute.reason = "  ".into();
        assert!(validate_dispute_data(&dispute).is_err());
        dispute.reason = "valid".into();
        dispute.seller = dispute.buyer.clone();
        assert!(validate_dispute_data(&dispute).is_err());
    }

    #[test]
    fn arbitrator_set_rejects_parties_and_duplicates() {
        let mut dispute = valid_dispute();
        dispute.arbitrators = vec![dispute.buyer.clone()];
        assert!(validate_arbitrator_set(&dispute).is_err());
        let arb = mock_agent(9);
        dispute.arbitrators = vec![arb.clone(), arb];
        assert!(validate_arbitrator_set(&dispute).is_err());
    }

    #[test]
    fn votes_are_equal_weight_until_snapshot_proofs_exist() {
        let vote = ArbitrationVote {
            dispute_hash: ActionHash::from_raw_36(vec![1; 36]),
            dispute_revision_hash: ActionHash::from_raw_36(vec![2; 36]),
            arbitrator: mock_agent(9),
            favor_buyer: true,
            reasoning: "Evidence favors the buyer".into(),
            arbitrator_matl_score: 1.0,
            voted_at: Timestamp::from_micros(2_000_000),
        };
        assert!(validate_vote_data(&vote).is_ok());
        assert!(validate_vote_data(&ArbitrationVote {
            arbitrator_matl_score: 0.9,
            ..vote
        })
        .is_err());
    }

    #[test]
    fn compensation_is_integer_and_deterministic() {
        assert_eq!(recommended_compensation(10_000, 1.0), 10_000);
        assert_eq!(recommended_compensation(10_000, 0.8), 7_500);
        assert_eq!(recommended_compensation(10_000, 2.0 / 3.0), 5_000);
    }

    #[test]
    fn result_data_requires_unique_bound_votes() {
        let hash = ActionHash::from_raw_36(vec![7; 36]);
        let result = ArbitrationResult {
            dispute_hash: ActionHash::from_raw_36(vec![1; 36]),
            dispute_revision_hash: ActionHash::from_raw_36(vec![2; 36]),
            vote_hashes: vec![hash.clone(), hash],
            winner: mock_agent(3),
            loser: mock_agent(4),
            weighted_vote: 1.0,
            total_votes: 2,
            compensation_cents: Some(10_000),
            summary: "Buyer wins".into(),
            finalized_at: Timestamp::from_micros(3_000_000),
        };
        assert!(validate_result_data(&result).is_err());
    }
}
