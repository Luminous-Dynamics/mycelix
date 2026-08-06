// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
use arbitration_integrity::*;
use hdk::prelude::*;
use mycelix_common::{error_handling, link_queries, remote_calls, time};

mod resolution;
pub use resolution::{DisputeResolution, DisputeResolutionState};

const MAX_ARBITRATORS: usize = 3;
const RESULT_THRESHOLD: f64 = 0.66;

/// File one idempotent dispute for a transaction.
///
/// The transaction must already have a single canonical Disputed revision. The
/// dispute stores both the stable transaction root and that exact revision so
/// integrity validation can prove the parties and lifecycle state.
#[hdk_extern]
pub fn file_dispute(input: FileDisputeInput) -> ExternResult<DisputeOutput> {
    let filer = agent_info()?.agent_initial_pubkey;
    let transaction_resolution = resolve_transaction(input.transaction_hash.clone())?;
    let transaction_root = transaction_resolution.root_transaction_hash.clone();
    let transaction = transaction_resolution.current()?;

    if filer != transaction.transaction.buyer && filer != transaction.transaction.seller {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Only buyer or seller can file dispute".into()
        )));
    }
    if transaction.transaction.status != TransactionStatusWire::Disputed {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Transaction must be in Disputed status before arbitration can be opened".into()
        )));
    }

    if let Some(existing) = get_single_dispute_for_transaction(transaction_root.clone())? {
        let root_dispute_hash = resolution::resolve_dispute(existing.dispute_hash.clone())?
            .ok_or_else(|| {
                wasm_error!(WasmErrorInner::Guest(
                    "Existing dispute disappeared during assignment".into()
                ))
            })?
            .root_dispute_hash;
        if existing.dispute.status == DisputeStatus::Filed
            && existing.dispute.arbitrators.is_empty()
        {
            return assign_arbitrators_internal(root_dispute_hash, existing);
        }
        if existing.dispute.status == DisputeStatus::UnderReview {
            ensure_opportunity_links(root_dispute_hash, &existing.dispute.arbitrators)?;
        }
        return Ok(existing);
    }

    let now = time::now()?;
    let dispute = Dispute {
        transaction_hash: transaction_root.clone(),
        transaction_revision_hash: transaction.transaction_hash.clone(),
        conflict_heads: Vec::new(),
        filed_by: filer,
        buyer: transaction.transaction.buyer.clone(),
        seller: transaction.transaction.seller.clone(),
        reason: input.reason,
        evidence_cids: input.evidence_cids,
        status: DisputeStatus::Filed,
        arbitrators: Vec::new(),
        result_hash: None,
        created_at: now,
        updated_at: now,
    };

    let root_dispute_hash = create_entry(&EntryTypes::Dispute(dispute.clone()))?;

    create_link(
        transaction_root,
        root_dispute_hash.clone(),
        LinkTypes::TransactionToDispute,
        (),
    )?;
    create_link(
        dispute.filed_by.clone(),
        root_dispute_hash.clone(),
        LinkTypes::AgentToFiledDisputes,
        (),
    )?;
    create_link(
        root_dispute_hash.clone(),
        root_dispute_hash.clone(),
        LinkTypes::AllDisputes,
        (),
    )?;

    let output = assign_arbitrators_internal(
        root_dispute_hash.clone(),
        DisputeOutput {
            dispute_hash: root_dispute_hash,
            dispute,
        },
    )?;

    monitoring::emit_metric(
        monitoring::MetricType::ArbitrationInitiated,
        1.0,
        Some(output.dispute.filed_by.clone()),
        Some(format!(
            "buyer:{:?},seller:{:?}",
            output.dispute.buyer, output.dispute.seller
        )),
    )?;

    Ok(output)
}

/// File or recover one arbitration case bound to an exact unsafe transaction conflict.
#[hdk_extern]
pub fn file_transaction_conflict_dispute(input: FileDisputeInput) -> ExternResult<DisputeOutput> {
    let filer = agent_info()?.agent_initial_pubkey;
    let transaction_resolution = resolve_transaction(input.transaction_hash.clone())?;
    if transaction_resolution.state != TransactionResolutionStateWire::Conflicted {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Conflict arbitration requires an unresolved transaction conflict".into(),
        )));
    }
    let transaction_root = transaction_resolution.root_transaction_hash.clone();
    let transaction = transaction_resolution.transaction_terms()?;
    if filer != transaction.transaction.buyer && filer != transaction.transaction.seller {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Only buyer or seller can file a transaction conflict dispute".into(),
        )));
    }
    let conflict_heads = transaction_resolution.head_hashes();
    if conflict_heads.len() < 2 {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Conflict arbitration requires at least two visible transaction heads".into(),
        )));
    }

    if let Some(existing) = get_single_dispute_for_transaction(transaction_root.clone())? {
        if existing.dispute.conflict_heads != conflict_heads {
            return Err(wasm_error!(WasmErrorInner::Guest(
                "A different dispute is already bound to this transaction".into(),
            )));
        }
        let root_dispute_hash = resolution::resolve_dispute(existing.dispute_hash.clone())?
            .ok_or_else(|| {
                wasm_error!(WasmErrorInner::Guest(
                    "Existing conflict dispute disappeared during recovery".into(),
                ))
            })?
            .root_dispute_hash;
        if existing.dispute.status == DisputeStatus::Filed
            && existing.dispute.arbitrators.is_empty()
        {
            return assign_arbitrators_internal(root_dispute_hash, existing);
        }
        if existing.dispute.status == DisputeStatus::UnderReview {
            ensure_opportunity_links(root_dispute_hash, &existing.dispute.arbitrators)?;
        }
        return Ok(existing);
    }

    let now = time::now()?;
    let dispute = Dispute {
        transaction_hash: transaction_root.clone(),
        transaction_revision_hash: transaction_root.clone(),
        conflict_heads,
        filed_by: filer,
        buyer: transaction.transaction.buyer.clone(),
        seller: transaction.transaction.seller.clone(),
        reason: input.reason,
        evidence_cids: input.evidence_cids,
        status: DisputeStatus::Filed,
        arbitrators: Vec::new(),
        result_hash: None,
        created_at: now,
        updated_at: now,
    };
    let root_dispute_hash = create_entry(&EntryTypes::Dispute(dispute.clone()))?;
    create_link(
        transaction_root,
        root_dispute_hash.clone(),
        LinkTypes::TransactionToDispute,
        (),
    )?;
    create_link(
        dispute.filed_by.clone(),
        root_dispute_hash.clone(),
        LinkTypes::AgentToFiledDisputes,
        (),
    )?;
    create_link(
        root_dispute_hash.clone(),
        root_dispute_hash.clone(),
        LinkTypes::AllDisputes,
        (),
    )?;
    assign_arbitrators_internal(
        root_dispute_hash.clone(),
        DisputeOutput {
            dispute_hash: root_dispute_hash,
            dispute,
        },
    )
}

fn arbitrator_pool_path() -> Path {
    Path::from("all_arbitrators")
}

/// Register the calling agent as available for assignment.
#[hdk_extern]
pub fn register_as_arbitrator(_: ()) -> ExternResult<()> {
    let agent = agent_info()?.agent_initial_pubkey;
    create_link(
        arbitrator_pool_path().path_entry_hash()?,
        agent,
        LinkTypes::AllArbitrators,
        (),
    )?;
    Ok(())
}

fn assign_arbitrators_internal(
    root_dispute_hash: ActionHash,
    current: DisputeOutput,
) -> ExternResult<DisputeOutput> {
    if current.dispute.status != DisputeStatus::Filed {
        return Ok(current);
    }

    let pool_links = get_links(
        LinkQuery::try_new(
            arbitrator_pool_path().path_entry_hash()?,
            LinkTypes::AllArbitrators,
        )?,
        GetStrategy::default(),
    )?;

    let mut eligible_arbitrators: Vec<AgentPubKey> = pool_links
        .into_iter()
        .filter_map(|link| AgentPubKey::try_from(link.target).ok())
        .filter(|candidate| {
            candidate != &current.dispute.buyer
                && candidate != &current.dispute.seller
                && candidate != &current.dispute.filed_by
        })
        .collect();

    // Assignment is deliberately reputation-neutral until an integrity-verifiable
    // score snapshot exists. Sorting makes the provisional coordinator policy
    // deterministic for a given visible registration set; integrity validation
    // still enforces uniqueness and party exclusion on the resulting case.
    eligible_arbitrators.sort_by_key(ToString::to_string);
    eligible_arbitrators.dedup();
    eligible_arbitrators.truncate(MAX_ARBITRATORS);

    if eligible_arbitrators.is_empty() {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "No eligible arbitrators are currently registered; the Filed dispute can be retried"
                .into()
        )));
    }

    let mut updated = current.dispute;
    updated.arbitrators = eligible_arbitrators.clone();
    updated.status = DisputeStatus::UnderReview;
    updated.updated_at = time::now()?;
    let current_hash = update_entry(current.dispute_hash, &updated)?;

    ensure_opportunity_links(root_dispute_hash, &eligible_arbitrators)?;

    Ok(DisputeOutput {
        dispute_hash: current_hash,
        dispute: updated,
    })
}

fn ensure_opportunity_links(
    root_dispute_hash: ActionHash,
    arbitrators: &[AgentPubKey],
) -> ExternResult<()> {
    for arbitrator in arbitrators {
        let existing = link_queries::get_links_local(
            arbitrator.clone(),
            LinkTypes::AgentToArbitrationOpportunities,
        )?;
        let already_linked = existing.into_iter().any(|link| {
            link.target
                .into_action_hash()
                .map(|target| target == root_dispute_hash)
                .unwrap_or(false)
        });
        if !already_linked {
            create_link(
                arbitrator.clone(),
                root_dispute_hash.clone(),
                LinkTypes::AgentToArbitrationOpportunities,
                (),
            )?;
        }
    }
    Ok(())
}

/// Submit one immutable, equal-weight vote.
///
/// Assignment and vote weight are reputation-neutral until a cryptographically
/// bound reputation snapshot can be validated by the integrity zome.
#[hdk_extern]
pub fn submit_arbitration_vote(
    input: SubmitArbitrationVoteInput,
) -> ExternResult<ArbitrationVoteOutput> {
    let arbitrator = agent_info()?.agent_initial_pubkey;
    let resolution = resolution::resolve_dispute(input.dispute_hash.clone())?
        .ok_or_else(|| wasm_error!(WasmErrorInner::Guest("Dispute not found".into())))?;
    let root_dispute_hash = resolution.root_dispute_hash.clone();
    let current = resolution
        .into_resolved()
        .map_err(|reason| wasm_error!(WasmErrorInner::Guest(reason)))?;

    if !matches!(
        current.dispute.status,
        DisputeStatus::UnderReview | DisputeStatus::Voting
    ) {
        return Err(wasm_error!(WasmErrorInner::Guest(format!(
            "Cannot vote on dispute with status {:?}",
            current.dispute.status
        ))));
    }
    if !current.dispute.arbitrators.contains(&arbitrator) {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Not an assigned arbitrator for this dispute".into()
        )));
    }

    let existing_votes = get_dispute_votes(root_dispute_hash.clone())?;
    if existing_votes
        .iter()
        .any(|output| output.vote.arbitrator == arbitrator)
    {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Arbitrator has already voted on this dispute".into()
        )));
    }

    let vote = ArbitrationVote {
        dispute_hash: root_dispute_hash.clone(),
        dispute_revision_hash: current.dispute_hash.clone(),
        arbitrator: arbitrator.clone(),
        favor_buyer: input.favor_buyer,
        reasoning: input.reasoning,
        arbitrator_matl_score: 1.0,
        voted_at: time::now()?,
    };
    let vote_hash = create_entry(&EntryTypes::ArbitrationVote(vote.clone()))?;
    create_link(
        root_dispute_hash.clone(),
        vote_hash.clone(),
        LinkTypes::DisputeToVotes,
        (),
    )?;

    let all_votes = get_dispute_votes(root_dispute_hash)?;
    if all_votes.len() == current.dispute.arbitrators.len()
        && current.dispute.status == DisputeStatus::UnderReview
    {
        let mut updated_dispute = current.dispute;
        updated_dispute.status = DisputeStatus::Voting;
        updated_dispute.updated_at = time::now()?;
        update_entry(current.dispute_hash, &updated_dispute)?;
    }

    Ok(ArbitrationVoteOutput { vote_hash, vote })
}

/// Finalize a dispute from exactly one vote per assigned arbitrator.
#[hdk_extern]
pub fn finalize_arbitration(dispute_hash: ActionHash) -> ExternResult<ArbitrationResultOutput> {
    let caller = agent_info()?.agent_initial_pubkey;
    let resolution = resolution::resolve_dispute(dispute_hash)?
        .ok_or_else(|| wasm_error!(WasmErrorInner::Guest("Dispute not found".into())))?;
    let root_dispute_hash = resolution.root_dispute_hash.clone();
    let current = resolution
        .into_resolved()
        .map_err(|reason| wasm_error!(WasmErrorInner::Guest(reason)))?;

    if !current.dispute.arbitrators.contains(&caller) {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Only an assigned arbitrator may finalize this dispute".into()
        )));
    }

    if let Some(existing) = get_single_result(root_dispute_hash.clone())? {
        match current.dispute.status {
            DisputeStatus::Voting => {
                bind_result_if_needed(current, existing.result_hash.clone(), &existing.result)?;
            }
            DisputeStatus::ResolvedBuyer | DisputeStatus::ResolvedSeller => {
                if current.dispute.result_hash.as_ref() != Some(&existing.result_hash) {
                    return Err(wasm_error!(WasmErrorInner::Guest(
                        "Resolved dispute does not bind the unique visible result".into()
                    )));
                }
            }
            _ => {
                return Err(wasm_error!(WasmErrorInner::Guest(format!(
                    "A result exists for a dispute in unexpected status {:?}",
                    current.dispute.status
                ))));
            }
        }
        project_result_reputation(existing.result_hash.clone())?;
        return Ok(existing);
    }

    if current.dispute.status != DisputeStatus::Voting {
        return Err(wasm_error!(WasmErrorInner::Guest(format!(
            "Cannot finalize dispute with status {:?}",
            current.dispute.status
        ))));
    }

    let votes = get_dispute_votes(root_dispute_hash.clone())?;
    if votes.len() != current.dispute.arbitrators.len() {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Finalization requires exactly one vote from every assigned arbitrator".into()
        )));
    }

    let buyer_votes = votes
        .iter()
        .filter(|output| output.vote.favor_buyer)
        .count();
    let buyer_vote_ratio = buyer_votes as f64 / votes.len() as f64;
    let buyer_wins = buyer_vote_ratio > RESULT_THRESHOLD;
    let (winner, loser, status) = if buyer_wins {
        (
            current.dispute.buyer.clone(),
            current.dispute.seller.clone(),
            DisputeStatus::ResolvedBuyer,
        )
    } else {
        (
            current.dispute.seller.clone(),
            current.dispute.buyer.clone(),
            DisputeStatus::ResolvedSeller,
        )
    };

    let transaction_resolution = resolve_transaction(current.dispute.transaction_hash.clone())?;
    let transaction = transaction_resolution.transaction_terms()?;
    let compensation_cents =
        recommended_compensation(transaction.transaction.total_price_cents, buyer_vote_ratio);
    let vote_hashes = votes
        .iter()
        .map(|output| output.vote_hash.clone())
        .collect::<Vec<_>>();

    let result = ArbitrationResult {
        dispute_hash: root_dispute_hash.clone(),
        dispute_revision_hash: current.dispute_hash.clone(),
        vote_hashes,
        winner: winner.clone(),
        loser,
        weighted_vote: buyer_vote_ratio,
        total_votes: votes.len() as u32,
        compensation_cents: Some(compensation_cents),
        summary: format!(
            "Resolved in favor of {} with {} of {} equal-weight votes. Recommended compensation: {} cents.",
            if buyer_wins { "buyer" } else { "seller" },
            if buyer_wins {
                buyer_votes
            } else {
                votes.len() - buyer_votes
            },
            votes.len(),
            compensation_cents
        ),
        finalized_at: time::now()?,
    };
    let result_hash = create_entry(&EntryTypes::ArbitrationResult(result.clone()))?;
    create_link(
        root_dispute_hash,
        result_hash.clone(),
        LinkTypes::DisputeToResult,
        (),
    )?;

    let mut updated_dispute = current.dispute;
    updated_dispute.status = status;
    updated_dispute.result_hash = Some(result_hash.clone());
    updated_dispute.updated_at = time::now()?;
    update_entry(current.dispute_hash, &updated_dispute)?;

    // Project immutable winner/loser evidence. If this call fails after the
    // result is durable, retrying finalization recovers the same result and
    // idempotently retries the projection.
    project_result_reputation(result_hash.clone())?;
    Ok(ArbitrationResultOutput {
        result_hash,
        result,
    })
}

fn project_result_reputation(result_hash: ActionHash) -> ExternResult<()> {
    let response: ReputationEventsResponseWire =
        remote_calls::call_zome("reputation", "project_arbitration_reputation", result_hash)?;
    if response.events.len() != 2 {
        return Err(wasm_error!(WasmErrorInner::Guest(format!(
            "Arbitration reputation projection returned {} events; expected winner and loser",
            response.events.len()
        ))));
    }
    Ok(())
}

/// Get current arbitration opportunities for the calling agent.
#[hdk_extern]
pub fn get_arbitration_opportunities(_: ()) -> ExternResult<DisputesResponse> {
    let agent = agent_info()?.agent_initial_pubkey;
    let links =
        link_queries::get_links_local(agent.clone(), LinkTypes::AgentToArbitrationOpportunities)?;
    let mut roots = Vec::new();
    let mut disputes = Vec::new();

    for link in links {
        let Some(hash) = link.target.into_action_hash() else {
            continue;
        };
        let Some(resolution) = resolution::resolve_dispute(hash)? else {
            continue;
        };
        if roots.contains(&resolution.root_dispute_hash) {
            continue;
        }
        let root = resolution.root_dispute_hash.clone();
        let Ok(current) = resolution.into_resolved() else {
            continue;
        };
        if current.dispute.status != DisputeStatus::UnderReview
            || !current.dispute.arbitrators.contains(&agent)
        {
            continue;
        }
        let votes = get_dispute_votes(root.clone())?;
        if votes.iter().any(|output| output.vote.arbitrator == agent) {
            continue;
        }
        roots.push(root);
        disputes.push(current);
    }

    Ok(DisputesResponse { disputes })
}

/// Get the sole current dispute revision. Conflicts fail closed.
#[hdk_extern]
pub fn get_dispute(dispute_hash: ActionHash) -> ExternResult<Option<DisputeOutput>> {
    let Some(resolution) = resolution::resolve_dispute(dispute_hash)? else {
        return Ok(None);
    };
    resolution
        .into_resolved()
        .map(Some)
        .map_err(|reason| wasm_error!(WasmErrorInner::Guest(reason)))
}

/// Get the complete conflict-aware dispute update tree.
#[hdk_extern]
pub fn get_dispute_resolution(dispute_hash: ActionHash) -> ExternResult<Option<DisputeResolution>> {
    resolution::resolve_dispute(dispute_hash)
}

/// Get all validated votes for a dispute, failing on duplicate arbitrators.
#[hdk_extern]
pub fn get_arbitration_votes(dispute_hash: ActionHash) -> ExternResult<ArbitrationVotesResponse> {
    let resolution = resolution::resolve_dispute(dispute_hash)?
        .ok_or_else(|| wasm_error!(WasmErrorInner::Guest("Dispute not found".into())))?;
    Ok(ArbitrationVotesResponse {
        votes: get_dispute_votes(resolution.root_dispute_hash)?,
    })
}

/// Get the unique arbitration result, if finalized.
#[hdk_extern]
pub fn get_arbitration_result(
    dispute_hash: ActionHash,
) -> ExternResult<Option<ArbitrationResultOutput>> {
    let resolution = resolution::resolve_dispute(dispute_hash)?
        .ok_or_else(|| wasm_error!(WasmErrorInner::Guest("Dispute not found".into())))?;
    get_single_result(resolution.root_dispute_hash)
}

fn get_single_dispute_for_transaction(
    transaction_root: ActionHash,
) -> ExternResult<Option<DisputeOutput>> {
    let links = link_queries::get_links_local(transaction_root, LinkTypes::TransactionToDispute)?;
    let mut roots = Vec::new();
    let mut current = None;

    for link in links {
        let Some(hash) = link.target.into_action_hash() else {
            continue;
        };
        let Some(resolution) = resolution::resolve_dispute(hash)? else {
            continue;
        };
        if roots.contains(&resolution.root_dispute_hash) {
            continue;
        }
        roots.push(resolution.root_dispute_hash.clone());
        current = Some(
            resolution
                .into_resolved()
                .map_err(|reason| wasm_error!(WasmErrorInner::Guest(reason)))?,
        );
    }

    match roots.len() {
        0 => Ok(None),
        1 => Ok(current),
        count => Err(wasm_error!(WasmErrorInner::Guest(format!(
            "Transaction has {count} distinct dispute roots; refusing to choose one"
        )))),
    }
}

fn get_dispute_votes(root_dispute_hash: ActionHash) -> ExternResult<Vec<ArbitrationVoteOutput>> {
    let links =
        link_queries::get_links_local(root_dispute_hash.clone(), LinkTypes::DisputeToVotes)?;
    let mut votes = Vec::new();
    let mut arbitrators = Vec::new();

    for link in links {
        let Some(action_hash) = link.target.into_action_hash() else {
            continue;
        };
        let Some(record) = get(action_hash.clone(), GetOptions::default())? else {
            continue;
        };
        let vote: ArbitrationVote = error_handling::deserialize_entry(&record)?;
        if vote.dispute_hash != root_dispute_hash {
            return Err(wasm_error!(WasmErrorInner::Guest(
                "Dispute vote link targets a vote from another dispute".into()
            )));
        }
        if arbitrators.contains(&vote.arbitrator) {
            return Err(wasm_error!(WasmErrorInner::Guest(format!(
                "Duplicate votes are visible for arbitrator {}",
                vote.arbitrator
            ))));
        }
        arbitrators.push(vote.arbitrator.clone());
        votes.push(ArbitrationVoteOutput {
            vote_hash: action_hash,
            vote,
        });
    }

    votes.sort_by(|left, right| {
        left.vote
            .arbitrator
            .to_string()
            .cmp(&right.vote.arbitrator.to_string())
    });
    Ok(votes)
}

fn get_single_result(
    root_dispute_hash: ActionHash,
) -> ExternResult<Option<ArbitrationResultOutput>> {
    let links =
        link_queries::get_links_local(root_dispute_hash.clone(), LinkTypes::DisputeToResult)?;
    let mut results = Vec::new();

    for link in links {
        let Some(action_hash) = link.target.into_action_hash() else {
            continue;
        };
        let Some(record) = get(action_hash.clone(), GetOptions::default())? else {
            continue;
        };
        let result: ArbitrationResult = error_handling::deserialize_entry(&record)?;
        if result.dispute_hash != root_dispute_hash {
            return Err(wasm_error!(WasmErrorInner::Guest(
                "Dispute result link targets a result from another dispute".into()
            )));
        }
        results.push(ArbitrationResultOutput {
            result_hash: action_hash,
            result,
        });
    }

    match results.len() {
        0 => Ok(None),
        1 => Ok(results.pop()),
        count => Err(wasm_error!(WasmErrorInner::Guest(format!(
            "Dispute has {count} arbitration results; refusing to choose one"
        )))),
    }
}

fn bind_result_if_needed(
    current: DisputeOutput,
    result_hash: ActionHash,
    result: &ArbitrationResult,
) -> ExternResult<()> {
    if current.dispute.result_hash.as_ref() == Some(&result_hash) {
        return Ok(());
    }
    let mut updated = current.dispute;
    updated.status = if result.winner == updated.buyer {
        DisputeStatus::ResolvedBuyer
    } else if result.winner == updated.seller {
        DisputeStatus::ResolvedSeller
    } else {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Existing result winner is not a dispute party".into()
        )));
    };
    updated.result_hash = Some(result_hash);
    updated.updated_at = time::now()?;
    update_entry(current.dispute_hash, &updated)?;
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

fn resolve_transaction(transaction_hash: ActionHash) -> ExternResult<TransactionResolutionWire> {
    let response: Option<TransactionResolutionWire> = remote_calls::call_zome(
        "transactions",
        "get_transaction_resolution",
        transaction_hash,
    )?;
    response.ok_or_else(|| wasm_error!(WasmErrorInner::Guest("Transaction not found".into())))
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct FileDisputeInput {
    pub transaction_hash: ActionHash,
    pub reason: String,
    pub evidence_cids: Vec<String>,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct DisputeOutput {
    /// Current Create/Update action hash.
    pub dispute_hash: ActionHash,
    pub dispute: Dispute,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct DisputesResponse {
    pub disputes: Vec<DisputeOutput>,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct SubmitArbitrationVoteInput {
    pub dispute_hash: ActionHash,
    pub favor_buyer: bool,
    pub reasoning: String,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ArbitrationVoteOutput {
    pub vote_hash: ActionHash,
    pub vote: ArbitrationVote,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ArbitrationVotesResponse {
    pub votes: Vec<ArbitrationVoteOutput>,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ArbitrationResultOutput {
    pub result_hash: ActionHash,
    pub result: ArbitrationResult,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
enum TransactionResolutionStateWire {
    Resolved,
    AutoResolved,
    AuthorizedResolved,
    Conflicted,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
struct TransactionResolutionWire {
    root_transaction_hash: ActionHash,
    state: TransactionResolutionStateWire,
    canonical: Option<TransactionOutputWire>,
    heads: Vec<TransactionOutputWire>,
    revision_count: u32,
}

impl TransactionResolutionWire {
    fn current(&self) -> ExternResult<&TransactionOutputWire> {
        match (&self.state, &self.canonical) {
            (
                TransactionResolutionStateWire::Resolved
                | TransactionResolutionStateWire::AutoResolved
                | TransactionResolutionStateWire::AuthorizedResolved,
                Some(current),
            ) => Ok(current),
            (
                TransactionResolutionStateWire::Resolved
                | TransactionResolutionStateWire::AutoResolved
                | TransactionResolutionStateWire::AuthorizedResolved,
                None,
            ) => Err(wasm_error!(WasmErrorInner::Guest(
                "Resolved transaction omitted its current revision".into(),
            ))),
            (TransactionResolutionStateWire::Conflicted, _) => {
                Err(wasm_error!(WasmErrorInner::Guest(format!(
                    "Transaction has {} concurrent heads",
                    self.heads.len()
                ))))
            }
        }
    }

    fn transaction_terms(&self) -> ExternResult<&TransactionOutputWire> {
        if self.state != TransactionResolutionStateWire::Conflicted {
            return self.current();
        }
        let first = self.heads.first().ok_or_else(|| {
            wasm_error!(WasmErrorInner::Guest(
                "Conflicted transaction omitted its raw heads".into(),
            ))
        })?;
        if self.heads.iter().skip(1).any(|head| {
            head.transaction.buyer != first.transaction.buyer
                || head.transaction.seller != first.transaction.seller
                || head.transaction.total_price_cents != first.transaction.total_price_cents
        }) {
            return Err(wasm_error!(WasmErrorInner::Guest(
                "Conflicted transaction heads disagree on immutable terms".into(),
            )));
        }
        Ok(first)
    }

    fn head_hashes(&self) -> Vec<ActionHash> {
        let mut hashes = self
            .heads
            .iter()
            .map(|head| head.transaction_hash.clone())
            .collect::<Vec<_>>();
        hashes.sort_by_key(ToString::to_string);
        hashes.dedup();
        hashes
    }
}

#[derive(Serialize, Deserialize, Debug, Clone)]
struct TransactionOutputWire {
    transaction_hash: ActionHash,
    transaction: TransactionWire,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
struct TransactionWire {
    buyer: AgentPubKey,
    seller: AgentPubKey,
    total_price_cents: u64,
    status: TransactionStatusWire,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
enum TransactionStatusWire {
    Pending,
    Confirmed,
    Shipped,
    Delivered,
    Completed,
    Disputed,
    Cancelled,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
struct ReputationEventsResponseWire {
    events: Vec<ReputationEventOutputWire>,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
struct ReputationEventOutputWire {
    event_hash: ActionHash,
}

#[cfg(test)]
mod tests;
