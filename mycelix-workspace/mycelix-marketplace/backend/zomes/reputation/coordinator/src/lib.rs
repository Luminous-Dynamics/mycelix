// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
use hdk::prelude::*;
use mycelix_common::{bridge, error_handling, link_queries, remote_calls, time};
use reputation_integrity::*;

mod cache;

/// Get or initialize MATL score for an agent
///
/// Legacy MATL lookup retained for compatibility.
/// Canonical new reputation is returned by `get_derived_reputation`.
#[hdk_extern]
pub fn get_agent_matl_score(agent: AgentPubKey) -> ExternResult<Option<MatlScore>> {
    let path = agent.clone();
    // Use shared utility for get_links
    let links = link_queries::get_links_local(path, LinkTypes::AgentToScore)?;

    if let Some(link) = links.first() {
        if let Some(action_hash) = link.target.clone().into_action_hash() {
            let record = get(action_hash, GetOptions::default())?;
            if let Some(record) = record {
                // Use shared utility for deserialization
                let score: MatlScore = error_handling::deserialize_entry(&record)?;
                if score.agent != agent {
                    return Err(wasm_error!(WasmErrorInner::Guest(
                        "AgentToScore link targets a score for another agent".into()
                    )));
                }
                return Ok(Some(score));
            }
        }
    }

    // No score exists - return None (will be initialized on first transaction)
    Ok(None)
}

/// Get MATL score with caching (10-100x faster)
///
/// This is the recommended method for querying MATL scores.
/// Uses intelligent caching with TTL and automatic invalidation.
#[hdk_extern]
pub fn get_agent_matl_score_fast(agent: AgentPubKey) -> ExternResult<MatlScore> {
    cache::get_agent_matl_score_cached(agent)
}

/// Get combined reputation score (local MATL + cross-app reputation)
///
/// This function implements cross-app reputation sharing via the Bridge zome.
/// Formula: combined = (local_rep * 0.6) + (cross_app_rep * 0.4)
///
/// If Bridge is unavailable, returns local reputation only.
#[hdk_extern]
pub fn get_combined_reputation(agent: AgentPubKey) -> ExternResult<CombinedReputationResponse> {
    // Get local MATL score
    let local_score = match get_agent_matl_score(agent.clone())? {
        Some(score) => score,
        None => {
            // No local score - return neutral with cross-app check
            let did = bridge::agent_to_did(&agent);
            let (combined, used_cross_app) = bridge::compute_combined_reputation(0.5, &did);
            return Ok(CombinedReputationResponse {
                local_score: 0.5,
                cross_app_score: if used_cross_app { Some(combined) } else { None },
                combined_score: combined,
                cross_app_available: used_cross_app,
                transaction_count: 0,
            });
        }
    };

    // Convert agent to DID for cross-app lookup
    let did = bridge::agent_to_did(&agent);

    // Get cross-app reputation from Bridge
    let (combined_score, cross_app_available) =
        bridge::compute_combined_reputation(local_score.composite, &did);

    // Get cross-app score directly for response
    let cross_app_score = match bridge::get_cross_app_reputation(&did) {
        bridge::BridgeResult::Success(rep) => Some(rep.score),
        _ => None,
    };

    Ok(CombinedReputationResponse {
        local_score: local_score.composite,
        cross_app_score,
        combined_score,
        cross_app_available,
        transaction_count: local_score.transaction_count,
    })
}

/// Get cross-app reputation from Bridge zome
///
/// This directly queries the Bridge for cross-app reputation data.
/// Useful for UI display of reputation breakdown.
#[hdk_extern]
pub fn get_cross_app_reputation(agent: AgentPubKey) -> ExternResult<CrossAppReputationResponse> {
    let did = bridge::agent_to_did(&agent);

    match bridge::get_cross_app_reputation(&did) {
        bridge::BridgeResult::Success(rep) => Ok(CrossAppReputationResponse {
            available: true,
            did: rep.did,
            score: Some(rep.score),
            app_count: Some(rep.app_count),
            total_transactions: Some(rep.total_transactions),
            error: None,
        }),
        bridge::BridgeResult::Unavailable => Ok(CrossAppReputationResponse {
            available: false,
            did,
            score: None,
            app_count: None,
            total_transactions: None,
            error: Some("Bridge zome unavailable".to_string()),
        }),
        bridge::BridgeResult::Error(e) => Ok(CrossAppReputationResponse {
            available: false,
            did,
            score: None,
            app_count: None,
            total_transactions: None,
            error: Some(e),
        }),
    }
}

/// Legacy mutable score updates are disabled.
///
/// Reputation must be projected from immutable, integrity-validated evidence
/// through `record_fulfillment_reputation` or
/// `project_arbitration_reputation`.
#[hdk_extern]
pub fn update_matl_score(_input: UpdateMatlInput) -> ExternResult<MatlScore> {
    Err(wasm_error!(WasmErrorInner::Guest(
        "Direct MATL mutation is disabled; use evidence-derived reputation events".into()
    )))
}

/// Record one buyer-authored, Delivered-transaction fulfillment event for
/// the seller. Repeating the call returns the existing event by semantic key.
#[hdk_extern]
pub fn record_fulfillment_reputation(
    transaction_hash: ActionHash,
) -> ExternResult<ReputationEventOutput> {
    let resolution: Option<TransactionResolutionWire> = remote_calls::call_zome(
        "transactions",
        "get_transaction_resolution",
        transaction_hash,
    )?;
    let resolution = resolution
        .ok_or_else(|| wasm_error!(WasmErrorInner::Guest("Transaction not found".into())))?;
    let current = resolution.current()?;
    if current.transaction.status != TransactionStatusWire::Delivered {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Fulfillment reputation requires a Delivered transaction".into()
        )));
    }
    let caller = agent_info()?.agent_initial_pubkey;
    if caller != current.transaction.buyer {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Only the buyer may attest delivered fulfillment".into()
        )));
    }

    let event = ReputationEvent {
        event_key: fulfillment_event_key(
            &resolution.root_transaction_hash,
            &current.transaction.seller,
        ),
        subject: current.transaction.seller.clone(),
        counterparty: current.transaction.buyer.clone(),
        transaction_hash: resolution.root_transaction_hash,
        source_hash: current.transaction_hash.clone(),
        kind: ReputationEventKind::FulfillmentDelivered,
        value_cents: current.transaction.total_price_cents,
        occurred_at: current.transaction.updated_at,
    };
    create_or_get_reputation_event(event)
}

/// Project winner and loser events from one immutable ArbitrationResult.
/// Any assigned arbitrator can recover this projection; retries are idempotent
/// by semantic event key and integrity validation binds the source result.
#[hdk_extern]
pub fn project_arbitration_reputation(
    result_hash: ActionHash,
) -> ExternResult<ReputationEventsResponse> {
    let result_record = get(result_hash.clone(), GetOptions::default())?
        .ok_or_else(|| wasm_error!(WasmErrorInner::Guest("Arbitration result not found".into())))?;
    let caller = agent_info()?.agent_initial_pubkey;
    let result: ArbitrationResultWire = error_handling::deserialize_entry(&result_record)?;
    let dispute_record = get(result.dispute_revision_hash.clone(), GetOptions::default())?
        .ok_or_else(|| wasm_error!(WasmErrorInner::Guest("Dispute revision not found".into())))?;
    let dispute: DisputeWire = error_handling::deserialize_entry(&dispute_record)?;
    if !dispute.arbitrators.contains(&caller) {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Only an assigned arbitrator may project arbitration reputation".into()
        )));
    }
    let transaction_record = get(
        dispute.transaction_revision_hash.clone(),
        GetOptions::default(),
    )?
    .ok_or_else(|| {
        wasm_error!(WasmErrorInner::Guest(
            "Transaction revision not found".into()
        ))
    })?;
    let transaction: TransactionEvidenceWire =
        error_handling::deserialize_entry(&transaction_record)?;

    if dispute.transaction_hash != transaction_root(dispute.transaction_revision_hash.clone())? {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Dispute transaction revision does not belong to its stable root".into()
        )));
    }
    if transaction.buyer != dispute.buyer || transaction.seller != dispute.seller {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Dispute parties do not match the transaction".into()
        )));
    }

    let winner = ReputationEvent {
        event_key: arbitration_event_key(
            &result_hash,
            &ReputationEventKind::ArbitrationWon,
            &result.winner,
        ),
        subject: result.winner.clone(),
        counterparty: result.loser.clone(),
        transaction_hash: dispute.transaction_hash.clone(),
        source_hash: result_hash.clone(),
        kind: ReputationEventKind::ArbitrationWon,
        value_cents: transaction.total_price_cents,
        occurred_at: result.finalized_at,
    };
    let loser = ReputationEvent {
        event_key: arbitration_event_key(
            &result_hash,
            &ReputationEventKind::ArbitrationLost,
            &result.loser,
        ),
        subject: result.loser.clone(),
        counterparty: result.winner,
        transaction_hash: dispute.transaction_hash,
        source_hash: result_hash,
        kind: ReputationEventKind::ArbitrationLost,
        value_cents: transaction.total_price_cents,
        occurred_at: result.finalized_at,
    };

    Ok(ReputationEventsResponse {
        events: vec![
            create_or_get_reputation_event(winner)?,
            create_or_get_reputation_event(loser)?,
        ],
    })
}

#[hdk_extern]
pub fn get_agent_reputation_events(agent: AgentPubKey) -> ExternResult<ReputationEventsResponse> {
    let links = link_queries::get_links_local(agent.clone(), LinkTypes::AgentToReputationEvents)?;
    let mut by_key: HashMap<String, ReputationEventOutput> = HashMap::new();
    for link in links {
        let Some(action_hash) = link.target.into_action_hash() else {
            continue;
        };
        let Some(record) = get(action_hash.clone(), GetOptions::default())? else {
            continue;
        };
        let event: ReputationEvent = error_handling::deserialize_entry(&record)?;
        if event.subject != agent {
            return Err(wasm_error!(WasmErrorInner::Guest(
                "Agent reputation link targets an event for another subject".into()
            )));
        }
        let output = ReputationEventOutput {
            event_hash: action_hash,
            event,
        };
        if let Some(existing) = by_key.get(&output.event.event_key) {
            if existing.event != output.event {
                return Err(wasm_error!(WasmErrorInner::Guest(format!(
                    "Conflicting reputation events share key {}",
                    output.event.event_key
                ))));
            }
            continue;
        }
        by_key.insert(output.event.event_key.clone(), output);
    }
    let mut events = by_key.into_values().collect::<Vec<_>>();
    events.sort_by_key(|output| output.event.occurred_at);
    Ok(ReputationEventsResponse { events })
}

#[hdk_extern]
pub fn get_derived_reputation(agent: AgentPubKey) -> ExternResult<DerivedReputation> {
    let events = get_agent_reputation_events(agent.clone())?.events;
    let mut positive_events = 0_u32;
    let mut negative_events = 0_u32;
    let mut fulfilled_value_cents = 0_u64;
    let mut arbitration_value_cents = 0_u64;

    for output in &events {
        match output.event.kind {
            ReputationEventKind::FulfillmentDelivered => {
                positive_events += 1;
                fulfilled_value_cents =
                    fulfilled_value_cents.saturating_add(output.event.value_cents);
            }
            ReputationEventKind::ArbitrationWon => {
                positive_events += 1;
                arbitration_value_cents =
                    arbitration_value_cents.saturating_add(output.event.value_cents);
            }
            ReputationEventKind::ArbitrationLost => {
                negative_events += 1;
                arbitration_value_cents =
                    arbitration_value_cents.saturating_add(output.event.value_cents);
            }
        }
    }

    // Transparent Laplace-smoothed evidence ratio. This is not a Byzantine
    // tolerance claim and is never used as an integrity authorization weight.
    let score =
        (positive_events as f64 + 1.0) / (positive_events as f64 + negative_events as f64 + 2.0);
    Ok(DerivedReputation {
        agent,
        score,
        positive_events,
        negative_events,
        event_count: events.len() as u32,
        fulfilled_value_cents,
        arbitration_value_cents,
    })
}

fn create_or_get_reputation_event(event: ReputationEvent) -> ExternResult<ReputationEventOutput> {
    let links = link_queries::get_links_local(
        event.source_hash.clone(),
        LinkTypes::SourceToReputationEvents,
    )?;
    let mut matching = Vec::new();
    for link in links {
        let Some(action_hash) = link.target.into_action_hash() else {
            continue;
        };
        let Some(record) = get(action_hash.clone(), GetOptions::default())? else {
            continue;
        };
        let existing: ReputationEvent = error_handling::deserialize_entry(&record)?;
        if existing.event_key == event.event_key {
            matching.push(ReputationEventOutput {
                event_hash: action_hash,
                event: existing,
            });
        }
    }
    if matching.len() > 1 {
        let first = &matching[0].event;
        if matching.iter().any(|output| &output.event != first) {
            return Err(wasm_error!(WasmErrorInner::Guest(format!(
                "Conflicting reputation events share key {}",
                event.event_key
            ))));
        }
    }
    if let Some(existing) = matching.into_iter().next() {
        if existing.event != event {
            return Err(wasm_error!(WasmErrorInner::Guest(format!(
                "Reputation event key {} was reused with different evidence",
                event.event_key
            ))));
        }
        return Ok(existing);
    }

    let event_hash = create_entry(&EntryTypes::ReputationEvent(event.clone()))?;
    create_link(
        event.subject.clone(),
        event_hash.clone(),
        LinkTypes::AgentToReputationEvents,
        (),
    )?;
    create_link(
        event.source_hash.clone(),
        event_hash.clone(),
        LinkTypes::SourceToReputationEvents,
        (),
    )?;
    Ok(ReputationEventOutput { event_hash, event })
}

fn fulfillment_event_key(transaction_hash: &ActionHash, seller: &AgentPubKey) -> String {
    format!("fulfillment:{transaction_hash}:seller:{seller}")
}

fn arbitration_event_key(
    result_hash: &ActionHash,
    kind: &ReputationEventKind,
    subject: &AgentPubKey,
) -> String {
    let kind = match kind {
        ReputationEventKind::ArbitrationWon => "won",
        ReputationEventKind::ArbitrationLost => "lost",
        ReputationEventKind::FulfillmentDelivered => "fulfillment",
    };
    format!("arbitration:{result_hash}:{kind}:{subject}")
}

fn transaction_root(mut cursor: ActionHash) -> ExternResult<ActionHash> {
    let mut visited = Vec::new();
    for _ in 0..=32 {
        if visited.contains(&cursor) {
            return Err(wasm_error!(WasmErrorInner::Guest(
                "Transaction update ancestry contains a cycle".into()
            )));
        }
        visited.push(cursor.clone());
        let record = get(cursor.clone(), GetOptions::default())?.ok_or_else(|| {
            wasm_error!(WasmErrorInner::Guest("Transaction action missing".into()))
        })?;
        match record.action() {
            Action::Create(_) => return Ok(cursor),
            Action::Update(update) => cursor = update.original_action_address.clone(),
            _ => {
                return Err(wasm_error!(WasmErrorInner::Guest(
                    "Transaction source must be a Create or Update action".into()
                )));
            }
        }
    }
    Err(wasm_error!(WasmErrorInner::Guest(
        "Transaction update ancestry exceeds the depth limit".into()
    )))
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ReputationEventOutput {
    pub event_hash: ActionHash,
    pub event: ReputationEvent,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ReputationEventsResponse {
    pub events: Vec<ReputationEventOutput>,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct DerivedReputation {
    pub agent: AgentPubKey,
    pub score: f64,
    pub positive_events: u32,
    pub negative_events: u32,
    pub event_count: u32,
    pub fulfilled_value_cents: u64,
    pub arbitration_value_cents: u64,
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
                "Resolved transaction omitted its current revision".into()
            ))),
            (TransactionResolutionStateWire::Conflicted, _) => Err(wasm_error!(
                WasmErrorInner::Guest("Conflicted transaction cannot produce reputation".into())
            )),
        }
    }
}

#[derive(Serialize, Deserialize, Debug, Clone)]
struct TransactionOutputWire {
    transaction_hash: ActionHash,
    transaction: TransactionEvidenceWire,
}

#[derive(Serialize, Deserialize, Debug, Clone, SerializedBytes)]
struct TransactionEvidenceWire {
    buyer: AgentPubKey,
    seller: AgentPubKey,
    total_price_cents: u64,
    status: TransactionStatusWire,
    updated_at: Timestamp,
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

#[derive(Serialize, Deserialize, Debug, Clone, SerializedBytes)]
struct ArbitrationResultWire {
    dispute_hash: ActionHash,
    dispute_revision_hash: ActionHash,
    winner: AgentPubKey,
    loser: AgentPubKey,
    finalized_at: Timestamp,
}

#[derive(Serialize, Deserialize, Debug, Clone, SerializedBytes)]
struct DisputeWire {
    transaction_hash: ActionHash,
    transaction_revision_hash: ActionHash,
    buyer: AgentPubKey,
    seller: AgentPubKey,
    arbitrators: Vec<AgentPubKey>,
}

/// Compute Proof of Gradient Quality
///
/// This measures the quality and consistency of an agent's behavior.
/// Higher quality + higher consistency = higher trust.
fn compute_pogq(
    score: &MatlScore,
    input: &UpdateMatlInput,
) -> ExternResult<ProofOfGradientQuality> {
    // Quality: weighted by transaction value and outcome
    let transaction_quality = if input.successful {
        // Successful transaction increases quality
        0.8 + (input.transaction_value_cents as f64 / 1_000_000.0).min(0.2)
    } else {
        // Failed transaction decreases quality
        0.2
    };

    // Exponential moving average for quality
    let alpha = 0.2;
    let new_quality = alpha * transaction_quality + (1.0 - alpha) * score.pogq.quality;

    // Consistency: measure variance in quality over time
    // Low variance = high consistency
    let quality_diff = (transaction_quality - score.pogq.quality).abs();
    let new_consistency = (1.0 - quality_diff.min(1.0)) * 0.7 + score.pogq.consistency * 0.3;

    // Entropy: measure of unpredictability
    // Lower entropy = more predictable (good)
    let new_entropy = compute_entropy(score)?;

    Ok(ProofOfGradientQuality {
        quality: new_quality.clamp(0.0, 1.0),
        consistency: new_consistency.clamp(0.0, 1.0),
        entropy: new_entropy,
        timestamp: time::now()?,
    })
}

/// Compute entropy of agent behavior
///
/// Low entropy = predictable, consistent behavior (trustworthy)
/// High entropy = erratic, unpredictable behavior (suspicious)
fn compute_entropy(score: &MatlScore) -> ExternResult<f64> {
    // Simplified entropy calculation
    // In production, this would analyze transaction patterns over time
    let quality_variance = (0.5 - score.pogq.quality).abs();
    let consistency_penalty = 1.0 - score.pogq.consistency;

    Ok((quality_variance + consistency_penalty) / 2.0)
}

/// Legacy heuristic flags retained for compatibility.
///
/// These indicators are not a proof of any Byzantine-tolerance threshold and
/// are not used as arbitration weights or integrity authorization.
fn detect_byzantine_patterns(score: &MatlScore) -> ExternResult<ByzantineFlags> {
    let mut flags = score.flags.clone();

    // 1. Volatile Reputation Detection
    // Rapid changes in reputation suggest manipulation
    flags.volatile_reputation = score.pogq.entropy > 0.7;

    // 2. Sybil Detection via Graph Analysis
    // Analyze transaction patterns to detect coordinated Sybil attacks
    let sybil_analysis = detect_sybil_patterns(score)?;
    flags.sybil_suspected = sybil_analysis.is_sybil_suspected;

    // 3. Quality Inconsistency
    // High quality but low consistency = suspicious
    let inconsistency = score.pogq.quality - score.pogq.consistency;
    if inconsistency > 0.3 {
        flags.risk_score = (flags.risk_score + 0.2).min(1.0);
    }

    // 4. Compute overall Byzantine risk score
    let mut risk: f64 = 0.0;
    if flags.cartel_detected {
        risk += 0.4;
    }
    if flags.volatile_reputation {
        risk += 0.2;
    }
    if flags.gradient_poisoning {
        risk += 0.3;
    }
    if flags.sybil_suspected {
        // Use the Sybil confidence score for weighted risk
        risk += sybil_analysis.confidence * 0.35;
    }

    flags.risk_score = risk.min(1.0);

    Ok(flags)
}

/// Sybil detection analysis result
#[derive(Clone, Debug)]
struct SybilAnalysisResult {
    /// Whether Sybil behavior is suspected
    is_sybil_suspected: bool,
    /// Confidence level (0.0 - 1.0)
    confidence: f64,
    /// Reasons for suspicion
    reasons: Vec<String>,
}

/// Detect Sybil patterns using graph analysis and behavioral heuristics
///
/// This implements multi-factor Sybil detection:
/// 1. Low transaction count with high reputation (boosted account)
/// 2. Clustered transaction timing (coordinated accounts)
/// 3. Similar transaction patterns (identical behavior)
/// 4. Shared counterparties (ring trading)
/// 5. Age-reputation mismatch (suspiciously fast growth)
fn detect_sybil_patterns(score: &MatlScore) -> ExternResult<SybilAnalysisResult> {
    let mut is_suspicious = false;
    let mut confidence: f64 = 0.0;
    let mut reasons: Vec<String> = Vec::new();

    // Heuristic 1: Low activity with high reputation
    // New accounts shouldn't have high trust without earning it
    if score.transaction_count < 5 && score.composite > 0.75 {
        is_suspicious = true;
        confidence += 0.25;
        reasons.push("High reputation with minimal activity".to_string());
    }

    // Heuristic 2: Perfect quality with no history
    // Real users make mistakes and have variance in performance
    if score.transaction_count < 10 && score.pogq.quality > 0.95 {
        is_suspicious = true;
        confidence += 0.2;
        reasons.push("Suspiciously perfect quality with little history".to_string());
    }

    // Heuristic 3: Age-reputation growth rate
    // Check if reputation grew faster than is organically possible
    // A new account shouldn't reach high reputation within hours
    let account_age_hours = score.transaction_count as f64 * 0.5; // Rough estimate
    let expected_max_reputation = (account_age_hours / 24.0).min(1.0) * 0.8; // Max 0.8 after 24h
    if score.composite > expected_max_reputation && score.transaction_count < 20 {
        is_suspicious = true;
        confidence += 0.15;
        reasons.push("Reputation growth exceeds organic rate".to_string());
    }

    // Heuristic 4: Transaction pattern entropy
    // Sybil accounts often have very predictable patterns
    if score.pogq.entropy < 0.1 && score.transaction_count > 5 {
        // Very low entropy = suspiciously uniform behavior
        is_suspicious = true;
        confidence += 0.2;
        reasons.push("Unusually uniform transaction patterns".to_string());
    }

    // Heuristic 5: PoGQ consistency without variance
    // Real ML nodes have natural variance in gradient quality
    if score.pogq.consistency > 0.98 && score.transaction_count > 10 {
        is_suspicious = true;
        confidence += 0.15;
        reasons.push("Unnaturally consistent gradient quality".to_string());
    }

    // Heuristic 6: Check for graph clustering
    // Sybil accounts often only transact with each other
    let clustering_score = analyze_transaction_clustering(score)?;
    if clustering_score > 0.8 {
        is_suspicious = true;
        confidence += 0.25;
        reasons.push("High transaction clustering indicates ring trading".to_string());
    }

    // Normalize confidence to [0, 1]
    confidence = confidence.min(1.0);

    // Only flag as Sybil if confidence is high enough
    let threshold = 0.35;
    is_suspicious = is_suspicious && confidence >= threshold;

    Ok(SybilAnalysisResult {
        is_sybil_suspected: is_suspicious,
        confidence,
        reasons,
    })
}

/// Analyze transaction clustering to detect ring trading
///
/// This checks if the agent's transactions are concentrated
/// among a small set of counterparties (potential Sybil ring).
fn analyze_transaction_clustering(score: &MatlScore) -> ExternResult<f64> {
    // In a full implementation, this would:
    // 1. Fetch all transactions for this agent
    // 2. Extract unique counterparties
    // 3. Calculate clustering coefficient
    // 4. Compare against known Sybil patterns

    // For now, use heuristics based on available data
    // High quality + high consistency + low transaction count = potential cluster
    let clustering_indicators = [
        // Low diversity indicator
        if score.transaction_count < 10 {
            0.3
        } else {
            0.0
        },
        // Perfect metrics indicator
        if score.pogq.quality > 0.9 && score.pogq.consistency > 0.9 {
            0.3
        } else {
            0.0
        },
        // Low entropy indicator (uniform patterns)
        if score.pogq.entropy < 0.2 {
            0.4
        } else {
            score.pogq.entropy * 0.2
        },
    ];

    let clustering_score: f64 = clustering_indicators.iter().sum();
    Ok(clustering_score.min(1.0))
}

/// Compute composite MATL score
///
/// This is the final trust score formula:
/// composite = 0.4 * quality + 0.3 * consistency + 0.3 * reputation
///
/// Why these weights?
/// - Quality (0.4): Most important - what they do
/// - Consistency (0.3): Important - reliability over time
/// - Reputation (0.3): Important - historical track record
pub fn compute_composite_score(pogq: &ProofOfGradientQuality, reputation: f64) -> f64 {
    const W_QUALITY: f64 = 0.4;
    const W_CONSISTENCY: f64 = 0.3;
    const W_REPUTATION: f64 = 0.3;

    (W_QUALITY * pogq.quality + W_CONSISTENCY * pogq.consistency + W_REPUTATION * reputation)
        .clamp(0.0, 1.0)
}

/// Check if agent is Byzantine (above risk threshold)
///
/// Default threshold: 0.5 (50% confidence of Byzantine behavior)
/// This can be adjusted per marketplace based on risk tolerance.
#[hdk_extern]
pub fn is_byzantine(agent: AgentPubKey) -> ExternResult<ByzantineCheckResult> {
    const THRESHOLD: f64 = 0.5;

    match get_agent_matl_score(agent)? {
        Some(score) => Ok(ByzantineCheckResult {
            is_byzantine: score.flags.risk_score >= THRESHOLD,
            risk_score: score.flags.risk_score,
            composite_score: score.composite,
            flags: score.flags,
        }),
        None => {
            // Unknown agent - treat as neutral
            Ok(ByzantineCheckResult {
                is_byzantine: false,
                risk_score: 0.0,
                composite_score: 0.5,
                flags: ByzantineFlags {
                    cartel_detected: false,
                    volatile_reputation: false,
                    gradient_poisoning: false,
                    sybil_suspected: false,
                    risk_score: 0.0,
                },
            })
        }
    }
}

/// Submit a review after a transaction
#[hdk_extern]
pub fn submit_review(input: SubmitReviewInput) -> ExternResult<ReviewOutput> {
    let agent_info = agent_info()?;

    // Create review entry
    let review = Review {
        transaction_hash: input.transaction_hash.clone(),
        listing_hash: input.listing_hash,
        rating: input.rating,
        comment: input.comment,
        reviewer: agent_info.agent_initial_pubkey.clone(),
        seller: input.seller.clone(),
        created_at: sys_time()?,
        epistemic: EpistemicClassification {
            // Reviews are privately verifiable (only buyer experienced it)
            empirical: EmpiricalLevel::E2PrivateVerify,
            // Community agreement (buyer-seller)
            normative: NormativeLevel::N1Communal,
            // Persistent (keep for reputation)
            materiality: MaterialityLevel::M2Persistent,
        },
    };

    let action_hash = create_entry(&EntryTypes::Review(review.clone()))?;

    // Create links
    create_link(
        input.seller.clone(),
        action_hash.clone(),
        LinkTypes::AgentToSellerReviews,
        (),
    )?;

    create_link(
        agent_info.agent_initial_pubkey,
        action_hash.clone(),
        LinkTypes::AgentToBuyerReviews,
        (),
    )?;

    create_link(
        input.transaction_hash,
        action_hash.clone(),
        LinkTypes::TransactionToReview,
        (),
    )?;

    // Emit monitoring metric
    monitoring::emit_metric(
        monitoring::MetricType::ReviewSubmitted,
        input.rating as f64,
        Some(review.reviewer.clone()),
        Some(format!("seller:{:?}", input.seller)),
    )?;

    Ok(ReviewOutput {
        review_hash: action_hash,
        review,
    })
}

/// Get reviews for a seller
#[hdk_extern]
pub fn get_seller_reviews(seller: AgentPubKey) -> ExternResult<ReviewsResponse> {
    // HDK 0.6.0: get_links with LinkQuery::try_new
    let links = get_links(
        LinkQuery::try_new(seller, LinkTypes::AgentToSellerReviews)?,
        GetStrategy::Local,
    )?;

    let mut reviews = Vec::new();

    for link in links {
        if let Some(action_hash) = link.target.into_action_hash() {
            if let Some(record) = get(action_hash, GetOptions::default())? {
                // HDK 0.6.0: to_app_option() returns SerializedBytesError
                let review: Review = record
                    .entry()
                    .to_app_option()
                    .map_err(|e| {
                        wasm_error!(WasmErrorInner::Guest(format!(
                            "Deserialization error: {:?}",
                            e
                        )))
                    })?
                    .ok_or(wasm_error!(WasmErrorInner::Guest(
                        "Could not deserialize review".into()
                    )))?;
                reviews.push(review);
            }
        }
    }

    Ok(ReviewsResponse { reviews })
}

// ===== Input/Output Types =====

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct UpdateMatlInput {
    pub agent: AgentPubKey,
    pub successful: bool,
    pub transaction_value_cents: u64,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ByzantineCheckResult {
    pub is_byzantine: bool,
    pub risk_score: f64,
    pub composite_score: f64,
    pub flags: ByzantineFlags,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct SubmitReviewInput {
    pub transaction_hash: ActionHash,
    pub listing_hash: ActionHash,
    pub seller: AgentPubKey,
    pub rating: u8,
    pub comment: String,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ReviewOutput {
    pub review_hash: ActionHash,
    pub review: Review,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ReviewsResponse {
    pub reviews: Vec<Review>,
}

/// Response for combined reputation query
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct CombinedReputationResponse {
    /// Local MATL composite score (0.0 - 1.0)
    pub local_score: f64,
    /// Cross-app reputation score from Bridge (if available)
    pub cross_app_score: Option<f64>,
    /// Weighted combined score
    pub combined_score: f64,
    /// Whether cross-app reputation was available
    pub cross_app_available: bool,
    /// Number of local transactions
    pub transaction_count: u32,
}

/// Response for cross-app reputation query
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct CrossAppReputationResponse {
    /// Whether Bridge/cross-app reputation is available
    pub available: bool,
    /// The DID used for lookup
    pub did: String,
    /// Cross-app reputation score (if available)
    pub score: Option<f64>,
    /// Number of apps contributing to score
    pub app_count: Option<u32>,
    /// Total transactions across all apps
    pub total_transactions: Option<u64>,
    /// Error message if unavailable
    pub error: Option<String>,
}

// ===== Tests =====
#[cfg(test)]
mod tests;
