use crate::{ClientError, ZomeTransport, contract};
use async_trait::async_trait;
use marketplace_domain::{
    ActionHash, AgentPubKey, ArbitrationResult, ArbitrationResultOutput, ArbitrationVote,
    ArbitrationVoteOutput, ArbitrationVotesResponse, CreateTransactionInput, Dispute,
    DisputeOutput, DisputeResolution, DisputeResolutionState, DisputeStatus, DisputesResponse,
    EmpiricalLevel, EpistemicClassification, Listing, ListingCategory, ListingOutput,
    ListingStatus, ListingsResponse, MarkShippedInput, MaterialityLevel, NormativeLevel,
    OpenDisputeInput, OpenDisputeOutput, ReputationEvent, ReputationEventKind,
    ReputationEventOutput, ReputationEventsResponse, DerivedReputation,
    SubmitArbitrationVoteInput, TimestampMicros, Transaction, TransactionOutput,
    TransactionResolution, TransactionResolutionReason, TransactionResolutionState, TransactionResolutionsResponse,
    TransactionSettlementResult, TransactionSettlementState, TransactionStatus,
    TransactionsResponse,
};
use serde::Serialize;
use std::{cell::RefCell, rc::Rc};

fn encode_fixture<T: Serialize>(value: &T) -> Result<Vec<u8>, ClientError> {
    rmp_serde::to_vec_named(value).map_err(|error| ClientError::Encode(error.to_string()))
}

fn fixture_error_for(zome: &str, function: &str, message: impl Into<String>) -> ClientError {
    ClientError::Call {
        zome: zome.into(),
        function: function.into(),
        message: message.into(),
    }
}

fn fixture_error(function: &str, message: impl Into<String>) -> ClientError {
    fixture_error_for(contract::transactions::ZOME, function, message)
}

fn arbitration_fixture_error(function: &str, message: impl Into<String>) -> ClientError {
    fixture_error_for(contract::arbitration::ZOME, function, message)
}

pub fn fixture_agent() -> AgentPubKey {
    AgentPubKey::new(vec![0x11; 39])
}

pub fn fixture_listings() -> Vec<ListingOutput> {
    vec![
        ListingOutput {
            listing_hash: ActionHash::new(vec![0x84; 39]),
            seller_agent_id: AgentPubKey::new(vec![0x42; 39]),
            listing: Listing {
                title: "Repairable solar field radio".into(),
                description: "Open-hardware radio with replaceable cells and a published repair trail.".into(),
                price_cents: 12_500,
                category: ListingCategory::Electronics,
                photos_ipfs_cids: vec!["bafybeigdyrztfixture-radio".into()],
                quantity_available: 4,
                status: ListingStatus::Active,
                epistemic: EpistemicClassification {
                    empirical: EmpiricalLevel::E1Testimonial,
                    normative: NormativeLevel::N0Personal,
                    materiality: MaterialityLevel::M2Persistent,
                },
                created_at: TimestampMicros(0),
                updated_at: TimestampMicros(0),
            },
        },
        ListingOutput {
            listing_hash: ActionHash::new(vec![0x91; 39]),
            seller_agent_id: AgentPubKey::new(vec![0x53; 39]),
            listing: Listing {
                title: "Community seed-library cabinet".into(),
                description: "Weather-sealed cabinet built for neighborhood seed exchange and provenance cards.".into(),
                price_cents: 38_000,
                category: ListingCategory::HomeGarden,
                photos_ipfs_cids: vec!["bafybeigdyrztfixture-seeds".into()],
                quantity_available: 2,
                status: ListingStatus::Active,
                epistemic: EpistemicClassification {
                    empirical: EmpiricalLevel::E2PrivateVerify,
                    normative: NormativeLevel::N1Communal,
                    materiality: MaterialityLevel::M2Persistent,
                },
                created_at: TimestampMicros(0),
                updated_at: TimestampMicros(0),
            },
        },
    ]
}

#[derive(Clone)]
struct FixtureRevision {
    root_transaction_hash: ActionHash,
    known_hashes: Vec<ActionHash>,
    current: TransactionOutput,
}

impl FixtureRevision {
    fn contains(&self, hash: &ActionHash) -> bool {
        self.known_hashes.contains(hash)
    }

    fn resolution(&self) -> TransactionResolution {
        TransactionResolution {
            root_transaction_hash: self.root_transaction_hash.clone(),
            policy_version: 2,
            state: TransactionResolutionState::Resolved,
            reason: TransactionResolutionReason::SingleHead,
            canonical: Some(self.current.clone()),
            heads: vec![self.current.clone()],
            superseded_heads: Vec::new(),
            applied_conflict_resolutions: Vec::new(),
            revision_count: u32::try_from(self.known_hashes.len()).unwrap_or(u32::MAX),
        }
    }
}

#[derive(Clone)]
struct FixtureDisputeRevision {
    root_dispute_hash: ActionHash,
    known_hashes: Vec<ActionHash>,
    current: DisputeOutput,
    votes: Vec<ArbitrationVoteOutput>,
    result: Option<ArbitrationResultOutput>,
}

impl FixtureDisputeRevision {
    fn contains(&self, hash: &ActionHash) -> bool {
        self.known_hashes.contains(hash)
    }

    fn resolution(&self) -> DisputeResolution {
        DisputeResolution {
            root_dispute_hash: self.root_dispute_hash.clone(),
            state: DisputeResolutionState::Resolved,
            canonical: Some(self.current.clone()),
            heads: vec![self.current.clone()],
            revision_count: u32::try_from(self.known_hashes.len()).unwrap_or(u32::MAX),
        }
    }
}

#[derive(Default)]
struct FixtureState {
    transactions: Vec<FixtureRevision>,
    disputes: Vec<FixtureDisputeRevision>,
    registered_arbitrators: Vec<AgentPubKey>,
    settled_transactions: Vec<ActionHash>,
    reputation_events: Vec<ReputationEventOutput>,
    next_hash: u8,
}

#[derive(Clone)]
pub struct FixtureTransport {
    state: Rc<RefCell<FixtureState>>,
    acting_agent: AgentPubKey,
    finance_enabled: bool,
}

impl Default for FixtureTransport {
    fn default() -> Self {
        Self {
            state: Rc::new(RefCell::new(FixtureState::default())),
            acting_agent: fixture_agent(),
            finance_enabled: false,
        }
    }
}

impl FixtureTransport {
    /// Return another explicit fixture identity over the same deterministic state.
    pub fn acting_as(&self, acting_agent: AgentPubKey) -> Self {
        Self {
            state: Rc::clone(&self.state),
            acting_agent,
            finance_enabled: self.finance_enabled,
        }
    }

    pub fn with_finance(mut self) -> Self {
        self.finance_enabled = true;
        self
    }

    fn next_action_hash(state: &mut FixtureState) -> ActionHash {
        state.next_hash = state.next_hash.saturating_add(1);
        let mut hash_bytes = vec![0xA5; 39];
        hash_bytes[38] = state.next_hash;
        ActionHash::new(hash_bytes)
    }

    fn assign_fixture_arbitrators(
        state: &mut FixtureState,
        dispute_index: usize,
    ) -> Result<(), ClientError> {
        if state.disputes[dispute_index].current.dispute.status != DisputeStatus::Filed {
            return Ok(());
        }
        let dispute = state.disputes[dispute_index].current.dispute.clone();
        let mut arbitrators = state
            .registered_arbitrators
            .iter()
            .filter(|candidate| {
                *candidate != &dispute.buyer
                    && *candidate != &dispute.seller
                    && *candidate != &dispute.filed_by
            })
            .cloned()
            .collect::<Vec<_>>();
        arbitrators.sort_by_key(ToString::to_string);
        arbitrators.dedup();
        arbitrators.truncate(3);
        if arbitrators.is_empty() {
            return Err(fixture_error(
                contract::transactions::OPEN_DISPUTE,
                "no non-party fixture arbitrator is registered; the Filed case remains recoverable",
            ));
        }

        let assigned_hash = Self::next_action_hash(state);
        let revision = &mut state.disputes[dispute_index];
        let mut assigned = revision.current.dispute.clone();
        assigned.status = DisputeStatus::UnderReview;
        assigned.arbitrators = arbitrators;
        assigned.updated_at = TimestampMicros(assigned.updated_at.0.saturating_add(1_000_000));
        revision.known_hashes.push(assigned_hash.clone());
        revision.current = DisputeOutput {
            dispute_hash: assigned_hash,
            dispute: assigned,
        };
        Ok(())
    }

    fn create_transaction(
        &self,
        input: CreateTransactionInput,
    ) -> Result<TransactionOutput, ClientError> {
        let listing = fixture_listings()
            .into_iter()
            .find(|candidate| candidate.listing_hash == input.listing_hash)
            .ok_or_else(|| fixture_error(contract::transactions::CREATE_TRANSACTION, "fixture listing not found"))?;

        let expected_total = listing
            .listing
            .price_cents
            .checked_mul(u64::from(input.quantity))
            .ok_or_else(|| fixture_error(contract::transactions::CREATE_TRANSACTION, "fixture total overflow"))?;

        if input.quantity == 0 || input.quantity > listing.listing.quantity_available {
            return Err(fixture_error(
                contract::transactions::CREATE_TRANSACTION,
                "requested fixture quantity is unavailable",
            ));
        }
        if self.acting_agent == listing.seller_agent_id {
            return Err(fixture_error(
                contract::transactions::CREATE_TRANSACTION,
                "fixture seller cannot purchase their own listing",
            ));
        }
        if input.seller != listing.seller_agent_id || input.total_price_cents != expected_total {
            return Err(fixture_error(
                contract::transactions::CREATE_TRANSACTION,
                "fixture purchase terms do not match the listing",
            ));
        }

        let mut state = self.state.borrow_mut();
        let transaction_hash = Self::next_action_hash(&mut state);
        let output = TransactionOutput {
            transaction_hash: transaction_hash.clone(),
            transaction: Transaction {
                buyer: self.acting_agent.clone(),
                seller: listing.seller_agent_id,
                listing_hash: listing.listing_hash,
                quantity: input.quantity,
                total_price_cents: expected_total,
                status: TransactionStatus::Pending,
                created_at: TimestampMicros(1_000_000),
                updated_at: TimestampMicros(1_000_000),
                tracking_info: None,
                epistemic: EpistemicClassification {
                    empirical: EmpiricalLevel::E1Testimonial,
                    normative: NormativeLevel::N1Communal,
                    materiality: MaterialityLevel::M1Temporal,
                },
            },
        };
        state.transactions.push(FixtureRevision {
            root_transaction_hash: transaction_hash.clone(),
            known_hashes: vec![transaction_hash],
            current: output.clone(),
        });
        Ok(output)
    }

    fn find_resolution(&self, requested: &ActionHash) -> Option<TransactionResolution> {
        self.state
            .borrow()
            .transactions
            .iter()
            .find(|revision| revision.contains(requested))
            .map(FixtureRevision::resolution)
    }

    fn my_resolutions(&self) -> Vec<TransactionResolution> {
        self.state
            .borrow()
            .transactions
            .iter()
            .filter(|revision| {
                revision.current.transaction.buyer == self.acting_agent
                    || revision.current.transaction.seller == self.acting_agent
            })
            .map(FixtureRevision::resolution)
            .collect()
    }

    fn register_arbitrator(&self) {
        let mut state = self.state.borrow_mut();
        if !state.registered_arbitrators.contains(&self.acting_agent) {
            state.registered_arbitrators.push(self.acting_agent.clone());
        }
    }

    fn open_dispute(&self, input: OpenDisputeInput) -> Result<OpenDisputeOutput, ClientError> {
        let reason = input.reason.trim();
        if reason.is_empty() || reason.len() > 5000 {
            return Err(fixture_error(
                contract::transactions::OPEN_DISPUTE,
                "dispute reason must contain 1-5000 characters",
            ));
        }
        if input.evidence_cids.len() > 32 {
            return Err(fixture_error(
                contract::transactions::OPEN_DISPUTE,
                "at most 32 evidence CIDs are accepted",
            ));
        }

        let mut state = self.state.borrow_mut();
        let transaction_index = state
            .transactions
            .iter()
            .position(|revision| revision.contains(&input.transaction_hash))
            .ok_or_else(|| fixture_error(contract::transactions::OPEN_DISPUTE, "fixture transaction not found"))?;
        let transaction_root = state.transactions[transaction_index].root_transaction_hash.clone();
        let current = state.transactions[transaction_index].current.clone();
        if self.acting_agent != current.transaction.buyer
            && self.acting_agent != current.transaction.seller
        {
            return Err(fixture_error(
                contract::transactions::OPEN_DISPUTE,
                "only a transaction party may open a dispute",
            ));
        }
        if matches!(
            current.transaction.status,
            TransactionStatus::Completed | TransactionStatus::Cancelled
        ) {
            return Err(fixture_error(
                contract::transactions::OPEN_DISPUTE,
                "terminal fixture transactions cannot be disputed",
            ));
        }

        let disputed = if current.transaction.status == TransactionStatus::Disputed {
            current
        } else {
            let action_hash = Self::next_action_hash(&mut state);
            let mut transaction = current.transaction;
            transaction.status = TransactionStatus::Disputed;
            transaction.updated_at = TimestampMicros(transaction.updated_at.0.saturating_add(1_000_000));
            let output = TransactionOutput {
                transaction_hash: action_hash.clone(),
                transaction,
            };
            let revision = &mut state.transactions[transaction_index];
            revision.known_hashes.push(action_hash);
            revision.current = output.clone();
            output
        };

        if let Some(existing_index) = state
            .disputes
            .iter()
            .position(|revision| revision.current.dispute.transaction_hash == transaction_root)
        {
            Self::assign_fixture_arbitrators(&mut state, existing_index)?;
            return Ok(OpenDisputeOutput {
                transaction: disputed,
                dispute_hash: state.disputes[existing_index].root_dispute_hash.clone(),
            });
        }

        let root_dispute_hash = Self::next_action_hash(&mut state);
        let filed = DisputeOutput {
            dispute_hash: root_dispute_hash.clone(),
            dispute: Dispute {
                transaction_hash: transaction_root,
                transaction_revision_hash: disputed.transaction_hash.clone(),
                conflict_heads: Vec::new(),
                filed_by: self.acting_agent.clone(),
                buyer: disputed.transaction.buyer.clone(),
                seller: disputed.transaction.seller.clone(),
                reason: input.reason,
                evidence_cids: input.evidence_cids,
                status: DisputeStatus::Filed,
                arbitrators: Vec::new(),
                result_hash: None,
                created_at: disputed.transaction.updated_at,
                updated_at: disputed.transaction.updated_at,
            },
        };
        state.disputes.push(FixtureDisputeRevision {
            root_dispute_hash: root_dispute_hash.clone(),
            known_hashes: vec![root_dispute_hash.clone()],
            current: filed,
            votes: Vec::new(),
            result: None,
        });
        let dispute_index = state.disputes.len() - 1;

        Self::assign_fixture_arbitrators(&mut state, dispute_index)?;

        Ok(OpenDisputeOutput {
            transaction: disputed,
            dispute_hash: root_dispute_hash,
        })
    }

    fn find_dispute_resolution(&self, requested: &ActionHash) -> Option<DisputeResolution> {
        self.state
            .borrow()
            .disputes
            .iter()
            .find(|revision| revision.contains(requested))
            .map(FixtureDisputeRevision::resolution)
    }

    fn arbitration_opportunities(&self) -> DisputesResponse {
        let disputes = self
            .state
            .borrow()
            .disputes
            .iter()
            .filter(|revision| {
                revision.current.dispute.status == DisputeStatus::UnderReview
                    && revision.current.dispute.arbitrators.contains(&self.acting_agent)
                    && !revision
                        .votes
                        .iter()
                        .any(|vote| vote.vote.arbitrator == self.acting_agent)
            })
            .map(|revision| revision.current.clone())
            .collect();
        DisputesResponse { disputes }
    }

    fn submit_vote(
        &self,
        input: SubmitArbitrationVoteInput,
    ) -> Result<ArbitrationVoteOutput, ClientError> {
        let mut state = self.state.borrow_mut();
        let dispute_index = state
            .disputes
            .iter()
            .position(|revision| revision.contains(&input.dispute_hash))
            .ok_or_else(|| arbitration_fixture_error(contract::arbitration::SUBMIT_ARBITRATION_VOTE, "fixture dispute not found"))?;
        let current = state.disputes[dispute_index].current.clone();
        if !matches!(current.dispute.status, DisputeStatus::UnderReview | DisputeStatus::Voting)
            || !current.dispute.arbitrators.contains(&self.acting_agent)
        {
            return Err(arbitration_fixture_error(
                contract::arbitration::SUBMIT_ARBITRATION_VOTE,
                "fixture agent is not authorized to vote",
            ));
        }
        if state.disputes[dispute_index]
            .votes
            .iter()
            .any(|vote| vote.vote.arbitrator == self.acting_agent)
        {
            return Err(arbitration_fixture_error(
                contract::arbitration::SUBMIT_ARBITRATION_VOTE,
                "fixture arbitrator has already voted",
            ));
        }

        let vote_hash = Self::next_action_hash(&mut state);
        let vote = ArbitrationVoteOutput {
            vote_hash: vote_hash.clone(),
            vote: ArbitrationVote {
                dispute_hash: state.disputes[dispute_index].root_dispute_hash.clone(),
                dispute_revision_hash: current.dispute_hash,
                arbitrator: self.acting_agent.clone(),
                favor_buyer: input.favor_buyer,
                reasoning: input.reasoning,
                arbitrator_matl_score: 1.0,
                voted_at: TimestampMicros(current.dispute.updated_at.0.saturating_add(1_000_000)),
            },
        };
        state.disputes[dispute_index].votes.push(vote.clone());

        let all_voted = state.disputes[dispute_index].votes.len()
            == state.disputes[dispute_index].current.dispute.arbitrators.len();
        if all_voted && state.disputes[dispute_index].current.dispute.status == DisputeStatus::UnderReview {
            let update_hash = Self::next_action_hash(&mut state);
            let revision = &mut state.disputes[dispute_index];
            let mut voting = revision.current.dispute.clone();
            voting.status = DisputeStatus::Voting;
            voting.updated_at = TimestampMicros(voting.updated_at.0.saturating_add(1_000_000));
            revision.known_hashes.push(update_hash.clone());
            revision.current = DisputeOutput {
                dispute_hash: update_hash,
                dispute: voting,
            };
        }
        Ok(vote)
    }

    fn finalize_dispute(
        &self,
        requested: ActionHash,
    ) -> Result<ArbitrationResultOutput, ClientError> {
        let mut state = self.state.borrow_mut();
        let dispute_index = state
            .disputes
            .iter()
            .position(|revision| revision.contains(&requested))
            .ok_or_else(|| arbitration_fixture_error(contract::arbitration::FINALIZE_ARBITRATION, "fixture dispute not found"))?;
        if let Some(result) = state.disputes[dispute_index].result.clone() {
            return Ok(result);
        }
        let current = state.disputes[dispute_index].current.clone();
        if current.dispute.status != DisputeStatus::Voting
            || !current.dispute.arbitrators.contains(&self.acting_agent)
        {
            return Err(arbitration_fixture_error(
                contract::arbitration::FINALIZE_ARBITRATION,
                "fixture dispute is not finalizable by this agent",
            ));
        }
        let votes = state.disputes[dispute_index].votes.clone();
        if votes.len() != current.dispute.arbitrators.len() || votes.is_empty() {
            return Err(arbitration_fixture_error(
                contract::arbitration::FINALIZE_ARBITRATION,
                "fixture finalization requires exactly one vote per arbitrator",
            ));
        }
        let buyer_votes = votes.iter().filter(|vote| vote.vote.favor_buyer).count();
        let ratio = buyer_votes as f64 / votes.len() as f64;
        let buyer_wins = ratio > 0.66;
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
        let transaction_value = state
            .transactions
            .iter()
            .find(|revision| revision.root_transaction_hash == current.dispute.transaction_hash)
            .map(|revision| revision.current.transaction.total_price_cents)
            .unwrap_or_default();
        let strength = if buyer_wins { ratio } else { 1.0 - ratio };
        let basis_points = if strength >= 0.85 { 10_000 } else if strength >= 0.75 { 7_500 } else { 5_000 };
        let compensation = transaction_value.saturating_mul(basis_points) / 10_000;
        let result_hash = Self::next_action_hash(&mut state);
        let result = ArbitrationResultOutput {
            result_hash: result_hash.clone(),
            result: ArbitrationResult {
                dispute_hash: state.disputes[dispute_index].root_dispute_hash.clone(),
                dispute_revision_hash: current.dispute_hash,
                vote_hashes: votes.iter().map(|vote| vote.vote_hash.clone()).collect(),
                winner,
                loser,
                weighted_vote: ratio,
                total_votes: votes.len() as u32,
                compensation_cents: Some(compensation),
                summary: "Deterministic equal-weight fixture result".into(),
                finalized_at: TimestampMicros(current.dispute.updated_at.0.saturating_add(1_000_000)),
            },
        };
        let update_hash = Self::next_action_hash(&mut state);
        let revision = &mut state.disputes[dispute_index];
        let mut resolved = revision.current.dispute.clone();
        resolved.status = status;
        resolved.result_hash = Some(result_hash);
        resolved.updated_at = TimestampMicros(resolved.updated_at.0.saturating_add(1_000_000));
        revision.known_hashes.push(update_hash.clone());
        revision.current = DisputeOutput {
            dispute_hash: update_hash,
            dispute: resolved,
        };
        revision.result = Some(result.clone());
        Ok(result)
    }

    fn settlement_result(
        &self,
        requested: ActionHash,
        initiate: bool,
    ) -> Result<TransactionSettlementResult, ClientError> {
        let mut state = self.state.borrow_mut();
        let revision = state
            .transactions
            .iter()
            .find(|revision| revision.contains(&requested))
            .cloned()
            .ok_or_else(|| fixture_error("settlement", "fixture transaction not found"))?;
        let current = revision.current;
        let reference = format!("marketplace_tx:{}", revision.root_transaction_hash);
        if !self.finance_enabled {
            return Ok(TransactionSettlementResult {
                root_transaction_hash: revision.root_transaction_hash,
                transaction_revision_hash: current.transaction_hash,
                state: TransactionSettlementState::Unavailable,
                settled: false,
                idempotency_reference: reference,
                finance_payment_id: None,
                finance_action_hash: None,
                error: Some("fixture Finance role is disabled".into()),
            });
        }
        if initiate {
            if self.acting_agent != current.transaction.buyer {
                return Err(fixture_error(
                    contract::transactions::SETTLE_TRANSACTION,
                    "only the fixture buyer may initiate settlement",
                ));
            }
            if current.transaction.status != TransactionStatus::Delivered {
                return Err(fixture_error(
                    contract::transactions::SETTLE_TRANSACTION,
                    "fixture settlement requires Delivered status",
                ));
            }
            if !state.settled_transactions.contains(&revision.root_transaction_hash) {
                state
                    .settled_transactions
                    .push(revision.root_transaction_hash.clone());
            }
        }
        let settled = state
            .settled_transactions
            .contains(&revision.root_transaction_hash);
        Ok(TransactionSettlementResult {
            root_transaction_hash: revision.root_transaction_hash,
            transaction_revision_hash: current.transaction_hash,
            state: if settled {
                TransactionSettlementState::Completed
            } else {
                TransactionSettlementState::NotFound
            },
            settled,
            idempotency_reference: reference.clone(),
            finance_payment_id: settled.then(|| format!("fixture:{reference}")),
            finance_action_hash: settled.then(|| ActionHash::new(vec![0xF1; 39])),
            error: None,
        })
    }

    fn record_fulfillment_event(
        &self,
        requested: ActionHash,
    ) -> Result<ReputationEventOutput, ClientError> {
        let mut state = self.state.borrow_mut();
        let revision = state
            .transactions
            .iter()
            .find(|revision| revision.contains(&requested))
            .cloned()
            .ok_or_else(|| fixture_error_for("reputation", "record_fulfillment_reputation", "fixture transaction not found"))?;
        let current = revision.current;
        if current.transaction.status != TransactionStatus::Delivered
            || self.acting_agent != current.transaction.buyer
        {
            return Err(fixture_error_for(
                "reputation",
                "record_fulfillment_reputation",
                "fixture fulfillment evidence must be buyer-authored and Delivered",
            ));
        }
        let key = format!(
            "fulfillment:{}:seller:{}",
            revision.root_transaction_hash, current.transaction.seller
        );
        if let Some(existing) = state
            .reputation_events
            .iter()
            .find(|output| output.event.event_key == key)
        {
            return Ok(existing.clone());
        }
        let event_hash = Self::next_action_hash(&mut state);
        let output = ReputationEventOutput {
            event_hash,
            event: ReputationEvent {
                event_key: key,
                subject: current.transaction.seller,
                counterparty: current.transaction.buyer,
                transaction_hash: revision.root_transaction_hash,
                source_hash: current.transaction_hash,
                kind: ReputationEventKind::FulfillmentDelivered,
                value_cents: current.transaction.total_price_cents,
                occurred_at: current.transaction.updated_at,
            },
        };
        state.reputation_events.push(output.clone());
        Ok(output)
    }

    fn project_arbitration_events(
        &self,
        result_hash: ActionHash,
    ) -> Result<ReputationEventsResponse, ClientError> {
        let mut state = self.state.borrow_mut();
        let revision = state
            .disputes
            .iter()
            .find(|revision| {
                revision
                    .result
                    .as_ref()
                    .is_some_and(|output| output.result_hash == result_hash)
            })
            .cloned()
            .ok_or_else(|| fixture_error_for(
                contract::reputation::ZOME,
                contract::reputation::PROJECT_ARBITRATION_REPUTATION,
                "fixture arbitration result not found",
            ))?;
        if !revision.current.dispute.arbitrators.contains(&self.acting_agent) {
            return Err(fixture_error_for(
                contract::reputation::ZOME,
                contract::reputation::PROJECT_ARBITRATION_REPUTATION,
                "only an assigned fixture arbitrator may project the result",
            ));
        }
        let result = revision.result.expect("matched result is present");
        let transaction = state
            .transactions
            .iter()
            .find(|transaction| {
                transaction.root_transaction_hash == revision.current.dispute.transaction_hash
            })
            .map(|transaction| transaction.current.transaction.clone())
            .ok_or_else(|| fixture_error_for(
                contract::reputation::ZOME,
                contract::reputation::PROJECT_ARBITRATION_REPUTATION,
                "fixture arbitration transaction not found",
            ))?;

        let specifications = [
            (
                ReputationEventKind::ArbitrationWon,
                result.result.winner.clone(),
                result.result.loser.clone(),
            ),
            (
                ReputationEventKind::ArbitrationLost,
                result.result.loser.clone(),
                result.result.winner.clone(),
            ),
        ];
        let mut outputs = Vec::with_capacity(2);
        for (kind, subject, counterparty) in specifications {
            let kind_label = match kind {
                ReputationEventKind::ArbitrationWon => "won",
                ReputationEventKind::ArbitrationLost => "lost",
                ReputationEventKind::FulfillmentDelivered => unreachable!(),
            };
            let event_key = format!(
                "arbitration:{}:{}:{}",
                result.result_hash, kind_label, subject
            );
            if let Some(existing) = state
                .reputation_events
                .iter()
                .find(|output| output.event.event_key == event_key)
                .cloned()
            {
                outputs.push(existing);
                continue;
            }
            let event_hash = Self::next_action_hash(&mut state);
            let output = ReputationEventOutput {
                event_hash,
                event: ReputationEvent {
                    event_key,
                    subject,
                    counterparty,
                    transaction_hash: revision.current.dispute.transaction_hash.clone(),
                    source_hash: result.result_hash.clone(),
                    kind,
                    value_cents: transaction.total_price_cents,
                    occurred_at: result.result.finalized_at,
                },
            };
            state.reputation_events.push(output.clone());
            outputs.push(output);
        }
        Ok(ReputationEventsResponse { events: outputs })
    }

    fn reputation_events(&self, agent: AgentPubKey) -> ReputationEventsResponse {
        ReputationEventsResponse {
            events: self
                .state
                .borrow()
                .reputation_events
                .iter()
                .filter(|output| output.event.subject == agent)
                .cloned()
                .collect(),
        }
    }

    fn derived_reputation(&self, agent: AgentPubKey) -> DerivedReputation {
        let events = self.reputation_events(agent.clone()).events;
        let positive_events = events
            .iter()
            .filter(|output| output.event.kind != ReputationEventKind::ArbitrationLost)
            .count() as u32;
        let negative_events = events.len() as u32 - positive_events;
        let fulfilled_value_cents = events
            .iter()
            .filter(|output| output.event.kind == ReputationEventKind::FulfillmentDelivered)
            .map(|output| output.event.value_cents)
            .sum();
        let arbitration_value_cents = events
            .iter()
            .filter(|output| output.event.kind != ReputationEventKind::FulfillmentDelivered)
            .map(|output| output.event.value_cents)
            .sum();
        let score = (positive_events as f64 + 1.0)
            / (positive_events as f64 + negative_events as f64 + 2.0);
        DerivedReputation {
            agent,
            score,
            positive_events,
            negative_events,
            event_count: events.len() as u32,
            fulfilled_value_cents,
            arbitration_value_cents,
        }
    }

    fn update_status(
        &self,
        function: &str,
        requested: ActionHash,
        next_status: TransactionStatus,
        tracking_info: Option<String>,
    ) -> Result<TransactionOutput, ClientError> {
        let mut state = self.state.borrow_mut();
        let index = state
            .transactions
            .iter()
            .position(|revision| revision.contains(&requested))
            .ok_or_else(|| fixture_error(function, "fixture transaction not found"))?;

        let current = state.transactions[index].current.clone();
        let is_buyer = self.acting_agent == current.transaction.buyer;
        let is_seller = self.acting_agent == current.transaction.seller;
        let allowed = match (&current.transaction.status, &next_status) {
            (TransactionStatus::Pending, TransactionStatus::Confirmed) => is_seller,
            (TransactionStatus::Confirmed, TransactionStatus::Shipped) => is_seller,
            (TransactionStatus::Shipped, TransactionStatus::Delivered) => is_buyer,
            (TransactionStatus::Pending | TransactionStatus::Confirmed, TransactionStatus::Cancelled) => {
                is_buyer || is_seller
            }
            _ => false,
        };
        if !allowed {
            return Err(fixture_error(
                function,
                format!(
                    "illegal or unauthorized fixture transition: {} -> {}",
                    current.transaction.status.label(),
                    next_status.label()
                ),
            ));
        }

        if next_status == TransactionStatus::Shipped {
            let tracking = tracking_info.as_deref().map(str::trim).unwrap_or_default();
            if tracking.is_empty() || tracking.len() > 256 {
                return Err(fixture_error(
                    function,
                    "tracking information must contain 1-256 characters",
                ));
            }
        } else if tracking_info.is_some() {
            return Err(fixture_error(
                function,
                "tracking information may only change during shipment",
            ));
        }

        let action_hash = Self::next_action_hash(&mut state);
        let revision = &mut state.transactions[index];
        let mut transaction = current.transaction;
        transaction.status = next_status;
        transaction.updated_at = TimestampMicros(transaction.updated_at.0.saturating_add(1_000_000));
        if tracking_info.is_some() {
            transaction.tracking_info = tracking_info;
        }
        let output = TransactionOutput {
            transaction_hash: action_hash.clone(),
            transaction,
        };
        revision.known_hashes.push(action_hash);
        revision.current = output.clone();
        Ok(output)
    }
}

#[async_trait(?Send)]
impl ZomeTransport for FixtureTransport {
    async fn call_zome(
        &self,
        _role: &str,
        zome: &str,
        function: &str,
        payload: Vec<u8>,
    ) -> Result<Vec<u8>, ClientError> {
        match (zome, function) {
            (contract::listings::ZOME, contract::listings::GET_ALL_LISTINGS) => {
                encode_fixture(&ListingsResponse {
                    listings: fixture_listings(),
                })
            }
            (contract::listings::ZOME, contract::listings::GET_LISTING) => {
                let requested: ActionHash = rmp_serde::from_slice(&payload)
                    .map_err(|error| ClientError::Decode(error.to_string()))?;
                encode_fixture(
                    &fixture_listings()
                        .into_iter()
                        .find(|candidate| candidate.listing_hash == requested),
                )
            }
            (contract::transactions::ZOME, contract::transactions::CREATE_TRANSACTION) => {
                let input: CreateTransactionInput = rmp_serde::from_slice(&payload)
                    .map_err(|error| ClientError::Decode(error.to_string()))?;
                encode_fixture(&self.create_transaction(input)?)
            }
            (contract::transactions::ZOME, contract::transactions::GET_TRANSACTION) => {
                let requested: ActionHash = rmp_serde::from_slice(&payload)
                    .map_err(|error| ClientError::Decode(error.to_string()))?;
                encode_fixture(
                    &self
                        .find_resolution(&requested)
                        .and_then(|resolution| resolution.canonical),
                )
            }
            (contract::transactions::ZOME, contract::transactions::GET_TRANSACTION_RESOLUTION) => {
                let requested: ActionHash = rmp_serde::from_slice(&payload)
                    .map_err(|error| ClientError::Decode(error.to_string()))?;
                encode_fixture(&self.find_resolution(&requested))
            }
            (contract::transactions::ZOME, contract::transactions::GET_MY_TRANSACTIONS) => {
                encode_fixture(&TransactionsResponse {
                    transactions: self
                        .my_resolutions()
                        .into_iter()
                        .filter_map(|resolution| resolution.canonical)
                        .collect(),
                })
            }
            (contract::transactions::ZOME, contract::transactions::GET_MY_TRANSACTION_RESOLUTIONS) => {
                encode_fixture(&TransactionResolutionsResponse {
                    resolutions: self.my_resolutions(),
                })
            }
            (contract::transactions::ZOME, contract::transactions::CONFIRM_TRANSACTION) => {
                let requested: ActionHash = rmp_serde::from_slice(&payload)
                    .map_err(|error| ClientError::Decode(error.to_string()))?;
                encode_fixture(&self.update_status(
                    function,
                    requested,
                    TransactionStatus::Confirmed,
                    None,
                )?)
            }
            (contract::transactions::ZOME, contract::transactions::MARK_SHIPPED) => {
                let input: MarkShippedInput = rmp_serde::from_slice(&payload)
                    .map_err(|error| ClientError::Decode(error.to_string()))?;
                encode_fixture(&self.update_status(
                    function,
                    input.transaction_hash,
                    TransactionStatus::Shipped,
                    input.tracking_info,
                )?)
            }
            (contract::transactions::ZOME, contract::transactions::CONFIRM_DELIVERY) => {
                let requested: ActionHash = rmp_serde::from_slice(&payload)
                    .map_err(|error| ClientError::Decode(error.to_string()))?;
                let existing = self
                    .find_resolution(&requested)
                    .and_then(|resolution| resolution.canonical)
                    .ok_or_else(|| fixture_error(function, "fixture transaction not found"))?;
                let delivered = if existing.transaction.status == TransactionStatus::Delivered {
                    if self.acting_agent != existing.transaction.buyer {
                        return Err(fixture_error(
                            function,
                            "only the fixture buyer may confirm delivery",
                        ));
                    }
                    existing
                } else {
                    self.update_status(
                        function,
                        requested,
                        TransactionStatus::Delivered,
                        None,
                    )?
                };
                self.record_fulfillment_event(delivered.transaction_hash.clone())?;
                encode_fixture(&delivered)
            }
            (contract::transactions::ZOME, contract::transactions::SETTLE_TRANSACTION) => {
                let requested: ActionHash = rmp_serde::from_slice(&payload)
                    .map_err(|error| ClientError::Decode(error.to_string()))?;
                encode_fixture(&self.settlement_result(requested, true)?)
            }
            (contract::transactions::ZOME, contract::transactions::GET_TRANSACTION_SETTLEMENT_STATUS) => {
                let requested: ActionHash = rmp_serde::from_slice(&payload)
                    .map_err(|error| ClientError::Decode(error.to_string()))?;
                encode_fixture(&self.settlement_result(requested, false)?)
            }
            (contract::transactions::ZOME, contract::transactions::CANCEL_TRANSACTION) => {
                let requested: ActionHash = rmp_serde::from_slice(&payload)
                    .map_err(|error| ClientError::Decode(error.to_string()))?;
                encode_fixture(&self.update_status(
                    function,
                    requested,
                    TransactionStatus::Cancelled,
                    None,
                )?)
            }
            (contract::transactions::ZOME, contract::transactions::OPEN_DISPUTE) => {
                let input: OpenDisputeInput = rmp_serde::from_slice(&payload)
                    .map_err(|error| ClientError::Decode(error.to_string()))?;
                encode_fixture(&self.open_dispute(input)?)
            }
            (contract::arbitration::ZOME, contract::arbitration::REGISTER_AS_ARBITRATOR) => {
                self.register_arbitrator();
                encode_fixture(&())
            }
            (contract::arbitration::ZOME, contract::arbitration::GET_ARBITRATION_OPPORTUNITIES) => {
                encode_fixture(&self.arbitration_opportunities())
            }
            (contract::arbitration::ZOME, contract::arbitration::GET_DISPUTE) => {
                let requested: ActionHash = rmp_serde::from_slice(&payload)
                    .map_err(|error| ClientError::Decode(error.to_string()))?;
                encode_fixture(
                    &self
                        .find_dispute_resolution(&requested)
                        .and_then(|resolution| resolution.canonical),
                )
            }
            (contract::arbitration::ZOME, contract::arbitration::GET_DISPUTE_RESOLUTION) => {
                let requested: ActionHash = rmp_serde::from_slice(&payload)
                    .map_err(|error| ClientError::Decode(error.to_string()))?;
                encode_fixture(&self.find_dispute_resolution(&requested))
            }
            (contract::arbitration::ZOME, contract::arbitration::SUBMIT_ARBITRATION_VOTE) => {
                let input: SubmitArbitrationVoteInput = rmp_serde::from_slice(&payload)
                    .map_err(|error| ClientError::Decode(error.to_string()))?;
                encode_fixture(&self.submit_vote(input)?)
            }
            (contract::arbitration::ZOME, contract::arbitration::GET_ARBITRATION_VOTES) => {
                let requested: ActionHash = rmp_serde::from_slice(&payload)
                    .map_err(|error| ClientError::Decode(error.to_string()))?;
                let state = self.state.borrow();
                let votes = state
                    .disputes
                    .iter()
                    .find(|revision| revision.contains(&requested))
                    .map(|revision| revision.votes.clone())
                    .ok_or_else(|| arbitration_fixture_error(function, "fixture dispute not found"))?;
                encode_fixture(&ArbitrationVotesResponse { votes })
            }
            (contract::arbitration::ZOME, contract::arbitration::FINALIZE_ARBITRATION) => {
                let requested: ActionHash = rmp_serde::from_slice(&payload)
                    .map_err(|error| ClientError::Decode(error.to_string()))?;
                let result = self.finalize_dispute(requested)?;
                self.project_arbitration_events(result.result_hash.clone())?;
                encode_fixture(&result)
            }
            (contract::arbitration::ZOME, contract::arbitration::GET_ARBITRATION_RESULT) => {
                let requested: ActionHash = rmp_serde::from_slice(&payload)
                    .map_err(|error| ClientError::Decode(error.to_string()))?;
                let state = self.state.borrow();
                let result = state
                    .disputes
                    .iter()
                    .find(|revision| revision.contains(&requested))
                    .and_then(|revision| revision.result.clone());
                encode_fixture(&result)
            }
            (contract::reputation::ZOME, contract::reputation::RECORD_FULFILLMENT_REPUTATION) => {
                let requested: ActionHash = rmp_serde::from_slice(&payload)
                    .map_err(|error| ClientError::Decode(error.to_string()))?;
                encode_fixture(&self.record_fulfillment_event(requested)?)
            }
            (contract::reputation::ZOME, contract::reputation::PROJECT_ARBITRATION_REPUTATION) => {
                let result_hash: ActionHash = rmp_serde::from_slice(&payload)
                    .map_err(|error| ClientError::Decode(error.to_string()))?;
                encode_fixture(&self.project_arbitration_events(result_hash)?)
            }
            (contract::reputation::ZOME, contract::reputation::GET_AGENT_REPUTATION_EVENTS) => {
                let agent: AgentPubKey = rmp_serde::from_slice(&payload)
                    .map_err(|error| ClientError::Decode(error.to_string()))?;
                encode_fixture(&self.reputation_events(agent))
            }
            (contract::reputation::ZOME, contract::reputation::GET_DERIVED_REPUTATION) => {
                let agent: AgentPubKey = rmp_serde::from_slice(&payload)
                    .map_err(|error| ClientError::Decode(error.to_string()))?;
                encode_fixture(&self.derived_reputation(agent))
            }
            _ => Err(ClientError::Call {
                zome: zome.into(),
                function: function.into(),
                message: "fixture transport implements only the guarded lifecycle slice".into(),
            }),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::MarketplaceClient;

    #[test]
    fn fixtures_are_explicit_and_have_distinct_sellers() {
        let listings = fixture_listings();
        assert_eq!(listings.len(), 2);
        assert!(listings.iter().all(|item| item.listing.price_cents > 0));
        assert!(
            listings
                .iter()
                .all(|item| item.seller_agent_id != fixture_agent())
        );
    }

    #[test]
    fn fixture_hashes_are_stable() {
        let first = fixture_listings().remove(0);
        assert_eq!(first.listing_hash.as_bytes().len(), 39);
    }

    #[test]
    fn fixture_revisions_resolve_old_and_new_hashes_to_one_head() {
        let buyer = FixtureTransport::default();
        let listing = fixture_listings().remove(0);
        let pending = buyer
            .create_transaction(CreateTransactionInput {
                seller: listing.seller_agent_id.clone(),
                listing_hash: listing.listing_hash,
                quantity: 1,
                total_price_cents: listing.listing.price_cents,
            })
            .unwrap();
        let seller = buyer.acting_as(listing.seller_agent_id);
        let confirmed = seller
            .update_status(
                contract::transactions::CONFIRM_TRANSACTION,
                pending.transaction_hash.clone(),
                TransactionStatus::Confirmed,
                None,
            )
            .unwrap();

        let from_root = buyer.find_resolution(&pending.transaction_hash).unwrap();
        let from_head = buyer.find_resolution(&confirmed.transaction_hash).unwrap();
        assert_eq!(from_root, from_head);
        assert_eq!(from_root.revision_count, 2);
        assert_eq!(
            from_root.current().unwrap().transaction.status,
            TransactionStatus::Confirmed
        );
    }

    #[test]
    fn fixture_arbitration_is_equal_weight_and_root_stable() {
        let buyer = FixtureTransport::default();
        let listing = fixture_listings().remove(0);
        let pending = buyer
            .create_transaction(CreateTransactionInput {
                seller: listing.seller_agent_id.clone(),
                listing_hash: listing.listing_hash,
                quantity: 1,
                total_price_cents: listing.listing.price_cents,
            })
            .unwrap();
        let arbitrator_agent = AgentPubKey::new(vec![0x77; 39]);
        let arbitrator = buyer.acting_as(arbitrator_agent.clone());
        arbitrator.register_arbitrator();

        let opened = buyer
            .open_dispute(OpenDisputeInput {
                transaction_hash: pending.transaction_hash,
                reason: "Item evidence does not match the listing".into(),
                evidence_cids: Vec::new(),
            })
            .unwrap();
        let assigned = buyer
            .find_dispute_resolution(&opened.dispute_hash)
            .unwrap();
        assert_eq!(assigned.root_dispute_hash, opened.dispute_hash);
        assert_eq!(assigned.current().unwrap().dispute.arbitrators, vec![arbitrator_agent]);

        let vote = arbitrator
            .submit_vote(SubmitArbitrationVoteInput {
                dispute_hash: opened.dispute_hash.clone(),
                favor_buyer: true,
                reasoning: "Fixture evidence supports the buyer".into(),
            })
            .unwrap();
        assert_eq!(vote.vote.arbitrator_matl_score, 1.0);

        let result = arbitrator
            .finalize_dispute(opened.dispute_hash.clone())
            .unwrap();
        assert_eq!(result.result.dispute_hash, opened.dispute_hash);
        assert_eq!(result.result.vote_hashes, vec![vote.vote_hash]);
        assert_eq!(result.result.weighted_vote, 1.0);
        assert_eq!(result.result.winner, fixture_agent());
        let winner = arbitrator.derived_reputation(fixture_agent());
        let loser = arbitrator.derived_reputation(listing.seller_agent_id);
        assert_eq!(winner.positive_events, 1);
        assert_eq!(loser.negative_events, 1);
    }


    #[test]
    fn fixture_settlement_is_idempotent_and_external_to_lifecycle() {
        let buyer = FixtureTransport::default().with_finance();
        let listing = fixture_listings().remove(0);
        let seller = buyer.acting_as(listing.seller_agent_id.clone());
        let pending = buyer
            .create_transaction(CreateTransactionInput {
                seller: listing.seller_agent_id.clone(),
                listing_hash: listing.listing_hash,
                quantity: 1,
                total_price_cents: listing.listing.price_cents,
            })
            .unwrap();
        let confirmed = seller
            .update_status(
                contract::transactions::CONFIRM_TRANSACTION,
                pending.transaction_hash.clone(),
                TransactionStatus::Confirmed,
                None,
            )
            .unwrap();
        let shipped = seller
            .update_status(
                contract::transactions::MARK_SHIPPED,
                confirmed.transaction_hash,
                TransactionStatus::Shipped,
                Some("fixture-tracking".into()),
            )
            .unwrap();
        let delivered = buyer
            .update_status(
                contract::transactions::CONFIRM_DELIVERY,
                shipped.transaction_hash,
                TransactionStatus::Delivered,
                None,
            )
            .unwrap();

        let first = buyer
            .settlement_result(delivered.transaction_hash.clone(), true)
            .unwrap();
        let second = buyer
            .settlement_result(pending.transaction_hash.clone(), true)
            .unwrap();
        assert!(first.settled && second.settled);
        assert_eq!(first.finance_payment_id, second.finance_payment_id);
        assert_eq!(first.idempotency_reference, second.idempotency_reference);
        assert_eq!(
            buyer
                .find_resolution(&pending.transaction_hash)
                .unwrap()
                .current()
                .unwrap()
                .transaction
                .status,
            TransactionStatus::Delivered
        );
    }

    #[test]
    fn fixture_fulfillment_reputation_is_idempotent() {
        let buyer = FixtureTransport::default();
        let listing = fixture_listings().remove(0);
        let seller = buyer.acting_as(listing.seller_agent_id.clone());
        let pending = buyer
            .create_transaction(CreateTransactionInput {
                seller: listing.seller_agent_id.clone(),
                listing_hash: listing.listing_hash,
                quantity: 1,
                total_price_cents: listing.listing.price_cents,
            })
            .unwrap();
        let confirmed = seller
            .update_status(
                contract::transactions::CONFIRM_TRANSACTION,
                pending.transaction_hash.clone(),
                TransactionStatus::Confirmed,
                None,
            )
            .unwrap();
        let shipped = seller
            .update_status(
                contract::transactions::MARK_SHIPPED,
                confirmed.transaction_hash,
                TransactionStatus::Shipped,
                Some("fixture-tracking".into()),
            )
            .unwrap();
        let delivered = buyer
            .update_status(
                contract::transactions::CONFIRM_DELIVERY,
                shipped.transaction_hash,
                TransactionStatus::Delivered,
                None,
            )
            .unwrap();

        let first = buyer
            .record_fulfillment_event(delivered.transaction_hash.clone())
            .unwrap();
        let second = buyer
            .record_fulfillment_event(pending.transaction_hash)
            .unwrap();
        assert_eq!(first, second);
        let summary = buyer.derived_reputation(listing.seller_agent_id);
        assert_eq!(summary.event_count, 1);
        assert_eq!(summary.positive_events, 1);
        assert_eq!(summary.negative_events, 0);
        assert_eq!(summary.fulfilled_value_cents, listing.listing.price_cents);
    }

    #[test]
    fn fixture_actor_rules_reject_buyer_confirmation() {
        let buyer = FixtureTransport::default();
        let listing = fixture_listings().remove(0);
        let pending = buyer
            .create_transaction(CreateTransactionInput {
                seller: listing.seller_agent_id,
                listing_hash: listing.listing_hash,
                quantity: 1,
                total_price_cents: listing.listing.price_cents,
            })
            .unwrap();
        assert!(
            buyer
                .update_status(
                    contract::transactions::CONFIRM_TRANSACTION,
                    pending.transaction_hash,
                    TransactionStatus::Confirmed,
                    None,
                )
                .is_err()
        );
    }

    #[allow(dead_code)]
    fn client_type_compiles() {
        let _client = MarketplaceClient::new(FixtureTransport::default());
    }
}
