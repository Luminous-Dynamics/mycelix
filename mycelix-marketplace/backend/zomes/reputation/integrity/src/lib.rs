// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
use hdi::prelude::*;

/// MATL Score Entry - Mycelix Adaptive Trust Layer
///
/// Legacy mutable MATL projection retained for wire compatibility.
///
/// New non-neutral scores cannot be created by this integrity zome. Canonical
/// Marketplace reputation is derived from immutable `ReputationEvent` evidence
/// and makes no Byzantine-tolerance claim.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct MatlScore {
    /// Agent being scored
    pub agent: AgentPubKey,

    /// Proof of Gradient Quality metrics
    pub pogq: ProofOfGradientQuality,

    /// Reputation history score (0.0 - 1.0)
    pub reputation: f64,

    /// Composite trust score (weighted combination)
    /// Formula: 0.4 * quality + 0.3 * consistency + 0.3 * reputation
    pub composite: f64,

    /// Number of transactions completed
    pub transaction_count: u32,

    /// Total value transacted (in cents)
    pub total_value_cents: u64,

    /// Timestamp of last update
    pub updated_at: Timestamp,

    /// Byzantine detection flags
    pub flags: ByzantineFlags,
}

/// Proof of Gradient Quality - Core Trust Mechanism
///
/// Measures consistency and quality of an agent's behavior
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct ProofOfGradientQuality {
    /// Quality score [0.0, 1.0]
    /// Measures how good the agent's contributions are
    pub quality: f64,

    /// Consistency over time [0.0, 1.0]
    /// Measures reliability of behavior
    pub consistency: f64,

    /// Entropy measure
    /// Low entropy = predictable (good), High entropy = erratic (suspicious)
    pub entropy: f64,

    /// Timestamp of measurement
    pub timestamp: Timestamp,
}

/// Byzantine Detection Flags
///
/// Identifies suspicious patterns that indicate malicious behavior
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct ByzantineFlags {
    /// Detected as part of a cartel (coordinated attack)
    pub cartel_detected: bool,

    /// Rapid reputation changes (suspicious)
    pub volatile_reputation: bool,

    /// Gradient poisoning detected (FL-specific)
    pub gradient_poisoning: bool,

    /// Sybil attack patterns
    pub sybil_suspected: bool,

    /// Overall Byzantine risk score [0.0, 1.0]
    /// Above 0.5 = likely Byzantine
    pub risk_score: f64,
}

/// Immutable evidence-derived reputation event.
///
/// Unlike the legacy mutable MATL projection, every accepted event binds a
/// same-DNA transaction or arbitration record that integrity validation can
/// independently retrieve and verify.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct ReputationEvent {
    /// Deterministic semantic deduplication key.
    pub event_key: String,
    /// Agent whose behavior this event describes.
    pub subject: AgentPubKey,
    /// Counterparty in the underlying exchange.
    pub counterparty: AgentPubKey,
    /// Stable transaction Create action.
    pub transaction_hash: ActionHash,
    /// Delivered transaction revision or ArbitrationResult action.
    pub source_hash: ActionHash,
    pub kind: ReputationEventKind,
    /// Authoritative transaction value; not a caller-selected score weight.
    pub value_cents: u64,
    pub occurred_at: Timestamp,
}

#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum ReputationEventKind {
    FulfillmentDelivered,
    ArbitrationWon,
    ArbitrationLost,
}

/// Review Entry - Verifiable feedback from transactions
///
/// Reviews upgrade listings from E1 (seller claim) to E2 (buyer verified)
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Review {
    /// Transaction this review is for
    pub transaction_hash: ActionHash,

    /// Listing being reviewed
    pub listing_hash: ActionHash,

    /// Star rating (1-5)
    pub rating: u8,

    /// Review comment
    pub comment: String,

    /// Reviewer agent
    pub reviewer: AgentPubKey,

    /// Seller agent
    pub seller: AgentPubKey,

    /// Review timestamp
    pub created_at: Timestamp,

    /// Epistemic classification
    /// Reviews are E2 (privately verifiable) - only buyer can verify their own experience
    pub epistemic: EpistemicClassification,
}

/// Epistemic classification (same as listings)
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct EpistemicClassification {
    pub empirical: EmpiricalLevel,
    pub normative: NormativeLevel,
    pub materiality: MaterialityLevel,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum EmpiricalLevel {
    E0Null,
    E1Testimonial,
    E2PrivateVerify,
    E3Cryptographic,
    E4PublicRepro,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum NormativeLevel {
    N0Personal,
    N1Communal,
    N2Network,
    N3Axiomatic,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum MaterialityLevel {
    M0Ephemeral,
    M1Temporal,
    M2Persistent,
    M3Foundational,
}

/// Link types for reputation data
#[hdk_link_types]
pub enum LinkTypes {
    /// Agent -> MatlScore
    AgentToScore,

    /// Agent -> Reviews (as seller)
    AgentToSellerReviews,

    /// Agent -> Reviews (as buyer)
    AgentToBuyerReviews,

    /// Transaction -> Review
    TransactionToReview,

    /// Subject agent -> immutable evidence-derived events.
    AgentToReputationEvents,

    /// Source transaction/result action -> projected events.
    SourceToReputationEvents,
}

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    MatlScore(MatlScore),
    Review(Review),
    ReputationEvent(ReputationEvent),
}

/// Validation for reputation entries
#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(store_entry) => match store_entry {
            OpEntry::CreateEntry { app_entry, action } => match app_entry {
                EntryTypes::MatlScore(score) => validate_matl_score(&score, &action),
                EntryTypes::Review(review) => validate_review(&review, &action),
                EntryTypes::ReputationEvent(event) => validate_reputation_event(&event, &action),
            },
            // MatlScore/Review are confirmed create-only (update_matl_score is
            // coordinator-disabled unconditionally, and no coordinator function ever calls
            // update_entry on Review) -- reject outright rather than leave the previous
            // unbound dead-code path (P0 wide-open RegisterUpdate gap, confirmed dozens of
            // times elsewhere in this pass).
            OpEntry::UpdateEntry { app_entry, .. } => match app_entry {
                EntryTypes::ReputationEvent(_) => Ok(ValidateCallbackResult::Invalid(
                    "Reputation events are immutable".into(),
                )),
                EntryTypes::MatlScore(_) => Ok(ValidateCallbackResult::Invalid(
                    "MatlScore entries cannot be updated".into(),
                )),
                EntryTypes::Review(_) => Ok(ValidateCallbackResult::Invalid(
                    "Review entries cannot be updated".into(),
                )),
            },
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterDelete(_) => Ok(ValidateCallbackResult::Invalid(
            "Reputation evidence is permanent and cannot be deleted".into(),
        )),
        _ => Ok(ValidateCallbackResult::Valid),
    }
}

fn validate_matl_score(score: &MatlScore, action: &Create) -> ExternResult<ValidateCallbackResult> {
    // Validate score ranges
    if score.pogq.quality < 0.0
        || score.pogq.quality > 1.0
        || score.pogq.consistency < 0.0
        || score.pogq.consistency > 1.0
        || score.reputation < 0.0
        || score.reputation > 1.0
        || score.composite < 0.0
        || score.composite > 1.0
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Score values must be between 0.0 and 1.0".into(),
        ));
    }

    // Verify composite score calculation
    let expected_composite =
        0.4 * score.pogq.quality + 0.3 * score.pogq.consistency + 0.3 * score.reputation;

    let diff = (score.composite - expected_composite).abs();
    if diff > 0.01 {
        return Ok(ValidateCallbackResult::Invalid(
            "Composite score calculation incorrect".into(),
        ));
    }

    let neutral = action.author == score.agent
        && (score.pogq.quality - 0.5).abs() <= f64::EPSILON
        && (score.pogq.consistency - 0.5).abs() <= f64::EPSILON
        && score.pogq.entropy.abs() <= f64::EPSILON
        && (score.reputation - 0.5).abs() <= f64::EPSILON
        && (score.composite - 0.5).abs() <= f64::EPSILON
        && score.transaction_count == 0
        && score.total_value_cents == 0
        && !score.flags.cartel_detected
        && !score.flags.volatile_reputation
        && !score.flags.gradient_poisoning
        && !score.flags.sybil_suspected
        && score.flags.risk_score.abs() <= f64::EPSILON;
    if !neutral {
        return Ok(ValidateCallbackResult::Invalid(
            "Legacy MATL entries may only be self-authored neutral bootstrap records".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

const MAX_REPUTATION_EVENT_KEY_LEN: usize = 512;
const MAX_REPUTATION_REVISION_DEPTH: usize = 32;

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
struct TransactionSnapshot {
    buyer: AgentPubKey,
    seller: AgentPubKey,
    total_price_cents: u64,
    status: TransactionStatusSnapshot,
    updated_at: Timestamp,
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

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
struct ArbitrationResultSnapshot {
    dispute_hash: ActionHash,
    dispute_revision_hash: ActionHash,
    winner: AgentPubKey,
    loser: AgentPubKey,
    finalized_at: Timestamp,
}

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
struct DisputeSnapshot {
    transaction_hash: ActionHash,
    transaction_revision_hash: ActionHash,
    buyer: AgentPubKey,
    seller: AgentPubKey,
    arbitrators: Vec<AgentPubKey>,
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

fn validate_reputation_event_data(event: &ReputationEvent) -> Result<(), String> {
    if event.event_key.is_empty() || event.event_key.len() > MAX_REPUTATION_EVENT_KEY_LEN {
        return Err("Reputation event key is required and must fit the size limit".into());
    }
    if event.subject == event.counterparty {
        return Err("Reputation event subject and counterparty must differ".into());
    }
    if event.value_cents == 0 {
        return Err("Reputation event value must be positive".into());
    }
    Ok(())
}

fn validate_reputation_event(
    event: &ReputationEvent,
    action: &Create,
) -> ExternResult<ValidateCallbackResult> {
    if let Err(reason) = validate_reputation_event_data(event) {
        return Ok(ValidateCallbackResult::Invalid(reason));
    }

    match event.kind {
        ReputationEventKind::FulfillmentDelivered => validate_fulfillment_event(event, action),
        ReputationEventKind::ArbitrationWon | ReputationEventKind::ArbitrationLost => {
            validate_arbitration_event(event, action)
        }
    }
}

fn validate_fulfillment_event(
    event: &ReputationEvent,
    action: &Create,
) -> ExternResult<ValidateCallbackResult> {
    let transaction_record = must_get_valid_record(event.source_hash.clone())?;
    let transaction: TransactionSnapshot = decode_record(&transaction_record)?;
    let root = find_action_root(event.source_hash.clone())?;

    if root != event.transaction_hash {
        return Ok(ValidateCallbackResult::Invalid(
            "Fulfillment event source does not belong to the declared transaction root".into(),
        ));
    }
    if transaction.status != TransactionStatusSnapshot::Delivered {
        return Ok(ValidateCallbackResult::Invalid(
            "Fulfillment reputation requires an exact Delivered transaction revision".into(),
        ));
    }
    if action.author != transaction.buyer
        || event.subject != transaction.seller
        || event.counterparty != transaction.buyer
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Fulfillment reputation must be buyer-authored for the authoritative seller".into(),
        ));
    }
    if event.value_cents != transaction.total_price_cents
        || event.occurred_at != transaction.updated_at
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Fulfillment reputation value and timestamp must match the transaction".into(),
        ));
    }
    if event.event_key != fulfillment_event_key(&root, &transaction.seller) {
        return Ok(ValidateCallbackResult::Invalid(
            "Fulfillment reputation event key is not canonical".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_arbitration_event(
    event: &ReputationEvent,
    action: &Create,
) -> ExternResult<ValidateCallbackResult> {
    let result_record = must_get_valid_record(event.source_hash.clone())?;
    let result: ArbitrationResultSnapshot = decode_record(&result_record)?;
    let dispute: DisputeSnapshot = decode_record(&must_get_valid_record(
        result.dispute_revision_hash.clone(),
    )?)?;
    let dispute_root = find_action_root(result.dispute_revision_hash.clone())?;
    if dispute_root != result.dispute_hash || dispute.transaction_hash != event.transaction_hash {
        return Ok(ValidateCallbackResult::Invalid(
            "Arbitration reputation event is not bound to the declared dispute and transaction"
                .into(),
        ));
    }
    if !dispute.arbitrators.contains(&action.author) {
        return Ok(ValidateCallbackResult::Invalid(
            "Arbitration reputation projection must be authored by an assigned arbitrator".into(),
        ));
    }

    let transaction: TransactionSnapshot = decode_record(&must_get_valid_record(
        dispute.transaction_revision_hash.clone(),
    )?)?;
    if transaction.buyer != dispute.buyer || transaction.seller != dispute.seller {
        return Ok(ValidateCallbackResult::Invalid(
            "Arbitration dispute parties do not match the transaction".into(),
        ));
    }

    let expected_counterparty = if event.subject == result.winner {
        if event.kind != ReputationEventKind::ArbitrationWon {
            return Ok(ValidateCallbackResult::Invalid(
                "Arbitration winner must receive an ArbitrationWon event".into(),
            ));
        }
        result.loser.clone()
    } else if event.subject == result.loser {
        if event.kind != ReputationEventKind::ArbitrationLost {
            return Ok(ValidateCallbackResult::Invalid(
                "Arbitration loser must receive an ArbitrationLost event".into(),
            ));
        }
        result.winner.clone()
    } else {
        return Ok(ValidateCallbackResult::Invalid(
            "Arbitration reputation subject must be the result winner or loser".into(),
        ));
    };

    if event.counterparty != expected_counterparty
        || event.value_cents != transaction.total_price_cents
        || event.occurred_at != result.finalized_at
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Arbitration reputation terms must match the validated result and transaction".into(),
        ));
    }
    if event.event_key != arbitration_event_key(&event.source_hash, &event.kind, &event.subject) {
        return Ok(ValidateCallbackResult::Invalid(
            "Arbitration reputation event key is not canonical".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
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
    for _ in 0..=MAX_REPUTATION_REVISION_DEPTH {
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
                    "Reputation source must be a Create or Update action".into()
                )));
            }
        }
    }
    Err(wasm_error!(WasmErrorInner::Guest(
        "Update ancestry exceeds the reputation depth limit".into()
    )))
}

fn validate_review(review: &Review, action: &Create) -> ExternResult<ValidateCallbackResult> {
    // Rating must be 1-5
    if review.rating < 1 || review.rating > 5 {
        return Ok(ValidateCallbackResult::Invalid(
            "Rating must be between 1 and 5".into(),
        ));
    }

    // Comment must not be empty and not too long
    if review.comment.is_empty() || review.comment.len() > 1000 {
        return Ok(ValidateCallbackResult::Invalid(
            "Comment must be 1-1000 characters".into(),
        ));
    }

    // Reviews should be E2 (privately verifiable)
    if review.epistemic.empirical != EmpiricalLevel::E2PrivateVerify {
        return Ok(ValidateCallbackResult::Invalid(
            "Reviews must be E2 (privately verifiable)".into(),
        ));
    }

    // Bind the review to its committer -- submit_review already derives reviewer from
    // agent_info() coordinator-side with zero user input (P0 author-binding gap).
    if review.reviewer != action.author {
        return Ok(ValidateCallbackResult::Invalid(
            "Review must be created by its reviewer (reviewer forgery)".to_string(),
        ));
    }

    // `seller` was previously a raw, caller-supplied reference with zero correlation check
    // to the actual transaction's real seller -- any agent could attribute a review to an
    // arbitrary agent as "the seller" for any transaction_hash. Cross-verify against the
    // real transaction using the same local-snapshot-decode pattern ReputationEvent already
    // establishes in this file (TransactionSnapshot / decode_record) rather than adding a
    // cross-crate dependency on transactions_integrity.
    let transaction: TransactionSnapshot =
        decode_record(&must_get_valid_record(review.transaction_hash.clone())?)?;
    if review.reviewer != transaction.buyer || review.seller != transaction.seller {
        return Ok(ValidateCallbackResult::Invalid(
            "Review reviewer/seller must match the referenced transaction's real buyer/seller"
                .to_string(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

#[cfg(test)]
mod reputation_event_tests {
    use super::*;

    fn agent(byte: u8) -> AgentPubKey {
        AgentPubKey::from_raw_36(vec![byte; 36])
    }

    fn event() -> ReputationEvent {
        ReputationEvent {
            event_key: "event-key".into(),
            subject: agent(1),
            counterparty: agent(2),
            transaction_hash: ActionHash::from_raw_36(vec![3; 36]),
            source_hash: ActionHash::from_raw_36(vec![4; 36]),
            kind: ReputationEventKind::FulfillmentDelivered,
            value_cents: 100,
            occurred_at: Timestamp::from_micros(1),
        }
    }

    #[test]
    fn reputation_event_requires_distinct_parties() {
        let mut value = event();
        value.counterparty = value.subject.clone();
        assert!(validate_reputation_event_data(&value).is_err());
    }

    #[test]
    fn reputation_event_requires_positive_value() {
        let mut value = event();
        value.value_cents = 0;
        assert!(validate_reputation_event_data(&value).is_err());
    }

    #[test]
    fn reputation_event_keys_are_deterministic() {
        let tx = ActionHash::from_raw_36(vec![5; 36]);
        let subject = agent(6);
        assert_eq!(
            fulfillment_event_key(&tx, &subject),
            fulfillment_event_key(&tx, &subject)
        );
        assert_eq!(
            arbitration_event_key(&tx, &ReputationEventKind::ArbitrationWon, &subject),
            arbitration_event_key(&tx, &ReputationEventKind::ArbitrationWon, &subject)
        );
    }
}

/// Proves `validate_review`'s P0 author-binding fix: a Review naming a `seller` that
/// doesn't match the referenced transaction's real seller is rejected (previously `seller`
/// was a raw, uncorrelated caller-supplied reference). Mocks the HDI host's
/// `must_get_valid_record` so this runs as a plain `cargo test`, no live conductor needed.
#[cfg(test)]
mod review_tests {
    use super::*;

    struct MockRecordHdi {
        records: std::collections::HashMap<ActionHash, Record>,
    }

    impl hdi::hdi::HdiT for MockRecordHdi {
        fn must_get_valid_record(&self, input: MustGetValidRecordInput) -> ExternResult<Record> {
            self.records
                .get(&input.0)
                .cloned()
                .ok_or_else(|| wasm_error!(WasmErrorInner::Guest("no such record in mock".into())))
        }
        fn verify_signature(&self, _: VerifySignature) -> ExternResult<bool> {
            unimplemented!("not exercised by this fix")
        }
        fn must_get_entry(&self, _: MustGetEntryInput) -> ExternResult<EntryHashed> {
            unimplemented!("not exercised by this fix")
        }
        fn must_get_action(&self, _: MustGetActionInput) -> ExternResult<SignedActionHashed> {
            unimplemented!("not exercised by this fix")
        }
        fn must_get_agent_activity(
            &self,
            _: MustGetAgentActivityInput,
        ) -> ExternResult<Vec<RegisterAgentActivity>> {
            unimplemented!("not exercised by this fix")
        }
        fn dna_info(&self, _: ()) -> ExternResult<DnaInfo> {
            unimplemented!("not exercised by this fix")
        }
        fn zome_info(&self, _: ()) -> ExternResult<ZomeInfo> {
            unimplemented!("not exercised by this fix")
        }
        fn trace(&self, _: TraceMsg) -> ExternResult<()> {
            unimplemented!("not exercised by this fix")
        }
        fn x_salsa20_poly1305_decrypt(
            &self,
            _: XSalsa20Poly1305Decrypt,
        ) -> ExternResult<Option<XSalsa20Poly1305Data>> {
            unimplemented!("not exercised by this fix")
        }
        fn x_25519_x_salsa20_poly1305_decrypt(
            &self,
            _: X25519XSalsa20Poly1305Decrypt,
        ) -> ExternResult<Option<XSalsa20Poly1305Data>> {
            unimplemented!("not exercised by this fix")
        }
        fn ed_25519_x_salsa20_poly1305_decrypt(
            &self,
            _: Ed25519XSalsa20Poly1305Decrypt,
        ) -> ExternResult<XSalsa20Poly1305Data> {
            unimplemented!("not exercised by this fix")
        }
    }

    fn wrap_entry_record<T>(author: AgentPubKey, value: T) -> Record
    where
        T: TryInto<SerializedBytes>,
        <T as TryInto<SerializedBytes>>::Error: std::fmt::Debug,
    {
        let entry = Entry::App(AppEntryBytes::try_from(value.try_into().unwrap()).unwrap());
        let action = Action::Create(Create {
            author,
            timestamp: Timestamp::from_micros(0),
            action_seq: 0,
            prev_action: ActionHash::from_raw_36(vec![0; 36]),
            entry_type: EntryType::App(AppEntryDef::new(
                EntryDefIndex(0),
                ZomeIndex(0),
                EntryVisibility::Public,
            )),
            entry_hash: EntryHash::from_raw_36(vec![1; 36]),
            weight: Default::default(),
        });
        let hashed = HoloHashed::from_content_sync(action);
        let signed_action = SignedActionHashed::with_presigned(hashed, Signature([0; 64]));
        Record::new(signed_action, Some(entry))
    }

    fn test_review(
        reviewer: AgentPubKey,
        seller: AgentPubKey,
        transaction_hash: ActionHash,
    ) -> Review {
        Review {
            transaction_hash,
            listing_hash: ActionHash::from_raw_36(vec![40; 36]),
            rating: 5,
            comment: "great seller".to_string(),
            reviewer,
            seller,
            created_at: Timestamp::from_micros(0),
            epistemic: EpistemicClassification {
                empirical: EmpiricalLevel::E2PrivateVerify,
                normative: NormativeLevel::N1Communal,
                materiality: MaterialityLevel::M1Temporal,
            },
        }
    }

    fn test_action(author: AgentPubKey) -> Create {
        Create {
            author,
            timestamp: Timestamp::from_micros(0),
            action_seq: 0,
            prev_action: ActionHash::from_raw_36(vec![0; 36]),
            entry_type: EntryType::App(AppEntryDef::new(
                EntryDefIndex(0),
                ZomeIndex(0),
                EntryVisibility::Public,
            )),
            entry_hash: EntryHash::from_raw_36(vec![1; 36]),
            weight: Default::default(),
        }
    }

    #[test]
    fn review_naming_a_seller_not_in_the_real_transaction_is_rejected() {
        let buyer = AgentPubKey::from_raw_36(vec![1; 36]);
        let real_seller = AgentPubKey::from_raw_36(vec![2; 36]);
        let framed_seller = AgentPubKey::from_raw_36(vec![3; 36]);
        let tx_hash = ActionHash::from_raw_36(vec![10; 36]);

        let snapshot = TransactionSnapshot {
            buyer: buyer.clone(),
            seller: real_seller,
            total_price_cents: 1000,
            status: TransactionStatusSnapshot::Completed,
            updated_at: Timestamp::from_micros(0),
        };
        hdi::hdi::set_hdi(MockRecordHdi {
            records: std::collections::HashMap::from([(
                tx_hash.clone(),
                wrap_entry_record(buyer.clone(), snapshot),
            )]),
        });

        let review = test_review(buyer.clone(), framed_seller, tx_hash);
        let result = validate_review(&review, &test_action(buyer)).unwrap();
        assert!(
            matches!(result, ValidateCallbackResult::Invalid(_)),
            "a Review naming a seller who isn't the real transaction's seller must be \
             rejected -- previously seller was a raw, uncorrelated caller-supplied reference"
        );
    }

    #[test]
    fn review_naming_the_real_seller_is_accepted() {
        let buyer = AgentPubKey::from_raw_36(vec![1; 36]);
        let real_seller = AgentPubKey::from_raw_36(vec![2; 36]);
        let tx_hash = ActionHash::from_raw_36(vec![10; 36]);

        let snapshot = TransactionSnapshot {
            buyer: buyer.clone(),
            seller: real_seller.clone(),
            total_price_cents: 1000,
            status: TransactionStatusSnapshot::Completed,
            updated_at: Timestamp::from_micros(0),
        };
        hdi::hdi::set_hdi(MockRecordHdi {
            records: std::collections::HashMap::from([(
                tx_hash.clone(),
                wrap_entry_record(buyer.clone(), snapshot),
            )]),
        });

        let review = test_review(buyer.clone(), real_seller, tx_hash);
        let result = validate_review(&review, &test_action(buyer)).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Valid));
    }
}
