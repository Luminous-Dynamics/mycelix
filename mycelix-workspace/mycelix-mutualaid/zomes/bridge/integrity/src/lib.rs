// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root//! Bridge Integrity Zome - Cross-hApp verification for mutual aid
//!
//! This zome defines the data structures and validation rules for cross-hApp
//! queries and MATL (Multi-Agent Trust Layer) integration within the Mycelix
//! mutual aid network.

use hdi::prelude::*;

/// Anchor entry type for string-based link bases
#[hdk_entry_helper]
#[derive(Clone, PartialEq, Eq)]
pub struct Anchor(pub String);

impl Anchor {
    pub fn new(value: impl Into<String>) -> Self {
        Anchor(value.into())
    }
}

/// Purpose of a cross-hApp query
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum QueryPurpose {
    /// Verify membership status for pool operations
    MemberVerification,
    /// Query contribution history for trust scoring
    ContributionHistory,
    /// Query request/fulfillment history
    RequestHistory,
    /// Query reputation score
    ReputationQuery,
    /// Query compliance status
    ComplianceCheck,
}

/// Query status tracking
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum QueryStatus {
    Pending,
    Processing,
    Completed,
    Failed,
    Expired,
}

/// A cross-hApp aid query
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct AidQuery {
    /// Unique identifier for this query
    pub id: String,
    /// DID of the subject being queried
    pub subject_did: String,
    /// DID of the requesting party
    pub requester_did: String,
    /// Purpose of the query
    pub purpose: QueryPurpose,
    /// Source hApp identifier
    pub source_happ: String,
    /// Target hApp identifier
    pub target_happ: String,
    /// Optional parameters for the query
    pub parameters: Option<String>,
    /// Query status
    pub status: QueryStatus,
    /// Timestamp when query was created
    pub created_at: Timestamp,
    /// Timestamp when query expires
    pub expires_at: Timestamp,
}

/// Result of a cross-hApp query
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct AidResult {
    /// Reference to the query this is responding to
    pub query_id: String,
    /// Whether the query was successful
    pub success: bool,
    /// Result data (JSON encoded)
    pub data: Option<String>,
    /// Error message if failed
    pub error: Option<String>,
    /// MATL trust score of the result provider
    pub provider_trust_score: f64,
    /// Timestamp when result was created
    pub created_at: Timestamp,
}

/// MATL reputation score for a member
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct MatlReputation {
    /// DID of the member
    pub member_did: String,
    /// Overall trust score (0.0 - 1.0)
    pub trust_score: f64,
    /// Contribution consistency score
    pub contribution_score: f64,
    /// Request fulfillment score (as helper)
    pub fulfillment_score: f64,
    /// Community engagement score
    pub engagement_score: f64,
    /// Number of pools participated in
    pub pools_count: u32,
    /// Total value contributed
    pub total_contributed: u64,
    /// Total value received
    pub total_received: u64,
    /// Number of successful aid offers
    pub successful_offers: u32,
    /// Number of requests made
    pub requests_made: u32,
    /// Last activity timestamp
    pub last_activity: Timestamp,
    /// Score calculation timestamp
    pub calculated_at: Timestamp,
}

/// Cross-hApp verification record
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct VerificationRecord {
    /// Unique identifier
    pub id: String,
    /// DID being verified
    pub subject_did: String,
    /// Type of verification
    pub verification_type: VerificationType,
    /// Verifying hApp
    pub verifier_happ: String,
    /// Verification result
    pub result: VerificationResult,
    /// Evidence supporting the verification
    pub evidence: Option<String>,
    /// Timestamp of verification
    pub verified_at: Timestamp,
    /// Validity period in seconds
    pub valid_for_seconds: u64,
}

/// Type of verification
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum VerificationType {
    Identity,
    PoolMembership,
    ContributionHistory,
    GoodStanding,
    SkillCredential,
    LocationProximity,
}

/// Verification result
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum VerificationResult {
    Verified,
    NotVerified,
    Inconclusive,
    Expired,
}

/// Bridge configuration for cross-hApp communication
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct BridgeConfig {
    /// Configuration ID
    pub id: String,
    /// List of trusted hApps (by DNA hash or identifier)
    pub trusted_happs: Vec<String>,
    /// Minimum trust score to accept queries
    pub min_trust_score: f64,
    /// Query timeout in seconds
    pub query_timeout_seconds: u64,
    /// Maximum concurrent queries
    pub max_concurrent_queries: u32,
    /// Whether to allow anonymous queries
    pub allow_anonymous: bool,
    /// Last updated timestamp
    pub updated_at: Timestamp,
}

/// All entry types for this zome
#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    #[entry_type(visibility = "public")]
    Anchor(Anchor),
    #[entry_type(visibility = "public")]
    AidQuery(AidQuery),
    #[entry_type(visibility = "public")]
    AidResult(AidResult),
    #[entry_type(visibility = "public")]
    MatlReputation(MatlReputation),
    #[entry_type(visibility = "public")]
    VerificationRecord(VerificationRecord),
    #[entry_type(visibility = "private")]
    BridgeConfig(BridgeConfig),
}

/// Link types for connecting entries
#[hdk_link_types]
pub enum LinkTypes {
    /// Anchor to all queries
    AnchorToQuery,
    /// Query to its result
    QueryToResult,
    /// Member DID to their reputation
    MemberToReputation,
    /// Member DID to verification records
    MemberToVerification,
    /// Anchor to pending queries
    AnchorToPendingQuery,
    /// Source hApp to queries
    SourceHappToQuery,
    /// Target hApp to queries
    TargetHappToQuery,
}

/// Validation errors for bridge zome
#[derive(Debug)]
pub enum BridgeError {
    InvalidDid(String),
    InvalidId(String),
    InvalidTrustScore,
    InvalidTimeout,
    QueryExpired,
    InvalidHappId,
}

impl std::fmt::Display for BridgeError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InvalidDid(s) => write!(f, "Invalid DID format: {}", s),
            Self::InvalidId(s) => write!(f, "Invalid ID format: {}", s),
            Self::InvalidTrustScore => {
                write!(f, "Invalid trust score: must be between 0.0 and 1.0")
            }
            Self::InvalidTimeout => write!(f, "Invalid timeout: must be positive"),
            Self::QueryExpired => write!(f, "Query expired"),
            Self::InvalidHappId => write!(f, "Invalid hApp identifier"),
        }
    }
}

/// Validate that a DID has a valid format
fn validate_did(did: &str) -> ExternResult<()> {
    if did.is_empty() {
        return Err(wasm_error!(WasmErrorInner::Guest(
            BridgeError::InvalidDid("DID cannot be empty".to_string()).to_string()
        )));
    }
    // Basic DID format check: did:method:identifier
    if !did.starts_with("did:") || did.split(':').count() < 3 {
        return Err(wasm_error!(WasmErrorInner::Guest(
            BridgeError::InvalidDid(format!("Invalid DID format: {}", did)).to_string()
        )));
    }
    Ok(())
}

/// Validate that an ID is non-empty
fn validate_id(id: &str, field_name: &str) -> ExternResult<()> {
    if id.is_empty() {
        return Err(wasm_error!(WasmErrorInner::Guest(
            BridgeError::InvalidId(format!("{} cannot be empty", field_name)).to_string()
        )));
    }
    Ok(())
}

/// Validate a trust score is in valid range
fn validate_trust_score(score: f64) -> ExternResult<()> {
    if score < 0.0 || score > 1.0 {
        return Err(wasm_error!(WasmErrorInner::Guest(
            BridgeError::InvalidTrustScore.to_string()
        )));
    }
    Ok(())
}

/// requester_did/subject_did are NOT bound to the committer -- see the
/// zome-wide disclosed-gap note on validate_update_entry_type above.
/// Reviewed 2026-07-09 during the P0 author-binding pass; case (d).
fn validate_aid_query(query: &AidQuery) -> ExternResult<ValidateCallbackResult> {
    // Validate IDs
    validate_id(&query.id, "Query ID")?;

    // Validate DIDs
    validate_did(&query.subject_did)?;
    validate_did(&query.requester_did)?;

    // Validate hApp identifiers
    if query.source_happ.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            BridgeError::InvalidHappId.to_string(),
        ));
    }
    if query.target_happ.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            BridgeError::InvalidHappId.to_string(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Validate an AidResult entry
fn validate_aid_result(result: &AidResult) -> ExternResult<ValidateCallbackResult> {
    // Validate query ID
    validate_id(&result.query_id, "Query ID")?;

    // Validate trust score
    validate_trust_score(result.provider_trust_score)?;

    Ok(ValidateCallbackResult::Valid)
}

/// Validate a MatlReputation entry. member_did is NOT bound to the
/// committer and the coordinator's update_reputation has ZERO caller
/// check at all -- see the zome-wide disclosed-gap note on
/// validate_update_entry_type above (the most severe instance of this
/// gap class found in the whole pass). Case (d).
fn validate_matl_reputation(reputation: &MatlReputation) -> ExternResult<ValidateCallbackResult> {
    // Validate DID
    validate_did(&reputation.member_did)?;

    // Validate all scores
    validate_trust_score(reputation.trust_score)?;
    validate_trust_score(reputation.contribution_score)?;
    validate_trust_score(reputation.fulfillment_score)?;
    validate_trust_score(reputation.engagement_score)?;

    Ok(ValidateCallbackResult::Valid)
}

/// Validate a VerificationRecord entry. subject_did/verifier_happ are
/// NOT bound to the committer -- see the zome-wide disclosed-gap note on
/// validate_update_entry_type above. Case (d).
fn validate_verification_record(
    record: &VerificationRecord,
) -> ExternResult<ValidateCallbackResult> {
    // Validate IDs
    validate_id(&record.id, "Verification ID")?;

    // Validate DID
    validate_did(&record.subject_did)?;

    // Validate hApp identifier
    if record.verifier_happ.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            BridgeError::InvalidHappId.to_string(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Validate a BridgeConfig entry
fn validate_bridge_config(config: &BridgeConfig) -> ExternResult<ValidateCallbackResult> {
    // Validate ID
    validate_id(&config.id, "Config ID")?;

    // Validate trust score threshold
    validate_trust_score(config.min_trust_score)?;

    // Validate timeout
    if config.query_timeout_seconds == 0 {
        return Ok(ValidateCallbackResult::Invalid(
            BridgeError::InvalidTimeout.to_string(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Genesis self-check callback
#[hdk_extern]
pub fn genesis_self_check(_data: GenesisSelfCheckData) -> ExternResult<ValidateCallbackResult> {
    Ok(ValidateCallbackResult::Valid)
}

/// Main validation callback using FlatOp pattern
#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(store_entry) => match store_entry {
            OpEntry::CreateEntry { app_entry, .. } => match app_entry {
                EntryTypes::Anchor(_) => Ok(ValidateCallbackResult::Valid),
                EntryTypes::AidQuery(query) => validate_aid_query(&query),
                EntryTypes::AidResult(result) => validate_aid_result(&result),
                EntryTypes::MatlReputation(reputation) => validate_matl_reputation(&reputation),
                EntryTypes::VerificationRecord(record) => validate_verification_record(&record),
                EntryTypes::BridgeConfig(config) => validate_bridge_config(&config),
            },
            OpEntry::UpdateEntry {
                app_entry,
                action,
                original_action_hash,
                ..
            } => validate_update_entry_type(action, original_action_hash, app_entry),
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterCreateLink { link_type, .. } => match link_type {
            LinkTypes::AnchorToQuery
            | LinkTypes::QueryToResult
            | LinkTypes::MemberToReputation
            | LinkTypes::MemberToVerification
            | LinkTypes::AnchorToPendingQuery
            | LinkTypes::SourceHappToQuery
            | LinkTypes::TargetHappToQuery => Ok(ValidateCallbackResult::Valid),
        },
        // Deliberately left fully permissive for ALL link types (reviewed
        // 2026-07-09 during the P0 author-binding pass, not a gap):
        // submit_result deletes an AnchorToPendingQuery link when a
        // cross-hApp responder marks a query complete/failed, and that
        // responder need not be the original querying agent (and thus
        // not the link's original creator).
        FlatOp::RegisterDeleteLink { link_type, .. } => match link_type {
            LinkTypes::AnchorToQuery
            | LinkTypes::QueryToResult
            | LinkTypes::MemberToReputation
            | LinkTypes::MemberToVerification
            | LinkTypes::AnchorToPendingQuery
            | LinkTypes::SourceHappToQuery
            | LinkTypes::TargetHappToQuery => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::StoreRecord(_) | FlatOp::RegisterAgentActivity(_) => {
            Ok(ValidateCallbackResult::Valid)
        }
        FlatOp::RegisterUpdate(op_update) => match op_update {
            // This DHT op was previously left fully permissive (`Ok(Valid)`
            // unconditionally) -- the 27th confirmed instance of this
            // exact bug pattern this pass. Found + fixed 2026-07-09
            // during the P0 author-binding pass. Route through the same
            // per-type validators as the StoreEntry perspective.
            OpUpdate::Entry {
                app_entry, action, ..
            } => validate_update_entry_type(
                action.clone(),
                action.original_action_address,
                app_entry,
            ),
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

/// **Zome-wide disclosed, NOT-fixed gap**: every identity-bearing field in
/// this zome (`AidQuery.requester_did`/`subject_did`,
/// `MatlReputation.member_did`, `VerificationRecord.subject_did`/
/// `verifier_happ`) is a free-form `String` DID/hApp-name asserted by the
/// caller, with NO local convention anywhere in this coordinator for
/// deriving or verifying a DID string from the committing agent's real
/// `action.author` (unlike zomes elsewhere in this pass that use a
/// `format!("did:mycelix:{}", author)` convention -- no such convention
/// exists in this hApp; DIDs here appear to originate from an external
/// identity system this bridge doesn't itself validate against).
/// Reviewed 2026-07-09 during the P0 author-binding pass; same class of
/// gap as mycelix-identity/bridge's `report_reputation` and
/// mycelix-knowledge/inference's `author_reputation` -- needs real
/// call-provenance/capability-grant infrastructure, not simple
/// author-binding, and inventing an unverified DID-format convention here
/// would be worse than leaving it honestly undone. **Most severe
/// instance of this gap class found in the whole pass so far**:
/// `update_reputation` has ZERO check of ANY kind (not even the DID
/// itself, no caller-derivation attempt) -- any agent can set ANY other
/// agent's MATL trust-score components (contribution/fulfillment/
/// engagement/pools_count/totals) to arbitrary values, and
/// `BridgeConfig.min_trust_score` elsewhere in this hApp gates real
/// query-acceptance decisions on this score. This is a live
/// reputation-forgery/self-mint vector, not merely theoretical. Flagged
/// prominently for dedicated follow-up.
fn validate_update_entry_type(
    action: Update,
    original_action_hash: ActionHash,
    app_entry: EntryTypes,
) -> ExternResult<ValidateCallbackResult> {
    match app_entry {
        EntryTypes::Anchor(_) => Ok(ValidateCallbackResult::Valid),
        EntryTypes::AidQuery(query) => {
            validate_update_aid_query(action, original_action_hash, query)
        }
        // No live update_entry call for AidResult (confirmed via grep) --
        // previously silently accepted any field change. Made explicitly
        // immutable.
        EntryTypes::AidResult(_) => Ok(ValidateCallbackResult::Invalid(
            "Aid results are immutable".into(),
        )),
        EntryTypes::MatlReputation(reputation) => {
            validate_update_reputation(action, original_action_hash, reputation)
        }
        // No live update_entry call for VerificationRecord either.
        EntryTypes::VerificationRecord(_) => Ok(ValidateCallbackResult::Invalid(
            "Verification records are immutable; create a new one".into(),
        )),
        // BridgeConfig has no live create OR update call at all
        // (confirmed via grep -- imported but never constructed by this
        // coordinator). Made explicitly immutable as defense-in-depth.
        EntryTypes::BridgeConfig(_) => Ok(ValidateCallbackResult::Invalid(
            "Bridge config is immutable".into(),
        )),
    }
}

/// No author requirement: submit_result has zero caller-identity check
/// (a cross-hApp responder may legitimately be a different agent than
/// the original requester) -- case (c). Content is restricted to status
/// only -- this closes the wide-open bug that previously let
/// subject_did/requester_did/purpose/source_happ/target_happ/parameters/
/// expires_at change unconditionally on update too.
fn validate_update_aid_query(
    action: Update,
    original_action_hash: ActionHash,
    query: AidQuery,
) -> ExternResult<ValidateCallbackResult> {
    let _ = &action;
    let original_record = must_get_valid_record(original_action_hash)?;
    let original: AidQuery = original_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Original query not found".into()
        )))?;

    if query.id != original.id
        || query.subject_did != original.subject_did
        || query.requester_did != original.requester_did
        || query.purpose != original.purpose
        || query.source_happ != original.source_happ
        || query.target_happ != original.target_happ
        || query.parameters != original.parameters
        || query.created_at != original.created_at
        || query.expires_at != original.expires_at
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Only status can change on a query update".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Content restricted to member_did staying the SAME (everything else
/// may legitimately change -- update_reputation recomputes nearly every
/// field). This does NOT fix the disclosed gap above (anyone can still
/// update anyone's reputation with arbitrary values), but it does close
/// a distinct, narrower wide-open-update issue: previously a modified
/// coordinator could also silently reassign a reputation record to a
/// DIFFERENT member_did entirely.
fn validate_update_reputation(
    action: Update,
    original_action_hash: ActionHash,
    reputation: MatlReputation,
) -> ExternResult<ValidateCallbackResult> {
    let _ = &action;
    let original_record = must_get_valid_record(original_action_hash)?;
    let original: MatlReputation = original_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Original reputation not found".into()
        )))?;

    if reputation.member_did != original.member_did {
        return Ok(ValidateCallbackResult::Invalid(
            "member_did cannot change on a reputation update".into(),
        ));
    }

    validate_matl_reputation(&reputation)
}

#[cfg(test)]
mod content_integrity_tests {
    use super::*;

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

    #[test]
    fn update_entry_type_rejects_aid_result_update() {
        // No live update_entry call exists for AidResult -- dead-path
        // immutability, testable without must_get_valid_record.
        let result = AidResult {
            query_id: "q-1".into(),
            success: true,
            data: None,
            error: None,
            provider_trust_score: 0.9,
            created_at: Timestamp::from_micros(0),
        };
        let action = update_action(me());
        let result = validate_update_entry_type(
            action.clone(),
            action.original_action_address,
            EntryTypes::AidResult(result),
        )
        .unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn update_entry_type_rejects_bridge_config_update() {
        let config = BridgeConfig {
            id: "cfg-1".into(),
            trusted_happs: vec![],
            min_trust_score: 0.5,
            query_timeout_seconds: 30,
            max_concurrent_queries: 10,
            allow_anonymous: false,
            updated_at: Timestamp::from_micros(0),
        };
        let action = update_action(me());
        let result = validate_update_entry_type(
            action.clone(),
            action.original_action_address,
            EntryTypes::BridgeConfig(config),
        )
        .unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }
}
