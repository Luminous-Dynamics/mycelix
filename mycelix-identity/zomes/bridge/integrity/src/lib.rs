// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Identity Bridge Integrity Zome
//!
//! Entry types and validation for cross-hApp identity operations.
//! Enables other hApps to query and verify identities via the Bridge protocol.
//!
//! Updated to use HDI 0.7 patterns with FlatOp validation

use hdi::prelude::*;

/// hApp registration with the identity bridge
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct HappRegistration {
    /// Unique identifier for the hApp
    pub happ_id: String,
    /// Human-readable name
    pub happ_name: String,
    /// Capabilities this hApp supports
    pub capabilities: Vec<String>,
    /// MATL trust score for this hApp (0.0 - 1.0)
    pub matl_score: f64,
    /// When this hApp was registered
    pub registered_at: Timestamp,
}

/// Identity query request from another hApp
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct IdentityQuery {
    /// Query ID for tracking
    pub id: String,
    /// DID being queried
    pub did: String,
    /// Requesting hApp
    pub source_happ: String,
    /// What fields are requested (selective disclosure)
    pub requested_fields: Vec<String>,
    /// Query timestamp
    pub queried_at: Timestamp,
}

/// Identity verification result
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct IdentityVerification {
    /// Verification ID
    pub id: String,
    /// DID that was verified
    pub did: String,
    /// Whether the DID exists and is active
    pub is_valid: bool,
    /// Whether the DID is deactivated
    pub is_deactivated: bool,
    /// MATL reputation score (0.0 - 1.0)
    pub matl_score: f64,
    /// Total credentials issued to this DID
    pub credential_count: u32,
    /// When the DID was created
    pub did_created: Option<Timestamp>,
    /// Verification timestamp
    pub verified_at: Timestamp,
}

/// Bridge event for pub/sub
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct BridgeEvent {
    /// Event ID
    pub id: String,
    /// Event type (e.g., "did_created", "did_updated", "did_deactivated")
    pub event_type: BridgeEventType,
    /// Agent/DID this event relates to
    pub subject: String,
    /// Event payload (JSON-encoded)
    pub payload: String,
    /// Source hApp
    pub source_happ: String,
    /// Event timestamp
    pub timestamp: Timestamp,
}

/// Types of bridge events
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum BridgeEventType {
    /// A new DID was created
    DidCreated,
    /// A DID was updated (keys rotated, services changed)
    DidUpdated,
    /// A DID was deactivated
    DidDeactivated,
    /// A credential was issued to a DID
    CredentialIssued,
    /// A credential was revoked
    CredentialRevoked,
    /// Recovery was initiated for a DID
    RecoveryInitiated,
    /// Recovery was completed for a DID
    RecoveryCompleted,
    /// hApp registered with bridge
    HappRegistered,
    /// MFA assurance level changed for a DID
    MfaAssuranceChanged,
    /// A DID was recovered via social recovery
    DidRecovered,
    /// Custom event type
    Custom(String),
}

/// Anchor entry for deterministic link bases
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Anchor(pub String);

/// Cross-hApp reputation record
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct IdentityReputation {
    /// DID this reputation is for
    pub did: String,
    /// hApp this reputation came from
    pub source_happ: String,
    /// Reputation score (0.0 - 1.0)
    pub score: f64,
    /// Number of interactions in the source hApp
    pub interactions: u64,
    /// Last update timestamp
    pub last_updated: Timestamp,
}

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    Anchor(Anchor),
    HappRegistration(HappRegistration),
    IdentityQuery(IdentityQuery),
    IdentityVerification(IdentityVerification),
    BridgeEvent(BridgeEvent),
    IdentityReputation(IdentityReputation),
}

#[hdk_link_types]
pub enum LinkTypes {
    /// Anchor to all registered hApps
    RegisteredHapps,
    /// DID to its reputation scores from various hApps
    DidToReputations,
    /// Anchor to recent bridge events
    RecentEvents,
    /// Event type to events (for filtering)
    EventTypeToEvents,
    /// hApp to its queries
    HappToQueries,
    /// DID to queries about it
    DidToQueries,
    /// Rate limit tracking: agent → anchor per dispatch call
    DispatchRateLimit,
}

/// Genesis self-check - called when app is installed
#[hdk_extern]
pub fn genesis_self_check(_data: GenesisSelfCheckData) -> ExternResult<ValidateCallbackResult> {
    Ok(ValidateCallbackResult::Valid)
}

/// Main validation callback using FlatOp pattern matching
#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(store_entry) => match store_entry {
            OpEntry::CreateEntry { app_entry, action } => match app_entry {
                EntryTypes::Anchor(_) => Ok(ValidateCallbackResult::Valid),
                EntryTypes::HappRegistration(registration) => validate_create_happ_registration(
                    EntryCreationAction::Create(action),
                    registration,
                ),
                EntryTypes::IdentityQuery(query) => {
                    validate_create_identity_query(EntryCreationAction::Create(action), query)
                }
                EntryTypes::IdentityVerification(verification) => {
                    validate_create_identity_verification(
                        EntryCreationAction::Create(action),
                        verification,
                    )
                }
                EntryTypes::BridgeEvent(event) => {
                    validate_create_bridge_event(EntryCreationAction::Create(action), event)
                }
                EntryTypes::IdentityReputation(reputation) => validate_create_identity_reputation(
                    EntryCreationAction::Create(action),
                    reputation,
                ),
            },
            OpEntry::UpdateEntry {
                app_entry, action, ..
            } => match app_entry {
                EntryTypes::Anchor(_) => Ok(ValidateCallbackResult::Invalid(
                    "Anchors cannot be updated".into(),
                )),
                EntryTypes::HappRegistration(registration) => {
                    validate_update_happ_registration(action, registration)
                }
                EntryTypes::IdentityReputation(reputation) => {
                    validate_update_identity_reputation(action, reputation)
                }
                // Queries, verifications, and events are immutable
                EntryTypes::IdentityQuery(_) => Ok(ValidateCallbackResult::Invalid(
                    "Identity queries cannot be updated".into(),
                )),
                EntryTypes::IdentityVerification(_) => Ok(ValidateCallbackResult::Invalid(
                    "Identity verifications cannot be updated".into(),
                )),
                EntryTypes::BridgeEvent(_) => Ok(ValidateCallbackResult::Invalid(
                    "Bridge events cannot be updated".into(),
                )),
            },
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterCreateLink { link_type, tag, .. } => {
            if tag.0.len() > 1024 {
                return Ok(ValidateCallbackResult::Invalid(
                    "Link tag exceeds maximum length of 1024 bytes".into(),
                ));
            }
            match link_type {
                LinkTypes::RegisteredHapps => Ok(ValidateCallbackResult::Valid),
                LinkTypes::DidToReputations => Ok(ValidateCallbackResult::Valid),
                LinkTypes::RecentEvents => Ok(ValidateCallbackResult::Valid),
                LinkTypes::EventTypeToEvents => Ok(ValidateCallbackResult::Valid),
                LinkTypes::HappToQueries => Ok(ValidateCallbackResult::Valid),
                LinkTypes::DidToQueries => Ok(ValidateCallbackResult::Valid),
                LinkTypes::DispatchRateLimit => Ok(ValidateCallbackResult::Valid),
            }
        }
        FlatOp::RegisterDeleteLink {
            original_action,
            action,
            ..
        } => {
            if action.author != original_action.author {
                return Ok(ValidateCallbackResult::Invalid(
                    "Only the link creator can delete their links".into(),
                ));
            }
            Ok(ValidateCallbackResult::Valid)
        }
        FlatOp::StoreRecord(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterAgentActivity(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterUpdate(update) => {
            let action = match &update {
                OpUpdate::Entry { action, .. }
                | OpUpdate::PrivateEntry { action, .. }
                | OpUpdate::Agent { action, .. }
                | OpUpdate::CapClaim { action, .. }
                | OpUpdate::CapGrant { action, .. } => action,
            };
            let original = must_get_action(action.original_action_address.clone())?;
            if *original.action().author() != action.author {
                return Ok(ValidateCallbackResult::Invalid(
                    "Only the original entry author can update their entries".into(),
                ));
            }
            Ok(ValidateCallbackResult::Valid)
        }
        FlatOp::RegisterDelete(OpDelete { action }) => {
            let original = must_get_action(action.deletes_address.clone())?;
            if *original.action().author() != action.author {
                return Ok(ValidateCallbackResult::Invalid(
                    "Only the original entry author can delete their entries".into(),
                ));
            }
            Ok(ValidateCallbackResult::Valid)
        }
    }
}

/// Validate hApp registration creation
///
/// No author-binding possible: HappRegistration has no self-declared
/// owner/creator AgentPubKey or DID field -- it's an announcement that any
/// agent can publish about an hApp cluster. Reviewed 2026-07-08 during the
/// P0 author-binding pass; case (a), nothing to bind against.
fn validate_create_happ_registration(
    _action: EntryCreationAction,
    registration: HappRegistration,
) -> ExternResult<ValidateCallbackResult> {
    // hApp ID must be non-empty
    if registration.happ_id.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "hApp ID cannot be empty".into(),
        ));
    }

    // hApp name must be non-empty
    if registration.happ_name.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "hApp name cannot be empty".into(),
        ));
    }

    // MATL score must be valid
    if !(0.0..=1.0).contains(&registration.matl_score) {
        return Ok(ValidateCallbackResult::Invalid(
            "MATL score must be between 0.0 and 1.0".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Validate hApp registration update
///
/// Author-binding for updates is already enforced universally by this
/// crate's `FlatOp::RegisterUpdate` arm in `validate()` (checks
/// `original.action().author() == action.author` for every entry type), so
/// this function only needs to check content invariants, not identity.
fn validate_update_happ_registration(
    action: Update,
    registration: HappRegistration,
) -> ExternResult<ValidateCallbackResult> {
    if !(0.0..=1.0).contains(&registration.matl_score) {
        return Ok(ValidateCallbackResult::Invalid(
            "MATL score must be between 0.0 and 1.0".into(),
        ));
    }

    // Fetch original to enforce invariants
    let original_record = must_get_valid_record(action.original_action_address.clone())?;
    let original: HappRegistration = original_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Original hApp registration not found".into()
        )))?;

    // Immutable fields
    if registration.happ_id != original.happ_id {
        return Ok(ValidateCallbackResult::Invalid(
            "hApp ID cannot be changed".into(),
        ));
    }
    if registration.registered_at != original.registered_at {
        return Ok(ValidateCallbackResult::Invalid(
            "Registration timestamp cannot be changed".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Validate identity query creation
///
/// No author-binding possible: `did` names the DID being LOOKED UP (a third
/// party), not the committer -- this is an audit-trail record of a lookup,
/// not a self-declared identity claim. `source_happ` is a free-text label,
/// not an agent-comparable identity, and can't be bound to action.author()
/// either (see the report_reputation forgery note on
/// validate_create_identity_reputation below for why that class of gap is
/// architecturally different from simple author-binding). Reviewed
/// 2026-07-08 during the P0 author-binding pass; case (b).
fn validate_create_identity_query(
    _action: EntryCreationAction,
    query: IdentityQuery,
) -> ExternResult<ValidateCallbackResult> {
    // DID must be valid format
    if !query.did.starts_with("did:mycelix:") {
        return Ok(ValidateCallbackResult::Invalid(
            "DID must use did:mycelix format".into(),
        ));
    }

    // Source hApp must be specified
    if query.source_happ.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Source hApp must be specified".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Validate identity verification creation
///
/// No author-binding possible: `did` names the DID being verified (a third
/// party), not the committer. This is a computed/cached result — the
/// coordinator's `query_identity` derives `is_valid`/`matl_score`/
/// `credential_count` from cross-zome lookups (did_registry, mfa,
/// verifiable_credential) at call time. A malicious coordinator running on
/// an attacker's own conductor could forge this entry's *contents* wholesale
/// (e.g. claim `is_valid: true, matl_score: 1.0` for any DID) without
/// running the real computation at all -- but that's a content-integrity
/// problem the HDI validate() callback can't practically re-derive (it would
/// require re-executing cross-zome calls during validation, which is
/// unreliable across validating authorities). Same architectural class as
/// the report_reputation forgery gap on validate_create_identity_reputation
/// below. Reviewed 2026-07-08 during the P0 author-binding pass; flagged as
/// a known limitation, not fixed here.
fn validate_create_identity_verification(
    _action: EntryCreationAction,
    verification: IdentityVerification,
) -> ExternResult<ValidateCallbackResult> {
    // DID must be valid format
    if !verification.did.starts_with("did:mycelix:") {
        return Ok(ValidateCallbackResult::Invalid(
            "DID must use did:mycelix format".into(),
        ));
    }

    // MATL score must be valid
    if !(0.0..=1.0).contains(&verification.matl_score) {
        return Ok(ValidateCallbackResult::Invalid(
            "MATL score must be between 0.0 and 1.0".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Validate bridge event creation
///
/// No author-binding possible: `subject` names the DID/agent the event is
/// ABOUT (e.g. "this DID was created/deactivated"), not necessarily the
/// committer -- these are notifications published by whichever local zome
/// witnessed the underlying action, not self-attestations. Reviewed
/// 2026-07-08 during the P0 author-binding pass; case (b).
fn validate_create_bridge_event(
    _action: EntryCreationAction,
    event: BridgeEvent,
) -> ExternResult<ValidateCallbackResult> {
    // Subject must be non-empty
    if event.subject.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Event subject cannot be empty".into(),
        ));
    }

    // Source hApp must be specified
    if event.source_happ.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Source hApp must be specified".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Validate identity reputation creation
///
/// KNOWN GAP, not fixable by simple author-binding (found 2026-07-08 during
/// the P0 pass): the coordinator's `report_reputation` takes `did`,
/// `source_happ`, `score`, and `interactions` entirely from caller input
/// with zero derivation from `agent_info()` -- any agent can call it
/// claiming to BE any `source_happ` (e.g. "mycelix-finance") reporting an
/// arbitrary score/interaction count for any victim DID, and
/// `get_aggregated_reputation`/`get_moral_resonance_score` fold these
/// straight into a DID's computed MATL/resonance score. Unlike the
/// self-declared-identity forgeries fixed elsewhere this pass (trust
/// attestations, MFA state, DIDs), `source_happ` is a free-text hApp-name
/// label, not an agent-comparable AgentPubKey/DID -- and because
/// CallTargetCell::OtherRole cross-DNA calls within one hApp bundle all
/// share the SAME agent key, action.author() can't distinguish "a call
/// legitimately routed through mycelix-finance's own bridge zome" from "an
/// arbitrary agent calling report_reputation directly." A real fix needs
/// call-provenance / capability-grant infrastructure, not a field binding --
/// same class of gap as mycelix-justice's Arbitration/Judgment needing
/// must_get juror-authorization (see MASTER_ROADMAP.md P0 #1). Out of scope
/// for this pass; flagging for a dedicated follow-up.
fn validate_create_identity_reputation(
    _action: EntryCreationAction,
    reputation: IdentityReputation,
) -> ExternResult<ValidateCallbackResult> {
    // DID must be valid format
    if !reputation.did.starts_with("did:mycelix:") {
        return Ok(ValidateCallbackResult::Invalid(
            "DID must use did:mycelix format".into(),
        ));
    }

    // Source hApp must be specified
    if reputation.source_happ.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Source hApp must be specified".into(),
        ));
    }

    // Score must be valid
    if !(0.0..=1.0).contains(&reputation.score) {
        return Ok(ValidateCallbackResult::Invalid(
            "Reputation score must be between 0.0 and 1.0".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Validate identity reputation update
///
/// Author-binding for updates is already enforced universally by this
/// crate's `FlatOp::RegisterUpdate` arm in `validate()` (checks
/// `original.action().author() == action.author` for every entry type).
/// Note this does NOT close the create-path forgery gap documented on
/// validate_create_identity_reputation above -- it only stops a DIFFERENT
/// agent from hijacking an update to someone else's already-forged entry.
fn validate_update_identity_reputation(
    action: Update,
    reputation: IdentityReputation,
) -> ExternResult<ValidateCallbackResult> {
    if !(0.0..=1.0).contains(&reputation.score) {
        return Ok(ValidateCallbackResult::Invalid(
            "Reputation score must be between 0.0 and 1.0".into(),
        ));
    }

    // Fetch original to enforce invariants
    let original_record = must_get_valid_record(action.original_action_address.clone())?;
    let original: IdentityReputation = original_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Original reputation entry not found".into()
        )))?;

    // Immutable fields
    if reputation.did != original.did {
        return Ok(ValidateCallbackResult::Invalid(
            "Reputation DID cannot be changed".into(),
        ));
    }
    if reputation.source_happ != original.source_happ {
        return Ok(ValidateCallbackResult::Invalid(
            "Reputation source hApp cannot be changed".into(),
        ));
    }

    // Interactions must be monotonically non-decreasing
    if reputation.interactions < original.interactions {
        return Ok(ValidateCallbackResult::Invalid(
            "Interaction count cannot decrease".into(),
        ));
    }

    // Timestamp must advance
    if reputation.last_updated <= original.last_updated {
        return Ok(ValidateCallbackResult::Invalid(
            "Reputation last_updated must advance".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ts(micros: i64) -> Timestamp {
        Timestamp::from_micros(micros)
    }

    // --- HappRegistration ---

    #[test]
    fn happ_registration_json_round_trip() {
        let reg = HappRegistration {
            happ_id: "mycelix-core".into(),
            happ_name: "Mycelix Core".into(),
            capabilities: vec!["identity".into(), "governance".into()],
            matl_score: 0.85,
            registered_at: ts(1_700_000_000_000_000),
        };
        let json = serde_json::to_string(&reg).unwrap();
        let back: HappRegistration = serde_json::from_str(&json).unwrap();
        assert_eq!(reg, back);
    }

    #[test]
    fn happ_registration_rejects_empty_id() {
        let reg = HappRegistration {
            happ_id: "".into(),
            happ_name: "Test".into(),
            capabilities: vec![],
            matl_score: 0.5,
            registered_at: ts(0),
        };
        assert!(
            reg.happ_id.is_empty(),
            "Empty ID should be rejected by validator"
        );
    }

    #[test]
    fn happ_registration_rejects_invalid_matl() {
        for score in [1.1, -0.1, f64::NAN, f64::INFINITY] {
            assert!(
                !(0.0..=1.0).contains(&score),
                "Score {} should be rejected",
                score
            );
        }
        for score in [0.0, 0.5, 1.0] {
            assert!(
                (0.0..=1.0).contains(&score),
                "Score {} should be accepted",
                score
            );
        }
    }

    // --- IdentityQuery ---

    #[test]
    fn identity_query_json_round_trip() {
        let query = IdentityQuery {
            id: "q-001".into(),
            did: "did:mycelix:abc123".into(),
            source_happ: "mycelix-finance".into(),
            requested_fields: vec!["name".into(), "email".into()],
            queried_at: ts(1_700_000_000_000_000),
        };
        let json = serde_json::to_string(&query).unwrap();
        let back: IdentityQuery = serde_json::from_str(&json).unwrap();
        assert_eq!(query, back);
    }

    #[test]
    fn identity_query_rejects_wrong_did_prefix() {
        assert!(!"did:web:example.com".starts_with("did:mycelix:"));
        assert!("did:mycelix:abc123".starts_with("did:mycelix:"));
    }

    // --- BridgeEventType ---

    #[test]
    fn bridge_event_type_json_variants() {
        let variants = vec![
            (BridgeEventType::DidCreated, "\"DidCreated\""),
            (BridgeEventType::DidUpdated, "\"DidUpdated\""),
            (BridgeEventType::DidDeactivated, "\"DidDeactivated\""),
            (BridgeEventType::CredentialIssued, "\"CredentialIssued\""),
            (BridgeEventType::CredentialRevoked, "\"CredentialRevoked\""),
            (BridgeEventType::RecoveryInitiated, "\"RecoveryInitiated\""),
            (BridgeEventType::RecoveryCompleted, "\"RecoveryCompleted\""),
            (BridgeEventType::HappRegistered, "\"HappRegistered\""),
            (
                BridgeEventType::MfaAssuranceChanged,
                "\"MfaAssuranceChanged\"",
            ),
            (BridgeEventType::DidRecovered, "\"DidRecovered\""),
        ];
        for (variant, expected) in variants {
            let json = serde_json::to_string(&variant).unwrap();
            assert_eq!(json, expected);
            let back: BridgeEventType = serde_json::from_str(&json).unwrap();
            assert_eq!(back, variant);
        }
    }

    #[test]
    fn bridge_event_type_custom_round_trip() {
        let custom = BridgeEventType::Custom("CustomEvent".into());
        let json = serde_json::to_string(&custom).unwrap();
        assert!(json.contains("CustomEvent"));
        let back: BridgeEventType = serde_json::from_str(&json).unwrap();
        assert_eq!(back, custom);
    }

    // --- BridgeEvent ---

    #[test]
    fn bridge_event_json_round_trip() {
        let event = BridgeEvent {
            id: "evt-001".into(),
            event_type: BridgeEventType::CredentialIssued,
            subject: "did:mycelix:holder1".into(),
            payload: r#"{"credential_id":"c-123"}"#.into(),
            source_happ: "mycelix-identity".into(),
            timestamp: ts(1_700_000_000_000_000),
        };
        let json = serde_json::to_string(&event).unwrap();
        let back: BridgeEvent = serde_json::from_str(&json).unwrap();
        assert_eq!(event, back);
    }

    #[test]
    fn bridge_event_rejects_empty_subject() {
        assert!(
            "".is_empty(),
            "Empty subject should be rejected by validator"
        );
        assert!(!"did:mycelix:abc".is_empty());
    }

    // --- IdentityVerification ---

    #[test]
    fn identity_verification_json_round_trip() {
        let v = IdentityVerification {
            id: "v-001".into(),
            did: "did:mycelix:abc".into(),
            is_valid: true,
            is_deactivated: false,
            matl_score: 0.92,
            credential_count: 5,
            did_created: Some(ts(1_600_000_000_000_000)),
            verified_at: ts(1_700_000_000_000_000),
        };
        let json = serde_json::to_string(&v).unwrap();
        let back: IdentityVerification = serde_json::from_str(&json).unwrap();
        assert_eq!(v, back);
    }

    // --- IdentityReputation ---

    #[test]
    fn identity_reputation_json_round_trip() {
        let rep = IdentityReputation {
            did: "did:mycelix:abc".into(),
            source_happ: "mycelix-finance".into(),
            score: 0.75,
            interactions: 42,
            last_updated: ts(1_700_000_000_000_000),
        };
        let json = serde_json::to_string(&rep).unwrap();
        let back: IdentityReputation = serde_json::from_str(&json).unwrap();
        assert_eq!(rep, back);
    }

    #[test]
    fn identity_reputation_rejects_invalid_score() {
        for score in [1.5, -0.01] {
            assert!(!(0.0..=1.0).contains(&score));
        }
    }
}
