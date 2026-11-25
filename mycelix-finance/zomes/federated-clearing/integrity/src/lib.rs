#![deny(unsafe_code)]
// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Federated clearing integrity contracts.
//!
//! PostgreSQL is the shared-coordinator transaction domain for online admission.
//! This zome stores immutable signed evidence and validates the designated home
//! agent's reservation/obligation chain. It does NOT create a cross-agent global
//! unique index, prove external PostgreSQL authority, or establish settlement finality.

use hdi::prelude::*;

const MAX_ID_LEN: usize = 256;
const MAX_DID_LEN: usize = 256;
const MAX_INSTRUMENT_ID_LEN: usize = 128;
const MAX_SHARD_ID_LEN: usize = 128;
const MAX_VERIFIER_VERSION_LEN: usize = 128;
const MAX_UNIT_DIGITS: usize = 78;

/// Gross direction across one bilateral agreement.
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq)]
pub enum ClearingDirection {
    AToB,
    BToA,
}

/// A signed, immutable reservation manifest. The PostgreSQL coordinator must
/// allocate non-overlapping capacity before the shard is permitted to spend it.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct FederatedReservationManifest {
    pub schema_version: u16,
    pub manifest_id: String,
    pub issuer_community_did: String,
    pub community_did: String,
    pub instrument_id: String,
    pub profile_revision: u32,
    pub agreement_id: String,
    pub direction: ClearingDirection,
    pub shard_id: String,
    pub epoch: u64,
    /// Canonical unsigned decimal integer; never float or exponent notation.
    pub reserved_units: String,
    /// Designated author for this shard/epoch's offline claims.
    pub home_agent: AgentPubKey,
    /// Digest of the governance/authority evidence checked by the coordinator.
    /// This field is evidence binding, not a substitute for independent governance
    /// authorization validation; that integration remains a qualification gate.
    pub authority_evidence_digest_hex: String,
    pub created_at: Timestamp,
}

/// Immutable claim emitted from the PostgreSQL transactional outbox. The entry is
/// authored by the reservation's home agent; aliases must resolve to the canonical
/// claim instead of creating a second economic obligation.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct FederatedObligationClaim {
    pub schema_version: u16,
    pub event_id: String,
    pub request_id: String,
    pub principal_did: String,
    pub community_did: String,
    pub instrument_id: String,
    pub profile_revision: u32,
    pub agreement_id: String,
    pub direction: ClearingDirection,
    pub reservation_manifest_action_hash: ActionHash,
    pub shard_id: String,
    pub epoch: u64,
    pub home_agent: AgentPubKey,
    /// Canonical decimal integer in exact instrument units.
    pub amount_units: String,
    pub canonical_work_digest_hex: String,
    pub payload_digest_hex: String,
    pub verifier_version: String,
    pub verifier_receipt_digest_hex: String,
    /// Digest of the PostgreSQL canonical receipt committed with the obligation.
    pub postgres_receipt_digest_hex: String,
    pub created_at: Timestamp,
}

/// Immutable settlement evidence. Settlement authority, payment finality, and
/// PostgreSQL counter updates are separately verified by the coordinator/service.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct FederatedSettlementEvidence {
    pub schema_version: u16,
    pub settlement_id: String,
    pub event_id: String,
    pub obligation_claim_action_hash: ActionHash,
    pub community_did: String,
    pub instrument_id: String,
    pub direction: ClearingDirection,
    pub amount_units: String,
    pub settlement_digest_hex: String,
    pub authority_evidence_digest_hex: String,
    pub home_agent: AgentPubKey,
    pub created_at: Timestamp,
}

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Anchor(pub String);

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    FederatedReservationManifest(FederatedReservationManifest),
    FederatedObligationClaim(FederatedObligationClaim),
    FederatedSettlementEvidence(FederatedSettlementEvidence),
    Anchor(Anchor),
}

#[hdk_link_types]
pub enum LinkTypes {
    ReservationManifestById,
    ObligationClaimByEventId,
    ObligationClaimByCanonicalWork,
    SettlementEvidenceById,
}

fn invalid(reason: impl Into<String>) -> ExternResult<ValidateCallbackResult> {
    Ok(ValidateCallbackResult::Invalid(reason.into()))
}

fn valid() -> ExternResult<ValidateCallbackResult> {
    Ok(ValidateCallbackResult::Valid)
}

fn bounded_id(value: &str, max: usize) -> bool {
    !value.trim().is_empty() && value.len() <= max && value.trim() == value
}

fn valid_did(value: &str) -> bool {
    bounded_id(value, MAX_DID_LEN) && value.starts_with("did:") && value.len() > 4
}

fn valid_sha256_hex(value: &str) -> bool {
    value.len() == 64
        && value
            .bytes()
            .all(|b| b.is_ascii_digit() || (b'a'..=b'f').contains(&b))
}

/// SQL NUMERIC(78,0) units are carried as canonical decimal strings. This accepts
/// up to 78 digits without parsing into a machine integer or losing precision.
fn valid_positive_units(value: &str) -> bool {
    !value.is_empty()
        && value.len() <= MAX_UNIT_DIGITS
        && value.as_bytes()[0] != b'0'
        && value.bytes().all(|b| b.is_ascii_digit())
        && value.bytes().any(|b| b != b'0')
}

fn validate_manifest_shape(m: &FederatedReservationManifest) -> Result<(), String> {
    if m.schema_version != 1 { return Err("unsupported reservation manifest version".into()); }
    if !bounded_id(&m.manifest_id, MAX_ID_LEN) { return Err("invalid manifest ID".into()); }
    if !valid_did(&m.issuer_community_did) || !valid_did(&m.community_did) {
        return Err("manifest community identities must be valid DIDs".into());
    }
    if !bounded_id(&m.instrument_id, MAX_INSTRUMENT_ID_LEN) { return Err("invalid instrument ID".into()); }
    if m.profile_revision == 0 { return Err("profile revision must be positive".into()); }
    if !bounded_id(&m.agreement_id, MAX_ID_LEN) { return Err("invalid agreement ID".into()); }
    if !bounded_id(&m.shard_id, MAX_SHARD_ID_LEN) { return Err("invalid shard ID".into()); }
    if !valid_positive_units(&m.reserved_units) { return Err("reserved units must be a positive canonical decimal integer".into()); }
    if !valid_sha256_hex(&m.authority_evidence_digest_hex) { return Err("invalid authority evidence digest".into()); }
    Ok(())
}

fn validate_claim_shape(c: &FederatedObligationClaim) -> Result<(), String> {
    if c.schema_version != 1 { return Err("unsupported obligation claim version".into()); }
    if !bounded_id(&c.event_id, MAX_ID_LEN) || !bounded_id(&c.request_id, MAX_ID_LEN) {
        return Err("invalid event/request ID".into());
    }
    if !valid_did(&c.principal_did) || !valid_did(&c.community_did) {
        return Err("principal/community must be valid DIDs".into());
    }
    if !bounded_id(&c.instrument_id, MAX_INSTRUMENT_ID_LEN) { return Err("invalid instrument ID".into()); }
    if c.profile_revision == 0 { return Err("profile revision must be positive".into()); }
    if !bounded_id(&c.agreement_id, MAX_ID_LEN) || !bounded_id(&c.shard_id, MAX_SHARD_ID_LEN) {
        return Err("invalid agreement or shard ID".into());
    }
    if !valid_positive_units(&c.amount_units) { return Err("amount units must be a positive canonical decimal integer".into()); }
    if !valid_sha256_hex(&c.canonical_work_digest_hex)
        || !valid_sha256_hex(&c.payload_digest_hex)
        || !valid_sha256_hex(&c.verifier_receipt_digest_hex)
        || !valid_sha256_hex(&c.postgres_receipt_digest_hex) {
        return Err("obligation claim contains an invalid SHA-256 digest".into());
    }
    if !bounded_id(&c.verifier_version, MAX_VERIFIER_VERSION_LEN) {
        return Err("invalid verifier version".into());
    }
    Ok(())
}

fn claim_matches_manifest(c: &FederatedObligationClaim, m: &FederatedReservationManifest) -> Result<(), String> {
    if c.home_agent != m.home_agent { return Err("claim home agent differs from reservation manifest".into()); }
    if c.community_did != m.community_did
        || c.instrument_id != m.instrument_id
        || c.profile_revision != m.profile_revision
        || c.agreement_id != m.agreement_id
        || c.direction != m.direction
        || c.shard_id != m.shard_id
        || c.epoch != m.epoch {
        return Err("obligation claim does not match its reservation manifest scope".into());
    }
    Ok(())
}

fn validate_settlement_shape(s: &FederatedSettlementEvidence) -> Result<(), String> {
    if s.schema_version != 1 { return Err("unsupported settlement evidence version".into()); }
    if !bounded_id(&s.settlement_id, MAX_ID_LEN) || !bounded_id(&s.event_id, MAX_ID_LEN) {
        return Err("invalid settlement/event ID".into());
    }
    if !valid_did(&s.community_did) { return Err("invalid settlement community DID".into()); }
    if !bounded_id(&s.instrument_id, MAX_INSTRUMENT_ID_LEN) { return Err("invalid instrument ID".into()); }
    if !valid_positive_units(&s.amount_units) { return Err("settlement units must be a positive canonical decimal integer".into()); }
    if !valid_sha256_hex(&s.settlement_digest_hex) || !valid_sha256_hex(&s.authority_evidence_digest_hex) {
        return Err("invalid settlement digest".into());
    }
    Ok(())
}

fn claim_matches_settlement(c: &FederatedObligationClaim, s: &FederatedSettlementEvidence) -> Result<(), String> {
    if c.community_did != s.community_did
        || c.instrument_id != s.instrument_id
        || c.direction != s.direction
        || c.amount_units != s.amount_units {
        return Err("settlement evidence differs from the canonical obligation".into());
    }
    if c.home_agent != s.home_agent {
        return Err("settlement author must be the obligation's designated home agent for profile v1".into());
    }
    Ok(())
}

#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(store) => match store {
            OpEntry::CreateEntry { app_entry, action } => match app_entry {
                EntryTypes::FederatedReservationManifest(m) => {
                    if let Err(e) = validate_manifest_shape(&m) { return invalid(e); }
                    if action.author() != &m.home_agent {
                        return invalid("reservation manifest author must equal its designated home agent");
                    }
                    valid()
                }
                EntryTypes::FederatedObligationClaim(c) => {
                    if let Err(e) = validate_claim_shape(&c) { return invalid(e); }
                    let record = must_get_valid_record(c.reservation_manifest_action_hash.clone())?;
                    let manifest = match record.entry().to_app_option::<FederatedReservationManifest>()
                        .map_err(|e| wasm_error!(WasmErrorInner::Guest(format!("reservation manifest decode: {e}"))))? {
                        Some(m) => m,
                        None => return invalid("reservation manifest dependency has the wrong entry type"),
                    };
                    if let Err(e) = validate_manifest_shape(&manifest) { return invalid(format!("invalid referenced manifest: {e}")); }
                    if let Err(e) = claim_matches_manifest(&c, &manifest) { return invalid(e); }
                    if action.author() != &manifest.home_agent || action.author() != &c.home_agent {
                        return invalid("obligation claim must be authored by the manifest's designated home agent");
                    }
                    valid()
                }
                EntryTypes::FederatedSettlementEvidence(s) => {
                    if let Err(e) = validate_settlement_shape(&s) { return invalid(e); }
                    let record = must_get_valid_record(s.obligation_claim_action_hash.clone())?;
                    let claim = match record.entry().to_app_option::<FederatedObligationClaim>()
                        .map_err(|e| wasm_error!(WasmErrorInner::Guest(format!("obligation claim decode: {e}"))))? {
                        Some(c) => c,
                        None => return invalid("settlement dependency has the wrong entry type"),
                    };
                    if let Err(e) = validate_claim_shape(&claim) { return invalid(format!("invalid referenced claim: {e}")); }
                    if let Err(e) = claim_matches_settlement(&claim, &s) { return invalid(e); }
                    if action.author() != &s.home_agent {
                        return invalid("settlement evidence must be authored by the designated home agent for profile v1");
                    }
                    valid()
                }
                EntryTypes::Anchor(_) => valid(),
            },
            OpEntry::UpdateEntry { app_entry, .. } => match app_entry {
                EntryTypes::FederatedReservationManifest(_)
                | EntryTypes::FederatedObligationClaim(_)
                | EntryTypes::FederatedSettlementEvidence(_) => invalid("federated clearing evidence is append-only"),
                EntryTypes::Anchor(_) => invalid("anchors are immutable"),
            },
            _ => valid(),
        },
        FlatOp::RegisterCreateLink { .. } => valid(),
        FlatOp::RegisterDeleteLink { link_type, .. } => match link_type {
            LinkTypes::ReservationManifestById
            | LinkTypes::ObligationClaimByEventId
            | LinkTypes::ObligationClaimByCanonicalWork
            | LinkTypes::SettlementEvidenceById => invalid("federated clearing index links cannot be deleted"),
        },
        FlatOp::StoreRecord(_) | FlatOp::RegisterUpdate(_) | FlatOp::RegisterDelete(_) | FlatOp::RegisterAgentActivity(_) => valid(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn agent() -> AgentPubKey { AgentPubKey::from_raw_32(vec![7u8; 32]) }
    fn valid_manifest() -> FederatedReservationManifest {
        FederatedReservationManifest {
            schema_version: 1,
            manifest_id: "manifest-001".into(),
            issuer_community_did: "did:mycelix:issuer".into(),
            community_did: "did:mycelix:community-a".into(),
            instrument_id: "UNIT-1".into(),
            profile_revision: 1,
            agreement_id: "agreement-1".into(),
            direction: ClearingDirection::AToB,
            shard_id: "shard-a".into(),
            epoch: 4,
            reserved_units: "1000".into(),
            home_agent: agent(),
            authority_evidence_digest_hex: "a".repeat(64),
            created_at: Timestamp::from_micros(1),
        }
    }
    fn valid_claim() -> FederatedObligationClaim {
        FederatedObligationClaim {
            schema_version: 1,
            event_id: "event-001".into(),
            request_id: "request-001".into(),
            principal_did: "did:mycelix:alice".into(),
            community_did: "did:mycelix:community-a".into(),
            instrument_id: "UNIT-1".into(),
            profile_revision: 1,
            agreement_id: "agreement-1".into(),
            direction: ClearingDirection::AToB,
            reservation_manifest_action_hash: ActionHash::from_raw_32(vec![4u8; 32]),
            shard_id: "shard-a".into(),
            epoch: 4,
            home_agent: agent(),
            amount_units: "60".into(),
            canonical_work_digest_hex: "b".repeat(64),
            payload_digest_hex: "c".repeat(64),
            verifier_version: "verifier-v1".into(),
            verifier_receipt_digest_hex: "d".repeat(64),
            postgres_receipt_digest_hex: "e".repeat(64),
            created_at: Timestamp::from_micros(2),
        }
    }

    #[test]
    fn exact_integer_units_accept_up_to_numeric_78_digit_bound() {
        assert!(valid_positive_units("1"));
        assert!(valid_positive_units(&"9".repeat(78)));
        assert!(!valid_positive_units("0"));
        assert!(!valid_positive_units("01"));
        assert!(!valid_positive_units("1.0"));
        assert!(!valid_positive_units("1e3"));
        assert!(!valid_positive_units("-1"));
        assert!(!valid_positive_units(&"9".repeat(79)));
    }

    #[test]
    fn manifest_requires_exact_ids_units_and_digest() {
        let mut m = valid_manifest();
        assert!(validate_manifest_shape(&m).is_ok());
        m.reserved_units = "001".into();
        assert!(validate_manifest_shape(&m).is_err());
        m = valid_manifest();
        m.authority_evidence_digest_hex = "xyz".into();
        assert!(validate_manifest_shape(&m).is_err());
        m = valid_manifest();
        m.community_did = "community-a".into();
        assert!(validate_manifest_shape(&m).is_err());
    }

    #[test]
    fn obligation_claim_must_match_manifest_scope_and_home_agent() {
        let m = valid_manifest();
        let mut c = valid_claim();
        assert!(validate_claim_shape(&c).is_ok());
        assert!(claim_matches_manifest(&c, &m).is_ok());
        c.epoch += 1;
        assert!(claim_matches_manifest(&c, &m).is_err());
        c = valid_claim();
        c.amount_units = "1.5".into();
        assert!(validate_claim_shape(&c).is_err());
        c = valid_claim();
        c.home_agent = AgentPubKey::from_raw_32(vec![9u8; 32]);
        assert!(claim_matches_manifest(&c, &m).is_err());
    }

    #[test]
    fn settlement_must_exactly_match_claim_instrument_direction_and_units() {
        let mut c = valid_claim();
        let mut s = FederatedSettlementEvidence {
            schema_version: 1,
            settlement_id: "settlement-1".into(),
            event_id: "settlement-event-1".into(),
            obligation_claim_action_hash: ActionHash::from_raw_32(vec![4u8; 32]),
            community_did: c.community_did.clone(),
            instrument_id: c.instrument_id.clone(),
            direction: c.direction,
            amount_units: c.amount_units.clone(),
            settlement_digest_hex: "f".repeat(64),
            authority_evidence_digest_hex: "1".repeat(64),
            home_agent: agent(),
            created_at: Timestamp::from_micros(3),
        };
        assert!(validate_settlement_shape(&s).is_ok());
        assert!(claim_matches_settlement(&c, &s).is_ok());
        s.amount_units = "59".into();
        assert!(claim_matches_settlement(&c, &s).is_err());
        s.amount_units = "60".into();
        s.instrument_id = "OTHER".into();
        assert!(claim_matches_settlement(&c, &s).is_err());
        s.instrument_id = c.instrument_id.clone();
        c.direction = ClearingDirection::BToA;
        assert!(claim_matches_settlement(&c, &s).is_err());
    }
}
