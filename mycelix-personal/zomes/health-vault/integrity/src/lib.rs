// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Health Vault Integrity Zome
//!
//! Defines entry types and validation for the agent's private health data.
//! Uses consent-gated access (mirrors the health MVP consent pattern).

use hdi::prelude::*;
use serde::{Deserialize, Serialize};

/// A health record stored privately on the source chain.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct HealthRecord {
    /// Type of record: "allergy", "medication", "condition", "vaccination", etc.
    pub record_type: String,
    /// Structured data payload (JSON).
    pub data: String,
    /// Who created this record: "self", "provider:did:key:z6Mk...", etc.
    pub source: String,
    /// When the health event occurred (not when the record was created).
    pub event_date: Timestamp,
    /// When the record was last updated in the vault.
    pub updated_at: Timestamp,
}

/// Biometric measurement stored privately.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Biometric {
    /// Type: "heart_rate", "blood_pressure", "temperature", "weight", etc.
    pub metric_type: String,
    /// Numeric value.
    pub value: f64,
    /// Unit: "bpm", "mmHg", "celsius", "kg", etc.
    pub unit: String,
    /// Measurement timestamp.
    pub measured_at: Timestamp,
}

/// Consent grant for sharing health data with a specific agent or service.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct ConsentGrant {
    /// Who is granted access.
    pub grantee: AgentPubKey,
    /// What record types are accessible: ["allergy", "medication"] or ["*"] for all.
    pub record_types: Vec<String>,
    /// When the consent expires (None = indefinite).
    pub expires_at: Option<Timestamp>,
    /// Whether this consent is currently active.
    pub active: bool,
    /// Timestamp of grant creation.
    pub created_at: Timestamp,
}

/// Append-only event revoking one specific consent grant.
/// The signed creation action timestamp is the authoritative revocation time.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct ConsentRevocation {
    /// Hash of the original ConsentGrant Create action, never an update action.
    pub grant_hash: ActionHash,
}

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    #[entry_type(visibility = "private")]
    HealthRecord(HealthRecord),
    #[entry_type(visibility = "private")]
    Biometric(Biometric),
    #[entry_type(visibility = "private")]
    ConsentGrant(ConsentGrant),
    #[entry_type(visibility = "private")]
    ConsentRevocation(ConsentRevocation),
}

#[hdk_link_types]
pub enum LinkTypes {
    AgentToRecords,
    AgentToBiometrics,
    AgentToConsents,
    RecordTypeToRecord,
    /// Owner -> immutable consent-revocation events.
    AgentToConsentRevocations,
}

#[hdk_extern]
pub fn genesis_self_check(_data: GenesisSelfCheckData) -> ExternResult<ValidateCallbackResult> {
    Ok(ValidateCallbackResult::Valid)
}

#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(OpEntry::CreateEntry {
            app_entry,
            action,
        }) => match app_entry {
            EntryTypes::HealthRecord(record) => validate_health_record(&record),
            EntryTypes::Biometric(biometric) => validate_biometric(&biometric),
            EntryTypes::ConsentGrant(consent) => validate_consent(&consent),
            EntryTypes::ConsentRevocation(revocation) => {
                validate_consent_revocation(&revocation, &action)
            }
        },
        FlatOp::StoreEntry(OpEntry::UpdateEntry { app_entry, .. }) => match app_entry {
            EntryTypes::HealthRecord(record) => validate_health_record(&record),
            EntryTypes::Biometric(biometric) => validate_biometric(&biometric),
            EntryTypes::ConsentGrant(_) => Ok(ValidateCallbackResult::Invalid(
                "ConsentGrant entries are immutable; create a new grant to change access".into(),
            )),
            EntryTypes::ConsentRevocation(_) => Ok(ValidateCallbackResult::Invalid(
                "Consent revocation events are immutable".into(),
            )),
        },
        FlatOp::StoreEntry(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterUpdate(update) => {
            let action = match &update {
                OpUpdate::Entry { action, .. }
                | OpUpdate::PrivateEntry { action, .. }
                | OpUpdate::Agent { action, .. }
                | OpUpdate::CapClaim { action, .. }
                | OpUpdate::CapGrant { action, .. } => action,
            };
            // Private entry content is unavailable to remote validators. The action header
            // still proves the original author and app entry type.
            let original = must_get_action(action.original_action_address.clone())?;
            if action_has_entry_type(original.action(), UnitEntryTypes::ConsentRevocation)? {
                return Ok(ValidateCallbackResult::Invalid(
                    "Consent revocation events are immutable".into(),
                ));
            }
            if action_has_entry_type(original.action(), UnitEntryTypes::ConsentGrant)? {
                return Ok(ValidateCallbackResult::Invalid(
                    "ConsentGrant entries are immutable; create a new grant to change access".into(),
                ));
            }
            if !action_authors_match(original.action().author(), &action.author) {
                return Ok(ValidateCallbackResult::Invalid(
                    "Only the original entry author can update health-vault entries".into(),
                ));
            }
            Ok(ValidateCallbackResult::Valid)
        }
        FlatOp::RegisterCreateLink { link_type, .. } => match link_type {
            LinkTypes::AgentToRecords
            | LinkTypes::AgentToBiometrics
            | LinkTypes::AgentToConsents
            | LinkTypes::RecordTypeToRecord
            | LinkTypes::AgentToConsentRevocations => Ok(ValidateCallbackResult::Invalid(
                "Public health-vault index links are disabled; query the owner's private source chain".into(),
            )),
        },
        FlatOp::RegisterDelete(OpDelete { action }) => {
            let original = must_get_action(action.deletes_address.clone())?;
            if action_has_entry_type(original.action(), UnitEntryTypes::ConsentRevocation)? {
                return Ok(ValidateCallbackResult::Invalid(
                    "Consent revocation events cannot be deleted".into(),
                ));
            }
            if action_has_entry_type(original.action(), UnitEntryTypes::ConsentGrant)? {
                return Ok(ValidateCallbackResult::Invalid(
                    "Consent grants cannot be deleted; use an explicit revocation event".into(),
                ));
            }
            Ok(ValidateCallbackResult::Valid)
        }
        FlatOp::RegisterDeleteLink { link_type, .. } => match link_type {
            LinkTypes::AgentToRecords
            | LinkTypes::AgentToBiometrics
            | LinkTypes::AgentToConsents
            | LinkTypes::RecordTypeToRecord
            | LinkTypes::AgentToConsentRevocations => Ok(ValidateCallbackResult::Invalid(
                "Public health-vault index links are disabled".into(),
            )),
        },
        _ => Ok(ValidateCallbackResult::Valid),
    }
}

const VALID_RECORD_TYPES: &[&str] = &[
    "allergy",
    "medication",
    "condition",
    "vaccination",
    "surgery",
    "lab_result",
    "imaging",
    "visit_note",
];

fn validate_health_record(record: &HealthRecord) -> ExternResult<ValidateCallbackResult> {
    if record.record_type.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "HealthRecord record_type cannot be empty".into(),
        ));
    }
    if !VALID_RECORD_TYPES.contains(&record.record_type.as_str()) {
        return Ok(ValidateCallbackResult::Invalid(format!(
            "Invalid record_type '{}'. Must be one of: {:?}",
            record.record_type, VALID_RECORD_TYPES
        )));
    }
    if record.data.len() > 65536 {
        return Ok(ValidateCallbackResult::Invalid(
            "HealthRecord data must be <= 65536 bytes".into(),
        ));
    }
    if record.source.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "HealthRecord source cannot be empty".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

const VALID_METRIC_TYPES: &[&str] = &[
    "heart_rate",
    "blood_pressure",
    "temperature",
    "weight",
    "blood_oxygen",
    "glucose",
    "steps",
    "sleep_hours",
];

fn validate_biometric(biometric: &Biometric) -> ExternResult<ValidateCallbackResult> {
    if biometric.metric_type.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Biometric metric_type cannot be empty".into(),
        ));
    }
    if !VALID_METRIC_TYPES.contains(&biometric.metric_type.as_str()) {
        return Ok(ValidateCallbackResult::Invalid(format!(
            "Invalid metric_type '{}'. Must be one of: {:?}",
            biometric.metric_type, VALID_METRIC_TYPES
        )));
    }
    if biometric.unit.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Biometric unit cannot be empty".into(),
        ));
    }
    if biometric.value.is_nan() || biometric.value.is_infinite() {
        return Ok(ValidateCallbackResult::Invalid(
            "Biometric value must be a finite number".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_consent(consent: &ConsentGrant) -> ExternResult<ValidateCallbackResult> {
    if consent.record_types.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "ConsentGrant must specify at least one record_type".into(),
        ));
    }
    if consent.record_types.len() > 20 {
        return Ok(ValidateCallbackResult::Invalid(
            "ConsentGrant must have <= 20 record_types".into(),
        ));
    }
    if let Some(expires_at) = &consent.expires_at {
        if expires_at.as_micros() <= consent.created_at.as_micros() {
            return Ok(ValidateCallbackResult::Invalid(
                "ConsentGrant expires_at must be later than created_at".into(),
            ));
        }
    }
    Ok(ValidateCallbackResult::Valid)
}

fn action_authors_match(original_author: &AgentPubKey, new_author: &AgentPubKey) -> bool {
    original_author == new_author
}

fn action_has_entry_type(action: &Action, expected: UnitEntryTypes) -> ExternResult<bool> {
    let expected = EntryType::App(AppEntryDef::try_from(expected)?);
    match action {
        Action::Create(create) => Ok(create.entry_type == expected),
        Action::Update(update) => Ok(update.entry_type == expected),
        _ => Ok(false),
    }
}

fn validate_consent_revocation(
    revocation: &ConsentRevocation,
    action: &Create,
) -> ExternResult<ValidateCallbackResult> {
    // ConsentGrant contents are private. Validate only properties proven by the public action
    // header here; the owner-side coordinator also verifies the actual local grant before write.
    let original = must_get_action(revocation.grant_hash.clone())?;
    if !matches!(original.action(), Action::Create(_)) {
        return Ok(ValidateCallbackResult::Invalid(
            "Consent revocation must reference the original grant creation action".into(),
        ));
    }
    if !action_has_entry_type(original.action(), UnitEntryTypes::ConsentGrant)? {
        return Ok(ValidateCallbackResult::Invalid(
            "Consent revocation target must be a ConsentGrant creation action".into(),
        ));
    }
    if !action_authors_match(original.action().author(), &action.author) {
        return Ok(ValidateCallbackResult::Invalid(
            "Only the consent grant owner can revoke the grant".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_record(record_type: &str) -> HealthRecord {
        HealthRecord {
            record_type: record_type.into(),
            data: r#"{"detail":"test"}"#.into(),
            source: "self".into(),
            event_date: Timestamp::from_micros(0),
            updated_at: Timestamp::from_micros(0),
        }
    }

    fn make_biometric(metric: &str, val: f64) -> Biometric {
        Biometric {
            metric_type: metric.into(),
            value: val,
            unit: "bpm".into(),
            measured_at: Timestamp::from_micros(0),
        }
    }

    fn make_consent() -> ConsentGrant {
        ConsentGrant {
            grantee: AgentPubKey::from_raw_36(vec![0u8; 36]),
            record_types: vec!["allergy".into()],
            expires_at: None,
            active: true,
            created_at: Timestamp::from_micros(0),
        }
    }

    // ── Health record validation ────────────────────────────────────

    #[test]
    fn valid_record_types_accepted() {
        for rt in VALID_RECORD_TYPES {
            let r = make_record(rt);
            assert!(
                matches!(
                    validate_health_record(&r).unwrap(),
                    ValidateCallbackResult::Valid
                ),
                "record_type '{}' should be valid",
                rt
            );
        }
    }

    #[test]
    fn invalid_record_type_rejected() {
        let r = make_record("x_ray_vision");
        match validate_health_record(&r).unwrap() {
            ValidateCallbackResult::Invalid(msg) => assert!(msg.contains("x_ray_vision")),
            other => panic!("expected Invalid, got {:?}", other),
        }
    }

    #[test]
    fn empty_record_type_rejected() {
        let r = make_record("");
        match validate_health_record(&r).unwrap() {
            ValidateCallbackResult::Invalid(msg) => assert!(msg.contains("empty")),
            other => panic!("expected Invalid, got {:?}", other),
        }
    }

    #[test]
    fn oversized_data_rejected() {
        let mut r = make_record("allergy");
        r.data = "x".repeat(65537);
        match validate_health_record(&r).unwrap() {
            ValidateCallbackResult::Invalid(msg) => assert!(msg.contains("65536")),
            other => panic!("expected Invalid, got {:?}", other),
        }
    }

    #[test]
    fn empty_source_rejected() {
        let mut r = make_record("allergy");
        r.source = String::new();
        match validate_health_record(&r).unwrap() {
            ValidateCallbackResult::Invalid(msg) => assert!(msg.contains("empty")),
            other => panic!("expected Invalid, got {:?}", other),
        }
    }

    // ── Biometric validation ────────────────────────────────────────

    #[test]
    fn valid_biometric_passes() {
        let b = make_biometric("heart_rate", 72.0);
        assert!(matches!(
            validate_biometric(&b).unwrap(),
            ValidateCallbackResult::Valid
        ));
    }

    #[test]
    fn nan_biometric_rejected() {
        let b = make_biometric("heart_rate", f64::NAN);
        match validate_biometric(&b).unwrap() {
            ValidateCallbackResult::Invalid(msg) => assert!(msg.contains("finite")),
            other => panic!("expected Invalid, got {:?}", other),
        }
    }

    #[test]
    fn infinite_biometric_rejected() {
        let b = make_biometric("heart_rate", f64::INFINITY);
        match validate_biometric(&b).unwrap() {
            ValidateCallbackResult::Invalid(msg) => assert!(msg.contains("finite")),
            other => panic!("expected Invalid, got {:?}", other),
        }
    }

    #[test]
    fn invalid_metric_type_rejected() {
        let b = make_biometric("chakra_alignment", 7.0);
        match validate_biometric(&b).unwrap() {
            ValidateCallbackResult::Invalid(msg) => assert!(msg.contains("chakra_alignment")),
            other => panic!("expected Invalid, got {:?}", other),
        }
    }

    #[test]
    fn empty_unit_rejected() {
        let mut b = make_biometric("heart_rate", 72.0);
        b.unit = String::new();
        match validate_biometric(&b).unwrap() {
            ValidateCallbackResult::Invalid(msg) => assert!(msg.contains("empty")),
            other => panic!("expected Invalid, got {:?}", other),
        }
    }

    // ── Consent validation ──────────────────────────────────────────

    #[test]
    fn valid_consent_passes() {
        let c = make_consent();
        assert!(matches!(
            validate_consent(&c).unwrap(),
            ValidateCallbackResult::Valid
        ));
    }

    #[test]
    fn empty_record_types_rejected() {
        let mut c = make_consent();
        c.record_types = vec![];
        match validate_consent(&c).unwrap() {
            ValidateCallbackResult::Invalid(msg) => assert!(msg.contains("at least one")),
            other => panic!("expected Invalid, got {:?}", other),
        }
    }

    #[test]
    fn too_many_record_types_rejected() {
        let mut c = make_consent();
        c.record_types = (0..21).map(|i| format!("type_{}", i)).collect();
        match validate_consent(&c).unwrap() {
            ValidateCallbackResult::Invalid(msg) => assert!(msg.contains("20")),
            other => panic!("expected Invalid, got {:?}", other),
        }
    }

    #[test]
    fn consent_expiry_must_follow_creation() {
        let mut consent = make_consent();
        consent.expires_at = Some(Timestamp::from_micros(0));
        match validate_consent(&consent).unwrap() {
            ValidateCallbackResult::Invalid(msg) => {
                assert!(msg.contains("later than created_at"));
            }
            other => panic!("expected Invalid, got {:?}", other),
        }

        consent.expires_at = Some(Timestamp::from_micros(1));
        assert!(matches!(
            validate_consent(&consent).unwrap(),
            ValidateCallbackResult::Valid
        ));
    }

    #[test]
    fn author_binding_accepts_original_owner_only() {
        let owner = AgentPubKey::from_raw_36(vec![1u8; 36]);
        let other = AgentPubKey::from_raw_36(vec![2u8; 36]);
        assert!(action_authors_match(&owner, &owner));
        assert!(!action_authors_match(&owner, &other));
    }

    #[test]
    fn private_entry_type_indices_remain_registered() {
        let _record = UnitEntryTypes::HealthRecord;
        let _biometric = UnitEntryTypes::Biometric;
        let _consent = UnitEntryTypes::ConsentGrant;
        let _revocation = UnitEntryTypes::ConsentRevocation;
    }

    // ── Serde roundtrips ────────────────────────────────────────────

    #[test]
    fn health_record_serde_roundtrip() {
        let r = make_record("medication");
        let json = serde_json::to_string(&r).unwrap();
        let back: HealthRecord = serde_json::from_str(&json).unwrap();
        assert_eq!(back, r);
    }

    #[test]
    fn biometric_serde_roundtrip() {
        let b = make_biometric("temperature", 36.6);
        let json = serde_json::to_string(&b).unwrap();
        let back: Biometric = serde_json::from_str(&json).unwrap();
        assert_eq!(back, b);
    }

    #[test]
    fn consent_grant_serde_roundtrip() {
        let c = make_consent();
        let json = serde_json::to_string(&c).unwrap();
        let back: ConsentGrant = serde_json::from_str(&json).unwrap();
        assert_eq!(back, c);
    }

    #[test]
    fn consent_revocation_serde_roundtrip() {
        let event = ConsentRevocation {
            grant_hash: ActionHash::from_raw_39(vec![0u8; 39]).unwrap(),
        };
        let json = serde_json::to_string(&event).unwrap();
        let back: ConsentRevocation = serde_json::from_str(&json).unwrap();
        assert_eq!(back, event);
    }
}
