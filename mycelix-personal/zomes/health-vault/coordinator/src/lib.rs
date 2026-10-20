// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Health Vault Coordinator Zome
//!
//! CRUD operations for the agent's private health data.
//! Access is gated by consent grants.

use base64::{Engine as _, engine::general_purpose::STANDARD as BASE64};
use hdk::prelude::*;
use health_vault_integrity::*;

getrandom::register_custom_getrandom!(my_custom_getrandom);

pub fn my_custom_getrandom(buf: &mut [u8]) -> Result<(), getrandom::Error> {
    let bytes = random_bytes(buf.len() as u32).map_err(|_| getrandom::Error::UNSUPPORTED)?;
    buf.copy_from_slice(bytes.as_ref());
    Ok(())
}

use personal_leptos_types::{
    BiometricView, ConsentGrantInputView, ConsentGrantView, HealthRecordView,
};

fn decode_agent_pubkey(pubkey: &str) -> ExternResult<AgentPubKey> {
    let bytes = BASE64.decode(pubkey).map_err(|e| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "Invalid agent pubkey encoding: {e}"
        )))
    })?;
    if bytes.len() != 39 {
        return Err(wasm_error!(WasmErrorInner::Guest(format!(
            "Agent pubkey wrong length: {} bytes (expected 39)",
            bytes.len()
        ))));
    }
    Ok(AgentPubKey::from_raw_39(bytes))
}

/// Create a health record on the agent's source chain.
#[hdk_extern]
pub fn create_health_record(record: HealthRecord) -> ExternResult<Record> {
    let action_hash = create_entry(&EntryTypes::HealthRecord(record.clone()))?;
    let agent = agent_info()?.agent_initial_pubkey;
    create_link(
        agent.clone(),
        action_hash.clone(),
        LinkTypes::AgentToRecords,
        (),
    )?;

    let type_anchor_hash = hash_entry(&EntryTypes::HealthRecord(HealthRecord {
        record_type: record.record_type.clone(),
        data: String::new(),
        source: String::new(),
        event_date: Timestamp::from_micros(0),
        updated_at: Timestamp::from_micros(0),
    }))?;
    let _ = create_link(
        agent,
        type_anchor_hash,
        LinkTypes::RecordTypeToRecord,
        record.record_type.as_bytes().to_vec(),
    );

    get(action_hash, GetOptions::default())?.ok_or(wasm_error!(WasmErrorInner::Guest(
        "Could not retrieve created health record".into()
    )))
}

/// Store a biometric measurement.
#[hdk_extern]
pub fn record_biometric(biometric: Biometric) -> ExternResult<Record> {
    let action_hash = create_entry(&EntryTypes::Biometric(biometric))?;
    let agent = agent_info()?.agent_initial_pubkey;
    create_link(agent, action_hash.clone(), LinkTypes::AgentToBiometrics, ())?;
    get(action_hash, GetOptions::default())?.ok_or(wasm_error!(WasmErrorInner::Guest(
        "Could not retrieve created biometric".into()
    )))
}

/// Grant consent for a specific agent to read health records.
#[hdk_extern]
pub fn grant_consent(consent: ConsentGrant) -> ExternResult<Record> {
    let action_hash = create_entry(&EntryTypes::ConsentGrant(consent))?;
    let agent = agent_info()?.agent_initial_pubkey;
    create_link(agent, action_hash.clone(), LinkTypes::AgentToConsents, ())?;
    get(action_hash, GetOptions::default())?.ok_or(wasm_error!(WasmErrorInner::Guest(
        "Could not retrieve created consent".into()
    )))
}

/// Revoke a grant by its original ConsentGrant creation action hash.
/// Repeated calls are idempotent: the pre-existing revocation event is returned.
#[hdk_extern]
pub fn revoke_consent(grant_hash: ActionHash) -> ExternResult<Record> {
    let owner = agent_info()?.agent_initial_pubkey;
    let target = get(grant_hash.clone(), GetOptions::default())?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Consent grant was not found".into()
        )))?;
    if !matches!(target.action(), Action::Create(_)) {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Only the original ConsentGrant creation action can be revoked".into()
        )));
    }
    let _consent = target
        .entry()
        .to_app_option::<ConsentGrant>()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Target is not a ConsentGrant".into()
        )))?;
    if target.action().author() != &owner {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Only the grant owner can revoke this consent".into()
        )));
    }

    let existing = get_my_consent_revocation_records()?;
    if let Some((record, _)) = existing
        .into_iter()
        .find(|(_, event)| event.grant_hash == grant_hash)
    {
        return Ok(record);
    }

    let revocation_hash = create_entry(&EntryTypes::ConsentRevocation(ConsentRevocation {
        grant_hash,
    }))?;
    create_link(
        owner,
        revocation_hash.clone(),
        LinkTypes::AgentToConsentRevocations,
        (),
    )?;
    get(revocation_hash, GetOptions::default())?.ok_or(wasm_error!(WasmErrorInner::Guest(
        "Could not retrieve created consent revocation".into()
    )))
}

/// Return the owner's immutable revocation-event history for audit and reconciliation.
#[hdk_extern]
pub fn get_my_consent_revocations(_: ()) -> ExternResult<Vec<Record>> {
    Ok(get_my_consent_revocation_records()?
        .into_iter()
        .map(|(record, _)| record)
        .collect())
}

#[hdk_extern]
pub fn grant_consent_view(input: ConsentGrantInputView) -> ExternResult<Record> {
    let grantee = decode_agent_pubkey(&input.grantee)?;
    grant_consent(ConsentGrant {
        grantee,
        record_types: input.record_types,
        expires_at: input.expires_at.map(Timestamp::from_micros),
        active: input.active,
        created_at: sys_time()?,
    })
}

/// Get all health records for this agent.
#[hdk_extern]
pub fn get_my_records(_: ()) -> ExternResult<Vec<Record>> {
    let agent = agent_info()?.agent_initial_pubkey;
    let links = get_links(
        LinkQuery::try_new(agent, LinkTypes::AgentToRecords)?,
        GetStrategy::Local,
    )?;
    let mut records = Vec::new();
    for link in links {
        let target = ActionHash::try_from(link.target.clone()).map_err(|e| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "Invalid link target: {:?}",
                e
            )))
        })?;
        if let Some(record) = get(target, GetOptions::default())? {
            records.push(record);
        }
    }
    Ok(records)
}

#[hdk_extern]
pub fn get_my_records_view(_: ()) -> ExternResult<Vec<HealthRecordView>> {
    let records = get_my_records(())?;
    let mut views = Vec::new();
    for record in records {
        if let Some(entry) = record
            .entry()
            .to_app_option::<HealthRecord>()
            .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        {
            views.push(HealthRecordView {
                hash: record.action_address().to_string(),
                record_type: entry.record_type,
                data: entry.data,
                source: entry.source,
                event_date: entry.event_date.as_micros(),
                updated_at: entry.updated_at.as_micros(),
            });
        }
    }
    Ok(views)
}

/// Get all biometric measurements for this agent.
#[hdk_extern]
pub fn get_my_biometrics(_: ()) -> ExternResult<Vec<Record>> {
    let agent = agent_info()?.agent_initial_pubkey;
    let links = get_links(
        LinkQuery::try_new(agent, LinkTypes::AgentToBiometrics)?,
        GetStrategy::Local,
    )?;
    let mut records = Vec::new();
    for link in links {
        let target = ActionHash::try_from(link.target.clone()).map_err(|e| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "Invalid link target: {:?}",
                e
            )))
        })?;
        if let Some(record) = get(target, GetOptions::default())? {
            records.push(record);
        }
    }
    Ok(records)
}

#[hdk_extern]
pub fn get_my_biometrics_view(_: ()) -> ExternResult<Vec<BiometricView>> {
    let records = get_my_biometrics(())?;
    let mut views = Vec::new();
    for record in records {
        if let Some(entry) = record
            .entry()
            .to_app_option::<Biometric>()
            .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        {
            views.push(BiometricView {
                hash: record.action_address().to_string(),
                metric_type: entry.metric_type,
                value: entry.value,
                unit: entry.unit,
                measured_at: entry.measured_at.as_micros(),
            });
        }
    }
    Ok(views)
}

/// A grant is effective at `now` only when active, time-valid, and not revoked.
/// Equality at the expiry boundary is expired (fail closed).
fn consent_is_effective_at(
    grant_hash: &ActionHash,
    consent: &ConsentGrant,
    now: &Timestamp,
    revocations: &[ConsentRevocation],
) -> bool {
    consent.active
        && consent.created_at.as_micros() <= now.as_micros()
        && consent
            .expires_at
            .as_ref()
            .map(|expires_at| expires_at.as_micros() > now.as_micros())
            .unwrap_or(true)
        && !consent_is_revoked(grant_hash, revocations)
}

fn consent_is_revoked(grant_hash: &ActionHash, revocations: &[ConsentRevocation]) -> bool {
    revocations
        .iter()
        .any(|revocation| &revocation.grant_hash == grant_hash)
}

/// Load revocation events linked by their authoring owner.
/// The creation action is the immutable source of the event timestamp.
fn get_my_consent_revocation_records() -> ExternResult<Vec<(Record, ConsentRevocation)>> {
    let agent = agent_info()?.agent_initial_pubkey;
    let links = get_links(
        LinkQuery::try_new(agent.clone(), LinkTypes::AgentToConsentRevocations)?,
        GetStrategy::Local,
    )?;
    let mut events = Vec::new();
    for link in links {
        let target = ActionHash::try_from(link.target.clone()).map_err(|e| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "Invalid revocation link target: {:?}",
                e
            )))
        })?;
        if let Some(record) = get(target, GetOptions::default())? {
            if record.action().author() != &agent {
                continue;
            }
            if let Some(event) = record
                .entry()
                .to_app_option::<ConsentRevocation>()
                .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
            {
                events.push((record, event));
            }
        }
    }
    Ok(events)
}

/// Fetch the current-effective grants using one evaluation timestamp.
/// This is a source-local listing projection, not a grantee read-authorization gate.
fn get_my_consents_at(now: &Timestamp) -> ExternResult<Vec<Record>> {
    let agent = agent_info()?.agent_initial_pubkey;
    let revocations: Vec<ConsentRevocation> = get_my_consent_revocation_records()?
        .into_iter()
        .map(|(_, event)| event)
        .collect();
    let links = get_links(
        LinkQuery::try_new(agent, LinkTypes::AgentToConsents)?,
        GetStrategy::Local,
    )?;
    let mut records = Vec::new();
    for link in links {
        let target = ActionHash::try_from(link.target.clone()).map_err(|e| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "Invalid link target: {:?}",
                e
            )))
        })?;
        if let Some(record) = get(target.clone(), GetOptions::default())? {
            let consent = record
                .entry()
                .to_app_option::<ConsentGrant>()
                .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?;
            if let Some(consent) = consent {
                if consent_is_effective_at(&target, &consent, now, &revocations) {
                    records.push(record);
                }
            }
        }
    }
    Ok(records)
}

/// Get all currently effective consent grants for this agent.
/// Expired, inactive, missing, and non-consent link targets are not returned.
#[hdk_extern]
pub fn get_my_consents(_: ()) -> ExternResult<Vec<Record>> {
    let now = sys_time()?;
    get_my_consents_at(&now)
}

#[hdk_extern]
pub fn get_my_consents_view(_: ()) -> ExternResult<Vec<ConsentGrantView>> {
    let now = sys_time()?;
    let records = get_my_consents_at(&now)?;
    let mut views = Vec::new();
    for record in records {
        if let Some(entry) = record
            .entry()
            .to_app_option::<ConsentGrant>()
            .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        {
            views.push(ConsentGrantView {
                hash: record.action_address().to_string(),
                grantee: entry.grantee.to_string(),
                record_types: entry.record_types,
                expires_at: entry.expires_at.map(|ts| ts.as_micros()),
                active: true,
                created_at: entry.created_at.as_micros(),
            });
        }
    }
    Ok(views)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn entry_types_exist() {
        let _record = UnitEntryTypes::HealthRecord;
        let _bio = UnitEntryTypes::Biometric;
        let _consent = UnitEntryTypes::ConsentGrant;
        let _revocation = UnitEntryTypes::ConsentRevocation;
    }

    #[test]
    fn consent_effectiveness_fails_closed_at_expiry_boundary() {
        let now = Timestamp::from_micros(100);
        let hash = ActionHash::from_raw_39(vec![7u8; 39]).unwrap();
        let mut consent = ConsentGrant {
            grantee: AgentPubKey::from_raw_36(vec![0u8; 36]),
            record_types: vec!["allergy".into()],
            expires_at: Some(Timestamp::from_micros(101)),
            active: true,
            created_at: Timestamp::from_micros(0),
        };
        let no_revocations: Vec<ConsentRevocation> = Vec::new();

        assert!(consent_is_effective_at(&hash, &consent, &now, &no_revocations));

        consent.expires_at = Some(Timestamp::from_micros(100));
        assert!(!consent_is_effective_at(&hash, &consent, &now, &no_revocations));

        consent.expires_at = Some(Timestamp::from_micros(101));
        consent.created_at = Timestamp::from_micros(101);
        assert!(!consent_is_effective_at(&hash, &consent, &now, &no_revocations));
        consent.created_at = Timestamp::from_micros(0);

        consent.expires_at = Some(Timestamp::from_micros(99));
        assert!(!consent_is_effective_at(&hash, &consent, &now, &no_revocations));
    }

    #[test]
    fn inactive_consent_is_never_effective_and_none_expiry_is_indefinite() {
        let now = Timestamp::from_micros(100);
        let hash = ActionHash::from_raw_39(vec![8u8; 39]).unwrap();
        let mut consent = ConsentGrant {
            grantee: AgentPubKey::from_raw_36(vec![0u8; 36]),
            record_types: vec!["medication".into()],
            expires_at: None,
            active: true,
            created_at: Timestamp::from_micros(0),
        };
        let no_revocations: Vec<ConsentRevocation> = Vec::new();

        assert!(consent_is_effective_at(&hash, &consent, &now, &no_revocations));
        consent.active = false;
        assert!(!consent_is_effective_at(&hash, &consent, &now, &no_revocations));
    }

    #[test]
    fn duplicate_revocations_never_reactivate_a_grant() {
        let target = ActionHash::from_raw_39(vec![9u8; 39]).unwrap();
        let unrelated = ActionHash::from_raw_39(vec![10u8; 39]).unwrap();
        let revocations = vec![
            ConsentRevocation { grant_hash: target.clone() },
            ConsentRevocation { grant_hash: target.clone() },
            ConsentRevocation { grant_hash: unrelated },
        ];
        assert!(consent_is_revoked(&target, &revocations));
        assert!(!consent_is_revoked(
            &ActionHash::from_raw_39(vec![11u8; 39]).unwrap(),
            &revocations
        ));
    }

    #[test]
    fn link_types_exist() {
        let _records = LinkTypes::AgentToRecords;
        let _bio = LinkTypes::AgentToBiometrics;
        let _consents = LinkTypes::AgentToConsents;
        let _type_to_record = LinkTypes::RecordTypeToRecord;
        let _revocations = LinkTypes::AgentToConsentRevocations;
    }
}
