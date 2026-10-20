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
use std::collections::{BTreeSet, HashSet};

getrandom::register_custom_getrandom!(my_custom_getrandom);

pub fn my_custom_getrandom(buf: &mut [u8]) -> Result<(), getrandom::Error> {
    let bytes = random_bytes(buf.len() as u32).map_err(|_| getrandom::Error::UNSUPPORTED)?;
    buf.copy_from_slice(bytes.as_ref());
    Ok(())
}

use personal_leptos_types::{
    BiometricView, ConsentGrantInputView, ConsentGrantView, HealthRecordView,
};

/// Capability offer returned only to the local grant owner. Never put the secret
/// in a shared entry, link tag, or DHT-published record.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ConsentCapabilityOffer {
    pub owner: AgentPubKey,
    pub grantee: AgentPubKey,
    pub grant_hash: ActionHash,
    pub capability_grant_hash: ActionHash,
    pub capability_secret: CapSecret,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GrantConsentWithRemoteCapabilityInput {
    pub grantee: AgentPubKey,
    pub record_types: Vec<String>,
    pub expires_at: Option<Timestamp>,
    pub active: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StoreConsentCapabilityInput {
    pub owner: AgentPubKey,
    pub grant_hash: ActionHash,
    pub capability_secret: CapSecret,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FetchSharedHealthRecordsInput {
    pub owner: AgentPubKey,
    pub grant_hash: ActionHash,
    pub requested_record_types: Vec<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SharedRecordsRemoteInput {
    pub grant_hash: ActionHash,
    pub requested_record_types: Vec<String>,
}

const SHAREABLE_RECORD_TYPES: &[&str] = &[
    "allergy", "medication", "condition", "vaccination", "surgery",
    "lab_result", "imaging", "visit_note",
];

fn consent_capability_tag(grant_hash: &ActionHash) -> String {
    format!("mycelix-health-consent-read-v0:{grant_hash}")
}

fn requested_scope_is_authorized(granted: &[String], requested: &[String]) -> bool {
    !requested.is_empty()
        && requested.len() <= 20
        && requested.iter().all(|requested_type| {
            SHAREABLE_RECORD_TYPES.contains(&requested_type.as_str())
                && granted.iter().any(|granted_type| {
                    granted_type == "*" || granted_type == requested_type
                })
        })
}

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
    let action_hash = create_entry(&EntryTypes::HealthRecord(record))?;
    let agent = agent_info()?.agent_initial_pubkey;
    // The target entry is private. This owner-local index link may reveal that a
    // record action exists, but it never includes the health category in a link tag.
    create_link(
        agent,
        action_hash.clone(),
        LinkTypes::AgentToRecords,
        (),
    )?;

    // Do not write the legacy RecordTypeToRecord link: its public tag exposed the
    // record_type. The variant is retained for entry/link-index compatibility.
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

/// Create a local consent grant without creating a remote-call capability.
/// Remote reads use the separate Assigned-capability flow below.
#[hdk_extern]
pub fn grant_consent(consent: ConsentGrant) -> ExternResult<Record> {
    let action_hash = create_entry(&EntryTypes::ConsentGrant(consent))?;
    let agent = agent_info()?.agent_initial_pubkey;
    create_link(agent, action_hash.clone(), LinkTypes::AgentToConsents, ())?;
    get(action_hash, GetOptions::default())?.ok_or(wasm_error!(WasmErrorInner::Guest(
        "Could not retrieve created consent".into()
    )))
}

/// Grant consent and issue an Assigned capability scoped to exactly one read
/// function. The returned secret is local-call output for delivery over a
/// separate authenticated/private channel, never persisted in the consent entry.
#[hdk_extern]
pub fn grant_consent_with_remote_capability(
    input: GrantConsentWithRemoteCapabilityInput,
) -> ExternResult<ConsentCapabilityOffer> {
    let now = sys_time()?;
    if !input.active {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Remote capability requires an active consent grant".into()
        )));
    }
    if input.record_types.is_empty() || input.record_types.len() > 20 {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Consent must contain between 1 and 20 record types".into()
        )));
    }
    if input.expires_at.is_some_and(|expires_at| expires_at.as_micros() <= now.as_micros()) {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Consent expiry must be later than the current time".into()
        )));
    }

    let grantee = input.grantee;
    let grant_record = grant_consent(ConsentGrant {
        grantee: grantee.clone(),
        record_types: input.record_types,
        expires_at: input.expires_at,
        active: true,
        created_at: now,
    })?;
    let grant_hash = grant_record.action_address().clone();
    let tag = consent_capability_tag(&grant_hash);
    let secret = generate_cap_secret()?;

    let mut functions = HashSet::new();
    functions.insert((
        zome_info()?.name,
        FunctionName::new("get_shared_records_remote"),
    ));
    let assignees = BTreeSet::from([grantee.clone()]);
    let capability_grant_hash = create_cap_grant(CapGrantEntry {
        tag,
        access: CapAccess::Assigned {
            secret: secret.clone(),
            assignees,
        },
        functions: GrantedFunctions::Listed(functions),
    })?;

    Ok(ConsentCapabilityOffer {
        owner: agent_info()?.agent_initial_pubkey,
        grantee,
        grant_hash,
        capability_grant_hash,
        capability_secret: secret,
    })
}


/// Browser-facing wrapper for consent input that represents the grantee key as base64.
/// The capability secret is returned only in this direct local call response.
#[hdk_extern]
pub fn grant_consent_with_remote_capability_view(
    input: ConsentGrantInputView,
) -> ExternResult<ConsentCapabilityOffer> {
    grant_consent_with_remote_capability(GrantConsentWithRemoteCapabilityInput {
        grantee: decode_agent_pubkey(&input.grantee)?,
        record_types: input.record_types,
        expires_at: input.expires_at.map(Timestamp::from_micros),
        active: input.active,
    })
}

/// Store an owner-issued capability as a local CapClaim on the grantee's
/// source chain. A stale/revoked secret is still rejected by the owner at call time.
#[hdk_extern]
pub fn store_shared_health_capability(
    input: StoreConsentCapabilityInput,
) -> ExternResult<ActionHash> {
    let tag = consent_capability_tag(&input.grant_hash);
    let claims = query(
        ChainQueryFilter::new()
            .action_type(ActionType::Create)
            .entry_type(EntryType::CapClaim)
            .include_entries(true),
    )?;
    for record in claims {
        if let Some(Entry::CapClaim(claim)) = record.entry().as_option() {
            if claim.grantor == input.owner && claim.tag == tag {
                return Ok(record.action_address().clone());
            }
        }
    }

    create_cap_claim(CapClaimEntry {
        tag,
        grantor: input.owner,
        secret: input.capability_secret,
    })
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

    // Delete the exact Assigned capability before recording revocation. A failed
    // subsequent write rolls back the zome call transaction.
    revoke_consent_capability(&grant_hash)?;
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

/// Delete owner-local capability grants tagged for this original consent action.
/// The CapGrant is a private system entry, never part of the shared consent record.
fn revoke_consent_capability(grant_hash: &ActionHash) -> ExternResult<()> {
    let wanted_tag = consent_capability_tag(grant_hash);
    let grants = query(
        ChainQueryFilter::new()
            .action_type(ActionType::Create)
            .entry_type(EntryType::CapGrant)
            .include_entries(true),
    )?;
    for record in grants {
        if let Some(Entry::CapGrant(cap_grant)) = record.entry().as_option() {
            if cap_grant.tag == wanted_tag {
                delete_cap_grant(record.action_address().clone())?;
            }
        }
    }
    Ok(())
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

/// Owner-side remote-read authorization. Holochain capability enforcement runs
/// before this handler; it also binds the authenticated caller to the exact grant
/// and current owner-side lifecycle state before returning protected content.
#[hdk_extern]
pub fn get_shared_records_remote(
    input: SharedRecordsRemoteInput,
) -> ExternResult<Vec<HealthRecordView>> {
    let requester = call_info()?.provenance;
    let owner = agent_info()?.agent_initial_pubkey;
    let grant_record = get(input.grant_hash.clone(), GetOptions::default())?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Consent grant is unavailable".into()
        )))?;
    if !matches!(grant_record.action(), Action::Create(_))
        || grant_record.action().author() != &owner
    {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Consent grant must be the owner's original create action".into()
        )));
    }
    let consent = grant_record
        .entry()
        .to_app_option::<ConsentGrant>()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Consent grant entry is invalid".into()
        )))?;
    if consent.grantee != requester {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Authenticated caller is not the consent grantee".into()
        )));
    }

    let now = sys_time()?;
    let revocations: Vec<ConsentRevocation> = get_my_consent_revocation_records()?
        .into_iter()
        .map(|(_, event)| event)
        .collect();
    if !consent_is_effective_at(&input.grant_hash, &consent, &now, &revocations) {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Consent is expired, inactive, revoked, or not yet effective".into()
        )));
    }
    if !requested_scope_is_authorized(&consent.record_types, &input.requested_record_types) {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Requested record types are empty, unsupported, or outside consent scope".into()
        )));
    }

    let requested: HashSet<&str> = input
        .requested_record_types
        .iter()
        .map(String::as_str)
        .collect();
    let records = get_my_records_view(())?;
    Ok(records
        .into_iter()
        .filter(|record| requested.contains(record.record_type.as_str()))
        .collect())
}

/// Call the owner's protected read endpoint using the matching local CapClaim.
/// Unauthorized and network-failure results are errors; no cached grants are used.
#[hdk_extern]
pub fn fetch_shared_health_records(
    input: FetchSharedHealthRecordsInput,
) -> ExternResult<Vec<HealthRecordView>> {
    if input.requested_record_types.is_empty()
        || input.requested_record_types.len() > 20
        || input.requested_record_types.iter().any(|record_type| {
            !SHAREABLE_RECORD_TYPES.contains(&record_type.as_str())
        })
    {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Requested record types must be a non-empty subset of supported categories".into()
        )));
    }

    let wanted_tag = consent_capability_tag(&input.grant_hash);
    let claims = query(
        ChainQueryFilter::new()
            .action_type(ActionType::Create)
            .entry_type(EntryType::CapClaim)
            .include_entries(true),
    )?;
    let claim = claims
        .into_iter()
        .find_map(|record| {
            if let Some(Entry::CapClaim(claim)) = record.entry().as_option() {
                if claim.grantor == input.owner && claim.tag == wanted_tag {
                    return Some(claim.clone());
                }
            }
            None
        })
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "No matching local health-consent capability claim".into()
        )))?;

    let response = call_remote(
        input.owner,
        ZomeName::new("health_vault"),
        FunctionName::new("get_shared_records_remote"),
        Some(claim.secret),
        SharedRecordsRemoteInput {
            grant_hash: input.grant_hash,
            requested_record_types: input.requested_record_types,
        },
    )?;
    match response {
        ZomeCallResponse::Ok(data) => data.decode().map_err(|e| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "Invalid shared-health response: {e}"
            )))
        }),
        ZomeCallResponse::Unauthorized(..) => Err(wasm_error!(
            WasmErrorInner::Guest("Owner revoked or denied this health-read capability".into())
        )),
        _ => Err(wasm_error!(WasmErrorInner::Guest(
            "Shared-health source unavailable or call failed".into()
        ))),
    }
}

/// Fetch the owner's current-effective grants using one evaluation timestamp.
/// This owner-side list is not a substitute for the protected remote-read endpoint.
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
            // Ignore foreign grants even if an older validator accepted a forged owner link.
            if record.action().author() != &agent {
                continue;
            }
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
    fn consent_capability_tag_is_bound_to_original_grant_hash() {
        let first = ActionHash::from_raw_39(vec![21u8; 39]).unwrap();
        let second = ActionHash::from_raw_39(vec![22u8; 39]).unwrap();
        assert_eq!(
            consent_capability_tag(&first),
            format!("mycelix-health-consent-read-v0:{first}")
        );
        assert_ne!(consent_capability_tag(&first), consent_capability_tag(&second));
    }

    #[test]
    fn requested_scope_requires_explicit_grant_and_supported_categories() {
        let grant = vec!["allergy".to_string(), "medication".to_string()];
        assert!(requested_scope_is_authorized(&grant, &["allergy".to_string()]));
        assert!(requested_scope_is_authorized(
            &["*".to_string()],
            &["lab_result".to_string(), "imaging".to_string()]
        ));
        assert!(!requested_scope_is_authorized(&grant, &["imaging".to_string()]));
        assert!(!requested_scope_is_authorized(&grant, &["unknown_record_type".to_string()]));
        assert!(!requested_scope_is_authorized(&grant, &[]));
        assert!(!requested_scope_is_authorized(&grant, &vec!["allergy".to_string(); 21]));
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
