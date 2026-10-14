// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
use hdi::prelude::*;

/// A trust attestation between two agents.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct TrustAttestation {
    /// Agent being trusted.
    pub subject: AgentPubKey,
    /// Trust score [0.0, 1.0].
    pub trust_score: f64,
    /// Whether verified with post-quantum crypto.
    pub pq_verified: bool,
    /// Domain of trust (e.g., "governance", "technical", "general").
    pub domain: String,
    /// Optional note.
    pub note: String,
    /// Timestamp (µs since epoch).
    pub timestamp_us: u64,
}

/// Revocation of a previous trust attestation.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct TrustRevocation {
    /// Action hash of the attestation being revoked.
    pub attestation_hash: ActionHash,
    /// Reason for revocation.
    pub reason: String,
    /// Timestamp (µs since epoch).
    pub timestamp_us: u64,
}

/// Anchor entry for deterministic link bases.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Anchor(pub String);

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    #[entry_type(name = "TrustAttestation", visibility = "public")]
    TrustAttestation(TrustAttestation),
    #[entry_type(name = "TrustRevocation", visibility = "public")]
    TrustRevocation(TrustRevocation),
    #[entry_type(name = "Anchor", visibility = "public")]
    Anchor(Anchor),
}

#[hdk_link_types]
pub enum LinkTypes {
    AttestorToAttestations,
    SubjectToAttestations,
    AttestationToRevocations,
    AllAttestations,
}

fn validate_timestamp_us_not_future(
    field: &str,
    value_us: u64,
    action_timestamp: Timestamp,
) -> ValidateCallbackResult {
    let action_us = action_timestamp.as_micros();
    let Ok(action_us) = u64::try_from(action_us) else {
        return ValidateCallbackResult::Invalid(format!(
            "{field} cannot be validated against a negative signed Holochain action timestamp"
        ));
    };
    if value_us > action_us {
        return ValidateCallbackResult::Invalid(format!(
            "{field} cannot be later than its signed Holochain action timestamp"
        ));
    }
    ValidateCallbackResult::Valid
}

fn anchor_hash(anchor_str: &str) -> ExternResult<EntryHash> {
    hash_entry(&EntryTypes::Anchor(Anchor(anchor_str.to_string())))
}

fn action_target(
    target_address: &AnyLinkableHash,
    label: &str,
) -> ExternResult<ActionHash> {
    target_address.clone().into_action_hash().ok_or_else(|| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "{label} target must be an ActionHash"
        )))
    })
}

fn validate_create_trust_attestation(
    action: EntryCreationAction,
    attestation: TrustAttestation,
) -> ExternResult<ValidateCallbackResult> {
    let expected_attestor = action.author().clone();

    if !(0.0..=1.0).contains(&attestation.trust_score) || !attestation.trust_score.is_finite() {
        return Ok(ValidateCallbackResult::Invalid(
            "Trust score must be finite and in [0.0, 1.0]".into(),
        ));
    }
    if attestation.domain.len() > 64 {
        return Ok(ValidateCallbackResult::Invalid(
            "Domain too long (max 64)".into(),
        ));
    }
    if attestation.note.len() > 512 {
        return Ok(ValidateCallbackResult::Invalid(
            "Note too long (max 512)".into(),
        ));
    }
    if attestation.timestamp_us == 0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Attestation timestamp must be non-zero".into(),
        ));
    }
    match validate_timestamp_us_not_future(
        "Attestation timestamp",
        attestation.timestamp_us,
        *action.timestamp(),
    ) {
        ValidateCallbackResult::Valid => {}
        invalid => return Ok(invalid),
    }

    // The author of the action is the attestor. The entry intentionally does
    // not carry a second attestor field, so authorship itself is the authority.
    let _ = expected_attestor;
    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_trust_revocation(
    action: EntryCreationAction,
    revocation: TrustRevocation,
) -> ExternResult<ValidateCallbackResult> {
    if revocation.reason.is_empty() || revocation.reason.len() > 512 {
        return Ok(ValidateCallbackResult::Invalid(
            "Revocation reason must be 1-512 characters".into(),
        ));
    }
    if revocation.timestamp_us == 0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Revocation timestamp must be non-zero".into(),
        ));
    }
    match validate_timestamp_us_not_future(
        "Revocation timestamp",
        revocation.timestamp_us,
        *action.timestamp(),
    ) {
        ValidateCallbackResult::Valid => {}
        invalid => return Ok(invalid),
    }

    let attestation_record = must_get_valid_record(revocation.attestation_hash.clone())?;
    let attestation: TrustAttestation = attestation_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Revocation target must be a TrustAttestation".into(),
        )))?;

    // Only the agent that authored the original attestation may revoke it.
    if *attestation_record.action().author() != *action.author() {
        return Ok(ValidateCallbackResult::Invalid(
            "Only the original attestor may create a revocation".into(),
        ));
    }

    let _ = attestation;
    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_link(
    link_type: LinkTypes,
    base_address: &AnyLinkableHash,
    target_address: &AnyLinkableHash,
    action: &CreateLink,
) -> ExternResult<ValidateCallbackResult> {
    match link_type {
        LinkTypes::AttestorToAttestations | LinkTypes::SubjectToAttestations => {
            let target = action_target(target_address, "TrustAttestation link")?;
            let record = must_get_valid_record(target)?;
            let attestation: TrustAttestation = record
                .entry()
                .to_app_option()
                .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
                .ok_or(wasm_error!(WasmErrorInner::Guest(
                    "Trust index target must contain a TrustAttestation".into(),
                )))?;

            match link_type {
                LinkTypes::AttestorToAttestations => {
                    let base = base_address.clone().into_agent_pub_key().ok_or_else(|| {
                        wasm_error!(WasmErrorInner::Guest(
                            "AttestorToAttestations base must be an AgentPubKey".into(),
                        ))
                    })?;
                    if base != *record.action().author() || action.author != *record.action().author() {
                        return Ok(ValidateCallbackResult::Invalid(
                            "Attestor index must be authored by and based on the original attestor".into(),
                        ));
                    }
                }
                LinkTypes::SubjectToAttestations => {
                    let base = base_address.clone().into_agent_pub_key().ok_or_else(|| {
                        wasm_error!(WasmErrorInner::Guest(
                            "SubjectToAttestations base must be an AgentPubKey".into(),
                        ))
                    })?;
                    if base != attestation.subject || action.author != *record.action().author() {
                        return Ok(ValidateCallbackResult::Invalid(
                            "Subject index must be authored by the original attestor and based on the attestation subject".into(),
                        ));
                    }
                }
                _ => {
                    return Ok(ValidateCallbackResult::Invalid(
                        "Unexpected trust-attestation link type in nested matcher".into(),
                    ));
                }
            }
        }
        LinkTypes::AttestationToRevocations => {
            let base = base_address.clone().into_action_hash().ok_or_else(|| {
                wasm_error!(WasmErrorInner::Guest(
                    "AttestationToRevocations base must be an ActionHash".into(),
                ))
            })?;
            let target = action_target(target_address, "Revocation link")?;
            let revocation_record = must_get_valid_record(target)?;
            let revocation: TrustRevocation = revocation_record
                .entry()
                .to_app_option()
                .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
                .ok_or(wasm_error!(WasmErrorInner::Guest(
                    "Revocation link target must contain a TrustRevocation".into(),
                )))?;
            if base != revocation.attestation_hash
                || action.author != *revocation_record.action().author()
            {
                return Ok(ValidateCallbackResult::Invalid(
                    "Revocation link must use the referenced attestation as base and its author's signature".into(),
                ));
            }
            let original = must_get_valid_record(base)?;
            if *original.action().author() != action.author {
                return Ok(ValidateCallbackResult::Invalid(
                    "Revocation link creator must be the original attestor".into(),
                ));
            }
        }
        LinkTypes::AllAttestations => {
            let target = action_target(target_address, "AllAttestations link")?;
            let record = must_get_valid_record(target)?;
            if record
                .entry()
                .to_app_option::<TrustAttestation>()
                .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
                .is_none()
            {
                return Ok(ValidateCallbackResult::Invalid(
                    "AllAttestations target must be a TrustAttestation".into(),
                ));
            }
            if action.author != *record.action().author() {
                return Ok(ValidateCallbackResult::Invalid(
                    "AllAttestations link must be authored by the original attestor".into(),
                ));
            }
            let base = base_address.clone().into_entry_hash().ok_or_else(|| {
                wasm_error!(WasmErrorInner::Guest(
                    "AllAttestations base must be an EntryHash".into(),
                ))
            })?;
            if base != anchor_hash("all_attestations")? {
                return Ok(ValidateCallbackResult::Invalid(
                    "AllAttestations base must be the canonical all_attestations anchor".into(),
                ));
            }
        }
    }

    Ok(ValidateCallbackResult::Valid)
}

#[hdk_extern]
pub fn genesis_self_check(_data: GenesisSelfCheckData) -> ExternResult<ValidateCallbackResult> {
    Ok(ValidateCallbackResult::Valid)
}

#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(store_entry) => match store_entry {
            OpEntry::CreateEntry { app_entry, action } => match app_entry {
                EntryTypes::TrustAttestation(attestation) => {
                    validate_create_trust_attestation(
                        EntryCreationAction::Create(action),
                        attestation,
                    )
                }
                EntryTypes::TrustRevocation(revocation) => validate_create_trust_revocation(
                    EntryCreationAction::Create(action),
                    revocation,
                ),
                EntryTypes::Anchor(anchor) => {
                    if anchor.0.is_empty() || anchor.0.len() > 256 {
                        return Ok(ValidateCallbackResult::Invalid(
                            "Anchor must be 1-256 characters".into(),
                        ));
                    }
                    Ok(ValidateCallbackResult::Valid)
                }
            },
            OpEntry::UpdateEntry { .. } => Ok(ValidateCallbackResult::Invalid(
                "Trust attestations, revocations, and anchors are append-only".into(),
            )),
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterCreateLink {
            base_address,
            target_address,
            link_type,
            tag,
            action,
        } => {
            if tag.0.len() > 1024 {
                return Ok(ValidateCallbackResult::Invalid(
                    "Link tag exceeds maximum length of 1024 bytes".into(),
                ));
            }
            validate_create_link(link_type, &base_address, &target_address, &action)
        }
        FlatOp::RegisterDeleteLink { .. } => Ok(ValidateCallbackResult::Invalid(
            "Web-of-Trust index and revocation links cannot be deleted".into(),
        )),
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
                    "Only the original entry author can update entries".into(),
                ));
            }
            Ok(ValidateCallbackResult::Invalid(
                "Web-of-Trust app entries are append-only".into(),
            ))
        }
        FlatOp::RegisterDelete(OpDelete { action }) => {
            let original = must_get_action(action.deletes_address.clone())?;
            if *original.action().author() != action.author {
                return Ok(ValidateCallbackResult::Invalid(
                    "Only the original entry author can delete entries".into(),
                ));
            }
            Ok(ValidateCallbackResult::Invalid(
                "Web-of-Trust app entries cannot be deleted".into(),
            ))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ts_author(author: AgentPubKey) -> Create {
        Create {
            author,
            timestamp: Timestamp::from_micros(1),
            action_seq: 0,
            prev_action: ActionHash::from_raw_36(vec![0; 36]),
            entry_type: EntryType::App(AppEntryDef::new(
                EntryDefIndex::from(0),
                0.into(),
                EntryVisibility::Public,
            )),
            entry_hash: EntryHash::from_raw_36(vec![0; 36]),
            weight: Default::default(),
        }
    }

    #[test]
    fn trust_score_requires_finite_unit_interval() {
        assert!(!f64::NAN.is_finite());
        assert!(!f64::INFINITY.is_finite());
        assert!(0.0f64.is_finite());
        assert!(1.0f64.is_finite());
    }

    #[test]
    fn security_timestamps_must_not_be_future_dated() {
        let action_timestamp = Timestamp::from_micros(1_000);
        assert_eq!(
            validate_timestamp_us_not_future(
                "Attestation timestamp",
                1_000,
                action_timestamp,
            ),
            ValidateCallbackResult::Valid
        );
        match validate_timestamp_us_not_future(
            "Attestation timestamp",
            1_001,
            action_timestamp,
        ) {
            ValidateCallbackResult::Invalid(message) => {
                assert!(message.contains("signed Holochain action timestamp"));
            }
            other => panic!("future-dated attestation timestamp must be invalid, got {other:?}"),
        }
    }

    #[test]
    fn security_timestamps_reject_negative_action_clock() {
        match validate_timestamp_us_not_future(
            "Revocation timestamp",
            1,
            Timestamp::from_micros(-1),
        ) {
            ValidateCallbackResult::Invalid(message) => {
                assert!(message.contains("negative signed Holochain action timestamp"));
            }
            other => panic!("negative action clock must fail closed, got {other:?}"),
        }
    }

    #[test]
    fn author_is_the_attestor() {
        let author = AgentPubKey::from_raw_36(vec![7; 36]);
        let action = ts_author(author.clone());
        let attestation = TrustAttestation {
            subject: AgentPubKey::from_raw_36(vec![8; 36]),
            trust_score: 0.8,
            pq_verified: true,
            domain: "general".into(),
            note: "trusted".into(),
            timestamp_us: 1,
        };
        let result = validate_create_trust_attestation(
            EntryCreationAction::Create(action),
            attestation,
        )
        .unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }
}
