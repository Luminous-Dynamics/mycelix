// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Append-only verifier-key generation lifecycle integrity for Identity V2.
//!
//! This zome stores lifecycle assertions that are deliberately separate from mutable
//! DID documents. A DID document proves method material; a generation record identifies
//! one lifecycle generation of one canonical verifier key ID and references the exact
//! DID-document action whose material a later observer must independently qualify.
//!
//! Important time boundary: Holochain action timestamps and the declared validity fields
//! are immutable author assertions/provenance. They are **not trusted wall-clock activation
//! evidence**. #337 requires an independent trusted-time activation theorem before a
//! generation may authenticate a historical K-vector verification record.

use hdi::prelude::*;

pub const VERIFIER_DID_MAX_LEN_V2: usize = 256;
pub const VERIFIER_KEY_ID_MAX_LEN_V2: usize = 256;
pub const DISCOVERY_TAG_MAX_LEN_V2: usize = 256;

/// Immutable lifecycle assertion for one verifier-key generation.
///
/// Public key bytes and algorithm are intentionally not duplicated here. They remain in
/// the exact DID document referenced by `did_document_action` and must later pass the
/// existing DID crypto/method theorems before becoming verifier material.
#[hdk_entry_helper]
#[derive(Clone, PartialEq, Debug)]
pub struct VerifierKeyGenerationRecord {
    pub verifier_did: String,
    /// Canonical full DID URL: `{verifier_did}#fragment`.
    pub verifier_key_id: String,
    /// Exact immutable DID-document action that contains the claimed method material.
    /// Integrity validation proves only stable source-chain provenance for this reference;
    /// the later observer bridge must prove exact DID entry type + method material.
    pub did_document_action: ActionHash,
    pub key_generation: u64,
    /// Author-declared semantic validity interval in Unix microseconds.
    /// This is not trusted activation evidence by itself.
    pub valid_from_micros: i64,
    pub valid_until_micros: i64,
    /// Exact preceding generation action for generation > 1.
    pub previous_generation_action: Option<ActionHash>,
}

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    VerifierKeyGeneration(VerifierKeyGenerationRecord),
}

/// Discovery-only link. A link can help find generation actions but never establishes
/// generation authority, completeness, currentness or branch selection.
#[hdk_link_types]
pub enum LinkTypes {
    VerifierKeyGenerationDiscovery,
}

fn bounded_nonempty(value: &str, max_len: usize) -> bool {
    !value.is_empty() && value.len() <= max_len
}

fn valid_verifier_did(value: &str) -> bool {
    bounded_nonempty(value, VERIFIER_DID_MAX_LEN_V2) && value.starts_with("did:mycelix:")
}

fn valid_canonical_key_id(verifier_did: &str, verifier_key_id: &str) -> bool {
    if !bounded_nonempty(verifier_key_id, VERIFIER_KEY_ID_MAX_LEN_V2) {
        return false;
    }
    let prefix = format!("{verifier_did}#");
    verifier_key_id.starts_with(&prefix) && verifier_key_id.len() > prefix.len()
}

/// Pure shape validation shared by host validation and unit tests.
fn validate_generation_shape(record: &VerifierKeyGenerationRecord) -> Result<(), &'static str> {
    if !valid_verifier_did(&record.verifier_did) {
        return Err("verifier DID must be a bounded did:mycelix identifier");
    }
    if !valid_canonical_key_id(&record.verifier_did, &record.verifier_key_id) {
        return Err("verifier key ID must be the canonical full DID URL");
    }
    if record.key_generation == 0 {
        return Err("key generation must be positive");
    }
    if record.valid_until_micros <= record.valid_from_micros {
        return Err("declared key-generation validity interval must be positive");
    }
    match (record.key_generation, &record.previous_generation_action) {
        (1, None) => {}
        (1, Some(_)) => return Err("generation 1 must not name a predecessor"),
        (_, None) => return Err("generation > 1 must name its exact predecessor action"),
        (_, Some(_)) => {}
    }
    Ok(())
}

fn invalid(message: impl Into<String>) -> ExternResult<ValidateCallbackResult> {
    Ok(ValidateCallbackResult::Invalid(message.into()))
}

/// Validate a new immutable generation assertion.
///
/// The direct predecessor checks are intentionally inductive/local. Concurrent competing
/// children can each be locally valid, so complete observation + branch rejection remains
/// the job of #333 and the future observation bridge rather than a timestamp winner rule.
fn validate_create_generation(
    action: Create,
    record: VerifierKeyGenerationRecord,
) -> ExternResult<ValidateCallbackResult> {
    if let Err(reason) = validate_generation_shape(&record) {
        return invalid(reason);
    }

    let expected_did = format!("did:mycelix:{}", action.author);
    if record.verifier_did != expected_did {
        return invalid("verifier generation DID must equal the committing agent DID");
    }

    // This comparison is only source-chain provenance consistency. Holochain timestamps
    // are author-controlled and MUST NOT be interpreted as trusted historical activation.
    if action.timestamp.as_micros() > record.valid_from_micros {
        return invalid("generation action timestamp may not follow declared valid_from");
    }

    // The referenced DID-material action must already exist on the same author's chain and
    // precede this generation assertion. Exact DID entry type and key material are verified
    // later by the #272/#294 observer bridge; this zome deliberately does not duplicate it.
    let did_record = must_get_valid_record(record.did_document_action.clone())?;
    if did_record.action().author() != &action.author {
        return invalid("referenced DID-material action must have the same author");
    }
    if did_record.action().action_seq() >= action.action_seq {
        return invalid("referenced DID-material action must precede the generation action");
    }
    if did_record.action().timestamp() > action.timestamp {
        return invalid("referenced DID-material action timestamp may not follow generation action");
    }
    if did_record.action().entry_type().is_none() {
        return invalid("referenced DID-material action must create an application entry");
    }
    if did_record.action().entry_type() == Some(&action.entry_type) {
        return invalid("generation record cannot use another generation entry as DID material");
    }

    if let Some(previous_hash) = record.previous_generation_action.clone() {
        let previous_record = must_get_valid_record(previous_hash)?;

        // Prevent type confusion: predecessor bytes alone are insufficient. Its action must
        // carry the exact same application entry type as this generation entry.
        if previous_record.action().entry_type() != Some(&action.entry_type) {
            return invalid("predecessor action is not a verifier-key generation entry");
        }
        let Action::Create(previous_action) = previous_record.action() else {
            return invalid("verifier-key generation predecessor must be a create action");
        };
        if previous_action.author != action.author {
            return invalid("verifier-key generation predecessor must have the same author");
        }
        if previous_action.action_seq >= action.action_seq {
            return invalid("verifier-key generation predecessor must precede the child action");
        }
        if previous_action.timestamp > action.timestamp {
            return invalid("generation action provenance timestamp may not regress");
        }

        let previous: VerifierKeyGenerationRecord = previous_record
            .entry()
            .to_app_option()
            .map_err(|error| wasm_error!(WasmErrorInner::Guest(error.to_string())))?
            .ok_or(wasm_error!(WasmErrorInner::Guest(
                "verifier-key generation predecessor entry missing".into()
            )))?;

        if previous.verifier_did != record.verifier_did
            || previous.verifier_key_id != record.verifier_key_id
        {
            return invalid("generation predecessor belongs to a different DID/key namespace");
        }
        let expected_generation = previous
            .key_generation
            .checked_add(1)
            .ok_or(wasm_error!(WasmErrorInner::Guest(
                "verifier-key generation counter overflow".into()
            )))?;
        if record.key_generation != expected_generation {
            return invalid("key generation must increment its direct predecessor by exactly one");
        }
        if record.valid_from_micros < previous.valid_until_micros {
            return invalid("successive verifier-key generation validity intervals may not overlap");
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
                EntryTypes::VerifierKeyGeneration(record) => {
                    validate_create_generation(action, record)
                }
            },
            OpEntry::UpdateEntry { .. } => {
                invalid("verifier-key generation records are append-only and cannot be updated")
            }
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterUpdate(_) => {
            invalid("verifier-key generation records are append-only and cannot be updated")
        }
        FlatOp::RegisterDelete(_) => {
            invalid("verifier-key generation records are append-only and cannot be deleted")
        }
        FlatOp::RegisterCreateLink { tag, .. } => {
            if tag.0.len() > DISCOVERY_TAG_MAX_LEN_V2 {
                return invalid("verifier-key generation discovery tag exceeds size bound");
            }
            Ok(ValidateCallbackResult::Valid)
        }
        FlatOp::RegisterDeleteLink { .. } => {
            invalid("verifier-key generation discovery links are append-only")
        }
        FlatOp::StoreRecord(_) | FlatOp::RegisterAgentActivity(_) => {
            Ok(ValidateCallbackResult::Valid)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn hash(byte: u8) -> ActionHash {
        ActionHash::from_raw_36(vec![byte; 36])
    }

    fn generation(number: u64, previous: Option<ActionHash>) -> VerifierKeyGenerationRecord {
        VerifierKeyGenerationRecord {
            verifier_did: "did:mycelix:uhCAkVERIFIER".into(),
            verifier_key_id: "did:mycelix:uhCAkVERIFIER#key-1".into(),
            did_document_action: hash(0x11),
            key_generation: number,
            valid_from_micros: 1_000_000,
            valid_until_micros: 2_000_000,
            previous_generation_action: previous,
        }
    }

    #[test]
    fn canonical_namespace_is_required() {
        let good = generation(1, None);
        assert_eq!(validate_generation_shape(&good), Ok(()));

        let fragment = VerifierKeyGenerationRecord {
            verifier_key_id: "#key-1".into(),
            ..good.clone()
        };
        assert_eq!(
            validate_generation_shape(&fragment),
            Err("verifier key ID must be the canonical full DID URL")
        );

        let foreign = VerifierKeyGenerationRecord {
            verifier_key_id: "did:mycelix:other#key-1".into(),
            ..good
        };
        assert_eq!(
            validate_generation_shape(&foreign),
            Err("verifier key ID must be the canonical full DID URL")
        );
    }

    #[test]
    fn root_and_successor_predecessor_shape_is_closed() {
        assert_eq!(validate_generation_shape(&generation(1, None)), Ok(()));
        assert_eq!(
            validate_generation_shape(&generation(1, Some(hash(0x22)))),
            Err("generation 1 must not name a predecessor")
        );
        assert_eq!(
            validate_generation_shape(&generation(2, None)),
            Err("generation > 1 must name its exact predecessor action")
        );
        assert_eq!(
            validate_generation_shape(&generation(2, Some(hash(0x22)))),
            Ok(())
        );
    }

    #[test]
    fn zero_generation_and_non_positive_validity_fail_closed() {
        assert_eq!(
            validate_generation_shape(&generation(0, None)),
            Err("key generation must be positive")
        );
        let invalid = VerifierKeyGenerationRecord {
            valid_until_micros: 1_000_000,
            ..generation(1, None)
        };
        assert_eq!(
            validate_generation_shape(&invalid),
            Err("declared key-generation validity interval must be positive")
        );
    }

    #[test]
    fn lifecycle_entry_does_not_duplicate_public_key_or_algorithm() {
        let source = include_str!("lib.rs");
        let entry_start = source.index("pub struct VerifierKeyGenerationRecord").unwrap();
        let entry_end = source[entry_start..].index("\n}").unwrap() + entry_start;
        let entry = &source[entry_start..=entry_end];
        assert!(!entry.contains("public_key"));
        assert!(!entry.contains("algorithm"));
        assert!(entry.contains("did_document_action"));
    }

    #[test]
    fn trusted_activation_is_not_claimed_by_integrity_entry() {
        let source = include_str!("lib.rs");
        let theorem = source[..source.index("#[cfg(test)]").unwrap()].to_lowercase();
        assert!(!theorem.contains("qualifiedhistoricalverifierkey"));
        assert!(!theorem.contains("signature_validated"));
        assert!(!theorem.contains("policy_current"));
    }
}
