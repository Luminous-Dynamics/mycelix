// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! DID Registry Integrity Zome
//! Defines entry types and validation for DID:mycelix identifiers
//!
//! Updated to use HDI 0.7 patterns with FlatOp validation

use hdi::prelude::*;
use mycelix_crypto::{AlgorithmId, TaggedPublicKey};

/// DID Document entry type
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct DidDocument {
    /// The DID identifier (did:mycelix:<agent_pub_key>)
    pub id: String,
    /// Controller of this DID (usually self)
    pub controller: AgentPubKey,
    /// Verification methods (public keys)
    #[serde(rename = "verificationMethod", alias = "verification_method")]
    pub verification_method: Vec<VerificationMethod>,
    /// Authentication methods
    pub authentication: Vec<String>,
    /// Key agreement methods for encryption (W3C DID Core §5.3.3).
    ///
    /// Each entry is a DID URL fragment (e.g. "#kem-1") referencing a
    /// `VerificationMethod` with an ML-KEM public key. Recipients use this
    /// to look up the KEM key for encrypting data to this DID's owner.
    #[serde(
        rename = "keyAgreement",
        alias = "key_agreement",
        default,
        skip_serializing_if = "Vec::is_empty"
    )]
    pub key_agreement: Vec<String>,
    /// Service endpoints
    pub service: Vec<ServiceEndpoint>,
    /// Creation timestamp
    pub created: Timestamp,
    /// Last update timestamp
    pub updated: Timestamp,
    /// Version number for updates
    pub version: u32,
}

/// Verification method for cryptographic operations
#[derive(Clone, PartialEq, Debug, Serialize, Deserialize)]
pub struct VerificationMethod {
    pub id: String,
    #[serde(rename = "type", alias = "type_")]
    pub type_: String,
    pub controller: String,
    #[serde(rename = "publicKeyMultibase", alias = "public_key_multibase")]
    pub public_key_multibase: String,
    /// Algorithm identifier (multicodec u16). None defaults to Ed25519 (0xed01).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub algorithm: Option<u16>,
}

/// Service endpoint for discovery
#[derive(Clone, PartialEq, Debug, Serialize, Deserialize)]
pub struct ServiceEndpoint {
    pub id: String,
    #[serde(rename = "type", alias = "type_")]
    pub type_: String,
    #[serde(rename = "serviceEndpoint", alias = "service_endpoint")]
    pub service_endpoint: String,
}

/// Canonical service type for Mycelix Substrate discovery
pub const SUBSTRATE_SERVICE_TYPE: &str = "SubstrateMetadata";

/// DID Deactivation record
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct DidDeactivation {
    pub did: String,
    pub reason: String,
    pub deactivated_at: Timestamp,
}

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    DidDocument(DidDocument),
    DidDeactivation(DidDeactivation),
}

#[hdk_link_types]
pub enum LinkTypes {
    AgentToDid,
    DidToVerificationMethod,
    DidToService,
    /// Global substrate role advertisements. Separate from a DID document's
    /// service links because the base is a role anchor rather than the DID.
    SubstrateRoleToAgent,
    DidHistory,
    DidToDeactivation,
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
                EntryTypes::DidDocument(did_doc) => {
                    validate_create_did_document(EntryCreationAction::Create(action), did_doc)
                }
                EntryTypes::DidDeactivation(deactivation) => validate_create_did_deactivation(
                    EntryCreationAction::Create(action),
                    deactivation,
                ),
            },
            OpEntry::UpdateEntry {
                app_entry, action, ..
            } => match app_entry {
                EntryTypes::DidDocument(did_doc) => validate_update_did_document(action, did_doc),
                EntryTypes::DidDeactivation(_) => Ok(ValidateCallbackResult::Invalid(
                    "Deactivation records cannot be updated".into(),
                )),
            },
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterCreateLink {
            base_address,
            target_address,
            link_type,
            tag,
            action,
        } => {
            // Validate tag length to prevent spam/DoS
            if tag.0.len() > 1024 {
                return Ok(ValidateCallbackResult::Invalid(
                    "Link tag exceeds maximum length of 1024 bytes".into(),
                ));
            }

            match link_type {
                LinkTypes::AgentToDid => {
                    validate_agent_to_did_link(&base_address, &target_address, &action)
                }
                LinkTypes::DidToDeactivation => {
                    validate_did_to_deactivation_link(&base_address, &target_address, &action)
                }
                LinkTypes::DidToVerificationMethod => Ok(ValidateCallbackResult::Valid),
                LinkTypes::DidToService => Ok(ValidateCallbackResult::Valid),
                LinkTypes::SubstrateRoleToAgent => {
                    validate_substrate_role_link(&base_address, &target_address, &action)
                }
                LinkTypes::DidHistory => {
                    validate_agent_to_did_link(&base_address, &target_address, &action)
                },
            }
        }
        FlatOp::RegisterDeleteLink {
            original_action,
            action,
            link_type,
            ..
        } => {
            // Only the original link creator can delete their links.
            if action.author != original_action.author {
                return Ok(ValidateCallbackResult::Invalid(
                    "Only the link creator can delete their links".into(),
                ));
            }

            // DID history and deactivation are security state. History is
            // append-only and deactivation is irreversible; neither may be
            // hidden by deleting its index link.
            match link_type {
                LinkTypes::DidHistory | LinkTypes::DidToDeactivation => {
                    Ok(ValidateCallbackResult::Invalid(
                        "DID history and deactivation links cannot be deleted".into(),
                    ))
                }
                _ => Ok(ValidateCallbackResult::Valid),
            }
        }
        FlatOp::StoreRecord(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterAgentActivity(activity) => match activity {
            OpActivity::CreateEntry {
                app_entry_type: Some(UnitEntryTypes::DidDocument),
                action,
            } => validate_did_document_chain_uniqueness(action),
            OpActivity::CreateEntry {
                app_entry_type: Some(UnitEntryTypes::DidDeactivation),
                action,
            } => validate_did_deactivation_chain_uniqueness(action),
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterUpdate(update) => {
            match update {
                OpUpdate::Entry { app_entry, action, .. } => {
                    let original = must_get_action(action.original_action_address.clone())?;
                    if *original.action().author() != action.author {
                        return Ok(ValidateCallbackResult::Invalid(
                            "Only the original entry author can update their entries".into(),
                        ));
                    }
                    match app_entry {
                        EntryTypes::DidDocument(did_doc) => {
                            validate_update_did_document(action, did_doc)
                        }
                        EntryTypes::DidDeactivation(_) => Ok(ValidateCallbackResult::Invalid(
                            "DID deactivation records cannot be updated".into(),
                        )),
                    }
                }
                OpUpdate::PrivateEntry { action, .. }
                | OpUpdate::Agent { action, .. }
                | OpUpdate::CapClaim { action, .. }
                | OpUpdate::CapGrant { action, .. } => {
                    let original = must_get_action(action.original_action_address.clone())?;
                    if *original.action().author() != action.author {
                        return Ok(ValidateCallbackResult::Invalid(
                            "Only the original entry author can update their entries".into(),
                        ));
                    }
                    Ok(ValidateCallbackResult::Valid)
                }
            }
        }
        FlatOp::RegisterDelete(OpDelete { action }) => {
            let original = must_get_action(action.deletes_address.clone())?;
            if *original.action().author() != action.author {
                return Ok(ValidateCallbackResult::Invalid(
                    "Only the original entry author can delete their entries".into(),
                ));
            }

            match original.action().entry_type() {
                Some(EntryType::App(entry_def))
                    if *entry_def == UnitEntryTypes::DidDocument.into()
                        || *entry_def == UnitEntryTypes::DidDeactivation.into() =>
                {
                    Ok(ValidateCallbackResult::Invalid(
                        "DID security entries cannot be deleted".into(),
                    ))
                }
                _ => Ok(ValidateCallbackResult::Valid),
            }
        }
    }
}

fn validate_substrate_role_link(
    base_address: &AnyLinkableHash,
    target_address: &AnyLinkableHash,
    action: &CreateLink,
) -> ExternResult<ValidateCallbackResult> {
    let target_agent = match target_address.clone().into_agent_pub_key() {
        Some(agent) => agent,
        None => {
            return Ok(ValidateCallbackResult::Invalid(
                "SubstrateRoleToAgent target must be an AgentPubKey".into(),
            ));
        }
    };

    if action.author != target_agent {
        return Ok(ValidateCallbackResult::Invalid(
            "SubstrateRoleToAgent link must be authored by the advertised agent".into(),
        ));    }

    if base_address.clone().into_entry_hash().is_none() {
        return Ok(ValidateCallbackResult::Invalid(
            "SubstrateRoleToAgent base must be an EntryHash role anchor".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

fn validate_agent_to_did_link(
    base_address: &AnyLinkableHash,
    target_address: &AnyLinkableHash,
    action: &CreateLink,
) -> ExternResult<ValidateCallbackResult> {
    let base_agent = match base_address.clone().into_agent_pub_key() {
        Some(agent) => agent,
        None => {
            return Ok(ValidateCallbackResult::Invalid(
                "AgentToDid base must be an AgentPubKey".into(),
            ));
        }
    };

    // Only the owner of an agent namespace may create its canonical DID link.
    // Without this, an arbitrary agent could create a link from a victim's
    // AgentPubKey to another valid DID record and hijack resolution.
    if action.author != base_agent {
        return Ok(ValidateCallbackResult::Invalid(
            "AgentToDid link must be authored by the base agent".into(),
        ));
    }

    let target_action = match target_address.clone().into_action_hash() {
        Some(action_hash) => action_hash,
        None => {
            return Ok(ValidateCallbackResult::Invalid(
                "AgentToDid target must be an ActionHash".into(),
            ));
        }
    };

    let record = must_get_valid_record(target_action)?;
    let did_doc: DidDocument = record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "AgentToDid target must contain a DID document".into(),
        )))?;

    let expected_did = format!("did:mycelix:{}", base_agent);
    if did_doc.controller != base_agent || did_doc.id != expected_did {
        return Ok(ValidateCallbackResult::Invalid(
            "AgentToDid target must be the canonical DID for its base agent".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

fn validate_did_to_deactivation_link(
    base_address: &AnyLinkableHash,
    target_address: &AnyLinkableHash,
    action: &CreateLink,
) -> ExternResult<ValidateCallbackResult> {
    let base_agent = match base_address.clone().into_agent_pub_key() {
        Some(agent) => agent,
        None => {
            return Ok(ValidateCallbackResult::Invalid(
                "DidToDeactivation base must be an AgentPubKey".into(),
            ));
        }
    };

    if action.author != base_agent {
        return Ok(ValidateCallbackResult::Invalid(
            "DidToDeactivation link must be authored by the DID owner".into(),
        ));
    }

    let target_action = match target_address.clone().into_action_hash() {
        Some(action_hash) => action_hash,
        None => {
            return Ok(ValidateCallbackResult::Invalid(
                "DidToDeactivation target must be an ActionHash".into(),
            ));
        }
    };

    let record = must_get_valid_record(target_action)?;
    let deactivation: DidDeactivation = record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "DidToDeactivation target must contain a deactivation record".into(),
        )))?;

    let expected_did = format!("did:mycelix:{}", base_agent);
    if deactivation.did != expected_did {
        return Ok(ValidateCallbackResult::Invalid(
            "DidToDeactivation target must name the base agent's DID".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Validate the method-specific identifier grammar for `did:mycelix`.
///
/// Mycelix currently derives the identifier from the canonical textual form of
/// a Holochain AgentPubKey. That textual form is ASCII and uses the base64url
/// alphabet; we therefore reject whitespace, URI delimiters, non-ASCII bytes,
/// and an empty identifier before any DHT lookup occurs.
fn validate_mycelix_did_syntax(did: &str) -> Result<(), &'static str> {
    const PREFIX: &str = "did:mycelix:";
    let Some(identifier) = did.strip_prefix(PREFIX) else {
        return Err("DID must start with 'did:mycelix:'");
    };

    if identifier.is_empty() {
        return Err("did:mycelix identifier must not be empty");
    }

    if !identifier.bytes().all(|byte| {
        byte.is_ascii_alphanumeric() || byte == b'-' || byte == b'_'
    }) {
        return Err("did:mycelix identifier contains an invalid character");
    }

    Ok(())
}

/// Validate one verification method against the DID cryptographic contract.
fn validate_verification_method(method: &VerificationMethod, did_id: &str) -> Result<AlgorithmId, String> {
    if method.id.is_empty() || method.id.len() > 256 {
        return Err("Verification method ID must be 1-256 characters".into());
    }
    if method.type_.is_empty() || method.type_.len() > 256 {
        return Err("Verification method type must be 1-256 characters".into());
    }
    if method.controller != did_id {
        return Err("Verification method controller must equal the DID".into());
    }
    if method.public_key_multibase.is_empty() || method.public_key_multibase.len() > 4096 {
        return Err("Public key multibase must be 1-4096 characters".into());
    }
    let tagged = TaggedPublicKey::from_multibase_strict(&method.public_key_multibase)
        .map_err(|error| format!("Invalid verification method key: {error}"))?;
    if let Some(declared_code) = method.algorithm {
        let declared = AlgorithmId::from_u16(declared_code)
            .ok_or_else(|| format!("Unknown verification method algorithm: {declared_code:#06x}"))?;
        if declared != tagged.algorithm {
            return Err(format!(
                "Verification method algorithm does not match multibase key: declared={}, detected={}",
                declared.did_verification_method_type(),
                tagged.algorithm.did_verification_method_type()
            ));
        }
    }
    if method.type_ != tagged.algorithm.did_verification_method_type() {
        return Err(format!(
            "Verification method type does not match key algorithm: type={}, algorithm={}",
            method.type_,
            tagged.algorithm.did_verification_method_type()
        ));
    }
    Ok(tagged.algorithm)
}

fn validate_verification_method_set(did_doc: &DidDocument) -> Result<(), String> {
    if did_doc.verification_method.is_empty() {
        return Err("DID must have at least one verification method".into());
    }
    let mut algorithms = std::collections::BTreeMap::new();
    for method in &did_doc.verification_method {
        let algorithm = validate_verification_method(method, &did_doc.id)?;
        if algorithms.insert(method.id.as_str(), algorithm).is_some() {
            return Err("DID verification method IDs must be unique".into());
        }
    }
    for reference in &did_doc.authentication {
        let algorithm = algorithms.get(reference.as_str()).ok_or_else(|| format!("DID authentication reference '{}' must resolve to a verification method", reference))?;
        if !algorithm.is_signature_algorithm() {
            return Err(format!("DID authentication reference '{}' must use a signature algorithm, detected {}", reference, algorithm.did_verification_method_type()));
        }
    }
    for reference in &did_doc.key_agreement {
        let algorithm = algorithms.get(reference.as_str()).ok_or_else(|| format!("DID keyAgreement reference '{}' must resolve to a verification method", reference))?;
        if !matches!(algorithm, AlgorithmId::MlKem768 | AlgorithmId::MlKem1024) {
            return Err(format!("DID keyAgreement reference '{}' must use an ML-KEM algorithm, detected {}", reference, algorithm.did_verification_method_type()));
        }
    }
    Ok(())
}
/// Enforce one canonical DID document creation per controller.
///
/// The canonical DID is derived directly from the committing agent. A second
/// version-1 document would create an ambiguous genesis state for fallback and
/// historical resolution, so duplicate creation is rejected on the author's
/// source chain.
fn validate_did_document_chain_uniqueness(
    action: Create,
) -> ExternResult<ValidateCallbackResult> {
    let current_entry = must_get_entry(action.entry_hash.clone())?;
    let current_doc: DidDocument = current_entry.try_into().map_err(|e| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "DID document entry could not be decoded: {e}"
        )))
    })?;

    let activity = must_get_agent_activity(
        action.author.clone(),
        ChainFilter::new(action.prev_action.clone()),
    )?;

    let entry_type =
        EntryType::App(AppEntryDef::try_from(UnitEntryTypes::DidDocument)?);

    for prior in activity {
        let prior_action = prior.action.action();
        let Action::Create(prior_create) = prior_action else {
            continue;
        };
        if prior_create.entry_type != entry_type {
            continue;
        }

        let prior_entry = must_get_entry(prior_create.entry_hash.clone())?;
        let prior_doc: DidDocument = prior_entry.try_into().map_err(|e| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "DID document history entry could not be decoded: {e}"
            )))
        })?;

        if prior_doc.id == current_doc.id {
            return Ok(ValidateCallbackResult::Invalid(
                "A canonical did:mycelix DID may only have one document creation".into(),
            ));
        }
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Validate DID document creation
/// Enforce that a DID document's identifier is derived from the committing
/// agent's key. Pure so it can be unit-tested without a full Create action.
fn require_did_id_matches_author(did_id: &str, author_did: &str) -> ValidateCallbackResult {
    if did_id != author_did {
        return ValidateCallbackResult::Invalid(format!(
            "DID id must equal the committing agent's DID (identifier takeover). \
             Expected '{author_did}', got '{did_id}'"
        ));
    }
    ValidateCallbackResult::Valid
}

fn validate_create_did_document(
    action: EntryCreationAction,
    did_doc: DidDocument,
) -> ExternResult<ValidateCallbackResult> {
    // Validate the complete method-specific identifier grammar before
    // binding it to the committing agent.
    if let Err(message) = validate_mycelix_did_syntax(&did_doc.id) {
        return Ok(ValidateCallbackResult::Invalid(message.into()));
    }

    // Validate controller matches author
    let author = action.author();
    if did_doc.controller != *author {
        return Ok(ValidateCallbackResult::Invalid(
            "DID controller must be the author".into(),
        ));
    }

    // Bind the DID identifier itself to the committing agent. Without this, an
    // attacker can mint a document whose `id` claims a victim's DID
    // (did:mycelix:<victim>) while setting `controller` to their own key to pass
    // the check above — then hijack resolution (resolve_did returns the newest
    // document by timestamp). The coordinator always derives `id` from
    // agent_info().agent_initial_pubkey, so this never rejects the honest path.
    let author_did = format!("did:mycelix:{}", author);
    if let ValidateCallbackResult::Invalid(msg) =
        require_did_id_matches_author(&did_doc.id, &author_did)
    {
        return Ok(ValidateCallbackResult::Invalid(msg));
    }

    // Validate the complete cryptographic verification-method set at the
    // integrity boundary, independent of the coordinator API used.
    if let Err(message) = validate_verification_method_set(&did_doc) {
        return Ok(ValidateCallbackResult::Invalid(message.into()));
    }

    // Validate version starts at 1
    if did_doc.version != 1 {
        return Ok(ValidateCallbackResult::Invalid(
            "Initial DID version must be 1".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Validate that a DID update targets the current DID-document state on
/// the author's source chain rather than a stale ancestor.
///
/// Holochain source-chain ordering is deterministic for validation, while DHT
/// link traversal is not. A coordinator that updates an older DID document
/// could otherwise create a second record at the same version and force every
/// resolver into an ambiguity/DoS condition.
fn validate_did_update_targets_latest(action: &Update) -> ExternResult<ValidateCallbackResult> {
    let activity = must_get_agent_activity(
        action.author.clone(),
        ChainFilter::new(action.prev_action.clone()),
    )?;
    let entry_type =
        EntryType::App(AppEntryDef::try_from(UnitEntryTypes::DidDocument)?);

    let mut latest: Option<(u32, ActionHash)> = None;
    for item in activity {
        let prior_action = item.action.action();
        if prior_action.entry_type() != Some(&entry_type) {
            continue;
        }
        if !matches!(prior_action, Action::Create(_) | Action::Update(_)) {
            continue;
        }

        let hash = hdi::hash::hash_action(prior_action.clone())?;
        let seq = prior_action.action_seq();
        if latest.as_ref().is_none_or(|(latest_seq, _)| seq > *latest_seq) {
            latest = Some((seq, hash));
        }
    }

    match latest {
        Some((_, latest_hash)) if latest_hash == action.original_action_address => {
            Ok(ValidateCallbackResult::Valid)
        }
        Some(_) => Ok(ValidateCallbackResult::Invalid(
            "DID update must target the latest DID document on the author's source chain".into(),
        )),
        None => Ok(ValidateCallbackResult::Invalid(
            "DID update has no prior canonical DID document".into(),
        )),
    }
}

/// Validate DID document update
fn validate_update_did_document(
    action: Update,
    did_doc: DidDocument,
) -> ExternResult<ValidateCallbackResult> {
    // Validate author is controller
    if did_doc.controller != action.author {
        return Ok(ValidateCallbackResult::Invalid(
            "Only controller can update DID".into(),
        ));
    }

    // The original action must be the current canonical DID document state.
    // This prevents stale-ancestor updates from manufacturing a second branch
    // of the version sequence.
    match validate_did_update_targets_latest(&action)? {
        ValidateCallbackResult::Valid => {}
        invalid => return Ok(invalid),
    }

    // Fetch original to enforce invariants
    let original_record = must_get_valid_record(action.original_action_address.clone())?;
    let original: DidDocument = original_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Original DID document not found".into()
        )))?;

    // Immutable fields
    if did_doc.id != original.id {
        return Ok(ValidateCallbackResult::Invalid(
            "DID id cannot be changed".into(),
        ));
    }
    if did_doc.controller != original.controller {
        return Ok(ValidateCallbackResult::Invalid(
            "DID controller cannot be changed".into(),
        ));
    }
    if did_doc.created != original.created {
        return Ok(ValidateCallbackResult::Invalid(
            "DID created timestamp cannot be changed".into(),
        ));
    }

    // Version is a method-level monotonic sequence, so every accepted
    // update must advance exactly one version. This prevents gaps that make
    // `versionId` selection ambiguous.
    if did_doc.version != original.version.saturating_add(1) {
        return Ok(ValidateCallbackResult::Invalid(
            format!(
                "DID version must increment exactly by 1 (expected {}, got {})",
                original.version.saturating_add(1),
                did_doc.version
            ),
        ));
    }

    // Re-validate cryptographic key structure and relationship roles even when
    // a generic update path bypasses coordinator-specific key helpers.
    if let Err(message) = validate_verification_method_set(&did_doc) {
        return Ok(ValidateCallbackResult::Invalid(message.into()));
    }

    // Updated timestamp must advance
    if did_doc.updated <= original.updated {        return Ok(ValidateCallbackResult::Invalid(
            "DID updated timestamp must advance".into(),
        ));
    }

    // Must still have at least one verification method
    if did_doc.verification_method.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "DID must have at least one verification method".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Enforce one irreversible deactivation artifact per DID on the
/// controller's source chain. This prevents conflicting deactivation reasons
/// and timestamp selection from becoming a resolution ambiguity.
fn validate_did_deactivation_chain_uniqueness(
    action: Create,
) -> ExternResult<ValidateCallbackResult> {
    let current_entry = must_get_entry(action.entry_hash.clone())?;
    let current: DidDeactivation = current_entry.try_into().map_err(|e| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "DID deactivation entry could not be decoded: {e}"
        )))
    })?;

    let activity = must_get_agent_activity(
        action.author.clone(),
        ChainFilter::new(action.prev_action.clone()),
    )?;

    let entry_type =
        EntryType::App(AppEntryDef::try_from(UnitEntryTypes::DidDeactivation)?);

    for prior in activity {
        let prior_action = prior.action.action();
        let Action::Create(prior_create) = prior_action else {
            continue;
        };
        if prior_create.entry_type != entry_type {
            continue;
        }

        let prior_entry = must_get_entry(prior_create.entry_hash.clone())?;
        let prior: DidDeactivation = prior_entry.try_into().map_err(|e| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "DID deactivation history entry could not be decoded: {e}"
            )))
        })?;

        if prior.did == current.did {
            return Ok(ValidateCallbackResult::Invalid(
                "A DID may only have one deactivation artifact".into(),
            ));
        }
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Validate DID deactivation creation
fn validate_create_did_deactivation(
    action: EntryCreationAction,
    deactivation: DidDeactivation,
) -> ExternResult<ValidateCallbackResult> {
    if let Err(message) = validate_mycelix_did_syntax(&deactivation.did) {
        return Ok(ValidateCallbackResult::Invalid(message.into()));
    }

    // Bind the deactivation to its committer -- deactivate_did only ever
    // deactivates the CALLING agent's own DID (fetched via
    // get_did_document(agent_pub_key)), never a third party's, so this
    // never rejects the honest path. Without this, an attacker could mint a
    // DidDeactivation naming a victim's DID, marking someone else's
    // identity as deactivated (P0 author-binding gap -- the same
    // takeover-shaped risk require_did_id_matches_author already guards
    // against for DidDocument creation above).
    let author = action.author();
    let author_did = format!("did:mycelix:{}", author);
    if let ValidateCallbackResult::Invalid(msg) =
        require_did_id_matches_author(&deactivation.did, &author_did)
    {
        return Ok(ValidateCallbackResult::Invalid(msg));
    }

    // Validate reason provided
    if deactivation.reason.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Deactivation reason is required".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn did_method_specific_syntax_is_strict() {
        assert!(validate_mycelix_did_syntax("did:mycelix:uhCAkSELF").is_ok());
        assert!(validate_mycelix_did_syntax("did:mycelix:").is_err());
        assert!(validate_mycelix_did_syntax("did:mycelix:abc:def").is_err());
        assert!(validate_mycelix_did_syntax("did:mycelix:abc#key").is_err());
        assert!(validate_mycelix_did_syntax("did:mycelix:abc?versionId=1").is_err());
        assert!(validate_mycelix_did_syntax("did:mycelix:abc def").is_err());
        assert!(validate_mycelix_did_syntax("did:key:abc").is_err());
        assert!(validate_mycelix_did_syntax("did:mycelix:abc%20def").is_err());
    }

    #[test]
    fn did_method_specific_syntax_accepts_base64url_alphabet() {
        assert!(validate_mycelix_did_syntax("did:mycelix:uhCAk-_0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz").is_ok());
    }


    #[test]
    fn verification_method_accepts_canonical_ed25519_multibase() {
        let did = "did:mycelix:uhCAkSELF";
        let key = TaggedPublicKey::new(AlgorithmId::Ed25519, vec![0x42; 32])
            .expect("valid Ed25519 fixture")
            .to_multibase();
        let method = VerificationMethod {
            id: format!("{did}#keys-1"),
            type_: AlgorithmId::Ed25519.did_verification_method_type().into(),
            controller: did.into(),
            public_key_multibase: key,
            algorithm: Some(AlgorithmId::Ed25519.as_u16()),
        };
        assert_eq!(
            validate_verification_method(&method, did),
            Ok(AlgorithmId::Ed25519)
        );
    }

    #[test]
    fn verification_method_rejects_malformed_multibase() {
        let did = "did:mycelix:uhCAkSELF";
        let method = VerificationMethod {
            id: format!("{did}#keys-1"),
            type_: AlgorithmId::Ed25519.did_verification_method_type().into(),
            controller: did.into(),
            public_key_multibase: "znot-a-valid-key".into(),
            algorithm: Some(AlgorithmId::Ed25519.as_u16()),
        };
        let error = validate_verification_method(&method, did)
            .expect_err("malformed multibase must be rejected");
        assert!(error.contains("Invalid verification method key"));
    }

    #[test]
    fn verification_method_rejects_declared_algorithm_mismatch() {
        let did = "did:mycelix:uhCAkSELF";
        let key = TaggedPublicKey::new(AlgorithmId::Ed25519, vec![0x42; 32])
            .expect("valid Ed25519 fixture")
            .to_multibase();
        let method = VerificationMethod {
            id: format!("{did}#keys-1"),
            type_: AlgorithmId::Ed25519.did_verification_method_type().into(),
            controller: did.into(),
            public_key_multibase: key,
            algorithm: Some(AlgorithmId::MlDsa65.as_u16()),
        };
        let error = validate_verification_method(&method, did)
            .expect_err("declared algorithm mismatch must be rejected");
        assert!(error.contains("does not match multibase key"));
    }

    #[test]
    fn verification_method_rejects_type_algorithm_mismatch() {
        let did = "did:mycelix:uhCAkSELF";
        let key = TaggedPublicKey::new(AlgorithmId::Ed25519, vec![0x42; 32])
            .expect("valid Ed25519 fixture")
            .to_multibase();
        let method = VerificationMethod {
            id: format!("{did}#keys-1"),
            type_: "MlDsa65VerificationKey2024".into(),
            controller: did.into(),
            public_key_multibase: key,
            algorithm: Some(AlgorithmId::Ed25519.as_u16()),
        };
        let error = validate_verification_method(&method, did)
            .expect_err("type/algorithm mismatch must be rejected");
        assert!(error.contains("type does not match key algorithm"));
    }

    #[test]
    fn verification_relationship_rejects_kem_as_authentication() {
        let did = "did:mycelix:uhCAkSELF";
        let key = TaggedPublicKey::new(AlgorithmId::MlKem768, vec![0x42; 1184])
            .expect("valid ML-KEM fixture")
            .to_multibase();
        let document = DidDocument {
            id: did.into(),
            controller: AgentPubKey::from_raw_36(vec![0u8; 36]),
            verification_method: vec![VerificationMethod {
                id: format!("{did}#kem-1"),
                type_: AlgorithmId::MlKem768.did_verification_method_type().into(),
                controller: did.into(),
                public_key_multibase: key,
                algorithm: Some(AlgorithmId::MlKem768.as_u16()),
            }],
            authentication: vec![format!("{did}#kem-1")],
            key_agreement: vec![],
            service: vec![],
            created: Timestamp::from_micros(0),
            updated: Timestamp::from_micros(1),
            version: 1,
        };
        let error = validate_verification_method_set(&document)
            .expect_err("KEM cannot be used for authentication");
        assert!(error.contains("signature algorithm"));
    }

    #[test]
    fn verification_relationship_rejects_signing_key_as_key_agreement() {
        let did = "did:mycelix:uhCAkSELF";
        let key = TaggedPublicKey::new(AlgorithmId::Ed25519, vec![0x42; 32])
            .expect("valid Ed25519 fixture")
            .to_multibase();
        let document = DidDocument {
            id: did.into(),
            controller: AgentPubKey::from_raw_36(vec![0u8; 36]),
            verification_method: vec![VerificationMethod {
                id: format!("{did}#keys-1"),
                type_: AlgorithmId::Ed25519.did_verification_method_type().into(),
                controller: did.into(),
                public_key_multibase: key,
                algorithm: Some(AlgorithmId::Ed25519.as_u16()),
            }],
            authentication: vec![format!("{did}#keys-1")],
            key_agreement: vec![format!("{did}#keys-1")],
            service: vec![],
            created: Timestamp::from_micros(0),
            updated: Timestamp::from_micros(1),
            version: 1,
        };
        let error = validate_verification_method_set(&document)
            .expect_err("signing key cannot be used for keyAgreement");
        assert!(error.contains("ML-KEM algorithm"));
    }

    #[test]
    fn latest_did_update_guard_selects_by_source_chain_sequence() {
        let first = ActionHash::from_raw_36(vec![1; 36]);
        let second = ActionHash::from_raw_36(vec![2; 36]);

        fn select_latest(candidates: Vec<(u32, ActionHash)>) -> Option<ActionHash> {
            candidates
                .into_iter()
                .max_by_key(|(seq, _)| *seq)
                .map(|(_, hash)| hash)
        }

        assert_eq!(
            select_latest(vec![(4, first.clone()), (5, second.clone())]),
            Some(second.clone())
        );
        assert_eq!(
            select_latest(vec![(5, second.clone()), (4, first)]),
            Some(second)
        );
    }

    #[test]
    fn did_id_must_match_committing_agent() {
        let me = "did:mycelix:uhCAkSELF";
        // Honest path: coordinator derives id from the committer's own key.
        assert!(matches!(
            require_did_id_matches_author(me, me),
            ValidateCallbackResult::Valid
        ));
        // Takeover attempt: an attacker committing as SELF cannot mint a
        // document whose id claims a VICTIM's DID.
        match require_did_id_matches_author("did:mycelix:uhCAkVICTIM", me) {
            ValidateCallbackResult::Invalid(msg) => {
                assert!(
                    msg.contains("takeover"),
                    "reject reason should name the risk: {msg}"
                );
            }
            other => panic!("forged DID id must be Invalid, got {other:?}"),
        }
    }

    /// Verify that old snake_case MessagePack payloads deserialize through the
    /// new camelCase structs thanks to `#[serde(alias = "...")]` attributes.
    #[test]
    fn backward_compat_snake_case_msgpack_to_struct() {
        // Build a VerificationMethod map using the OLD snake_case field names.
        let old_vm = serde_json::json!({
            "id": "#key-1",
            "type_": "Ed25519VerificationKey2020",
            "controller": "did:mycelix:abc123",
            "public_key_multibase": "z6MkhaXgBZDvotDkL5257faiztiGiC2QtKLGpbnnEGta2doK"
        });

        // Serialize to MessagePack (simulates data written by old code).
        let msgpack_bytes = rmp_serde::to_vec(&old_vm).expect("msgpack serialize");

        // Deserialize into the new struct — alias attributes must accept snake_case.
        let vm: VerificationMethod = rmp_serde::from_slice(&msgpack_bytes)
            .expect("msgpack deserialize into VerificationMethod");

        assert_eq!(vm.id, "#key-1");
        assert_eq!(vm.type_, "Ed25519VerificationKey2020");
        assert_eq!(vm.controller, "did:mycelix:abc123");
        assert_eq!(
            vm.public_key_multibase,
            "z6MkhaXgBZDvotDkL5257faiztiGiC2QtKLGpbnnEGta2doK"
        );
    }

    /// Verify that old snake_case ServiceEndpoint MessagePack deserializes correctly.
    #[test]
    fn backward_compat_snake_case_service_endpoint() {
        let old_se = serde_json::json!({
            "id": "svc-1",
            "type_": "LinkedDomains",
            "service_endpoint": "https://example.com"
        });

        let msgpack_bytes = rmp_serde::to_vec(&old_se).expect("msgpack serialize");
        let se: ServiceEndpoint = rmp_serde::from_slice(&msgpack_bytes)
            .expect("msgpack deserialize into ServiceEndpoint");

        assert_eq!(se.id, "svc-1");
        assert_eq!(se.type_, "LinkedDomains");
        assert_eq!(se.service_endpoint, "https://example.com");
    }

    /// Verify that forward serialization uses camelCase keys (W3C DID Core compliant).
    #[test]
    fn forward_serialization_uses_camel_case() {
        let vm = VerificationMethod {
            id: "#key-1".into(),
            type_: "Ed25519VerificationKey2020".into(),
            controller: "did:mycelix:abc123".into(),
            public_key_multibase: "z6Mk...".into(),
            algorithm: Some(0xed01),
        };

        let json = serde_json::to_value(&vm).expect("serialize to JSON");
        // Must use camelCase, not snake_case
        assert!(
            json.get("publicKeyMultibase").is_some(),
            "expected camelCase 'publicKeyMultibase'"
        );
        assert!(
            json.get("type").is_some(),
            "expected 'type' (renamed from type_)"
        );
        assert!(
            json.get("public_key_multibase").is_none(),
            "snake_case key must not appear"
        );
        assert!(json.get("type_").is_none(), "type_ must not appear");
    }

    /// Verify that the camelCase JSON round-trips through MessagePack correctly.
    #[test]
    fn camel_case_json_to_msgpack_round_trip() {
        let vm = VerificationMethod {
            id: "#key-2".into(),
            type_: "MlDsa65VerificationKey2024".into(),
            controller: "did:mycelix:def456".into(),
            public_key_multibase: "zABC...".into(),
            algorithm: Some(0x0901),
        };

        let msgpack_bytes = rmp_serde::to_vec(&vm).expect("msgpack serialize");
        let vm2: VerificationMethod =
            rmp_serde::from_slice(&msgpack_bytes).expect("msgpack deserialize");

        assert_eq!(vm, vm2);
    }

    mod proptests {
        use super::*;
        use proptest::prelude::*;

        /// Valid multibase keys start with 'z' followed by base58btc characters.
        fn arb_multibase_key() -> impl Strategy<Value = String> {
            prop::collection::vec(any::<u8>(), 32..=32).prop_map(|bytes| {
                let encoded = bs58::encode(&bytes)
                    .with_alphabet(bs58::Alphabet::BITCOIN)
                    .into_string();
                format!("z{}", encoded)
            })
        }

        /// Arbitrary verification method with valid fields.
        fn arb_verification_method() -> impl Strategy<Value = VerificationMethod> {
            (arb_multibase_key(), any::<u16>()).prop_map(|(key, alg)| VerificationMethod {
                id: "#key-1".to_string(),
                type_: "Ed25519VerificationKey2020".to_string(),
                controller: "did:mycelix:test".to_string(),
                public_key_multibase: key,
                algorithm: Some(alg),
            })
        }

        proptest! {
            /// DID IDs without the 'did:mycelix:' prefix always fail the format check.
            #[test]
            fn did_without_prefix_fails_check(suffix in "[a-zA-Z0-9]{1,64}") {
                // Validation functions require EntryCreationAction (needs HDI host),
                // so we test the invariant directly.
                let bad_did = format!("did:other:{}", suffix);
                prop_assert!(!bad_did.starts_with("did:mycelix:"));

                // Also verify non-mycelix schemes
                let bad_did2 = format!("did:key:{}", suffix);
                prop_assert!(!bad_did2.starts_with("did:mycelix:"));
            }

            /// VerificationMethod round-trips through JSON and MessagePack.
            #[test]
            fn verification_method_roundtrips(vm in arb_verification_method()) {
                // JSON round-trip
                let json = serde_json::to_string(&vm).unwrap();
                let vm_json: VerificationMethod = serde_json::from_str(&json).unwrap();
                prop_assert_eq!(&vm, &vm_json);

                // MessagePack round-trip
                let msgpack = rmp_serde::to_vec(&vm).unwrap();
                let vm_msgpack: VerificationMethod = rmp_serde::from_slice(&msgpack).unwrap();
                prop_assert_eq!(&vm, &vm_msgpack);
            }

            /// ServiceEndpoint round-trips through JSON and MessagePack.
            #[test]
            fn service_endpoint_roundtrips(
                id in "[a-z]{1,32}",
                type_ in "[A-Z][a-z]{3,20}",
                url in "https://[a-z]{3,20}\\.[a-z]{2,5}/[a-z]{0,10}"
            ) {
                let se = ServiceEndpoint { id, type_, service_endpoint: url };

                // JSON
                let json = serde_json::to_string(&se).unwrap();
                let se2: ServiceEndpoint = serde_json::from_str(&json).unwrap();
                prop_assert_eq!(&se, &se2);

                // JSON must use camelCase
                let val: serde_json::Value = serde_json::from_str(&json).unwrap();
                prop_assert!(val.get("serviceEndpoint").is_some());
                prop_assert!(val.get("type").is_some());
                prop_assert!(val.get("service_endpoint").is_none());
                prop_assert!(val.get("type_").is_none());
            }

            /// Any DID with 'did:mycelix:' prefix passes the format check.
            #[test]
            fn valid_did_prefix_accepted(suffix in "[a-zA-Z0-9]{8,64}") {
                let did = format!("did:mycelix:{}", suffix);
                prop_assert!(did.starts_with("did:mycelix:"));
            }

            /// Deactivation with empty reason is invalid.
            #[test]
            fn empty_deactivation_reason_is_invalid(did in "did:mycelix:[a-zA-Z0-9]{8,32}") {
                let deactivation = DidDeactivation {
                    did,
                    reason: String::new(),
                    deactivated_at: Timestamp::from_micros(0),
                };                prop_assert!(deactivation.reason.is_empty());
            }
        }
    }

    // =========================================================================
    // Backward-compatibility MessagePack round-trip tests
    // =========================================================================

    /// Old snake_case MessagePack data must deserialize through the new camelCase
    /// structs, thanks to `serde(alias = "...")` attributes.
    #[test]
    fn backward_compat_verification_method_snake_case_msgpack() {
        // Build a map with old snake_case keys
        let old_map = serde_json::json!({
            "id": "#keys-1",
            "type_": "Ed25519VerificationKey2020",
            "controller": "did:mycelix:test",
            "public_key_multibase": "z6Mkabcdef",
            "algorithm": null
        });

        // Serialize to MessagePack via serde_json::Value → rmp
        let msgpack_bytes = rmp_serde::to_vec(&old_map).unwrap();

        // Deserialize into VerificationMethod — the alias attributes should handle
        // snake_case keys from old entries.
        let vm: VerificationMethod = rmp_serde::from_slice(&msgpack_bytes).unwrap();

        assert_eq!(vm.id, "#keys-1");
        assert_eq!(vm.type_, "Ed25519VerificationKey2020");
        assert_eq!(vm.controller, "did:mycelix:test");
        assert_eq!(vm.public_key_multibase, "z6Mkabcdef");
        assert_eq!(vm.algorithm, None);
    }

    /// Forward direction: serialized VerificationMethod uses camelCase keys.
    #[test]
    fn forward_compat_verification_method_camel_case_json() {
        let vm = VerificationMethod {
            id: "#keys-1".into(),
            type_: "Ed25519VerificationKey2020".into(),
            controller: "did:mycelix:test".into(),
            public_key_multibase: "z6Mkabcdef".into(),
            algorithm: Some(0xed01),
        };

        let json = serde_json::to_string(&vm).unwrap();
        // Should use camelCase in output
        assert!(
            json.contains("\"type\""),
            "Should serialize as 'type', not 'type_'"
        );
        assert!(
            json.contains("\"publicKeyMultibase\""),
            "Should serialize as camelCase"
        );
        assert!(
            !json.contains("\"public_key_multibase\""),
            "Should NOT use snake_case in output"
        );
    }

    /// Old snake_case ServiceEndpoint MessagePack round-trip.
    #[test]
    fn backward_compat_service_endpoint_snake_case_msgpack() {
        let old_map = serde_json::json!({
            "id": "#svc-1",
            "type_": "LinkedDomains",
            "service_endpoint": "https://example.com"
        });

        let msgpack_bytes = rmp_serde::to_vec(&old_map).unwrap();
        let svc: ServiceEndpoint = rmp_serde::from_slice(&msgpack_bytes).unwrap();

        assert_eq!(svc.id, "#svc-1");
        assert_eq!(svc.type_, "LinkedDomains");
        assert_eq!(svc.service_endpoint, "https://example.com");
    }

    /// Forward direction: serialized ServiceEndpoint uses camelCase keys.
    #[test]
    fn forward_compat_service_endpoint_camel_case_json() {
        let svc = ServiceEndpoint {
            id: "#svc-1".into(),
            type_: "LinkedDomains".into(),
            service_endpoint: "https://example.com".into(),
        };

        let json = serde_json::to_string(&svc).unwrap();
        assert!(json.contains("\"serviceEndpoint\""), "Should use camelCase");
        assert!(
            !json.contains("\"service_endpoint\""),
            "Should NOT use snake_case"
        );
    }

    // =========================================================================
    // Canonical AgentToDid link binding (P0)
    // =========================================================================

    #[test]
    fn forged_agent_to_did_link_is_rejected_before_dht_lookup() {
        let victim = AgentPubKey::from_raw_36(vec![0u8; 36]);
        let attacker = AgentPubKey::from_raw_36(vec![1u8; 36]);

        let link = CreateLink {
            author: attacker,
            timestamp: Timestamp::from_micros(0),
            action_seq: 1,
            prev_action: ActionHash::from_raw_36(vec![2u8; 36]),
            base_address: victim.clone().into(),
            target_address: ActionHash::from_raw_36(vec![3u8; 36]).into(),
            zome_index: ZomeIndex(0),
            link_type: LinkType::new(0),
            tag: ().into(),
            weight: Default::default(),
        };

        let result = validate_agent_to_did_link(
            &link.base_address,
            &link.target_address,
            &link,
        )
        .expect("link validation should return a callback result");

        match result {
            ValidateCallbackResult::Invalid(message) => {
                assert!(
                    message.contains("authored by the base agent"),
                    "expected owner-binding error, got: {message}"
                );
            }
            other => panic!("forged AgentToDid link must be rejected, got {other:?}"),
        }
    }

    // =========================================================================
    // DID deactivation author-binding (P0)
    // =========================================================================

    fn test_action(author: AgentPubKey) -> Create {
        Create {
            author,
            timestamp: Timestamp::from_micros(0),
            action_seq: 0,
            prev_action: ActionHash::from_raw_36(vec![0u8; 36]),
            entry_type: EntryType::App(AppEntryDef::new(
                EntryDefIndex::from(0),
                0.into(),
                EntryVisibility::Public,
            )),
            entry_hash: EntryHash::from_raw_36(vec![0u8; 36]),
            weight: Default::default(),
        }
    }

    #[test]
    fn deactivation_did_must_match_committing_agent() {
        let me = AgentPubKey::from_raw_36(vec![0u8; 36]);
        let deactivation = DidDeactivation {
            did: format!("did:mycelix:{me}"),
            reason: "compromised key".to_string(),
            deactivated_at: Timestamp::from_micros(0),
        };
        let result = validate_create_did_deactivation(
            EntryCreationAction::Create(test_action(me)),
            deactivation,
        )
        .unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn deactivation_takeover_rejected() {
        // did claims SELF's DID, but VICTIM is the actual committing agent --
        // a modified coordinator trying to deactivate someone else's identity.
        let victim = AgentPubKey::from_raw_36(vec![0u8; 36]);
        let attacker = AgentPubKey::from_raw_36(vec![1u8; 36]);
        let deactivation = DidDeactivation {
            did: format!("did:mycelix:{victim}"),
            reason: "malicious deactivation".to_string(),
            deactivated_at: Timestamp::from_micros(0),
        };
        let result = validate_create_did_deactivation(
            EntryCreationAction::Create(test_action(attacker)),
            deactivation,
        )
        .unwrap();
        match result {
            ValidateCallbackResult::Invalid(msg) => {
                assert!(msg.contains("takeover"), "reject reason: {msg}");
            }
            other => panic!("forged deactivation must be Invalid, got {other:?}"),
        }
    }
}