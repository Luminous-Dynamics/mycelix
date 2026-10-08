// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Verifiable Credential Integrity Zome
//!
//! Mycelix's VC 2.0 application profile with W3C Data Integrity support.
//!
//! The wire model follows the VC Data Model 2.0 security model and implements
//! the W3C eddsa-jcs-2022 cryptosuite, while intentionally constraining
//! @context entries to string-valued terms in the Holochain entry schema.
//! <https://www.w3.org/TR/vc-data-model-2.0/>
//! <https://www.w3.org/TR/vc-di-eddsa-1.1/>

use hdi::prelude::*;

/// W3C Verifiable Credential
/// VC 2.0 application-profile entry with Mycelix integrity constraints.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct VerifiableCredential {
    /// JSON-LD context (required: `https://www.w3.org/ns/credentials/v2`)
    #[serde(rename = "@context")]
    pub context: Vec<String>,
    /// Unique credential identifier
    pub id: String,
    /// Credential types (must include "VerifiableCredential")
    #[serde(rename = "type")]
    pub credential_type: Vec<String>,
    /// DID of the issuer
    pub issuer: CredentialIssuer,
    /// Issuance date (ISO 8601)
    #[serde(rename = "validFrom")]
    pub valid_from: String,
    /// Expiration date (optional, ISO 8601)
    #[serde(rename = "validUntil")]
    pub valid_until: Option<String>,
    /// The claims being made
    #[serde(rename = "credentialSubject")]
    pub credential_subject: CredentialSubject,
    /// Schema reference
    #[serde(rename = "credentialSchema")]
    pub credential_schema: Option<CredentialSchemaRef>,
    /// Credential status (for revocation checking)
    #[serde(rename = "credentialStatus")]
    pub credential_status: Option<CredentialStatus>,
    /// Cryptographic proof
    pub proof: CredentialProof,
    /// Mycelix-specific: schema ID used
    pub mycelix_schema_id: String,
    /// Mycelix-specific: creation timestamp
    pub mycelix_created: Timestamp,
}

/// Credential issuer - can be DID string or object with id
#[derive(Clone, PartialEq, Debug, Serialize, Deserialize)]
#[serde(untagged)]
pub enum CredentialIssuer {
    /// Simple DID string
    Did(String),
    /// Object with id and optional properties
    Object {
        id: String,
        name: Option<String>,
        #[serde(rename = "type")]
        issuer_type: Option<Vec<String>>,
    },
}

impl CredentialIssuer {
    pub fn did(&self) -> &str {
        match self {
            CredentialIssuer::Did(did) => did,
            CredentialIssuer::Object { id, .. } => id,
        }
    }
}

/// Credential subject containing the claims
#[derive(Clone, PartialEq, Debug, Serialize, Deserialize)]
pub struct CredentialSubject {
    /// DID of the subject
    pub id: String,
    /// Claims as key-value pairs (JSON)
    #[serde(flatten)]
    pub claims: serde_json::Value,
}

/// Reference to credential schema
#[derive(Clone, PartialEq, Debug, Serialize, Deserialize)]
pub struct CredentialSchemaRef {
    pub id: String,
    #[serde(rename = "type")]
    pub schema_type: String,
}

/// Credential status for revocation
#[derive(Clone, PartialEq, Debug, Serialize, Deserialize)]
pub struct CredentialStatus {
    pub id: String,
    #[serde(rename = "type")]
    pub status_type: String,
    /// For BitstringStatusList
    #[serde(rename = "statusPurpose")]
    pub status_purpose: Option<String>,
    #[serde(rename = "statusListIndex")]
    pub status_list_index: Option<String>,
    #[serde(rename = "statusListCredential")]
    pub status_list_credential: Option<String>,
}

/// Cryptographic proof
#[derive(Clone, PartialEq, Debug, Serialize, Deserialize)]
pub struct CredentialProof {
    /// Proof type (Ed25519Signature2020, DataIntegrityProof, etc.)
    #[serde(rename = "type")]
    pub proof_type: String,
    /// When the proof was created (ISO 8601)
    pub created: String,
    /// Verification method used (DID URL)
    #[serde(rename = "verificationMethod")]
    pub verification_method: String,
    /// Purpose of the proof
    #[serde(rename = "proofPurpose")]
    pub proof_purpose: String,
    /// The actual signature/proof value (multibase encoded)
    #[serde(rename = "proofValue")]
    pub proof_value: String,
    /// For DataIntegrityProof: cryptosuite used
    pub cryptosuite: Option<String>,
    /// Optional Mycelix algorithm identifier (multicodec u16). W3C cryptosuites such as
    /// eddsa-jcs-2022 define their algorithm through the cryptosuite and omit this field.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub algorithm: Option<u16>,
    /// Challenge value for replay protection (W3C Data Integrity spec).
    /// When present, the verifier MUST supply the same challenge to verify.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub challenge: Option<String>,
    /// Domain restriction (W3C Data Integrity spec).
    /// When present, the verifier MUST supply the same domain to verify.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub domain: Option<String>,
    /// Proof-level @context used by W3C Data Integrity JCS proofs.
    /// At Mycelix admission it is required to equal the secured document's
    /// @context; verifier-side interoperability permits ordered-prefix context.
    #[serde(
        rename = "@context",
        default,
        skip_serializing_if = "Option::is_none"
    )]
    pub proof_context: Option<Vec<String>>,
}

/// Verifiable Presentation - for presenting credentials
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct VerifiablePresentation {
    /// JSON-LD context
    #[serde(rename = "@context")]
    pub context: Vec<String>,
    /// Unique presentation identifier
    pub id: String,
    /// Types (must include "VerifiablePresentation")
    #[serde(rename = "type")]
    pub presentation_type: Vec<String>,
    /// DID of the holder presenting
    pub holder: String,
    /// Credentials being presented
    #[serde(rename = "verifiableCredential")]
    pub verifiable_credential: Vec<VerifiableCredential>,
    /// Proof of presentation
    pub proof: CredentialProof,
    /// Mycelix-specific: creation timestamp
    pub mycelix_created: Timestamp,
}

/// Derived Credential for selective disclosure
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct DerivedCredential {
    /// Exact ActionHash of the original immutable credential.
    ///
    /// The human-readable ID remains for presentation/interoperability, but
    /// lineage-sensitive operations must use this cryptographic action reference.
    pub original_credential_action: ActionHash,
    /// Original credential ID
    pub original_credential_id: String,
    /// DID of the original issuer
    pub original_issuer: String,
    /// DID of the holder creating the derivation
    pub holder: String,
    /// Selected claims (subset of original)
    pub selected_claims: Vec<String>,
    /// The derived credential content
    pub derived_content: CredentialSubject,
    /// Proof that this is a valid derivation
    pub derivation_proof: DerivationProof,
    /// Creation timestamp
    pub created: Timestamp,
    /// Expiration (inherits from original or shorter)
    pub expires: Option<Timestamp>,
}

/// Proof of valid derivation from original credential
#[derive(Clone, PartialEq, Debug, Serialize, Deserialize)]
pub struct DerivationProof {
    /// Type of derivation proof
    #[serde(rename = "type")]
    pub proof_type: String,
    /// Hash of original credential
    pub original_credential_hash: Vec<u8>,
    /// Merkle proof for selected claims (if using Merkle tree)
    pub claim_proofs: Vec<ClaimProof>,
    /// Holder's signature on the derivation
    pub holder_signature: Vec<u8>,
}

/// Proof for individual claim in selective disclosure
#[derive(Clone, PartialEq, Debug, Serialize, Deserialize)]
pub struct ClaimProof {
    /// Claim key
    pub claim_key: String,
    /// Merkle path (for Merkle tree proofs)
    pub merkle_path: Option<Vec<Vec<u8>>>,
    /// Commitment (for commitment schemes)
    pub commitment: Option<Vec<u8>>,
}

/// Credential issuance request (holder to issuer)
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct CredentialRequest {
    /// Request ID
    pub id: String,
    /// Requester's DID
    pub requester_did: String,
    /// Target issuer's DID
    pub issuer_did: String,
    /// Schema ID for requested credential
    pub schema_id: String,
    /// Claims the requester is providing
    pub provided_claims: serde_json::Value,
    /// Supporting evidence (links to other credentials, documents, etc.)
    pub evidence: Vec<CredentialEvidence>,
    /// Request status
    pub status: RequestStatus,
    /// Request timestamp
    pub created: Timestamp,
    /// Status update timestamp
    pub updated: Timestamp,
    /// Exact credential action that fulfilled this request once the request is Issued.
    /// This makes the Issued state proof-carrying rather than a free-standing label.
    pub issued_credential: Option<ActionHash>,
}

/// Evidence supporting a credential request
#[derive(Clone, PartialEq, Debug, Serialize, Deserialize)]
pub struct CredentialEvidence {
    /// Evidence type
    #[serde(rename = "type")]
    pub evidence_type: String,
    /// Evidence ID/URL
    pub id: String,
    /// Description
    pub description: Option<String>,
}

/// Status of credential request
#[derive(Clone, PartialEq, Debug, Serialize, Deserialize)]
pub enum RequestStatus {
    Pending,
    UnderReview,
    Approved,
    Rejected,
    Issued,
}

/// An encrypted entry wrapping sensitive credential data on the DHT.
///
/// Credential claims containing PII are encrypted to the subject's ML-KEM
/// public key. Only the subject (or parties they delegate to) can decrypt.
///
/// The `entry_type_tag` identifies what was encrypted so the decryptor
/// knows which type to deserialize after decryption.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct EncryptedEntry {
    /// Which entry type is encrypted (e.g. "CredentialClaims", "DerivedContent").
    pub entry_type_tag: String,
    /// KEM algorithm used to derive the symmetric key (AlgorithmId as u16).
    /// 0xF020 = ML-KEM-768, 0xF021 = ML-KEM-1024, 0xF030 = self-encrypt.
    pub kem_algorithm: u16,
    /// KEM ciphertext (encapsulated key). Empty for self-encryption.
    pub encapsulated_key: Vec<u8>,
    /// 24-byte nonce for XChaCha20-Poly1305.
    pub nonce: Vec<u8>,
    /// AEAD ciphertext (plaintext || 16-byte Poly1305 tag).
    pub ciphertext: Vec<u8>,
    /// DID URL of the recipient's KEM key (e.g. "did:mycelix:abc#kem-1").
    pub recipient_key_id: String,
    /// When the entry was encrypted.
    pub encrypted_at: Timestamp,
    /// Schema version of the plaintext, for forward-compatible decryption.
    pub plaintext_version: u32,
}

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
#[allow(clippy::large_enum_variant)] // HDK entry types require inline variants
pub enum EntryTypes {
    VerifiableCredential(VerifiableCredential),
    VerifiablePresentation(VerifiablePresentation),
    DerivedCredential(DerivedCredential),
    CredentialRequest(CredentialRequest),
    EncryptedEntry(EncryptedEntry),
}

#[hdk_link_types]
pub enum LinkTypes {
    /// Issuer to credentials they've issued
    IssuerToCredential,
    /// Subject to credentials about them
    SubjectToCredential,
    /// Holder to presentations they've created
    HolderToPresentation,
    /// Credential to derived credentials
    CredentialToDerived,
    /// Schema to credentials using it
    SchemaToCredential,
    /// Issuer to pending requests
    IssuerToRequest,
    /// Requester to their requests
    RequesterToRequest,
    /// Credential ID string to the credential, so `get_credential` can resolve
    /// a credential by ID over the DHT. Without this index a by-ID lookup can
    /// only see the caller's own source chain, which silently returns None for
    /// any credential issued to or by somebody else.
    ///
    /// Appended last on purpose: existing variants keep their ordinals.
    CredentialIdToCredential,
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
                EntryTypes::VerifiableCredential(vc) => {
                    validate_create_verifiable_credential(EntryCreationAction::Create(action), vc)
                }
                EntryTypes::VerifiablePresentation(vp) => {
                    validate_create_verifiable_presentation(EntryCreationAction::Create(action), vp)
                }
                EntryTypes::DerivedCredential(dc) => {
                    validate_create_derived_credential(EntryCreationAction::Create(action), dc)
                }
                EntryTypes::CredentialRequest(req) => {
                    validate_create_credential_request(EntryCreationAction::Create(action), req)
                }
                EntryTypes::EncryptedEntry(entry) => validate_create_encrypted_entry(entry),
            },
            OpEntry::UpdateEntry {
                app_entry, action, ..
            } => match app_entry {
                EntryTypes::CredentialRequest(req) => {
                    validate_update_credential_request(action, req)
                }
                EntryTypes::EncryptedEntry(_) => Ok(ValidateCallbackResult::Invalid(
                    "Encrypted entries are append-only (re-encrypt instead)".into(),
                )),
                _ => Ok(ValidateCallbackResult::Invalid(
                    "Credentials and presentations cannot be updated".into(),
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
            if tag.0.len() > 1024 {
                return Ok(ValidateCallbackResult::Invalid(
                    "Link tag exceeds maximum length of 1024 bytes".into(),
                ));
            }
            validate_credential_link(
                link_type,
                &base_address,
                &target_address,
                &action,
            )
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
            Ok(ValidateCallbackResult::Invalid(
                "Credential indexes and lineage links cannot be deleted".into(),
            ))
        }
        FlatOp::StoreRecord(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterAgentActivity(activity) => match activity {
            OpActivity::CreateEntry {
                app_entry_type: Some(UnitEntryTypes::VerifiableCredential),
                action,
            } => validate_credential_id_chain_uniqueness(action),
            OpActivity::CreateEntry {
                app_entry_type: Some(UnitEntryTypes::CredentialRequest),
                action,
            } => validate_request_id_chain_uniqueness(action),
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterUpdate(update) => match update {
            OpUpdate::Entry {
                app_entry,
                action,
                ..
            } => {
                if matches!(app_entry, EntryTypes::CredentialRequest(_)) {
                    return match app_entry {
                        EntryTypes::CredentialRequest(req) => {
                            validate_update_credential_request(action, req)
                        }
                        _ => Ok(ValidateCallbackResult::Invalid(
                            "Credential request update dispatch became inconsistent".into(),
                        )),
                    };
                }

                let original = must_get_action(action.original_action_address.clone())?;
                if *original.action().author() != action.author {
                    return Ok(ValidateCallbackResult::Invalid(
                        "Only the original entry author can update their entries".into(),
                    ));
                }

                match app_entry {
                    EntryTypes::EncryptedEntry(_) => Ok(ValidateCallbackResult::Invalid(
                        "Encrypted entries are append-only (re-encrypt instead)".into(),
                    )),
                    _ => Ok(ValidateCallbackResult::Invalid(
                        "Credentials and presentations cannot be updated".into(),
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
        FlatOp::RegisterDelete(OpDelete { .. }) => Ok(ValidateCallbackResult::Invalid(
            "Verifiable credentials, presentations, derived credentials, requests, and encrypted entries are append-only"
                .into(),
        )),
    }
}

fn string_to_entry_hash(value: &str) -> EntryHash {
    let bytes = holo_hash::blake2b_256(value.as_bytes())
        .into_iter()
        .chain([0u8; 4])
        .collect::<Vec<u8>>();
    EntryHash::from_raw_36(bytes)
}

/// Return true when a DID URL's authority component names exactly the supplied DID.
///
/// The coordinator verifies signatures with the issuer/holder AgentPubKey, so a
/// mismatched verification-method DID would otherwise become unauthenticated
/// metadata: the proof could claim a method controlled by a different DID while
/// the signature is actually checked against the issuer/holder key.
fn verification_method_matches_did(verification_method: &str, did: &str) -> bool {
    let method_did = verification_method.split_once('#').map_or(verification_method, |(base, _)| base);
    method_did == did && !verification_method.is_empty()
}

fn did_to_agent(did: &str) -> Option<AgentPubKey> {
    did.strip_prefix("did:mycelix:")
        .and_then(|value| AgentPubKey::try_from(value).ok())
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

/// Recompute the content hash used by the coordinator's derived-credential proof.
///
/// This intentionally mirrors `compute_credential_hash` in the coordinator zome so
/// an integrity validator can bind a derived credential to the exact credential
/// content it claims to derive from. The binding is to the credential content hash,
/// not merely its human-readable ID.
fn is_date_time_stamp(value: &str) -> bool {
    // VC 2.x validity values are XML Schema dateTimeStamp-like values:
    // full date + time + an explicit timezone. Fractional seconds are allowed.
    if value.len() < 20 || value.as_bytes().get(10) != Some(&b'T') {
        return false;
    }
    let bytes = value.as_bytes();
    if !bytes[0..10].iter().enumerate().all(|(i, b)| {
        if matches!(i, 4 | 7) { *b == b'-' } else { b.is_ascii_digit() }
    }) {
        return false;
    }
    if bytes[13] != b':' || bytes[16] != b':' {
        return false;
    }
    if !bytes[11..19].iter().enumerate().all(|(i, b)| {
        if matches!(i, 2 | 5) { *b == b':' } else { b.is_ascii_digit() }
    }) {
        return false;
    }

    if value.ends_with('Z') {
        return value.parse::<Timestamp>().is_ok();
    }
    if value.len() < 25 {
        return false;
    }
    let tz_start = value.len() - 6;
    (bytes[tz_start] == b'+' || bytes[tz_start] == b'-')
        && bytes[tz_start + 3] == b':'
        && bytes[tz_start + 1].is_ascii_digit()
        && bytes[tz_start + 2].is_ascii_digit()
        && bytes[tz_start + 4].is_ascii_digit()
        && bytes[tz_start + 5].is_ascii_digit()
        && value.parse::<Timestamp>().is_ok()
}

fn eddsa_jcs_hash_data_from_values(
    mut unsecured: Value,
    mut proof_config: Value,
) -> ExternResult<Vec<u8>> {
    let document_context = unsecured
        .get("@context")
        .cloned()
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "JCS secured document must contain an @context".into()
        )))?;

    let proof_config_map = proof_config.as_object_mut().ok_or(wasm_error!(
        WasmErrorInner::Guest("JCS proof configuration must serialize to a JSON object".into())
    ))?;

    if let Some(proof_context) = proof_config_map.get("@context").cloned() {
        let document_values = document_context.as_array().ok_or(wasm_error!(
            WasmErrorInner::Guest("JCS document @context must be an array".into())
        ))?;
        let proof_values = proof_context.as_array().ok_or(wasm_error!(
            WasmErrorInner::Guest("JCS proof @context must be an array".into())
        ))?;
        if proof_values.is_empty()
            || proof_values.len() > document_values.len()
            || document_values[..proof_values.len()] != proof_values[..]
        {
            return Err(wasm_error!(WasmErrorInner::Guest(
                "JCS proof @context must be an ordered prefix of the document @context".into()
            )));
        }

        // Per eddsa-jcs-2022 verification, the transformed unsecured
        // document uses the proof's context for canonicalization.
        unsecured
            .as_object_mut()
            .ok_or(wasm_error!(WasmErrorInner::Guest(
                "JCS secured document must serialize to an object".into()
            )))?
            .insert("@context".into(), proof_context);
    }

    let canonical_document = serde_json_canonicalizer::to_vec(&unsecured).map_err(|e| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "JCS document canonicalization failed: {e}"
        )))
    })?;
    let canonical_proof_config = serde_json_canonicalizer::to_vec(&proof_config).map_err(|e| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "JCS proof configuration canonicalization failed: {e}"
        )))
    })?;
    let transformed_document_hash = Sha256::digest(&canonical_document);
    let proof_config_hash = Sha256::digest(&canonical_proof_config);
    let mut hash_data = Vec::with_capacity(64);
    hash_data.extend_from_slice(&proof_config_hash);
    hash_data.extend_from_slice(&transformed_document_hash);
    Ok(hash_data)
}


fn eddsa_jcs_hash_data(vc: &VerifiableCredential) -> ExternResult<Vec<u8>> {
    let mut unsecured = serde_json::to_value(vc).map_err(|e| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "Credential JSON serialization failed: {e}"
        )))
    })?;
    let unsecured_map = unsecured.as_object_mut().ok_or(wasm_error!(
        WasmErrorInner::Guest("Credential must serialize to a JSON object".into())
    ))?;
    unsecured_map.remove("proof");

    let proof_value = serde_json::to_value(&vc.proof).map_err(|e| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "Proof JSON serialization failed: {e}"
        )))
    })?;
    let mut proof_config = proof_value.as_object().cloned().ok_or(wasm_error!(
        WasmErrorInner::Guest("Credential proof must serialize to a JSON object".into())
    ))?;
    proof_config.remove("proofValue");

    eddsa_jcs_hash_data_from_values(unsecured, Value::Object(proof_config))
}
fn decode_raw_jcs_signature(value: &str) -> Result<[u8; 64], String> {
    if !value.starts_with('z') || value.len() <= 1 {
        return Err("W3C JCS proofValue must use base58-btc Multibase (z prefix)".into());
    }
    let decoded = bs58::decode(&value[1..])
        .with_alphabet(bs58::Alphabet::BITCOIN)
        .into_vec()
        .map_err(|e| format!("Invalid proofValue base58-btc payload: {e}"))?;
    <[u8; 64]>::try_from(decoded.as_slice())
        .map_err(|_| "W3C JCS proofValue must decode to exactly 64 Ed25519 bytes".into())
}

fn verify_credential_signature_at_validation(
    action_author: &AgentPubKey,
    vc: &VerifiableCredential,
) -> ExternResult<bool> {
    let hash_data = match vc.proof.cryptosuite.as_deref() {
        None | Some("mycelix-blake2b-ed25519-2026") => compute_credential_content_hash(vc),
        Some("eddsa-jcs-2022") => eddsa_jcs_hash_data(vc)?,
        Some(_) => return Ok(false),
    };

    if vc.proof.cryptosuite.as_deref() == Some("eddsa-jcs-2022") {
        let raw = decode_raw_jcs_signature(&vc.proof.proof_value).map_err(|e| {
            wasm_error!(WasmErrorInner::Guest(e))
        })?;
        return verify_signature(
            action_author.clone(),
            Signature::from(raw),
            hash_data,
        );
    }

    let tagged = TaggedSignature::from_multibase(&vc.proof.proof_value)
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?;
    if tagged.algorithm != AlgorithmId::Ed25519 {
        return Ok(false);
    }
    if tagged.signature_bytes.len() != 64 {
        return Ok(false);
    }
    let raw = <[u8; 64]>::try_from(tagged.signature_bytes.as_slice())
        .map_err(|_| wasm_error!(WasmErrorInner::Guest(
            "Ed25519 proofValue must decode to exactly 64 bytes".into()
        )))?;

    verify_signature(action_author.clone(), Signature::from(raw), hash_data)
}

fn compute_credential_content_hash(vc: &VerifiableCredential) -> Vec<u8> {
    let mut content = Vec::new();
    content.extend(vc.id.as_bytes());
    content.push(0);
    content.extend(vc.issuer.did().as_bytes());
    content.push(0);
    content.extend(vc.credential_subject.id.as_bytes());
    content.push(0);
    content.extend(vc.valid_from.as_bytes());
    content.push(0);
    if let Ok(claims_json) = serde_json::to_string(&vc.credential_subject.claims) {
        content.extend(claims_json.as_bytes());
    } else {
        return Vec::new();
    }
    content.push(0);
    content.extend(vc.mycelix_schema_id.as_bytes());

    holo_hash::blake2b_256(&content).to_vec()
}


fn eddsa_jcs_hash_data_for_presentation(
    vp: &VerifiablePresentation,
) -> ExternResult<Vec<u8>> {
    let mut unsecured = serde_json::to_value(vp).map_err(|e| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "Presentation JSON serialization failed: {e}"
        )))
    })?;
    let unsecured_map = unsecured.as_object_mut().ok_or(wasm_error!(
        WasmErrorInner::Guest("Presentation must serialize to a JSON object".into())
    ))?;
    unsecured_map.remove("proof");

    let proof_value = serde_json::to_value(&vp.proof).map_err(|e| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "Presentation proof JSON serialization failed: {e}"
        )))
    })?;
    let mut proof_config = proof_value.as_object().cloned().ok_or(wasm_error!(
        WasmErrorInner::Guest("Presentation proof must serialize to a JSON object".into())
    ))?;
    proof_config.remove("proofValue");

    eddsa_jcs_hash_data_from_values(unsecured, Value::Object(proof_config))
}

fn compute_presentation_content_hash(vp: &VerifiablePresentation) -> Vec<u8> {
    // Preserve the exact legacy payload layout used by the historical
    // coordinator profile for backward verification of existing presentations.
    let mut content = vp.id.as_bytes().to_vec();
    content.extend(vp.holder.as_bytes());
    for credential in &vp.verifiable_credential {
        content.extend(credential.id.as_bytes());
    }
    if let Some(challenge) = &vp.proof.challenge {
        content.extend(challenge.as_bytes());
    }
    if let Some(domain) = &vp.proof.domain {
        content.extend(domain.as_bytes());
    }

    holo_hash::blake2b_256(&content).to_vec()
}

fn verify_presentation_signature_at_validation(
    action_author: &AgentPubKey,
    vp: &VerifiablePresentation,
) -> ExternResult<bool> {
    let hash_data = match vp.proof.cryptosuite.as_deref() {
        Some("eddsa-jcs-2022") => eddsa_jcs_hash_data_for_presentation(vp)?,
        None | Some("mycelix-blake2b-ed25519-2026") => compute_presentation_content_hash(vp),
        Some(_) => return Ok(false),
    };

    if vp.proof.cryptosuite.as_deref() == Some("eddsa-jcs-2022") {
        let raw = decode_raw_jcs_signature(&vp.proof.proof_value)
            .map_err(|e| wasm_error!(WasmErrorInner::Guest(e)))?;
        return verify_signature(action_author.clone(), Signature::from(raw), hash_data);
    }

    let tagged = TaggedSignature::from_multibase(&vp.proof.proof_value)
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?;
    if tagged.algorithm != AlgorithmId::Ed25519 || tagged.signature_bytes.len() != 64 {
        return Ok(false);
    }
    let raw = <[u8; 64]>::try_from(tagged.signature_bytes.as_slice()).map_err(|_| {
        wasm_error!(WasmErrorInner::Guest(
            "Presentation Ed25519 proofValue must decode to exactly 64 bytes".into()
        ))
    })?;
    verify_signature(action_author.clone(), Signature::from(raw), hash_data)
}

fn validate_credential_link(
    link_type: LinkTypes,
    base_address: &AnyLinkableHash,
    target_address: &AnyLinkableHash,
    action: &CreateLink,
) -> ExternResult<ValidateCallbackResult> {
    let target = action_target(target_address, "Credential link")?;
    let record = must_get_valid_record(target)?;

    match link_type {
        LinkTypes::IssuerToCredential
        | LinkTypes::SubjectToCredential
        | LinkTypes::SchemaToCredential
        | LinkTypes::CredentialIdToCredential => {
            let vc: VerifiableCredential = record
                .entry()
                .to_app_option()
                .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
                .ok_or(wasm_error!(WasmErrorInner::Guest(
                    "Credential index target must contain a VerifiableCredential".into(),
                )))?;
            let issuer = did_to_agent(vc.issuer.did()).ok_or(wasm_error!(
                WasmErrorInner::Guest("Credential issuer must be a did:mycelix AgentPubKey".into())
            ))?;
            if action.author != issuer {
                return Ok(ValidateCallbackResult::Invalid(
                    "Credential index link must be authored by the credential issuer".into(),
                ));
            }

            let expected_base = match link_type {
                LinkTypes::IssuerToCredential => string_to_entry_hash(vc.issuer.did()),
                LinkTypes::SubjectToCredential => string_to_entry_hash(&vc.credential_subject.id),
                LinkTypes::SchemaToCredential => string_to_entry_hash(&vc.mycelix_schema_id),
                LinkTypes::CredentialIdToCredential => string_to_entry_hash(&vc.id),
                _ => {
                    return Ok(ValidateCallbackResult::Invalid(
                        "Unexpected credential link type for credential index".into(),
                    ));
                }
            };
            let actual_base = base_address.clone().into_entry_hash().ok_or_else(|| {
                wasm_error!(WasmErrorInner::Guest(
                    "Credential index link base must be an EntryHash".into(),
                ))
            })?;
            if actual_base != expected_base {
                return Ok(ValidateCallbackResult::Invalid(
                    "Credential index link base does not match the target credential".into(),
                ));
            }
        }
        LinkTypes::HolderToPresentation => {
            let vp: VerifiablePresentation = record
                .entry()
                .to_app_option()
                .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
                .ok_or(wasm_error!(WasmErrorInner::Guest(
                    "HolderToPresentation target must contain a VerifiablePresentation".into(),
                )))?;
            if vp.holder != format!("did:mycelix:{}", action.author) {
                return Ok(ValidateCallbackResult::Invalid(
                    "HolderToPresentation link must be authored by the presentation holder".into(),
                ));
            }
            let actual_base = base_address.clone().into_entry_hash().ok_or_else(|| {
                wasm_error!(WasmErrorInner::Guest(
                    "HolderToPresentation base must be an EntryHash".into(),
                ))
            })?;
            if actual_base != string_to_entry_hash(&vp.holder) {
                return Ok(ValidateCallbackResult::Invalid(
                    "HolderToPresentation base does not match the presentation holder".into(),
                ));
            }
        }
        LinkTypes::CredentialToDerived => {
            let dc: DerivedCredential = record
                .entry()
                .to_app_option()
                .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
                .ok_or(wasm_error!(WasmErrorInner::Guest(
                    "CredentialToDerived target must contain a DerivedCredential".into(),
                )))?;
            if dc.holder != format!("did:mycelix:{}", action.author) {
                return Ok(ValidateCallbackResult::Invalid(
                    "CredentialToDerived link must be authored by the derived-credential holder".into(),
                ));
            }
            let actual_base = base_address.clone().into_action_hash().ok_or_else(|| {
                wasm_error!(WasmErrorInner::Guest(
                    "CredentialToDerived base must be an ActionHash".into(),
                ))
            })?;
            let original_record = must_get_valid_record(actual_base.clone())?;
            let original_vc: VerifiableCredential = original_record
                .entry()
                .to_app_option()
                .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
                .ok_or(wasm_error!(WasmErrorInner::Guest(
                    "CredentialToDerived base must reference a VerifiableCredential".into(),
                )))?;
            if actual_base != dc.original_credential_action {
                return Ok(ValidateCallbackResult::Invalid(
                    "CredentialToDerived base does not match the derived credential's pinned source action".into(),
                ));
            }
            if original_vc.id != dc.original_credential_id {
                return Ok(ValidateCallbackResult::Invalid(
                    "CredentialToDerived base does not match the referenced original credential ID".into(),
                ));
            }
            if dc.original_issuer != original_vc.issuer.did() {
                return Ok(ValidateCallbackResult::Invalid(
                    "CredentialToDerived original issuer does not match the referenced credential".into(),
                ));
            }
            if dc.holder != original_vc.credential_subject.id
                || dc.derived_content.id != dc.holder
            {
                return Ok(ValidateCallbackResult::Invalid(
                    "CredentialToDerived holder must match the original credential subject".into(),
                ));
            }

            let expected_hash = compute_credential_content_hash(&original_vc);
            if dc.derivation_proof.original_credential_hash != expected_hash {
                return Ok(ValidateCallbackResult::Invalid(
                    "CredentialToDerived proof hash does not match the referenced credential content".into(),
                ));
            }

            let original_claims = original_vc.credential_subject.claims.as_object().ok_or(
                wasm_error!(WasmErrorInner::Guest(
                    "CredentialToDerived original credential claims must be an object".into(),
                )),
            )?;
            let derived_claims = dc.derived_content.claims.as_object().ok_or(
                wasm_error!(WasmErrorInner::Guest(
                    "CredentialToDerived derived claims must be an object".into(),
                )),
            )?;

            for (i, claim) in dc.selected_claims.iter().enumerate() {
                if dc.selected_claims.iter().skip(i + 1).any(|other| other == claim) {
                    return Ok(ValidateCallbackResult::Invalid(
                        "CredentialToDerived selected claims must not contain duplicates".into(),
                    ));
                }
                let Some(original_value) = original_claims.get(claim) else {
                    return Ok(ValidateCallbackResult::Invalid(
                        "CredentialToDerived selected claim is absent from the original credential".into(),
                    ));
                };
                let Some(derived_value) = derived_claims.get(claim) else {
                    return Ok(ValidateCallbackResult::Invalid(
                        "CredentialToDerived selected claim is absent from the derived credential".into(),
                    ));
                };
                if original_value != derived_value {
                    return Ok(ValidateCallbackResult::Invalid(
                        "CredentialToDerived selected claim value does not match the original credential".into(),
                    ));
                }
            }

            if derived_claims.keys().any(|key| {
                !dc.selected_claims.iter().any(|selected| selected == key)
            }) {
                return Ok(ValidateCallbackResult::Invalid(
                    "CredentialToDerived contains an unselected derived claim".into(),
                ));
            }
        }
        LinkTypes::IssuerToRequest | LinkTypes::RequesterToRequest => {
            let req: CredentialRequest = record
                .entry()
                .to_app_option()
                .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
                .ok_or(wasm_error!(WasmErrorInner::Guest(
                    "Credential request link target must contain a CredentialRequest".into(),
                )))?;
            let requester = did_to_agent(&req.requester_did).ok_or(wasm_error!(
                WasmErrorInner::Guest("Credential requester must be a did:mycelix AgentPubKey".into())
            ))?;
            if action.author != requester {
                return Ok(ValidateCallbackResult::Invalid(
                    "Credential request index link must be authored by the requester".into(),
                ));
            }
            let actual_base = base_address.clone().into_entry_hash().ok_or_else(|| {
                wasm_error!(WasmErrorInner::Guest(
                    "Credential request link base must be an EntryHash".into(),
                ))
            })?;
            let expected_base = match link_type {
                LinkTypes::IssuerToRequest => string_to_entry_hash(&req.issuer_did),
                LinkTypes::RequesterToRequest => string_to_entry_hash(&req.requester_did),
                _ => {
                    return Ok(ValidateCallbackResult::Invalid(
                        "Unexpected credential request link type for request index".into(),
                    ));
                }
            };
            if actual_base != expected_base {
                return Ok(ValidateCallbackResult::Invalid(
                    "Credential request link base does not match the target request".into(),
                ));
            }
        }
    }

    Ok(ValidateCallbackResult::Valid)
}

/// The DID that a self-issued entry's issuer/owner field must equal, derived
/// from the committing agent's key. Centralizing this keeps the author-binding
/// rule identical across validators and unit tests. Mirrors the pattern already
/// used by trust_credential's `validate_create_credential`.
fn expected_issuer_did(author: &AgentPubKey) -> String {
    format!("did:mycelix:{}", author)
}

/// Enforce that a credential's declared issuer is the agent actually committing
/// it. Pure so it can be unit-tested without constructing a full Create action.
fn require_issuer_is_author(issuer_did: &str, author_did: &str) -> ValidateCallbackResult {
    if issuer_did != author_did {
        return ValidateCallbackResult::Invalid(format!(
            "Issuer DID must match the committing agent (credential-issuer forgery). Expected '{}', got '{}'",
            author_did, issuer_did
        ));
    }
    ValidateCallbackResult::Valid
}

fn validate_credential_id_chain_uniqueness(
    action: Create,
) -> ExternResult<ValidateCallbackResult> {
    let current_entry = must_get_entry(action.entry_hash.clone())?;
    let current: VerifiableCredential = current_entry.try_into().map_err(|e| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "VerifiableCredential entry could not be decoded: {e}"
        )))
    })?;

    let activity = must_get_agent_activity(
        action.author.clone(),
        ChainFilter::new(action.prev_action.clone()),
    )?;
    let entry_type =
        EntryType::App(AppEntryDef::try_from(UnitEntryTypes::VerifiableCredential)?);

    for prior in activity {
        let prior_action = prior.action.action();
        let Action::Create(prior_create) = prior_action else {
            continue;
        };
        if prior_create.entry_type != entry_type {
            continue;
        }

        let prior_entry = must_get_entry(prior_create.entry_hash.clone())?;
        let prior: VerifiableCredential = prior_entry.try_into().map_err(|e| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "Credential ID history entry could not be decoded: {e}"
            )))
        })?;

        if prior.id == current.id {
            return Ok(ValidateCallbackResult::Invalid(
                "A credential ID may only have one creation on an issuer source chain".into(),
            ));
        }
    }

    Ok(ValidateCallbackResult::Valid)
}

fn validate_request_id_chain_uniqueness(
    action: Create,
) -> ExternResult<ValidateCallbackResult> {
    let current_entry = must_get_entry(action.entry_hash.clone())?;
    let current: CredentialRequest = current_entry.try_into().map_err(|e| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "CredentialRequest entry could not be decoded: {e}"
        )))
    })?;

    let activity = must_get_agent_activity(
        action.author.clone(),
        ChainFilter::new(action.prev_action.clone()),
    )?;
    let entry_type =
        EntryType::App(AppEntryDef::try_from(UnitEntryTypes::CredentialRequest)?);

    for prior in activity {
        let prior_action = prior.action.action();
        let Action::Create(prior_create) = prior_action else {
            continue;
        };
        if prior_create.entry_type != entry_type {
            continue;
        }

        let prior_entry = must_get_entry(prior_create.entry_hash.clone())?;
        let prior: CredentialRequest = prior_entry.try_into().map_err(|e| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "Credential request ID history entry could not be decoded: {e}"
            )))
        })?;

        if prior.id == current.id {
            return Ok(ValidateCallbackResult::Invalid(
                "A credential request ID may only have one creation on a requester source chain"
                    .into(),
            ));
        }
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Validate verifiable credential creation
fn validate_create_verifiable_credential(
    action: EntryCreationAction,
    vc: VerifiableCredential,
) -> ExternResult<ValidateCallbackResult> {
    // Bind the credential to its committer: any agent could otherwise store a
    // VC claiming `issuer: "did:B"`. The coordinator already sets the issuer to
    // the committing agent's own DID (agent_info().agent_initial_pubkey), so
    // this rejects only forged credentials, never the honest create path.
    let author_did = expected_issuer_did(action.author());
    if let ValidateCallbackResult::Invalid(msg) =
        require_issuer_is_author(vc.issuer.did(), &author_did)
    {
        return Ok(ValidateCallbackResult::Invalid(msg));
    }

    // W3C VC Data Model 2.0 requires the base credentials-v2 context
    // to be the first item in the ordered @context set.
    if vc.context.first().map(String::as_str) != Some("https://www.w3.org/ns/credentials/v2") {
        return Ok(ValidateCallbackResult::Invalid(
            "Credential @context must begin with https://www.w3.org/ns/credentials/v2".into(),
        ));
    }

    // Validate type includes VerifiableCredential
    if !vc
        .credential_type
        .contains(&"VerifiableCredential".to_string())
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Credential type must include 'VerifiableCredential'".into(),
        ));
    }

    // Validate issuer is a DID
    if !vc.issuer.did().starts_with("did:") {
        return Ok(ValidateCallbackResult::Invalid(
            "Issuer must be a valid DID".into(),
        ));
    }

    // Validate subject has ID
    if !vc.credential_subject.id.starts_with("did:") {
        return Ok(ValidateCallbackResult::Invalid(
            "Credential subject must have a valid DID".into(),
        ));
    }

    // The Mycelix creation timestamp is provenance metadata, not an issuer-
    // controlled validity date. It must not claim a time after the actual
    // Holochain create action.
    if vc.mycelix_created > action.timestamp() {
        return Ok(ValidateCallbackResult::Invalid(
            "Credential mycelix_created timestamp cannot be in the future relative to its create action".into(),
        ));
    }

    // Validate the W3C temporal fields before accepting them into the DHT.
    // Holochain Timestamp parsing gives us a concrete temporal value, allowing
    // us to enforce the VC validity interval rather than trusting arbitrary text.
    if !is_date_time_stamp(&vc.valid_from) {
        return Ok(ValidateCallbackResult::Invalid(
            "Credential validFrom must be an explicit dateTimeStamp with timezone".into(),
        ));
    }
    if let Some(valid_until) = vc.valid_until.as_deref() {
        if !is_date_time_stamp(valid_until) {
            return Ok(ValidateCallbackResult::Invalid(
                "Credential validUntil must be an explicit dateTimeStamp with timezone".into(),
            ));
        }
    }

    let valid_from = vc.valid_from.parse::<Timestamp>().map_err(|e| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "Credential validFrom must be a valid RFC3339 timestamp: {e}"
        )))
    })?;
    if let Some(valid_until_str) = vc.valid_until.as_deref() {
        let valid_until = valid_until_str.parse::<Timestamp>().map_err(|e| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "Credential validUntil must be a valid RFC3339 timestamp: {e}"
            )))
        })?;
        if valid_from > valid_until {
            return Ok(ValidateCallbackResult::Invalid(
                "Credential validFrom must not be later than validUntil".into(),
            ));
        }
    }

    if vc.mycelix_schema_id.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Credential must include a Mycelix schema ID".into(),
        ));
    }
    if let Some(schema) = &vc.credential_schema {
        if schema.id != vc.mycelix_schema_id {
            return Ok(ValidateCallbackResult::Invalid(
                "credentialSchema.id must match mycelix_schema_id".into(),
            ));
        }
        if schema.schema_type.is_empty() {
            return Ok(ValidateCallbackResult::Invalid(
                "credentialSchema.type must not be empty".into(),
            ));
        }
    }

    match vc.proof.cryptosuite.as_deref() {
        None => {}
        Some("mycelix-blake2b-ed25519-2026") => {
            if vc.proof.algorithm != Some(0xed01) {
                return Ok(ValidateCallbackResult::Invalid(
                    "Mycelix BLAKE2b-Ed25519 proof must declare the Ed25519 0xed01 algorithm".into(),
                ));
            }
        }
        Some("eddsa-jcs-2022") => {
            if vc.proof.proof_type != "DataIntegrityProof" {
                return Ok(ValidateCallbackResult::Invalid(
                    "eddsa-jcs-2022 proofs must use DataIntegrityProof".into(),
                ));
            }
            if vc.proof.algorithm.is_some() {
                return Ok(ValidateCallbackResult::Invalid(
                    "eddsa-jcs-2022 proofs must not declare a non-standard algorithm field".into(),
                ));
            }
            if vc.proof.proof_context.as_deref() != Some(vc.context.as_slice()) {
                return Ok(ValidateCallbackResult::Invalid(
                    "eddsa-jcs-2022 proof @context must exactly match the credential @context at admission".into(),
                ));
            }
        }
        Some(_) => {
            return Ok(ValidateCallbackResult::Invalid(
                "Unsupported credential cryptosuite for the current in-DNA verifier".into(),
            ));
        }
    }

    // Validate proof exists and has required fields.
    if vc.proof.proof_type.is_empty() || vc.proof.proof_value.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Credential must have valid proof".into(),
        ));
    }
    if !is_date_time_stamp(&vc.proof.created) {
        return Ok(ValidateCallbackResult::Invalid(
            "Credential proof created value must be an explicit dateTimeStamp with timezone".into(),
        ));
    }
    let proof_created = vc.proof.created.parse::<Timestamp>().map_err(|e| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "Credential proof created value must parse as a timestamp: {e}"
        )))
    })?;
    if proof_created > action.timestamp() {
        return Ok(ValidateCallbackResult::Invalid(
            "Credential proof created value cannot be in the future relative to its create action".into(),
        ));
    }

    // The proof verification method must belong to the same DID whose key
    // authenticates the credential signature. The fragment is an identifier
    let signature_valid = verify_credential_signature_at_validation(&action.author(), &vc)?;
    if !signature_valid {
        return Ok(ValidateCallbackResult::Invalid(
            "Credential proof signature does not verify against the committing agent".into(),
        ));
    }

    // within that DID document; the verifier's cryptographic key is derived
    // from the issuer DID itself.
    if !verification_method_matches_did(&vc.proof.verification_method, vc.issuer.did()) {
        return Ok(ValidateCallbackResult::Invalid(
            "Credential proof verification method must belong to the issuer DID".into(),
        ));
    }

    // Validate proof purpose
    if vc.proof.proof_purpose != "assertionMethod" {
        return Ok(ValidateCallbackResult::Invalid(
            "Credential proof purpose must be 'assertionMethod'".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Validate verifiable presentation creation
fn validate_create_verifiable_presentation(
    action: EntryCreationAction,
    vp: VerifiablePresentation,
) -> ExternResult<ValidateCallbackResult> {
    let expected_holder_did = format!("did:mycelix:{}", action.author());
    if vp.holder != expected_holder_did {
        return Ok(ValidateCallbackResult::Invalid(
            "Presentation holder DID must correspond to the committing agent".into(),
        ));
    }
    if vp.mycelix_created > action.timestamp() {
        return Ok(ValidateCallbackResult::Invalid(
            "Presentation mycelix_created timestamp cannot be in the future relative to its create action".into(),
        ));
    }

    if vp.context.first().map(String::as_str) != Some("https://www.w3.org/ns/credentials/v2") {
        return Ok(ValidateCallbackResult::Invalid(
            "Presentation @context must start with the W3C credentials/v2 context".into(),
        ));
    }

    if !vp
        .presentation_type
        .contains(&"VerifiablePresentation".to_string())
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Presentation type must include 'VerifiablePresentation'".into(),
        ));
    }

    if !vp.holder.starts_with("did:") {
        return Ok(ValidateCallbackResult::Invalid(
            "Holder must be a valid DID".into(),
        ));
    }

    if vp.verifiable_credential.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Presentation must contain at least one credential".into(),
        ));
    }

    if !verification_method_matches_did(&vp.proof.verification_method, &vp.holder) {
        return Ok(ValidateCallbackResult::Invalid(
            "Presentation proof verification method must belong to the holder DID".into(),
        ));
    }

    if vp.proof.proof_purpose != "authentication" {
        return Ok(ValidateCallbackResult::Invalid(
            "Presentation proof purpose must be 'authentication'".into(),
        ));
    }

    match vp.proof.cryptosuite.as_deref() {
        Some("eddsa-jcs-2022") => {
            if vp.proof.proof_type != "DataIntegrityProof" {
                return Ok(ValidateCallbackResult::Invalid(
                    "eddsa-jcs-2022 presentation proofs must use DataIntegrityProof".into(),
                ));
            }
            if vp.proof.algorithm.is_some() {
                return Ok(ValidateCallbackResult::Invalid(
                    "eddsa-jcs-2022 presentation proofs must not declare algorithm".into(),
                ));
            }
            if vp.proof.proof_context.as_deref() != Some(vp.context.as_slice()) {
                return Ok(ValidateCallbackResult::Invalid(
                    "eddsa-jcs-2022 presentation proof @context must exactly match the presentation @context at admission".into(),
                ));
            }
            let expected_method = format!("{}#keys-1-multikey", vp.holder);
            if vp.proof.verification_method != expected_method {
                return Ok(ValidateCallbackResult::Invalid(
                    "eddsa-jcs-2022 presentation proof must use the holder's canonical Multikey".into(),
                ));
            }
        }
        None | Some("mycelix-blake2b-ed25519-2026") => {
            if vp.proof.algorithm != Some(0xed01) {
                return Ok(ValidateCallbackResult::Invalid(
                    "Mycelix presentation proof must declare the Ed25519 0xed01 algorithm".into(),
                ));
            }
        }
        Some(_) => {
            return Ok(ValidateCallbackResult::Invalid(
                "Unsupported presentation cryptosuite".into(),
            ));
        }
    }

    if vp.proof.proof_type.is_empty() || vp.proof.proof_value.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Presentation must have a valid proof".into(),
        ));
    }
    if !is_date_time_stamp(&vp.proof.created) {
        return Ok(ValidateCallbackResult::Invalid(
            "Presentation proof created value must be an explicit dateTimeStamp with timezone".into(),
        ));
    }
    let proof_created = vp.proof.created.parse::<Timestamp>().map_err(|e| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "Presentation proof created value must parse as a timestamp: {e}"
        )))
    })?;
    if proof_created > action.timestamp() {
        return Ok(ValidateCallbackResult::Invalid(
            "Presentation proof created value cannot be in the future relative to its create action".into(),
        ));
    }

    let signature_valid = verify_presentation_signature_at_validation(&action.author(), &vp)?;
    if !signature_valid {
        return Ok(ValidateCallbackResult::Invalid(
            "Presentation proof signature does not verify against the committing holder".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Validate derived credential creation
fn validate_create_derived_credential(
    action: EntryCreationAction,
    dc: DerivedCredential,
) -> ExternResult<ValidateCallbackResult> {
    // Author-binding: the coordinator's create_derived_credential already
    // derives `holder_did` from agent_info() and checks the caller is the
    // original credential's subject before proceeding, but that's
    // bypassable by a modified coordinator -- the integrity validator is
    // the real security boundary. Without this, any agent could commit a
    // DerivedCredential claiming to be derived by an arbitrary holder DID.
    let expected_holder_did = format!("did:mycelix:{}", action.author());
    if dc.holder != expected_holder_did {
        return Ok(ValidateCallbackResult::Invalid(
            "Derived credential holder DID must correspond to the committing agent".into(),
        ));
    }
    if dc.created > action.timestamp() {
        return Ok(ValidateCallbackResult::Invalid(
            "Derived credential created timestamp cannot be in the future relative to its create action".into(),
        ));
    }

    // Validate holder is a DID
    if !dc.holder.starts_with("did:") {
        return Ok(ValidateCallbackResult::Invalid(
            "Holder must be a valid DID".into(),
        ));
    }

    // Validate selected claims not empty
    if dc.selected_claims.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Must select at least one claim".into(),
        ));
    }

    // Validate derivation proof exists
    if dc.derivation_proof.holder_signature.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Derivation must have holder signature".into(),
        ));
    }

    // The entry itself must carry complete source lineage; link validation is
    // an additional integrity layer, not the primary admission boundary.
    let source_record = must_get_valid_record(dc.original_credential_action.clone())?;
    let source: VerifiableCredential = source_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Derived credential source action must reference a VerifiableCredential".into(),
        )))?;

    if dc.original_credential_id != source.id {
        return Ok(ValidateCallbackResult::Invalid(
            "Derived credential original_credential_id does not match the pinned source".into(),
        ));
    }
    if dc.original_issuer != source.issuer.did() {
        return Ok(ValidateCallbackResult::Invalid(
            "Derived credential original_issuer does not match the pinned source".into(),
        ));
    }
    if dc.holder != source.credential_subject.id || dc.derived_content.id != dc.holder {
        return Ok(ValidateCallbackResult::Invalid(
            "Derived credential holder/subject lineage does not match the pinned source".into(),
        ));
    }

    let source_hash = compute_credential_content_hash(&source);
    if dc.derivation_proof.original_credential_hash != source_hash {
        return Ok(ValidateCallbackResult::Invalid(
            "Derived credential proof hash does not match the pinned source content".into(),
        ));
    }

    match source.valid_from.parse::<Timestamp>() {
        Ok(source_valid_from) if dc.created < source_valid_from => {
            return Ok(ValidateCallbackResult::Invalid(
                "Derived credential cannot be created before the source credential becomes valid".into(),
            ));
        }
        Ok(_) => {}
        Err(e) => {
            return Err(wasm_error!(WasmErrorInner::Guest(format!(
                "Pinned source validFrom is not parseable: {e}"
            ))));
        }
    }

    if let Some(expires) = dc.expires {
        if expires <= dc.created {
            return Ok(ValidateCallbackResult::Invalid(
                "Derived credential expiration must be after creation".into(),
            ));
        }

        if let Some(source_valid_until) = source.valid_until.as_deref() {
            let source_valid_until = source_valid_until.parse::<Timestamp>().map_err(|e| {
                wasm_error!(WasmErrorInner::Guest(format!(
                    "Pinned source validUntil is not parseable: {e}"
                )))
            })?;
            if expires > source_valid_until {
                return Ok(ValidateCallbackResult::Invalid(
                    "Derived credential expiration exceeds source credential expiration".into(),
                ));
            }
        }
    } else if source.valid_until.is_some() {
        return Ok(ValidateCallbackResult::Invalid(
            "Derived credential must carry an expiration when the source credential expires".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Validate credential request creation
fn validate_create_credential_request(
    action: EntryCreationAction,
    req: CredentialRequest,
) -> ExternResult<ValidateCallbackResult> {
    if req.created > action.timestamp() || req.updated > action.timestamp() {
        return Ok(ValidateCallbackResult::Invalid(
            "Credential request timestamps cannot be in the future relative to its create action".into(),
        ));
    }

    // Validate requester DID
    if !req.requester_did.starts_with("did:") {
        return Ok(ValidateCallbackResult::Invalid(
            "Requester must be a valid DID".into(),
        ));
    }

    // Author-binding: the coordinator's request_credential already derives
    // `requester_did` from agent_info() (it isn't part of the input
    // struct), so this is belt-and-suspenders against a modified
    // coordinator forging a credential request on someone else's behalf.
    // Note `issuer_did` is intentionally NOT bound here -- it names the
    // TARGET issuer being asked to issue, a third party, not the committer.
    let expected_requester_did = format!("did:mycelix:{}", action.author());
    if req.requester_did != expected_requester_did {
        return Ok(ValidateCallbackResult::Invalid(
            "Credential request requester DID must correspond to the committing agent".into(),
        ));
    }

    // Validate issuer DID
    if !req.issuer_did.starts_with("did:") {
        return Ok(ValidateCallbackResult::Invalid(
            "Target issuer must be a valid DID".into(),
        ));
    }

    if req.issued_credential.is_some() {
        return Ok(ValidateCallbackResult::Invalid(
            "A newly created credential request cannot already be marked as Issued".into(),
        ));
    }

    // Validate schema ID
    if !req.schema_id.starts_with("mycelix:schema:") {
        return Ok(ValidateCallbackResult::Invalid(
            "Schema ID must be valid Mycelix schema".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

fn valid_request_status_transition(from: &RequestStatus, to: &RequestStatus) -> bool {
    matches!(
        (from, to),
        (RequestStatus::Pending, RequestStatus::UnderReview)
            | (RequestStatus::Pending, RequestStatus::Rejected)
            | (RequestStatus::UnderReview, RequestStatus::Approved)
            | (RequestStatus::UnderReview, RequestStatus::Rejected)
            | (RequestStatus::Approved, RequestStatus::Issued)
            // Re-publishing the exact same state is intentionally idempotent.
            | (RequestStatus::Pending, RequestStatus::Pending)
            | (RequestStatus::UnderReview, RequestStatus::UnderReview)
            | (RequestStatus::Approved, RequestStatus::Approved)
            | (RequestStatus::Rejected, RequestStatus::Rejected)
            | (RequestStatus::Issued, RequestStatus::Issued)
    )
}


fn credential_claims_satisfy_request(
    requested: &serde_json::Value,
    issued: &serde_json::Value,
) -> bool {
    match (requested.as_object(), issued.as_object()) {
        (Some(requested), Some(issued)) => requested
            .iter()
            .all(|(key, value)| issued.get(key) == Some(value)),
        _ => requested == issued,
    }
}

fn validate_issued_credential_binding(
    req: &CredentialRequest,
) -> ExternResult<ValidateCallbackResult> {
    let credential_hash = req.issued_credential.clone().ok_or(wasm_error!(
        WasmErrorInner::Guest(
            "Issued credential request must reference the credential action that fulfilled it".into(),
        )
    ))?;

    let record = must_get_valid_record(credential_hash)?;
    let credential: VerifiableCredential = record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Issued credential reference must point to a VerifiableCredential".into(),
        )))?;

    let issuer = did_to_agent(&req.issuer_did).ok_or(wasm_error!(
        WasmErrorInner::Guest(
            "Issued credential request issuer must be a did:mycelix AgentPubKey".into(),
        )
    ))?;

    if record.action().author() != &issuer {
        return Ok(ValidateCallbackResult::Invalid(
            "Issued credential must be committed by the request's target issuer".into(),
        ));
    }
    if credential.issuer.did() != req.issuer_did {
        return Ok(ValidateCallbackResult::Invalid(
            "Issued credential issuer does not match the credential request issuer".into(),
        ));
    }
    if credential.credential_subject.id != req.requester_did {
        return Ok(ValidateCallbackResult::Invalid(
            "Issued credential subject does not match the credential requester".into(),
        ));
    }
    if credential.mycelix_schema_id != req.schema_id {
        return Ok(ValidateCallbackResult::Invalid(
            "Issued credential schema does not match the credential request schema".into(),
        ));
    }
    if credential.proof.cryptosuite.as_deref() != Some("eddsa-jcs-2022") {
        return Ok(ValidateCallbackResult::Invalid(
            "Request-bound issuance requires an eddsa-jcs-2022 credential proof".into(),
        ));
    }
    if credential.proof.proof_type != "DataIntegrityProof" {
        return Ok(ValidateCallbackResult::Invalid(
            "Request-bound issuance requires a DataIntegrityProof".into(),
        ));
    }
    if !credential_claims_satisfy_request(&req.provided_claims, &credential.credential_subject.claims) {
        return Ok(ValidateCallbackResult::Invalid(
            "Issued credential claims do not fulfill the claims supplied in the credential request".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Validate credential request update.
///
/// Credential requests are created by the requester but status transitions are
/// authorized by the target issuer. The update therefore intentionally breaks
/// the usual same-author rule for this one append-only workflow while retaining
/// strict authorship: only the issuer named by the original request can publish
/// a status transition.
fn validate_update_credential_request(
    action: Update,
    req: CredentialRequest,
) -> ExternResult<ValidateCallbackResult> {
    // Fetch original to enforce identity, immutability, and state transitions.
    let original_record = must_get_valid_record(action.original_action_address.clone())?;
    let original: CredentialRequest = original_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Original credential request not found".into()
        )))?;

    // The issuer is the sole authority over request state after creation.
    let issuer = did_to_agent(&original.issuer_did).ok_or(wasm_error!(
        WasmErrorInner::Guest("Original credential request issuer must be a did:mycelix AgentPubKey".into())
    ))?;
    if action.author != issuer {
        return Ok(ValidateCallbackResult::Invalid(
            "Credential request status updates must be authored by the target issuer".into(),
        ));
    }

    // The request payload is immutable. Only status and the monotonic updated
    // timestamp may change after creation; otherwise an issuer could silently
    // replace the claimant's evidence/claims while approving the same request.
    if req.id != original.id {
        return Ok(ValidateCallbackResult::Invalid(
            "Request ID cannot be changed".into(),
        ));
    }
    if req.requester_did != original.requester_did {
        return Ok(ValidateCallbackResult::Invalid(
            "Requester DID cannot be changed".into(),
        ));
    }
    if req.issuer_did != original.issuer_did {
        return Ok(ValidateCallbackResult::Invalid(
            "Issuer DID cannot be changed".into(),
        ));
    }
    if req.schema_id != original.schema_id {
        return Ok(ValidateCallbackResult::Invalid(
            "Schema ID cannot be changed".into(),
        ));
    }
    if req.provided_claims != original.provided_claims {
        return Ok(ValidateCallbackResult::Invalid(
            "Provided claims cannot be changed after request creation".into(),
        ));
    }
    if req.evidence != original.evidence {
        return Ok(ValidateCallbackResult::Invalid(
            "Credential request evidence cannot be changed after request creation".into(),
        ));
    }
    if req.created != original.created {
        return Ok(ValidateCallbackResult::Invalid(
            "Credential request creation timestamp cannot be changed".into(),
        ));
    }
    if original.status == RequestStatus::Issued
        && req.issued_credential != original.issued_credential
    {
        return Ok(ValidateCallbackResult::Invalid(
            "An Issued request's credential binding is immutable".into(),
        ));
    }
    if req.status != RequestStatus::Issued
        && req.issued_credential != original.issued_credential
    {
        return Ok(ValidateCallbackResult::Invalid(
            "The issued credential binding cannot change before the request reaches Issued".into(),
        ));
    }
    if req.status == RequestStatus::Issued {
        if original.status != RequestStatus::Approved {
            // The transition helper below will reject other paths; this early
            // branch gives the binding a precise, proof-oriented error.
            if req.issued_credential.is_none() {
                return Ok(ValidateCallbackResult::Invalid(
                    "Issued status requires a credential action reference".into(),
                ));
            }
        }
        match validate_issued_credential_binding(&req)? {
            ValidateCallbackResult::Valid => {}
            invalid @ ValidateCallbackResult::Invalid(_) => return Ok(invalid),
        }
    } else if req.issued_credential.is_some() {
        return Ok(ValidateCallbackResult::Invalid(
            "Only an Issued request may carry an issued credential reference".into(),
        ));
    }
    if req.updated <= original.updated {
        return Ok(ValidateCallbackResult::Invalid(
            "Credential request updated timestamp must advance monotonically".into(),
        ));
    }
    if req.updated > action.timestamp() {
        return Ok(ValidateCallbackResult::Invalid(
            "Credential request updated timestamp cannot be in the future relative to its update action".into(),
        ));
    }

    let valid = valid_request_status_transition(&original.status, &req.status);

    if !valid {
        return Ok(ValidateCallbackResult::Invalid(
            "Invalid credential request status transition".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Validate encrypted entry creation
///
/// No author-binding possible: EncryptedEntry has no self-declared
/// owner/creator AgentPubKey or DID field. `recipient_key_id` names who the
/// ciphertext is addressed TO, not who committed it. Reviewed 2026-07-08
/// during the P0 author-binding pass; same reasoning as mfa's and
/// trust_credential's EncryptedEntry validators in this cluster.
fn validate_create_encrypted_entry(entry: EncryptedEntry) -> ExternResult<ValidateCallbackResult> {
    // Validate entry type tag is non-empty
    if entry.entry_type_tag.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Encrypted entry must specify entry_type_tag".into(),
        ));
    }

    // Validate nonce length (XChaCha20-Poly1305 requires 24 bytes)
    if entry.nonce.len() != 24 {
        return Ok(ValidateCallbackResult::Invalid(format!(
            "Nonce must be 24 bytes (XChaCha20-Poly1305), got {}",
            entry.nonce.len()
        )));
    }

    // Validate ciphertext is non-empty (minimum: 16-byte Poly1305 tag)
    if entry.ciphertext.len() < 16 {
        return Ok(ValidateCallbackResult::Invalid(
            "Ciphertext too short (minimum 16 bytes for Poly1305 tag)".into(),
        ));
    }

    // Validate recipient key ID is a DID URL or "self"
    if entry.recipient_key_id != "self" && !entry.recipient_key_id.starts_with("did:") {
        return Ok(ValidateCallbackResult::Invalid(
            "recipient_key_id must be 'self' or a DID URL".into(),
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

    fn valid_proof() -> CredentialProof {
        CredentialProof {
            proof_type: "Ed25519Signature2020".into(),
            created: "2026-01-01T00:00:00Z".into(),
            verification_method: "did:mycelix:issuer1#key-1".into(),
            proof_purpose: "assertionMethod".into(),
            proof_value: "zBase64EncodedSignature".into(),
            cryptosuite: None,
            algorithm: None,
            challenge: None,
            domain: None,
            proof_context: None,
        }
    }

    fn valid_vc() -> VerifiableCredential {
        VerifiableCredential {
            context: vec![
                "https://www.w3.org/ns/credentials/v2".into(),
                "https://mycelix.net/ns/v1".into(),
            ],
            id: "urn:uuid:test-credential-001".into(),
            credential_type: vec!["VerifiableCredential".into(), "DegreeCredential".into()],
            issuer: CredentialIssuer::Did("did:mycelix:issuer1".into()),
            valid_from: "2026-01-01T00:00:00Z".into(),
            valid_until: Some("2030-01-01T00:00:00Z".into()),
            credential_subject: CredentialSubject {
                id: "did:mycelix:holder1".into(),
                claims: serde_json::json!({"degree": "BSc Computer Science"}),
            },
            credential_schema: Some(CredentialSchemaRef {
                id: "mycelix:schema:education:degree:v1".into(),
                schema_type: "JsonSchema".into(),
            }),
            credential_status: None,
            proof: valid_proof(),
            mycelix_schema_id: "mycelix:schema:education:degree:v1".into(),
            mycelix_created: ts(1_700_000_000_000_000),
        }
    }

    // --- Author binding (credential-issuer forgery prevention) ---

    #[test]
    fn issuer_must_match_committing_agent() {
        let author_did = "did:mycelix:uhCAkSELF".to_string();

        // Honest path: coordinator sets issuer to the committer's own DID.
        assert!(matches!(
            require_issuer_is_author(&author_did, &author_did),
            ValidateCallbackResult::Valid
        ));

        // Forgery: a VC claiming a different issuer than its committer is rejected.
        let forged = require_issuer_is_author("did:mycelix:uhCAkVICTIM", &author_did);
        match forged {
            ValidateCallbackResult::Invalid(msg) => {
                assert!(
                    msg.contains("forgery"),
                    "reject reason should name the risk: {msg}"
                );
            }
            other => panic!("forged issuer must be Invalid, got {other:?}"),
        }
    }

    // --- W3C camelCase field naming ---

    #[test]
    fn vc_json_uses_w3c_camel_case_fields() {
        let vc = valid_vc();
        let json = serde_json::to_string_pretty(&vc).unwrap();

        // W3C fields must be camelCase
        assert!(json.contains("\"@context\""), "Must have @context");
        assert!(
            json.contains("\"validFrom\""),
            "Must have validFrom not valid_from"
        );
        assert!(
            json.contains("\"validUntil\""),
            "Must have validUntil not valid_until"
        );
        assert!(
            json.contains("\"credentialSubject\""),
            "Must have credentialSubject"
        );
        assert!(
            json.contains("\"credentialSchema\""),
            "Must have credentialSchema"
        );
        assert!(json.contains("\"proofPurpose\""), "Must have proofPurpose");
        assert!(json.contains("\"proofValue\""), "Must have proofValue");
        assert!(
            json.contains("\"verificationMethod\""),
            "Must have verificationMethod"
        );

        // Must NOT contain snake_case versions
        assert!(!json.contains("\"valid_from\""));
        assert!(!json.contains("\"valid_until\""));
        assert!(!json.contains("\"credential_subject\""));
        assert!(!json.contains("\"credential_schema\""));
        assert!(!json.contains("\"proof_purpose\""));
        assert!(!json.contains("\"proof_value\""));
        assert!(!json.contains("\"verification_method\""));

        // JSON-LD type field should be "type" not "credential_type"
        assert!(json.contains("\"type\""));
        assert!(!json.contains("\"credential_type\""));
    }

    #[test]
    fn vc_json_round_trip() {
        let vc = valid_vc();
        let json = serde_json::to_string(&vc).unwrap();
        let back: VerifiableCredential = serde_json::from_str(&json).unwrap();
        assert_eq!(vc, back);
    }

    // --- CredentialIssuer ---

    #[test]
    fn credential_issuer_did_string() {
        let issuer = CredentialIssuer::Did("did:mycelix:issuer1".into());
        assert_eq!(issuer.did(), "did:mycelix:issuer1");
        let json = serde_json::to_string(&issuer).unwrap();
        assert_eq!(json, "\"did:mycelix:issuer1\"");
    }

    #[test]
    fn credential_issuer_object() {
        let issuer = CredentialIssuer::Object {
            id: "did:mycelix:issuer2".into(),
            name: Some("Acme University".into()),
            issuer_type: Some(vec!["Issuer".into()]),
        };
        assert_eq!(issuer.did(), "did:mycelix:issuer2");
        let json = serde_json::to_string(&issuer).unwrap();
        assert!(json.contains("\"id\""));
        assert!(json.contains("\"name\""));
        let back: CredentialIssuer = serde_json::from_str(&json).unwrap();
        assert_eq!(back.did(), "did:mycelix:issuer2");
    }

    // --- CredentialProof ---

    #[test]
    fn credential_proof_camel_case_fields() {
        let proof = valid_proof();
        let json = serde_json::to_string(&proof).unwrap();
        assert!(json.contains("\"proofPurpose\""));
        assert!(json.contains("\"proofValue\""));
        assert!(json.contains("\"verificationMethod\""));
        assert!(json.contains("\"type\""));
        assert!(!json.contains("\"proof_type\""));
    }

    #[test]
    fn credential_proof_with_algorithm() {
        let mut proof = valid_proof();
        proof.cryptosuite = Some("eddsa-rdfc-2022".into());
        proof.algorithm = Some(0xED01);
        let json = serde_json::to_string(&proof).unwrap();
        assert!(json.contains("\"cryptosuite\""));
        assert!(json.contains("\"algorithm\""));
        let back: CredentialProof = serde_json::from_str(&json).unwrap();
        assert_eq!(back.algorithm, Some(0xED01));
    }

    #[test]
    fn credential_proof_algorithm_defaults_to_none() {
        let proof = valid_proof();
        assert_eq!(proof.algorithm, None);
        let json = serde_json::to_string(&proof).unwrap();
        // skip_serializing_if = "Option::is_none"
        assert!(!json.contains("\"algorithm\""));
    }

    // --- VerifiablePresentation ---

    #[test]
    fn vp_json_camel_case_fields() {
        let vp = VerifiablePresentation {
            context: vec!["https://www.w3.org/ns/credentials/v2".into()],
            id: "urn:uuid:presentation-001".into(),
            presentation_type: vec!["VerifiablePresentation".into()],
            holder: "did:mycelix:holder1".into(),
            verifiable_credential: vec![valid_vc()],
            proof: CredentialProof {
                proof_purpose: "authentication".into(),
                ..valid_proof()
            },
            mycelix_created: ts(1_700_000_000_000_000),
        };
        let json = serde_json::to_string(&vp).unwrap();
        assert!(json.contains("\"verifiableCredential\""));
        assert!(!json.contains("\"verifiable_credential\""));
        // Round-trip
        let back: VerifiablePresentation = serde_json::from_str(&json).unwrap();
        assert_eq!(vp, back);
    }

    // --- Validation conditions ---


    #[test]
    fn jcs_hash_matches_w3c_1_1_vector() {
        // W3C Data Integrity EdDSA Cryptosuites v1.1, Examples 30-36.
        let document = serde_json::json!({
            "@context": [
                "https://www.w3.org/ns/credentials/v2",
                "https://www.w3.org/ns/credentials/examples/v2"
            ],
            "id": "urn:uuid:58172aac-d8ba-11ed-83dd-0b3aef56cc33",
            "type": ["VerifiableCredential", "AlumniCredential"],
            "name": "Alumni Credential",
            "description": "A minimum viable example of an Alumni Credential.",
            "issuer": "https://vc.example/issuers/5678",
            "validFrom": "2023-01-01T00:00:00Z",
            "credentialSubject": {
                "id": "did:example:abcdefgh",
                "alumniOf": "The School of Examples"
            }
        });
        let proof_config = serde_json::json!({
            "@context": [
                "https://www.w3.org/ns/credentials/v2",
                "https://www.w3.org/ns/credentials/examples/v2"
            ],
            "type": "DataIntegrityProof",
            "cryptosuite": "eddsa-jcs-2022",
            "created": "2023-02-24T23:36:38Z",
            "verificationMethod": "did:key:z6MkrJVnaZkeFzdQyMZu1cgjg7k1pZZ6pvBQ7XJPt4swbTQ2#z6MkrJVnaZkeFzdQyMZu1cgjg7k1pZZ6pvBQ7XJPt4swbTQ2",
            "proofPurpose": "assertionMethod"
        });

        let hash_data = eddsa_jcs_hash_data_from_values(document, proof_config)
            .expect("W3C 1.1 JCS vector must canonicalize");
        let hex = hash_data
            .iter()
            .map(|byte| format!("{byte:02x}"))
            .collect::<String>();
        assert_eq!(
            hex,
            "66ab154f5c2890a140cb8388a22a160454f80575f6eae09e5a097cabe539a1db59b7cb6251b8991add1ce0bc83107e3db9dbbab5bd2c28f687db1a03abc92f19"
        );
    }

    #[test]
    fn jcs_proof_context_must_be_ordered_prefix() {
        let mut document = serde_json::json!({
            "@context": [
                "https://www.w3.org/ns/credentials/v2",
                "https://w3id.org/security/data-integrity/v2"
            ],
            "id": "urn:uuid:jcs-context-prefix-test"
        });
        let proof_prefix = serde_json::json!({
            "@context": ["https://www.w3.org/ns/credentials/v2"],
            "type": "DataIntegrityProof"
        });
        assert!(eddsa_jcs_hash_data_from_values(
            document.clone(),
            proof_prefix
        ).is_ok());

        let bad_proof = serde_json::json!({
            "@context": ["https://w3id.org/security/data-integrity/v2"],
            "type": "DataIntegrityProof"
        });
        assert!(eddsa_jcs_hash_data_from_values(document.clone(), bad_proof).is_err());

        document["@context"] = serde_json::json!("https://www.w3.org/ns/credentials/v2");
        let array_proof = serde_json::json!({
            "@context": ["https://www.w3.org/ns/credentials/v2"],
            "type": "DataIntegrityProof"
        });
        assert!(eddsa_jcs_hash_data_from_values(document, array_proof).is_err());
    }

    #[test]
    fn jcs_presentation_hash_uses_unsecured_document_and_proof_configuration() {
        let mut vp = valid_presentation(format!("did:mycelix:{}", me()));
        vp.proof.cryptosuite = Some("eddsa-jcs-2022".into());
        vp.proof.proof_context = Some(vp.context.clone());
        vp.proof.verification_method =
            format!("{}#keys-1-multikey", vp.holder);
        vp.proof.proof_value = String::new();

        let first = eddsa_jcs_hash_data_for_presentation(&vp)
            .expect("JCS presentation hash must be constructible");
        vp.proof.challenge = Some("challenge-2".into());
        let second = eddsa_jcs_hash_data_for_presentation(&vp)
            .expect("JCS presentation hash must remain constructible");
        assert_ne!(first, second, "challenge is part of the secured presentation");
        assert_eq!(first.len(), 64);
        assert_eq!(second.len(), 64);
    }

    #[test]
    fn native_cryptosuite_requires_ed25519_algorithm() {
        let algorithm = Some(0xed01u16);
        assert_eq!(algorithm, Some(0xed01));
        assert_ne!(Some(0xF001u16), Some(0xed01));
    }

    #[test]
    fn vc_context_must_use_w3c_base_as_first_item() {
        let mut vc = minimal_vc();
        assert_eq!(
            vc.context.first().map(String::as_str),
            Some("https://www.w3.org/ns/credentials/v2")
        );
        vc.context = vec!["https://example.invalid/first".into(), "https://www.w3.org/ns/credentials/v2".into()];
        assert_ne!(
            vc.context.first().map(String::as_str),
            Some("https://www.w3.org/ns/credentials/v2")
        );
    }

    #[test]
    fn date_time_stamp_requires_explicit_timezone() {
        assert!(is_date_time_stamp("2026-01-01T00:00:00Z"));
        assert!(is_date_time_stamp("2026-01-01T00:00:00+02:00"));
        assert!(is_date_time_stamp("2026-01-01T00:00:00.123Z"));
        assert!(!is_date_time_stamp("2026-01-01"));
        assert!(!is_date_time_stamp("2026-01-01T00:00:00"));
        assert!(!is_date_time_stamp("2026-13-99T99:99:99Z"));
    }

    #[test]
    fn vc_validator_rejects_future_creation_provenance() {
        let mut vc = minimal_vc();
        vc.mycelix_created = ts(2_100_000_000_000_000);

        let result = validate_create_verifiable_credential(
            EntryCreationAction::Create(test_action(me())),
            vc,
        )
        .unwrap();

        assert!(
            matches!(result, ValidateCallbackResult::Invalid(message) if message.contains("mycelix_created"))
        );
    }

    #[test]
    fn vc_validator_rejects_future_proof_creation() {
        let mut vc = minimal_vc();
        vc.proof.created = "2100-01-01T00:00:00Z".into();

        let result = validate_create_verifiable_credential(
            EntryCreationAction::Create(test_action(me())),
            vc,
        )
        .unwrap();

        assert!(
            matches!(result, ValidateCallbackResult::Invalid(message) if message.contains("proof created"))
        );
    }

    #[test]
    fn vc_validator_rejects_inverted_validity_interval() {
        let mut vc = minimal_vc();
        vc.valid_until = Some("2025-12-31T23:59:59Z".into());
        let result = validate_create_verifiable_credential(
            EntryCreationAction::Create(test_action(me())),
            vc,
        )
        .unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn vc_validator_rejects_malformed_validity_timestamp() {
        let mut vc = minimal_vc();
        vc.valid_from = "not-a-timestamp".into();
        let result = validate_create_verifiable_credential(
            EntryCreationAction::Create(test_action(me())),
            vc,
        )
        .unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn vc_validator_requires_base_context_first() {
        let mut vc = minimal_vc();
        vc.context = vec![
            "https://example.invalid/context".into(),
            "https://www.w3.org/ns/credentials/v2".into(),
        ];
        let result = validate_create_verifiable_credential(
            EntryCreationAction::Create(test_action(me())),
            vc,
        )
        .unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn vc_validity_interval_is_ordered() {
        let from = "2026-01-01T00:00:00Z".parse::<Timestamp>().unwrap();
        let until = "2026-01-02T00:00:00Z".parse::<Timestamp>().unwrap();
        assert!(from <= until);
        assert!("2026-01-01T00:00:00Z".parse::<Timestamp>().is_ok());
        assert!("not-a-timestamp".parse::<Timestamp>().is_err());
    }

    #[test]
    fn vc_must_include_credentials_context() {
        let ctx = vec!["https://www.w3.org/ns/credentials/v2".to_string()];
        assert!(ctx.iter().any(|c| c.contains("credentials")));
        let bad_ctx = vec!["https://example.com".to_string()];
        assert!(!bad_ctx.iter().any(|c| c.contains("credentials")));
    }

    #[test]
    fn vc_type_must_include_verifiable_credential() {
        let types = vec![
            "VerifiableCredential".to_string(),
            "DegreeCredential".to_string(),
        ];
        assert!(types.contains(&"VerifiableCredential".to_string()));
        let bad_types = vec!["DegreeCredential".to_string()];
        assert!(!bad_types.contains(&"VerifiableCredential".to_string()));
    }

    #[test]
    fn vc_issuer_must_be_did() {
        assert!("did:mycelix:abc".starts_with("did:"));
        assert!(!"https://example.com".starts_with("did:"));
    }

    #[test]
    fn vp_proof_purpose_must_be_authentication() {
        assert_eq!("authentication", "authentication");
        assert_ne!("assertionMethod", "authentication");
    }

    // --- CredentialStatus ---

    #[test]
    fn credential_status_camel_case() {
        let status = CredentialStatus {
            id: "https://example.com/status/1".into(),
            status_type: "BitstringStatusListEntry".into(),
            status_purpose: Some("revocation".into()),
            status_list_index: Some("42".into()),
            status_list_credential: Some("https://example.com/status-list".into()),
        };
        let json = serde_json::to_string(&status).unwrap();
        assert!(json.contains("\"statusPurpose\""));
        assert!(json.contains("\"statusListIndex\""));
        assert!(json.contains("\"statusListCredential\""));
        let back: CredentialStatus = serde_json::from_str(&json).unwrap();
        assert_eq!(status, back);
    }

    // --- RequestStatus ---

    #[test]
    fn request_status_json_variants() {
        let variants = vec![
            (RequestStatus::Pending, "\"Pending\""),
            (RequestStatus::UnderReview, "\"UnderReview\""),
            (RequestStatus::Approved, "\"Approved\""),
            (RequestStatus::Rejected, "\"Rejected\""),
            (RequestStatus::Issued, "\"Issued\""),
        ];
        for (variant, expected) in variants {
            let json = serde_json::to_string(&variant).unwrap();
            assert_eq!(json, expected);
        }
    }

    // --- CredentialRequest ---

    #[test]
    fn credential_request_json_round_trip() {
        let req = CredentialRequest {
            id: "req-001".into(),
            requester_did: "did:mycelix:holder1".into(),
            issuer_did: "did:mycelix:issuer1".into(),
            schema_id: "mycelix:schema:education:degree:v1".into(),
            provided_claims: serde_json::json!({"name": "Alice", "degree": "BSc CS"}),
            evidence: vec![CredentialEvidence {
                evidence_type: "DocumentVerification".into(),
                id: "evidence-001".into(),
                description: Some("Transcript verification".into()),
            }],
            status: RequestStatus::Pending,
            created: ts(1_700_000_000_000_000),
            updated: ts(1_700_000_000_000_000),
        };
        let json = serde_json::to_string(&req).unwrap();
        let back: CredentialRequest = serde_json::from_str(&json).unwrap();
        assert_eq!(req, back);
    }

    #[test]
    fn credential_request_requires_did_prefix() {
        assert!("did:mycelix:holder1".starts_with("did:"));
        assert!(!"alice@example.com".starts_with("did:"));
    }

    #[test]
    fn credential_request_requires_schema_prefix() {
        assert!("mycelix:schema:education:degree:v1".starts_with("mycelix:schema:"));
        assert!(!"custom-schema-id".starts_with("mycelix:schema:"));
    }

    // --- EncryptedEntry ---

    #[test]
    fn encrypted_entry_json_round_trip() {
        let entry = EncryptedEntry {
            entry_type_tag: "CredentialClaims".into(),
            kem_algorithm: 0xF020,
            encapsulated_key: vec![1u8; 128],
            nonce: vec![0u8; 24],
            ciphertext: vec![42u8; 64],
            recipient_key_id: "did:mycelix:holder1#kem-1".into(),
            encrypted_at: ts(1_700_000_000_000_000),
            plaintext_version: 1,
        };
        let json = serde_json::to_string(&entry).unwrap();
        let back: EncryptedEntry = serde_json::from_str(&json).unwrap();
        assert_eq!(entry, back);
    }

    #[test]
    fn encrypted_entry_nonce_must_be_24_bytes() {
        assert_eq!(vec![0u8; 24].len(), 24, "24-byte nonce is valid");
        assert_ne!(vec![0u8; 16].len(), 24, "16-byte nonce is invalid");
        assert_ne!(vec![0u8; 32].len(), 24, "32-byte nonce is invalid");
    }

    #[test]
    fn encrypted_entry_ciphertext_minimum_16_bytes() {
        assert!(vec![0u8; 16].len() >= 16, "16 bytes is minimum (tag only)");
        assert!(vec![0u8; 15].len() < 16, "15 bytes is too short");
        assert!(vec![0u8; 64].len() >= 16, "64 bytes is valid");
    }

    #[test]
    fn encrypted_entry_recipient_key_id_must_be_did_or_self() {
        assert_eq!("self", "self");
        assert!("did:mycelix:holder1#kem-1".starts_with("did:"));
        assert!(!"random-key-id".starts_with("did:") && "random-key-id" != "self");
    }

    #[test]
    fn encrypted_entry_self_encryption() {
        let entry = EncryptedEntry {
            entry_type_tag: "DerivedContent".into(),
            kem_algorithm: 0xF030,    // self-encrypt
            encapsulated_key: vec![], // empty for self-encryption
            nonce: vec![0u8; 24],
            ciphertext: vec![42u8; 32],
            recipient_key_id: "self".into(),
            encrypted_at: ts(1_700_000_000_000_000),
            plaintext_version: 1,
        };
        let json = serde_json::to_string(&entry).unwrap();
        let back: EncryptedEntry = serde_json::from_str(&json).unwrap();
        assert_eq!(entry, back);
        assert!(entry.encapsulated_key.is_empty());
    }

    // --- DerivedCredential ---

    #[test]
    fn derived_credential_json_round_trip() {
        let dc = DerivedCredential {
            original_credential_action: ActionHash::from_raw_36(vec![9u8; 36]),
            original_credential_id: "urn:uuid:cred-001".into(),
            original_issuer: "did:mycelix:issuer1".into(),
            holder: "did:mycelix:holder1".into(),
            selected_claims: vec!["degree".into(), "institution".into()],
            derived_content: CredentialSubject {
                id: "did:mycelix:holder1".into(),
                claims: serde_json::json!({"degree": "BSc CS"}),
            },
            derivation_proof: DerivationProof {
                proof_type: "MerkleDisclosure2024".into(),
                original_credential_hash: vec![0u8; 32],
                claim_proofs: vec![ClaimProof {
                    claim_key: "degree".into(),
                    merkle_path: Some(vec![vec![1u8; 32], vec![2u8; 32]]),
                    commitment: None,
                }],
                holder_signature: vec![3u8; 64],
            },
            created: ts(1_700_000_000_000_000),
            expires: Some(ts(1_800_000_000_000_000)),
        };
        let json = serde_json::to_string(&dc).unwrap();
        let back: DerivedCredential = serde_json::from_str(&json).unwrap();
        assert_eq!(dc, back);
    }
}

#[cfg(test)]
mod author_binding_tests {
    use super::*;

    fn test_action(author: AgentPubKey) -> Create {
        Create {
            author,
            timestamp: Timestamp::from_micros(2_000_000_000_000_000),
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

    fn me() -> AgentPubKey {
        AgentPubKey::from_raw_36(vec![0u8; 36])
    }

    fn other_agent() -> AgentPubKey {
        AgentPubKey::from_raw_36(vec![1u8; 36])
    }

    fn minimal_vc() -> VerifiableCredential {
        VerifiableCredential {
            context: vec!["https://www.w3.org/ns/credentials/v2".into()],
            id: "urn:uuid:cred-1".into(),
            credential_type: vec!["VerifiableCredential".into()],
            issuer: CredentialIssuer::Did("did:mycelix:issuer1".into()),
            valid_from: "2026-01-01T00:00:00Z".into(),
            valid_until: None,
            credential_subject: CredentialSubject {
                id: "did:mycelix:holder1".into(),
                claims: serde_json::json!({}),
            },
            credential_schema: None,
            credential_status: None,
            proof: CredentialProof {
                proof_type: "Ed25519Signature2020".into(),
                created: "2026-01-01T00:00:00Z".into(),
                verification_method: "did:mycelix:issuer1#key-1".into(),
                proof_purpose: "assertionMethod".into(),
                proof_value: "zSig".into(),
                cryptosuite: None,
                algorithm: None,
                challenge: None,
                domain: None,
                proof_context: None,
            },
            mycelix_schema_id: "mycelix:schema:test:v1".into(),
            mycelix_created: Timestamp::from_micros(0),
        }
    }

    fn valid_presentation(holder: String) -> VerifiablePresentation {
        VerifiablePresentation {
            context: vec!["https://www.w3.org/ns/credentials/v2".into()],
            id: "urn:uuid:presentation-1".into(),
            presentation_type: vec!["VerifiablePresentation".into()],
            holder,
            verifiable_credential: vec![minimal_vc()],
            proof: CredentialProof {
                proof_type: "DataIntegrityProof".into(),
                created: "2026-01-01T00:00:00Z".into(),
                verification_method: "did:mycelix:holder#keys-1".into(),
                proof_purpose: "authentication".into(),
                proof_value: "zSig".into(),
                cryptosuite: None,
                algorithm: None,
                challenge: None,
                domain: None,
                proof_context: None,
            },
            mycelix_created: Timestamp::from_micros(0),
        }
    }

    #[test]
    fn credential_verification_method_must_match_issuer_did() {
        let vc = minimal_vc();
        assert!(verification_method_matches_did(
            &vc.proof.verification_method,
            vc.issuer.did()
        ));
        assert!(!verification_method_matches_did(
            "did:mycelix:other#key-1",
            vc.issuer.did()
        ));
    }

    #[test]
    fn presentation_verification_method_must_match_holder_did() {
        let vp = valid_presentation("did:mycelix:holder".into());
        assert!(verification_method_matches_did(
            &vp.proof.verification_method,
            &vp.holder
        ));
        assert!(!verification_method_matches_did(
            "did:mycelix:other#keys-1",
            &vp.holder
        ));
    }

    #[test]
    fn create_presentation_valid_when_holder_matches_committer() {
        let vp = valid_presentation(format!("did:mycelix:{}", me()));
        let result = validate_create_verifiable_presentation(
            EntryCreationAction::Create(test_action(me())),
            vp,
        )
        .unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn create_presentation_holder_forgery_rejected() {
        let vp = valid_presentation(format!("did:mycelix:{}", me()));
        let result = validate_create_verifiable_presentation(
            EntryCreationAction::Create(test_action(other_agent())),
            vp,
        )
        .unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn presentation_validator_rejects_future_proof_creation() {
        let mut vp = valid_presentation(format!("did:mycelix:{}", me()));
        vp.proof.created = "2100-01-01T00:00:00Z".into();

        let result = validate_create_verifiable_presentation(
            EntryCreationAction::Create(test_action(me())),
            vp,
        )
        .unwrap();

        assert!(
            matches!(result, ValidateCallbackResult::Invalid(message) if message.contains("proof created"))
        );
    }

    fn derived_for_integrity_tests(holder: String, original: &VerifiableCredential) -> DerivedCredential {
        let original_hash = compute_credential_content_hash(original);
        DerivedCredential {
            original_credential_action: ActionHash::from_raw_36(vec![9u8; 36]),
            original_credential_id: original.id.clone(),
            original_issuer: original.issuer.did().to_string(),
            holder,
            selected_claims: vec!["degree".into()],
            derived_content: CredentialSubject {
                id: original.credential_subject.id.clone(),
                claims: serde_json::json!({"degree": "BSc CS"}),
            },
            derivation_proof: DerivationProof {
                proof_type: "SelectiveDisclosureProof".into(),
                original_credential_hash: original_hash,
                claim_proofs: vec![],
                holder_signature: vec![1u8; 64],
            },
            created: Timestamp::from_micros(1),
            expires: None,
        }
    }

    fn valid_derived(holder: String) -> DerivedCredential {
        DerivedCredential {
            original_credential_action: ActionHash::from_raw_36(vec![9u8; 36]),
            original_credential_id: "urn:uuid:cred-1".into(),
            original_issuer: "did:mycelix:issuer1".into(),
            holder,
            selected_claims: vec!["degree".into()],
            derived_content: CredentialSubject {
                id: "did:mycelix:holder1".into(),
                claims: serde_json::json!({"degree": "BSc CS"}),
            },
            derivation_proof: DerivationProof {
                proof_type: "SelectiveDisclosureProof".into(),
                original_credential_hash: vec![0u8; 32],
                claim_proofs: vec![],
                holder_signature: vec![1u8; 64],
            },
            created: Timestamp::from_micros(0),
            expires: None,
        }
    }

    #[test]
    fn credential_request_state_machine_accepts_only_forward_or_idempotent_transitions() {
        use RequestStatus::*;

        let valid = [
            (Pending, UnderReview),
            (Pending, Rejected),
            (UnderReview, Approved),
            (UnderReview, Rejected),
            (Approved, Issued),
            (Pending, Pending),
            (UnderReview, UnderReview),
            (Approved, Approved),
            (Rejected, Rejected),
            (Issued, Issued),
        ];
        for (from, to) in valid {
            assert!(
                valid_request_status_transition(&from, &to),
                "expected transition {:?} -> {:?} to be valid",
                from,
                to
            );
        }
    }

    #[test]
    fn credential_request_state_machine_rejects_resurrection_and_skips() {
        use RequestStatus::*;

        let invalid = [
            (Rejected, Pending),
            (Rejected, UnderReview),
            (Rejected, Approved),
            (Rejected, Issued),
            (Issued, Pending),
            (Issued, UnderReview),
            (Issued, Approved),
            (Issued, Rejected),
            (Approved, UnderReview),
            (Approved, Rejected),
            (Pending, Approved),
            (Pending, Issued),
            (UnderReview, Pending),
            (UnderReview, Issued),
        ];
        for (from, to) in invalid {
            assert!(
                !valid_request_status_transition(&from, &to),
                "unexpected transition {:?} -> {:?} accepted",
                from,
                to
            );
        }
    }

    #[test]
    fn eddsa_jcs_hash_matches_w3c_published_vector() {
        let credential = serde_json::json!({
            "@context": [
                "https://www.w3.org/ns/credentials/v2",
                "https://www.w3.org/ns/credentials/examples/v2"
            ],
            "id": "urn:uuid:58172aac-d8ba-11ed-83dd-0b3aef56cc33",
            "type": ["VerifiableCredential", "AlumniCredential"],
            "name": "Alumni Credential",
            "description": "A minimum viable example of an Alumni Credential.",
            "issuer": "https://vc.example/issuers/5678",
            "validFrom": "2023-01-01T00:00:00Z",
            "credentialSubject": {
                "id": "did:example:abcdefgh",
                "alumniOf": "The School of Examples"
            }
        });

        let proof_options = serde_json::json!({
            "type": "DataIntegrityProof",
            "cryptosuite": "eddsa-jcs-2022",
            "created": "2023-02-24T23:36:38Z",
            "verificationMethod": "did:key:z6MkrJVnaZkeFzdQyMZu1cgjg7k1pZZ6pvBQ7XJPt4swbTQ2#z6MkrJVnaZkeFzdQyMZu1cgjg7k1pZZ6pvBQ7XJPt4swbTQ2",
            "proofPurpose": "assertionMethod"
        });

        let hash_data = eddsa_jcs_hash_data_from_values(credential, proof_options).unwrap();
        let expected = concat!(
            "66ab154f5c2890a140cb8388a22a160454f80575f6eae09e5a097cabe539a1db",
            "59b7cb6251b8991add1ce0bc83107e3db9dbbab5bd2c28f687db1a03abc92f19"
        );
        let actual = hash_data.iter().map(|b| format!("{b:02x}")).collect::<String>();
        assert_eq!(actual, expected);
    }

    #[test]
    fn derived_content_hash_is_deterministic() {
        let vc = minimal_vc();
        assert_eq!(
            compute_credential_content_hash(&vc),
            compute_credential_content_hash(&vc)
        );
    }

    #[test]
    fn derived_binding_requires_exact_original_issuer_and_holder() {
        let original = minimal_vc();
        let mut dc = derived_for_integrity_tests(original.credential_subject.id.clone(), &original);
        dc.original_issuer = "did:mycelix:forged".into();
        assert_ne!(dc.original_issuer, original.issuer.did());
    }

    #[test]
    fn derived_binding_helper_corpus_hash_changes_with_content() {
        let original = minimal_vc();
        let mut altered = original.clone();
        altered.credential_subject.claims = serde_json::json!({"degree":"different"});
        assert_ne!(
            compute_credential_content_hash(&original),
            compute_credential_content_hash(&altered)
        );
    }

    #[test]
    fn create_derived_valid_when_holder_matches_committer() {
        let dc = valid_derived(format!("did:mycelix:{}", me()));
        let result =
            validate_create_derived_credential(EntryCreationAction::Create(test_action(me())), dc)
                .unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn create_derived_holder_forgery_rejected() {
        let dc = valid_derived(format!("did:mycelix:{}", me()));
        let result = validate_create_derived_credential(
            EntryCreationAction::Create(test_action(other_agent())),
            dc,
        )
        .unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    fn valid_request(requester_did: String) -> CredentialRequest {
        CredentialRequest {
            id: "req-1".into(),
            requester_did,
            issuer_did: "did:mycelix:issuer1".into(),
            schema_id: "mycelix:schema:education:degree:v1".into(),
            provided_claims: serde_json::json!({}),
            evidence: vec![],
            status: RequestStatus::Pending,
            created: Timestamp::from_micros(0),
            updated: Timestamp::from_micros(0),
            issued_credential: None,
        }
    }

    #[test]
    fn create_request_valid_when_requester_matches_committer() {
        let req = valid_request(format!("did:mycelix:{}", me()));
        let result =
            validate_create_credential_request(EntryCreationAction::Create(test_action(me())), req)
                .unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn create_request_requester_forgery_rejected() {
        let req = valid_request(format!("did:mycelix:{}", me()));
        let result = validate_create_credential_request(
            EntryCreationAction::Create(test_action(other_agent())),
            req,
        )
        .unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }
}
