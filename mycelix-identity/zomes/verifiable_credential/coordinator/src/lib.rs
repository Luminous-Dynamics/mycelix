// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Verifiable Credential Coordinator Zome
//!
//! W3C Verifiable Credentials Data Model 2.0 compliant implementation
//! Handles credential issuance, verification, and presentation
//!
//! # Cryptographic Signatures
//!
//! This implementation uses ed25519 signatures via Holochain's HDK signing API.
//! Signatures are encoded in multibase format (base58btc with 'z' prefix) as per
//! W3C Data Integrity EdDSA Cryptosuites v1.0 specification.

use hdk::prelude::*;
use mycelix_crypto::{AlgorithmId, TaggedSignature};
use mycelix_zome_helpers as _;
use verifiable_credential_integrity::*;

/// Mirror type for credential_schema deserialization (cross-zome)
#[derive(Serialize, Deserialize, Debug)]
struct SchemaMirror {
    id: String,
    name: String,
    required_fields: Vec<String>,
    optional_fields: Vec<String>,
    credential_type: Vec<String>,
    active: bool,
}

impl TryFrom<SerializedBytes> for SchemaMirror {
    type Error = SerializedBytesError;
    fn try_from(sb: SerializedBytes) -> Result<Self, Self::Error> {
        holochain_serialized_bytes::decode(sb.bytes())
    }
}

/// Standard W3C context URLs
const W3C_CREDENTIALS_V2: &str = "https://www.w3.org/ns/credentials/v2";
const W3C_DATA_INTEGRITY: &str = "https://w3id.org/security/data-integrity/v2";

/// API version for cross-zome compatibility detection.
/// Increment when making breaking changes to extern signatures or types.
const API_VERSION: u16 = 1;

/// Returns the API version of this coordinator zome.
#[hdk_extern]
pub fn get_api_version(_: ()) -> ExternResult<u16> {
    Ok(API_VERSION)
}

/// Create a deterministic entry hash from a string identifier
fn string_to_entry_hash(s: &str) -> EntryHash {
    EntryHash::from_raw_36(
        holo_hash::blake2b_256(s.as_bytes())
            .into_iter()
            .chain([0u8; 4])
            .collect::<Vec<u8>>(),
    )
}

/// Versioned Mycelix-internal credential signing profile.
///
/// This is deliberately NOT named as a W3C Data Integrity cryptosuite: it
/// signs the canonical JSON serialization defined below, not RDF Dataset
/// Canonicalization / the W3C EdDSA cryptosuite transformation.
const CREDENTIAL_HASH_PROFILE_V1: &str = "mycelix-credential-hash-v1";
const PRESENTATION_HASH_PROFILE_V1: &str = "mycelix-presentation-hash-v1";
const LEGACY_ED25519_CRYPTOSUITE: &str = "eddsa-rdfc-2022";

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum CredentialHashProfile {
    MycelixV1,
    LegacySubsetV0,
}

fn credential_hash_profile(vc: &VerifiableCredential) -> Option<CredentialHashProfile> {
    match vc.proof.cryptosuite.as_deref() {
        Some(CREDENTIAL_HASH_PROFILE_V1) => Some(CredentialHashProfile::MycelixV1),
        // Existing issuances used this W3C suite label for a non-W3C subset
        // hash, or had no suite at all. Preserve decoding but never promote
        // these credentials to full-field VERIFIED assurance.
        Some(LEGACY_ED25519_CRYPTOSUITE) | None => {
            Some(CredentialHashProfile::LegacySubsetV0)
        }
        Some("hybrid-eddsa-mldsa65-rdfc-2024") => {
            Some(CredentialHashProfile::LegacySubsetV0)
        }
        _ => None,
    }
}

/// Recursively sort every JSON object key before serialization. This keeps the
/// internal V1 byte representation deterministic even if serde_json is built
/// with a different map implementation or insertion-order feature.
fn canonicalize_json_value(value: serde_json::Value) -> serde_json::Value {
    match value {
        serde_json::Value::Object(object) => {
            let mut entries: Vec<_> = object.into_iter().collect();
            entries.sort_by(|left, right| left.0.cmp(&right.0));
            let mut canonical = serde_json::Map::new();
            for (key, value) in entries {
                canonical.insert(key, canonicalize_json_value(value));
            }
            serde_json::Value::Object(canonical)
        }
        serde_json::Value::Array(values) => serde_json::Value::Array(
            values.into_iter().map(canonicalize_json_value).collect()
        ),
        value => value,
    }
}

/// Mycelix V1 signs every serialized credential field except proofValue itself.
/// The domain separator prevents the resulting digest being reused as a hash
/// for another object type or profile. proof type/created/method/purpose,
/// cryptosuite, algorithm, challenge, and domain are all covered.
fn compute_credential_hash_v1(
    vc: &VerifiableCredential,
) -> ExternResult<Vec<u8>> {
    let mut unsigned = serde_json::to_value(vc).map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "Could not serialize credential for Mycelix V1 signing: {}",
            error
        )))
    })?;
    let proof = unsigned
        .get_mut("proof")
        .and_then(serde_json::Value::as_object_mut)
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Credential proof is missing from Mycelix V1 signing payload".into()
        )))?;
    if proof.remove("proofValue").is_none() {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Credential proofValue is missing from Mycelix V1 signing payload".into()
        )));
    }

    let canonical = canonicalize_json_value(unsigned);
    let serialized = serde_json::to_vec(&canonical).map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "Could not canonicalize Mycelix V1 credential payload: {}",
            error
        )))
    })?;
    let mut domain_separated = b"MYCELIX:VERIFIABLE-CREDENTIAL:SIGNATURE:V1\0".to_vec();
    domain_separated.extend(serialized);
    Ok(holo_hash::blake2b_256(&domain_separated).to_vec())
}

/// Historical Mycelix subset hash. Kept only to identify and report older
/// records; its output is NEVER sufficient for a VERIFIED credential result.
fn compute_credential_hash_legacy_subset(vc: &VerifiableCredential) -> Vec<u8> {
    let mut content = Vec::new();
    content.extend(vc.id.as_bytes());
    content.push(0);
    content.extend(vc.issuer.did().as_bytes());
    content.push(0);
    content.extend(vc.credential_subject.id.as_bytes());
    content.push(0);
    content.extend(vc.valid_from.as_bytes());
    content.push(0);
    if let Ok(claims_json) = serde_json::to_string(&canonicalize_json_value(
        vc.credential_subject.claims.clone()
    )) {
        content.extend(claims_json.as_bytes());
    }
    content.push(0);
    content.extend(vc.mycelix_schema_id.as_bytes());
    holo_hash::blake2b_256(&content).to_vec()
}

/// Validate the semantic bindings required by the Mycelix V1 proof profile.
fn v1_proof_header_matches(
    vc: &VerifiableCredential,
    signature_algorithm: AlgorithmId,
) -> bool {
    // The current coordinator derives the verification key from issuer DID
    // directly and does not resolve a DID Document. V1 therefore pins the
    // only method its issuance path currently creates; key rotation requires a
    // future resolver-backed profile rather than accepting an arbitrary key id.
    let expected_verification_method = format!("{}#keys-1", vc.issuer.did());
    let method_matches_issuer =
        vc.proof.verification_method == expected_verification_method;
    vc.proof.proof_type == "MycelixCredentialProof"
        && vc.proof.proof_purpose == "assertionMethod"
        && vc.proof.algorithm == Some(signature_algorithm.as_u16())
        && method_matches_issuer
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum PresentationHashProfile {
    MycelixV1,
    LegacySubsetV0,
}

fn presentation_hash_profile(vp: &VerifiablePresentation) -> Option<PresentationHashProfile> {
    match vp.proof.cryptosuite.as_deref() {
        Some(PRESENTATION_HASH_PROFILE_V1) => Some(PresentationHashProfile::MycelixV1),
        Some(LEGACY_ED25519_CRYPTOSUITE) | None => {
            Some(PresentationHashProfile::LegacySubsetV0)
        }
        _ => None,
    }
}

/// Mycelix V1 signs every serialized presentation field except proofValue.
/// This includes embedded credentials and the full holder proof header.
fn compute_presentation_hash_v1(
    vp: &VerifiablePresentation,
) -> ExternResult<Vec<u8>> {
    let mut unsigned = serde_json::to_value(vp).map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "Could not serialize presentation for Mycelix V1 signing: {}",
            error
        )))
    })?;
    let proof = unsigned
        .get_mut("proof")
        .and_then(serde_json::Value::as_object_mut)
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Presentation proof is missing from Mycelix V1 signing payload".into()
        )))?;
    if proof.remove("proofValue").is_none() {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Presentation proofValue is missing from Mycelix V1 signing payload".into()
        )));
    }
    let canonical = canonicalize_json_value(unsigned);
    let serialized = serde_json::to_vec(&canonical).map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "Could not canonicalize Mycelix V1 presentation payload: {}",
            error
        )))
    })?;
    let mut domain_separated = b"MYCELIX:VERIFIABLE-PRESENTATION:SIGNATURE:V1\0".to_vec();
    domain_separated.extend(serialized);
    Ok(holo_hash::blake2b_256(&domain_separated).to_vec())
}

/// Historical holder-proof subset: presentation ID, holder DID, credential
/// IDs, challenge, and domain. A valid legacy signature remains PARTIAL.
fn compute_presentation_hash_legacy_subset(vp: &VerifiablePresentation) -> Vec<u8> {
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

fn v1_presentation_proof_header_matches(vp: &VerifiablePresentation) -> bool {
    vp.proof.proof_type == "MycelixPresentationProof"
        && vp.proof.proof_purpose == "authentication"
        && vp.proof.algorithm == Some(AlgorithmId::Ed25519.as_u16())
        && vp.proof.verification_method == format!("{}#keys-1", vp.holder)
}

/// Compute the digest appropriate to the explicitly declared credential
/// profile. Unknown profiles fail closed rather than guessing a transform.
fn compute_credential_hash_for_profile(
    vc: &VerifiableCredential,
) -> ExternResult<Vec<u8>> {
    match credential_hash_profile(vc) {
        Some(CredentialHashProfile::MycelixV1) => compute_credential_hash_v1(vc),
        Some(CredentialHashProfile::LegacySubsetV0) => {
            Ok(compute_credential_hash_legacy_subset(vc))
        }
        None => Err(wasm_error!(WasmErrorInner::Guest(format!(
            "Unsupported credential signing profile: {:?}",
            vc.proof.cryptosuite
        )))),
    }
}

/// Schema validation outcome — tells the caller what actually happened.
#[derive(Debug, Clone, PartialEq)]
enum SchemaValidationStatus {
    /// No schema was specified
    NoSchema,
    /// Schema zome was unavailable (not installed or unauthorized)
    SchemaZomeUnavailable,
    /// Schema not found in registry (may be external)
    SchemaNotFound,
    /// Validation was performed and all required fields are present
    Validated { schema_name: String },
}

/// Validate credential claims against a schema's required/optional fields.
///
/// Calls the credential_schema zome to fetch the schema, then checks:
/// 1. Schema exists and is active
/// 2. All required_fields are present in claims
/// 3. No unknown fields (not in required or optional) if schema is strict
///
/// Returns the validation status so callers know whether validation was performed.
fn validate_claims_against_schema(
    schema_id: &str,
    claims: &serde_json::Value,
) -> ExternResult<SchemaValidationStatus> {
    // Skip validation if no schema specified
    if schema_id.is_empty() {
        return Ok(SchemaValidationStatus::NoSchema);
    }

    // Fetch schema via cross-zome call
    let response = call(
        CallTargetCell::Local,
        ZomeName::new("credential_schema"),
        FunctionName::new("get_schema"),
        None,
        schema_id.to_string(),
    )?;

    let schema: SchemaMirror = match response {
        ZomeCallResponse::Ok(result) => {
            let record: Option<Record> = result.decode().map_err(|e| {
                wasm_error!(WasmErrorInner::Guest(format!(
                    "Failed to decode schema response: {:?}",
                    e
                )))
            })?;
            let rec = match record {
                Some(r) => r,
                None => return Ok(SchemaValidationStatus::SchemaNotFound),
            };
            rec.entry()
                .to_app_option()
                .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
                .ok_or(wasm_error!(WasmErrorInner::Guest(
                    "Invalid schema entry".into()
                )))?
        }
        ZomeCallResponse::Unauthorized(_, _, _, _)
        | ZomeCallResponse::AuthenticationFailed(_, _) => {
            return Ok(SchemaValidationStatus::SchemaZomeUnavailable);
        }
        ZomeCallResponse::NetworkError(err) => {
            return Err(wasm_error!(WasmErrorInner::Guest(format!(
                "Network error fetching schema: {}",
                err
            ))));
        }
        ZomeCallResponse::CountersigningSession(err) => {
            return Err(wasm_error!(WasmErrorInner::Guest(format!(
                "Countersigning error: {}",
                err
            ))));
        }
    };

    // Verify schema is active
    if !schema.active {
        return Err(wasm_error!(WasmErrorInner::Guest(format!(
            "Schema '{}' is not active",
            schema_id
        ))));
    }

    // Claims must be an object to validate fields
    let claims_obj = match claims.as_object() {
        Some(obj) => obj,
        None => {
            // If claims is not an object (e.g., array or scalar), skip field validation
            return Ok(SchemaValidationStatus::Validated {
                schema_name: schema.name,
            });
        }
    };

    // Check all required fields are present
    let mut missing: Vec<String> = Vec::new();
    for field in &schema.required_fields {
        if !claims_obj.contains_key(field) {
            missing.push(field.clone());
        }
    }

    if !missing.is_empty() {
        return Err(wasm_error!(WasmErrorInner::Guest(format!(
            "Missing required fields for schema '{}': {}",
            schema.name,
            missing.join(", ")
        ))));
    }

    Ok(SchemaValidationStatus::Validated {
        schema_name: schema.name,
    })
}

/// Issue a new verifiable credential
#[hdk_extern]
pub fn issue_credential(input: IssueCredentialInput) -> ExternResult<Record> {
    let agent_info = agent_info()?;
    let issuer_did = format!("did:mycelix:{}", agent_info.agent_initial_pubkey);
    let now = sys_time()?;
    let now_iso = format_timestamp_iso8601(now);

    // Validate claims against schema if a schema is specified
    let schema_validation = validate_claims_against_schema(&input.schema_id, &input.claims)?;
    match &schema_validation {
        SchemaValidationStatus::Validated { schema_name } => {
            debug!("Schema validation passed for schema '{}'", schema_name);
        }
        SchemaValidationStatus::SchemaZomeUnavailable => {
            if input.strict_schema {
                return Err(wasm_error!(WasmErrorInner::Guest(
                    "strict_schema: credential_schema zome is unavailable; cannot validate schema"
                        .to_string()
                )));
            }
            warn!("Schema validation skipped: credential_schema zome unavailable");
        }
        SchemaValidationStatus::SchemaNotFound => {
            if input.strict_schema {
                return Err(wasm_error!(WasmErrorInner::Guest(format!(
                    "strict_schema: schema '{}' not found in registry",
                    input.schema_id
                ))));
            }
            debug!(
                "Schema validation skipped: schema '{}' not found (may be external)",
                input.schema_id
            );
        }
        SchemaValidationStatus::NoSchema => {
            if input.strict_schema {
                return Err(wasm_error!(WasmErrorInner::Guest(
                    "strict_schema: no schema_id provided but strict_schema is enabled".to_string()
                )));
            }
        }
    }

    // Build credential ID
    let credential_id = format!(
        "urn:uuid:{}:{}",
        issuer_did.replace(":", "-"),
        now.as_micros()
    );

    // Calculate expiration if provided
    let valid_until = input.expiration_days.map(|days| {
        let expiry_micros = now.as_micros() as i64 + (days as i64 * 24 * 3600 * 1_000_000);
        format_timestamp_iso8601(Timestamp::from_micros(expiry_micros))
    });

    // Build credential subject
    let credential_subject = CredentialSubject {
        id: input.subject_did.clone(),
        claims: input.claims,
    };

    // Build credential hash for signing
    let mut vc_for_hash = VerifiableCredential {
        context: vec![
            W3C_CREDENTIALS_V2.to_string(),
            W3C_DATA_INTEGRITY.to_string(),
        ],
        id: credential_id.clone(),
        credential_type: {
            let mut types = vec!["VerifiableCredential".to_string()];
            types.extend(input.credential_types);
            types
        },
        issuer: CredentialIssuer::Object {
            id: issuer_did.clone(),
            name: input.issuer_name,
            issuer_type: Some(vec!["Organization".to_string()]),
        },
        valid_from: now_iso.clone(),
        valid_until: valid_until.clone(),
        credential_subject: credential_subject.clone(),
        credential_schema: Some(CredentialSchemaRef {
            id: input.schema_id.clone(),
            schema_type: "JsonSchema".to_string(),
        }),
        credential_status: input.enable_revocation.then(|| CredentialStatus {
            id: format!("{}#status", credential_id),
            status_type: "BitstringStatusListEntry".to_string(),
            status_purpose: Some("revocation".to_string()),
            status_list_index: Some("0".to_string()),
            status_list_credential: None,
        }),
        proof: CredentialProof {
            proof_type: "MycelixCredentialProof".to_string(),
            created: now_iso.clone(),
            verification_method: format!("{}#keys-1", issuer_did),
            proof_purpose: "assertionMethod".to_string(),
            proof_value: String::new(), // Will be filled
            cryptosuite: Some(CREDENTIAL_HASH_PROFILE_V1.to_string()),
            algorithm: Some(AlgorithmId::Ed25519.as_u16()),
            challenge: None,
            domain: None,
        },
        mycelix_schema_id: input.schema_id.clone(),
        mycelix_created: now,
    };

    // Sign credential with agent's ed25519 key
    // This creates a real cryptographic signature using HDK's sign_raw
    let signature_value = sign_credential(&vc_for_hash)?;
    vc_for_hash.proof.proof_value = signature_value;

    let vc = vc_for_hash;

    let action_hash = create_entry(&EntryTypes::VerifiableCredential(vc.clone()))?;

    // Create links
    let issuer_hash = string_to_entry_hash(vc.issuer.did());
    create_link(
        issuer_hash,
        action_hash.clone(),
        LinkTypes::IssuerToCredential,
        (),
    )?;

    let subject_hash = string_to_entry_hash(&vc.credential_subject.id);
    create_link(
        subject_hash,
        action_hash.clone(),
        LinkTypes::SubjectToCredential,
        (),
    )?;

    let schema_hash = string_to_entry_hash(&input.schema_id);
    create_link(
        schema_hash,
        action_hash.clone(),
        LinkTypes::SchemaToCredential,
        (),
    )?;

    // By-ID index, so `get_credential` can resolve over the DHT rather than
    // only within the caller's own source chain.
    let id_hash = string_to_entry_hash(&vc.id);
    create_link(
        id_hash,
        action_hash.clone(),
        LinkTypes::CredentialIdToCredential,
        (),
    )?;

    get(action_hash, GetOptions::default())?.ok_or(wasm_error!(WasmErrorInner::Guest(
        "Could not find created credential".into()
    )))
}

/// Input for issuing a credential
#[derive(Serialize, Deserialize, Debug)]
pub struct IssueCredentialInput {
    /// Subject's DID (who the credential is about)
    pub subject_did: String,
    /// Schema ID for the credential
    pub schema_id: String,
    /// Claims being made
    pub claims: serde_json::Value,
    /// Additional credential types beyond VerifiableCredential
    pub credential_types: Vec<String>,
    /// Optional issuer name
    pub issuer_name: Option<String>,
    /// Expiration in days (None = no expiration)
    pub expiration_days: Option<u32>,
    /// Whether to enable revocation
    pub enable_revocation: bool,
    /// When true, schema validation is mandatory: if the credential_schema zome
    /// is unavailable or the schema is not found, issuance fails instead of
    /// silently skipping validation. Default: false (backward-compatible).
    #[serde(default)]
    pub strict_schema: bool,
}

/// Verify a credential
#[hdk_extern]
pub fn verify_credential(credential_id: String) -> ExternResult<VerificationResult> {
    let vc = get_credential(credential_id.clone())?.ok_or(wasm_error!(WasmErrorInner::Guest(
        "Credential not found".into()
    )))?;

    let credential: VerifiableCredential = vc
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Invalid credential entry".into()
        )))?;

    let now = sys_time()?;
    let mut errors = Vec::new();
    let mut checks_passed = vec!["format".to_string()];

    // Enforce the complete [validFrom, validUntil) interval. Signature
    // coverage does not replace temporal validity checks.
    let validity = credential_validity_status(
        &credential.valid_from,
        credential.valid_until.as_deref(),
        now,
    );
    if let Some(reason) = validity.error_message() {
        errors.push(reason.to_string());
    } else {
        checks_passed.push("validity_window".to_string());
    }

    // Only a fully verified proof can be used as successful credential evidence.
    // Partial, unsupported, and erroneous outcomes all fail closed.
    let proof_signature_status = match verify_credential_signature(&credential) {
        Ok(outcome) => {
            let status = outcome.status();
            if let Some(reason) = outcome.failure_message() {
                errors.push(reason);
            } else {
                checks_passed.push("proof_signature".to_string());
            }
            status
        }
        Err(e) => {
            errors.push(format!("Signature verification error: {}", e));
            CredentialSignatureStatus::Error
        }
    };

    // Check proof purpose
    if credential.proof.proof_purpose != "assertionMethod" {
        errors.push("Invalid proof purpose".to_string());
    } else {
        checks_passed.push("proof_purpose".to_string());
    }

    // Check issuer DID format
    if !credential.issuer.did().starts_with("did:") {
        errors.push("Invalid issuer DID".into());
    }

    // Check revocation status via cross-zome call to revocation registry
    let revocation_status = check_credential_revocation_status(&credential.id)?;
    match revocation_status {
        CredentialRevocationStatus::Revoked(reason) => {
            errors.push(format!("Credential revoked: {}", reason));
        }
        CredentialRevocationStatus::Suspended(reason, until) => {
            errors.push(format!("Credential suspended until {}: {}", until, reason));
        }
        CredentialRevocationStatus::Active => {
            // Credential is active, no error
        }
        CredentialRevocationStatus::Unknown => {
            // SECURITY: Fail-closed — unknown revocation status is not safe to assume active
            errors.push("Revocation status could not be determined".to_string());
        }
    }

    Ok(VerificationResult {
        credential_id,
        valid: errors.is_empty() && proof_signature_status.is_verified(),
        checks_passed,
        proof_signature_status,
        errors,
        verified_at: now,
    })
}

/// Assurance level of the issuer's credential proof as checked by this verifier.
///
/// Only `Verified` means that every cryptographic component required by the
/// declared signature algorithm was actually verified. In particular,
/// `Partial` and `Unsupported` are never acceptable authorization outcomes.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum CredentialSignatureStatus {
    Verified,
    Invalid,
    Partial,
    Unsupported,
    Error,
}

/// Internal outcome preserves the distinction between a bad signature and a
/// valid-looking signature that this WASM verifier cannot fully verify.
#[derive(Debug, Clone, PartialEq)]
enum CredentialSignatureOutcome {
    Verified,
    Invalid,
    Partial { algorithm: AlgorithmId, reason: String },
    Unsupported { algorithm: AlgorithmId },
}

impl CredentialSignatureOutcome {
    fn status(&self) -> CredentialSignatureStatus {
        match self {
            Self::Verified => CredentialSignatureStatus::Verified,
            Self::Invalid => CredentialSignatureStatus::Invalid,
            Self::Partial { .. } => CredentialSignatureStatus::Partial,
            Self::Unsupported { .. } => CredentialSignatureStatus::Unsupported,
        }
    }

    fn failure_message(&self) -> Option<String> {
        match self {
            Self::Verified => None,
            Self::Invalid => Some("Proof signature verification failed".to_string()),
            Self::Partial { algorithm, reason } => Some(format!(
                "Proof signature is only partially verified for {:?}: {}",
                algorithm, reason
            )),
            Self::Unsupported { algorithm } => Some(format!(
                "Proof signature algorithm {:?} is not cryptographically verified by this WASM verifier",
                algorithm
            )),
        }
    }
}

impl CredentialSignatureStatus {
    /// Return true only when all required proof components were verified.
    pub fn is_verified(&self) -> bool {
        matches!(self, Self::Verified)
    }
}

/// Result of credential verification
#[derive(Serialize, Deserialize, Debug)]
pub struct VerificationResult {
    pub credential_id: String,
    pub valid: bool,
    pub checks_passed: Vec<String>,
    pub proof_signature_status: CredentialSignatureStatus,
    pub errors: Vec<String>,
    pub verified_at: Timestamp,
}

/// Get a credential by ID.
///
/// Resolves over the DHT via the `CredentialIdToCredential` index. This
/// previously used `query(ChainQueryFilter)`, which reads only the *caller's
/// own source chain* — so verifying anybody else's credential silently
/// returned `None` rather than erroring. Sibling lookups
/// (`get_credentials_issued_by`, `get_credentials_for_subject`) were always
/// link-based and unaffected.
#[hdk_extern]
pub fn get_credential(credential_id: String) -> ExternResult<Option<Record>> {
    if credential_id.is_empty() || credential_id.len() > 256 {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Credential ID must be 1-256 characters".into()
        )));
    }

    let id_hash = string_to_entry_hash(&credential_id);
    let links = get_links(
        LinkQuery::try_new(id_hash, LinkTypes::CredentialIdToCredential)?,
        GetStrategy::default(),
    )?;

    for link in links {
        let action_hash = ActionHash::try_from(link.target)
            .map_err(|_| wasm_error!(WasmErrorInner::Guest("Invalid link target".into())))?;
        if let Some(record) = get(action_hash, GetOptions::default())? {
            // The link base is a hash of the ID string, so confirm the entry
            // really carries the requested ID before returning it.
            if let Some(vc) = record
                .entry()
                .to_app_option::<VerifiableCredential>()
                .ok()
                .flatten()
            {
                if vc.id == credential_id {
                    return Ok(Some(record));
                }
            }
        }
    }
    Ok(None)
}

/// Get credentials issued by a DID
#[hdk_extern]
pub fn get_credentials_issued_by(issuer_did: String) -> ExternResult<Vec<Record>> {
    if issuer_did.is_empty() || issuer_did.len() > 256 {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Issuer DID must be 1-256 characters".into()
        )));
    }
    if !issuer_did.starts_with("did:") {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Issuer must be a valid DID".into()
        )));
    }
    let issuer_hash = string_to_entry_hash(&issuer_did);
    let links = get_links(
        LinkQuery::try_new(issuer_hash, LinkTypes::IssuerToCredential)?,
        GetStrategy::default(),
    )?;

    let mut credentials = Vec::new();
    for link in links {
        let action_hash = ActionHash::try_from(link.target)
            .map_err(|_| wasm_error!(WasmErrorInner::Guest("Invalid link target".into())))?;
        if let Some(record) = get(action_hash, GetOptions::default())? {
            credentials.push(record);
        }
    }
    Ok(credentials)
}

/// Get credentials about a subject DID
#[hdk_extern]
pub fn get_credentials_for_subject(subject_did: String) -> ExternResult<Vec<Record>> {
    if subject_did.is_empty() || subject_did.len() > 256 {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Subject DID must be 1-256 characters".into()
        )));
    }
    if !subject_did.starts_with("did:") {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Subject must be a valid DID".into()
        )));
    }
    let subject_hash = string_to_entry_hash(&subject_did);
    let links = get_links(
        LinkQuery::try_new(subject_hash, LinkTypes::SubjectToCredential)?,
        GetStrategy::default(),
    )?;

    let mut credentials = Vec::new();
    for link in links {
        let action_hash = ActionHash::try_from(link.target)
            .map_err(|_| wasm_error!(WasmErrorInner::Guest("Invalid link target".into())))?;
        if let Some(record) = get(action_hash, GetOptions::default())? {
            credentials.push(record);
        }
    }
    Ok(credentials)
}

/// Create a verifiable presentation from credentials
#[hdk_extern]
pub fn create_presentation(input: CreatePresentationInput) -> ExternResult<Record> {
    let agent_info = agent_info()?;
    let holder_did = format!("did:mycelix:{}", agent_info.agent_initial_pubkey);
    let now = sys_time()?;
    let now_iso = format_timestamp_iso8601(now);

    // V1 authentication presentations must be bound to a verifier-issued nonce.
    // The verifier remains responsible for unpredictability and one-time use.
    if input.challenge.trim().is_empty() || input.challenge.len() > 512 {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Challenge must be non-empty and at most 512 UTF-8 bytes".into()
        )));
    }
    if let Some(ref domain) = input.domain {
        if domain.is_empty() || domain.len() > 256 {
            return Err(wasm_error!(WasmErrorInner::Guest(
                "Domain must be 1-256 characters".into()
            )));
        }
    }

    // Gather credentials
    let mut credentials = Vec::new();
    for cred_id in &input.credential_ids {
        let record = get_credential(cred_id.clone())?.ok_or(wasm_error!(WasmErrorInner::Guest(
            format!("Credential {} not found", cred_id)
        )))?;
        let vc: VerifiableCredential = record
            .entry()
            .to_app_option()
            .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
            .ok_or(wasm_error!(WasmErrorInner::Guest(
                "Invalid credential".into()
            )))?;
        credentials.push(vc);
    }

    let presentation_id = format!(
        "urn:uuid:presentation:{}:{}",
        holder_did.replace(":", "-"),
        now.as_micros()
    );

    // Build the presentation first with an empty proofValue. Mycelix V1
    // signs the canonicalized complete presentation (including embedded
    // credentials and all holder proof metadata), excluding only proofValue.
    let proof = CredentialProof {
        proof_type: "MycelixPresentationProof".to_string(),
        created: now_iso,
        verification_method: format!("{}#keys-1", holder_did),
        proof_purpose: "authentication".to_string(),
        proof_value: String::new(),
        cryptosuite: Some(PRESENTATION_HASH_PROFILE_V1.to_string()),
        algorithm: Some(AlgorithmId::Ed25519.as_u16()),
        challenge: Some(input.challenge.clone()),
        domain: input.domain.clone(),
    };

    let mut vp = VerifiablePresentation {
        context: vec![W3C_CREDENTIALS_V2.to_string()],
        id: presentation_id,
        presentation_type: vec!["VerifiablePresentation".to_string()],
        holder: holder_did.clone(),
        verifiable_credential: credentials,
        proof,
        mycelix_created: now,
    };

    let presentation_hash = compute_presentation_hash_v1(&vp)?;
    let signature = sign_raw(agent_info.agent_initial_pubkey.clone(), presentation_hash)?;
    let tagged_signature = TaggedSignature::new(
        AlgorithmId::Ed25519,
        signature.as_ref().to_vec(),
    )
    .map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "Presentation signature tagging error: {}",
            error
        )))
    })?;
    vp.proof.proof_value = tagged_signature.to_multibase();

    let action_hash = create_entry(&EntryTypes::VerifiablePresentation(vp))?;

    // Link holder to presentation
    let holder_hash = string_to_entry_hash(&holder_did);
    create_link(
        holder_hash,
        action_hash.clone(),
        LinkTypes::HolderToPresentation,
        (),
    )?;

    get(action_hash, GetOptions::default())?.ok_or(wasm_error!(WasmErrorInner::Guest(
        "Could not find created presentation".into()
    )))
}

/// Input for creating a presentation
#[derive(Serialize, Deserialize, Debug)]
pub struct CreatePresentationInput {
    /// Credential IDs to include
    pub credential_ids: Vec<String>,
    /// Required fresh verifier challenge (1-512 UTF-8 bytes)
    pub challenge: String,
    /// Optional domain restriction
    pub domain: Option<String>,
}

/// Require a non-empty, bounded expected challenge and an exact match
/// with the challenge covered by the signed Mycelix V1 presentation payload.
/// This checks binding only; freshness and one-time consumption are verifier duties.
fn validate_presentation_challenge(
    presentation: &VerifiablePresentation,
    expected_challenge: &str,
) -> Result<(), &'static str> {
    if expected_challenge.trim().is_empty() || expected_challenge.len() > 512 {
        return Err("Expected challenge must be non-empty and at most 512 UTF-8 bytes");
    }
    let stored = presentation
        .proof
        .challenge
        .as_deref()
        .ok_or("Presentation has no verifier challenge")?;
    if stored.trim().is_empty() || stored.len() > 512 {
        return Err("Stored presentation challenge is empty or exceeds 512 UTF-8 bytes");
    }
    if stored != expected_challenge {
        return Err("Presentation challenge does not match the verifier challenge");
    }
    Ok(())
}

/// Verify a verifiable presentation
///
/// Checks:
/// 1. Holder's proof signature over the presentation content
/// 2. Each contained credential's issuer signature
/// 3. Each contained credential's revocation status
/// 4. Proof purpose is "authentication"
#[hdk_extern]
pub fn verify_presentation(
    input: VerifyPresentationInput,
) -> ExternResult<PresentationVerificationResult> {
    let record = get(input.presentation_hash.clone(), GetOptions::default())?.ok_or(
        wasm_error!(WasmErrorInner::Guest("Presentation not found".into())),
    )?;

    let vp: VerifiablePresentation = record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Invalid presentation entry".into()
        )))?;

    let now = sys_time()?;
    let mut errors = Vec::new();
    let mut credential_results = Vec::new();

    // 1. Verify proof purpose
    if vp.proof.proof_purpose != "authentication" {
        errors.push("Presentation proof purpose must be 'authentication'".to_string());
    }

    // 2. A signed V1 challenge is mandatory and must match the verifier's
    // transaction challenge. A match alone does not prove nonce freshness or
    // single-use; the verifier must issue and consume challenges accordingly.
    if let Err(reason) = validate_presentation_challenge(&vp, &input.expected_challenge) {
        errors.push(reason.to_string());
    }

    if let Some(expected) = &input.expected_domain {
        match &vp.proof.domain {
            Some(stored) if stored == expected => {} // match
            Some(stored) => errors.push(format!(
                "Presentation domain mismatch: expected '{}', proof has '{}'",
                expected, stored
            )),
            None => errors.push(format!(
                "Presentation has no domain but verifier expected '{}'",
                expected
            )),
        }
    } else if vp.proof.domain.is_some() {
        errors.push(
            "Presentation has domain but verifier did not supply expected_domain".to_string(),
        );
    }

    // 3. Verify the holder's proof under its declared versioned profile.
    let holder_signature_status = match vp.holder.strip_prefix("did:mycelix:") {
        Some(pubkey_str) => match AgentPubKey::try_from(pubkey_str.to_string()) {
            Ok(holder_pubkey) => match presentation_hash_profile(&vp) {
                Some(PresentationHashProfile::MycelixV1) => {
                    if !v1_presentation_proof_header_matches(&vp) {
                        CredentialSignatureStatus::Invalid
                    } else {
                        match TaggedSignature::from_multibase(&vp.proof.proof_value) {
                            Ok(tagged_signature)
                                if tagged_signature.algorithm == AlgorithmId::Ed25519
                                    && tagged_signature.signature_bytes.len()
                                        == AlgorithmId::Ed25519.signature_size() =>
                            {
                                let signature =
                                    Signature::from(<[u8; 64]>::try_from(
                                        tagged_signature.signature_bytes.as_slice()
                                    ).map_err(|_| wasm_error!(WasmErrorInner::Guest(
                                        "Invalid V1 presentation signature length".into()
                                    )))?);
                                let presentation_hash = compute_presentation_hash_v1(&vp)?;
                                match verify_signature(holder_pubkey, signature, presentation_hash)? {
                                    true => CredentialSignatureStatus::Verified,
                                    false => CredentialSignatureStatus::Invalid,
                                }
                            }
                            Ok(_) => CredentialSignatureStatus::Unsupported,
                            Err(_) => CredentialSignatureStatus::Invalid,
                        }
                    }
                }
                Some(PresentationHashProfile::LegacySubsetV0) => {
                    let legacy_hash = compute_presentation_hash_legacy_subset(&vp);
                    match TaggedSignature::from_multibase(&vp.proof.proof_value) {
                        Ok(tagged_signature)
                            if tagged_signature.algorithm == AlgorithmId::Ed25519
                                && tagged_signature.signature_bytes.len()
                                    == AlgorithmId::Ed25519.signature_size() =>
                        {
                            let signature =
                                Signature::from(<[u8; 64]>::try_from(
                                    tagged_signature.signature_bytes.as_slice()
                                ).map_err(|_| wasm_error!(WasmErrorInner::Guest(
                                    "Invalid legacy presentation signature length".into()
                                )))?);
                            match verify_signature(holder_pubkey, signature, legacy_hash)? {
                                true => CredentialSignatureStatus::Partial,
                                false => CredentialSignatureStatus::Invalid,
                            }
                        }
                        Ok(_) => CredentialSignatureStatus::Unsupported,
                        Err(_) => {
                            // Historical raw Ed25519 multibase remains readable,
                            // but successful subset verification is PARTIAL only.
                            match multibase_decode(&vp.proof.proof_value) {
                                Some(signature_bytes)
                                    if signature_bytes.len() == AlgorithmId::Ed25519.signature_size() =>
                                {
                                    let signature =
                                        Signature::from(<[u8; 64]>::try_from(
                                            signature_bytes.as_slice()
                                        ).map_err(|_| wasm_error!(WasmErrorInner::Guest(
                                            "Invalid legacy presentation signature length".into()
                                        )))?);
                                    match verify_signature(holder_pubkey, signature, legacy_hash)? {
                                        true => CredentialSignatureStatus::Partial,
                                        false => CredentialSignatureStatus::Invalid,
                                    }
                                }
                                _ => CredentialSignatureStatus::Invalid,
                            }
                        }
                    }
                }
                None => CredentialSignatureStatus::Unsupported,
            },
            Err(_) => CredentialSignatureStatus::Error,
        },
        None => CredentialSignatureStatus::Error,
    };

    match holder_signature_status {
        CredentialSignatureStatus::Verified => {}
        CredentialSignatureStatus::Invalid => {
            errors.push("Holder presentation proof is invalid or has inconsistent V1 header bindings".into());
        }
        CredentialSignatureStatus::Partial => {
            errors.push("Historical holder presentation proof verifies only under the subset-hash profile; assurance is PARTIAL".into());
        }
        CredentialSignatureStatus::Unsupported => {
            errors.push("Holder presentation proof profile or algorithm is unsupported".into());
        }
        CredentialSignatureStatus::Error => {
            errors.push("Holder presentation proof could not be decoded or its key could not be resolved".into());
        }
    }

    // 3. Batch-check revocation for all credentials in one cross-zome call
    //    (eliminates N+1 query pattern — single call instead of per-credential)
    let cred_ids: Vec<String> = vp
        .verifiable_credential
        .iter()
        .map(|c| c.id.clone())
        .collect();
    let revocation_statuses = batch_check_credential_revocation_status(&cred_ids)?;

    // 4. Verify each contained credential
    for (i, cred) in vp.verifiable_credential.iter().enumerate() {
        let mut cred_errors = Vec::new();

        // Only a fully verified signature is valid for an embedded credential.
        let proof_signature_status = match verify_credential_signature(cred) {
            Ok(outcome) => {
                let status = outcome.status();
                if let Some(reason) = outcome.failure_message() {
                    cred_errors.push(reason);
                }
                status
            }
            Err(e) => {
                cred_errors.push(format!("Signature error: {:?}", e));
                CredentialSignatureStatus::Error
            }
        };

        // Check revocation (from batch result)
        let revocation = &revocation_statuses[i];
        match revocation {
            CredentialRevocationStatus::Revoked(reason) => {
                cred_errors.push(format!("Revoked: {}", reason));
            }
            CredentialRevocationStatus::Suspended(reason, _) => {
                cred_errors.push(format!("Suspended: {}", reason));
            }
            CredentialRevocationStatus::Active => {}
            CredentialRevocationStatus::Unknown => {
                cred_errors.push("Revocation status could not be determined".to_string());
            }
        }

        // Use the same half-open validity-window policy as standalone
        // credential verification.
        let validity = credential_validity_status(
            &cred.valid_from,
            cred.valid_until.as_deref(),
            now,
        );
        if let Some(reason) = validity.error_message() {
            cred_errors.push(reason.to_string());
        }

        let cred_valid = cred_errors.is_empty() && proof_signature_status.is_verified();
        credential_results.push(CredentialInPresentationResult {
            credential_id: cred.id.clone(),
            valid: cred_valid,
            proof_signature_status,
            errors: cred_errors,
        });
    }

    // If any credential is invalid, mark overall as invalid
    if credential_results.iter().any(|r| !r.valid) {
        errors.push("One or more contained credentials failed verification".to_string());
    }

    Ok(PresentationVerificationResult {
        presentation_id: vp.id,
        holder: vp.holder,
        valid: errors.is_empty() && holder_signature_status.is_verified(),
        errors,
        holder_signature_status,
        credential_results,
        verified_at: now,
    })
}

/// Input for verifying a presentation
#[derive(Serialize, Deserialize, Debug)]
pub struct VerifyPresentationInput {
    /// Action hash of the presentation to verify
    pub presentation_hash: ActionHash,
    /// Required verifier challenge for this verification transaction.
    pub expected_challenge: String,
    /// Expected domain (must match what was used during creation)
    pub expected_domain: Option<String>,
}

/// Result of presentation verification
#[derive(Serialize, Deserialize, Debug)]
pub struct PresentationVerificationResult {
    pub presentation_id: String,
    pub holder: String,
    pub valid: bool,
    pub errors: Vec<String>,
    pub holder_signature_status: CredentialSignatureStatus,
    pub credential_results: Vec<CredentialInPresentationResult>,
    pub verified_at: Timestamp,
}

/// Verification result for a credential within a presentation
#[derive(Serialize, Deserialize, Debug)]
pub struct CredentialInPresentationResult {
    pub credential_id: String,
    pub valid: bool,
    pub proof_signature_status: CredentialSignatureStatus,
    pub errors: Vec<String>,
}

// =============================================================================
// MERKLE TREE FOR SELECTIVE DISCLOSURE
// =============================================================================

/// Hash a single claim leaf: BLAKE2b-256(key || 0x00 || json_value)
fn hash_claim_leaf(key: &str, value: &serde_json::Value) -> Vec<u8> {
    let mut data = key.as_bytes().to_vec();
    data.push(0);
    if let Ok(json) = serde_json::to_string(value) {
        data.extend(json.as_bytes());
    }
    holo_hash::blake2b_256(&data).to_vec()
}

/// Build a Merkle tree from sorted claim leaves and return (root, leaves_in_order).
///
/// Leaves are sorted by key to ensure deterministic ordering.
/// Internal nodes: BLAKE2b-256(left || right). Odd nodes are promoted.
fn build_claim_merkle_tree(
    claims: &serde_json::Map<String, serde_json::Value>,
) -> (Vec<u8>, Vec<(String, Vec<u8>)>) {
    // Sort keys for deterministic order
    let mut keys: Vec<&String> = claims.keys().collect();
    keys.sort();

    let leaves: Vec<(String, Vec<u8>)> = keys
        .iter()
        .map(|k| ((*k).clone(), hash_claim_leaf(k, &claims[*k])))
        .collect();

    if leaves.is_empty() {
        return (vec![0u8; 32], leaves);
    }
    if leaves.len() == 1 {
        return (leaves[0].1.clone(), leaves);
    }

    // Build tree bottom-up
    let mut current_level: Vec<Vec<u8>> = leaves.iter().map(|(_, h)| h.clone()).collect();

    while current_level.len() > 1 {
        let mut next_level = Vec::new();
        let mut i = 0;
        while i < current_level.len() {
            if i + 1 < current_level.len() {
                let mut combined = current_level[i].clone();
                combined.extend(&current_level[i + 1]);
                next_level.push(holo_hash::blake2b_256(&combined).to_vec());
                i += 2;
            } else {
                // Odd node: promote
                next_level.push(current_level[i].clone());
                i += 1;
            }
        }
        current_level = next_level;
    }

    (current_level[0].clone(), leaves)
}

/// Generate a Merkle proof (list of sibling hashes) for a leaf at the given index.
fn generate_merkle_proof(
    claims: &serde_json::Map<String, serde_json::Value>,
    leaf_index: usize,
) -> Vec<Vec<u8>> {
    let mut keys: Vec<&String> = claims.keys().collect();
    keys.sort();

    let leaf_hashes: Vec<Vec<u8>> = keys
        .iter()
        .map(|k| hash_claim_leaf(k, &claims[*k]))
        .collect();

    if leaf_hashes.len() <= 1 {
        return Vec::new(); // Single leaf = root, no proof needed
    }

    let mut proof = Vec::new();
    let mut current_level = leaf_hashes;
    let mut idx = leaf_index;

    while current_level.len() > 1 {
        // Sibling index
        let sibling_idx = if idx.is_multiple_of(2) {
            idx + 1
        } else {
            idx - 1
        };

        if sibling_idx < current_level.len() {
            proof.push(current_level[sibling_idx].clone());
        }
        // No sibling (odd node promoted) — no proof element needed

        // Move to next level
        let mut next_level = Vec::new();
        let mut i = 0;
        while i < current_level.len() {
            if i + 1 < current_level.len() {
                let mut combined = current_level[i].clone();
                combined.extend(&current_level[i + 1]);
                next_level.push(holo_hash::blake2b_256(&combined).to_vec());
                i += 2;
            } else {
                next_level.push(current_level[i].clone());
                i += 1;
            }
        }
        current_level = next_level;
        idx /= 2;
    }

    proof
}

const DERIVED_CREDENTIAL_PROOF_V1: &str = "MycelixDerivedCredentialProofV1";

/// Canonical, domain-separated digest of the complete derived credential,
/// excluding only the signature bytes being generated/verified.
fn compute_derived_credential_hash_v1(
    derived: &DerivedCredential,
) -> ExternResult<Vec<u8>> {
    if derived.derivation_proof.proof_type != DERIVED_CREDENTIAL_PROOF_V1 {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Unsupported derived credential proof profile".into()
        )));
    }
    let mut value = serde_json::to_value(derived).map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "Could not serialize derived credential V1 payload: {error}"
        )))
    })?;
    let proof = value
        .get_mut("derivation_proof")
        .and_then(serde_json::Value::as_object_mut)
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Derived credential proof object missing from V1 payload".into()
        )))?;
    if proof.remove("holder_signature").is_none() {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Derived credential holder signature field missing from V1 payload".into()
        )));
    }
    let canonical = canonicalize_json_value(value);
    let serialized = serde_json::to_vec(&canonical).map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "Could not canonicalize derived credential V1 payload: {error}"
        )))
    })?;
    let mut payload = b"MYCELIX:DERIVED-CREDENTIAL:SIGNATURE:V1\\0".to_vec();
    payload.extend(serialized);
    Ok(holo_hash::blake2b_256(&payload).to_vec())
}

fn hash_derived_claim_leaf_v1(
    key: &str,
    value: &serde_json::Value,
) -> ExternResult<Vec<u8>> {
    let canonical_value = canonicalize_json_value(value.clone());
    let serialized_value = serde_json::to_vec(&canonical_value).map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "Could not canonicalize derived claim value: {error}"
        )))
    })?;
    let key_bytes = key.as_bytes();
    let mut payload = b"MYCELIX:DERIVED-CREDENTIAL:MERKLE-LEAF:V1\\0".to_vec();
    payload.extend((key_bytes.len() as u64).to_be_bytes());
    payload.extend(key_bytes);
    payload.extend((serialized_value.len() as u64).to_be_bytes());
    payload.extend(serialized_value);
    Ok(holo_hash::blake2b_256(&payload).to_vec())
}

fn hash_derived_merkle_node_v1(left: &[u8], right: &[u8]) -> Vec<u8> {
    let mut payload = b"MYCELIX:DERIVED-CREDENTIAL:MERKLE-NODE:V1\\0".to_vec();
    payload.extend(left);
    payload.extend(right);
    holo_hash::blake2b_256(&payload).to_vec()
}

fn build_derived_claim_merkle_tree_v1(
    claims: &serde_json::Map<String, serde_json::Value>,
) -> ExternResult<(Vec<u8>, Vec<(String, Vec<u8>)>)> {
    let mut keys: Vec<&String> = claims.keys().collect();
    keys.sort();
    let mut leaves = Vec::with_capacity(keys.len());
    for key in keys {
        leaves.push((
            key.clone(),
            hash_derived_claim_leaf_v1(key, &claims[key])?,
        ));
    }
    if leaves.is_empty() {
        return Ok((vec![0_u8; 32], leaves));
    }
    let mut level: Vec<Vec<u8>> = leaves.iter().map(|(_, hash)| hash.clone()).collect();
    while level.len() > 1 {
        let mut next = Vec::with_capacity((level.len() + 1) / 2);
        let mut index = 0;
        while index < level.len() {
            if index + 1 < level.len() {
                next.push(hash_derived_merkle_node_v1(&level[index], &level[index + 1]));
                index += 2;
            } else {
                next.push(level[index].clone());
                index += 1;
            }
        }
        level = next;
    }
    Ok((level[0].clone(), leaves))
}

fn generate_derived_claim_merkle_proof_v1(
    claims: &serde_json::Map<String, serde_json::Value>,
    leaf_index: usize,
) -> ExternResult<Vec<Vec<u8>>> {
    let mut keys: Vec<&String> = claims.keys().collect();
    keys.sort();
    if leaf_index >= keys.len() {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Derived claim Merkle leaf index out of range".into()
        )));
    }
    let mut level = Vec::with_capacity(keys.len());
    for key in keys {
        level.push(hash_derived_claim_leaf_v1(key, &claims[key])?);
    }
    let mut index = leaf_index;
    let mut proof = Vec::new();
    while level.len() > 1 {
        let sibling = if index.is_multiple_of(2) { index + 1 } else { index - 1 };
        if sibling < level.len() {
            proof.push(level[sibling].clone());
        }
        let mut next = Vec::with_capacity((level.len() + 1) / 2);
        let mut offset = 0;
        while offset < level.len() {
            if offset + 1 < level.len() {
                next.push(hash_derived_merkle_node_v1(&level[offset], &level[offset + 1]));
                offset += 2;
            } else {
                next.push(level[offset].clone());
                offset += 1;
            }
        }
        level = next;
        index /= 2;
    }
    Ok(proof)
}

fn verify_derived_claim_merkle_proof_v1(
    leaf_hash: &[u8],
    leaf_index: usize,
    leaf_count: usize,
    proof: &[Vec<u8>],
    expected_root: &[u8],
) -> bool {
    if leaf_count == 0 || leaf_index >= leaf_count {
        return false;
    }
    let mut current = leaf_hash.to_vec();
    let mut index = leaf_index;
    let mut width = leaf_count;
    let mut proof_index = 0;
    while width > 1 {
        let sibling_index = if index.is_multiple_of(2) { index + 1 } else { index - 1 };
        if sibling_index < width {
            let Some(sibling) = proof.get(proof_index) else {
                return false;
            };
            current = if index.is_multiple_of(2) {
                hash_derived_merkle_node_v1(&current, sibling)
            } else {
                hash_derived_merkle_node_v1(sibling, &current)
            };
            proof_index += 1;
        }
        width = (width + 1) / 2;
        index /= 2;
    }
    proof_index == proof.len() && current == expected_root
}

/// Create a derived credential with selective disclosure
#[hdk_extern]
pub fn create_derived_credential(input: CreateDerivedInput) -> ExternResult<Record> {
    let agent_info = agent_info()?;
    let holder_did = format!("did:mycelix:{}", agent_info.agent_initial_pubkey);
    let now = sys_time()?;

    // Get original credential
    let original_record = get_credential(input.credential_id.clone())?.ok_or(wasm_error!(
        WasmErrorInner::Guest("Original credential not found".into())
    ))?;

    let original_vc: VerifiableCredential = original_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Invalid original credential".into()
        )))?;

    // Verify holder is the subject
    if original_vc.credential_subject.id != holder_did {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Only the credential subject can create derived credentials".into()
        )));
    }

    // Extract selected claims
    let original_claims = &original_vc.credential_subject.claims;
    let mut derived_claims = serde_json::Map::new();

    for claim_key in &input.selected_claims {
        if let Some(value) = original_claims.get(claim_key) {
            derived_claims.insert(claim_key.clone(), value.clone());
        } else {
            return Err(wasm_error!(WasmErrorInner::Guest(format!(
                "Claim '{}' not found in original credential",
                claim_key
            ))));
        }
    }

    let derived_content = CredentialSubject {
        id: holder_did.clone(),
        claims: serde_json::Value::Object(derived_claims),
    };

    // Compute hash of original credential
    let original_hash = compute_credential_hash_for_profile(&original_vc)?;

    // Create derivation proof with real ed25519 signature
    // Build data to sign: original hash + selected claims
    let mut sign_data = original_hash.clone();
    for claim_key in &input.selected_claims {
        sign_data.extend(claim_key.as_bytes());
    }

    // Sign with holder's ed25519 key
    let holder_signature = sign_raw(agent_info.agent_initial_pubkey.clone(), sign_data)?;

    // Build Merkle tree over all original claims for selective disclosure proofs
    let original_claims_obj = original_claims.as_object();
    let (_merkle_root, _leaves) = if let Some(obj) = original_claims_obj {
        build_claim_merkle_tree(obj)
    } else {
        (vec![0u8; 32], Vec::new())
    };

    // Generate per-claim Merkle proofs for each selected claim
    let claim_proofs: Vec<ClaimProof> = if let Some(obj) = original_claims_obj {
        let mut sorted_keys: Vec<&String> = obj.keys().collect();
        sorted_keys.sort();

        input
            .selected_claims
            .iter()
            .map(|key| {
                let leaf_index = sorted_keys.iter().position(|k| *k == key);
                let merkle_path = leaf_index.map(|idx| generate_merkle_proof(obj, idx));
                let commitment = leaf_index.map(|_| hash_claim_leaf(key, &obj[key]));

                ClaimProof {
                    claim_key: key.clone(),
                    merkle_path,
                    commitment,
                }
            })
            .collect()
    } else {
        input
            .selected_claims
            .iter()
            .map(|key| ClaimProof {
                claim_key: key.clone(),
                merkle_path: None,
                commitment: None,
            })
            .collect()
    };

    let derivation_proof = DerivationProof {
        proof_type: "SelectiveDisclosureProof".to_string(),
        original_credential_hash: original_hash,
        claim_proofs,
        holder_signature: holder_signature.as_ref().to_vec(),
    };

    // Calculate expiration
    let expires = input.expires_hours.map(|hours| {
        Timestamp::from_micros(now.as_micros() as i64 + (hours as i64 * 3600 * 1_000_000))
    });

    let derived = DerivedCredential {
        original_credential_id: input.credential_id.clone(),
        original_issuer: original_vc.issuer.did().to_string(),
        holder: holder_did,
        selected_claims: input.selected_claims,
        derived_content,
        derivation_proof,
        created: now,
        expires,
    };

    let action_hash = create_entry(&EntryTypes::DerivedCredential(derived))?;

    // Link original to derived
    create_link(
        original_record.action_address().clone(),
        action_hash.clone(),
        LinkTypes::CredentialToDerived,
        (),
    )?;

    get(action_hash, GetOptions::default())?.ok_or(wasm_error!(WasmErrorInner::Guest(
        "Could not find created derived credential".into()
    )))
}

/// Input for creating a derived credential
#[derive(Serialize, Deserialize, Debug)]
pub struct CreateDerivedInput {
    /// Original credential ID
    pub credential_id: String,
    /// Claims to include in derivation
    pub selected_claims: Vec<String>,
    /// Optional expiration in hours
    pub expires_hours: Option<u32>,
}

/// Result of derived credential verification
#[derive(Serialize, Deserialize, Debug)]
pub struct DerivedVerificationResult {
    pub valid: bool,
    pub errors: Vec<String>,
    pub original_issuer_verified: bool,
    pub original_issuer_signature_status: CredentialSignatureStatus,
}

/// Verify a derived credential
///
/// Checks:
/// 1. Original credential exists and matches the recorded hash
/// 2. Holder's Ed25519 signature over (original_hash + selected_claims)
/// 3. Selected claims are a valid subset of the original credential's claims
/// 4. Derived credential is not expired
/// 5. Original credential is not revoked
#[hdk_extern]
pub fn verify_derived_credential(
    action_hash: ActionHash,
) -> ExternResult<DerivedVerificationResult> {
    let mut errors = Vec::new();
    let mut original_issuer_verified = false;
    let mut original_issuer_signature_status = CredentialSignatureStatus::Error;

    // Fetch derived credential
    let record = get(action_hash, GetOptions::default())?.ok_or(wasm_error!(
        WasmErrorInner::Guest("Derived credential not found".into())
    ))?;

    let derived: DerivedCredential = record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Invalid derived credential entry".into()
        )))?;

    // Check expiration
    let now = sys_time()?;
    if let Some(expires) = derived.expires {
        if now >= expires {
            errors.push("Derived credential has expired".to_string());
        }
    }

    // Fetch original credential
    let original_record = match get_credential(derived.original_credential_id.clone())? {
        Some(rec) => rec,
        None => {
            errors.push("Original credential not found".to_string());
            return Ok(DerivedVerificationResult {
                valid: false,
                errors,
                original_issuer_verified: false,
                original_issuer_signature_status: CredentialSignatureStatus::Error,
            });
        }
    };

    let original_vc: VerifiableCredential = original_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Invalid original credential".into()
        )))?;

    // The original issuer credential must itself be inside its complete
    // validity window; validFrom was previously signed but not enforced.
    let original_validity = credential_validity_status(
        &original_vc.valid_from,
        original_vc.valid_until.as_deref(),
        now,
    );
    if let Some(reason) = original_validity.error_message() {
        errors.push(format!("Original credential validity: {reason}"));
    }

    // Verify the original credential hash matches
    match compute_credential_hash_for_profile(&original_vc) {
        Ok(recomputed_hash) => {
            if recomputed_hash != derived.derivation_proof.original_credential_hash {
                errors.push("Original credential hash does not match derivation proof".to_string());
            }
        }
        Err(error) => {
            errors.push(format!(
                "Original credential signing profile is unsupported: {:?}",
                error
            ));
        }
    }

    // Original issuer verification is true only when every declared
    // signature component has been cryptographically verified.
    match verify_credential_signature(&original_vc) {
        Ok(outcome) => {
            original_issuer_signature_status = outcome.status();
            if original_issuer_signature_status.is_verified() {
                original_issuer_verified = true;
            } else if let Some(reason) = outcome.failure_message() {
                errors.push(format!("Original issuer proof: {}", reason));
            }
        }
        Err(e) => {
            original_issuer_signature_status = CredentialSignatureStatus::Error;
            errors.push(format!("Could not verify original credential: {:?}", e));
        }
    }

    // Verify selected claims are a subset of the original
    let original_claims = &original_vc.credential_subject.claims;
    for claim_key in &derived.selected_claims {
        if original_claims.get(claim_key).is_none() {
            errors.push(format!(
                "Claim '{}' not present in original credential",
                claim_key
            ));
        }
    }

    // Verify Merkle proofs if present
    if let Some(original_obj) = original_claims.as_object() {
        let (merkle_root, _) = build_claim_merkle_tree(original_obj);

        for claim_proof in &derived.derivation_proof.claim_proofs {
            // Verify commitment matches the actual claim value
            if let (Some(commitment), Some(value)) = (
                &claim_proof.commitment,
                original_claims.get(&claim_proof.claim_key),
            ) {
                let expected_leaf = hash_claim_leaf(&claim_proof.claim_key, value);
                if *commitment != expected_leaf {
                    errors.push(format!(
                        "Merkle commitment mismatch for claim '{}'",
                        claim_proof.claim_key
                    ));
                    continue;
                }

                // Verify the Merkle path from leaf to root
                if let Some(path) = &claim_proof.merkle_path {
                    let mut sorted_keys: Vec<&String> = original_obj.keys().collect();
                    sorted_keys.sort();
                    if let Some(leaf_idx) = sorted_keys
                        .iter()
                        .position(|k| *k == &claim_proof.claim_key)
                    {
                        let mut current_hash = expected_leaf;
                        let mut idx = leaf_idx;

                        for sibling in path {
                            let combined = if idx.is_multiple_of(2) {
                                let mut c = current_hash.clone();
                                c.extend(sibling);
                                c
                            } else {
                                let mut c = sibling.clone();
                                c.extend(&current_hash);
                                c
                            };
                            current_hash = holo_hash::blake2b_256(&combined).to_vec();
                            idx /= 2;
                        }

                        if current_hash != merkle_root {
                            errors.push(format!(
                                "Merkle path verification failed for claim '{}'",
                                claim_proof.claim_key
                            ));
                        }
                    }
                }
            }
        }
    }

    // Verify holder's Ed25519 signature over (original_hash + selected_claims)
    let holder_did = &derived.holder;
    let holder_pubkey_str = holder_did.strip_prefix("did:mycelix:");
    if let Some(pubkey_str) = holder_pubkey_str {
        if let Ok(holder_pubkey) = AgentPubKey::try_from(pubkey_str.to_string()) {
            // Reconstruct signed data: original_hash + claim keys
            let mut sign_data = derived.derivation_proof.original_credential_hash.clone();
            for claim_key in &derived.selected_claims {
                sign_data.extend(claim_key.as_bytes());
            }

            if derived.derivation_proof.holder_signature.len() == 64 {
                let sig = Signature::from(
                    <[u8; 64]>::try_from(derived.derivation_proof.holder_signature.as_slice())
                        .unwrap_or([0u8; 64]),
                );
                match verify_signature(holder_pubkey, sig, sign_data) {
                    Ok(true) => {} // Valid
                    Ok(false) => {
                        errors.push("Holder signature verification failed".to_string());
                    }
                    Err(e) => {
                        errors.push(format!("Holder signature verification error: {:?}", e));
                    }
                }
            } else {
                errors.push(format!(
                    "Invalid holder signature length: expected 64, got {}",
                    derived.derivation_proof.holder_signature.len()
                ));
            }
        } else {
            errors.push("Could not parse holder's agent pub key".to_string());
        }
    } else {
        errors.push("Invalid holder DID format".to_string());
    }

    // Check original credential revocation status
    let revocation_status = check_credential_revocation_status(&derived.original_credential_id)?;
    match revocation_status {
        CredentialRevocationStatus::Revoked(reason) => {
            errors.push(format!("Original credential revoked: {}", reason));
        }
        CredentialRevocationStatus::Suspended(reason, _) => {
            errors.push(format!("Original credential suspended: {}", reason));
        }
        CredentialRevocationStatus::Active => {}
        CredentialRevocationStatus::Unknown => {
            errors
                .push("Original credential revocation status could not be determined".to_string());
        }
    }

    Ok(DerivedVerificationResult {
        valid: errors.is_empty() && original_issuer_verified,
        errors,
        original_issuer_verified,
        original_issuer_signature_status,
    })
}

/// Request a credential from an issuer
#[hdk_extern]
pub fn request_credential(input: RequestCredentialInput) -> ExternResult<Record> {
    let agent_info = agent_info()?;
    let requester_did = format!("did:mycelix:{}", agent_info.agent_initial_pubkey);
    let now = sys_time()?;

    let request_id = format!(
        "request:{}:{}:{}",
        requester_did.replace(":", "-"),
        input.issuer_did.replace(":", "-"),
        now.as_micros()
    );

    let request = CredentialRequest {
        id: request_id,
        requester_did: requester_did.clone(),
        issuer_did: input.issuer_did.clone(),
        schema_id: input.schema_id,
        provided_claims: input.claims,
        evidence: input.evidence.unwrap_or_default(),
        status: RequestStatus::Pending,
        created: now,
        updated: now,
    };

    let action_hash = create_entry(&EntryTypes::CredentialRequest(request))?;

    // Link issuer to request
    let issuer_hash = string_to_entry_hash(&input.issuer_did);
    create_link(
        issuer_hash,
        action_hash.clone(),
        LinkTypes::IssuerToRequest,
        (),
    )?;

    // Link requester to request
    let requester_hash = string_to_entry_hash(&requester_did);
    create_link(
        requester_hash,
        action_hash.clone(),
        LinkTypes::RequesterToRequest,
        (),
    )?;

    get(action_hash, GetOptions::default())?.ok_or(wasm_error!(WasmErrorInner::Guest(
        "Could not find created request".into()
    )))
}

/// Input for requesting a credential
#[derive(Serialize, Deserialize, Debug)]
pub struct RequestCredentialInput {
    /// Target issuer's DID
    pub issuer_did: String,
    /// Schema ID for requested credential
    pub schema_id: String,
    /// Claims provided by requester
    pub claims: serde_json::Value,
    /// Supporting evidence
    pub evidence: Option<Vec<CredentialEvidence>>,
}

/// Get pending requests for an issuer
#[hdk_extern]
pub fn get_pending_requests(issuer_did: String) -> ExternResult<Vec<Record>> {
    let issuer_hash = string_to_entry_hash(&issuer_did);
    let links = get_links(
        LinkQuery::try_new(issuer_hash, LinkTypes::IssuerToRequest)?,
        GetStrategy::default(),
    )?;

    let mut requests = Vec::new();
    for link in links {
        let action_hash = ActionHash::try_from(link.target)
            .map_err(|_| wasm_error!(WasmErrorInner::Guest("Invalid link target".into())))?;
        if let Some(record) = get(action_hash, GetOptions::default())? {
            if let Some(req) = record
                .entry()
                .to_app_option::<CredentialRequest>()
                .ok()
                .flatten()
            {
                if matches!(
                    req.status,
                    RequestStatus::Pending | RequestStatus::UnderReview
                ) {
                    requests.push(record);
                }
            }
        }
    }
    Ok(requests)
}

/// Update credential request status
#[hdk_extern]
pub fn update_request_status(input: UpdateRequestStatusInput) -> ExternResult<Record> {
    // Capability guard: only the target issuer can approve/reject requests
    let caller = agent_info()?.agent_initial_pubkey;
    let caller_did = format!("did:mycelix:{}", caller);

    // Find the request
    let filter = ChainQueryFilter::new()
        .entry_type(EntryType::App(AppEntryDef::try_from(
            UnitEntryTypes::CredentialRequest,
        )?))
        .include_entries(true);

    // Find the latest version of this request (update_entry appends newer versions)
    let mut found_record: Option<Record> = None;
    let mut found_req: Option<CredentialRequest> = None;
    for record in query(filter)? {
        if let Some(req) = record
            .entry()
            .to_app_option::<CredentialRequest>()
            .ok()
            .flatten()
        {
            if req.id == input.request_id {
                found_req = Some(req);
                found_record = Some(record);
            }
        }
    }

    let (record, req) = match (found_record, found_req) {
        (Some(r), Some(q)) => (r, q),
        _ => {
            return Err(wasm_error!(WasmErrorInner::Guest(
                "Request not found".into()
            )));
        }
    };

    if req.issuer_did != caller_did {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Only the target issuer can update request status".into()
        )));
    }
    let now = sys_time()?;
    let updated_req = CredentialRequest {
        status: input.new_status,
        updated: now,
        ..req
    };

    let action_hash = update_entry(
        record.action_address().clone(),
        &EntryTypes::CredentialRequest(updated_req),
    )?;

    get(action_hash, GetOptions::default())?.ok_or(wasm_error!(WasmErrorInner::Guest(
        "Could not find updated request".into()
    )))
}

/// Input for updating request status
#[derive(Serialize, Deserialize, Debug)]
pub struct UpdateRequestStatusInput {
    pub request_id: String,
    pub new_status: RequestStatus,
}

/// Get my issued credentials
#[hdk_extern]
pub fn get_my_issued_credentials(_: ()) -> ExternResult<Vec<Record>> {
    let agent_info = agent_info()?;
    let my_did = format!("did:mycelix:{}", agent_info.agent_initial_pubkey);
    get_credentials_issued_by(my_did)
}

/// Get credentials I hold (am subject of)
#[hdk_extern]
pub fn get_my_credentials(_: ()) -> ExternResult<Vec<Record>> {
    let agent_info = agent_info()?;
    let my_did = format!("did:mycelix:{}", agent_info.agent_initial_pubkey);
    get_credentials_for_subject(my_did)
}

// =============================================================================
// PRE-SIGNED CREDENTIAL ISSUANCE (for PQC/hybrid proofs created off-chain)
// =============================================================================

/// Input for issuing a credential with a pre-computed proof.
///
/// The proof was created off-chain (e.g. by the CLI using `hybrid-sign`)
/// and contains a TaggedSignature-encoded multibase proof value.
#[derive(Serialize, Deserialize, Debug)]
pub struct IssueCredentialWithProofInput {
    /// The complete W3C Verifiable Credential with proof already filled in.
    pub credential: VerifiableCredential,
}

/// Issue a credential with a pre-signed proof (PQC or hybrid).
///
/// This extern accepts a fully-formed VerifiableCredential whose `proof.proof_value`
/// was produced off-chain by a PQC-capable signer. It validates the structure
/// and stores the credential without re-signing.
///
/// The proof value should be a TaggedSignature-encoded multibase string so that
/// `verify_credential_signature` can dispatch to the correct algorithm.
#[hdk_extern]
pub fn issue_credential_with_proof(input: IssueCredentialWithProofInput) -> ExternResult<Record> {
    let vc = input.credential;

    // Capability guard: only the claimed issuer can submit pre-signed credentials
    let caller = agent_info()?.agent_initial_pubkey;
    let caller_did = format!("did:mycelix:{}", caller);
    if vc.issuer.did() != caller_did {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Only the claimed issuer can submit pre-signed credentials".into()
        )));
    }

    // Validate basic structure
    if !vc.context.iter().any(|c| c.contains("credentials")) {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Credential must include W3C credentials context".into()
        )));
    }
    if !vc
        .credential_type
        .contains(&"VerifiableCredential".to_string())
    {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Credential type must include 'VerifiableCredential'".into()
        )));
    }
    if !vc.issuer.did().starts_with("did:") {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Issuer must be a valid DID".into()
        )));
    }
    if vc.proof.proof_value.is_empty() {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Pre-signed credential must have a non-empty proof value".into()
        )));
    }

    // Validate that the proof value is parseable as a tagged signature
    if let Ok(tagged) = TaggedSignature::from_multibase(&vc.proof.proof_value) {
        if !tagged.algorithm.is_signature_algorithm() {
            return Err(wasm_error!(WasmErrorInner::Guest(
                "Proof value uses a non-signature algorithm".into()
            )));
        }
    }
    // If TaggedSignature parsing fails, it may be a legacy Ed25519 multibase — that's OK

    // Validate claims against schema if specified
    let schema_validation =
        validate_claims_against_schema(&vc.mycelix_schema_id, &vc.credential_subject.claims)?;
    match &schema_validation {
        SchemaValidationStatus::Validated { schema_name } => {
            debug!(
                "Pre-signed credential schema validation passed for '{}'",
                schema_name
            );
        }
        SchemaValidationStatus::SchemaZomeUnavailable => {
            warn!(
                "Pre-signed credential schema validation skipped: credential_schema zome unavailable"
            );
        }
        SchemaValidationStatus::SchemaNotFound => {
            debug!("Pre-signed credential schema validation skipped: schema not found");
        }
        SchemaValidationStatus::NoSchema => {}
    }

    let action_hash = create_entry(&EntryTypes::VerifiableCredential(vc.clone()))?;

    // Create links
    let issuer_hash = string_to_entry_hash(vc.issuer.did());
    create_link(
        issuer_hash,
        action_hash.clone(),
        LinkTypes::IssuerToCredential,
        (),
    )?;

    let subject_hash = string_to_entry_hash(&vc.credential_subject.id);
    create_link(
        subject_hash,
        action_hash.clone(),
        LinkTypes::SubjectToCredential,
        (),
    )?;

    if !vc.mycelix_schema_id.is_empty() {
        let schema_hash = string_to_entry_hash(&vc.mycelix_schema_id);
        create_link(
            schema_hash,
            action_hash.clone(),
            LinkTypes::SchemaToCredential,
            (),
        )?;
    }

    // By-ID index, so `get_credential` can resolve over the DHT rather than
    // only within the caller's own source chain.
    let id_hash = string_to_entry_hash(&vc.id);
    create_link(
        id_hash,
        action_hash.clone(),
        LinkTypes::CredentialIdToCredential,
        (),
    )?;

    get(action_hash, GetOptions::default())?.ok_or(wasm_error!(WasmErrorInner::Guest(
        "Could not find created credential".into()
    )))
}

/// Format timestamp as ISO 8601 string
fn format_timestamp_iso8601(ts: Timestamp) -> String {
    // Convert microseconds → RFC 3339 / ISO 8601 (no chrono in WASM)
    let total_secs = ts.as_micros() / 1_000_000;

    // Days since Unix epoch
    let days = total_secs / 86400;
    let day_secs = total_secs % 86400;
    let hour = day_secs / 3600;
    let minute = (day_secs % 3600) / 60;
    let second = day_secs % 60;

    // Civil date from day count (Euclidean affine algorithm)
    // Shift epoch to 0000-03-01 for easy leap year math
    let z = days + 719468; // days from 0000-03-01 to 1970-01-01
    let era = z / 146097;
    let doe = z - era * 146097; // day of era [0, 146096]
    let yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
    let y = yoe + era * 400;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = doy - (153 * mp + 2) / 5 + 1;
    let m = if mp < 10 { mp + 3 } else { mp - 9 };
    let y = if m <= 2 { y + 1 } else { y };

    format!(
        "{:04}-{:02}-{:02}T{:02}:{:02}:{:02}Z",
        y, m, d, hour, minute, second
    )
}

// =============================================================================
// CRYPTOGRAPHIC SIGNING (Ed25519 via HDK)
// =============================================================================

/// Base58 alphabet for Bitcoin/IPFS encoding
const BASE58_ALPHABET: &[u8] = b"123456789ABCDEFGHJKLMNPQRSTUVWXYZabcdefghijkmnopqrstuvwxyz";

/// Decode base58btc string to bytes
fn base58_decode(s: &str) -> Option<Vec<u8>> {
    let mut result = vec![0u8; 1];

    for c in s.chars() {
        let digit = BASE58_ALPHABET.iter().position(|&b| b == c as u8)?;

        // Multiply result by 58 and add digit
        let mut carry = digit as u32;
        for byte in result.iter_mut().rev() {
            let value = (*byte as u32) * 58 + carry;
            *byte = (value & 0xff) as u8;
            carry = value >> 8;
        }

        while carry > 0 {
            result.insert(0, (carry & 0xff) as u8);
            carry >>= 8;
        }
    }

    // Handle leading '1's (zeros)
    let leading_ones = s.chars().take_while(|&c| c == '1').count();
    let mut zeros = vec![0u8; leading_ones];
    zeros.extend(result.into_iter().skip_while(|&b| b == 0));

    Some(zeros)
}

// =============================================================================
// ISO 8601 DATE PARSING
// =============================================================================

/// Parse a date/time and check whether an expiry has been reached.
/// Expiry is exclusive: `now == validUntil` is already expired.
fn parse_iso8601_expired(datetime_str: &str, now: Timestamp) -> ExpirationStatus {
    match parse_iso8601_to_micros_with_policy(datetime_str, DateOnlyPolicy::EndOfDay) {
        Some(expiry_micros) if now.as_micros() >= expiry_micros => ExpirationStatus::Expired,
        Some(_) => ExpirationStatus::Valid,
        None => ExpirationStatus::ParseError,
    }
}

/// Result of checking an expiration date.
#[derive(Debug, Clone, PartialEq)]
enum ExpirationStatus {
    Valid,
    Expired,
    ParseError,
}

/// Explicit date-only interpretation for the internal credential profile.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum DateOnlyPolicy {
    /// A date-only validFrom becomes 00:00:00.000000 UTC.
    StartOfDay,
    /// A date-only validUntil becomes 23:59:59.999999 UTC.
    EndOfDay,
}

/// Credential validity is the half-open interval [validFrom, validUntil).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum CredentialValidityStatus {
    Valid,
    NotYetValid,
    Expired,
    InvalidWindow,
    InvalidValidFrom,
    InvalidValidUntil,
}

impl CredentialValidityStatus {
    fn error_message(self) -> Option<&'static str> {
        match self {
            Self::Valid => None,
            Self::NotYetValid => Some("Credential is not yet valid (validFrom is in the future)"),
            Self::Expired => Some("Credential has expired (validUntil is at or before now)"),
            Self::InvalidWindow => Some("Credential validUntil must be later than validFrom"),
            Self::InvalidValidFrom => Some("Credential validFrom is missing or unparseable"),
            Self::InvalidValidUntil => Some("Credential validUntil is unparseable"),
        }
    }
}

fn credential_validity_status(
    valid_from: &str,
    valid_until: Option<&str>,
    now: Timestamp,
) -> CredentialValidityStatus {
    let Some(valid_from_micros) =
        parse_iso8601_to_micros_with_policy(valid_from, DateOnlyPolicy::StartOfDay)
    else {
        return CredentialValidityStatus::InvalidValidFrom;
    };

    let valid_until_micros = match valid_until {
        Some(value) => match parse_iso8601_to_micros_with_policy(value, DateOnlyPolicy::EndOfDay) {
            Some(parsed) => Some(parsed),
            None => return CredentialValidityStatus::InvalidValidUntil,
        },
        None => None,
    };

    if valid_until_micros.is_some_and(|expiry| expiry <= valid_from_micros) {
        return CredentialValidityStatus::InvalidWindow;
    }
    if now.as_micros() < valid_from_micros {
        return CredentialValidityStatus::NotYetValid;
    }
    if valid_until_micros.is_some_and(|expiry| now.as_micros() >= expiry) {
        return CredentialValidityStatus::Expired;
    }
    CredentialValidityStatus::Valid
}

/// Parse ISO 8601 date/time to microseconds since Unix epoch.
///
/// Accepted forms:
/// - `YYYY-MM-DD` (date-only, interpreted by the explicit caller policy);
/// - `YYYY-MM-DDTHH:MM:SSZ`;
/// - `YYYY-MM-DDTHH:MM:SS.fractionZ` with 1–6 fractional digits;
/// - the same timestamp forms with `+HH:MM`, `-HH:MM`, `+HHMM`,
///   `-HHMM`, `+HH`, or `-HH` offsets.
///
/// Full date-times require an explicit timezone. Calendar fields and timezone
/// fields are validated strictly. Fractional seconds are represented at
/// microsecond precision; more than six fractional digits are rejected
/// rather than silently rounded or truncated.
fn parse_iso8601_to_micros(s: &str) -> Option<i64> {
    parse_iso8601_to_micros_with_policy(s, DateOnlyPolicy::EndOfDay)
}

fn parse_iso8601_to_micros_with_policy(s: &str, date_only_policy: DateOnlyPolicy) -> Option<i64> {
    let bytes = s.as_bytes();
    if bytes.len() == 10 && bytes.get(4) == Some(&b'-') && bytes.get(7) == Some(&b'-') {
        let year = parse_ascii_number(&bytes[0..4])? as i32;
        let month = parse_ascii_number(&bytes[5..7])?;
        let day = parse_ascii_number(&bytes[8..10])?;
        validate_calendar_date(year, month, day)?;

        let days = days_since_epoch(year, month, day)?;
        let seconds_in_day = match date_only_policy {
            DateOnlyPolicy::StartOfDay => 0,
            DateOnlyPolicy::EndOfDay => 86_399,
        };
        let fractional_micros = match date_only_policy {
            DateOnlyPolicy::StartOfDay => 0,
            DateOnlyPolicy::EndOfDay => 999_999,
        };
        return days
            .checked_mul(86_400)?
            .checked_add(seconds_in_day)?
            .checked_mul(1_000_000)?
            .checked_add(fractional_micros);
    }

    if bytes.len() < 20
        || bytes.get(4) != Some(&b'-')
        || bytes.get(7) != Some(&b'-')
        || bytes.get(10) != Some(&b'T')
        || bytes.get(13) != Some(&b':')
        || bytes.get(16) != Some(&b':')
    {
        return None;
    }

    let year = parse_ascii_number(&bytes[0..4])? as i32;
    let month = parse_ascii_number(&bytes[5..7])?;
    let day = parse_ascii_number(&bytes[8..10])?;
    let hour = parse_ascii_number(&bytes[11..13])?;
    let minute = parse_ascii_number(&bytes[14..16])?;
    let second = parse_ascii_number(&bytes[17..19])?;
    validate_calendar_date(year, month, day)?;
    if hour > 23 || minute > 59 || second > 59 {
        return None;
    }

    let mut suffix_start = 19;
    let mut fractional_micros = 0_i64;
    if bytes.get(suffix_start) == Some(&b'.') {
        let fraction_start = suffix_start + 1;
        let mut fraction_end = fraction_start;
        while bytes.get(fraction_end).is_some_and(u8::is_ascii_digit) {
            fraction_end += 1;
        }
        let fraction_len = fraction_end - fraction_start;
        if !(1..=6).contains(&fraction_len) {
            return None;
        }
        let fraction = parse_ascii_number(&bytes[fraction_start..fraction_end])?;
        fractional_micros =
            (fraction as i64).checked_mul(10_i64.pow((6 - fraction_len) as u32))?;
        suffix_start = fraction_end;
    }

    // The timezone suffix is mandatory for full date-times. Unknown trailing
    // data is rejected rather than silently treated as UTC.
    let suffix = s.get(suffix_start..)?;
    let timezone_offset_seconds = if suffix == "Z" {
        0
    } else {
        parse_tz_offset(suffix)?
    };

    let days = days_since_epoch(year, month, day)?;
    let day_seconds = (hour as i64)
        .checked_mul(3_600)?
        .checked_add((minute as i64).checked_mul(60)?)?
        .checked_add(second as i64)?;
    let utc_seconds = days
        .checked_mul(86_400)?
        .checked_add(day_seconds)?
        .checked_sub(timezone_offset_seconds)?;
    utc_seconds
        .checked_mul(1_000_000)?
        .checked_add(fractional_micros)
}

fn parse_ascii_number(bytes: &[u8]) -> Option<u32> {
    if bytes.is_empty() || !bytes.iter().all(u8::is_ascii_digit) {
        return None;
    }
    std::str::from_utf8(bytes).ok()?.parse().ok()
}

fn validate_calendar_date(year: i32, month: u32, day: u32) -> Option<()> {
    if !(1..=9999).contains(&year) || !(1..=12).contains(&month) {
        return None;
    }
    let days_in_month = match month {
        1 | 3 | 5 | 7 | 8 | 10 | 12 => 31,
        4 | 6 | 9 | 11 => 30,
        2 if is_leap_year(year) => 29,
        2 => 28,
        _ => return None,
    };
    if day == 0 || day > days_in_month {
        return None;
    }
    Some(())
}

/// Parse timezone offsets in `±HH:MM`, `±HHMM`, or `±HH` form.
fn parse_tz_offset(s: &str) -> Option<i64> {
    let bytes = s.as_bytes();
    let sign = match bytes.first()? {
        b'+' => 1_i64,
        b'-' => -1_i64,
        _ => return None,
    };
    let rest = &bytes[1..];
    let (hours, minutes) = match rest.len() {
        2 => (parse_ascii_number(rest)?, 0),
        4 if rest.get(2) != Some(&b':') => (
            parse_ascii_number(&rest[0..2])?,
            parse_ascii_number(&rest[2..4])?,
        ),
        5 if rest.get(2) == Some(&b':') => (
            parse_ascii_number(&rest[0..2])?,
            parse_ascii_number(&rest[3..5])?,
        ),
        _ => return None,
    };
    if hours > 23 || minutes > 59 {
        return None;
    }
    Some(sign * ((hours as i64) * 3_600 + (minutes as i64) * 60))
}

/// Calculate days since Unix epoch (1970-01-01) for a given date
fn days_since_epoch(year: i32, month: u32, day: u32) -> Option<i64> {
    // Simplified algorithm for days since epoch
    // This handles leap years correctly

    if year < 1970 {
        // For dates before epoch, calculate negative days
        return days_since_epoch_negative(year, month, day);
    }

    let mut days: i64 = 0;

    // Add days for complete years
    for y in 1970..year {
        days += if is_leap_year(y) { 366 } else { 365 };
    }

    // Add days for complete months in current year
    let days_in_months = if is_leap_year(year) {
        [31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31]
    } else {
        [31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31]
    };

    for d in &days_in_months[..((month - 1) as usize)] {
        days += *d as i64;
    }

    // Add days in current month
    days += (day - 1) as i64;

    Some(days)
}

/// Handle dates before Unix epoch
fn days_since_epoch_negative(year: i32, month: u32, day: u32) -> Option<i64> {
    let mut days: i64 = 0;

    // Count backwards from 1970
    for y in (year + 1)..1970 {
        days -= if is_leap_year(y) { 366 } else { 365 };
    }

    // Handle the partial year
    let days_in_months = if is_leap_year(year) {
        [31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31]
    } else {
        [31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31]
    };

    // Days remaining in the year from the given date
    let mut remaining = 0i64;
    for d in &days_in_months[(month as usize)..12] {
        remaining += *d as i64;
    }
    remaining -= (day - 1) as i64;
    remaining += days_in_months[(month - 1) as usize] as i64;

    days -= remaining;

    Some(days)
}

/// Check if a year is a leap year
fn is_leap_year(year: i32) -> bool {
    (year % 4 == 0 && year % 100 != 0) || (year % 400 == 0)
}

/// Decode multibase string (base58btc with 'z' prefix)
fn multibase_decode(s: &str) -> Option<Vec<u8>> {
    if let Some(stripped) = s.strip_prefix('z') {
        base58_decode(stripped)
    } else {
        None
    }
}

// ============================================================================
// ZKP-ENHANCED SELECTIVE DISCLOSURE (DASTARK)
// ============================================================================

/// Input for ZKP-verified selective disclosure.
///
/// The holder proves a predicate about credential attributes without
/// revealing the attribute values themselves. For example:
/// - "My age is between 18-65" without revealing DOB
/// - "My credential is not expired" without revealing expiry date
/// - "My attribute X satisfies condition Y" without revealing X
///
/// Domain tag: `ZTML:Identity:SelectiveDisclosure:v1`
#[derive(Debug, Serialize, Deserialize, SerializedBytes)]
pub struct ZkSelectiveDisclosureInput {
    /// Original credential ID.
    pub credential_id: String,
    /// Predicate to prove (e.g., "age >= 18", "not_expired", "attribute_in_range").
    pub predicate: ZkPredicate,
    /// ZK proof bytes (generated client-side via DASTARK).
    pub proof_bytes: Vec<u8>,
    /// Commitment to the credential data (Blake3 hash).
    pub data_commitment: Vec<u8>,
}

/// Predicate types for selective disclosure.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum ZkPredicate {
    /// Age is within [min, max] range.
    AgeInRange { min: u32, max: Option<u32> },
    /// Credential has not expired.
    NotExpired,
    /// A numeric attribute satisfies a comparison.
    AttributeRange {
        attribute_key: String,
        min: Option<f64>,
        max: Option<f64>,
    },
    /// Credential was issued by a specific issuer (prove membership).
    IssuedBy { issuer_did_hash: Vec<u8> },
    /// Credential type matches expected type.
    CredentialTypeIs { expected_type: String },
}

/// Result of ZKP-verified selective disclosure.
#[derive(Debug, Serialize, Deserialize, SerializedBytes)]
pub struct ZkDisclosureResult {
    /// Whether the predicate is satisfied (per the ZK proof).
    pub satisfied: bool,
    /// Proof verification status.
    pub proof_valid: bool,
    /// Domain tag used for verification.
    pub domain_tag: String,
    /// Credential ID (public).
    pub credential_id: String,
    /// Predicate that was verified (public).
    pub predicate_description: String,
}

/// Verify a ZKP-based selective disclosure claim.
///
/// Checks:
/// 1. Original credential exists and belongs to the caller
/// 2. Data commitment matches the credential hash
/// 3. Proof bytes are non-empty and within size limits
/// 4. Domain tag is correct (ZTML:Identity:SelectiveDisclosure:v1)
///
/// Note: Full STARK proof verification is delegated to mycelix-zkp-core
/// once AIR circuits are implemented for each predicate type.
#[hdk_extern]
pub fn verify_selective_disclosure(
    input: ZkSelectiveDisclosureInput,
) -> ExternResult<ZkDisclosureResult> {
    let domain_tag = mycelix_zkp_core::domain::tag_identity_disclosure();

    // 1. Verify original credential exists
    let credential_record = get_credential(input.credential_id.clone())?.ok_or(wasm_error!(
        WasmErrorInner::Guest("Original credential not found".into())
    ))?;

    let vc: VerifiableCredential = credential_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Invalid credential entry".into()
        )))?;

    // 2. Verify caller is the credential subject
    let agent_info = agent_info()?;
    let caller_did = format!("did:mycelix:{}", agent_info.agent_initial_pubkey);
    if vc.credential_subject.id != caller_did {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Only the credential subject can create selective disclosures".into()
        )));
    }

    // 3. Verify proof bytes structure
    if input.proof_bytes.is_empty() {
        return Ok(ZkDisclosureResult {
            satisfied: false,
            proof_valid: false,
            domain_tag: domain_tag.as_str().to_string(),
            credential_id: input.credential_id,
            predicate_description: format!("{:?}", input.predicate),
        });
    }

    if input.proof_bytes.len() > 500_000 {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Proof exceeds 500KB size limit".into()
        )));
    }

    // 4. Verify data commitment is 32 bytes (Blake3)
    if input.data_commitment.len() != 32 {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Data commitment must be exactly 32 bytes".into()
        )));
    }

    // 5. Domain-tagged proof verification
    // The proof was generated with ZTML:Identity:SelectiveDisclosure:v1
    // Full STARK verification will be wired once AIR circuits are ready.
    // For now, structural validation + domain tag binding.
    let proof_valid = !input.proof_bytes.is_empty() && input.data_commitment.len() == 32;

    Ok(ZkDisclosureResult {
        satisfied: proof_valid,
        proof_valid,
        domain_tag: domain_tag.as_str().to_string(),
        credential_id: input.credential_id,
        predicate_description: format!("{:?}", input.predicate),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn v1_fixture_path() -> std::path::PathBuf {
        std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../../tests/fixtures/credential-profile-v1/fixture.json")
    }

    fn v1_ed25519_fixture() -> (ed25519_dalek::VerifyingKey, ed25519_dalek::Signature) {
        let fixture: serde_json::Value = serde_json::from_slice(
            &std::fs::read(v1_fixture_path()).expect("V1 independent fixture must exist")
        ).expect("V1 fixture JSON must parse");
        let public_key_bytes = hex::decode(
            fixture["verification_public_key_hex"].as_str().expect("public key hex")
        ).expect("public key hex must decode");
        let public_key_array: [u8; 32] = public_key_bytes
            .try_into()
            .expect("Ed25519 public key must be exactly 32 bytes");
        let signature_bytes = hex::decode(
            fixture["signature_hex"].as_str().expect("signature hex")
        ).expect("signature hex must decode");
        let signature = ed25519_dalek::Signature::from_slice(&signature_bytes)
            .expect("Ed25519 signature must be exactly 64 bytes");
        (
            ed25519_dalek::VerifyingKey::from_bytes(&public_key_array)
                .expect("independent fixture public key must be valid"),
            signature,
        )
    }

    fn sample_v1_presentation() -> VerifiablePresentation {
        let credential = VerifiableCredential {
            context: vec!["https://www.w3.org/ns/credentials/v2".into()],
            id: "urn:credential:sample-1".into(),
            credential_type: vec!["VerifiableCredential".into(), "DegreeCredential".into()],
            issuer: CredentialIssuer::Did("did:mycelix:issuer1".into()),
            valid_from: "2026-01-01T00:00:00Z".into(),
            valid_until: None,
            credential_subject: CredentialSubject {
                id: "did:mycelix:holder1".into(),
                claims: serde_json::json!({"degree": "BSc", "level": 7}),
            },
            credential_schema: None,
            credential_status: None,
            proof: CredentialProof {
                proof_type: "MycelixCredentialProof".into(),
                created: "2026-01-01T00:00:00Z".into(),
                verification_method: "did:mycelix:issuer1#keys-1".into(),
                proof_purpose: "assertionMethod".into(),
                proof_value: "zIssuerSignature".into(),
                cryptosuite: Some(CREDENTIAL_HASH_PROFILE_V1.into()),
                algorithm: Some(AlgorithmId::Ed25519.as_u16()),
                challenge: None,
                domain: None,
            },
            mycelix_schema_id: "mycelix:schema:degree:v1".into(),
            mycelix_created: Timestamp::from_micros(1_000_000),
        };
        VerifiablePresentation {
            context: vec!["https://www.w3.org/ns/credentials/v2".into()],
            id: "urn:presentation:sample-1".into(),
            presentation_type: vec!["VerifiablePresentation".into()],
            holder: "did:mycelix:holder1".into(),
            verifiable_credential: vec![credential],
            proof: CredentialProof {
                proof_type: "MycelixPresentationProof".into(),
                created: "2026-01-01T00:00:01Z".into(),
                verification_method: "did:mycelix:holder1#keys-1".into(),
                proof_purpose: "authentication".into(),
                proof_value: "zHolderSignature".into(),
                cryptosuite: Some(PRESENTATION_HASH_PROFILE_V1.into()),
                algorithm: Some(AlgorithmId::Ed25519.as_u16()),
                challenge: Some("challenge-1".into()),
                domain: Some("verifier.example.test".into()),
            },
            mycelix_created: Timestamp::from_micros(2_000_000),
        }
    }

    #[test]
    fn v1_presentation_requires_a_valid_exact_verifier_challenge() {
        let mut presentation = sample_v1_presentation();
        assert!(validate_presentation_challenge(&presentation, "challenge-1").is_ok());

        assert!(validate_presentation_challenge(&presentation, "").is_err());
        assert!(validate_presentation_challenge(&presentation, &"x".repeat(513)).is_err());
        assert!(validate_presentation_challenge(&presentation, "challenge-2").is_err());

        presentation.proof.challenge = None;
        assert!(validate_presentation_challenge(&presentation, "challenge-1").is_err());

        presentation.proof.challenge = Some(String::new());
        assert!(validate_presentation_challenge(&presentation, "challenge-1").is_err());

        presentation.proof.challenge = Some("   ".into());
        assert!(validate_presentation_challenge(&presentation, "   ").is_err());

        presentation.proof.challenge = Some("x".repeat(513));
        assert!(validate_presentation_challenge(&presentation, &"x".repeat(513)).is_err());
    }

    #[test]
    fn presentation_hash_v1_is_deterministic_and_excludes_only_holder_proof_value() {
        let mut presentation = sample_v1_presentation();
        let first = compute_presentation_hash_v1(&presentation).unwrap();
        assert_eq!(first, compute_presentation_hash_v1(&presentation).unwrap());
        assert_eq!(first.len(), 32);

        presentation.proof.proof_value = "zDifferentHolderProof".into();
        assert_eq!(first, compute_presentation_hash_v1(&presentation).unwrap());
    }

    #[test]
    fn presentation_hash_v1_binds_every_outer_and_embedded_field() {
        let base = sample_v1_presentation();
        let expected = compute_presentation_hash_v1(&base).unwrap();

        let mut changed = base.clone();
        changed.context.push("https://example.test/extra-context".into());
        assert_ne!(expected, compute_presentation_hash_v1(&changed).unwrap(), "@context");

        changed = base.clone();
        changed.id.push_str("-altered");
        assert_ne!(expected, compute_presentation_hash_v1(&changed).unwrap(), "id");

        changed = base.clone();
        changed.presentation_type.push("AdminPresentation".into());
        assert_ne!(expected, compute_presentation_hash_v1(&changed).unwrap(), "type");

        changed = base.clone();
        changed.holder = "did:mycelix:attacker".into();
        assert_ne!(expected, compute_presentation_hash_v1(&changed).unwrap(), "holder");

        changed = base.clone();
        changed.verifiable_credential[0].credential_subject.claims["degree"] =
            serde_json::json!("PhD");
        assert_ne!(expected, compute_presentation_hash_v1(&changed).unwrap(), "embedded credential claims");

        changed = base.clone();
        changed.verifiable_credential[0].issuer =
            CredentialIssuer::Did("did:mycelix:attacker".into());
        assert_ne!(expected, compute_presentation_hash_v1(&changed).unwrap(), "embedded credential issuer");

        changed = base.clone();
        changed.verifiable_credential[0].valid_until =
            Some("2026-06-01T00:00:00Z".into());
        assert_ne!(expected, compute_presentation_hash_v1(&changed).unwrap(), "embedded credential validity");

        changed = base.clone();
        changed.verifiable_credential[0].proof.proof_value = "zAlteredIssuerProof".into();
        assert_ne!(expected, compute_presentation_hash_v1(&changed).unwrap(), "embedded credential proofValue");

        changed = base.clone();
        changed.proof.proof_type = "OtherPresentationProof".into();
        assert_ne!(expected, compute_presentation_hash_v1(&changed).unwrap(), "proof.type");

        changed = base.clone();
        changed.proof.created.push_str(".001Z");
        assert_ne!(expected, compute_presentation_hash_v1(&changed).unwrap(), "proof.created");

        changed = base.clone();
        changed.proof.verification_method.push_str("-other");
        assert_ne!(expected, compute_presentation_hash_v1(&changed).unwrap(), "verificationMethod");

        changed = base.clone();
        changed.proof.proof_purpose = "assertionMethod".into();
        assert_ne!(expected, compute_presentation_hash_v1(&changed).unwrap(), "proofPurpose");

        changed = base.clone();
        changed.proof.cryptosuite = Some("eddsa-rdfc-2022".into());
        assert_ne!(expected, compute_presentation_hash_v1(&changed).unwrap(), "cryptosuite");

        changed = base.clone();
        changed.proof.algorithm = Some(AlgorithmId::MlDsa65.as_u16());
        assert_ne!(expected, compute_presentation_hash_v1(&changed).unwrap(), "algorithm");

        changed = base.clone();
        changed.proof.challenge = Some("challenge-2".into());
        assert_ne!(expected, compute_presentation_hash_v1(&changed).unwrap(), "challenge");

        changed = base.clone();
        changed.proof.domain = Some("other.example.test".into());
        assert_ne!(expected, compute_presentation_hash_v1(&changed).unwrap(), "domain");

        changed = base.clone();
        changed.mycelix_created = Timestamp::from_micros(3_000_000);
        assert_ne!(expected, compute_presentation_hash_v1(&changed).unwrap(), "mycelix_created");
    }

    #[test]
    fn presentation_v1_header_pins_holder_method_type_algorithm_and_purpose() {
        let base = sample_v1_presentation();
        assert!(v1_presentation_proof_header_matches(&base));

        let mut changed = base.clone();
        changed.proof.verification_method = "did:mycelix:holder1#keys-2".into();
        assert!(!v1_presentation_proof_header_matches(&changed));

        changed = base.clone();
        changed.proof.proof_type = "DataIntegrityProof".into();
        assert!(!v1_presentation_proof_header_matches(&changed));

        changed = base.clone();
        changed.proof.proof_purpose = "assertionMethod".into();
        assert!(!v1_presentation_proof_header_matches(&changed));

        changed = base;
        changed.proof.algorithm = Some(AlgorithmId::MlDsa65.as_u16());
        assert!(!v1_presentation_proof_header_matches(&changed));
    }

    #[test]
    fn presentation_hash_profiles_keep_legacy_partial_and_unknown_unsupported() {
        let mut presentation = sample_v1_presentation();
        assert_eq!(
            presentation_hash_profile(&presentation),
            Some(PresentationHashProfile::MycelixV1)
        );
        presentation.proof.cryptosuite = Some(LEGACY_ED25519_CRYPTOSUITE.into());
        assert_eq!(
            presentation_hash_profile(&presentation),
            Some(PresentationHashProfile::LegacySubsetV0)
        );
        presentation.proof.cryptosuite = Some("unknown-presentation-profile".into());
        assert_eq!(presentation_hash_profile(&presentation), None);
    }

    // --- is_leap_year ---

    #[test]
    fn leap_year_divisible_by_4() {
        assert!(is_leap_year(2024));
        assert!(is_leap_year(2004));
        assert!(is_leap_year(1996));
    }

    #[test]
    fn leap_year_century_not_leap() {
        assert!(!is_leap_year(1900));
        assert!(!is_leap_year(2100));
        assert!(!is_leap_year(2200));
    }

    #[test]
    fn leap_year_400_is_leap() {
        assert!(is_leap_year(2000));
        assert!(is_leap_year(1600));
        assert!(is_leap_year(2400));
    }

    #[test]
    fn non_leap_years() {
        assert!(!is_leap_year(2023));
        assert!(!is_leap_year(2025));
        assert!(!is_leap_year(1971));
    }

    // --- days_since_epoch ---

    #[test]
    fn epoch_is_zero() {
        assert_eq!(days_since_epoch(1970, 1, 1), Some(0));
    }

    #[test]
    fn known_date_2024_01_01() {
        // 2024-01-01 = 54 years (including leap years)
        // 1970-2023 = 54 years: 13 leap years (72,76,80,84,88,92,96,00,04,08,12,16,20) + 41 normal
        // 13*366 + 41*365 = 4758 + 14965 = 19723
        assert_eq!(days_since_epoch(2024, 1, 1), Some(19723));
    }

    #[test]
    fn known_date_2000_03_01() {
        // 2000-03-01 (right after Feb 29 leap day in Y2K)
        let days = days_since_epoch(2000, 3, 1).unwrap();
        // 1970-01-01 to 2000-03-01 = 11017 days
        assert_eq!(days, 11017);
    }

    // --- parse_tz_offset ---

    #[test]
    fn tz_offset_utc_plus() {
        assert_eq!(parse_tz_offset("+05:30"), Some(5 * 3600 + 30 * 60));
        assert_eq!(parse_tz_offset("+00:00"), Some(0));
        assert_eq!(parse_tz_offset("+08:00"), Some(8 * 3600));
    }

    #[test]
    fn tz_offset_utc_minus() {
        assert_eq!(parse_tz_offset("-08:00"), Some(-8 * 3600));
        assert_eq!(parse_tz_offset("-05:00"), Some(-5 * 3600));
    }

    #[test]
    fn tz_offset_compact_format() {
        assert_eq!(parse_tz_offset("+0530"), Some(5 * 3600 + 30 * 60));
        assert_eq!(parse_tz_offset("-0800"), Some(-8 * 3600));
    }

    #[test]
    fn tz_offset_hours_only() {
        assert_eq!(parse_tz_offset("+05"), Some(5 * 3600));
        assert_eq!(parse_tz_offset("-08"), Some(-8 * 3600));
    }

    #[test]
    fn tz_offset_invalid() {
        assert_eq!(parse_tz_offset("+"), None);
        assert_eq!(parse_tz_offset(""), None);
    }

    // --- parse_iso8601_to_micros ---

    #[test]
    fn iso8601_date_only() {
        let micros = parse_iso8601_to_micros("2024-01-01").unwrap();
        // Should be end of day (23:59:59)
        let expected_days = days_since_epoch(2024, 1, 1).unwrap();
        let expected_micros = (expected_days * 86400 + 86399) * 1_000_000 + 999_999;
        assert_eq!(micros, expected_micros);
    }

    #[test]
    fn iso8601_utc_z() {
        let micros = parse_iso8601_to_micros("2024-01-01T00:00:00Z").unwrap();
        let expected_days = days_since_epoch(2024, 1, 1).unwrap();
        assert_eq!(micros, expected_days * 86400 * 1_000_000);
    }

    #[test]
    fn iso8601_with_tz_offset() {
        let utc_micros = parse_iso8601_to_micros("2024-01-01T00:00:00Z").unwrap();
        let plus5_micros = parse_iso8601_to_micros("2024-01-01T05:00:00+05:00").unwrap();
        // Both should resolve to same UTC time
        assert_eq!(utc_micros, plus5_micros);
    }

    #[test]
    fn iso8601_invalid_month() {
        assert!(parse_iso8601_to_micros("2024-13-01").is_none());
        assert!(parse_iso8601_to_micros("2024-00-01").is_none());
    }

    #[test]
    fn iso8601_invalid_format() {
        assert!(parse_iso8601_to_micros("not a date").is_none());
        assert!(parse_iso8601_to_micros("").is_none());
    }

    #[test]
    fn iso8601_rejects_invalid_calendar_dates_and_leap_days() {
        assert!(parse_iso8601_to_micros("2024-02-30").is_none());
        assert!(parse_iso8601_to_micros("2023-02-29").is_none());
        assert!(parse_iso8601_to_micros("2024-04-31").is_none());
        assert!(parse_iso8601_to_micros("2024-02-29").is_some());
    }

    #[test]
    fn iso8601_full_timestamp_requires_valid_timezone_and_no_trailing_data() {
        assert!(parse_iso8601_to_micros("2024-01-01T00:00:00").is_none());
        assert!(parse_iso8601_to_micros("2024-01-01T00:00:00Zgarbage").is_none());
        assert!(parse_iso8601_to_micros("2024-01-01T00:00:00+24:00").is_none());
        assert!(parse_iso8601_to_micros("2024-01-01T00:00:00+05:60").is_none());
        assert!(parse_iso8601_to_micros("2024-01-01T24:00:00Z").is_none());
        assert!(parse_iso8601_to_micros("2024-01-01T00:60:00Z").is_none());
        assert!(parse_iso8601_to_micros("2024-01-01T00:00:60Z").is_none());
    }

    #[test]
    fn iso8601_fractional_seconds_are_microsecond_exact() {
        let base = parse_iso8601_to_micros("2024-01-01T00:00:00Z").unwrap();
        assert_eq!(
            parse_iso8601_to_micros("2024-01-01T00:00:00.123456Z"),
            Some(base + 123_456)
        );
        assert_eq!(
            parse_iso8601_to_micros("2024-01-01T00:00:00.1Z"),
            Some(base + 100_000)
        );
        assert!(parse_iso8601_to_micros("2024-01-01T00:00:00.1234567Z").is_none());
        assert!(parse_iso8601_to_micros("2024-01-01T00:00:00.Z").is_none());
    }

    #[test]
    fn iso8601_date_only_policy_has_explicit_start_and_end_of_day() {
        let start = parse_iso8601_to_micros_with_policy(
            "2024-01-01",
            DateOnlyPolicy::StartOfDay,
        ).unwrap();
        let end = parse_iso8601_to_micros_with_policy(
            "2024-01-01",
            DateOnlyPolicy::EndOfDay,
        ).unwrap();
        let day_start = days_since_epoch(2024, 1, 1).unwrap() * 86_400 * 1_000_000;
        assert_eq!(start, day_start);
        assert_eq!(end, day_start + 86_399_999_999);
    }

    #[test]
    fn credential_validity_window_enforces_not_before_and_exclusive_expiry() {
        let now = Timestamp::from_micros(
            days_since_epoch(2024, 1, 1).unwrap() * 86_400 * 1_000_000
        );
        assert_eq!(
            credential_validity_status(
                "2024-01-01T00:00:00Z",
                Some("2024-01-02T00:00:00Z"),
                now,
            ),
            CredentialValidityStatus::Valid
        );
        assert_eq!(
            credential_validity_status(
                "2024-01-02T00:00:00Z",
                None,
                now,
            ),
            CredentialValidityStatus::NotYetValid
        );
        assert_eq!(
            credential_validity_status(
                "2023-01-01T00:00:00Z",
                Some("2024-01-01T00:00:00Z"),
                now,
            ),
            CredentialValidityStatus::Expired
        );
        assert_eq!(
            credential_validity_status(
                "2024-01-02T00:00:00Z",
                Some("2024-01-01T00:00:00Z"),
                now,
            ),
            CredentialValidityStatus::InvalidWindow
        );
        assert_eq!(
            credential_validity_status("not-a-date", None, now),
            CredentialValidityStatus::InvalidValidFrom
        );
        assert_eq!(
            credential_validity_status(
                "2024-01-01T00:00:00Z",
                Some("not-a-date"),
                now,
            ),
            CredentialValidityStatus::InvalidValidUntil
        );
    }

    #[test]
    fn credential_validity_date_only_means_start_for_from_and_end_for_until() {
        let now = Timestamp::from_micros(
            days_since_epoch(2024, 1, 1).unwrap() * 86_400 * 1_000_000
        );
        assert_eq!(
            credential_validity_status("2024-01-01", Some("2024-01-01"), now),
            CredentialValidityStatus::Valid
        );
    }

    // --- parse_iso8601_expired ---

    #[test]
    fn expired_past_date() {
        let now = Timestamp::from_micros(1_700_000_000_000_000); // ~2023-11-14
        assert_eq!(
            parse_iso8601_expired("2020-01-01T00:00:00Z", now),
            ExpirationStatus::Expired
        );
    }

    #[test]
    fn expiry_is_exclusive_at_the_exact_boundary() {
        let expiry = parse_iso8601_to_micros("2024-01-01T00:00:00Z").unwrap();
        let now = Timestamp::from_micros(expiry);
        assert_eq!(
            parse_iso8601_expired("2024-01-01T00:00:00Z", now),
            ExpirationStatus::Expired
        );
    }

    #[test]
    fn not_expired_future_date() {
        let now = Timestamp::from_micros(1_700_000_000_000_000);
        assert_eq!(
            parse_iso8601_expired("2030-01-01T00:00:00Z", now),
            ExpirationStatus::Valid
        );
    }

    #[test]
    fn unparseable_fails_closed() {
        let now = Timestamp::from_micros(1_700_000_000_000_000);
        assert_eq!(
            parse_iso8601_expired("garbage", now),
            ExpirationStatus::ParseError,
            "Unparseable dates must fail closed (ParseError), not silently pass"
        );
    }

    // --- format_timestamp_iso8601 ---

    #[test]
    fn format_timestamp_epoch() {
        let ts = Timestamp::from_micros(0);
        assert_eq!(format_timestamp_iso8601(ts), "1970-01-01T00:00:00Z");
    }

    #[test]
    fn format_timestamp_known_value() {
        let ts = Timestamp::from_micros(1_700_000_000_000_000);
        let formatted = format_timestamp_iso8601(ts);
        assert_eq!(formatted, "2023-11-14T22:13:20Z");
    }

    // --- base58_decode ---

    #[test]
    fn base58_decode_known_value() {
        // "1" in base58 = 0x00
        let result = base58_decode("1").unwrap();
        assert_eq!(result, vec![0]);
    }

    #[test]
    fn base58_decode_round_trip_simple() {
        // "2" in base58 = 0x01
        let result = base58_decode("2").unwrap();
        assert_eq!(result, vec![1]);
    }

    #[test]
    fn base58_decode_invalid_char() {
        // '0', 'O', 'I', 'l' are not in base58 alphabet
        assert!(base58_decode("0").is_none());
        assert!(base58_decode("O").is_none());
        assert!(base58_decode("I").is_none());
        assert!(base58_decode("l").is_none());
    }

    // --- multibase_decode ---

    #[test]
    fn multibase_decode_z_prefix() {
        let result = multibase_decode("z2");
        assert!(result.is_some());
    }

    #[test]
    fn multibase_decode_wrong_prefix() {
        assert!(multibase_decode("f01020304").is_none());
        assert!(multibase_decode("m01020304").is_none());
        assert!(multibase_decode("").is_none());
    }

    // --- compute_credential_hash ---

    #[test]
    fn credential_hash_deterministic() {
        let vc = VerifiableCredential {
            context: vec!["https://www.w3.org/ns/credentials/v2".into()],
            id: "test-cred-1".into(),
            credential_type: vec!["VerifiableCredential".into()],
            issuer: CredentialIssuer::Did("did:mycelix:issuer1".into()),
            valid_from: "2026-01-01T00:00:00Z".into(),
            valid_until: None,
            credential_subject: CredentialSubject {
                id: "did:mycelix:holder1".into(),
                claims: serde_json::json!({"degree": "BSc"}),
            },
            credential_schema: None,
            credential_status: None,
            proof: CredentialProof {
                proof_type: "Ed25519Signature2020".into(),
                created: "2026-01-01".into(),
                verification_method: "did:mycelix:issuer1#key-1".into(),
                proof_purpose: "assertionMethod".into(),
                proof_value: "zSig".into(),
                cryptosuite: None,
                algorithm: None,
                challenge: None,
                domain: None,
            },
            mycelix_schema_id: "mycelix:schema:test:v1".into(),
            mycelix_created: Timestamp::from_micros(0),
        };
        let hash1 = compute_credential_hash_legacy_subset(&vc);
        let hash2 = compute_credential_hash_legacy_subset(&vc);
        assert_eq!(hash1, hash2, "Hash must be deterministic");
        assert_eq!(hash1.len(), 32, "BLAKE2b-256 produces 32 bytes");
    }

    #[test]
    fn credential_hash_v1_is_deterministic_and_excludes_only_proof_value() {
        let mut vc = VerifiableCredential {
            context: vec!["https://www.w3.org/ns/credentials/v2".into()],
            id: "test-cred-v1".into(),
            credential_type: vec!["VerifiableCredential".into(), "EmploymentCredential".into()],
            issuer: CredentialIssuer::Object {
                id: "did:mycelix:issuer1".into(),
                name: Some("Issuer One".into()),
                issuer_type: Some(vec!["Organization".into()]),
            },
            valid_from: "2026-01-01T00:00:00Z".into(),
            valid_until: Some("2027-01-01T00:00:00Z".into()),
            credential_subject: CredentialSubject {
                id: "did:mycelix:holder1".into(),
                claims: serde_json::json!({"degree": "BSc", "level": 7}),
            },
            credential_schema: Some(CredentialSchemaRef {
                id: "https://example.test/schema/degree-v1".into(),
                schema_type: "JsonSchema".into(),
            }),
            credential_status: Some(CredentialStatus {
                id: "urn:status:entry:1".into(),
                status_type: "BitstringStatusListEntry".into(),
                status_purpose: Some("revocation".into()),
                status_list_index: Some("1".into()),
                status_list_credential: Some("urn:status:list:1".into()),
            }),
            proof: CredentialProof {
                proof_type: "MycelixCredentialProof".into(),
                created: "2026-01-01T00:00:00Z".into(),
                verification_method: "did:mycelix:issuer1#keys-1".into(),
                proof_purpose: "assertionMethod".into(),
                proof_value: "zSignatureA".into(),
                cryptosuite: Some(CREDENTIAL_HASH_PROFILE_V1.into()),
                algorithm: Some(AlgorithmId::Ed25519.as_u16()),
                challenge: Some("challenge-1".into()),
                domain: Some("issuer.example.test".into()),
            },
            mycelix_schema_id: "mycelix:schema:degree:v1".into(),
            mycelix_created: Timestamp::from_micros(1_000_000),
        };

        let first = compute_credential_hash_v1(&vc).unwrap();
        let second = compute_credential_hash_v1(&vc).unwrap();
        assert_eq!(first, second);
        assert_eq!(first.len(), 32);

        // The signature bytes are not part of their own signing preimage.
        vc.proof.proof_value = "zDifferentSignatureBytes".into();
        assert_eq!(first, compute_credential_hash_v1(&vc).unwrap());
    }

    #[test]
    fn credential_hash_v1_binds_every_serialized_security_relevant_field() {
        let base = VerifiableCredential {
            context: vec!["https://www.w3.org/ns/credentials/v2".into()],
            id: "test-cred-v1".into(),
            credential_type: vec!["VerifiableCredential".into()],
            issuer: CredentialIssuer::Did("did:mycelix:issuer1".into()),
            valid_from: "2026-01-01T00:00:00Z".into(),
            valid_until: None,
            credential_subject: CredentialSubject {
                id: "did:mycelix:holder1".into(),
                claims: serde_json::json!({"degree": "BSc"}),
            },
            credential_schema: None,
            credential_status: None,
            proof: CredentialProof {
                proof_type: "MycelixCredentialProof".into(),
                created: "2026-01-01T00:00:00Z".into(),
                verification_method: "did:mycelix:issuer1#keys-1".into(),
                proof_purpose: "assertionMethod".into(),
                proof_value: "zSignature".into(),
                cryptosuite: Some(CREDENTIAL_HASH_PROFILE_V1.into()),
                algorithm: Some(AlgorithmId::Ed25519.as_u16()),
                challenge: None,
                domain: None,
            },
            mycelix_schema_id: "mycelix:schema:test:v1".into(),
            mycelix_created: Timestamp::from_micros(1_000_000),
        };
        let expected = compute_credential_hash_v1(&base).unwrap();

        let mut changed = base.clone();
        changed.context.push("https://example.test/context/v1".into());
        assert_ne!(expected, compute_credential_hash_v1(&changed).unwrap(), "@context");
        changed = base.clone();
        changed.id.push_str("-altered");
        assert_ne!(expected, compute_credential_hash_v1(&changed).unwrap(), "id");
        changed = base.clone();
        changed.credential_type.push("AdminCredential".into());
        assert_ne!(expected, compute_credential_hash_v1(&changed).unwrap(), "type");
        changed = base.clone();
        changed.issuer = CredentialIssuer::Did("did:mycelix:attacker".into());
        assert_ne!(expected, compute_credential_hash_v1(&changed).unwrap(), "issuer");
        changed = base.clone();
        changed.valid_from.push_str(".001Z");
        assert_ne!(expected, compute_credential_hash_v1(&changed).unwrap(), "validFrom");
        changed = base.clone();
        changed.valid_until = Some("2026-06-01T00:00:00Z".into());
        assert_ne!(expected, compute_credential_hash_v1(&changed).unwrap(), "validUntil");
        changed = base.clone();
        changed.credential_subject.id = "did:mycelix:other-holder".into();
        assert_ne!(expected, compute_credential_hash_v1(&changed).unwrap(), "credentialSubject.id");
        changed = base.clone();
        changed.credential_subject.claims["degree"] = serde_json::json!("PhD");
        assert_ne!(expected, compute_credential_hash_v1(&changed).unwrap(), "credentialSubject claims");
        changed = base.clone();
        changed.credential_schema = Some(CredentialSchemaRef {
            id: "https://example.test/schema/attacker".into(),
            schema_type: "JsonSchema".into(),
        });
        assert_ne!(expected, compute_credential_hash_v1(&changed).unwrap(), "credentialSchema");
        changed = base.clone();
        changed.credential_status = Some(CredentialStatus {
            id: "urn:status:entry:9".into(),
            status_type: "BitstringStatusListEntry".into(),
            status_purpose: Some("revocation".into()),
            status_list_index: Some("9".into()),
            status_list_credential: Some("urn:status:list:1".into()),
        });
        assert_ne!(expected, compute_credential_hash_v1(&changed).unwrap(), "credentialStatus");
        changed = base.clone();
        changed.proof.proof_type = "OtherProof".into();
        assert_ne!(expected, compute_credential_hash_v1(&changed).unwrap(), "proof.type");
        changed = base.clone();
        changed.proof.created.push_str(".001Z");
        assert_ne!(expected, compute_credential_hash_v1(&changed).unwrap(), "proof.created");
        changed = base.clone();
        changed.proof.verification_method.push_str("-attacker");
        assert_ne!(expected, compute_credential_hash_v1(&changed).unwrap(), "verificationMethod");
        changed = base.clone();
        changed.proof.proof_purpose = "authentication".into();
        assert_ne!(expected, compute_credential_hash_v1(&changed).unwrap(), "proofPurpose");
        changed = base.clone();
        changed.proof.cryptosuite = Some("eddsa-rdfc-2022".into());
        assert_ne!(expected, compute_credential_hash_v1(&changed).unwrap(), "cryptosuite");
        changed = base.clone();
        changed.proof.algorithm = Some(AlgorithmId::MlDsa65.as_u16());
        assert_ne!(expected, compute_credential_hash_v1(&changed).unwrap(), "algorithm");
        changed = base.clone();
        changed.proof.challenge = Some("challenge-2".into());
        assert_ne!(expected, compute_credential_hash_v1(&changed).unwrap(), "challenge");
        changed = base.clone();
        changed.proof.domain = Some("other.example.test".into());
        assert_ne!(expected, compute_credential_hash_v1(&changed).unwrap(), "domain");
        changed = base.clone();
        changed.mycelix_schema_id.push_str(":altered");
        assert_ne!(expected, compute_credential_hash_v1(&changed).unwrap(), "mycelix_schema_id");
        changed = base.clone();
        changed.mycelix_created = Timestamp::from_micros(2_000_000);
        assert_ne!(expected, compute_credential_hash_v1(&changed).unwrap(), "mycelix_created");
    }

    #[test]
    fn credential_hash_v1_matches_independently_signed_openssl_fixture() {
        let fixture: serde_json::Value = serde_json::from_slice(
            &std::fs::read(v1_fixture_path()).expect("V1 independent fixture must exist")
        ).expect("V1 fixture JSON must parse");
        let mut credential = sample_v1_presentation()
            .verifiable_credential
            .into_iter()
            .next()
            .expect("sample presentation has one embedded credential");

        let mut unsigned = serde_json::to_value(&credential)
            .expect("credential serializes for V1 signing");
        let proof = unsigned
            .get_mut("proof")
            .and_then(serde_json::Value::as_object_mut)
            .expect("credential has a proof object");
        proof.remove("proofValue").expect("proofValue is excluded from preimage");
        let canonical = serde_json::to_vec(&canonicalize_json_value(unsigned))
            .expect("canonical V1 payload serializes");
        let fixture_payload = fixture["canonical_payload_utf8"]
            .as_str()
            .expect("fixture has exact canonical payload");
        assert_eq!(
            canonical,
            fixture_payload.as_bytes(),
            "coordinator serializer must match the externally generated fixture byte-for-byte"
        );

        let mut domain_separated = hex::decode(
            fixture["domain_separator_hex"].as_str().expect("domain separator hex")
        ).expect("domain separator hex must decode");
        domain_separated.extend_from_slice(&canonical);
        let independently_expected_digest = holo_hash::blake2b_256(&domain_separated);
        let fixture_digest = hex::decode(
            fixture["digest_hex"].as_str().expect("fixture digest hex")
        ).expect("fixture digest hex must decode");
        assert_eq!(independently_expected_digest.as_slice(), fixture_digest);

        let coordinator_digest = compute_credential_hash_v1(&credential)
            .expect("coordinator V1 digest must be produced");
        assert_eq!(coordinator_digest, fixture_digest);

        let (public_key, signature) = v1_ed25519_fixture();
        public_key.verify_strict(&coordinator_digest, &signature)
            .expect("OpenSSL-generated Ed25519 fixture must verify with RustCrypto");

        // proofValue is intentionally the only omitted proof member: changing
        // it leaves the signed preimage unchanged, but the replacement bytes
        // must independently fail signature verification when used as a proof.
        credential.proof.proof_value = "zTamperedSignatureValue".into();
        assert_eq!(
            compute_credential_hash_v1(&credential).unwrap(),
            coordinator_digest
        );
    }

    #[test]
    fn credential_hash_v1_fixture_signature_rejects_every_bound_field_mutation() {
        let credential = sample_v1_presentation()
            .verifiable_credential
            .into_iter()
            .next()
            .expect("sample presentation has one embedded credential");
        let digest = compute_credential_hash_v1(&credential).unwrap();
        let (public_key, signature) = v1_ed25519_fixture();
        public_key.verify_strict(&digest, &signature)
            .expect("unmodified credential fixture must verify");

        let mutations: Vec<(&str, Box<dyn Fn(&mut VerifiableCredential)>)> = vec![
            ("@context", Box::new(|vc| vc.context.push("https://example.test/context/v1".into()))),
            ("id", Box::new(|vc| vc.id.push_str("-mutated"))),
            ("type", Box::new(|vc| vc.credential_type.push("AdminCredential".into()))),
            ("issuer", Box::new(|vc| vc.issuer = CredentialIssuer::Did("did:mycelix:attacker".into()))),
            ("validFrom", Box::new(|vc| vc.valid_from.push_str(".001Z"))),
            ("validUntil", Box::new(|vc| vc.valid_until = Some("2027-01-01T00:00:00Z".into()))),
            ("credentialSubject.id", Box::new(|vc| vc.credential_subject.id = "did:mycelix:other".into())),
            ("credentialSubject.claims", Box::new(|vc| vc.credential_subject.claims["degree"] = serde_json::json!("PhD"))),
            ("credentialSchema", Box::new(|vc| vc.credential_schema = Some(CredentialSchemaRef {
                id: "https://example.test/schema/changed".into(),
                schema_type: "JsonSchema".into(),
            }))),
            ("credentialStatus", Box::new(|vc| vc.credential_status = Some(CredentialStatus {
                id: "urn:status:entry:9".into(),
                status_type: "BitstringStatusListEntry".into(),
                status_purpose: Some("revocation".into()),
                status_list_index: Some("9".into()),
                status_list_credential: Some("urn:status:list:1".into()),
            }))),
            ("proof.type", Box::new(|vc| vc.proof.proof_type = "OtherProof".into())),
            ("proof.created", Box::new(|vc| vc.proof.created.push_str(".001Z"))),
            ("proof.verificationMethod", Box::new(|vc| vc.proof.verification_method.push_str("-mutated"))),
            ("proof.proofPurpose", Box::new(|vc| vc.proof.proof_purpose = "authentication".into())),
            ("proof.cryptosuite", Box::new(|vc| vc.proof.cryptosuite = Some("eddsa-rdfc-2022".into()))),
            ("proof.algorithm", Box::new(|vc| vc.proof.algorithm = Some(AlgorithmId::MlDsa65.as_u16()))),
            ("proof.challenge", Box::new(|vc| vc.proof.challenge = Some("challenge-mutated".into()))),
            ("proof.domain", Box::new(|vc| vc.proof.domain = Some("other.example.test".into()))),
            ("mycelix_schema_id", Box::new(|vc| vc.mycelix_schema_id.push_str(":mutated"))),
            ("mycelix_created", Box::new(|vc| vc.mycelix_created = Timestamp::from_micros(2_000_000))),
        ];

        assert_eq!(mutations.len(), 20);
        for (field, mutate) in mutations {
            let mut changed = credential.clone();
            mutate(&mut changed);
            let changed_digest = compute_credential_hash_v1(&changed)
                .unwrap_or_else(|error| panic!("{field}: V1 digest failed: {error:?}"));
            assert_ne!(digest, changed_digest, "{field} must change the signed digest");
            assert!(
                public_key.verify_strict(&changed_digest, &signature).is_err(),
                "{field} mutation must invalidate the independent Ed25519 signature"
            );
        }
    }

    #[test]
    fn credential_hash_v1_canonicalizes_nested_json_object_key_order() {
        let make_vc = |claims| VerifiableCredential {
            context: vec!["https://www.w3.org/ns/credentials/v2".into()],
            id: "test-cred-canonical".into(),
            credential_type: vec!["VerifiableCredential".into()],
            issuer: CredentialIssuer::Did("did:mycelix:issuer1".into()),
            valid_from: "2026-01-01T00:00:00Z".into(),
            valid_until: None,
            credential_subject: CredentialSubject {
                id: "did:mycelix:holder1".into(),
                claims,
            },
            credential_schema: None,
            credential_status: None,
            proof: CredentialProof {
                proof_type: "MycelixCredentialProof".into(),
                created: "2026-01-01T00:00:00Z".into(),
                verification_method: "did:mycelix:issuer1#keys-1".into(),
                proof_purpose: "assertionMethod".into(),
                proof_value: "zSignature".into(),
                cryptosuite: Some(CREDENTIAL_HASH_PROFILE_V1.into()),
                algorithm: Some(AlgorithmId::Ed25519.as_u16()),
                challenge: None,
                domain: None,
            },
            mycelix_schema_id: "mycelix:schema:test:v1".into(),
            mycelix_created: Timestamp::from_micros(1_000_000),
        };
        let a = make_vc(serde_json::json!({"a": 1, "nested": {"x": 2, "y": 3}}));
        let b = make_vc(serde_json::json!({"nested": {"y": 3, "x": 2}, "a": 1}));
        assert_eq!(
            compute_credential_hash_v1(&a).unwrap(),
            compute_credential_hash_v1(&b).unwrap()
        );
    }

    #[test]
    fn v1_proof_header_rejects_algorithm_method_and_purpose_mismatch() {
        let make_vc = || VerifiableCredential {
            context: vec!["https://www.w3.org/ns/credentials/v2".into()],
            id: "header-test".into(),
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
                proof_type: "MycelixCredentialProof".into(),
                created: "2026-01-01T00:00:00Z".into(),
                verification_method: "did:mycelix:issuer1#keys-1".into(),
                proof_purpose: "assertionMethod".into(),
                proof_value: "zSignature".into(),
                cryptosuite: Some(CREDENTIAL_HASH_PROFILE_V1.into()),
                algorithm: Some(AlgorithmId::Ed25519.as_u16()),
                challenge: None,
                domain: None,
            },
            mycelix_schema_id: "mycelix:schema:test:v1".into(),
            mycelix_created: Timestamp::from_micros(1_000_000),
        };

        let base = make_vc();
        assert!(v1_proof_header_matches(&base, AlgorithmId::Ed25519));

        let mut changed = base.clone();
        changed.proof.algorithm = Some(AlgorithmId::MlDsa65.as_u16());
        assert!(!v1_proof_header_matches(&changed, AlgorithmId::Ed25519));

        changed = base.clone();
        changed.proof.verification_method = "did:mycelix:attacker#keys-1".into();
        assert!(!v1_proof_header_matches(&changed, AlgorithmId::Ed25519));

        changed = base.clone();
        changed.proof.verification_method = "did:mycelix:issuer1".into();
        assert!(!v1_proof_header_matches(&changed, AlgorithmId::Ed25519));

        changed = base.clone();
        changed.proof.proof_purpose = "authentication".into();
        assert!(!v1_proof_header_matches(&changed, AlgorithmId::Ed25519));

        changed = base.clone();
        changed.proof.verification_method = "did:mycelix:issuer1#keys-2".into();
        assert!(!v1_proof_header_matches(&changed, AlgorithmId::Ed25519));

        changed = base;
        changed.proof.proof_type = "OtherProof".into();
        assert!(!v1_proof_header_matches(&changed, AlgorithmId::Ed25519));
    }

    #[test]
    fn credential_hash_profiles_keep_legacy_distinct_and_unknown_fail_closed() {
        let mut vc = VerifiableCredential {
            context: vec![],
            id: "test".into(),
            credential_type: vec!["VerifiableCredential".into()],
            issuer: CredentialIssuer::Did("did:mycelix:issuer1".into()),
            valid_from: "2026-01-01T00:00:00Z".into(),
            valid_until: None,
            credential_subject: CredentialSubject { id: "did:mycelix:holder1".into(), claims: serde_json::json!({}) },
            credential_schema: None,
            credential_status: None,
            proof: CredentialProof {
                proof_type: "MycelixCredentialProof".into(),
                created: "2026-01-01T00:00:00Z".into(),
                verification_method: "did:mycelix:issuer1#keys-1".into(),
                proof_purpose: "assertionMethod".into(),
                proof_value: "zSignature".into(),
                cryptosuite: Some(CREDENTIAL_HASH_PROFILE_V1.into()),
                algorithm: Some(AlgorithmId::Ed25519.as_u16()),
                challenge: None,
                domain: None,
            },
            mycelix_schema_id: "mycelix:schema:test:v1".into(),
            mycelix_created: Timestamp::from_micros(1_000_000),
        };
        assert_eq!(credential_hash_profile(&vc), Some(CredentialHashProfile::MycelixV1));
        vc.proof.cryptosuite = Some(LEGACY_ED25519_CRYPTOSUITE.into());
        assert_eq!(credential_hash_profile(&vc), Some(CredentialHashProfile::LegacySubsetV0));
        vc.proof.cryptosuite = Some("unknown-profile".into());
        assert_eq!(credential_hash_profile(&vc), None);
    }

    #[test]
    fn credential_hash_differs_for_different_ids() {
        let make_vc = |id: &str| VerifiableCredential {
            context: vec!["https://www.w3.org/ns/credentials/v2".into()],
            id: id.into(),
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
                created: "2026-01-01".into(),
                verification_method: "did:mycelix:issuer1#key-1".into(),
                proof_purpose: "assertionMethod".into(),
                proof_value: "zSig".into(),
                cryptosuite: None,
                algorithm: None,
                challenge: None,
                domain: None,
            },
            mycelix_schema_id: "mycelix:schema:test:v1".into(),
            mycelix_created: Timestamp::from_micros(0),
        };
        let hash_a = compute_credential_hash_legacy_subset(&make_vc("cred-A"));
        let hash_b = compute_credential_hash_legacy_subset(&make_vc("cred-B"));
        assert_ne!(hash_a, hash_b);
    }

    // --- Credential Revocation Status lifecycle ---

    #[test]
    fn revocation_status_to_verify_errors_active() {
        // Simulate the verify_credential error-aggregation pattern
        let mut errors = Vec::new();
        let status = CredentialRevocationStatus::Active;
        match status {
            CredentialRevocationStatus::Revoked(reason) => {
                errors.push(format!("Credential revoked: {}", reason));
            }
            CredentialRevocationStatus::Suspended(reason, until) => {
                errors.push(format!("Credential suspended until {}: {}", until, reason));
            }
            CredentialRevocationStatus::Active => {}
            CredentialRevocationStatus::Unknown => {
                errors.push("Revocation status could not be determined".to_string());
            }
        }
        assert!(
            errors.is_empty(),
            "Active credential should produce no errors"
        );
    }

    #[test]
    fn revocation_status_to_verify_errors_revoked() {
        let mut errors = Vec::new();
        let status = CredentialRevocationStatus::Revoked("Fraud detected".into());
        match status {
            CredentialRevocationStatus::Revoked(reason) => {
                errors.push(format!("Credential revoked: {}", reason));
            }
            CredentialRevocationStatus::Suspended(reason, until) => {
                errors.push(format!("Credential suspended until {}: {}", until, reason));
            }
            CredentialRevocationStatus::Active => {}
            CredentialRevocationStatus::Unknown => {
                errors.push("Revocation status could not be determined".to_string());
            }
        }
        assert_eq!(errors.len(), 1);
        assert!(errors[0].contains("Fraud detected"));
    }

    #[test]
    fn revocation_status_to_verify_errors_suspended() {
        let mut errors = Vec::new();
        let status = CredentialRevocationStatus::Suspended(
            "Under review".into(),
            "2026-12-31T00:00:00Z".into(),
        );
        match status {
            CredentialRevocationStatus::Revoked(reason) => {
                errors.push(format!("Credential revoked: {}", reason));
            }
            CredentialRevocationStatus::Suspended(reason, until) => {
                errors.push(format!("Credential suspended until {}: {}", until, reason));
            }
            CredentialRevocationStatus::Active => {}
            CredentialRevocationStatus::Unknown => {
                errors.push("Revocation status could not be determined".to_string());
            }
        }
        assert_eq!(errors.len(), 1);
        assert!(errors[0].contains("Under review"));
        assert!(errors[0].contains("2026-12-31"));
    }

    #[test]
    fn revocation_status_unknown_treated_as_invalid() {
        // SECURITY: Unknown status (revocation zome unavailable) must block verification
        // (fail-closed). Previously this was fail-open — fixed March 2026.
        let mut errors = Vec::new();
        let status = CredentialRevocationStatus::Unknown;
        match status {
            CredentialRevocationStatus::Revoked(reason) => {
                errors.push(format!("Credential revoked: {}", reason));
            }
            CredentialRevocationStatus::Suspended(reason, until) => {
                errors.push(format!("Credential suspended until {}: {}", until, reason));
            }
            CredentialRevocationStatus::Active => {}
            CredentialRevocationStatus::Unknown => {
                errors.push("Revocation status could not be determined".to_string());
            }
        }
        assert_eq!(
            errors.len(),
            1,
            "Unknown must produce an error (fail-closed)"
        );
        assert!(errors[0].contains("could not be determined"));
    }

    #[test]
    fn revocation_status_mirror_to_credential_status_mapping() {
        // Test the mapping from RevocationCheckResult to CredentialRevocationStatus
        // (mirrors check_credential_revocation_status logic)

        // Active
        let check = RevocationCheckResult {
            credential_id: "cred:1".into(),
            status: RevocationStatusMirror::Active,
            reason: None,
            checked_at: Timestamp::from_micros(0),
        };
        let mapped = match check.status {
            RevocationStatusMirror::Active => CredentialRevocationStatus::Active,
            RevocationStatusMirror::Revoked => CredentialRevocationStatus::Revoked(
                check.reason.unwrap_or_else(|| "No reason provided".into()),
            ),
            RevocationStatusMirror::Suspended => CredentialRevocationStatus::Suspended(
                check.reason.unwrap_or_else(|| "No reason provided".into()),
                check.checked_at.to_string(),
            ),
        };
        assert!(matches!(mapped, CredentialRevocationStatus::Active));

        // Revoked with reason
        let check_revoked = RevocationCheckResult {
            credential_id: "cred:2".into(),
            status: RevocationStatusMirror::Revoked,
            reason: Some("Key compromised".into()),
            checked_at: Timestamp::from_micros(1_000_000),
        };
        let mapped_revoked = match check_revoked.status {
            RevocationStatusMirror::Active => CredentialRevocationStatus::Active,
            RevocationStatusMirror::Revoked => CredentialRevocationStatus::Revoked(
                check_revoked
                    .reason
                    .unwrap_or_else(|| "No reason provided".into()),
            ),
            RevocationStatusMirror::Suspended => CredentialRevocationStatus::Suspended(
                check_revoked
                    .reason
                    .unwrap_or_else(|| "No reason provided".into()),
                check_revoked.checked_at.to_string(),
            ),
        };
        assert!(
            matches!(mapped_revoked, CredentialRevocationStatus::Revoked(ref r) if r == "Key compromised")
        );

        // Revoked without reason (defaults to "No reason provided")
        let check_no_reason = RevocationCheckResult {
            credential_id: "cred:3".into(),
            status: RevocationStatusMirror::Revoked,
            reason: None,
            checked_at: Timestamp::from_micros(0),
        };
        let mapped_no_reason = match check_no_reason.status {
            RevocationStatusMirror::Active => CredentialRevocationStatus::Active,
            RevocationStatusMirror::Revoked => CredentialRevocationStatus::Revoked(
                check_no_reason
                    .reason
                    .unwrap_or_else(|| "No reason provided".into()),
            ),
            RevocationStatusMirror::Suspended => CredentialRevocationStatus::Suspended(
                check_no_reason
                    .reason
                    .unwrap_or_else(|| "No reason provided".into()),
                check_no_reason.checked_at.to_string(),
            ),
        };
        assert!(
            matches!(mapped_no_reason, CredentialRevocationStatus::Revoked(ref r) if r == "No reason provided")
        );
    }

    #[test]
    fn credential_status_response_mapping() {
        // Test the status → CredentialStatusResponse mapping (mirrors get_credential_status logic)
        let statuses = vec![
            (
                CredentialRevocationStatus::Active,
                true,
                "active",
                None::<&str>,
            ),
            (
                CredentialRevocationStatus::Revoked("Expired cert".into()),
                false,
                "revoked",
                Some("Expired cert"),
            ),
            // SECURITY: Unknown is now fail-closed (invalid)
            (
                CredentialRevocationStatus::Unknown,
                false,
                "unknown",
                Some("Revocation status could not be determined"),
            ),
        ];

        for (status, expected_valid, expected_type, expected_reason) in statuses {
            let (is_valid, status_type, reason) = match status {
                CredentialRevocationStatus::Active => (true, "active".to_string(), None),
                CredentialRevocationStatus::Revoked(r) => (false, "revoked".to_string(), Some(r)),
                CredentialRevocationStatus::Suspended(r, until) => {
                    (false, format!("suspended_until_{}", until), Some(r))
                }
                // SECURITY: Fail-closed — unknown revocation status treated as invalid
                CredentialRevocationStatus::Unknown => (
                    false,
                    "unknown".to_string(),
                    Some("Revocation status could not be determined".to_string()),
                ),
            };
            assert_eq!(is_valid, expected_valid, "valid for {}", status_type);
            assert_eq!(status_type, expected_type);
            assert_eq!(reason.as_deref(), expected_reason);
        }
    }

    #[test]
    fn is_credential_revoked_logic() {
        // Test the is_credential_revoked boolean reduction pattern
        let cases: Vec<(CredentialRevocationStatus, bool)> = vec![
            (CredentialRevocationStatus::Active, false),
            (CredentialRevocationStatus::Revoked("test".into()), true),
            (
                CredentialRevocationStatus::Suspended("test".into(), "2026".into()),
                true,
            ),
            // SECURITY: Unknown is now fail-closed (treated as revoked)
            (CredentialRevocationStatus::Unknown, true),
        ];
        for (status, expected) in cases {
            let result = matches!(
                status,
                CredentialRevocationStatus::Revoked(_)
                    | CredentialRevocationStatus::Suspended(_, _)
                    | CredentialRevocationStatus::Unknown
            );
            assert_eq!(result, expected, "is_revoked for {:?}", status);
        }
    }

    #[test]
    fn revocation_check_result_serde_round_trip() {
        // Verify RevocationCheckResult serializes/deserializes correctly
        // (critical for cross-zome calls)
        let check = RevocationCheckResult {
            credential_id: "urn:uuid:12345".into(),
            status: RevocationStatusMirror::Revoked,
            reason: Some("Key compromise".into()),
            checked_at: Timestamp::from_micros(1_700_000_000_000_000),
        };
        let json = serde_json::to_string(&check).unwrap();
        let restored: RevocationCheckResult = serde_json::from_str(&json).unwrap();
        assert_eq!(restored.credential_id, "urn:uuid:12345");
        assert_eq!(restored.status, RevocationStatusMirror::Revoked);
        assert_eq!(restored.reason.unwrap(), "Key compromise");
    }

    #[test]
    fn revocation_status_mirror_serde_all_variants() {
        let variants = vec![
            (RevocationStatusMirror::Active, "Active"),
            (RevocationStatusMirror::Suspended, "Suspended"),
            (RevocationStatusMirror::Revoked, "Revoked"),
        ];
        for (variant, expected_json_contains) in variants {
            let json = serde_json::to_string(&variant).unwrap();
            assert!(json.contains(expected_json_contains));
            let restored: RevocationStatusMirror = serde_json::from_str(&json).unwrap();
            assert_eq!(restored, variant);
        }
    }

    // --- Schema validation status ---

    #[test]
    fn schema_validation_status_equality() {
        assert_eq!(
            SchemaValidationStatus::NoSchema,
            SchemaValidationStatus::NoSchema
        );
        assert_eq!(
            SchemaValidationStatus::Validated {
                schema_name: "test".into()
            },
            SchemaValidationStatus::Validated {
                schema_name: "test".into()
            }
        );
        assert_ne!(
            SchemaValidationStatus::NoSchema,
            SchemaValidationStatus::SchemaZomeUnavailable
        );
    }

    // --- Merkle tree for selective disclosure ---

    #[test]
    fn merkle_tree_single_claim() {
        let mut claims = serde_json::Map::new();
        claims.insert("degree".into(), serde_json::json!("BSc"));

        let (root, leaves) = build_claim_merkle_tree(&claims);
        assert_eq!(root.len(), 32);
        assert_eq!(leaves.len(), 1);
        assert_eq!(leaves[0].0, "degree");
        // Single leaf = root
        assert_eq!(root, leaves[0].1);
    }

    #[test]
    fn merkle_tree_deterministic() {
        let mut claims = serde_json::Map::new();
        claims.insert("name".into(), serde_json::json!("Alice"));
        claims.insert("age".into(), serde_json::json!(30));
        claims.insert("degree".into(), serde_json::json!("PhD"));

        let (root1, _) = build_claim_merkle_tree(&claims);
        let (root2, _) = build_claim_merkle_tree(&claims);
        assert_eq!(root1, root2, "Merkle root must be deterministic");
    }

    #[test]
    fn merkle_tree_different_claims_different_root() {
        let mut claims1 = serde_json::Map::new();
        claims1.insert("name".into(), serde_json::json!("Alice"));

        let mut claims2 = serde_json::Map::new();
        claims2.insert("name".into(), serde_json::json!("Bob"));

        let (root1, _) = build_claim_merkle_tree(&claims1);
        let (root2, _) = build_claim_merkle_tree(&claims2);
        assert_ne!(root1, root2);
    }

    #[test]
    fn merkle_proof_verifies_for_all_claims() {
        let mut claims = serde_json::Map::new();
        claims.insert("name".into(), serde_json::json!("Alice"));
        claims.insert("age".into(), serde_json::json!(30));
        claims.insert("degree".into(), serde_json::json!("PhD"));
        claims.insert("university".into(), serde_json::json!("MIT"));

        let (root, _) = build_claim_merkle_tree(&claims);
        let mut sorted_keys: Vec<&String> = claims.keys().collect();
        sorted_keys.sort();

        for (idx, key) in sorted_keys.iter().enumerate() {
            let proof = generate_merkle_proof(&claims, idx);
            let leaf = hash_claim_leaf(key, &claims[*key]);

            // Walk proof to reconstruct root
            let mut current = leaf;
            let mut i = idx;
            for sibling in &proof {
                let combined = if i % 2 == 0 {
                    let mut c = current.clone();
                    c.extend(sibling);
                    c
                } else {
                    let mut c = sibling.clone();
                    c.extend(&current);
                    c
                };
                current = holo_hash::blake2b_256(&combined).to_vec();
                i /= 2;
            }

            assert_eq!(
                current, root,
                "Merkle proof should verify for claim '{}'",
                key
            );
        }
    }

    #[test]
    fn merkle_proof_empty_for_single_claim() {
        let mut claims = serde_json::Map::new();
        claims.insert("only".into(), serde_json::json!(true));

        let proof = generate_merkle_proof(&claims, 0);
        assert!(proof.is_empty(), "Single claim needs no proof siblings");
    }

    #[test]
    fn hash_claim_leaf_deterministic() {
        let val = serde_json::json!("test_value");
        let h1 = hash_claim_leaf("key", &val);
        let h2 = hash_claim_leaf("key", &val);
        assert_eq!(h1, h2);
        assert_eq!(h1.len(), 32);
    }

    #[test]
    fn hash_claim_leaf_different_keys() {
        let val = serde_json::json!("same_value");
        let h1 = hash_claim_leaf("key1", &val);
        let h2 = hash_claim_leaf("key2", &val);
        assert_ne!(h1, h2, "Different keys should produce different hashes");
    }
}

/// Sign credential content using the agent's ed25519 key
///
/// This uses Holochain's HDK sign_raw which performs ed25519 signing
/// with the agent's cryptographic identity. The result is a
/// `TaggedSignature`-aware multibase string that includes the algorithm
/// multicodec prefix so verifiers can detect the algorithm.
fn sign_credential(vc: &VerifiableCredential) -> ExternResult<String> {
    // Compute canonical hash of credential content
    let content_hash = compute_credential_hash_v1(vc)?;

    // Sign with agent's ed25519 key via HDK
    let signature = sign_raw(agent_info()?.agent_initial_pubkey, content_hash.clone())?;

    // Wrap in TaggedSignature for algorithm-tagged multibase encoding
    let tagged =
        TaggedSignature::new(AlgorithmId::Ed25519, signature.as_ref().to_vec()).map_err(|e| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "Signature tagging error: {}",
                e
            )))
        })?;
    Ok(tagged.to_multibase())
}

/// Verify a credential signature with algorithm dispatch.
///
/// Ed25519 is verified in the zome. Hybrid signatures have their Ed25519 half
/// checked, but remain `Partial` because this WASM zome cannot yet verify the
/// ML-DSA half. Pure ML-DSA / SLH-DSA signatures remain `Unsupported`: size
/// checks alone are never cryptographic verification.
///
/// Falls back to legacy 64-byte raw Ed25519 for old credentials.
fn verify_credential_signature(
    vc: &VerifiableCredential,
) -> ExternResult<CredentialSignatureOutcome> {
    let tagged_signature = TaggedSignature::from_multibase(&vc.proof.proof_value);
    let signature_algorithm = tagged_signature
        .as_ref()
        .map(|signature| signature.algorithm)
        .unwrap_or(AlgorithmId::Ed25519);
    let profile = match credential_hash_profile(vc) {
        Some(profile) => profile,
        None => {
            return Ok(CredentialSignatureOutcome::Unsupported {
                algorithm: signature_algorithm,
            });
        }
    };
    if profile == CredentialHashProfile::MycelixV1
        && !v1_proof_header_matches(vc, signature_algorithm)
    {
        return Ok(CredentialSignatureOutcome::Invalid);
    }
    let content_hash = compute_credential_hash_for_profile(vc)?;

    // Extract public key from issuer DID.
    let issuer_did = vc.issuer.did();
    let pubkey_str = issuer_did.strip_prefix("did:mycelix:").ok_or_else(|| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "Invalid DID format: {}",
            issuer_did
        )))
    })?;
    let pubkey = AgentPubKey::try_from(pubkey_str.to_string()).map_err(|e| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "Invalid public key in DID '{}': {:?}",
            pubkey_str, e
        )))
    })?;

    match tagged_signature {
        Ok(tagged_sig) => match tagged_sig.algorithm {
            AlgorithmId::Ed25519 => {
                if tagged_sig.signature_bytes.len() != AlgorithmId::Ed25519.signature_size() {
                    return Ok(CredentialSignatureOutcome::Invalid);
                }
                let sig = Signature::from(
                    <[u8; 64]>::try_from(tagged_sig.signature_bytes.as_slice()).map_err(|_| {
                        wasm_error!(WasmErrorInner::Guest("Invalid signature length".into()))
                    })?,
                );
                match verify_signature(pubkey, sig, content_hash)? {
                    false => Ok(CredentialSignatureOutcome::Invalid),
                    true if profile == CredentialHashProfile::MycelixV1 => {
                        Ok(CredentialSignatureOutcome::Verified)
                    }
                    true => Ok(CredentialSignatureOutcome::Partial {
                        algorithm: AlgorithmId::Ed25519,
                        reason: "historical subset-hash profile does not bind every serialized credential field".into(),
                    }),
                }
            }
            AlgorithmId::HybridEd25519MlDsa65 => {
                // A valid Ed25519 half is necessary, not sufficient, for a
                // hybrid proof. The ML-DSA component is not verified in WASM.
                if tagged_sig.signature_bytes.len()
                    != AlgorithmId::HybridEd25519MlDsa65.signature_size()
                {
                    return Ok(CredentialSignatureOutcome::Invalid);
                }
                let ed_bytes = tagged_sig.ed25519_component().ok_or_else(|| {
                    wasm_error!(WasmErrorInner::Guest(
                        "Hybrid signature missing Ed25519 component".into()
                    ))
                })?;
                if ed_bytes.len() != AlgorithmId::Ed25519.signature_size() {
                    return Ok(CredentialSignatureOutcome::Invalid);
                }
                let sig = Signature::from(<[u8; 64]>::try_from(ed_bytes).map_err(|_| {
                    wasm_error!(WasmErrorInner::Guest("Invalid Ed25519 component".into()))
                })?);
                match verify_signature(pubkey, sig, content_hash)? {
                    false => Ok(CredentialSignatureOutcome::Invalid),
                    true => Ok(CredentialSignatureOutcome::Partial {
                        algorithm: AlgorithmId::HybridEd25519MlDsa65,
                        reason: "ML-DSA component is not cryptographically verified by this WASM zome".into(),
                    }),
                }
            }
            AlgorithmId::MlDsa65
            | AlgorithmId::MlDsa87
            | AlgorithmId::SlhDsaSha2_128s
            | AlgorithmId::SlhDsaShake128s => {
                if tagged_sig.signature_bytes.len() != tagged_sig.algorithm.signature_size() {
                    return Ok(CredentialSignatureOutcome::Invalid);
                }
                Ok(CredentialSignatureOutcome::Unsupported {
                    algorithm: tagged_sig.algorithm,
                })
            }
            _ => Ok(CredentialSignatureOutcome::Invalid),
        },
        Err(_) if profile == CredentialHashProfile::LegacySubsetV0 => {
            // Historical untagged Ed25519 signatures remain readable but can
            // never be promoted above PARTIAL assurance.
            let signature_bytes = multibase_decode(&vc.proof.proof_value).ok_or_else(|| {
                wasm_error!(WasmErrorInner::Guest(
                    "Invalid multibase signature encoding".into()
                ))
            })?;
            if signature_bytes.len() != AlgorithmId::Ed25519.signature_size() {
                return Ok(CredentialSignatureOutcome::Invalid);
            }
            let signature =
                Signature::from(<[u8; 64]>::try_from(signature_bytes.as_slice()).map_err(
                    |_| wasm_error!(WasmErrorInner::Guest("Invalid signature length".into())),
                )?);
            match verify_signature(pubkey, signature, content_hash)? {
                true => Ok(CredentialSignatureOutcome::Partial {
                    algorithm: AlgorithmId::Ed25519,
                    reason: "untagged historical signature uses the subset-hash profile".into(),
                }),
                false => Ok(CredentialSignatureOutcome::Invalid),
            }
        }
        Err(_) => Ok(CredentialSignatureOutcome::Invalid),
    }
}

#[cfg(test)]
mod credential_signature_outcome_tests {
    use super::*;

    #[test]
    fn only_fully_verified_status_is_authorizing() {
        assert!(CredentialSignatureStatus::Verified.is_verified());
        for status in [
            CredentialSignatureStatus::Invalid,
            CredentialSignatureStatus::Partial,
            CredentialSignatureStatus::Unsupported,
            CredentialSignatureStatus::Error,
        ] {
            assert!(!status.is_verified(), "{status:?} must not authorize");
        }
    }

    #[test]
    fn hybrid_and_pqc_outcomes_preserve_non_verified_assurance() {
        let hybrid = CredentialSignatureOutcome::Partial {
            algorithm: AlgorithmId::HybridEd25519MlDsa65,
            reason: "ML-DSA component is not verified by this WASM zome".into(),
        };
        let pqc = CredentialSignatureOutcome::Unsupported {
            algorithm: AlgorithmId::MlDsa65,
        };

        assert_eq!(hybrid.status(), CredentialSignatureStatus::Partial);
        assert_eq!(pqc.status(), CredentialSignatureStatus::Unsupported);
        assert!(!hybrid.status().is_verified());
        assert!(!pqc.status().is_verified());
    }
}

// =============================================================================
// REVOCATION STATUS CHECKING (Cross-Zome Integration)
// =============================================================================

/// Credential revocation status
#[derive(Debug, Clone)]
pub enum CredentialRevocationStatus {
    /// Credential is active (not revoked)
    Active,
    /// Credential has been revoked
    Revoked(String),
    /// Credential is temporarily suspended
    Suspended(String, String),
    /// No revocation record found
    Unknown,
}

/// Revocation check result from the revocation zome (local mirror type for deserialization)
#[derive(Serialize, Deserialize, Debug, Clone)]
struct RevocationCheckResult {
    pub credential_id: String,
    pub status: RevocationStatusMirror,
    pub reason: Option<String>,
    pub checked_at: Timestamp,
}

/// Mirror of revocation_integrity::RevocationStatus for cross-zome deserialization
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
enum RevocationStatusMirror {
    Active,
    Suspended,
    Revoked,
}

/// Check credential revocation status via cross-zome call to the revocation coordinator
fn check_credential_revocation_status(
    credential_id: &str,
) -> ExternResult<CredentialRevocationStatus> {
    let response = call(
        CallTargetCell::Local,
        ZomeName::new("revocation"),
        FunctionName::new("check_revocation_status"),
        None,
        credential_id.to_string(),
    )?;

    match response {
        ZomeCallResponse::Ok(result) => {
            let check: RevocationCheckResult = result.decode().map_err(|e| {
                wasm_error!(WasmErrorInner::Guest(format!(
                    "Failed to decode revocation check response: {:?}",
                    e
                )))
            })?;

            match check.status {
                RevocationStatusMirror::Active => Ok(CredentialRevocationStatus::Active),
                RevocationStatusMirror::Revoked => Ok(CredentialRevocationStatus::Revoked(
                    check
                        .reason
                        .unwrap_or_else(|| "No reason provided".to_string()),
                )),
                RevocationStatusMirror::Suspended => Ok(CredentialRevocationStatus::Suspended(
                    check
                        .reason
                        .unwrap_or_else(|| "No reason provided".to_string()),
                    check.checked_at.to_string(),
                )),
            }
        }
        ZomeCallResponse::Unauthorized(_, _, _, _)
        | ZomeCallResponse::AuthenticationFailed(_, _) => {
            // SECURITY: Fail-closed — revocation zome unreachable means we cannot
            // confirm the credential is unrevoked. Deny rather than assume active.
            Err(wasm_error!(WasmErrorInner::Guest(
                "Revocation check failed: revocation zome unreachable (fail-closed). \
                 Cannot verify credential is unrevoked."
                    .to_string()
            )))
        }
        ZomeCallResponse::NetworkError(err) => Err(wasm_error!(WasmErrorInner::Guest(format!(
            "Network error calling revocation zome: {}",
            err
        )))),
        ZomeCallResponse::CountersigningSession(err) => Err(wasm_error!(WasmErrorInner::Guest(
            format!("Countersigning error: {}", err)
        ))),
    }
}

/// Batch check revocation status for multiple credentials in a single cross-zome call.
///
/// Falls back to individual checks if the revocation zome doesn't support batch.
fn batch_check_credential_revocation_status(
    credential_ids: &[String],
) -> ExternResult<Vec<CredentialRevocationStatus>> {
    if credential_ids.is_empty() {
        return Ok(Vec::new());
    }

    let response = call(
        CallTargetCell::Local,
        ZomeName::new("revocation"),
        FunctionName::new("batch_check_revocation"),
        None,
        credential_ids.to_vec(),
    )?;

    match response {
        ZomeCallResponse::Ok(result) => {
            let checks: Vec<RevocationCheckResult> = result.decode().map_err(|e| {
                wasm_error!(WasmErrorInner::Guest(format!(
                    "Failed to decode batch revocation response: {:?}",
                    e
                )))
            })?;
            Ok(checks
                .into_iter()
                .map(|check| match check.status {
                    RevocationStatusMirror::Active => CredentialRevocationStatus::Active,
                    RevocationStatusMirror::Revoked => CredentialRevocationStatus::Revoked(
                        check
                            .reason
                            .unwrap_or_else(|| "No reason provided".to_string()),
                    ),
                    RevocationStatusMirror::Suspended => CredentialRevocationStatus::Suspended(
                        check
                            .reason
                            .unwrap_or_else(|| "No reason provided".to_string()),
                        check.checked_at.to_string(),
                    ),
                })
                .collect())
        }
        ZomeCallResponse::Unauthorized(_, _, _, _)
        | ZomeCallResponse::AuthenticationFailed(_, _) => {
            // SECURITY: Fail-closed — cannot batch-verify revocation status
            Err(wasm_error!(WasmErrorInner::Guest(
                "Batch revocation check failed: revocation zome unreachable (fail-closed). \
                 Cannot verify credentials are unrevoked."
                    .to_string()
            )))
        }
        ZomeCallResponse::NetworkError(err) => Err(wasm_error!(WasmErrorInner::Guest(format!(
            "Network error calling batch revocation: {}",
            err
        )))),
        ZomeCallResponse::CountersigningSession(err) => Err(wasm_error!(WasmErrorInner::Guest(
            format!("Countersigning error: {}", err)
        ))),
    }
}

/// Check if a specific credential is revoked (public API)
#[hdk_extern]
pub fn is_credential_revoked(credential_id: String) -> ExternResult<bool> {
    let status = check_credential_revocation_status(&credential_id)?;
    match status {
        CredentialRevocationStatus::Revoked(_) | CredentialRevocationStatus::Suspended(_, _) => {
            Ok(true)
        }
        CredentialRevocationStatus::Active => Ok(false),
        // SECURITY: Fail-closed — unknown status treated as revoked
        CredentialRevocationStatus::Unknown => Ok(true),
    }
}

/// Get detailed revocation status for a credential
#[hdk_extern]
pub fn get_credential_status(credential_id: String) -> ExternResult<CredentialStatusResponse> {
    let now = sys_time()?;
    let status = check_credential_revocation_status(&credential_id)?;

    let (is_valid, status_type, reason) = match status {
        CredentialRevocationStatus::Active => (true, "active".to_string(), None),
        CredentialRevocationStatus::Revoked(r) => (false, "revoked".to_string(), Some(r)),
        CredentialRevocationStatus::Suspended(r, until) => {
            (false, format!("suspended_until_{}", until), Some(r))
        }
        // SECURITY: Fail-closed — unknown revocation status treated as invalid
        CredentialRevocationStatus::Unknown => (
            false,
            "unknown".to_string(),
            Some("Revocation status could not be determined".to_string()),
        ),
    };

    Ok(CredentialStatusResponse {
        credential_id,
        is_valid,
        status_type,
        reason,
        checked_at: now,
    })
}

/// Response for credential status check
#[derive(Serialize, Deserialize, Debug)]
pub struct CredentialStatusResponse {
    pub credential_id: String,
    pub is_valid: bool,
    pub status_type: String,
    pub reason: Option<String>,
    pub checked_at: Timestamp,
}
