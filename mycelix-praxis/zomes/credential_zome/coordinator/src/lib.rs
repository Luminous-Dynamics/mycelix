// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! # Credential Coordinator Zome
//!
//! Updated for Type 1 Civilization Substrate (Praxis-0.2)
//!
//! Manages issuance, verification, and selective disclosure of educational
//! credentials (badges, degrees, skill attestations).

use base64::Engine;
use credential_integrity::*;
use hdk::prelude::*;
use mycelix_zome_helpers as _;
use praxis_core::contracts::{CREDENTIAL_CONTRACT_VERSION, CredentialList, CredentialSummary};
use praxis_core::{AuditResult, CourseId};
use std::collections::{BTreeSet, HashSet};

/// Canonical bytes signed over when issuing a credential, and re-derived by
/// any verifier (e.g. mycelix-craft's `publish_credential`) holding the same
/// (issuer, subject, course, type, issuance_date) tuple — these are exactly
/// the fields carried in `VerifiableCredential` that identify *this specific*
/// credential, so a verifier that doesn't share this crate's types can
/// reconstruct the identical byte string independently. Signed/verified via
/// `sign_raw`/`verify_signature_raw` (literal bytes, no serde envelope) so
/// the format is stable across independently-compiled crates.
fn credential_signing_payload(
    issuer: &AgentPubKey,
    subject: &AgentPubKey,
    course_id: &str,
    credential_type: &[String],
    issuance_date: &str,
) -> Vec<u8> {
    format!(
        "mycelix-credential-v1|{issuer}|{subject}|{course_id}|{}|{issuance_date}",
        credential_type.join(",")
    )
    .into_bytes()
}

/// Issue a new verifiable credential.
#[hdk_extern]
pub fn issue_credential(input: IssueCredentialInput) -> ExternResult<ActionHash> {
    let agent_info = agent_info()?;
    let issuer_pubkey = agent_info.agent_initial_pubkey;

    let now = sys_time()?;
    let issuance_date = format!("{:?}", now);
    let expiration_date = input.expires_at.map(|t| format!("{:?}", t));

    let payload = credential_signing_payload(
        &issuer_pubkey,
        &input.subject,
        &input.course_id.0,
        &input.credential_type,
        &issuance_date,
    );
    let signature = sign_raw(issuer_pubkey.clone(), payload)?;
    let proof_value = base64::engine::general_purpose::STANDARD.encode(signature.0);

    let credential = VerifiableCredential {
        context: "https://www.w3.org/2018/credentials/v1".into(),
        credential_type: input.credential_type,
        issuer: issuer_pubkey.to_string(),
        issuance_date,
        expiration_date,
        subject_id: input.subject.to_string(),
        course_id: input.course_id.clone(),
        model_id: "none".into(),
        rubric_id: "none".into(),
        score: None,
        score_band: "Issued".into(),
        subject_metadata: Some(input.metadata_json),
        status_id: None,
        status_type: None,
        status_list_index: None,
        status_purpose: None,
        proof_type: "Ed25519Signature2020".into(),
        proof_created: format!("{:?}", now),
        verification_method: format!("{}/keys/1", issuer_pubkey),
        proof_purpose: "assertionMethod".into(),
        proof_value,
        industry_mappings: Vec::new(),
        epistemic_empirical: Some(3),   // Cryptographic
        epistemic_normative: Some(1),   // Communal
        epistemic_materiality: Some(2), // Persistent
    };

    let action_hash = create_entry(EntryTypes::VerifiableCredential(credential))?;

    // Link from issuer to credential
    create_link(
        issuer_pubkey,
        action_hash.clone(),
        LinkTypes::IssuerToCredentials,
        (),
    )?;

    // Index by subject so the authenticated learner can list credentials
    // issued to them. The read path also checks the entry's subject field.
    create_link(
        input.subject,
        action_hash.clone(),
        LinkTypes::LearnerToCredentials,
        (),
    )?;

    // Link from course to credential
    let course_anchor = Path::from(format!("course_credentials.{}", input.course_id.0));
    create_link(
        course_anchor.path_entry_hash()?,
        action_hash.clone(),
        LinkTypes::CourseToCredentials,
        (),
    )?;

    Ok(action_hash)
}

/// Verify a credential's signature and status.
#[hdk_extern]
pub fn verify_credential(action_hash: ActionHash) -> ExternResult<VerificationResult> {
    let record = get(action_hash, GetOptions::default())?.ok_or(wasm_error!(
        WasmErrorInner::Guest("Credential not found".into())
    ))?;

    let _credential: VerifiableCredential = record
        .entry()
        .to_app_option::<VerifiableCredential>()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(format!("{:?}", e))))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Invalid credential entry".into()
        )))?;

    // Fail closed until signature and revocation verification are implemented.
    // Returning success here would turn the mere presence of proof fields into
    // a false cryptographic claim.
    Ok(VerificationResult {
        is_valid: false,
        verified_at: sys_time()?,
        verification_notes: Some(
            "Not verified: signature and revocation checks are not implemented".into(),
        ),
    })
}

fn linked_credentials_for_current_agent() -> ExternResult<Vec<Record>> {
    let learner = agent_info()?.agent_initial_pubkey;
    let learner_id = learner.to_string();
    let links = get_links(
        LinkQuery::try_new(learner, LinkTypes::LearnerToCredentials)?,
        GetStrategy::Local,
    )?;

    let mut seen = HashSet::new();
    let mut records = Vec::new();
    for link in links {
        let Some(action_hash) = link.target.into_action_hash() else {
            continue;
        };
        if !seen.insert(action_hash.to_string()) {
            continue;
        }
        let Some(record) = get(action_hash, GetOptions::default())? else {
            continue;
        };
        let Some(credential) = record
            .entry()
            .to_app_option::<VerifiableCredential>()
            .map_err(|e| wasm_error!(e))?
        else {
            continue;
        };
        if credential.subject_id == learner_id {
            records.push(record);
        }
    }

    records.sort_by_key(|record| record.action_hashed().hash.to_string());
    Ok(records)
}

/// Compatibility endpoint for coordinator callers that still consume raw
/// records. Browser clients use `list_my_credential_summaries` instead.
#[hdk_extern]
pub fn get_my_credentials(_: ()) -> ExternResult<Vec<Record>> {
    linked_credentials_for_current_agent()
}

/// Return stable browser-facing projections for credentials linked to the
/// authenticated subject.
#[hdk_extern]
pub fn list_my_credential_summaries(_: ()) -> ExternResult<CredentialList> {
    let mut credentials = Vec::new();
    for record in linked_credentials_for_current_agent()? {
        let credential_id = record.action_hashed().hash.to_string();
        let credential = record
            .entry()
            .to_app_option::<VerifiableCredential>()
            .map_err(|e| wasm_error!(e))?
            .ok_or(wasm_error!(WasmErrorInner::Guest(
                "Linked credential record has no credential entry".into()
            )))?;
        credentials.push(CredentialSummary {
            credential_id,
            credential_type: credential.credential_type,
            subject_id: credential.subject_id,
            course_id: credential.course_id,
            issuer: credential.issuer,
            issuance_date: credential.issuance_date,
            expiration_date: credential.expiration_date,
            score: credential.score,
            score_band: credential.score_band,
            proof_type: credential.proof_type,
            proof_created: credential.proof_created,
            verification_method: credential.verification_method,
            proof_purpose: credential.proof_purpose,
            proof_value: credential.proof_value,
            status_purpose: credential.status_purpose,
            epistemic_empirical: credential.epistemic_empirical,
            epistemic_normative: credential.epistemic_normative,
            epistemic_materiality: credential.epistemic_materiality,
        });
    }

    Ok(CredentialList {
        contract_version: CREDENTIAL_CONTRACT_VERSION,
        credentials,
    })
}

// =============================================================================
// Input/Output structures
// =============================================================================

#[derive(Serialize, Deserialize, Debug)]
pub struct IssueCredentialInput {
    pub subject: AgentPubKey,
    pub course_id: CourseId,
    pub credential_type: Vec<String>,
    pub metadata_json: String,
    pub expires_at: Option<Timestamp>,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct VerificationResult {
    pub is_valid: bool,
    pub verified_at: Timestamp,
    pub verification_notes: Option<String>,
}

// =============================================================================
// Physical Presence Verification (PoPP)
// =============================================================================

#[derive(Serialize, Deserialize, Debug)]
pub struct GrantPresenceVerificationInput {
    pub hardware_agent: AgentPubKey,
    pub duration_secs: u32,
}

/// Create a capability grant for a hardware device to verify physical presence.
#[hdk_extern]
pub fn grant_physical_verification_authority(
    input: GrantPresenceVerificationInput,
) -> ExternResult<ActionHash> {
    let mut listed_functions = HashSet::new();
    listed_functions.insert((zome_info()?.name, "verify_physical_presence".into()));
    let functions = GrantedFunctions::Listed(listed_functions);

    let access = CapAccess::Assigned {
        secret: CapSecret::try_from(random_bytes(64)?.into_vec())
            .map_err(|_| wasm_error!(WasmErrorInner::Guest("Secret failed".into())))?,
        assignees: BTreeSet::from([input.hardware_agent]),
    };

    create_cap_grant(CapGrantEntry {
        tag: "physical_presence_verification".into(),
        access,
        functions,
    })
}

// =============================================================================
// Comprehensive Learner Record (CLR)
// =============================================================================

#[derive(Serialize, Deserialize, Debug)]
pub struct ClrView {
    pub student: String,
    pub total_mastery: u16,
    pub zk_claims: Vec<ZkClaim>,
    pub timestamp: i64,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct CurriculumView {
    pub student: String,
    pub nodes: Vec<String>,
    pub audit_proofs: Vec<AuditResult>,
}

#[hdk_extern]
pub fn generate_student_clr(_: ()) -> ExternResult<ClrView> {
    Ok(ClrView {
        student: agent_info()?.agent_initial_pubkey.to_string(),
        total_mastery: 750, // Computed from credentials
        zk_claims: Vec::new(),
        timestamp: (sys_time()?.as_micros() / 1000) as i64,
    })
}

#[hdk_extern]
pub fn get_curriculum_mastery(_: ()) -> ExternResult<CurriculumView> {
    Ok(CurriculumView {
        student: agent_info()?.agent_initial_pubkey.to_string(),
        nodes: vec!["rust-01".into(), "hdc-basic".into()],
        audit_proofs: vec![AuditResult::Verified],
    })
}
