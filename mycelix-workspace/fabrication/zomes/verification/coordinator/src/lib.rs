// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Verification Coordinator Zome
//!
//! Functions for submitting verifications, safety claims, and
//! bridging to the Knowledge hApp for epistemic scoring.

use hdk::prelude::*;
use verification_integrity::*;
use fabrication_common::*;

use sha2::{Digest, Sha256};
use std::cell::RefCell;
use std::collections::HashMap;

const EPISTEMIC_CACHE_TTL_MICROS: i64 = 300_000_000; // 5 min
const EPISTEMIC_CACHE_MAX_ENTRIES: usize = 128;
const EPISTEMIC_CACHE_DOMAIN: &[u8] = b"mycelix-fabrication-epistemic-cache:v1";

thread_local! {
    static CONFIG: RefCell<Option<FabricationConfig>> = const { RefCell::new(None) };
    static EPISTEMIC_CACHE: RefCell<HashMap<[u8; 32], (i64, ClaimEpistemic)>> = RefCell::new(HashMap::new());
}

fn get_config() -> FabricationConfig {
    CONFIG.with(|c| {
        c.borrow_mut()
            .get_or_insert_with(|| {
                dna_info()
                    .map(|info| FabricationConfig::from_properties_or_default(info.modifiers.properties.bytes()))
                    .unwrap_or_default()
            })
            .clone()
    })
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct CreateFpmRegistrationAnchorInput {
    pub envelope: RegistrationEnvelope,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ResolveFpmRegistrationActionAnchorInput {
    pub action_hash: ActionHash,
    pub claimed_envelope_digest: String,
    pub expected_author: Option<AgentPubKey>,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ResolveFpmRegistrationEntryAnchorInput {
    pub entry_hash: EntryHash,
    pub claimed_envelope_digest: String,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ResolvedFpmRegistrationAnchor {
    pub anchor_kind: RegistrationAnchorKind,
    pub anchor_reference: String,
    pub registration_envelope_digest: String,
    pub entry_hash: EntryHash,
    pub action_hash: Option<ActionHash>,
    pub author: Option<AgentPubKey>,
    pub timestamp: Option<Timestamp>,
    pub envelope: RegistrationEnvelope,
}

fn valid_fpm_digest(value: &str) -> bool {
    value.len() == 64
        && value.bytes().all(|byte| matches!(byte, b'0'..=b'9' | b'a'..=b'f'))
}

fn fpm_anchor_error(reason: impl Into<String>) -> WasmError {
    FabricationError::ValidationFailed {
        field: "fpm_registration_anchor".into(),
        reason: reason.into(),
    }
    .to_wasm_error()
}

fn validate_resolved_anchor_envelope(
    anchor: &FpmRegistrationAnchor,
    claimed_envelope_digest: &str,
) -> ExternResult<()> {
    if anchor.schema_version != FPM_REGISTRATION_ANCHOR_SCHEMA_VERSION {
        return Err(fpm_anchor_error("unsupported registration anchor schema"));
    }
    if !valid_fpm_digest(claimed_envelope_digest) {
        return Err(fpm_anchor_error("claimed envelope digest is not canonical SHA-256"));
    }
    if !valid_fpm_digest(&anchor.envelope_digest) {
        return Err(fpm_anchor_error("stored envelope digest is not canonical SHA-256"));
    }
    if anchor.envelope_digest != claimed_envelope_digest {
        return Err(fpm_anchor_error("claimed envelope digest does not match stored anchor"));
    }
    let computed = anchor.envelope.digest().map_err(|e| {
        fpm_anchor_error(format!("failed to hash anchored registration envelope: {e}"))
    })?;
    if computed != anchor.envelope_digest {
        return Err(fpm_anchor_error("stored envelope digest does not match envelope content"));
    }
    Ok(())
}

#[hdk_extern]
pub fn create_fpm_registration_anchor(
    input: CreateFpmRegistrationAnchorInput,
) -> ExternResult<Record> {
    rate_limit_caller()?;

    let envelope_digest = input.envelope.digest().map_err(|e| {
        fpm_anchor_error(format!("failed to hash registration envelope: {e}"))
    })?;

    let anchor = FpmRegistrationAnchor {
        schema_version: FPM_REGISTRATION_ANCHOR_SCHEMA_VERSION.into(),
        envelope: input.envelope,
        envelope_digest,
    };

    let action_hash =
        create_entry(EntryTypes::FpmRegistrationAnchor(anchor))?;

    get(action_hash, GetOptions::default())?
        .ok_or_else(|| FabricationError::not_found(
            "FpmRegistrationAnchor",
            &"newly-created action",
        ))
}

fn resolve_action_anchor(
    input: ResolveFpmRegistrationActionAnchorInput,
) -> ExternResult<ResolvedFpmRegistrationAnchor> {
    let details = get_details(input.action_hash.clone(), GetOptions::network())?
        .ok_or_else(|| FabricationError::not_found("FpmRegistrationAnchor", &input.action_hash))?;

    let Details::Record(record_details) = details else {
        return Err(fpm_anchor_error("ActionHash did not resolve to record details"));
    };

    if record_details.validation_status != ValidationStatus::Valid {
        return Err(fpm_anchor_error("registration anchor record is not valid"));
    }
    if !record_details.updates.is_empty() {
        return Err(fpm_anchor_error("registration anchor record has updates"));
    }
    if !record_details.deletes.is_empty() {
        return Err(fpm_anchor_error("registration anchor record has deletes"));
    }

    let record = record_details.record;
    if record.action().action_type() != ActionType::Create {
        return Err(fpm_anchor_error(
            "ActionHash anchor must resolve to the original Create action",
        ));
    }
    let anchor: FpmRegistrationAnchor = record
        .entry()
        .to_app_option()
        .map_err(|e| fpm_anchor_error(format!("could not decode registration anchor: {e}")))?
        .ok_or_else(|| fpm_anchor_error("record is not an FPM registration anchor entry"))?;

    validate_resolved_anchor_envelope(&anchor, &input.claimed_envelope_digest)?;

    let author = *record.action().author();
    if let Some(expected_author) = input.expected_author.as_ref() {
        if *expected_author != author {
            return Err(fpm_anchor_error("ActionHash author does not match expected author"));
        }
    }

    let entry_hash = record
        .action()
        .entry_hash()
        .ok_or_else(|| fpm_anchor_error("registration anchor action has no entry hash"))?;

    Ok(ResolvedFpmRegistrationAnchor {
        anchor_kind: RegistrationAnchorKind::HolochainAction,
        anchor_reference: format!("holochain-action:{input_action}", input_action = input.action_hash),
        registration_envelope_digest: anchor.envelope_digest,
        entry_hash,
        action_hash: Some(input.action_hash),
        author: Some(author),
        timestamp: Some(record.action().timestamp()),
        envelope: anchor.envelope,
    })
}

fn resolve_entry_anchor(
    input: ResolveFpmRegistrationEntryAnchorInput,
) -> ExternResult<ResolvedFpmRegistrationAnchor> {
    let details = get_details(input.entry_hash.clone(), GetOptions::network())?
        .ok_or_else(|| FabricationError::not_found("FpmRegistrationAnchor", &input.entry_hash))?;

    let Details::Entry(entry_details) = details else {
        return Err(fpm_anchor_error("EntryHash did not resolve to entry details"));
    };

    if entry_details.entry_dht_status != EntryDhtStatus::Live {
        return Err(fpm_anchor_error("registration anchor entry is not live"));
    }
    if !entry_details.rejected_actions.is_empty() {
        return Err(fpm_anchor_error("registration anchor entry has rejected creation actions"));
    }
    if !entry_details.deletes.is_empty() {
        return Err(fpm_anchor_error("registration anchor entry has deletes"));
    }
    if !entry_details.updates.is_empty() {
        return Err(fpm_anchor_error("registration anchor entry has updates"));
    }

    let record = get(input.entry_hash.clone(), GetOptions::network())?
        .ok_or_else(|| FabricationError::not_found("FpmRegistrationAnchor", &input.entry_hash))?;

    let actual_entry_hash = record
        .action()
        .entry_hash()
        .ok_or_else(|| fpm_anchor_error("resolved entry record has no entry hash"))?;
    if actual_entry_hash != input.entry_hash {
        return Err(fpm_anchor_error("resolved record entry hash does not match requested EntryHash"));
    }

    let anchor: FpmRegistrationAnchor = record
        .entry()
        .to_app_option()
        .map_err(|e| fpm_anchor_error(format!("could not decode registration anchor: {e}")))?
        .ok_or_else(|| fpm_anchor_error("entry is not an FPM registration anchor entry"))?;

    validate_resolved_anchor_envelope(&anchor, &input.claimed_envelope_digest)?;

    Ok(ResolvedFpmRegistrationAnchor {
        anchor_kind: RegistrationAnchorKind::HolochainEntry,
        anchor_reference: format!("holochain-entry:{input_entry}", input_entry = input.entry_hash),
        registration_envelope_digest: anchor.envelope_digest,
        entry_hash: input.entry_hash,
        action_hash: None,
        author: None,
        timestamp: None,
        envelope: anchor.envelope,
    })
}

#[hdk_extern]
pub fn resolve_fpm_registration_action_anchor(
    input: ResolveFpmRegistrationActionAnchorInput,
) -> ExternResult<ResolvedFpmRegistrationAnchor> {
    rate_limit_caller()?;
    resolve_action_anchor(input)
}

#[hdk_extern]
pub fn resolve_fpm_registration_entry_anchor(
    input: ResolveFpmRegistrationEntryAnchorInput,
) -> ExternResult<ResolvedFpmRegistrationAnchor> {
    rate_limit_caller()?;
    resolve_entry_anchor(input)
}

// =============================================================================
// RATE LIMITING
// =============================================================================

fn rate_limit_anchor(agent: &AgentPubKey) -> ExternResult<EntryHash> {
    let anchor_bytes = SerializedBytes::from(UnsafeBytes::from(
        format!("rate_limit:{}", agent).into_bytes(),
    ));
    hash_entry(Entry::App(AppEntryBytes(anchor_bytes)))
}

fn enforce_rate_limit(caller: &AgentPubKey) -> ExternResult<()> {
    let cfg = get_config();
    let max_ops = cfg.rate_limit_max_ops as usize;
    let window_micros = cfg.rate_limit_window_secs as i64 * 1_000_000;

    let anchor = rate_limit_anchor(caller)?;
    let links = get_links(
        LinkQuery::try_new(anchor.clone(), LinkTypes::RateLimitBucket)?,
        GetStrategy::default(),
    )?;

    let now = sys_time()?;
    let window_start = now.as_micros() - window_micros;

    let recent_count = links
        .iter()
        .filter(|l| l.timestamp.as_micros() >= window_start)
        .count();

    if recent_count >= max_ops {
        return Err(FabricationError::RateLimited {
            max_ops: cfg.rate_limit_max_ops,
            window_secs: cfg.rate_limit_window_secs,
        }.to_wasm_error());
    }

    create_link(anchor.clone(), anchor, LinkTypes::RateLimitBucket, ())?;
    Ok(())
}

fn rate_limit_caller() -> ExternResult<()> {
    let agent = agent_info()?.agent_initial_pubkey;
    enforce_rfn epistemic_cache_key(claim_text: &str, claim_type_key: &str) -> [u8; 32] {
    let mut hasher = Sha256::new();
    hasher.update(EPISTEMIC_CACHE_DOMAIN);
    hasher.update((claim_type_key.len() as u64).to_le_bytes());
    hasher.update(claim_type_key.as_bytes());
    hasher.update((claim_text.len() as u64).to_le_bytes());
    hasher.update(claim_text.as_bytes());
    hasher.finalize().into()
}

/// Validate an epistemic classification returned by Knowledge.
///
/// Only finite scores in the closed interval [0, 1] are accepted. This is a
/// semantic boundary in addition to wire decoding: valid serialization alone
/// does not make an out-of-range classification trustworthy.
fn validate_epistemic_response(ep: &ClaimEpistemic) -> ExternResult<()> {
    for (field, value) in [
        ("empirical", ep.empirical),
        ("normative", ep.normative),
        ("mythic", ep.mythic),
    ] {
        if !value.is_finite() || !(0.0..=1.0).contains(&value) {
            return Err(FabricationError::ValidationFailed {
                field: format!("knowledge.{}", field),
                reason: "classification score must be finite and in [0, 1]".to_string(),
            }
            .to_wasm_error());
        }
    }
    Ok(())
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum EpistemicFetchFailure {
    Unavailable,
    Malformed,
}

/// Fetch epistemic classification from Knowledge with an exact-claim cache.
///
/// Missing or malformed classification becomes an explicit unclassified state rather
/// than a fabricated score. Only validated classifications are cached.
fn fetch_epistemic(
    claim_text: &str,
    claim_type_key: &str,
) -> Result<ClaimEpistemic, EpistemicFetchFailure> {
    let now = match sys_time() {
        Ok(time) => time.as_micros(),
        Err(_) => return Err(EpistemicFetchFailure::Unavailable),
    };
    let key = epistemic_cache_key(claim_text, claim_type_key);

    let cached = EPISTEMIC_CACHE.with(|c| {
        c.borrow().get(&key).and_then(|(ts, ep)| {
            now.checked_sub(*ts)
                .filter(|age| *age >= 0 && *age < EPISTEMIC_CACHE_TTL_MICROS)
                .map(|_| ep.clone())
        })
    });
    if let Some(ep) = cached {
        return Ok(ep);
    }

    let response = match call(
        CallTargetCell::OtherRole("mycelix-knowledge".into()),
        ZomeName::from("epistemic"),
        FunctionName::from("classify_claim"),
        None,
        claim_text,
    ) {
        Ok(response) => response,
        Err(_) => return Err(EpistemicFetchFailure::Unavailable),
    };

    let ep = match response {
        ZomeCallResponse::Ok(bytes) => bytes
            .decode::<ClaimEpistemic>()
            .map_err(|_| EpistemicFetchFailure::Malformed)?,
        _ => return Err(EpistemicFetchFailure::Unavailable),
    };

    validate_epistemic_response(&ep)
        .map_err(|_| EpistemicFetchFailure::Malformed)?;

    EPISTEMIC_CACHE.with(|c| {
        let mut cache = c.borrow_mut();

        cache.retain(|_, (ts, _)| {
            now.checked_sub(*ts)
                .filter(|age| *age >= 0 && *age < EPISTEMIC_CACHE_TTL_MICROS)
                .is_some()
        });

        if !cache.contains_key(&key) && cache.len() >= EPISTEMIC_CACHE_MAX_ENTRIES {
            if let Some(oldest_key) = cache
                .iter()
                .min_by_key(|(_, (ts, _))| *ts)
                .map(|(key, _)| *key)
            {
                cache.remove(&oldest_key);
            }
        }

        cache.insert(key, (now, ep.clone()));
    });

    Ok(ep)
}

#[derive(Serialize, Deserialize, Debug)]
pub struct SubmitVerificationInput {
    pub design_hash: ActionHash,
    pub verification_type: VerificationType,
    pub result: VerificationResult,
    pub evidence: Vec<ActionHash>,
    pub credentials: Vec<String>,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct SubmitClaimInput {
    pub design_hash: ActionHash,
    pub claim_type: SafetyClaimType,
    pub claim_text: String,
    pub supporting_evidence: Vec<String>,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct VerificationSummary {
    pub design_hash: ActionHash,
    pub total_verifications: u32,
    pub passed: u32,
    pub failed: u32,
    pub claims_count: u32,
    pub average_confidence: f32,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct EpistemicScore {
    pub empirical: f32,
    pub normative: f32,
    pub mythic: f32,
    pub overall_confidence: f32,
    /// Explicitly distinguishes complete, partial, absent, and incomplete evidence.
    pub evidence_status: EpistemicAggregateStatus,
    /// Number of safety claims with validated Knowledge classifications.
    pub classified_claims: u32,
    /// Number of records linked from the design and considered by the aggregate.
    pub total_claims: u32,
    /// Number of linked records that could not be interpreted as SafetyClaim records.
    pub uninterpretable_records: u32,
}

#[hdk_extern]
pub fn submit_verification(input: SubmitVerificationInput) -> ExternResult<Record> {
    rate_limit_caller()?;
    let verifier = agent_info()?.agent_initial_pubkey;
    let now = sys_time()?;

    let verification = DesignVerification {
        design_hash: input.design_hash.clone(),
        verification_type: input.verification_type,
        result: input.result,
        evidence: input.evidence,
        verifier: verifier.clone(),
        verifier_credentials: input.credentials,
        created_at: Timestamp::from_micros(now.as_micros() as i64),
    };

    let hash = create_entry(EntryTypes::DesignVerification(verification))?;

    let _ = emit_signal(&TypedFabricationSignal {
        domain: FabricationDomain::Verification,
        event_type: FabricationEventType::VerificationSubmitted,
        payload: format!(r#"{{"hash":"{}"}}"#, hash),
    });

    create_link(input.design_hash, hash.clone(), LinkTypes::DesignToVerifications, ())?;
    create_link(verifier, hash.clone(), LinkTypes::VerifierToVerifications, ())?;

    get(hash.clone(), GetOptions::default())?.ok_or(FabricationError::not_found("Verification", &hash))
}

/// Internal: get all verifications for a design (used by summary/score functions)
fn get_design_verifications_all(design_hash: ActionHash) -> ExternResult<Vec<Record>> {
    let links = get_links(LinkQuery::try_new(design_hash, LinkTypes::DesignToVerifications)?, GetStrategy::default())?;
    let mut results = Vec::new();
    for link in links {
        if let Some(hash) = link.target.into_action_hash() {
            if let Some(record) = get(hash, GetOptions::default())? {
                results.push(record);
            }
        }
    }
    Ok(results)
}

#[hdk_extern]
pub fn get_design_verifications(input: HashPaginationInput) -> ExternResult<PaginatedResponse<Record>> {
    let items = get_design_verifications_all(input.hash)?;
    Ok(paginate(items, input.pagination.as_ref()))
}

#[hdk_extern]
pub fn get_verification_summary(design_hash: ActionHash) -> ExternResult<VerificationSummary> {
    let verifications = get_design_verifications_all(design_hash.clone())?;
    let claims = get_design_claims_all(design_hash.clone())?;

    let mut passed = 0u32;
    let mut failed = 0u32;
    let mut confidence_sum = 0.0f32;

    for record in &verifications {
        if let Some(v) = record.entry().to_app_option::<DesignVerification>().ok().flatten() {
            match v.result {
                VerificationResult::Passed { confidence, .. } => {
                    passed += 1;
                    confidence_sum += confidence;
                }
                VerificationResult::Failed { .. } => failed += 1,
                VerificationResult::ConditionalPass { confidence, .. } => {
                    passed += 1;
                    confidence_sum += confidence * 0.8;
                }
                _ => {}
            }
        }
    }

    let total = verifications.len() as u32;
    let avg_confidence = if passed > 0 { confidence_sum / passed as f32 } else { 0.0 };

    Ok(VerificationSummary {
        design_hash,
        total_verifications: total,
        passed,
        failed,
        claims_count: claims.len() as u32,
        average_confidence: avg_confidence,
    })
}

#[hdk_extern]
pub fn submit_safety_claim(input: SubmitClaimInput) -> ExternResult<Record> {
    rate_limit_caller()?;
    let author = agent_info()?.agent_initial_pubkey;
    let now = sys_time()?;

    // Preserve the safety claim itself, but never promote missing or malformed
    // Knowledge enrichment into a positive epistemic score.
    let claim_type_key = format!("{:?}", input.claim_type);
    let (epistemic, epistemic_provenance) =
        match fetch_epistemic(&input.claim_text, &claim_type_key) {
            Ok(ep) => (Some(ep), EpistemicProvenance::KnowledgeClassified),
            Err(EpistemicFetchFailure::Unavailable) => {
                (None, EpistemicProvenance::KnowledgeUnavailable)
            }
            Err(EpistemicFetchFailure::Malformed) => {
                (None, EpistemicProvenance::KnowledgeMalformed)
            }
        };

    let claim = SafetyClaim {
        design_hash: input.design_hash.clone(),
        claim_type: input.claim_type,
        claim_text: input.claim_text,
        epistemic,
        epistemic_provenance,
        supporting_evidence: input.supporting_evidence,
        knowledge_claim_hash: None,
        author,
        created_at: Timestamp::from_micros(now.as_micros() as i64),
    };

    let hash = create_entry(EntryTypes::SafetyClaim(claim))?;

    let _ = emit_signal(&TypedFabricationSignal {
        domain: FabricationDomain::Verification,
        event_type: FabricationEventType::ClaimSubmitted,
        payload: format!(r#"{{"hash":"{}"}}"#, hash),
    });

    create_link(input.design_hash, hash.clone(), LinkTypes::DesignToClaims, ())?;

    get(hash.clone(), GetOptions::default())?.ok_or(FabricationError::not_found("SafetyClaim", &hash))
}

/// Internal: get all claims for a design (used by summary/score functions)
fn get_design_claims_all(design_hash: ActionHash) -> ExternResult<Vec<Record>> {
    let links = get_links(LinkQuery::try_new(design_hash, LinkTypes::DesignToClaims)?, GetStrategy::default())?;
    let mut results = Vec::new();
    for link in links {
        if let Some(hash) = link.target.into_action_hash() {
            if let Some(record) = get(hash, GetOptions::default())? {
                results.push(record);
            }
        }
    }
    Ok(results)
}

#[hdk_extern]
pub fn get_design_claims(input: HashPaginationInput) -> ExternResult<PaginatedResponse<Record>> {
    let items = get_design_claims_all(input.hash)?;
    Ok(paginate(items, input.pagination.as_ref()))
}

fn epistemic_aggregate_status(
    classified_claims: u32,
    total_claims: u32,
    uninterpretable_records: u32,
) -> EpistemicAggregateStatus {
    if uninterpretable_records > 0 {
        return EpistemicAggregateStatus::IncompleteEvidence;
    }

    match (classified_claims, total_claims) {
        (0, _) => EpistemicAggregateStatus::NoClassifiedEvidence,
        (classified, total) if classified == total => EpistemicAggregateStatus::Classified,
        _ => EpistemicAggregateStatus::PartialClassifiedEvidence,
    }
}

#[hdk_extern]
pub fn get_epistemic_score(design_hash: ActionHash) -> ExternResult<EpistemicScore> {
    let claims = get_design_claims_all(design_hash)?;

    let mut e_sum = 0.0f32;
    let mut n_sum = 0.0f32;
    let mut m_sum = 0.0f32;
    let mut classified_count = 0u32;
    let total_claims = claims.len() as u32;
    let mut uninterpretable_records = 0u32;

    for record in claims {
        let claim = match record.entry().to_app_option::<SafetyClaim>() {
            Ok(Some(claim)) => claim,
            Ok(None) | Err(_) => {
                uninterpretable_records += 1;
                continue;
            }
        };

        if claim.epistemic_provenance != EpistemicProvenance::KnowledgeClassified {
            continue;
        }
        let Some(epistemic) = claim.epistemic else {
            continue;
        };
        e_sum += epistemic.empirical;
        n_sum += epistemic.normative;
        m_sum += epistemic.mythic;
        classified_count += 1;
    }

    let count_f = classified_count.max(1) as f32;
    let evidence_status =
        epistemic_aggregate_status(classified_count, total_claims, uninterpretable_records);

    Ok(EpistemicScore {
        empirical: e_sum / count_f,
        normative: n_sum / count_f,
        mythic: m_sum / count_f,
        overall_confidence: (e_sum + n_sum) / (2.0 * count_f),
        evidence_status,
        classified_claims: classified_count,
        total_claims,
        uninterpretable_records,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    // ── Helpers ──────────────────────────────────────────────────────────────

    fn test_action_hash() -> ActionHash {
        ActionHash::from_raw_36(vec![0u8; 36])
    }

    // ── 1. SubmitVerificationInput serde roundtrip ────────────────────────────

    #[test]
    fn test_submit_verification_input_serde() {
        let input = SubmitVerificationInput {
            design_hash: test_action_hash(),
            verification_type: VerificationType::StructuralAnalysis,
            result: VerificationResult::Passed {
                confidence: 0.95,
                notes: "All finite-element checks passed".to_string(),
            },
            evidence: vec![ActionHash::from_raw_36(vec![1u8; 36])],
            credentials: vec!["PE License #12345".to_string()],
        };

        let json = serde_json::to_string(&input)
            .expect("SubmitVerificationInput should serialize to JSON");
        let restored: SubmitVerificationInput = serde_json::from_str(&json)
            .expect("SubmitVerificationInput should deserialize from JSON");

        // Verify structural fields survive the roundtrip.
        assert_eq!(restored.design_hash, input.design_hash);
        assert_eq!(restored.verification_type, input.verification_type);
        assert_eq!(restored.credentials, input.credentials);
        assert_eq!(restored.evidence.len(), 1);

        // Verify the result variant and its inner values.
        match restored.result {
            VerificationResult::Passed { confidence, ref notes } => {
                assert!((confidence - 0.95).abs() < f32::EPSILON);
                assert_eq!(notes, "All finite-element checks passed");
            }
            other => panic!("Expected Passed variant, got {:?}", other),
        }
    }

    // ── 2. SubmitClaimInput serde roundtrip ───────────────────────────────────

    #[test]
    fn test_submit_safety_claim_input_serde() {
        let input = SubmitClaimInput {
            design_hash: test_action_hash(),
            claim_type: SafetyClaimType::LoadCapacity("Supports 50kg static load".to_string()),
            claim_text: "Bracket rated for 50 kg at SWL 3:1 safety factor".to_string(),
            supporting_evidence: vec!["FEA report v2.1".to_string(), "Physical test #7".to_string()],
        };

        let json = serde_json::to_string(&input)
            .expect("SubmitClaimInput should serialize to JSON");
        let restored: SubmitClaimInput = serde_json::from_str(&json)
            .expect("SubmitClaimInput should deserialize from JSON");

        assert_eq!(restored.design_hash, input.design_hash);
        assert_eq!(restored.claim_text, input.claim_text);
        assert_eq!(restored.supporting_evidence, input.supporting_evidence);
        assert_eq!(
            restored.claim_type,
            SafetyClaimType::LoadCapacity("Supports 50kg static load".to_string())
        );
    }

    // ── 3. VerificationType — all variants roundtrip ──────────────────────────

    #[test]
    fn test_verification_type_all_variants_serde() {
        let variants = vec![
            VerificationType::StructuralAnalysis,
            VerificationType::MaterialCompatibility,
            VerificationType::PrintabilityTest,
            VerificationType::SafetyReview,
            VerificationType::FoodSafeCertification,
            VerificationType::MedicalCertification,
            VerificationType::CommunityReview,
        ];

        for variant in &variants {
            let json = serde_json::to_string(variant)
                .unwrap_or_else(|e| panic!("Failed to serialize {:?}: {}", variant, e));
            let restored: VerificationType = serde_json::from_str(&json)
                .unwrap_or_else(|e| panic!("Failed to deserialize {:?}: {}", json, e));
            assert_eq!(
                &restored, variant,
                "VerificationType::{:?} did not survive serde roundtrip",
                variant
            );
        }
    }

    // ── 4. SafetyClaimType — all variants roundtrip ───────────────────────────

    #[test]
    fn test_safety_claim_type_variants_serde() {
        let variants = vec![
            SafetyClaimType::LoadCapacity("Supports 80kg".to_string()),
            SafetyClaimType::MaterialSafety("Food-safe in PETG".to_string()),
            SafetyClaimType::DimensionalAccuracy("Fits M8 bolt".to_string()),
            SafetyClaimType::TemperatureRange("Safe to 80°C".to_string()),
            SafetyClaimType::ChemicalResistance("Resistant to IPA".to_string()),
            SafetyClaimType::Custom("Outdoor UV rating 10yr".to_string()),
        ];

        for variant in &variants {
            let json = serde_json::to_string(variant)
                .unwrap_or_else(|e| panic!("Failed to serialize {:?}: {}", variant, e));
            let restored: SafetyClaimType = serde_json::from_str(&json)
                .unwrap_or_else(|e| panic!("Failed to deserialize {:?}: {}", json, e));
            assert_eq!(
                &restored, variant,
                "SafetyClaimType::{:?} did not survive serde roundtrip",
                variant
            );
        }
    }

    // ── 5. Knowledge response validation ─────────────────────────────────────

    #[test]
    fn test_knowledge_epistemic_response_validation_accepts_bounds() {
        for ep in [
            ClaimEpistemic { empirical: 0.0, normative: 0.5, mythic: 1.0 },
            ClaimEpistemic { empirical: 0.5, normative: 0.5, mythic: 0.5 },
        ] {
            assert!(validate_epistemic_response(&ep).is_ok());
        }
    }

    #[test]
    fn test_knowledge_epistemic_response_validation_rejects_nonfinite_or_out_of_range() {
        for ep in [
            ClaimEpistemic { empirical: f32::NAN, normative: 0.5, mythic: 0.5 },
            ClaimEpistemic { empirical: f32::INFINITY, normative: 0.5, mythic: 0.5 },
            ClaimEpistemic { empirical: 0.5, normative: -0.01, mythic: 0.5 },
            ClaimEpistemic { empirical: 0.5, normative: 0.5, mythic: 1.01 },
        ] {
            assert!(validate_epistemic_response(&ep).is_err());
        }
    }

    #[test]
    fn test_epistemic_cache_key_includes_claim_text_and_type() {
        let a = epistemic_cache_key("same claim", "LoadCapacity");
        let b = epistemic_cache_key("different claim", "LoadCapacity");
        let c = epistemic_cache_key("same claim", "MaterialSafety");

        assert_ne!(a, b, "distinct claim text must not share the cache key");
        assert_ne!(a, c, "distinct claim types must not share the cache key");
        assert_eq!(
            a,
            epistemic_cache_key("same claim", "LoadCapacity"),
            "cache key must be deterministic"
        );
    }

    #[test]
    fn test_epistemic_cache_ttl_constant() {
        assert_eq!(EPISTEMIC_CACHE_TTL_MICROS, 300_000_000);
        assert_eq!(EPISTEMIC_CACHE_TTL_MICROS / 1_000_000, 300);
        assert_eq!(EPISTEMIC_CACHE_MAX_ENTRIES, 128);
    }

    #[test]
    fn test_epistemic_aggregate_status_serde_roundtrip() {
        for status in [
            EpistemicAggregateStatus::Classified,
            EpistemicAggregateStatus::PartialClassifiedEvidence,
            EpistemicAggregateStatus::NoClassifiedEvidence,
            EpistemicAggregateStatus::IncompleteEvidence,
        ] {
            let encoded = serde_json::to_string(&status).unwrap();
            let decoded: EpistemicAggregateStatus = serde_json::from_str(&encoded).unwrap();
            assert_eq!(decoded, status);
        }
    }

    #[test]
    fn test_epistemic_aggregate_status_classification_matrix() {
        assert_eq!(
            epistemic_aggregate_status(0, 0, 0),
            EpistemicAggregateStatus::NoClassifiedEvidence
        );
        assert_eq!(
            epistemic_aggregate_status(0, 3, 0),
            EpistemicAggregateStatus::NoClassifiedEvidence
        );
        assert_eq!(
            epistemic_aggregate_status(1, 1, 0),
            EpistemicAggregateStatus::Classified
        );
        assert_eq!(
            epistemic_aggregate_status(2, 3, 0),
            EpistemicAggregateStatus::PartialClassifiedEvidence
        );
        assert_eq!(
            epistemic_aggregate_status(3, 3, 0),
            EpistemicAggregateStatus::Classified
        );
        assert_eq!(
            epistemic_aggregate_status(1, 1, 1),
            EpistemicAggregateStatus::IncompleteEvidence
        );
        assert_eq!(
            epistemic_aggregate_status(0, 1, 1),
            EpistemicAggregateStatus::IncompleteEvidence
        );
    }


    #[test]
    fn fpm_anchor_envelope_binding_accepts_exact_digest() {
        let reference = ModalityObservationRef {
            source_id: "thermal-1".into(),
            modality: "thermal".into(),
            clock_domain: "ptp-domain-1".into(),
            source_sequence: 10,
            correlation_domain: "frame-domain".into(),
            correlation_id: "frame-10".into(),
            source_timestamp_micros: Some(1_000_000),
            calibration_profile_digest: "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa".into(),
            process_context_digest: "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb".into(),
            source_data_digest: "cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc".into(),
        };
        let related = ModalityObservationRef {
            source_id: "vibration-1".into(),
            modality: "vibration".into(),
            ..reference.clone()
        };
        let envelope = RegistrationEnvelope {
            schema_version: FPM_REGISTRATION_SCHEMA_VERSION.into(),
            reference,
            related: vec![related],
            alignment_method: Some(AlignmentMethod::ExactCorrelationId),
        };
        let digest = envelope.digest().expect("envelope digest");
        let anchor = FpmRegistrationAnchor {
            schema_version: FPM_REGISTRATION_ANCHOR_SCHEMA_VERSION.into(),
            envelope,
            envelope_digest: digest.clone(),
        };

        assert!(validate_resolved_anchor_envelope(&anchor, &digest).is_ok());
    }

    #[test]
    fn fpm_anchor_envelope_substitution_is_rejected() {
        let mut envelope = RegistrationEnvelope {
            schema_version: FPM_REGISTRATION_SCHEMA_VERSION.into(),
            reference: ModalityObservationRef {
                source_id: "thermal-1".into(),
                modality: "thermal".into(),
                clock_domain: "ptp-domain-1".into(),
                source_sequence: 10,
                correlation_domain: "frame-domain".into(),
                correlation_id: "frame-10".into(),
                source_timestamp_micros: Some(1_000_000),
                calibration_profile_digest: "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa".into(),
                process_context_digest: "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb".into(),
                source_data_digest: "cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc".into(),
            },
            related: vec![],
            alignment_method: None,
        };
        let digest = envelope.digest().expect("envelope digest");
        envelope.correlation_id = "replay".into();
        let anchor = FpmRegistrationAnchor {
            schema_version: FPM_REGISTRATION_ANCHOR_SCHEMA_VERSION.into(),
            envelope,
            envelope_digest: digest.clone(),
        };

        assert!(validate_resolved_anchor_envelope(&anchor, &digest).is_err());
    }

    #[test]
    fn fpm_anchor_claimed_digest_mismatch_is_rejected() {
        let envelope = RegistrationEnvelope {
            schema_version: FPM_REGISTRATION_SCHEMA_VERSION.into(),
            reference: ModalityObservationRef {
                source_id: "thermal-1".into(),
                modality: "thermal".into(),
                clock_domain: "ptp-domain-1".into(),
                source_sequence: 10,
                correlation_domain: "frame-domain".into(),
                correlation_id: "frame-10".into(),
                source_timestamp_micros: Some(1_000_000),
                calibration_profile_digest: "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa".into(),
                process_context_digest: "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb".into(),
                source_data_digest: "cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc".into(),
            },
            related: vec![],
            alignment_method: None,
        };
        let digest = envelope.digest().expect("envelope digest");
        let anchor = FpmRegistrationAnchor {
            schema_version: FPM_REGISTRATION_ANCHOR_SCHEMA_VERSION.into(),
            envelope,
            envelope_digest: digest,
        };
        let wrong = "dddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd";

        assert!(validate_resolved_anchor_envelope(&anchor, wrong).is_err());
    }

    #[test]
    fn fpm_anchor_uppercase_digest_is_rejected() {
        let envelope = RegistrationEnvelope {
            schema_version: FPM_REGISTRATION_SCHEMA_VERSION.into(),
            reference: ModalityObservationRef {
                source_id: "thermal-1".into(),
                modality: "thermal".into(),
                clock_domain: "ptp-domain-1".into(),
                source_sequence: 10,
                correlation_domain: "frame-domain".into(),
                correlation_id: "frame-10".into(),
                source_timestamp_micros: Some(1_000_000),
                calibration_profile_digest: "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa".into(),
                process_context_digest: "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb".into(),
                source_data_digest: "cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc".into(),
            },
            related: vec![],
            alignment_method: None,
        };
        let digest = envelope.digest().expect("envelope digest");
        let anchor = FpmRegistrationAnchor {
            schema_version: FPM_REGISTRATION_ANCHOR_SCHEMA_VERSION.into(),
            envelope,
            envelope_digest: digest.clone(),
        };

        assert!(validate_resolved_anchor_envelope(&anchor, &digest.to_uppercase()).is_err());
    }

}
