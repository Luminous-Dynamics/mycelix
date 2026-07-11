// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Inference Integrity Zome
//! Defines entry types and validation for ML-powered knowledge inference
//!
//! Updated to use HDI 0.7 patterns

use hdi::prelude::*;

/// Anchor entry for deterministic link bases
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Anchor(pub String);

/// An inferred relationship or pattern
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Inference {
    /// Inference identifier
    pub id: String,
    /// Type of inference
    pub inference_type: InferenceType,
    /// Source claims used for inference
    pub source_claims: Vec<String>,
    /// Inferred conclusion
    pub conclusion: String,
    /// Confidence in the inference (0.0 to 1.0)
    pub confidence: f64,
    /// Reasoning/explanation
    pub reasoning: String,
    /// Model or algorithm used
    pub model: String,
    /// When the inference was made
    pub created: Timestamp,
    /// Whether this has been verified by humans
    pub verified: bool,
}

/// Types of inferences
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum InferenceType {
    /// Implicit relationship between claims
    ImpliedRelation,
    /// Contradiction detected
    Contradiction,
    /// Pattern across multiple claims
    Pattern,
    /// Prediction based on trends
    Prediction,
    /// Synthesis of multiple claims
    Synthesis,
    /// Similarity between claims
    Similarity,
    /// Anomaly detection
    Anomaly,
    /// Credibility assessment
    CredibilityAssessment,
}

/// Credibility assessment for a claim or source
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct CredibilityScore {
    /// Score identifier
    pub id: String,
    /// Subject (claim ID or source URI)
    pub subject: String,
    /// Subject type
    pub subject_type: CredibilitySubjectType,
    /// Overall credibility score (0.0 to 1.0)
    pub score: f64,
    /// Component scores
    pub components: CredibilityComponents,
    /// Factors contributing to the score
    pub factors: Vec<CredibilityFactor>,
    /// When assessed
    pub assessed_at: Timestamp,
    /// Assessment expiration
    pub expires_at: Option<Timestamp>,
}

/// Types of subjects for credibility assessment
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum CredibilitySubjectType {
    Claim,
    Source,
    Author,
}

/// Component scores for credibility
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct CredibilityComponents {
    /// Accuracy based on verification
    pub accuracy: f64,
    /// Consistency with other claims
    pub consistency: f64,
    /// Transparency of sources
    pub transparency: f64,
    /// Track record of author/source
    pub track_record: f64,
    /// Corroboration by others
    pub corroboration: f64,
}

/// A factor contributing to credibility
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct CredibilityFactor {
    /// Factor name
    pub name: String,
    /// Factor value (positive or negative)
    pub value: f64,
    /// Explanation
    pub explanation: String,
}

/// Pattern detected in the knowledge graph
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Pattern {
    /// Pattern identifier
    pub id: String,
    /// Pattern type
    pub pattern_type: PatternType,
    /// Description of the pattern
    pub description: String,
    /// Claims involved in the pattern
    pub claims: Vec<String>,
    /// Pattern strength (0.0 to 1.0)
    pub strength: f64,
    /// Detection timestamp
    pub detected_at: Timestamp,
}

/// Types of patterns
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum PatternType {
    /// Temporal pattern (trends over time)
    Temporal,
    /// Cluster of related claims
    Cluster,
    /// Causal chain
    Causal,
    /// Correlation pattern
    Correlation,
    /// Contradiction cluster
    ContradictionCluster,
    /// Echo chamber
    EchoChamber,
}

// ============================================================================
// MATL-ENHANCED CREDIBILITY TYPES
// ============================================================================

/// Enhanced credibility assessment with MATL integration
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct EnhancedCredibilityScore {
    /// Score identifier
    pub id: String,

    /// Subject (claim ID, source URI, or author DID)
    pub subject: String,

    /// Subject type
    pub subject_type: CredibilitySubjectType,

    /// Overall credibility score (0.0 to 1.0)
    pub overall_score: f64,

    /// Basic component scores
    pub components: CredibilityComponents,

    /// MATL integration scores
    pub matl: MatlCredibilityComponents,

    /// Evidence strength analysis
    pub evidence_strength: EvidenceStrengthComponents,

    /// Author reputation (if subject is a claim)
    pub author_reputation: Option<AuthorReputation>,

    /// Factors contributing to the score
    pub factors: Vec<CredibilityFactor>,

    /// When assessed
    pub assessed_at: Timestamp,

    /// Assessment expiration
    pub expires_at: Option<Timestamp>,

    /// Model/version used for assessment
    pub assessment_model: String,

    /// Confidence in this assessment
    pub assessment_confidence: f64,
}

/// MATL (Multi-dimensional Adaptive Trust Layer) credibility components
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct MatlCredibilityComponents {
    /// Composite MATL score (0.0 to 1.0)
    pub matl_composite: f64,

    /// Quality dimension from MATL
    pub matl_quality: f64,

    /// Consistency dimension from MATL
    pub matl_consistency: f64,

    /// Reputation dimension from MATL
    pub matl_reputation: f64,

    /// Stake-weighted dimension (for market integration)
    pub matl_stake_weighted: f64,

    /// Oracle agreement from verification markets
    pub oracle_agreement: Option<f64>,

    /// Number of oracle assessments
    pub oracle_count: u32,
}

impl Default for MatlCredibilityComponents {
    fn default() -> Self {
        Self {
            matl_composite: 0.5,
            matl_quality: 0.5,
            matl_consistency: 0.5,
            matl_reputation: 0.5,
            matl_stake_weighted: 0.5,
            oracle_agreement: None,
            oracle_count: 0,
        }
    }
}

/// Evidence strength analysis
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct EvidenceStrengthComponents {
    /// Number of empirical evidence pieces
    pub empirical_evidence_count: u32,

    /// Number of testimonial evidence pieces
    pub testimonial_evidence_count: u32,

    /// Number of cryptographic proofs
    pub crypto_evidence_count: u32,

    /// Number of cross-references to other claims
    pub cross_reference_count: u32,

    /// Number of verification market resolutions
    pub market_resolution_count: u32,

    /// Average strength of empirical evidence
    pub empirical_average_strength: f64,

    /// Average strength of testimonial evidence
    pub testimonial_average_strength: f64,

    /// Total evidence pieces
    pub total_evidence_count: u32,

    /// Evidence diversity score (0.0-1.0)
    pub evidence_diversity: f64,
}

impl Default for EvidenceStrengthComponents {
    fn default() -> Self {
        Self {
            empirical_evidence_count: 0,
            testimonial_evidence_count: 0,
            crypto_evidence_count: 0,
            cross_reference_count: 0,
            market_resolution_count: 0,
            empirical_average_strength: 0.5,
            testimonial_average_strength: 0.5,
            total_evidence_count: 0,
            evidence_diversity: 0.0,
        }
    }
}

/// Author reputation profile
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct AuthorReputation {
    /// Unique identifier
    pub id: String,

    /// Author's DID
    pub author_did: String,

    /// Overall reputation score (0.0-1.0)
    pub overall_score: f64,

    /// Domain-specific reputation scores
    pub domain_scores: Vec<DomainReputation>,

    /// Historical accuracy rate
    pub historical_accuracy: f64,

    /// Number of claims authored
    pub claims_authored: u32,

    /// Number of claims verified true
    pub claims_verified_true: u32,

    /// Number of claims verified false
    pub claims_verified_false: u32,

    /// Number of claims pending verification
    pub claims_pending: u32,

    /// Average epistemic level of claims
    pub average_epistemic_e: f64,

    /// MATL trust score from markets
    pub matl_trust: f64,

    /// Last updated
    pub updated_at: Timestamp,

    /// Reputation age (how long tracked)
    pub reputation_age_days: u32,
}

/// Reputation within a specific domain
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct DomainReputation {
    /// Domain name (e.g., "climate", "finance", "technology")
    pub domain: String,

    /// Score within this domain (0.0-1.0)
    pub score: f64,

    /// Number of claims in this domain
    pub claim_count: u32,

    /// Accuracy rate in this domain
    pub accuracy_rate: f64,
}

/// Batch credibility result
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct BatchCredibilityResult {
    /// Individual results
    pub results: Vec<EnhancedCredibilitySummary>,

    /// Total claims assessed
    pub total_assessed: u32,

    /// Average credibility score
    pub average_score: f64,

    /// High credibility count (>0.7)
    pub high_credibility_count: u32,

    /// Low credibility count (<0.3)
    pub low_credibility_count: u32,

    /// Processing time in milliseconds
    pub processing_time_ms: u64,
}

/// Summary of enhanced credibility (for batch results)
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct EnhancedCredibilitySummary {
    /// Subject ID
    pub subject: String,

    /// Overall score
    pub overall_score: f64,

    /// MATL composite
    pub matl_composite: f64,

    /// Evidence count
    pub evidence_count: u32,

    /// Assessment confidence
    pub assessment_confidence: f64,
}

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    Anchor(Anchor),
    Inference(Inference),
    CredibilityScore(CredibilityScore),
    Pattern(Pattern),
    EnhancedCredibilityScore(EnhancedCredibilityScore),
    AuthorReputation(AuthorReputation),
}

#[hdk_link_types]
pub enum LinkTypes {
    /// Claim to inferences about it
    ClaimToInference,
    /// Subject to credibility score
    SubjectToCredibility,
    /// Pattern type to patterns
    TypeToPattern,
    /// Subject to enhanced credibility score
    SubjectToEnhancedCredibility,
    /// Author to reputation
    AuthorToReputation,
    /// Index by credibility score range
    CredibilityScoreIndex,
    /// Index by MATL composite score
    MatlScoreIndex,
    /// Inference ID to inference record
    InferenceIdToInference,
}

/// HDI 0.7 single validation callback using FlatOp pattern
#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(store_entry) => match store_entry {
            OpEntry::CreateEntry { app_entry, action } => match app_entry {
                EntryTypes::Anchor(_) => Ok(ValidateCallbackResult::Valid),
                EntryTypes::Inference(inference) => validate_create_inference(action, inference),
                EntryTypes::CredibilityScore(score) => {
                    validate_create_credibility_score(action, score)
                }
                EntryTypes::Pattern(pattern) => validate_create_pattern(action, pattern),
                EntryTypes::EnhancedCredibilityScore(score) => {
                    validate_create_enhanced_credibility(action, score)
                }
                EntryTypes::AuthorReputation(rep) => validate_create_author_reputation(action, rep),
            },
            OpEntry::UpdateEntry {
                app_entry,
                action,
                original_action_hash: _,
                original_entry_hash: _,
            } => validate_update_entry_type(action, app_entry),
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterCreateLink {
            link_type,
            base_address: _,
            target_address: _,
            tag: _,
            action: _,
        } => match link_type {
            LinkTypes::ClaimToInference => Ok(ValidateCallbackResult::Valid),
            LinkTypes::SubjectToCredibility => Ok(ValidateCallbackResult::Valid),
            LinkTypes::TypeToPattern => Ok(ValidateCallbackResult::Valid),
            LinkTypes::SubjectToEnhancedCredibility => Ok(ValidateCallbackResult::Valid),
            LinkTypes::AuthorToReputation => Ok(ValidateCallbackResult::Valid),
            LinkTypes::CredibilityScoreIndex => Ok(ValidateCallbackResult::Valid),
            LinkTypes::MatlScoreIndex => Ok(ValidateCallbackResult::Valid),
            LinkTypes::InferenceIdToInference => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterDeleteLink {
            original_action,
            action,
            ..
        } => {
            // Only the original link creator can delete a link
            if action.author != original_action.author {
                return Ok(ValidateCallbackResult::Invalid(
                    "Only the original link creator can delete a link".into(),
                ));
            }
            Ok(ValidateCallbackResult::Valid)
        }
        FlatOp::StoreRecord(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterAgentActivity(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterUpdate(op_update) => match op_update {
            // This DHT op was previously left fully permissive (`Ok(Valid)`
            // unconditionally) -- meaning any agent could rewrite the
            // content of ANY entry in this zome via a modified coordinator,
            // regardless of what the StoreEntry-side validate_update_*
            // functions checked. Found + fixed 2026-07-09 during the P0
            // author-binding pass (same class of gap as
            // mycelix-knowledge/bridge's identical issue). Route through
            // the same per-type validators as the StoreEntry perspective.
            OpUpdate::Entry { app_entry, action } => validate_update_entry_type(action, app_entry),
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterDelete(OpDelete { action }) => {
            // Also previously fully permissive. The coordinator never
            // calls delete_entry for any entry type in this zome, but a
            // modified coordinator could otherwise delete anyone's entries
            // freely. Bind to the standard "only the original author can
            // delete their own entries" convention used elsewhere.
            let original = must_get_action(action.deletes_address.clone())?;
            if action.author != *original.action().author() {
                return Ok(ValidateCallbackResult::Invalid(
                    "Only the original entry author can delete an entry".into(),
                ));
            }
            Ok(ValidateCallbackResult::Valid)
        }
    }
}

/// Shared per-entry-type update validation, called from BOTH the StoreEntry
/// (OpEntry::UpdateEntry) and RegisterUpdate DHT-op perspectives so they
/// agree.
fn validate_update_entry_type(
    action: Update,
    app_entry: EntryTypes,
) -> ExternResult<ValidateCallbackResult> {
    match app_entry {
        EntryTypes::Anchor(_) => Ok(ValidateCallbackResult::Valid),
        EntryTypes::Inference(inference) => validate_update_inference(action, inference),
        EntryTypes::CredibilityScore(_) => Ok(ValidateCallbackResult::Invalid(
            "Credibility scores are immutable (re-assess to create a new one)".into(),
        )),
        EntryTypes::Pattern(_) => Ok(ValidateCallbackResult::Invalid(
            "Patterns are immutable (re-detect to create a new one)".into(),
        )),
        EntryTypes::EnhancedCredibilityScore(_) => Ok(ValidateCallbackResult::Invalid(
            "Enhanced credibility scores are immutable (re-assess to create a new one)".into(),
        )),
        EntryTypes::AuthorReputation(rep) => validate_update_author_reputation(action, rep),
    }
}

/// Validate inference creation
///
/// No author-binding possible: Inference has no self-declared owner/
/// creator field -- it's a computed ML-inference result (source_claims,
/// conclusion, model), not an identity claim. Reviewed 2026-07-09 during
/// the P0 author-binding pass; case (a).
fn validate_create_inference(
    _action: Create,
    inference: Inference,
) -> ExternResult<ValidateCallbackResult> {
    // Validate confidence range
    if inference.confidence < 0.0 || inference.confidence > 1.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Confidence must be between 0.0 and 1.0".into(),
        ));
    }

    // Validate at least one source claim
    if inference.source_claims.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Inference must have at least one source claim".into(),
        ));
    }

    // Validate conclusion not empty
    if inference.conclusion.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Inference conclusion cannot be empty".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Validate credibility score creation
///
/// No author-binding possible: CredibilityScore has no self-declared
/// owner/creator field -- it's a computed assessment (`subject` names the
/// claim/source being scored, a third party, not the committer). Reviewed
/// 2026-07-09 during the P0 author-binding pass; case (a)/(b).
fn validate_create_credibility_score(
    _action: Create,
    score: CredibilityScore,
) -> ExternResult<ValidateCallbackResult> {
    // Validate overall score range
    if score.score < 0.0 || score.score > 1.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Score must be between 0.0 and 1.0".into(),
        ));
    }

    // Validate component scores
    let components = &score.components;
    for (name, value) in [
        ("accuracy", components.accuracy),
        ("consistency", components.consistency),
        ("transparency", components.transparency),
        ("track_record", components.track_record),
        ("corroboration", components.corroboration),
    ] {
        if value < 0.0 || value > 1.0 {
            return Ok(ValidateCallbackResult::Invalid(format!(
                "{} must be between 0.0 and 1.0",
                name
            )));
        }
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Validate pattern creation
///
/// No author-binding possible: Pattern has no self-declared owner/creator
/// field -- it's a computed detection result across claims. Reviewed
/// 2026-07-09 during the P0 author-binding pass; case (a).
fn validate_create_pattern(
    _action: Create,
    pattern: Pattern,
) -> ExternResult<ValidateCallbackResult> {
    // Validate strength range
    if pattern.strength < 0.0 || pattern.strength > 1.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Pattern strength must be between 0.0 and 1.0".into(),
        ));
    }

    // Validate at least two claims in pattern
    if pattern.claims.len() < 2 {
        return Ok(ValidateCallbackResult::Invalid(
            "Pattern must involve at least two claims".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Validate enhanced credibility score creation
///
/// No author-binding possible: same reasoning as CredibilityScore --
/// computed assessment, `subject` names a third party. Reviewed 2026-07-09
/// during the P0 author-binding pass; case (a)/(b).
fn validate_create_enhanced_credibility(
    _action: Create,
    score: EnhancedCredibilityScore,
) -> ExternResult<ValidateCallbackResult> {
    // Validate overall score range
    if score.overall_score < 0.0 || score.overall_score > 1.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Overall score must be between 0.0 and 1.0".into(),
        ));
    }

    // Validate assessment confidence
    if score.assessment_confidence < 0.0 || score.assessment_confidence > 1.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Assessment confidence must be between 0.0 and 1.0".into(),
        ));
    }

    // Validate MATL components
    let matl = &score.matl;
    if matl.matl_composite < 0.0 || matl.matl_composite > 1.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "MATL composite must be between 0.0 and 1.0".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Validate author reputation creation
///
/// No author-binding possible: `author_did` names the THIRD PARTY whose
/// reputation this profile tracks, not the committer -- the coordinator's
/// get_author_reputation creates a fresh neutral-default profile for
/// whatever author_did is requested (a "get or create" cache pattern, not
/// a self-attestation). See the KNOWN GAP note on
/// validate_update_author_reputation below for the related content-forgery
/// risk on the *update* path, which is a different, harder problem than
/// author-binding. Reviewed 2026-07-09 during the P0 author-binding pass;
/// case (b).
fn validate_create_author_reputation(
    _action: Create,
    rep: AuthorReputation,
) -> ExternResult<ValidateCallbackResult> {
    // Validate overall score range
    if rep.overall_score < 0.0 || rep.overall_score > 1.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Overall score must be between 0.0 and 1.0".into(),
        ));
    }

    // Validate author DID
    if !rep.author_did.starts_with("did:") {
        return Ok(ValidateCallbackResult::Invalid(
            "Author must be a valid DID".into(),
        ));
    }

    // Validate historical accuracy
    if rep.historical_accuracy < 0.0 || rep.historical_accuracy > 1.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Historical accuracy must be between 0.0 and 1.0".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Validate inference update (verify_inference marks an inference
/// verified/unverified and adjusts confidence).
///
/// No author-binding: the coordinator's verify_inference is an
/// intentionally cross-agent flow (a different agent than the inference's
/// original creator can mark it verified -- that's the point of
/// independent verification) and Inference doesn't even store a
/// "verifier" identity field to bind. Instead this enforces content
/// integrity: only `confidence` and `verified` may change; everything else
/// is immutable.
fn validate_update_inference(
    action: Update,
    inference: Inference,
) -> ExternResult<ValidateCallbackResult> {
    if inference.confidence < 0.0 || inference.confidence > 1.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Confidence must be between 0.0 and 1.0".into(),
        ));
    }

    let original_record = must_get_valid_record(action.original_action_address.clone())?;
    let original: Inference = original_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Original inference not found".into()
        )))?;

    if inference.id != original.id
        || inference.inference_type != original.inference_type
        || inference.source_claims != original.source_claims
        || inference.conclusion != original.conclusion
        || inference.reasoning != original.reasoning
        || inference.model != original.model
        || inference.created != original.created
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Only confidence/verified can change on an inference update".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Validate author reputation update.
///
/// KNOWN GAP, not fixable by simple author-binding (found 2026-07-09
/// during the P0 pass): the coordinator's update_author_reputation takes
/// `author_did` and score deltas (claims_added/verified_true/
/// verified_false/matl_trust_update) entirely from caller input with zero
/// derivation from or check against agent_info() -- any agent can inflate
/// or deflate ANY other author's reputation by calling it with a forged
/// `author_did`. Unlike the self-declared-identity forgeries fixed
/// elsewhere this pass, `author_did` here legitimately names a THIRD PARTY
/// (the subject whose reputation this profile tracks), not the committer
/// -- so binding it to action.author() would be wrong, not just
/// insufficient. Same architectural class as the report_reputation
/// forgery gap documented in the mycelix-identity/bridge zome commit
/// (`ea30870f92`). Out of scope for this pass; flagging for a dedicated
/// follow-up (needs call-provenance / capability-grant infrastructure, or
/// restricting updates to the identity zome's own verified claim-
/// verification events).
///
/// What IS fixed here: content-integrity hardening. `id`/`author_did` are
/// now immutable, `updated_at` must advance, and the three claim counters
/// can only increase (matches the "counts can't decrease" convention used
/// elsewhere in this codebase for monotonic tallies).
fn validate_update_author_reputation(
    action: Update,
    rep: AuthorReputation,
) -> ExternResult<ValidateCallbackResult> {
    if rep.overall_score < 0.0 || rep.overall_score > 1.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Overall score must be between 0.0 and 1.0".into(),
        ));
    }
    if rep.historical_accuracy < 0.0 || rep.historical_accuracy > 1.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Historical accuracy must be between 0.0 and 1.0".into(),
        ));
    }

    let original_record = must_get_valid_record(action.original_action_address.clone())?;
    let original: AuthorReputation = original_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Original author reputation not found".into()
        )))?;

    if rep.id != original.id || rep.author_did != original.author_did {
        return Ok(ValidateCallbackResult::Invalid(
            "Author reputation id/author_did cannot be changed".into(),
        ));
    }
    if rep.updated_at <= original.updated_at {
        return Ok(ValidateCallbackResult::Invalid(
            "Author reputation updated_at must advance".into(),
        ));
    }
    if rep.claims_authored < original.claims_authored
        || rep.claims_verified_true < original.claims_verified_true
        || rep.claims_verified_false < original.claims_verified_false
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Author reputation claim counters cannot decrease".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

#[cfg(test)]
mod author_binding_tests {
    use super::*;

    fn update_action(author: AgentPubKey) -> Update {
        Update {
            author,
            timestamp: Timestamp::from_micros(1),
            action_seq: 1,
            prev_action: ActionHash::from_raw_36(vec![0u8; 36]),
            original_action_address: ActionHash::from_raw_36(vec![9u8; 36]),
            original_entry_address: EntryHash::from_raw_36(vec![0u8; 36]),
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

    #[test]
    fn update_entry_type_rejects_credibility_score_update() {
        let score = CredibilityScore {
            id: "score-1".into(),
            subject: "claim-1".into(),
            subject_type: CredibilitySubjectType::Claim,
            score: 0.5,
            components: CredibilityComponents {
                accuracy: 0.5,
                consistency: 0.5,
                transparency: 0.5,
                track_record: 0.5,
                corroboration: 0.5,
            },
            factors: vec![],
            assessed_at: Timestamp::from_micros(0),
            expires_at: None,
        };
        let result =
            validate_update_entry_type(update_action(me()), EntryTypes::CredibilityScore(score))
                .unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn update_entry_type_rejects_pattern_update() {
        let pattern = Pattern {
            id: "pattern-1".into(),
            pattern_type: PatternType::Cluster,
            description: "desc".into(),
            claims: vec!["c1".into(), "c2".into()],
            strength: 0.5,
            detected_at: Timestamp::from_micros(0),
        };
        let result =
            validate_update_entry_type(update_action(me()), EntryTypes::Pattern(pattern)).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn update_entry_type_rejects_enhanced_credibility_update() {
        let score = EnhancedCredibilityScore {
            id: "enh-1".into(),
            subject: "claim-1".into(),
            subject_type: CredibilitySubjectType::Claim,
            overall_score: 0.5,
            components: CredibilityComponents {
                accuracy: 0.5,
                consistency: 0.5,
                transparency: 0.5,
                track_record: 0.5,
                corroboration: 0.5,
            },
            matl: MatlCredibilityComponents::default(),
            evidence_strength: EvidenceStrengthComponents::default(),
            author_reputation: None,
            factors: vec![],
            assessed_at: Timestamp::from_micros(0),
            expires_at: None,
            assessment_model: "v1".into(),
            assessment_confidence: 0.5,
        };
        let result = validate_update_entry_type(
            update_action(me()),
            EntryTypes::EnhancedCredibilityScore(score),
        )
        .unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }
}
