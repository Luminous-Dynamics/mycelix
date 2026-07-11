// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Invention Integrity Zome
//!
//! Defines entry types and validation for the decentralized prior art registry.
//! Inventions are timestamped on the DHT with Blake3 witness commitments,
//! enabling provable priority without a central authority.

use hdi::prelude::*;
use serde::{Deserialize, Serialize};

// ── Entry Types ────────────────────────────────────────────────────────

/// Anchor entry for deterministic link bases
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Anchor(pub String);

// ── Enums ──────────────────────────────────────────────────────────────

/// Type of supporting evidence
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum EvidenceType {
    SourceCode,
    Paper,
    Prototype,
    Documentation,
    TestResults,
    ExternalPublication,
    Other,
}

/// License type for invention terms
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum LicenseType {
    AGPL3OrLater,
    Apache2,
    MIT,
    CreativeCommons(CCVariant),
    Proprietary,
    DualLicense {
        open: Box<LicenseType>,
        commercial: Box<LicenseType>,
    },
    Custom,
}

/// Creative Commons variants
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum CCVariant {
    BY,
    BYSA,
    BYNC,
    BYNCSA,
    BYND,
    BYNCND,
}

/// Lifecycle status of an invention claim
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum InventionStatus {
    /// Not yet published — visible only to the inventor
    Draft,
    /// Visible on DHT, open for challenge
    Published,
    /// Under dispute via an InventionChallenge
    Challenged,
    /// Passed challenge period without successful dispute
    Verified,
    /// Replaced by a newer invention
    Superseded,
    /// Withdrawn by inventor
    Revoked,
}

impl InventionStatus {
    /// Returns the set of statuses this status can legally transition to.
    pub fn valid_transitions(&self) -> &[InventionStatus] {
        match self {
            InventionStatus::Draft => &[InventionStatus::Published, InventionStatus::Revoked],
            InventionStatus::Published => &[
                InventionStatus::Challenged,
                InventionStatus::Verified,
                InventionStatus::Superseded,
                InventionStatus::Revoked,
            ],
            InventionStatus::Challenged => &[
                InventionStatus::Published,
                InventionStatus::Verified,
                InventionStatus::Revoked,
            ],
            InventionStatus::Verified => &[InventionStatus::Superseded, InventionStatus::Revoked],
            InventionStatus::Superseded => &[],
            InventionStatus::Revoked => &[],
        }
    }

    /// Check whether transitioning from self to `target` is allowed.
    pub fn can_transition_to(&self, target: &InventionStatus) -> bool {
        self.valid_transitions().contains(target)
    }
}

/// Reason for challenging an invention claim
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum ChallengeReason {
    PriorArt { existing_ref: String },
    InvalidClaim,
    InsufficientEvidence,
    LicenseViolation,
    Other { description: String },
}

/// Status of a challenge
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum ChallengeStatus {
    Filed,
    UnderReview,
    Upheld,
    Dismissed,
}

/// What event triggers a royalty obligation
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum RoyaltyTrigger {
    /// When someone registers a derivative invention referencing this one
    PerDerivativeWork,
    /// When a commercial license is activated
    PerCommercialUse,
    /// When the invention is deployed to production
    PerProductionDeploy,
    /// Recurring payment (monthly/quarterly)
    Subscription,
    /// Single payment on first use
    OneTime,
}

/// Lifecycle status of a royalty ledger entry
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum RoyaltyStatus {
    /// Owed but not yet paid
    Pending,
    /// Settlement complete
    Paid,
    /// Under arbitration
    Disputed,
    /// Forgiven by receiver
    Waived,
    /// Past collection window
    Expired,
}

// ── Supporting Structs ─────────────────────────────────────────────────

/// Blake3 hash of a piece of evidence
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct EvidenceHash {
    /// Blake3 hash (32 bytes)
    pub hash: Vec<u8>,
    /// Human-readable description of the evidence
    pub description: String,
    /// Category of evidence
    pub evidence_type: EvidenceType,
}

/// License terms governing use of the invention
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct LicenseTerms {
    pub license_type: LicenseType,
    pub attribution_required: bool,
    pub derivative_allowed: bool,
    pub commercial_allowed: bool,
    /// Royalty percentage (0.0-100.0), if applicable
    pub royalty_percentage: Option<f64>,
    /// DID of entity that receives royalties
    pub royalty_receiver_did: Option<String>,
    /// Free-text custom terms
    pub custom_terms: Option<String>,
}

/// Simplified epistemic classification (E/N/M) for inventions.
/// Uses u8 levels for compact storage; full EpistemicClassification
/// lives in claims_integrity.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct EpistemicClassification {
    /// Empirical level (0-4)
    pub empirical: u8,
    /// Normative level (0-3)
    pub normative: u8,
    /// Materiality level (0-3)
    pub materiality: u8,
}

// ── Main Entry Types ───────────────────────────────────────────────────

/// An invention claim in the decentralized prior art registry.
///
/// The `witness_commitment` field is a Blake3 hash of
/// `title || description || inventor_did || timestamp` and serves as a
/// compact, tamper-evident commitment anchored at creation time.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct InventionClaim {
    /// Unique identifier (e.g., UUID)
    pub id: String,
    /// Human-readable title (1-256 chars)
    pub title: String,
    /// Technical description (1-10000 chars)
    pub description: String,
    /// Primary inventor DID (must start with "did:")
    pub inventor_did: String,
    /// Co-inventor DIDs
    pub co_inventors: Vec<String>,
    /// ActionHash hex strings of claims this builds on
    pub prior_art_refs: Vec<String>,
    /// Blake3 hashes of code, papers, prototypes, etc.
    pub evidence_hashes: Vec<EvidenceHash>,
    /// Blake3(title || description || inventor_did || timestamp) — 32 bytes
    pub witness_commitment: Vec<u8>,
    /// License governing use of the invention
    pub license_terms: LicenseTerms,
    /// Domain / field (e.g., "cryptography", "machine-learning")
    pub domain: String,
    /// E/N/M epistemic classification
    pub classification: EpistemicClassification,
    /// Lifecycle status
    pub status: InventionStatus,
    /// Creation timestamp
    pub created_at: Timestamp,
    /// Last-updated timestamp
    pub updated_at: Timestamp,
}

/// A challenge filed against an InventionClaim.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct InventionChallenge {
    /// ActionHash of the challenged InventionClaim
    pub invention_hash: ActionHash,
    /// DID of the challenger
    pub challenger_did: String,
    /// Reason for the challenge
    pub reason: ChallengeReason,
    /// Supporting evidence hashes
    pub evidence: Vec<EvidenceHash>,
    /// Current challenge status
    pub status: ChallengeStatus,
    /// When the challenge was filed
    pub created_at: Timestamp,
}

/// A royalty rule attached to an invention, defining how TEND hours
/// are owed when certain events occur.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct InventionRoyaltyRule {
    /// Unique identifier (e.g., "royalty:{invention_id}:{trigger}:{timestamp}")
    pub id: String,
    /// References InventionClaim.id
    pub invention_id: String,
    /// What event triggers this royalty
    pub rule_type: RoyaltyTrigger,
    /// Percentage of value owed (0.0-100.0)
    pub percentage: f64,
    /// Minimum TEND hours per trigger event
    pub minimum_amount_tend: Option<f64>,
    /// DID of the royalty receiver (inventor or treasury)
    pub receiver_did: String,
    /// Whether this rule is currently active
    pub active: bool,
    /// Creation timestamp
    pub created_at: i64,
}

/// A record of a royalty payment due or completed.
/// Created automatically when a trigger event occurs (e.g., derivative work)
/// or manually via `record_royalty_event`.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct RoyaltyLedgerEntry {
    /// Unique identifier
    pub id: String,
    /// References InventionRoyaltyRule.id
    pub royalty_rule_id: String,
    /// References InventionClaim.id
    pub invention_id: String,
    /// DID of the party who owes the royalty
    pub payer_did: String,
    /// DID of the party who receives the royalty
    pub receiver_did: String,
    /// TEND hours owed or paid
    pub amount_tend: f64,
    /// Human-readable description of the triggering event
    pub trigger_event: String,
    /// ActionHash (hex) of the action that triggered this entry
    pub trigger_action_hash: Option<String>,
    /// Current status
    pub status: RoyaltyStatus,
    /// When this entry was created
    pub created_at: i64,
    /// When payment was completed (if Paid)
    pub paid_at: Option<i64>,
}

// ── Entry & Link Enums ─────────────────────────────────────────────────

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    Anchor(Anchor),
    InventionClaim(InventionClaim),
    InventionChallenge(InventionChallenge),
    InventionRoyaltyRule(InventionRoyaltyRule),
    RoyaltyLedgerEntry(RoyaltyLedgerEntry),
}

#[hdk_link_types]
pub enum LinkTypes {
    /// InventionClaim → InventionChallenge
    InventionToChallenge,
    /// Anchor("invention:{id}") → InventionClaim
    IdToInvention,
    /// Anchor("inventor:{did}") → InventionClaim
    InventorToInvention,
    /// Anchor("domain:{domain}") → InventionClaim
    DomainToInvention,
    /// Anchor("all_inventions") → InventionClaim
    AllInventions,
    /// Rate-limit tracking
    InventionRateLimit,
    /// InventionClaim → InventionRoyaltyRule
    InventionToRoyaltyRule,
    /// InventionRoyaltyRule → RoyaltyLedgerEntry
    RoyaltyRuleToLedger,
    /// Anchor("royalty_owed:{payer_did}") → RoyaltyLedgerEntry
    AgentToRoyaltyOwed,
    /// Anchor("royalty_receivable:{receiver_did}") → RoyaltyLedgerEntry
    AgentToRoyaltyReceivable,
}

// ── Pure Validation Functions ──────────────────────────────────────────

pub fn validate_create_invention_claim(
    action: Create,
    claim: InventionClaim,
) -> ExternResult<ValidateCallbackResult> {
    // Author-binding: the coordinator's register_invention now derives
    // inventor_did from agent_info() rather than trusting caller input,
    // but that's bypassable by a modified coordinator -- the integrity
    // validator is the real security boundary. Without this, any agent
    // could register an invention naming an arbitrary victim (or a
    // fabricated) DID as inventor, which would defeat the entire
    // "provable priority without a central authority" purpose of the
    // witness commitment. Found + fixed 2026-07-09 during the P0
    // author-binding pass.
    let expected_inventor_did = format!("did:mycelix:{}", action.author);
    if claim.inventor_did != expected_inventor_did {
        return Ok(ValidateCallbackResult::Invalid(
            "inventor_did must correspond to the committing agent".into(),
        ));
    }

    // Title length
    if claim.title.is_empty() || claim.title.len() > 256 {
        return Ok(ValidateCallbackResult::Invalid(
            "title must be 1-256 characters".into(),
        ));
    }
    // Description length
    if claim.description.is_empty() || claim.description.len() > 10_000 {
        return Ok(ValidateCallbackResult::Invalid(
            "description must be 1-10000 characters".into(),
        ));
    }
    // Inventor DID
    if !claim.inventor_did.starts_with("did:") {
        return Ok(ValidateCallbackResult::Invalid(
            "inventor_did must start with 'did:'".into(),
        ));
    }
    // Co-inventor DIDs
    for (i, co) in claim.co_inventors.iter().enumerate() {
        if !co.starts_with("did:") {
            return Ok(ValidateCallbackResult::Invalid(format!(
                "co_inventors[{}] must start with 'did:'",
                i
            )));
        }
    }
    // Witness commitment
    if claim.witness_commitment.len() != 32 {
        return Ok(ValidateCallbackResult::Invalid(
            "witness_commitment must be exactly 32 bytes (Blake3)".into(),
        ));
    }
    // Evidence hashes
    for (i, eh) in claim.evidence_hashes.iter().enumerate() {
        if eh.hash.len() != 32 {
            return Ok(ValidateCallbackResult::Invalid(format!(
                "evidence_hashes[{}].hash must be exactly 32 bytes (Blake3)",
                i
            )));
        }
        if eh.description.is_empty() {
            return Ok(ValidateCallbackResult::Invalid(format!(
                "evidence_hashes[{}].description must not be empty",
                i
            )));
        }
    }
    // Royalty percentage
    if let Some(pct) = claim.license_terms.royalty_percentage {
        if !(0.0..=100.0).contains(&pct) {
            return Ok(ValidateCallbackResult::Invalid(
                "royalty_percentage must be 0.0-100.0".into(),
            ));
        }
    }
    // Royalty receiver DID
    if let Some(ref did) = claim.license_terms.royalty_receiver_did {
        if !did.starts_with("did:") {
            return Ok(ValidateCallbackResult::Invalid(
                "royalty_receiver_did must start with 'did:'".into(),
            ));
        }
    }
    // Epistemic classification bounds
    if claim.classification.empirical > 4 {
        return Ok(ValidateCallbackResult::Invalid(
            "empirical must be 0-4".into(),
        ));
    }
    if claim.classification.normative > 3 {
        return Ok(ValidateCallbackResult::Invalid(
            "normative must be 0-3".into(),
        ));
    }
    if claim.classification.materiality > 3 {
        return Ok(ValidateCallbackResult::Invalid(
            "materiality must be 0-3".into(),
        ));
    }
    // ID must not be empty
    if claim.id.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "id must not be empty".into(),
        ));
    }
    // Domain must not be empty
    if claim.domain.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "domain must not be empty".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Validate an InventionClaim update.
///
/// Two distinct, legitimate update flows share this entry type:
/// - The SAME agent as the original inventor, via the coordinator's
///   update_invention, which may change any field (title/description/
///   co_inventors/prior_art_refs/evidence_hashes/license_terms/domain/
///   classification/status).
/// - A DIFFERENT agent (a challenger, via challenge_invention, or anyone,
///   via verify_invention), which may ONLY advance `status` -- everything
///   else must stay identical to the original.
///
/// Previously this function only re-ran the create-time field checks and
/// never fetched the original entry at all -- meaning status-transition
/// validity and "who's allowed to change what" were enforced ONLY in the
/// coordinator (a comment here said as much), which a modified coordinator
/// could simply skip. Fixed 2026-07-09 during the P0 author-binding pass:
/// this now does the must_get_valid_record fetch and enforces both rules
/// at the integrity layer, the real security boundary.
pub fn validate_update_invention_claim(
    action: Update,
    claim: InventionClaim,
) -> ExternResult<ValidateCallbackResult> {
    // Same field validations as create
    if claim.title.is_empty() || claim.title.len() > 256 {
        return Ok(ValidateCallbackResult::Invalid(
            "title must be 1-256 characters".into(),
        ));
    }
    if claim.description.is_empty() || claim.description.len() > 10_000 {
        return Ok(ValidateCallbackResult::Invalid(
            "description must be 1-10000 characters".into(),
        ));
    }
    if !claim.inventor_did.starts_with("did:") {
        return Ok(ValidateCallbackResult::Invalid(
            "inventor_did must start with 'did:'".into(),
        ));
    }
    if claim.witness_commitment.len() != 32 {
        return Ok(ValidateCallbackResult::Invalid(
            "witness_commitment must be exactly 32 bytes (Blake3)".into(),
        ));
    }
    for (i, eh) in claim.evidence_hashes.iter().enumerate() {
        if eh.hash.len() != 32 {
            return Ok(ValidateCallbackResult::Invalid(format!(
                "evidence_hashes[{}].hash must be exactly 32 bytes (Blake3)",
                i
            )));
        }
    }
    if let Some(pct) = claim.license_terms.royalty_percentage {
        if !(0.0..=100.0).contains(&pct) {
            return Ok(ValidateCallbackResult::Invalid(
                "royalty_percentage must be 0.0-100.0".into(),
            ));
        }
    }
    if claim.classification.empirical > 4 {
        return Ok(ValidateCallbackResult::Invalid(
            "empirical must be 0-4".into(),
        ));
    }
    if claim.classification.normative > 3 {
        return Ok(ValidateCallbackResult::Invalid(
            "normative must be 0-3".into(),
        ));
    }
    if claim.classification.materiality > 3 {
        return Ok(ValidateCallbackResult::Invalid(
            "materiality must be 0-3".into(),
        ));
    }

    let original_record = must_get_valid_record(action.original_action_address.clone())?;
    let original: InventionClaim = original_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Original invention claim not found".into()
        )))?;

    if !original.status.can_transition_to(&claim.status) && original.status != claim.status {
        return Ok(ValidateCallbackResult::Invalid(format!(
            "Invalid invention status transition from {:?} to {:?}",
            original.status, claim.status
        )));
    }

    let is_original_author = action.author == *original_record.action().author();
    if !is_original_author {
        // A different agent (challenger/verifier) may ONLY change status.
        if claim.title != original.title
            || claim.description != original.description
            || claim.inventor_did != original.inventor_did
            || claim.co_inventors != original.co_inventors
            || claim.prior_art_refs != original.prior_art_refs
            || claim.evidence_hashes != original.evidence_hashes
            || claim.witness_commitment != original.witness_commitment
            || claim.license_terms != original.license_terms
            || claim.domain != original.domain
            || claim.classification != original.classification
            || claim.created_at != original.created_at
        {
            return Ok(ValidateCallbackResult::Invalid(
                "Only the original inventor can change invention fields other than status".into(),
            ));
        }
    }

    Ok(ValidateCallbackResult::Valid)
}

pub fn validate_create_invention_challenge(
    action: Create,
    challenge: InventionChallenge,
) -> ExternResult<ValidateCallbackResult> {
    if !challenge.challenger_did.starts_with("did:") {
        return Ok(ValidateCallbackResult::Invalid(
            "challenger_did must start with 'did:'".into(),
        ));
    }

    // Author-binding: the coordinator's challenge_invention now derives
    // challenger_did from agent_info() rather than trusting caller input,
    // so this is belt-and-suspenders against a modified coordinator
    // forging a challenge as an arbitrary victim challenger. Found + fixed
    // 2026-07-09 during the P0 author-binding pass.
    let expected_challenger_did = format!("did:mycelix:{}", action.author);
    if challenge.challenger_did != expected_challenger_did {
        return Ok(ValidateCallbackResult::Invalid(
            "challenger_did must correspond to the committing agent".into(),
        ));
    }
    for (i, eh) in challenge.evidence.iter().enumerate() {
        if eh.hash.len() != 32 {
            return Ok(ValidateCallbackResult::Invalid(format!(
                "evidence[{}].hash must be exactly 32 bytes (Blake3)",
                i
            )));
        }
    }
    if let ChallengeReason::Other { ref description } = challenge.reason {
        if description.is_empty() {
            return Ok(ValidateCallbackResult::Invalid(
                "ChallengeReason::Other description must not be empty".into(),
            ));
        }
    }
    Ok(ValidateCallbackResult::Valid)
}

/// Validate an InventionChallenge update.
///
/// The coordinator's resolve_challenge is deliberately open to ANY agent
/// (per its own doc comment: "Currently any agent can resolve (in
/// production, this would be restricted to arbitrators...)") -- so this
/// does NOT require author == original author. Instead it enforces
/// content integrity: only `status` may change (Filed/UnderReview ->
/// Upheld/Dismissed), everything else is immutable. Hardened 2026-07-09
/// during the P0 author-binding pass -- previously this function never
/// fetched the original at all, so a modified coordinator could rewrite
/// invention_hash/challenger_did/reason/evidence/created_at freely on
/// "resolution".
pub fn validate_update_invention_challenge(
    action: Update,
    challenge: InventionChallenge,
) -> ExternResult<ValidateCallbackResult> {
    // Only status changes are allowed; field validation still applies
    if !challenge.challenger_did.starts_with("did:") {
        return Ok(ValidateCallbackResult::Invalid(
            "challenger_did must start with 'did:'".into(),
        ));
    }
    for (i, eh) in challenge.evidence.iter().enumerate() {
        if eh.hash.len() != 32 {
            return Ok(ValidateCallbackResult::Invalid(format!(
                "evidence[{}].hash must be exactly 32 bytes (Blake3)",
                i
            )));
        }
    }

    let original_record = must_get_valid_record(action.original_action_address.clone())?;
    let original: InventionChallenge = original_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Original invention challenge not found".into()
        )))?;

    if challenge.invention_hash != original.invention_hash
        || challenge.challenger_did != original.challenger_did
        || challenge.reason != original.reason
        || challenge.evidence != original.evidence
        || challenge.created_at != original.created_at
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Only status can change when resolving a challenge".into(),
        ));
    }

    let valid_transition = matches!(
        (&original.status, &challenge.status),
        (ChallengeStatus::Filed, ChallengeStatus::UnderReview)
            | (ChallengeStatus::Filed, ChallengeStatus::Upheld)
            | (ChallengeStatus::Filed, ChallengeStatus::Dismissed)
            | (ChallengeStatus::UnderReview, ChallengeStatus::Upheld)
            | (ChallengeStatus::UnderReview, ChallengeStatus::Dismissed)
    ) || original.status == challenge.status;
    if !valid_transition {
        return Ok(ValidateCallbackResult::Invalid(format!(
            "Invalid challenge status transition from {:?} to {:?}",
            original.status, challenge.status
        )));
    }

    Ok(ValidateCallbackResult::Valid)
}

/// No author-binding on `receiver_did`: it names the payee destination
/// (which the coordinator's create_royalty_rule -- already gated to the
/// invention's own author -- may legitimately set to a third party, e.g. a
/// DAO treasury, not necessarily the caller themselves). Reviewed
/// 2026-07-09 during the P0 author-binding pass; case (b).
pub fn validate_create_royalty_rule(
    _action: Create,
    rule: InventionRoyaltyRule,
) -> ExternResult<ValidateCallbackResult> {
    if rule.id.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "royalty rule id must not be empty".into(),
        ));
    }
    if rule.invention_id.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "invention_id must not be empty".into(),
        ));
    }
    if !(0.0..=100.0).contains(&rule.percentage) {
        return Ok(ValidateCallbackResult::Invalid(
            "percentage must be 0.0-100.0".into(),
        ));
    }
    if let Some(min) = rule.minimum_amount_tend {
        if min < 0.0 {
            return Ok(ValidateCallbackResult::Invalid(
                "minimum_amount_tend must be non-negative".into(),
            ));
        }
    }
    if !rule.receiver_did.starts_with("did:") {
        return Ok(ValidateCallbackResult::Invalid(
            "receiver_did must start with 'did:'".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

/// InventionRoyaltyRule is immutable: confirmed via grep that no
/// coordinator function ever calls update_entry for this type (rules are
/// created once via create_royalty_rule and never modified). Made
/// explicit 2026-07-09 during the P0 author-binding pass -- previously
/// this function re-ran the create-time field checks but never fetched
/// the original, so a modified coordinator could rewrite any field
/// (including receiver_did, redirecting royalties) freely.
pub fn validate_update_royalty_rule(
    _action: Update,
    _rule: InventionRoyaltyRule,
) -> ExternResult<ValidateCallbackResult> {
    Ok(ValidateCallbackResult::Invalid(
        "Royalty rules are immutable".into(),
    ))
}

/// KNOWN GAP, not fixable by simple author-binding (found 2026-07-09
/// during the P0 pass): the coordinator's record_royalty_event takes
/// `payer_did` entirely from caller input with zero derivation from or
/// check against agent_info() -- any agent can create a
/// RoyaltyLedgerEntry claiming an arbitrary victim owes a royalty. Unlike
/// the self-declared-identity forgeries fixed elsewhere this pass, it's
/// genuinely ambiguous whether `payer_did` should be the committer (a
/// self-report: "I used this, I owe you") or a third party (the receiver
/// recording someone else's observed usage) -- the doc comment doesn't
/// disambiguate and there's no existing access control to infer intent
/// from. Binding it to action.author() could break a legitimate
/// receiver-reports-third-party-usage flow if that's the intended design.
/// Same architectural class as the report_reputation forgery gap
/// documented in the mycelix-identity/bridge zome commit (`ea30870f92`).
/// Out of scope for this pass; flagging for a dedicated follow-up once the
/// intended access model is clarified.
pub fn validate_create_ledger_entry(
    _action: Create,
    entry: RoyaltyLedgerEntry,
) -> ExternResult<ValidateCallbackResult> {
    if entry.id.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "ledger entry id must not be empty".into(),
        ));
    }
    if entry.royalty_rule_id.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "royalty_rule_id must not be empty".into(),
        ));
    }
    if entry.invention_id.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "invention_id must not be empty".into(),
        ));
    }
    if !entry.payer_did.starts_with("did:") {
        return Ok(ValidateCallbackResult::Invalid(
            "payer_did must start with 'did:'".into(),
        ));
    }
    if !entry.receiver_did.starts_with("did:") {
        return Ok(ValidateCallbackResult::Invalid(
            "receiver_did must start with 'did:'".into(),
        ));
    }
    if entry.amount_tend < 0.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "amount_tend must be non-negative".into(),
        ));
    }
    if entry.trigger_event.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "trigger_event must not be empty".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

/// Validate a RoyaltyLedgerEntry update.
///
/// The coordinator's mark_royalty_paid/waive_royalty are deliberately open
/// to ANY agent (per their own doc comments -- DID<->agent mapping is
/// external, so identity is left to the application layer). This does NOT
/// require author == original author. Instead it enforces content
/// integrity: only `status`/`paid_at` may change (Pending -> Paid or
/// Pending -> Waived), everything else is immutable. Hardened 2026-07-09
/// during the P0 author-binding pass -- previously this function never
/// fetched the original, so a modified coordinator could rewrite
/// payer_did/receiver_did/amount_tend freely under the guise of "marking
/// paid".
pub fn validate_update_ledger_entry(
    action: Update,
    entry: RoyaltyLedgerEntry,
) -> ExternResult<ValidateCallbackResult> {
    if entry.id.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "ledger entry id must not be empty".into(),
        ));
    }
    if !entry.payer_did.starts_with("did:") {
        return Ok(ValidateCallbackResult::Invalid(
            "payer_did must start with 'did:'".into(),
        ));
    }
    if !entry.receiver_did.starts_with("did:") {
        return Ok(ValidateCallbackResult::Invalid(
            "receiver_did must start with 'did:'".into(),
        ));
    }
    if entry.amount_tend < 0.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "amount_tend must be non-negative".into(),
        ));
    }

    let original_record = must_get_valid_record(action.original_action_address.clone())?;
    let original: RoyaltyLedgerEntry = original_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Original ledger entry not found".into()
        )))?;

    if entry.id != original.id
        || entry.royalty_rule_id != original.royalty_rule_id
        || entry.invention_id != original.invention_id
        || entry.payer_did != original.payer_did
        || entry.receiver_did != original.receiver_did
        || entry.amount_tend != original.amount_tend
        || entry.trigger_event != original.trigger_event
        || entry.trigger_action_hash != original.trigger_action_hash
        || entry.created_at != original.created_at
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Only status/paid_at can change on a ledger entry update".into(),
        ));
    }

    let valid_transition = matches!(
        (&original.status, &entry.status),
        (RoyaltyStatus::Pending, RoyaltyStatus::Paid)
            | (RoyaltyStatus::Pending, RoyaltyStatus::Waived)
    ) || original.status == entry.status;
    if !valid_transition {
        return Ok(ValidateCallbackResult::Invalid(format!(
            "Invalid royalty status transition from {:?} to {:?}",
            original.status, entry.status
        )));
    }

    Ok(ValidateCallbackResult::Valid)
}

// ── HDI Validation Callback ────────────────────────────────────────────

#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(store_entry) => match store_entry {
            OpEntry::CreateEntry { app_entry, action } => match app_entry {
                EntryTypes::Anchor(_) => Ok(ValidateCallbackResult::Valid),
                EntryTypes::InventionClaim(claim) => validate_create_invention_claim(action, claim),
                EntryTypes::InventionChallenge(challenge) => {
                    validate_create_invention_challenge(action, challenge)
                }
                EntryTypes::InventionRoyaltyRule(rule) => {
                    validate_create_royalty_rule(action, rule)
                }
                EntryTypes::RoyaltyLedgerEntry(entry) => {
                    validate_create_ledger_entry(action, entry)
                }
            },
            OpEntry::UpdateEntry {
                app_entry,
                action,
                original_action_hash: _,
                original_entry_hash: _,
            } => match app_entry {
                EntryTypes::Anchor(_) => Ok(ValidateCallbackResult::Valid),
                EntryTypes::InventionClaim(claim) => validate_update_invention_claim(action, claim),
                EntryTypes::InventionChallenge(challenge) => {
                    validate_update_invention_challenge(action, challenge)
                }
                EntryTypes::InventionRoyaltyRule(rule) => {
                    validate_update_royalty_rule(action, rule)
                }
                EntryTypes::RoyaltyLedgerEntry(entry) => {
                    validate_update_ledger_entry(action, entry)
                }
            },
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterCreateLink {
            link_type,
            base_address: _,
            target_address: _,
            tag: _,
            action: _,
        } => match link_type {
            LinkTypes::InventionToChallenge
            | LinkTypes::IdToInvention
            | LinkTypes::InventorToInvention
            | LinkTypes::DomainToInvention
            | LinkTypes::AllInventions
            | LinkTypes::InventionRateLimit
            | LinkTypes::InventionToRoyaltyRule
            | LinkTypes::RoyaltyRuleToLedger
            | LinkTypes::AgentToRoyaltyOwed
            | LinkTypes::AgentToRoyaltyReceivable => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterDeleteLink {
            link_type,
            original_action,
            base_address: _,
            target_address: _,
            tag: _,
            action,
        } => {
            // Previously accepted unconditionally regardless of author --
            // the coordinator never calls delete_link here, so this is
            // pure hardening (zero functional impact). Bind to the
            // standard "only the original link creator can delete it"
            // convention used elsewhere. Found + fixed 2026-07-09 during
            // the P0 author-binding pass.
            if action.author != original_action.author {
                return Ok(ValidateCallbackResult::Invalid(
                    "Only the original link creator can delete a link".into(),
                ));
            }
            match link_type {
                LinkTypes::InventionToChallenge
                | LinkTypes::IdToInvention
                | LinkTypes::InventorToInvention
                | LinkTypes::DomainToInvention
                | LinkTypes::AllInventions
                | LinkTypes::InventionRateLimit
                | LinkTypes::InventionToRoyaltyRule
                | LinkTypes::RoyaltyRuleToLedger
                | LinkTypes::AgentToRoyaltyOwed
                | LinkTypes::AgentToRoyaltyReceivable => Ok(ValidateCallbackResult::Valid),
            }
        }
        FlatOp::StoreRecord(_) | FlatOp::RegisterAgentActivity(_) => {
            Ok(ValidateCallbackResult::Valid)
        }
        FlatOp::RegisterUpdate(op_update) => match op_update {
            // This DHT op was previously left fully permissive (`Ok(Valid)`
            // unconditionally) -- meaning any agent could rewrite the
            // content of ANY entry in this zome via a modified coordinator,
            // regardless of what the StoreEntry-side validate_update_*
            // functions checked (which themselves, before this pass, never
            // even fetched the original entry -- see the per-function doc
            // comments above). Found + fixed 2026-07-09 during the P0
            // author-binding pass. Route through the same per-type
            // validators as the StoreEntry perspective so both DHT op
            // views agree.
            OpUpdate::Entry { app_entry, action } => match app_entry {
                EntryTypes::Anchor(_) => Ok(ValidateCallbackResult::Valid),
                EntryTypes::InventionClaim(claim) => validate_update_invention_claim(action, claim),
                EntryTypes::InventionChallenge(challenge) => {
                    validate_update_invention_challenge(action, challenge)
                }
                EntryTypes::InventionRoyaltyRule(rule) => {
                    validate_update_royalty_rule(action, rule)
                }
                EntryTypes::RoyaltyLedgerEntry(entry) => {
                    validate_update_ledger_entry(action, entry)
                }
            },
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterDelete(OpDelete { action }) => {
            // Also previously fully permissive. No coordinator function in
            // this zome ever calls delete_entry, so this is pure hardening
            // against a modified coordinator, zero functional impact.
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

// ── Unit Tests ─────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn test_author() -> AgentPubKey {
        AgentPubKey::from_raw_36(vec![0u8; 36])
    }

    fn test_author_did() -> String {
        format!("did:mycelix:{}", test_author())
    }

    fn test_action() -> Create {
        Create {
            author: test_author(),
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

    fn test_update_action() -> Update {
        Update {
            author: test_author(),
            timestamp: Timestamp::from_micros(0),
            action_seq: 1,
            prev_action: ActionHash::from_raw_36(vec![0u8; 36]),
            original_action_address: ActionHash::from_raw_36(vec![0u8; 36]),
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

    fn valid_license() -> LicenseTerms {
        LicenseTerms {
            license_type: LicenseType::AGPL3OrLater,
            attribution_required: true,
            derivative_allowed: true,
            commercial_allowed: false,
            royalty_percentage: None,
            royalty_receiver_did: None,
            custom_terms: None,
        }
    }

    fn valid_claim() -> InventionClaim {
        InventionClaim {
            id: "inv-001".into(),
            title: "Hyperdimensional Liquid Time-Constant Neuron".into(),
            description: "A unified neuron model combining HDC and LTC dynamics.".into(),
            inventor_did: test_author_did(),
            co_inventors: vec![],
            prior_art_refs: vec![],
            evidence_hashes: vec![EvidenceHash {
                hash: vec![0xAB; 32],
                description: "Source code commit abc123".into(),
                evidence_type: EvidenceType::SourceCode,
            }],
            witness_commitment: vec![0xCD; 32],
            license_terms: valid_license(),
            domain: "machine-learning".into(),
            classification: EpistemicClassification {
                empirical: 2,
                normative: 1,
                materiality: 2,
            },
            status: InventionStatus::Draft,
            created_at: Timestamp::from_micros(0),
            updated_at: Timestamp::from_micros(0),
        }
    }

    fn valid_challenge() -> InventionChallenge {
        InventionChallenge {
            invention_hash: ActionHash::from_raw_36(vec![0u8; 36]),
            challenger_did: test_author_did(),
            reason: ChallengeReason::PriorArt {
                existing_ref: "abc123def456".into(),
            },
            evidence: vec![EvidenceHash {
                hash: vec![0xEF; 32],
                description: "Published paper from 2024".into(),
                evidence_type: EvidenceType::Paper,
            }],
            status: ChallengeStatus::Filed,
            created_at: Timestamp::from_micros(0),
        }
    }

    // ── InventionClaim validation ──────────────────────────────────

    #[test]
    fn test_valid_invention_claim() {
        let result = validate_create_invention_claim(test_action(), valid_claim()).unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn test_empty_id_rejected() {
        let mut c = valid_claim();
        c.id = String::new();
        let result = validate_create_invention_claim(test_action(), c).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_empty_title_rejected() {
        let mut c = valid_claim();
        c.title = String::new();
        let result = validate_create_invention_claim(test_action(), c).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_title_too_long_rejected() {
        let mut c = valid_claim();
        c.title = "x".repeat(257);
        let result = validate_create_invention_claim(test_action(), c).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_title_exactly_256_valid() {
        let mut c = valid_claim();
        c.title = "x".repeat(256);
        let result = validate_create_invention_claim(test_action(), c).unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn test_empty_description_rejected() {
        let mut c = valid_claim();
        c.description = String::new();
        let result = validate_create_invention_claim(test_action(), c).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_description_too_long_rejected() {
        let mut c = valid_claim();
        c.description = "x".repeat(10_001);
        let result = validate_create_invention_claim(test_action(), c).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_description_exactly_10000_valid() {
        let mut c = valid_claim();
        c.description = "x".repeat(10_000);
        let result = validate_create_invention_claim(test_action(), c).unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn test_invalid_inventor_did_rejected() {
        let mut c = valid_claim();
        c.inventor_did = "not-a-did".into();
        let result = validate_create_invention_claim(test_action(), c).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_inventor_did_forgery_rejected() {
        let mut c = valid_claim();
        c.inventor_did = "did:mycelix:someone-else".into();
        let result = validate_create_invention_claim(test_action(), c).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_invalid_co_inventor_did_rejected() {
        let mut c = valid_claim();
        c.co_inventors = vec!["did:mycelix:bob".into(), "no-did".into()];
        let result = validate_create_invention_claim(test_action(), c).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_wrong_witness_commitment_size_rejected() {
        let mut c = valid_claim();
        c.witness_commitment = vec![0xCD; 16];
        let result = validate_create_invention_claim(test_action(), c).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_wrong_evidence_hash_size_rejected() {
        let mut c = valid_claim();
        c.evidence_hashes = vec![EvidenceHash {
            hash: vec![0xAB; 31],
            description: "bad hash".into(),
            evidence_type: EvidenceType::SourceCode,
        }];
        let result = validate_create_invention_claim(test_action(), c).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_empty_evidence_description_rejected() {
        let mut c = valid_claim();
        c.evidence_hashes = vec![EvidenceHash {
            hash: vec![0xAB; 32],
            description: String::new(),
            evidence_type: EvidenceType::SourceCode,
        }];
        let result = validate_create_invention_claim(test_action(), c).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_royalty_percentage_over_100_rejected() {
        let mut c = valid_claim();
        c.license_terms.royalty_percentage = Some(100.1);
        let result = validate_create_invention_claim(test_action(), c).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_royalty_percentage_negative_rejected() {
        let mut c = valid_claim();
        c.license_terms.royalty_percentage = Some(-0.1);
        let result = validate_create_invention_claim(test_action(), c).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_royalty_percentage_zero_valid() {
        let mut c = valid_claim();
        c.license_terms.royalty_percentage = Some(0.0);
        let result = validate_create_invention_claim(test_action(), c).unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn test_royalty_percentage_100_valid() {
        let mut c = valid_claim();
        c.license_terms.royalty_percentage = Some(100.0);
        let result = validate_create_invention_claim(test_action(), c).unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn test_invalid_royalty_receiver_did_rejected() {
        let mut c = valid_claim();
        c.license_terms.royalty_receiver_did = Some("not-did".into());
        let result = validate_create_invention_claim(test_action(), c).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_empirical_over_4_rejected() {
        let mut c = valid_claim();
        c.classification.empirical = 5;
        let result = validate_create_invention_claim(test_action(), c).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_normative_over_3_rejected() {
        let mut c = valid_claim();
        c.classification.normative = 4;
        let result = validate_create_invention_claim(test_action(), c).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_materiality_over_3_rejected() {
        let mut c = valid_claim();
        c.classification.materiality = 4;
        let result = validate_create_invention_claim(test_action(), c).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_empty_domain_rejected() {
        let mut c = valid_claim();
        c.domain = String::new();
        let result = validate_create_invention_claim(test_action(), c).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_no_evidence_valid() {
        let mut c = valid_claim();
        c.evidence_hashes = vec![];
        let result = validate_create_invention_claim(test_action(), c).unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn test_dual_license_valid() {
        let mut c = valid_claim();
        c.license_terms.license_type = LicenseType::DualLicense {
            open: Box::new(LicenseType::AGPL3OrLater),
            commercial: Box::new(LicenseType::Proprietary),
        };
        let result = validate_create_invention_claim(test_action(), c).unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn test_creative_commons_valid() {
        let mut c = valid_claim();
        c.license_terms.license_type = LicenseType::CreativeCommons(CCVariant::BYNCSA);
        let result = validate_create_invention_claim(test_action(), c).unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    // ── Update validation ──────────────────────────────────────────
    //
    // Note: a "fully valid update" case for validate_update_invention_claim
    // can't be exercised as a plain unit test anymore -- it now calls
    // must_get_valid_record to fetch the original entry and check the
    // author/status-transition rules, which needs a live HDI host this
    // crate's tests don't mock. Coverage for the "valid" path needs a
    // sweettest / live-conductor check instead. Field-validation rejections
    // that fail BEFORE the must_get_valid_record call (like the title check
    // below) are still directly testable.

    #[test]
    fn test_update_bad_title_rejected() {
        let mut c = valid_claim();
        c.title = String::new();
        let result = validate_update_invention_claim(test_update_action(), c).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    // ── InventionChallenge validation ──────────────────────────────

    #[test]
    fn test_valid_challenge() {
        let result = validate_create_invention_challenge(test_action(), valid_challenge()).unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn test_challenge_bad_did_rejected() {
        let mut ch = valid_challenge();
        ch.challenger_did = "not-did".into();
        let result = validate_create_invention_challenge(test_action(), ch).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_challenger_did_forgery_rejected() {
        let mut ch = valid_challenge();
        ch.challenger_did = "did:mycelix:someone-else".into();
        let result = validate_create_invention_challenge(test_action(), ch).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_challenge_bad_evidence_hash_rejected() {
        let mut ch = valid_challenge();
        ch.evidence = vec![EvidenceHash {
            hash: vec![0xEF; 16],
            description: "too short".into(),
            evidence_type: EvidenceType::Paper,
        }];
        let result = validate_create_invention_challenge(test_action(), ch).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_challenge_other_reason_empty_description_rejected() {
        let mut ch = valid_challenge();
        ch.reason = ChallengeReason::Other {
            description: String::new(),
        };
        let result = validate_create_invention_challenge(test_action(), ch).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_challenge_no_evidence_valid() {
        let mut ch = valid_challenge();
        ch.evidence = vec![];
        let result = validate_create_invention_challenge(test_action(), ch).unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    // ── Status transitions ─────────────────────────────────────────

    #[test]
    fn test_draft_can_transition_to_published() {
        assert!(InventionStatus::Draft.can_transition_to(&InventionStatus::Published));
    }

    #[test]
    fn test_draft_cannot_transition_to_verified() {
        assert!(!InventionStatus::Draft.can_transition_to(&InventionStatus::Verified));
    }

    #[test]
    fn test_published_can_transition_to_challenged() {
        assert!(InventionStatus::Published.can_transition_to(&InventionStatus::Challenged));
    }

    #[test]
    fn test_verified_cannot_transition_to_draft() {
        assert!(!InventionStatus::Verified.can_transition_to(&InventionStatus::Draft));
    }

    #[test]
    fn test_revoked_is_terminal() {
        assert!(InventionStatus::Revoked.valid_transitions().is_empty());
    }

    #[test]
    fn test_superseded_is_terminal() {
        assert!(InventionStatus::Superseded.valid_transitions().is_empty());
    }

    #[test]
    fn test_challenged_can_return_to_published() {
        assert!(InventionStatus::Challenged.can_transition_to(&InventionStatus::Published));
    }

    // ── Serde roundtrips ───────────────────────────────────────────

    #[test]
    fn test_evidence_type_serde_roundtrip() {
        let types = vec![
            EvidenceType::SourceCode,
            EvidenceType::Paper,
            EvidenceType::Prototype,
            EvidenceType::Documentation,
            EvidenceType::TestResults,
            EvidenceType::ExternalPublication,
            EvidenceType::Other,
        ];
        for et in types {
            let json = serde_json::to_string(&et).unwrap();
            let back: EvidenceType = serde_json::from_str(&json).unwrap();
            assert_eq!(et, back);
        }
    }

    #[test]
    fn test_invention_status_serde_roundtrip() {
        let statuses = vec![
            InventionStatus::Draft,
            InventionStatus::Published,
            InventionStatus::Challenged,
            InventionStatus::Verified,
            InventionStatus::Superseded,
            InventionStatus::Revoked,
        ];
        for s in statuses {
            let json = serde_json::to_string(&s).unwrap();
            let back: InventionStatus = serde_json::from_str(&json).unwrap();
            assert_eq!(s, back);
        }
    }

    #[test]
    fn test_license_type_serde_roundtrip() {
        let licenses = vec![
            LicenseType::AGPL3OrLater,
            LicenseType::Apache2,
            LicenseType::MIT,
            LicenseType::CreativeCommons(CCVariant::BY),
            LicenseType::Proprietary,
            LicenseType::Custom,
            LicenseType::DualLicense {
                open: Box::new(LicenseType::MIT),
                commercial: Box::new(LicenseType::Proprietary),
            },
        ];
        for l in licenses {
            let json = serde_json::to_string(&l).unwrap();
            let back: LicenseType = serde_json::from_str(&json).unwrap();
            assert_eq!(l, back);
        }
    }

    #[test]
    fn test_challenge_reason_serde_roundtrip() {
        let reasons = vec![
            ChallengeReason::PriorArt {
                existing_ref: "ref".into(),
            },
            ChallengeReason::InvalidClaim,
            ChallengeReason::InsufficientEvidence,
            ChallengeReason::LicenseViolation,
            ChallengeReason::Other {
                description: "test".into(),
            },
        ];
        for r in reasons {
            let json = serde_json::to_string(&r).unwrap();
            let back: ChallengeReason = serde_json::from_str(&json).unwrap();
            assert_eq!(r, back);
        }
    }

    #[test]
    fn test_anchor_serde_roundtrip() {
        let anchor = Anchor("invention:test-001".into());
        let json = serde_json::to_string(&anchor).unwrap();
        let back: Anchor = serde_json::from_str(&json).unwrap();
        assert_eq!(anchor, back);
    }

    // ── InventionRoyaltyRule validation ───────────────────────────────

    fn valid_royalty_rule() -> InventionRoyaltyRule {
        InventionRoyaltyRule {
            id: "royalty:inv-001:derivative:1234".into(),
            invention_id: "inv-001".into(),
            rule_type: RoyaltyTrigger::PerDerivativeWork,
            percentage: 5.0,
            minimum_amount_tend: Some(0.5),
            receiver_did: "did:mycelix:alice".into(),
            active: true,
            created_at: 1234567890,
        }
    }

    #[test]
    fn test_valid_royalty_rule() {
        let result = validate_create_royalty_rule(test_action(), valid_royalty_rule()).unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn test_royalty_rule_empty_id_rejected() {
        let mut r = valid_royalty_rule();
        r.id = String::new();
        let result = validate_create_royalty_rule(test_action(), r).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_royalty_rule_empty_invention_id_rejected() {
        let mut r = valid_royalty_rule();
        r.invention_id = String::new();
        let result = validate_create_royalty_rule(test_action(), r).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_royalty_rule_percentage_over_100_rejected() {
        let mut r = valid_royalty_rule();
        r.percentage = 100.1;
        let result = validate_create_royalty_rule(test_action(), r).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_royalty_rule_percentage_negative_rejected() {
        let mut r = valid_royalty_rule();
        r.percentage = -0.1;
        let result = validate_create_royalty_rule(test_action(), r).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_royalty_rule_percentage_zero_valid() {
        let mut r = valid_royalty_rule();
        r.percentage = 0.0;
        let result = validate_create_royalty_rule(test_action(), r).unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn test_royalty_rule_percentage_100_valid() {
        let mut r = valid_royalty_rule();
        r.percentage = 100.0;
        let result = validate_create_royalty_rule(test_action(), r).unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn test_royalty_rule_negative_minimum_rejected() {
        let mut r = valid_royalty_rule();
        r.minimum_amount_tend = Some(-1.0);
        let result = validate_create_royalty_rule(test_action(), r).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_royalty_rule_no_minimum_valid() {
        let mut r = valid_royalty_rule();
        r.minimum_amount_tend = None;
        let result = validate_create_royalty_rule(test_action(), r).unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn test_royalty_rule_bad_receiver_did_rejected() {
        let mut r = valid_royalty_rule();
        r.receiver_did = "not-a-did".into();
        let result = validate_create_royalty_rule(test_action(), r).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_royalty_rule_update_rejected_immutable() {
        // Royalty rules are now immutable (2026-07-09 P0 pass) -- no
        // coordinator function ever updates one.
        let result =
            validate_update_royalty_rule(test_update_action(), valid_royalty_rule()).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    // ── RoyaltyLedgerEntry validation ────────────────────────────────

    fn valid_ledger_entry() -> RoyaltyLedgerEntry {
        RoyaltyLedgerEntry {
            id: "ledger:inv-001:derivative:5678".into(),
            royalty_rule_id: "royalty:inv-001:derivative:1234".into(),
            invention_id: "inv-001".into(),
            payer_did: "did:mycelix:bob".into(),
            receiver_did: "did:mycelix:alice".into(),
            amount_tend: 0.5,
            trigger_event: "Derivative invention inv-002 registered".into(),
            trigger_action_hash: None,
            status: RoyaltyStatus::Pending,
            created_at: 1234567890,
            paid_at: None,
        }
    }

    #[test]
    fn test_valid_ledger_entry() {
        let result = validate_create_ledger_entry(test_action(), valid_ledger_entry()).unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn test_ledger_entry_empty_id_rejected() {
        let mut e = valid_ledger_entry();
        e.id = String::new();
        let result = validate_create_ledger_entry(test_action(), e).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_ledger_entry_empty_rule_id_rejected() {
        let mut e = valid_ledger_entry();
        e.royalty_rule_id = String::new();
        let result = validate_create_ledger_entry(test_action(), e).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_ledger_entry_empty_invention_id_rejected() {
        let mut e = valid_ledger_entry();
        e.invention_id = String::new();
        let result = validate_create_ledger_entry(test_action(), e).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_ledger_entry_bad_payer_did_rejected() {
        let mut e = valid_ledger_entry();
        e.payer_did = "not-did".into();
        let result = validate_create_ledger_entry(test_action(), e).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_ledger_entry_bad_receiver_did_rejected() {
        let mut e = valid_ledger_entry();
        e.receiver_did = "not-did".into();
        let result = validate_create_ledger_entry(test_action(), e).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_ledger_entry_negative_amount_rejected() {
        let mut e = valid_ledger_entry();
        e.amount_tend = -1.0;
        let result = validate_create_ledger_entry(test_action(), e).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_ledger_entry_empty_trigger_rejected() {
        let mut e = valid_ledger_entry();
        e.trigger_event = String::new();
        let result = validate_create_ledger_entry(test_action(), e).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_ledger_entry_zero_amount_valid() {
        let mut e = valid_ledger_entry();
        e.amount_tend = 0.0;
        let result = validate_create_ledger_entry(test_action(), e).unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn test_ledger_entry_with_trigger_hash_valid() {
        let mut e = valid_ledger_entry();
        e.trigger_action_hash = Some("abc123def456".into());
        let result = validate_create_ledger_entry(test_action(), e).unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    // Note: a "fully valid update" case for validate_update_ledger_entry
    // can't be exercised as a plain unit test anymore -- it now calls
    // must_get_valid_record to fetch the original entry and check the
    // status-transition rules, which needs a live HDI host this crate's
    // tests don't mock. Coverage for the "valid" path needs a sweettest /
    // live-conductor check instead.

    // ── Royalty enum serde roundtrips ─────────────────────────────────

    #[test]
    fn test_royalty_trigger_serde_roundtrip() {
        let triggers = vec![
            RoyaltyTrigger::PerDerivativeWork,
            RoyaltyTrigger::PerCommercialUse,
            RoyaltyTrigger::PerProductionDeploy,
            RoyaltyTrigger::Subscription,
            RoyaltyTrigger::OneTime,
        ];
        for t in triggers {
            let json = serde_json::to_string(&t).unwrap();
            let back: RoyaltyTrigger = serde_json::from_str(&json).unwrap();
            assert_eq!(t, back);
        }
    }

    #[test]
    fn test_royalty_status_serde_roundtrip() {
        let statuses = vec![
            RoyaltyStatus::Pending,
            RoyaltyStatus::Paid,
            RoyaltyStatus::Disputed,
            RoyaltyStatus::Waived,
            RoyaltyStatus::Expired,
        ];
        for s in statuses {
            let json = serde_json::to_string(&s).unwrap();
            let back: RoyaltyStatus = serde_json::from_str(&json).unwrap();
            assert_eq!(s, back);
        }
    }
}
