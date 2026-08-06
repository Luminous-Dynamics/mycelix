// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Trust Integrity Zome - MATL (Mycelix Advanced Trust Logic)
//!
//! Implements a sophisticated web-of-trust system with:
//! - Direct trust attestations
//! - Transitive trust propagation with decay
//! - Byzantine fault tolerance
//! - Temporal decay of trust scores
//! - Multi-factor trust evidence
//! - Reputation staking

use hdi::prelude::*;

/// Direct trust attestation between two agents
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct TrustAttestation {
    /// Who is attesting (the truster)
    pub truster: AgentPubKey,
    /// Who is being trusted (the trustee)
    pub trustee: AgentPubKey,
    /// Trust level (-1.0 to 1.0, where negative is distrust)
    pub trust_level: f64,
    /// Category of trust
    pub category: TrustCategory,
    /// Evidence supporting this attestation
    pub evidence: Vec<TrustEvidence>,
    /// Optional message/reason
    pub reason: Option<String>,
    /// When this attestation was made
    pub created_at: Timestamp,
    /// Expiration (attestations can expire)
    pub expires_at: Option<Timestamp>,
    /// Signature proving authenticity
    pub signature: Vec<u8>,
    /// Is this attestation revoked?
    pub revoked: bool,
    /// Stake amount (optional reputation staking)
    pub stake: Option<ReputationStake>,
}

/// Categories of trust (different contexts)
#[hdk_entry_helper]
#[derive(Clone, PartialEq, Default)]
pub enum TrustCategory {
    /// General identity trust
    #[default]
    Identity,
    /// Trust for email communication
    Communication,
    /// Trust for sharing documents/files
    FileSharing,
    /// Trust for calendar/scheduling
    Scheduling,
    /// Trust for financial transactions
    Financial,
    /// Trust as credential issuer
    CredentialIssuer,
    /// Trust within organization
    Organization,
    /// Personal relationship trust
    Personal,
    /// Professional relationship trust
    Professional,
    /// Custom category
    Custom(String),
}

/// Evidence supporting a trust attestation
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct TrustEvidence {
    /// Type of evidence
    pub evidence_type: EvidenceType,
    /// Reference to evidence (hash, URL, etc.)
    pub reference: String,
    /// Weight of this evidence (0.0 to 1.0)
    pub weight: f64,
    /// When evidence was collected
    pub collected_at: Timestamp,
    /// Optional verification status
    pub verified: bool,
}

/// Types of evidence that can support trust
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub enum EvidenceType {
    /// In-person verification
    InPersonMeeting,
    /// Video call verification
    VideoVerification,
    /// Phone verification
    PhoneVerification,
    /// Verified credential presentation
    VerifiableCredential,
    /// Social media cross-reference
    SocialMediaVerification,
    /// Email domain verification (e.g., @company.com)
    DomainVerification,
    /// PGP key signing
    PgpKeySigning,
    /// Mutual connection vouching
    MutualVouch,
    /// Long-term communication history
    CommunicationHistory,
    /// Blockchain attestation
    BlockchainAttestation,
    /// Government ID verification
    GovernmentId,
    /// Professional certification
    ProfessionalCertification,
    /// Organization membership
    OrganizationMembership,
    /// Custom evidence type
    Custom(String),
}

/// Reputation stake for weighted attestations
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct ReputationStake {
    /// Amount of reputation staked
    pub amount: u64,
    /// Lock period (can't unstake until then)
    pub locked_until: Timestamp,
    /// Penalty multiplier if attestation is disputed
    pub penalty_multiplier: f64,
}

/// Aggregated trust score for an agent
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct TrustScore {
    /// The agent this score is for
    pub agent: AgentPubKey,
    /// Overall computed trust score (0.0 to 1.0)
    pub score: f64,
    /// Confidence level (how certain we are)
    pub confidence: f64,
    /// Number of direct attestations
    pub direct_attestation_count: u32,
    /// Number of transitive paths
    pub transitive_path_count: u32,
    /// Category-specific scores
    pub category_scores: Vec<(TrustCategory, f64)>,
    /// When this score was computed
    pub computed_at: Timestamp,
    /// Hash of attestations used in computation
    pub attestation_hashes: Vec<ActionHash>,
    /// Byzantine indicators
    pub byzantine_flags: Vec<ByzantineFlag>,
}

/// Flags indicating potential Byzantine behavior
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct ByzantineFlag {
    pub flag_type: ByzantineFlagType,
    pub severity: f64,
    pub detected_at: Timestamp,
    pub evidence: String,
}

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub enum ByzantineFlagType {
    /// Sybil attack detection (fake identities)
    SybilSuspicion,
    /// Collusion detection
    CollusionPattern,
    /// Rapid trust changes
    TrustVolatility,
    /// Inconsistent attestations
    InconsistentAttestations,
    /// Self-dealing patterns
    SelfDealing,
    /// Attestation farming
    AttestationFarming,
}

/// Trust query from one agent about another
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct TrustQuery {
    /// Who is asking
    pub querier: AgentPubKey,
    /// Who they're asking about
    pub subject: AgentPubKey,
    /// Category of interest
    pub category: Option<TrustCategory>,
    /// Maximum depth for transitive trust
    pub max_depth: u8,
    /// Minimum confidence threshold
    pub min_confidence: f64,
    /// When queried
    pub timestamp: Timestamp,
}

/// Trust dispute/challenge
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct TrustDispute {
    /// The attestation being disputed
    pub attestation_hash: ActionHash,
    /// Who is disputing
    pub disputer: AgentPubKey,
    /// Reason for dispute
    pub reason: String,
    /// Counter-evidence
    pub counter_evidence: Vec<TrustEvidence>,
    /// Status of dispute
    pub status: DisputeStatus,
    /// When filed
    pub filed_at: Timestamp,
    /// Resolution (if any)
    pub resolution: Option<DisputeResolution>,
}

#[hdk_entry_helper]
#[derive(Clone, PartialEq, Default)]
pub enum DisputeStatus {
    #[default]
    Open,
    UnderReview,
    Resolved,
    Rejected,
}

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct DisputeResolution {
    pub outcome: DisputeOutcome,
    pub resolved_at: Timestamp,
    pub resolver_notes: String,
    pub attestation_modified: bool,
    pub stake_slashed: bool,
}

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub enum DisputeOutcome {
    AttestationUpheld,
    AttestationRevoked,
    PartialRevocation,
    DisputeRejected,
}

/// Introduction request (asking for trust propagation)
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct TrustIntroduction {
    /// Who is introducing
    pub introducer: AgentPubKey,
    /// Who is being introduced
    pub introduced: AgentPubKey,
    /// Who they're being introduced to
    pub target: AgentPubKey,
    /// Introducer's recommendation level
    pub recommendation_level: f64,
    /// Category
    pub category: TrustCategory,
    /// Message from introducer
    pub message: Option<String>,
    /// When introduced
    pub introduced_at: Timestamp,
    /// Whether target accepted
    pub accepted: Option<bool>,
}

/// Web-of-trust path (for auditing trust propagation)
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct TrustPath {
    /// Starting agent
    pub from: AgentPubKey,
    /// Ending agent
    pub to: AgentPubKey,
    /// Agents along the path
    pub path: Vec<AgentPubKey>,
    /// Attestation hashes for each hop
    pub attestation_hashes: Vec<ActionHash>,
    /// Computed trust at each hop
    pub trust_at_hop: Vec<f64>,
    /// Final computed trust
    pub final_trust: f64,
    /// When computed
    pub computed_at: Timestamp,
}

/// Link types for trust graph
#[hdk_link_types]
pub enum LinkTypes {
    /// Agent -> attestations they've made
    AgentToGivenAttestations,
    /// Agent -> attestations about them
    AgentToReceivedAttestations,
    /// Agent -> their computed trust score
    AgentToTrustScore,
    /// Agent -> trust queries they've made
    AgentToQueries,
    /// Attestation -> disputes about it
    AttestationToDisputes,
    /// Agent -> introductions they've made
    AgentToIntroductions,
    /// Agent -> introductions received
    AgentToReceivedIntroductions,
    /// Agent -> their reputation stake balance
    AgentToStake,
    /// Category anchor -> attestations in category
    CategoryToAttestations,
    /// Global anchor for discovery
    GlobalTrustAnchor,
}

/// Entry types
#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    #[entry_type(required_validations = 3)]
    TrustAttestation(TrustAttestation),
    #[entry_type(required_validations = 2)]
    TrustScore(TrustScore),
    #[entry_type(required_validations = 2)]
    TrustDispute(TrustDispute),
    #[entry_type(required_validations = 2)]
    TrustIntroduction(TrustIntroduction),
    TrustPath(TrustPath),
    TrustQuery(TrustQuery),
}

// ==================== VALIDATION ====================

#[hdk_extern]
pub fn genesis_self_check(_data: GenesisSelfCheckData) -> ExternResult<ValidateCallbackResult> {
    Ok(ValidateCallbackResult::Valid)
}

#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(store_entry) => match store_entry {
            OpEntry::CreateEntry { app_entry, action } => validate_create_entry(app_entry, action),
            OpEntry::UpdateEntry {
                app_entry, action, ..
            } => validate_update_entry(app_entry, action),
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::StoreRecord(store_record) => match store_record {
            OpRecord::CreateEntry { app_entry, action } => validate_create_entry(app_entry, action),
            _ => Ok(ValidateCallbackResult::Valid),
        },
        _ => Ok(ValidateCallbackResult::Valid),
    }
}

fn validate_create_entry(
    entry: EntryTypes,
    action: Create,
) -> ExternResult<ValidateCallbackResult> {
    match entry {
        EntryTypes::TrustAttestation(attestation) => validate_attestation(&attestation, &action),
        EntryTypes::TrustScore(score) => validate_score(&score, &action),
        EntryTypes::TrustDispute(dispute) => validate_dispute(&dispute, &action),
        EntryTypes::TrustIntroduction(intro) => validate_introduction(&intro, &action),
        _ => Ok(ValidateCallbackResult::Valid),
    }
}

fn validate_update_entry(
    entry: EntryTypes,
    action: Update,
) -> ExternResult<ValidateCallbackResult> {
    match entry {
        // Attestations can only be updated by truster (to revoke)
        EntryTypes::TrustAttestation(attestation) => {
            if attestation.truster != action.author {
                return Ok(ValidateCallbackResult::Invalid(
                    "Only truster can update attestation".to_string(),
                ));
            }
            Ok(ValidateCallbackResult::Valid)
        }
        // Disputes can be updated for resolution by disputer only
        EntryTypes::TrustDispute(dispute) => {
            if dispute.disputer != action.author {
                return Ok(ValidateCallbackResult::Invalid(
                    "Only the disputer can update a dispute".to_string(),
                ));
            }
            Ok(ValidateCallbackResult::Valid)
        }
        _ => Ok(ValidateCallbackResult::Valid),
    }
}

fn validate_attestation(
    attestation: &TrustAttestation,
    action: &Create,
) -> ExternResult<ValidateCallbackResult> {
    // Truster must be the author
    if attestation.truster != action.author {
        return Ok(ValidateCallbackResult::Invalid(
            "Truster must match action author".to_string(),
        ));
    }

    // Can't attest to yourself
    if attestation.truster == attestation.trustee {
        return Ok(ValidateCallbackResult::Invalid(
            "Cannot create self-attestation".to_string(),
        ));
    }

    // Trust level must be in valid range and finite
    if !attestation.trust_level.is_finite()
        || attestation.trust_level < -1.0
        || attestation.trust_level > 1.0
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Trust level must be a finite number between -1.0 and 1.0".to_string(),
        ));
    }

    // Must have signature
    if attestation.signature.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Attestation must be signed".to_string(),
        ));
    }

    // Validate evidence weights (must be finite)
    for evidence in &attestation.evidence {
        if !evidence.weight.is_finite() || evidence.weight < 0.0 || evidence.weight > 1.0 {
            return Ok(ValidateCallbackResult::Invalid(
                "Evidence weight must be a finite number between 0.0 and 1.0".to_string(),
            ));
        }
    }

    // If staking, validate stake (penalty_multiplier must be finite)
    if let Some(stake) = &attestation.stake {
        if !stake.penalty_multiplier.is_finite() || stake.penalty_multiplier < 1.0 {
            return Ok(ValidateCallbackResult::Invalid(
                "Penalty multiplier must be a finite number at least 1.0".to_string(),
            ));
        }
    }

    // Expiration must be in future
    if let Some(expires) = attestation.expires_at {
        if expires <= attestation.created_at {
            return Ok(ValidateCallbackResult::Invalid(
                "Expiration must be after creation".to_string(),
            ));
        }
    }

    Ok(ValidateCallbackResult::Valid)
}

fn validate_score(score: &TrustScore, _action: &Create) -> ExternResult<ValidateCallbackResult> {
    // Score must be in valid range and finite
    if !score.score.is_finite() || score.score < 0.0 || score.score > 1.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Score must be a finite number between 0.0 and 1.0".to_string(),
        ));
    }

    // Confidence must be in valid range and finite
    if !score.confidence.is_finite() || score.confidence < 0.0 || score.confidence > 1.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Confidence must be a finite number between 0.0 and 1.0".to_string(),
        ));
    }

    // direct_attestation_count is claimed to equal the number of cited attestation
    // hashes. This is the one part of the MATL score that is bounded and
    // deterministically checkable at the DHT layer.
    if score.attestation_hashes.len() as u32 != score.direct_attestation_count {
        return Ok(ValidateCallbackResult::Invalid(
            "direct_attestation_count must equal the number of cited attestation_hashes"
                .to_string(),
        ));
    }

    // No duplicate citations (a coordinator could otherwise double-count the same
    // attestation to inflate direct_attestation_count / confidence).
    let mut seen: Vec<&ActionHash> = Vec::new();
    for hash in &score.attestation_hashes {
        if seen.contains(&hash) {
            return Ok(ValidateCallbackResult::Invalid(
                "attestation_hashes contains a duplicate reference".to_string(),
            ));
        }
        seen.push(hash);
    }

    // Every cited hash must be a real, non-revoked TrustAttestation whose trustee
    // is the subject this score is about.
    for hash in &score.attestation_hashes {
        let record = must_get_valid_record(hash.clone())?;
        let attestation: TrustAttestation = record
            .entry()
            .to_app_option()
            .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
            .ok_or(wasm_error!(WasmErrorInner::Guest(
                "attestation_hashes entry must decode as TrustAttestation".to_string()
            )))?;

        if attestation.trustee != score.agent {
            return Ok(ValidateCallbackResult::Invalid(
                "Cited attestation is not about this score's subject".to_string(),
            ));
        }
        if attestation.revoked {
            return Ok(ValidateCallbackResult::Invalid(
                "Cited attestation is revoked".to_string(),
            ));
        }
    }

    // NOTE: `score`/`confidence`/`transitive_path_count`/`byzantine_flags` are NOT
    // re-derived here. MATL's transitive-trust component (`calculate_transitive_trust`
    // in the coordinator) performs an open-ended BFS over `get_links`, walking other
    // agents' attestation graphs from the querying agent's own perspective. That is
    // not deterministically replayable in a validation callback using only
    // `must_get_*` against explicit hashes (get_links results are eventually
    // consistent and peer-dependent, and the entry does not record which edges the
    // BFS actually traversed). This check therefore proves the cited direct
    // attestations are real, non-revoked, and about the right subject -- it does NOT
    // prove `score` itself was honestly computed from them plus a genuine transitive
    // walk. Closing that gap would require either embedding a full serialized BFS
    // trace (attestation hashes visited, depth, accumulated weight) so a validator
    // could replay a bounded computation, or accepting social/economic security
    // (TrustAttestation's existing staking/penalty mechanism) as the backstop for the
    // transitive and Byzantine-detection components instead of a DHT-level proof.
    Ok(ValidateCallbackResult::Valid)
}

fn validate_dispute(
    dispute: &TrustDispute,
    action: &Create,
) -> ExternResult<ValidateCallbackResult> {
    // Disputer must be author
    if dispute.disputer != action.author {
        return Ok(ValidateCallbackResult::Invalid(
            "Disputer must match action author".to_string(),
        ));
    }

    // Must have reason
    if dispute.reason.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Dispute must have a reason".to_string(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

fn validate_introduction(
    intro: &TrustIntroduction,
    action: &Create,
) -> ExternResult<ValidateCallbackResult> {
    // Introducer must be author
    if intro.introducer != action.author {
        return Ok(ValidateCallbackResult::Invalid(
            "Introducer must match action author".to_string(),
        ));
    }

    // All three parties must be different
    if intro.introducer == intro.introduced
        || intro.introducer == intro.target
        || intro.introduced == intro.target
    {
        return Ok(ValidateCallbackResult::Invalid(
            "All parties in introduction must be different".to_string(),
        ));
    }

    // Recommendation level must be valid and finite
    if !intro.recommendation_level.is_finite()
        || intro.recommendation_level < 0.0
        || intro.recommendation_level > 1.0
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Recommendation level must be a finite number between 0.0 and 1.0".to_string(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Proves `validate_score`'s P0 author-binding fix: a TrustScore citing an attestation
/// hash that either isn't about the score's own subject, or has been revoked, is rejected
/// (previously TrustScore had zero verification beyond range checks). Mocks the HDI
/// host's `must_get_valid_record` so this runs as a plain `cargo test`, no live conductor
/// needed.
#[cfg(test)]
mod tests {
    use super::*;

    struct MockRecordHdi {
        records: std::collections::HashMap<ActionHash, Record>,
    }

    impl hdi::hdi::HdiT for MockRecordHdi {
        fn must_get_valid_record(&self, input: MustGetValidRecordInput) -> ExternResult<Record> {
            self.records
                .get(&input.0)
                .cloned()
                .ok_or_else(|| wasm_error!(WasmErrorInner::Guest("no such record in mock".into())))
        }
        fn verify_signature(&self, _: VerifySignature) -> ExternResult<bool> {
            unimplemented!("not exercised by this fix")
        }
        fn must_get_entry(&self, _: MustGetEntryInput) -> ExternResult<EntryHashed> {
            unimplemented!("not exercised by this fix")
        }
        fn must_get_action(&self, _: MustGetActionInput) -> ExternResult<SignedActionHashed> {
            unimplemented!("not exercised by this fix")
        }
        fn must_get_agent_activity(
            &self,
            _: MustGetAgentActivityInput,
        ) -> ExternResult<Vec<RegisterAgentActivity>> {
            unimplemented!("not exercised by this fix")
        }
        fn dna_info(&self, _: ()) -> ExternResult<DnaInfo> {
            unimplemented!("not exercised by this fix")
        }
        fn zome_info(&self, _: ()) -> ExternResult<ZomeInfo> {
            unimplemented!("not exercised by this fix")
        }
        fn trace(&self, _: TraceMsg) -> ExternResult<()> {
            unimplemented!("not exercised by this fix")
        }
        fn x_salsa20_poly1305_decrypt(
            &self,
            _: XSalsa20Poly1305Decrypt,
        ) -> ExternResult<Option<XSalsa20Poly1305Data>> {
            unimplemented!("not exercised by this fix")
        }
        fn x_25519_x_salsa20_poly1305_decrypt(
            &self,
            _: X25519XSalsa20Poly1305Decrypt,
        ) -> ExternResult<Option<XSalsa20Poly1305Data>> {
            unimplemented!("not exercised by this fix")
        }
        fn ed_25519_x_salsa20_poly1305_decrypt(
            &self,
            _: Ed25519XSalsa20Poly1305Decrypt,
        ) -> ExternResult<XSalsa20Poly1305Data> {
            unimplemented!("not exercised by this fix")
        }
    }

    fn wrap_entry_record<T>(author: AgentPubKey, value: T) -> Record
    where
        T: TryInto<SerializedBytes>,
        <T as TryInto<SerializedBytes>>::Error: std::fmt::Debug,
    {
        let entry = Entry::App(AppEntryBytes::try_from(value.try_into().unwrap()).unwrap());
        let action = Action::Create(Create {
            author,
            timestamp: Timestamp::from_micros(0),
            action_seq: 0,
            prev_action: ActionHash::from_raw_36(vec![0; 36]),
            entry_type: EntryType::App(AppEntryDef::new(
                EntryDefIndex(0),
                ZomeIndex(0),
                EntryVisibility::Public,
            )),
            entry_hash: EntryHash::from_raw_36(vec![1; 36]),
            weight: Default::default(),
        });
        let hashed = HoloHashed::from_content_sync(action);
        let signed_action = SignedActionHashed::with_presigned(hashed, Signature([0; 64]));
        Record::new(signed_action, Some(entry))
    }

    fn test_attestation(trustee: AgentPubKey, revoked: bool) -> TrustAttestation {
        TrustAttestation {
            truster: AgentPubKey::from_raw_36(vec![50; 36]),
            trustee,
            trust_level: 0.8,
            category: TrustCategory::Identity,
            evidence: vec![],
            reason: None,
            created_at: Timestamp::from_micros(0),
            expires_at: None,
            signature: vec![],
            revoked,
            stake: None,
        }
    }

    fn test_score(agent: AgentPubKey, attestation_hashes: Vec<ActionHash>) -> TrustScore {
        let count = attestation_hashes.len() as u32;
        TrustScore {
            agent,
            score: 0.8,
            confidence: 0.5,
            direct_attestation_count: count,
            transitive_path_count: 0,
            category_scores: vec![],
            computed_at: Timestamp::from_micros(0),
            attestation_hashes,
            byzantine_flags: vec![],
        }
    }

    fn test_action() -> Create {
        Create {
            author: AgentPubKey::from_raw_36(vec![99; 36]),
            timestamp: Timestamp::from_micros(0),
            action_seq: 0,
            prev_action: ActionHash::from_raw_36(vec![0; 36]),
            entry_type: EntryType::App(AppEntryDef::new(
                EntryDefIndex(0),
                ZomeIndex(0),
                EntryVisibility::Public,
            )),
            entry_hash: EntryHash::from_raw_36(vec![1; 36]),
            weight: Default::default(),
        }
    }

    #[test]
    fn score_citing_attestation_about_a_different_agent_is_rejected() {
        let subject = AgentPubKey::from_raw_36(vec![1; 36]);
        let someone_else = AgentPubKey::from_raw_36(vec![2; 36]);
        let hash = ActionHash::from_raw_36(vec![10; 36]);
        hdi::hdi::set_hdi(MockRecordHdi {
            records: std::collections::HashMap::from([(
                hash.clone(),
                wrap_entry_record(
                    AgentPubKey::from_raw_36(vec![50; 36]),
                    test_attestation(someone_else, false),
                ),
            )]),
        });

        let score = test_score(subject, vec![hash]);
        let result = validate_score(&score, &test_action()).unwrap();
        assert!(
            matches!(result, ValidateCallbackResult::Invalid(_)),
            "a cited attestation whose trustee isn't this score's subject must be rejected \
             -- previously TrustScore had zero verification beyond range checks"
        );
    }

    #[test]
    fn score_citing_a_revoked_attestation_is_rejected() {
        let subject = AgentPubKey::from_raw_36(vec![1; 36]);
        let hash = ActionHash::from_raw_36(vec![10; 36]);
        hdi::hdi::set_hdi(MockRecordHdi {
            records: std::collections::HashMap::from([(
                hash.clone(),
                wrap_entry_record(
                    AgentPubKey::from_raw_36(vec![50; 36]),
                    test_attestation(subject.clone(), true),
                ),
            )]),
        });

        let score = test_score(subject, vec![hash]);
        let result = validate_score(&score, &test_action()).unwrap();
        assert!(
            matches!(result, ValidateCallbackResult::Invalid(_)),
            "a cited attestation that has been revoked must be rejected"
        );
    }

    #[test]
    fn score_citing_a_real_matching_attestation_is_accepted() {
        let subject = AgentPubKey::from_raw_36(vec![1; 36]);
        let hash = ActionHash::from_raw_36(vec![10; 36]);
        hdi::hdi::set_hdi(MockRecordHdi {
            records: std::collections::HashMap::from([(
                hash.clone(),
                wrap_entry_record(
                    AgentPubKey::from_raw_36(vec![50; 36]),
                    test_attestation(subject.clone(), false),
                ),
            )]),
        });

        let score = test_score(subject, vec![hash]);
        let result = validate_score(&score, &test_action()).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Valid));
    }
}
