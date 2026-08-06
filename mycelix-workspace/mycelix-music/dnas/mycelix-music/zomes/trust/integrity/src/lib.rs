// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Trust Integrity Zome
//!
//! Implements Multi-Agent Trust Logic (MATL) for Mycelix Music.
//! - Artist verification through web-of-trust
//! - CDN node reputation scoring
//! - Byzantine detection integration from Mycelix-Core

use hdi::prelude::*;

/// Trust claim - one agent vouches for another
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct TrustClaim {
    /// Agent making the claim
    pub from: AgentPubKey,
    /// Agent being vouched for
    pub to: AgentPubKey,
    /// Type of trust claim
    pub claim_type: TrustClaimType,
    /// Confidence level (0-1000, basis points)
    pub confidence_bps: u32,
    /// Evidence supporting the claim (IPFS hash, URL, etc.)
    pub evidence: Option<String>,
    /// Timestamp
    pub created_at: Timestamp,
    /// Expiry (if applicable)
    pub expires_at: Option<Timestamp>,
    /// Whether this claim is still active
    pub active: bool,
}

/// Types of trust claims
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub enum TrustClaimType {
    /// Identity verification (real artist)
    IdentityVerification,
    /// Content authenticity (original work)
    ContentAuthenticity,
    /// Quality attestation (good music)
    QualityAttestation,
    /// CDN reliability (node uptime)
    CdnReliability,
    /// Payment reliability (pays on time)
    PaymentReliability,
    /// General endorsement
    GeneralEndorsement,
}

/// Artist verification status
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct VerificationStatus {
    /// Artist being verified
    pub artist: AgentPubKey,
    /// Overall trust score (0-1000)
    pub trust_score: u32,
    /// Verification tier
    pub tier: VerificationTier,
    /// Number of vouches
    pub vouch_count: u32,
    /// Last computed timestamp
    pub computed_at: Timestamp,
    /// The exact set of active TrustClaim hashes this computation aggregated over. Lets
    /// the DHT re-derive recompute_verification's own formula from real, checkable source
    /// data instead of trusting an arbitrary coordinator-supplied trust_score/tier/
    /// vouch_count (P0 author-binding Turn B -- previously this entry had zero validation
    /// of any kind).
    pub source_claims: Vec<ActionHash>,
}

/// Verification tiers
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub enum VerificationTier {
    /// No verification
    Unverified,
    /// Community vouched (3+ claims)
    CommunityVerified,
    /// Highly trusted (10+ claims, high scores)
    Trusted,
    /// Platform verified (official verification)
    PlatformVerified,
    /// Founding artist
    FoundingArtist,
}

/// CDN node reputation
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct CdnNodeReputation {
    /// Node's agent pub key
    pub node: AgentPubKey,
    /// Ethereum address (for staking/rewards)
    pub eth_address: String,
    /// IPFS peer ID
    pub ipfs_peer_id: String,
    /// Geographic region
    pub region: String,
    /// Total bytes served
    pub bytes_served: u64,
    /// Successful requests
    pub successful_requests: u64,
    /// Failed requests
    pub failed_requests: u64,
    /// Average latency (ms)
    pub avg_latency_ms: u32,
    /// Uptime percentage (basis points)
    pub uptime_bps: u32,
    /// PoGQ score from Mycelix-Core
    pub pogq_score: f64,
    /// Last activity
    pub last_active: Timestamp,
    /// Stake amount (in wei)
    pub stake_amount: u64,
    /// Slashing events
    pub slash_count: u32,
    /// The ServiceQualityReport this create/update actually incorporated (None at
    /// registration, before any report has been folded in). Lets the DHT re-derive
    /// update_cdn_reputation's exact formula against one real, matching report instead of
    /// trusting an arbitrary coordinator-supplied stat set (P0 author-binding Turn B --
    /// previously this update path had zero validation of any kind).
    pub last_report_hash: Option<ActionHash>,
}

/// Service quality report (for CDN nodes)
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct ServiceQualityReport {
    /// Reporter (listener)
    pub reporter: AgentPubKey,
    /// CDN node being reported on
    pub node: AgentPubKey,
    /// Song that was served
    pub song_hash: ActionHash,
    /// Latency experienced (ms)
    pub latency_ms: u32,
    /// Was the request successful?
    pub success: bool,
    /// Error code if failed
    pub error_code: Option<String>,
    /// Timestamp
    pub reported_at: Timestamp,
}

/// Byzantine behavior report
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct ByzantineReport {
    /// Reporter
    pub reporter: AgentPubKey,
    /// Accused agent
    pub accused: AgentPubKey,
    /// Type of misbehavior
    pub behavior_type: ByzantineBehavior,
    /// Evidence
    pub evidence: String,
    /// Severity (0-100)
    pub severity: u8,
    /// Timestamp
    pub reported_at: Timestamp,
    /// Resolution status
    pub status: ReportStatus,
}

/// Types of Byzantine behavior
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub enum ByzantineBehavior {
    /// Serving corrupted content
    ContentCorruption,
    /// Claiming plays that didn't happen
    FakePlayClaims,
    /// Node serving wrong content
    WrongContent,
    /// Replay attacks
    ReplayAttack,
    /// Sybil attack (multiple fake identities)
    SybilAttack,
    /// Other
    Other,
}

/// Report status
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub enum ReportStatus {
    /// Under review
    Pending,
    /// Confirmed misbehavior
    Confirmed,
    /// Dismissed
    Dismissed,
    /// Slashing executed
    Slashed,
}

/// Link types
#[hdk_link_types]
pub enum LinkTypes {
    /// Agent -> Trust claims they made
    AgentToClaimsMade,
    /// Agent -> Trust claims about them
    AgentToClaimsReceived,
    /// Agent -> Verification status
    AgentToVerification,
    /// CDN node -> Reputation
    NodeToReputation,
    /// Agent -> Quality reports made
    AgentToReports,
    /// Byzantine reports anchor
    ByzantineReports,
}

/// Entry types
#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    TrustClaim(TrustClaim),
    VerificationStatus(VerificationStatus),
    CdnNodeReputation(CdnNodeReputation),
    ServiceQualityReport(ServiceQualityReport),
    ByzantineReport(ByzantineReport),
}

/// Validation
#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(store_entry) => match store_entry {
            OpEntry::CreateEntry { app_entry, action } => match app_entry {
                EntryTypes::TrustClaim(claim) => validate_trust_claim(claim, action),
                EntryTypes::VerificationStatus(status) => validate_verification_status(status),
                EntryTypes::CdnNodeReputation(rep) => validate_cdn_reputation(rep, action),
                EntryTypes::ServiceQualityReport(report) => validate_quality_report(report, action),
                EntryTypes::ByzantineReport(report) => validate_byzantine_report(report, action),
            },
            OpEntry::UpdateEntry {
                app_entry,
                action,
                original_action_hash,
                original_entry_hash: _,
            } => match app_entry {
                EntryTypes::TrustClaim(claim) => {
                    validate_update_trust_claim(claim, action, original_action_hash)
                }
                // VerificationStatus/ServiceQualityReport/ByzantineReport are confirmed
                // create-only (no coordinator function ever calls update_entry on any of
                // them) -- reject outright rather than leave an unbound dead-code path (P0
                // wide-open RegisterUpdate gap, confirmed dozens of times elsewhere in this
                // pass). CdnNodeReputation has a real update path (update_cdn_reputation,
                // triggered by third-party ServiceQualityReport submissions, not
                // self-authored) -- now re-derives that function's exact aggregation formula
                // against the real report named by last_report_hash.
                EntryTypes::VerificationStatus(_) => Ok(ValidateCallbackResult::Invalid(
                    "VerificationStatus entries cannot be updated".to_string(),
                )),
                EntryTypes::CdnNodeReputation(rep) => {
                    validate_update_cdn_reputation(rep, original_action_hash)
                }
                EntryTypes::ServiceQualityReport(_) => Ok(ValidateCallbackResult::Invalid(
                    "ServiceQualityReport entries cannot be updated".to_string(),
                )),
                EntryTypes::ByzantineReport(_) => Ok(ValidateCallbackResult::Invalid(
                    "ByzantineReport entries cannot be updated".to_string(),
                )),
            },
            _ => Ok(ValidateCallbackResult::Valid),
        },
        _ => Ok(ValidateCallbackResult::Valid),
    }
}

/// Re-derives revoke_trust_claim's own real, previously coordinator-only invariant at the
/// DHT level ("only the original claimant may update their own claim") -- since `from` is
/// already bound to the committer on create, checking against the original action's author
/// is equivalent. revoke_trust_claim only ever flips `active` to false; every other field
/// must stay byte-identical to the original (P0 wide-open RegisterUpdate gap).
fn validate_update_trust_claim(
    claim: TrustClaim,
    action: Update,
    original_action_hash: ActionHash,
) -> ExternResult<ValidateCallbackResult> {
    let original_action = must_get_action(original_action_hash)?;
    if original_action.action().author() != &action.author {
        return Ok(ValidateCallbackResult::Invalid(
            "Only the original claimant can update a trust claim".to_string(),
        ));
    }

    let original_record = must_get_valid_record(action.original_action_address.clone())?;
    let Some(original): Option<TrustClaim> = original_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(e))?
    else {
        return Ok(ValidateCallbackResult::Invalid(
            "Invalid original TrustClaim entry".to_string(),
        ));
    };

    if claim.from != original.from
        || claim.to != original.to
        || claim.claim_type != original.claim_type
        || claim.confidence_bps != original.confidence_bps
        || claim.evidence != original.evidence
        || claim.created_at != original.created_at
        || claim.expires_at != original.expires_at
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Trust claim updates may only change 'active' -- all other fields must be unchanged"
                .to_string(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

fn validate_trust_claim(claim: TrustClaim, action: Create) -> ExternResult<ValidateCallbackResult> {
    // From must match author
    if claim.from != action.author {
        return Ok(ValidateCallbackResult::Invalid(
            "Trust claim 'from' must match action author".to_string(),
        ));
    }

    // Cannot vouch for self
    if claim.from == claim.to {
        return Ok(ValidateCallbackResult::Invalid(
            "Cannot create trust claim for self".to_string(),
        ));
    }

    // Confidence must be valid
    if claim.confidence_bps > 1000 {
        return Ok(ValidateCallbackResult::Invalid(
            "Confidence must be 0-1000 basis points".to_string(),
        ));
    }

    // Evidence bounded
    if let Some(ref evidence) = claim.evidence {
        if evidence.len() > 4096 {
            return Ok(ValidateCallbackResult::Invalid(
                "Evidence must be <= 4KB".to_string(),
            ));
        }
    }

    Ok(ValidateCallbackResult::Valid)
}

/// VerificationStatus is a third-party computed aggregate (recompute_verification writes it
/// on behalf of the artist being verified, never self-reported, and always via a fresh
/// create_entry -- there is no incremental update to re-derive a delta against). Previously
/// it had zero validation of any kind, so a modified coordinator could conjure an arbitrary
/// trust_score/tier/vouch_count from nothing. This re-derives recompute_verification's exact
/// formula from the real TrustClaims named in source_claims.
///
/// Disclosed residual limitation: this proves every claim counted is real, active, and
/// correctly aggregated -- it does NOT prove completeness (that source_claims includes every
/// active TrustClaim actually addressed to this artist). A coordinator could still submit a
/// status that only cites a cherry-picked favorable subset. Closing that needs a stronger
/// completeness proof this pass doesn't attempt; see
/// memory/mycelix_attribution_author_binding_jul8.md.
fn validate_verification_status(
    status: VerificationStatus,
) -> ExternResult<ValidateCallbackResult> {
    if status.source_claims.len() as u32 != status.vouch_count {
        return Ok(ValidateCallbackResult::Invalid(
            "vouch_count must equal the number of source_claims".to_string(),
        ));
    }

    let mut seen: Vec<ActionHash> = Vec::with_capacity(status.source_claims.len());
    let mut total_confidence: u32 = 0;
    for claim_hash in &status.source_claims {
        if seen.contains(claim_hash) {
            return Ok(ValidateCallbackResult::Invalid(
                "source_claims must not contain duplicate hashes".to_string(),
            ));
        }
        seen.push(claim_hash.clone());

        let record = must_get_valid_record(claim_hash.clone())?;
        let Some(claim): Option<TrustClaim> =
            record.entry().to_app_option().map_err(|e| wasm_error!(e))?
        else {
            return Ok(ValidateCallbackResult::Invalid(
                "source_claims must reference valid TrustClaim entries".to_string(),
            ));
        };
        if claim.to != status.artist {
            return Ok(ValidateCallbackResult::Invalid(
                "Every source claim must be addressed to the verified artist".to_string(),
            ));
        }
        if !claim.active {
            return Ok(ValidateCallbackResult::Invalid(
                "Every source claim must be active".to_string(),
            ));
        }
        total_confidence += claim.confidence_bps;
    }

    let vouch_count = status.vouch_count;
    let expected_trust_score = if vouch_count > 0 {
        total_confidence / vouch_count
    } else {
        0
    };
    if status.trust_score != expected_trust_score {
        return Ok(ValidateCallbackResult::Invalid(
            "trust_score does not match the expected average of source claims".to_string(),
        ));
    }

    let expected_tier = if vouch_count >= 10 && expected_trust_score >= 800 {
        VerificationTier::Trusted
    } else if vouch_count >= 3 {
        VerificationTier::CommunityVerified
    } else {
        VerificationTier::Unverified
    };
    if status.tier != expected_tier {
        return Ok(ValidateCallbackResult::Invalid(
            "tier does not match the expected tier for this vouch_count/trust_score".to_string(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

fn validate_cdn_reputation(
    rep: CdnNodeReputation,
    action: Create,
) -> ExternResult<ValidateCallbackResult> {
    // Node must match author (nodes register themselves)
    if rep.node != action.author {
        return Ok(ValidateCallbackResult::Invalid(
            "CDN node must match action author".to_string(),
        ));
    }

    // Validate Ethereum address format
    if !rep.eth_address.starts_with("0x") || rep.eth_address.len() != 42 {
        return Ok(ValidateCallbackResult::Invalid(
            "Invalid Ethereum address format".to_string(),
        ));
    }

    // PoGQ score must be finite
    if !rep.pogq_score.is_finite() {
        return Ok(ValidateCallbackResult::Invalid(
            "PoGQ score must be a finite number".to_string(),
        ));
    }

    // New registrations must start with zero traffic history and no report reference --
    // otherwise a modified coordinator could bypass validate_update_cdn_reputation's
    // aggregation checks entirely by fabricating a fresh registration with pre-loaded
    // stats (mirrors the listener_starts_at_zero pattern already used in this cluster's
    // balances zome).
    if rep.bytes_served != 0
        || rep.successful_requests != 0
        || rep.failed_requests != 0
        || rep.avg_latency_ms != 0
        || rep.last_report_hash.is_some()
    {
        return Ok(ValidateCallbackResult::Invalid(
            "New CDN node registrations must start with zero traffic history".to_string(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Re-derives update_cdn_reputation's exact aggregation formula against the one real
/// ServiceQualityReport named by last_report_hash. Authorization is deliberately open --
/// any listener may legitimately report quality about any CDN node, so the check here is
/// purely about arithmetic honesty, not who may call it.
///
/// Disclosed residual limitation: this proves every counted report is real and correctly
/// folded in with the right arithmetic -- it does NOT prove completeness, and doesn't
/// defend against a node's own agent (or a colluding agent) submitting self-serving
/// positive reports about itself, which is out of scope for this pass. See
/// memory/mycelix_attribution_author_binding_jul8.md.
fn validate_update_cdn_reputation(
    rep: CdnNodeReputation,
    original_action_hash: ActionHash,
) -> ExternResult<ValidateCallbackResult> {
    let original_record = must_get_valid_record(original_action_hash)?;
    let Some(original): Option<CdnNodeReputation> = original_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(e))?
    else {
        return Ok(ValidateCallbackResult::Invalid(
            "Invalid original CdnNodeReputation entry".to_string(),
        ));
    };

    if rep.node != original.node
        || rep.eth_address != original.eth_address
        || rep.ipfs_peer_id != original.ipfs_peer_id
        || rep.region != original.region
        || rep.bytes_served != original.bytes_served
        || rep.stake_amount != original.stake_amount
        || rep.slash_count != original.slash_count
    {
        return Ok(ValidateCallbackResult::Invalid(
            "CDN reputation updates may only change traffic stats derived from a new \
             service quality report -- identity/registration fields must be unchanged"
                .to_string(),
        ));
    }

    let Some(report_hash) = rep.last_report_hash.clone() else {
        return Ok(ValidateCallbackResult::Invalid(
            "CDN reputation updates must reference the report that caused them".to_string(),
        ));
    };
    if Some(&report_hash) == original.last_report_hash.as_ref() {
        return Ok(ValidateCallbackResult::Invalid(
            "CDN reputation update must reference a new report, not the previous one".to_string(),
        ));
    }

    let report_record = must_get_valid_record(report_hash)?;
    let Some(report): Option<ServiceQualityReport> = report_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(e))?
    else {
        return Ok(ValidateCallbackResult::Invalid(
            "last_report_hash must reference a valid ServiceQualityReport entry".to_string(),
        ));
    };
    if report.node != rep.node {
        return Ok(ValidateCallbackResult::Invalid(
            "Referenced report must be about this CDN node".to_string(),
        ));
    }

    // Re-derive update_cdn_reputation's exact formula.
    let (expected_successful, expected_failed, expected_avg_latency) = if report.success {
        let successful = original.successful_requests + 1;
        let total = successful + original.failed_requests;
        let avg_latency = ((original.avg_latency_ms as u64 * (total - 1)
            + report.latency_ms as u64)
            / total) as u32;
        (successful, original.failed_requests, avg_latency)
    } else {
        (
            original.successful_requests,
            original.failed_requests + 1,
            original.avg_latency_ms,
        )
    };
    if rep.successful_requests != expected_successful
        || rep.failed_requests != expected_failed
        || rep.avg_latency_ms != expected_avg_latency
    {
        return Ok(ValidateCallbackResult::Invalid(
            "CDN reputation traffic stats do not match the expected update from the \
             referenced report"
                .to_string(),
        ));
    }

    let total = rep.successful_requests + rep.failed_requests;
    let expected_uptime_bps = ((rep.successful_requests as f64 / total as f64) * 1000.0) as u32;
    if rep.uptime_bps != expected_uptime_bps {
        return Ok(ValidateCallbackResult::Invalid(
            "uptime_bps does not match the expected value".to_string(),
        ));
    }

    let uptime_factor = rep.uptime_bps as f64 / 1000.0;
    let latency_factor = if rep.avg_latency_ms < 100 {
        1.0
    } else if rep.avg_latency_ms < 500 {
        0.8
    } else {
        0.5
    };
    let expected_pogq_score = uptime_factor * latency_factor;
    if rep.pogq_score != expected_pogq_score {
        return Ok(ValidateCallbackResult::Invalid(
            "pogq_score does not match the expected value".to_string(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

fn validate_quality_report(
    report: ServiceQualityReport,
    action: Create,
) -> ExternResult<ValidateCallbackResult> {
    // Reporter must match author
    if report.reporter != action.author {
        return Ok(ValidateCallbackResult::Invalid(
            "Reporter must match action author".to_string(),
        ));
    }

    // Cannot report self
    if report.reporter == report.node {
        return Ok(ValidateCallbackResult::Invalid(
            "Cannot report on self".to_string(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

fn validate_byzantine_report(
    report: ByzantineReport,
    action: Create,
) -> ExternResult<ValidateCallbackResult> {
    // Reporter must match author
    if report.reporter != action.author {
        return Ok(ValidateCallbackResult::Invalid(
            "Reporter must match action author".to_string(),
        ));
    }

    // Cannot report self
    if report.reporter == report.accused {
        return Ok(ValidateCallbackResult::Invalid(
            "Cannot report self".to_string(),
        ));
    }

    // Must have evidence, bounded
    if report.evidence.is_empty() || report.evidence.len() > 8192 {
        return Ok(ValidateCallbackResult::Invalid(
            "Byzantine report evidence must be 1-8192 chars".to_string(),
        ));
    }

    // Severity must be valid
    if report.severity > 100 {
        return Ok(ValidateCallbackResult::Invalid(
            "Severity must be 0-100".to_string(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Proves `validate_verification_status`'s P0 author-binding fix: a forged trust_score
/// that doesn't match the real average of the cited source_claims is rejected (previously
/// VerificationStatus had zero validation of any kind). Mocks the HDI host's
/// `must_get_valid_record` so this runs as a plain `cargo test`, no live conductor needed.
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

    fn test_claim(to: AgentPubKey, confidence_bps: u32, active: bool) -> TrustClaim {
        TrustClaim {
            from: AgentPubKey::from_raw_36(vec![50; 36]),
            to,
            claim_type: TrustClaimType::GeneralEndorsement,
            confidence_bps,
            evidence: None,
            created_at: Timestamp::from_micros(0),
            expires_at: None,
            active,
        }
    }

    #[test]
    fn verification_status_with_forged_trust_score_is_rejected() {
        let artist = AgentPubKey::from_raw_36(vec![1; 36]);
        let hash = ActionHash::from_raw_36(vec![10; 36]);
        hdi::hdi::set_hdi(MockRecordHdi {
            records: std::collections::HashMap::from([(
                hash.clone(),
                wrap_entry_record(
                    AgentPubKey::from_raw_36(vec![50; 36]),
                    test_claim(artist.clone(), 500, true),
                ),
            )]),
        });

        let status = VerificationStatus {
            artist,
            trust_score: 999, // forged -- real average is 500
            tier: VerificationTier::Unverified,
            vouch_count: 1,
            computed_at: Timestamp::from_micros(0),
            source_claims: vec![hash],
        };

        let result = validate_verification_status(status).unwrap();
        assert!(
            matches!(result, ValidateCallbackResult::Invalid(_)),
            "a forged trust_score not matching the real average of source_claims must be \
             rejected -- previously VerificationStatus had zero validation of any kind"
        );
    }

    #[test]
    fn verification_status_with_correct_trust_score_is_accepted() {
        let artist = AgentPubKey::from_raw_36(vec![1; 36]);
        let hash = ActionHash::from_raw_36(vec![10; 36]);
        hdi::hdi::set_hdi(MockRecordHdi {
            records: std::collections::HashMap::from([(
                hash.clone(),
                wrap_entry_record(
                    AgentPubKey::from_raw_36(vec![50; 36]),
                    test_claim(artist.clone(), 500, true),
                ),
            )]),
        });

        let status = VerificationStatus {
            artist,
            trust_score: 500,
            tier: VerificationTier::Unverified,
            vouch_count: 1,
            computed_at: Timestamp::from_micros(0),
            source_claims: vec![hash],
        };

        let result = validate_verification_status(status).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Valid));
    }

    #[test]
    fn verification_status_citing_a_revoked_claim_is_rejected() {
        let artist = AgentPubKey::from_raw_36(vec![1; 36]);
        let hash = ActionHash::from_raw_36(vec![10; 36]);
        hdi::hdi::set_hdi(MockRecordHdi {
            records: std::collections::HashMap::from([(
                hash.clone(),
                wrap_entry_record(
                    AgentPubKey::from_raw_36(vec![50; 36]),
                    test_claim(artist.clone(), 500, false),
                ),
            )]),
        });

        let status = VerificationStatus {
            artist,
            trust_score: 500,
            tier: VerificationTier::Unverified,
            vouch_count: 1,
            computed_at: Timestamp::from_micros(0),
            source_claims: vec![hash],
        };

        let result = validate_verification_status(status).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }
}
