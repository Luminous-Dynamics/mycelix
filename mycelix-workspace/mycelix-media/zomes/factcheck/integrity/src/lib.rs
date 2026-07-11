// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root//! Fact-Check Integrity Zome
//! Updated to use HDI 0.7 patterns with FlatOp validation
use hdi::prelude::*;

/// Anchor entry for deterministic link bases
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Anchor(pub String);

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct FactCheck {
    pub id: String,
    pub publication_id: String,
    pub claim_text: String,
    pub claim_location: String,
    pub epistemic_position: EpistemicPosition,
    pub verdict: FactCheckVerdict,
    pub evidence: Vec<EvidenceItem>,
    pub checker_did: String,
    pub checked: Timestamp,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct EpistemicPosition {
    pub empirical: f64, // 0.0 to 1.0
    pub normative: f64, // 0.0 to 1.0
    pub mythic: f64,    // 0.0 to 1.0
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum FactCheckVerdict {
    True,
    MostlyTrue,
    HalfTrue,
    MostlyFalse,
    False,
    Unverifiable,
    OutOfContext,
    Satire,
    Opinion,
    PartiallyTrue,
    Misleading,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct EvidenceItem {
    pub source_type: SourceType,
    pub source_url: Option<String>,
    pub source_did: Option<String>,
    pub description: String,
    pub supports_claim: bool,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum SourceType {
    PrimarySource,
    SecondarySource,
    ExpertOpinion,
    OfficialDocument,
    ScientificStudy,
    EyewitnessAccount,
    DataAnalysis,
    Other(String),
}

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct SourceCredibility {
    pub source_id: String,
    pub source_type: SourceType,
    pub credibility_score: f64,
    pub verification_count: u32,
    pub dispute_count: u32,
    pub last_assessed: Timestamp,
}

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct FactCheckDispute {
    pub id: String,
    pub fact_check_id: String,
    pub disputer_did: String,
    pub reason: String,
    pub counter_evidence: Vec<EvidenceItem>,
    pub status: DisputeStatus,
    pub created: Timestamp,
    pub resolved: Option<Timestamp>,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum DisputeStatus {
    Pending,
    Upheld,
    Rejected,
    PartiallyUpheld,
}

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    #[entry_type(visibility = "public")]
    Anchor(Anchor),
    FactCheck(FactCheck),
    SourceCredibility(SourceCredibility),
    FactCheckDispute(FactCheckDispute),
}

#[hdk_link_types]
pub enum LinkTypes {
    PublicationToFactChecks,
    CheckerToFactChecks,
    ClaimToFactCheck,
    FactCheckToDisputes,
}

/// Genesis self-check
#[hdk_extern]
pub fn genesis_self_check(_data: GenesisSelfCheckData) -> ExternResult<ValidateCallbackResult> {
    Ok(ValidateCallbackResult::Valid)
}

/// Main validation callback using FlatOp pattern
///
/// **P0 author-binding pass, 2026-07-09.** Two findings, one a pre-existing
/// functional bug this pass also fixes, one disclosed-not-fixed:
///
/// 1. **Pre-existing correctness bug**: this validator rejected ALL
///    FactCheck updates outright ("Fact checks cannot be updated"), but
///    the coordinator's `add_evidence`/`update_verdict` both call
///    `update_entry` on FactCheck -- meaning those two coordinator
///    functions were completely non-functional, always failing DHT
///    validation. Fixed by allowing exactly the fields those two flows
///    change (evidence, verdict), content-restricted via must_get.
/// 2. **Disclosed, not fixed**: every identity here (`checker_did`,
///    `disputer_did`) is a free-form String with no local DID-to-agent
///    verification convention. `add_evidence`/`update_verdict` compare
///    `check.checker_did` against a caller-supplied `requester_did` --
///    both attacker-controlled, authenticating nothing (same bug class as
///    this cluster's attribution/curation zomes). `resolve_dispute` has
///    NO authorization check of any kind (case c, no authority model
///    exists to bind against -- anyone can resolve anyone's dispute).
///    Neither is fixable at the integrity layer without inventing an
///    unverified convention or a larger authority-model feature.
///
/// Also fixed: the wide-open RegisterUpdate/RegisterDelete bug (37th
/// confirmed instance this pass). SourceCredibility has no live update
/// path (confirmed via grep -- update_source_credibility always creates a
/// fresh record) and is now made immutable, replacing a dead-code update
/// validator.
#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(store_entry) => match store_entry {
            OpEntry::CreateEntry { app_entry, action } => match app_entry {
                EntryTypes::Anchor(_) => Ok(ValidateCallbackResult::Valid),
                EntryTypes::FactCheck(check) => {
                    validate_create_fact_check(EntryCreationAction::Create(action), check)
                }
                EntryTypes::SourceCredibility(source) => {
                    validate_create_source_credibility(EntryCreationAction::Create(action), source)
                }
                EntryTypes::FactCheckDispute(dispute) => {
                    validate_create_fact_check_dispute(EntryCreationAction::Create(action), dispute)
                }
            },
            OpEntry::UpdateEntry {
                app_entry,
                original_action_hash,
                action,
                ..
            } => match app_entry {
                EntryTypes::Anchor(_) => Ok(ValidateCallbackResult::Valid),
                EntryTypes::FactCheck(check) => {
                    validate_update_fact_check(action, original_action_hash, check)
                }
                EntryTypes::SourceCredibility(_) => Ok(ValidateCallbackResult::Invalid(
                    "Source credibility records are immutable".into(),
                )),
                EntryTypes::FactCheckDispute(dispute) => {
                    validate_update_fact_check_dispute(action, original_action_hash, dispute)
                }
            },
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterCreateLink { link_type, .. } => match link_type {
            LinkTypes::PublicationToFactChecks => Ok(ValidateCallbackResult::Valid),
            LinkTypes::CheckerToFactChecks => Ok(ValidateCallbackResult::Valid),
            LinkTypes::ClaimToFactCheck => Ok(ValidateCallbackResult::Valid),
            LinkTypes::FactCheckToDisputes => Ok(ValidateCallbackResult::Valid),
        },
        // Deliberately left permissive: the coordinator never calls
        // delete_link. Reviewed 2026-07-09 during the P0 author-binding pass.
        FlatOp::RegisterDeleteLink { .. } => Ok(ValidateCallbackResult::Valid),
        FlatOp::StoreRecord(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterAgentActivity(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterUpdate(op_update) => match op_update {
            // Previously fully permissive (`Ok(Valid)` unconditionally) --
            // the 37th confirmed instance of this exact bug pattern this
            // pass. Found + fixed 2026-07-09.
            OpUpdate::Entry { app_entry, action } => match app_entry {
                EntryTypes::Anchor(_) => Ok(ValidateCallbackResult::Valid),
                EntryTypes::FactCheck(check) => validate_update_fact_check(
                    action.clone(),
                    action.original_action_address,
                    check,
                ),
                EntryTypes::SourceCredibility(_) => Ok(ValidateCallbackResult::Invalid(
                    "Source credibility records are immutable".into(),
                )),
                EntryTypes::FactCheckDispute(dispute) => validate_update_fact_check_dispute(
                    action.clone(),
                    action.original_action_address,
                    dispute,
                ),
            },
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterDelete(OpDelete { action }) => {
            let _ = must_get_action(action.deletes_address.clone())?;
            // No author field exists anywhere in this zome's entries to
            // compare against (case d/c, see the module doc comment
            // above) -- deletion authorization is left as-is.
            Ok(ValidateCallbackResult::Valid)
        }
    }
}

fn validate_create_fact_check(
    _action: EntryCreationAction,
    check: FactCheck,
) -> ExternResult<ValidateCallbackResult> {
    if !check.checker_did.starts_with("did:") {
        return Ok(ValidateCallbackResult::Invalid(
            "Checker must be a valid DID".into(),
        ));
    }
    if check.claim_text.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Claim text required".into(),
        ));
    }
    let ep = &check.epistemic_position;
    if ep.empirical < 0.0
        || ep.empirical > 1.0
        || ep.normative < 0.0
        || ep.normative > 1.0
        || ep.mythic < 0.0
        || ep.mythic > 1.0
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Epistemic values must be 0-1".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_source_credibility(
    _action: EntryCreationAction,
    source: SourceCredibility,
) -> ExternResult<ValidateCallbackResult> {
    if source.credibility_score < 0.0 || source.credibility_score > 1.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Credibility must be 0-1".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

/// Fixes a pre-existing correctness bug (see module doc comment on
/// `validate`): content-restricted to evidence/verdict, the only two
/// fields add_evidence/update_verdict ever change.
fn validate_update_fact_check(
    _action: Update,
    original_action_hash: ActionHash,
    check: FactCheck,
) -> ExternResult<ValidateCallbackResult> {
    if check.claim_text.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Claim text required".into(),
        ));
    }

    let original_record = must_get_valid_record(original_action_hash)?;
    let original: FactCheck = original_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Original fact check not found".into()
        )))?;

    if check.id != original.id
        || check.publication_id != original.publication_id
        || check.claim_text != original.claim_text
        || check.claim_location != original.claim_location
        || check.epistemic_position != original.epistemic_position
        || check.checker_did != original.checker_did
        || check.checked != original.checked
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Only evidence/verdict can change on a fact check update".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_fact_check_dispute(
    _action: EntryCreationAction,
    dispute: FactCheckDispute,
) -> ExternResult<ValidateCallbackResult> {
    if !dispute.disputer_did.starts_with("did:") {
        return Ok(ValidateCallbackResult::Invalid(
            "Disputer must be a valid DID".into(),
        ));
    }
    if dispute.reason.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Dispute reason required".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

/// resolve_dispute has NO authorization check of any kind at the
/// coordinator level (case c, no authority model exists to bind against
/// -- see module doc comment on `validate` above). Content restricted to
/// status/resolved, the only two fields resolve_dispute ever changes.
fn validate_update_fact_check_dispute(
    _action: Update,
    original_action_hash: ActionHash,
    dispute: FactCheckDispute,
) -> ExternResult<ValidateCallbackResult> {
    let original_record = must_get_valid_record(original_action_hash)?;
    let original: FactCheckDispute = original_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Original dispute not found".into()
        )))?;

    if dispute.id != original.id
        || dispute.fact_check_id != original.fact_check_id
        || dispute.disputer_did != original.disputer_did
        || dispute.reason != original.reason
        || dispute.counter_evidence != original.counter_evidence
        || dispute.created != original.created
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Only status/resolved can change on a dispute update".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

#[cfg(test)]
mod content_restriction_tests {
    use super::*;

    fn dummy_action() -> EntryCreationAction {
        EntryCreationAction::Create(Create {
            author: AgentPubKey::from_raw_36(vec![0u8; 36]),
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
        })
    }

    #[test]
    fn fact_check_requires_did_prefix() {
        let check = FactCheck {
            id: "fc-1".into(),
            publication_id: "pub-1".into(),
            claim_text: "The sky is blue".into(),
            claim_location: "para-1".into(),
            epistemic_position: EpistemicPosition {
                empirical: 0.9,
                normative: 0.1,
                mythic: 0.0,
            },
            verdict: FactCheckVerdict::True,
            evidence: vec![],
            checker_did: "not-a-did".into(),
            checked: Timestamp::from_micros(0),
        };
        let result = validate_create_fact_check(dummy_action(), check).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn fact_check_rejects_out_of_range_epistemic_position() {
        let check = FactCheck {
            id: "fc-1".into(),
            publication_id: "pub-1".into(),
            claim_text: "The sky is blue".into(),
            claim_location: "para-1".into(),
            epistemic_position: EpistemicPosition {
                empirical: 1.5,
                normative: 0.1,
                mythic: 0.0,
            },
            verdict: FactCheckVerdict::True,
            evidence: vec![],
            checker_did: "did:key:z6Mkfoo".into(),
            checked: Timestamp::from_micros(0),
        };
        let result = validate_create_fact_check(dummy_action(), check).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn source_credibility_rejects_out_of_range() {
        let source = SourceCredibility {
            source_id: "src-1".into(),
            source_type: SourceType::PrimarySource,
            credibility_score: -0.5,
            verification_count: 0,
            dispute_count: 0,
            last_assessed: Timestamp::from_micros(0),
        };
        let result = validate_create_source_credibility(dummy_action(), source).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn dispute_requires_did_prefix_and_reason() {
        let dispute = FactCheckDispute {
            id: "d-1".into(),
            fact_check_id: "fc-1".into(),
            disputer_did: "not-a-did".into(),
            reason: "".into(),
            counter_evidence: vec![],
            status: DisputeStatus::Pending,
            created: Timestamp::from_micros(0),
            resolved: None,
        };
        let result = validate_create_fact_check_dispute(dummy_action(), dispute).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    // validate_update_{fact_check,fact_check_dispute} both call
    // must_get_valid_record, which requires a live HDI host and can't run
    // in a plain unit test -- matching the established pattern from every
    // other zome's update validator this pass.
}
