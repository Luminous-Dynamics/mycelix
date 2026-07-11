// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root//! Property Disputes Integrity Zome
use hdi::prelude::*;

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct PropertyDispute {
    pub id: String,
    pub property_id: String,
    pub dispute_type: DisputeType,
    pub claimant_did: String,
    pub respondent_did: String,
    pub description: String,
    pub evidence_ids: Vec<String>,
    pub status: DisputeStatus,
    pub justice_case_id: Option<String>,
    pub filed: Timestamp,
    pub resolved: Option<Timestamp>,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum DisputeType {
    Boundary,
    Ownership,
    Encumbrance,
    Easement,
    Trespass,
    Damage,
    Other(String),
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum DisputeStatus {
    Filed,
    UnderReview,
    Mediation,
    Arbitration,
    Resolved,
    Dismissed,
}

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct OwnershipClaim {
    pub id: String,
    pub property_id: String,
    pub claimant_did: String,
    pub claim_basis: ClaimBasis,
    pub supporting_documents: Vec<String>,
    pub status: ClaimStatus,
    pub filed: Timestamp,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum ClaimBasis {
    PriorOwnership,
    Inheritance,
    AdversePossession,
    FraudulentTransfer,
    DocumentaryEvidence,
    Other(String),
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum ClaimStatus {
    Pending,
    UnderInvestigation,
    Validated,
    Rejected,
    Superseded,
}

/// Anchor entry for deterministic link bases from strings
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Anchor(pub String);

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    PropertyDispute(PropertyDispute),
    OwnershipClaim(OwnershipClaim),
    #[entry_type(visibility = "public")]
    Anchor(Anchor),
}

#[hdk_link_types]
pub enum LinkTypes {
    PropertyToDisputes,
    ClaimantToDisputes,
    PropertyToClaims,
    DisputeToJustice,
}

/// Genesis self-check
#[hdk_extern]
pub fn genesis_self_check(_data: GenesisSelfCheckData) -> ExternResult<ValidateCallbackResult> {
    Ok(ValidateCallbackResult::Valid)
}

/// Main validation callback using FlatOp pattern
#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(store_entry) => match store_entry {
            OpEntry::CreateEntry { app_entry, action } => match app_entry {
                EntryTypes::PropertyDispute(dispute) => {
                    validate_create_property_dispute(EntryCreationAction::Create(action), dispute)
                }
                EntryTypes::OwnershipClaim(claim) => {
                    validate_create_ownership_claim(EntryCreationAction::Create(action), claim)
                }
                EntryTypes::Anchor(_) => Ok(ValidateCallbackResult::Valid),
            },
            OpEntry::UpdateEntry {
                app_entry, action, ..
            } => match app_entry {
                EntryTypes::PropertyDispute(dispute) => {
                    validate_update_property_dispute(action, dispute)
                }
                EntryTypes::OwnershipClaim(claim) => validate_update_ownership_claim(action, claim),
                EntryTypes::Anchor(_) => Ok(ValidateCallbackResult::Invalid(
                    "Anchors cannot be updated".into(),
                )),
            },
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterCreateLink { link_type, .. } => match link_type {
            LinkTypes::PropertyToDisputes => Ok(ValidateCallbackResult::Valid),
            LinkTypes::ClaimantToDisputes => Ok(ValidateCallbackResult::Valid),
            LinkTypes::PropertyToClaims => Ok(ValidateCallbackResult::Valid),
            LinkTypes::DisputeToJustice => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterDeleteLink { .. } => Ok(ValidateCallbackResult::Valid),
        FlatOp::StoreRecord(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterAgentActivity(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterUpdate(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterDelete(_) => Ok(ValidateCallbackResult::Valid),
    }
}

fn validate_create_property_dispute(
    action: EntryCreationAction,
    dispute: PropertyDispute,
) -> ExternResult<ValidateCallbackResult> {
    // Bind the dispute to its filer -- file_dispute already derives
    // `claimant_did` from agent_info() coordinator-side with zero user
    // input, so this never rejects a legitimate filing; it's the real
    // DHT-level enforcement a modified coordinator could otherwise bypass
    // (P0 author-binding gap). respondent_did is legitimately named by the
    // filer (the party being disputed), so it is NOT bound to the author.
    let expected_claimant = format!("did:mycelix:{}", action.author());
    if dispute.claimant_did != expected_claimant {
        return Ok(ValidateCallbackResult::Invalid(
            "Dispute claimant must be the committing agent (forgery)".to_string(),
        ));
    }

    if !dispute.claimant_did.starts_with("did:") {
        return Ok(ValidateCallbackResult::Invalid(
            "Claimant must be a valid DID".into(),
        ));
    }
    if !dispute.respondent_did.starts_with("did:") {
        return Ok(ValidateCallbackResult::Invalid(
            "Respondent must be a valid DID".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_update_property_dispute(
    _action: Update,
    _dispute: PropertyDispute,
) -> ExternResult<ValidateCallbackResult> {
    // Deliberately NOT author-bound: dispute resolution (escalate_to_justice,
    // resolve_dispute, update_dispute_status) has no claimant/respondent
    // restriction in the coordinator today -- these are meant to be actioned
    // by an arbiter/justice-system role that isn't modeled as either party.
    // add_dispute_evidence's own claimant-or-respondent check is enforced
    // coordinator-side (submitter_did is now agent-derived there); binding
    // this callback to that same OR-check would incorrectly reject the
    // (currently unrestricted) resolution paths. Same class of gap as
    // mycelix-justice's Arbitration/Judgment, which needs real
    // juror-authorization rather than simple author-binding -- skip unless
    // doing a dedicated authorization-model pass across both.
    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_ownership_claim(
    action: EntryCreationAction,
    claim: OwnershipClaim,
) -> ExternResult<ValidateCallbackResult> {
    // Bind the claim to its filer -- same rationale as
    // validate_create_property_dispute above.
    let expected_claimant = format!("did:mycelix:{}", action.author());
    if claim.claimant_did != expected_claimant {
        return Ok(ValidateCallbackResult::Invalid(
            "Ownership claim claimant must be the committing agent (forgery)".to_string(),
        ));
    }

    if !claim.claimant_did.starts_with("did:") {
        return Ok(ValidateCallbackResult::Invalid(
            "Claimant must be a valid DID".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_update_ownership_claim(
    _action: Update,
    _claim: OwnershipClaim,
) -> ExternResult<ValidateCallbackResult> {
    // Deliberately NOT author-bound -- same rationale as
    // validate_update_property_dispute above (update_claim_status has no
    // restriction today; add_claim_document's claimant check is enforced
    // coordinator-side).
    Ok(ValidateCallbackResult::Valid)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn test_action() -> Create {
        Create {
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
        }
    }

    fn test_author_did() -> String {
        format!("did:mycelix:{}", test_action().author)
    }

    fn valid_dispute(claimant_did: String) -> PropertyDispute {
        PropertyDispute {
            id: "dispute:test:0".to_string(),
            property_id: "property:test:0".to_string(),
            dispute_type: DisputeType::Boundary,
            claimant_did,
            respondent_did: "did:mycelix:respondent".to_string(),
            description: "Test dispute".to_string(),
            evidence_ids: vec![],
            status: DisputeStatus::Filed,
            justice_case_id: None,
            filed: Timestamp::from_micros(0),
            resolved: None,
        }
    }

    #[test]
    fn test_create_dispute_valid() {
        let dispute = valid_dispute(test_author_did());
        let result =
            validate_create_property_dispute(EntryCreationAction::Create(test_action()), dispute)
                .unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn test_create_dispute_claimant_forgery_rejected() {
        // claimant_did claims test_author_did(), but the committing action's
        // author is a different agent.
        let mut forged_action = test_action();
        forged_action.author = AgentPubKey::from_raw_36(vec![1u8; 36]);
        let dispute = valid_dispute(test_author_did());
        let result =
            validate_create_property_dispute(EntryCreationAction::Create(forged_action), dispute)
                .unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    fn valid_claim(claimant_did: String) -> OwnershipClaim {
        OwnershipClaim {
            id: "claim:test:0".to_string(),
            property_id: "property:test:0".to_string(),
            claimant_did,
            claim_basis: ClaimBasis::PriorOwnership,
            supporting_documents: vec![],
            status: ClaimStatus::Pending,
            filed: Timestamp::from_micros(0),
        }
    }

    #[test]
    fn test_create_claim_valid() {
        let claim = valid_claim(test_author_did());
        let result =
            validate_create_ownership_claim(EntryCreationAction::Create(test_action()), claim)
                .unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn test_create_claim_claimant_forgery_rejected() {
        let mut forged_action = test_action();
        forged_action.author = AgentPubKey::from_raw_36(vec![1u8; 36]);
        let claim = valid_claim(test_author_did());
        let result =
            validate_create_ownership_claim(EntryCreationAction::Create(forged_action), claim)
                .unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }
}
