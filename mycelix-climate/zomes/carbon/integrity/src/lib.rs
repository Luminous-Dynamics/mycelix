// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Carbon Integrity Zome
//!
//! Defines entry types and validation for carbon footprint tracking and carbon credits.
//! Uses HDI 0.7.0-dev.1 with FlatOp validation pattern.

use hdi::prelude::*;
use mycelix_bridge_entry_types::{check_link_author_match, did_for_author};

/// Anchor entry for creating deterministic link bases
#[hdk_entry_helper]
#[derive(Clone, PartialEq, Eq)]
pub struct Anchor(pub String);

/// Status of a carbon credit
#[hdk_entry_helper]
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum CreditStatus {
    /// Credit is active and can be transferred
    Active,
    /// Credit has been transferred to another owner
    Transferred,
    /// Credit has been retired (used for offsetting)
    Retired,
}

/// Carbon footprint measurement for an entity over a time period
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct CarbonFootprint {
    /// DID of the entity being measured
    pub entity_did: String,
    /// Start of measurement period (Unix timestamp)
    pub period_start: i64,
    /// End of measurement period (Unix timestamp)
    pub period_end: i64,
    /// Scope 1 emissions in tonnes CO2e (direct emissions)
    pub scope1: f64,
    /// Scope 2 emissions in tonnes CO2e (indirect from purchased energy)
    pub scope2: f64,
    /// Scope 3 emissions in tonnes CO2e (other indirect emissions)
    pub scope3: f64,
    /// Methodology used for measurement (e.g., "GHG Protocol", "ISO 14064")
    pub methodology: String,
    /// DID of verifier (if verified)
    pub verified_by: Option<String>,
}

/// A tradeable carbon credit representing verified emission reductions
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct CarbonCredit {
    /// Unique identifier for this credit
    pub id: String,
    /// ID of the climate project that generated this credit
    pub project_id: String,
    /// Year the emission reduction occurred
    pub vintage_year: u32,
    /// Amount of CO2 equivalent in tonnes
    pub tonnes_co2e: f64,
    /// Current status of the credit
    pub status: CreditStatus,
    /// DID of current owner
    pub owner_did: String,
    /// Timestamp when credit was retired (if retired)
    pub retired_at: Option<i64>,
}

/// Entry types for the carbon zome
#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    #[entry_type(visibility = "public")]
    Anchor(Anchor),
    #[entry_type(visibility = "public")]
    CarbonFootprint(CarbonFootprint),
    #[entry_type(visibility = "public")]
    CarbonCredit(CarbonCredit),
}

/// Link types for the carbon zome
#[hdk_link_types]
pub enum LinkTypes {
    /// Anchor to footprints for an entity
    AnchorToFootprints,
    /// Anchor to credits by owner
    AnchorToCredits,
    /// Anchor to credits by project
    ProjectToCredits,
    /// Footprint updates chain
    FootprintUpdates,
    /// Credit transfer history
    CreditTransfers,
}

/// Validate DIDs have proper format
fn validate_did(did: &str) -> ExternResult<ValidateCallbackResult> {
    if did.is_empty() {
        return Ok(ValidateCallbackResult::Invalid("DID cannot be empty".to_string()));
    }
    if !did.starts_with("did:") {
        return Ok(ValidateCallbackResult::Invalid(
            "DID must start with 'did:' prefix".to_string(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

/// Validate that emissions values are finite and non-negative.
fn validate_emissions(scope1: f64, scope2: f64, scope3: f64) -> ExternResult<ValidateCallbackResult> {
    for (label, value) in [("Scope 1", scope1), ("Scope 2", scope2), ("Scope 3", scope3)] {
        if !value.is_finite() {
            return Ok(ValidateCallbackResult::Invalid(format!(
                "{label} emissions must be finite"
            )));
        }
        if value < 0.0 {
            return Ok(ValidateCallbackResult::Invalid(format!(
                "{label} emissions cannot be negative"
            )));
        }
    }
    Ok(ValidateCallbackResult::Valid)
}

/// Validate a CarbonFootprint entry
fn validate_carbon_footprint(footprint: &CarbonFootprint) -> ExternResult<ValidateCallbackResult> {
    // Validate entity DID
    let did_result = validate_did(&footprint.entity_did)?;
    if let ValidateCallbackResult::Invalid(_) = did_result {
        return Ok(did_result);
    }

    // Validate verifier DID if present
    if let Some(ref verifier) = footprint.verified_by {
        let verifier_result = validate_did(verifier)?;
        if let ValidateCallbackResult::Invalid(_) = verifier_result {
            return Ok(verifier_result);
        }
    }

    // Validate emissions are finite and non-negative
    let emissions_result = validate_emissions(footprint.scope1, footprint.scope2, footprint.scope3)?;
    if let ValidateCallbackResult::Invalid(_) = emissions_result {
        return Ok(emissions_result);
    }

    // Validate time period
    if footprint.period_start >= footprint.period_end {
        return Ok(ValidateCallbackResult::Invalid(
            "Period start must be before period end".to_string(),
        ));
    }

    // Validate methodology
    if footprint.methodology.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Methodology cannot be empty".to_string(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Validate a CarbonCredit entry
fn validate_carbon_credit(credit: &CarbonCredit) -> ExternResult<ValidateCallbackResult> {
    // Validate credit ID
    if credit.id.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Credit ID cannot be empty".to_string(),
        ));
    }

    // Validate project ID
    if credit.project_id.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Project ID cannot be empty".to_string(),
        ));
    }

    // Validate owner DID
    let did_result = validate_did(&credit.owner_did)?;
    if let ValidateCallbackResult::Invalid(_) = did_result {
        return Ok(did_result);
    }

    // Validate tonnes are finite and positive
    if !credit.tonnes_co2e.is_finite() {
        return Ok(ValidateCallbackResult::Invalid(
            "Credit tonnes must be finite".to_string(),
        ));
    }
    if credit.tonnes_co2e <= 0.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Credit tonnes must be positive".to_string(),
        ));
    }

    // Validate vintage year is reasonable (1990-2100)
    if credit.vintage_year < 1990 || credit.vintage_year > 2100 {
        return Ok(ValidateCallbackResult::Invalid(
            "Vintage year must be between 1990 and 2100".to_string(),
        ));
    }

    // Validate retired_at matches status
    match credit.status {
        CreditStatus::Retired => {
            if credit.retired_at.is_none() {
                return Ok(ValidateCallbackResult::Invalid(
                    "Retired credits must have retired_at timestamp".to_string(),
                ));
            }
        }
        _ => {
            if credit.retired_at.is_some() {
                return Ok(ValidateCallbackResult::Invalid(
                    "Non-retired credits cannot have retired_at timestamp".to_string(),
                ));
            }
        }
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Pure transition policy for footprint verification.
///
/// The measurement itself is immutable. The only legal update is a single
/// transition from `verified_by = None` to the canonical DID of the agent that
/// commits the verification update. Verifier *authority* is a separate concern
/// and will be bound to evidence/credentials in a follow-up PR.
fn validate_footprint_transition(
    original: &CarbonFootprint,
    updated: &CarbonFootprint,
    author: &AgentPubKey,
) -> ValidateCallbackResult {
    if original.entity_did != updated.entity_did
        || original.period_start != updated.period_start
        || original.period_end != updated.period_end
        || original.scope1 != updated.scope1
        || original.scope2 != updated.scope2
        || original.scope3 != updated.scope3
        || original.methodology != updated.methodology
    {
        return ValidateCallbackResult::Invalid(
            "Carbon footprint measurements are immutable during verification".into(),
        );
    }

    if original.verified_by.is_some() {
        return ValidateCallbackResult::Invalid(
            "A verified carbon footprint cannot be re-verified or reassigned".into(),
        );
    }

    let expected_verifier = did_for_author(author);
    match updated.verified_by.as_deref() {
        Some(verifier) if verifier == expected_verifier => ValidateCallbackResult::Valid,
        Some(_) => ValidateCallbackResult::Invalid(
            "Carbon footprint verifier must be the committing agent".into(),
        ),
        None => ValidateCallbackResult::Invalid(
            "Carbon footprint update must add a verifier".into(),
        ),
    }
}

fn validate_update_footprint(
    action: Update,
    updated: CarbonFootprint,
    original_action_hash: ActionHash,
) -> ExternResult<ValidateCallbackResult> {
    let fields = validate_carbon_footprint(&updated)?;
    if let ValidateCallbackResult::Invalid(_) = fields {
        return Ok(fields);
    }

    let original_record = must_get_valid_record(original_action_hash)?;
    let original: CarbonFootprint = original_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(e))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Original CarbonFootprint entry not found".to_string()
        )))?;

    Ok(validate_footprint_transition(
        &original,
        &updated,
        &action.author,
    ))
}

/// Pure transition policy for carbon-credit ownership and retirement.
///
/// Credit identity and quantity are immutable after issuance. The current owner
/// is derived from the predecessor entry and is the only principal permitted to
/// transfer or retire the credit. Transfers keep the successor active; retirement
/// is terminal and preserves ownership.
fn validate_credit_transition(
    original: &CarbonCredit,
    updated: &CarbonCredit,
    author: &AgentPubKey,
) -> ValidateCallbackResult {
    if original.id != updated.id
        || original.project_id != updated.project_id
        || original.vintage_year != updated.vintage_year
        || original.tonnes_co2e != updated.tonnes_co2e
    {
        return ValidateCallbackResult::Invalid(
            "Carbon credit identity, project, vintage, and tonnes are immutable".into(),
        );
    }

    if original.status != CreditStatus::Active || original.retired_at.is_some() {
        return ValidateCallbackResult::Invalid(
            "Only an active, unretired carbon credit can transition".into(),
        );
    }

    let author_did = did_for_author(author);
    if author_did != original.owner_did {
        return ValidateCallbackResult::Invalid(
            "Only the current carbon credit owner can transfer or retire the credit".into(),
        );
    }

    let owner_changed = updated.owner_did != original.owner_did;
    match (owner_changed, updated.status, updated.retired_at) {
        // Whole-credit ownership transfer. The successor is immediately active
        // under the new owner; transfer history is preserved by CreditTransfers.
        (true, CreditStatus::Active, None) => ValidateCallbackResult::Valid,
        // Terminal retirement by the current owner.
        (false, CreditStatus::Retired, Some(retired_at)) if retired_at > 0 => {
            ValidateCallbackResult::Valid
        }
        (false, CreditStatus::Retired, Some(_)) => ValidateCallbackResult::Invalid(
            "Carbon credit retirement timestamp must be positive".into(),
        ),
        _ => ValidateCallbackResult::Invalid(
            "Carbon credit update must be either an owner transfer or terminal retirement".into(),
        ),
    }
}

fn validate_update_credit(
    action: Update,
    updated: CarbonCredit,
    original_action_hash: ActionHash,
) -> ExternResult<ValidateCallbackResult> {
    let fields = validate_carbon_credit(&updated)?;
    if let ValidateCallbackResult::Invalid(_) = fields {
        return Ok(fields);
    }

    let original_record = must_get_valid_record(original_action_hash)?;
    let original: CarbonCredit = original_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(e))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Original CarbonCredit entry not found".to_string()
        )))?;

    Ok(validate_credit_transition(
        &original,
        &updated,
        &action.author,
    ))
}

/// Main validation callback using FlatOp pattern
#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(store_entry) => match store_entry {
            OpEntry::CreateEntry { app_entry, .. } => match app_entry {
                EntryTypes::Anchor(_) => Ok(ValidateCallbackResult::Valid),
                EntryTypes::CarbonFootprint(footprint) => validate_carbon_footprint(&footprint),
                EntryTypes::CarbonCredit(credit) => validate_carbon_credit(&credit),
            },
            OpEntry::UpdateEntry {
                app_entry,
                action,
                original_action_hash,
                original_entry_hash: _,
            } => match app_entry {
                EntryTypes::Anchor(_) => Ok(ValidateCallbackResult::Invalid(
                    "Carbon anchors cannot be updated".into(),
                )),
                EntryTypes::CarbonFootprint(footprint) => {
                    validate_update_footprint(action, footprint, original_action_hash)
                }
                EntryTypes::CarbonCredit(credit) => {
                    validate_update_credit(action, credit, original_action_hash)
                }
            },
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterCreateLink { link_type, .. } => match link_type {
            LinkTypes::AnchorToFootprints
            | LinkTypes::AnchorToCredits
            | LinkTypes::ProjectToCredits
            | LinkTypes::FootprintUpdates
            | LinkTypes::CreditTransfers => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterDeleteLink { link_type, action, .. } => match link_type {
            LinkTypes::CreditTransfers | LinkTypes::FootprintUpdates => {
                Ok(ValidateCallbackResult::Invalid(
                    "Carbon audit-history links cannot be deleted".to_string(),
                ))
            }
            _ => {
                let original_action = must_get_action(action.link_add_address.clone())?;
                Ok(check_link_author_match(
                    original_action.action().author(),
                    &action.author,
                ))
            }
        },
        FlatOp::StoreRecord(_)
        | FlatOp::RegisterAgentActivity(_)
        | FlatOp::RegisterUpdate(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterDelete(_) => Ok(ValidateCallbackResult::Invalid(
            "Carbon footprints and credits are audit records and cannot be deleted".into(),
        )),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fake_agent(byte: u8) -> AgentPubKey {
        AgentPubKey::from_raw_36(vec![byte; 36])
    }

    fn valid_footprint() -> CarbonFootprint {
        CarbonFootprint {
            entity_did: "did:mycelix:org-1".into(),
            period_start: 1_704_067_200,
            period_end: 1_735_689_599,
            scope1: 10.0,
            scope2: 20.0,
            scope3: 30.0,
            methodology: "GHG Protocol".into(),
            verified_by: None,
        }
    }

    fn valid_credit() -> CarbonCredit {
        CarbonCredit {
            id: "credit:project-1:2026:1".into(),
            project_id: "project-1".into(),
            vintage_year: 2026,
            tonnes_co2e: 10.0,
            status: CreditStatus::Active,
            owner_did: "did:mycelix:owner-1".into(),
            retired_at: None,
        }
    }

    fn owned_credit(owner: &AgentPubKey) -> CarbonCredit {
        CarbonCredit {
            owner_did: did_for_author(owner),
            ..valid_credit()
        }
    }

    fn assert_valid(result: ExternResult<ValidateCallbackResult>) {
        assert!(matches!(
            result.expect("validator should execute"),
            ValidateCallbackResult::Valid
        ));
    }

    fn assert_invalid_contains(result: ExternResult<ValidateCallbackResult>, needle: &str) {
        match result.expect("validator should execute") {
            ValidateCallbackResult::Invalid(reason) => assert!(
                reason.contains(needle),
                "expected rejection containing {needle:?}, got {reason:?}"
            ),
            other => panic!("expected invalid result, got {other:?}"),
        }
    }

    fn assert_transition_invalid(result: ValidateCallbackResult, needle: &str) {
        match result {
            ValidateCallbackResult::Invalid(reason) => assert!(
                reason.contains(needle),
                "expected rejection containing {needle:?}, got {reason:?}"
            ),
            other => panic!("expected invalid result, got {other:?}"),
        }
    }

    #[test]
    fn production_footprint_validator_accepts_valid_measurement() {
        assert_valid(validate_carbon_footprint(&valid_footprint()));
    }

    #[test]
    fn production_footprint_validator_rejects_invalid_identity() {
        let mut footprint = valid_footprint();
        footprint.entity_did = "org-1".into();
        assert_invalid_contains(validate_carbon_footprint(&footprint), "did:");
    }

    #[test]
    fn production_footprint_validator_rejects_negative_emissions() {
        let mut footprint = valid_footprint();
        footprint.scope2 = -0.01;
        assert_invalid_contains(validate_carbon_footprint(&footprint), "Scope 2");
    }

    #[test]
    fn production_footprint_validator_rejects_non_finite_emissions() {
        let mut footprint = valid_footprint();
        footprint.scope2 = f64::NAN;
        assert_invalid_contains(validate_carbon_footprint(&footprint), "finite");
    }

    #[test]
    fn production_footprint_validator_rejects_inverted_period() {
        let mut footprint = valid_footprint();
        footprint.period_start = footprint.period_end;
        assert_invalid_contains(validate_carbon_footprint(&footprint), "Period start");
    }

    #[test]
    fn production_footprint_validator_rejects_empty_methodology() {
        let mut footprint = valid_footprint();
        footprint.methodology.clear();
        assert_invalid_contains(validate_carbon_footprint(&footprint), "Methodology");
    }

    #[test]
    fn production_credit_validator_accepts_valid_credit() {
        assert_valid(validate_carbon_credit(&valid_credit()));
    }

    #[test]
    fn production_credit_validator_rejects_non_positive_tonnes() {
        let mut credit = valid_credit();
        credit.tonnes_co2e = 0.0;
        assert_invalid_contains(validate_carbon_credit(&credit), "tonnes");
    }

    #[test]
    fn production_credit_validator_rejects_non_finite_tonnes() {
        let mut credit = valid_credit();
        credit.tonnes_co2e = f64::INFINITY;
        assert_invalid_contains(validate_carbon_credit(&credit), "finite");
    }

    #[test]
    fn production_credit_validator_rejects_invalid_vintage() {
        let mut credit = valid_credit();
        credit.vintage_year = 2101;
        assert_invalid_contains(validate_carbon_credit(&credit), "Vintage year");
    }

    #[test]
    fn production_credit_validator_requires_retirement_timestamp() {
        let mut credit = valid_credit();
        credit.status = CreditStatus::Retired;
        assert_invalid_contains(validate_carbon_credit(&credit), "retired_at");
    }

    #[test]
    fn production_credit_validator_rejects_timestamp_on_active_credit() {
        let mut credit = valid_credit();
        credit.retired_at = Some(1_800_000_000);
        assert_invalid_contains(validate_carbon_credit(&credit), "Non-retired");
    }

    #[test]
    fn footprint_verification_binds_verifier_to_update_author() {
        let verifier = fake_agent(1);
        let original = valid_footprint();
        let mut updated = original.clone();
        updated.verified_by = Some(did_for_author(&verifier));
        assert!(matches!(
            validate_footprint_transition(&original, &updated, &verifier),
            ValidateCallbackResult::Valid
        ));
    }

    #[test]
    fn footprint_verification_rejects_forged_verifier() {
        let verifier = fake_agent(1);
        let victim = fake_agent(2);
        let original = valid_footprint();
        let mut updated = original.clone();
        updated.verified_by = Some(did_for_author(&victim));
        assert_transition_invalid(
            validate_footprint_transition(&original, &updated, &verifier),
            "committing agent",
        );
    }

    #[test]
    fn footprint_verification_cannot_rewrite_measurement() {
        let verifier = fake_agent(1);
        let original = valid_footprint();
        let mut updated = original.clone();
        updated.scope1 += 1.0;
        updated.verified_by = Some(did_for_author(&verifier));
        assert_transition_invalid(
            validate_footprint_transition(&original, &updated, &verifier),
            "immutable",
        );
    }

    #[test]
    fn footprint_cannot_be_reverified() {
        let first = fake_agent(1);
        let second = fake_agent(2);
        let mut original = valid_footprint();
        original.verified_by = Some(did_for_author(&first));
        let mut updated = original.clone();
        updated.verified_by = Some(did_for_author(&second));
        assert_transition_invalid(
            validate_footprint_transition(&original, &updated, &second),
            "re-verified",
        );
    }

    #[test]
    fn current_owner_can_transfer_credit() {
        let owner = fake_agent(3);
        let new_owner = fake_agent(4);
        let original = owned_credit(&owner);
        let mut updated = original.clone();
        updated.owner_did = did_for_author(&new_owner);
        assert!(matches!(
            validate_credit_transition(&original, &updated, &owner),
            ValidateCallbackResult::Valid
        ));
    }

    #[test]
    fn non_owner_cannot_transfer_credit() {
        let owner = fake_agent(3);
        let attacker = fake_agent(5);
        let new_owner = fake_agent(4);
        let original = owned_credit(&owner);
        let mut updated = original.clone();
        updated.owner_did = did_for_author(&new_owner);
        assert_transition_invalid(
            validate_credit_transition(&original, &updated, &attacker),
            "current carbon credit owner",
        );
    }

    #[test]
    fn transfer_cannot_change_credit_quantity() {
        let owner = fake_agent(3);
        let new_owner = fake_agent(4);
        let original = owned_credit(&owner);
        let mut updated = original.clone();
        updated.owner_did = did_for_author(&new_owner);
        updated.tonnes_co2e += 1.0;
        assert_transition_invalid(
            validate_credit_transition(&original, &updated, &owner),
            "immutable",
        );
    }

    #[test]
    fn current_owner_can_retire_credit() {
        let owner = fake_agent(3);
        let original = owned_credit(&owner);
        let mut updated = original.clone();
        updated.status = CreditStatus::Retired;
        updated.retired_at = Some(1_800_000_000);
        assert!(matches!(
            validate_credit_transition(&original, &updated, &owner),
            ValidateCallbackResult::Valid
        ));
    }

    #[test]
    fn non_owner_cannot_retire_credit() {
        let owner = fake_agent(3);
        let attacker = fake_agent(5);
        let original = owned_credit(&owner);
        let mut updated = original.clone();
        updated.status = CreditStatus::Retired;
        updated.retired_at = Some(1_800_000_000);
        assert_transition_invalid(
            validate_credit_transition(&original, &updated, &attacker),
            "current carbon credit owner",
        );
    }

    #[test]
    fn retired_credit_is_terminal() {
        let owner = fake_agent(3);
        let mut original = owned_credit(&owner);
        original.status = CreditStatus::Retired;
        original.retired_at = Some(1_800_000_000);
        let mut updated = original.clone();
        updated.owner_did = did_for_author(&fake_agent(4));
        updated.status = CreditStatus::Active;
        updated.retired_at = None;
        assert_transition_invalid(
            validate_credit_transition(&original, &updated, &owner),
            "Only an active",
        );
    }
}
