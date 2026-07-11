// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root//! Community Land Trust Integrity Zome
//! Entry types and validation for land trusts, ground leases, resale formulas,
//! and affordability reporting.

use hdi::prelude::*;

/// Anchor entry for deterministic link bases
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Anchor(pub String);

/// A Community Land Trust
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct LandTrust {
    pub id: String,
    pub name: String,
    pub mission: String,
    pub boundary: Vec<(f64, f64)>,
    pub charter_hash: Option<ActionHash>,
    pub stewardship_board: Vec<AgentPubKey>,
    pub affordability_target_ami_percent: u8,
    pub created_at: Timestamp,
}

/// The type of resale formula
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum FormulaType {
    AppreciationCap,
    AreaMedianIncome,
    ConsumerPriceIndex,
    Hybrid,
}

/// Resale formula configuration for a ground lease
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct ResaleFormula {
    pub formula_type: FormulaType,
    pub max_appreciation_percent_annual: Option<u8>,
    pub ami_cap_percent: Option<u8>,
    pub improvement_credit_percent: Option<u8>,
}

/// A ground lease linking a unit to the land trust
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct GroundLease {
    pub trust_hash: ActionHash,
    pub unit_hash: ActionHash,
    pub leaseholder: AgentPubKey,
    pub lease_term_years: u32,
    pub ground_rent_monthly_cents: u64,
    pub resale_formula: ResaleFormula,
    pub started_at: Timestamp,
    pub expires_at: Timestamp,
}

/// A resale price calculation
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct ResaleCalculation {
    pub lease_hash: ActionHash,
    pub original_price_cents: u64,
    pub years_held: u32,
    pub improvements_value_cents: u64,
    pub calculated_max_price_cents: u64,
    pub ami_at_purchase: Option<u64>,
    pub current_ami: Option<u64>,
}

/// An affordability report for a trust
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct AffordabilityReport {
    pub trust_hash: ActionHash,
    pub report_date: Timestamp,
    pub total_units: u32,
    pub affordable_units: u32,
    pub average_monthly_cost_cents: u64,
    pub median_area_income_cents: u64,
    pub affordability_ratio: f32,
}

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    Anchor(Anchor),
    LandTrust(LandTrust),
    GroundLease(GroundLease),
    ResaleCalculation(ResaleCalculation),
    AffordabilityReport(AffordabilityReport),
}

#[hdk_link_types]
pub enum LinkTypes {
    /// All trusts anchor
    AllTrusts,
    /// Trust to its ground leases
    TrustToLease,
    /// Leaseholder to their leases
    LeaseholderToLease,
    /// Lease to resale calculations
    LeaseToResaleCalc,
    /// Trust to affordability reports
    TrustToReport,
    /// Unit to lease
    UnitToLease,
}

#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(store_entry) => match store_entry {
            OpEntry::CreateEntry { app_entry, action } => match app_entry {
                EntryTypes::Anchor(_) => Ok(ValidateCallbackResult::Valid),
                EntryTypes::LandTrust(trust) => validate_create_trust(action, trust),
                EntryTypes::GroundLease(lease) => validate_create_lease(action, lease),
                EntryTypes::ResaleCalculation(calc) => validate_resale_calc(action, calc),
                EntryTypes::AffordabilityReport(report) => {
                    validate_affordability_report(action, report)
                }
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
            LinkTypes::AllTrusts => Ok(ValidateCallbackResult::Valid),
            LinkTypes::TrustToLease => Ok(ValidateCallbackResult::Valid),
            LinkTypes::LeaseholderToLease => Ok(ValidateCallbackResult::Valid),
            LinkTypes::LeaseToResaleCalc => Ok(ValidateCallbackResult::Valid),
            LinkTypes::TrustToReport => Ok(ValidateCallbackResult::Valid),
            LinkTypes::UnitToLease => Ok(ValidateCallbackResult::Valid),
        },
        // Deliberately left fully permissive for ALL link types (reviewed
        // 2026-07-09 during the P0 author-binding pass, not a gap): the
        // coordinator's transfer_lease deletes the old LeaseholderToLease
        // link when a lease changes hands, and that transfer may
        // legitimately be approved by a trust steward who is not the
        // link's original creator (the departing leaseholder). Adding the
        // usual "only original link creator can delete" check here would
        // break that legitimate flow.
        FlatOp::RegisterDeleteLink {
            link_type: _,
            original_action: _,
            base_address: _,
            target_address: _,
            tag: _,
            action: _,
        } => Ok(ValidateCallbackResult::Valid),
        FlatOp::StoreRecord(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterAgentActivity(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterUpdate(op_update) => match op_update {
            // This DHT op was previously left fully permissive (`Ok(Valid)`
            // unconditionally) -- the 21st confirmed instance of this exact
            // bug pattern this pass. Found + fixed 2026-07-09 during the P0
            // author-binding pass. Route through the same per-type
            // validators as the StoreEntry perspective.
            OpUpdate::Entry { app_entry, action } => validate_update_entry_type(action, app_entry),
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterDelete(OpDelete { action }) => {
            // Also previously fully permissive. The coordinator never
            // calls delete_entry here, so this is pure hardening, zero
            // functional impact.
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

/// Shared per-entry-type update validation, called from BOTH the
/// StoreEntry (OpEntry::UpdateEntry) and RegisterUpdate DHT-op
/// perspectives so they agree.
fn validate_update_entry_type(
    action: Update,
    app_entry: EntryTypes,
) -> ExternResult<ValidateCallbackResult> {
    match app_entry {
        EntryTypes::Anchor(_) => Ok(ValidateCallbackResult::Valid),
        EntryTypes::LandTrust(trust) => validate_update_trust(action, trust),
        EntryTypes::GroundLease(lease) => validate_update_lease(action, lease),
        EntryTypes::ResaleCalculation(_) => Ok(ValidateCallbackResult::Invalid(
            "Resale calculations are immutable records".into(),
        )),
        EntryTypes::AffordabilityReport(_) => Ok(ValidateCallbackResult::Invalid(
            "Affordability reports are immutable records".into(),
        )),
    }
}

/// **Real privilege-escalation fix**: the coordinator's update_trust_board
/// had ZERO caller-identity check at all -- ANY agent could replace ANY
/// trust's entire stewardship_board with agents of their own choosing,
/// full governance takeover of a community land trust. Found + fixed
/// 2026-07-09 during the P0 author-binding pass. Requires the committer
/// be a member of the ORIGINAL (pre-update) stewardship_board -- the
/// real DHT-level enforcement of what should have been the coordinator's
/// own authorization model. Content otherwise restricted to
/// stewardship_board only.
fn validate_update_trust(action: Update, trust: LandTrust) -> ExternResult<ValidateCallbackResult> {
    let original_record = must_get_valid_record(action.original_action_address.clone())?;
    let original: LandTrust = original_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Original trust not found".into()
        )))?;

    if !original.stewardship_board.contains(&action.author) {
        return Ok(ValidateCallbackResult::Invalid(
            "Only an existing stewardship board member can update the trust".into(),
        ));
    }

    if trust.id != original.id
        || trust.name != original.name
        || trust.mission != original.mission
        || trust.boundary != original.boundary
        || trust.charter_hash != original.charter_hash
        || trust.affordability_target_ami_percent != original.affordability_target_ami_percent
        || trust.created_at != original.created_at
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Only stewardship_board can change on a trust update".into(),
        ));
    }

    if trust.stewardship_board.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Trust must have at least one stewardship board member".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

/// **Real privilege-escalation fix**: the coordinator's transfer_lease had
/// ZERO caller-identity check at all -- ANY agent could transfer ANY
/// ground lease to a new leaseholder, a real lease-theft vector. Found +
/// fixed 2026-07-09 during the P0 author-binding pass. Requires the
/// committer be EITHER the current leaseholder (voluntary transfer) OR a
/// member of the trust's stewardship board (fetched via a cross-entry
/// must_get on trust_hash, mirroring mycelix-water/flow's cross-entry
/// steward-check pattern from earlier in this pass -- involuntary
/// reassignment, e.g. eviction). Content otherwise restricted to
/// leaseholder only.
fn validate_update_lease(
    action: Update,
    lease: GroundLease,
) -> ExternResult<ValidateCallbackResult> {
    let original_record = must_get_valid_record(action.original_action_address.clone())?;
    let original: GroundLease = original_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Original lease not found".into()
        )))?;

    if action.author != original.leaseholder {
        let trust_record = must_get_valid_record(original.trust_hash.clone())?;
        let trust: LandTrust = trust_record
            .entry()
            .to_app_option()
            .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
            .ok_or(wasm_error!(WasmErrorInner::Guest(
                "Trust for lease not found".into()
            )))?;
        if !trust.stewardship_board.contains(&action.author) {
            return Ok(ValidateCallbackResult::Invalid(
                "Only the current leaseholder or a trust steward can transfer a lease".into(),
            ));
        }
    }

    if lease.trust_hash != original.trust_hash
        || lease.unit_hash != original.unit_hash
        || lease.lease_term_years != original.lease_term_years
        || lease.ground_rent_monthly_cents != original.ground_rent_monthly_cents
        || lease.resale_formula != original.resale_formula
        || lease.started_at != original.started_at
        || lease.expires_at != original.expires_at
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Only leaseholder can change on a lease update".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Bootstrap invariant mirroring validate_update_trust's "only an
/// existing board member may govern the trust" rule: the founding agent
/// must be one of their own trust's founding stewardship board members.
/// Without this, a modified coordinator could create a trust naming only
/// OTHER agents as stewards, permanently locking the real creator out of
/// (or, more importantly, letting an attacker create a trust claiming
/// arbitrary victim agents as board members with no consent). Found +
/// fixed 2026-07-09 during the P0 author-binding pass.
fn validate_create_trust(action: Create, trust: LandTrust) -> ExternResult<ValidateCallbackResult> {
    if !trust.stewardship_board.contains(&action.author) {
        return Ok(ValidateCallbackResult::Invalid(
            "The committing agent must be a member of the founding stewardship board".into(),
        ));
    }

    if trust.id.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Trust ID cannot be empty".into(),
        ));
    }
    if trust.name.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Trust name cannot be empty".into(),
        ));
    }
    if trust.mission.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Trust mission cannot be empty".into(),
        ));
    }
    if trust.boundary.len() < 3 {
        return Ok(ValidateCallbackResult::Invalid(
            "Trust boundary must have at least 3 coordinate points".into(),
        ));
    }
    for (lat, lon) in &trust.boundary {
        if *lat < -90.0 || *lat > 90.0 {
            return Ok(ValidateCallbackResult::Invalid(
                "Boundary latitude must be between -90 and 90".into(),
            ));
        }
        if *lon < -180.0 || *lon > 180.0 {
            return Ok(ValidateCallbackResult::Invalid(
                "Boundary longitude must be between -180 and 180".into(),
            ));
        }
    }
    if trust.stewardship_board.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Trust must have at least one stewardship board member".into(),
        ));
    }
    if trust.affordability_target_ami_percent == 0 || trust.affordability_target_ami_percent > 200 {
        return Ok(ValidateCallbackResult::Invalid(
            "Affordability target must be between 1 and 200 percent of AMI".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

/// leaseholder is deliberately NOT bound to the committer: it's a
/// third-party field by design (a housing coordinator on the trust's
/// staff issues a lease TO a resident, who need not be the committer).
/// But WHO may issue a lease under a trust at all was previously
/// completely unchecked -- the coordinator's issue_ground_lease had zero
/// caller-identity check, so any agent could self-issue a ground lease
/// under any existing trust naming themselves as leaseholder. Fixed by
/// requiring the committer be a member of the trust's stewardship board
/// (fetched via a cross-entry must_get on trust_hash, same pattern as
/// validate_update_lease). Found + fixed 2026-07-09 during the P0
/// author-binding pass.
fn validate_create_lease(
    action: Create,
    lease: GroundLease,
) -> ExternResult<ValidateCallbackResult> {
    let trust_record = must_get_valid_record(lease.trust_hash.clone())?;
    let trust: LandTrust = trust_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Trust for lease not found".into()
        )))?;
    if !trust.stewardship_board.contains(&action.author) {
        return Ok(ValidateCallbackResult::Invalid(
            "Only a trust steward can issue a ground lease".into(),
        ));
    }

    if lease.lease_term_years == 0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Lease term must be at least 1 year".into(),
        ));
    }
    if lease.lease_term_years > 199 {
        return Ok(ValidateCallbackResult::Invalid(
            "Lease term cannot exceed 199 years".into(),
        ));
    }
    if lease.expires_at <= lease.started_at {
        return Ok(ValidateCallbackResult::Invalid(
            "Lease expiration must be after start date".into(),
        ));
    }
    // Validate resale formula consistency
    match lease.resale_formula.formula_type {
        FormulaType::AppreciationCap => {
            if lease
                .resale_formula
                .max_appreciation_percent_annual
                .is_none()
            {
                return Ok(ValidateCallbackResult::Invalid(
                    "AppreciationCap formula requires max_appreciation_percent_annual".into(),
                ));
            }
        }
        FormulaType::AreaMedianIncome => {
            if lease.resale_formula.ami_cap_percent.is_none() {
                return Ok(ValidateCallbackResult::Invalid(
                    "AreaMedianIncome formula requires ami_cap_percent".into(),
                ));
            }
        }
        FormulaType::ConsumerPriceIndex => {}
        FormulaType::Hybrid => {
            if lease
                .resale_formula
                .max_appreciation_percent_annual
                .is_none()
                && lease.resale_formula.ami_cap_percent.is_none()
            {
                return Ok(ValidateCallbackResult::Invalid(
                    "Hybrid formula requires at least one cap parameter".into(),
                ));
            }
        }
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_resale_calc(
    _action: Create,
    calc: ResaleCalculation,
) -> ExternResult<ValidateCallbackResult> {
    if calc.original_price_cents == 0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Original price must be greater than 0".into(),
        ));
    }
    if calc.calculated_max_price_cents == 0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Calculated max price must be greater than 0".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_affordability_report(
    _action: Create,
    report: AffordabilityReport,
) -> ExternResult<ValidateCallbackResult> {
    if report.total_units == 0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Total units must be greater than 0".into(),
        ));
    }
    if report.affordable_units > report.total_units {
        return Ok(ValidateCallbackResult::Invalid(
            "Affordable units cannot exceed total units".into(),
        ));
    }
    if report.affordability_ratio < 0.0 || report.affordability_ratio > 1.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Affordability ratio must be between 0 and 1".into(),
        ));
    }
    if report.median_area_income_cents == 0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Median area income must be greater than 0".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

#[cfg(test)]
mod author_binding_tests {
    use super::*;

    fn create_action(author: AgentPubKey) -> Create {
        Create {
            author,
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

    fn other_agent() -> AgentPubKey {
        AgentPubKey::from_raw_36(vec![1u8; 36])
    }

    fn valid_trust(board: Vec<AgentPubKey>) -> LandTrust {
        LandTrust {
            id: "t-1".into(),
            name: "Riverside CLT".into(),
            mission: "Permanently affordable housing".into(),
            boundary: vec![(0.0, 0.0), (0.0, 1.0), (1.0, 1.0)],
            charter_hash: None,
            stewardship_board: board,
            affordability_target_ami_percent: 80,
            created_at: Timestamp::from_micros(0),
        }
    }

    #[test]
    fn create_trust_valid_when_committer_is_board_member() {
        let author = me();
        let t = valid_trust(vec![author.clone(), other_agent()]);
        let result = validate_create_trust(create_action(author), t).unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn create_trust_rejected_when_committer_not_on_board() {
        let t = valid_trust(vec![other_agent()]);
        let result = validate_create_trust(create_action(me()), t).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    fn valid_resale_calc() -> ResaleCalculation {
        ResaleCalculation {
            lease_hash: ActionHash::from_raw_36(vec![2u8; 36]),
            original_price_cents: 20_000_000,
            years_held: 5,
            improvements_value_cents: 100_000,
            calculated_max_price_cents: 22_000_000,
            ami_at_purchase: None,
            current_ami: None,
        }
    }

    #[test]
    fn update_entry_type_rejects_resale_calc_update() {
        let c = valid_resale_calc();
        let result =
            validate_update_entry_type(update_action(me()), EntryTypes::ResaleCalculation(c))
                .unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }
}
