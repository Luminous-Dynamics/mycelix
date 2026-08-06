// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root//! Credit Scoring Integrity Zome
//! Updated to use HDI 0.7 patterns with FlatOp validation
use hdi::prelude::*;
use mycelix_bridge_entry_types::{did_for_author, require_did_is_author};

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct CreditProfile {
    pub did: String,
    pub matl_score: f64,
    pub payment_history_score: f64,
    pub collateral_ratio: f64,
    pub account_age_days: u64,
    pub activity_score: f64,
    pub computed_score: f64,
    pub last_updated: Timestamp,
    pub version: u32,
}

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct PaymentRecord {
    pub id: String,
    pub profile_did: String,
    pub loan_id: String,
    pub amount: f64,
    pub currency: String,
    pub due_date: Timestamp,
    pub paid_date: Option<Timestamp>,
    pub status: PaymentStatus,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum PaymentStatus {
    Pending,
    OnTime,
    Late,
    Missed,
    Forgiven,
}

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct CollateralRecord {
    pub id: String,
    pub owner_did: String,
    pub asset_type: CollateralType,
    pub asset_id: String,
    pub appraised_value: f64,
    pub currency: String,
    pub locked_for_loan: Option<String>,
    pub registered: Timestamp,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum CollateralType {
    Property,
    EnergyAsset,
    Credential,
    Token,
    Other(String),
}

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    CreditProfile(CreditProfile),
    PaymentRecord(PaymentRecord),
    CollateralRecord(CollateralRecord),
}

#[hdk_link_types]
pub enum LinkTypes {
    DidToProfile,
    ProfileToPayments,
    ProfileToCollateral,
    LoanToPayments,
}

/// Genesis self-check - called when app is installed
#[hdk_extern]
pub fn genesis_self_check(_data: GenesisSelfCheckData) -> ExternResult<ValidateCallbackResult> {
    Ok(ValidateCallbackResult::Valid)
}

/// Main validation callback using FlatOp pattern matching
#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(store_entry) => match store_entry {
            OpEntry::CreateEntry { app_entry, action } => match app_entry {
                EntryTypes::CreditProfile(profile) => {
                    validate_create_credit_profile(EntryCreationAction::Create(action), profile)
                }
                EntryTypes::PaymentRecord(record) => {
                    validate_create_payment_record(EntryCreationAction::Create(action), record)
                }
                EntryTypes::CollateralRecord(record) => {
                    validate_create_collateral_record(EntryCreationAction::Create(action), record)
                }
            },
            OpEntry::UpdateEntry {
                app_entry, action, ..
            } => match app_entry {
                EntryTypes::CreditProfile(profile) => {
                    validate_update_credit_profile(action, profile)
                }
                EntryTypes::PaymentRecord(record) => validate_update_payment_record(action, record),
                EntryTypes::CollateralRecord(_) => Ok(ValidateCallbackResult::Invalid(
                    "Collateral records cannot be updated, only new ones created".into(),
                )),
            },
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterCreateLink { link_type, .. } => match link_type {
            LinkTypes::DidToProfile => Ok(ValidateCallbackResult::Valid),
            LinkTypes::ProfileToPayments => Ok(ValidateCallbackResult::Valid),
            LinkTypes::ProfileToCollateral => Ok(ValidateCallbackResult::Valid),
            LinkTypes::LoanToPayments => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterDeleteLink { .. } => Ok(ValidateCallbackResult::Valid),
        FlatOp::StoreRecord(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterAgentActivity(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterUpdate(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterDelete(_) => Ok(ValidateCallbackResult::Valid),
    }
}

fn validate_create_credit_profile(
    action: EntryCreationAction,
    profile: CreditProfile,
) -> ExternResult<ValidateCallbackResult> {
    // Bind the profile to its committer. `create_credit_profile`
    // (credit_scoring/coordinator/src/lib.rs:79) takes `input.did` straight from
    // the client with NO caller check at either layer, so before this any agent
    // could create or replace a credit profile for anyone
    // (MYCELIX_AUTHOR_BINDING_TRIAGE_2026-07-09.md, finance Class-A,
    // `credit_scoring:131`).
    //
    // Safe to bind unconditionally: the coordinator has exactly one creation path
    // and no third-party/on-behalf-of flow (verified 2026-07-28).
    //
    // RESIDUAL, deliberately NOT closed here (Class D, needs its own unit):
    // `matl_score` is also client-supplied and feeds `computed_score`, so an
    // agent can still inflate *its own* score. Binding `did` closes impersonation,
    // not self-scoring. The real fix is score provenance — deriving matl_score
    // from an authenticated recognition/MATL source rather than trusting input.
    let author_did = did_for_author(action.author());
    if let ValidateCallbackResult::Invalid(msg) =
        require_did_is_author("CreditProfile", "did", &profile.did, &author_did)
    {
        return Ok(ValidateCallbackResult::Invalid(msg));
    }

    if !profile.did.starts_with("did:") {
        return Ok(ValidateCallbackResult::Invalid(
            "DID must be a valid DID format".into(),
        ));
    }
    if profile.computed_score < 0.0 || profile.computed_score > 1000.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Credit score must be between 0 and 1000".into(),
        ));
    }
    if profile.matl_score < 0.0 || profile.matl_score > 1.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "MATL score must be between 0 and 1".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

/// Validate credit profile update
fn validate_update_credit_profile(
    action: Update,
    profile: CreditProfile,
) -> ExternResult<ValidateCallbackResult> {
    // Bind updates too. Binding create alone would be incoherent: the update path
    // was the easier hole — `update_matl_score` is a public extern taking an
    // arbitrary `did` AND an arbitrary `matl_score`, so at the DHT level any peer
    // could rewrite anyone's score.
    //
    // Safe to bind: both coordinator update paths (`update_credit_score:258`,
    // `update_matl_score:476`) locate the profile with `query()`, which reads the
    // CALLER'S OWN source chain only — so they can already only touch profiles the
    // caller authored. Binding matches that de-facto behaviour and additionally
    // enforces it against a malicious peer that bypasses the coordinator.
    //
    // (Aside: that `query()` usage is a fresh instance of the chain-local-read bug
    // class tracked in MASTER_ROADMAP P0-#1 — here it happens to fail safe, but
    // `update_credit_score` silently no-ops for any profile not on the caller's
    // chain. Flagged, not fixed here — it is a coordinator correctness bug, not an
    // integrity one.)
    //
    // NOTE: on `Update`, `author` is a FIELD; on `EntryCreationAction` it is a
    // method. Hence the asymmetry with `validate_create_credit_profile` above.
    let author_did = did_for_author(&action.author);
    if let ValidateCallbackResult::Invalid(msg) =
        require_did_is_author("CreditProfile", "did", &profile.did, &author_did)
    {
        return Ok(ValidateCallbackResult::Invalid(msg));
    }

    // Apply same rules as creation
    if profile.computed_score < 0.0 || profile.computed_score > 1000.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Credit score must be between 0 and 1000".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

/// Validate payment record creation
fn validate_create_payment_record(
    _action: EntryCreationAction,
    record: PaymentRecord,
) -> ExternResult<ValidateCallbackResult> {
    if record.amount <= 0.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Payment amount must be positive".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

/// Validate payment record update
fn validate_update_payment_record(
    _action: Update,
    record: PaymentRecord,
) -> ExternResult<ValidateCallbackResult> {
    if record.amount <= 0.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Payment amount must be positive".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

/// Validate collateral record creation
fn validate_create_collateral_record(
    action: EntryCreationAction,
    record: CollateralRecord,
) -> ExternResult<ValidateCallbackResult> {
    if !record.owner_did.starts_with("did:") {
        return Ok(ValidateCallbackResult::Invalid(
            "Owner must be a valid DID".into(),
        ));
    }

    // Bind the collateral to its committer. `register_collateral`
    // (credit_scoring/coordinator/src/lib.rs:204) takes `input.owner_did` from the
    // client with no caller check, so before this any agent could register
    // collateral in someone else's name — which feeds `collateral_ratio` and
    // therefore their credit score
    // (MYCELIX_AUTHOR_BINDING_TRIAGE_2026-07-09.md, finance Class-A, `credit_scoring:194`).
    //
    // Placed after the DID-format check so malformed input still reports the
    // precise format error.
    //
    // Create-only: no update binding is needed because the dispatcher above
    // already rejects every CollateralRecord update outright.
    let author_did = did_for_author(action.author());
    if let ValidateCallbackResult::Invalid(msg) = require_did_is_author(
        "CollateralRecord",
        "owner_did",
        &record.owner_did,
        &author_did,
    ) {
        return Ok(ValidateCallbackResult::Invalid(msg));
    }
    if record.appraised_value <= 0.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Appraised value must be positive".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ts(micros: i64) -> Timestamp {
        Timestamp::from_micros(micros)
    }

    fn make_create() -> Create {
        Create {
            author: AgentPubKey::from_raw_36(vec![0; 36]),
            timestamp: ts(1_000_000),
            action_seq: 0,
            prev_action: ActionHash::from_raw_36(vec![0; 36]),
            entry_type: EntryType::CapClaim,
            entry_hash: EntryHash::from_raw_36(vec![0; 36]),
            weight: Default::default(),
        }
    }

    /// DID of the agent `make_create()` attributes actions to. Fixtures meant to
    /// be VALID must use this — `validate_create_credit_profile` binds `did` to
    /// the committing agent.
    fn test_author_did() -> String {
        format!("did:mycelix:{}", AgentPubKey::from_raw_36(vec![0; 36]))
    }

    fn valid_profile() -> CreditProfile {
        CreditProfile {
            did: test_author_did(),
            matl_score: 0.8,
            payment_history_score: 1.0,
            collateral_ratio: 1.5,
            account_age_days: 0,
            activity_score: 0.5,
            computed_score: 700.0,
            last_updated: ts(1_000_000),
            version: 1,
        }
    }

    #[test]
    fn test_profile_for_the_committing_agent_is_accepted() {
        let result = validate_create_credit_profile(
            EntryCreationAction::Create(make_create()),
            valid_profile(),
        )
        .unwrap();
        assert!(matches!(result, ValidateCallbackResult::Valid));
    }

    #[test]
    fn test_profile_forged_for_another_did_is_rejected() {
        // Before 2026-07-28 this returned Valid: any agent could create or
        // replace a credit profile belonging to anyone.
        let mut forged = valid_profile();
        forged.did = "did:mycelix:uhCAkSomeoneElse".into();
        let result =
            validate_create_credit_profile(EntryCreationAction::Create(make_create()), forged)
                .unwrap();
        match result {
            ValidateCallbackResult::Invalid(msg) => {
                assert!(msg.contains("forgery"), "got: {msg}")
            }
            other => panic!("forged did must be rejected, got {other:?}"),
        }
    }

    #[test]
    fn test_require_did_is_author_rejects_empty_author() {
        let result = require_did_is_author("CreditProfile", "did", "did:mycelix:uhCAkalice", "");
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    fn make_update() -> Update {
        Update {
            author: AgentPubKey::from_raw_36(vec![0; 36]),
            timestamp: ts(1_000_000),
            action_seq: 1,
            prev_action: ActionHash::from_raw_36(vec![0; 36]),
            original_action_address: ActionHash::from_raw_36(vec![0; 36]),
            original_entry_address: EntryHash::from_raw_36(vec![0; 36]),
            entry_type: EntryType::CapClaim,
            entry_hash: EntryHash::from_raw_36(vec![0; 36]),
            weight: Default::default(),
        }
    }

    #[test]
    fn test_update_own_profile_is_accepted() {
        let result = validate_update_credit_profile(make_update(), valid_profile()).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Valid));
    }

    #[test]
    fn test_update_another_agents_profile_is_rejected() {
        // The easier of the two holes: `update_matl_score` takes an arbitrary did
        // AND an arbitrary score, so before this any peer could rewrite anyone's
        // credit score at the DHT level.
        let mut forged = valid_profile();
        forged.did = "did:mycelix:uhCAkSomeoneElse".into();
        let result = validate_update_credit_profile(make_update(), forged).unwrap();
        match result {
            ValidateCallbackResult::Invalid(msg) => {
                assert!(msg.contains("forgery"), "got: {msg}")
            }
            other => panic!("forged did on update must be rejected, got {other:?}"),
        }
    }

    fn valid_collateral() -> CollateralRecord {
        CollateralRecord {
            id: "coll:test:001".into(),
            owner_did: test_author_did(),
            asset_type: CollateralType::Token,
            asset_id: "asset:001".into(),
            appraised_value: 1000.0,
            currency: "SAP".into(),
            locked_for_loan: None,
            registered: ts(1_000_000),
        }
    }

    #[test]
    fn test_collateral_from_the_committing_agent_is_accepted() {
        let result = validate_create_collateral_record(
            EntryCreationAction::Create(make_create()),
            valid_collateral(),
        )
        .unwrap();
        assert!(matches!(result, ValidateCallbackResult::Valid));
    }

    #[test]
    fn test_collateral_forged_for_another_owner_is_rejected() {
        // Registering collateral in someone else's name inflates their
        // collateral_ratio, and therefore their credit score.
        let mut forged = valid_collateral();
        forged.owner_did = "did:mycelix:uhCAkSomeoneElse".into();
        let result =
            validate_create_collateral_record(EntryCreationAction::Create(make_create()), forged)
                .unwrap();
        match result {
            ValidateCallbackResult::Invalid(msg) => {
                assert!(msg.contains("forgery"), "got: {msg}")
            }
            other => panic!("forged owner_did must be rejected, got {other:?}"),
        }
    }
}
