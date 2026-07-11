// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root//! Membership Integrity Zome
//! Entry types and validation for co-op members, applications, waitlists, and rent-to-own.

use hdi::prelude::*;

/// Unit type preference (local copy to avoid cross-integrity linking)
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum UnitType {
    Studio,
    OneBedroom,
    TwoBedroom,
    ThreeBedroom,
    FourPlus,
    Accessible,
    Family,
}

/// Anchor entry for deterministic link bases
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Anchor(pub String);

/// Type of membership in the cooperative
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum MembershipType {
    FullShare,
    LimitedEquity,
    RentToOwn,
    Renter,
    Associate,
}

/// Current status of a member
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum MemberStatus {
    Active,
    OnLeave,
    Suspended,
    Former,
}

/// A cooperative member
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Member {
    pub agent: AgentPubKey,
    pub unit_hash: Option<ActionHash>,
    pub membership_type: MembershipType,
    pub share_equity_cents: u64,
    pub joined_at: Timestamp,
    pub monthly_charge_cents: u64,
    pub voting_rights: bool,
    pub status: MemberStatus,
}

/// Application status
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum ApplicationStatus {
    Pending,
    UnderReview,
    Approved,
    Rejected,
    Waitlisted,
}

/// A membership application
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct MemberApplication {
    pub applicant: AgentPubKey,
    pub requested_unit: Option<ActionHash>,
    pub membership_type_requested: MembershipType,
    pub applied_at: Timestamp,
    pub household_size: u8,
    pub income_verified: bool,
    pub references: Vec<String>,
    pub status: ApplicationStatus,
}

/// A waitlist entry
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct WaitListEntry {
    pub application_hash: ActionHash,
    pub position: u32,
    pub unit_type_preference: Option<UnitType>,
    pub added_at: Timestamp,
}

/// Status of a rent-to-own agreement
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum AgreementStatus {
    Active,
    Completed,
    Defaulted,
    Terminated,
}

/// A rent-to-own agreement
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct RentToOwnAgreement {
    pub member: AgentPubKey,
    pub unit_hash: ActionHash,
    pub total_purchase_price_cents: u64,
    pub monthly_rent_cents: u64,
    pub equity_portion_percent: u8,
    pub accumulated_equity_cents: u64,
    pub started_at: Timestamp,
    pub target_completion: Timestamp,
    pub status: AgreementStatus,
}

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    Anchor(Anchor),
    Member(Member),
    MemberApplication(MemberApplication),
    WaitListEntry(WaitListEntry),
    RentToOwnAgreement(RentToOwnAgreement),
}

#[hdk_link_types]
pub enum LinkTypes {
    /// All members anchor
    AllMembers,
    /// Agent to their member record
    AgentToMember,
    /// All applications anchor
    AllApplications,
    /// Applicant to application
    ApplicantToApplication,
    /// Waitlist anchor
    Waitlist,
    /// Member to rent-to-own agreement
    MemberToAgreement,
    /// Unit to rent-to-own agreement
    UnitToAgreement,
}

#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(store_entry) => match store_entry {
            OpEntry::CreateEntry { app_entry, action } => match app_entry {
                EntryTypes::Anchor(_) => Ok(ValidateCallbackResult::Valid),
                EntryTypes::Member(member) => validate_create_member(action, member),
                EntryTypes::MemberApplication(app) => validate_create_application(action, app),
                EntryTypes::WaitListEntry(entry) => validate_create_waitlist(action, entry),
                EntryTypes::RentToOwnAgreement(agreement) => {
                    validate_create_agreement(action, agreement)
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
            LinkTypes::AllMembers => Ok(ValidateCallbackResult::Valid),
            LinkTypes::AgentToMember => Ok(ValidateCallbackResult::Valid),
            LinkTypes::AllApplications => Ok(ValidateCallbackResult::Valid),
            LinkTypes::ApplicantToApplication => Ok(ValidateCallbackResult::Valid),
            LinkTypes::Waitlist => Ok(ValidateCallbackResult::Valid),
            LinkTypes::MemberToAgreement => Ok(ValidateCallbackResult::Valid),
            LinkTypes::UnitToAgreement => Ok(ValidateCallbackResult::Valid),
        },
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
            // unconditionally) -- the 25th confirmed instance of this exact
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
        // No live update_entry call for Member (confirmed via grep) --
        // previously silently accepted any field change. Made explicitly
        // immutable.
        EntryTypes::Member(_) => Ok(ValidateCallbackResult::Invalid(
            "Member records are immutable".into(),
        )),
        EntryTypes::MemberApplication(app) => validate_update_application(action, app),
        // No live update_entry call for WaitListEntry either.
        EntryTypes::WaitListEntry(_) => Ok(ValidateCallbackResult::Invalid(
            "Waitlist entries are immutable".into(),
        )),
        EntryTypes::RentToOwnAgreement(agreement) => validate_update_agreement(action, agreement),
    }
}

/// No author requirement: review_application/approve_member both change
/// only `status`, with zero caller-identity check in the coordinator (no
/// board/admin role concept exists here to bind against) -- case (c).
/// Content is restricted to status only -- this closes the wide-open
/// bug that previously let applicant/requested_unit/membership_type_
/// requested/household_size/references change unconditionally on update
/// too.
fn validate_update_application(
    action: Update,
    app: MemberApplication,
) -> ExternResult<ValidateCallbackResult> {
    let original_record = must_get_valid_record(action.original_action_address.clone())?;
    let original: MemberApplication = original_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Original application not found".into()
        )))?;

    if app.applicant != original.applicant
        || app.requested_unit != original.requested_unit
        || app.membership_type_requested != original.membership_type_requested
        || app.applied_at != original.applied_at
        || app.household_size != original.household_size
        || app.income_verified != original.income_verified
        || app.references != original.references
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Only status can change on an application update".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Member.agent is deliberately NOT bound to the committer here: Member
/// entries are created by approve_member on behalf of an applicant, not
/// by the member self-declaring membership. This is safe by
/// construction, not merely unchecked -- approve_member derives `agent`
/// from the STORED MemberApplication.applicant (fixed below to be
/// author-bound at submission time), so the identity was already
/// validated at that earlier point, not re-validated here. Reviewed
/// 2026-07-09 during the P0 author-binding pass; case (b), safety is
/// transitive through the application record.
fn validate_create_member(_action: Create, member: Member) -> ExternResult<ValidateCallbackResult> {
    if member.monthly_charge_cents == 0 && member.membership_type != MembershipType::Associate {
        return Ok(ValidateCallbackResult::Invalid(
            "Non-associate members must have a monthly charge".into(),
        ));
    }
    if member.membership_type == MembershipType::FullShare && !member.voting_rights {
        return Ok(ValidateCallbackResult::Invalid(
            "Full share members must have voting rights".into(),
        ));
    }
    if member.equity_portion_percent_valid() {
        Ok(ValidateCallbackResult::Valid)
    } else {
        Ok(ValidateCallbackResult::Invalid(
            "Invalid equity configuration".into(),
        ))
    }
}

fn validate_create_application(
    action: Create,
    app: MemberApplication,
) -> ExternResult<ValidateCallbackResult> {
    // Author-binding: the coordinator's submit_application previously
    // took the FULL struct straight from caller input with ZERO
    // derivation from agent_info() -- any agent could forge a victim
    // agent as applicant. Found + fixed 2026-07-09 during the P0
    // author-binding pass (coordinator-side fix applied alongside this).
    // This binding is also what makes approve_member's downstream
    // Member.agent := applicant derivation safe.
    if app.applicant != action.author {
        return Ok(ValidateCallbackResult::Invalid(
            "Application applicant must correspond to the committing agent".into(),
        ));
    }

    if app.household_size == 0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Household size must be at least 1".into(),
        ));
    }
    if app.references.len() > 10 {
        return Ok(ValidateCallbackResult::Invalid(
            "Maximum 10 references allowed".into(),
        ));
    }
    if app.status != ApplicationStatus::Pending {
        return Ok(ValidateCallbackResult::Invalid(
            "New applications must have Pending status".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_waitlist(
    _action: Create,
    _entry: WaitListEntry,
) -> ExternResult<ValidateCallbackResult> {
    Ok(ValidateCallbackResult::Valid)
}

/// RentToOwnAgreement.member is deliberately NOT bound to the committer:
/// create_rent_to_own is an admin operation creating an agreement FOR a
/// specific member, a third-party field by design. WHO may call
/// create_rent_to_own at all is unchecked (no established authority
/// model exists in this zome to bind against) -- case (c), a real but
/// separate gap, not fixed here.
fn validate_create_agreement(
    _action: Create,
    agreement: RentToOwnAgreement,
) -> ExternResult<ValidateCallbackResult> {
    if agreement.total_purchase_price_cents == 0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Purchase price must be greater than 0".into(),
        ));
    }
    if agreement.monthly_rent_cents == 0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Monthly rent must be greater than 0".into(),
        ));
    }
    if agreement.equity_portion_percent > 100 {
        return Ok(ValidateCallbackResult::Invalid(
            "Equity portion percent cannot exceed 100".into(),
        ));
    }
    if agreement.accumulated_equity_cents != 0 {
        return Ok(ValidateCallbackResult::Invalid(
            "New agreements must start with 0 accumulated equity".into(),
        ));
    }
    if agreement.status != AgreementStatus::Active {
        return Ok(ValidateCallbackResult::Invalid(
            "New agreements must be Active".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

/// No author requirement: record_rent_payment has zero caller-identity
/// check either -- case (c). Content restricted to
/// accumulated_equity_cents/status -- this closes the wide-open bug
/// that previously let member/unit_hash/total_purchase_price_cents/
/// monthly_rent_cents/equity_portion_percent change unconditionally on
/// update too (member's own binding, above, would otherwise have been
/// pointless if the update path could silently reassign an agreement to
/// a different member).
fn validate_update_agreement(
    action: Update,
    agreement: RentToOwnAgreement,
) -> ExternResult<ValidateCallbackResult> {
    let original_record = must_get_valid_record(action.original_action_address.clone())?;
    let original: RentToOwnAgreement = original_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Original agreement not found".into()
        )))?;

    if agreement.member != original.member
        || agreement.unit_hash != original.unit_hash
        || agreement.total_purchase_price_cents != original.total_purchase_price_cents
        || agreement.monthly_rent_cents != original.monthly_rent_cents
        || agreement.equity_portion_percent != original.equity_portion_percent
        || agreement.started_at != original.started_at
        || agreement.target_completion != original.target_completion
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Only accumulated_equity_cents/status can change on an agreement update".into(),
        ));
    }

    if agreement.equity_portion_percent > 100 {
        return Ok(ValidateCallbackResult::Invalid(
            "Equity portion percent cannot exceed 100".into(),
        ));
    }
    if agreement.accumulated_equity_cents > agreement.total_purchase_price_cents {
        return Ok(ValidateCallbackResult::Invalid(
            "Accumulated equity cannot exceed purchase price".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

/// Helper trait for member validation
impl Member {
    fn equity_portion_percent_valid(&self) -> bool {
        match self.membership_type {
            MembershipType::FullShare => self.share_equity_cents > 0,
            MembershipType::LimitedEquity => true,
            MembershipType::RentToOwn => true,
            MembershipType::Renter => self.share_equity_cents == 0,
            MembershipType::Associate => self.share_equity_cents == 0,
        }
    }
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

    fn valid_application(applicant: AgentPubKey) -> MemberApplication {
        MemberApplication {
            applicant,
            requested_unit: None,
            membership_type_requested: MembershipType::FullShare,
            applied_at: Timestamp::from_micros(0),
            household_size: 2,
            income_verified: false,
            references: vec![],
            status: ApplicationStatus::Pending,
        }
    }

    #[test]
    fn create_application_valid_when_applicant_matches_committer() {
        let a = valid_application(me());
        let result = validate_create_application(create_action(me()), a).unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn create_application_forgery_rejected() {
        let a = valid_application(me());
        let result = validate_create_application(create_action(other_agent()), a).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn update_entry_type_rejects_member_update() {
        let m = Member {
            agent: me(),
            unit_hash: None,
            membership_type: MembershipType::FullShare,
            share_equity_cents: 500_000,
            joined_at: Timestamp::from_micros(0),
            monthly_charge_cents: 100_000,
            voting_rights: true,
            status: MemberStatus::Active,
        };
        let result =
            validate_update_entry_type(update_action(me()), EntryTypes::Member(m)).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn update_entry_type_rejects_waitlist_update() {
        let w = WaitListEntry {
            application_hash: ActionHash::from_raw_36(vec![2u8; 36]),
            position: 1,
            unit_type_preference: None,
            added_at: Timestamp::from_micros(0),
        };
        let result =
            validate_update_entry_type(update_action(me()), EntryTypes::WaitListEntry(w)).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }
}
