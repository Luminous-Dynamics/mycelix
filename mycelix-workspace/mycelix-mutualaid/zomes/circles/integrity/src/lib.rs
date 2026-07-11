// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root//! Circles Integrity Zome
//!
//! This zome defines entry types and validation rules for community credit circles
//! in the Mycelix Mutual Aid hApp. Implements mutual credit with automatic clearing.

use hdi::prelude::*;
use mutualaid_common::*;

/// Entry types for the circles zome
#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    /// A mutual credit circle
    #[entry_type(visibility = "public")]
    CreditCircle(CreditCircle),
    /// A member's credit line within a circle
    #[entry_type(visibility = "public")]
    CreditLine(CreditLine),
    /// A credit transaction
    #[entry_type(visibility = "public")]
    CreditTransaction(CreditTransaction),
    /// Balance snapshot
    #[entry_type(visibility = "public")]
    Balance(Balance),
}

/// Link types for the circles zome
#[hdk_link_types]
pub enum LinkTypes {
    /// Link from circle to its members
    CircleToMembers,
    /// Link from member to their circles
    MemberToCircles,
    /// Link from member to their credit lines
    MemberToCreditLines,
    /// Link from circle to transactions
    CircleToTransactions,
    /// Link from member to their transactions
    MemberToTransactions,
    /// Link for all circles discovery
    AllCircles,
    /// Link from circle to latest balances
    CircleToBalances,
}

/// Genesis self-check
#[hdk_extern]
pub fn genesis_self_check(_data: GenesisSelfCheckData) -> ExternResult<ValidateCallbackResult> {
    Ok(ValidateCallbackResult::Valid)
}

/// Main validation callback
#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(store_entry) => match store_entry {
            OpEntry::CreateEntry { app_entry, action } => validate_create_entry(action, app_entry),
            OpEntry::UpdateEntry {
                app_entry,
                original_action_hash,
                action,
                ..
            } => validate_update_entry_type(action, original_action_hash, app_entry),
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterCreateLink {
            link_type,
            base_address,
            target_address,
            tag,
            ..
        } => validate_create_link(link_type, base_address, target_address, tag),
        // Deliberately left fully permissive for ALL link types (reviewed
        // 2026-07-09 during the P0 author-binding pass, not a gap): this
        // coordinator never calls delete_link, so there's nothing to
        // harden against either way.
        FlatOp::RegisterDeleteLink { link_type, .. } => {
            let _ = link_type;
            Ok(ValidateCallbackResult::Valid)
        }
        FlatOp::RegisterUpdate(op_update) => match op_update {
            // This DHT op was previously left fully permissive (`Ok(Valid)`
            // unconditionally via the catch-all `_` arm) -- the 30th
            // confirmed instance of this exact bug pattern this pass.
            // Found + fixed 2026-07-09 during the P0 author-binding pass.
            OpUpdate::Entry { app_entry, action } => validate_update_entry_type(
                action.clone(),
                action.original_action_address,
                app_entry,
            ),
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
        _ => Ok(ValidateCallbackResult::Valid),
    }
}

/// Validate entry creation
fn validate_create_entry(
    action: Create,
    entry: EntryTypes,
) -> ExternResult<ValidateCallbackResult> {
    match entry {
        EntryTypes::CreditCircle(circle) => validate_credit_circle(action, circle),
        EntryTypes::CreditLine(line) => {
            // Author-binding: the coordinator's join_circle already
            // derives member from agent_info(), so this is
            // belt-and-suspenders. Found + fixed 2026-07-09 during the
            // P0 author-binding pass.
            if line.member != action.author {
                return Ok(ValidateCallbackResult::Invalid(
                    "Credit line member must correspond to the committing agent".into(),
                ));
            }
            validate_credit_line(line)
        }
        EntryTypes::CreditTransaction(tx) => {
            // Author-binding: transfer() always derives `from` from
            // agent_info() (never from caller input), so this is
            // belt-and-suspenders EXCEPT for Clearing transactions,
            // which run_clearing legitimately creates with `from`/`to`
            // set to computed debtor/creditor pairs, not the triggering
            // agent -- a genuine third-party case. Disclosed, NOT
            // fixed: run_clearing itself has no authorization check on
            // who may trigger a clearing round at all (a separate real
            // gap from identity forgery). Found + fixed 2026-07-09
            // during the P0 author-binding pass.
            if tx.transaction_type != TransactionType::Clearing && tx.from != action.author {
                return Ok(ValidateCallbackResult::Invalid(
                    "Transaction from must correspond to the committing agent (except Clearing transactions)".into(),
                ));
            }
            validate_credit_transaction(tx)
        }
        // Balance is never actually create_entry'd anywhere in the
        // coordinator (confirmed via grep) -- it's a purely ephemeral
        // computed return value from get_my_balance_in_circle/
        // get_circle_balances, despite being declared as a real entry
        // type. Content validation kept as defense-in-depth only.
        EntryTypes::Balance(balance) => validate_balance(balance),
    }
}

/// Only CreditLine has a live coordinator update path (transfer/
/// run_clearing/adjust_credit_limit). CreditCircle/CreditTransaction/
/// Balance have no live update call at all (confirmed via grep for
/// `update_entry`) and are made explicitly immutable. Reviewed
/// 2026-07-09 during the P0 author-binding pass: previously all 4 entry
/// types routed updates through the same create-shaped validator with
/// no comparison to the original.
fn validate_update_entry_type(
    action: Update,
    original_action_hash: ActionHash,
    entry: EntryTypes,
) -> ExternResult<ValidateCallbackResult> {
    match entry {
        EntryTypes::CreditCircle(_) => Ok(ValidateCallbackResult::Invalid(
            "Credit circles are immutable".into(),
        )),
        EntryTypes::CreditLine(line) => {
            validate_update_credit_line(action, original_action_hash, line)
        }
        EntryTypes::CreditTransaction(_) => Ok(ValidateCallbackResult::Invalid(
            "Credit transactions are immutable".into(),
        )),
        EntryTypes::Balance(_) => Ok(ValidateCallbackResult::Invalid(
            "Balance snapshots are immutable (and never actually updated by this coordinator)"
                .into(),
        )),
    }
}

/// **Real, fixable authorization gap, unlike mycelix-mutualaid/bridge,
/// requests, and pools (which all use unverifiable String DIDs) --
/// CreditCircle/CreditLine here use real `AgentPubKey` fields, so a
/// genuine must_get-based authorization check IS possible.**
/// adjust_credit_limit previously had ZERO caller-identity check at
/// all -- any agent could raise or lower ANY other member's credit
/// limit (a self-mint / arbitrary-limit-manipulation vector in a mutual
/// credit system). Fixed: when `credit_limit` is the field that
/// changed, the committer must be a member of the circle's `founders`
/// list (fetched via a cross-entry must_get on the line's
/// `circle_hash`), mirroring mycelix-housing/clt's
/// stewardship-board-authorization pattern from earlier in this pass.
/// When only balance/total_credit_extended/total_credit_received/
/// last_activity change (transfer/run_clearing), NO author requirement
/// applies -- both a self-transfer-out and a counter-party
/// credit-in are legitimate, and (disclosed, NOT fixed here)
/// run_clearing itself has no authorization on who may trigger a
/// clearing round at all, a separate real gap.
fn validate_update_credit_line(
    action: Update,
    original_action_hash: ActionHash,
    line: CreditLine,
) -> ExternResult<ValidateCallbackResult> {
    let original_record = must_get_valid_record(original_action_hash)?;
    let original: CreditLine = original_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Original credit line not found".into()
        )))?;

    if line.circle_hash != original.circle_hash
        || line.member != original.member
        || line.status != original.status
    {
        return Ok(ValidateCallbackResult::Invalid(
            "circle_hash/member/status cannot change on a credit line update".into(),
        ));
    }

    if line.credit_limit != original.credit_limit {
        let circle_record = must_get_valid_record(line.circle_hash.clone())?;
        let circle: CreditCircle = circle_record
            .entry()
            .to_app_option()
            .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
            .ok_or(wasm_error!(WasmErrorInner::Guest(
                "Circle for credit line not found".into()
            )))?;
        if !circle.founders.contains(&action.author) {
            return Ok(ValidateCallbackResult::Invalid(
                "Only a circle founder can adjust a member's credit limit".into(),
            ));
        }
    }

    validate_credit_line(line)
}

/// Validate a credit circle. `founders` is bound to the committer
/// (belt-and-suspenders -- create_circle already derives the sole
/// founder from agent_info()). Found + fixed 2026-07-09 during the P0
/// author-binding pass.
fn validate_credit_circle(
    action: Create,
    circle: CreditCircle,
) -> ExternResult<ValidateCallbackResult> {
    if !circle.founders.contains(&action.author) {
        return Ok(ValidateCallbackResult::Invalid(
            "The committing agent must be one of the circle's founders".into(),
        ));
    }
    // ID must not be empty
    if circle.id.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Circle ID cannot be empty".to_string(),
        ));
    }

    // Name must not be empty
    if circle.name.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Circle name cannot be empty".to_string(),
        ));
    }

    // Name length limit
    if circle.name.len() > 100 {
        return Ok(ValidateCallbackResult::Invalid(
            "Circle name cannot exceed 100 characters".to_string(),
        ));
    }

    // Description length limit
    if circle.description.len() > 2000 {
        return Ok(ValidateCallbackResult::Invalid(
            "Circle description cannot exceed 2000 characters".to_string(),
        ));
    }

    // Currency name must not be empty
    if circle.currency_name.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Currency name cannot be empty".to_string(),
        ));
    }

    // Currency symbol must not be empty
    if circle.currency_symbol.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Currency symbol cannot be empty".to_string(),
        ));
    }

    // Default credit limit must be positive
    if circle.default_credit_limit <= 0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Default credit limit must be positive".to_string(),
        ));
    }

    // Max credit limit must be >= default
    if circle.max_credit_limit < circle.default_credit_limit {
        return Ok(ValidateCallbackResult::Invalid(
            "Max credit limit cannot be less than default".to_string(),
        ));
    }

    // Transaction fee must be non-negative
    if circle.transaction_fee_percent < 0.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Transaction fee cannot be negative".to_string(),
        ));
    }

    // Fee must be reasonable (max 10%)
    if circle.transaction_fee_percent > 10.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Transaction fee cannot exceed 10%".to_string(),
        ));
    }

    // Demurrage rate must be non-negative
    if circle.demurrage_rate_percent < 0.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Demurrage rate cannot be negative".to_string(),
        ));
    }

    // Must have at least one founder
    if circle.founders.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Circle must have at least one founder".to_string(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Validate a credit line
fn validate_credit_line(line: CreditLine) -> ExternResult<ValidateCallbackResult> {
    // Credit limit must be positive
    if line.credit_limit <= 0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Credit limit must be positive".to_string(),
        ));
    }

    // Balance cannot exceed credit limit (too negative)
    if line.balance < -(line.credit_limit as i64) {
        return Ok(ValidateCallbackResult::Invalid(
            "Balance exceeds credit limit".to_string(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Validate a credit transaction
fn validate_credit_transaction(tx: CreditTransaction) -> ExternResult<ValidateCallbackResult> {
    // ID must not be empty
    if tx.id.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Transaction ID cannot be empty".to_string(),
        ));
    }

    // Amount must be positive (transfer direction determined by from/to)
    if tx.amount <= 0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Transaction amount must be positive".to_string(),
        ));
    }

    // From and to must be different
    if tx.from == tx.to {
        return Ok(ValidateCallbackResult::Invalid(
            "Cannot transfer to yourself".to_string(),
        ));
    }

    // Memo length limit
    if tx.memo.len() > 500 {
        return Ok(ValidateCallbackResult::Invalid(
            "Transaction memo cannot exceed 500 characters".to_string(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Validate a balance entry
fn validate_balance(balance: Balance) -> ExternResult<ValidateCallbackResult> {
    // Credit available cannot be negative
    if balance.credit_available < 0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Credit available cannot be negative".to_string(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Validate link creation
fn validate_create_link(
    link_type: LinkTypes,
    _base_address: AnyLinkableHash,
    _target_address: AnyLinkableHash,
    _tag: LinkTag,
) -> ExternResult<ValidateCallbackResult> {
    match link_type {
        LinkTypes::CircleToMembers => Ok(ValidateCallbackResult::Valid),
        LinkTypes::MemberToCircles => Ok(ValidateCallbackResult::Valid),
        LinkTypes::MemberToCreditLines => Ok(ValidateCallbackResult::Valid),
        LinkTypes::CircleToTransactions => Ok(ValidateCallbackResult::Valid),
        LinkTypes::MemberToTransactions => Ok(ValidateCallbackResult::Valid),
        LinkTypes::AllCircles => Ok(ValidateCallbackResult::Valid),
        LinkTypes::CircleToBalances => Ok(ValidateCallbackResult::Valid),
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

    fn third_agent() -> AgentPubKey {
        AgentPubKey::from_raw_36(vec![3u8; 36])
    }

    fn valid_circle(founders: Vec<AgentPubKey>) -> CreditCircle {
        CreditCircle {
            id: "c-1".into(),
            name: "Neighborhood Circle".into(),
            description: "".into(),
            currency_name: "Hours".into(),
            currency_symbol: "H".into(),
            default_credit_limit: 100,
            max_credit_limit: 1000,
            transaction_fee_percent: 0.0,
            demurrage_rate_percent: 0.0,
            geographic_scope: None,
            founders,
            rules_hash: None,
            created_at: Timestamp::from_micros(0),
            active: true,
        }
    }

    #[test]
    fn create_circle_valid_when_committer_is_founder() {
        let author = me();
        let circle = valid_circle(vec![author.clone()]);
        let result = validate_credit_circle(create_action(author), circle).unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn create_circle_rejected_when_committer_not_founder() {
        let circle = valid_circle(vec![other_agent()]);
        let result = validate_credit_circle(create_action(me()), circle).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    fn valid_line(member: AgentPubKey) -> CreditLine {
        CreditLine {
            circle_hash: ActionHash::from_raw_36(vec![2u8; 36]),
            member,
            credit_limit: 100,
            balance: 0,
            total_credit_extended: 0,
            total_credit_received: 0,
            joined_at: Timestamp::from_micros(0),
            status: CreditLineStatus::Active,
            last_activity: Timestamp::from_micros(0),
        }
    }

    #[test]
    fn create_line_forgery_rejected() {
        let line = valid_line(me());
        let result =
            validate_create_entry(create_action(other_agent()), EntryTypes::CreditLine(line))
                .unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    fn valid_tx(from: AgentPubKey, transaction_type: TransactionType) -> CreditTransaction {
        CreditTransaction {
            id: "tx-1".into(),
            circle_hash: ActionHash::from_raw_36(vec![2u8; 36]),
            from,
            to: other_agent(),
            amount: 10,
            transaction_type,
            memo: "".into(),
            related_exchange_hash: None,
            created_at: Timestamp::from_micros(0),
            confirmed: true,
        }
    }

    #[test]
    fn create_transaction_from_forgery_rejected() {
        let tx = valid_tx(me(), TransactionType::Payment);
        let result = validate_create_entry(
            create_action(other_agent()),
            EntryTypes::CreditTransaction(tx),
        )
        .unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn create_transaction_clearing_type_allows_third_party_from() {
        // Clearing transactions legitimately name debtor/creditor pairs
        // as from/to, not the triggering agent -- run_clearing's own
        // missing authorization is a separate, disclosed gap, not
        // identity forgery.
        let tx = valid_tx(third_agent(), TransactionType::Clearing);
        let result =
            validate_create_entry(create_action(me()), EntryTypes::CreditTransaction(tx)).unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn update_entry_type_rejects_credit_circle_update() {
        let circle = valid_circle(vec![me()]);
        let result = validate_update_entry_type(
            update_action(me()),
            ActionHash::from_raw_36(vec![9u8; 36]),
            EntryTypes::CreditCircle(circle),
        )
        .unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn update_entry_type_rejects_credit_transaction_update() {
        let tx = valid_tx(me(), TransactionType::Payment);
        let result = validate_update_entry_type(
            update_action(me()),
            ActionHash::from_raw_36(vec![9u8; 36]),
            EntryTypes::CreditTransaction(tx),
        )
        .unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn update_entry_type_rejects_balance_update() {
        let balance = Balance {
            member: me(),
            circle_hash: ActionHash::from_raw_36(vec![2u8; 36]),
            balance: 0,
            credit_available: 100,
            as_of: Timestamp::from_micros(0),
        };
        let result = validate_update_entry_type(
            update_action(me()),
            ActionHash::from_raw_36(vec![9u8; 36]),
            EntryTypes::Balance(balance),
        )
        .unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }
}
