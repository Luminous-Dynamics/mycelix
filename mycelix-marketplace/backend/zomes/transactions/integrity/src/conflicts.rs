use super::Transaction;
use hdi::prelude::*;

pub const TRANSACTION_CONFLICT_PROTOCOL_VERSION: u16 = 1;
const MAX_CONFLICT_HEADS: usize = 16;
const MAX_REVISION_DEPTH: usize = 32;

/// One party's immutable approval of one existing transaction branch.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct TransactionConflictApproval {
    pub protocol_version: u16,
    pub root_transaction_hash: ActionHash,
    pub head_hashes: Vec<ActionHash>,
    pub selected_head_hash: ActionHash,
    pub approver: AgentPubKey,
    pub rationale: String,
    pub created_at: Timestamp,
}

/// Authority evidence used by a final conflict-resolution entry.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum TransactionConflictAuthority {
    Bilateral {
        buyer_approval_hash: ActionHash,
        seller_approval_hash: ActionHash,
    },
    Arbitration {
        dispute_hash: ActionHash,
        result_hash: ActionHash,
    },
}

/// Immutable authority for projecting one already-authored transaction branch.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct TransactionConflictResolutionEntry {
    pub protocol_version: u16,
    pub root_transaction_hash: ActionHash,
    pub head_hashes: Vec<ActionHash>,
    pub selected_head_hash: ActionHash,
    pub authority: TransactionConflictAuthority,
    pub summary: String,
    pub created_at: Timestamp,
}

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
struct ConflictDisputeSnapshot {
    transaction_hash: ActionHash,
    buyer: AgentPubKey,
    seller: AgentPubKey,
    arbitrators: Vec<AgentPubKey>,
    #[serde(default)]
    conflict_heads: Vec<ActionHash>,
    status: ConflictDisputeStatusSnapshot,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
#[serde(rename_all = "lowercase")]
enum ConflictDisputeStatusSnapshot {
    Filed,
    UnderReview,
    Voting,
    ResolvedBuyer,
    ResolvedSeller,
    Withdrawn,
}

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
struct ArbitrationResultSnapshot {
    dispute_hash: ActionHash,
    dispute_revision_hash: ActionHash,
    winner: AgentPubKey,
    loser: AgentPubKey,
}

pub fn validate_create_conflict_approval(
    approval: &TransactionConflictApproval,
    action: &Create,
) -> ExternResult<ValidateCallbackResult> {
    let transaction = match validate_conflict_binding(
        approval.protocol_version,
        &approval.root_transaction_hash,
        &approval.head_hashes,
        &approval.selected_head_hash,
    )? {
        Ok(transaction) => transaction,
        Err(reason) => return Ok(ValidateCallbackResult::Invalid(reason)),
    };

    if action.author != approval.approver {
        return Ok(ValidateCallbackResult::Invalid(
            "Conflict approval must be authored by its declared approver".into(),
        ));
    }
    if approval.approver != transaction.buyer && approval.approver != transaction.seller {
        return Ok(ValidateCallbackResult::Invalid(
            "Only the authoritative buyer or seller may approve a conflict resolution".into(),
        ));
    }
    if approval.rationale.trim().is_empty() || approval.rationale.len() > 2000 {
        return Ok(ValidateCallbackResult::Invalid(
            "Conflict approval rationale must be 1-2000 characters".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

pub fn validate_create_conflict_resolution(
    resolution: &TransactionConflictResolutionEntry,
    action: &Create,
) -> ExternResult<ValidateCallbackResult> {
    let transaction = match validate_conflict_binding(
        resolution.protocol_version,
        &resolution.root_transaction_hash,
        &resolution.head_hashes,
        &resolution.selected_head_hash,
    )? {
        Ok(transaction) => transaction,
        Err(reason) => return Ok(ValidateCallbackResult::Invalid(reason)),
    };

    if resolution.summary.trim().is_empty() || resolution.summary.len() > 2000 {
        return Ok(ValidateCallbackResult::Invalid(
            "Conflict resolution summary must be 1-2000 characters".into(),
        ));
    }

    let authority_result = match &resolution.authority {
        TransactionConflictAuthority::Bilateral {
            buyer_approval_hash,
            seller_approval_hash,
        } => validate_bilateral_authority(
            resolution,
            action,
            &transaction,
            buyer_approval_hash,
            seller_approval_hash,
        )?,
        TransactionConflictAuthority::Arbitration {
            dispute_hash,
            result_hash,
        } => validate_arbitration_authority(
            resolution,
            action,
            &transaction,
            dispute_hash,
            result_hash,
        )?,
    };

    match authority_result {
        Ok(()) => Ok(ValidateCallbackResult::Valid),
        Err(reason) => Ok(ValidateCallbackResult::Invalid(reason)),
    }
}

fn validate_bilateral_authority(
    resolution: &TransactionConflictResolutionEntry,
    action: &Create,
    transaction: &Transaction,
    buyer_approval_hash: &ActionHash,
    seller_approval_hash: &ActionHash,
) -> ExternResult<Result<(), String>> {
    if buyer_approval_hash == seller_approval_hash {
        return Ok(Err("Bilateral resolution requires two distinct approvals".into()));
    }
    if action.author != transaction.buyer && action.author != transaction.seller {
        return Ok(Err(
            "Only a transaction party may publish the bilateral resolution".into(),
        ));
    }

    let buyer_approval: TransactionConflictApproval =
        decode_record(&must_get_valid_record(buyer_approval_hash.clone())?)?;
    let seller_approval: TransactionConflictApproval =
        decode_record(&must_get_valid_record(seller_approval_hash.clone())?)?;

    if buyer_approval.approver != transaction.buyer
        || seller_approval.approver != transaction.seller
    {
        return Ok(Err(
            "Bilateral resolution must bind one buyer approval and one seller approval".into(),
        ));
    }
    if !approval_matches_resolution(&buyer_approval, resolution)
        || !approval_matches_resolution(&seller_approval, resolution)
    {
        return Ok(Err(
            "Buyer and seller approvals must bind the exact same conflict and selected head"
                .into(),
        ));
    }

    Ok(Ok(()))
}

fn validate_arbitration_authority(
    resolution: &TransactionConflictResolutionEntry,
    action: &Create,
    transaction: &Transaction,
    dispute_hash: &ActionHash,
    result_hash: &ActionHash,
) -> ExternResult<Result<(), String>> {
    let result: ArbitrationResultSnapshot =
        decode_record(&must_get_valid_record(result_hash.clone())?)?;
    if result.dispute_hash != *dispute_hash
        || find_action_root(result.dispute_revision_hash.clone())? != *dispute_hash
    {
        return Ok(Err(
            "Arbitration result belongs to a different dispute".into(),
        ));
    }
    let dispute: ConflictDisputeSnapshot = decode_record(&must_get_valid_record(
        result.dispute_revision_hash.clone(),
    )?)?;

    if dispute.transaction_hash != resolution.root_transaction_hash
        || dispute.buyer != transaction.buyer
        || dispute.seller != transaction.seller
    {
        return Ok(Err(
            "Arbitration dispute is not bound to the authoritative transaction parties".into(),
        ));
    }
    if dispute.conflict_heads != resolution.head_hashes
        || dispute.status != ConflictDisputeStatusSnapshot::Voting
    {
        return Ok(Err(
            "Arbitration result must bind the exact conflict at its Voting revision".into(),
        ));
    }
    if result.winner != transaction.buyer && result.winner != transaction.seller {
        return Ok(Err(
            "Arbitration winner must be one of the transaction parties".into(),
        ));
    }
    if result.loser == result.winner
        || (result.loser != transaction.buyer && result.loser != transaction.seller)
    {
        return Ok(Err(
            "Arbitration loser must be the other transaction party".into(),
        ));
    }
    if !dispute.arbitrators.contains(&action.author) {
        return Ok(Err(
            "Only an assigned arbitrator may publish an arbitration conflict resolution".into(),
        ));
    }

    let mut winner_heads = Vec::new();
    for head_hash in &resolution.head_hashes {
        let record = must_get_valid_record(head_hash.clone())?;
        if record.action().author() == &result.winner {
            winner_heads.push(head_hash.clone());
        }
    }
    if winner_heads.len() != 1 || winner_heads.first() != Some(&resolution.selected_head_hash) {
        return Ok(Err(
            "Arbitration may select only the unique conflict head authored by the result winner"
                .into(),
        ));
    }

    Ok(Ok(()))
}

fn approval_matches_resolution(
    approval: &TransactionConflictApproval,
    resolution: &TransactionConflictResolutionEntry,
) -> bool {
    approval.protocol_version == resolution.protocol_version
        && approval.root_transaction_hash == resolution.root_transaction_hash
        && approval.head_hashes == resolution.head_hashes
        && approval.selected_head_hash == resolution.selected_head_hash
}

fn validate_conflict_binding(
    protocol_version: u16,
    root_transaction_hash: &ActionHash,
    head_hashes: &[ActionHash],
    selected_head_hash: &ActionHash,
) -> ExternResult<Result<Transaction, String>> {
    if protocol_version != TRANSACTION_CONFLICT_PROTOCOL_VERSION {
        return Ok(Err(format!(
            "Unsupported transaction conflict protocol version {protocol_version}"
        )));
    }
    if !(2..=MAX_CONFLICT_HEADS).contains(&head_hashes.len()) {
        return Ok(Err(format!(
            "Conflict binding requires 2-{MAX_CONFLICT_HEADS} heads"
        )));
    }
    if !is_sorted_unique(head_hashes) {
        return Ok(Err(
            "Conflict head hashes must be sorted and duplicate-free".into(),
        ));
    }
    if !head_hashes.contains(selected_head_hash) {
        return Ok(Err(
            "Selected transaction head must be present in the bound head set".into(),
        ));
    }

    let root_record = must_get_valid_record(root_transaction_hash.clone())?;
    if !matches!(root_record.action(), Action::Create(_)) {
        return Ok(Err(
            "Conflict root must be the original Transaction Create action".into(),
        ));
    }
    let root_transaction: Transaction = decode_record(&root_record)?;

    for head_hash in head_hashes {
        if find_action_root(head_hash.clone())? != *root_transaction_hash {
            return Ok(Err(
                "Every conflict head must belong to the declared transaction root".into(),
            ));
        }
        let transaction: Transaction =
            decode_record(&must_get_valid_record(head_hash.clone())?)?;
        if !same_immutable_identity(&root_transaction, &transaction) {
            return Ok(Err(
                "Every conflict head must share the immutable transaction identity".into(),
            ));
        }
    }

    Ok(Ok(root_transaction))
}

fn same_immutable_identity(left: &Transaction, right: &Transaction) -> bool {
    left.buyer == right.buyer
        && left.seller == right.seller
        && left.listing_hash == right.listing_hash
        && left.quantity == right.quantity
        && left.total_price_cents == right.total_price_cents
        && left.created_at == right.created_at
        && left.epistemic == right.epistemic
}

fn is_sorted_unique(hashes: &[ActionHash]) -> bool {
    hashes.windows(2).all(|pair| pair[0].to_string() < pair[1].to_string())
}

fn decode_record<T>(record: &Record) -> ExternResult<T>
where
    T: TryFrom<SerializedBytes, Error = SerializedBytesError>,
{
    record
        .entry()
        .to_app_option()
        .map_err(|error| wasm_error!(error))?
        .ok_or_else(|| wasm_error!(WasmErrorInner::Guest("Application entry missing".into())))
}

fn find_action_root(mut cursor: ActionHash) -> ExternResult<ActionHash> {
    let mut visited = Vec::new();
    for _ in 0..=MAX_REVISION_DEPTH {
        if visited.contains(&cursor) {
            return Err(wasm_error!(WasmErrorInner::Guest(
                "Transaction update ancestry contains a cycle".into(),
            )));
        }
        visited.push(cursor.clone());
        let record = must_get_valid_record(cursor.clone())?;
        match record.action() {
            Action::Create(_) => return Ok(cursor),
            Action::Update(update) => cursor = update.original_action_address.clone(),
            _ => {
                return Err(wasm_error!(WasmErrorInner::Guest(
                    "Conflict head must reference a Create or Update action".into(),
                )))
            }
        }
    }
    Err(wasm_error!(WasmErrorInner::Guest(
        "Transaction update ancestry exceeds the depth limit".into(),
    )))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn hash(byte: u8) -> ActionHash {
        ActionHash::from_raw_36(vec![byte; 36])
    }

    #[test]
    fn head_bindings_must_be_sorted_unique() {
        assert!(is_sorted_unique(&[hash(1), hash(2)]));
        assert!(!is_sorted_unique(&[hash(2), hash(1)]));
        assert!(!is_sorted_unique(&[hash(1), hash(1)]));
    }

    #[test]
    fn approvals_match_only_exact_resolution_binding() {
        let approval = TransactionConflictApproval {
            protocol_version: 1,
            root_transaction_hash: hash(1),
            head_hashes: vec![hash(2), hash(3)],
            selected_head_hash: hash(2),
            approver: AgentPubKey::from_raw_36(vec![4; 36]),
            rationale: "Agree".into(),
            created_at: Timestamp::from_micros(1),
        };
        let resolution = TransactionConflictResolutionEntry {
            protocol_version: 1,
            root_transaction_hash: hash(1),
            head_hashes: vec![hash(2), hash(3)],
            selected_head_hash: hash(2),
            authority: TransactionConflictAuthority::Bilateral {
                buyer_approval_hash: hash(5),
                seller_approval_hash: hash(6),
            },
            summary: "Both parties agree".into(),
            created_at: Timestamp::from_micros(2),
        };
        assert!(approval_matches_resolution(&approval, &resolution));
    }
}
