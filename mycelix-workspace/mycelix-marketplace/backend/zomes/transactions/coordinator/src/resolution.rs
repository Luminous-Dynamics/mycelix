use super::{Transaction, TransactionOutput, authority};
use hdk::prelude::*;
use mycelix_common::error_handling;
use transactions_integrity::TransactionStatus;

const MAX_TRANSACTION_REVISION_DEPTH: u16 = 32;
const MAX_TRANSACTION_REVISIONS: usize = 128;
pub const TRANSACTION_CONFLICT_POLICY_VERSION: u16 = 2;

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum TransactionResolutionState {
    Resolved,
    AutoResolved,
    AuthorizedResolved,
    Conflicted,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum TransactionResolutionReason {
    SingleHead,
    CancellationDominatesPreShipment,
    DisputeDominatesLifecycle,
    BilateralAgreement,
    ArbitrationAward,
    ConvergentExplicitAuthorities,
    ConflictingExplicitAuthorities,
    UnsafeConcurrentLifecycle,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct TransactionResolution {
    /// Stable identity of the transaction: the original Create action.
    pub root_transaction_hash: ActionHash,
    /// Versioned policy used to project the raw update leaves.
    pub policy_version: u16,
    /// A single leaf is resolved, a narrow safety rule may project one authored
    /// terminal leaf, and every other multi-head state remains conflicted.
    pub state: TransactionResolutionState,
    /// Machine-readable explanation for the projection decision.
    pub reason: TransactionResolutionReason,
    /// Present only when one existing authored head is safe to treat as current.
    pub canonical: Option<TransactionOutput>,
    /// Every locally observed raw leaf, sorted by action hash.
    pub heads: Vec<TransactionOutput>,
    /// Raw leaves safely dominated by the canonical authored head. These remain
    /// visible audit evidence and are never deleted or rewritten.
    pub superseded_heads: Vec<TransactionOutput>,
    /// Explicit authority records applied to this projection. Empty for one-head
    /// and automatic safety projections.
    pub applied_conflict_resolutions: Vec<authority::AppliedTransactionConflictResolution>,
    /// Number of Create/Update records traversed while reducing the tree.
    pub revision_count: u32,
}

impl TransactionResolution {
    pub fn resolved(&self) -> Option<&TransactionOutput> {
        match self.state {
            TransactionResolutionState::Resolved
            | TransactionResolutionState::AutoResolved
            | TransactionResolutionState::AuthorizedResolved => self.canonical.as_ref(),
            TransactionResolutionState::Conflicted => None,
        }
    }

    pub fn into_resolved(self) -> Result<TransactionOutput, String> {
        match (self.state, self.canonical) {
            (
                TransactionResolutionState::Resolved
                | TransactionResolutionState::AutoResolved
                | TransactionResolutionState::AuthorizedResolved,
                Some(output),
            ) => Ok(output),
            (
                TransactionResolutionState::Resolved
                | TransactionResolutionState::AutoResolved
                | TransactionResolutionState::AuthorizedResolved,
                None,
            ) => Err("Resolved transaction omitted its canonical revision".into()),
            (TransactionResolutionState::Conflicted, _) => Err(format!(
                "Transaction update conflict: {} concurrent heads require explicit resolution",
                self.heads.len()
            )),
        }
    }
}

/// Resolve an arbitrary Create/Update action to the locally observed transaction tree.
///
/// Holochain CRUD updates form a tree and are not automatically followed by `get`.
/// Marketplace policy preserves every raw leaf. One leaf is current; a narrow
/// authored-head safety policy may project cancellation before shipment or a
/// dispute that halts ordinary progression. Every other multi-head state remains
/// an explicit conflict.
pub fn resolve_transaction(
    transaction_hash: ActionHash,
) -> ExternResult<Option<TransactionResolution>> {
    let root_transaction_hash = match find_root(transaction_hash)? {
        Some(root) => root,
        None => return Ok(None),
    };

    let mut visited = Vec::new();
    let mut heads = Vec::new();
    collect_heads(root_transaction_hash.clone(), 0, &mut visited, &mut heads)?;

    heads.sort_by(|left, right| {
        left.transaction_hash
            .to_string()
            .cmp(&right.transaction_hash.to_string())
    });

    let projection = match authority::project_authorized_resolution(&root_transaction_hash, &heads)?
    {
        authority::AuthorityProjection::None => {
            reduce_heads(&heads).map_err(|reason| wasm_error!(WasmErrorInner::Guest(reason)))?
        }
        authority::AuthorityProjection::Resolved {
            canonical_index,
            applied,
        } => authorized_projection(&heads, canonical_index, applied),
        authority::AuthorityProjection::Conflicting { applied } => HeadProjection {
            state: TransactionResolutionState::Conflicted,
            reason: TransactionResolutionReason::ConflictingExplicitAuthorities,
            canonical: None,
            superseded_heads: Vec::new(),
            applied_conflict_resolutions: applied,
        },
    };

    Ok(Some(TransactionResolution {
        root_transaction_hash,
        policy_version: TRANSACTION_CONFLICT_POLICY_VERSION,
        state: projection.state,
        reason: projection.reason,
        canonical: projection.canonical,
        heads,
        superseded_heads: projection.superseded_heads,
        applied_conflict_resolutions: projection.applied_conflict_resolutions,
        revision_count: u32::try_from(visited.len()).unwrap_or(u32::MAX),
    }))
}

struct HeadProjection {
    state: TransactionResolutionState,
    reason: TransactionResolutionReason,
    canonical: Option<TransactionOutput>,
    superseded_heads: Vec<TransactionOutput>,
    applied_conflict_resolutions: Vec<authority::AppliedTransactionConflictResolution>,
}

fn reduce_heads(heads: &[TransactionOutput]) -> Result<HeadProjection, String> {
    match heads.len() {
        0 => return Err("Transaction update tree has no live heads".into()),
        1 => {
            return Ok(HeadProjection {
                state: TransactionResolutionState::Resolved,
                reason: TransactionResolutionReason::SingleHead,
                canonical: heads.first().cloned(),
                superseded_heads: Vec::new(),
                applied_conflict_resolutions: Vec::new(),
            });
        }
        _ => {}
    }

    if !heads_share_identity(heads) {
        return Ok(conflicted_projection());
    }

    if let Some(index) = unique_head_with_status(heads, TransactionStatus::Cancelled) {
        let safe = heads.iter().enumerate().all(|(candidate_index, head)| {
            candidate_index == index
                || matches!(
                    head.transaction.status,
                    TransactionStatus::Pending | TransactionStatus::Confirmed
                )
        });
        if safe {
            return Ok(auto_resolved_projection(
                heads,
                index,
                TransactionResolutionReason::CancellationDominatesPreShipment,
            ));
        }
    }

    if let Some(index) = unique_head_with_status(heads, TransactionStatus::Disputed) {
        let safe = heads.iter().enumerate().all(|(candidate_index, head)| {
            candidate_index == index
                || matches!(
                    head.transaction.status,
                    TransactionStatus::Pending
                        | TransactionStatus::Confirmed
                        | TransactionStatus::Shipped
                        | TransactionStatus::Delivered
                )
        });
        if safe {
            return Ok(auto_resolved_projection(
                heads,
                index,
                TransactionResolutionReason::DisputeDominatesLifecycle,
            ));
        }
    }

    Ok(conflicted_projection())
}

fn heads_share_identity(heads: &[TransactionOutput]) -> bool {
    let Some(first) = heads.first() else {
        return false;
    };
    heads
        .iter()
        .skip(1)
        .all(|candidate| same_immutable_identity(&first.transaction, &candidate.transaction))
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

fn unique_head_with_status(
    heads: &[TransactionOutput],
    expected: TransactionStatus,
) -> Option<usize> {
    let mut matches = heads
        .iter()
        .enumerate()
        .filter(|(_, head)| head.transaction.status == expected)
        .map(|(index, _)| index);
    let first = matches.next()?;
    if matches.next().is_some() {
        return None;
    }
    Some(first)
}

fn auto_resolved_projection(
    heads: &[TransactionOutput],
    canonical_index: usize,
    reason: TransactionResolutionReason,
) -> HeadProjection {
    HeadProjection {
        state: TransactionResolutionState::AutoResolved,
        reason,
        canonical: heads.get(canonical_index).cloned(),
        superseded_heads: heads
            .iter()
            .enumerate()
            .filter(|(index, _)| *index != canonical_index)
            .map(|(_, head)| head.clone())
            .collect(),
        applied_conflict_resolutions: Vec::new(),
    }
}

fn authorized_projection(
    heads: &[TransactionOutput],
    canonical_index: usize,
    applied: Vec<authority::AppliedTransactionConflictResolution>,
) -> HeadProjection {
    let has_bilateral = applied.iter().any(|evidence| {
        matches!(
            &evidence.authority,
            transactions_integrity::TransactionConflictAuthority::Bilateral { .. }
        )
    });
    let has_arbitration = applied.iter().any(|evidence| {
        matches!(
            &evidence.authority,
            transactions_integrity::TransactionConflictAuthority::Arbitration { .. }
        )
    });
    let reason = match (has_bilateral, has_arbitration) {
        (true, false) => TransactionResolutionReason::BilateralAgreement,
        (false, true) => TransactionResolutionReason::ArbitrationAward,
        _ => TransactionResolutionReason::ConvergentExplicitAuthorities,
    };
    HeadProjection {
        state: TransactionResolutionState::AuthorizedResolved,
        reason,
        canonical: heads.get(canonical_index).cloned(),
        superseded_heads: heads
            .iter()
            .enumerate()
            .filter(|(index, _)| *index != canonical_index)
            .map(|(_, head)| head.clone())
            .collect(),
        applied_conflict_resolutions: applied,
    }
}

fn conflicted_projection() -> HeadProjection {
    HeadProjection {
        state: TransactionResolutionState::Conflicted,
        reason: TransactionResolutionReason::UnsafeConcurrentLifecycle,
        canonical: None,
        superseded_heads: Vec::new(),
        applied_conflict_resolutions: Vec::new(),
    }
}

fn find_root(mut cursor: ActionHash) -> ExternResult<Option<ActionHash>> {
    let mut visited = Vec::new();

    for _ in 0..=MAX_TRANSACTION_REVISION_DEPTH {
        if visited.contains(&cursor) {
            return Err(wasm_error!(WasmErrorInner::Guest(
                "Transaction update ancestry contains a cycle".into()
            )));
        }
        visited.push(cursor.clone());

        let Some(record) = get(cursor.clone(), GetOptions::default())? else {
            if visited.len() == 1 {
                return Ok(None);
            }
            return Err(wasm_error!(WasmErrorInner::Guest(format!(
                "Transaction update ancestry is incomplete at {cursor}"
            ))));
        };

        match record.action() {
            Action::Create(_) => return Ok(Some(cursor)),
            Action::Update(update) => cursor = update.original_action_address.clone(),
            _ => {
                return Err(wasm_error!(WasmErrorInner::Guest(
                    "Transaction hash must reference a Create or Update action".into()
                )));
            }
        }
    }

    Err(wasm_error!(WasmErrorInner::Guest(format!(
        "Transaction update ancestry exceeds {MAX_TRANSACTION_REVISION_DEPTH} revisions"
    ))))
}

fn collect_heads(
    action_hash: ActionHash,
    depth: u16,
    visited: &mut Vec<ActionHash>,
    heads: &mut Vec<TransactionOutput>,
) -> ExternResult<()> {
    if depth > MAX_TRANSACTION_REVISION_DEPTH {
        return Err(wasm_error!(WasmErrorInner::Guest(format!(
            "Transaction update tree exceeds depth limit {MAX_TRANSACTION_REVISION_DEPTH}"
        ))));
    }
    if visited.len() >= MAX_TRANSACTION_REVISIONS {
        return Err(wasm_error!(WasmErrorInner::Guest(format!(
            "Transaction update tree exceeds revision limit {MAX_TRANSACTION_REVISIONS}"
        ))));
    }
    if visited.contains(&action_hash) {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Transaction update tree contains a cycle or duplicate edge".into()
        )));
    }
    visited.push(action_hash.clone());

    let details = get_details(action_hash.clone(), GetOptions::default())?.ok_or_else(|| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "Transaction revision {action_hash} is unavailable"
        )))
    })?;
    let record_details = match details {
        Details::Record(details) => details,
        Details::Entry(_) => {
            return Err(wasm_error!(WasmErrorInner::Guest(
                "Transaction revision lookup unexpectedly returned entry details".into()
            )));
        }
    };

    if !record_details.deletes.is_empty() {
        return Err(wasm_error!(WasmErrorInner::Guest(format!(
            "Transaction revision {action_hash} has been deleted; transaction deletion is forbidden"
        ))));
    }

    let transaction: Transaction = error_handling::deserialize_entry(&record_details.record)?;
    let mut child_hashes: Vec<ActionHash> = record_details
        .updates
        .into_iter()
        .map(|update| update.as_hash().clone())
        .collect();
    child_hashes.sort_by_key(ToString::to_string);
    child_hashes.dedup();

    if child_hashes.is_empty() {
        heads.push(TransactionOutput {
            transaction_hash: action_hash,
            transaction,
        });
        return Ok(());
    }

    for child_hash in child_hashes {
        collect_heads(child_hash, depth + 1, visited, heads)?;
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use transactions_integrity::{
        EmpiricalLevel, EpistemicClassification, MaterialityLevel, NormativeLevel,
    };

    fn agent(byte: u8) -> AgentPubKey {
        AgentPubKey::from_raw_36(vec![byte; 36])
    }

    fn output(byte: u8, status: TransactionStatus) -> TransactionOutput {
        TransactionOutput {
            transaction_hash: ActionHash::from_raw_36(vec![byte; 36]),
            transaction: Transaction {
                buyer: agent(1),
                seller: agent(2),
                listing_hash: ActionHash::from_raw_36(vec![3; 36]),
                quantity: 1,
                total_price_cents: 1_000,
                status,
                created_at: Timestamp::from_micros(1),
                updated_at: Timestamp::from_micros(i64::from(byte)),
                tracking_info: None,
                epistemic: EpistemicClassification {
                    empirical: EmpiricalLevel::E1Testimonial,
                    normative: NormativeLevel::N1Communal,
                    materiality: MaterialityLevel::M1Temporal,
                },
            },
        }
    }

    #[test]
    fn one_leaf_is_resolved() {
        let projection = reduce_heads(&[output(1, TransactionStatus::Pending)]).unwrap();
        assert_eq!(projection.state, TransactionResolutionState::Resolved);
        assert_eq!(projection.reason, TransactionResolutionReason::SingleHead);
    }

    #[test]
    fn cancellation_safely_dominates_pre_shipment_confirmation() {
        let heads = [
            output(1, TransactionStatus::Confirmed),
            output(2, TransactionStatus::Cancelled),
        ];
        let projection = reduce_heads(&heads).unwrap();
        assert_eq!(projection.state, TransactionResolutionState::AutoResolved);
        assert_eq!(
            projection.reason,
            TransactionResolutionReason::CancellationDominatesPreShipment
        );
        assert_eq!(
            projection.canonical.unwrap().transaction.status,
            TransactionStatus::Cancelled
        );
        assert_eq!(projection.superseded_heads.len(), 1);
    }

    #[test]
    fn dispute_safely_halts_a_shipment_branch() {
        let mut shipped = output(1, TransactionStatus::Shipped);
        shipped.transaction.tracking_info = Some("carrier-123".into());
        let heads = [shipped, output(2, TransactionStatus::Disputed)];
        let projection = reduce_heads(&heads).unwrap();
        assert_eq!(projection.state, TransactionResolutionState::AutoResolved);
        assert_eq!(
            projection.reason,
            TransactionResolutionReason::DisputeDominatesLifecycle
        );
    }

    #[test]
    fn shipped_versus_cancelled_remains_conflicted() {
        let mut shipped = output(1, TransactionStatus::Shipped);
        shipped.transaction.tracking_info = Some("carrier-123".into());
        let heads = [shipped, output(2, TransactionStatus::Cancelled)];
        let projection = reduce_heads(&heads).unwrap();
        assert_eq!(projection.state, TransactionResolutionState::Conflicted);
        assert!(projection.canonical.is_none());
    }

    #[test]
    fn two_terminal_heads_never_use_hash_order_as_authority() {
        let heads = [
            output(1, TransactionStatus::Cancelled),
            output(2, TransactionStatus::Cancelled),
        ];
        let projection = reduce_heads(&heads).unwrap();
        assert_eq!(projection.state, TransactionResolutionState::Conflicted);
    }

    #[test]
    fn mismatched_transaction_identity_never_auto_resolves() {
        let cancelled = output(1, TransactionStatus::Cancelled);
        let mut confirmed = output(2, TransactionStatus::Confirmed);
        confirmed.transaction.total_price_cents += 1;
        let projection = reduce_heads(&[cancelled, confirmed]).unwrap();
        assert_eq!(projection.state, TransactionResolutionState::Conflicted);
    }
}
