use super::{TransactionOutput, resolution::TRANSACTION_CONFLICT_POLICY_VERSION};
use hdk::prelude::*;
use mycelix_common::{error_handling, link_queries, time};
use transactions_integrity::{
    EntryTypes, LinkTypes, TRANSACTION_CONFLICT_PROTOCOL_VERSION, TransactionConflictApproval,
    TransactionConflictAuthority, TransactionConflictResolutionEntry,
};

const MAX_ANCESTRY_DEPTH: usize = 32;

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct TransactionConflictApprovalOutput {
    pub approval_hash: ActionHash,
    pub approval: TransactionConflictApproval,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct TransactionConflictResolutionOutput {
    pub resolution_hash: ActionHash,
    pub resolution: TransactionConflictResolutionEntry,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct AppliedTransactionConflictResolution {
    pub resolution_hash: ActionHash,
    pub protocol_version: u16,
    pub selected_head_hash: ActionHash,
    pub bound_head_hashes: Vec<ActionHash>,
    pub authority: TransactionConflictAuthority,
}

pub enum AuthorityProjection {
    None,
    Resolved {
        canonical_index: usize,
        applied: Vec<AppliedTransactionConflictResolution>,
    },
    Conflicting {
        applied: Vec<AppliedTransactionConflictResolution>,
    },
}

pub fn project_authorized_resolution(
    root_transaction_hash: &ActionHash,
    heads: &[TransactionOutput],
) -> ExternResult<AuthorityProjection> {
    let decisions = get_conflict_resolutions(root_transaction_hash.clone())?;
    if decisions.is_empty() {
        return Ok(AuthorityProjection::None);
    }

    let current_hashes = current_head_hashes(heads);
    let mut exact = Vec::new();
    let mut historical = Vec::new();
    let mut blocked = Vec::new();

    for decision in decisions {
        if decision.resolution.protocol_version != TRANSACTION_CONFLICT_PROTOCOL_VERSION
            || decision.resolution.root_transaction_hash != *root_transaction_hash
        {
            continue;
        }
        let evidence = AppliedTransactionConflictResolution {
            resolution_hash: decision.resolution_hash,
            protocol_version: decision.resolution.protocol_version,
            selected_head_hash: decision.resolution.selected_head_hash.clone(),
            bound_head_hashes: decision.resolution.head_hashes.clone(),
            authority: decision.resolution.authority,
        };
        match applicable_canonical_index(&decision.resolution, heads)? {
            Some(canonical_index) if decision.resolution.head_hashes == current_hashes => {
                exact.push((canonical_index, evidence));
            }
            Some(canonical_index) => historical.push((canonical_index, evidence)),
            None => blocked.push(evidence),
        }
    }

    // A decision bound to the exact current head set is fresh authority for a
    // later fork. It may supersede older blocked projections without erasing
    // their immutable records from the DHT.
    if !exact.is_empty() {
        return projection_from_applicable(exact);
    }

    if !blocked.is_empty() {
        blocked.extend(historical.into_iter().map(|(_, evidence)| evidence));
        blocked.sort_by_key(|evidence| evidence.resolution_hash.to_string());
        return Ok(AuthorityProjection::Conflicting { applied: blocked });
    }

    if historical.is_empty() {
        Ok(AuthorityProjection::None)
    } else {
        projection_from_applicable(historical)
    }
}

fn projection_from_applicable(
    mut applicable: Vec<(usize, AppliedTransactionConflictResolution)>,
) -> ExternResult<AuthorityProjection> {
    applicable.sort_by(|left, right| {
        left.1
            .resolution_hash
            .to_string()
            .cmp(&right.1.resolution_hash.to_string())
    });
    let canonical_index = applicable[0].0;
    let applied = applicable
        .iter()
        .map(|(_, evidence)| evidence.clone())
        .collect::<Vec<_>>();
    if applicable
        .iter()
        .all(|(index, _)| *index == canonical_index)
    {
        Ok(AuthorityProjection::Resolved {
            canonical_index,
            applied,
        })
    } else {
        Ok(AuthorityProjection::Conflicting { applied })
    }
}

fn applicable_canonical_index(
    resolution: &TransactionConflictResolutionEntry,
    heads: &[TransactionOutput],
) -> ExternResult<Option<usize>> {
    if resolution.head_hashes.len() < 2
        || !resolution
            .head_hashes
            .contains(&resolution.selected_head_hash)
    {
        return Ok(None);
    }

    let mut selected_descendants = Vec::new();
    for (current_index, current) in heads.iter().enumerate() {
        let mut bound_ancestors = Vec::new();
        for bound in &resolution.head_hashes {
            if is_descendant_or_same(&current.transaction_hash, bound)? {
                bound_ancestors.push(bound);
            }
        }
        if bound_ancestors.len() != 1 {
            return Ok(None);
        }
        if *bound_ancestors[0] == resolution.selected_head_hash {
            selected_descendants.push(current_index);
        }
    }

    if selected_descendants.len() == 1 {
        Ok(selected_descendants.first().copied())
    } else {
        // Zero selected descendants means incomplete evidence. More than one
        // means the selected branch itself forked and requires fresh authority.
        Ok(None)
    }
}

fn is_descendant_or_same(cursor: &ActionHash, ancestor: &ActionHash) -> ExternResult<bool> {
    let mut current = cursor.clone();
    let mut visited = Vec::new();
    for _ in 0..=MAX_ANCESTRY_DEPTH {
        if &current == ancestor {
            return Ok(true);
        }
        if visited.contains(&current) {
            return Err(wasm_error!(WasmErrorInner::Guest(
                "Transaction authority ancestry contains a cycle".into(),
            )));
        }
        visited.push(current.clone());
        let Some(record) = get(current.clone(), GetOptions::default())? else {
            return Ok(false);
        };
        match record.action() {
            Action::Create(_) => return Ok(false),
            Action::Update(update) => current = update.original_action_address.clone(),
            _ => return Ok(false),
        }
    }
    Err(wasm_error!(WasmErrorInner::Guest(
        "Transaction authority ancestry exceeds the depth limit".into(),
    )))
}

pub fn create_conflict_approval(
    root_transaction_hash: ActionHash,
    head_hashes: Vec<ActionHash>,
    selected_head_hash: ActionHash,
    rationale: String,
) -> ExternResult<TransactionConflictApprovalOutput> {
    let approver = agent_info()?.agent_initial_pubkey;
    let existing = get_conflict_approvals(root_transaction_hash.clone())?;
    for output in existing {
        if output.approval.approver == approver && output.approval.head_hashes == head_hashes {
            if output.approval.selected_head_hash == selected_head_hash {
                return Ok(output);
            }
            return Err(wasm_error!(WasmErrorInner::Guest(
                "This party already approved a different branch for the same conflict".into(),
            )));
        }
    }

    let approval = TransactionConflictApproval {
        protocol_version: TRANSACTION_CONFLICT_PROTOCOL_VERSION,
        root_transaction_hash: root_transaction_hash.clone(),
        head_hashes,
        selected_head_hash,
        approver,
        rationale,
        created_at: time::now()?,
    };
    let approval_hash = create_entry(&EntryTypes::TransactionConflictApproval(approval.clone()))?;
    create_link(
        root_transaction_hash,
        approval_hash.clone(),
        LinkTypes::TransactionToConflictApprovals,
        (),
    )?;
    Ok(TransactionConflictApprovalOutput {
        approval_hash,
        approval,
    })
}

pub fn create_conflict_resolution(
    root_transaction_hash: ActionHash,
    head_hashes: Vec<ActionHash>,
    selected_head_hash: ActionHash,
    authority: TransactionConflictAuthority,
    summary: String,
) -> ExternResult<TransactionConflictResolutionOutput> {
    let existing = get_conflict_resolutions(root_transaction_hash.clone())?;
    for output in existing {
        if output.resolution.head_hashes == head_hashes {
            if output.resolution.selected_head_hash == selected_head_hash
                && output.resolution.authority == authority
            {
                return Ok(output);
            }
            if output.resolution.selected_head_hash != selected_head_hash {
                return Err(wasm_error!(WasmErrorInner::Guest(
                    "A conflicting authorized resolution already exists for this head set".into(),
                )));
            }
        }
    }

    let resolution = TransactionConflictResolutionEntry {
        protocol_version: TRANSACTION_CONFLICT_PROTOCOL_VERSION,
        root_transaction_hash: root_transaction_hash.clone(),
        head_hashes,
        selected_head_hash,
        authority,
        summary,
        created_at: time::now()?,
    };
    let resolution_hash = create_entry(&EntryTypes::TransactionConflictResolution(
        resolution.clone(),
    ))?;
    create_link(
        root_transaction_hash,
        resolution_hash.clone(),
        LinkTypes::TransactionToConflictResolutions,
        (),
    )?;
    Ok(TransactionConflictResolutionOutput {
        resolution_hash,
        resolution,
    })
}

pub fn get_conflict_approval(
    approval_hash: ActionHash,
) -> ExternResult<Option<TransactionConflictApprovalOutput>> {
    let Some(record) = get(approval_hash.clone(), GetOptions::default())? else {
        return Ok(None);
    };
    let approval = match error_handling::deserialize_entry::<TransactionConflictApproval>(&record) {
        Ok(approval) => approval,
        Err(_) => return Ok(None),
    };
    Ok(Some(TransactionConflictApprovalOutput {
        approval_hash,
        approval,
    }))
}

pub fn get_conflict_resolution(
    resolution_hash: ActionHash,
) -> ExternResult<Option<TransactionConflictResolutionOutput>> {
    let Some(record) = get(resolution_hash.clone(), GetOptions::default())? else {
        return Ok(None);
    };
    let resolution =
        match error_handling::deserialize_entry::<TransactionConflictResolutionEntry>(&record) {
            Ok(resolution) => resolution,
            Err(_) => return Ok(None),
        };
    Ok(Some(TransactionConflictResolutionOutput {
        resolution_hash,
        resolution,
    }))
}

pub fn get_conflict_approvals(
    root_transaction_hash: ActionHash,
) -> ExternResult<Vec<TransactionConflictApprovalOutput>> {
    let links = link_queries::get_links_local(
        root_transaction_hash.clone(),
        LinkTypes::TransactionToConflictApprovals,
    )?;
    let mut outputs = Vec::new();
    for link in links {
        let Some(hash) = link.target.into_action_hash() else {
            continue;
        };
        let Some(record) = get(hash.clone(), GetOptions::default())? else {
            continue;
        };
        let Ok(approval) =
            error_handling::deserialize_entry::<TransactionConflictApproval>(&record)
        else {
            continue;
        };
        if approval.root_transaction_hash == root_transaction_hash {
            outputs.push(TransactionConflictApprovalOutput {
                approval_hash: hash,
                approval,
            });
        }
    }
    outputs.sort_by_key(|output| output.approval_hash.to_string());
    Ok(outputs)
}

pub fn get_conflict_resolutions(
    root_transaction_hash: ActionHash,
) -> ExternResult<Vec<TransactionConflictResolutionOutput>> {
    let links = link_queries::get_links_local(
        root_transaction_hash.clone(),
        LinkTypes::TransactionToConflictResolutions,
    )?;
    let mut outputs = Vec::new();
    for link in links {
        let Some(hash) = link.target.into_action_hash() else {
            continue;
        };
        let Some(record) = get(hash.clone(), GetOptions::default())? else {
            continue;
        };
        let Ok(resolution) =
            error_handling::deserialize_entry::<TransactionConflictResolutionEntry>(&record)
        else {
            continue;
        };
        if resolution.root_transaction_hash == root_transaction_hash {
            outputs.push(TransactionConflictResolutionOutput {
                resolution_hash: hash,
                resolution,
            });
        }
    }
    outputs.sort_by_key(|output| output.resolution_hash.to_string());
    Ok(outputs)
}

pub fn current_head_hashes(heads: &[TransactionOutput]) -> Vec<ActionHash> {
    let mut hashes = heads
        .iter()
        .map(|output| output.transaction_hash.clone())
        .collect::<Vec<_>>();
    hashes.sort_by_key(ToString::to_string);
    hashes.dedup();
    hashes
}

pub fn selected_head<'a>(
    heads: &'a [TransactionOutput],
    selected_head_hash: &ActionHash,
) -> Option<&'a TransactionOutput> {
    heads
        .iter()
        .find(|head| &head.transaction_hash == selected_head_hash)
}

pub fn ensure_resolution_policy_version(policy_version: u16) -> ExternResult<()> {
    if policy_version != TRANSACTION_CONFLICT_POLICY_VERSION {
        return Err(wasm_error!(WasmErrorInner::Guest(format!(
            "Transaction resolution policy version changed during conflict authorization: {policy_version}"
        ))));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn hash(byte: u8) -> ActionHash {
        ActionHash::from_raw_36(vec![byte; 36])
    }

    fn evidence(byte: u8) -> AppliedTransactionConflictResolution {
        AppliedTransactionConflictResolution {
            resolution_hash: hash(byte),
            protocol_version: TRANSACTION_CONFLICT_PROTOCOL_VERSION,
            selected_head_hash: hash(20),
            bound_head_hashes: vec![hash(10), hash(20)],
            authority: TransactionConflictAuthority::Bilateral {
                buyer_approval_hash: hash(30),
                seller_approval_hash: hash(31),
            },
        }
    }

    #[test]
    fn convergent_authorities_select_one_projection() {
        let projection = projection_from_applicable(vec![(1, evidence(1)), (1, evidence(2))])
            .expect("projection should be valid");
        match projection {
            AuthorityProjection::Resolved {
                canonical_index,
                applied,
            } => {
                assert_eq!(canonical_index, 1);
                assert_eq!(applied.len(), 2);
            }
            _ => panic!("convergent authorities must resolve"),
        }
    }

    #[test]
    fn disagreeing_authorities_remain_conflicted() {
        let projection = projection_from_applicable(vec![(0, evidence(1)), (1, evidence(2))])
            .expect("projection should be valid");
        assert!(matches!(
            projection,
            AuthorityProjection::Conflicting { .. }
        ));
    }
}
