use super::{Dispute, DisputeOutput};
use hdk::prelude::*;
use mycelix_common::error_handling;

const MAX_DISPUTE_REVISION_DEPTH: u16 = 32;
const MAX_DISPUTE_REVISIONS: usize = 128;

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum DisputeResolutionState {
    Resolved,
    Conflicted,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct DisputeResolution {
    /// Stable identity of the dispute: the original Create action.
    pub root_dispute_hash: ActionHash,
    /// A single head is resolved; multiple heads are explicit conflict evidence.
    pub state: DisputeResolutionState,
    /// Present only when the locally observed update tree has one leaf.
    pub canonical: Option<DisputeOutput>,
    /// Every locally observed leaf, sorted deterministically by action hash.
    pub heads: Vec<DisputeOutput>,
    /// Number of Create/Update records traversed while reducing the tree.
    pub revision_count: u32,
}

impl DisputeResolution {
    pub fn into_resolved(self) -> Result<DisputeOutput, String> {
        match (self.state, self.canonical) {
            (DisputeResolutionState::Resolved, Some(output)) => Ok(output),
            (DisputeResolutionState::Resolved, None) => {
                Err("Resolved dispute omitted its canonical revision".into())
            }
            (DisputeResolutionState::Conflicted, _) => Err(format!(
                "Dispute update conflict: {} concurrent heads are visible",
                self.heads.len()
            )),
        }
    }
}

/// Resolve an arbitrary Create/Update action to the locally observed dispute tree.
pub fn resolve_dispute(dispute_hash: ActionHash) -> ExternResult<Option<DisputeResolution>> {
    let root_dispute_hash = match find_root(dispute_hash)? {
        Some(root) => root,
        None => return Ok(None),
    };

    let mut visited = Vec::new();
    let mut heads = Vec::new();
    collect_heads(root_dispute_hash.clone(), 0, &mut visited, &mut heads)?;

    heads.sort_by(|left, right| {
        left.dispute_hash
            .to_string()
            .cmp(&right.dispute_hash.to_string())
    });

    let state = match heads.len() {
        0 => {
            return Err(wasm_error!(WasmErrorInner::Guest(
                "Dispute update tree has no live heads".into()
            )))
        }
        1 => DisputeResolutionState::Resolved,
        _ => DisputeResolutionState::Conflicted,
    };
    let canonical = match state {
        DisputeResolutionState::Resolved => heads.first().cloned(),
        DisputeResolutionState::Conflicted => None,
    };

    Ok(Some(DisputeResolution {
        root_dispute_hash,
        state,
        canonical,
        heads,
        revision_count: u32::try_from(visited.len()).unwrap_or(u32::MAX),
    }))
}

fn find_root(mut cursor: ActionHash) -> ExternResult<Option<ActionHash>> {
    let mut visited = Vec::new();

    for _ in 0..=MAX_DISPUTE_REVISION_DEPTH {
        if visited.contains(&cursor) {
            return Err(wasm_error!(WasmErrorInner::Guest(
                "Dispute update ancestry contains a cycle".into()
            )));
        }
        visited.push(cursor.clone());

        let Some(record) = get(cursor.clone(), GetOptions::default())? else {
            if visited.len() == 1 {
                return Ok(None);
            }
            return Err(wasm_error!(WasmErrorInner::Guest(format!(
                "Dispute update ancestry is incomplete at {cursor}"
            ))));
        };

        match record.action() {
            Action::Create(_) => return Ok(Some(cursor)),
            Action::Update(update) => cursor = update.original_action_address.clone(),
            _ => {
                return Err(wasm_error!(WasmErrorInner::Guest(
                    "Dispute hash must reference a Create or Update action".into()
                )))
            }
        }
    }

    Err(wasm_error!(WasmErrorInner::Guest(format!(
        "Dispute update ancestry exceeds {MAX_DISPUTE_REVISION_DEPTH} revisions"
    ))))
}

fn collect_heads(
    action_hash: ActionHash,
    depth: u16,
    visited: &mut Vec<ActionHash>,
    heads: &mut Vec<DisputeOutput>,
) -> ExternResult<()> {
    if depth > MAX_DISPUTE_REVISION_DEPTH {
        return Err(wasm_error!(WasmErrorInner::Guest(format!(
            "Dispute update tree exceeds depth limit {MAX_DISPUTE_REVISION_DEPTH}"
        ))));
    }
    if visited.len() >= MAX_DISPUTE_REVISIONS {
        return Err(wasm_error!(WasmErrorInner::Guest(format!(
            "Dispute update tree exceeds revision limit {MAX_DISPUTE_REVISIONS}"
        ))));
    }
    if visited.contains(&action_hash) {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Dispute update tree contains a cycle or duplicate edge".into()
        )));
    }
    visited.push(action_hash.clone());

    let details = get_details(action_hash.clone(), GetOptions::default())?.ok_or_else(|| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "Dispute revision {action_hash} is unavailable"
        )))
    })?;
    let record_details = match details {
        Details::Record(details) => details,
        Details::Entry(_) => {
            return Err(wasm_error!(WasmErrorInner::Guest(
                "Dispute revision lookup unexpectedly returned entry details".into()
            )))
        }
    };

    if !record_details.deletes.is_empty() {
        return Err(wasm_error!(WasmErrorInner::Guest(format!(
            "Dispute revision {action_hash} has been deleted; dispute deletion is forbidden"
        ))));
    }

    let dispute: Dispute = error_handling::deserialize_entry(&record_details.record)?;
    let mut child_hashes: Vec<ActionHash> = record_details
        .updates
        .into_iter()
        .map(|update| update.as_hash().clone())
        .collect();
    child_hashes.sort_by_key(ToString::to_string);
    child_hashes.dedup();

    if child_hashes.is_empty() {
        heads.push(DisputeOutput {
            dispute_hash: action_hash,
            dispute,
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

    #[test]
    fn resolution_state_wire_values_are_lowercase() {
        assert_eq!(
            serde_json::to_string(&DisputeResolutionState::Resolved).unwrap(),
            "\"resolved\""
        );
    }
}
