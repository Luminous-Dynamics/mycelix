// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Fail-closed resolution of the locally observed Marketplace listing update graph.
//!
//! A timestamp, hash ordering, or arbitrary "last update" is not authority for
//! choosing between concurrent listing revisions. The resolver preserves every
//! observed leaf and exposes a canonical listing only when exactly one leaf is
//! visible from the local DHT view. This is not a global catalogue completeness
//! proof: Holochain DHT integration is eventually consistent.

use hdk::prelude::*;
use super::ListingOutput;
use listings_integrity::Listing;
use mycelix_common::error_handling;

pub const LISTING_RESOLUTION_POLICY_VERSION: u16 = 1;
const MAX_LISTING_REVISION_DEPTH: u16 = 32;
const MAX_LISTING_REVISIONS: usize = 128;

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum ListingResolutionState {
    /// Exactly one leaf was visible in this conductor's locally observed graph.
    /// This does not imply DHT convergence or global currentness.
    SingleObservedHead,
    /// Multiple leaf revisions were observed. No head is selected.
    ConcurrentHeads,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum ListingObservationScope {
    LocalDhtUpdateGraph,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum ListingResolutionReason {
    SingleObservedHead,
    ConcurrentObservedHeads,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum ListingCatalogCoverage {
    /// Only links visible to this conductor during this read were examined.
    /// This is not proof of globally complete or converged catalogue contents.
    LocalObservedLinksOnly,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ListingResolution {
    /// Stable identity: the original Create action for this listing.
    pub root_listing_hash: ActionHash,
    /// Version of the fail-closed leaf projection policy.
    pub policy_version: u16,
    pub observation_scope: ListingObservationScope,
    pub state: ListingResolutionState,
    pub reason: ListingResolutionReason,
    /// Sole locally observed leaf only; not a global-currentness guarantee.
    pub sole_observed_head: Option<ListingOutput>,
    /// All locally observed leaf actions, sorted by action hash for reproducibility.
    pub heads: Vec<ListingOutput>,
    /// Number of Create/Update actions traversed in the locally observed graph.
    pub revision_count: u32,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ListingCatalogSnapshotV1 {
    pub policy_version: u16,
    /// Deliberately does not assert global catalogue completeness.
    pub coverage: ListingCatalogCoverage,
    /// Number of local discovery links seen before duplicate elimination.
    pub observed_link_count: u32,
    /// Number of unique listing roots after resolving links to their stable roots.
    pub unique_listing_count: u32,
    pub resolved_listing_count: u32,
    pub conflicted_listing_count: u32,
    /// Includes conflicted listings and resolved soft-deleted listings.
    pub listings: Vec<ListingResolution>,
}

/// Resolve any known listing Create/Update action to its stable root and all
/// locally observed leaf revisions. Missing input hashes return None; incomplete
/// ancestry or traversal errors return an error instead of a guessed current item.
pub fn resolve_listing(listing_hash: ActionHash) -> ExternResult<Option<ListingResolution>> {
    let Some(root_listing_hash) = find_root(listing_hash)? else {
        return Ok(None);
    };

    let mut visited = Vec::new();
    let mut heads = Vec::new();
    collect_heads(root_listing_hash.clone(), 0, &mut visited, &mut heads)?;

    let revision_count = u32::try_from(visited.len()).unwrap_or(u32::MAX);
    project_observed_heads(root_listing_hash, revision_count, heads)
        .map(Some)
        .map_err(|reason| wasm_error!(WasmErrorInner::Guest(reason)))
}

fn find_root(mut cursor: ActionHash) -> ExternResult<Option<ActionHash>> {
    let mut visited = Vec::new();

    for _ in 0..=MAX_LISTING_REVISION_DEPTH {
        if visited.contains(&cursor) {
            return Err(wasm_error!(WasmErrorInner::Guest(
                "Listing update ancestry contains a cycle".into()
            )));
        }
        if visited.len() >= MAX_LISTING_REVISIONS {
            return Err(wasm_error!(WasmErrorInner::Guest(format!(
                "Listing update ancestry exceeds revision limit {MAX_LISTING_REVISIONS}"
            ))));
        }
        visited.push(cursor.clone());

        let Some(record) = get(cursor.clone(), GetOptions::default())? else {
            if visited.len() == 1 {
                return Ok(None);
            }
            return Err(wasm_error!(WasmErrorInner::Guest(format!(
                "Listing update ancestry is incomplete at {cursor}"
            ))));
        };

        match record.action() {
            Action::Create(_) => return Ok(Some(cursor)),
            Action::Update(update) => cursor = update.original_action_address.clone(),
            _ => {
                return Err(wasm_error!(WasmErrorInner::Guest(
                    "Listing hash must reference a Create or Update action".into()
                )));
            }
        }
    }

    Err(wasm_error!(WasmErrorInner::Guest(format!(
        "Listing update ancestry exceeds depth limit {MAX_LISTING_REVISION_DEPTH}"
    ))))
}

fn collect_heads(
    action_hash: ActionHash,
    depth: u16,
    visited: &mut Vec<ActionHash>,
    heads: &mut Vec<ListingOutput>,
) -> ExternResult<()> {
    if depth > MAX_LISTING_REVISION_DEPTH {
        return Err(wasm_error!(WasmErrorInner::Guest(format!(
            "Listing update tree exceeds depth limit {MAX_LISTING_REVISION_DEPTH}"
        ))));
    }
    if visited.len() >= MAX_LISTING_REVISIONS {
        return Err(wasm_error!(WasmErrorInner::Guest(format!(
            "Listing update tree exceeds revision limit {MAX_LISTING_REVISIONS}"
        ))));
    }
    if visited.contains(&action_hash) {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Listing update tree contains a cycle or duplicate edge".into()
        )));
    }
    visited.push(action_hash.clone());

    let details = get_details(action_hash.clone(), GetOptions::default())?.ok_or_else(|| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "Listing revision {action_hash} is unavailable"
        )))
    })?;
    let record_details = match details {
        Details::Record(details) => details,
        Details::Entry(_) => {
            return Err(wasm_error!(WasmErrorInner::Guest(
                "Listing revision lookup unexpectedly returned entry details".into()
            )));
        }
    };

    if !record_details.deletes.is_empty() {
        return Err(wasm_error!(WasmErrorInner::Guest(format!(
            "Listing revision {action_hash} has a DHT delete; refusing to resolve deleted history"
        ))));
    }

    let listing: Listing = error_handling::deserialize_entry(&record_details.record)?;
    let mut child_hashes: Vec<ActionHash> = record_details
        .updates
        .into_iter()
        .map(|update| update.as_hash().clone())
        .collect();
    child_hashes.sort_by_key(ToString::to_string);
    child_hashes.dedup();

    if child_hashes.is_empty() {
        heads.push(ListingOutput {
            listing_hash: action_hash,
            listing,
            seller_agent_id: record_details.record.action().author().clone().into(),
        });
        return Ok(());
    }

    for child_hash in child_hashes {
        collect_heads(child_hash, depth + 1, visited, heads)?;
    }

    Ok(())
}

fn project_observed_heads(
    root_listing_hash: ActionHash,
    revision_count: u32,
    mut heads: Vec<ListingOutput>,
) -> Result<ListingResolution, String> {
    heads.sort_by_key(|head| head.listing_hash.to_string());

    let (state, reason, canonical) = match heads.len() {
        0 => return Err("Listing update tree has no observed leaf revisions".into()),
        1 => (
            ListingResolutionState::SingleObservedHead,
            ListingResolutionReason::SingleObservedHead,
            heads.first().cloned(),
        ),
        _ => (
            ListingResolutionState::ConcurrentHeads,
            ListingResolutionReason::ConcurrentObservedHeads,
            None,
        ),
    };

    Ok(ListingResolution {
        root_listing_hash,
        policy_version: LISTING_RESOLUTION_POLICY_VERSION,
        observation_scope: ListingObservationScope::LocalDhtUpdateGraph,
        state,
        reason,
        sole_observed_head: canonical,
        heads,
        revision_count,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use listings_integrity::{
        EmpiricalLevel, EpistemicClassification, ListingCategory, ListingStatus, MaterialityLevel,
        NormativeLevel,
    };

    fn hash(byte: u8) -> ActionHash {
        ActionHash::from_raw_36(vec![byte; 36])
    }

    fn agent(byte: u8) -> AgentPubKey {
        AgentPubKey::from_raw_36(vec![byte; 36])
    }

    fn listing_output(
        hash_byte: u8,
        price_cents: u64,
        quantity_available: u32,
        status: ListingStatus,
    ) -> ListingOutput {
        ListingOutput {
            listing_hash: hash(hash_byte),
            seller_agent_id: agent(77),
            listing: Listing {
                title: "Modular desk".into(),
                description: "Synthetic test listing".into(),
                price_cents,
                category: ListingCategory::HomeGarden,
                photos_ipfs_cids: vec!["QmYwAPJzv5CZsnA625s3Xf2nemtYgPpHdWEz79ojWnPbdG".into()],
                quantity_available,
                status,
                epistemic: EpistemicClassification {
                    empirical: EmpiricalLevel::E1Testimonial,
                    normative: NormativeLevel::N0Personal,
                    materiality: MaterialityLevel::M1Temporal,
                },
                created_at: Timestamp::from_micros(1_000),
                updated_at: Timestamp::from_micros(i64::from(hash_byte)),
            },
        }
    }

    #[test]
    fn one_observed_leaf_is_resolved_and_preserves_root_and_current_hash() {
        let root = hash(1);
        let output = listing_output(2, 25_000, 3, ListingStatus::Active);
        let projection = project_observed_heads(root.clone(), 2, vec![output]).unwrap();

        assert_eq!(projection.state, ListingResolutionState::SingleObservedHead);
        assert_eq!(
            projection.reason,
            ListingResolutionReason::SingleObservedHead
        );
        assert_eq!(projection.root_listing_hash, root);
        assert_eq!(projection.sole_observed_head.as_ref().unwrap().listing_hash, hash(2));
        assert_eq!(projection.revision_count, 2);
        assert_eq!(projection.heads.len(), 1);
    }

    #[test]
    fn concurrent_price_or_inventory_edits_never_choose_a_canonical_head() {
        let root = hash(1);
        let first = listing_output(2, 25_000, 3, ListingStatus::Active);
        let second = listing_output(3, 22_000, 1, ListingStatus::Active);
        let projection = project_observed_heads(root, 3, vec![first, second]).unwrap();

        assert_eq!(projection.state, ListingResolutionState::ConcurrentHeads);
        assert_eq!(
            projection.reason,
            ListingResolutionReason::ConcurrentObservedHeads
        );
        assert!(projection.sole_observed_head.is_none());
        assert_eq!(projection.heads.len(), 2);
    }

    #[test]
    fn identical_concurrent_heads_remain_conflicted_without_authority() {
        let root = hash(1);
        let first = listing_output(2, 25_000, 3, ListingStatus::Active);
        let second = listing_output(3, 25_000, 3, ListingStatus::Active);
        let projection = project_observed_heads(root, 3, vec![second, first]).unwrap();

        assert_eq!(projection.state, ListingResolutionState::ConcurrentHeads);
        assert!(projection.sole_observed_head.is_none());
        assert_eq!(projection.heads[0].listing_hash, hash(2));
        assert_eq!(projection.heads[1].listing_hash, hash(3));
    }

    #[test]
    fn soft_delete_is_a_resolved_observed_status_not_an_active_offer() {
        let root = hash(1);
        let deleted = listing_output(2, 25_000, 3, ListingStatus::Deleted);
        let projection = project_observed_heads(root, 2, vec![deleted]).unwrap();

        assert_eq!(projection.state, ListingResolutionState::SingleObservedHead);
        assert_eq!(
            projection.sole_observed_head.as_ref().unwrap().listing.status,
            ListingStatus::Deleted
        );
    }

    #[test]
    fn empty_or_incomplete_tree_never_becomes_a_successful_resolution() {
        let error = project_observed_heads(hash(1), 1, Vec::new()).unwrap_err();
        assert!(error.contains("no observed leaf revisions"));
    }

    #[test]
    fn identical_input_graphs_produce_stable_head_order() {
        let root = hash(1);
        let a = listing_output(2, 25_000, 3, ListingStatus::Active);
        let b = listing_output(3, 22_000, 1, ListingStatus::Active);
        let first = project_observed_heads(root.clone(), 3, vec![a.clone(), b.clone()]).unwrap();
        let second = project_observed_heads(root, 3, vec![b, a]).unwrap();

        let first_hashes: Vec<String> = first
            .heads
            .iter()
            .map(|head| head.listing_hash.to_string())
            .collect();
        let second_hashes: Vec<String> = second
            .heads
            .iter()
            .map(|head| head.listing_hash.to_string())
            .collect();
        assert_eq!(first_hashes, second_hashes);
        assert!(first.sole_observed_head.is_none());
        assert!(second.sole_observed_head.is_none());
    }
}
