// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Policy-neutral observed DID-document lineage theorem for Identity V2.
//!
//! This crate proves only that one complete supplied DID-document action set forms one
//! rooted, non-branching update lineage for one expected DID. It does not select a
//! verifier key, evaluate verifier policy, interpret deactivation, perform network reads,
//! or verify signatures.
//!
//! Success retains the exact qualified DID-document action IDs so downstream generation
//! provenance can require that a referenced DID document belongs to the observed lineage.

#![forbid(unsafe_code)]

use std::collections::{BTreeMap, BTreeSet};

pub const DID_MAX_LEN_V2: usize = 256;
pub const ACTION_ID_MAX_LEN_V2: usize = 256;
pub const MAX_OBSERVED_DID_DOCUMENT_ACTIONS_V2: usize = 4096;

#[derive(Debug, Clone, Copy)]
pub struct ObservedDidDocumentLineageActionV2<'a> {
    pub action_id: &'a str,
    /// `None` denotes a create/root action. Updates must name the exact original
    /// DID-document action they directly update.
    pub previous_action_id: Option<&'a str>,
    pub version: u32,
    pub did: &'a str,
}

#[derive(Debug, Clone, PartialEq, Eq)]
struct QualifiedDidDocumentActionV2 {
    action_id: String,
    version: u32,
}

/// Opaque exact membership capability for one observed DID-document lineage.
#[derive(Debug)]
pub struct QualifiedDidDocumentLineageV2 {
    did: String,
    root_action_id: String,
    terminal_action_id: String,
    terminal_version: u32,
    actions: Vec<QualifiedDidDocumentActionV2>,
}

impl QualifiedDidDocumentLineageV2 {
    pub fn did(&self) -> &str {
        &self.did
    }

    pub fn root_action_id(&self) -> &str {
        &self.root_action_id
    }

    pub fn terminal_action_id(&self) -> &str {
        &self.terminal_action_id
    }

    pub fn terminal_version(&self) -> u32 {
        self.terminal_version
    }

    pub fn observed_action_count(&self) -> usize {
        self.actions.len()
    }

    pub fn contains_document_action(&self, action_id: &str) -> bool {
        self.actions.iter().any(|action| action.action_id == action_id)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DidDocumentLineageErrorV2 {
    ExpectedDidInvalid,
    EmptyHistory,
    HistoryTooLarge,
    ActionIdInvalid,
    DuplicateActionId,
    DidMismatch,
    CreationRootMissing,
    MultipleCreationRoots,
    CreationVersionInvalid,
    SelfParentReference,
    UpdateParentMissing,
    UpdateVersionNotIncreasing,
    BranchingHistory,
    DisconnectedHistory,
}

fn valid_did(value: &str) -> bool {
    !value.is_empty() && value.len() <= DID_MAX_LEN_V2 && value.starts_with("did:mycelix:")
}

fn valid_action_id(value: &str) -> bool {
    !value.is_empty() && value.len() <= ACTION_ID_MAX_LEN_V2
}

/// Qualify one complete observed DID-document lineage without policy coupling.
///
/// Authority is graph-topological, never chronology-based:
///
/// 1. exactly one action has no parent and it is version 1;
/// 2. all actions belong to the exact expected DID;
/// 3. every update names one observed direct parent;
/// 4. update version strictly increases relative to its direct parent;
/// 5. no parent has competing children;
/// 6. every supplied document action is reachable from the one root.
pub fn qualify_did_document_lineage_v2(
    expected_did: &str,
    history: &[ObservedDidDocumentLineageActionV2<'_>],
) -> Result<QualifiedDidDocumentLineageV2, DidDocumentLineageErrorV2> {
    if !valid_did(expected_did) {
        return Err(DidDocumentLineageErrorV2::ExpectedDidInvalid);
    }
    if history.is_empty() {
        return Err(DidDocumentLineageErrorV2::EmptyHistory);
    }
    if history.len() > MAX_OBSERVED_DID_DOCUMENT_ACTIONS_V2 {
        return Err(DidDocumentLineageErrorV2::HistoryTooLarge);
    }

    let mut action_to_index: BTreeMap<&str, usize> = BTreeMap::new();
    let mut root_index: Option<usize> = None;

    for (index, observed) in history.iter().enumerate() {
        if !valid_action_id(observed.action_id) {
            return Err(DidDocumentLineageErrorV2::ActionIdInvalid);
        }
        if observed.did != expected_did {
            return Err(DidDocumentLineageErrorV2::DidMismatch);
        }
        if let Some(parent) = observed.previous_action_id {
            if !valid_action_id(parent) {
                return Err(DidDocumentLineageErrorV2::ActionIdInvalid);
            }
            if parent == observed.action_id {
                return Err(DidDocumentLineageErrorV2::SelfParentReference);
            }
        }
        if action_to_index.insert(observed.action_id, index).is_some() {
            return Err(DidDocumentLineageErrorV2::DuplicateActionId);
        }
        if observed.previous_action_id.is_none() && root_index.replace(index).is_some() {
            return Err(DidDocumentLineageErrorV2::MultipleCreationRoots);
        }
    }

    let root_index = root_index.ok_or(DidDocumentLineageErrorV2::CreationRootMissing)?;
    if history[root_index].version != 1 {
        return Err(DidDocumentLineageErrorV2::CreationVersionInvalid);
    }

    let mut child_of: BTreeMap<&str, usize> = BTreeMap::new();
    for (index, observed) in history.iter().enumerate() {
        let Some(parent_action_id) = observed.previous_action_id else {
            continue;
        };
        let parent_index = *action_to_index
            .get(parent_action_id)
            .ok_or(DidDocumentLineageErrorV2::UpdateParentMissing)?;
        let parent = history[parent_index];
        if observed.version <= parent.version {
            return Err(DidDocumentLineageErrorV2::UpdateVersionNotIncreasing);
        }
        if child_of.insert(parent_action_id, index).is_some() {
            return Err(DidDocumentLineageErrorV2::BranchingHistory);
        }
    }

    let mut visited: BTreeSet<&str> = BTreeSet::new();
    let mut terminal_index = root_index;
    loop {
        let current_action_id = history[terminal_index].action_id;
        if !visited.insert(current_action_id) {
            return Err(DidDocumentLineageErrorV2::DisconnectedHistory);
        }
        match child_of.get(current_action_id) {
            Some(next_index) => terminal_index = *next_index,
            None => break,
        }
    }
    if visited.len() != history.len() {
        return Err(DidDocumentLineageErrorV2::DisconnectedHistory);
    }

    let actions = history
        .iter()
        .map(|observed| QualifiedDidDocumentActionV2 {
            action_id: observed.action_id.to_string(),
            version: observed.version,
        })
        .collect();

    Ok(QualifiedDidDocumentLineageV2 {
        did: expected_did.to_string(),
        root_action_id: history[root_index].action_id.to_string(),
        terminal_action_id: history[terminal_index].action_id.to_string(),
        terminal_version: history[terminal_index].version,
        actions,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    const DID: &str = "did:mycelix:verifier";

    fn observed(
        action_id: &'static str,
        previous_action_id: Option<&'static str>,
        version: u32,
    ) -> ObservedDidDocumentLineageActionV2<'static> {
        ObservedDidDocumentLineageActionV2 {
            action_id,
            previous_action_id,
            version,
            did: DID,
        }
    }

    #[test]
    fn linear_lineage_retains_exact_document_membership() {
        let history = [
            observed("d1", None, 1),
            observed("d2", Some("d1"), 2),
            observed("d3", Some("d2"), 7),
        ];
        let qualified = qualify_did_document_lineage_v2(DID, &history).unwrap();
        assert_eq!(qualified.root_action_id(), "d1");
        assert_eq!(qualified.terminal_action_id(), "d3");
        assert_eq!(qualified.terminal_version(), 7);
        assert_eq!(qualified.observed_action_count(), 3);
        assert!(qualified.contains_document_action("d1"));
        assert!(qualified.contains_document_action("d2"));
        assert!(qualified.contains_document_action("d3"));
        assert!(!qualified.contains_document_action("missing"));
    }

    #[test]
    fn multiple_creation_roots_fail_closed() {
        let history = [observed("d1", None, 1), observed("rogue", None, 1)];
        assert_eq!(
            qualify_did_document_lineage_v2(DID, &history).unwrap_err(),
            DidDocumentLineageErrorV2::MultipleCreationRoots
        );
    }

    #[test]
    fn competing_children_fail_closed_without_version_tiebreaker() {
        let history = [
            observed("d1", None, 1),
            observed("d2", Some("d1"), 2),
            observed("rogue", Some("d1"), 999),
        ];
        assert_eq!(
            qualify_did_document_lineage_v2(DID, &history).unwrap_err(),
            DidDocumentLineageErrorV2::BranchingHistory
        );
    }

    #[test]
    fn missing_direct_parent_fails_closed() {
        let history = [
            observed("d1", None, 1),
            observed("d2", Some("missing"), 2),
        ];
        assert_eq!(
            qualify_did_document_lineage_v2(DID, &history).unwrap_err(),
            DidDocumentLineageErrorV2::UpdateParentMissing
        );
    }

    #[test]
    fn update_version_must_advance_relative_to_parent() {
        let history = [observed("d1", None, 1), observed("d2", Some("d1"), 1)];
        assert_eq!(
            qualify_did_document_lineage_v2(DID, &history).unwrap_err(),
            DidDocumentLineageErrorV2::UpdateVersionNotIncreasing
        );
    }

    #[test]
    fn root_must_be_version_one() {
        let history = [observed("d1", None, 2)];
        assert_eq!(
            qualify_did_document_lineage_v2(DID, &history).unwrap_err(),
            DidDocumentLineageErrorV2::CreationVersionInvalid
        );
    }

    #[test]
    fn did_namespace_mismatch_fails_closed() {
        let history = [ObservedDidDocumentLineageActionV2 {
            action_id: "d1",
            previous_action_id: None,
            version: 1,
            did: "did:mycelix:other",
        }];
        assert_eq!(
            qualify_did_document_lineage_v2(DID, &history).unwrap_err(),
            DidDocumentLineageErrorV2::DidMismatch
        );
    }

    #[test]
    fn self_parent_reference_fails_closed() {
        let history = [
            observed("d1", None, 1),
            observed("d2", Some("d2"), 2),
        ];
        assert_eq!(
            qualify_did_document_lineage_v2(DID, &history).unwrap_err(),
            DidDocumentLineageErrorV2::SelfParentReference
        );
    }

    #[test]
    fn oversized_history_requires_explicit_checkpointing() {
        let root = observed("d1", None, 1);
        let history = vec![root; MAX_OBSERVED_DID_DOCUMENT_ACTIONS_V2 + 1];
        assert_eq!(
            qualify_did_document_lineage_v2(DID, &history).unwrap_err(),
            DidDocumentLineageErrorV2::HistoryTooLarge
        );
    }

    #[test]
    fn qualified_lineage_fields_are_verifier_owned() {
        let source = include_str!("lib.rs");
        let start = source.index("pub struct QualifiedDidDocumentLineageV2").unwrap();
        let end = source[start..]
            .index("impl QualifiedDidDocumentLineageV2")
            .unwrap()
            + start;
        let body = &source[start..end];
        for public_field in [
            "pub did:",
            "pub root_action_id:",
            "pub terminal_action_id:",
            "pub terminal_version:",
            "pub actions:",
        ] {
            assert!(!body.contains(public_field));
        }
    }
}
