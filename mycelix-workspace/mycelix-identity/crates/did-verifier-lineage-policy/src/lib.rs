// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Branch-aware observed DID verifier-key lineage theorem for Identity V2.
//!
//! This crate consumes already-gathered DID document history facts and proves only
//! that the observation forms one unambiguous rooted update chain whose terminal
//! document structurally contains the policy-selected verifier key.
//!
//! It deliberately does not perform network reads, select by chronology, verify a
//! cryptographic signature, evaluate verifier-policy acceptance, or emit positive
//! verification evidence.

#![forbid(unsafe_code)]

use std::collections::{BTreeMap, BTreeSet};

use mycelix_kvector_verifier_policy_body::KVectorVerifierPolicyBodyV2;
use mycelix_verifier_key_method_policy::{
    resolve_policy_verifier_key_from_document_v2, DidVerificationDocumentViewV2,
    PolicyBoundVerifierKeyV2, VerifierKeyMethodErrorV2,
};

/// Qualification cap for a complete observed root-to-tip DID history.
///
/// Larger histories require an explicit checkpoint/compaction theorem. Callers must
/// never truncate history to fit this bound and still claim successful qualification.
pub const MAX_OBSERVED_DID_HISTORY_ACTIONS_V2: usize = 4096;
pub const MAX_OBSERVED_DEACTIVATIONS_V2: usize = 16;
pub const ACTION_ID_MAX_LEN_V2: usize = 256;

#[derive(Debug, Clone, Copy)]
pub struct ObservedDidDocumentV2<'a> {
    pub action_id: &'a str,
    /// `None` identifies the canonical creation root supplied by the evidence gatherer.
    /// Every update must name exactly one directly observed parent action.
    pub previous_action_id: Option<&'a str>,
    pub version: u32,
    pub document: DidVerificationDocumentViewV2<'a>,
}

#[derive(Debug, Clone, Copy)]
pub struct ObservedDidDeactivationV2<'a> {
    pub action_id: &'a str,
    pub did: &'a str,
}

/// Observation-scoped verifier-key evidence.
///
/// This type intentionally says only that the supplied network history observation was
/// linear, non-deactivated, and structurally bound to the frozen verifier policy.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ObservedVerifierKeyV2 {
    pub did: String,
    pub creation_action_id: String,
    pub document_action_id: String,
    pub document_version: u32,
    pub observed_lineage_actions: usize,
    pub key: PolicyBoundVerifierKeyV2,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DidVerifierLineageErrorV2 {
    EmptyHistory,
    HistoryTooLarge,
    TooManyDeactivationObservations,
    ActionIdInvalid,
    DuplicateActionId,
    CreationRootMissing,
    MultipleCreationRoots,
    CreationVersionInvalid,
    DidMismatch,
    UpdateParentMissing,
    UpdateVersionNotIncreasing,
    BranchingHistory,
    DisconnectedHistory,
    DeactivationDidMismatch,
    DidDeactivated,
    VerifierKeyMethod(VerifierKeyMethodErrorV2),
}

fn bounded_nonempty(value: &str, max_len: usize) -> bool {
    !value.is_empty() && value.len() <= max_len
}

fn valid_action_id(value: &str) -> bool {
    bounded_nonempty(value, ACTION_ID_MAX_LEN_V2)
}

/// Resolve the policy-selected verifier key from one complete observed DID lineage.
///
/// Lineage authority is graph-topological, never chronology-based:
///
/// 1. exactly one document has no parent and it must be version 1;
/// 2. every update references one observed direct parent;
/// 3. update versions strictly increase relative to that parent;
/// 4. no parent may have more than one child;
/// 5. every observed document must be reachable from the one creation root;
/// 6. any supplied valid deactivation observation for the DID fails qualification;
/// 7. the unique terminal document must satisfy the strict verifier-key method theorem.
///
/// A successful result is still only observation-scoped key evidence. A later adapter
/// must independently establish full cryptographic signature authenticity and verifier
/// policy authority before producing positive verification evidence.
pub fn resolve_observed_verifier_key_v2<'a>(
    history: &'a [ObservedDidDocumentV2<'a>],
    deactivations: &'a [ObservedDidDeactivationV2<'a>],
    policy: KVectorVerifierPolicyBodyV2<'a>,
) -> Result<ObservedVerifierKeyV2, DidVerifierLineageErrorV2> {
    if history.is_empty() {
        return Err(DidVerifierLineageErrorV2::EmptyHistory);
    }
    if history.len() > MAX_OBSERVED_DID_HISTORY_ACTIONS_V2 {
        return Err(DidVerifierLineageErrorV2::HistoryTooLarge);
    }
    if deactivations.len() > MAX_OBSERVED_DEACTIVATIONS_V2 {
        return Err(DidVerifierLineageErrorV2::TooManyDeactivationObservations);
    }

    let mut action_to_index: BTreeMap<&str, usize> = BTreeMap::new();
    let mut root_index: Option<usize> = None;

    for (index, observed) in history.iter().enumerate() {
        if !valid_action_id(observed.action_id) {
            return Err(DidVerifierLineageErrorV2::ActionIdInvalid);
        }
        if let Some(parent) = observed.previous_action_id {
            if !valid_action_id(parent) {
                return Err(DidVerifierLineageErrorV2::ActionIdInvalid);
            }
        }
        if observed.document.did != policy.verifier_did {
            return Err(DidVerifierLineageErrorV2::DidMismatch);
        }
        if action_to_index.insert(observed.action_id, index).is_some() {
            return Err(DidVerifierLineageErrorV2::DuplicateActionId);
        }

        if observed.previous_action_id.is_none() {
            if root_index.replace(index).is_some() {
                return Err(DidVerifierLineageErrorV2::MultipleCreationRoots);
            }
        }
    }

    let root_index = root_index.ok_or(DidVerifierLineageErrorV2::CreationRootMissing)?;
    if history[root_index].version != 1 {
        return Err(DidVerifierLineageErrorV2::CreationVersionInvalid);
    }

    let mut child_of: BTreeMap<&str, usize> = BTreeMap::new();
    for (index, observed) in history.iter().enumerate() {
        let Some(parent_action_id) = observed.previous_action_id else {
            continue;
        };

        let parent_index = *action_to_index
            .get(parent_action_id)
            .ok_or(DidVerifierLineageErrorV2::UpdateParentMissing)?;
        let parent = history[parent_index];

        if observed.version <= parent.version {
            return Err(DidVerifierLineageErrorV2::UpdateVersionNotIncreasing);
        }
        if child_of.insert(parent_action_id, index).is_some() {
            return Err(DidVerifierLineageErrorV2::BranchingHistory);
        }
    }

    let mut visited: BTreeSet<&str> = BTreeSet::new();
    let mut terminal_index = root_index;
    loop {
        let current_action_id = history[terminal_index].action_id;
        if !visited.insert(current_action_id) {
            return Err(DidVerifierLineageErrorV2::DisconnectedHistory);
        }

        match child_of.get(current_action_id) {
            Some(next_index) => terminal_index = *next_index,
            None => break,
        }
    }

    if visited.len() != history.len() {
        return Err(DidVerifierLineageErrorV2::DisconnectedHistory);
    }

    let mut deactivation_ids: BTreeSet<&str> = BTreeSet::new();
    for deactivation in deactivations {
        if !valid_action_id(deactivation.action_id) {
            return Err(DidVerifierLineageErrorV2::ActionIdInvalid);
        }
        if deactivation.did != policy.verifier_did {
            return Err(DidVerifierLineageErrorV2::DeactivationDidMismatch);
        }
        if action_to_index.contains_key(deactivation.action_id)
            || !deactivation_ids.insert(deactivation.action_id)
        {
            return Err(DidVerifierLineageErrorV2::DuplicateActionId);
        }
        return Err(DidVerifierLineageErrorV2::DidDeactivated);
    }

    let terminal = history[terminal_index];
    let key = resolve_policy_verifier_key_from_document_v2(terminal.document, policy)
        .map_err(DidVerifierLineageErrorV2::VerifierKeyMethod)?;

    Ok(ObservedVerifierKeyV2 {
        did: terminal.document.did.to_string(),
        creation_action_id: history[root_index].action_id.to_string(),
        document_action_id: terminal.action_id.to_string(),
        document_version: terminal.version,
        observed_lineage_actions: history.len(),
        key,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_verifier_key_method_policy::{
        DidVerificationMethodViewV2, VerifierKeyMethodErrorV2,
    };

    const DID: &str = "did:mycelix:verifier";
    const KEY_ID: &str = "did:mycelix:verifier#key-1";

    static METHODS: [DidVerificationMethodViewV2<'static>; 1] = [DidVerificationMethodViewV2 {
        id: "#key-1",
        type_: "Ed25519VerificationKey2020",
        controller: DID,
        public_key_multibase: "z6MkhaXgBZDvotDkL5257faiztiGiC2QtKLGpbnnEGta2doK",
        algorithm: Some(0xed01),
    }];
    static AUTH: [&str; 1] = ["#key-1"];
    static NO_AUTH: [&str; 0] = [];

    fn document() -> DidVerificationDocumentViewV2<'static> {
        DidVerificationDocumentViewV2 {
            did: DID,
            verification_methods: &METHODS,
            authentication: &AUTH,
        }
    }

    fn unauthenticated_document() -> DidVerificationDocumentViewV2<'static> {
        DidVerificationDocumentViewV2 {
            did: DID,
            verification_methods: &METHODS,
            authentication: &NO_AUTH,
        }
    }

    fn policy() -> KVectorVerifierPolicyBodyV2<'static> {
        KVectorVerifierPolicyBodyV2 {
            policy_id: "policy:kvector-prod-v2",
            policy_version: "2.0.0",
            backend_id: "winterfell-v2",
            circuit_id: "mycelix-kvector-range-v2",
            circuit_version: "2.0.0",
            verifier_did: DID,
            verifier_key_id: KEY_ID,
            signature_scheme_id: "ed25519-v1",
            valid_from_micros: 1_000_000,
            valid_until_micros: 9_000_000,
            max_record_lifetime_micros: 2_000_000,
        }
    }

    fn observed(
        action_id: &'static str,
        previous_action_id: Option<&'static str>,
        version: u32,
    ) -> ObservedDidDocumentV2<'static> {
        ObservedDidDocumentV2 {
            action_id,
            previous_action_id,
            version,
            document: document(),
        }
    }

    #[test]
    fn linear_history_selects_unique_topological_terminal() {
        let history = [
            observed("a1", None, 1),
            observed("a2", Some("a1"), 2),
            observed("a3", Some("a2"), 7),
        ];
        let resolved = resolve_observed_verifier_key_v2(&history, &[], policy()).unwrap();
        assert_eq!(resolved.creation_action_id, "a1");
        assert_eq!(resolved.document_action_id, "a3");
        assert_eq!(resolved.document_version, 7);
        assert_eq!(resolved.observed_lineage_actions, 3);
        assert_eq!(resolved.key.canonical_key_id, KEY_ID);
    }

    #[test]
    fn competing_children_fail_closed_even_with_different_versions() {
        let history = [
            observed("a1", None, 1),
            observed("a2", Some("a1"), 2),
            observed("a3", Some("a1"), 999),
        ];
        assert_eq!(
            resolve_observed_verifier_key_v2(&history, &[], policy()),
            Err(DidVerifierLineageErrorV2::BranchingHistory)
        );
    }

    #[test]
    fn missing_direct_parent_fails_closed() {
        let history = [
            observed("a1", None, 1),
            observed("a3", Some("missing"), 3),
        ];
        assert_eq!(
            resolve_observed_verifier_key_v2(&history, &[], policy()),
            Err(DidVerifierLineageErrorV2::UpdateParentMissing)
        );
    }

    #[test]
    fn versions_must_strictly_advance_along_parent_edges() {
        let history = [
            observed("a1", None, 1),
            observed("a2", Some("a1"), 1),
        ];
        assert_eq!(
            resolve_observed_verifier_key_v2(&history, &[], policy()),
            Err(DidVerifierLineageErrorV2::UpdateVersionNotIncreasing)
        );
    }

    #[test]
    fn multiple_creation_roots_fail_closed() {
        let history = [observed("a1", None, 1), observed("b1", None, 1)];
        assert_eq!(
            resolve_observed_verifier_key_v2(&history, &[], policy()),
            Err(DidVerifierLineageErrorV2::MultipleCreationRoots)
        );
    }

    #[test]
    fn any_valid_deactivation_observation_fails_qualification() {
        let history = [observed("a1", None, 1)];
        let deactivations = [ObservedDidDeactivationV2 {
            action_id: "d1",
            did: DID,
        }];
        assert_eq!(
            resolve_observed_verifier_key_v2(&history, &deactivations, policy()),
            Err(DidVerifierLineageErrorV2::DidDeactivated)
        );
    }

    #[test]
    fn terminal_document_must_still_satisfy_structural_key_theorem() {
        let history = [ObservedDidDocumentV2 {
            action_id: "a1",
            previous_action_id: None,
            version: 1,
            document: unauthenticated_document(),
        }];
        assert_eq!(
            resolve_observed_verifier_key_v2(&history, &[], policy()),
            Err(DidVerifierLineageErrorV2::VerifierKeyMethod(
                VerifierKeyMethodErrorV2::PolicyVerifierKeyNotAuthenticated
            ))
        );
    }

    #[test]
    fn oversized_history_requires_a_future_checkpoint_theorem() {
        let root = observed("a1", None, 1);
        let history = vec![root; MAX_OBSERVED_DID_HISTORY_ACTIONS_V2 + 1];
        assert_eq!(
            resolve_observed_verifier_key_v2(&history, &[], policy()),
            Err(DidVerifierLineageErrorV2::HistoryTooLarge)
        );
    }
}
