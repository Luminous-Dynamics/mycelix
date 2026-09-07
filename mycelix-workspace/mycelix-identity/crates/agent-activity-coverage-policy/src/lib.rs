// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Viewpoint-scoped agent-activity coverage theorem for Identity V2.
//!
//! This crate qualifies one supplied Holochain-style agent-activity observation before
//! a Holochain adapter is allowed to use filtered DID action hashes as the discovery
//! set for branch-aware DID lineage analysis.
//!
//! Success means only that the supplied observation reports a settled, internally
//! consistent valid chain head from its observation perspective, with no known warrants,
//! no rejected matching activity, and no unintegrated activity beyond that head.
//!
//! The qualified output is intentionally opaque to callers and retains the exact action
//! set that passed qualification. A downstream adapter cannot substitute a different set
//! after qualification without re-running this theorem.
//!
//! It does **not** establish global consensus across all authorities, DID-document
//! semantics, cryptographic signature authenticity, verifier-policy currentness, or
//! positive verification evidence.

#![forbid(unsafe_code)]

use std::collections::{BTreeMap, BTreeSet};

pub const ACTION_ID_MAX_LEN_V2: usize = 256;
pub const MAX_MATCHING_ACTIVITY_ACTIONS_V2: usize = 4096;
pub const MAX_HIGHEST_OBSERVED_HEADS_V2: usize = 16;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ObservedActivityActionV2<'a> {
    pub action_seq: u32,
    pub action_id: &'a str,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ObservedChainStatusV2<'a> {
    Empty,
    Valid {
        action_seq: u32,
        action_id: &'a str,
    },
    Forked {
        fork_seq: u32,
        first_action_id: &'a str,
        second_action_id: &'a str,
    },
    Invalid {
        action_seq: u32,
        action_id: &'a str,
    },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ObservedHighestActivityV2<'a> {
    pub action_seq: u32,
    pub action_ids: &'a [&'a str],
}

#[derive(Debug, Clone, Copy)]
pub struct AgentActivityObservationV2<'a> {
    pub status: ObservedChainStatusV2<'a>,
    /// Valid actions returned by the adapter's qualified matching-activity query.
    /// These may be sparse in sequence space because the adapter can filter to DID entries.
    pub valid_activity: &'a [ObservedActivityActionV2<'a>],
    /// Rejected actions matching that same query.
    pub rejected_activity: &'a [ObservedActivityActionV2<'a>],
    /// Highest action(s) observed by the authority view, irrespective of the filter.
    pub highest_observed: Option<ObservedHighestActivityV2<'a>>,
    /// Number of warrants supplied by the authority observation.
    pub warrant_count: usize,
}

/// One exact matching action retained inside a qualified coverage capability.
///
/// Fields are private so callers cannot manufacture qualified membership without
/// receiving it from [`qualify_agent_activity_coverage_v2`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct QualifiedActivityActionV2 {
    action_seq: u32,
    action_id: String,
}

impl QualifiedActivityActionV2 {
    pub fn action_seq(&self) -> u32 {
        self.action_seq
    }

    pub fn action_id(&self) -> &str {
        &self.action_id
    }
}

/// Viewpoint-scoped activity coverage capability.
///
/// This type is verifier-owned: all fields are private and there is no public
/// constructor. Callers can inspect the exact qualified facts through getters but
/// cannot synthesize a successful qualification result.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct QualifiedAgentActivityCoverageV2 {
    valid_head_action_seq: u32,
    valid_head_action_id: String,
    matching_valid_actions: Vec<QualifiedActivityActionV2>,
}

impl QualifiedAgentActivityCoverageV2 {
    pub fn valid_head_action_seq(&self) -> u32 {
        self.valid_head_action_seq
    }

    pub fn valid_head_action_id(&self) -> &str {
        &self.valid_head_action_id
    }

    pub fn matching_valid_actions(&self) -> &[QualifiedActivityActionV2] {
        &self.matching_valid_actions
    }

    pub fn matching_valid_action_count(&self) -> usize {
        self.matching_valid_actions.len()
    }

    pub fn contains_action(&self, action_seq: u32, action_id: &str) -> bool {
        self.matching_valid_actions
            .iter()
            .any(|action| action.action_seq == action_seq && action.action_id == action_id)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AgentActivityCoverageErrorV2 {
    EmptyChainStatus,
    ForkedChainStatus,
    InvalidChainStatus,
    WarrantsPresent,
    RejectedActivityPresent,
    HighestObservedMissing,
    HighestObservedEmpty,
    TooManyHighestObservedHeads,
    MultipleHighestObservedHeads,
    HighestObservedDoesNotMatchValidHead,
    MatchingActivityTooLarge,
    ActionIdInvalid,
    DuplicateActionId,
    DuplicateActionSequence,
    MatchingActivityBeyondValidHead,
}

fn valid_action_id(value: &str) -> bool {
    !value.is_empty() && value.len() <= ACTION_ID_MAX_LEN_V2
}

fn validate_action(
    action: ObservedActivityActionV2<'_>,
) -> Result<(), AgentActivityCoverageErrorV2> {
    if !valid_action_id(action.action_id) {
        return Err(AgentActivityCoverageErrorV2::ActionIdInvalid);
    }
    Ok(())
}

/// Qualify one viewpoint-scoped agent activity observation.
///
/// The Holochain adapter that feeds this theorem must separately machine-prove that:
///
/// - the status/highest/warrant fields came from an agent-activity network observation;
/// - `valid_activity` / `rejected_activity` came from `ActivityRequest::Full` using the
///   intended DID-entry filter and no chronology-based truncation;
/// - all action IDs passed onward are fetched and typed before #267 lineage analysis.
///
/// This pure theorem then requires:
///
/// 1. `ChainStatus::Valid` equivalent status;
/// 2. zero warrants in the supplied observation;
/// 3. zero rejected matching activity;
/// 4. a single highest-observed action;
/// 5. highest-observed sequence/hash exactly equal the valid chain head;
/// 6. bounded matching valid activity;
/// 7. unique action IDs and action sequences in matching valid activity;
/// 8. no matching valid action beyond the reported valid head.
///
/// Success retains the exact qualified action set, preventing a downstream caller from
/// substituting different action hashes after the coverage theorem has passed.
/// Success remains viewpoint-scoped evidence, not global network consensus/currentness.
pub fn qualify_agent_activity_coverage_v2(
    observation: AgentActivityObservationV2<'_>,
) -> Result<QualifiedAgentActivityCoverageV2, AgentActivityCoverageErrorV2> {
    let (head_seq, head_id) = match observation.status {
        ObservedChainStatusV2::Empty => {
            return Err(AgentActivityCoverageErrorV2::EmptyChainStatus);
        }
        ObservedChainStatusV2::Forked { .. } => {
            return Err(AgentActivityCoverageErrorV2::ForkedChainStatus);
        }
        ObservedChainStatusV2::Invalid { .. } => {
            return Err(AgentActivityCoverageErrorV2::InvalidChainStatus);
        }
        ObservedChainStatusV2::Valid {
            action_seq,
            action_id,
        } => {
            if !valid_action_id(action_id) {
                return Err(AgentActivityCoverageErrorV2::ActionIdInvalid);
            }
            (action_seq, action_id)
        }
    };

    if observation.warrant_count != 0 {
        return Err(AgentActivityCoverageErrorV2::WarrantsPresent);
    }
    if !observation.rejected_activity.is_empty() {
        for action in observation.rejected_activity {
            validate_action(*action)?;
        }
        return Err(AgentActivityCoverageErrorV2::RejectedActivityPresent);
    }

    let highest = observation
        .highest_observed
        .ok_or(AgentActivityCoverageErrorV2::HighestObservedMissing)?;
    if highest.action_ids.is_empty() {
        return Err(AgentActivityCoverageErrorV2::HighestObservedEmpty);
    }
    if highest.action_ids.len() > MAX_HIGHEST_OBSERVED_HEADS_V2 {
        return Err(AgentActivityCoverageErrorV2::TooManyHighestObservedHeads);
    }
    for action_id in highest.action_ids {
        if !valid_action_id(action_id) {
            return Err(AgentActivityCoverageErrorV2::ActionIdInvalid);
        }
    }
    if highest.action_ids.len() != 1 {
        return Err(AgentActivityCoverageErrorV2::MultipleHighestObservedHeads);
    }
    if highest.action_seq != head_seq || highest.action_ids[0] != head_id {
        return Err(AgentActivityCoverageErrorV2::HighestObservedDoesNotMatchValidHead);
    }

    if observation.valid_activity.len() > MAX_MATCHING_ACTIVITY_ACTIONS_V2 {
        return Err(AgentActivityCoverageErrorV2::MatchingActivityTooLarge);
    }

    let mut action_ids: BTreeSet<&str> = BTreeSet::new();
    let mut action_seqs: BTreeMap<u32, &str> = BTreeMap::new();
    let mut qualified_actions = Vec::with_capacity(observation.valid_activity.len());
    for action in observation.valid_activity {
        validate_action(*action)?;
        if action.action_seq > head_seq {
            return Err(AgentActivityCoverageErrorV2::MatchingActivityBeyondValidHead);
        }
        if !action_ids.insert(action.action_id) {
            return Err(AgentActivityCoverageErrorV2::DuplicateActionId);
        }
        if action_seqs.insert(action.action_seq, action.action_id).is_some() {
            return Err(AgentActivityCoverageErrorV2::DuplicateActionSequence);
        }
        qualified_actions.push(QualifiedActivityActionV2 {
            action_seq: action.action_seq,
            action_id: action.action_id.to_string(),
        });
    }

    Ok(QualifiedAgentActivityCoverageV2 {
        valid_head_action_seq: head_seq,
        valid_head_action_id: head_id.to_string(),
        matching_valid_actions: qualified_actions,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn action(seq: u32, id: &'static str) -> ObservedActivityActionV2<'static> {
        ObservedActivityActionV2 {
            action_seq: seq,
            action_id: id,
        }
    }

    fn valid_observation<'a>(
        valid: &'a [ObservedActivityActionV2<'a>],
        highest_ids: &'a [&'a str],
    ) -> AgentActivityObservationV2<'a> {
        AgentActivityObservationV2 {
            status: ObservedChainStatusV2::Valid {
                action_seq: 42,
                action_id: "head-42",
            },
            valid_activity: valid,
            rejected_activity: &[],
            highest_observed: Some(ObservedHighestActivityV2 {
                action_seq: 42,
                action_ids: highest_ids,
            }),
            warrant_count: 0,
        }
    }

    #[test]
    fn sparse_matching_activity_can_be_covered_by_a_settled_chain_head() {
        let valid = [action(8, "did-create"), action(21, "did-update-1")];
        let highest = ["head-42"];
        let qualified = qualify_agent_activity_coverage_v2(valid_observation(&valid, &highest))
            .expect("settled observation should qualify");
        assert_eq!(qualified.valid_head_action_seq(), 42);
        assert_eq!(qualified.valid_head_action_id(), "head-42");
        assert_eq!(qualified.matching_valid_action_count(), 2);
        assert!(qualified.contains_action(8, "did-create"));
        assert!(qualified.contains_action(21, "did-update-1"));
        assert!(!qualified.contains_action(21, "substituted-update"));
        assert_eq!(qualified.matching_valid_actions()[0].action_seq(), 8);
        assert_eq!(qualified.matching_valid_actions()[0].action_id(), "did-create");
    }

    #[test]
    fn qualified_output_retains_exact_input_action_set() {
        let valid = [action(3, "did-create"), action(17, "did-update")];
        let highest = ["head-42"];
        let qualified = qualify_agent_activity_coverage_v2(valid_observation(&valid, &highest))
            .expect("settled observation should qualify");

        let retained: Vec<(u32, &str)> = qualified
            .matching_valid_actions()
            .iter()
            .map(|action| (action.action_seq(), action.action_id()))
            .collect();
        assert_eq!(retained, vec![(3, "did-create"), (17, "did-update")]);
    }

    #[test]
    fn forked_chain_status_fails_closed() {
        let observation = AgentActivityObservationV2 {
            status: ObservedChainStatusV2::Forked {
                fork_seq: 10,
                first_action_id: "fork-a",
                second_action_id: "fork-b",
            },
            valid_activity: &[],
            rejected_activity: &[],
            highest_observed: None,
            warrant_count: 0,
        };
        assert_eq!(
            qualify_agent_activity_coverage_v2(observation),
            Err(AgentActivityCoverageErrorV2::ForkedChainStatus)
        );
    }

    #[test]
    fn warrants_fail_closed_even_when_status_is_valid() {
        let highest = ["head-42"];
        let mut observation = valid_observation(&[], &highest);
        observation.warrant_count = 1;
        assert_eq!(
            qualify_agent_activity_coverage_v2(observation),
            Err(AgentActivityCoverageErrorV2::WarrantsPresent)
        );
    }

    #[test]
    fn rejected_matching_activity_fails_closed() {
        let highest = ["head-42"];
        let rejected = [action(20, "rejected-did-update")];
        let observation = AgentActivityObservationV2 {
            rejected_activity: &rejected,
            ..valid_observation(&[], &highest)
        };
        assert_eq!(
            qualify_agent_activity_coverage_v2(observation),
            Err(AgentActivityCoverageErrorV2::RejectedActivityPresent)
        );
    }

    #[test]
    fn unintegrated_higher_observation_fails_closed() {
        let highest = ["head-43"];
        let mut observation = valid_observation(&[], &highest);
        observation.highest_observed = Some(ObservedHighestActivityV2 {
            action_seq: 43,
            action_ids: &highest,
        });
        assert_eq!(
            qualify_agent_activity_coverage_v2(observation),
            Err(AgentActivityCoverageErrorV2::HighestObservedDoesNotMatchValidHead)
        );
    }

    #[test]
    fn multiple_highest_observed_hashes_fail_closed() {
        let highest = ["fork-a", "fork-b"];
        assert_eq!(
            qualify_agent_activity_coverage_v2(valid_observation(&[], &highest)),
            Err(AgentActivityCoverageErrorV2::MultipleHighestObservedHeads)
        );
    }

    #[test]
    fn duplicate_matching_sequence_fails_closed() {
        let valid = [action(12, "did-a"), action(12, "did-b")];
        let highest = ["head-42"];
        assert_eq!(
            qualify_agent_activity_coverage_v2(valid_observation(&valid, &highest)),
            Err(AgentActivityCoverageErrorV2::DuplicateActionSequence)
        );
    }

    #[test]
    fn matching_activity_cannot_be_beyond_valid_head() {
        let valid = [action(43, "future-did-update")];
        let highest = ["head-42"];
        assert_eq!(
            qualify_agent_activity_coverage_v2(valid_observation(&valid, &highest)),
            Err(AgentActivityCoverageErrorV2::MatchingActivityBeyondValidHead)
        );
    }
}
