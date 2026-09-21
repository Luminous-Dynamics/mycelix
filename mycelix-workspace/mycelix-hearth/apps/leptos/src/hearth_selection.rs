// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Pure selection semantics for an already-authoritative Active Hearth catalog.
//!
//! This module deliberately performs no conductor calls and grants no write
//! authority. The Kinship source is responsible for establishing current Active
//! membership; this kernel decides only whether the browser may auto-select one
//! of those established Hearths or must ask the user to choose.

use hearth_leptos_types::HearthView;
use mycelix_leptos_client::HoloHashBytes;
use mycelix_leptos_core::AvailabilityStateKind;
use std::collections::BTreeSet;

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RememberedHearthPreference {
    pub agent: String,
    pub hearth_hash: String,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SelectionSource {
    OnlyActiveHearth,
    RememberedPreference,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum HearthSelectionProblem {
    InvalidConnectedAgent,
    CatalogStateMismatch,
    InvalidHearthHash,
    DuplicateHearthHash,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum HearthSelectionResolution {
    /// The authoritative catalog is not established strongly enough to select.
    Blocked(AvailabilityStateKind),
    /// Explicit Demo provenance. Live selection semantics are not inferred.
    Mock,
    /// A successful authoritative catalog established zero Active Hearths.
    NoActiveHearths,
    /// Exactly one Active Hearth, or a valid remembered preference among many.
    Selected {
        hearth_hash: String,
        source: SelectionSource,
    },
    /// Multiple Active Hearths and no valid remembered preference.
    ChoiceRequired { hearth_hashes: Vec<String> },
    /// The catalog/source shape is internally inconsistent or malformed.
    Degraded(HearthSelectionProblem),
}

/// Resolve browser selection from an already-authoritative Active Hearth catalog.
///
/// This function does not decide whether a caller is an Active member. Its
/// `catalog` input must come from a source that already established that fact.
/// A remembered preference is navigation convenience only: it is accepted only
/// when scoped to the exact connected AgentPubKey and exactly matching one of
/// the freshly established Active Hearth ActionHashes.
pub fn resolve_active_hearth_selection(
    catalog_state: AvailabilityStateKind,
    catalog: &[HearthView],
    connected_agent: &str,
    remembered: Option<&RememberedHearthPreference>,
) -> HearthSelectionResolution {
    match catalog_state {
        AvailabilityStateKind::Mock => return HearthSelectionResolution::Mock,
        AvailabilityStateKind::Unknown
        | AvailabilityStateKind::Unavailable
        | AvailabilityStateKind::Locked
        | AvailabilityStateKind::Degraded => {
            return HearthSelectionResolution::Blocked(catalog_state);
        }
        AvailabilityStateKind::Empty => {
            return if catalog.is_empty() {
                HearthSelectionResolution::NoActiveHearths
            } else {
                HearthSelectionResolution::Degraded(
                    HearthSelectionProblem::CatalogStateMismatch,
                )
            };
        }
        AvailabilityStateKind::Live => {}
    }

    if HoloHashBytes::from_agent_display(connected_agent).is_err() {
        return HearthSelectionResolution::Degraded(
            HearthSelectionProblem::InvalidConnectedAgent,
        );
    }

    if catalog.is_empty() {
        return HearthSelectionResolution::Degraded(HearthSelectionProblem::CatalogStateMismatch);
    }

    let mut hashes = Vec::with_capacity(catalog.len());
    let mut seen = BTreeSet::new();

    for hearth in catalog {
        if HoloHashBytes::from_action_raw_base64(&hearth.hash).is_err() {
            return HearthSelectionResolution::Degraded(
                HearthSelectionProblem::InvalidHearthHash,
            );
        }
        if !seen.insert(hearth.hash.clone()) {
            return HearthSelectionResolution::Degraded(
                HearthSelectionProblem::DuplicateHearthHash,
            );
        }
        hashes.push(hearth.hash.clone());
    }

    if hashes.len() == 1 {
        return HearthSelectionResolution::Selected {
            hearth_hash: hashes.remove(0),
            source: SelectionSource::OnlyActiveHearth,
        };
    }

    if let Some(preference) = remembered
        && preference.agent == connected_agent
        && HoloHashBytes::from_action_raw_base64(&preference.hearth_hash).is_ok()
        && seen.contains(&preference.hearth_hash)
    {
        return HearthSelectionResolution::Selected {
            hearth_hash: preference.hearth_hash.clone(),
            source: SelectionSource::RememberedPreference,
        };
    }

    // Canonicalize only the choice-list representation. This is not a selection
    // rule: with >1 Hearth, no item is promoted merely because it sorts first.
    hashes.sort();
    HearthSelectionResolution::ChoiceRequired {
        hearth_hashes: hashes,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use hearth_leptos_types::HearthType;
    use mycelix_leptos_client::{HoloHashKind, HOLO_HASH_WIRE_LEN};

    fn hash_of_kind(kind: HoloHashKind, discriminator: u8) -> HoloHashBytes {
        let mut bytes = vec![0u8; HOLO_HASH_WIRE_LEN];
        bytes[..3].copy_from_slice(&kind.prefix());
        bytes[3] = discriminator;
        HoloHashBytes::from_raw_39(bytes).expect("test hash must be 39 bytes")
    }

    fn action_hash(discriminator: u8) -> String {
        hash_of_kind(HoloHashKind::Action, discriminator).to_raw_base64()
    }

    fn agent(discriminator: u8) -> String {
        hash_of_kind(HoloHashKind::Agent, discriminator).to_holochain_display()
    }

    fn hearth(discriminator: u8, name: &str) -> HearthView {
        HearthView {
            hash: action_hash(discriminator),
            name: name.to_string(),
            description: String::new(),
            hearth_type: HearthType::Chosen,
            created_by: agent(90),
            created_at: discriminator as i64,
            max_members: 10,
        }
    }

    #[test]
    fn established_empty_catalog_selects_nothing() {
        assert_eq!(
            resolve_active_hearth_selection(
                AvailabilityStateKind::Empty,
                &[],
                &agent(1),
                None,
            ),
            HearthSelectionResolution::NoActiveHearths
        );
    }

    #[test]
    fn exactly_one_active_hearth_auto_selects() {
        let only = hearth(1, "Only");
        assert_eq!(
            resolve_active_hearth_selection(
                AvailabilityStateKind::Live,
                &[only.clone()],
                &agent(1),
                None,
            ),
            HearthSelectionResolution::Selected {
                hearth_hash: only.hash,
                source: SelectionSource::OnlyActiveHearth,
            }
        );
    }

    #[test]
    fn multiple_hearths_require_choice_without_valid_preference() {
        let a = hearth(1, "A");
        let b = hearth(2, "B");
        let result = resolve_active_hearth_selection(
            AvailabilityStateKind::Live,
            &[b.clone(), a.clone()],
            &agent(1),
            None,
        );

        let mut expected = vec![a.hash, b.hash];
        expected.sort();
        assert_eq!(
            result,
            HearthSelectionResolution::ChoiceRequired {
                hearth_hashes: expected,
            }
        );
    }

    #[test]
    fn source_order_never_selects_one_of_multiple_hearths() {
        let a = hearth(1, "A");
        let b = hearth(2, "B");
        let forward = resolve_active_hearth_selection(
            AvailabilityStateKind::Live,
            &[a.clone(), b.clone()],
            &agent(1),
            None,
        );
        let reverse = resolve_active_hearth_selection(
            AvailabilityStateKind::Live,
            &[b, a],
            &agent(1),
            None,
        );
        assert_eq!(forward, reverse);
        assert!(matches!(
            forward,
            HearthSelectionResolution::ChoiceRequired { .. }
        ));
    }

    #[test]
    fn matching_agent_and_catalog_preference_may_select() {
        let current_agent = agent(1);
        let a = hearth(1, "A");
        let b = hearth(2, "B");
        let preference = RememberedHearthPreference {
            agent: current_agent.clone(),
            hearth_hash: b.hash.clone(),
        };

        assert_eq!(
            resolve_active_hearth_selection(
                AvailabilityStateKind::Live,
                &[a, b.clone()],
                &current_agent,
                Some(&preference),
            ),
            HearthSelectionResolution::Selected {
                hearth_hash: b.hash,
                source: SelectionSource::RememberedPreference,
            }
        );
    }

    #[test]
    fn preference_from_another_agent_is_ignored() {
        let a = hearth(1, "A");
        let b = hearth(2, "B");
        let preference = RememberedHearthPreference {
            agent: agent(2),
            hearth_hash: b.hash.clone(),
        };
        let result = resolve_active_hearth_selection(
            AvailabilityStateKind::Live,
            &[a, b],
            &agent(1),
            Some(&preference),
        );
        assert!(matches!(
            result,
            HearthSelectionResolution::ChoiceRequired { .. }
        ));
    }

    #[test]
    fn departed_or_non_catalog_preference_is_ignored() {
        let a = hearth(1, "A");
        let b = hearth(2, "B");
        let preference = RememberedHearthPreference {
            agent: agent(1),
            hearth_hash: action_hash(99),
        };
        let result = resolve_active_hearth_selection(
            AvailabilityStateKind::Live,
            &[a, b],
            &agent(1),
            Some(&preference),
        );
        assert!(matches!(
            result,
            HearthSelectionResolution::ChoiceRequired { .. }
        ));
    }

    #[test]
    fn malformed_preference_does_not_poison_valid_catalog() {
        let a = hearth(1, "A");
        let b = hearth(2, "B");
        let preference = RememberedHearthPreference {
            agent: agent(1),
            hearth_hash: "not-an-action-hash".to_string(),
        };
        let result = resolve_active_hearth_selection(
            AvailabilityStateKind::Live,
            &[a, b],
            &agent(1),
            Some(&preference),
        );
        assert!(matches!(
            result,
            HearthSelectionResolution::ChoiceRequired { .. }
        ));
    }

    #[test]
    fn malformed_or_duplicate_catalog_degrades_instead_of_selecting() {
        let malformed = HearthView {
            hash: "bad".to_string(),
            name: "bad".to_string(),
            description: String::new(),
            hearth_type: HearthType::Chosen,
            created_by: agent(90),
            created_at: 0,
            max_members: 10,
        };
        assert_eq!(
            resolve_active_hearth_selection(
                AvailabilityStateKind::Live,
                &[malformed],
                &agent(1),
                None,
            ),
            HearthSelectionResolution::Degraded(HearthSelectionProblem::InvalidHearthHash)
        );

        let a = hearth(1, "A");
        let duplicate = HearthView {
            name: "duplicate source record".to_string(),
            ..a.clone()
        };
        assert_eq!(
            resolve_active_hearth_selection(
                AvailabilityStateKind::Live,
                &[a, duplicate],
                &agent(1),
                None,
            ),
            HearthSelectionResolution::Degraded(HearthSelectionProblem::DuplicateHearthHash)
        );
    }

    #[test]
    fn live_empty_shape_mismatch_degrades() {
        assert_eq!(
            resolve_active_hearth_selection(
                AvailabilityStateKind::Live,
                &[],
                &agent(1),
                None,
            ),
            HearthSelectionResolution::Degraded(HearthSelectionProblem::CatalogStateMismatch)
        );
    }

    #[test]
    fn unavailable_or_degraded_catalog_cannot_select() {
        let a = hearth(1, "A");
        assert_eq!(
            resolve_active_hearth_selection(
                AvailabilityStateKind::Unavailable,
                &[a.clone()],
                &agent(1),
                None,
            ),
            HearthSelectionResolution::Blocked(AvailabilityStateKind::Unavailable)
        );
        assert_eq!(
            resolve_active_hearth_selection(
                AvailabilityStateKind::Degraded,
                &[a],
                &agent(1),
                None,
            ),
            HearthSelectionResolution::Blocked(AvailabilityStateKind::Degraded)
        );
    }

    #[test]
    fn wrong_kind_connected_identity_degrades_live_selection() {
        let a = hearth(1, "A");
        let action_as_agent = hash_of_kind(HoloHashKind::Action, 3).to_holochain_display();
        assert_eq!(
            resolve_active_hearth_selection(
                AvailabilityStateKind::Live,
                &[a],
                &action_as_agent,
                None,
            ),
            HearthSelectionResolution::Degraded(
                HearthSelectionProblem::InvalidConnectedAgent
            )
        );
    }
}
