// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Fail-closed binding between the source-backed Active Hearth catalog and the
//! pure browser selection kernel.
//!
//! The selection kernel deliberately accepts a normalized catalog slice. This
//! adapter preserves source identity/completeness evidence that would otherwise
//! be lost at that seam. A positive/empty selection result is therefore possible
//! only when the catalog snapshot is bound to the exact current AgentPubKey and
//! its published accounting/payload shape is internally coherent.

use crate::active_hearth_catalog::ActiveHearthCatalogSnapshot;
use crate::hearth_selection::{
    HearthSelectionResolution, RememberedHearthPreference, resolve_active_hearth_selection,
};
use mycelix_leptos_client::HoloHashBytes;
use mycelix_leptos_core::AvailabilityStateKind;
use std::collections::BTreeSet;

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum CatalogSelectionBindingProblem {
    InvalidCurrentAgent,
    MissingCatalogAgent,
    CatalogAgentMismatch,
    SourceAccountingMismatch,
    SourceDefectsInSelectableState,
    ActivePayloadCountMismatch,
    RoleMapMismatch,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum BoundHearthSelectionResolution {
    Resolved(HearthSelectionResolution),
    Degraded(CatalogSelectionBindingProblem),
}

/// Resolve selection from a complete catalog snapshot without discarding the
/// provenance/accounting information required to decide whether that snapshot
/// is safe to use as positive selection input.
///
/// Non-selectable source states remain non-selectable and are forwarded to the
/// pure kernel. `Live` and `Empty` are stronger claims: before either may drive
/// selection/no-selection, this adapter requires exact current-agent binding,
/// closed source accounting, zero recorded source defects, and coherent
/// payload/role shape.
pub fn resolve_bound_active_hearth_selection(
    snapshot: &ActiveHearthCatalogSnapshot,
    current_connected_agent: &str,
    remembered: Option<&RememberedHearthPreference>,
) -> BoundHearthSelectionResolution {
    if !matches!(
        snapshot.availability,
        AvailabilityStateKind::Live | AvailabilityStateKind::Empty
    ) {
        return BoundHearthSelectionResolution::Resolved(resolve_active_hearth_selection(
            snapshot.availability,
            &snapshot.hearths,
            current_connected_agent,
            remembered,
        ));
    }

    if HoloHashBytes::from_agent_display(current_connected_agent).is_err() {
        return BoundHearthSelectionResolution::Degraded(
            CatalogSelectionBindingProblem::InvalidCurrentAgent,
        );
    }

    let Some(catalog_agent) = snapshot.connected_agent.as_deref() else {
        return BoundHearthSelectionResolution::Degraded(
            CatalogSelectionBindingProblem::MissingCatalogAgent,
        );
    };
    if catalog_agent != current_connected_agent {
        return BoundHearthSelectionResolution::Degraded(
            CatalogSelectionBindingProblem::CatalogAgentMismatch,
        );
    }

    let accounted = snapshot
        .unique_candidates
        .checked_add(snapshot.malformed_candidates)
        .and_then(|value| value.checked_add(snapshot.identity_ambiguous_candidates));
    if accounted != Some(snapshot.discovered_records) {
        // This catches, among other things, duplicate discovery identities that
        // were silently normalized before publication. Duplicate source identity
        // is not evidence of wrongdoing; it is simply not a complete unambiguous
        // catalog for positive selection.
        return BoundHearthSelectionResolution::Degraded(
            CatalogSelectionBindingProblem::SourceAccountingMismatch,
        );
    }

    if snapshot.malformed_candidates != 0
        || snapshot.identity_ambiguous_candidates != 0
        || snapshot.membership_query_failures != 0
    {
        return BoundHearthSelectionResolution::Degraded(
            CatalogSelectionBindingProblem::SourceDefectsInSelectableState,
        );
    }

    match snapshot.availability {
        AvailabilityStateKind::Empty => {
            if snapshot.verified_active != 0
                || !snapshot.hearths.is_empty()
                || !snapshot.roles.is_empty()
            {
                return BoundHearthSelectionResolution::Degraded(
                    CatalogSelectionBindingProblem::ActivePayloadCountMismatch,
                );
            }
        }
        AvailabilityStateKind::Live => {
            if snapshot.hearths.is_empty()
                || snapshot.verified_active != snapshot.hearths.len()
                || snapshot.roles.len() != snapshot.hearths.len()
            {
                return BoundHearthSelectionResolution::Degraded(
                    CatalogSelectionBindingProblem::ActivePayloadCountMismatch,
                );
            }

            let hearth_hashes = snapshot
                .hearths
                .iter()
                .map(|hearth| hearth.hash.as_str())
                .collect::<BTreeSet<_>>();
            let role_hashes = snapshot
                .roles
                .keys()
                .map(String::as_str)
                .collect::<BTreeSet<_>>();
            if hearth_hashes.len() != snapshot.hearths.len() || hearth_hashes != role_hashes {
                return BoundHearthSelectionResolution::Degraded(
                    CatalogSelectionBindingProblem::RoleMapMismatch,
                );
            }
        }
        _ => unreachable!("non-selectable source states returned before binding checks"),
    }

    BoundHearthSelectionResolution::Resolved(resolve_active_hearth_selection(
        snapshot.availability,
        &snapshot.hearths,
        current_connected_agent,
        remembered,
    ))
}

#[cfg(test)]
mod tests {
    use super::*;
    use hearth_leptos_types::{HearthType, HearthView, MemberRole};
    use mycelix_leptos_client::{HOLO_HASH_WIRE_LEN, HoloHashKind};
    use std::collections::BTreeMap;

    fn hash_of_kind(kind: HoloHashKind, discriminator: u8) -> HoloHashBytes {
        let mut bytes = vec![0u8; HOLO_HASH_WIRE_LEN];
        bytes[..3].copy_from_slice(&kind.prefix());
        bytes[3] = discriminator;
        HoloHashBytes::from_raw_39(bytes).expect("test hash must be valid")
    }

    fn agent(discriminator: u8) -> String {
        hash_of_kind(HoloHashKind::Agent, discriminator).to_holochain_display()
    }

    fn hearth(discriminator: u8) -> HearthView {
        HearthView {
            hash: hash_of_kind(HoloHashKind::Action, discriminator).to_raw_base64(),
            name: format!("Hearth {discriminator}"),
            description: String::new(),
            hearth_type: HearthType::Chosen,
            created_by: agent(90),
            created_at: discriminator as i64,
            max_members: 10,
        }
    }

    fn live_snapshot(current_agent: &str, hearths: Vec<HearthView>) -> ActiveHearthCatalogSnapshot {
        let roles = hearths
            .iter()
            .map(|hearth| (hearth.hash.clone(), MemberRole::Adult))
            .collect::<BTreeMap<_, _>>();
        ActiveHearthCatalogSnapshot {
            availability: AvailabilityStateKind::Live,
            connected_agent: Some(current_agent.to_string()),
            discovered_records: hearths.len(),
            unique_candidates: hearths.len(),
            verified_active: hearths.len(),
            hearths,
            roles,
            malformed_candidates: 0,
            identity_ambiguous_candidates: 0,
            membership_query_failures: 0,
        }
    }

    #[test]
    fn complete_live_snapshot_reaches_pure_selection_kernel() {
        let current_agent = agent(1);
        let only = hearth(1);
        let snapshot = live_snapshot(&current_agent, vec![only.clone()]);

        assert_eq!(
            resolve_bound_active_hearth_selection(&snapshot, &current_agent, None),
            BoundHearthSelectionResolution::Resolved(HearthSelectionResolution::Selected {
                hearth_hash: only.hash,
                source: crate::hearth_selection::SelectionSource::OnlyActiveHearth,
            })
        );
    }

    #[test]
    fn silent_duplicate_accounting_cannot_become_selectable_live_catalog() {
        let current_agent = agent(1);
        let mut snapshot = live_snapshot(&current_agent, vec![hearth(1)]);
        snapshot.discovered_records = 2;
        snapshot.unique_candidates = 1;

        assert_eq!(
            resolve_bound_active_hearth_selection(&snapshot, &current_agent, None),
            BoundHearthSelectionResolution::Degraded(
                CatalogSelectionBindingProblem::SourceAccountingMismatch
            )
        );
    }

    #[test]
    fn catalog_bound_to_another_agent_cannot_drive_selection() {
        let current_agent = agent(1);
        let snapshot = live_snapshot(&agent(2), vec![hearth(1)]);

        assert_eq!(
            resolve_bound_active_hearth_selection(&snapshot, &current_agent, None),
            BoundHearthSelectionResolution::Degraded(
                CatalogSelectionBindingProblem::CatalogAgentMismatch
            )
        );
    }

    #[test]
    fn live_payload_and_role_map_must_cover_the_same_hearths() {
        let current_agent = agent(1);
        let mut snapshot = live_snapshot(&current_agent, vec![hearth(1)]);
        snapshot.roles.clear();

        assert_eq!(
            resolve_bound_active_hearth_selection(&snapshot, &current_agent, None),
            BoundHearthSelectionResolution::Degraded(
                CatalogSelectionBindingProblem::ActivePayloadCountMismatch
            )
        );
    }

    #[test]
    fn clean_historical_only_catalog_can_establish_no_active_hearths() {
        let current_agent = agent(1);
        let snapshot = ActiveHearthCatalogSnapshot {
            availability: AvailabilityStateKind::Empty,
            connected_agent: Some(current_agent.clone()),
            hearths: Vec::new(),
            roles: BTreeMap::new(),
            discovered_records: 2,
            unique_candidates: 2,
            verified_active: 0,
            malformed_candidates: 0,
            identity_ambiguous_candidates: 0,
            membership_query_failures: 0,
        };

        assert_eq!(
            resolve_bound_active_hearth_selection(&snapshot, &current_agent, None),
            BoundHearthSelectionResolution::Resolved(
                HearthSelectionResolution::NoActiveHearths
            )
        );
    }

    #[test]
    fn empty_catalog_still_requires_valid_current_agent_binding() {
        let snapshot = ActiveHearthCatalogSnapshot {
            availability: AvailabilityStateKind::Empty,
            connected_agent: Some("not-an-agent".to_string()),
            hearths: Vec::new(),
            roles: BTreeMap::new(),
            discovered_records: 0,
            unique_candidates: 0,
            verified_active: 0,
            malformed_candidates: 0,
            identity_ambiguous_candidates: 0,
            membership_query_failures: 0,
        };

        assert_eq!(
            resolve_bound_active_hearth_selection(&snapshot, "not-an-agent", None),
            BoundHearthSelectionResolution::Degraded(
                CatalogSelectionBindingProblem::InvalidCurrentAgent
            )
        );
    }

    #[test]
    fn non_selectable_source_state_remains_blocked_without_promotion() {
        let snapshot = ActiveHearthCatalogSnapshot {
            availability: AvailabilityStateKind::Unavailable,
            connected_agent: None,
            hearths: Vec::new(),
            roles: BTreeMap::new(),
            discovered_records: 0,
            unique_candidates: 0,
            verified_active: 0,
            malformed_candidates: 0,
            identity_ambiguous_candidates: 0,
            membership_query_failures: 0,
        };

        assert_eq!(
            resolve_bound_active_hearth_selection(&snapshot, "not-an-agent", None),
            BoundHearthSelectionResolution::Resolved(HearthSelectionResolution::Blocked(
                AvailabilityStateKind::Unavailable
            ))
        );
    }
}
