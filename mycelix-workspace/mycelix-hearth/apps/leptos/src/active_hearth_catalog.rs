// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Source-backed catalog of Hearths in which the connected agent is currently Active.
//!
//! `hearth_kinship.get_my_hearths` is discovery/history evidence: historical
//! `AgentToHearths` links can survive after a membership becomes `Departed`.
//! This provider therefore verifies every discovered Hearth independently with
//! `get_caller_role`. Only `Some(role)` establishes current Active membership.
//!
//! There is one additional fail-closed boundary: `get_my_hearths` follows Hearth
//! update chains and currently returns only the latest Record, while membership
//! links are keyed by the stable Hearth ActionHash. Until Kinship exposes stable
//! identity and latest display state together, an Update record is rejected as
//! identity-ambiguous instead of using its newer ActionHash as membership truth.
//!
//! The resulting catalog is read evidence only. It does not grant mutation
//! authority; every consequential zome call must continue enforcing membership
//! and role at the source.

use crate::record_bridge::{self, WireRecord};
use hearth_leptos_types::{HearthType, HearthView, MemberRole};
use leptos::prelude::*;
use mycelix_leptos_client::HoloHashBytes;
use mycelix_leptos_core::holochain_provider::use_holochain;
use mycelix_leptos_core::{AvailabilityStateKind, ConnectionStatus};
use serde::Deserialize;
use std::cell::{Cell, RefCell};
use std::collections::BTreeMap;
use std::rc::Rc;
use wasm_bindgen_futures::spawn_local;

#[derive(Clone, Debug)]
pub struct ActiveHearthCatalogSnapshot {
    /// Completeness of the Active-membership catalog as a whole.
    pub availability: AvailabilityStateKind,
    /// Exact connected AgentPubKey display identity the catalog was established for.
    pub connected_agent: Option<String>,
    /// Complete Active catalog. Deliberately empty unless availability is Live.
    pub hearths: Vec<HearthView>,
    /// Current role proven by `get_caller_role`, keyed by Hearth ActionHash.
    /// Deliberately empty unless availability is Live.
    pub roles: BTreeMap<String, MemberRole>,
    /// Number of records returned by historical discovery.
    pub discovered_records: usize,
    /// Number of unique, well-formed Hearth candidates derived from discovery.
    pub unique_candidates: usize,
    /// Number of candidates for which current Active membership was proven.
    pub verified_active: usize,
    /// Number of malformed discovery records.
    pub malformed_candidates: usize,
    /// Number of latest Hearth Update records rejected because the stable
    /// membership identity is not present in the discovery response.
    pub identity_ambiguous_candidates: usize,
    /// Number of current-membership queries that failed transport/source validation.
    pub membership_query_failures: usize,
}

impl ActiveHearthCatalogSnapshot {
    fn with_state(availability: AvailabilityStateKind, connected_agent: Option<String>) -> Self {
        Self {
            availability,
            connected_agent,
            hearths: Vec::new(),
            roles: BTreeMap::new(),
            discovered_records: 0,
            unique_candidates: 0,
            verified_active: 0,
            malformed_candidates: 0,
            identity_ambiguous_candidates: 0,
            membership_query_failures: 0,
        }
    }

    fn unknown(connected_agent: Option<String>) -> Self {
        Self::with_state(AvailabilityStateKind::Unknown, connected_agent)
    }
}

#[derive(Clone)]
pub struct ActiveHearthCatalogState {
    pub snapshot: RwSignal<ActiveHearthCatalogSnapshot>,
    pub loading: RwSignal<bool>,
    refresh_nonce: RwSignal<u64>,
}

impl ActiveHearthCatalogState {
    /// Request a fresh source-backed catalog read.
    ///
    /// This is reconciliation only; it does not change membership or selection.
    pub fn refresh(&self) {
        self.refresh_nonce
            .update(|nonce| *nonce = nonce.wrapping_add(1));
    }
}

#[derive(Debug, Clone, Deserialize)]
struct WireHearth {
    name: String,
    description: String,
    hearth_type: HearthType,
    created_by: Vec<u8>,
    created_at: i64,
    max_members: u32,
}

/// Holochain Update actions carry `original_action_address`; Create actions do
/// not. The generic browser Record mirror already exposes action content as a
/// map because fields such as `author` are read from the same representation.
fn record_has_identity_ambiguous_update(record: &WireRecord) -> bool {
    record
        .signed_action
        .hashed
        .content
        .get("original_action_address")
        .is_some()
}

fn record_to_hearth(record: &WireRecord) -> Option<HearthView> {
    if record_has_identity_ambiguous_update(record) {
        return None;
    }

    let hash = record.action_hash_b64();
    HoloHashBytes::from_action_raw_base64(&hash).ok()?;
    let hearth: WireHearth = record.decode_entry()?;

    Some(HearthView {
        hash,
        name: hearth.name,
        description: hearth.description,
        hearth_type: hearth.hearth_type,
        created_by: record_bridge::agent_display(&hearth.created_by)?,
        created_at: hearth.created_at / 1_000_000,
        max_members: hearth.max_members,
    })
}

#[derive(Default)]
struct CatalogEvidence {
    discovered_records: usize,
    unique_candidates: usize,
    malformed_candidates: usize,
    identity_ambiguous_candidates: usize,
    membership_queries_succeeded: usize,
    membership_query_failures: usize,
    active: Vec<(HearthView, MemberRole)>,
}

/// Convert a completed probe into a publishable catalog.
///
/// Any malformed candidate, identity ambiguity, failed membership query, or
/// internal count mismatch means completeness was not established. Partial
/// Active results are therefore discarded rather than published under
/// `Degraded` and accidentally consumed as a complete catalog.
fn finalize_catalog(
    connected_agent: String,
    mut evidence: CatalogEvidence,
) -> ActiveHearthCatalogSnapshot {
    let query_shape_complete = evidence.membership_queries_succeeded
        + evidence.membership_query_failures
        == evidence.unique_candidates;
    let complete = evidence.malformed_candidates == 0
        && evidence.identity_ambiguous_candidates == 0
        && evidence.membership_query_failures == 0
        && query_shape_complete;

    let verified_active = evidence.active.len();
    if !complete {
        return ActiveHearthCatalogSnapshot {
            availability: AvailabilityStateKind::Degraded,
            connected_agent: Some(connected_agent),
            hearths: Vec::new(),
            roles: BTreeMap::new(),
            discovered_records: evidence.discovered_records,
            unique_candidates: evidence.unique_candidates,
            verified_active,
            malformed_candidates: evidence.malformed_candidates,
            identity_ambiguous_candidates: evidence.identity_ambiguous_candidates,
            membership_query_failures: evidence.membership_query_failures,
        };
    }

    // Canonicalization is representational only. Selection policy must never use
    // catalog ordering; `hearth_selection` enforces zero/one/many explicitly.
    evidence
        .active
        .sort_by(|(a, _), (b, _)| a.hash.cmp(&b.hash));

    let roles = evidence
        .active
        .iter()
        .map(|(hearth, role)| (hearth.hash.clone(), role.clone()))
        .collect::<BTreeMap<_, _>>();
    let hearths = evidence
        .active
        .into_iter()
        .map(|(hearth, _)| hearth)
        .collect::<Vec<_>>();
    let availability = if hearths.is_empty() {
        AvailabilityStateKind::Empty
    } else {
        AvailabilityStateKind::Live
    };

    ActiveHearthCatalogSnapshot {
        availability,
        connected_agent: Some(connected_agent),
        hearths,
        roles,
        discovered_records: evidence.discovered_records,
        unique_candidates: evidence.unique_candidates,
        verified_active,
        malformed_candidates: 0,
        identity_ambiguous_candidates: 0,
        membership_query_failures: 0,
    }
}

fn invalidate(generation: &Rc<Cell<u64>>) {
    generation.set(generation.get().wrapping_add(1));
}

async fn load_active_catalog(
    hc: mycelix_leptos_core::HolochainCtx,
    connected_agent: String,
    generation: Rc<Cell<u64>>,
    token: u64,
) -> Option<ActiveHearthCatalogSnapshot> {
    let records = match hc
        .call_zome_default::<(), Vec<WireRecord>>("hearth_kinship", "get_my_hearths", &())
        .await
    {
        Ok(records) => {
            if generation.get() != token {
                return None;
            }
            records
        }
        Err(error) => {
            if generation.get() != token {
                return None;
            }
            web_sys::console::log_1(
                &format!("[Hearth] Active catalog discovery failed: {error}").into(),
            );
            return Some(ActiveHearthCatalogSnapshot::with_state(
                AvailabilityStateKind::Unavailable,
                Some(connected_agent),
            ));
        }
    };

    if records.is_empty() {
        return Some(ActiveHearthCatalogSnapshot {
            availability: AvailabilityStateKind::Empty,
            connected_agent: Some(connected_agent),
            hearths: Vec::new(),
            roles: BTreeMap::new(),
            discovered_records: 0,
            unique_candidates: 0,
            verified_active: 0,
            malformed_candidates: 0,
            identity_ambiguous_candidates: 0,
            membership_query_failures: 0,
        });
    }

    let mut evidence = CatalogEvidence {
        discovered_records: records.len(),
        ..CatalogEvidence::default()
    };
    let mut candidates = BTreeMap::<String, HearthView>::new();

    for record in &records {
        if record_has_identity_ambiguous_update(record) {
            evidence.identity_ambiguous_candidates += 1;
            continue;
        }

        match record_to_hearth(record) {
            Some(hearth) => {
                candidates.entry(hearth.hash.clone()).or_insert(hearth);
            }
            None => evidence.malformed_candidates += 1,
        }
    }
    evidence.unique_candidates = candidates.len();

    for (hearth_hash_text, hearth) in candidates {
        let hearth_hash = match HoloHashBytes::from_action_raw_base64(&hearth_hash_text) {
            Ok(hash) => hash,
            Err(_) => {
                // `record_to_hearth` already validates this; retain fail-closed
                // behavior if that invariant ever changes.
                evidence.malformed_candidates += 1;
                continue;
            }
        };

        let role = hc
            .call_zome_default::<HoloHashBytes, Option<MemberRole>>(
                "hearth_kinship",
                "get_caller_role",
                &hearth_hash,
            )
            .await;

        if generation.get() != token {
            return None;
        }

        match role {
            Ok(Some(role)) => {
                evidence.membership_queries_succeeded += 1;
                evidence.active.push((hearth, role));
            }
            Ok(None) => {
                // A historical/departed discovery link is a valid negative
                // result, not a source failure.
                evidence.membership_queries_succeeded += 1;
            }
            Err(error) => {
                evidence.membership_query_failures += 1;
                web_sys::console::log_1(
                    &format!(
                        "[Hearth] Active membership proof failed for {hearth_hash_text}: {error}"
                    )
                    .into(),
                );
            }
        }
    }

    Some(finalize_catalog(connected_agent, evidence))
}

pub fn provide_active_hearth_catalog() -> ActiveHearthCatalogState {
    let state = ActiveHearthCatalogState {
        snapshot: RwSignal::new(ActiveHearthCatalogSnapshot::unknown(None)),
        loading: RwSignal::new(false),
        refresh_nonce: RwSignal::new(0),
    };
    provide_context(state.clone());

    let hc = use_holochain();
    let generation = Rc::new(Cell::new(0u64));
    let last_key = Rc::new(RefCell::new(None::<String>));

    let state_effect = state.clone();
    let hc_effect = hc.clone();
    let generation_effect = generation.clone();
    let key_effect = last_key.clone();

    Effect::new(move |_| {
        let status = hc_effect.status.get();
        let signer_ready = hc_effect.zome_call_signing_ready.get();
        let refresh_nonce = state_effect.refresh_nonce.get();

        match status {
            ConnectionStatus::Mock => {
                invalidate(&generation_effect);
                *key_effect.borrow_mut() = None;
                state_effect.loading.set(false);
                state_effect.snapshot.set(ActiveHearthCatalogSnapshot::with_state(
                    AvailabilityStateKind::Mock,
                    None,
                ));
                return;
            }
            ConnectionStatus::Disconnected | ConnectionStatus::Reconnecting => {
                invalidate(&generation_effect);
                *key_effect.borrow_mut() = None;
                state_effect.loading.set(false);
                state_effect.snapshot.set(ActiveHearthCatalogSnapshot::with_state(
                    AvailabilityStateKind::Degraded,
                    None,
                ));
                return;
            }
            ConnectionStatus::Connecting => {
                invalidate(&generation_effect);
                *key_effect.borrow_mut() = None;
                state_effect.loading.set(false);
                state_effect
                    .snapshot
                    .set(ActiveHearthCatalogSnapshot::unknown(None));
                return;
            }
            ConnectionStatus::Connected if !signer_ready => {
                invalidate(&generation_effect);
                *key_effect.borrow_mut() = None;
                state_effect.loading.set(false);
                state_effect.snapshot.set(ActiveHearthCatalogSnapshot::with_state(
                    AvailabilityStateKind::Unavailable,
                    None,
                ));
                return;
            }
            ConnectionStatus::Connected => {}
        }

        let Some(connected_agent) = hc_effect.connected_agent_pub_key_b64() else {
            invalidate(&generation_effect);
            *key_effect.borrow_mut() = None;
            state_effect.loading.set(false);
            state_effect.snapshot.set(ActiveHearthCatalogSnapshot::with_state(
                AvailabilityStateKind::Degraded,
                None,
            ));
            return;
        };
        if HoloHashBytes::from_agent_display(&connected_agent).is_err() {
            invalidate(&generation_effect);
            *key_effect.borrow_mut() = None;
            state_effect.loading.set(false);
            state_effect.snapshot.set(ActiveHearthCatalogSnapshot::with_state(
                AvailabilityStateKind::Degraded,
                Some(connected_agent),
            ));
            return;
        }

        let key = format!("{connected_agent}|{refresh_nonce}");
        if key_effect.borrow().as_deref() == Some(&key) {
            return;
        }
        *key_effect.borrow_mut() = Some(key);

        invalidate(&generation_effect);
        let token = generation_effect.get();
        state_effect.loading.set(true);
        state_effect
            .snapshot
            .set(ActiveHearthCatalogSnapshot::unknown(Some(connected_agent.clone())));

        let state_load = state_effect.clone();
        let hc_load = hc_effect.clone();
        let generation_load = generation_effect.clone();
        spawn_local(async move {
            let result = load_active_catalog(
                hc_load,
                connected_agent,
                generation_load.clone(),
                token,
            )
            .await;

            if generation_load.get() != token {
                return;
            }
            if let Some(snapshot) = result {
                state_load.snapshot.set(snapshot);
                state_load.loading.set(false);
            }
        });
    });

    state
}

pub fn use_active_hearth_catalog() -> ActiveHearthCatalogState {
    expect_context::<ActiveHearthCatalogState>()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::record_bridge::{WireHashedAction, WireRecordEntry, WireSignedAction};
    use hearth_leptos_types::HearthType;
    use mycelix_leptos_client::{HoloHashKind, HOLO_HASH_WIRE_LEN};

    fn hash_of_kind(kind: HoloHashKind, discriminator: u8) -> HoloHashBytes {
        let mut bytes = vec![0u8; HOLO_HASH_WIRE_LEN];
        bytes[..3].copy_from_slice(&kind.prefix());
        bytes[3] = discriminator;
        HoloHashBytes::from_raw_39(bytes).expect("test hash must be valid")
    }

    fn agent(discriminator: u8) -> String {
        hash_of_kind(HoloHashKind::Agent, discriminator).to_holochain_display()
    }

    fn hearth(discriminator: u8, name: &str) -> HearthView {
        HearthView {
            hash: hash_of_kind(HoloHashKind::Action, discriminator).to_raw_base64(),
            name: name.to_string(),
            description: String::new(),
            hearth_type: HearthType::Chosen,
            created_by: agent(90),
            created_at: discriminator as i64,
            max_members: 10,
        }
    }

    fn record_with_action_content(content: serde_json::Value) -> WireRecord {
        WireRecord {
            signed_action: WireSignedAction {
                hashed: WireHashedAction {
                    hash: hash_of_kind(HoloHashKind::Action, 7).into_raw_39(),
                    content,
                },
                signature: serde_json::Value::Null,
            },
            entry: WireRecordEntry::Other(serde_json::Value::Null),
        }
    }

    #[test]
    fn update_record_is_identity_ambiguous_until_source_returns_stable_hash() {
        let create = record_with_action_content(serde_json::json!({
            "author": []
        }));
        let update = record_with_action_content(serde_json::json!({
            "author": [],
            "original_action_address": []
        }));

        assert!(!record_has_identity_ambiguous_update(&create));
        assert!(record_has_identity_ambiguous_update(&update));
    }

    #[test]
    fn all_historical_candidates_establish_empty_active_catalog() {
        let snapshot = finalize_catalog(
            agent(1),
            CatalogEvidence {
                discovered_records: 2,
                unique_candidates: 2,
                membership_queries_succeeded: 2,
                ..CatalogEvidence::default()
            },
        );

        assert_eq!(snapshot.availability, AvailabilityStateKind::Empty);
        assert!(snapshot.hearths.is_empty());
        assert_eq!(snapshot.discovered_records, 2);
        assert_eq!(snapshot.verified_active, 0);
    }

    #[test]
    fn complete_active_proofs_publish_live_catalog_and_roles_together() {
        let a = hearth(1, "A");
        let b = hearth(2, "B");
        let snapshot = finalize_catalog(
            agent(1),
            CatalogEvidence {
                discovered_records: 2,
                unique_candidates: 2,
                membership_queries_succeeded: 2,
                active: vec![(b.clone(), MemberRole::Adult), (a.clone(), MemberRole::Founder)],
                ..CatalogEvidence::default()
            },
        );

        assert_eq!(snapshot.availability, AvailabilityStateKind::Live);
        assert_eq!(snapshot.hearths.len(), 2);
        assert_eq!(snapshot.hearths[0].hash, a.hash);
        assert_eq!(snapshot.hearths[1].hash, b.hash);
        assert_eq!(snapshot.roles.get(&a.hash), Some(&MemberRole::Founder));
        assert_eq!(snapshot.roles.get(&b.hash), Some(&MemberRole::Adult));
    }

    #[test]
    fn one_failed_membership_query_discards_partial_active_catalog() {
        let active = hearth(1, "A");
        let snapshot = finalize_catalog(
            agent(1),
            CatalogEvidence {
                discovered_records: 2,
                unique_candidates: 2,
                membership_queries_succeeded: 1,
                membership_query_failures: 1,
                active: vec![(active, MemberRole::Adult)],
                ..CatalogEvidence::default()
            },
        );

        assert_eq!(snapshot.availability, AvailabilityStateKind::Degraded);
        assert!(snapshot.hearths.is_empty());
        assert!(snapshot.roles.is_empty());
        assert_eq!(snapshot.verified_active, 1);
        assert_eq!(snapshot.membership_query_failures, 1);
    }

    #[test]
    fn malformed_discovery_candidate_prevents_complete_catalog_claim() {
        let snapshot = finalize_catalog(
            agent(1),
            CatalogEvidence {
                discovered_records: 2,
                unique_candidates: 1,
                malformed_candidates: 1,
                membership_queries_succeeded: 1,
                active: vec![(hearth(1, "A"), MemberRole::Adult)],
                ..CatalogEvidence::default()
            },
        );

        assert_eq!(snapshot.availability, AvailabilityStateKind::Degraded);
        assert!(snapshot.hearths.is_empty());
        assert_eq!(snapshot.malformed_candidates, 1);
    }

    #[test]
    fn updated_hearth_identity_ambiguity_prevents_complete_catalog_claim() {
        let snapshot = finalize_catalog(
            agent(1),
            CatalogEvidence {
                discovered_records: 2,
                unique_candidates: 1,
                identity_ambiguous_candidates: 1,
                membership_queries_succeeded: 1,
                active: vec![(hearth(1, "A"), MemberRole::Adult)],
                ..CatalogEvidence::default()
            },
        );

        assert_eq!(snapshot.availability, AvailabilityStateKind::Degraded);
        assert!(snapshot.hearths.is_empty());
        assert_eq!(snapshot.identity_ambiguous_candidates, 1);
    }

    #[test]
    fn incomplete_query_shape_cannot_be_mislabeled_empty() {
        let snapshot = finalize_catalog(
            agent(1),
            CatalogEvidence {
                discovered_records: 2,
                unique_candidates: 2,
                membership_queries_succeeded: 1,
                ..CatalogEvidence::default()
            },
        );

        assert_eq!(snapshot.availability, AvailabilityStateKind::Degraded);
        assert!(snapshot.hearths.is_empty());
    }
}
