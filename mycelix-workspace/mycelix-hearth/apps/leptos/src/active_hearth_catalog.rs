// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Source-backed catalog of Hearths in which the connected agent is currently Active.
//!
//! The Kinship source contract is authoritative for the catalog shape:
//! hearth_kinship.get_my_active_hearths binds stable Hearth identity,
//! latest display state, current membership provenance, and role in one response.
//!
//! The browser still treats the result as read evidence rather than mutation
//! authority. Every consequential zome call must enforce membership and role at
//! its own source boundary.

use hearth_leptos_types::{ActiveHearthView, HearthView, MemberRole};
use leptos::prelude::*;
use mycelix_leptos_client::HoloHashBytes;
use mycelix_leptos_core::holochain_provider::use_holochain;
use mycelix_leptos_core::{AvailabilityStateKind, ConnectionStatus};
use std::cell::{Cell, RefCell};
use std::collections::{BTreeMap, BTreeSet};
use std::rc::Rc;
use wasm_bindgen_futures::spawn_local;

const ACTIVE_HEARTHS_SOURCE: &str = "hearth_kinship.get_my_active_hearths";

#[derive(Clone, Debug)]
pub struct ActiveHearthCatalogSnapshot {
    pub availability: AvailabilityStateKind,
    pub connected_agent: Option<String>,
    pub hearths: Vec<HearthView>,
    pub roles: BTreeMap<String, MemberRole>,
    pub discovered_records: usize,
    pub unique_candidates: usize,
    pub verified_active: usize,
    pub malformed_candidates: usize,
    pub identity_ambiguous_candidates: usize,
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
    pub fn refresh(&self) {
        self.refresh_nonce
            .update(|nonce| *nonce = nonce.wrapping_add(1));
    }
}

fn source_view_to_hearth(
    view: ActiveHearthView,
    connected_agent: &str,
) -> Option<(HearthView, MemberRole)> {
    if view.agent != connected_agent {
        return None;
    }

    HoloHashBytes::from_action_raw_base64(&view.hearth_hash).ok()?;
    HoloHashBytes::from_action_raw_base64(&view.latest_hearth_record_hash).ok()?;
    HoloHashBytes::from_action_raw_base64(&view.membership_record_hash).ok()?;
    HoloHashBytes::from_agent_display(&view.agent).ok()?;
    HoloHashBytes::from_agent_display(&view.created_by).ok()?;

    Some((
        HearthView {
            hash: view.hearth_hash,
            name: view.name,
            description: view.description,
            hearth_type: view.hearth_type,
            created_by: view.created_by,
            created_at: view.created_at,
            max_members: view.max_members,
        },
        view.role,
    ))
}

fn finalize_source_catalog(
    connected_agent: String,
    records: Vec<ActiveHearthView>,
) -> ActiveHearthCatalogSnapshot {
    let discovered_records = records.len();
    let mut hearths = Vec::with_capacity(discovered_records);
    let mut roles = BTreeMap::new();
    let mut stable_hashes = BTreeSet::new();
    let mut malformed_candidates = 0usize;

    for view in records {
        let Some((hearth, role)) = source_view_to_hearth(view, &connected_agent) else {
            malformed_candidates += 1;
            continue;
        };

        if !stable_hashes.insert(hearth.hash.clone()) {
            malformed_candidates += 1;
            continue;
        }

        roles.insert(hearth.hash.clone(), role);
        hearths.push(hearth);
    }

    let complete = malformed_candidates == 0 && stable_hashes.len() == discovered_records;

    if !complete {
        return ActiveHearthCatalogSnapshot {
            availability: AvailabilityStateKind::Degraded,
            connected_agent: Some(connected_agent),
            hearths: Vec::new(),
            roles: BTreeMap::new(),
            discovered_records,
            unique_candidates: stable_hashes.len(),
            verified_active: hearths.len(),
            malformed_candidates,
            identity_ambiguous_candidates: 0,
            membership_query_failures: 0,
        };
    }

    hearths.sort_by(|a, b| a.hash.cmp(&b.hash));

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
        discovered_records,
        unique_candidates: stable_hashes.len(),
        verified_active: discovered_records,
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
        .call_zome_default::<(), Vec<ActiveHearthView>>(
            "hearth_kinship",
            "get_my_active_hearths",
            &(),
        )
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
                &format!("[Hearth] {ACTIVE_HEARTHS_SOURCE} failed: {error}").into(),
            );
            return Some(ActiveHearthCatalogSnapshot::with_state(
                AvailabilityStateKind::Unavailable,
                Some(connected_agent),
            ));
        }
    };

    Some(finalize_source_catalog(connected_agent, records))
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
    use hearth_leptos_types::HearthType;

    fn action_hash(discriminator: u8) -> String {
        let mut bytes = vec![0u8; 39];
        bytes[..3].copy_from_slice(b"uhCkk");
        bytes[3] = discriminator;
        HoloHashBytes::from_raw_39(bytes)
            .expect("test action hash")
            .to_raw_base64()
    }

    fn agent(discriminator: u8) -> String {
        let mut bytes = vec![0u8; 39];
        bytes[..3].copy_from_slice(b"uhCAk");
        bytes[3] = discriminator;
        HoloHashBytes::from_raw_39(bytes)
            .expect("test agent")
            .to_holochain_display()
    }

    fn source_view(discriminator: u8, source_agent: String) -> ActiveHearthView {
        ActiveHearthView {
            hearth_hash: action_hash(discriminator),
            latest_hearth_record_hash: action_hash(discriminator.wrapping_add(10)),
            membership_record_hash: action_hash(discriminator.wrapping_add(20)),
            agent: source_agent,
            role: MemberRole::Adult,
            name: format!("Hearth {discriminator}"),
            description: String::new(),
            hearth_type: HearthType::Chosen,
            created_by: agent(90),
            created_at: discriminator as i64,
            max_members: 10,
            observed_at: discriminator as i64,
        }
    }

    #[test]
    fn source_contract_keeps_stable_and_latest_record_identities_separate() {
        let view = source_view(1, agent(1));
        assert_ne!(view.hearth_hash, view.latest_hearth_record_hash);
        assert_ne!(view.hearth_hash, view.membership_record_hash);
    }

    #[test]
    fn source_view_must_bind_to_connected_agent() {
        let view = source_view(1, agent(2));
        assert!(source_view_to_hearth(view, &agent(1)).is_none());
    }

    #[test]
    fn complete_source_catalog_is_live_and_uses_stable_identity() {
        let connected = agent(1);
        let records = vec![source_view(2, connected.clone())];
        let snapshot = finalize_source_catalog(connected, records);

        assert_eq!(snapshot.availability, AvailabilityStateKind::Live);
        assert_eq!(snapshot.discovered_records, 1);
        assert_eq!(snapshot.unique_candidates, 1);
        assert_eq!(snapshot.verified_active, 1);
        assert_eq!(snapshot.hearths[0].hash, action_hash(2));
        assert_eq!(snapshot.roles.get(&action_hash(2)), Some(&MemberRole::Adult));
    }

    #[test]
    fn duplicate_stable_identity_degrades_instead_of_deduplicating_authority() {
        let connected = agent(1);
        let records = vec![source_view(2, connected.clone()), source_view(2, connected.clone())];
        let snapshot = finalize_source_catalog(connected, records);

        assert_eq!(snapshot.availability, AvailabilityStateKind::Degraded);
        assert!(snapshot.hearths.is_empty());
        assert_eq!(snapshot.malformed_candidates, 1);
    }

    #[test]
    fn malformed_agent_binding_degrades_the_whole_catalog() {
        let connected = agent(1);
        let mut record = source_view(2, connected.clone());
        record.created_by = "not-an-agent".into();

        let snapshot = finalize_source_catalog(connected, vec![record]);

        assert_eq!(snapshot.availability, AvailabilityStateKind::Degraded);
        assert!(snapshot.hearths.is_empty());
        assert_eq!(snapshot.malformed_candidates, 1);
    }

    #[test]
    fn empty_source_catalog_is_authoritatively_empty() {
        let connected = agent(1);
        let snapshot = finalize_source_catalog(connected, Vec::new());

        assert_eq!(snapshot.availability, AvailabilityStateKind::Empty);
        assert!(snapshot.hearths.is_empty());
        assert!(snapshot.roles.is_empty());
    }
}
