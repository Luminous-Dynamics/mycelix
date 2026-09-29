// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Source-backed catalog of Hearths in which the connected agent is currently Active.
//!
//! Kinship establishes the catalog semantics. The browser performs only
//! defense-in-depth validation and carries the source observation/provenance
//! through the view model.
//!
//! The source observation is not a global-consensus claim: Holochain DHT reads
//! reflect the caller's current observed view, and may be stale relative to
//! peers. The observation timestamp is therefore provenance, not a browser-
//! computed freshness verdict.

use hearth_leptos_types::{
    ActiveHearthCatalogProvenance, ActiveHearthCatalogView, ActiveHearthEvidence, ActiveHearthView,
    HearthView, MemberRole,
};
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
    /// Completeness of the source observation as represented to the UI.
    pub availability: AvailabilityStateKind,
    /// Exact connected AgentPubKey display identity the catalog was established for.
    pub connected_agent: Option<String>,
    /// Source provenance for a successful catalog observation, including empty observations.
    pub provenance: Option<ActiveHearthCatalogProvenance>,
    /// Complete stable-identity Hearth projection retained for the selection seam.
    pub hearths: Vec<HearthView>,
    /// Stable Hearth ActionHash -> current role established by Kinship.
    pub roles: BTreeMap<String, MemberRole>,
    /// Full source evidence, including latest display and membership record hashes.
    pub evidence: Vec<ActiveHearthEvidence>,
    /// Number of source records returned by Kinship.
    pub discovered_records: usize,
    /// Number of unique, well-formed stable Hearth identities accepted.
    pub unique_candidates: usize,
    /// Number of Active memberships established by the source contract.
    pub verified_active: usize,
    /// Number of malformed or internally inconsistent source items.
    pub malformed_candidates: usize,
    /// Retained for compatibility with the old selection accounting seam.
    /// Canonical Kinship resolution now establishes stable identity directly.
    pub identity_ambiguous_candidates: usize,
    /// Retained for compatibility with the old selection accounting seam.
    /// Membership resolution is atomic inside the source endpoint.
    pub membership_query_failures: usize,
}

impl ActiveHearthCatalogSnapshot {
    fn with_state(
        availability: AvailabilityStateKind,
        connected_agent: Option<String>,
    ) -> Self {
        Self {
            availability,
            connected_agent,
            provenance: None,
            hearths: Vec::new(),
            roles: BTreeMap::new(),
            evidence: Vec::new(),
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

/// Validate and normalize one source-backed ActiveHearthView.
///
/// The source endpoint already establishes membership and role. The browser
/// only verifies that the returned wire identities are structurally valid and
/// remain bound to the current connected agent.
fn source_view_to_evidence(
    view: ActiveHearthView,
    connected_agent: &str,
) -> Option<ActiveHearthEvidence> {
    HoloHashBytes::from_action_raw_base64(&view.hearth_hash).ok()?;
    HoloHashBytes::from_action_raw_base64(&view.latest_hearth_record_hash).ok()?;
    HoloHashBytes::from_action_raw_base64(&view.membership_record_hash).ok()?;
    HoloHashBytes::from_agent_display(connected_agent).ok()?;
    HoloHashBytes::from_agent_display(&view.created_by).ok()?;

    // A membership record cannot itself be the Hearth identity.
    if view.membership_record_hash == view.hearth_hash {
        return None;
    }

    let hearth = HearthView {
        // Stable identity, never the latest update action.
        hash: view.hearth_hash,
        name: view.name,
        description: view.description,
        hearth_type: view.hearth_type,
        created_by: view.created_by,
        created_at: view.created_at,
        max_members: view.max_members,
    };

    Some(ActiveHearthEvidence {
        hearth,
        latest_hearth_record_hash: view.latest_hearth_record_hash,
        membership_record_hash: view.membership_record_hash,
        role: view.role,
    })
}

fn degraded_snapshot(
    connected_agent: String,
    provenance: Option<ActiveHearthCatalogProvenance>,
    discovered_records: usize,
    unique_candidates: usize,
    verified_active: usize,
    malformed_candidates: usize,
) -> ActiveHearthCatalogSnapshot {
    ActiveHearthCatalogSnapshot {
        availability: AvailabilityStateKind::Degraded,
        connected_agent: Some(connected_agent),
        provenance,
        hearths: Vec::new(),
        roles: BTreeMap::new(),
        evidence: Vec::new(),
        discovered_records,
        unique_candidates,
        verified_active,
        malformed_candidates,
        identity_ambiguous_candidates: 0,
        membership_query_failures: 0,
    }
}

fn finalize_source_catalog(
    connected_agent: String,
    source: ActiveHearthCatalogView,
) -> ActiveHearthCatalogSnapshot {
    let provenance = ActiveHearthCatalogProvenance {
        source: ACTIVE_HEARTHS_SOURCE.to_string(),
        agent: source.agent.clone(),
        observed_at: source.observed_at,
    };
    let discovered_records = source.hearths.len();

    if source.agent != connected_agent {
        return degraded_snapshot(
            connected_agent,
            Some(provenance),
            discovered_records,
            0,
            0,
            discovered_records,
        );
    }

    let mut evidence = Vec::with_capacity(discovered_records);
    let mut stable_hashes = BTreeSet::new();
    let mut malformed_candidates = 0usize;

    for view in source.hearths {
        let Some(item) = source_view_to_evidence(view, &connected_agent) else {
            malformed_candidates += 1;
            continue;
        };

        if !stable_hashes.insert(item.hearth.hash.clone()) {
            // Duplicate stable identity means the source response is not a
            // canonical catalog. Do not silently choose one representation.
            malformed_candidates += 1;
            continue;
        }

        evidence.push(item);
    }

    let complete = malformed_candidates == 0 && stable_hashes.len() == discovered_records;
    let verified_active = evidence.len();

    if !complete {
        return degraded_snapshot(
            connected_agent,
            Some(provenance),
            discovered_records,
            stable_hashes.len(),
            verified_active,
            malformed_candidates,
        );
    }

    // Ordering is representational only. Selection policy must never use it.
    evidence.sort_by(|a, b| a.hearth.hash.cmp(&b.hearth.hash));

    let roles = evidence
        .iter()
        .map(|item| (item.hearth.hash.clone(), item.role.clone()))
        .collect::<BTreeMap<_, _>>();

    let hearths = evidence
        .iter()
        .map(|item| item.hearth.clone())
        .collect::<Vec<_>>();

    let availability = if hearths.is_empty() {
        AvailabilityStateKind::Empty
    } else {
        AvailabilityStateKind::Live
    };

    ActiveHearthCatalogSnapshot {
        availability,
        connected_agent: Some(connected_agent),
        provenance: Some(provenance),
        hearths,
        roles,
        evidence,
        discovered_records,
        unique_candidates: stable_hashes.len(),
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
    let source = match hc
        .call_zome_default::<(), ActiveHearthCatalogView>(
            "hearth_kinship",
            "get_my_active_hearths",
            &(),
        )
        .await
    {
        Ok(source) => {
            if generation.get() != token {
                return None;
            }
            source
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

    Some(finalize_source_catalog(connected_agent, source))
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
    use mycelix_leptos_client::{HoloHashKind, HOLO_HASH_WIRE_LEN};

    fn hash_of_kind(kind: HoloHashKind, discriminator: u8) -> HoloHashBytes {
        let mut bytes = vec![0u8; HOLO_HASH_WIRE_LEN];
        bytes[..3].copy_from_slice(&kind.prefix());
        bytes[3] = discriminator;
        HoloHashBytes::from_raw_39(bytes).expect("test hash must be valid")
    }

    fn action_hash(discriminator: u8) -> String {
        hash_of_kind(HoloHashKind::Action, discriminator).to_raw_base64()
    }

    fn agent(discriminator: u8) -> String {
        hash_of_kind(HoloHashKind::Agent, discriminator).to_holochain_display()
    }

    fn source_view(discriminator: u8, source_agent: String) -> ActiveHearthView {
        ActiveHearthView {
            hearth_hash: action_hash(discriminator),
            latest_hearth_record_hash: action_hash(discriminator.wrapping_add(10)),
            membership_record_hash: action_hash(discriminator.wrapping_add(20)),
            role: MemberRole::Adult,
            name: format!("Hearth {discriminator}"),
            description: String::new(),
            hearth_type: HearthType::Chosen,
            created_by: agent(90),
            created_at: discriminator as i64,
            max_members: 10,
        }
    }

    fn source(agent: String, observed_at: i64, hearths: Vec<ActiveHearthView>) -> ActiveHearthCatalogView {
        ActiveHearthCatalogView {
            agent,
            observed_at,
            hearths,
        }
    }

    #[test]
    fn source_contract_keeps_stable_and_latest_record_identities_separate() {
        let view = source_view(1, agent(1));
        assert_ne!(view.hearth_hash, view.latest_hearth_record_hash);
        assert_ne!(view.hearth_hash, view.membership_record_hash);
    }

    #[test]
    fn empty_source_observation_retains_provenance() {
        let connected = agent(1);
        let snapshot = finalize_source_catalog(
            connected.clone(),
            source(connected.clone(), 123_456, Vec::new()),
        );

        assert_eq!(snapshot.availability, AvailabilityStateKind::Empty);
        assert!(snapshot.hearths.is_empty());
        assert!(snapshot.evidence.is_empty());
        assert_eq!(
            snapshot.provenance.as_ref().map(|p| p.observed_at),
            Some(123_456)
        );
        assert_eq!(
            snapshot.provenance.as_ref().map(|p| p.agent.as_str()),
            Some(connected.as_str())
        );
        assert_eq!(
            snapshot.provenance.as_ref().map(|p| p.source.as_str()),
            Some(ACTIVE_HEARTHS_SOURCE)
        );
    }

    #[test]
    fn complete_source_catalog_is_live_and_preserves_evidence() {
        let connected = agent(1);
        let records = vec![source_view(2, connected.clone())];
        let snapshot = finalize_source_catalog(
            connected.clone(),
            source(connected.clone(), 200, records),
        );

        assert_eq!(snapshot.availability, AvailabilityStateKind::Live);
        assert_eq!(snapshot.discovered_records, 1);
        assert_eq!(snapshot.unique_candidates, 1);
        assert_eq!(snapshot.verified_active, 1);
        assert_eq!(snapshot.hearths[0].hash, action_hash(2));
        assert_eq!(
            snapshot.evidence[0].latest_hearth_record_hash,
            action_hash(12)
        );
        assert_eq!(
            snapshot.evidence[0].membership_record_hash,
            action_hash(22)
        );
        assert_eq!(snapshot.evidence[0].role, MemberRole::Adult);
        assert_eq!(snapshot.provenance.as_ref().map(|p| p.observed_at), Some(200));
    }

    #[test]
    fn source_agent_mismatch_degrades_the_whole_catalog() {
        let connected = agent(1);
        let snapshot = finalize_source_catalog(
            connected,
            source(agent(2), 300, vec![source_view(2, agent(2))]),
        );

        assert_eq!(snapshot.availability, AvailabilityStateKind::Degraded);
        assert!(snapshot.hearths.is_empty());
        assert!(snapshot.evidence.is_empty());
        assert_eq!(snapshot.malformed_candidates, 1);
    }

    #[test]
    fn duplicate_stable_identity_degrades_instead_of_deduplicating_authority() {
        let connected = agent(1);
        let records = vec![
            source_view(2, connected.clone()),
            source_view(2, connected.clone()),
        ];
        let snapshot = finalize_source_catalog(
            connected.clone(),
            source(connected, 400, records),
        );

        assert_eq!(snapshot.availability, AvailabilityStateKind::Degraded);
        assert!(snapshot.hearths.is_empty());
        assert!(snapshot.evidence.is_empty());
        assert_eq!(snapshot.malformed_candidates, 1);
    }

    #[test]
    fn membership_hash_cannot_be_the_stable_hearth_identity() {
        let connected = agent(1);
        let mut record = source_view(2, connected.clone());
        record.membership_record_hash = record.hearth_hash.clone();

        let snapshot = finalize_source_catalog(
            connected.clone(),
            source(connected, 500, vec![record]),
        );

        assert_eq!(snapshot.availability, AvailabilityStateKind::Degraded);
        assert!(snapshot.hearths.is_empty());
        assert_eq!(snapshot.malformed_candidates, 1);
    }

    #[test]
    fn malformed_source_hash_degrades_the_whole_catalog() {
        let connected = agent(1);
        let mut record = source_view(2, connected.clone());
        record.latest_hearth_record_hash = "not-an-action-hash".into();

        let snapshot = finalize_source_catalog(
            connected.clone(),
            source(connected, 600, vec![record]),
        );

        assert_eq!(snapshot.availability, AvailabilityStateKind::Degraded);
        assert!(snapshot.hearths.is_empty());
        assert!(snapshot.evidence.is_empty());
    }

    #[test]
    fn mock_or_unknown_state_has_no_source_provenance() {
        let mock = ActiveHearthCatalogSnapshot::with_state(
            AvailabilityStateKind::Mock,
            None,
        );
        let unknown = ActiveHearthCatalogSnapshot::unknown(None);

        assert!(mock.provenance.is_none());
        assert!(unknown.provenance.is_none());
    }
}
