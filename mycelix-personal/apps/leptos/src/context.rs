// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Personal frontend context.
//!
//! Runtime provenance is explicit: Demo owns illustrative fixture data; Live
//! starts empty and only publishes values returned by typed conductor view
//! endpoints. Reconciliation is additionally bound to a local connection epoch
//! so an asynchronous result from an obsolete conductor/signer session cannot
//! publish into a later one.

use std::collections::HashSet;

use leptos::prelude::*;
use serde::{Deserialize, Serialize};
use wasm_bindgen_futures::spawn_local;

use mycelix_leptos_core::holochain_provider::{use_holochain, HolochainCtx};
use personal_leptos_types::{
    ActivityItemView, BiometricView, ConsentGrantView, DataSharingPreferenceEvidenceView,
    DataSharingPreferenceView, HealthRecordView, MasterKeyView, PreferenceChangeLogView,
    ProfileEvidenceView, ProfileView, StoredCredentialView,
};

use crate::mock_data;
use crate::reconciliation::{ReconciliationEpoch, ReconciliationTransition};
use crate::runtime_mode::PersonalRuntimeMode;

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SymbolRegistry {
    pub hearth_alias: String,
    pub mycel_alias: String,
    pub genesis_alias: String,
    pub orientation: String,
}

impl Default for SymbolRegistry {
    fn default() -> Self {
        Self {
            hearth_alias: "HEARTH".into(),
            mycel_alias: "MYCEL".into(),
            genesis_alias: "Thermodynamic Genesis".into(),
            orientation: "Canonical Mycelix terminology".into(),
        }
    }
}

#[derive(Clone, Copy)]
pub struct CulturalContext {
    pub symbols: RwSignal<SymbolRegistry>,
}

pub fn provide_cultural_context() {
    let symbols = RwSignal::new(SymbolRegistry::default());
    provide_context(CulturalContext { symbols });
}

pub fn use_cultural() -> CulturalContext {
    use_context::<CulturalContext>().expect("CulturalContext not provided")
}

/// What the most recent completed query set established for one Personal source.
///
/// This deliberately does not encode whether the result is still fresh for the
/// current conductor/signer session. See [`PersonalSnapshotFreshness`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PersonalSourceState {
    Demo,
    AwaitingLive,
    LoadingLive,
    Live,
    Empty,
    Degraded,
    Unavailable,
}

/// Freshness/provenance of the aggregate Personal snapshot.
///
/// A previously loaded snapshot can remain visible while `Stale`; callers must
/// not present it as evidence from the current connection epoch.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PersonalSnapshotFreshness {
    Illustrative,
    AwaitingLive,
    Refreshing,
    Current,
    Stale,
}

fn classify_source(successes: usize, failures: usize, present_items: usize) -> PersonalSourceState {
    if successes == 0 && failures > 0 {
        PersonalSourceState::Unavailable
    } else if failures > 0 {
        PersonalSourceState::Degraded
    } else if successes == 0 {
        PersonalSourceState::AwaitingLive
    } else if present_items == 0 {
        PersonalSourceState::Empty
    } else {
        PersonalSourceState::Live
    }
}

fn blank_profile() -> ProfileView {
    ProfileView {
        display_name: String::new(),
        avatar: None,
        bio: None,
        metadata: Default::default(),
        updated_at: 0,
    }
}

#[derive(Debug)]
struct IdentitySourceSnapshot {
    profile: ProfileView,
    profile_action_hash: Option<String>,
    keys: Vec<MasterKeyView>,
    state: PersonalSourceState,
}

fn stage_identity_source<ProfileError, KeysError>(
    profile_result: Result<Option<ProfileEvidenceView>, ProfileError>,
    keys_result: Result<Vec<MasterKeyView>, KeysError>,
) -> Result<IdentitySourceSnapshot, PersonalSourceState> {
    let mut successes = 0;
    let mut failures = 0;
    let mut present_items = 0;

    let (profile, profile_action_hash) = match profile_result {
        Ok(profile) => {
            successes += 1;
            present_items += usize::from(profile.is_some());
            profile
                .map(|evidence| (evidence.profile, Some(evidence.action_hash)))
                .unwrap_or_else(|| (blank_profile(), None))
        }
        Err(_) => {
            failures += 1;
            (blank_profile(), None)
        }
    };

    let keys = match keys_result {
        Ok(keys) => {
            successes += 1;
            present_items += keys.len();
            keys
        }
        Err(_) => {
            failures += 1;
            Vec::new()
        }
    };

    let state = classify_source(successes, failures, present_items);
    if failures > 0 {
        Err(state)
    } else {
        Ok(IdentitySourceSnapshot {
            profile,
            profile_action_hash,
            keys,
            state,
        })
    }
}

fn publish_identity_source(ctx: &PersonalCtx, snapshot: IdentitySourceSnapshot) {
    batch(move || {
        ctx.profile.set(snapshot.profile.clone());
        ctx.draft_profile.set(snapshot.profile);
        ctx.profile_action_hash.set(snapshot.profile_action_hash);
        ctx.keys.set(snapshot.keys);
        ctx.identity_state.set(snapshot.state);
    });
}

#[derive(Debug)]
struct PreferencesSourceSnapshot {
    preferences: Vec<DataSharingPreferenceView>,
    action_hashes: HashSet<String>,
    change_log: Vec<PreferenceChangeLogView>,
    state: PersonalSourceState,
}

fn stage_preferences_source<PreferencesError, LogError>(
    preferences_result: Result<Vec<DataSharingPreferenceEvidenceView>, PreferencesError>,
    log_result: Result<Vec<PreferenceChangeLogView>, LogError>,
) -> Result<PreferencesSourceSnapshot, PersonalSourceState> {
    let mut successes = 0;
    let mut failures = 0;
    let mut present_items = 0;

    let (preferences, action_hashes) = match preferences_result {
        Ok(evidence) => {
            successes += 1;
            present_items += evidence.len();
            let mut action_hashes = HashSet::with_capacity(evidence.len());
            let preferences = evidence
                .into_iter()
                .map(|item| {
                    action_hashes.insert(item.action_hash);
                    item.preference
                })
                .collect();
            (preferences, action_hashes)
        }
        Err(_) => {
            failures += 1;
            (Vec::new(), HashSet::new())
        }
    };

    let change_log = match log_result {
        Ok(log) => {
            successes += 1;
            present_items += log.len();
            log
        }
        Err(_) => {
            failures += 1;
            Vec::new()
        }
    };

    let state = classify_source(successes, failures, present_items);
    if failures > 0 {
        Err(state)
    } else {
        Ok(PreferencesSourceSnapshot {
            preferences,
            action_hashes,
            change_log,
            state,
        })
    }
}

fn publish_preferences_source(ctx: &PersonalCtx, snapshot: PreferencesSourceSnapshot) {
    batch(move || {
        ctx.preferences.set(snapshot.preferences);
        ctx.preference_action_hashes.set(snapshot.action_hashes);
        ctx.preference_log.set(snapshot.change_log);
        ctx.preferences_state.set(snapshot.state);
    });
}

#[derive(Debug)]
struct HealthSourceSnapshot {
    biometrics: Vec<BiometricView>,
    consents: Vec<ConsentGrantView>,
    record_count: usize,
    state: PersonalSourceState,
}

fn stage_health_source<BiometricsError, ConsentsError, RecordsError>(
    biometrics_result: Result<Vec<BiometricView>, BiometricsError>,
    consents_result: Result<Vec<ConsentGrantView>, ConsentsError>,
    records_result: Result<Vec<HealthRecordView>, RecordsError>,
) -> Result<HealthSourceSnapshot, PersonalSourceState> {
    let mut successes = 0;
    let mut failures = 0;
    let mut present_items = 0;

    let biometrics = match biometrics_result {
        Ok(biometrics) => {
            successes += 1;
            present_items += biometrics.len();
            biometrics
        }
        Err(_) => {
            failures += 1;
            Vec::new()
        }
    };

    let consents = match consents_result {
        Ok(consents) => {
            successes += 1;
            present_items += consents.len();
            consents
        }
        Err(_) => {
            failures += 1;
            Vec::new()
        }
    };

    let record_count = match records_result {
        Ok(records) => {
            successes += 1;
            present_items += records.len();
            records.len()
        }
        Err(_) => {
            failures += 1;
            0
        }
    };

    let state = classify_source(successes, failures, present_items);
    if failures > 0 {
        Err(state)
    } else {
        Ok(HealthSourceSnapshot {
            biometrics,
            consents,
            record_count,
            state,
        })
    }
}

fn publish_health_source(ctx: &PersonalCtx, snapshot: HealthSourceSnapshot) {
    batch(move || {
        ctx.biometrics.set(snapshot.biometrics);
        ctx.consents.set(snapshot.consents);
        ctx.health_record_count.set(snapshot.record_count);
        ctx.health_state.set(snapshot.state);
    });
}

#[derive(Clone)]
pub struct PersonalCtx {
    pub runtime_mode: PersonalRuntimeMode,
    pub profile: RwSignal<ProfileView>,
    pub draft_profile: RwSignal<ProfileView>,
    pub profile_action_hash: RwSignal<Option<String>>,
    pub keys: RwSignal<Vec<MasterKeyView>>,
    pub credentials: RwSignal<Vec<StoredCredentialView>>,
    pub biometrics: RwSignal<Vec<BiometricView>>,
    pub consents: RwSignal<Vec<ConsentGrantView>>,
    pub preferences: RwSignal<Vec<DataSharingPreferenceView>>,
    pub preference_action_hashes: RwSignal<HashSet<String>>,
    pub preference_log: RwSignal<Vec<PreferenceChangeLogView>>,
    pub health_record_count: RwSignal<usize>,
    pub activity: RwSignal<Vec<ActivityItemView>>,
    pub identity_state: RwSignal<PersonalSourceState>,
    pub wallet_state: RwSignal<PersonalSourceState>,
    pub health_state: RwSignal<PersonalSourceState>,
    pub preferences_state: RwSignal<PersonalSourceState>,
    pub activity_state: RwSignal<PersonalSourceState>,
    pub snapshot_freshness: RwSignal<PersonalSnapshotFreshness>,
    pub reconciliation: RwSignal<ReconciliationEpoch>,
    pub loading: RwSignal<bool>,
    pub status_note: RwSignal<String>,
}

impl PersonalCtx {
    pub fn source_states(&self) -> [PersonalSourceState; 5] {
        [
            self.identity_state.get(),
            self.wallet_state.get(),
            self.health_state.get(),
            self.preferences_state.get(),
            self.activity_state.get(),
        ]
    }

    fn source_states_untracked(&self) -> [PersonalSourceState; 5] {
        [
            self.identity_state.get_untracked(),
            self.wallet_state.get_untracked(),
            self.health_state.get_untracked(),
            self.preferences_state.get_untracked(),
            self.activity_state.get_untracked(),
        ]
    }

    fn accepts_epoch(&self, epoch: u64) -> bool {
        self.reconciliation.get_untracked().accepts(epoch)
    }

    fn current_usable_epoch(&self) -> Option<u64> {
        let gate = self.reconciliation.get_untracked();
        gate.is_usable().then(|| gate.current_epoch())
    }
}

pub fn provide_personal_context(runtime_mode: PersonalRuntimeMode) {
    let source_state = if runtime_mode.is_demo() {
        PersonalSourceState::Demo
    } else {
        PersonalSourceState::AwaitingLive
    };

    let profile = if runtime_mode.is_demo() {
        mock_data::mock_profile()
    } else {
        blank_profile()
    };

    let ctx = PersonalCtx {
        runtime_mode,
        draft_profile: RwSignal::new(profile.clone()),
        profile: RwSignal::new(profile),
        profile_action_hash: RwSignal::new(None),
        keys: RwSignal::new(if runtime_mode.is_demo() {
            mock_data::mock_keys()
        } else {
            Vec::new()
        }),
        credentials: RwSignal::new(if runtime_mode.is_demo() {
            mock_data::mock_credentials()
        } else {
            Vec::new()
        }),
        biometrics: RwSignal::new(if runtime_mode.is_demo() {
            mock_data::mock_biometrics()
        } else {
            Vec::new()
        }),
        consents: RwSignal::new(if runtime_mode.is_demo() {
            mock_data::mock_consents()
        } else {
            Vec::new()
        }),
        preferences: RwSignal::new(if runtime_mode.is_demo() {
            mock_data::mock_preferences()
        } else {
            Vec::new()
        }),
        preference_action_hashes: RwSignal::new(HashSet::new()),
        preference_log: RwSignal::new(if runtime_mode.is_demo() {
            mock_data::mock_preference_log()
        } else {
            Vec::new()
        }),
        health_record_count: RwSignal::new(if runtime_mode.is_demo() {
            mock_data::mock_health_records()
        } else {
            0
        }),
        activity: RwSignal::new(if runtime_mode.is_demo() {
            mock_data::mock_activity()
        } else {
            Vec::new()
        }),
        identity_state: RwSignal::new(source_state),
        wallet_state: RwSignal::new(source_state),
        health_state: RwSignal::new(source_state),
        preferences_state: RwSignal::new(source_state),
        activity_state: RwSignal::new(source_state),
        snapshot_freshness: RwSignal::new(if runtime_mode.is_demo() {
            PersonalSnapshotFreshness::Illustrative
        } else {
            PersonalSnapshotFreshness::AwaitingLive
        }),
        reconciliation: RwSignal::new(ReconciliationEpoch::default()),
        loading: RwSignal::new(false),
        status_note: RwSignal::new(if runtime_mode.is_demo() {
            "Explicit Demo mode. Personal records shown here are illustrative and are not conductor-backed evidence."
                .into()
        } else {
            "Live mode. Waiting for an authenticated conductor and authorized zome-call signer; no demo records are substituted."
                .into()
        }),
    };

    provide_context(ctx.clone());

    if runtime_mode.is_demo() {
        return;
    }

    let hc = use_holochain();
    let ctx_for_effect = ctx.clone();

    Effect::new(move |_| {
        let usable = hc.zome_calls_ready();
        let mut gate = ctx_for_effect.reconciliation.get_untracked();
        let transition = gate.observe_usable(usable);
        let had_snapshot = gate.has_completed();

        if transition == ReconciliationTransition::None {
            return;
        }

        ctx_for_effect.reconciliation.set(gate);

        match transition {
            ReconciliationTransition::None => {}
            ReconciliationTransition::Start { epoch } => {
                ctx_for_effect.loading.set(true);
                ctx_for_effect
                    .snapshot_freshness
                    .set(PersonalSnapshotFreshness::Refreshing);
                if !had_snapshot {
                    set_all_source_states(&ctx_for_effect, PersonalSourceState::LoadingLive);
                }
                ctx_for_effect.status_note.set(format!(
                    "Reconciling Personal state from live conductor epoch {epoch}. Previously cached values remain non-authoritative until this epoch completes."
                ));

                let ctx = ctx_for_effect.clone();
                let hc = hc.clone();
                spawn_local(async move {
                    hydrate_live(ctx, hc, epoch).await;
                });
            }
            ReconciliationTransition::Invalidated { had_completed, .. } => {
                ctx_for_effect.loading.set(false);
                if had_completed {
                    ctx_for_effect
                        .snapshot_freshness
                        .set(PersonalSnapshotFreshness::Stale);
                    ctx_for_effect.status_note.set(
                        "The live conductor/signer epoch ended. Previously loaded Personal values remain visible as a stale local snapshot and are not current source evidence."
                            .into(),
                    );
                } else {
                    clear_uncommitted_live_snapshot(&ctx_for_effect);
                    ctx_for_effect
                        .snapshot_freshness
                        .set(PersonalSnapshotFreshness::AwaitingLive);
                    set_all_source_states(&ctx_for_effect, PersonalSourceState::AwaitingLive);
                    ctx_for_effect.status_note.set(
                        "Live Personal reconciliation was interrupted before an authoritative snapshot completed. Partial results were discarded; waiting for a usable conductor and signer."
                            .into(),
                    );
                }
            }
        }
    });
}

fn set_all_source_states(ctx: &PersonalCtx, state: PersonalSourceState) {
    ctx.identity_state.set(state);
    ctx.wallet_state.set(state);
    ctx.health_state.set(state);
    ctx.preferences_state.set(state);
    ctx.activity_state.set(state);
}

fn clear_uncommitted_live_snapshot(ctx: &PersonalCtx) {
    let profile = blank_profile();
    ctx.profile.set(profile.clone());
    ctx.draft_profile.set(profile);
    ctx.profile_action_hash.set(None);
    ctx.keys.set(Vec::new());
    ctx.credentials.set(Vec::new());
    ctx.biometrics.set(Vec::new());
    ctx.consents.set(Vec::new());
    ctx.preferences.set(Vec::new());
    ctx.preference_action_hashes.set(HashSet::new());
    ctx.preference_log.set(Vec::new());
    ctx.health_record_count.set(0);
    ctx.activity.set(Vec::new());
}

async fn hydrate_live(ctx: PersonalCtx, hc: HolochainCtx, epoch: u64) {
    if !load_identity_source(&ctx, &hc, epoch).await {
        return;
    }
    if !load_wallet_source(&ctx, &hc, epoch).await {
        return;
    }
    if !load_health_source(&ctx, &hc, epoch).await {
        return;
    }
    if !load_preferences_source(&ctx, &hc, epoch).await {
        return;
    }
    if !load_activity_source(&ctx, &hc, epoch).await {
        return;
    }

    let mut gate = ctx.reconciliation.get_untracked();
    if !gate.finish(epoch) {
        return;
    }
    ctx.reconciliation.set(gate);
    ctx.loading.set(false);
    ctx.snapshot_freshness
        .set(PersonalSnapshotFreshness::Current);

    let states = ctx.source_states_untracked();

    if states.iter().any(|state| {
        matches!(
            state,
            PersonalSourceState::Degraded | PersonalSourceState::Unavailable
        )
    }) {
        ctx.status_note.set(format!(
            "Personal epoch {epoch} completed, but one or more sources could not be fully established. Available results are current for this epoch; missing sources remain explicit."
        ));
    } else if states
        .iter()
        .all(|state| *state == PersonalSourceState::Empty)
    {
        ctx.status_note.set(format!(
            "Personal epoch {epoch} completed with no records. This empty snapshot is authoritative for the completed view queries."
        ));
    } else {
        ctx.status_note.set(format!(
            "Personal epoch {epoch} loaded from typed conductor view endpoints. Successful empty results remain empty and no demo records are substituted."
        ));
    }
}

async fn load_identity_source(ctx: &PersonalCtx, hc: &HolochainCtx, epoch: u64) -> bool {
    let profile_result = hc
        .call_zome_default::<(), Option<ProfileEvidenceView>>(
            "identity_vault",
            "get_my_profile_evidence_view",
            &(),
        )
        .await;
    if !ctx.accepts_epoch(epoch) {
        return false;
    }

    let keys_result = hc
        .call_zome_default::<(), Vec<MasterKeyView>>(
            "identity_vault",
            "get_my_keys_view",
            &(),
        )
        .await;
    if !ctx.accepts_epoch(epoch) {
        return false;
    }

    let staged = stage_identity_source(profile_result, keys_result);
    if !ctx.accepts_epoch(epoch) {
        return false;
    }

    match staged {
        Ok(snapshot) => publish_identity_source(ctx, snapshot),
        Err(state) => ctx.identity_state.set(state),
    }
    true
}

async fn load_wallet_source(ctx: &PersonalCtx, hc: &HolochainCtx, epoch: u64) -> bool {
    let result = hc
        .call_zome_default::<(), Vec<StoredCredentialView>>(
            "credential_wallet",
            "get_my_credentials_view",
            &(),
        )
        .await;
    if !ctx.accepts_epoch(epoch) {
        return false;
    }

    match result {
        Ok(credentials) => {
            let count = credentials.len();
            ctx.credentials.set(credentials);
            ctx.wallet_state.set(classify_source(1, 0, count));
        }
        Err(_) => ctx.wallet_state.set(PersonalSourceState::Unavailable),
    }
    true
}

async fn load_health_source(ctx: &PersonalCtx, hc: &HolochainCtx, epoch: u64) -> bool {
    let biometrics_result = hc
        .call_zome_default::<(), Vec<BiometricView>>(
            "health_vault",
            "get_my_biometrics_view",
            &(),
        )
        .await;
    if !ctx.accepts_epoch(epoch) {
        return false;
    }

    let consents_result = hc
        .call_zome_default::<(), Vec<ConsentGrantView>>(
            "health_vault",
            "get_my_consents_view",
            &(),
        )
        .await;
    if !ctx.accepts_epoch(epoch) {
        return false;
    }

    let records_result = hc
        .call_zome_default::<(), Vec<HealthRecordView>>(
            "health_vault",
            "get_my_records_view",
            &(),
        )
        .await;
    if !ctx.accepts_epoch(epoch) {
        return false;
    }

    let staged = stage_health_source(biometrics_result, consents_result, records_result);
    if !ctx.accepts_epoch(epoch) {
        return false;
    }

    match staged {
        Ok(snapshot) => publish_health_source(ctx, snapshot),
        Err(state) => ctx.health_state.set(state),
    }
    true
}

async fn load_preferences_source(ctx: &PersonalCtx, hc: &HolochainCtx, epoch: u64) -> bool {
    let preferences_result = hc
        .call_zome_default::<(), Vec<DataSharingPreferenceEvidenceView>>(
            "data_preferences",
            "get_my_preferences_evidence_view",
            &(),
        )
        .await;
    if !ctx.accepts_epoch(epoch) {
        return false;
    }

    let log_result = hc
        .call_zome_default::<(), Vec<PreferenceChangeLogView>>(
            "data_preferences",
            "get_change_log_view",
            &(),
        )
        .await;
    if !ctx.accepts_epoch(epoch) {
        return false;
    }

    let staged = stage_preferences_source(preferences_result, log_result);
    if !ctx.accepts_epoch(epoch) {
        return false;
    }

    match staged {
        Ok(snapshot) => publish_preferences_source(ctx, snapshot),
        Err(state) => ctx.preferences_state.set(state),
    }
    true
}

async fn load_activity_source(ctx: &PersonalCtx, hc: &HolochainCtx, epoch: u64) -> bool {
    let result = hc
        .call_zome_default::<(), Vec<ActivityItemView>>(
            "personal_bridge",
            "get_recent_activity_view",
            &(),
        )
        .await;
    if !ctx.accepts_epoch(epoch) {
        return false;
    }

    match result {
        Ok(activity) => {
            let count = activity.len();
            ctx.activity.set(activity);
            ctx.activity_state.set(classify_source(1, 0, count));
        }
        Err(_) => ctx.activity_state.set(PersonalSourceState::Unavailable),
    }
    true
}

pub fn use_personal() -> PersonalCtx {
    expect_context::<PersonalCtx>()
}

pub async fn refresh_identity_state(ctx: PersonalCtx, hc: HolochainCtx) {
    let Some(epoch) = ctx.current_usable_epoch() else {
        return;
    };
    let _ = load_identity_source(&ctx, &hc, epoch).await;
}

pub async fn refresh_preferences_state(ctx: PersonalCtx, hc: HolochainCtx) {
    let Some(epoch) = ctx.current_usable_epoch() else {
        return;
    };
    let _ = load_preferences_source(&ctx, &hc, epoch).await;
}

pub async fn refresh_health_state(ctx: PersonalCtx, hc: HolochainCtx) {
    let Some(epoch) = ctx.current_usable_epoch() else {
        return;
    };
    let _ = load_health_source(&ctx, &hc, epoch).await;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn complete_empty_source_is_empty_not_unavailable() {
        assert_eq!(classify_source(2, 0, 0), PersonalSourceState::Empty);
    }

    #[test]
    fn partial_failure_is_degraded_even_with_data() {
        assert_eq!(classify_source(1, 1, 4), PersonalSourceState::Degraded);
    }

    #[test]
    fn total_failure_is_unavailable() {
        assert_eq!(
            classify_source(0, 2, 0),
            PersonalSourceState::Unavailable
        );
    }

    #[test]
    fn successful_nonempty_source_is_live() {
        assert_eq!(classify_source(2, 0, 3), PersonalSourceState::Live);
    }

    #[test]
    fn source_disposition_and_snapshot_freshness_are_independent() {
        let source = PersonalSourceState::Live;
        let freshness = PersonalSnapshotFreshness::Stale;
        assert_eq!(source, PersonalSourceState::Live);
        assert_eq!(freshness, PersonalSnapshotFreshness::Stale);
    }

    #[test]
    fn identity_stage_requires_every_query_before_publication() {
        let evidence = ProfileEvidenceView {
            action_hash: "uhCkk-profile".into(),
            profile: blank_profile(),
        };
        let state = stage_identity_source::<(), ()>(Ok(Some(evidence)), Err(()))
            .expect_err("a partial Identity source must not produce a publishable snapshot");
        assert_eq!(state, PersonalSourceState::Degraded);
    }

    #[test]
    fn identity_stage_commits_empty_results_as_one_empty_snapshot() {
        let snapshot = stage_identity_source::<(), ()>(Ok(None), Ok(Vec::new()))
            .expect("complete empty Identity results are authoritative");
        assert_eq!(snapshot.state, PersonalSourceState::Empty);
        assert!(snapshot.profile.display_name.is_empty());
        assert!(snapshot.profile_action_hash.is_none());
        assert!(snapshot.keys.is_empty());
    }

    #[test]
    fn identity_stage_preserves_exact_profile_action_identity() {
        let evidence = ProfileEvidenceView {
            action_hash: "uhCkk-profile".into(),
            profile: blank_profile(),
        };
        let snapshot = stage_identity_source::<(), ()>(Ok(Some(evidence)), Ok(Vec::new()))
            .expect("complete Identity evidence should stage");
        assert_eq!(snapshot.profile_action_hash.as_deref(), Some("uhCkk-profile"));
    }

    #[test]
    fn preferences_stage_requires_every_query_before_publication() {
        let state = stage_preferences_source::<(), ()>(Ok(Vec::new()), Err(()))
            .expect_err("a partial Preferences source must not produce a publishable snapshot");
        assert_eq!(state, PersonalSourceState::Degraded);
    }

    #[test]
    fn preferences_stage_commits_complete_empty_results_atomically() {
        let snapshot = stage_preferences_source::<(), ()>(Ok(Vec::new()), Ok(Vec::new()))
            .expect("complete empty Preferences results are authoritative");
        assert_eq!(snapshot.state, PersonalSourceState::Empty);
        assert!(snapshot.preferences.is_empty());
        assert!(snapshot.action_hashes.is_empty());
        assert!(snapshot.change_log.is_empty());
    }

    #[test]
    fn preferences_stage_preserves_every_observed_action_identity() {
        let evidence = DataSharingPreferenceEvidenceView {
            action_hash: "uhCkk-preference".into(),
            preference: DataSharingPreferenceView {
                source_cluster: "personal".into(),
                target_cluster: "health".into(),
                allowed: true,
                blocked_zomes: Vec::new(),
                reason: "test".into(),
                updated_at: 1,
            },
        };
        let snapshot = stage_preferences_source::<(), ()>(Ok(vec![evidence]), Ok(Vec::new()))
            .expect("complete Preferences evidence should stage");
        assert!(snapshot.action_hashes.contains("uhCkk-preference"));
        assert_eq!(snapshot.preferences.len(), 1);
    }

    #[test]
    fn health_stage_requires_all_three_queries_before_publication() {
        let state = stage_health_source::<(), (), ()>(Ok(Vec::new()), Ok(Vec::new()), Err(()))
            .expect_err("a partial Health source must not produce a publishable snapshot");
        assert_eq!(state, PersonalSourceState::Degraded);
    }

    #[test]
    fn health_stage_commits_complete_empty_results_atomically() {
        let snapshot = stage_health_source::<(), (), ()>(Ok(Vec::new()), Ok(Vec::new()), Ok(Vec::new()))
            .expect("complete empty Health results are authoritative");
        assert_eq!(snapshot.state, PersonalSourceState::Empty);
        assert!(snapshot.biometrics.is_empty());
        assert!(snapshot.consents.is_empty());
        assert_eq!(snapshot.record_count, 0);
    }
}
