// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Personal frontend context.
//!
//! Runtime provenance is explicit: Demo owns illustrative fixture data; Live
//! starts empty and only publishes values returned by typed conductor view
//! endpoints. An authoritative empty result therefore remains empty rather
//! than preserving demo records.

use leptos::prelude::*;
use serde::{Deserialize, Serialize};
use wasm_bindgen_futures::spawn_local;

use mycelix_leptos_core::holochain_provider::{use_holochain, HolochainCtx};
use personal_leptos_types::{
    ActivityItemView, BiometricView, ConsentGrantView, DataSharingPreferenceView, HealthRecordView,
    MasterKeyView, PreferenceChangeLogView, ProfileView, StoredCredentialView,
};

use crate::mock_data;
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

#[derive(Clone)]
pub struct PersonalCtx {
    pub runtime_mode: PersonalRuntimeMode,
    pub profile: RwSignal<ProfileView>,
    pub draft_profile: RwSignal<ProfileView>,
    pub keys: RwSignal<Vec<MasterKeyView>>,
    pub credentials: RwSignal<Vec<StoredCredentialView>>,
    pub biometrics: RwSignal<Vec<BiometricView>>,
    pub consents: RwSignal<Vec<ConsentGrantView>>,
    pub preferences: RwSignal<Vec<DataSharingPreferenceView>>,
    pub preference_log: RwSignal<Vec<PreferenceChangeLogView>>,
    pub health_record_count: RwSignal<usize>,
    pub activity: RwSignal<Vec<ActivityItemView>>,
    pub identity_state: RwSignal<PersonalSourceState>,
    pub wallet_state: RwSignal<PersonalSourceState>,
    pub health_state: RwSignal<PersonalSourceState>,
    pub preferences_state: RwSignal<PersonalSourceState>,
    pub activity_state: RwSignal<PersonalSourceState>,
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
    let hydration_started = RwSignal::new(false);
    let ctx_for_effect = ctx.clone();

    Effect::new(move |_| {
        if !hc.zome_calls_ready() || hydration_started.get() {
            return;
        }

        hydration_started.set(true);
        ctx_for_effect.loading.set(true);
        set_all_source_states(&ctx_for_effect, PersonalSourceState::LoadingLive);

        let ctx = ctx_for_effect.clone();
        let hc = hc.clone();
        spawn_local(async move {
            hydrate_live(ctx, hc).await;
        });
    });
}

fn set_all_source_states(ctx: &PersonalCtx, state: PersonalSourceState) {
    ctx.identity_state.set(state);
    ctx.wallet_state.set(state);
    ctx.health_state.set(state);
    ctx.preferences_state.set(state);
    ctx.activity_state.set(state);
}

async fn hydrate_live(ctx: PersonalCtx, hc: HolochainCtx) {
    load_identity_source(&ctx, &hc).await;
    load_wallet_source(&ctx, &hc).await;
    load_health_source(&ctx, &hc).await;
    load_preferences_source(&ctx, &hc).await;
    load_activity_source(&ctx, &hc).await;

    ctx.loading.set(false);
    let states = ctx.source_states_untracked();

    if states.iter().any(|state| {
        matches!(
            state,
            PersonalSourceState::Degraded | PersonalSourceState::Unavailable
        )
    }) {
        ctx.status_note.set(
            "Live conductor reached, but one or more Personal sources could not be established. Unavailable sources remain explicit; demo records are not substituted."
                .into(),
        );
    } else if states
        .iter()
        .all(|state| *state == PersonalSourceState::Empty)
    {
        ctx.status_note.set(
            "Live conductor returned no Personal records. This empty state is authoritative for the completed view queries."
                .into(),
        );
    } else {
        ctx.status_note.set(
            "Live Personal state loaded from typed conductor view endpoints. Empty source results remain empty and are not replaced with fixtures."
                .into(),
        );
    }
}

async fn load_identity_source(ctx: &PersonalCtx, hc: &HolochainCtx) {
    let mut successes = 0;
    let mut failures = 0;
    let mut present_items = 0;

    match hc
        .call_zome_default::<(), Option<ProfileView>>(
            "identity_vault",
            "get_my_profile_view",
            &(),
        )
        .await
    {
        Ok(profile) => {
            successes += 1;
            if profile.is_some() {
                present_items += 1;
            }
            let profile = profile.unwrap_or_else(blank_profile);
            ctx.profile.set(profile.clone());
            ctx.draft_profile.set(profile);
        }
        Err(_) => failures += 1,
    }

    match hc
        .call_zome_default::<(), Vec<MasterKeyView>>(
            "identity_vault",
            "get_my_keys_view",
            &(),
        )
        .await
    {
        Ok(keys) => {
            successes += 1;
            present_items += keys.len();
            ctx.keys.set(keys);
        }
        Err(_) => failures += 1,
    }

    ctx.identity_state
        .set(classify_source(successes, failures, present_items));
}

async fn load_wallet_source(ctx: &PersonalCtx, hc: &HolochainCtx) {
    match hc
        .call_zome_default::<(), Vec<StoredCredentialView>>(
            "credential_wallet",
            "get_my_credentials_view",
            &(),
        )
        .await
    {
        Ok(credentials) => {
            let count = credentials.len();
            ctx.credentials.set(credentials);
            ctx.wallet_state.set(classify_source(1, 0, count));
        }
        Err(_) => ctx.wallet_state.set(PersonalSourceState::Unavailable),
    }
}

async fn load_health_source(ctx: &PersonalCtx, hc: &HolochainCtx) {
    let mut successes = 0;
    let mut failures = 0;
    let mut present_items = 0;

    match hc
        .call_zome_default::<(), Vec<BiometricView>>(
            "health_vault",
            "get_my_biometrics_view",
            &(),
        )
        .await
    {
        Ok(biometrics) => {
            successes += 1;
            present_items += biometrics.len();
            ctx.biometrics.set(biometrics);
        }
        Err(_) => failures += 1,
    }

    match hc
        .call_zome_default::<(), Vec<ConsentGrantView>>(
            "health_vault",
            "get_my_consents_view",
            &(),
        )
        .await
    {
        Ok(consents) => {
            successes += 1;
            present_items += consents.len();
            ctx.consents.set(consents);
        }
        Err(_) => failures += 1,
    }

    match hc
        .call_zome_default::<(), Vec<HealthRecordView>>(
            "health_vault",
            "get_my_records_view",
            &(),
        )
        .await
    {
        Ok(records) => {
            successes += 1;
            present_items += records.len();
            ctx.health_record_count.set(records.len());
        }
        Err(_) => failures += 1,
    }

    ctx.health_state
        .set(classify_source(successes, failures, present_items));
}

async fn load_preferences_source(ctx: &PersonalCtx, hc: &HolochainCtx) {
    let mut successes = 0;
    let mut failures = 0;
    let mut present_items = 0;

    match hc
        .call_zome_default::<(), Vec<DataSharingPreferenceView>>(
            "data_preferences",
            "get_my_preferences_view",
            &(),
        )
        .await
    {
        Ok(preferences) => {
            successes += 1;
            present_items += preferences.len();
            ctx.preferences.set(preferences);
        }
        Err(_) => failures += 1,
    }

    match hc
        .call_zome_default::<(), Vec<PreferenceChangeLogView>>(
            "data_preferences",
            "get_change_log_view",
            &(),
        )
        .await
    {
        Ok(log) => {
            successes += 1;
            present_items += log.len();
            ctx.preference_log.set(log);
        }
        Err(_) => failures += 1,
    }

    ctx.preferences_state
        .set(classify_source(successes, failures, present_items));
}

async fn load_activity_source(ctx: &PersonalCtx, hc: &HolochainCtx) {
    match hc
        .call_zome_default::<(), Vec<ActivityItemView>>(
            "personal_bridge",
            "get_recent_activity_view",
            &(),
        )
        .await
    {
        Ok(activity) => {
            let count = activity.len();
            ctx.activity.set(activity);
            ctx.activity_state.set(classify_source(1, 0, count));
        }
        Err(_) => ctx.activity_state.set(PersonalSourceState::Unavailable),
    }
}

pub fn use_personal() -> PersonalCtx {
    expect_context::<PersonalCtx>()
}

pub async fn refresh_identity_state(ctx: PersonalCtx, hc: HolochainCtx) {
    ctx.identity_state.set(PersonalSourceState::LoadingLive);
    load_identity_source(&ctx, &hc).await;
}

pub async fn refresh_preferences_state(ctx: PersonalCtx, hc: HolochainCtx) {
    ctx.preferences_state
        .set(PersonalSourceState::LoadingLive);
    load_preferences_source(&ctx, &hc).await;
}

pub async fn refresh_health_state(ctx: PersonalCtx, hc: HolochainCtx) {
    ctx.health_state.set(PersonalSourceState::LoadingLive);
    load_health_source(&ctx, &hc).await;
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
}
