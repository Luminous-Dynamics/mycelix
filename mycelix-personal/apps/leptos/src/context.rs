// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Personal context.
//!
//! This scaffold keeps the data boundary explicit: the current Personal zomes
//! largely return raw Holochain `Record` values rather than dedicated
//! frontend-facing views. The app therefore renders stable typed mock data now
//! and centralizes future live adapters here.

use leptos::prelude::*;
use wasm_bindgen_futures::spawn_local;

use mycelix_leptos_core::holochain_provider::{HolochainCtx, use_holochain};
use personal_leptos_types::{
    ActivityItemView, BiometricView, ConsentGrantView, DataSharingPreferenceView, HealthRecordView,
    MasterKeyView, PreferenceChangeLogView, ProfileView, SourceReadState,
    SourceUnavailableReason, StoredCredentialView,
};

use crate::mock_data;
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SymbolRegistry {
    pub hearth_alias: String,  // e.g. "Campfire", "Well", "Plaza"
    pub mycel_alias: String,   // e.g. "Spark", "Ember", "Leaf"
    pub genesis_alias: String, // e.g. "The Ignition", "Sun-Rise"
    pub orientation: String,   // Cultural context description
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

#[derive(Clone)]
pub struct PersonalCtx {
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
    pub profile_source_state: RwSignal<SourceReadState>,
    pub keys_source_state: RwSignal<SourceReadState>,
    pub credentials_source_state: RwSignal<SourceReadState>,
    pub biometrics_source_state: RwSignal<SourceReadState>,
    pub consents_source_state: RwSignal<SourceReadState>,
    pub preferences_source_state: RwSignal<SourceReadState>,
    pub preference_log_source_state: RwSignal<SourceReadState>,
    pub health_records_source_state: RwSignal<SourceReadState>,
    pub activity_source_state: RwSignal<SourceReadState>,
    pub loading: RwSignal<bool>,
    pub live_sync_ready: RwSignal<bool>,
    pub status_note: RwSignal<String>,
}

fn empty_profile() -> ProfileView {
    ProfileView {
        display_name: String::new(),
        avatar: None,
        bio: None,
        metadata: std::collections::HashMap::new(),
        updated_at: 0,
    }
}

fn live_source_status(successful_sources: &[&str], unavailable_sources: &[&str]) -> String {
    if unavailable_sources.is_empty() {
        format!(
            "Live mode: all {} Personal source views returned successfully. An empty list means that specific source returned no records at this read time.",
            successful_sources.len()
        )
    } else if successful_sources.is_empty() {
        format!(
            "Live mode: no Personal source views could be read ({}). Records are unavailable, not confirmed absent.",
            unavailable_sources.join(", ")
        )
    } else {
        format!(
            "Live mode: partial source read. Unavailable: {}. Empty lists are meaningful only for the sources that returned successfully; unavailable sources are not confirmed empty.",
            unavailable_sources.join(", ")
        )
    }
}

fn source_check_time_micros() -> i64 {
    #[cfg(target_arch = "wasm32")]
    {
        (js_sys::Date::now() * 1_000.0) as i64
    }
    #[cfg(not(target_arch = "wasm32"))]
    {
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap_or_default()
            .as_micros()
            .min(i64::MAX as u128) as i64
    }
}

fn live_collection_state(is_empty: bool) -> SourceReadState {
    let checked_at = source_check_time_micros();
    if is_empty {
        SourceReadState::LiveConfirmedEmpty { checked_at }
    } else {
        SourceReadState::LiveWithRecords { checked_at }
    }
}

fn unavailable_source_state() -> SourceReadState {
    SourceReadState::Unavailable {
        reason: SourceUnavailableReason::Unknown,
    }
}

/// Preserve stale-data epistemics when a refresh fails after a prior success.
fn failed_refresh_state(previous: SourceReadState) -> SourceReadState {
    let last_successful_read_at = match previous {
        SourceReadState::LiveWithRecords { checked_at }
        | SourceReadState::LiveConfirmedEmpty { checked_at } => Some(checked_at),
        SourceReadState::Stale {
            last_successful_read_at,
        } => last_successful_read_at,
        _ => None,
    };
    match last_successful_read_at {
        Some(last_successful_read_at) => SourceReadState::Stale {
            last_successful_read_at: Some(last_successful_read_at),
        },
        None => unavailable_source_state(),
    }
}

pub fn provide_personal_context() {
    let profile = empty_profile();
    let ctx = PersonalCtx {
        draft_profile: RwSignal::new(profile.clone()),
        profile: RwSignal::new(profile),
        // Start empty in every mode. Synthetic data is populated only after
        // the provider explicitly reports Mock mode.
        keys: RwSignal::new(Vec::new()),
        credentials: RwSignal::new(Vec::new()),
        biometrics: RwSignal::new(Vec::new()),
        consents: RwSignal::new(Vec::new()),
        preferences: RwSignal::new(Vec::new()),
        preference_log: RwSignal::new(Vec::new()),
        health_record_count: RwSignal::new(0),
        activity: RwSignal::new(Vec::new()),
        profile_source_state: RwSignal::new(SourceReadState::NotChecked),
        keys_source_state: RwSignal::new(SourceReadState::NotChecked),
        credentials_source_state: RwSignal::new(SourceReadState::NotChecked),
        biometrics_source_state: RwSignal::new(SourceReadState::NotChecked),
        consents_source_state: RwSignal::new(SourceReadState::NotChecked),
        preferences_source_state: RwSignal::new(SourceReadState::NotChecked),
        preference_log_source_state: RwSignal::new(SourceReadState::NotChecked),
        health_records_source_state: RwSignal::new(SourceReadState::NotChecked),
        activity_source_state: RwSignal::new(SourceReadState::NotChecked),
        loading: RwSignal::new(true),
        live_sync_ready: RwSignal::new(false),
        status_note: RwSignal::new("Personal source state has not been checked yet.".into()),
    };

    provide_context(ctx.clone());

    spawn_local(async move {
        gloo_timers::future::sleep(std::time::Duration::from_millis(300)).await;
        let hc = use_holochain();

        if hc.is_mock() {
            // Synthetic fixtures exist only in explicit Mock mode. They are
            // never used as fallback values for failed/empty Live reads.
            let profile = mock_data::mock_profile();
            ctx.profile.set(profile.clone());
            ctx.draft_profile.set(profile);
            ctx.keys.set(mock_data::mock_keys());
            ctx.credentials.set(mock_data::mock_credentials());
            ctx.biometrics.set(mock_data::mock_biometrics());
            ctx.consents.set(mock_data::mock_consents());
            ctx.preferences.set(mock_data::mock_preferences());
            ctx.preference_log.set(mock_data::mock_preference_log());
            ctx.health_record_count
                .set(mock_data::mock_health_records());
            ctx.activity.set(mock_data::mock_activity());
            ctx.profile_source_state.set(SourceReadState::Demo);
            ctx.keys_source_state.set(SourceReadState::Demo);
            ctx.credentials_source_state.set(SourceReadState::Demo);
            ctx.biometrics_source_state.set(SourceReadState::Demo);
            ctx.consents_source_state.set(SourceReadState::Demo);
            ctx.preferences_source_state.set(SourceReadState::Demo);
            ctx.preference_log_source_state.set(SourceReadState::Demo);
            ctx.health_records_source_state.set(SourceReadState::Demo);
            ctx.activity_source_state.set(SourceReadState::Demo);
            ctx.status_note.set(
                "Mock mode: all displayed Personal records are synthetic examples, not live vault records."
                    .into(),
            );
        } else {
            // Count source responses, not record counts. An Ok(empty) result is
            // a successful source read and must clear any previously shown rows.
            let mut successful_sources: Vec<&'static str> = Vec::new();
            let mut unavailable_sources: Vec<&'static str> = Vec::new();

            match hc
                .call_zome_default::<(), Option<ProfileView>>(
                    "identity_vault",
                    "get_my_profile_view",
                    &(),
                )
                .await
            {
                Ok(profile) => {
                    ctx.profile_source_state
                        .set(live_collection_state(profile.is_none()));
                    let profile = profile.unwrap_or_else(empty_profile);
                    ctx.profile.set(profile.clone());
                    ctx.draft_profile.set(profile);
                    successful_sources.push("profile");
                }
                Err(_) => {
                    ctx.profile_source_state.set(unavailable_source_state());
                    unavailable_sources.push("profile");
                }
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
                    ctx.keys_source_state
                        .set(live_collection_state(keys.is_empty()));
                    ctx.keys.set(keys);
                    successful_sources.push("identity keys");
                }
                Err(_) => {
                    ctx.keys_source_state.set(unavailable_source_state());
                    unavailable_sources.push("identity keys");
                }
            }

            match hc
                .call_zome_default::<(), Vec<StoredCredentialView>>(
                    "credential_wallet",
                    "get_my_credentials_view",
                    &(),
                )
                .await
            {
                Ok(credentials) => {
                    ctx.credentials_source_state
                        .set(live_collection_state(credentials.is_empty()));
                    ctx.credentials.set(credentials);
                    successful_sources.push("credentials");
                }
                Err(_) => {
                    ctx.credentials_source_state.set(unavailable_source_state());
                    unavailable_sources.push("credentials");
                }
            }

            match hc
                .call_zome_default::<(), Vec<BiometricView>>(
                    "health_vault",
                    "get_my_biometrics_view",
                    &(),
                )
                .await
            {
                Ok(biometrics) => {
                    ctx.biometrics_source_state
                        .set(live_collection_state(biometrics.is_empty()));
                    ctx.biometrics.set(biometrics);
                    successful_sources.push("biometrics");
                }
                Err(_) => {
                    ctx.biometrics_source_state.set(unavailable_source_state());
                    unavailable_sources.push("biometrics");
                }
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
                    ctx.consents_source_state
                        .set(live_collection_state(consents.is_empty()));
                    ctx.consents.set(consents);
                    successful_sources.push("consents");
                }
                Err(_) => {
                    ctx.consents_source_state.set(unavailable_source_state());
                    unavailable_sources.push("consents");
                }
            }

            match hc
                .call_zome_default::<(), Vec<DataSharingPreferenceView>>(
                    "data_preferences",
                    "get_my_preferences_view",
                    &(),
                )
                .await
            {
                Ok(preferences) => {
                    ctx.preferences_source_state
                        .set(live_collection_state(preferences.is_empty()));
                    ctx.preferences.set(preferences);
                    successful_sources.push("sharing preferences");
                }
                Err(_) => {
                    ctx.preferences_source_state.set(unavailable_source_state());
                    unavailable_sources.push("sharing preferences");
                }
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
                    ctx.preference_log_source_state
                        .set(live_collection_state(log.is_empty()));
                    ctx.preference_log.set(log);
                    successful_sources.push("preference history");
                }
                Err(_) => {
                    ctx.preference_log_source_state.set(unavailable_source_state());
                    unavailable_sources.push("preference history");
                }
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
                    ctx.health_records_source_state
                        .set(live_collection_state(records.is_empty()));
                    ctx.health_record_count.set(records.len());
                    successful_sources.push("health records");
                }
                Err(_) => {
                    ctx.health_records_source_state.set(unavailable_source_state());
                    unavailable_sources.push("health records");
                }
            }

            match hc
                .call_zome_default::<(), Vec<ActivityItemView>>(
                    "personal_bridge",
                    "get_recent_activity_view",
                    &(),
                )
                .await
            {
                Ok(activity) => {
                    ctx.activity_source_state
                        .set(live_collection_state(activity.is_empty()));
                    ctx.activity.set(activity);
                    successful_sources.push("activity history");
                }
                Err(_) => {
                    ctx.activity_source_state.set(unavailable_source_state());
                    unavailable_sources.push("activity history");
                }
            }

            ctx.live_sync_ready.set(!successful_sources.is_empty());
            ctx.status_note.set(live_source_status(
                &successful_sources,
                &unavailable_sources,
            ));
        }

        ctx.loading.set(false);
    });
}

pub fn use_personal() -> PersonalCtx {
    expect_context::<PersonalCtx>()
}

pub async fn refresh_identity_state(ctx: PersonalCtx, hc: HolochainCtx) {
    match hc
        .call_zome_default::<(), Option<ProfileView>>("identity_vault", "get_my_profile_view", &())
        .await
    {
        Ok(profile) => {
            ctx.profile_source_state
                .set(live_collection_state(profile.is_none()));
            let profile = profile.unwrap_or_else(empty_profile);
            ctx.profile.set(profile.clone());
            ctx.draft_profile.set(profile);
        }
        Err(_) => {
            let previous = ctx.profile_source_state.get_untracked();
            ctx.profile_source_state.set(failed_refresh_state(previous));
            ctx.status_note.set(
                "Live mode: the profile source could not be refreshed; source status now indicates whether prior data is stale or unavailable."
                    .into(),
            );
        }
    }

    match hc
        .call_zome_default::<(), Vec<MasterKeyView>>("identity_vault", "get_my_keys_view", &())
        .await
    {
        Ok(keys) => {
            ctx.keys_source_state
                .set(live_collection_state(keys.is_empty()));
            ctx.keys.set(keys);
        }
        Err(_) => {
            let previous = ctx.keys_source_state.get_untracked();
            ctx.keys_source_state.set(failed_refresh_state(previous));
            ctx.status_note.set(
                "Live mode: identity keys could not be refreshed; source status now indicates whether prior data is stale or unavailable."
                    .into(),
            );
        }
    }
}

pub async fn refresh_preferences_state(ctx: PersonalCtx, hc: HolochainCtx) {
    match hc
        .call_zome_default::<(), Vec<DataSharingPreferenceView>>(
            "data_preferences",
            "get_my_preferences_view",
            &(),
        )
        .await
    {
        Ok(preferences) => {
            ctx.preferences_source_state
                .set(live_collection_state(preferences.is_empty()));
            ctx.preferences.set(preferences);
        }
        Err(_) => {
            let previous = ctx.preferences_source_state.get_untracked();
            ctx.preferences_source_state.set(failed_refresh_state(previous));
            ctx.status_note.set(
                "Live mode: sharing preferences could not be refreshed; source status now indicates whether prior data is stale or unavailable."
                    .into(),
            );
        }
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
            ctx.preference_log_source_state
                .set(live_collection_state(log.is_empty()));
            ctx.preference_log.set(log);
        }
        Err(_) => {
            let previous = ctx.preference_log_source_state.get_untracked();
            ctx.preference_log_source_state
                .set(failed_refresh_state(previous));
            ctx.status_note.set(
                "Live mode: preference history could not be refreshed; source status now indicates whether prior data is stale or unavailable."
                    .into(),
            );
        }
    }
}

pub async fn refresh_health_state(ctx: PersonalCtx, hc: HolochainCtx) {
    match hc
        .call_zome_default::<(), Vec<BiometricView>>("health_vault", "get_my_biometrics_view", &())
        .await
    {
        Ok(biometrics) => {
            ctx.biometrics_source_state
                .set(live_collection_state(biometrics.is_empty()));
            ctx.biometrics.set(biometrics);
        }
        Err(_) => {
            let previous = ctx.biometrics_source_state.get_untracked();
            ctx.biometrics_source_state.set(failed_refresh_state(previous));
            ctx.status_note.set(
                "Live mode: biometric records could not be refreshed; source status now indicates whether prior data is stale or unavailable."
                    .into(),
            );
        }
    }

    match hc
        .call_zome_default::<(), Vec<ConsentGrantView>>("health_vault", "get_my_consents_view", &())
        .await
    {
        Ok(consents) => {
            ctx.consents_source_state
                .set(live_collection_state(consents.is_empty()));
            ctx.consents.set(consents);
        }
        Err(_) => {
            let previous = ctx.consents_source_state.get_untracked();
            ctx.consents_source_state.set(failed_refresh_state(previous));
            ctx.status_note.set(
                "Live mode: consent grants could not be refreshed; source status now indicates whether prior data is stale or unavailable."
                    .into(),
            );
        }
    }

    match hc
        .call_zome_default::<(), Vec<HealthRecordView>>("health_vault", "get_my_records_view", &())
        .await
    {
        Ok(records) => {
            ctx.health_records_source_state
                .set(live_collection_state(records.is_empty()));
            ctx.health_record_count.set(records.len());
        }
        Err(_) => {
            let previous = ctx.health_records_source_state.get_untracked();
            ctx.health_records_source_state.set(failed_refresh_state(previous));
            ctx.status_note.set(
                "Live mode: health records could not be refreshed; source status now indicates whether prior data is stale or unavailable."
                    .into(),
            );
        }
    }
}

#[cfg(test)]
mod source_status_tests {
    use super::{
        empty_profile, failed_refresh_state, live_collection_state, live_source_status,
        SourceReadState, SourceUnavailableReason,
    };

    #[test]
    fn live_empty_results_are_not_reported_as_missing_sources() {
        let note = live_source_status(&["credentials", "activity history"], &[]);
        assert!(note.contains("returned successfully"));
        assert!(note.contains("empty list means that specific source returned no records"));
    }

    #[test]
    fn failed_source_reads_are_distinguished_from_confirmed_empty() {
        let note = live_source_status(&["profile"], &["credentials", "health records"]);
        assert!(note.contains("partial source read"));
        assert!(note.contains("credentials, health records"));
        assert!(note.contains("not confirmed empty"));
    }

    #[test]
    fn all_failed_live_reads_never_claim_the_vault_is_empty() {
        let note = live_source_status(&[], &["profile", "credentials"]);
        assert!(note.contains("Records are unavailable, not confirmed absent"));
    }

    #[test]
    fn empty_profile_contains_no_mock_identity_fields() {
        let profile = empty_profile();
        assert!(profile.display_name.is_empty());
        assert!(profile.avatar.is_none());
        assert!(profile.bio.is_none());
        assert!(profile.metadata.is_empty());
        assert_eq!(profile.updated_at, 0);
    }

    #[test]
    fn live_empty_and_live_with_records_are_distinct() {
        let empty = live_collection_state(true);
        let records = live_collection_state(false);
        assert!(matches!(empty, SourceReadState::LiveConfirmedEmpty { .. }));
        assert!(matches!(records, SourceReadState::LiveWithRecords { .. }));
    }

    #[test]
    fn failed_refresh_marks_previous_live_state_stale() {
        let previous = SourceReadState::LiveWithRecords { checked_at: 123 };
        let next = failed_refresh_state(previous);
        assert_eq!(
            next,
            SourceReadState::Stale {
                last_successful_read_at: Some(123)
            }
        );
    }

    #[test]
    fn failed_refresh_without_prior_success_is_unavailable() {
        assert_eq!(
            failed_refresh_state(SourceReadState::NotChecked),
            SourceReadState::Unavailable {
                reason: SourceUnavailableReason::Unknown
            }
        );
    }
}
