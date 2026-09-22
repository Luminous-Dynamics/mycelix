#![forbid(unsafe_code)]
//! Pure Steam capability assessment for Mycelix.
//!
//! This crate evaluates bounded, supplied observations under one exact
//! Mycelix-owned semantic profile. It does not call Steamworks, authenticate a
//! live account, establish entitlement truth, or expose provider effects.
//!
//! ```compile_fail
//! use mycelix_steam_capability::SteamCapabilityAssessmentV1;
//!
//! fn requires_deserialize<T: for<'de> serde::Deserialize<'de>>() {}
//! requires_deserialize::<SteamCapabilityAssessmentV1>();
//! ```

use serde::{Deserialize, Serialize};
use std::fmt;

pub const STEAM_CAPABILITY_PROFILE_V0_1: &str = "mycelix:game-steam:capability-profile:v0.1";
pub const STEAM_CAPABILITY_CORPUS_V0_1: &str = "mycelix:game-steam:capability-profile:v0.1";

const MAX_PROFILE_ID_BYTES: usize = 256;
const MAX_APP_ID_BYTES: usize = 128;
const MAX_RUNTIME_PROFILE_REF_BYTES: usize = 512;
const MAX_PROVIDER_ID_BYTES: usize = 1024;
const MAX_LOCATOR_BYTES: usize = 4096;
const MAX_CAPABILITY_NAME_BYTES: usize = 128;
const MAX_OBSERVATIONS: usize = 256;

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum SteamCapabilityKindV1 {
    RuntimeInitialized,
    AuthenticatedSteamId,
    AppOwnershipObservation,
    SteamInputAvailable,
    StatsAchievementsAvailable,
    CloudAvailable,
    WorkshopUGCAvailable,
    LobbyMatchmakingAvailable,
    SteamNetworkingAvailable,
    FriendsPresenceAvailable,
    InventoryAvailable,
}

impl SteamCapabilityKindV1 {
    pub const ALL: [Self; 11] = [
        Self::RuntimeInitialized,
        Self::AuthenticatedSteamId,
        Self::AppOwnershipObservation,
        Self::SteamInputAvailable,
        Self::StatsAchievementsAvailable,
        Self::CloudAvailable,
        Self::WorkshopUGCAvailable,
        Self::LobbyMatchmakingAvailable,
        Self::SteamNetworkingAvailable,
        Self::FriendsPresenceAvailable,
        Self::InventoryAvailable,
    ];

    fn from_profile_name(name: &str) -> Option<Self> {
        match name {
            "RuntimeInitialized" => Some(Self::RuntimeInitialized),
            "AuthenticatedSteamId" => Some(Self::AuthenticatedSteamId),
            "AppOwnershipObservation" => Some(Self::AppOwnershipObservation),
            "SteamInputAvailable" => Some(Self::SteamInputAvailable),
            "StatsAchievementsAvailable" => Some(Self::StatsAchievementsAvailable),
            "CloudAvailable" => Some(Self::CloudAvailable),
            "WorkshopUGCAvailable" => Some(Self::WorkshopUGCAvailable),
            "LobbyMatchmakingAvailable" => Some(Self::LobbyMatchmakingAvailable),
            "SteamNetworkingAvailable" => Some(Self::SteamNetworkingAvailable),
            "FriendsPresenceAvailable" => Some(Self::FriendsPresenceAvailable),
            "InventoryAvailable" => Some(Self::InventoryAvailable),
            _ => None,
        }
    }

    fn accepts_generic_availability(self) -> bool {
        !matches!(
            self,
            Self::RuntimeInitialized | Self::AuthenticatedSteamId | Self::AppOwnershipObservation
        )
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum CapabilityStateV1 {
    Unknown,
    ObservedAvailable,
    Unavailable,
    ObservedOwned,
    ObservedNotOwned,
    ObservedUnderBackendVerificationProfile,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RawSteamEvaluationProfileV1 {
    pub semantic_profile_id: String,
    pub app_id: String,
    pub runtime_profile_ref: String,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RawAvailabilityStateV1 {
    Available,
    Unavailable,
    Unknown,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RawTicketVerificationResultV1 {
    Valid,
    Invalid,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RawOwnershipStateV1 {
    Owned,
    NotOwned,
    Unknown,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RawProviderLocatorKindV1 {
    WorkshopPublishedFileId,
    SteamCloudPath,
    UnknownProviderKind(String),
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(tag = "role", rename_all = "snake_case", deny_unknown_fields)]
pub enum RawSteamObservationV1 {
    RuntimeObservation {
        state: RawAvailabilityStateV1,
    },
    LocalSteamIdObservation {
        steam_id: String,
    },
    BackendTicketVerification {
        steam_id: String,
        app_id: String,
        result: RawTicketVerificationResultV1,
    },
    OwnershipObservation {
        app_id: String,
        state: RawOwnershipStateV1,
    },
    CapabilityAvailabilityObservation {
        capability: String,
        state: RawAvailabilityStateV1,
    },
    ProviderLocatorObservation {
        kind: RawProviderLocatorKindV1,
        value: String,
    },
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct AdmittedSteamProfileV1 {
    semantic_profile_id: String,
    app_id: String,
    runtime_profile_ref: String,
}

impl AdmittedSteamProfileV1 {
    pub fn semantic_profile_id(&self) -> &str {
        &self.semantic_profile_id
    }

    pub fn app_id(&self) -> &str {
        &self.app_id
    }

    pub fn runtime_profile_ref(&self) -> &str {
        &self.runtime_profile_ref
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct CapabilityAssessmentEntryV1 {
    kind: SteamCapabilityKindV1,
    state: CapabilityStateV1,
}

impl CapabilityAssessmentEntryV1 {
    pub fn kind(&self) -> SteamCapabilityKindV1 {
        self.kind
    }

    pub fn state(&self) -> CapabilityStateV1 {
        self.state
    }
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize)]
pub struct AdmittedLocalSteamIdObservationV1 {
    steam_id: String,
}

impl AdmittedLocalSteamIdObservationV1 {
    pub fn steam_id(&self) -> &str {
        &self.steam_id
    }
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize)]
pub struct AdmittedProviderLocatorObservationV1 {
    kind: AdmittedProviderLocatorKindV1,
    value: String,
}

impl AdmittedProviderLocatorObservationV1 {
    pub fn kind(&self) -> &AdmittedProviderLocatorKindV1 {
        &self.kind
    }

    pub fn value(&self) -> &str {
        &self.value
    }
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum AdmittedProviderLocatorKindV1 {
    WorkshopPublishedFileId,
    SteamCloudPath,
    UnknownProviderKind(String),
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct SteamCapabilityAssessmentV1 {
    profile: AdmittedSteamProfileV1,
    capabilities: Vec<CapabilityAssessmentEntryV1>,
    authenticated_steam_id: Option<String>,
    local_steam_ids: Vec<AdmittedLocalSteamIdObservationV1>,
    provider_locators: Vec<AdmittedProviderLocatorObservationV1>,
}

impl SteamCapabilityAssessmentV1 {
    pub fn profile(&self) -> &AdmittedSteamProfileV1 {
        &self.profile
    }

    pub fn capabilities(&self) -> &[CapabilityAssessmentEntryV1] {
        &self.capabilities
    }

    pub fn state(&self, kind: SteamCapabilityKindV1) -> CapabilityStateV1 {
        self.capabilities
            .iter()
            .find(|entry| entry.kind == kind)
            .map(|entry| entry.state)
            .unwrap_or(CapabilityStateV1::Unknown)
    }

    pub fn authenticated_steam_id(&self) -> Option<&str> {
        self.authenticated_steam_id.as_deref()
    }

    pub fn local_steam_ids(&self) -> &[AdmittedLocalSteamIdObservationV1] {
        &self.local_steam_ids
    }

    pub fn provider_locators(&self) -> &[AdmittedProviderLocatorObservationV1] {
        &self.provider_locators
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum SteamAssessmentErrorV1 {
    ProviderProfileMismatch(&'static str),
    MalformedProfileField(&'static str),
    MalformedProviderIdentifier(&'static str),
    ResourceBoundExceeded(&'static str),
    UnknownCapability(String),
    ObservationRoleMismatch(SteamCapabilityKindV1),
    Conflict(SteamCapabilityKindV1),
    AuthenticatedIdentityConflict,
}

impl fmt::Display for SteamAssessmentErrorV1 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::ProviderProfileMismatch(field) => {
                write!(f, "Steam provider profile mismatch: {field}")
            }
            Self::MalformedProfileField(field) => write!(f, "malformed profile field: {field}"),
            Self::MalformedProviderIdentifier(field) => {
                write!(f, "malformed provider identifier: {field}")
            }
            Self::ResourceBoundExceeded(field) => write!(f, "resource bound exceeded: {field}"),
            Self::UnknownCapability(name) => write!(f, "unknown Steam capability: {name}"),
            Self::ObservationRoleMismatch(kind) => {
                write!(f, "observation role cannot establish {kind:?}")
            }
            Self::Conflict(kind) => write!(f, "conflicting observations for {kind:?}"),
            Self::AuthenticatedIdentityConflict => {
                write!(f, "conflicting backend-authenticated Steam identities")
            }
        }
    }
}

impl std::error::Error for SteamAssessmentErrorV1 {}

pub fn assess_steam_capabilities_v1(
    raw_profile: RawSteamEvaluationProfileV1,
    observations: Vec<RawSteamObservationV1>,
) -> Result<SteamCapabilityAssessmentV1, SteamAssessmentErrorV1> {
    let profile = admit_profile(raw_profile)?;
    if observations.len() > MAX_OBSERVATIONS {
        return Err(SteamAssessmentErrorV1::ResourceBoundExceeded(
            "observations",
        ));
    }

    let mut states = [CapabilityStateV1::Unknown; 11];
    let mut authenticated_steam_id: Option<String> = None;
    let mut local_steam_ids = Vec::new();
    let mut provider_locators = Vec::new();

    for observation in observations {
        match observation {
            RawSteamObservationV1::RuntimeObservation { state } => {
                merge_state(
                    &mut states[capability_index(SteamCapabilityKindV1::RuntimeInitialized)],
                    availability_state(state),
                    SteamCapabilityKindV1::RuntimeInitialized,
                )?;
            }
            RawSteamObservationV1::LocalSteamIdObservation { steam_id } => {
                validate_provider_id("local_steam_id", &steam_id)?;
                local_steam_ids.push(AdmittedLocalSteamIdObservationV1 { steam_id });
            }
            RawSteamObservationV1::BackendTicketVerification {
                steam_id,
                app_id,
                result,
            } => {
                validate_provider_id("backend_ticket.steam_id", &steam_id)?;
                if app_id != profile.app_id {
                    return Err(SteamAssessmentErrorV1::ProviderProfileMismatch(
                        "backend_ticket.app_id",
                    ));
                }
                let state = match result {
                    RawTicketVerificationResultV1::Valid => {
                        if authenticated_steam_id
                            .as_ref()
                            .is_some_and(|existing| existing != &steam_id)
                        {
                            return Err(SteamAssessmentErrorV1::AuthenticatedIdentityConflict);
                        }
                        authenticated_steam_id = Some(steam_id);
                        CapabilityStateV1::ObservedUnderBackendVerificationProfile
                    }
                    RawTicketVerificationResultV1::Invalid => CapabilityStateV1::Unavailable,
                };
                merge_state(
                    &mut states[capability_index(SteamCapabilityKindV1::AuthenticatedSteamId)],
                    state,
                    SteamCapabilityKindV1::AuthenticatedSteamId,
                )?;
            }
            RawSteamObservationV1::OwnershipObservation { app_id, state } => {
                if app_id != profile.app_id {
                    return Err(SteamAssessmentErrorV1::ProviderProfileMismatch(
                        "ownership.app_id",
                    ));
                }
                let state = match state {
                    RawOwnershipStateV1::Owned => CapabilityStateV1::ObservedOwned,
                    RawOwnershipStateV1::NotOwned => CapabilityStateV1::ObservedNotOwned,
                    RawOwnershipStateV1::Unknown => CapabilityStateV1::Unknown,
                };
                merge_state(
                    &mut states[capability_index(
                        SteamCapabilityKindV1::AppOwnershipObservation,
                    )],
                    state,
                    SteamCapabilityKindV1::AppOwnershipObservation,
                )?;
            }
            RawSteamObservationV1::CapabilityAvailabilityObservation { capability, state } => {
                validate_text(
                    "capability",
                    &capability,
                    MAX_CAPABILITY_NAME_BYTES,
                    false,
                )?;
                let Some(kind) = SteamCapabilityKindV1::from_profile_name(&capability) else {
                    return Err(SteamAssessmentErrorV1::UnknownCapability(capability));
                };
                if !kind.accepts_generic_availability() {
                    return Err(SteamAssessmentErrorV1::ObservationRoleMismatch(kind));
                }
                merge_state(
                    &mut states[capability_index(kind)],
                    availability_state(state),
                    kind,
                )?;
            }
            RawSteamObservationV1::ProviderLocatorObservation { kind, value } => {
                validate_text("provider_locator", &value, MAX_LOCATOR_BYTES, false)?;
                let kind = match kind {
                    RawProviderLocatorKindV1::WorkshopPublishedFileId => {
                        AdmittedProviderLocatorKindV1::WorkshopPublishedFileId
                    }
                    RawProviderLocatorKindV1::SteamCloudPath => {
                        AdmittedProviderLocatorKindV1::SteamCloudPath
                    }
                    RawProviderLocatorKindV1::UnknownProviderKind(value) => {
                        validate_text(
                            "provider_locator_kind",
                            &value,
                            MAX_CAPABILITY_NAME_BYTES,
                            false,
                        )?;
                        AdmittedProviderLocatorKindV1::UnknownProviderKind(value)
                    }
                };
                provider_locators.push(AdmittedProviderLocatorObservationV1 { kind, value });
            }
        }
    }

    local_steam_ids.sort_unstable();
    local_steam_ids.dedup();
    provider_locators.sort_unstable();
    provider_locators.dedup();

    let capabilities = SteamCapabilityKindV1::ALL
        .into_iter()
        .map(|kind| CapabilityAssessmentEntryV1 {
            kind,
            state: states[capability_index(kind)],
        })
        .collect();

    Ok(SteamCapabilityAssessmentV1 {
        profile,
        capabilities,
        authenticated_steam_id,
        local_steam_ids,
        provider_locators,
    })
}

fn admit_profile(
    raw: RawSteamEvaluationProfileV1,
) -> Result<AdmittedSteamProfileV1, SteamAssessmentErrorV1> {
    if raw.semantic_profile_id != STEAM_CAPABILITY_PROFILE_V0_1 {
        return Err(SteamAssessmentErrorV1::ProviderProfileMismatch(
            "semantic_profile_id",
        ));
    }
    validate_text(
        "semantic_profile_id",
        &raw.semantic_profile_id,
        MAX_PROFILE_ID_BYTES,
        false,
    )?;
    validate_text("app_id", &raw.app_id, MAX_APP_ID_BYTES, false)?;
    validate_text(
        "runtime_profile_ref",
        &raw.runtime_profile_ref,
        MAX_RUNTIME_PROFILE_REF_BYTES,
        false,
    )?;
    Ok(AdmittedSteamProfileV1 {
        semantic_profile_id: raw.semantic_profile_id,
        app_id: raw.app_id,
        runtime_profile_ref: raw.runtime_profile_ref,
    })
}

fn availability_state(raw: RawAvailabilityStateV1) -> CapabilityStateV1 {
    match raw {
        RawAvailabilityStateV1::Available => CapabilityStateV1::ObservedAvailable,
        RawAvailabilityStateV1::Unavailable => CapabilityStateV1::Unavailable,
        RawAvailabilityStateV1::Unknown => CapabilityStateV1::Unknown,
    }
}

fn merge_state(
    slot: &mut CapabilityStateV1,
    incoming: CapabilityStateV1,
    kind: SteamCapabilityKindV1,
) -> Result<(), SteamAssessmentErrorV1> {
    if incoming == CapabilityStateV1::Unknown {
        return Ok(());
    }
    if *slot == CapabilityStateV1::Unknown {
        *slot = incoming;
        return Ok(());
    }
    if *slot == incoming {
        return Ok(());
    }
    Err(SteamAssessmentErrorV1::Conflict(kind))
}

const fn capability_index(kind: SteamCapabilityKindV1) -> usize {
    match kind {
        SteamCapabilityKindV1::RuntimeInitialized => 0,
        SteamCapabilityKindV1::AuthenticatedSteamId => 1,
        SteamCapabilityKindV1::AppOwnershipObservation => 2,
        SteamCapabilityKindV1::SteamInputAvailable => 3,
        SteamCapabilityKindV1::StatsAchievementsAvailable => 4,
        SteamCapabilityKindV1::CloudAvailable => 5,
        SteamCapabilityKindV1::WorkshopUGCAvailable => 6,
        SteamCapabilityKindV1::LobbyMatchmakingAvailable => 7,
        SteamCapabilityKindV1::SteamNetworkingAvailable => 8,
        SteamCapabilityKindV1::FriendsPresenceAvailable => 9,
        SteamCapabilityKindV1::InventoryAvailable => 10,
    }
}

fn validate_provider_id(
    field: &'static str,
    value: &str,
) -> Result<(), SteamAssessmentErrorV1> {
    if value.is_empty()
        || value.len() > MAX_PROVIDER_ID_BYTES
        || value.chars().any(char::is_control)
    {
        return Err(SteamAssessmentErrorV1::MalformedProviderIdentifier(field));
    }
    Ok(())
}

fn validate_text(
    field: &'static str,
    value: &str,
    max_bytes: usize,
    allow_empty: bool,
) -> Result<(), SteamAssessmentErrorV1> {
    if (!allow_empty && value.is_empty()) || value.chars().any(char::is_control) {
        return Err(SteamAssessmentErrorV1::MalformedProfileField(field));
    }
    if value.len() > max_bytes {
        return Err(SteamAssessmentErrorV1::ResourceBoundExceeded(field));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn profile() -> RawSteamEvaluationProfileV1 {
        RawSteamEvaluationProfileV1 {
            semantic_profile_id: STEAM_CAPABILITY_PROFILE_V0_1.to_owned(),
            app_id: "profile-app".to_owned(),
            runtime_profile_ref: "symtropy:steam-runtime:test-profile:v0.1".to_owned(),
        }
    }

    #[test]
    fn frozen_corpus_is_exactly_bound() {
        let fixture: serde_json::Value = serde_json::from_str(include_str!(
            "../../../../docs/integrations/providers/steam/fixtures/GAME_STEAM_001_V0_1.json"
        ))
        .unwrap();
        assert_eq!(
            fixture["profile"],
            serde_json::Value::String(STEAM_CAPABILITY_CORPUS_V0_1.to_owned())
        );
        assert_eq!(fixture["schema_version"], 1);
        assert_eq!(fixture["cases"].as_array().unwrap().len(), 15);
        assert_eq!(
            fixture["forbidden_fixture_fields"],
            serde_json::json!([
                "publisher_web_api_key",
                "encrypted_ticket_private_key",
                "backend_secret"
            ])
        );
    }

    #[test]
    fn runtime_does_not_authenticate_or_establish_ownership() {
        let assessment = assess_steam_capabilities_v1(
            profile(),
            vec![RawSteamObservationV1::RuntimeObservation {
                state: RawAvailabilityStateV1::Available,
            }],
        )
        .unwrap();
        assert_eq!(
            assessment.state(SteamCapabilityKindV1::RuntimeInitialized),
            CapabilityStateV1::ObservedAvailable
        );
        assert_eq!(
            assessment.state(SteamCapabilityKindV1::AuthenticatedSteamId),
            CapabilityStateV1::Unknown
        );
        assert_eq!(
            assessment.state(SteamCapabilityKindV1::AppOwnershipObservation),
            CapabilityStateV1::Unknown
        );
    }

    #[test]
    fn local_steam_id_is_observation_not_authentication() {
        let assessment = assess_steam_capabilities_v1(
            profile(),
            vec![RawSteamObservationV1::LocalSteamIdObservation {
                steam_id: "76561198000000001".to_owned(),
            }],
        )
        .unwrap();
        assert_eq!(assessment.local_steam_ids().len(), 1);
        assert_eq!(
            assessment.state(SteamCapabilityKindV1::AuthenticatedSteamId),
            CapabilityStateV1::Unknown
        );
        assert_eq!(assessment.authenticated_steam_id(), None);
    }

    #[test]
    fn backend_verification_is_the_only_authentication_role() {
        let assessment = assess_steam_capabilities_v1(
            profile(),
            vec![RawSteamObservationV1::BackendTicketVerification {
                steam_id: "76561198000000001".to_owned(),
                app_id: "profile-app".to_owned(),
                result: RawTicketVerificationResultV1::Valid,
            }],
        )
        .unwrap();
        assert_eq!(
            assessment.state(SteamCapabilityKindV1::AuthenticatedSteamId),
            CapabilityStateV1::ObservedUnderBackendVerificationProfile
        );
        assert_eq!(
            assessment.authenticated_steam_id(),
            Some("76561198000000001")
        );
    }

    #[test]
    fn backend_appid_substitution_fails_closed() {
        let err = assess_steam_capabilities_v1(
            profile(),
            vec![RawSteamObservationV1::BackendTicketVerification {
                steam_id: "76561198000000001".to_owned(),
                app_id: "wrong-app".to_owned(),
                result: RawTicketVerificationResultV1::Valid,
            }],
        )
        .unwrap_err();
        assert_eq!(
            err,
            SteamAssessmentErrorV1::ProviderProfileMismatch("backend_ticket.app_id")
        );
    }

    #[test]
    fn ownership_does_not_imply_workshop_or_other_capabilities() {
        let assessment = assess_steam_capabilities_v1(
            profile(),
            vec![RawSteamObservationV1::OwnershipObservation {
                app_id: "profile-app".to_owned(),
                state: RawOwnershipStateV1::Owned,
            }],
        )
        .unwrap();
        assert_eq!(
            assessment.state(SteamCapabilityKindV1::AppOwnershipObservation),
            CapabilityStateV1::ObservedOwned
        );
        assert_eq!(
            assessment.state(SteamCapabilityKindV1::WorkshopUGCAvailable),
            CapabilityStateV1::Unknown
        );
        assert_eq!(
            assessment.state(SteamCapabilityKindV1::CloudAvailable),
            CapabilityStateV1::Unknown
        );
    }

    #[test]
    fn steam_input_is_independent() {
        let assessment = assess_steam_capabilities_v1(
            profile(),
            vec![RawSteamObservationV1::CapabilityAvailabilityObservation {
                capability: "SteamInputAvailable".to_owned(),
                state: RawAvailabilityStateV1::Available,
            }],
        )
        .unwrap();
        assert_eq!(
            assessment.state(SteamCapabilityKindV1::SteamInputAvailable),
            CapabilityStateV1::ObservedAvailable
        );
        assert_eq!(
            assessment.state(SteamCapabilityKindV1::AuthenticatedSteamId),
            CapabilityStateV1::Unknown
        );
    }

    #[test]
    fn lobby_availability_does_not_imply_networking() {
        let assessment = assess_steam_capabilities_v1(
            profile(),
            vec![RawSteamObservationV1::CapabilityAvailabilityObservation {
                capability: "LobbyMatchmakingAvailable".to_owned(),
                state: RawAvailabilityStateV1::Available,
            }],
        )
        .unwrap();
        assert_eq!(
            assessment.state(SteamCapabilityKindV1::LobbyMatchmakingAvailable),
            CapabilityStateV1::ObservedAvailable
        );
        assert_eq!(
            assessment.state(SteamCapabilityKindV1::SteamNetworkingAvailable),
            CapabilityStateV1::Unknown
        );
    }

    #[test]
    fn workshop_and_cloud_locators_remain_provider_locators() {
        let assessment = assess_steam_capabilities_v1(
            profile(),
            vec![
                RawSteamObservationV1::ProviderLocatorObservation {
                    kind: RawProviderLocatorKindV1::WorkshopPublishedFileId,
                    value: "12345".to_owned(),
                },
                RawSteamObservationV1::ProviderLocatorObservation {
                    kind: RawProviderLocatorKindV1::SteamCloudPath,
                    value: "slot1.sav".to_owned(),
                },
            ],
        )
        .unwrap();
        assert_eq!(assessment.provider_locators().len(), 2);
        assert_eq!(
            assessment.state(SteamCapabilityKindV1::WorkshopUGCAvailable),
            CapabilityStateV1::Unknown
        );
        assert_eq!(
            assessment.state(SteamCapabilityKindV1::CloudAvailable),
            CapabilityStateV1::Unknown
        );
    }

    #[test]
    fn conflicting_runtime_observations_fail_independent_of_order() {
        let left = vec![
            RawSteamObservationV1::RuntimeObservation {
                state: RawAvailabilityStateV1::Available,
            },
            RawSteamObservationV1::RuntimeObservation {
                state: RawAvailabilityStateV1::Unavailable,
            },
        ];
        let right = left.iter().cloned().rev().collect();
        for observations in [left, right] {
            assert_eq!(
                assess_steam_capabilities_v1(profile(), observations),
                Err(SteamAssessmentErrorV1::Conflict(
                    SteamCapabilityKindV1::RuntimeInitialized
                ))
            );
        }
    }

    #[test]
    fn unknown_future_capability_fails_typed() {
        assert_eq!(
            assess_steam_capabilities_v1(
                profile(),
                vec![RawSteamObservationV1::CapabilityAvailabilityObservation {
                    capability: "FutureUnknownCapability".to_owned(),
                    state: RawAvailabilityStateV1::Available,
                }]
            ),
            Err(SteamAssessmentErrorV1::UnknownCapability(
                "FutureUnknownCapability".to_owned()
            ))
        );
    }

    #[test]
    fn generic_availability_cannot_mint_authenticated_account() {
        assert_eq!(
            assess_steam_capabilities_v1(
                profile(),
                vec![RawSteamObservationV1::CapabilityAvailabilityObservation {
                    capability: "AuthenticatedSteamId".to_owned(),
                    state: RawAvailabilityStateV1::Available,
                }]
            ),
            Err(SteamAssessmentErrorV1::ObservationRoleMismatch(
                SteamCapabilityKindV1::AuthenticatedSteamId
            ))
        );
    }

    #[test]
    fn authenticated_identity_conflict_fails_closed() {
        let observations = vec![
            RawSteamObservationV1::BackendTicketVerification {
                steam_id: "76561198000000001".to_owned(),
                app_id: "profile-app".to_owned(),
                result: RawTicketVerificationResultV1::Valid,
            },
            RawSteamObservationV1::BackendTicketVerification {
                steam_id: "76561198000000002".to_owned(),
                app_id: "profile-app".to_owned(),
                result: RawTicketVerificationResultV1::Valid,
            },
        ];
        assert_eq!(
            assess_steam_capabilities_v1(profile(), observations),
            Err(SteamAssessmentErrorV1::AuthenticatedIdentityConflict)
        );
    }

    #[test]
    fn set_like_evidence_is_canonical_under_input_permutation() {
        let observations = vec![
            RawSteamObservationV1::LocalSteamIdObservation {
                steam_id: "76561198000000002".to_owned(),
            },
            RawSteamObservationV1::LocalSteamIdObservation {
                steam_id: "76561198000000001".to_owned(),
            },
            RawSteamObservationV1::ProviderLocatorObservation {
                kind: RawProviderLocatorKindV1::SteamCloudPath,
                value: "slot1.sav".to_owned(),
            },
            RawSteamObservationV1::CapabilityAvailabilityObservation {
                capability: "SteamInputAvailable".to_owned(),
                state: RawAvailabilityStateV1::Available,
            },
        ];
        let reversed = observations.iter().cloned().rev().collect();
        let left = assess_steam_capabilities_v1(profile(), observations).unwrap();
        let right = assess_steam_capabilities_v1(profile(), reversed).unwrap();
        assert_eq!(left, right);
    }

    #[test]
    fn unknown_locator_is_preserved_but_grants_no_capability() {
        let assessment = assess_steam_capabilities_v1(
            profile(),
            vec![RawSteamObservationV1::ProviderLocatorObservation {
                kind: RawProviderLocatorKindV1::UnknownProviderKind(
                    "future_locator_v9".to_owned(),
                ),
                value: "opaque-value".to_owned(),
            }],
        )
        .unwrap();
        assert!(matches!(
            assessment.provider_locators()[0].kind(),
            AdmittedProviderLocatorKindV1::UnknownProviderKind(value)
                if value == "future_locator_v9"
        ));
        assert!(SteamCapabilityKindV1::ALL
            .into_iter()
            .all(|kind| assessment.state(kind) == CapabilityStateV1::Unknown));
    }
}
