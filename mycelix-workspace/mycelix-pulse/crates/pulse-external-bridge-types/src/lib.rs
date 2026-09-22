#![forbid(unsafe_code)]
//! Pure, loss-aware admission of bounded Matrix/Discord provider observations.
//!
//! This crate intentionally owns no network client, credentials, native Pulse
//! message construction, Holochain persistence, or provider publication.
//!
//! ```text
//! provider observation admitted
//! != Pulse MessageId
//! != RecipientReceipt
//! != native DHT state
//! != outbound publication authority
//! ```
//!
//! Positive admitted values are constructor-controlled and Serialize-only.
//!
//! ```compile_fail
//! use pulse_external_bridge_types::AdmittedExternalBridgeObservationV1;
//! let _: AdmittedExternalBridgeObservationV1 = serde_json::from_str("{}").unwrap();
//! ```

use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;
use std::fmt;

pub const FROZEN_PROFILE_V0_1: &str =
    include_str!("../../../docs/bridges/PULSE_MATRIX_DISCORD_BRIDGE_PROFILE_V0_1.md");
pub const FROZEN_BRIDGE_CORPUS_V0_1: &str =
    include_str!("../../../docs/bridges/fixtures/PULSE_BRIDGE_001_V0_1.json");

pub const MATRIX_PROFILE_V0_1: &str =
    "mycelix:pulse:matrix-appservice:review-2026-09-22:v0.1";
pub const DISCORD_PROFILE_V0_1: &str =
    "mycelix:pulse:discord-bot-oauth-gateway-webhook:review-2026-09-22:v0.1";

const MAX_OBSERVATIONS: usize = 256;
const MAX_ID_BYTES: usize = 512;
const MAX_KIND_BYTES: usize = 256;
const MAX_CONTENT_BYTES: usize = 65_536;
const MAX_COLLECTION_ITEMS: usize = 128;
const MAX_EXTENSION_BYTES: usize = 512;

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ProviderFamily {
    Matrix,
    Discord,
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case", tag = "kind")]
pub enum ProviderScopeV1 {
    Matrix {
        room_id: Option<String>,
        state_key: Option<String>,
    },
    Discord {
        guild_id: Option<String>,
        channel_id: Option<String>,
        thread_id: Option<String>,
    },
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ProviderEventClassV1 {
    MatrixPersistentRoomEvent,
    MatrixStateEvent,
    MatrixEphemeralEvent,
    DiscordMessage,
    DiscordReaction,
    DiscordOther,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ProviderActorKindV1 {
    MatrixUser,
    DiscordUser,
    DiscordBotApplication,
    DiscordWebhook,
    UnknownProviderActor,
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub struct RawProviderActorV1 {
    pub actor_id: String,
    pub actor_kind: ProviderActorKindV1,
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub struct RawProviderDeliveryV1 {
    pub delivery_id: String,
    /// Commitment supplied by the upstream provider parser/source theorem.
    /// This crate validates syntax but does not claim to authenticate or
    /// independently derive provider bytes.
    pub delivery_commitment: Option<String>,
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case", tag = "kind", content = "value")]
pub enum ContentObservationV1 {
    Present(String),
    UnavailableUnderProviderAccessProfile,
    NotProjected,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ProviderRelationKindV1 {
    ReplyToProviderEvent,
    ThreadParentProviderEvent,
    ReactionToProviderEvent,
    RevisionOfProviderEvent,
    TombstoneOfProviderEvent,
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub struct ProviderRelationObservationV1 {
    pub kind: ProviderRelationKindV1,
    pub target_provider_event_id: String,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AttachmentAvailabilityV1 {
    ReferenceOnly,
    SuppliedByUpstream,
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub struct ProviderAttachmentRefV1 {
    pub provider_attachment_id: String,
    pub provider_locator: Option<String>,
    pub availability: AttachmentAvailabilityV1,
    /// Optional portable content commitment supplied by an upstream content
    /// theorem. Presence here does not establish retrieval/disclosure authority.
    pub content_commitment: Option<String>,
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub struct ProviderModerationObservationV1 {
    pub provider_kind: String,
    pub provider_ref: String,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum LossMarkerV1 {
    LosslessWithinProjectionProfile,
    UnsupportedProviderRelation,
    AudienceSemanticsNotRepresentable,
    AttachmentNotFetched,
    ProviderModerationObservationOnly,
    ContentUnavailableUnderProviderAccessProfile,
    UnknownProviderExtension,
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub struct RawBridgeOriginV1 {
    pub source_provider: ProviderFamily,
    pub source_provider_event_id: Option<String>,
    /// Exact commitment to the prior bridge/source semantic observation.
    pub source_semantic_commitment: String,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RawExternalBridgeObservationV1 {
    pub provider_family: ProviderFamily,
    pub provider_profile: String,
    pub provider_event_class: ProviderEventClassV1,
    pub provider_event_kind: String,
    pub provider_event_id: Option<String>,
    pub provider_delivery: Option<RawProviderDeliveryV1>,
    pub provider_scope: ProviderScopeV1,
    pub provider_actor: Option<RawProviderActorV1>,
    pub provider_chronology: Option<String>,
    pub content: ContentObservationV1,
    #[serde(default)]
    pub relations: Vec<ProviderRelationObservationV1>,
    #[serde(default)]
    pub attachments: Vec<ProviderAttachmentRefV1>,
    #[serde(default)]
    pub moderation_observations: Vec<ProviderModerationObservationV1>,
    pub visibility_observation: Option<String>,
    pub bridge_origin: Option<RawBridgeOriginV1>,
    #[serde(default)]
    pub provider_extension_markers: Vec<String>,
    #[serde(default)]
    pub losses: Vec<LossMarkerV1>,
}

/// Constructor-controlled positive external observation.
///
/// This type deliberately does not implement `Deserialize`. It is still only
/// an external observation and has no native Pulse or provider-effect methods.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize)]
pub struct AdmittedExternalBridgeObservationV1 {
    provider_family: ProviderFamily,
    provider_profile: String,
    provider_event_class: ProviderEventClassV1,
    provider_event_kind: String,
    provider_event_id: Option<String>,
    provider_delivery: Option<RawProviderDeliveryV1>,
    provider_scope: ProviderScopeV1,
    provider_actor: Option<RawProviderActorV1>,
    provider_chronology: Option<String>,
    content: ContentObservationV1,
    relations: Vec<ProviderRelationObservationV1>,
    attachments: Vec<ProviderAttachmentRefV1>,
    moderation_observations: Vec<ProviderModerationObservationV1>,
    visibility_observation: Option<String>,
    bridge_origin: Option<RawBridgeOriginV1>,
    provider_extension_markers: Vec<String>,
    losses: Vec<LossMarkerV1>,
}

impl AdmittedExternalBridgeObservationV1 {
    pub fn provider_family(&self) -> ProviderFamily {
        self.provider_family
    }

    pub fn provider_profile(&self) -> &str {
        &self.provider_profile
    }

    pub fn provider_event_class(&self) -> ProviderEventClassV1 {
        self.provider_event_class
    }

    pub fn provider_event_kind(&self) -> &str {
        &self.provider_event_kind
    }

    pub fn provider_event_id(&self) -> Option<&str> {
        self.provider_event_id.as_deref()
    }

    pub fn provider_delivery(&self) -> Option<&RawProviderDeliveryV1> {
        self.provider_delivery.as_ref()
    }

    pub fn provider_scope(&self) -> &ProviderScopeV1 {
        &self.provider_scope
    }

    pub fn provider_actor(&self) -> Option<&RawProviderActorV1> {
        self.provider_actor.as_ref()
    }

    pub fn provider_chronology(&self) -> Option<&str> {
        self.provider_chronology.as_deref()
    }

    pub fn content(&self) -> &ContentObservationV1 {
        &self.content
    }

    pub fn relations(&self) -> &[ProviderRelationObservationV1] {
        &self.relations
    }

    pub fn attachments(&self) -> &[ProviderAttachmentRefV1] {
        &self.attachments
    }

    pub fn moderation_observations(&self) -> &[ProviderModerationObservationV1] {
        &self.moderation_observations
    }

    pub fn visibility_observation(&self) -> Option<&str> {
        self.visibility_observation.as_deref()
    }

    pub fn bridge_origin(&self) -> Option<&RawBridgeOriginV1> {
        self.bridge_origin.as_ref()
    }

    pub fn provider_extension_markers(&self) -> &[String] {
        &self.provider_extension_markers
    }

    pub fn losses(&self) -> &[LossMarkerV1] {
        &self.losses
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum BridgeAdmissionError {
    ProviderProfileMismatch,
    ProviderScopeMismatch,
    ProviderEventClassMismatch,
    MissingProviderCoordinate(&'static str),
    EmptyField(&'static str),
    FieldTooLong(&'static str),
    TooManyItems(&'static str),
    InvalidCommitment(&'static str),
    ConflictingLossClaim,
    MissingRequiredLoss(&'static str),
    ConflictingDeliveryCoordinate,
}

impl fmt::Display for BridgeAdmissionError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::ProviderProfileMismatch => write!(f, "provider profile mismatch"),
            Self::ProviderScopeMismatch => write!(f, "provider scope mismatch"),
            Self::ProviderEventClassMismatch => write!(f, "provider event class mismatch"),
            Self::MissingProviderCoordinate(field) => {
                write!(f, "missing provider coordinate: {field}")
            }
            Self::EmptyField(field) => write!(f, "empty field: {field}"),
            Self::FieldTooLong(field) => write!(f, "field too long: {field}"),
            Self::TooManyItems(field) => write!(f, "too many items: {field}"),
            Self::InvalidCommitment(field) => write!(f, "invalid commitment: {field}"),
            Self::ConflictingLossClaim => write!(f, "conflicting loss claim"),
            Self::MissingRequiredLoss(field) => {
                write!(f, "missing required loss marker: {field}")
            }
            Self::ConflictingDeliveryCoordinate => {
                write!(f, "conflicting provider delivery coordinate")
            }
        }
    }
}

impl std::error::Error for BridgeAdmissionError {}

pub fn admit_external_bridge_observation(
    raw: RawExternalBridgeObservationV1,
) -> Result<AdmittedExternalBridgeObservationV1, BridgeAdmissionError> {
    validate_profile_and_shape(&raw)?;
    validate_text("provider_event_kind", &raw.provider_event_kind, MAX_KIND_BYTES)?;
    validate_optional_text("provider_event_id", raw.provider_event_id.as_deref(), MAX_ID_BYTES)?;
    validate_optional_text(
        "provider_chronology",
        raw.provider_chronology.as_deref(),
        MAX_ID_BYTES,
    )?;
    validate_optional_text(
        "visibility_observation",
        raw.visibility_observation.as_deref(),
        MAX_KIND_BYTES,
    )?;
    validate_scope(&raw.provider_scope)?;
    validate_actor(raw.provider_actor.as_ref())?;
    validate_delivery(raw.provider_family, raw.provider_delivery.as_ref())?;
    validate_content(&raw.content)?;
    validate_collection_lengths(&raw)?;
    validate_relations(&raw.relations)?;
    validate_attachments(&raw.attachments)?;
    validate_moderation(&raw.moderation_observations)?;
    validate_bridge_origin(raw.bridge_origin.as_ref())?;
    validate_extensions(&raw.provider_extension_markers)?;

    let mut relations = raw.relations;
    relations.sort();
    relations.dedup();

    let mut attachments = raw.attachments;
    attachments.sort();
    attachments.dedup();

    let mut moderation_observations = raw.moderation_observations;
    moderation_observations.sort();
    moderation_observations.dedup();

    let mut provider_extension_markers = raw.provider_extension_markers;
    provider_extension_markers.sort();
    provider_extension_markers.dedup();

    let mut losses = raw.losses;
    losses.sort();
    losses.dedup();

    validate_loss_coherence(
        &raw.content,
        &attachments,
        &moderation_observations,
        &provider_extension_markers,
        &losses,
    )?;

    Ok(AdmittedExternalBridgeObservationV1 {
        provider_family: raw.provider_family,
        provider_profile: raw.provider_profile,
        provider_event_class: raw.provider_event_class,
        provider_event_kind: raw.provider_event_kind,
        provider_event_id: raw.provider_event_id,
        provider_delivery: raw.provider_delivery,
        provider_scope: raw.provider_scope,
        provider_actor: raw.provider_actor,
        provider_chronology: raw.provider_chronology,
        content: raw.content,
        relations,
        attachments,
        moderation_observations,
        visibility_observation: raw.visibility_observation,
        bridge_origin: raw.bridge_origin,
        provider_extension_markers,
        losses,
    })
}

/// Admit a set of external observations and fail closed if the same exact
/// provider delivery coordinate carries conflicting commitments.
///
/// Output ordering is canonical for set-like callers.
pub fn admit_external_bridge_batch(
    raw: Vec<RawExternalBridgeObservationV1>,
) -> Result<Vec<AdmittedExternalBridgeObservationV1>, BridgeAdmissionError> {
    if raw.len() > MAX_OBSERVATIONS {
        return Err(BridgeAdmissionError::TooManyItems("observations"));
    }

    let mut admitted = Vec::with_capacity(raw.len());
    for observation in raw {
        admitted.push(admit_external_bridge_observation(observation)?);
    }

    let mut deliveries: BTreeMap<(ProviderFamily, String, String), Option<String>> = BTreeMap::new();

    for observation in &admitted {
        let Some(delivery) = observation.provider_delivery.as_ref() else {
            continue;
        };
        let key = (
            observation.provider_family,
            observation.provider_profile.clone(),
            delivery.delivery_id.clone(),
        );
        if let Some(previous) = deliveries.get(&key) {
            if previous != &delivery.delivery_commitment {
                return Err(BridgeAdmissionError::ConflictingDeliveryCoordinate);
            }
        } else {
            deliveries.insert(key, delivery.delivery_commitment.clone());
        }
    }

    admitted.sort();
    admitted.dedup();
    Ok(admitted)
}

fn validate_profile_and_shape(
    raw: &RawExternalBridgeObservationV1,
) -> Result<(), BridgeAdmissionError> {
    match raw.provider_family {
        ProviderFamily::Matrix => {
            if raw.provider_profile != MATRIX_PROFILE_V0_1 {
                return Err(BridgeAdmissionError::ProviderProfileMismatch);
            }
            if !matches!(&raw.provider_scope, ProviderScopeV1::Matrix { .. }) {
                return Err(BridgeAdmissionError::ProviderScopeMismatch);
            }
            if raw.provider_delivery.is_none() {
                return Err(BridgeAdmissionError::MissingProviderCoordinate(
                    "matrix_delivery",
                ));
            }
            if let Some(actor) = &raw.provider_actor {
                if !matches!(
                    actor.actor_kind,
                    ProviderActorKindV1::MatrixUser | ProviderActorKindV1::UnknownProviderActor
                ) {
                    return Err(BridgeAdmissionError::ProviderScopeMismatch);
                }
            }
            match raw.provider_event_class {
                ProviderEventClassV1::MatrixStateEvent => {
                    if raw.provider_event_id.is_none() {
                        return Err(BridgeAdmissionError::MissingProviderCoordinate(
                            "matrix_event_id",
                        ));
                    }
                    let ProviderScopeV1::Matrix { state_key, .. } = &raw.provider_scope else {
                        unreachable!("scope checked above");
                    };
                    if state_key.is_none() {
                        return Err(BridgeAdmissionError::ProviderEventClassMismatch);
                    }
                }
                ProviderEventClassV1::MatrixPersistentRoomEvent => {
                    if raw.provider_event_id.is_none() {
                        return Err(BridgeAdmissionError::MissingProviderCoordinate(
                            "matrix_event_id",
                        ));
                    }
                    let ProviderScopeV1::Matrix { state_key, .. } = &raw.provider_scope else {
                        unreachable!("scope checked above");
                    };
                    if state_key.is_some() {
                        return Err(BridgeAdmissionError::ProviderEventClassMismatch);
                    }
                }
                ProviderEventClassV1::MatrixEphemeralEvent => {}
                _ => return Err(BridgeAdmissionError::ProviderEventClassMismatch),
            }
        }
        ProviderFamily::Discord => {
            if raw.provider_profile != DISCORD_PROFILE_V0_1 {
                return Err(BridgeAdmissionError::ProviderProfileMismatch);
            }
            let ProviderScopeV1::Discord {
                guild_id,
                channel_id,
                thread_id,
            } = &raw.provider_scope
            else {
                return Err(BridgeAdmissionError::ProviderScopeMismatch);
            };
            if guild_id.is_none() && channel_id.is_none() && thread_id.is_none() {
                return Err(BridgeAdmissionError::MissingProviderCoordinate(
                    "discord_scope",
                ));
            }
            if matches!(
                raw.provider_event_class,
                ProviderEventClassV1::MatrixPersistentRoomEvent
                    | ProviderEventClassV1::MatrixStateEvent
                    | ProviderEventClassV1::MatrixEphemeralEvent
            ) {
                return Err(BridgeAdmissionError::ProviderEventClassMismatch);
            }
            if matches!(
                raw.provider_event_class,
                ProviderEventClassV1::DiscordMessage | ProviderEventClassV1::DiscordReaction
            ) && raw.provider_event_id.is_none()
            {
                return Err(BridgeAdmissionError::MissingProviderCoordinate(
                    "discord_event_id",
                ));
            }
            if let Some(actor) = &raw.provider_actor {
                if actor.actor_kind == ProviderActorKindV1::MatrixUser {
                    return Err(BridgeAdmissionError::ProviderScopeMismatch);
                }
            }
        }
    }
    Ok(())
}

fn validate_scope(scope: &ProviderScopeV1) -> Result<(), BridgeAdmissionError> {
    match scope {
        ProviderScopeV1::Matrix { room_id, state_key } => {
            validate_optional_text("matrix_room_id", room_id.as_deref(), MAX_ID_BYTES)?;
            if let Some(state_key) = state_key {
                if state_key.len() > MAX_ID_BYTES {
                    return Err(BridgeAdmissionError::FieldTooLong("matrix_state_key"));
                }
            }
        }
        ProviderScopeV1::Discord {
            guild_id,
            channel_id,
            thread_id,
        } => {
            validate_optional_text("discord_guild_id", guild_id.as_deref(), MAX_ID_BYTES)?;
            validate_optional_text("discord_channel_id", channel_id.as_deref(), MAX_ID_BYTES)?;
            validate_optional_text("discord_thread_id", thread_id.as_deref(), MAX_ID_BYTES)?;
        }
    }
    Ok(())
}

fn validate_actor(actor: Option<&RawProviderActorV1>) -> Result<(), BridgeAdmissionError> {
    if let Some(actor) = actor {
        validate_text("provider_actor_id", &actor.actor_id, MAX_ID_BYTES)?;
    }
    Ok(())
}

fn validate_delivery(
    family: ProviderFamily,
    delivery: Option<&RawProviderDeliveryV1>,
) -> Result<(), BridgeAdmissionError> {
    let Some(delivery) = delivery else {
        return Ok(());
    };
    validate_text("provider_delivery_id", &delivery.delivery_id, MAX_ID_BYTES)?;
    if let Some(commitment) = &delivery.delivery_commitment {
        validate_sha256_commitment("provider_delivery_commitment", commitment)?;
    } else if family == ProviderFamily::Matrix {
        return Err(BridgeAdmissionError::MissingProviderCoordinate(
            "matrix_delivery_commitment",
        ));
    }
    Ok(())
}

fn validate_content(content: &ContentObservationV1) -> Result<(), BridgeAdmissionError> {
    if let ContentObservationV1::Present(value) = content {
        if value.len() > MAX_CONTENT_BYTES {
            return Err(BridgeAdmissionError::FieldTooLong("content"));
        }
    }
    Ok(())
}

fn validate_collection_lengths(
    raw: &RawExternalBridgeObservationV1,
) -> Result<(), BridgeAdmissionError> {
    for (field, len) in [
        ("relations", raw.relations.len()),
        ("attachments", raw.attachments.len()),
        ("moderation_observations", raw.moderation_observations.len()),
        (
            "provider_extension_markers",
            raw.provider_extension_markers.len(),
        ),
        ("losses", raw.losses.len()),
    ] {
        if len > MAX_COLLECTION_ITEMS {
            return Err(BridgeAdmissionError::TooManyItems(field));
        }
    }
    Ok(())
}

fn validate_relations(
    relations: &[ProviderRelationObservationV1],
) -> Result<(), BridgeAdmissionError> {
    for relation in relations {
        validate_text(
            "relation_target_provider_event_id",
            &relation.target_provider_event_id,
            MAX_ID_BYTES,
        )?;
    }
    Ok(())
}

fn validate_attachments(
    attachments: &[ProviderAttachmentRefV1],
) -> Result<(), BridgeAdmissionError> {
    for attachment in attachments {
        validate_text(
            "provider_attachment_id",
            &attachment.provider_attachment_id,
            MAX_ID_BYTES,
        )?;
        validate_optional_text(
            "provider_attachment_locator",
            attachment.provider_locator.as_deref(),
            MAX_ID_BYTES,
        )?;
        if let Some(commitment) = &attachment.content_commitment {
            validate_sha256_commitment("attachment_content_commitment", commitment)?;
        }
    }
    Ok(())
}

fn validate_moderation(
    moderation: &[ProviderModerationObservationV1],
) -> Result<(), BridgeAdmissionError> {
    for observation in moderation {
        validate_text(
            "moderation_provider_kind",
            &observation.provider_kind,
            MAX_KIND_BYTES,
        )?;
        validate_text(
            "moderation_provider_ref",
            &observation.provider_ref,
            MAX_ID_BYTES,
        )?;
    }
    Ok(())
}

fn validate_bridge_origin(origin: Option<&RawBridgeOriginV1>) -> Result<(), BridgeAdmissionError> {
    let Some(origin) = origin else {
        return Ok(());
    };
    validate_optional_text(
        "bridge_origin_provider_event_id",
        origin.source_provider_event_id.as_deref(),
        MAX_ID_BYTES,
    )?;
    validate_sha256_commitment(
        "bridge_origin_source_semantic_commitment",
        &origin.source_semantic_commitment,
    )
}

fn validate_extensions(extensions: &[String]) -> Result<(), BridgeAdmissionError> {
    for extension in extensions {
        validate_text(
            "provider_extension_marker",
            extension,
            MAX_EXTENSION_BYTES,
        )?;
    }
    Ok(())
}

fn validate_loss_coherence(
    content: &ContentObservationV1,
    attachments: &[ProviderAttachmentRefV1],
    moderation: &[ProviderModerationObservationV1],
    extensions: &[String],
    losses: &[LossMarkerV1],
) -> Result<(), BridgeAdmissionError> {
    if losses.is_empty() {
        return Err(BridgeAdmissionError::MissingRequiredLoss(
            "projection_loss_classification",
        ));
    }

    let has_lossless = losses.contains(&LossMarkerV1::LosslessWithinProjectionProfile);
    if has_lossless && losses.len() != 1 {
        return Err(BridgeAdmissionError::ConflictingLossClaim);
    }

    match content {
        ContentObservationV1::UnavailableUnderProviderAccessProfile => {
            require_loss(
                losses,
                LossMarkerV1::ContentUnavailableUnderProviderAccessProfile,
                "content_unavailable",
            )?;
        }
        ContentObservationV1::Present(_) => {
            if losses.contains(&LossMarkerV1::ContentUnavailableUnderProviderAccessProfile) {
                return Err(BridgeAdmissionError::ConflictingLossClaim);
            }
        }
        ContentObservationV1::NotProjected => {}
    }

    if attachments
        .iter()
        .any(|attachment| attachment.availability == AttachmentAvailabilityV1::ReferenceOnly)
    {
        require_loss(
            losses,
            LossMarkerV1::AttachmentNotFetched,
            "attachment_not_fetched",
        )?;
    }

    if !moderation.is_empty() {
        require_loss(
            losses,
            LossMarkerV1::ProviderModerationObservationOnly,
            "provider_moderation_observation_only",
        )?;
    }

    if !extensions.is_empty() {
        require_loss(
            losses,
            LossMarkerV1::UnknownProviderExtension,
            "unknown_provider_extension",
        )?;
    }

    Ok(())
}

fn require_loss(
    losses: &[LossMarkerV1],
    expected: LossMarkerV1,
    field: &'static str,
) -> Result<(), BridgeAdmissionError> {
    if losses.contains(&expected) {
        Ok(())
    } else {
        Err(BridgeAdmissionError::MissingRequiredLoss(field))
    }
}

fn validate_text(
    field: &'static str,
    value: &str,
    max: usize,
) -> Result<(), BridgeAdmissionError> {
    if value.is_empty() {
        return Err(BridgeAdmissionError::EmptyField(field));
    }
    if value.len() > max {
        return Err(BridgeAdmissionError::FieldTooLong(field));
    }
    Ok(())
}

fn validate_optional_text(
    field: &'static str,
    value: Option<&str>,
    max: usize,
) -> Result<(), BridgeAdmissionError> {
    if let Some(value) = value {
        validate_text(field, value, max)?;
    }
    Ok(())
}

fn validate_sha256_commitment(
    field: &'static str,
    value: &str,
) -> Result<(), BridgeAdmissionError> {
    let Some(hex) = value.strip_prefix("sha256:") else {
        return Err(BridgeAdmissionError::InvalidCommitment(field));
    };
    if hex.len() != 64
        || !hex
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
    {
        return Err(BridgeAdmissionError::InvalidCommitment(field));
    }
    Ok(())
}
