#![forbid(unsafe_code)]
//! Pure, loss-aware Matrix/Discord external bridge observations for Mycelix Pulse.
//!
//! This crate performs no network I/O, holds no credentials, commits nothing to
//! Holochain, and cannot mint native Pulse message/receipt authority.
//!
//! ```compile_fail
//! use pulse_external_bridge_types::AdmittedExternalBridgeObservationV1;
//!
//! fn requires_deserialize<T: for<'de> serde::Deserialize<'de>>() {}
//! requires_deserialize::<AdmittedExternalBridgeObservationV1>();
//! ```

use serde::{Deserialize, Serialize};
use std::fmt;

pub const MATRIX_PROFILE_V0_1: &str =
    "mycelix:pulse:matrix-appservice:review-2026-09-22:v0.1";
pub const DISCORD_PROFILE_V0_1: &str =
    "mycelix:pulse:discord-bot-oauth-gateway-webhook:review-2026-09-22:v0.1";
pub const BRIDGE_CORPUS_PROFILE_V0_1: &str =
    "mycelix:pulse:matrix-discord-bridge-corpus:v0.1";

const MAX_PROVIDER_ID_BYTES: usize = 1024;
const MAX_PROFILE_BYTES: usize = 256;
const MAX_EVENT_KIND_BYTES: usize = 256;
const MAX_CONTENT_BYTES: usize = 65_536;
const MAX_LOCATOR_BYTES: usize = 4096;
const MAX_METADATA_VALUE_BYTES: usize = 4096;
const MAX_RELATIONS: usize = 64;
const MAX_ATTACHMENTS: usize = 64;
const MAX_MODERATION_OBSERVATIONS: usize = 128;

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ProviderFamilyV1 {
    Matrix,
    Discord,
}

impl ProviderFamilyV1 {
    pub const fn expected_profile(self) -> &'static str {
        match self {
            Self::Matrix => MATRIX_PROFILE_V0_1,
            Self::Discord => DISCORD_PROFILE_V0_1,
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(tag = "provider", rename_all = "snake_case", deny_unknown_fields)]
pub enum RawProviderScopeV1 {
    Matrix { room_id: Option<String> },
    Discord {
        guild_id: Option<String>,
        channel_id: String,
        thread_id: Option<String>,
    },
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(tag = "provider", rename_all = "snake_case", deny_unknown_fields)]
pub enum RawDeliveryCoordinateV1 {
    Matrix {
        transaction_id: String,
        transaction_commitment: String,
    },
    Discord {
        gateway_session_id: Option<String>,
        gateway_sequence: Option<u64>,
        webhook_delivery_id: Option<String>,
    },
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RawActorKindV1 {
    MatrixUser,
    MatrixApplicationService,
    DiscordUser,
    DiscordBotApplication,
    DiscordWebhook,
    DiscordApplication,
    UnknownProviderKind(String),
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RawActorObservationV1 {
    pub actor_id: String,
    pub kind: RawActorKindV1,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RawDiscordEventKindV1 {
    MessageCreate,
    MessageUpdate,
    MessageDelete,
    ReactionAdd,
    ReactionRemove,
    ThreadCreate,
    ThreadUpdate,
    ThreadDelete,
    UnknownProviderKind(String),
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(tag = "provider", rename_all = "snake_case", deny_unknown_fields)]
pub enum RawProviderEventKindV1 {
    Matrix {
        event_type: String,
        state_key: Option<String>,
        ephemeral: bool,
    },
    Discord { kind: RawDiscordEventKindV1 },
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(tag = "state", content = "value", rename_all = "snake_case")]
pub enum RawContentObservationV1 {
    AvailableUtf8(String),
    UnavailableUnderProviderAccessProfile,
    NotApplicable,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RawRelationKindV1 {
    RevisionOf,
    DeletionOf,
    ReplyTo,
    ThreadParent,
    ReactionTo,
    UnknownProviderRelation(String),
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RawRelationObservationV1 {
    pub kind: RawRelationKindV1,
    pub target_event_id: String,
    pub provider_relation_ref: Option<String>,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RawAttachmentReferenceV1 {
    pub provider_attachment_id: String,
    pub locator: String,
    pub media_type_hint: Option<String>,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RawModerationKindV1 {
    MatrixPowerLevel,
    DiscordRole,
    DiscordPermission,
    UnknownProviderModerationKind(String),
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RawModerationObservationV1 {
    pub kind: RawModerationKindV1,
    pub subject_provider_id: String,
    pub value: String,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RawBridgeOriginV1 {
    pub source_provider: ProviderFamilyV1,
    pub source_provider_profile: String,
    pub source_event_id: String,
    pub source_semantic_commitment: String,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RawExternalBridgeObservationV1 {
    pub provider: ProviderFamilyV1,
    pub provider_profile: String,
    pub event_id: String,
    pub delivery: Option<RawDeliveryCoordinateV1>,
    pub scope: RawProviderScopeV1,
    pub actor: Option<RawActorObservationV1>,
    pub event_kind: RawProviderEventKindV1,
    pub content: RawContentObservationV1,
    #[serde(default)]
    pub relations: Vec<RawRelationObservationV1>,
    #[serde(default)]
    pub attachments: Vec<RawAttachmentReferenceV1>,
    #[serde(default)]
    pub moderation: Vec<RawModerationObservationV1>,
    pub bridge_origin: Option<RawBridgeOriginV1>,
    #[serde(default)]
    pub audience_semantics_not_representable: bool,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct MatrixScopeV1 {
    room_id: Option<String>,
}

impl MatrixScopeV1 {
    pub fn room_id(&self) -> Option<&str> {
        self.room_id.as_deref()
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct DiscordScopeV1 {
    guild_id: Option<String>,
    channel_id: String,
    thread_id: Option<String>,
}

impl DiscordScopeV1 {
    pub fn guild_id(&self) -> Option<&str> {
        self.guild_id.as_deref()
    }

    pub fn channel_id(&self) -> &str {
        &self.channel_id
    }

    pub fn thread_id(&self) -> Option<&str> {
        self.thread_id.as_deref()
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
#[serde(tag = "provider", content = "scope", rename_all = "snake_case")]
pub enum AdmittedProviderScopeV1 {
    Matrix(MatrixScopeV1),
    Discord(DiscordScopeV1),
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct MatrixDeliveryCoordinateV1 {
    transaction_id: String,
    transaction_commitment: String,
}

impl MatrixDeliveryCoordinateV1 {
    pub fn transaction_id(&self) -> &str {
        &self.transaction_id
    }

    pub fn transaction_commitment(&self) -> &str {
        &self.transaction_commitment
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct DiscordDeliveryCoordinateV1 {
    gateway_session_id: Option<String>,
    gateway_sequence: Option<u64>,
    webhook_delivery_id: Option<String>,
}

impl DiscordDeliveryCoordinateV1 {
    pub fn gateway_session_id(&self) -> Option<&str> {
        self.gateway_session_id.as_deref()
    }

    pub fn gateway_sequence(&self) -> Option<u64> {
        self.gateway_sequence
    }

    pub fn webhook_delivery_id(&self) -> Option<&str> {
        self.webhook_delivery_id.as_deref()
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
#[serde(tag = "provider", content = "delivery", rename_all = "snake_case")]
pub enum AdmittedDeliveryCoordinateV1 {
    Matrix(MatrixDeliveryCoordinateV1),
    Discord(DiscordDeliveryCoordinateV1),
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct AdmittedActorObservationV1 {
    actor_id: String,
    kind: RawActorKindV1,
}

impl AdmittedActorObservationV1 {
    pub fn actor_id(&self) -> &str {
        &self.actor_id
    }

    pub fn kind(&self) -> &RawActorKindV1 {
        &self.kind
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub enum MatrixEventClassV1 {
    PersistentEvent,
    StateEvent,
    EphemeralEvent,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
#[serde(tag = "provider", rename_all = "snake_case")]
pub enum AdmittedProviderEventKindV1 {
    Matrix {
        event_type: String,
        state_key: Option<String>,
        class: MatrixEventClassV1,
    },
    Discord { kind: RawDiscordEventKindV1 },
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub enum AdmittedContentObservationV1 {
    AvailableUtf8(String),
    UnavailableUnderProviderAccessProfile,
    NotApplicable,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct AdmittedRelationObservationV1 {
    kind: RawRelationKindV1,
    target_event_id: String,
    provider_relation_ref: Option<String>,
}

impl AdmittedRelationObservationV1 {
    pub fn kind(&self) -> &RawRelationKindV1 {
        &self.kind
    }

    pub fn target_event_id(&self) -> &str {
        &self.target_event_id
    }

    pub fn provider_relation_ref(&self) -> Option<&str> {
        self.provider_relation_ref.as_deref()
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct AdmittedAttachmentReferenceV1 {
    provider_attachment_id: String,
    locator: String,
    media_type_hint: Option<String>,
}

impl AdmittedAttachmentReferenceV1 {
    pub fn provider_attachment_id(&self) -> &str {
        &self.provider_attachment_id
    }

    pub fn locator(&self) -> &str {
        &self.locator
    }

    pub fn media_type_hint(&self) -> Option<&str> {
        self.media_type_hint.as_deref()
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct AdmittedModerationObservationV1 {
    kind: RawModerationKindV1,
    subject_provider_id: String,
    value: String,
}

impl AdmittedModerationObservationV1 {
    pub fn kind(&self) -> &RawModerationKindV1 {
        &self.kind
    }

    pub fn subject_provider_id(&self) -> &str {
        &self.subject_provider_id
    }

    pub fn value(&self) -> &str {
        &self.value
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct AdmittedBridgeOriginV1 {
    source_provider: ProviderFamilyV1,
    source_provider_profile: String,
    source_event_id: String,
    source_semantic_commitment: String,
}

impl AdmittedBridgeOriginV1 {
    pub fn source_provider(&self) -> ProviderFamilyV1 {
        self.source_provider
    }

    pub fn source_provider_profile(&self) -> &str {
        &self.source_provider_profile
    }

    pub fn source_event_id(&self) -> &str {
        &self.source_event_id
    }

    pub fn source_semantic_commitment(&self) -> &str {
        &self.source_semantic_commitment
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum LossMarkerV1 {
    DroppedUnsupportedRelation,
    AudienceSemanticsNotRepresentable,
    AttachmentNotFetched,
    ContentUnavailableUnderProviderAccessProfile,
    ProviderModerationStateOnly,
    UnknownProviderExtension,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub enum ProjectionClassV1 {
    LosslessWithinProfile,
    LossyWithinProfile(Vec<LossMarkerV1>),
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct AdmittedExternalBridgeObservationV1 {
    provider: ProviderFamilyV1,
    provider_profile: String,
    event_id: String,
    delivery: Option<AdmittedDeliveryCoordinateV1>,
    scope: AdmittedProviderScopeV1,
    actor: Option<AdmittedActorObservationV1>,
    event_kind: AdmittedProviderEventKindV1,
    content: AdmittedContentObservationV1,
    relations: Vec<AdmittedRelationObservationV1>,
    attachments: Vec<AdmittedAttachmentReferenceV1>,
    moderation: Vec<AdmittedModerationObservationV1>,
    bridge_origin: Option<AdmittedBridgeOriginV1>,
    projection_class: ProjectionClassV1,
}

impl AdmittedExternalBridgeObservationV1 {
    pub fn provider(&self) -> ProviderFamilyV1 {
        self.provider
    }

    pub fn provider_profile(&self) -> &str {
        &self.provider_profile
    }

    pub fn event_id(&self) -> &str {
        &self.event_id
    }

    pub fn delivery(&self) -> Option<&AdmittedDeliveryCoordinateV1> {
        self.delivery.as_ref()
    }

    pub fn scope(&self) -> &AdmittedProviderScopeV1 {
        &self.scope
    }

    pub fn actor(&self) -> Option<&AdmittedActorObservationV1> {
        self.actor.as_ref()
    }

    pub fn event_kind(&self) -> &AdmittedProviderEventKindV1 {
        &self.event_kind
    }

    pub fn content(&self) -> &AdmittedContentObservationV1 {
        &self.content
    }

    pub fn relations(&self) -> &[AdmittedRelationObservationV1] {
        &self.relations
    }

    pub fn attachments(&self) -> &[AdmittedAttachmentReferenceV1] {
        &self.attachments
    }

    pub fn moderation(&self) -> &[AdmittedModerationObservationV1] {
        &self.moderation
    }

    pub fn bridge_origin(&self) -> Option<&AdmittedBridgeOriginV1> {
        self.bridge_origin.as_ref()
    }

    pub fn projection_class(&self) -> &ProjectionClassV1 {
        &self.projection_class
    }

    pub fn same_provider_event_as(&self, other: &Self) -> bool {
        self.provider == other.provider
            && self.provider_profile == other.provider_profile
            && self.event_id == other.event_id
    }

    pub fn same_bridge_origin_as(&self, other: &Self) -> bool {
        self.bridge_origin.is_some() && self.bridge_origin == other.bridge_origin
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ProjectionErrorV1 {
    ProviderProfileMismatch {
        provider: ProviderFamilyV1,
        expected: &'static str,
    },
    ProviderScopeMismatch,
    ProviderDeliveryMismatch,
    ProviderActorMismatch,
    ProviderEventKindMismatch,
    MalformedIdentityEvidence(&'static str),
    MalformedCommitment(&'static str),
    ResourceBoundExceeded(&'static str),
    IncompatibleMatrixEventShape,
}

impl fmt::Display for ProjectionErrorV1 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::ProviderProfileMismatch { provider, expected } => {
                write!(f, "provider profile mismatch for {provider:?}; expected {expected}")
            }
            Self::ProviderScopeMismatch => write!(f, "provider scope mismatch"),
            Self::ProviderDeliveryMismatch => write!(f, "provider delivery mismatch"),
            Self::ProviderActorMismatch => write!(f, "provider actor mismatch"),
            Self::ProviderEventKindMismatch => write!(f, "provider event-kind mismatch"),
            Self::MalformedIdentityEvidence(field) => {
                write!(f, "malformed provider identity evidence: {field}")
            }
            Self::MalformedCommitment(field) => write!(f, "malformed commitment: {field}"),
            Self::ResourceBoundExceeded(field) => write!(f, "resource bound exceeded: {field}"),
            Self::IncompatibleMatrixEventShape => write!(f, "incompatible Matrix event shape"),
        }
    }
}

impl std::error::Error for ProjectionErrorV1 {}

pub fn admit_external_bridge_observation_v1(
    raw: RawExternalBridgeObservationV1,
) -> Result<AdmittedExternalBridgeObservationV1, ProjectionErrorV1> {
    if raw.provider_profile != raw.provider.expected_profile() {
        return Err(ProjectionErrorV1::ProviderProfileMismatch {
            provider: raw.provider,
            expected: raw.provider.expected_profile(),
        });
    }
    validate_bounded_identity("provider_profile", &raw.provider_profile, MAX_PROFILE_BYTES)?;
    validate_bounded_identity("event_id", &raw.event_id, MAX_PROVIDER_ID_BYTES)?;

    if raw.relations.len() > MAX_RELATIONS {
        return Err(ProjectionErrorV1::ResourceBoundExceeded("relations"));
    }
    if raw.attachments.len() > MAX_ATTACHMENTS {
        return Err(ProjectionErrorV1::ResourceBoundExceeded("attachments"));
    }
    if raw.moderation.len() > MAX_MODERATION_OBSERVATIONS {
        return Err(ProjectionErrorV1::ResourceBoundExceeded("moderation"));
    }

    let scope = admit_scope(raw.provider, raw.scope)?;
    let delivery = raw
        .delivery
        .map(|value| admit_delivery(raw.provider, value))
        .transpose()?;
    let (actor, unknown_actor_kind) = match raw.actor {
        Some(value) => {
            let (actor, unknown) = admit_actor(raw.provider, value)?;
            (Some(actor), unknown)
        }
        None => (None, false),
    };
    let (event_kind, unknown_event_kind) = admit_event_kind(raw.provider, raw.event_kind)?;
    let (content, content_withheld) = admit_content(raw.content)?;

    let mut unknown_relation = false;
    let mut relations = Vec::with_capacity(raw.relations.len());
    for relation in raw.relations {
        let (admitted, unknown) = admit_relation(relation)?;
        unknown_relation |= unknown;
        relations.push(admitted);
    }

    let mut attachments = Vec::with_capacity(raw.attachments.len());
    for attachment in raw.attachments {
        attachments.push(admit_attachment(attachment)?);
    }

    let mut unknown_moderation = false;
    let mut moderation = Vec::with_capacity(raw.moderation.len());
    for observation in raw.moderation {
        let (admitted, unknown) = admit_moderation(observation)?;
        unknown_moderation |= unknown;
        moderation.push(admitted);
    }

    let bridge_origin = raw.bridge_origin.map(admit_bridge_origin).transpose()?;

    let mut losses = Vec::new();
    if unknown_relation {
        losses.push(LossMarkerV1::DroppedUnsupportedRelation);
    }
    if raw.audience_semantics_not_representable {
        losses.push(LossMarkerV1::AudienceSemanticsNotRepresentable);
    }
    if !attachments.is_empty() {
        losses.push(LossMarkerV1::AttachmentNotFetched);
    }
    if content_withheld {
        losses.push(LossMarkerV1::ContentUnavailableUnderProviderAccessProfile);
    }
    if !moderation.is_empty() {
        losses.push(LossMarkerV1::ProviderModerationStateOnly);
    }
    if unknown_actor_kind || unknown_event_kind || unknown_moderation {
        losses.push(LossMarkerV1::UnknownProviderExtension);
    }
    losses.sort_unstable();
    losses.dedup();

    let projection_class = if losses.is_empty() {
        ProjectionClassV1::LosslessWithinProfile
    } else {
        ProjectionClassV1::LossyWithinProfile(losses)
    };

    Ok(AdmittedExternalBridgeObservationV1 {
        provider: raw.provider,
        provider_profile: raw.provider_profile,
        event_id: raw.event_id,
        delivery,
        scope,
        actor,
        event_kind,
        content,
        relations,
        attachments,
        moderation,
        bridge_origin,
        projection_class,
    })
}

fn admit_scope(
    provider: ProviderFamilyV1,
    raw: RawProviderScopeV1,
) -> Result<AdmittedProviderScopeV1, ProjectionErrorV1> {
    match (provider, raw) {
        (ProviderFamilyV1::Matrix, RawProviderScopeV1::Matrix { room_id }) => {
            if let Some(room_id) = &room_id {
                validate_bounded_identity("matrix.room_id", room_id, MAX_PROVIDER_ID_BYTES)?;
            }
            Ok(AdmittedProviderScopeV1::Matrix(MatrixScopeV1 { room_id }))
        }
        (
            ProviderFamilyV1::Discord,
            RawProviderScopeV1::Discord {
                guild_id,
                channel_id,
                thread_id,
            },
        ) => {
            if let Some(guild_id) = &guild_id {
                validate_bounded_identity("discord.guild_id", guild_id, MAX_PROVIDER_ID_BYTES)?;
            }
            validate_bounded_identity("discord.channel_id", &channel_id, MAX_PROVIDER_ID_BYTES)?;
            if let Some(thread_id) = &thread_id {
                validate_bounded_identity("discord.thread_id", thread_id, MAX_PROVIDER_ID_BYTES)?;
            }
            Ok(AdmittedProviderScopeV1::Discord(DiscordScopeV1 {
                guild_id,
                channel_id,
                thread_id,
            }))
        }
        _ => Err(ProjectionErrorV1::ProviderScopeMismatch),
    }
}

fn admit_delivery(
    provider: ProviderFamilyV1,
    raw: RawDeliveryCoordinateV1,
) -> Result<AdmittedDeliveryCoordinateV1, ProjectionErrorV1> {
    match (provider, raw) {
        (
            ProviderFamilyV1::Matrix,
            RawDeliveryCoordinateV1::Matrix {
                transaction_id,
                transaction_commitment,
            },
        ) => {
            validate_bounded_identity(
                "matrix.transaction_id",
                &transaction_id,
                MAX_PROVIDER_ID_BYTES,
            )?;
            validate_sha256_commitment(
                "matrix.transaction_commitment",
                &transaction_commitment,
            )?;
            Ok(AdmittedDeliveryCoordinateV1::Matrix(
                MatrixDeliveryCoordinateV1 {
                    transaction_id,
                    transaction_commitment,
                },
            ))
        }
        (
            ProviderFamilyV1::Discord,
            RawDeliveryCoordinateV1::Discord {
                gateway_session_id,
                gateway_sequence,
                webhook_delivery_id,
            },
        ) => {
            if gateway_session_id.is_none()
                && gateway_sequence.is_none()
                && webhook_delivery_id.is_none()
            {
                return Err(ProjectionErrorV1::MalformedIdentityEvidence(
                    "discord.delivery",
                ));
            }
            if let Some(value) = &gateway_session_id {
                validate_bounded_identity(
                    "discord.gateway_session_id",
                    value,
                    MAX_PROVIDER_ID_BYTES,
                )?;
            }
            if let Some(value) = &webhook_delivery_id {
                validate_bounded_identity(
                    "discord.webhook_delivery_id",
                    value,
                    MAX_PROVIDER_ID_BYTES,
                )?;
            }
            Ok(AdmittedDeliveryCoordinateV1::Discord(
                DiscordDeliveryCoordinateV1 {
                    gateway_session_id,
                    gateway_sequence,
                    webhook_delivery_id,
                },
            ))
        }
        _ => Err(ProjectionErrorV1::ProviderDeliveryMismatch),
    }
}

fn admit_actor(
    provider: ProviderFamilyV1,
    raw: RawActorObservationV1,
) -> Result<(AdmittedActorObservationV1, bool), ProjectionErrorV1> {
    validate_bounded_identity("actor_id", &raw.actor_id, MAX_PROVIDER_ID_BYTES)?;
    if !actor_kind_matches_provider(provider, &raw.kind) {
        return Err(ProjectionErrorV1::ProviderActorMismatch);
    }
    let unknown = if let RawActorKindV1::UnknownProviderKind(value) = &raw.kind {
        validate_bounded_identity("actor_kind", value, MAX_EVENT_KIND_BYTES)?;
        true
    } else {
        false
    };
    Ok((
        AdmittedActorObservationV1 {
            actor_id: raw.actor_id,
            kind: raw.kind,
        },
        unknown,
    ))
}

fn actor_kind_matches_provider(provider: ProviderFamilyV1, kind: &RawActorKindV1) -> bool {
    matches!(
        (provider, kind),
        (
            ProviderFamilyV1::Matrix,
            RawActorKindV1::MatrixUser
                | RawActorKindV1::MatrixApplicationService
                | RawActorKindV1::UnknownProviderKind(_)
        ) | (
            ProviderFamilyV1::Discord,
            RawActorKindV1::DiscordUser
                | RawActorKindV1::DiscordBotApplication
                | RawActorKindV1::DiscordWebhook
                | RawActorKindV1::DiscordApplication
                | RawActorKindV1::UnknownProviderKind(_)
        )
    )
}

fn admit_event_kind(
    provider: ProviderFamilyV1,
    raw: RawProviderEventKindV1,
) -> Result<(AdmittedProviderEventKindV1, bool), ProjectionErrorV1> {
    match (provider, raw) {
        (
            ProviderFamilyV1::Matrix,
            RawProviderEventKindV1::Matrix {
                event_type,
                state_key,
                ephemeral,
            },
        ) => {
            validate_bounded_identity("matrix.event_type", &event_type, MAX_EVENT_KIND_BYTES)?;
            if let Some(value) = &state_key {
                validate_bounded_text("matrix.state_key", value, MAX_PROVIDER_ID_BYTES)?;
            }
            if ephemeral && state_key.is_some() {
                return Err(ProjectionErrorV1::IncompatibleMatrixEventShape);
            }
            let class = if ephemeral {
                MatrixEventClassV1::EphemeralEvent
            } else if state_key.is_some() {
                MatrixEventClassV1::StateEvent
            } else {
                MatrixEventClassV1::PersistentEvent
            };
            let unknown = !is_known_matrix_event_type(&event_type);
            Ok((
                AdmittedProviderEventKindV1::Matrix {
                    event_type,
                    state_key,
                    class,
                },
                unknown,
            ))
        }
        (ProviderFamilyV1::Discord, RawProviderEventKindV1::Discord { kind }) => {
            let unknown = if let RawDiscordEventKindV1::UnknownProviderKind(value) = &kind {
                validate_bounded_identity("discord.event_kind", value, MAX_EVENT_KIND_BYTES)?;
                true
            } else {
                false
            };
            Ok((AdmittedProviderEventKindV1::Discord { kind }, unknown))
        }
        _ => Err(ProjectionErrorV1::ProviderEventKindMismatch),
    }
}

fn is_known_matrix_event_type(event_type: &str) -> bool {
    matches!(
        event_type,
        "m.room.message"
            | "m.room.topic"
            | "m.room.name"
            | "m.room.member"
            | "m.room.power_levels"
            | "m.room.redaction"
            | "m.reaction"
            | "m.typing"
            | "m.receipt"
            | "m.presence"
    )
}

fn admit_content(
    raw: RawContentObservationV1,
) -> Result<(AdmittedContentObservationV1, bool), ProjectionErrorV1> {
    match raw {
        RawContentObservationV1::AvailableUtf8(value) => {
            validate_bounded_text("content", &value, MAX_CONTENT_BYTES)?;
            Ok((AdmittedContentObservationV1::AvailableUtf8(value), false))
        }
        RawContentObservationV1::UnavailableUnderProviderAccessProfile => Ok((
            AdmittedContentObservationV1::UnavailableUnderProviderAccessProfile,
            true,
        )),
        RawContentObservationV1::NotApplicable => {
            Ok((AdmittedContentObservationV1::NotApplicable, false))
        }
    }
}

fn admit_relation(
    raw: RawRelationObservationV1,
) -> Result<(AdmittedRelationObservationV1, bool), ProjectionErrorV1> {
    validate_bounded_identity(
        "relation.target_event_id",
        &raw.target_event_id,
        MAX_PROVIDER_ID_BYTES,
    )?;
    if let Some(value) = &raw.provider_relation_ref {
        validate_bounded_identity(
            "relation.provider_relation_ref",
            value,
            MAX_METADATA_VALUE_BYTES,
        )?;
    }
    let unknown = if let RawRelationKindV1::UnknownProviderRelation(value) = &raw.kind {
        validate_bounded_identity("relation.kind", value, MAX_EVENT_KIND_BYTES)?;
        true
    } else {
        false
    };
    Ok((
        AdmittedRelationObservationV1 {
            kind: raw.kind,
            target_event_id: raw.target_event_id,
            provider_relation_ref: raw.provider_relation_ref,
        },
        unknown,
    ))
}

fn admit_attachment(
    raw: RawAttachmentReferenceV1,
) -> Result<AdmittedAttachmentReferenceV1, ProjectionErrorV1> {
    validate_bounded_identity(
        "attachment.provider_attachment_id",
        &raw.provider_attachment_id,
        MAX_PROVIDER_ID_BYTES,
    )?;
    validate_bounded_text("attachment.locator", &raw.locator, MAX_LOCATOR_BYTES)?;
    if let Some(value) = &raw.media_type_hint {
        validate_bounded_text("attachment.media_type_hint", value, MAX_METADATA_VALUE_BYTES)?;
    }
    Ok(AdmittedAttachmentReferenceV1 {
        provider_attachment_id: raw.provider_attachment_id,
        locator: raw.locator,
        media_type_hint: raw.media_type_hint,
    })
}

fn admit_moderation(
    raw: RawModerationObservationV1,
) -> Result<(AdmittedModerationObservationV1, bool), ProjectionErrorV1> {
    validate_bounded_identity(
        "moderation.subject_provider_id",
        &raw.subject_provider_id,
        MAX_PROVIDER_ID_BYTES,
    )?;
    validate_bounded_text("moderation.value", &raw.value, MAX_METADATA_VALUE_BYTES)?;
    let unknown = if let RawModerationKindV1::UnknownProviderModerationKind(value) = &raw.kind {
        validate_bounded_identity("moderation.kind", value, MAX_EVENT_KIND_BYTES)?;
        true
    } else {
        false
    };
    Ok((
        AdmittedModerationObservationV1 {
            kind: raw.kind,
            subject_provider_id: raw.subject_provider_id,
            value: raw.value,
        },
        unknown,
    ))
}

fn admit_bridge_origin(
    raw: RawBridgeOriginV1,
) -> Result<AdmittedBridgeOriginV1, ProjectionErrorV1> {
    if raw.source_provider_profile != raw.source_provider.expected_profile() {
        return Err(ProjectionErrorV1::ProviderProfileMismatch {
            provider: raw.source_provider,
            expected: raw.source_provider.expected_profile(),
        });
    }
    validate_bounded_identity(
        "bridge_origin.source_event_id",
        &raw.source_event_id,
        MAX_PROVIDER_ID_BYTES,
    )?;
    validate_sha256_commitment(
        "bridge_origin.source_semantic_commitment",
        &raw.source_semantic_commitment,
    )?;
    Ok(AdmittedBridgeOriginV1 {
        source_provider: raw.source_provider,
        source_provider_profile: raw.source_provider_profile,
        source_event_id: raw.source_event_id,
        source_semantic_commitment: raw.source_semantic_commitment,
    })
}

fn validate_bounded_identity(
    field: &'static str,
    value: &str,
    max_bytes: usize,
) -> Result<(), ProjectionErrorV1> {
    if value.is_empty() {
        return Err(ProjectionErrorV1::MalformedIdentityEvidence(field));
    }
    if value.len() > max_bytes {
        return Err(ProjectionErrorV1::ResourceBoundExceeded(field));
    }
    if value.chars().any(char::is_control) {
        return Err(ProjectionErrorV1::MalformedIdentityEvidence(field));
    }
    Ok(())
}

fn validate_bounded_text(
    field: &'static str,
    value: &str,
    max_bytes: usize,
) -> Result<(), ProjectionErrorV1> {
    if value.len() > max_bytes {
        return Err(ProjectionErrorV1::ResourceBoundExceeded(field));
    }
    Ok(())
}

fn validate_sha256_commitment(
    field: &'static str,
    value: &str,
) -> Result<(), ProjectionErrorV1> {
    let Some(hex) = value.strip_prefix("sha256:") else {
        return Err(ProjectionErrorV1::MalformedCommitment(field));
    };
    if hex.len() != 64
        || !hex
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
    {
        return Err(ProjectionErrorV1::MalformedCommitment(field));
    }
    Ok(())
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MatrixTransactionReplayV1 {
    IdempotentDeliveryObservation,
    Conflict,
    DistinctTransactionCoordinates,
}

pub fn classify_matrix_transaction_replay_v1(
    left: &AdmittedExternalBridgeObservationV1,
    right: &AdmittedExternalBridgeObservationV1,
) -> Result<MatrixTransactionReplayV1, ProjectionErrorV1> {
    let (
        Some(AdmittedDeliveryCoordinateV1::Matrix(left_delivery)),
        Some(AdmittedDeliveryCoordinateV1::Matrix(right_delivery)),
    ) = (left.delivery(), right.delivery())
    else {
        return Err(ProjectionErrorV1::ProviderDeliveryMismatch);
    };

    if left_delivery.transaction_id != right_delivery.transaction_id {
        return Ok(MatrixTransactionReplayV1::DistinctTransactionCoordinates);
    }
    if left_delivery.transaction_commitment == right_delivery.transaction_commitment {
        Ok(MatrixTransactionReplayV1::IdempotentDeliveryObservation)
    } else {
        Ok(MatrixTransactionReplayV1::Conflict)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn matrix_base() -> RawExternalBridgeObservationV1 {
        RawExternalBridgeObservationV1 {
            provider: ProviderFamilyV1::Matrix,
            provider_profile: MATRIX_PROFILE_V0_1.to_owned(),
            event_id: "$event:example.org".to_owned(),
            delivery: Some(RawDeliveryCoordinateV1::Matrix {
                transaction_id: "txn-1".to_owned(),
                transaction_commitment: format!("sha256:{}", "a".repeat(64)),
            }),
            scope: RawProviderScopeV1::Matrix {
                room_id: Some("!room:example.org".to_owned()),
            },
            actor: Some(RawActorObservationV1 {
                actor_id: "@alice:example.org".to_owned(),
                kind: RawActorKindV1::MatrixUser,
            }),
            event_kind: RawProviderEventKindV1::Matrix {
                event_type: "m.room.message".to_owned(),
                state_key: None,
                ephemeral: false,
            },
            content: RawContentObservationV1::AvailableUtf8("hello".to_owned()),
            relations: vec![],
            attachments: vec![],
            moderation: vec![],
            bridge_origin: None,
            audience_semantics_not_representable: false,
        }
    }

    fn discord_base() -> RawExternalBridgeObservationV1 {
        RawExternalBridgeObservationV1 {
            provider: ProviderFamilyV1::Discord,
            provider_profile: DISCORD_PROFILE_V0_1.to_owned(),
            event_id: "1001".to_owned(),
            delivery: Some(RawDeliveryCoordinateV1::Discord {
                gateway_session_id: Some("session-a".to_owned()),
                gateway_sequence: Some(42),
                webhook_delivery_id: None,
            }),
            scope: RawProviderScopeV1::Discord {
                guild_id: Some("guild-1".to_owned()),
                channel_id: "channel-1".to_owned(),
                thread_id: Some("thread-1".to_owned()),
            },
            actor: Some(RawActorObservationV1 {
                actor_id: "bot-1".to_owned(),
                kind: RawActorKindV1::DiscordBotApplication,
            }),
            event_kind: RawProviderEventKindV1::Discord {
                kind: RawDiscordEventKindV1::MessageCreate,
            },
            content: RawContentObservationV1::AvailableUtf8("hello".to_owned()),
            relations: vec![],
            attachments: vec![],
            moderation: vec![],
            bridge_origin: None,
            audience_semantics_not_representable: false,
        }
    }

    #[test]
    fn frozen_corpus_shape_stays_bound() {
        let fixture: serde_json::Value = serde_json::from_str(include_str!(
            "../../../docs/bridges/fixtures/PULSE_BRIDGE_001_V0_1.json"
        ))
        .unwrap();
        assert_eq!(
            fixture["profile"],
            serde_json::Value::String(BRIDGE_CORPUS_PROFILE_V0_1.to_owned())
        );
        assert_eq!(
            fixture["provider_profiles"]["matrix"],
            serde_json::Value::String(MATRIX_PROFILE_V0_1.to_owned())
        );
        assert_eq!(
            fixture["provider_profiles"]["discord"],
            serde_json::Value::String(DISCORD_PROFILE_V0_1.to_owned())
        );
        assert_eq!(fixture["cases"].as_array().unwrap().len(), 20);
    }

    #[test]
    fn profile_substitution_fails_closed() {
        let mut raw = discord_base();
        raw.provider_profile = "unreviewed-profile".to_owned();
        assert!(matches!(
            admit_external_bridge_observation_v1(raw),
            Err(ProjectionErrorV1::ProviderProfileMismatch { .. })
        ));
    }

    #[test]
    fn discord_scope_preserves_guild_channel_and_thread() {
        let admitted = admit_external_bridge_observation_v1(discord_base()).unwrap();
        let AdmittedProviderScopeV1::Discord(scope) = admitted.scope() else {
            panic!("expected discord scope");
        };
        assert_eq!(scope.guild_id(), Some("guild-1"));
        assert_eq!(scope.channel_id(), "channel-1");
        assert_eq!(scope.thread_id(), Some("thread-1"));
    }

    #[test]
    fn webhook_authorship_stays_webhook_authorship() {
        let mut raw = discord_base();
        raw.actor = Some(RawActorObservationV1 {
            actor_id: "webhook-7".to_owned(),
            kind: RawActorKindV1::DiscordWebhook,
        });
        let admitted = admit_external_bridge_observation_v1(raw).unwrap();
        assert!(matches!(
            admitted.actor().unwrap().kind(),
            RawActorKindV1::DiscordWebhook
        ));
    }

    #[test]
    fn provider_withheld_content_is_not_empty_content() {
        let mut raw = discord_base();
        raw.content = RawContentObservationV1::UnavailableUnderProviderAccessProfile;
        let admitted = admit_external_bridge_observation_v1(raw).unwrap();
        assert!(matches!(
            admitted.content(),
            AdmittedContentObservationV1::UnavailableUnderProviderAccessProfile
        ));
        assert_eq!(
            admitted.projection_class(),
            &ProjectionClassV1::LossyWithinProfile(vec![
                LossMarkerV1::ContentUnavailableUnderProviderAccessProfile
            ])
        );
    }

    #[test]
    fn state_event_classification_uses_state_key_presence() {
        let mut raw = matrix_base();
        raw.event_kind = RawProviderEventKindV1::Matrix {
            event_type: "m.room.topic".to_owned(),
            state_key: Some(String::new()),
            ephemeral: false,
        };
        let admitted = admit_external_bridge_observation_v1(raw).unwrap();
        assert!(matches!(
            admitted.event_kind(),
            AdmittedProviderEventKindV1::Matrix {
                class: MatrixEventClassV1::StateEvent,
                ..
            }
        ));
    }

    #[test]
    fn attachment_and_moderation_refs_are_preserved_with_explicit_losses() {
        let mut raw = discord_base();
        raw.attachments.push(RawAttachmentReferenceV1 {
            provider_attachment_id: "attachment-1".to_owned(),
            locator: "https://cdn.example.invalid/a".to_owned(),
            media_type_hint: Some("image/png".to_owned()),
        });
        raw.moderation.push(RawModerationObservationV1 {
            kind: RawModerationKindV1::DiscordRole,
            subject_provider_id: "user-1".to_owned(),
            value: "role-3001".to_owned(),
        });
        let admitted = admit_external_bridge_observation_v1(raw).unwrap();
        assert_eq!(
            admitted.attachments()[0].provider_attachment_id(),
            "attachment-1"
        );
        assert_eq!(admitted.moderation()[0].value(), "role-3001");
        assert_eq!(
            admitted.projection_class(),
            &ProjectionClassV1::LossyWithinProfile(vec![
                LossMarkerV1::AttachmentNotFetched,
                LossMarkerV1::ProviderModerationStateOnly,
            ])
        );
    }

    #[test]
    fn unknown_relation_is_retained_and_marked_lossy() {
        let mut raw = discord_base();
        raw.relations.push(RawRelationObservationV1 {
            kind: RawRelationKindV1::UnknownProviderRelation("future_relation_v9".to_owned()),
            target_event_id: "1000".to_owned(),
            provider_relation_ref: Some("opaque-7".to_owned()),
        });
        let admitted = admit_external_bridge_observation_v1(raw).unwrap();
        assert_eq!(admitted.relations().len(), 1);
        assert_eq!(admitted.relations()[0].target_event_id(), "1000");
        assert_eq!(
            admitted.projection_class(),
            &ProjectionClassV1::LossyWithinProfile(vec![
                LossMarkerV1::DroppedUnsupportedRelation
            ])
        );
    }

    #[test]
    fn matrix_retry_semantics_separate_delivery_from_event_identity() {
        let first = admit_external_bridge_observation_v1(matrix_base()).unwrap();
        let same = admit_external_bridge_observation_v1(matrix_base()).unwrap();
        assert!(first.same_provider_event_as(&same));
        assert_eq!(
            classify_matrix_transaction_replay_v1(&first, &same).unwrap(),
            MatrixTransactionReplayV1::IdempotentDeliveryObservation
        );

        let mut conflicting = matrix_base();
        conflicting.delivery = Some(RawDeliveryCoordinateV1::Matrix {
            transaction_id: "txn-1".to_owned(),
            transaction_commitment: format!("sha256:{}", "b".repeat(64)),
        });
        let conflicting = admit_external_bridge_observation_v1(conflicting).unwrap();
        assert_eq!(
            classify_matrix_transaction_replay_v1(&first, &conflicting).unwrap(),
            MatrixTransactionReplayV1::Conflict
        );
    }

    #[test]
    fn same_text_does_not_deduplicate_distinct_provider_events() {
        let matrix = admit_external_bridge_observation_v1(matrix_base()).unwrap();
        let discord = admit_external_bridge_observation_v1(discord_base()).unwrap();
        assert!(!matrix.same_provider_event_as(&discord));

        let mut another_matrix = matrix_base();
        another_matrix.event_id = "$other:example.org".to_owned();
        let another_matrix = admit_external_bridge_observation_v1(another_matrix).unwrap();
        assert!(!matrix.same_provider_event_as(&another_matrix));
    }

    #[test]
    fn bridge_origin_loop_uses_identity_and_commitment_not_text() {
        let origin = RawBridgeOriginV1 {
            source_provider: ProviderFamilyV1::Matrix,
            source_provider_profile: MATRIX_PROFILE_V0_1.to_owned(),
            source_event_id: "$loop:example.org".to_owned(),
            source_semantic_commitment: format!("sha256:{}", "c".repeat(64)),
        };
        let mut left = discord_base();
        left.bridge_origin = Some(origin.clone());
        let mut right = discord_base();
        right.event_id = "different-return-message".to_owned();
        right.bridge_origin = Some(origin);

        let left = admit_external_bridge_observation_v1(left).unwrap();
        let right = admit_external_bridge_observation_v1(right).unwrap();
        assert!(left.same_bridge_origin_as(&right));
        assert!(!left.same_provider_event_as(&right));
    }

    #[test]
    fn unknown_matrix_event_type_is_preserved_and_marked_unknown() {
        let mut raw = matrix_base();
        raw.event_kind = RawProviderEventKindV1::Matrix {
            event_type: "org.example.future.unknown".to_owned(),
            state_key: None,
            ephemeral: false,
        };
        let admitted = admit_external_bridge_observation_v1(raw).unwrap();
        assert!(matches!(
            admitted.event_kind(),
            AdmittedProviderEventKindV1::Matrix { event_type, .. }
                if event_type == "org.example.future.unknown"
        ));
        assert_eq!(
            admitted.projection_class(),
            &ProjectionClassV1::LossyWithinProfile(vec![
                LossMarkerV1::UnknownProviderExtension
            ])
        );
    }

    #[test]
    fn unknown_actor_kind_is_not_silently_lossless() {
        let mut raw = discord_base();
        raw.actor = Some(RawActorObservationV1 {
            actor_id: "future-actor".to_owned(),
            kind: RawActorKindV1::UnknownProviderKind("future_actor_v9".to_owned()),
        });
        let admitted = admit_external_bridge_observation_v1(raw).unwrap();
        assert_eq!(
            admitted.projection_class(),
            &ProjectionClassV1::LossyWithinProfile(vec![
                LossMarkerV1::UnknownProviderExtension
            ])
        );
    }

    #[test]
    fn matrix_ephemeral_state_shape_conflict_fails_closed() {
        let mut raw = matrix_base();
        raw.event_kind = RawProviderEventKindV1::Matrix {
            event_type: "m.typing".to_owned(),
            state_key: Some(String::new()),
            ephemeral: true,
        };
        assert_eq!(
            admit_external_bridge_observation_v1(raw),
            Err(ProjectionErrorV1::IncompatibleMatrixEventShape)
        );
    }
}
