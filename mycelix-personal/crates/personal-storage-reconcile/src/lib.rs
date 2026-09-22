#![forbid(unsafe_code)]

use serde::{Deserialize, Serialize};
use std::cmp::Ordering;
use std::collections::BTreeMap;
use thiserror::Error;

const MAX_PROFILE_LEN: usize = 128;
const MAX_SCOPE_LEN: usize = 512;
const MAX_LOCATOR_LEN: usize = 4096;
const MAX_METADATA_LEN: usize = 1024;
const MAX_FRONTIER_LEN: usize = 512;
const MAX_LINEAGE_REF_LEN: usize = 512;
const MAX_DIGEST_HEX_LEN: usize = 256;

#[derive(Debug, Error, Clone, PartialEq, Eq)]
pub enum ReconcileError {
    #[error("field `{field}` is empty")]
    EmptyField { field: &'static str },
    #[error("field `{field}` exceeds maximum length {max}")]
    FieldTooLong { field: &'static str, max: usize },
    #[error("field `{field}` contains control characters")]
    ControlCharacter { field: &'static str },
    #[error("field `{field}` is not in canonical form")]
    NonCanonicalText { field: &'static str },
    #[error("content digest must be canonical lowercase hexadecimal")]
    NonCanonicalDigest,
    #[error("content digest length is outside the supported bounded profile")]
    InvalidDigestLength,
    #[error("conflicting observations share one exact locator/frontier identity")]
    ConflictingDuplicateObservation,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ContentCommitmentInputV1 {
    pub profile: String,
    pub digest_hex: String,
}

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Serialize)]
pub struct ContentCommitmentV1 {
    profile: String,
    digest_hex: String,
}

impl ContentCommitmentV1 {
    pub fn profile(&self) -> &str {
        &self.profile
    }

    pub fn digest_hex(&self) -> &str {
        &self.digest_hex
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct SemanticLineageInputV1 {
    pub lineage_ref: String,
    pub generation: u64,
}

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Serialize)]
pub struct SemanticLineageV1 {
    lineage_ref: String,
    generation: u64,
}

impl SemanticLineageV1 {
    pub fn lineage_ref(&self) -> &str {
        &self.lineage_ref
    }

    pub fn generation(&self) -> u64 {
        self.generation
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ReplicaLocatorInputV1 {
    pub provider_family: String,
    pub provider_profile: String,
    pub account_scope: String,
    pub locator: String,
}

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Serialize)]
pub struct ReplicaLocatorV1 {
    provider_family: String,
    provider_profile: String,
    account_scope: String,
    locator: String,
}

impl ReplicaLocatorV1 {
    pub fn provider_family(&self) -> &str {
        &self.provider_family
    }

    pub fn provider_profile(&self) -> &str {
        &self.provider_profile
    }

    pub fn account_scope(&self) -> &str {
        &self.account_scope
    }

    pub fn locator(&self) -> &str {
        &self.locator
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum ReplicaPresenceInputV1 {
    Present {
        content: ContentCommitmentInputV1,
        lineage: Option<SemanticLineageInputV1>,
    },
    Tombstone {
        lineage: Option<SemanticLineageInputV1>,
    },
    Unknown,
}

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Serialize)]
pub enum ReplicaPresenceV1 {
    Present {
        content: ContentCommitmentV1,
        lineage: Option<SemanticLineageV1>,
    },
    Tombstone {
        lineage: Option<SemanticLineageV1>,
    },
    Unknown,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum ReplicaCurrentnessInputV1 {
    Complete { frontier: String },
    Partial { frontier: Option<String> },
    Unknown,
}

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Serialize)]
pub enum ReplicaCurrentnessV1 {
    Complete { frontier: String },
    Partial { frontier: Option<String> },
    Unknown,
}

impl ReplicaCurrentnessV1 {
    pub fn is_complete(&self) -> bool {
        matches!(self, Self::Complete { .. })
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ReplicaObservationInputV1 {
    pub locator: ReplicaLocatorInputV1,
    pub presence: ReplicaPresenceInputV1,
    pub currentness: ReplicaCurrentnessInputV1,
    pub provider_etag: Option<String>,
    pub provider_version: Option<String>,
}

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Serialize)]
pub struct AdmittedReplicaObservationV1 {
    locator: ReplicaLocatorV1,
    presence: ReplicaPresenceV1,
    currentness: ReplicaCurrentnessV1,
    provider_etag: Option<String>,
    provider_version: Option<String>,
}

impl AdmittedReplicaObservationV1 {
    pub fn locator(&self) -> &ReplicaLocatorV1 {
        &self.locator
    }

    pub fn presence(&self) -> &ReplicaPresenceV1 {
        &self.presence
    }

    pub fn currentness(&self) -> &ReplicaCurrentnessV1 {
        &self.currentness
    }

    pub fn provider_etag(&self) -> Option<&str> {
        self.provider_etag.as_deref()
    }

    pub fn provider_version(&self) -> Option<&str> {
        self.provider_version.as_deref()
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
pub enum ReplicaRelationV1 {
    InSync,
    LeftAhead,
    RightAhead,
    Diverged,
    LeftTombstoned,
    RightTombstoned,
    BothTombstoned,
    UnknownCurrentness,
    Conflict,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub struct ReplicaReconciliationV1 {
    relation: ReplicaRelationV1,
    content_equivalent: bool,
    same_lineage: Option<bool>,
}

impl ReplicaReconciliationV1 {
    pub fn relation(&self) -> ReplicaRelationV1 {
        self.relation
    }

    pub fn content_equivalent(&self) -> bool {
        self.content_equivalent
    }

    pub fn same_lineage(&self) -> Option<bool> {
        self.same_lineage
    }
}

pub fn admit_observation(
    input: ReplicaObservationInputV1,
) -> Result<AdmittedReplicaObservationV1, ReconcileError> {
    let locator = ReplicaLocatorV1 {
        provider_family: canonical_text(
            "provider_family",
            input.locator.provider_family,
            MAX_PROFILE_LEN,
        )?,
        provider_profile: canonical_text(
            "provider_profile",
            input.locator.provider_profile,
            MAX_PROFILE_LEN,
        )?,
        account_scope: canonical_text(
            "account_scope",
            input.locator.account_scope,
            MAX_SCOPE_LEN,
        )?,
        locator: canonical_text("locator", input.locator.locator, MAX_LOCATOR_LEN)?,
    };

    let presence = match input.presence {
        ReplicaPresenceInputV1::Present { content, lineage } => ReplicaPresenceV1::Present {
            content: admit_content_commitment(content)?,
            lineage: lineage.map(admit_lineage).transpose()?,
        },
        ReplicaPresenceInputV1::Tombstone { lineage } => ReplicaPresenceV1::Tombstone {
            lineage: lineage.map(admit_lineage).transpose()?,
        },
        ReplicaPresenceInputV1::Unknown => ReplicaPresenceV1::Unknown,
    };

    let currentness = match input.currentness {
        ReplicaCurrentnessInputV1::Complete { frontier } => ReplicaCurrentnessV1::Complete {
            frontier: canonical_text("frontier", frontier, MAX_FRONTIER_LEN)?,
        },
        ReplicaCurrentnessInputV1::Partial { frontier } => ReplicaCurrentnessV1::Partial {
            frontier: frontier
                .map(|value| canonical_text("frontier", value, MAX_FRONTIER_LEN))
                .transpose()?,
        },
        ReplicaCurrentnessInputV1::Unknown => ReplicaCurrentnessV1::Unknown,
    };

    Ok(AdmittedReplicaObservationV1 {
        locator,
        presence,
        currentness,
        provider_etag: input
            .provider_etag
            .map(|value| canonical_text("provider_etag", value, MAX_METADATA_LEN))
            .transpose()?,
        provider_version: input
            .provider_version
            .map(|value| canonical_text("provider_version", value, MAX_METADATA_LEN))
            .transpose()?,
    })
}

pub fn canonicalize_observations(
    inputs: impl IntoIterator<Item = ReplicaObservationInputV1>,
) -> Result<Vec<AdmittedReplicaObservationV1>, ReconcileError> {
    let mut by_identity: BTreeMap<
        (ReplicaLocatorV1, ReplicaCurrentnessV1),
        AdmittedReplicaObservationV1,
    > = BTreeMap::new();

    for input in inputs {
        let admitted = admit_observation(input)?;
        let key = (admitted.locator.clone(), admitted.currentness.clone());
        if let Some(existing) = by_identity.get(&key) {
            if existing != &admitted {
                return Err(ReconcileError::ConflictingDuplicateObservation);
            }
            continue;
        }
        by_identity.insert(key, admitted);
    }

    Ok(by_identity.into_values().collect())
}

pub fn reconcile_pair(
    left: &AdmittedReplicaObservationV1,
    right: &AdmittedReplicaObservationV1,
) -> ReplicaReconciliationV1 {
    if left.locator == right.locator && left.currentness == right.currentness && left != right {
        return result(ReplicaRelationV1::Conflict, false, lineage_equivalence(left, right));
    }

    if !left.currentness.is_complete() || !right.currentness.is_complete() {
        return result(
            ReplicaRelationV1::UnknownCurrentness,
            content_equivalence(left, right),
            lineage_equivalence(left, right),
        );
    }

    match (&left.presence, &right.presence) {
        (ReplicaPresenceV1::Unknown, _) | (_, ReplicaPresenceV1::Unknown) => result(
            ReplicaRelationV1::UnknownCurrentness,
            false,
            lineage_equivalence(left, right),
        ),
        (
            ReplicaPresenceV1::Present {
                content: left_content,
                lineage: left_lineage,
            },
            ReplicaPresenceV1::Present {
                content: right_content,
                lineage: right_lineage,
            },
        ) => {
            if left_content == right_content {
                return result(
                    ReplicaRelationV1::InSync,
                    true,
                    option_lineage_equivalence(left_lineage.as_ref(), right_lineage.as_ref()),
                );
            }
            relation_for_changed_content(left_lineage.as_ref(), right_lineage.as_ref())
        }
        (
            ReplicaPresenceV1::Tombstone {
                lineage: left_lineage,
            },
            ReplicaPresenceV1::Tombstone {
                lineage: right_lineage,
            },
        ) => result(
            ReplicaRelationV1::BothTombstoned,
            false,
            option_lineage_equivalence(left_lineage.as_ref(), right_lineage.as_ref()),
        ),
        (
            ReplicaPresenceV1::Tombstone {
                lineage: left_lineage,
            },
            ReplicaPresenceV1::Present {
                lineage: right_lineage,
                ..
            },
        ) => relation_for_tombstone_vs_present(left_lineage.as_ref(), right_lineage.as_ref(), true),
        (
            ReplicaPresenceV1::Present {
                lineage: left_lineage,
                ..
            },
            ReplicaPresenceV1::Tombstone {
                lineage: right_lineage,
            },
        ) => relation_for_tombstone_vs_present(right_lineage.as_ref(), left_lineage.as_ref(), false),
    }
}

fn admit_content_commitment(
    input: ContentCommitmentInputV1,
) -> Result<ContentCommitmentV1, ReconcileError> {
    let profile = canonical_text("content_profile", input.profile, MAX_PROFILE_LEN)?;
    let digest_hex = input.digest_hex;
    if digest_hex.len() < 2 || digest_hex.len() > MAX_DIGEST_HEX_LEN || digest_hex.len() % 2 != 0 {
        return Err(ReconcileError::InvalidDigestLength);
    }
    if !digest_hex
        .bytes()
        .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
    {
        return Err(ReconcileError::NonCanonicalDigest);
    }
    Ok(ContentCommitmentV1 {
        profile,
        digest_hex,
    })
}

fn admit_lineage(input: SemanticLineageInputV1) -> Result<SemanticLineageV1, ReconcileError> {
    Ok(SemanticLineageV1 {
        lineage_ref: canonical_text("lineage_ref", input.lineage_ref, MAX_LINEAGE_REF_LEN)?,
        generation: input.generation,
    })
}

fn canonical_text(
    field: &'static str,
    value: String,
    max: usize,
) -> Result<String, ReconcileError> {
    if value.is_empty() {
        return Err(ReconcileError::EmptyField { field });
    }
    if value.len() > max {
        return Err(ReconcileError::FieldTooLong { field, max });
    }
    if value.trim() != value {
        return Err(ReconcileError::NonCanonicalText { field });
    }
    if value.chars().any(char::is_control) {
        return Err(ReconcileError::ControlCharacter { field });
    }
    Ok(value)
}

fn relation_for_changed_content(
    left_lineage: Option<&SemanticLineageV1>,
    right_lineage: Option<&SemanticLineageV1>,
) -> ReplicaReconciliationV1 {
    match (left_lineage, right_lineage) {
        (Some(left), Some(right)) if left.lineage_ref == right.lineage_ref => {
            let relation = match left.generation.cmp(&right.generation) {
                Ordering::Greater => ReplicaRelationV1::LeftAhead,
                Ordering::Less => ReplicaRelationV1::RightAhead,
                Ordering::Equal => ReplicaRelationV1::Conflict,
            };
            result(relation, false, Some(true))
        }
        (Some(_), Some(_)) => result(ReplicaRelationV1::Diverged, false, Some(false)),
        _ => result(ReplicaRelationV1::Diverged, false, None),
    }
}

fn relation_for_tombstone_vs_present(
    tombstone_lineage: Option<&SemanticLineageV1>,
    present_lineage: Option<&SemanticLineageV1>,
    tombstone_is_left: bool,
) -> ReplicaReconciliationV1 {
    match (tombstone_lineage, present_lineage) {
        (Some(tombstone), Some(present)) if tombstone.lineage_ref == present.lineage_ref => {
            let relation = match tombstone.generation.cmp(&present.generation) {
                Ordering::Greater => {
                    if tombstone_is_left {
                        ReplicaRelationV1::LeftTombstoned
                    } else {
                        ReplicaRelationV1::RightTombstoned
                    }
                }
                Ordering::Less => {
                    if tombstone_is_left {
                        ReplicaRelationV1::RightAhead
                    } else {
                        ReplicaRelationV1::LeftAhead
                    }
                }
                Ordering::Equal => ReplicaRelationV1::Conflict,
            };
            result(relation, false, Some(true))
        }
        (Some(_), Some(_)) => result(ReplicaRelationV1::Diverged, false, Some(false)),
        _ => result(ReplicaRelationV1::Diverged, false, None),
    }
}

fn content_equivalence(
    left: &AdmittedReplicaObservationV1,
    right: &AdmittedReplicaObservationV1,
) -> bool {
    match (&left.presence, &right.presence) {
        (
            ReplicaPresenceV1::Present { content: left, .. },
            ReplicaPresenceV1::Present { content: right, .. },
        ) => left == right,
        _ => false,
    }
}

fn lineage_equivalence(
    left: &AdmittedReplicaObservationV1,
    right: &AdmittedReplicaObservationV1,
) -> Option<bool> {
    option_lineage_equivalence(lineage_of(&left.presence), lineage_of(&right.presence))
}

fn lineage_of(presence: &ReplicaPresenceV1) -> Option<&SemanticLineageV1> {
    match presence {
        ReplicaPresenceV1::Present { lineage, .. } | ReplicaPresenceV1::Tombstone { lineage } => {
            lineage.as_ref()
        }
        ReplicaPresenceV1::Unknown => None,
    }
}

fn option_lineage_equivalence(
    left: Option<&SemanticLineageV1>,
    right: Option<&SemanticLineageV1>,
) -> Option<bool> {
    match (left, right) {
        (Some(left), Some(right)) => Some(left.lineage_ref == right.lineage_ref),
        _ => None,
    }
}

fn result(
    relation: ReplicaRelationV1,
    content_equivalent: bool,
    same_lineage: Option<bool>,
) -> ReplicaReconciliationV1 {
    ReplicaReconciliationV1 {
        relation,
        content_equivalent,
        same_lineage,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn digest(byte: &str) -> ContentCommitmentInputV1 {
        ContentCommitmentInputV1 {
            profile: "sha256-hex-v1".into(),
            digest_hex: byte.repeat(64),
        }
    }

    fn lineage(generation: u64) -> Option<SemanticLineageInputV1> {
        Some(SemanticLineageInputV1 {
            lineage_ref: "lineage-A".into(),
            generation,
        })
    }

    fn observation(
        provider: &str,
        locator: &str,
        frontier: &str,
        presence: ReplicaPresenceInputV1,
    ) -> ReplicaObservationInputV1 {
        ReplicaObservationInputV1 {
            locator: ReplicaLocatorInputV1 {
                provider_family: provider.into(),
                provider_profile: format!("{provider}-profile-v1"),
                account_scope: "scope-1".into(),
                locator: locator.into(),
            },
            presence,
            currentness: ReplicaCurrentnessInputV1::Complete {
                frontier: frontier.into(),
            },
            provider_etag: None,
            provider_version: None,
        }
    }

    #[test]
    fn same_content_different_locators_is_content_sync_not_identity_collapse() {
        let left = admit_observation(observation(
            "webdav",
            "/a",
            "f1",
            ReplicaPresenceInputV1::Present {
                content: digest("a"),
                lineage: lineage(1),
            },
        ))
        .unwrap();
        let right = admit_observation(observation(
            "s3",
            "bucket/a",
            "f2",
            ReplicaPresenceInputV1::Present {
                content: digest("a"),
                lineage: lineage(1),
            },
        ))
        .unwrap();

        let result = reconcile_pair(&left, &right);
        assert_eq!(result.relation(), ReplicaRelationV1::InSync);
        assert!(result.content_equivalent());
        assert_ne!(left.locator(), right.locator());
    }

    #[test]
    fn same_etag_never_overrides_different_content() {
        let mut left_input = observation(
            "webdav",
            "/a",
            "f1",
            ReplicaPresenceInputV1::Present {
                content: digest("a"),
                lineage: None,
            },
        );
        left_input.provider_etag = Some("same-etag".into());
        let mut right_input = observation(
            "s3",
            "bucket/a",
            "f2",
            ReplicaPresenceInputV1::Present {
                content: digest("b"),
                lineage: None,
            },
        );
        right_input.provider_etag = Some("same-etag".into());

        let result = reconcile_pair(
            &admit_observation(left_input).unwrap(),
            &admit_observation(right_input).unwrap(),
        );
        assert_eq!(result.relation(), ReplicaRelationV1::Diverged);
        assert!(!result.content_equivalent());
    }

    #[test]
    fn same_lineage_generation_orders_changed_content() {
        let left = admit_observation(observation(
            "webdav",
            "/a",
            "f2",
            ReplicaPresenceInputV1::Present {
                content: digest("b"),
                lineage: lineage(2),
            },
        ))
        .unwrap();
        let right = admit_observation(observation(
            "s3",
            "bucket/a",
            "f1",
            ReplicaPresenceInputV1::Present {
                content: digest("a"),
                lineage: lineage(1),
            },
        ))
        .unwrap();

        assert_eq!(
            reconcile_pair(&left, &right).relation(),
            ReplicaRelationV1::LeftAhead
        );
    }

    #[test]
    fn equal_generation_with_conflicting_content_is_conflict() {
        let left = admit_observation(observation(
            "webdav",
            "/a",
            "f1",
            ReplicaPresenceInputV1::Present {
                content: digest("a"),
                lineage: lineage(1),
            },
        ))
        .unwrap();
        let right = admit_observation(observation(
            "s3",
            "bucket/a",
            "f2",
            ReplicaPresenceInputV1::Present {
                content: digest("b"),
                lineage: lineage(1),
            },
        ))
        .unwrap();

        assert_eq!(
            reconcile_pair(&left, &right).relation(),
            ReplicaRelationV1::Conflict
        );
    }

    #[test]
    fn partial_inventory_withholds_currentness_claim() {
        let mut right_input = observation(
            "s3",
            "bucket/a",
            "f1",
            ReplicaPresenceInputV1::Present {
                content: digest("a"),
                lineage: lineage(1),
            },
        );
        right_input.currentness = ReplicaCurrentnessInputV1::Partial {
            frontier: Some("page-1".into()),
        };

        let left = admit_observation(observation(
            "webdav",
            "/a",
            "f1",
            ReplicaPresenceInputV1::Present {
                content: digest("a"),
                lineage: lineage(1),
            },
        ))
        .unwrap();
        let right = admit_observation(right_input).unwrap();

        assert_eq!(
            reconcile_pair(&left, &right).relation(),
            ReplicaRelationV1::UnknownCurrentness
        );
    }

    #[test]
    fn newer_tombstone_is_explicit_not_effect_authority() {
        let left = admit_observation(observation(
            "webdav",
            "/a",
            "f2",
            ReplicaPresenceInputV1::Tombstone {
                lineage: lineage(2),
            },
        ))
        .unwrap();
        let right = admit_observation(observation(
            "s3",
            "bucket/a",
            "f1",
            ReplicaPresenceInputV1::Present {
                content: digest("a"),
                lineage: lineage(1),
            },
        ))
        .unwrap();

        assert_eq!(
            reconcile_pair(&left, &right).relation(),
            ReplicaRelationV1::LeftTombstoned
        );
    }

    #[test]
    fn conflicting_same_locator_frontier_fails_canonicalization() {
        let first = observation(
            "s3",
            "bucket/a",
            "f1",
            ReplicaPresenceInputV1::Present {
                content: digest("a"),
                lineage: lineage(1),
            },
        );
        let second = observation(
            "s3",
            "bucket/a",
            "f1",
            ReplicaPresenceInputV1::Present {
                content: digest("b"),
                lineage: lineage(1),
            },
        );

        assert_eq!(
            canonicalize_observations([first, second]),
            Err(ReconcileError::ConflictingDuplicateObservation)
        );
    }

    #[test]
    fn canonicalization_is_input_order_independent() {
        let a = observation(
            "s3",
            "bucket/a",
            "f1",
            ReplicaPresenceInputV1::Present {
                content: digest("a"),
                lineage: lineage(1),
            },
        );
        let b = observation(
            "webdav",
            "/b",
            "f2",
            ReplicaPresenceInputV1::Present {
                content: digest("b"),
                lineage: lineage(2),
            },
        );

        assert_eq!(
            canonicalize_observations([a.clone(), b.clone()]).unwrap(),
            canonicalize_observations([b, a]).unwrap()
        );
    }

    #[test]
    fn noncanonical_digest_is_rejected() {
        let input = observation(
            "s3",
            "bucket/a",
            "f1",
            ReplicaPresenceInputV1::Present {
                content: ContentCommitmentInputV1 {
                    profile: "sha256-hex-v1".into(),
                    digest_hex: "AA".repeat(32),
                },
                lineage: lineage(1),
            },
        );

        assert_eq!(
            admit_observation(input),
            Err(ReconcileError::NonCanonicalDigest)
        );
    }

    #[test]
    fn provider_profile_is_part_of_locator_identity() {
        let mut a = observation(
            "s3",
            "bucket/a",
            "f1",
            ReplicaPresenceInputV1::Present {
                content: digest("a"),
                lineage: lineage(1),
            },
        );
        let mut b = a.clone();
        a.locator.provider_profile = "aws-s3-v1".into();
        b.locator.provider_profile = "other-s3-v1".into();

        let a = admit_observation(a).unwrap();
        let b = admit_observation(b).unwrap();
        assert_ne!(a.locator(), b.locator());
    }
}
