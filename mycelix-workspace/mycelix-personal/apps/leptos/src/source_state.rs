// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Typed, fail-closed source observations for the Personal life-continuity UI.
//!
//! These types describe what a source read established. They do not establish
//! the truth of the source's claims, grant authority to a source, or authorize
//! an action. Callers must apply domain-owned authority and freshness policies.
//!
//! In particular, unavailable is never empty, stale is never current, mock data
//! is never live evidence, and disagreement is preserved rather than resolved
//! by display order.

use serde::{Deserialize, Serialize};

/// Stable machine-readable classification for rendering, metrics, and tests.
/// Do not use this classification as an authorization decision.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum SourceStateKind {
    NotChecked,
    LiveWithRecords,
    ConfirmedEmpty,
    Partial,
    Unavailable,
    Stale,
    Mock,
    Disputed,
    NotIntegrated,
}

impl SourceStateKind {
    pub const fn label(self) -> &'static str {
        match self {
            Self::NotChecked => "Not checked",
            Self::LiveWithRecords => "Records returned",
            Self::ConfirmedEmpty => "Confirmed empty",
            Self::Partial => "Partial result",
            Self::Unavailable => "Unavailable",
            Self::Stale => "Stale evidence",
            Self::Mock => "Demo data",
            Self::Disputed => "Disputed",
            Self::NotIntegrated => "Not integrated",
        }
    }

    pub const fn explanation(self) -> &'static str {
        match self {
            Self::NotChecked => "This source has not been checked yet.",
            Self::LiveWithRecords => {
                "The source returned records for this read; claim truth and authority are not automatically established."
            }
            Self::ConfirmedEmpty => {
                "The source successfully returned no records at this read time."
            }
            Self::Partial => "Some subreads succeeded and others failed; the result is incomplete.",
            Self::Unavailable => {
                "The read failed; absence has not been established. Any retained last-known value is stale."
            }
            Self::Stale => "Last-known evidence exists but must not be treated as current.",
            Self::Mock => "These are synthetic fixtures, not live source records.",
            Self::Disputed => "Attributable claims conflict; no winner has been selected.",
            Self::NotIntegrated => {
                "No source adapter is configured; this is not evidence of absence."
            }
        }
    }
}

/// Evidence describing one successful read or attributed claim.
///
/// `authority_scope` is descriptive metadata, not a capability or proof of
/// authority. `read_at_unix_ms` is the time the source was observed, not
/// necessarily the time a claim became true. Domain policy must decide whether
/// the evidence is sufficiently current for a particular use.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct SourceEvidence {
    pub source_id: String,
    pub read_at_unix_ms: u64,
    pub authority_scope: String,
    pub freshness_policy_id: String,
    pub evidence_digest: Option<String>,
    pub limitations: Vec<String>,
}

/// A vector that cannot represent an empty collection.
///
/// This lets `LiveWithRecords` and stale evidence carry at least one record.
/// Use `SourceObservation::ConfirmedEmpty` for an empty successful response.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct NonEmptyVec<T> {
    first: T,
    rest: Vec<T>,
}

impl<T> NonEmptyVec<T> {
    pub fn new(first: T) -> Self {
        Self {
            first,
            rest: Vec::new(),
        }
    }

    /// Return the non-empty vector or return the original empty vector.
    pub fn try_from_vec(values: Vec<T>) -> Result<Self, Vec<T>> {
        let mut iter = values.into_iter();
        match iter.next() {
            Some(first) => Ok(Self {
                first,
                rest: iter.collect(),
            }),
            None => Err(Vec::new()),
        }
    }

    pub fn first(&self) -> &T {
        &self.first
    }

    pub fn rest(&self) -> &[T] {
        &self.rest
    }

    pub fn len(&self) -> usize {
        1 + self.rest.len()
    }

    /// A non-empty vector is never empty by construction.
    pub fn is_empty(&self) -> bool {
        false
    }

    pub fn iter(&self) -> impl Iterator<Item = &T> {
        std::iter::once(&self.first).chain(self.rest.iter())
    }

    pub fn into_vec(self) -> Vec<T> {
        let mut values = Vec::with_capacity(1 + self.rest.len());
        values.push(self.first);
        values.extend(self.rest);
        values
    }
}

/// Why a previously observed value is not safe to present as current.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum StaleReason {
    AgeLimitExceeded,
    SourceNotRechecked,
    SourceUnavailable,
    ClockUncertain,
    Unclassified,
}

/// Last-known records, explicitly marked stale rather than re-labelled as
/// current merely because a new read failed.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct StaleObservation<T> {
    pub records: NonEmptyVec<T>,
    pub evidence: SourceEvidence,
    pub last_attempt_at_unix_ms: Option<u64>,
    pub reason: StaleReason,
}

/// Categorized read failures. Avoid storing raw provider error payloads here:
/// they may contain identifiers, request data, or other sensitive material.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum SourceFailure {
    Timeout,
    ProviderUnavailable,
    Transport,
    StorageUnavailable,
    MalformedResponse,
    Unauthorized,
    UnclassifiedReadFailure,
}

/// A successful component of a multi-part read. Each component explicitly
/// reports records or a successful empty result.
///
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum SuccessfulSourceRead<T> {
    Records {
        records: NonEmptyVec<T>,
        evidence: SourceEvidence,
    },
    ConfirmedEmpty { evidence: SourceEvidence },
}

/// One failed component of a multi-part read. The error reason is a code, not
/// a raw provider message.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct SourceReadFailure {
    pub source_id: String,
    pub attempted_at_unix_ms: Option<u64>,
    pub reason: SourceFailure,
}

/// A genuinely partial read requires at least one successful component and at
/// least one failed component. Successful empty components remain explicit.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct PartialObservation<T> {
    pub successful_reads: NonEmptyVec<SuccessfulSourceRead<T>>,
    pub failed_reads: NonEmptyVec<SourceReadFailure>,
}

/// An attributable claim retained when sources disagree. Attribution alone
/// does not prove the claim is true or that the source is authoritative.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct AttributedClaim<T> {
    pub value: T,
    pub evidence: SourceEvidence,
}

/// Why attributed claims conflict. Codes avoid retaining provider-supplied
/// free-form messages that could contain personal data.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum ConflictReason {
    ContradictoryClaims,
    AuthorityScopeDisagreement,
    FreshnessDisagreement,
    StatusDisagreement,
    Unclassified,
}

/// Why a source adapter is not available for this view.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum NotIntegratedReason {
    NoAdapterConfigured,
    RequiresAuthorizedProvider,
    UnsupportedInThisBuild,
    Unclassified,
}

/// Two or more attributed claims. A single claim cannot form a dispute.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ConflictingClaims<T> {
    first: T,
    second: T,
    rest: Vec<T>,
}

impl<T> ConflictingClaims<T> {
    pub fn try_from_vec(values: Vec<T>) -> Result<Self, Vec<T>> {
        let mut iter = values.into_iter();
        let first = match iter.next() {
            Some(value) => value,
            None => return Err(Vec::new()),
        };
        let second = match iter.next() {
            Some(value) => value,
            None => return Err(vec![first]),
        };
        Ok(Self {
            first,
            second,
            rest: iter.collect(),
        })
    }

    pub fn len(&self) -> usize {
        2 + self.rest.len()
    }

    /// ConflictingClaims always contains at least two claims.
    pub fn is_empty(&self) -> bool {
        false
    }

    pub fn iter(&self) -> impl Iterator<Item = &T> {
        std::iter::once(&self.first)
            .chain(std::iter::once(&self.second))
            .chain(self.rest.iter())
    }
}

/// A source result whose variants deliberately encode distinct epistemic
/// states. The type is a read-model contract, not an access-control primitive.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum SourceObservation<T> {
    /// No source read has completed for this view.
    NotChecked { source_id: String },
    /// A successful read returned at least one record.
    LiveWithRecords {
        records: NonEmptyVec<T>,
        evidence: SourceEvidence,
    },
    /// A successful read returned no records at the stated read time.
    ConfirmedEmpty {
        evidence: SourceEvidence,
    },
    /// Some components succeeded and some failed. Component outcomes are
    /// preserved rather than flattened into either success or total failure.
    Partial { observation: PartialObservation<T> },
    /// The current read failed. Any retained previous value is separately
    /// represented as stale and must not be shown as a current source result.
    Unavailable {
        source_id: String,
        attempted_at_unix_ms: Option<u64>,
        reason: SourceFailure,
        last_known: Option<StaleObservation<T>>,
    },
    /// Previously observed records are retained, but a domain freshness policy
    /// has determined that they must not be presented as current.
    Stale { observation: StaleObservation<T> },
    /// Explicit fixtures only. This variant must never be emitted as live
    /// evidence or used to authorize real disclosure or action.
    Mock {
        source_id: String,
        fixture_id: String,
        records: Vec<T>,
    },
    /// Conflicting attributed claims. No winner is selected by this type.
    Disputed {
        source_id: String,
        claims: ConflictingClaims<AttributedClaim<T>>,
        reason: ConflictReason,
    },
    /// No adapter exists for this source in this product/build.
    NotIntegrated {
        source_id: String,
        reason: NotIntegratedReason,
    },
}

impl<T: Clone> SourceObservation<T> {
    /// Preserve prior live evidence as stale when a subsequent read fails.
    /// Mock, unsupported, disputed, and never-checked observations do not
    /// become stale live evidence.
    pub fn stale_snapshot(
        &self,
        last_attempt_at_unix_ms: Option<u64>,
        reason: StaleReason,
    ) -> Option<StaleObservation<T>> {
        match self {
            Self::LiveWithRecords { records, evidence } => Some(StaleObservation {
                records: records.clone(),
                evidence: evidence.clone(),
                last_attempt_at_unix_ms,
                reason,
            }),
            Self::Stale { observation } => Some(StaleObservation {
                records: observation.records.clone(),
                evidence: observation.evidence.clone(),
                last_attempt_at_unix_ms,
                reason,
            }),
            Self::Unavailable { last_known, .. } => {
                last_known.as_ref().map(|observation| StaleObservation {
                    records: observation.records.clone(),
                    evidence: observation.evidence.clone(),
                    last_attempt_at_unix_ms,
                    reason,
                })
            }
            Self::NotChecked { .. }
            | Self::ConfirmedEmpty { .. }
            | Self::Partial { .. }
            | Self::Mock { .. }
            | Self::Disputed { .. }
            | Self::NotIntegrated { .. } => None,
        }
    }
}

impl<T> SourceObservation<T> {
    /// Classify a successful read without allowing an empty vector to be
    /// mislabeled as a live result containing records.
    pub fn from_successful_read(records: Vec<T>, evidence: SourceEvidence) -> Self {
        match NonEmptyVec::try_from_vec(records) {
            Ok(records) => Self::LiveWithRecords { records, evidence },
            Err(_) => Self::ConfirmedEmpty { evidence },
        }
    }

    /// Preserve a failed attempt as unavailable. Any prior records must be
    /// passed separately as a stale observation, never as current records.
    pub fn from_failed_read(
        source_id: impl Into<String>,
        attempted_at_unix_ms: Option<u64>,
        reason: SourceFailure,
        last_known: Option<StaleObservation<T>>,
    ) -> Self {
        Self::Unavailable {
            source_id: source_id.into(),
            attempted_at_unix_ms,
            reason,
            last_known,
        }
    }

    pub fn kind(&self) -> SourceStateKind {
        match self {
            Self::NotChecked { .. } => SourceStateKind::NotChecked,
            Self::LiveWithRecords { .. } => SourceStateKind::LiveWithRecords,
            Self::ConfirmedEmpty { .. } => SourceStateKind::ConfirmedEmpty,
            Self::Partial { .. } => SourceStateKind::Partial,
            Self::Unavailable { .. } => SourceStateKind::Unavailable,
            Self::Stale { .. } => SourceStateKind::Stale,
            Self::Mock { .. } => SourceStateKind::Mock,
            Self::Disputed { .. } => SourceStateKind::Disputed,
            Self::NotIntegrated { .. } => SourceStateKind::NotIntegrated,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn evidence(source_id: &str) -> SourceEvidence {
        SourceEvidence {
            source_id: source_id.into(),
            read_at_unix_ms: 1_800_000_000_000,
            authority_scope: "profile-read-only".into(),
            freshness_policy_id: "personal-profile-v1".into(),
            evidence_digest: None,
            limitations: vec!["synthetic contract fixture".into()],
        }
    }

    #[test]
    fn empty_records_cannot_be_classified_as_live_with_records() {
        assert_eq!(NonEmptyVec::<u8>::try_from_vec(Vec::new()), Err(Vec::new()));
        let records = NonEmptyVec::try_from_vec(vec![4_u8, 5]).expect("non-empty");
        assert_eq!(records.len(), 2);
        assert_eq!(records.iter().copied().collect::<Vec<_>>(), vec![4, 5]);
    }

    #[test]
    fn read_adapter_maps_empty_and_non_empty_successes_to_distinct_states() {
        let empty =
            SourceObservation::<u8>::from_successful_read(Vec::new(), evidence("profile-service"));
        assert_eq!(empty.kind(), SourceStateKind::ConfirmedEmpty);

        let populated =
            SourceObservation::from_successful_read(vec![9_u8, 10], evidence("profile-service"));
        match populated {
            SourceObservation::LiveWithRecords { records, .. } => {
                assert_eq!(records.into_vec(), vec![9, 10]);
            }
            other => panic!("non-empty success should be live: {other:?}"),
        }
    }

    #[test]
    fn only_prior_live_evidence_can_be_carried_forward_as_stale() {
        let live = SourceObservation::from_successful_read(
            vec![4_u8, 5],
            evidence("profile-service"),
        );
        let stale = live
            .stale_snapshot(Some(1_800_000_000_300), StaleReason::SourceUnavailable)
            .expect("prior live evidence can be retained as stale");
        assert_eq!(stale.records.into_vec(), vec![4, 5]);
        assert_eq!(stale.evidence.source_id, "profile-service");
        assert_eq!(stale.reason, StaleReason::SourceUnavailable);

        let mock = SourceObservation::Mock {
            source_id: "fixture-source".into(),
            fixture_id: "synthetic-v1".into(),
            records: vec![4_u8],
        };
        assert!(
            mock.stale_snapshot(Some(1_800_000_000_300), StaleReason::SourceUnavailable)
                .is_none()
        );
    }

    #[test]
    fn failed_read_constructor_never_reports_confirmed_empty() {
        let failed: SourceObservation<u8> = SourceObservation::from_failed_read(
            "profile-service",
            Some(1_800_000_000_100),
            SourceFailure::Timeout,
            None,
        );
        assert_eq!(failed.kind(), SourceStateKind::Unavailable);
    }

    #[test]
    fn a_single_claim_cannot_be_constructed_as_a_dispute() {
        let only_claim = AttributedClaim {
            value: "one claim",
            evidence: evidence("single-source"),
        };
        assert!(ConflictingClaims::try_from_vec(vec![only_claim]).is_err());
    }

    #[test]
    fn confirmed_empty_is_distinct_from_unavailable_and_not_integrated() {
        let empty: SourceObservation<u8> = SourceObservation::ConfirmedEmpty {
            evidence: evidence("profile-service"),
        };
        let unavailable: SourceObservation<u8> = SourceObservation::Unavailable {
            source_id: "profile-service".into(),
            attempted_at_unix_ms: Some(1_800_000_000_100),
            reason: SourceFailure::Timeout,
            last_known: None,
        };
        let unsupported: SourceObservation<u8> = SourceObservation::NotIntegrated {
            source_id: "property-registry".into(),
            reason: NotIntegratedReason::RequiresAuthorizedProvider,
        };

        assert_eq!(empty.kind(), SourceStateKind::ConfirmedEmpty);
        assert_eq!(unavailable.kind(), SourceStateKind::Unavailable);
        assert_eq!(unsupported.kind(), SourceStateKind::NotIntegrated);
        assert_ne!(empty.kind(), unavailable.kind());
        assert_ne!(unavailable.kind(), unsupported.kind());
    }

    #[test]
    fn partial_observation_preserves_successes_and_failures_separately() {
        let successful_reads = NonEmptyVec::try_from_vec(vec![
            SuccessfulSourceRead::Records {
                records: NonEmptyVec::new("maintenance record"),
                evidence: evidence("maintenance-source"),
            },
            SuccessfulSourceRead::ConfirmedEmpty {
                evidence: evidence("recall-source"),
            },
        ])
        .expect("at least one successful subread");
        let failed_reads = NonEmptyVec::new(SourceReadFailure {
            source_id: "insurance-source".into(),
            attempted_at_unix_ms: Some(1_800_000_000_500),
            reason: SourceFailure::Timeout,
        });

        let partial = SourceObservation::Partial {
            observation: PartialObservation {
                successful_reads,
                failed_reads,
            },
        };
        assert_eq!(partial.kind(), SourceStateKind::Partial);
        match partial {
            SourceObservation::Partial { observation } => {
                assert_eq!(observation.successful_reads.len(), 2);
                assert_eq!(observation.failed_reads.len(), 1);
                assert_eq!(
                    observation.failed_reads.first().source_id,
                    "insurance-source"
                );
            }
            _ => panic!("partial result must not be flattened"),
        }
    }

    #[test]
    fn partial_observation_cannot_be_laundered_into_stale_live_evidence() {
        let partial = SourceObservation::Partial {
            observation: PartialObservation {
                successful_reads: NonEmptyVec::new(SuccessfulSourceRead::Records {
                    records: NonEmptyVec::new(8_u8),
                    evidence: evidence("one-source"),
                }),
                failed_reads: NonEmptyVec::new(SourceReadFailure {
                    source_id: "second-source".into(),
                    attempted_at_unix_ms: None,
                    reason: SourceFailure::ProviderUnavailable,
                }),
            },
        };
        assert!(
            partial
                .stale_snapshot(Some(1_800_000_000_600), StaleReason::SourceUnavailable)
                .is_none()
        );
    }

    #[test]
    fn mock_and_live_records_have_different_variants() {
        let live = SourceObservation::LiveWithRecords {
            records: NonEmptyVec::new(7_u8),
            evidence: evidence("live-source"),
        };
        let mock = SourceObservation::Mock {
            source_id: "fixture-source".into(),
            fixture_id: "profile-empty-outage-v1".into(),
            records: vec![7_u8],
        };

        assert_eq!(live.kind(), SourceStateKind::LiveWithRecords);
        assert_eq!(mock.kind(), SourceStateKind::Mock);
    }

    #[test]
    fn unavailable_last_known_data_remains_explicitly_stale() {
        let last_known = StaleObservation {
            records: NonEmptyVec::new("prior value".to_string()),
            evidence: evidence("remote-source"),
            last_attempt_at_unix_ms: Some(1_800_000_000_200),
            reason: StaleReason::SourceUnavailable,
        };
        let unavailable: SourceObservation<String> = SourceObservation::Unavailable {
            source_id: "remote-source".into(),
            attempted_at_unix_ms: Some(1_800_000_000_200),
            reason: SourceFailure::ProviderUnavailable,
            last_known: Some(last_known),
        };

        match unavailable {
            SourceObservation::Unavailable {
                last_known: Some(stale),
                ..
            } => {
                assert_eq!(stale.evidence.source_id, "remote-source");
                assert_eq!(stale.reason, StaleReason::SourceUnavailable);
            }
            _ => panic!("failed read must remain unavailable"),
        }
    }

    #[test]
    fn disputed_claims_preserve_each_claim_source() {
        let claims = ConflictingClaims::try_from_vec(vec![
            AttributedClaim {
                value: "claim A",
                evidence: evidence("registry-a"),
            },
            AttributedClaim {
                value: "claim B",
                evidence: evidence("registry-b"),
            },
        ])
        .expect("two claims");

        let disputed = SourceObservation::Disputed {
            source_id: "asset-status".into(),
            claims,
            reason: ConflictReason::ContradictoryClaims,
        };

        match disputed {
            SourceObservation::Disputed { claims, .. } => {
                assert_eq!(claims.len(), 2);
                assert_eq!(
                    claims
                        .iter()
                        .map(|claim| claim.evidence.source_id.as_str())
                        .collect::<Vec<_>>(),
                    vec!["registry-a", "registry-b"]
                );
            }
            _ => panic!("conflict must not be flattened"),
        }
    }
    #[test]
    fn user_facing_labels_preserve_uncertainty() {
        assert_eq!(SourceStateKind::LiveWithRecords.label(), "Records returned");
        assert!(
            SourceStateKind::LiveWithRecords
                .explanation()
                .contains("not automatically established")
        );

        assert_eq!(SourceStateKind::ConfirmedEmpty.label(), "Confirmed empty");
        assert_eq!(SourceStateKind::Partial.label(), "Partial result");
        assert!(
            SourceStateKind::Partial
                .explanation()
                .contains("result is incomplete")
        );
        assert!(
            SourceStateKind::ConfirmedEmpty
                .explanation()
                .contains("at this read time")
        );

        assert_eq!(SourceStateKind::Unavailable.label(), "Unavailable");
        assert!(
            SourceStateKind::Unavailable
                .explanation()
                .contains("absence has not been established")
        );
        assert!(
            SourceStateKind::Unavailable
                .explanation()
                .contains("retained last-known value is stale")
        );

        assert_eq!(SourceStateKind::Stale.label(), "Stale evidence");
        assert!(
            SourceStateKind::Stale
                .explanation()
                .contains("must not be treated as current")
        );

        assert_eq!(SourceStateKind::Mock.label(), "Demo data");
        assert!(
            SourceStateKind::Mock
                .explanation()
                .contains("not live source records")
        );

        assert_eq!(SourceStateKind::Disputed.label(), "Disputed");
        assert!(
            SourceStateKind::Disputed
                .explanation()
                .contains("no winner has been selected")
        );

        assert_eq!(SourceStateKind::NotIntegrated.label(), "Not integrated");
        assert!(
            SourceStateKind::NotIntegrated
                .explanation()
                .contains("not evidence of absence")
        );
    }

    #[test]
    fn every_state_has_a_distinct_serializable_classification() {
        let stale = StaleObservation {
            records: NonEmptyVec::new(1_u8),
            evidence: evidence("stale-source"),
            last_attempt_at_unix_ms: Some(1_800_000_000_300),
            reason: StaleReason::SourceNotRechecked,
        };
        let claims = ConflictingClaims::try_from_vec(vec![
            AttributedClaim {
                value: 1_u8,
                evidence: evidence("claim-source-a"),
            },
            AttributedClaim {
                value: 2_u8,
                evidence: evidence("claim-source-b"),
            },
        ])
        .expect("a dispute needs at least two claims");

        let cases: Vec<(SourceObservation<u8>, SourceStateKind)> = vec![
            (
                SourceObservation::NotChecked {
                    source_id: "not-checked".into(),
                },
                SourceStateKind::NotChecked,
            ),
            (
                SourceObservation::LiveWithRecords {
                    records: NonEmptyVec::new(1),
                    evidence: evidence("live-source"),
                },
                SourceStateKind::LiveWithRecords,
            ),
            (
                SourceObservation::ConfirmedEmpty {
                    evidence: evidence("empty-source"),
                },
                SourceStateKind::ConfirmedEmpty,
            ),
            (
                SourceObservation::Partial {
                    observation: PartialObservation {
                        successful_reads: NonEmptyVec::new(SuccessfulSourceRead::ConfirmedEmpty {
                            evidence: evidence("partial-empty-source"),
                        }),
                        failed_reads: NonEmptyVec::new(SourceReadFailure {
                            source_id: "partial-failed-source".into(),
                            attempted_at_unix_ms: Some(1_800_000_000_450),
                            reason: SourceFailure::Timeout,
                        }),
                    },
                },
                SourceStateKind::Partial,
            ),
            (
                SourceObservation::Unavailable {
                    source_id: "unavailable-source".into(),
                    attempted_at_unix_ms: Some(1_800_000_000_400),
                    reason: SourceFailure::Timeout,
                    last_known: Some(stale.clone()),
                },
                SourceStateKind::Unavailable,
            ),
            (
                SourceObservation::Stale { observation: stale },
                SourceStateKind::Stale,
            ),
            (
                SourceObservation::Mock {
                    source_id: "mock-source".into(),
                    fixture_id: "synthetic-v1".into(),
                    records: vec![1],
                },
                SourceStateKind::Mock,
            ),
            (
                SourceObservation::Disputed {
                    source_id: "disputed-source".into(),
                    claims,
                    reason: ConflictReason::ContradictoryClaims,
                },
                SourceStateKind::Disputed,
            ),
            (
                SourceObservation::NotIntegrated {
                    source_id: "unsupported-source".into(),
                    reason: NotIntegratedReason::NoAdapterConfigured,
                },
                SourceStateKind::NotIntegrated,
            ),
        ];

        let mut seen = std::collections::HashSet::new();
        for (observation, expected) in cases {
            assert_eq!(observation.kind(), expected);
            assert!(!expected.label().is_empty());
            assert!(!expected.explanation().is_empty());
            assert!(seen.insert(expected), "state classification must be unique");
            let encoded = serde_json::to_string(&observation).expect("serialize source state");
            let decoded: SourceObservation<u8> =
                serde_json::from_str(&encoded).expect("deserialize source state");
            assert_eq!(
                decoded, observation,
                "serde round-trip must preserve state and data"
            );
        }
        assert_eq!(seen.len(), 9, "all source states must be covered");
    }
    #[test]
    fn seeded_read_classification_stress_100k_cases() {
        // Deterministic state-classification stress test; this is not a
        // substitute for cross-tab or storage-fault integration testing.
        let mut seed = 0xD1CE_BA5E_u64;
        for index in 0..100_000_u64 {
            seed = seed
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1_442_695_040_888_963_407);

            match seed % 3 {
                0 => {
                    let observation = SourceObservation::<u64>::from_successful_read(
                        Vec::new(),
                        evidence("stress-source"),
                    );
                    assert_eq!(
                        observation.kind(),
                        SourceStateKind::ConfirmedEmpty,
                        "seed=0xD1CEBA5E case={index}"
                    );
                }
                1 => {
                    let observation = SourceObservation::from_successful_read(
                        vec![seed],
                        evidence("stress-source"),
                    );
                    match observation {
                        SourceObservation::LiveWithRecords { records, .. } => {
                            assert_eq!(records.first(), &seed, "case={index}");
                        }
                        _ => panic!("non-empty success misclassified at case={index}"),
                    }
                }
                _ => {
                    let observation: SourceObservation<u64> = SourceObservation::from_failed_read(
                        "stress-source",
                        Some(index),
                        SourceFailure::Timeout,
                        None,
                    );
                    assert_eq!(
                        observation.kind(),
                        SourceStateKind::Unavailable,
                        "seed=0xD1CEBA5E case={index}"
                    );
                }
            }
        }
    }
}
