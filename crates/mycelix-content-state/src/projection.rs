use std::collections::{BTreeMap, BTreeSet};

use crate::model::*;

const MICROS_PER_SECOND: i64 = 1_000_000;

trait HasActionV1 {
    fn action(&self) -> ActionRefV1;
}

impl HasActionV1 for ProviderAdvertisementEvidenceV1 {
    fn action(&self) -> ActionRefV1 {
        self.action
    }
}
impl HasActionV1 for AvailabilityEvidenceV1 {
    fn action(&self) -> ActionRefV1 {
        self.action
    }
}
impl HasActionV1 for WithdrawalEvidenceV1 {
    fn action(&self) -> ActionRefV1 {
        self.action
    }
}
impl HasActionV1 for ObservationEvidenceV1 {
    fn action(&self) -> ActionRefV1 {
        self.action
    }
}

fn normalize<T>(items: Vec<T>, issues: &mut Vec<ProjectionIssueV1>) -> BTreeMap<ActionRefV1, T>
where
    T: HasActionV1 + Eq,
{
    let mut values = BTreeMap::new();
    let mut conflicted = BTreeSet::new();
    for item in items {
        let action = item.action();
        if conflicted.contains(&action) {
            continue;
        }
        if let Some(existing) = values.get(&action) {
            if existing != &item {
                values.remove(&action);
                conflicted.insert(action);
                issues.push(ProjectionIssueV1 {
                    action,
                    kind: ProjectionIssueKindV1::ConflictingDuplicateAction,
                });
            }
            continue;
        }
        values.insert(action, item);
    }
    values
}

fn remove_cross_kind_conflicts(
    advertisements: &mut BTreeMap<ActionRefV1, ProviderAdvertisementEvidenceV1>,
    availability: &mut BTreeMap<ActionRefV1, AvailabilityEvidenceV1>,
    withdrawals: &mut BTreeMap<ActionRefV1, WithdrawalEvidenceV1>,
    observations: &mut BTreeMap<ActionRefV1, ObservationEvidenceV1>,
    issues: &mut Vec<ProjectionIssueV1>,
) {
    let mut counts = BTreeMap::<ActionRefV1, u8>::new();
    for action in advertisements
        .keys()
        .chain(availability.keys())
        .chain(withdrawals.keys())
        .chain(observations.keys())
    {
        *counts.entry(*action).or_default() += 1;
    }
    for (action, count) in counts {
        if count <= 1 {
            continue;
        }
        advertisements.remove(&action);
        availability.remove(&action);
        withdrawals.remove(&action);
        observations.remove(&action);
        if !issues.iter().any(|issue| {
            issue.action == action
                && issue.kind == ProjectionIssueKindV1::ConflictingDuplicateAction
        }) {
            issues.push(ProjectionIssueV1 {
                action,
                kind: ProjectionIssueKindV1::ConflictingDuplicateAction,
            });
        }
    }
}

fn expiry(authored_at: TimestampMicrosV1, ttl_seconds: u32) -> Option<TimestampMicrosV1> {
    let delta = i64::from(ttl_seconds).checked_mul(MICROS_PER_SECOND)?;
    authored_at.0.checked_add(delta).map(TimestampMicrosV1)
}

fn first_withdrawal(
    advertisement: ActionRefV1,
    withdrawals: &BTreeMap<ActionRefV1, WithdrawalEvidenceV1>,
) -> Option<&WithdrawalEvidenceV1> {
    withdrawals
        .values()
        .filter(|withdrawal| withdrawal.advertisement == advertisement)
        .min_by_key(|withdrawal| (withdrawal.authored_at, withdrawal.action))
}

fn withdrawal_projection(withdrawal: Option<&WithdrawalEvidenceV1>) -> WithdrawalObservationV1 {
    match withdrawal {
        Some(withdrawal) => WithdrawalObservationV1::Withdrawn {
            action: withdrawal.action,
            authored_at: withdrawal.authored_at,
        },
        None => WithdrawalObservationV1::NoWithdrawalObserved,
    }
}

fn push_issue(
    issues: &mut Vec<ProjectionIssueV1>,
    action: ActionRefV1,
    kind: ProjectionIssueKindV1,
) {
    issues.push(ProjectionIssueV1 { action, kind });
}

fn observation_outcome_valid(
    observation: &ObservationEvidenceV1,
    ad: &ProviderAdvertisementEvidenceV1,
) -> bool {
    match &observation.outcome {
        ObservationOutcomeEvidenceV1::VerifiedComplete { size_bytes } => {
            *size_bytes > 0 && *size_bytes <= ad.max_blob_size_bytes
        }
        ObservationOutcomeEvidenceV1::DigestMismatch {
            observed_digest,
            size_bytes,
        } => {
            *size_bytes > 0
                && *size_bytes <= ad.max_blob_size_bytes
                && observed_digest.algorithm == observation.digest.algorithm
                && observed_digest != &observation.digest
        }
        ObservationOutcomeEvidenceV1::UnavailableOrHidden
        | ObservationOutcomeEvidenceV1::Busy
        | ObservationOutcomeEvidenceV1::TransferFailed
        | ObservationOutcomeEvidenceV1::ProviderReportedIntegrityFailure => true,
    }
}

/// Deterministically projects a supplied evidence snapshot at an explicit timestamp.
///
/// Evidence authored after `evaluated_at` is discarded before deduplication,
/// referential validation, or diagnostics. Later evidence therefore cannot alter
/// an earlier historical replay.
///
/// This function performs no I/O and never reads the system clock. Its result is a
/// statement about the supplied snapshot only, not global Holochain finality.
pub fn project_content_state_v1(
    snapshot: EvidenceSnapshotV1,
    evaluated_at: TimestampMicrosV1,
) -> ProjectedContentStateV1 {
    let coverage = snapshot.coverage;
    let mut issues = Vec::new();

    // Causal replay boundary: future evidence is invisible to the projection in
    // every respect, including conflict detection and diagnostics.
    let mut advertisements = normalize(
        snapshot
            .advertisements
            .into_iter()
            .filter(|record| record.authored_at <= evaluated_at)
            .collect(),
        &mut issues,
    );
    let mut availability = normalize(
        snapshot
            .availability
            .into_iter()
            .filter(|record| record.authored_at <= evaluated_at)
            .collect(),
        &mut issues,
    );
    let mut withdrawals = normalize(
        snapshot
            .withdrawals
            .into_iter()
            .filter(|record| record.authored_at <= evaluated_at)
            .collect(),
        &mut issues,
    );
    let mut observations = normalize(
        snapshot
            .observations
            .into_iter()
            .filter(|record| record.authored_at <= evaluated_at)
            .collect(),
        &mut issues,
    );
    remove_cross_kind_conflicts(
        &mut advertisements,
        &mut availability,
        &mut withdrawals,
        &mut observations,
        &mut issues,
    );

    // Invalid parent advertisements are excluded from every derived projection.
    let mut invalid_ads = BTreeSet::new();
    for ad in advertisements.values() {
        if ad.ttl_seconds == 0 {
            push_issue(&mut issues, ad.action, ProjectionIssueKindV1::ZeroTtl);
            invalid_ads.insert(ad.action);
        }
        if ad.max_blob_size_bytes == 0 {
            push_issue(&mut issues, ad.action, ProjectionIssueKindV1::InvalidSize);
            invalid_ads.insert(ad.action);
        }
        if expiry(ad.authored_at, ad.ttl_seconds).is_none() {
            push_issue(
                &mut issues,
                ad.action,
                ProjectionIssueKindV1::TimestampOverflow,
            );
            invalid_ads.insert(ad.action);
        }
    }
    for action in invalid_ads {
        advertisements.remove(&action);
    }

    // Referentially validate withdrawals before they can terminate a claim.
    let mut invalid_withdrawals = BTreeSet::new();
    for withdrawal in withdrawals.values() {
        let Some(ad) = advertisements.get(&withdrawal.advertisement) else {
            push_issue(
                &mut issues,
                withdrawal.action,
                ProjectionIssueKindV1::MissingAdvertisement {
                    advertisement: withdrawal.advertisement,
                },
            );
            invalid_withdrawals.insert(withdrawal.action);
            continue;
        };
        if withdrawal.provider != ad.provider {
            push_issue(
                &mut issues,
                withdrawal.action,
                ProjectionIssueKindV1::ProviderMismatch,
            );
            invalid_withdrawals.insert(withdrawal.action);
        }
        if withdrawal.authored_at < ad.authored_at {
            push_issue(
                &mut issues,
                withdrawal.action,
                ProjectionIssueKindV1::EvidencePredatesAdvertisement,
            );
            invalid_withdrawals.insert(withdrawal.action);
        }
    }
    for action in invalid_withdrawals {
        withdrawals.remove(&action);
    }

    let mut projected_ads = Vec::new();
    for ad in advertisements.values() {
        let expires_at = expiry(ad.authored_at, ad.ttl_seconds)
            .expect("invalid advertisement expiries were removed above");
        let temporal_state = if evaluated_at < expires_at {
            AdvertisementTemporalStateV1::Live { expires_at }
        } else {
            AdvertisementTemporalStateV1::Expired {
                expired_at: expires_at,
            }
        };
        let withdrawal = first_withdrawal(ad.action, &withdrawals);
        projected_ads.push(ProjectedAdvertisementV1 {
            action: ad.action,
            provider: ad.provider,
            iroh_endpoint_id: ad.iroh_endpoint_id,
            supported_algorithms: ad.supported_algorithms.clone(),
            max_blob_size_bytes: ad.max_blob_size_bytes,
            failure_domains: ad.failure_domains.clone(),
            authored_at: ad.authored_at,
            temporal_state,
            withdrawal: withdrawal_projection(withdrawal),
        });
    }

    let mut candidates = Vec::new();
    for claim in availability.values() {
        let Some(ad) = advertisements.get(&claim.advertisement) else {
            push_issue(
                &mut issues,
                claim.action,
                ProjectionIssueKindV1::MissingAdvertisement {
                    advertisement: claim.advertisement,
                },
            );
            continue;
        };
        if claim.provider != ad.provider {
            push_issue(
                &mut issues,
                claim.action,
                ProjectionIssueKindV1::ProviderMismatch,
            );
            continue;
        }
        if claim.authored_at < ad.authored_at {
            push_issue(
                &mut issues,
                claim.action,
                ProjectionIssueKindV1::EvidencePredatesAdvertisement,
            );
            continue;
        }
        if !ad.supported_algorithms.contains(&claim.digest.algorithm) {
            push_issue(
                &mut issues,
                claim.action,
                ProjectionIssueKindV1::UnsupportedDigestAlgorithm,
            );
            continue;
        }
        if claim.size_bytes == 0 || claim.size_bytes > ad.max_blob_size_bytes {
            push_issue(
                &mut issues,
                claim.action,
                ProjectionIssueKindV1::InvalidSize,
            );
            continue;
        }
        if claim.ttl_seconds == 0 {
            push_issue(&mut issues, claim.action, ProjectionIssueKindV1::ZeroTtl);
            continue;
        }
        if claim.ttl_seconds > ad.ttl_seconds {
            push_issue(
                &mut issues,
                claim.action,
                ProjectionIssueKindV1::TtlExceedsAdvertisement,
            );
            continue;
        }

        let ad_expires = expiry(ad.authored_at, ad.ttl_seconds)
            .expect("invalid advertisement expiries were removed above");
        if claim.authored_at >= ad_expires {
            push_issue(
                &mut issues,
                claim.action,
                ProjectionIssueKindV1::AvailabilityAfterAdvertisementExpiry,
            );
            continue;
        }
        let observed_withdrawal = first_withdrawal(ad.action, &withdrawals);
        if observed_withdrawal.is_some_and(|withdrawal| withdrawal.authored_at <= claim.authored_at)
        {
            push_issue(
                &mut issues,
                claim.action,
                ProjectionIssueKindV1::AvailabilityAfterObservedWithdrawal,
            );
            continue;
        }
        let Some(claim_expires) = expiry(claim.authored_at, claim.ttl_seconds) else {
            push_issue(
                &mut issues,
                claim.action,
                ProjectionIssueKindV1::TimestampOverflow,
            );
            continue;
        };
        let withdrawal_boundary = observed_withdrawal.map(|withdrawal| withdrawal.authored_at);
        let effective_until = withdrawal_boundary
            .into_iter()
            .chain([claim_expires, ad_expires])
            .min()
            .expect("claim and advertisement expiry always exist");

        // Half-open time semantics: [claim_authored_at, effective_until).
        if evaluated_at >= effective_until {
            continue;
        }
        // A withdrawal observed at or before evaluation always suppresses the
        // candidate. At this point a prior-to-claim withdrawal has already been
        // diagnosed separately.
        if observed_withdrawal.is_some() {
            continue;
        }
        candidates.push(SnapshotServiceCandidateV1 {
            availability_action: claim.action,
            advertisement_action: ad.action,
            provider: ad.provider,
            iroh_endpoint_id: ad.iroh_endpoint_id,
            digest: claim.digest,
            size_bytes: claim.size_bytes,
            failure_domains: ad.failure_domains.clone(),
            claim_authored_at: claim.authored_at,
            effective_until,
            withdrawal: WithdrawalObservationV1::NoWithdrawalObserved,
        });
    }

    let mut projected_observations = Vec::new();
    for observation in observations.values() {
        let Some(ad) = advertisements.get(&observation.advertisement) else {
            push_issue(
                &mut issues,
                observation.action,
                ProjectionIssueKindV1::MissingAdvertisement {
                    advertisement: observation.advertisement,
                },
            );
            continue;
        };
        if observation.provider != ad.provider {
            push_issue(
                &mut issues,
                observation.action,
                ProjectionIssueKindV1::ProviderMismatch,
            );
            continue;
        }
        if observation.authored_at < ad.authored_at {
            push_issue(
                &mut issues,
                observation.action,
                ProjectionIssueKindV1::EvidencePredatesAdvertisement,
            );
            continue;
        }
        if !ad.supported_algorithms.contains(&observation.digest.algorithm) {
            push_issue(
                &mut issues,
                observation.action,
                ProjectionIssueKindV1::UnsupportedDigestAlgorithm,
            );
            continue;
        }
        if !observation_outcome_valid(observation, ad) {
            push_issue(
                &mut issues,
                observation.action,
                ProjectionIssueKindV1::InvalidObservationOutcome,
            );
            continue;
        }
        let Some(age) = evaluated_at.0.checked_sub(observation.authored_at.0) else {
            push_issue(
                &mut issues,
                observation.action,
                ProjectionIssueKindV1::TimestampOverflow,
            );
            continue;
        };
        let Ok(age_micros) = u64::try_from(age) else {
            push_issue(
                &mut issues,
                observation.action,
                ProjectionIssueKindV1::TimestampOverflow,
            );
            continue;
        };
        projected_observations.push(ProjectedObservationV1 {
            action: observation.action,
            authored_at: observation.authored_at,
            age_micros,
            observer: observation.observer,
            provider: observation.provider,
            advertisement: observation.advertisement,
            digest: observation.digest,
            outcome: observation.outcome.clone(),
            latency_ms: observation.latency_ms,
        });
    }

    projected_ads.sort_by_key(|ad| ad.action);
    candidates.sort_by_key(|candidate| {
        (
            candidate.digest,
            candidate.provider,
            candidate.advertisement_action,
            candidate.availability_action,
        )
    });
    projected_observations.sort_by_key(|observation| (observation.authored_at, observation.action));
    issues.sort();
    issues.dedup();

    ProjectedContentStateV1 {
        evaluated_at,
        coverage,
        advertisements: projected_ads,
        service_candidates: candidates,
        observations: projected_observations,
        issues,
    }
}
