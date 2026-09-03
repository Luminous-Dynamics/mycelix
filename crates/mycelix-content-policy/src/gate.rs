use std::collections::{BTreeMap, BTreeSet};

use mycelix_content_core::{EncryptionRequirementV1, RetentionRequirementV1, StorageIntentV1};
use mycelix_content_state::{
    ActionRefV1, ProjectedContentStateV1, SnapshotCoverageV1, SnapshotServiceCandidateV1,
    WithdrawalObservationV1,
};

use crate::{
    CandidateRejectionReasonV1, CandidateRejectionV1, HardPolicyEvaluationV1,
    HardPolicyGateConfigV1, PlacementTargetV1, PolicyEligibleCandidateV1,
    PolicyQualifiedPoolV1, PoolFailureV1, ProviderPolicyEvidenceV1, RetentionCapabilityEvidenceV1,
    SelectionPolicyErrorV1,
};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum RetentionNeedV1 {
    None,
    Until(u64),
    Indefinite,
}

fn observed_count(value: usize) -> u16 {
    u16::try_from(value).unwrap_or(u16::MAX)
}

fn is_canonical_domain_value(value: &str) -> bool {
    let bytes = value.as_bytes();
    !bytes.is_empty()
        && bytes.len() <= 64
        && bytes.iter().all(|byte| {
            byte.is_ascii_lowercase()
                || byte.is_ascii_digit()
                || matches!(*byte, b'-' | b'_' | b'.' | b':')
        })
}

fn normalize_provider_evidence(
    evidence: Vec<ProviderPolicyEvidenceV1>,
) -> (
    BTreeMap<ActionRefV1, ProviderPolicyEvidenceV1>,
    BTreeSet<ActionRefV1>,
) {
    let mut normalized = BTreeMap::new();
    let mut conflicted = BTreeSet::new();

    for item in evidence {
        if conflicted.contains(&item.advertisement) {
            continue;
        }
        if let Some(existing) = normalized.get(&item.advertisement) {
            if existing != &item {
                normalized.remove(&item.advertisement);
                conflicted.insert(item.advertisement);
            }
            continue;
        }
        normalized.insert(item.advertisement, item);
    }

    (normalized, conflicted)
}

fn canonical_target_candidates<'a>(
    state: &'a ProjectedContentStateV1,
    target: &PlacementTargetV1,
) -> Vec<&'a SnapshotServiceCandidateV1> {
    let mut by_advertisement = BTreeMap::<ActionRefV1, &SnapshotServiceCandidateV1>::new();
    for candidate in &state.service_candidates {
        if candidate.digest != target.digest {
            continue;
        }
        match by_advertisement.get(&candidate.advertisement_action) {
            None => {
                by_advertisement.insert(candidate.advertisement_action, candidate);
            }
            Some(existing)
                if (candidate.claim_authored_at, candidate.availability_action)
                    > (existing.claim_authored_at, existing.availability_action) =>
            {
                by_advertisement.insert(candidate.advertisement_action, candidate);
            }
            Some(_) => {}
        }
    }
    by_advertisement.into_values().collect()
}

fn retention_need(
    requirement: RetentionRequirementV1,
    evaluated_at_unix_ms: u64,
) -> Result<RetentionNeedV1, PoolFailureV1> {
    match requirement {
        RetentionRequirementV1::BestEffort => Ok(RetentionNeedV1::None),
        RetentionRequirementV1::MinimumSeconds(seconds) => {
            let delta = seconds
                .checked_mul(1_000)
                .ok_or(PoolFailureV1::RetentionRequirementOverflow)?;
            let until = evaluated_at_unix_ms
                .checked_add(delta)
                .ok_or(PoolFailureV1::RetentionRequirementOverflow)?;
            Ok(RetentionNeedV1::Until(until))
        }
        RetentionRequirementV1::Until(until) => {
            if until.0 <= evaluated_at_unix_ms {
                Ok(RetentionNeedV1::None)
            } else {
                Ok(RetentionNeedV1::Until(until.0))
            }
        }
        RetentionRequirementV1::Indefinite => Ok(RetentionNeedV1::Indefinite),
    }
}

fn client_side_required(requirement: EncryptionRequirementV1) -> bool {
    matches!(
        requirement,
        EncryptionRequirementV1::ClientSide | EncryptionRequirementV1::ClientSideAndProviderAtRest
    )
}

fn provider_at_rest_required(requirement: EncryptionRequirementV1) -> bool {
    matches!(
        requirement,
        EncryptionRequirementV1::ProviderAtRest
            | EncryptionRequirementV1::ClientSideAndProviderAtRest
    )
}

fn retention_satisfies(
    evidence: &RetentionCapabilityEvidenceV1,
    need: RetentionNeedV1,
) -> bool {
    match need {
        RetentionNeedV1::None => true,
        RetentionNeedV1::Until(required_until) => {
            evidence.supports_indefinite
                || evidence
                    .guaranteed_until_unix_ms
                    .is_some_and(|until| until >= required_until)
        }
        RetentionNeedV1::Indefinite => evidence.supports_indefinite,
    }
}

fn push_rejection(
    rejections: &mut Vec<CandidateRejectionV1>,
    candidate: &SnapshotServiceCandidateV1,
    mut reasons: Vec<CandidateRejectionReasonV1>,
) {
    reasons.sort();
    reasons.dedup();
    rejections.push(CandidateRejectionV1 {
        availability_action: candidate.availability_action,
        advertisement_action: candidate.advertisement_action,
        provider: candidate.provider,
        reasons,
    });
}

/// Apply Content Fabric hard policy before any optimizer or preference scoring sees candidates.
///
/// Provider policy evidence is deliberately separate from CF-05 self-claims. The gate does not
/// verify attestation signatures itself; an upstream assurance boundary must assign assurance
/// levels honestly. Strict mode accepts only `IndependentlyAttested` provider facts.
pub fn evaluate_hard_policy_v1(
    intent: &StorageIntentV1,
    target: PlacementTargetV1,
    state: &ProjectedContentStateV1,
    provider_evidence: Vec<ProviderPolicyEvidenceV1>,
    config: HardPolicyGateConfigV1,
) -> HardPolicyEvaluationV1 {
    let mut failures = Vec::new();

    if intent.validate().is_err() {
        failures.push(PoolFailureV1::InvalidStorageIntent);
    }
    if target.size_bytes == 0 {
        failures.push(PoolFailureV1::InvalidPlacementTarget);
    }
    if target.object_id != intent.object_id {
        failures.push(PoolFailureV1::TargetObjectMismatch);
    }
    if state.evaluated_at.0 < 0 {
        failures.push(PoolFailureV1::InvalidEvaluationTimestamp);
    }

    let evaluated_at_unix_ms = if state.evaluated_at.0 < 0 {
        None
    } else {
        u64::try_from(state.evaluated_at.0 / 1_000).ok()
    };
    if evaluated_at_unix_ms.is_none()
        && !failures.contains(&PoolFailureV1::InvalidEvaluationTimestamp)
    {
        failures.push(PoolFailureV1::InvalidEvaluationTimestamp);
    }
    if let Some(now_ms) = evaluated_at_unix_ms {
        if now_ms < intent.created_at.0 {
            failures.push(PoolFailureV1::EvaluationPredatesIntent);
        }
    }
    if config.require_queried_indexes_complete
        && state.coverage != SnapshotCoverageV1::QueriedIndexesComplete
    {
        failures.push(PoolFailureV1::SnapshotCoverageInsufficient);
    }
    if config.require_clean_projection && !state.issues.is_empty() {
        failures.push(PoolFailureV1::ProjectionContainsIssues);
    }

    let retention_need = evaluated_at_unix_ms.and_then(|now_ms| {
        match retention_need(intent.requirements.retention, now_ms) {
            Ok(need) => Some(need),
            Err(failure) => {
                failures.push(failure);
                None
            }
        }
    });

    failures.sort();
    failures.dedup();
    if !failures.is_empty() {
        return HardPolicyEvaluationV1 {
            qualified_pool: None,
            rejections: Vec::new(),
            failures,
        };
    }

    let now_ms = evaluated_at_unix_ms.expect("validated non-negative evaluation timestamp");
    let retention_need =
        retention_need.expect("retention requirement evaluated after global checks");
    let (evidence_by_advertisement, conflicted_evidence) =
        normalize_provider_evidence(provider_evidence);
    let requirements = &intent.requirements;
    let jurisdiction_constrained = !requirements.allowed_jurisdictions.is_empty()
        || !requirements.forbidden_jurisdictions.is_empty();
    let failure_domains_required = requirements.failure_domains.iter().next().is_some();
    let provider_facts_required = jurisdiction_constrained
        || provider_at_rest_required(requirements.encryption)
        || retention_need != RetentionNeedV1::None
        || failure_domains_required;

    let mut eligible = Vec::new();
    let mut rejections = Vec::new();

    for candidate in canonical_target_candidates(state, &target) {
        let mut reasons = Vec::new();
        if state.evaluated_at < candidate.claim_authored_at
            || state.evaluated_at >= candidate.effective_until
            || !matches!(candidate.withdrawal, WithdrawalObservationV1::NoWithdrawalObserved)
        {
            reasons.push(CandidateRejectionReasonV1::CandidateNotTemporallyLive);
        }
        if candidate.size_bytes != target.size_bytes {
            reasons.push(CandidateRejectionReasonV1::TargetSizeMismatch);
        }
        if client_side_required(requirements.encryption) && !target.client_side_encrypted {
            reasons.push(CandidateRejectionReasonV1::ClientSideEncryptionRequired);
        }

        let evidence = if provider_facts_required {
            if conflicted_evidence.contains(&candidate.advertisement_action) {
                reasons.push(CandidateRejectionReasonV1::ConflictingProviderPolicyEvidence);
                None
            } else {
                evidence_by_advertisement.get(&candidate.advertisement_action)
            }
        } else {
            None
        };

        if provider_facts_required
            && evidence.is_none()
            && reasons.iter().all(|reason| {
                !matches!(
                    reason,
                    CandidateRejectionReasonV1::ConflictingProviderPolicyEvidence
                )
            })
        {
            reasons.push(CandidateRejectionReasonV1::MissingProviderPolicyEvidence);
        }

        let mut accepted_jurisdictions = BTreeSet::new();
        let mut accepted_failure_domains = BTreeMap::new();

        if let Some(evidence) = evidence {
            if evidence.provider != candidate.provider
                || evidence.advertisement != candidate.advertisement_action
            {
                reasons.push(CandidateRejectionReasonV1::ProviderEvidenceIdentityMismatch);
            }
            if evidence.valid_from_unix_ms >= evidence.valid_until_unix_ms {
                reasons.push(CandidateRejectionReasonV1::InvalidProviderPolicyEvidenceWindow);
            } else if now_ms < evidence.valid_from_unix_ms
                || now_ms >= evidence.valid_until_unix_ms
            {
                reasons.push(CandidateRejectionReasonV1::ProviderPolicyEvidenceNotCurrent);
            }

            if jurisdiction_constrained {
                let all_jurisdiction_facts = &evidence.storage_jurisdictions;
                if all_jurisdiction_facts.is_empty() {
                    reasons.push(CandidateRejectionReasonV1::MissingJurisdictionEvidence);
                } else {
                    let qualifying: Vec<_> = all_jurisdiction_facts
                        .iter()
                        .filter(|fact| {
                            fact.assurance
                                .meets(config.minimum_provider_fact_assurance)
                        })
                        .collect();
                    if qualifying.is_empty() {
                        reasons.push(CandidateRejectionReasonV1::InsufficientJurisdictionAssurance);
                    }
                    for fact in qualifying {
                        if fact.jurisdiction.validate().is_err() {
                            reasons.push(CandidateRejectionReasonV1::InvalidJurisdictionEvidence);
                            continue;
                        }
                        if !requirements.jurisdiction_allowed(&fact.jurisdiction) {
                            reasons.push(CandidateRejectionReasonV1::JurisdictionNotAllowed {
                                jurisdiction: fact.jurisdiction.clone(),
                            });
                        } else {
                            accepted_jurisdictions.insert(fact.jurisdiction.clone());
                        }
                    }
                }
            }

            if provider_at_rest_required(requirements.encryption) {
                match evidence.provider_at_rest_encryption {
                    None => reasons.push(
                        CandidateRejectionReasonV1::MissingProviderAtRestEncryptionEvidence,
                    ),
                    Some(assurance)
                        if !assurance.meets(config.minimum_provider_fact_assurance) =>
                    {
                        reasons.push(
                            CandidateRejectionReasonV1::InsufficientProviderAtRestEncryptionAssurance,
                        );
                    }
                    Some(_) => {}
                }
            }

            if retention_need != RetentionNeedV1::None {
                match &evidence.retention {
                    None => reasons.push(CandidateRejectionReasonV1::MissingRetentionEvidence),
                    Some(retention)
                        if !retention
                            .assurance
                            .meets(config.minimum_provider_fact_assurance) =>
                    {
                        reasons.push(CandidateRejectionReasonV1::InsufficientRetentionAssurance);
                    }
                    Some(retention) if !retention_satisfies(retention, retention_need) => {
                        reasons.push(CandidateRejectionReasonV1::RetentionCapabilityInsufficient);
                    }
                    Some(_) => {}
                }
            }

            for (kind, _) in requirements.failure_domains.iter() {
                let all_facts: Vec<_> = evidence
                    .failure_domains
                    .iter()
                    .filter(|fact| fact.kind == *kind)
                    .collect();
                if all_facts.is_empty() {
                    reasons.push(CandidateRejectionReasonV1::MissingFailureDomainEvidence {
                        kind: *kind,
                    });
                    continue;
                }
                let qualifying: Vec<_> = all_facts
                    .iter()
                    .copied()
                    .filter(|fact| {
                        fact.assurance
                            .meets(config.minimum_provider_fact_assurance)
                    })
                    .collect();
                if qualifying.is_empty() {
                    reasons.push(
                        CandidateRejectionReasonV1::InsufficientFailureDomainAssurance {
                            kind: *kind,
                        },
                    );
                    continue;
                }
                let mut values = BTreeSet::new();
                let mut invalid_value = false;
                for fact in qualifying {
                    if !is_canonical_domain_value(&fact.value) {
                        invalid_value = true;
                    } else {
                        values.insert(fact.value.clone());
                    }
                }
                if invalid_value {
                    reasons.push(CandidateRejectionReasonV1::InvalidFailureDomainValue {
                        kind: *kind,
                    });
                    continue;
                }
                if values.len() != 1 {
                    reasons.push(CandidateRejectionReasonV1::AmbiguousFailureDomainEvidence {
                        kind: *kind,
                    });
                    continue;
                }
                let value = values
                    .into_iter()
                    .next()
                    .expect("exactly one attested domain");
                let provider_claims: BTreeSet<_> = candidate
                    .failure_domains
                    .iter()
                    .filter(|claim| claim.kind == *kind)
                    .map(|claim| claim.value.as_str())
                    .collect();
                if !provider_claims.is_empty()
                    && (provider_claims.len() != 1 || !provider_claims.contains(value.as_str()))
                {
                    reasons.push(
                        CandidateRejectionReasonV1::AttestedFailureDomainConflictsWithProviderClaim {
                            kind: *kind,
                        },
                    );
                    continue;
                }
                accepted_failure_domains.insert(*kind, value);
            }
        }

        if reasons.is_empty() {
            eligible.push(PolicyEligibleCandidateV1 {
                candidate: candidate.clone(),
                accepted_jurisdictions,
                accepted_failure_domains,
            });
        } else {
            push_rejection(&mut rejections, candidate, reasons);
        }
    }

    eligible.sort_by_key(|candidate| {
        (
            candidate.candidate.provider,
            candidate.candidate.advertisement_action,
            candidate.candidate.availability_action,
        )
    });
    rejections.sort_by_key(|rejection| {
        (
            rejection.provider,
            rejection.advertisement_action,
            rejection.availability_action,
        )
    });

    let mut pool_failures = Vec::new();
    if eligible.len() < usize::from(requirements.minimum_replicas) {
        pool_failures.push(PoolFailureV1::InsufficientEligibleReplicas {
            required: requirements.minimum_replicas,
            observed: observed_count(eligible.len()),
        });
    }
    for (kind, required) in requirements.failure_domains.iter() {
        let distinct: BTreeSet<_> = eligible
            .iter()
            .filter_map(|candidate| candidate.accepted_failure_domains.get(kind))
            .collect();
        if distinct.len() < usize::from(*required) {
            pool_failures.push(PoolFailureV1::InsufficientFailureDomainDiversity {
                kind: *kind,
                required: *required,
                observed: observed_count(distinct.len()),
            });
        }
    }
    pool_failures.sort();
    pool_failures.dedup();

    let qualified_pool = if pool_failures.is_empty() {
        Some(PolicyQualifiedPoolV1 {
            storage_intent_id: intent.id,
            target,
            evaluated_at: state.evaluated_at,
            evaluated_at_unix_ms: now_ms,
            requirements: requirements.clone(),
            candidates: eligible,
        })
    } else {
        None
    };

    HardPolicyEvaluationV1 {
        qualified_pool,
        rejections,
        failures: pool_failures,
    }
}

impl PolicyQualifiedPoolV1 {
    /// Validate a planner-selected subset before any proposal can become executable work.
    pub fn validate_selection(
        &self,
        selected_availability_actions: &[ActionRefV1],
    ) -> Result<(), SelectionPolicyErrorV1> {
        let by_action: BTreeMap<_, _> = self
            .candidates
            .iter()
            .map(|candidate| (candidate.candidate.availability_action, candidate))
            .collect();
        let mut seen = BTreeSet::new();
        let mut selected = Vec::new();

        for action in selected_availability_actions {
            if !seen.insert(*action) {
                return Err(SelectionPolicyErrorV1::DuplicateCandidate {
                    availability_action: *action,
                });
            }
            let Some(candidate) = by_action.get(action) else {
                return Err(SelectionPolicyErrorV1::UnknownCandidate {
                    availability_action: *action,
                });
            };
            selected.push(*candidate);
        }

        if selected.len() < usize::from(self.requirements.minimum_replicas) {
            return Err(SelectionPolicyErrorV1::InsufficientReplicas {
                required: self.requirements.minimum_replicas,
                selected: observed_count(selected.len()),
            });
        }

        for (kind, required) in self.requirements.failure_domains.iter() {
            let distinct: BTreeSet<_> = selected
                .iter()
                .filter_map(|candidate| candidate.accepted_failure_domains.get(kind))
                .collect();
            if distinct.len() < usize::from(*required) {
                return Err(SelectionPolicyErrorV1::InsufficientFailureDomainDiversity {
                    kind: *kind,
                    required: *required,
                    observed: observed_count(distinct.len()),
                });
            }
        }

        Ok(())
    }
}
