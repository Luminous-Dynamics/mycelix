use mycelix_finance_sync_graph::{Commitment32, SemanticProfileRefV1};

use crate::canonical::derive_profile_commitment;
use crate::{
    AtomicityCapabilityV1, CancelCapabilityV1, CapabilityError, CapacityLockCapabilityV1,
    CommitCapabilityV1, EvidenceCapabilityV1, IdempotencyCollisionBehaviorV1,
    IdempotencyKeyScopeV1, IdempotencyMechanismV1, IdempotencyProfileV1,
    IdempotencySemanticScopeV1, PrepareCapabilityV1, QueryCapabilityV1,
    RailCapabilityProfileInputV1, RailCapabilityProfileV1, RetentionHorizonV1,
    ReversalCapabilityV1, UnknownOutcomeRetryV1, MAX_CAPABILITIES_PER_DIMENSION,
    MAX_PROFILE_REFS,
};

pub fn build_rail_capability_profile_v1(
    input: RailCapabilityProfileInputV1,
) -> Result<RailCapabilityProfileV1, CapabilityError> {
    let RailCapabilityProfileInputV1 {
        adapter_profile,
        adapter_build_profile,
        provider_profile,
        rail,
        network,
        operation_profile,
        mut capacity_lock_capabilities,
        mut prepare_capabilities,
        mut commit_capabilities,
        mut cancel_capabilities,
        mut query_capabilities,
        mut evidence_capabilities,
        mut reversal_capabilities,
        mut atomicity_capabilities,
        idempotency,
        mut producible_finality_profiles,
        mut synchronization_profiles,
        disclosure_profile,
        timing,
        resources,
        profile_revision,
    } = input;

    normalize_dimension(
        &mut capacity_lock_capabilities,
        CapacityLockCapabilityV1::canonical_tag,
    )?;
    normalize_dimension(&mut prepare_capabilities, PrepareCapabilityV1::canonical_tag)?;
    normalize_dimension(&mut commit_capabilities, CommitCapabilityV1::canonical_tag)?;
    normalize_dimension(&mut cancel_capabilities, CancelCapabilityV1::canonical_tag)?;
    normalize_dimension(&mut query_capabilities, QueryCapabilityV1::canonical_tag)?;
    normalize_dimension(
        &mut evidence_capabilities,
        EvidenceCapabilityV1::canonical_tag,
    )?;
    normalize_dimension(
        &mut reversal_capabilities,
        ReversalCapabilityV1::canonical_tag,
    )?;
    normalize_dimension(
        &mut atomicity_capabilities,
        AtomicityCapabilityV1::canonical_tag,
    )?;

    reject_negative_with_positive(
        &capacity_lock_capabilities,
        CapacityLockCapabilityV1::NoCapacityLock,
    )?;
    reject_negative_with_positive(&prepare_capabilities, PrepareCapabilityV1::NoPrepare)?;
    reject_negative_with_positive(
        &commit_capabilities,
        CommitCapabilityV1::NoCommitPrimitive,
    )?;
    reject_negative_with_positive(&cancel_capabilities, CancelCapabilityV1::NotCancelable)?;
    reject_negative_with_positive(&query_capabilities, QueryCapabilityV1::NoQueryPrimitive)?;
    reject_negative_with_positive(
        &evidence_capabilities,
        EvidenceCapabilityV1::NoStructuredEvidence,
    )?;
    reject_negative_with_positive(
        &reversal_capabilities,
        ReversalCapabilityV1::NoProtocolReversal,
    )?;
    reject_negative_with_positive(
        &atomicity_capabilities,
        AtomicityCapabilityV1::NoMultiLegAtomicity,
    )?;

    normalize_profile_refs(&mut producible_finality_profiles)?;
    normalize_profile_refs(&mut synchronization_profiles)?;

    validate_cross_dimension_semantics(
        &prepare_capabilities,
        &commit_capabilities,
        &cancel_capabilities,
        &evidence_capabilities,
        &atomicity_capabilities,
        &producible_finality_profiles,
        &synchronization_profiles,
    )?;
    validate_idempotency(&idempotency)?;
    validate_timing(&timing)?;
    validate_resources(&resources)?;
    if profile_revision == 0 {
        return Err(CapabilityError::InvalidProfileRevision);
    }

    let mut profile = RailCapabilityProfileV1 {
        adapter_profile,
        adapter_build_profile,
        provider_profile,
        rail,
        network,
        operation_profile,
        capacity_lock_capabilities,
        prepare_capabilities,
        commit_capabilities,
        cancel_capabilities,
        query_capabilities,
        evidence_capabilities,
        reversal_capabilities,
        atomicity_capabilities,
        idempotency,
        producible_finality_profiles,
        synchronization_profiles,
        disclosure_profile,
        timing,
        resources,
        profile_revision,
        profile_commitment: Commitment32::from_bytes([0_u8; 32]),
    };
    profile.profile_commitment = derive_profile_commitment(&profile)?;
    Ok(profile)
}

fn normalize_dimension<T>(values: &mut Vec<T>, tag: fn(T) -> u8) -> Result<(), CapabilityError>
where
    T: Copy + PartialEq,
{
    if values.is_empty() {
        return Err(CapabilityError::EmptyCapabilityDimension);
    }
    if values.len() > MAX_CAPABILITIES_PER_DIMENSION {
        return Err(CapabilityError::TooManyCapabilities);
    }
    values.sort_by_key(|value| tag(*value));
    if values.windows(2).any(|pair| pair[0] == pair[1]) {
        return Err(CapabilityError::DuplicateCapability);
    }
    Ok(())
}

fn reject_negative_with_positive<T>(values: &[T], negative: T) -> Result<(), CapabilityError>
where
    T: Copy + PartialEq,
{
    if values.len() > 1 && values.contains(&negative) {
        Err(CapabilityError::ContradictoryCapabilityDimension)
    } else {
        Ok(())
    }
}

fn normalize_profile_refs(
    profiles: &mut Vec<SemanticProfileRefV1>,
) -> Result<(), CapabilityError> {
    if profiles.len() > MAX_PROFILE_REFS {
        return Err(CapabilityError::TooManyProfileRefs);
    }
    profiles.sort_by_key(profile_sort_key);
    if profiles.windows(2).any(|pair| pair[0] == pair[1]) {
        return Err(CapabilityError::DuplicateProfileRef);
    }
    Ok(())
}

fn profile_sort_key(profile: &SemanticProfileRefV1) -> Vec<u8> {
    let id = profile.id().as_str().as_bytes();
    let mut out = Vec::with_capacity(4 + id.len() + 8 + 32);
    out.extend_from_slice(&(id.len() as u32).to_be_bytes());
    out.extend_from_slice(id);
    out.extend_from_slice(&profile.revision().to_be_bytes());
    out.extend_from_slice(profile.digest().as_bytes());
    out
}

fn validate_cross_dimension_semantics(
    prepare: &[PrepareCapabilityV1],
    commit: &[CommitCapabilityV1],
    cancel: &[CancelCapabilityV1],
    evidence: &[EvidenceCapabilityV1],
    atomicity: &[AtomicityCapabilityV1],
    finality_profiles: &[SemanticProfileRefV1],
    synchronization_profiles: &[SemanticProfileRefV1],
) -> Result<(), CapabilityError> {
    let supports_prepared_state = prepare.iter().any(|capability| {
        matches!(
            capability,
            PrepareCapabilityV1::PreparedOperationHandle
                | PrepareCapabilityV1::AtomicTransactionPrepare
        )
    });

    if commit.contains(&CommitCapabilityV1::CommitAgainstPreparedState)
        && !supports_prepared_state
    {
        return Err(CapabilityError::InvalidPreparedStateCombination);
    }
    if cancel.iter().any(|capability| {
        matches!(
            capability,
            CancelCapabilityV1::CancelableWhilePrepared
                | CancelCapabilityV1::GuaranteedAbortOfPreparedState
        )
    }) && !supports_prepared_state
    {
        return Err(CapabilityError::InvalidPreparedStateCombination);
    }

    let sync_commit = commit.contains(&CommitCapabilityV1::ExternallySynchronizedCommit);
    let sync_atomicity = atomicity.contains(&AtomicityCapabilityV1::ExternalSynchronizationProtocol);
    let sync_profiles = !synchronization_profiles.is_empty();
    if !(sync_commit == sync_atomicity && sync_atomicity == sync_profiles) {
        return Err(CapabilityError::InvalidSynchronizationCombination);
    }

    let finality_evidence = evidence.contains(&EvidenceCapabilityV1::FinalityEvidenceProduction);
    if finality_evidence != !finality_profiles.is_empty() {
        return Err(CapabilityError::InvalidFinalityCombination);
    }

    let single_owner_commit = commit.contains(&CommitCapabilityV1::AtomicCommitWithinSingleOwner);
    let single_owner_atomicity =
        atomicity.contains(&AtomicityCapabilityV1::MultiInstructionSingleTransactionOwner);
    if single_owner_commit != single_owner_atomicity {
        return Err(CapabilityError::InvalidAtomicityCombination);
    }

    Ok(())
}

fn validate_idempotency(profile: &IdempotencyProfileV1) -> Result<(), CapabilityError> {
    let retention_is_guaranteed = match &profile.retention {
        RetentionHorizonV1::NotGuaranteed => false,
        RetentionHorizonV1::BoundedSeconds { seconds } => {
            if *seconds == 0 {
                return Err(CapabilityError::InvalidIdempotencyProfile);
            }
            true
        }
        RetentionHorizonV1::ProfileBounded { .. } => true,
    };

    match profile.mechanism {
        IdempotencyMechanismV1::NoGuarantee => {
            if profile.key_scope != IdempotencyKeyScopeV1::None
                || profile.semantic_scope != IdempotencySemanticScopeV1::None
                || retention_is_guaranteed
                || profile.retry_after_unknown != UnknownOutcomeRetryV1::NeverBlindRetry
                || profile.collision_behavior != IdempotencyCollisionBehaviorV1::Undefined
            {
                return Err(CapabilityError::InvalidIdempotencyProfile);
            }
        }
        IdempotencyMechanismV1::ClientKeyBestEffort => {
            require_nonempty_idempotency_scope(profile)?;
            if matches!(
                profile.retry_after_unknown,
                UnknownOutcomeRetryV1::ReplaySameSemanticOperationWithinHorizon
                    | UnknownOutcomeRetryV1::ProviderNativeSafeReplay
            ) {
                return Err(CapabilityError::InvalidIdempotencyProfile);
            }
        }
        IdempotencyMechanismV1::ProviderKeyDedupWindow => {
            require_nonempty_idempotency_scope(profile)?;
            if !matches!(profile.retention, RetentionHorizonV1::BoundedSeconds { .. })
                || profile.collision_behavior == IdempotencyCollisionBehaviorV1::Undefined
                || profile.retry_after_unknown == UnknownOutcomeRetryV1::ProviderNativeSafeReplay
            {
                return Err(CapabilityError::InvalidIdempotencyProfile);
            }
        }
        IdempotencyMechanismV1::ProviderKeyExactlyOnceWithinProfile => {
            require_nonempty_idempotency_scope(profile)?;
            if !retention_is_guaranteed
                || profile.collision_behavior == IdempotencyCollisionBehaviorV1::Undefined
                || profile.retry_after_unknown == UnknownOutcomeRetryV1::ProviderNativeSafeReplay
            {
                return Err(CapabilityError::InvalidIdempotencyProfile);
            }
        }
        IdempotencyMechanismV1::NativeSemanticOperationIdentity => {
            if profile.key_scope != IdempotencyKeyScopeV1::NativeOperation
                || profile.semantic_scope != IdempotencySemanticScopeV1::NativeOperation
                || !retention_is_guaranteed
                || profile.collision_behavior
                    != IdempotencyCollisionBehaviorV1::NativeIdentityCannotBeRepurposed
                || !matches!(
                    profile.retry_after_unknown,
                    UnknownOutcomeRetryV1::QueryBeforeReplay
                        | UnknownOutcomeRetryV1::ProviderNativeSafeReplay
                )
            {
                return Err(CapabilityError::InvalidIdempotencyProfile);
            }
        }
    }

    if profile.retry_after_unknown
        == UnknownOutcomeRetryV1::ReplaySameSemanticOperationWithinHorizon
        && !retention_is_guaranteed
    {
        return Err(CapabilityError::InvalidIdempotencyProfile);
    }
    if profile.collision_behavior == IdempotencyCollisionBehaviorV1::NativeIdentityCannotBeRepurposed
        && profile.mechanism != IdempotencyMechanismV1::NativeSemanticOperationIdentity
    {
        return Err(CapabilityError::InvalidIdempotencyProfile);
    }

    Ok(())
}

fn require_nonempty_idempotency_scope(
    profile: &IdempotencyProfileV1,
) -> Result<(), CapabilityError> {
    if profile.key_scope == IdempotencyKeyScopeV1::None
        || profile.semantic_scope == IdempotencySemanticScopeV1::None
    {
        Err(CapabilityError::InvalidIdempotencyProfile)
    } else {
        Ok(())
    }
}

fn validate_timing(profile: &crate::TimingProfileV1) -> Result<(), CapabilityError> {
    if profile.max_provider_deadline_ms == 0
        || profile.max_prepare_lifetime_seconds == Some(0)
        || profile.min_query_poll_interval_ms == Some(0)
        || profile.max_query_attempts_per_window == Some(0)
    {
        Err(CapabilityError::InvalidTimingProfile)
    } else {
        Ok(())
    }
}

fn validate_resources(profile: &crate::ResourceProfileV1) -> Result<(), CapabilityError> {
    if profile.max_request_bytes == 0
        || profile.max_batch_items == 0
        || profile.max_inflight_per_subject == 0
    {
        Err(CapabilityError::InvalidResourceProfile)
    } else {
        Ok(())
    }
}
