use mycelix_finance_sync_graph::{Commitment32, SemanticProfileRefV1};
use sha2::{Digest, Sha256};

use crate::{
    AtomicityCapabilityV1, CancelCapabilityV1, CapabilityError, CapacityLockCapabilityV1,
    CommitCapabilityV1, EvidenceCapabilityV1, IdempotencyMechanismV1, PrepareCapabilityV1,
    QueryCapabilityV1, RailCapabilityProfileV1, RetentionHorizonV1, ReversalCapabilityV1,
};

pub(crate) const COMMITMENT_PROFILE_REVISION: u16 = 1;
const PROFILE_DOMAIN: &[u8] = b"MYCELIX_FIN_SYNC_RAIL_CAPABILITY_V1\0";

pub(crate) fn derive_profile_commitment(
    profile: &RailCapabilityProfileV1,
) -> Result<Commitment32, CapabilityError> {
    let mut out = Vec::with_capacity(2048);
    out.extend_from_slice(PROFILE_DOMAIN);
    push_u16(&mut out, COMMITMENT_PROFILE_REVISION);

    push_profile(&mut out, &profile.adapter_profile)?;
    push_profile(&mut out, &profile.adapter_build_profile)?;
    push_profile(&mut out, &profile.provider_profile)?;
    push_text(&mut out, profile.rail.as_str())?;
    push_text(&mut out, profile.network.as_str())?;
    push_profile(&mut out, &profile.operation_profile)?;

    push_capabilities(
        &mut out,
        &profile.capacity_lock_capabilities,
        CapacityLockCapabilityV1::canonical_tag,
    )?;
    push_capabilities(
        &mut out,
        &profile.prepare_capabilities,
        PrepareCapabilityV1::canonical_tag,
    )?;
    push_capabilities(
        &mut out,
        &profile.commit_capabilities,
        CommitCapabilityV1::canonical_tag,
    )?;
    push_capabilities(
        &mut out,
        &profile.cancel_capabilities,
        CancelCapabilityV1::canonical_tag,
    )?;
    push_capabilities(
        &mut out,
        &profile.query_capabilities,
        QueryCapabilityV1::canonical_tag,
    )?;
    push_capabilities(
        &mut out,
        &profile.evidence_capabilities,
        EvidenceCapabilityV1::canonical_tag,
    )?;
    push_capabilities(
        &mut out,
        &profile.reversal_capabilities,
        ReversalCapabilityV1::canonical_tag,
    )?;
    push_capabilities(
        &mut out,
        &profile.atomicity_capabilities,
        AtomicityCapabilityV1::canonical_tag,
    )?;

    push_u8(&mut out, profile.idempotency.mechanism.canonical_tag());
    push_u8(&mut out, profile.idempotency.key_scope.canonical_tag());
    push_u8(&mut out, profile.idempotency.semantic_scope.canonical_tag());
    push_retention(&mut out, &profile.idempotency.retention)?;
    push_u8(
        &mut out,
        profile.idempotency.retry_after_unknown.canonical_tag(),
    );
    push_u8(
        &mut out,
        profile.idempotency.collision_behavior.canonical_tag(),
    );

    push_profile_collection(&mut out, &profile.producible_finality_profiles)?;
    push_profile_collection(&mut out, &profile.synchronization_profiles)?;
    push_profile(&mut out, &profile.disclosure_profile)?;

    push_u64(&mut out, profile.timing.max_provider_deadline_ms);
    push_optional_u64(&mut out, profile.timing.max_prepare_lifetime_seconds);
    push_optional_u64(&mut out, profile.timing.min_query_poll_interval_ms);
    push_optional_u32(&mut out, profile.timing.max_query_attempts_per_window);

    push_u32(&mut out, profile.resources.max_request_bytes);
    push_u32(&mut out, profile.resources.max_batch_items);
    push_u32(&mut out, profile.resources.max_inflight_per_subject);
    push_u64(&mut out, profile.profile_revision);

    Ok(sha256(&out))
}

fn push_capabilities<T: Copy>(
    out: &mut Vec<u8>,
    values: &[T],
    tag: fn(T) -> u8,
) -> Result<(), CapabilityError> {
    push_count(out, values.len())?;
    for value in values {
        push_u8(out, tag(*value));
    }
    Ok(())
}

fn push_profile_collection(
    out: &mut Vec<u8>,
    profiles: &[SemanticProfileRefV1],
) -> Result<(), CapabilityError> {
    push_count(out, profiles.len())?;
    for profile in profiles {
        push_profile(out, profile)?;
    }
    Ok(())
}

fn push_retention(
    out: &mut Vec<u8>,
    retention: &RetentionHorizonV1,
) -> Result<(), CapabilityError> {
    match retention {
        RetentionHorizonV1::NotGuaranteed => push_u8(out, 1),
        RetentionHorizonV1::BoundedSeconds { seconds } => {
            push_u8(out, 2);
            push_u64(out, *seconds);
        }
        RetentionHorizonV1::ProfileBounded { profile } => {
            push_u8(out, 3);
            push_profile(out, profile)?;
        }
    }
    Ok(())
}

fn sha256(bytes: &[u8]) -> Commitment32 {
    let digest = Sha256::digest(bytes);
    let mut out = [0_u8; 32];
    out.copy_from_slice(&digest);
    Commitment32::from_bytes(out)
}

fn push_u8(out: &mut Vec<u8>, value: u8) {
    out.push(value);
}

fn push_u16(out: &mut Vec<u8>, value: u16) {
    out.extend_from_slice(&value.to_be_bytes());
}

fn push_u32(out: &mut Vec<u8>, value: u32) {
    out.extend_from_slice(&value.to_be_bytes());
}

fn push_u64(out: &mut Vec<u8>, value: u64) {
    out.extend_from_slice(&value.to_be_bytes());
}

fn push_count(out: &mut Vec<u8>, value: usize) -> Result<(), CapabilityError> {
    let value = u32::try_from(value).map_err(|_| CapabilityError::CanonicalLengthOverflow)?;
    push_u32(out, value);
    Ok(())
}

fn push_optional_u64(out: &mut Vec<u8>, value: Option<u64>) {
    match value {
        Some(value) => {
            push_u8(out, 1);
            push_u64(out, value);
        }
        None => push_u8(out, 0),
    }
}

fn push_optional_u32(out: &mut Vec<u8>, value: Option<u32>) {
    match value {
        Some(value) => {
            push_u8(out, 1);
            push_u32(out, value);
        }
        None => push_u8(out, 0),
    }
}

fn push_text(out: &mut Vec<u8>, value: &str) -> Result<(), CapabilityError> {
    let length = u32::try_from(value.len()).map_err(|_| CapabilityError::CanonicalLengthOverflow)?;
    push_u32(out, length);
    out.extend_from_slice(value.as_bytes());
    Ok(())
}

fn push_profile(
    out: &mut Vec<u8>,
    profile: &SemanticProfileRefV1,
) -> Result<(), CapabilityError> {
    push_text(out, profile.id().as_str())?;
    push_u64(out, profile.revision());
    out.extend_from_slice(profile.digest().as_bytes());
    Ok(())
}
