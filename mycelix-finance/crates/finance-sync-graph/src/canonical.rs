use sha2::{Digest, Sha256};

use crate::{
    Commitment32, CoordinationGroupClassV1, GraphError, SemanticProfileRefV1,
    SettlementDependencyV1, SettlementGraphV1, SettlementLegSpecV1,
};

pub(crate) const COMMITMENT_PROFILE_REVISION: u16 = 1;

const LEG_DOMAIN: &[u8] = b"MYCELIX_FIN_SYNC_LEG_V1\0";
const GROUP_DOMAIN: &[u8] = b"MYCELIX_FIN_SYNC_GROUP_V1\0";
const GRAPH_DOMAIN: &[u8] = b"MYCELIX_FIN_SYNC_GRAPH_V1\0";

pub(crate) fn derive_leg_commitment(
    economic_effect_commitment: Commitment32,
    graph_profile: &SemanticProfileRefV1,
    leg: &SettlementLegSpecV1,
) -> Result<Commitment32, GraphError> {
    Ok(sha256(&canonical_leg_bytes(
        economic_effect_commitment,
        graph_profile,
        leg,
    )?))
}

fn canonical_leg_bytes(
    economic_effect_commitment: Commitment32,
    graph_profile: &SemanticProfileRefV1,
    leg: &SettlementLegSpecV1,
) -> Result<Vec<u8>, GraphError> {
    let mut out = Vec::with_capacity(512);
    out.extend_from_slice(LEG_DOMAIN);
    push_u16(&mut out, COMMITMENT_PROFILE_REVISION);
    push_digest(&mut out, economic_effect_commitment);
    push_profile(&mut out, graph_profile)?;
    push_profile(&mut out, &leg.adapter_profile)?;
    push_text(&mut out, leg.rail.as_str())?;
    push_text(&mut out, leg.network.as_str())?;
    push_u8(&mut out, leg.role.canonical_tag());
    push_text(&mut out, leg.source_subject.as_str())?;
    push_text(&mut out, leg.destination_subject.as_str())?;
    push_text(&mut out, leg.amount.asset().as_str())?;
    push_u64(&mut out, leg.amount.atomic_units());
    push_profile(&mut out, &leg.asset_unit_profile)?;
    push_profile(&mut out, &leg.required_finality_profile)?;
    push_text(&mut out, leg.semantic_idempotency_ref.as_str())?;
    push_optional_profile(&mut out, leg.purpose_profile.as_ref())?;
    push_optional_text(&mut out, leg.delivery_asset_subject.as_ref())?;
    Ok(out)
}

pub(crate) fn derive_group_commitment(
    graph_profile: &SemanticProfileRefV1,
    class: CoordinationGroupClassV1,
    coordination_profile: &SemanticProfileRefV1,
    member_leg_ids: &[Commitment32],
) -> Result<Commitment32, GraphError> {
    let mut out = Vec::with_capacity(256);
    out.extend_from_slice(GROUP_DOMAIN);
    push_u16(&mut out, COMMITMENT_PROFILE_REVISION);
    push_profile(&mut out, graph_profile)?;
    push_u8(&mut out, class.canonical_tag());
    push_count(&mut out, member_leg_ids.len())?;
    for member in member_leg_ids {
        push_digest(&mut out, *member);
    }
    push_profile(&mut out, coordination_profile)?;
    Ok(sha256(&out))
}

pub(crate) fn canonical_dependency_bytes(
    dependency: &SettlementDependencyV1,
) -> Result<Vec<u8>, GraphError> {
    let mut out = Vec::with_capacity(128);
    match dependency {
        SettlementDependencyV1::Requires {
            leg_id,
            prerequisite_id,
        } => {
            push_u8(&mut out, 1);
            push_digest(&mut out, *leg_id);
            push_digest(&mut out, *prerequisite_id);
        }
        SettlementDependencyV1::Before {
            before_id,
            after_id,
        } => {
            push_u8(&mut out, 2);
            push_digest(&mut out, *before_id);
            push_digest(&mut out, *after_id);
        }
        SettlementDependencyV1::ConditionalOnEvidence {
            leg_id,
            predicate_profile,
        } => {
            push_u8(&mut out, 3);
            push_digest(&mut out, *leg_id);
            push_profile(&mut out, predicate_profile)?;
        }
    }
    Ok(out)
}

pub(crate) fn derive_graph_commitment(
    graph: &SettlementGraphV1,
) -> Result<Commitment32, GraphError> {
    Ok(sha256(&canonical_graph_bytes(graph)?))
}

fn canonical_graph_bytes(graph: &SettlementGraphV1) -> Result<Vec<u8>, GraphError> {
    let mut out = Vec::with_capacity(1024);
    out.extend_from_slice(GRAPH_DOMAIN);
    push_u16(&mut out, COMMITMENT_PROFILE_REVISION);
    push_digest(&mut out, graph.economic_effect_commitment);
    push_profile(&mut out, &graph.graph_profile)?;
    push_optional_profile(&mut out, graph.temporal_profile.as_ref())?;

    let mut leg_ids: Vec<_> = graph.legs.iter().map(|leg| leg.leg_id).collect();
    leg_ids.sort_unstable();
    push_count(&mut out, leg_ids.len())?;
    for leg_id in leg_ids {
        push_digest(&mut out, leg_id);
    }

    let mut dependency_bytes = Vec::with_capacity(graph.dependencies.len());
    for dependency in &graph.dependencies {
        dependency_bytes.push(canonical_dependency_bytes(dependency)?);
    }
    dependency_bytes.sort();
    push_count(&mut out, dependency_bytes.len())?;
    for dependency in dependency_bytes {
        out.extend_from_slice(&dependency);
    }

    let mut group_ids: Vec<_> = graph
        .coordination_groups
        .iter()
        .map(|group| group.group_id)
        .collect();
    group_ids.sort_unstable();
    push_count(&mut out, group_ids.len())?;
    for group_id in group_ids {
        push_digest(&mut out, group_id);
    }

    Ok(out)
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

fn push_count(out: &mut Vec<u8>, value: usize) -> Result<(), GraphError> {
    let value = u32::try_from(value).map_err(|_| GraphError::CanonicalLengthOverflow)?;
    push_u32(out, value);
    Ok(())
}

fn push_digest(out: &mut Vec<u8>, digest: Commitment32) {
    out.extend_from_slice(digest.as_bytes());
}

fn push_text(out: &mut Vec<u8>, value: &str) -> Result<(), GraphError> {
    let length = u32::try_from(value.len()).map_err(|_| GraphError::CanonicalLengthOverflow)?;
    push_u32(out, length);
    out.extend_from_slice(value.as_bytes());
    Ok(())
}

fn push_profile(
    out: &mut Vec<u8>,
    profile: &SemanticProfileRefV1,
) -> Result<(), GraphError> {
    push_text(out, profile.id().as_str())?;
    push_u64(out, profile.revision());
    push_digest(out, profile.digest());
    Ok(())
}

fn push_optional_profile(
    out: &mut Vec<u8>,
    profile: Option<&SemanticProfileRefV1>,
) -> Result<(), GraphError> {
    match profile {
        Some(profile) => {
            push_u8(out, 1);
            push_profile(out, profile)?;
        }
        None => push_u8(out, 0),
    }
    Ok(())
}

fn push_optional_text(
    out: &mut Vec<u8>,
    value: Option<&crate::BoundedText>,
) -> Result<(), GraphError> {
    match value {
        Some(value) => {
            push_u8(out, 1);
            push_text(out, value.as_str())?;
        }
        None => push_u8(out, 0),
    }
    Ok(())
}
