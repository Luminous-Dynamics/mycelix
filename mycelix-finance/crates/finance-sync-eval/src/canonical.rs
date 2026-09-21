use mycelix_finance_sync_graph::{Commitment32, SemanticProfileRefV1};
use sha2::{Digest, Sha256};

use crate::{
    EvalError, LegObservationInputV1, LegObservedDispositionV1, ObservedGraphReceiptV1,
    SelectedObservationStreamV1,
};

pub(crate) const COMMITMENT_PROFILE_REVISION: u16 = 1;

const OBSERVATION_DOMAIN: &[u8] = b"MYCELIX_FIN_SYNC_UNQUALIFIED_OBSERVATION_V1\0";
const RECEIPT_DOMAIN: &[u8] = b"MYCELIX_FIN_SYNC_OBSERVED_GRAPH_RECEIPT_V1\0";

pub(crate) fn derive_observation_commitment(
    graph_commitment: Commitment32,
    observation: &LegObservationInputV1,
) -> Result<Commitment32, EvalError> {
    let mut out = Vec::with_capacity(384);
    out.extend_from_slice(OBSERVATION_DOMAIN);
    push_u16(&mut out, COMMITMENT_PROFILE_REVISION);
    push_digest(&mut out, graph_commitment);
    push_digest(&mut out, observation.leg_id);
    push_text(&mut out, observation.observation_stream_ref.as_str())?;
    push_text(&mut out, observation.evidence_id.as_str())?;
    push_digest(&mut out, observation.evidence_commitment);
    push_u64(&mut out, observation.observation_revision);
    push_u8(&mut out, observation.class.canonical_tag());
    push_profile(&mut out, &observation.observation_profile)?;
    Ok(sha256(&out))
}

pub(crate) fn derive_receipt_commitment(
    receipt: &ObservedGraphReceiptV1,
) -> Result<Commitment32, EvalError> {
    let mut out = Vec::with_capacity(1536);
    out.extend_from_slice(RECEIPT_DOMAIN);
    push_u16(&mut out, COMMITMENT_PROFILE_REVISION);
    push_digest(&mut out, receipt.graph_commitment);
    push_profile(&mut out, &receipt.evaluation_profile)?;
    push_digest(&mut out, receipt.evaluation_context_commitment);

    push_count(&mut out, receipt.observation_commitments.len())?;
    for commitment in &receipt.observation_commitments {
        push_digest(&mut out, *commitment);
    }

    push_count(&mut out, receipt.leg_dispositions.len())?;
    for disposition in &receipt.leg_dispositions {
        push_leg_disposition(&mut out, disposition)?;
    }

    push_u8(&mut out, receipt.disposition.canonical_tag());
    Ok(sha256(&out))
}

fn push_leg_disposition(
    out: &mut Vec<u8>,
    disposition: &LegObservedDispositionV1,
) -> Result<(), EvalError> {
    push_digest(out, disposition.leg_id);
    push_u8(out, disposition.state.canonical_tag());
    push_count(out, disposition.selected_streams.len())?;
    for stream in &disposition.selected_streams {
        push_selected_stream(out, stream)?;
    }
    Ok(())
}

fn push_selected_stream(
    out: &mut Vec<u8>,
    stream: &SelectedObservationStreamV1,
) -> Result<(), EvalError> {
    push_text(out, stream.observation_stream_ref.as_str())?;
    push_u64(out, stream.selected_revision);
    push_u8(out, stream.class.canonical_tag());
    push_profile(out, &stream.observation_profile)?;
    push_count(out, stream.selected_observation_commitments.len())?;
    for commitment in &stream.selected_observation_commitments {
        push_digest(out, *commitment);
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

fn push_count(out: &mut Vec<u8>, value: usize) -> Result<(), EvalError> {
    let value = u32::try_from(value).map_err(|_| EvalError::CanonicalLengthOverflow)?;
    push_u32(out, value);
    Ok(())
}

fn push_digest(out: &mut Vec<u8>, digest: Commitment32) {
    out.extend_from_slice(digest.as_bytes());
}

fn push_text(out: &mut Vec<u8>, value: &str) -> Result<(), EvalError> {
    let length = u32::try_from(value.len()).map_err(|_| EvalError::CanonicalLengthOverflow)?;
    push_u32(out, length);
    out.extend_from_slice(value.as_bytes());
    Ok(())
}

fn push_profile(out: &mut Vec<u8>, profile: &SemanticProfileRefV1) -> Result<(), EvalError> {
    push_text(out, profile.id().as_str())?;
    push_u64(out, profile.revision());
    push_digest(out, profile.digest());
    Ok(())
}
