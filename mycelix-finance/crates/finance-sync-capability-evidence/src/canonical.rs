use mycelix_finance_sync_graph::{Commitment32, SemanticProfileRefV1};
use sha2::{Digest, Sha256};

use crate::{
    ContradictedClaimSubjectV1, EvidenceError, RailCapabilityEvidenceBundleV1,
    RailCapabilityEvidenceItemInputV1, StructuralEvidenceRecordV1,
};

pub(crate) const COMMITMENT_PROFILE_REVISION: u16 = 1;

const ITEM_DOMAIN: &[u8] = b"MYCELIX_FIN_SYNC_CAPABILITY_EVIDENCE_ITEM_V1\0";
const BUNDLE_DOMAIN: &[u8] = b"MYCELIX_FIN_SYNC_CAPABILITY_EVIDENCE_BUNDLE_V1\0";

pub(crate) fn derive_item_commitment(
    static_profile_commitment: Commitment32,
    item: &RailCapabilityEvidenceItemInputV1,
) -> Result<Commitment32, EvidenceError> {
    let mut out = Vec::with_capacity(384);
    out.extend_from_slice(ITEM_DOMAIN);
    push_u16(&mut out, COMMITMENT_PROFILE_REVISION);
    push_digest(&mut out, static_profile_commitment);
    push_u8(&mut out, item.dimension.canonical_tag());
    push_text(&mut out, item.evidence_id.as_str())?;
    push_digest(&mut out, item.evidence_commitment);
    push_profile(&mut out, &item.source_profile)?;
    push_u8(&mut out, item.claim.canonical_tag());
    push_digest(&mut out, item.claim_subject_commitment);
    push_optional_u64(&mut out, item.source_revision);
    push_optional_chronology(&mut out, item.chronology.as_ref())?;
    Ok(sha256(&out))
}

pub(crate) fn derive_bundle_commitment(
    bundle: &RailCapabilityEvidenceBundleV1,
) -> Result<Commitment32, EvidenceError> {
    let mut out = Vec::with_capacity(2304);
    out.extend_from_slice(BUNDLE_DOMAIN);
    push_u16(&mut out, COMMITMENT_PROFILE_REVISION);
    push_digest(&mut out, bundle.static_profile_commitment);
    push_profile(&mut out, &bundle.evidence_profile)?;
    push_digest(&mut out, bundle.evidence_context_commitment);

    push_count(&mut out, bundle.evidence_records.len())?;
    for record in &bundle.evidence_records {
        push_evidence_record(&mut out, record)?;
    }

    push_count(&mut out, bundle.conflicted_evidence_ids.len())?;
    for evidence_id in &bundle.conflicted_evidence_ids {
        push_text(&mut out, evidence_id.as_str())?;
    }

    push_count(&mut out, bundle.contradicted_claim_subjects.len())?;
    for subject in &bundle.contradicted_claim_subjects {
        push_contradicted_subject(&mut out, subject);
    }

    push_u8(&mut out, bundle.disposition.canonical_tag());
    Ok(sha256(&out))
}

fn push_evidence_record(
    out: &mut Vec<u8>,
    record: &StructuralEvidenceRecordV1,
) -> Result<(), EvidenceError> {
    push_u8(out, record.dimension.canonical_tag());
    push_text(out, record.evidence_id.as_str())?;
    push_digest(out, record.item_commitment);
    push_u8(out, record.claim.canonical_tag());
    push_digest(out, record.claim_subject_commitment);
    Ok(())
}

fn push_contradicted_subject(out: &mut Vec<u8>, subject: &ContradictedClaimSubjectV1) {
    push_u8(out, subject.dimension.canonical_tag());
    push_digest(out, subject.claim_subject_commitment);
}

fn push_optional_chronology(
    out: &mut Vec<u8>,
    chronology: Option<&crate::SourceChronologyEvidenceV1>,
) -> Result<(), EvidenceError> {
    match chronology {
        None => push_u8(out, 0),
        Some(chronology) => {
            push_u8(out, 1);
            push_profile(out, &chronology.chronology_profile)?;
            push_digest(out, chronology.chronology_commitment);
        }
    }
    Ok(())
}

fn push_optional_u64(out: &mut Vec<u8>, value: Option<u64>) {
    match value {
        None => push_u8(out, 0),
        Some(value) => {
            push_u8(out, 1);
            push_u64(out, value);
        }
    }
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

fn push_count(out: &mut Vec<u8>, value: usize) -> Result<(), EvidenceError> {
    let value = u32::try_from(value).map_err(|_| EvidenceError::CanonicalLengthOverflow)?;
    push_u32(out, value);
    Ok(())
}

fn push_digest(out: &mut Vec<u8>, digest: Commitment32) {
    out.extend_from_slice(digest.as_bytes());
}

fn push_text(out: &mut Vec<u8>, value: &str) -> Result<(), EvidenceError> {
    let length = u32::try_from(value.len()).map_err(|_| EvidenceError::CanonicalLengthOverflow)?;
    push_u32(out, length);
    out.extend_from_slice(value.as_bytes());
    Ok(())
}

fn push_profile(out: &mut Vec<u8>, profile: &SemanticProfileRefV1) -> Result<(), EvidenceError> {
    push_text(out, profile.id().as_str())?;
    push_u64(out, profile.revision());
    push_digest(out, profile.digest());
    Ok(())
}
