use std::collections::BTreeSet;

use mycelix_business_core::{
    ActionContractRef, AuthorizedIntentRef, Digest32, ReferenceId, ReservationId, SubjectRef,
};
use mycelix_business_decision::DecisionCapsuleRef;
use mycelix_finance_exact::AssetAmount;
use mycelix_finance_settlement::FinalityProfileRef;
use sha2::{Digest, Sha256};

pub const RESERVATION_COMMITMENT_PROFILE_REVISION: u16 = 1;
pub const RESERVATION_DESCRIPTOR_DIGEST_PROFILE: &str = "sha256";
const DESCRIPTOR_DOMAIN: &[u8] = b"MYCELIX_FINANCE_RESERVATION_DESCRIPTOR_V1\0";

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ReservationDescriptor {
    pub commitment_profile_revision: u16,
    pub reservation_id: ReservationId,
    pub request_id: ReferenceId,
    pub finance_domain: ReferenceId,
    /// Exact authority identity expected to author the runtime reservation record.
    /// FIN-ECO-004B defines the Holochain author -> canonical authority mapping.
    pub issuer_authority: ReferenceId,
    pub issuer_profile: ReferenceId,
    pub issuer_profile_revision: u64,
    pub issuer_profile_digest: Digest32,
    /// Exact Finance policy frontier under which the reservation was issued.
    pub finance_policy_sequence: u64,
    pub finance_policy_digest: Digest32,
    pub subject: SubjectRef,
    pub amount: AssetAmount,
    pub effect_class: ReferenceId,
    pub action_contract: ActionContractRef,
    pub decision: DecisionCapsuleRef,
    pub authorized_intent: AuthorizedIntentRef,
    pub intent_digest: Digest32,
    pub authority_lease_id: ReferenceId,
    pub authority_epoch: u64,
    pub fencing_token: u64,
    pub aggregate_policy_keys: BTreeSet<ReferenceId>,
    pub idempotency_key: ReferenceId,
    pub required_finality_profile: FinalityProfileRef,
    pub issued_at_unix_ms: u64,
    pub expires_at_unix_ms: u64,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ReservationDescriptorError {
    UnsupportedCommitmentProfile { expected: u16, actual: u16 },
    ZeroAmount,
    ZeroIssuerProfileRevision,
    ZeroFinancePolicySequence,
    ZeroAuthorityEpoch,
    ZeroFencingToken,
    ZeroFinalityProfileRevision,
    InvalidWindow,
    LengthOverflow,
}

impl ReservationDescriptor {
    pub fn validate(&self) -> Result<(), ReservationDescriptorError> {
        if self.commitment_profile_revision != RESERVATION_COMMITMENT_PROFILE_REVISION {
            return Err(ReservationDescriptorError::UnsupportedCommitmentProfile {
                expected: RESERVATION_COMMITMENT_PROFILE_REVISION,
                actual: self.commitment_profile_revision,
            });
        }
        if self.amount.atomic_units() == 0 {
            return Err(ReservationDescriptorError::ZeroAmount);
        }
        if self.issuer_profile_revision == 0 {
            return Err(ReservationDescriptorError::ZeroIssuerProfileRevision);
        }
        if self.finance_policy_sequence == 0 {
            return Err(ReservationDescriptorError::ZeroFinancePolicySequence);
        }
        if self.authority_epoch == 0 {
            return Err(ReservationDescriptorError::ZeroAuthorityEpoch);
        }
        if self.fencing_token == 0 {
            return Err(ReservationDescriptorError::ZeroFencingToken);
        }
        if self.required_finality_profile.revision == 0 {
            return Err(ReservationDescriptorError::ZeroFinalityProfileRevision);
        }
        if self.issued_at_unix_ms >= self.expires_at_unix_ms {
            return Err(ReservationDescriptorError::InvalidWindow);
        }
        Ok(())
    }
}

/// Sealed SHA-256 commitment over the canonical immutable reservation descriptor.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct ReservationCommitment([u8; 32]);

impl ReservationCommitment {
    pub fn as_bytes(&self) -> &[u8; 32] {
        &self.0
    }

    pub fn into_bytes(self) -> [u8; 32] {
        self.0
    }
}

pub fn canonical_descriptor_bytes(
    descriptor: &ReservationDescriptor,
) -> Result<Vec<u8>, ReservationDescriptorError> {
    descriptor.validate()?;

    let mut out = Vec::with_capacity(1280);
    out.extend_from_slice(DESCRIPTOR_DOMAIN);
    push_u16(&mut out, descriptor.commitment_profile_revision);
    push_reference(&mut out, &descriptor.reservation_id.0)?;
    push_reference(&mut out, &descriptor.request_id)?;
    push_reference(&mut out, &descriptor.finance_domain)?;
    push_reference(&mut out, &descriptor.issuer_authority)?;
    push_reference(&mut out, &descriptor.issuer_profile)?;
    push_u64(&mut out, descriptor.issuer_profile_revision);
    push_digest(&mut out, descriptor.issuer_profile_digest);
    push_u64(&mut out, descriptor.finance_policy_sequence);
    push_digest(&mut out, descriptor.finance_policy_digest);
    push_reference(&mut out, &descriptor.subject.0)?;
    push_string(&mut out, descriptor.amount.asset().as_str())?;
    push_u64(&mut out, descriptor.amount.atomic_units());
    push_reference(&mut out, &descriptor.effect_class)?;
    push_reference(&mut out, &descriptor.action_contract.semantic_id)?;
    push_digest(&mut out, descriptor.action_contract.digest);
    push_reference(&mut out, &descriptor.decision.id)?;
    push_digest(&mut out, descriptor.decision.digest);
    push_reference(&mut out, &descriptor.authorized_intent.0)?;
    push_digest(&mut out, descriptor.intent_digest);
    push_reference(&mut out, &descriptor.authority_lease_id)?;
    push_u64(&mut out, descriptor.authority_epoch);
    push_u64(&mut out, descriptor.fencing_token);
    push_reference_set(&mut out, &descriptor.aggregate_policy_keys)?;
    push_reference(&mut out, &descriptor.idempotency_key)?;
    push_reference(&mut out, &descriptor.required_finality_profile.id)?;
    push_u64(&mut out, descriptor.required_finality_profile.revision);
    push_digest(&mut out, descriptor.required_finality_profile.digest);
    push_u64(&mut out, descriptor.issued_at_unix_ms);
    push_u64(&mut out, descriptor.expires_at_unix_ms);

    Ok(out)
}

pub fn reservation_commitment(
    descriptor: &ReservationDescriptor,
) -> Result<ReservationCommitment, ReservationDescriptorError> {
    let bytes = canonical_descriptor_bytes(descriptor)?;
    let digest = Sha256::digest(bytes);
    let mut out = [0_u8; 32];
    out.copy_from_slice(&digest);
    Ok(ReservationCommitment(out))
}

pub(crate) fn push_u8(out: &mut Vec<u8>, value: u8) {
    out.push(value);
}

pub(crate) fn push_u16(out: &mut Vec<u8>, value: u16) {
    out.extend_from_slice(&value.to_be_bytes());
}

pub(crate) fn push_u32(out: &mut Vec<u8>, value: u32) {
    out.extend_from_slice(&value.to_be_bytes());
}

pub(crate) fn push_u64(out: &mut Vec<u8>, value: u64) {
    out.extend_from_slice(&value.to_be_bytes());
}

pub(crate) fn push_digest(out: &mut Vec<u8>, digest: Digest32) {
    out.extend_from_slice(&digest.0);
}

pub(crate) fn push_string(
    out: &mut Vec<u8>,
    value: &str,
) -> Result<(), ReservationDescriptorError> {
    let length = u32::try_from(value.len()).map_err(|_| ReservationDescriptorError::LengthOverflow)?;
    push_u32(out, length);
    out.extend_from_slice(value.as_bytes());
    Ok(())
}

pub(crate) fn push_reference(
    out: &mut Vec<u8>,
    value: &ReferenceId,
) -> Result<(), ReservationDescriptorError> {
    push_string(out, value.as_str())
}

fn push_reference_set(
    out: &mut Vec<u8>,
    values: &BTreeSet<ReferenceId>,
) -> Result<(), ReservationDescriptorError> {
    let count = u32::try_from(values.len()).map_err(|_| ReservationDescriptorError::LengthOverflow)?;
    push_u32(out, count);
    for value in values {
        push_reference(out, value)?;
    }
    Ok(())
}
