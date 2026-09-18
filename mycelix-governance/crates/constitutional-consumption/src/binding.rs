use crate::{ConsumptionClaim, ConsumptionError};
use constitutional_envelope::MatterId;
use serde::{Deserialize, Serialize};

pub const CLAIM_BINDING_SCHEMA_VERSION: u16 = 1;
pub const CLAIM_BINDING_DOMAIN_SEPARATOR: &[u8] =
    b"MYCELIX-CONSTITUTIONAL-CLAIM-BINDING\0V1\0";

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq, Hash)]
pub struct ClaimBinding {
    pub schema_version: u16,
    pub claim_id: String,
    pub envelope_digest: String,
    pub nonce: String,
    pub use_index: u32,
    pub jurisdiction: String,
    pub matter: MatterId,
    pub target_digest: String,
    pub payload_digest: String,
    pub budget_id: String,
}

impl ClaimBinding {
    pub fn from_claim(claim: &ConsumptionClaim) -> Self {
        Self {
            schema_version: CLAIM_BINDING_SCHEMA_VERSION,
            claim_id: claim.claim_id.clone(),
            envelope_digest: claim.key.envelope_digest.clone(),
            nonce: claim.key.nonce.clone(),
            use_index: claim.key.use_index,
            jurisdiction: claim.key.jurisdiction.clone(),
            matter: claim.matter.clone(),
            target_digest: claim.target_digest.clone(),
            payload_digest: claim.payload_digest.clone(),
            budget_id: claim.budget_id.clone(),
        }
    }

    pub fn validate(&self) -> Result<(), ConsumptionError> {
        if self.schema_version != CLAIM_BINDING_SCHEMA_VERSION {
            return Err(ConsumptionError::UnsupportedClaimBindingVersion);
        }
        if self.claim_id.trim().is_empty() {
            return Err(ConsumptionError::EmptyClaimId);
        }
        if self.envelope_digest.trim().is_empty() {
            return Err(ConsumptionError::EmptyEnvelopeDigest);
        }
        if self.nonce.trim().is_empty() {
            return Err(ConsumptionError::EmptyNonce);
        }
        if self.jurisdiction.trim().is_empty() {
            return Err(ConsumptionError::EmptyJurisdiction);
        }
        self.matter
            .validate()
            .map_err(|_| ConsumptionError::InvalidMatter)?;
        if self.target_digest.trim().is_empty() {
            return Err(ConsumptionError::EmptyTargetDigest);
        }
        if self.payload_digest.trim().is_empty() {
            return Err(ConsumptionError::EmptyPayloadDigest);
        }
        if self.budget_id.trim().is_empty() {
            return Err(ConsumptionError::EmptyBudgetId);
        }
        Ok(())
    }

    /// Stable semantic bytes for hashing/signing by an authenticated runtime.
    /// Hash/signature algorithm selection deliberately remains outside this crate.
    pub fn canonical_bytes(&self) -> Result<Vec<u8>, ConsumptionError> {
        self.validate()?;
        let mut out = Vec::with_capacity(384);
        out.extend_from_slice(CLAIM_BINDING_DOMAIN_SEPARATOR);
        push_u16(&mut out, self.schema_version);
        push_str(&mut out, &self.claim_id)?;
        push_str(&mut out, &self.envelope_digest)?;
        push_str(&mut out, &self.nonce)?;
        push_u32(&mut out, self.use_index);
        push_str(&mut out, &self.jurisdiction)?;
        push_str(&mut out, &self.matter.namespace)?;
        push_str(&mut out, &self.matter.stable_id)?;
        push_str(&mut out, &self.target_digest)?;
        push_str(&mut out, &self.payload_digest)?;
        push_str(&mut out, &self.budget_id)?;
        Ok(out)
    }
}

fn push_u16(out: &mut Vec<u8>, value: u16) {
    out.extend_from_slice(&value.to_be_bytes());
}

fn push_u32(out: &mut Vec<u8>, value: u32) {
    out.extend_from_slice(&value.to_be_bytes());
}

fn push_str(out: &mut Vec<u8>, value: &str) -> Result<(), ConsumptionError> {
    let len = u32::try_from(value.len())
        .map_err(|_| ConsumptionError::CanonicalClaimFieldTooLarge)?;
    push_u32(out, len);
    out.extend_from_slice(value.as_bytes());
    Ok(())
}
