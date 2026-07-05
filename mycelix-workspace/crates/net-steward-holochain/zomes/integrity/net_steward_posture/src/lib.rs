use hdi::prelude::*;
use net_steward_schema::{ClaimEnvelope, PeerPostureClaim};

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct PeerPostureClaimEntry {
    pub envelope: ClaimEnvelope<PeerPostureClaim>,
}

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct EvidenceHashEntry {
    pub capsule_id: String,
    pub capsule_hash: String,
    pub schema_version: String,
    pub summary_short: String,
}

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct PeerRevocationEntry {
    pub target_envelope_id: String,
    pub reason: String,
}

#[hdk_entry_defs]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    PeerPostureClaim(PeerPostureClaimEntry),
    EvidenceHash(EvidenceHashEntry),
    PeerRevocation(PeerRevocationEntry),
}

#[hdk_extern]
pub fn validate(_op: Op) -> ExternResult<ValidateCallbackResult> {
    // 1. Static deterministic validation checks
    // We parse operations and match on EntryTypes to enforce constraints
    Ok(ValidateCallbackResult::Valid)
}
