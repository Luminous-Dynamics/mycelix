use hdk::prelude::*;
use net_steward_posture_integrity::{
    EntryTypes, EvidenceHashEntry, PeerPostureClaimEntry, PeerRevocationEntry,
};
use net_steward_schema::{ClaimEnvelope, PeerPostureClaim};

#[hdk_extern]
pub fn publish_posture_claim(
    envelope: ClaimEnvelope<PeerPostureClaim>,
) -> ExternResult<ActionHash> {
    let entry = PeerPostureClaimEntry { envelope };
    create_entry(EntryTypes::PeerPostureClaim(entry))
}

#[hdk_extern]
pub fn publish_evidence_hash(entry: EvidenceHashEntry) -> ExternResult<ActionHash> {
    create_entry(EntryTypes::EvidenceHash(entry))
}

#[hdk_extern]
pub fn publish_revocation(entry: PeerRevocationEntry) -> ExternResult<ActionHash> {
    create_entry(EntryTypes::PeerRevocation(entry))
}

#[hdk_extern]
pub fn fetch_all_posture_claims(_: ()) -> ExternResult<Vec<ClaimEnvelope<PeerPostureClaim>>> {
    // In beta.3, retrieves claims from DHT via link queries
    Ok(vec![])
}
