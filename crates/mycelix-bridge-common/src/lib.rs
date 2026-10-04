pub use collective_phi::{
    AgentConsciousnessVector, COLLECTIVE_PHI_MAX_SYNC, CollectivePhiEngine, CollectivePhiResult,
};

pub mod security_kernel;
pub use security_kernel::{
    AdvisoryResult, AuthorizationDecision, AuthorizationDenial, AuthorizationIndeterminacy,
    AuthorizationPermit, AuthorizationRequest, Capability, CapabilityAction, EnforcementRequest,
    MAX_AUTHORIZATION_PERMIT_LIFETIME_US, MAX_SECURITY_WIRE_BYTES, VerificationEvidence,
    checked_authority_lease_until_us,
    VerifiedCapability, authorize_permit, deserialize_bounded_security_json,
    deserialize_bounded_security_json_reader, verify_capability,
};

pub mod security_events;
pub use security_events::{
    MAX_PROVENANCE_IDENTIFIER_BYTES, ProvenanceRef, ProvenanceRelation, SecurityEvent,
    SecurityEventDecision,
};
