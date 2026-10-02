pub mod terrain_fl;
// #[cfg(feature = "federated")]
// pub mod consciousness_sync;
// #[cfg(feature = "federated")]
// pub mod federated_genomics;

#[cfg(feature = "hdk")]
pub mod validation;
#[cfg(feature = "hdk")]
pub use validation::{check_author_match, check_link_author_match};

pub mod collective_phi;
pub use collective_phi::{
    AgentConsciousnessVector, COLLECTIVE_PHI_MAX_SYNC, CollectivePhiEngine, CollectivePhiResult,
};

pub mod security_kernel;
pub use security_kernel::{
    AdvisoryResult, AuthorizationDecision, AuthorizationDenial, AuthorizationIndeterminacy,
    AuthorizationPermit, AuthorizationRequest, Capability, CapabilityAction, EnforcementRequest,
    MAX_AUTHORIZATION_PERMIT_LIFETIME_US, VerificationEvidence, VerifiedCapability,
    authorize_permit, revalidate_permit, verify_capability,
};

pub mod security_events;
pub use security_events::{
    MAX_PROVENANCE_IDENTIFIER_BYTES, ProvenanceRef, ProvenanceRelation, SecurityEvent,
};

pub mod routing;
pub use routing::{
    BridgeDomain, CIVIC_DOMAINS, COMMONS_DOMAINS, CivicZome, CommonsZome, CrossClusterRole,
    resolve_civic_zome, resolve_commons_zome,