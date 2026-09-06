// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Dependency-free normative contracts for the Mycelix Solar-System
//! Federation (SSF) security profile.
//!
//! This crate deliberately contains no Holochain, networking, cryptography,
//! clock, storage, or federated-learning implementation. Its job is to make
//! security-critical meanings non-interchangeable and to provide pure
//! validation functions that downstream adapters can share.
//!
//! The core rule is:
//!
//! **Foreign evidence may inform local qualification, but it is not local
//! authority.**

#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

/// First SSF schema version.
pub const SSF_SCHEMA_V1: u16 = 1;

macro_rules! digest_type {
    ($name:ident) => {
        #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
        #[repr(transparent)]
        pub struct $name([u8; 32]);

        impl $name {
            /// Construct from already-verified canonical 32-byte digest bytes.
            ///
            /// This crate does not choose or implement the digest algorithm.
            pub const fn from_bytes(bytes: [u8; 32]) -> Self {
                Self(bytes)
            }

            /// Return the canonical digest bytes.
            pub const fn as_bytes(&self) -> &[u8; 32] {
                &self.0
            }
        }
    };
}

macro_rules! generation_type {
    ($name:ident) => {
        #[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
        #[repr(transparent)]
        pub struct $name(u64);

        impl $name {
            pub const fn new(value: u64) -> Self {
                Self(value)
            }

            pub const fn get(self) -> u64 {
                self.0
            }
        }
    };
}

digest_type!(FederationId);
digest_type!(AuthorityRootCommitment);
digest_type!(PolicyDigest);
digest_type!(EvidenceCommitment);
digest_type!(ArtifactCommitment);
digest_type!(SoftwareGenerationCommitment);
digest_type!(MachineHealthCommitment);

generation_type!(FederationEpoch);
generation_type!(AuthorityGeneration);
generation_type!(RevocationGeneration);
generation_type!(CausalGeneration);

/// Exact local federation context under which a security decision is made.
///
/// Equal logical federation names are intentionally irrelevant. Different
/// epochs, authority generations, roots, policies, or revocation generations
/// are different security contexts.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct FederationContextV1 {
    pub federation_id: FederationId,
    pub federation_epoch: FederationEpoch,
    pub authority_root: AuthorityRootCommitment,
    pub authority_generation: AuthorityGeneration,
    pub revocation_generation: RevocationGeneration,
    pub policy_digest: PolicyDigest,
}

/// Consequence classes are monotonically ordered from observation-only to
/// irreversible/life-critical action.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[repr(u8)]
pub enum ConsequenceClass {
    /// Observe, measure, or append non-authoritative diagnostic evidence.
    C0Observe = 0,
    /// Request challenge, increase monitoring, or gather more evidence.
    C1Challenge = 1,
    /// Reduce learning influence or defer a candidate.
    C2LearningRestriction = 2,
    /// Reject one contribution or restrict one narrow capability.
    C3ScopedRestriction = 3,
    /// Temporarily quarantine one local workload/application.
    C4WorkloadQuarantine = 4,
    /// Isolate one machine/service or equivalent high-impact local action.
    C5MachineIsolation = 5,
    /// Federation-level revocation or critical-infrastructure action.
    C6FederationCritical = 6,
    /// Irreversible or life-critical action.
    C7Irreversible = 7,
}

/// Quality/source class of time evidence available to a decision.
///
/// These variants are intentionally not `Ord`: causal, monotonic, and
/// absolute time are different evidence kinds, not a single scalar quality.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum TimeEvidenceKind {
    TrustedAbsolute,
    LocalMonotonic,
    CausalGeneration,
    ExternallyWitnessed,
    UnknownOrDegraded,
}

/// A validity boundary contributed by one prerequisite.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ValidityBoundary {
    /// This prerequisite is known valid through the given local time unit.
    KnownUntil(u64),
    /// This prerequisite does not impose a time boundary under this policy.
    NotApplicable,
    /// The required prerequisite exists but its freshness is not known.
    Unknown,
}

/// Freshness prerequisites for downstream authority.
///
/// Units are deliberately not interpreted by this crate. A downstream
/// profile must use one canonical local time basis for every `KnownUntil`
/// value in the same instance.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ValidityInputsV1 {
    pub membership: ValidityBoundary,
    pub identity_key: ValidityBoundary,
    pub revocation_snapshot: ValidityBoundary,
    pub policy: ValidityBoundary,
    pub machine_health: ValidityBoundary,
    pub scientific_qualification: ValidityBoundary,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FreshnessError {
    /// Required freshness cannot be established; authority must fail closed.
    UnknownRequiredBoundary,
}

/// Derive the downstream authority ceiling.
///
/// SSF-006: a downstream authority cannot outlive any applicable upstream
/// prerequisite. `None` means no input imposed a time boundary. `Unknown`
/// fails closed rather than becoming an arbitrary/default lifetime.
pub fn derive_valid_until(inputs: ValidityInputsV1) -> Result<Option<u64>, FreshnessError> {
    let boundaries = [
        inputs.membership,
        inputs.identity_key,
        inputs.revocation_snapshot,
        inputs.policy,
        inputs.machine_health,
        inputs.scientific_qualification,
    ];

    let mut minimum: Option<u64> = None;

    for boundary in boundaries {
        match boundary {
            ValidityBoundary::KnownUntil(value) => {
                minimum = Some(match minimum {
                    Some(current) => current.min(value),
                    None => value,
                });
            }
            ValidityBoundary::NotApplicable => {}
            ValidityBoundary::Unknown => {
                return Err(FreshnessError::UnknownRequiredBoundary);
            }
        }
    }

    Ok(minimum)
}

/// An explicit bounded-autonomy grant for one exact federation generation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AutonomyBudgetV1 {
    pub context: FederationContextV1,
    pub max_consequence: ConsequenceClass,
    pub max_blast_radius: u32,
    pub max_delegation_depth: u8,
    pub max_resource_commitment: u64,
    pub max_disconnected_duration: u64,
    pub max_evidence_age: u64,
    pub min_local_quorum: u16,
    pub required_software_generation: Option<SoftwareGenerationCommitment>,
    pub required_machine_health: Option<MachineHealthCommitment>,
    pub emergency_authority_allowed: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AttenuationError {
    ContextMismatch,
    ConsequenceExpanded,
    BlastRadiusExpanded,
    DelegationExpanded,
    ResourceCommitmentExpanded,
    DisconnectedDurationExpanded,
    EvidenceAgeExpanded,
    LocalQuorumWeakened,
    SoftwareGenerationRequirementWeakened,
    MachineHealthRequirementWeakened,
    EmergencyAuthorityExpanded,
}

/// Validate SSF-002/003: delegation/disconnection cannot increase authority.
///
/// A child may be stricter than its parent. It may not silently drop exact
/// software or machine-health requirements inherited from the parent.
pub fn validate_autonomy_attenuation(
    parent: &AutonomyBudgetV1,
    child: &AutonomyBudgetV1,
) -> Result<(), AttenuationError> {
    if parent.context != child.context {
        return Err(AttenuationError::ContextMismatch);
    }
    if child.max_consequence > parent.max_consequence {
        return Err(AttenuationError::ConsequenceExpanded);
    }
    if child.max_blast_radius > parent.max_blast_radius {
        return Err(AttenuationError::BlastRadiusExpanded);
    }
    if child.max_delegation_depth > parent.max_delegation_depth {
        return Err(AttenuationError::DelegationExpanded);
    }
    if child.max_resource_commitment > parent.max_resource_commitment {
        return Err(AttenuationError::ResourceCommitmentExpanded);
    }
    if child.max_disconnected_duration > parent.max_disconnected_duration {
        return Err(AttenuationError::DisconnectedDurationExpanded);
    }
    if child.max_evidence_age > parent.max_evidence_age {
        return Err(AttenuationError::EvidenceAgeExpanded);
    }
    if child.min_local_quorum < parent.min_local_quorum {
        return Err(AttenuationError::LocalQuorumWeakened);
    }

    match (
        parent.required_software_generation,
        child.required_software_generation,
    ) {
        (Some(required), Some(actual)) if required == actual => {}
        (Some(_), _) => {
            return Err(AttenuationError::SoftwareGenerationRequirementWeakened);
        }
        (None, _) => {}
    }

    match (parent.required_machine_health, child.required_machine_health) {
        (Some(required), Some(actual)) if required == actual => {}
        (Some(_), _) => {
            return Err(AttenuationError::MachineHealthRequirementWeakened);
        }
        (None, _) => {}
    }

    if child.emergency_authority_allowed && !parent.emergency_authority_allowed {
        return Err(AttenuationError::EmergencyAuthorityExpanded);
    }

    Ok(())
}

/// Immutable cross-federation evidence envelope.
///
/// This type intentionally contains no local authority field. Receiving this
/// object proves only that a foreign artifact can be addressed as evidence;
/// cryptographic verification and local qualification are separate layers.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ForeignArtifactEnvelopeV1 {
    pub schema_version: u16,
    pub origin: FederationContextV1,
    pub artifact_type: u16,
    pub artifact_commitment: ArtifactCommitment,
    pub causal_generation: CausalGeneration,
    pub predecessor: Option<ArtifactCommitment>,
    pub evidence_root: EvidenceCommitment,
}

impl ForeignArtifactEnvelopeV1 {
    pub const fn new(
        origin: FederationContextV1,
        artifact_type: u16,
        artifact_commitment: ArtifactCommitment,
        causal_generation: CausalGeneration,
        predecessor: Option<ArtifactCommitment>,
        evidence_root: EvidenceCommitment,
    ) -> Self {
        Self {
            schema_version: SSF_SCHEMA_V1,
            origin,
            artifact_type,
            artifact_commitment,
            causal_generation,
            predecessor,
            evidence_root,
        }
    }
}

/// Local qualification claim referencing foreign evidence.
///
/// This remains *evidence*, not execution authority. Downstream policy and
/// authority adapters must independently verify who may issue this claim.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LocalQualificationClaimV1 {
    pub schema_version: u16,
    pub local_context: FederationContextV1,
    pub foreign_artifact: ArtifactCommitment,
    pub local_evidence_root: EvidenceCommitment,
    pub maximum_justified_consequence: ConsequenceClass,
    pub valid_until: Option<u64>,
}

impl LocalQualificationClaimV1 {
    pub const fn new(
        local_context: FederationContextV1,
        foreign_artifact: ArtifactCommitment,
        local_evidence_root: EvidenceCommitment,
        maximum_justified_consequence: ConsequenceClass,
        valid_until: Option<u64>,
    ) -> Self {
        Self {
            schema_version: SSF_SCHEMA_V1,
            local_context,
            foreign_artifact,
            local_evidence_root,
            maximum_justified_consequence,
            valid_until,
        }
    }
}

/// Closed-world outcome for consequential external effects.
///
/// SSF-013: `OutcomeUnknown` is a distinct state and must not be treated as
/// ordinary failure/proven non-application.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum EffectDisposition {
    Confirmed,
    ProvenNotApplied,
    OutcomeUnknown,
}

#[cfg(test)]
mod tests {
    use super::*;

    const fn bytes(byte: u8) -> [u8; 32] {
        [byte; 32]
    }

    fn context(authority_generation: u64) -> FederationContextV1 {
        FederationContextV1 {
            federation_id: FederationId::from_bytes(bytes(1)),
            federation_epoch: FederationEpoch::new(7),
            authority_root: AuthorityRootCommitment::from_bytes(bytes(2)),
            authority_generation: AuthorityGeneration::new(authority_generation),
            revocation_generation: RevocationGeneration::new(9),
            policy_digest: PolicyDigest::from_bytes(bytes(3)),
        }
    }

    fn parent_budget() -> AutonomyBudgetV1 {
        AutonomyBudgetV1 {
            context: context(11),
            max_consequence: ConsequenceClass::C5MachineIsolation,
            max_blast_radius: 100,
            max_delegation_depth: 3,
            max_resource_commitment: 1_000,
            max_disconnected_duration: 86_400,
            max_evidence_age: 10_000,
            min_local_quorum: 3,
            required_software_generation: Some(SoftwareGenerationCommitment::from_bytes(bytes(4))),
            required_machine_health: Some(MachineHealthCommitment::from_bytes(bytes(5))),
            emergency_authority_allowed: false,
        }
    }

    #[test]
    fn ssf_005_distinguishes_exact_authority_generations() {
        let a = context(11);
        let b = context(12);
        assert_ne!(a, b);
    }

    #[test]
    fn ssf_003_allows_stricter_child_budget() {
        let parent = parent_budget();
        let mut child = parent;
        child.max_consequence = ConsequenceClass::C3ScopedRestriction;
        child.max_blast_radius = 20;
        child.max_delegation_depth = 1;
        child.max_resource_commitment = 100;
        child.max_disconnected_duration = 3_600;
        child.max_evidence_age = 1_000;
        child.min_local_quorum = 5;

        assert_eq!(validate_autonomy_attenuation(&parent, &child), Ok(()));
    }

    #[test]
    fn ssf_002_rejects_consequence_expansion_during_delegation() {
        let parent = parent_budget();
        let mut child = parent;
        child.max_consequence = ConsequenceClass::C6FederationCritical;

        assert_eq!(
            validate_autonomy_attenuation(&parent, &child),
            Err(AttenuationError::ConsequenceExpanded)
        );
    }

    #[test]
    fn ssf_003_rejects_dropped_software_generation_requirement() {
        let parent = parent_budget();
        let mut child = parent;
        child.required_software_generation = None;

        assert_eq!(
            validate_autonomy_attenuation(&parent, &child),
            Err(AttenuationError::SoftwareGenerationRequirementWeakened)
        );
    }

    #[test]
    fn ssf_003_rejects_dropped_machine_health_requirement() {
        let parent = parent_budget();
        let mut child = parent;
        child.required_machine_health = None;

        assert_eq!(
            validate_autonomy_attenuation(&parent, &child),
            Err(AttenuationError::MachineHealthRequirementWeakened)
        );
    }

    #[test]
    fn ssf_016_rejects_emergency_authority_expansion() {
        let parent = parent_budget();
        let mut child = parent;
        child.emergency_authority_allowed = true;

        assert_eq!(
            validate_autonomy_attenuation(&parent, &child),
            Err(AttenuationError::EmergencyAuthorityExpanded)
        );
    }

    #[test]
    fn ssf_006_downstream_validity_is_minimum_upstream_boundary() {
        let inputs = ValidityInputsV1 {
            membership: ValidityBoundary::KnownUntil(100),
            identity_key: ValidityBoundary::KnownUntil(90),
            revocation_snapshot: ValidityBoundary::KnownUntil(80),
            policy: ValidityBoundary::KnownUntil(95),
            machine_health: ValidityBoundary::KnownUntil(70),
            scientific_qualification: ValidityBoundary::KnownUntil(85),
        };

        assert_eq!(derive_valid_until(inputs), Ok(Some(70)));
    }

    #[test]
    fn ssf_006_unknown_required_freshness_fails_closed() {
        let inputs = ValidityInputsV1 {
            membership: ValidityBoundary::KnownUntil(100),
            identity_key: ValidityBoundary::KnownUntil(90),
            revocation_snapshot: ValidityBoundary::Unknown,
            policy: ValidityBoundary::KnownUntil(95),
            machine_health: ValidityBoundary::NotApplicable,
            scientific_qualification: ValidityBoundary::NotApplicable,
        };

        assert_eq!(
            derive_valid_until(inputs),
            Err(FreshnessError::UnknownRequiredBoundary)
        );
    }

    #[test]
    fn consequence_classes_are_monotonic() {
        assert!(ConsequenceClass::C0Observe < ConsequenceClass::C1Challenge);
        assert!(ConsequenceClass::C3ScopedRestriction < ConsequenceClass::C5MachineIsolation);
        assert!(ConsequenceClass::C6FederationCritical < ConsequenceClass::C7Irreversible);
    }

    #[test]
    fn foreign_artifact_remains_distinct_from_local_qualification() {
        let foreign = ForeignArtifactEnvelopeV1::new(
            context(11),
            1,
            ArtifactCommitment::from_bytes(bytes(6)),
            CausalGeneration::new(42),
            None,
            EvidenceCommitment::from_bytes(bytes(7)),
        );

        let local = LocalQualificationClaimV1::new(
            context(12),
            foreign.artifact_commitment,
            EvidenceCommitment::from_bytes(bytes(8)),
            ConsequenceClass::C2LearningRestriction,
            Some(1_000),
        );

        assert_eq!(local.foreign_artifact, foreign.artifact_commitment);
        assert_ne!(local.local_context, foreign.origin);
    }

    #[test]
    fn outcome_unknown_is_not_proven_not_applied() {
        assert_ne!(
            EffectDisposition::OutcomeUnknown,
            EffectDisposition::ProvenNotApplied
        );
    }
}
