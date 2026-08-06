// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Mycelix-DeSci Core
//!
//! Core functionality for Mycelix-DeSci including:
//! - Epistemic claim management with Charter v2.0 LEM Cube
//! - Multi-layer epistemic fingerprinting (LEM, Type, Quality, Network)
//! - Proof of Gradient Quality (PoGQ) for federated learning
//! - MATL (Mycelix Adaptive Trust Layer) integration
//! - Claim relationships and knowledge graph support
//! - Data verification and provenance tracking
//! - Claim evolution and versioning (Constitution Schema v2.0)
//! - Dispute resolution system (Epistemic Charter §5)
//! - Cartel detection for anti-collusion
//! - Reproducibility tracking for scientific claims
//! - Prediction markets for epistemic outcomes
//! - Semantic similarity detection
//! - Time-based decay mechanics
//! - Cross-claim inference engine (transitive support, contradiction detection)
//! - Citation graph analysis (PageRank importance scoring)
//! - Expertise domains and weighted verification
//! - Temporal consensus tracking (paradigm shift detection)
//! - Meta-claims and systematic reviews (PRISMA, GRADE)
//! - Bayesian belief networks for probabilistic reasoning
//! - Zero-knowledge verification proofs

pub mod authority_delivery;
pub mod authority_epoch;
pub mod authority_fencing;
pub mod authority_signing;
pub mod checkpoint_mirror;
// `claims`, `error`, and `storage` moved out to the sibling
// `mycelix-desci-types` crate 2026-08-05 -- kept dependency-light (no sqlx,
// no Holochain) so it can be shared with `mycelix-desci-holochain-bridge`
// without pulling this crate's `sqlx` dependency along with it. Re-exported
// below so every existing `crate::claims::X` / `crate::{Error, Result}` /
// `crate::storage::Y` reference elsewhere in this crate keeps working
// unchanged.
pub use mycelix_desci_types::{claims, error, storage};
pub mod config;
pub mod hash;
// `knowledge_storage` (KnowledgeDhtStorage, the Holochain DHT storage
// backend) moved out to the sibling `mycelix-desci-holochain-bridge` crate
// 2026-08-05 -- it depended on holochain_types (-> rusqlite), which
// declares `links = "sqlite3"` and is unconditionally incompatible (in one
// Cargo.lock, regardless of feature activation) with sqlx's optional
// sqlx-sqlite dependency, needed here for the Postgres authority backend.
// See that crate's module doc comment for the full explanation.
pub mod logging;
pub mod pogq;
pub mod postgres_authority;
pub mod postgres_authority_epoch;
pub mod postgres_credential_authority;
pub mod query;
mod transactional_file;
pub mod trust;
pub mod utils;

// Epistemic modules (Phase 1 - v0.3.0)
pub mod cartel;
pub mod decay;
pub mod dispute;
pub mod evolution;
pub mod legacy_migration;
pub mod prediction;
pub mod reproducibility;
pub mod scientific_authority_audit;
pub mod scientific_credential_governance;
pub mod scientific_credentials;
pub mod scientific_events;
pub mod scientific_governance;
pub mod scientific_governance_quorum;
pub mod semantic;

// Advanced epistemic modules (Phase 2 - v0.4.0)
pub mod bayesian;
pub mod citation;
pub mod consensus;
pub mod expertise;
pub mod inference;
pub mod meta;
pub mod zkproof;

pub use authority_fencing::{
    AUTHORITY_WRITE_LEASE_CODEC, AUTHORITY_WRITE_LEASE_PROTOCOL,
    AUTHORITY_WRITE_LEASE_PROTOCOL_VERSION, AUTHORITY_WRITE_LEASE_SCHEMA_VERSION,
    AuthorityWriteLease, AuthorityWriteLeasePhase, AuthorityWriteLeaseProvider,
    AuthorityWriteScope, FileAuthorityWriteLeaseProvider, SignedAuthorityWriteLease,
    StaticAuthorityWriteLeaseProvider,
};
#[cfg(unix)]
pub use authority_signing::UnixSocketAuthoritySigner;
pub use authority_signing::{AuthoritySigner, NamedSoftwareAuthoritySigner};

pub use authority_epoch::{
    AUTHORITY_DATABASE_EPOCH_CODEC, AUTHORITY_DATABASE_EPOCH_PROTOCOL,
    AUTHORITY_DATABASE_EPOCH_PROTOCOL_VERSION, AUTHORITY_DATABASE_EPOCH_SCHEMA_VERSION,
    AUTHORITY_DELIVERY_ACK_PROTOCOL, AUTHORITY_RECOVERY_RECONCILIATION_PROTOCOL,
    AuthorityDatabaseEpochCertificate, AuthorityDatabaseStateCommitment,
    AuthorityDeliveryAcknowledgement, AuthorityRecoveryReconciliation,
    DatabaseEpochPromotionIntent, DatabasePromotionMode, EmergencyRecoveryCeremony,
    RecoveryReconciliationStatus, SignedAuthorityDatabaseEpochCertificate,
    SignedAuthorityDeliveryAcknowledgement, SignedAuthorityRecoveryReconciliation,
};

pub use authority_delivery::{
    AUTHORITY_DELIVERY_CODEC, AUTHORITY_DELIVERY_PROTOCOL, AUTHORITY_DELIVERY_PROTOCOL_VERSION,
    AUTHORITY_DELIVERY_SCHEMA_VERSION, AuthorityDeliveryEnvelope, SignedAuthorityDeliveryEnvelope,
};

pub use checkpoint_mirror::{CheckpointMirrorObservationId, SignedCheckpointMirrorObservation};

// Core claim types
pub use claims::{
    ClaimContent, DesciClaim, EpistemicPosition, EpistemicTier, Provenance, Verification,
};

// Layer 1: LEM Cube (Charter v2.0)
pub use claims::{EmpiricalAxis, LEMCube, MaterialityAxis, NormativeAxis};

// Layer 3: Quality Metrics
pub use claims::QualityMetrics;

// Layer 4: Network Position
pub use claims::{ClaimRelation, ClaimRelationType, NetworkPosition};

// MATL Integration
pub use claims::MATLTrust;

// Unified Fingerprint
pub use claims::EpistemicFingerprint;

// Claim Evolution (Constitution Schema v2.0)
pub use evolution::{ClaimEvolution, ClaimStatus, EvolutionType};

// Dispute Resolution (Epistemic Charter §5)
pub use dispute::{
    ActionType, AuthorResponse, ChallengeType, Dispute, DisputeStatus, Evidence, EvidenceType,
    RequiredAction, Resolution, ResolutionOutcome,
};

// Cartel Detection (Anti-Collusion)
pub use cartel::{
    CartelDetectionConfig, CartelDetectionResult, CartelDetector, CartelPattern,
    CartelRecommendation, VerificationEvent,
};

// Reproducibility Tracking
pub use reproducibility::{
    MethodologyMatch, ReplicationAttempt, ReplicationOutcome, ReplicationStatus,
    ReproducibilityRegistry, ReproducibilityStats,
};

// Prediction Markets
pub use prediction::{
    MarketError, MarketState, MarketType, Position, PredictionMarket, PredictionMarketRegistry,
    ResolutionMethod, Settlement,
};

// Semantic Similarity
pub use semantic::{
    ClaimContent as SemanticClaimContent, DuplicateCheckResult, DuplicateRecommendation,
    SimilarityComponents, SimilarityEngine, SimilarityRelationship, SimilarityScore,
};

// Decay Mechanics
pub use decay::{DecayCalculator, DecayConfig, DecayFunction, DecayStats, DecayingAccumulator};

// Cross-Claim Inference Engine
pub use inference::{
    ClaimEdge, ClaimGraph, ClaimNode, ContradictionReport, ContradictionType, InferenceEngine,
    InferenceRule, InferredRelation, SuggestedRelation,
};

// Citation Graph Analysis
pub use citation::{
    Citation, CitationAnalyzer, CitationGraph, CitationMetrics, CitationType, PageRankConfig,
};

// Expertise Domains & Weighted Verification
pub use expertise::{
    DomainTaxonomy, ExpertiseDomain, ExpertiseLevel, ExpertiseProfile, ExpertiseVerifier,
    ExpertiseWeightConfig, WeightedVerificationResult,
};

// Temporal Consensus Tracking
pub use consensus::{
    ConsensusHistory, ConsensusSnapshot, ConsensusState, ConsensusTracker, ConsensusTrackerConfig,
    ConsensusTrend, ParadigmShift, ShiftType,
};

// Meta-Claims & Systematic Reviews
pub use meta::{
    EffectDirection, GradeQuality, HeterogeneityAssessment, HeterogeneityLevel, MetaClaim,
    MetaClaimType, MetaSynthesizer, PrismaFlow, RiskOfBias, SourceQualityAssessment,
    SynthesisMethod, SynthesisResult,
};

// Bayesian Belief Networks
pub use bayesian::{
    BayesianInference, BeliefNetwork, BeliefNode, CPTEntry, ConditionalProbabilityTable,
};

pub use postgres_authority_epoch::{
    AuthorityDatabaseEpochRecord, AuthorityDatabaseEpochSummary,
    AuthorityDeliveryAcknowledgementRecord, AuthorityRecoveryReconciliationRecord,
};

pub use postgres_authority::{
    AuthorityOutboxMessage, AuthorityOutboxSummary, AuthorityWriteFencingStatus,
    CheckpointMirrorRecord, PostgresAuthorityConfig, PostgresAuthorityFencingConfig,
    PostgresAuthorityStore,
};

// Append-only scientific event kernel
pub use scientific_events::{
    ActorId, AppendReceipt, ArtifactAvailability, ArtifactId, AtomicClaim, Attestation,
    AttestationId, AttestationKind, AttestationStatus, ClaimId, ClaimLifecycle, ClaimOrigin,
    ClaimProjection, ContentHash, DEFAULT_EVIDENCE_POLICY_ID, DEFAULT_EVIDENCE_POLICY_VERSION,
    EventPage, EvidenceArtifact, EvidenceAssessment, EvidenceMaturity, EvidenceOutcome,
    EvidenceProfile, FileScientificEventLog, LegacyImportMetadata, MAX_EVENT_STREAM_FILE_BYTES,
    MIN_SUPPORTED_SCIENTIFIC_EVENT_SCHEMA_VERSION, MemoryScientificEventLog, OrganizationId,
    RecordedAttestation, ResearchObject, ResearchObjectId, ResearchObjectType,
    SCIENTIFIC_EVENT_CODEC, SCIENTIFIC_EVENT_PROTOCOL, SCIENTIFIC_EVENT_PROTOCOL_VERSION,
    SCIENTIFIC_EVENT_SCHEMA_VERSION, ScientificEventEnvelope, ScientificEventId,
    ScientificEventLog, ScientificEventPayload, SignedScientificEvent, StreamHead,
};

pub use legacy_migration::{
    LegacyMigrationContext, LegacyMigrationReport, LegacyMigrationStatus,
    canonical_legacy_source_bytes, migrate_legacy_claim,
};

pub use scientific_credential_governance::{
    AuthorizedCredentialAcceptanceKey, AuthorizedCredentialTransparencyWitness,
    CREDENTIAL_GOVERNANCE_CODEC, CREDENTIAL_GOVERNANCE_PROTOCOL,
    CREDENTIAL_GOVERNANCE_PROTOCOL_VERSION, CREDENTIAL_GOVERNANCE_SCHEMA_VERSION,
    CredentialGovernanceAction, CredentialGovernanceAppendReceipt,
    CredentialGovernanceApprovalStatus, CredentialGovernanceEnvelope, CredentialGovernanceEventId,
    CredentialGovernancePayload, CredentialGovernancePolicy, CredentialGovernanceProjection,
    CredentialGovernanceProposal, CredentialGovernanceProposalId,
    CredentialGovernanceProposalStatus, CredentialGovernanceSummary,
    CredentialTransparencyCheckpoint, MAX_CREDENTIAL_GOVERNANCE_FILE_BYTES,
    RecordedCredentialGovernanceEvent, ScientificCredentialGovernance,
    SignedCredentialGovernanceEvent, SignedCredentialTransparencyWitness,
    TransparencyWitnessCompromiseInterval,
};

pub use scientific_credentials::{
    MAX_CREDENTIAL_REGISTRY_FILE_BYTES, RecordedScientificCredentialEvent,
    SCIENTIFIC_CREDENTIAL_CODEC, SCIENTIFIC_CREDENTIAL_PROTOCOL,
    SCIENTIFIC_CREDENTIAL_PROTOCOL_VERSION, SCIENTIFIC_CREDENTIAL_SCHEMA_VERSION,
    ScientificCredentialAppendReceipt, ScientificCredentialEnvelope, ScientificCredentialEventId,
    ScientificCredentialPayload, ScientificCredentialRegistry,
    ScientificCredentialRegistryProjection, ScientificCredentialRegistrySummary,
    SignedScientificCredentialEvent,
};

pub use scientific_authority_audit::{
    AUTHORITY_RECEIPT_PROTOCOL, AUTHORITY_RECEIPT_PROTOCOL_VERSION, AuthorityAttestationStatus,
    AuthorityAuditSummary, FileScientificAuthorityAuditStore, MAX_AUTHORITY_RECEIPT_FILE_BYTES,
    MIN_SUPPORTED_AUTHORITY_RECEIPT_PROTOCOL_VERSION, MemoryScientificAuthorityAuditStore,
    ScientificAuthorityAuditStore, ScientificAuthorityReceipt, SignedScientificAuthorityReceipt,
};

pub use scientific_governance_quorum::{
    CredentialGovernanceApprovalRule, CredentialGovernanceRiskPolicy, CredentialGovernanceRiskTier,
};

pub use scientific_governance::{
    AtomicScientificCommitStore, AuthorizationDecision, AuthorizedActorKey,
    DefaultScientificAuthorizationPolicy, GovernedScientificEventLog,
    MemoryScientificIdentityResolver, ResolvedScientificActor, ScientificAction,
    ScientificAuthorizationPolicy, ScientificIdentityResolver, ScientificRole,
};

// Zero-Knowledge Verification Proofs
pub use zkproof::{Witness, ZKProof, ZKProofError, ZKProofType, ZKProver, ZKStatement, ZKVerifier};

// Error handling and configuration
pub use config::Config;
pub use error::{Error, Result};
pub use query::{QueryEngine, QueryFilter};

/// Version of the Mycelix-DeSci protocol
pub const PROTOCOL_VERSION: &str = "0.10.0";

/// Default Byzantine fault tolerance threshold for PoGQ (45%)
pub const DEFAULT_BFT_THRESHOLD: f64 = 0.45;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_protocol_version() {
        assert_eq!(PROTOCOL_VERSION, "0.10.0");
    }
}
