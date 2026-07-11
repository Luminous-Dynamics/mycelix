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

pub mod claims;
pub mod config;
pub mod error;
pub mod hash;
#[cfg(feature = "holochain")]
pub mod knowledge_storage;
pub mod logging;
pub mod pogq;
pub mod query;
pub mod storage;
pub mod trust;
pub mod utils;

// Epistemic modules (Phase 1 - v0.3.0)
pub mod cartel;
pub mod decay;
pub mod dispute;
pub mod evolution;
pub mod prediction;
pub mod reproducibility;
pub mod semantic;

// Advanced epistemic modules (Phase 2 - v0.4.0)
pub mod bayesian;
pub mod citation;
pub mod consensus;
pub mod expertise;
pub mod inference;
pub mod meta;
pub mod zkproof;

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

// Zero-Knowledge Verification Proofs
pub use zkproof::{Witness, ZKProof, ZKProofError, ZKProofType, ZKProver, ZKStatement, ZKVerifier};

// Error handling and configuration
pub use config::Config;
pub use error::{Error, Result};
pub use query::{QueryEngine, QueryFilter};

/// Version of the Mycelix-DeSci protocol
pub const PROTOCOL_VERSION: &str = "0.4.0";

/// Default Byzantine fault tolerance threshold for PoGQ (45%)
pub const DEFAULT_BFT_THRESHOLD: f64 = 0.45;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_protocol_version() {
        assert_eq!(PROTOCOL_VERSION, "0.4.0");
    }
}
