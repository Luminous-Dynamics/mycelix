// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Dynamic covered-current-head composition over stable rooted lineages.
//!
//! The consuming domain authenticates source and closed-world coverage before
//! projecting facts into this crate. This crate exact-matches those supplied
//! facts to one already-qualified stable lineage endpoint and preserves the
//! independent EvidenceLease without widening it.

use mycelix_authority_evidence_lease::{EvidenceLease, EvidenceLeaseError};
use mycelix_core_lineage::{ProfiledDigest32, ProjectedRootedLineage};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CoverageState {
    Complete,
    Indeterminate,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CoveredHeadMode {
    Live,
    HistoricalAsOf { as_of_ms: u64 },
}

/// Projection of an already domain-qualified closed-world covered-head observation.
///
/// Construction is an adapter boundary, not source authentication. A consuming
/// domain must only project facts after its own verifier has authenticated the
/// source, coverage semantics, and verifier origin appropriate to that domain.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CoveredHeadObservationFacts {
    lineage_domain_identity: ProfiledDigest32,
    source_descriptor_identity: ProfiledDigest32,
    head_generation: u64,
    head_node_identity: ProfiledDigest32,
    head_record_identity: ProfiledDigest32,
    coverage_evidence_identity: ProfiledDigest32,
    verification_evidence_identity: ProfiledDigest32,
    coverage_state: CoverageState,
    mode: CoveredHeadMode,
    evidence_lease: EvidenceLease,
    next_known_transition_effective_at_ms: Option<u64>,
}

impl CoveredHeadObservationFacts {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        lineage_domain_identity: ProfiledDigest32,
        source_descriptor_identity: ProfiledDigest32,
        head_generation: u64,
        head_node_identity: ProfiledDigest32,
        head_record_identity: ProfiledDigest32,
        coverage_evidence_identity: ProfiledDigest32,
        verification_evidence_identity: ProfiledDigest32,
        coverage_state: CoverageState,
        mode: CoveredHeadMode,
        evidence_lease: EvidenceLease,
        next_known_transition_effective_at_ms: Option<u64>,
    ) -> Self {
        Self {
            lineage_domain_identity,
            source_descriptor_identity,
            head_generation,
            head_node_identity,
            head_record_identity,
            coverage_evidence_identity,
            verification_evidence_identity,
            coverage_state,
            mode,
            evidence_lease,
            next_known_transition_effective_at_ms,
        }
    }

    pub fn lineage_domain_identity(&self) -> &ProfiledDigest32 {
        &self.lineage_domain_identity
    }

    pub fn source_descriptor_identity(&self) -> &ProfiledDigest32 {
        &self.source_descriptor_identity
    }

    pub const fn head_generation(&self) -> u64 {
        self.head_generation
    }

    pub fn head_node_identity(&self) -> &ProfiledDigest32 {
        &self.head_node_identity
    }

    pub fn head_record_identity(&self) -> &ProfiledDigest32 {
        &self.head_record_identity
    }

    pub fn coverage_evidence_identity(&self) -> &ProfiledDigest32 {
        &self.coverage_evidence_identity
    }

    pub fn verification_evidence_identity(&self) -> &ProfiledDigest32 {
        &self.verification_evidence_identity
    }

    pub const fn coverage_state(&self) -> CoverageState {
        self.coverage_state
    }

    pub const fn mode(&self) -> CoveredHeadMode {
        self.mode
    }

    pub fn evidence_lease(&self) -> &EvidenceLease {
        &self.evidence_lease
    }

    pub const fn next_known_transition_effective_at_ms(&self) -> Option<u64> {
        self.next_known_transition_effective_at_ms
    }

    pub const fn source_authentication_verified_here(&self) -> bool {
        false
    }
}

/// Process-local proof that one already-qualified covered-head observation
/// exact-matched the supplied stable lineage endpoint while its evidence horizon
/// remained live and did not cross a known scheduled transition.
///
/// This token is deliberately non-serializable and must be revalidated before
/// reuse because dynamic evidence can expire after construction.
#[derive(Debug, Eq, PartialEq)]
pub struct QualifiedCoveredCurrentHead {
    lineage_stable_commitment: [u8; 32],
    endpoint_generation: u64,
    endpoint_node_identity: ProfiledDigest32,
    endpoint_source_descriptor_identity: ProfiledDigest32,
    head_record_identity: ProfiledDigest32,
    coverage_evidence_identity: ProfiledDigest32,
    verification_evidence_identity: ProfiledDigest32,
    evidence_lease: EvidenceLease,
    next_known_transition_effective_at_ms: Option<u64>,
}

impl QualifiedCoveredCurrentHead {
    pub const fn lineage_stable_commitment(&self) -> &[u8; 32] {
        &self.lineage_stable_commitment
    }

    pub const fn endpoint_generation(&self) -> u64 {
        self.endpoint_generation
    }

    pub fn endpoint_node_identity(&self) -> &ProfiledDigest32 {
        &self.endpoint_node_identity
    }

    pub fn endpoint_source_descriptor_identity(&self) -> &ProfiledDigest32 {
        &self.endpoint_source_descriptor_identity
    }

    pub fn head_record_identity(&self) -> &ProfiledDigest32 {
        &self.head_record_identity
    }

    pub fn coverage_evidence_identity(&self) -> &ProfiledDigest32 {
        &self.coverage_evidence_identity
    }

    pub fn verification_evidence_identity(&self) -> &ProfiledDigest32 {
        &self.verification_evidence_identity
    }

    pub fn evidence_lease(&self) -> &EvidenceLease {
        &self.evidence_lease
    }

    pub const fn next_known_transition_effective_at_ms(&self) -> Option<u64> {
        self.next_known_transition_effective_at_ms
    }

    /// Revalidate dynamic evidence before reusing this positive token.
    pub fn validate_reuse_at(&self, now_ms: u64) -> Result<(), CoveredHeadError> {
        if let Some(effective_at_ms) = self.next_known_transition_effective_at_ms {
            if effective_at_ms <= now_ms {
                return Err(CoveredHeadError::ScheduledTransitionAlreadyEffective);
            }
            if self.evidence_lease.valid_until_ms > effective_at_ms {
                return Err(CoveredHeadError::ScheduledTransitionHorizonViolation);
            }
        }
        self.evidence_lease.validate_at(now_ms).map_err(Into::into)
    }

    pub const fn source_authentication_verified_here(&self) -> bool {
        false
    }

    pub const fn grants_institutional_authority(&self) -> bool {
        false
    }

    pub const fn grants_policy_authority(&self) -> bool {
        false
    }

    pub const fn grants_execution_authority(&self) -> bool {
        false
    }

    pub const fn grants_effect_authority(&self) -> bool {
        false
    }
}

#[derive(Debug, Eq, PartialEq)]
pub enum CoveredHeadError {
    DomainMismatch,
    SourceDescriptorMismatch,
    HeadGenerationMismatch,
    HeadNodeMismatch,
    CoverageIndeterminate,
    HistoricalObservation,
    ScheduledTransitionAlreadyEffective,
    ScheduledTransitionHorizonViolation,
    EvidenceLease(EvidenceLeaseError),
}

impl From<EvidenceLeaseError> for CoveredHeadError {
    fn from(value: EvidenceLeaseError) -> Self {
        Self::EvidenceLease(value)
    }
}

/// Exact-match independently qualified dynamic coverage to one stable lineage endpoint.
///
/// The function does not authenticate the observation's source. It is a generic
/// structural composition boundary used only after a domain-specific source and
/// closed-world verifier has already qualified the supplied facts.
pub fn qualify_covered_current_head(
    lineage: &ProjectedRootedLineage,
    observation: CoveredHeadObservationFacts,
    now_ms: u64,
) -> Result<QualifiedCoveredCurrentHead, CoveredHeadError> {
    if observation.coverage_state != CoverageState::Complete {
        return Err(CoveredHeadError::CoverageIndeterminate);
    }
    if !matches!(observation.mode, CoveredHeadMode::Live) {
        return Err(CoveredHeadError::HistoricalObservation);
    }
    if observation.lineage_domain_identity != *lineage.root().lineage_domain_identity() {
        return Err(CoveredHeadError::DomainMismatch);
    }
    if observation.source_descriptor_identity != *lineage.endpoint_source_descriptor_identity() {
        return Err(CoveredHeadError::SourceDescriptorMismatch);
    }
    if observation.head_generation != lineage.endpoint_generation() {
        return Err(CoveredHeadError::HeadGenerationMismatch);
    }
    if observation.head_node_identity != *lineage.endpoint_node_identity() {
        return Err(CoveredHeadError::HeadNodeMismatch);
    }

    let qualified = QualifiedCoveredCurrentHead {
        lineage_stable_commitment: *lineage.stable_commitment(),
        endpoint_generation: lineage.endpoint_generation(),
        endpoint_node_identity: lineage.endpoint_node_identity().clone(),
        endpoint_source_descriptor_identity: lineage.endpoint_source_descriptor_identity().clone(),
        head_record_identity: observation.head_record_identity,
        coverage_evidence_identity: observation.coverage_evidence_identity,
        verification_evidence_identity: observation.verification_evidence_identity,
        evidence_lease: observation.evidence_lease,
        next_known_transition_effective_at_ms: observation.next_known_transition_effective_at_ms,
    };
    qualified.validate_reuse_at(now_ms)?;
    Ok(qualified)
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_authority_evidence_lease::PROTOCOL_VERSION as LEASE_PROTOCOL;
    use mycelix_core_lineage::{RootAnchorFacts, project_rooted_lineage};

    fn id(profile: &str, byte: u8) -> ProfiledDigest32 {
        ProfiledDigest32::try_new(profile, [byte; 32]).expect("valid fixture identity")
    }

    fn lineage() -> ProjectedRootedLineage {
        let root = RootAnchorFacts::new(
            id("example-lineage-domain-v1", 0x11),
            7,
            id("example-node-v1", 0x22),
            1_000,
            id("example-source-v1", 0x33),
        );
        project_rooted_lineage(root, &[]).expect("valid stable rooted lineage")
    }

    fn lease(verified_at_ms: u64, valid_until_ms: u64, now_ms: u64) -> EvidenceLease {
        EvidenceLease::new(verified_at_ms, valid_until_ms, now_ms).expect("valid fixture lease")
    }

    fn observation() -> CoveredHeadObservationFacts {
        CoveredHeadObservationFacts::new(
            id("example-lineage-domain-v1", 0x11),
            id("example-source-v1", 0x33),
            7,
            id("example-node-v1", 0x22),
            id("example-head-record-v1", 0x44),
            id("example-coverage-evidence-v1", 0x55),
            id("example-verification-evidence-v1", 0x66),
            CoverageState::Complete,
            CoveredHeadMode::Live,
            lease(100, 900, 200),
            None,
        )
    }

    #[test]
    fn exact_live_covered_endpoint_qualifies_without_authority_amplification() {
        let stable = lineage();
        let qualified = qualify_covered_current_head(&stable, observation(), 200).unwrap();
        assert_eq!(qualified.endpoint_generation(), 7);
        assert_eq!(
            qualified.lineage_stable_commitment(),
            stable.stable_commitment()
        );
        assert!(!qualified.source_authentication_verified_here());
        assert!(!qualified.grants_institutional_authority());
        assert!(!qualified.grants_policy_authority());
        assert!(!qualified.grants_execution_authority());
        assert!(!qualified.grants_effect_authority());
    }

    #[test]
    fn valid_old_prefix_does_not_become_current_when_coverage_names_later_generation() {
        let mut observed = observation();
        observed.head_generation = 8;
        assert_eq!(
            qualify_covered_current_head(&lineage(), observed, 200),
            Err(CoveredHeadError::HeadGenerationMismatch)
        );
    }

    #[test]
    fn domain_node_and_source_substitution_fail_closed() {
        let mut observed = observation();
        observed.lineage_domain_identity = id("other-domain-v1", 0x11);
        assert_eq!(
            qualify_covered_current_head(&lineage(), observed, 200),
            Err(CoveredHeadError::DomainMismatch)
        );

        let mut observed = observation();
        observed.head_node_identity = id("example-node-v1", 0x99);
        assert_eq!(
            qualify_covered_current_head(&lineage(), observed, 200),
            Err(CoveredHeadError::HeadNodeMismatch)
        );

        let mut observed = observation();
        observed.source_descriptor_identity = id("example-source-v1", 0xaa);
        assert_eq!(
            qualify_covered_current_head(&lineage(), observed, 200),
            Err(CoveredHeadError::SourceDescriptorMismatch)
        );
    }

    #[test]
    fn indeterminate_and_historical_observations_cannot_become_live_tokens() {
        let mut observed = observation();
        observed.coverage_state = CoverageState::Indeterminate;
        assert_eq!(
            qualify_covered_current_head(&lineage(), observed, 200),
            Err(CoveredHeadError::CoverageIndeterminate)
        );

        let mut observed = observation();
        observed.mode = CoveredHeadMode::HistoricalAsOf { as_of_ms: 150 };
        assert_eq!(
            qualify_covered_current_head(&lineage(), observed, 200),
            Err(CoveredHeadError::HistoricalObservation)
        );
    }

    #[test]
    fn future_and_expired_evidence_fail_through_shared_lease_theorem() {
        let mut future = observation();
        future.evidence_lease = EvidenceLease {
            protocol_version: LEASE_PROTOCOL.into(),
            verified_at_ms: 300,
            valid_until_ms: 900,
        };
        assert_eq!(
            qualify_covered_current_head(&lineage(), future, 200),
            Err(CoveredHeadError::EvidenceLease(
                EvidenceLeaseError::VerificationFromFuture
            ))
        );

        let mut expired = observation();
        expired.evidence_lease = EvidenceLease {
            protocol_version: LEASE_PROTOCOL.into(),
            verified_at_ms: 100,
            valid_until_ms: 150,
        };
        assert_eq!(
            qualify_covered_current_head(&lineage(), expired, 200),
            Err(CoveredHeadError::EvidenceLease(
                EvidenceLeaseError::ExpiredLease
            ))
        );
    }

    #[test]
    fn scheduled_transition_horizon_must_already_bound_coverage() {
        let mut overlong = observation();
        overlong.next_known_transition_effective_at_ms = Some(800);
        assert_eq!(
            qualify_covered_current_head(&lineage(), overlong, 200),
            Err(CoveredHeadError::ScheduledTransitionHorizonViolation)
        );

        let mut bounded = observation();
        bounded.evidence_lease = lease(100, 800, 200);
        bounded.next_known_transition_effective_at_ms = Some(800);
        let qualified = qualify_covered_current_head(&lineage(), bounded, 200).unwrap();
        assert_eq!(qualified.evidence_lease().valid_until_ms, 800);
        assert_eq!(qualified.next_known_transition_effective_at_ms(), Some(800));
    }

    #[test]
    fn already_effective_scheduled_transition_denies_old_head() {
        let mut observed = observation();
        observed.evidence_lease = EvidenceLease {
            protocol_version: LEASE_PROTOCOL.into(),
            verified_at_ms: 100,
            valid_until_ms: 900,
        };
        observed.next_known_transition_effective_at_ms = Some(200);
        assert_eq!(
            qualify_covered_current_head(&lineage(), observed, 200),
            Err(CoveredHeadError::ScheduledTransitionAlreadyEffective)
        );
    }

    #[test]
    fn positive_token_must_be_revalidated_before_reuse() {
        let qualified = qualify_covered_current_head(&lineage(), observation(), 200).unwrap();
        assert_eq!(qualified.validate_reuse_at(899), Ok(()));
        assert_eq!(
            qualified.validate_reuse_at(900),
            Err(CoveredHeadError::EvidenceLease(
                EvidenceLeaseError::ExpiredLease
            ))
        );
    }

    #[test]
    fn refreshed_dynamic_evidence_does_not_change_stable_lineage_identity() {
        let stable = lineage();
        let first = qualify_covered_current_head(&stable, observation(), 200).unwrap();
        let mut refreshed = observation();
        refreshed.coverage_evidence_identity = id("example-coverage-evidence-v1", 0x77);
        refreshed.verification_evidence_identity = id("example-verification-evidence-v1", 0x88);
        refreshed.evidence_lease = lease(180, 950, 200);
        let second = qualify_covered_current_head(&stable, refreshed, 200).unwrap();

        assert_eq!(
            first.lineage_stable_commitment(),
            second.lineage_stable_commitment()
        );
        assert_ne!(
            first.coverage_evidence_identity(),
            second.coverage_evidence_identity()
        );
        assert_ne!(
            first.verification_evidence_identity(),
            second.verification_evidence_identity()
        );
    }
}
