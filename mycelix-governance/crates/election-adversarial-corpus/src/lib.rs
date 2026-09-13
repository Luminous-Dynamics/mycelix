//! Deterministic structural adversarial corpus for Mycelix public elections.
//!
//! ADV-001 executes the fail-closed contracts established by ELECT-001..012.
//! It intentionally distinguishes structural attacks that are executable today
//! from cryptographic properties that remain blocked on concrete protocol choice.

use std::collections::BTreeSet;

use election_anonymous_authority as authority;
use election_integrity_types::{Digest32, ElectionTheoremId, PUBLIC_ELECTION_PROFILE_ID};
use election_physical_evidence as physical;
use election_transparency_witness as transparency;
use election_verifier_contract as verifier;

pub const ADVERSARIAL_CORPUS_PROFILE_ID: &str = "mycelix-public-election-adversarial-corpus-v1";
pub const MIN_REQUIRED_EXECUTABLE_CASES: usize = 26;

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum AttackCaseId {
    ConflictingNullifier,
    NullifierCensusConflict,
    NullifierCensusCountMismatch,
    CheckpointSequenceGap,
    CheckpointPredecessorMismatch,
    SplitViewEquivocation,
    WitnessControlDomainCollapse,
    DuplicateWitnessKey,
    PackagePathTraversal,
    PackageSecretMaterial,
    PackageMissingChallengeLedger,
    OfflineNetworkDependency,
    VerifierMissingStage,
    VerifierFailedStageDominates,
    VerifierLineageCollapse,
    VerifierBuilderCollapse,
    BallotConservationLoss,
    BallotAccountingOverflow,
    CustodySequenceGap,
    CustodyStateDiscontinuity,
    CustodySiblingFork,
    CvrDuplicateOrMissing,
    InvalidRlaRiskLimit,
    AuditSampleExceedsPopulation,
    MissingAdjudicationEvidence,
    PhysicalFailureDominatesVerifier,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum CorpusLayer {
    AnonymousAuthority,
    TransparencyLineage,
    WitnessQuorum,
    EvidencePackage,
    OfflineExecution,
    VerifierRun,
    VerifierAgreement,
    PhysicalAccounting,
    PhysicalCustody,
    CaptureReconciliation,
    AuditPolicy,
    AuditEvidence,
    CrossLayerComposition,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ExpectedDisposition {
    FailClosed,
    FreezePendingEvidenceResolution,
    Discrepant,
    AggregateFailure,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum FindingId {
    ConflictingUseOfSameAuthority,
    ConflictingAuthorityObserved,
    NullifierCountMismatch,
    CheckpointSequenceNotContiguous,
    CheckpointPredecessorMismatch,
    SplitViewEquivocation,
    InsufficientIndependentWitnessDomains,
    DuplicateWitnessKey,
    NonCanonicalPackagePath,
    NonPublicArtifactForbidden,
    MissingChallengeLedger,
    NetworkAccessForbidden,
    MissingCertificationStage,
    FailedStageDominates,
    InsufficientVerifierLineages,
    InsufficientVerifierBuilderDomains,
    BallotConservationFailure,
    BallotAccountingOverflow,
    CustodySequenceNotContiguous,
    CustodyContainerStateDiscontinuity,
    CustodySiblingFork,
    CvrReconciliationDiscrepant,
    InvalidRiskLimit,
    AuditSampleExceedsPopulation,
    MissingAdjudicationEvidence,
    PhysicalAuditFailureDominates,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct AdversarialCaseSpecV1 {
    pub id: AttackCaseId,
    pub layer: CorpusLayer,
    pub primary_theorem: ElectionTheoremId,
    pub expected_disposition: ExpectedDisposition,
    pub expected_finding: FindingId,
    pub certification_blocking: bool,
    pub single_fault: bool,
}

pub const REQUIRED_EXECUTABLE_CASES: [AdversarialCaseSpecV1; 26] = [
    AdversarialCaseSpecV1 {
        id: AttackCaseId::ConflictingNullifier,
        layer: CorpusLayer::AnonymousAuthority,
        primary_theorem: ElectionTheoremId::Uniqueness,
        expected_disposition: ExpectedDisposition::FreezePendingEvidenceResolution,
        expected_finding: FindingId::ConflictingUseOfSameAuthority,
        certification_blocking: true,
        single_fault: true,
    },
    AdversarialCaseSpecV1 {
        id: AttackCaseId::NullifierCensusConflict,
        layer: CorpusLayer::AnonymousAuthority,
        primary_theorem: ElectionTheoremId::Uniqueness,
        expected_disposition: ExpectedDisposition::FailClosed,
        expected_finding: FindingId::ConflictingAuthorityObserved,
        certification_blocking: true,
        single_fault: true,
    },
    AdversarialCaseSpecV1 {
        id: AttackCaseId::NullifierCensusCountMismatch,
        layer: CorpusLayer::AnonymousAuthority,
        primary_theorem: ElectionTheoremId::Uniqueness,
        expected_disposition: ExpectedDisposition::FailClosed,
        expected_finding: FindingId::NullifierCountMismatch,
        certification_blocking: true,
        single_fault: true,
    },
    AdversarialCaseSpecV1 {
        id: AttackCaseId::CheckpointSequenceGap,
        layer: CorpusLayer::TransparencyLineage,
        primary_theorem: ElectionTheoremId::EvidenceContinuity,
        expected_disposition: ExpectedDisposition::FailClosed,
        expected_finding: FindingId::CheckpointSequenceNotContiguous,
        certification_blocking: true,
        single_fault: true,
    },
    AdversarialCaseSpecV1 {
        id: AttackCaseId::CheckpointPredecessorMismatch,
        layer: CorpusLayer::TransparencyLineage,
        primary_theorem: ElectionTheoremId::EvidenceContinuity,
        expected_disposition: ExpectedDisposition::FailClosed,
        expected_finding: FindingId::CheckpointPredecessorMismatch,
        certification_blocking: true,
        single_fault: true,
    },
    AdversarialCaseSpecV1 {
        id: AttackCaseId::SplitViewEquivocation,
        layer: CorpusLayer::TransparencyLineage,
        primary_theorem: ElectionTheoremId::EvidenceContinuity,
        expected_disposition: ExpectedDisposition::FailClosed,
        expected_finding: FindingId::SplitViewEquivocation,
        certification_blocking: true,
        single_fault: true,
    },
    AdversarialCaseSpecV1 {
        id: AttackCaseId::WitnessControlDomainCollapse,
        layer: CorpusLayer::WitnessQuorum,
        primary_theorem: ElectionTheoremId::AdministrativeNonAuthority,
        expected_disposition: ExpectedDisposition::FailClosed,
        expected_finding: FindingId::InsufficientIndependentWitnessDomains,
        certification_blocking: true,
        single_fault: true,
    },
    AdversarialCaseSpecV1 {
        id: AttackCaseId::DuplicateWitnessKey,
        layer: CorpusLayer::WitnessQuorum,
        primary_theorem: ElectionTheoremId::AdministrativeNonAuthority,
        expected_disposition: ExpectedDisposition::FailClosed,
        expected_finding: FindingId::DuplicateWitnessKey,
        certification_blocking: true,
        single_fault: true,
    },
    AdversarialCaseSpecV1 {
        id: AttackCaseId::PackagePathTraversal,
        layer: CorpusLayer::EvidencePackage,
        primary_theorem: ElectionTheoremId::RecoverableVerification,
        expected_disposition: ExpectedDisposition::FailClosed,
        expected_finding: FindingId::NonCanonicalPackagePath,
        certification_blocking: true,
        single_fault: true,
    },
    AdversarialCaseSpecV1 {
        id: AttackCaseId::PackageSecretMaterial,
        layer: CorpusLayer::EvidencePackage,
        primary_theorem: ElectionTheoremId::BallotSecrecy,
        expected_disposition: ExpectedDisposition::FailClosed,
        expected_finding: FindingId::NonPublicArtifactForbidden,
        certification_blocking: true,
        single_fault: true,
    },
    AdversarialCaseSpecV1 {
        id: AttackCaseId::PackageMissingChallengeLedger,
        layer: CorpusLayer::EvidencePackage,
        primary_theorem: ElectionTheoremId::EvidenceBeforeCertification,
        expected_disposition: ExpectedDisposition::FailClosed,
        expected_finding: FindingId::MissingChallengeLedger,
        certification_blocking: true,
        single_fault: true,
    },
    AdversarialCaseSpecV1 {
        id: AttackCaseId::OfflineNetworkDependency,
        layer: CorpusLayer::OfflineExecution,
        primary_theorem: ElectionTheoremId::RecoverableVerification,
        expected_disposition: ExpectedDisposition::FailClosed,
        expected_finding: FindingId::NetworkAccessForbidden,
        certification_blocking: true,
        single_fault: true,
    },
    AdversarialCaseSpecV1 {
        id: AttackCaseId::VerifierMissingStage,
        layer: CorpusLayer::VerifierRun,
        primary_theorem: ElectionTheoremId::EvidenceBeforeCertification,
        expected_disposition: ExpectedDisposition::FailClosed,
        expected_finding: FindingId::MissingCertificationStage,
        certification_blocking: true,
        single_fault: true,
    },
    AdversarialCaseSpecV1 {
        id: AttackCaseId::VerifierFailedStageDominates,
        layer: CorpusLayer::VerifierRun,
        primary_theorem: ElectionTheoremId::RecoverableVerification,
        expected_disposition: ExpectedDisposition::AggregateFailure,
        expected_finding: FindingId::FailedStageDominates,
        certification_blocking: true,
        single_fault: false,
    },
    AdversarialCaseSpecV1 {
        id: AttackCaseId::VerifierLineageCollapse,
        layer: CorpusLayer::VerifierAgreement,
        primary_theorem: ElectionTheoremId::AdministrativeNonAuthority,
        expected_disposition: ExpectedDisposition::FailClosed,
        expected_finding: FindingId::InsufficientVerifierLineages,
        certification_blocking: true,
        single_fault: true,
    },
    AdversarialCaseSpecV1 {
        id: AttackCaseId::VerifierBuilderCollapse,
        layer: CorpusLayer::VerifierAgreement,
        primary_theorem: ElectionTheoremId::AdministrativeNonAuthority,
        expected_disposition: ExpectedDisposition::FailClosed,
        expected_finding: FindingId::InsufficientVerifierBuilderDomains,
        certification_blocking: true,
        single_fault: true,
    },
    AdversarialCaseSpecV1 {
        id: AttackCaseId::BallotConservationLoss,
        layer: CorpusLayer::PhysicalAccounting,
        primary_theorem: ElectionTheoremId::SoftwareIndependence,
        expected_disposition: ExpectedDisposition::FailClosed,
        expected_finding: FindingId::BallotConservationFailure,
        certification_blocking: true,
        single_fault: true,
    },
    AdversarialCaseSpecV1 {
        id: AttackCaseId::BallotAccountingOverflow,
        layer: CorpusLayer::PhysicalAccounting,
        primary_theorem: ElectionTheoremId::SoftwareIndependence,
        expected_disposition: ExpectedDisposition::FailClosed,
        expected_finding: FindingId::BallotAccountingOverflow,
        certification_blocking: true,
        single_fault: true,
    },
    AdversarialCaseSpecV1 {
        id: AttackCaseId::CustodySequenceGap,
        layer: CorpusLayer::PhysicalCustody,
        primary_theorem: ElectionTheoremId::EvidenceContinuity,
        expected_disposition: ExpectedDisposition::FailClosed,
        expected_finding: FindingId::CustodySequenceNotContiguous,
        certification_blocking: true,
        single_fault: true,
    },
    AdversarialCaseSpecV1 {
        id: AttackCaseId::CustodyStateDiscontinuity,
        layer: CorpusLayer::PhysicalCustody,
        primary_theorem: ElectionTheoremId::EvidenceContinuity,
        expected_disposition: ExpectedDisposition::FailClosed,
        expected_finding: FindingId::CustodyContainerStateDiscontinuity,
        certification_blocking: true,
        single_fault: true,
    },
    AdversarialCaseSpecV1 {
        id: AttackCaseId::CustodySiblingFork,
        layer: CorpusLayer::PhysicalCustody,
        primary_theorem: ElectionTheoremId::EvidenceContinuity,
        expected_disposition: ExpectedDisposition::FreezePendingEvidenceResolution,
        expected_finding: FindingId::CustodySiblingFork,
        certification_blocking: true,
        single_fault: true,
    },
    AdversarialCaseSpecV1 {
        id: AttackCaseId::CvrDuplicateOrMissing,
        layer: CorpusLayer::CaptureReconciliation,
        primary_theorem: ElectionTheoremId::SoftwareIndependence,
        expected_disposition: ExpectedDisposition::Discrepant,
        expected_finding: FindingId::CvrReconciliationDiscrepant,
        certification_blocking: true,
        single_fault: true,
    },
    AdversarialCaseSpecV1 {
        id: AttackCaseId::InvalidRlaRiskLimit,
        layer: CorpusLayer::AuditPolicy,
        primary_theorem: ElectionTheoremId::EvidenceBeforeCertification,
        expected_disposition: ExpectedDisposition::FailClosed,
        expected_finding: FindingId::InvalidRiskLimit,
        certification_blocking: true,
        single_fault: true,
    },
    AdversarialCaseSpecV1 {
        id: AttackCaseId::AuditSampleExceedsPopulation,
        layer: CorpusLayer::AuditEvidence,
        primary_theorem: ElectionTheoremId::SoftwareIndependence,
        expected_disposition: ExpectedDisposition::FailClosed,
        expected_finding: FindingId::AuditSampleExceedsPopulation,
        certification_blocking: true,
        single_fault: true,
    },
    AdversarialCaseSpecV1 {
        id: AttackCaseId::MissingAdjudicationEvidence,
        layer: CorpusLayer::AuditEvidence,
        primary_theorem: ElectionTheoremId::EvidenceBeforeCertification,
        expected_disposition: ExpectedDisposition::FailClosed,
        expected_finding: FindingId::MissingAdjudicationEvidence,
        certification_blocking: true,
        single_fault: true,
    },
    AdversarialCaseSpecV1 {
        id: AttackCaseId::PhysicalFailureDominatesVerifier,
        layer: CorpusLayer::CrossLayerComposition,
        primary_theorem: ElectionTheoremId::EvidenceBeforeCertification,
        expected_disposition: ExpectedDisposition::AggregateFailure,
        expected_finding: FindingId::PhysicalAuditFailureDominates,
        certification_blocking: true,
        single_fault: false,
    },
];

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum ProtocolGapId {
    CryptographicEligibilitySoundness,
    BallotSecrecyAgainstConcreteTranscript,
    ReceiptFreeness,
    CastAsIntended,
    RecordedAsCast,
    TalliedAsRecorded,
    CoercionResistance,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ProtocolDependentGapV1 {
    pub id: ProtocolGapId,
    pub primary_theorem: ElectionTheoremId,
    pub blocks_protocol_security_claim: bool,
}

pub const PROTOCOL_DEPENDENT_GAPS: [ProtocolDependentGapV1; 7] = [
    ProtocolDependentGapV1 {
        id: ProtocolGapId::CryptographicEligibilitySoundness,
        primary_theorem: ElectionTheoremId::Eligibility,
        blocks_protocol_security_claim: true,
    },
    ProtocolDependentGapV1 {
        id: ProtocolGapId::BallotSecrecyAgainstConcreteTranscript,
        primary_theorem: ElectionTheoremId::BallotSecrecy,
        blocks_protocol_security_claim: true,
    },
    ProtocolDependentGapV1 {
        id: ProtocolGapId::ReceiptFreeness,
        primary_theorem: ElectionTheoremId::ReceiptFreeness,
        blocks_protocol_security_claim: true,
    },
    ProtocolDependentGapV1 {
        id: ProtocolGapId::CastAsIntended,
        primary_theorem: ElectionTheoremId::CastAsIntended,
        blocks_protocol_security_claim: true,
    },
    ProtocolDependentGapV1 {
        id: ProtocolGapId::RecordedAsCast,
        primary_theorem: ElectionTheoremId::RecordedAsCast,
        blocks_protocol_security_claim: true,
    },
    ProtocolDependentGapV1 {
        id: ProtocolGapId::TalliedAsRecorded,
        primary_theorem: ElectionTheoremId::TalliedAsRecorded,
        blocks_protocol_security_claim: true,
    },
    ProtocolDependentGapV1 {
        id: ProtocolGapId::CoercionResistance,
        primary_theorem: ElectionTheoremId::ReceiptFreeness,
        blocks_protocol_security_claim: true,
    },
];

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CorpusRegistryViolation {
    TooFewExecutableCases,
    DuplicateAttackCase,
    DuplicateProtocolGap,
    NonBlockingAttackCase,
    NonBlockingProtocolGap,
}

pub fn validate_corpus_registry() -> Result<(), CorpusRegistryViolation> {
    if REQUIRED_EXECUTABLE_CASES.len() < MIN_REQUIRED_EXECUTABLE_CASES {
        return Err(CorpusRegistryViolation::TooFewExecutableCases);
    }

    let mut attack_ids = BTreeSet::new();
    for case in REQUIRED_EXECUTABLE_CASES {
        if !attack_ids.insert(case.id) {
            return Err(CorpusRegistryViolation::DuplicateAttackCase);
        }
        if !case.certification_blocking {
            return Err(CorpusRegistryViolation::NonBlockingAttackCase);
        }
    }

    let mut gap_ids = BTreeSet::new();
    for gap in PROTOCOL_DEPENDENT_GAPS {
        if !gap_ids.insert(gap.id) {
            return Err(CorpusRegistryViolation::DuplicateProtocolGap);
        }
        if !gap.blocks_protocol_security_claim {
            return Err(CorpusRegistryViolation::NonBlockingProtocolGap);
        }
    }
    Ok(())
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CaseObservationV1 {
    pub disposition: ExpectedDisposition,
    pub finding: FindingId,
}

fn observation(disposition: ExpectedDisposition, finding: FindingId) -> CaseObservationV1 {
    CaseObservationV1 {
        disposition,
        finding,
    }
}

fn digest(byte: u8) -> Digest32 {
    [byte; 32]
}

fn authorization_scope(contest: u8) -> authority::AuthorizationScopeV1 {
    authority::AuthorizationScopeV1 {
        election_constitution_digest: digest(1),
        election_definition_digest: digest(2),
        contest_scope_digest: digest(contest),
        eligibility_rules_digest: digest(4),
    }
}

fn authority_claim(nullifier: u8, ballot: u8, proof: u8) -> authority::ScopedAuthorityClaimV1 {
    authority::ScopedAuthorityClaimV1 {
        scope: authorization_scope(3),
        scope_local_nullifier: digest(nullifier),
        ballot_commitment_digest: digest(ballot),
        anonymous_eligibility_proof_digest: digest(proof),
    }
}

fn checkpoint(
    sequence: u64,
    tree_size: u64,
    predecessor: Option<Digest32>,
) -> transparency::ElectionTransparencyCheckpointV1 {
    transparency::ElectionTransparencyCheckpointV1 {
        public_election_profile_id: PUBLIC_ELECTION_PROFILE_ID.to_owned(),
        transparency_profile_id: transparency::TRANSPARENCY_PROFILE_ID.to_owned(),
        election_constitution_digest: digest(1),
        log_parameters_digest: digest(2),
        canonicalization_profile_digest: digest(3),
        checkpoint_sequence: sequence,
        tree_size,
        root_digest: digest(20 + sequence as u8),
        previous_checkpoint_digest: predecessor,
        consistency_proof_digest: if sequence == 0 {
            None
        } else {
            Some(digest(30))
        },
    }
}

fn witness_policy() -> transparency::WitnessQuorumPolicyV1 {
    transparency::WitnessQuorumPolicyV1 {
        witness_profile_id: transparency::WITNESS_PROFILE_ID.to_owned(),
        required_distinct_control_domains: 3,
        minimum_total_witnesses: 3,
        control_domain_profile_digest: digest(40),
        witness_signature_profile_digest: digest(41),
    }
}

fn witness_attestation(key: u8, domain: u8) -> transparency::WitnessAttestationV1 {
    transparency::WitnessAttestationV1 {
        witness_profile_id: transparency::WITNESS_PROFILE_ID.to_owned(),
        checkpoint_digest: digest(42),
        witness_key_digest: digest(key),
        control_domain_digest: digest(domain),
        control_domain_credential_digest: digest(domain.wrapping_add(20)),
        observation_evidence_digest: digest(key.wrapping_add(40)),
        attestation_digest: digest(key.wrapping_add(60)),
    }
}

fn artifact(
    kind: verifier::EvidenceArtifactKind,
    path: &str,
    byte: u8,
) -> verifier::EvidenceArtifactRefV1 {
    verifier::EvidenceArtifactRefV1 {
        kind,
        canonical_path: path.to_owned(),
        content_digest: digest(byte),
        byte_length: 10,
        disclosure: verifier::ArtifactDisclosureClass::PublicVerificationEvidence,
    }
}

fn valid_manifest() -> verifier::ElectionEvidencePackageManifestV1 {
    verifier::ElectionEvidencePackageManifestV1 {
        public_election_profile_id: PUBLIC_ELECTION_PROFILE_ID.to_owned(),
        evidence_package_profile_id: verifier::EVIDENCE_PACKAGE_PROFILE_ID.to_owned(),
        election_constitution_digest: digest(1),
        package_canonicalization_profile_digest: digest(2),
        artifact_index_digest: digest(3),
        package_root_digest: digest(4),
        final_transparency_checkpoint_digest: digest(5),
        artifacts: vec![
            artifact(
                verifier::EvidenceArtifactKind::ElectionConstitution,
                "election/constitution.bin",
                10,
            ),
            artifact(
                verifier::EvidenceArtifactKind::TransparencyCheckpointChain,
                "transparency/checkpoints.bin",
                11,
            ),
            artifact(
                verifier::EvidenceArtifactKind::WitnessAttestations,
                "transparency/witnesses.bin",
                12,
            ),
            artifact(
                verifier::EvidenceArtifactKind::AnonymousAuthorityCensus,
                "authority/census.bin",
                13,
            ),
            artifact(
                verifier::EvidenceArtifactKind::TallyEvidence,
                "tally/evidence.bin",
                14,
            ),
            artifact(
                verifier::EvidenceArtifactKind::PhysicalAuditEvidence,
                "physical/audit.bin",
                15,
            ),
            artifact(
                verifier::EvidenceArtifactKind::ChallengeLedger,
                "challenges/ledger.bin",
                16,
            ),
            artifact(
                verifier::EvidenceArtifactKind::CertificationPolicy,
                "certification/policy.bin",
                17,
            ),
        ],
        interoperability_profiles: Vec::new(),
    }
}

fn stage_receipt(
    stage: verifier::VerificationStageId,
    disposition: verifier::VerificationStageDisposition,
) -> verifier::VerificationStageReceiptV1 {
    verifier::VerificationStageReceiptV1 {
        stage,
        package_root_digest: digest(4),
        subject_digest: digest(30 + stage as u8),
        verifier_release_digest: digest(20),
        disposition,
        finding_digest: digest(60 + stage as u8),
    }
}

fn valid_verifier_run() -> verifier::VerifierRunReceiptV1 {
    verifier::VerifierRunReceiptV1 {
        offline_verifier_profile_id: verifier::OFFLINE_VERIFIER_PROFILE_ID.to_owned(),
        package_root_digest: digest(4),
        verifier_implementation_id: "corpus-reference-verifier".to_owned(),
        verifier_lineage_digest: digest(18),
        verifier_release_digest: digest(20),
        source_digest: digest(21),
        build_provenance_digest: digest(22),
        execution_policy_digest: digest(23),
        stages: verifier::REQUIRED_VERIFICATION_STAGES
            .iter()
            .copied()
            .map(|stage| stage_receipt(stage, verifier::VerificationStageDisposition::Pass))
            .collect(),
    }
}

fn agreement_attestation(
    release: u8,
    lineage: u8,
    builder: u8,
) -> verifier::VerifierAgreementAttestationV1 {
    verifier::VerifierAgreementAttestationV1 {
        package_root_digest: digest(4),
        verifier_lineage_digest: digest(lineage),
        verifier_release_digest: digest(release),
        builder_control_domain_digest: digest(builder),
        run_receipt_digest: digest(release.wrapping_add(40)),
        run_disposition: verifier::VerificationStageDisposition::Pass,
    }
}

fn agreement_policy() -> verifier::VerifierAgreementPolicyV1 {
    verifier::VerifierAgreementPolicyV1 {
        minimum_total_verifiers: 3,
        minimum_distinct_implementation_lineages: 3,
        minimum_distinct_builder_control_domains: 2,
    }
}

fn accounting_scope() -> physical::BallotAccountingScopeV1 {
    physical::BallotAccountingScopeV1 {
        election_constitution_digest: digest(1),
        jurisdiction_scope_digest: digest(2),
        location_or_batch_scope_digest: digest(3),
        ballot_style_digest: digest(4),
        physical_unit_definition_digest: digest(5),
    }
}

fn valid_accounting() -> physical::BallotStockAccountingV1 {
    physical::BallotStockAccountingV1 {
        public_election_profile_id: PUBLIC_ELECTION_PROFILE_ID.to_owned(),
        physical_evidence_profile_id: physical::PHYSICAL_EVIDENCE_PROFILE_ID.to_owned(),
        scope: accounting_scope(),
        opening_stock: 100,
        supplemental_stock_received: 0,
        cast_regular: 80,
        cast_provisional_sealed: 0,
        spoiled: 5,
        issued_not_cast: 0,
        unused: 15,
        quarantined: 0,
        exception_evidence_digest: None,
        accounting_evidence_digest: digest(7),
    }
}

fn sealed(container: u8, seal: u8) -> physical::ContainerStateV1 {
    physical::ContainerStateV1 {
        container_identifier_digest: digest(container),
        seal_state: physical::SealStateV1::Sealed {
            seal_identifier_digest: digest(seal),
            seal_application_evidence_digest: digest(seal.wrapping_add(20)),
        },
    }
}

fn custody_event(sequence: u64, predecessor: Option<Digest32>) -> physical::CustodyEventV1 {
    physical::CustodyEventV1 {
        election_constitution_digest: digest(1),
        ballot_batch_digest: digest(10),
        custody_sequence: sequence,
        previous_event_digest: predecessor,
        event_kind: physical::CustodyEventKind::SecureStorageTransfer,
        from_control_domain_digest: digest(11),
        to_control_domain_digest: digest(12),
        before_state: sealed(13, 14),
        after_state: sealed(13, 15),
        governing_authorization_digest: digest(16),
        witness_evidence_digest: digest(17),
        temporal_evidence_digest: digest(18),
        event_evidence_digest: digest(19),
    }
}

pub fn execute_case(id: AttackCaseId) -> CaseObservationV1 {
    match id {
        AttackCaseId::ConflictingNullifier => {
            let first = authority_claim(11, 12, 13);
            let conflicting = authority_claim(11, 99, 14);
            assert_eq!(
                authority::classify_pairwise_authority(&first, &conflicting),
                authority::PairwiseAuthorityClassification::ConflictingUseOfSameAuthority
            );
            observation(
                ExpectedDisposition::FreezePendingEvidenceResolution,
                FindingId::ConflictingUseOfSameAuthority,
            )
        }
        AttackCaseId::NullifierCensusConflict => {
            let census = authority::NullifierCensusV1 {
                scope: authorization_scope(3),
                complete_checkpoint_digest: digest(20),
                observed_claim_count: 101,
                unique_nullifier_count: 100,
                conflicting_nullifier_count: 1,
            };
            assert_eq!(
                authority::validate_nullifier_census_for_tally(&census),
                Err(authority::NullifierCensusViolation::ConflictingAuthorityObserved)
            );
            observation(
                ExpectedDisposition::FailClosed,
                FindingId::ConflictingAuthorityObserved,
            )
        }
        AttackCaseId::NullifierCensusCountMismatch => {
            let census = authority::NullifierCensusV1 {
                scope: authorization_scope(3),
                complete_checkpoint_digest: digest(20),
                observed_claim_count: 100,
                unique_nullifier_count: 99,
                conflicting_nullifier_count: 0,
            };
            assert_eq!(
                authority::validate_nullifier_census_for_tally(&census),
                Err(authority::NullifierCensusViolation::CountMismatch)
            );
            observation(
                ExpectedDisposition::FailClosed,
                FindingId::NullifierCountMismatch,
            )
        }
        AttackCaseId::CheckpointSequenceGap => {
            let previous = checkpoint(0, 10, None);
            let next = checkpoint(2, 11, Some(digest(50)));
            assert_eq!(
                transparency::validate_checkpoint_successor(&previous, digest(50), &next),
                Err(transparency::CheckpointSuccessorViolation::SequenceNotContiguous)
            );
            observation(
                ExpectedDisposition::FailClosed,
                FindingId::CheckpointSequenceNotContiguous,
            )
        }
        AttackCaseId::CheckpointPredecessorMismatch => {
            let previous = checkpoint(0, 10, None);
            let next = checkpoint(1, 11, Some(digest(51)));
            assert_eq!(
                transparency::validate_checkpoint_successor(&previous, digest(50), &next),
                Err(transparency::CheckpointSuccessorViolation::PredecessorDigestMismatch)
            );
            observation(
                ExpectedDisposition::FailClosed,
                FindingId::CheckpointPredecessorMismatch,
            )
        }
        AttackCaseId::SplitViewEquivocation => {
            let first = transparency::CheckpointRefV1 {
                checkpoint_digest: digest(60),
                election_constitution_digest: digest(1),
                checkpoint_sequence: 4,
                tree_size: 100,
                root_digest: digest(61),
                previous_checkpoint_digest: Some(digest(59)),
            };
            let second = transparency::CheckpointRefV1 {
                checkpoint_digest: digest(62),
                root_digest: digest(63),
                ..first.clone()
            };
            assert_eq!(
                transparency::classify_checkpoint_pair(&first, &second),
                transparency::CheckpointPairClassification::EquivocatingSuccessors
            );
            observation(
                ExpectedDisposition::FailClosed,
                FindingId::SplitViewEquivocation,
            )
        }
        AttackCaseId::WitnessControlDomainCollapse => {
            let attestations = vec![
                witness_attestation(1, 8),
                witness_attestation(2, 8),
                witness_attestation(3, 8),
            ];
            assert_eq!(
                transparency::validate_witness_quorum(digest(42), &witness_policy(), &attestations),
                Err(transparency::WitnessQuorumViolation::InsufficientIndependentControlDomains)
            );
            observation(
                ExpectedDisposition::FailClosed,
                FindingId::InsufficientIndependentWitnessDomains,
            )
        }
        AttackCaseId::DuplicateWitnessKey => {
            let attestations = vec![
                witness_attestation(1, 8),
                witness_attestation(1, 9),
                witness_attestation(3, 10),
            ];
            assert_eq!(
                transparency::validate_witness_quorum(digest(42), &witness_policy(), &attestations),
                Err(transparency::WitnessQuorumViolation::DuplicateWitnessKey)
            );
            observation(
                ExpectedDisposition::FailClosed,
                FindingId::DuplicateWitnessKey,
            )
        }
        AttackCaseId::PackagePathTraversal => {
            let mut manifest = valid_manifest();
            manifest.artifacts[0].canonical_path = "../constitution.bin".to_owned();
            assert_eq!(
                verifier::validate_evidence_package_manifest(&manifest),
                Err(verifier::EvidencePackageViolation::Artifact {
                    index: 0,
                    violation: verifier::ArtifactRefViolation::NonCanonicalPathComponent,
                })
            );
            observation(
                ExpectedDisposition::FailClosed,
                FindingId::NonCanonicalPackagePath,
            )
        }
        AttackCaseId::PackageSecretMaterial => {
            let mut manifest = valid_manifest();
            manifest.artifacts[0].disclosure = verifier::ArtifactDisclosureClass::NonPublicOrSecret;
            assert_eq!(
                verifier::validate_evidence_package_manifest(&manifest),
                Err(verifier::EvidencePackageViolation::Artifact {
                    index: 0,
                    violation: verifier::ArtifactRefViolation::NonPublicArtifactForbidden,
                })
            );
            observation(
                ExpectedDisposition::FailClosed,
                FindingId::NonPublicArtifactForbidden,
            )
        }
        AttackCaseId::PackageMissingChallengeLedger => {
            let mut manifest = valid_manifest();
            manifest
                .artifacts
                .retain(|item| item.kind != verifier::EvidenceArtifactKind::ChallengeLedger);
            assert_eq!(
                verifier::validate_evidence_package_manifest(&manifest),
                Err(
                    verifier::EvidencePackageViolation::MissingRequiredArtifactKind(
                        verifier::EvidenceArtifactKind::ChallengeLedger,
                    )
                )
            );
            observation(
                ExpectedDisposition::FailClosed,
                FindingId::MissingChallengeLedger,
            )
        }
        AttackCaseId::OfflineNetworkDependency => {
            let policy = verifier::OfflineVerifierExecutionPolicyV1 {
                network_access: verifier::CapabilityRule::Permitted,
                ..verifier::OfflineVerifierExecutionPolicyV1::default()
            };
            assert_eq!(
                verifier::validate_offline_execution_policy(&policy),
                Err(verifier::ExecutionPolicyViolation::NetworkAccessMustBeForbidden)
            );
            observation(
                ExpectedDisposition::FailClosed,
                FindingId::NetworkAccessForbidden,
            )
        }
        AttackCaseId::VerifierMissingStage => {
            let mut run = valid_verifier_run();
            run.stages.retain(|stage| {
                stage.stage != verifier::VerificationStageId::CertificationEvidence
            });
            assert_eq!(
                verifier::classify_verifier_run(&run),
                Err(verifier::VerifierRunViolation::MissingStage(
                    verifier::VerificationStageId::CertificationEvidence,
                ))
            );
            observation(
                ExpectedDisposition::FailClosed,
                FindingId::MissingCertificationStage,
            )
        }
        AttackCaseId::VerifierFailedStageDominates => {
            let mut run = valid_verifier_run();
            run.stages[0].disposition = verifier::VerificationStageDisposition::Indeterminate;
            run.stages[1].disposition = verifier::VerificationStageDisposition::Fail;
            assert_eq!(
                verifier::classify_verifier_run(&run),
                Ok(verifier::VerifierRunDisposition::AtLeastOneStageFailed)
            );
            observation(
                ExpectedDisposition::AggregateFailure,
                FindingId::FailedStageDominates,
            )
        }
        AttackCaseId::VerifierLineageCollapse => {
            let attestations = vec![
                agreement_attestation(1, 9, 20),
                agreement_attestation(2, 9, 21),
                agreement_attestation(3, 9, 22),
            ];
            assert_eq!(
                verifier::validate_independent_verifier_agreement(
                    digest(4),
                    &agreement_policy(),
                    &attestations,
                ),
                Err(verifier::VerifierAgreementViolation::InsufficientImplementationLineages)
            );
            observation(
                ExpectedDisposition::FailClosed,
                FindingId::InsufficientVerifierLineages,
            )
        }
        AttackCaseId::VerifierBuilderCollapse => {
            let attestations = vec![
                agreement_attestation(1, 9, 20),
                agreement_attestation(2, 10, 20),
                agreement_attestation(3, 11, 20),
            ];
            assert_eq!(
                verifier::validate_independent_verifier_agreement(
                    digest(4),
                    &agreement_policy(),
                    &attestations,
                ),
                Err(verifier::VerifierAgreementViolation::InsufficientBuilderControlDomains)
            );
            observation(
                ExpectedDisposition::FailClosed,
                FindingId::InsufficientVerifierBuilderDomains,
            )
        }
        AttackCaseId::BallotConservationLoss => {
            let mut accounting = valid_accounting();
            accounting.unused = 14;
            assert_eq!(
                physical::validate_ballot_stock_accounting(&accounting),
                Err(physical::BallotAccountingViolation::ConservationFailure {
                    available: 100,
                    dispositioned: 99,
                })
            );
            observation(
                ExpectedDisposition::FailClosed,
                FindingId::BallotConservationFailure,
            )
        }
        AttackCaseId::BallotAccountingOverflow => {
            let mut accounting = valid_accounting();
            accounting.opening_stock = u64::MAX;
            accounting.supplemental_stock_received = 1;
            assert_eq!(
                physical::validate_ballot_stock_accounting(&accounting),
                Err(physical::BallotAccountingViolation::CountOverflow)
            );
            observation(
                ExpectedDisposition::FailClosed,
                FindingId::BallotAccountingOverflow,
            )
        }
        AttackCaseId::CustodySequenceGap => {
            let previous = custody_event(0, None);
            let mut next = custody_event(2, Some(digest(30)));
            next.before_state = previous.after_state.clone();
            assert_eq!(
                physical::validate_custody_successor(&previous, digest(30), &next),
                Err(physical::CustodySuccessorViolation::SequenceNotContiguous)
            );
            observation(
                ExpectedDisposition::FailClosed,
                FindingId::CustodySequenceNotContiguous,
            )
        }
        AttackCaseId::CustodyStateDiscontinuity => {
            let previous = custody_event(0, None);
            let next = custody_event(1, Some(digest(30)));
            assert_eq!(
                physical::validate_custody_successor(&previous, digest(30), &next),
                Err(physical::CustodySuccessorViolation::ContainerStateDiscontinuity)
            );
            observation(
                ExpectedDisposition::FailClosed,
                FindingId::CustodyContainerStateDiscontinuity,
            )
        }
        AttackCaseId::CustodySiblingFork => {
            let first = physical::CustodyEventRefV1 {
                custody_sequence: 5,
                event_digest: digest(40),
                previous_event_digest: Some(digest(39)),
            };
            let second = physical::CustodyEventRefV1 {
                custody_sequence: 5,
                event_digest: digest(41),
                previous_event_digest: Some(digest(39)),
            };
            assert_eq!(
                physical::classify_custody_pair(&first, &second),
                physical::CustodyPairClassification::SiblingFork
            );
            observation(
                ExpectedDisposition::FreezePendingEvidenceResolution,
                FindingId::CustodySiblingFork,
            )
        }
        AttackCaseId::CvrDuplicateOrMissing => {
            let reconciliation = physical::CaptureReconciliationV1 {
                election_constitution_digest: digest(1),
                accounting_scope_digest: digest(2),
                cardinality_profile_digest: digest(3),
                physical_cast_unit_count: 100,
                expected_unique_cvr_count: 100,
                observed_unique_cvr_count: 99,
                duplicate_cvr_count: 1,
                unresolved_mapping_count: 0,
                scanner_manifest_digest: digest(4),
                cvr_export_digest: digest(5),
                cvr_interoperability_profile_digest: digest(6),
                reconciliation_evidence_digest: digest(7),
            };
            assert_eq!(
                physical::classify_capture_reconciliation(&reconciliation),
                Ok(physical::CaptureReconciliationDisposition::Discrepant)
            );
            observation(
                ExpectedDisposition::Discrepant,
                FindingId::CvrReconciliationDiscrepant,
            )
        }
        AttackCaseId::InvalidRlaRiskLimit => {
            let policy = physical::AuditMethodPolicyV1 {
                audit_method_class: physical::AuditMethodClass::RiskLimiting,
                audit_policy_digest: digest(1),
                method_profile_digest: digest(2),
                method_parameters_digest: digest(3),
                risk_limit_parts_per_million: Some(0),
            };
            assert_eq!(
                physical::validate_audit_method_policy(&policy),
                Err(physical::AuditMethodPolicyViolation::InvalidRiskLimit)
            );
            observation(ExpectedDisposition::FailClosed, FindingId::InvalidRiskLimit)
        }
        AttackCaseId::AuditSampleExceedsPopulation => {
            let sample = physical::AuditSampleV1 {
                population_checkpoint_digest: digest(1),
                ballot_manifest_digest: digest(2),
                audit_policy_digest: digest(3),
                sample_selection_evidence_digest: digest(4),
                sampled_positions_digest: digest(5),
                population_size: 100,
                sample_size: 101,
            };
            assert_eq!(
                physical::validate_audit_sample(&sample),
                Err(physical::AuditSampleViolation::SampleExceedsPopulation)
            );
            observation(
                ExpectedDisposition::FailClosed,
                FindingId::AuditSampleExceedsPopulation,
            )
        }
        AttackCaseId::MissingAdjudicationEvidence => {
            let item = physical::AuditObservationRefV1 {
                sample_ordinal: 0,
                anonymized_physical_unit_digest: digest(1),
                cvr_digest: Some(digest(2)),
                observation_class: physical::AuditObservationClass::GovernedAdjudicationRequired,
                observation_evidence_digest: digest(3),
                adjudication_evidence_digest: None,
            };
            assert_eq!(
                physical::validate_audit_observation(&item),
                Err(physical::AuditObservationViolation::MissingAdjudicationEvidence)
            );
            observation(
                ExpectedDisposition::FailClosed,
                FindingId::MissingAdjudicationEvidence,
            )
        }
        AttackCaseId::PhysicalFailureDominatesVerifier => {
            let mut run = valid_verifier_run();
            let physical_stage = run
                .stages
                .iter_mut()
                .find(|stage| stage.stage == verifier::VerificationStageId::PhysicalAudit)
                .expect("required PhysicalAudit stage must exist in positive control");
            physical_stage.disposition = verifier::VerificationStageDisposition::Fail;
            assert_eq!(
                verifier::classify_verifier_run(&run),
                Ok(verifier::VerifierRunDisposition::AtLeastOneStageFailed)
            );
            observation(
                ExpectedDisposition::AggregateFailure,
                FindingId::PhysicalAuditFailureDominates,
            )
        }
    }
}

pub fn expected_observation(id: AttackCaseId) -> CaseObservationV1 {
    let spec = REQUIRED_EXECUTABLE_CASES
        .iter()
        .find(|case| case.id == id)
        .expect("every executable attack ID must have a registry entry");
    observation(spec.expected_disposition, spec.expected_finding)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn registry_is_complete_and_fail_visible() {
        assert_eq!(validate_corpus_registry(), Ok(()));
        assert_eq!(REQUIRED_EXECUTABLE_CASES.len(), 26);
        assert_eq!(PROTOCOL_DEPENDENT_GAPS.len(), 7);
    }

    #[test]
    fn every_registered_attack_produces_exact_expected_disposition() {
        for case in REQUIRED_EXECUTABLE_CASES {
            assert_eq!(
                execute_case(case.id),
                expected_observation(case.id),
                "case {:?}",
                case.id
            );
        }
    }

    #[test]
    fn positive_controls_still_pass() {
        let census = authority::NullifierCensusV1 {
            scope: authorization_scope(3),
            complete_checkpoint_digest: digest(20),
            observed_claim_count: 100,
            unique_nullifier_count: 100,
            conflicting_nullifier_count: 0,
        };
        assert_eq!(
            authority::validate_nullifier_census_for_tally(&census),
            Ok(())
        );

        let previous = checkpoint(0, 10, None);
        let next = checkpoint(1, 11, Some(digest(50)));
        assert_eq!(
            transparency::validate_checkpoint_successor(&previous, digest(50), &next),
            Ok(())
        );

        let attestations = vec![
            witness_attestation(1, 8),
            witness_attestation(2, 9),
            witness_attestation(3, 10),
        ];
        assert_eq!(
            transparency::validate_witness_quorum(digest(42), &witness_policy(), &attestations),
            Ok(())
        );

        assert_eq!(
            verifier::validate_evidence_package_manifest(&valid_manifest()),
            Ok(())
        );
        assert_eq!(
            verifier::validate_offline_execution_policy(
                &verifier::OfflineVerifierExecutionPolicyV1::default()
            ),
            Ok(())
        );
        assert_eq!(
            verifier::classify_verifier_run(&valid_verifier_run()),
            Ok(verifier::VerifierRunDisposition::AllRequiredStagesPass)
        );
        assert_eq!(
            physical::validate_ballot_stock_accounting(&valid_accounting()),
            Ok(())
        );
    }

    #[test]
    fn protocol_dependent_gaps_cannot_be_mistaken_for_executed_evidence() {
        let gap_ids: BTreeSet<_> = PROTOCOL_DEPENDENT_GAPS.iter().map(|gap| gap.id).collect();
        assert!(gap_ids.contains(&ProtocolGapId::ReceiptFreeness));
        assert!(gap_ids.contains(&ProtocolGapId::CastAsIntended));
        assert!(gap_ids.contains(&ProtocolGapId::TalliedAsRecorded));
        assert!(
            PROTOCOL_DEPENDENT_GAPS
                .iter()
                .all(|gap| gap.blocks_protocol_security_claim)
        );
    }
}
