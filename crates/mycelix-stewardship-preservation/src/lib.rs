// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Evidence-oriented preservation primitives for exact representations.

#![forbid(unsafe_code)]

use core::fmt;
use mycelix_stewardship_core::{
    CanonicalIdErrorV1, CanonicalIdV1, ContentDigestV1, StewardedSubjectIdentityV1,
};

pub const PRESERVATION_PROFILE_V1: &str = "mycelix/preservation/v1";
pub const MAX_PRESERVATION_EVIDENCE_REFS_V1: usize = 32;
pub const MAX_MANIFEST_REFS_PER_KIND_V1: usize = 128;

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct PreservationEvidenceRefV1(CanonicalIdV1);

impl PreservationEvidenceRefV1 {
    pub fn new(value: impl Into<String>) -> Result<Self, CanonicalIdErrorV1> {
        CanonicalIdV1::new(value).map(Self)
    }

    pub fn as_str(&self) -> &str {
        self.0.as_str()
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FixityDispositionV1 {
    Match,
    Mismatch,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RecoveryOutcomeV1 {
    ReportedSuccess,
    ReportedFailure,
    Indeterminate,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PreservationRecordErrorV1 {
    NoEvidenceReferences,
    TooManyEvidenceReferences,
    DuplicateEvidenceReference,
    ExactMigrationSelfEdge,
}

impl fmt::Display for PreservationRecordErrorV1 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NoEvidenceReferences => {
                f.write_str("preservation record requires evidence references")
            }
            Self::TooManyEvidenceReferences => {
                f.write_str("too many preservation evidence references for v1")
            }
            Self::DuplicateEvidenceReference => {
                f.write_str("duplicate preservation evidence reference")
            }
            Self::ExactMigrationSelfEdge => {
                f.write_str("migration requires distinct exact representations")
            }
        }
    }
}

fn validate_evidence(
    evidence_refs: &[PreservationEvidenceRefV1],
) -> Result<(), PreservationRecordErrorV1> {
    if evidence_refs.is_empty() {
        return Err(PreservationRecordErrorV1::NoEvidenceReferences);
    }
    if evidence_refs.len() > MAX_PRESERVATION_EVIDENCE_REFS_V1 {
        return Err(PreservationRecordErrorV1::TooManyEvidenceReferences);
    }
    for (index, reference) in evidence_refs.iter().enumerate() {
        if evidence_refs[..index].contains(reference) {
            return Err(PreservationRecordErrorV1::DuplicateEvidenceReference);
        }
    }
    Ok(())
}

/// One observed digest compared with the exact target commitment.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FixityObservationV1 {
    observation_id: CanonicalIdV1,
    target: StewardedSubjectIdentityV1,
    observed_digest: ContentDigestV1,
    observer_ref: CanonicalIdV1,
    evidence_refs: Vec<PreservationEvidenceRefV1>,
}

impl FixityObservationV1 {
    pub fn new(
        observation_id: CanonicalIdV1,
        target: StewardedSubjectIdentityV1,
        observed_digest: ContentDigestV1,
        observer_ref: CanonicalIdV1,
        evidence_refs: Vec<PreservationEvidenceRefV1>,
    ) -> Result<Self, PreservationRecordErrorV1> {
        validate_evidence(&evidence_refs)?;
        Ok(Self {
            observation_id,
            target,
            observed_digest,
            observer_ref,
            evidence_refs,
        })
    }

    pub fn observation_id(&self) -> &CanonicalIdV1 {
        &self.observation_id
    }

    pub fn disposition(&self) -> FixityDispositionV1 {
        if self.observed_digest == self.target.content_digest {
            FixityDispositionV1::Match
        } else {
            FixityDispositionV1::Mismatch
        }
    }

    pub fn target(&self) -> &StewardedSubjectIdentityV1 {
        &self.target
    }

    pub const fn observed_digest(&self) -> &ContentDigestV1 {
        &self.observed_digest
    }

    pub fn observer_ref(&self) -> &CanonicalIdV1 {
        &self.observer_ref
    }

    pub fn evidence_refs(&self) -> &[PreservationEvidenceRefV1] {
        &self.evidence_refs
    }
}

/// Assertion that an exact representation is held in a storage domain.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ReplicaAttestationV1 {
    attestation_id: CanonicalIdV1,
    target: StewardedSubjectIdentityV1,
    storage_domain_ref: CanonicalIdV1,
    custodian_ref: CanonicalIdV1,
    evidence_refs: Vec<PreservationEvidenceRefV1>,
}

impl ReplicaAttestationV1 {
    pub fn new(
        attestation_id: CanonicalIdV1,
        target: StewardedSubjectIdentityV1,
        storage_domain_ref: CanonicalIdV1,
        custodian_ref: CanonicalIdV1,
        evidence_refs: Vec<PreservationEvidenceRefV1>,
    ) -> Result<Self, PreservationRecordErrorV1> {
        validate_evidence(&evidence_refs)?;
        Ok(Self {
            attestation_id,
            target,
            storage_domain_ref,
            custodian_ref,
            evidence_refs,
        })
    }

    pub fn attestation_id(&self) -> &CanonicalIdV1 {
        &self.attestation_id
    }

    pub fn target(&self) -> &StewardedSubjectIdentityV1 {
        &self.target
    }

    pub fn storage_domain_ref(&self) -> &CanonicalIdV1 {
        &self.storage_domain_ref
    }

    pub fn custodian_ref(&self) -> &CanonicalIdV1 {
        &self.custodian_ref
    }

    pub fn evidence_refs(&self) -> &[PreservationEvidenceRefV1] {
        &self.evidence_refs
    }
}

/// Exact original + migrated representation, preserving both identities.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MigrationRecordV1 {
    migration_id: CanonicalIdV1,
    original: StewardedSubjectIdentityV1,
    migrated: StewardedSubjectIdentityV1,
    process_ref: CanonicalIdV1,
    evidence_refs: Vec<PreservationEvidenceRefV1>,
}

impl MigrationRecordV1 {
    pub fn new(
        migration_id: CanonicalIdV1,
        original: StewardedSubjectIdentityV1,
        migrated: StewardedSubjectIdentityV1,
        process_ref: CanonicalIdV1,
        evidence_refs: Vec<PreservationEvidenceRefV1>,
    ) -> Result<Self, PreservationRecordErrorV1> {
        if original == migrated {
            return Err(PreservationRecordErrorV1::ExactMigrationSelfEdge);
        }
        validate_evidence(&evidence_refs)?;
        Ok(Self {
            migration_id,
            original,
            migrated,
            process_ref,
            evidence_refs,
        })
    }

    pub fn migration_id(&self) -> &CanonicalIdV1 {
        &self.migration_id
    }

    pub fn original(&self) -> &StewardedSubjectIdentityV1 {
        &self.original
    }

    pub fn migrated(&self) -> &StewardedSubjectIdentityV1 {
        &self.migrated
    }

    pub fn process_ref(&self) -> &CanonicalIdV1 {
        &self.process_ref
    }

    pub fn evidence_refs(&self) -> &[PreservationEvidenceRefV1] {
        &self.evidence_refs
    }
}

/// Evidence-bearing reported recovery-test outcome.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RecoveryTestV1 {
    test_id: CanonicalIdV1,
    target: StewardedSubjectIdentityV1,
    recovery_profile_ref: CanonicalIdV1,
    outcome: RecoveryOutcomeV1,
    evidence_refs: Vec<PreservationEvidenceRefV1>,
}

impl RecoveryTestV1 {
    pub fn new(
        test_id: CanonicalIdV1,
        target: StewardedSubjectIdentityV1,
        recovery_profile_ref: CanonicalIdV1,
        outcome: RecoveryOutcomeV1,
        evidence_refs: Vec<PreservationEvidenceRefV1>,
    ) -> Result<Self, PreservationRecordErrorV1> {
        validate_evidence(&evidence_refs)?;
        Ok(Self {
            test_id,
            target,
            recovery_profile_ref,
            outcome,
            evidence_refs,
        })
    }

    pub fn test_id(&self) -> &CanonicalIdV1 {
        &self.test_id
    }

    pub const fn outcome(&self) -> RecoveryOutcomeV1 {
        self.outcome
    }

    pub fn target(&self) -> &StewardedSubjectIdentityV1 {
        &self.target
    }

    pub fn recovery_profile_ref(&self) -> &CanonicalIdV1 {
        &self.recovery_profile_ref
    }

    pub fn evidence_refs(&self) -> &[PreservationEvidenceRefV1] {
        &self.evidence_refs
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PreservationManifestErrorV1 {
    TooManyReferences,
    DuplicateReference,
}

impl fmt::Display for PreservationManifestErrorV1 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::TooManyReferences => {
                f.write_str("too many preservation manifest references for v1")
            }
            Self::DuplicateReference => {
                f.write_str("duplicate preservation manifest reference")
            }
        }
    }
}

fn validate_manifest_refs(refs: &[CanonicalIdV1]) -> Result<(), PreservationManifestErrorV1> {
    if refs.len() > MAX_MANIFEST_REFS_PER_KIND_V1 {
        return Err(PreservationManifestErrorV1::TooManyReferences);
    }
    for (index, reference) in refs.iter().enumerate() {
        if refs[..index].contains(reference) {
            return Err(PreservationManifestErrorV1::DuplicateReference);
        }
    }
    Ok(())
}

/// Aggregates preservation evidence references without producing an overall
/// preservation-success boolean.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PreservationManifestV1 {
    manifest_id: CanonicalIdV1,
    target: StewardedSubjectIdentityV1,
    format_ref: CanonicalIdV1,
    fixity_observation_refs: Vec<CanonicalIdV1>,
    replica_attestation_refs: Vec<CanonicalIdV1>,
    migration_record_refs: Vec<CanonicalIdV1>,
    recovery_test_refs: Vec<CanonicalIdV1>,
}

impl PreservationManifestV1 {
    pub fn new(
        manifest_id: CanonicalIdV1,
        target: StewardedSubjectIdentityV1,
        format_ref: CanonicalIdV1,
        fixity_observation_refs: Vec<CanonicalIdV1>,
        replica_attestation_refs: Vec<CanonicalIdV1>,
        migration_record_refs: Vec<CanonicalIdV1>,
        recovery_test_refs: Vec<CanonicalIdV1>,
    ) -> Result<Self, PreservationManifestErrorV1> {
        for refs in [
            fixity_observation_refs.as_slice(),
            replica_attestation_refs.as_slice(),
            migration_record_refs.as_slice(),
            recovery_test_refs.as_slice(),
        ] {
            validate_manifest_refs(refs)?;
        }

        Ok(Self {
            manifest_id,
            target,
            format_ref,
            fixity_observation_refs,
            replica_attestation_refs,
            migration_record_refs,
            recovery_test_refs,
        })
    }

    pub fn manifest_id(&self) -> &CanonicalIdV1 {
        &self.manifest_id
    }

    pub fn target(&self) -> &StewardedSubjectIdentityV1 {
        &self.target
    }

    pub fn format_ref(&self) -> &CanonicalIdV1 {
        &self.format_ref
    }

    pub fn fixity_observation_refs(&self) -> &[CanonicalIdV1] {
        &self.fixity_observation_refs
    }

    pub fn replica_attestation_refs(&self) -> &[CanonicalIdV1] {
        &self.replica_attestation_refs
    }

    pub fn migration_record_refs(&self) -> &[CanonicalIdV1] {
        &self.migration_record_refs
    }

    pub fn recovery_test_refs(&self) -> &[CanonicalIdV1] {
        &self.recovery_test_refs
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_stewardship_core::{
        DigestAlgorithmV1, RepresentationIdV1, RepresentationKindV1, RevisionIdV1,
        StewardedSubjectIdV1,
    };

    fn identity(rep: &str, byte: u8) -> StewardedSubjectIdentityV1 {
        StewardedSubjectIdentityV1 {
            subject: StewardedSubjectIdV1::new("subject:archive:example").unwrap(),
            revision: RevisionIdV1::new("revision:1").unwrap(),
            representation: RepresentationIdV1::new(rep).unwrap(),
            kind: RepresentationKindV1::Other,
            content_digest: ContentDigestV1::new(DigestAlgorithmV1::Blake3_256, [byte; 32]),
        }
    }

    fn ev(id: &str) -> PreservationEvidenceRefV1 {
        PreservationEvidenceRefV1::new(id).unwrap()
    }

    #[test]
    fn fixity_match_and_mismatch_are_structural_digest_comparisons() {
        let target = identity("representation:original", 9);
        let matching = FixityObservationV1::new(
            CanonicalIdV1::new("fixity:1").unwrap(),
            target.clone(),
            target.content_digest,
            CanonicalIdV1::new("observer:1").unwrap(),
            vec![ev("evidence:fixity:1")],
        )
        .unwrap();
        assert_eq!(matching.disposition(), FixityDispositionV1::Match);

        let mismatch = FixityObservationV1::new(
            CanonicalIdV1::new("fixity:2").unwrap(),
            target,
            ContentDigestV1::new(DigestAlgorithmV1::Blake3_256, [8; 32]),
            CanonicalIdV1::new("observer:1").unwrap(),
            vec![ev("evidence:fixity:2")],
        )
        .unwrap();
        assert_eq!(mismatch.disposition(), FixityDispositionV1::Mismatch);
    }

    #[test]
    fn evidence_is_required_for_preservation_records() {
        let result = ReplicaAttestationV1::new(
            CanonicalIdV1::new("replica:1").unwrap(),
            identity("representation:1", 1),
            CanonicalIdV1::new("storage-domain:1").unwrap(),
            CanonicalIdV1::new("custodian:1").unwrap(),
            vec![],
        );
        assert_eq!(result, Err(PreservationRecordErrorV1::NoEvidenceReferences));
    }

    #[test]
    fn migration_retains_original_and_migrated_exact_identities() {
        let original = identity("representation:legacy", 1);
        let migrated = identity("representation:open-format", 2);
        let record = MigrationRecordV1::new(
            CanonicalIdV1::new("migration:1").unwrap(),
            original.clone(),
            migrated.clone(),
            CanonicalIdV1::new("process:converter-build:1").unwrap(),
            vec![ev("evidence:migration:1")],
        )
        .unwrap();
        assert_eq!(record.original(), &original);
        assert_eq!(record.migrated(), &migrated);
    }

    #[test]
    fn exact_migration_self_edge_is_rejected() {
        let exact = identity("representation:1", 1);
        let result = MigrationRecordV1::new(
            CanonicalIdV1::new("migration:2").unwrap(),
            exact.clone(),
            exact,
            CanonicalIdV1::new("process:1").unwrap(),
            vec![ev("evidence:1")],
        );
        assert_eq!(result, Err(PreservationRecordErrorV1::ExactMigrationSelfEdge));
    }

    #[test]
    fn recovery_success_is_explicitly_reported_not_promoted_to_guarantee() {
        let test = RecoveryTestV1::new(
            CanonicalIdV1::new("recovery:1").unwrap(),
            identity("representation:1", 1),
            CanonicalIdV1::new("recovery-profile:1").unwrap(),
            RecoveryOutcomeV1::ReportedSuccess,
            vec![ev("evidence:recovery-log:1")],
        )
        .unwrap();
        assert_eq!(test.outcome(), RecoveryOutcomeV1::ReportedSuccess);
    }

    #[test]
    fn empty_manifest_categories_are_valid_and_do_not_imply_success() {
        let manifest = PreservationManifestV1::new(
            CanonicalIdV1::new("manifest:1").unwrap(),
            identity("representation:1", 1),
            CanonicalIdV1::new("format:application-octet-stream").unwrap(),
            vec![],
            vec![],
            vec![],
            vec![],
        )
        .unwrap();
        assert!(manifest.fixity_observation_refs().is_empty());
        assert!(manifest.replica_attestation_refs().is_empty());
        assert!(manifest.migration_record_refs().is_empty());
        assert!(manifest.recovery_test_refs().is_empty());
    }

    #[test]
    fn duplicate_manifest_refs_are_rejected_within_kind() {
        let duplicate = CanonicalIdV1::new("fixity:1").unwrap();
        let result = PreservationManifestV1::new(
            CanonicalIdV1::new("manifest:2").unwrap(),
            identity("representation:1", 1),
            CanonicalIdV1::new("format:binary").unwrap(),
            vec![duplicate.clone(), duplicate],
            vec![],
            vec![],
            vec![],
        );
        assert_eq!(result, Err(PreservationManifestErrorV1::DuplicateReference));
    }

    #[test]
    fn record_identifiers_remain_observable_without_becoming_authority() {
        let target = identity("representation:1", 1);
        let fixity = FixityObservationV1::new(
            CanonicalIdV1::new("fixity:observable").unwrap(),
            target.clone(),
            target.content_digest,
            CanonicalIdV1::new("observer:1").unwrap(),
            vec![ev("evidence:fixity:observable")],
        )
        .unwrap();
        assert_eq!(fixity.observation_id().as_str(), "fixity:observable");

        let replica = ReplicaAttestationV1::new(
            CanonicalIdV1::new("replica:observable").unwrap(),
            target.clone(),
            CanonicalIdV1::new("storage-domain:1").unwrap(),
            CanonicalIdV1::new("custodian:1").unwrap(),
            vec![ev("evidence:replica:observable")],
        )
        .unwrap();
        assert_eq!(replica.attestation_id().as_str(), "replica:observable");

        let recovery = RecoveryTestV1::new(
            CanonicalIdV1::new("recovery:observable").unwrap(),
            target,
            CanonicalIdV1::new("recovery-profile:1").unwrap(),
            RecoveryOutcomeV1::Indeterminate,
            vec![ev("evidence:recovery:observable")],
        )
        .unwrap();
        assert_eq!(recovery.test_id().as_str(), "recovery:observable");
    }

    #[test]
    fn profile_identifier_is_frozen() {
        assert_eq!(PRESERVATION_PROFILE_V1, "mycelix/preservation/v1");
    }
}
