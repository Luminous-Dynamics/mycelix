use mycelix_infrastructure_types::StableIdV1;

use crate::{ReaderEnrollmentErrorV1, ReaderEnrollmentRegistryV1, ReaderEnrollmentV1};

/// Reconstruct a persisted/wire registry snapshot and verify its claimed stable ID.
///
/// The complete enrollment set is canonicalized through
/// [`ReaderEnrollmentRegistryV1::from_enrollments`] before the registry
/// commitment is compared. Callers therefore cannot attach an arbitrary
/// snapshot ID to otherwise valid enrollment records.
pub fn reconstruct_reader_enrollment_registry_v1(
    enrollments: impl IntoIterator<Item = ReaderEnrollmentV1>,
    claimed_id: StableIdV1,
) -> Result<ReaderEnrollmentRegistryV1, ReaderEnrollmentErrorV1> {
    let registry = ReaderEnrollmentRegistryV1::from_enrollments(enrollments)?;
    if registry.id() != claimed_id {
        return Err(ReaderEnrollmentErrorV1::RegistryCommitmentMismatch);
    }
    Ok(registry)
}
