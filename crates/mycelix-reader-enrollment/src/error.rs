use mycelix_infrastructure_types::{InfrastructureErrorV1, StableIdV1};
use thiserror::Error;

/// Validation failures for the CF-07C1 reader-enrollment registry.
#[derive(Debug, Error)]
pub enum ReaderEnrollmentErrorV1 {
    /// The supplied ML-DSA-65 public key did not have the exact FIPS 204 length
    /// used by Xenia's current hybrid peer identity.
    #[error("ML-DSA-65 public key length {actual} does not equal required length {expected}")]
    InvalidMlDsa65PublicKeyLength { actual: usize, expected: usize },
    /// Application principals must be explicit non-zero identities.
    #[error("reader enrollment principal must be non-zero")]
    ZeroPrincipal,
    /// Group identifiers are authority-bearing and may not use the zero sentinel.
    #[error("reader enrollment group must be non-zero")]
    ZeroGroup,
    /// A claimed credential commitment did not match the exact hybrid key pair.
    #[error("reader credential commitment does not match the supplied hybrid key pair")]
    CredentialCommitmentMismatch,
    /// A claimed enrollment commitment did not match credential, principal, and groups.
    #[error("reader enrollment commitment does not match its canonical fields")]
    EnrollmentCommitmentMismatch,
    /// A claimed registry commitment did not match the complete canonical enrollment snapshot.
    #[error("reader enrollment registry commitment does not match its canonical snapshot")]
    RegistryCommitmentMismatch,
    /// Two records attempted to enroll the same exact hybrid credential.
    #[error("duplicate reader credential enrollment {0:?}")]
    DuplicateCredential(StableIdV1),
    /// Stable-ID derivation or schema validation failed.
    #[error(transparent)]
    Infrastructure(#[from] InfrastructureErrorV1),
}
