use mycelix_infrastructure_types::StableIdV1;
use mycelix_nix_exposure::ExposureErrorV1;
use mycelix_reader_enrollment::ReaderEnrollmentErrorV1;
use thiserror::Error;

/// Fail-closed identity-binding failures for CF-07C2.
#[derive(Debug, Error)]
pub enum XeniaReaderBridgeErrorV1 {
    /// The Xenia application channel was not the Content Fabric reader domain.
    #[error(
        "opened Xenia application payload type 0x{actual:02x} does not equal Content Fabric reader type 0x{expected:02x}"
    )]
    WrongPayloadType {
        /// Required Content Fabric application payload type.
        expected: u8,
        /// Payload type that was cryptographically opened.
        actual: u8,
    },
    /// The supplied enrollment registry was not the exact snapshot pinned by
    /// the caller's higher-level policy/configuration.
    #[error("reader enrollment registry commitment does not match pinned snapshot")]
    RegistryCommitmentMismatch {
        /// Stable registry commitment expected by policy.
        expected: StableIdV1,
        /// Stable commitment of the supplied immutable registry snapshot.
        actual: StableIdV1,
    },
    /// The exact hybrid Xenia peer credential has no enrollment in the pinned
    /// registry snapshot.
    #[error("authenticated Xenia hybrid peer credential is not enrolled")]
    PeerNotEnrolled,
    /// CF-07C1 credential/enrollment validation failed.
    #[error(transparent)]
    Enrollment(#[from] ReaderEnrollmentErrorV1),
    /// CF-07A reader construction rejected the enrolled principal/group facts.
    #[error(transparent)]
    Exposure(#[from] ExposureErrorV1),
}
