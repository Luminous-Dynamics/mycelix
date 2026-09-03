use std::{io, path::PathBuf};

use mycelix_content_core::{ContentDigestV1, ContentErrorV1};
use mycelix_content_node::CasErrorV1;
use thiserror::Error;

#[derive(Debug, Error, Clone, PartialEq, Eq)]
pub enum WireErrorV1 {
    #[error("invalid wire frame length")]
    InvalidLength,
    #[error("invalid wire magic")]
    InvalidMagic,
    #[error("unsupported wire version")]
    UnsupportedVersion,
    #[error("unsupported operation")]
    UnsupportedOperation,
    #[error("unsupported digest algorithm")]
    UnsupportedAlgorithm,
    #[error("reserved or flags bytes must be zero")]
    NonZeroReserved,
    #[error("unsupported response status")]
    UnsupportedStatus,
}

#[derive(Debug, Error)]
pub enum TransportErrorV1 {
    #[error(transparent)]
    Wire(#[from] WireErrorV1),
    #[error(transparent)]
    Content(#[from] ContentErrorV1),
    #[error(transparent)]
    Cas(#[from] CasErrorV1),
    #[error("I/O error: {0}")]
    Io(#[from] io::Error),
    #[error("Iroh connect failed: {0}")]
    Connect(String),
    #[error("Iroh stream failed: {0}")]
    Stream(String),
    #[error("configured maximum blob size must be non-zero")]
    ZeroMaxBlobSize,
    #[error("invalid temporary download directory: {0}")]
    InvalidTempDirectory(PathBuf),
    #[error("invalid transfer concurrency {value}; allowed range is 1..={maximum}")]
    InvalidConcurrency { value: usize, maximum: usize },
    #[error("blob size {size_bytes} exceeds configured transport maximum {max_bytes}")]
    BlobTooLarge { size_bytes: u64, max_bytes: u64 },
    #[error("remote provider does not have requested blob {0:?}")]
    RemoteNotFound(ContentDigestV1),
    #[error("remote provider is busy")]
    RemoteBusy,
    #[error("remote provider reported an integrity failure")]
    RemoteIntegrityFailure,
    #[error("remote provider rejected the protocol request")]
    RemoteProtocolError,
    #[error("remote provider reported an internal error")]
    RemoteInternalError,
    #[error("remote size mismatch: expected {expected}, got {actual}")]
    SizeMismatch { expected: u64, actual: u64 },
    #[error("remote digest mismatch: expected {expected:?}, got {actual:?}")]
    DigestMismatch {
        expected: ContentDigestV1,
        actual: ContentDigestV1,
    },
    #[error("remote sent bytes after the declared payload")]
    TrailingData,
}
