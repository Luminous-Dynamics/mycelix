use std::{io, path::PathBuf};

use mycelix_content_core::{ContentDigestV1, ContentErrorV1};
use thiserror::Error;

#[derive(Debug, Error)]
pub enum CasErrorV1 {
    #[error(transparent)]
    InvalidDescriptor(#[from] ContentErrorV1),
    #[error("I/O error at {path}: {source}")]
    Io {
        path: PathBuf,
        #[source]
        source: io::Error,
    },
    #[error("CAS root is already locked by another process: {0}")]
    AlreadyLocked(PathBuf),
    #[error("CAS path is not a real directory: {0}")]
    InvalidDirectory(PathBuf),
    #[error("unexpected CAS entry: {0}")]
    UnexpectedEntry(PathBuf),
    #[error("invalid immutable blob filename: {0}")]
    InvalidBlobFilename(PathBuf),
    #[error("immutable blob file is writable: {0}")]
    MutableBlobFile(PathBuf),
    #[error("blob not found: {0:?}")]
    NotFound(ContentDigestV1),
    #[error("blob size mismatch: expected {expected} bytes, observed {actual}")]
    SizeMismatch { expected: u64, actual: u64 },
    #[error("blob digest mismatch: expected {expected:?}, observed {actual:?}")]
    DigestMismatch {
        expected: ContentDigestV1,
        actual: ContentDigestV1,
    },
    #[error(
        "CAS quota exceeded: quota={quota_bytes}, used={used_bytes}, reserved={reserved_bytes}, requested={requested_bytes}"
    )]
    QuotaExceeded {
        quota_bytes: u64,
        used_bytes: u64,
        reserved_bytes: u64,
        requested_bytes: u64,
    },
    #[error("CAS usage accounting overflow")]
    UsageOverflow,
}

impl CasErrorV1 {
    pub(crate) fn io(path: impl Into<PathBuf>, source: io::Error) -> Self {
        Self::Io {
            path: path.into(),
            source,
        }
    }
}
