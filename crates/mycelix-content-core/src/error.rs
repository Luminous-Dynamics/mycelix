use core::fmt;

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ContentErrorV1 {
    UnsupportedSchemaVersion,
    InvalidMediaType,
    EmptyManifest,
    TooManyBlobs,
    TotalSizeOverflow,
    TotalSizeMismatch,
    IdMismatch,
    InvalidLogicalName,
    InvalidVersionLabel,
    InvalidJurisdiction,
    TooManyJurisdictions,
    JurisdictionConflict,
    ZeroReplicas,
    ZeroFailureDomainMinimum,
    FailureDomainExceedsReplicas,
    DuplicateFailureDomainRequirement,
    ZeroRetentionDuration,
    InvalidLatencyTarget,
    PrivateBackupRequiresClientEncryption,
}

impl fmt::Display for ContentErrorV1 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let msg = match self {
            Self::UnsupportedSchemaVersion => "unsupported content schema version",
            Self::InvalidMediaType => "invalid media type",
            Self::EmptyManifest => "object manifest must contain at least one blob",
            Self::TooManyBlobs => "object manifest exceeds the v1 blob-count limit",
            Self::TotalSizeOverflow => "object manifest total size overflows u64",
            Self::TotalSizeMismatch => "stored object total does not match blob sizes",
            Self::IdMismatch => "stored identifier does not match canonical recomputation",
            Self::InvalidLogicalName => "invalid publication logical name",
            Self::InvalidVersionLabel => "invalid publication version label",
            Self::InvalidJurisdiction => "invalid jurisdiction token",
            Self::TooManyJurisdictions => "jurisdiction set exceeds the v1 limit",
            Self::JurisdictionConflict => "jurisdiction appears in both allowed and forbidden sets",
            Self::ZeroReplicas => "minimum replicas must be at least one",
            Self::ZeroFailureDomainMinimum => "failure-domain minimum must be at least one",
            Self::FailureDomainExceedsReplicas => "failure-domain minimum exceeds minimum replicas",
            Self::DuplicateFailureDomainRequirement => "duplicate failure-domain requirement",
            Self::ZeroRetentionDuration => "retention duration must be non-zero",
            Self::InvalidLatencyTarget => "latency target must be non-zero",
            Self::PrivateBackupRequiresClientEncryption => {
                "private backup requires client-side encryption"
            }
        };
        f.write_str(msg)
    }
}

impl std::error::Error for ContentErrorV1 {}
