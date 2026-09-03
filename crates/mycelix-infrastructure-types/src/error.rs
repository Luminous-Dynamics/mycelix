use core::fmt;

/// Validation failures for infrastructure wire types.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum InfrastructureErrorV1 {
    InvalidResourceKey,
    EmptyResourceVector,
    ZeroResourceAmount,
    DuplicateResourceKey,
    TooManyResourceDimensions,
    InvalidTimeWindow,
    WindowOutsideParent,
    ResourceExceedsParent,
    PartyMismatch,
    ParentIdMismatch,
    ReceiptBeforeServiceEnd,
    InvalidSchemaTag,
    UnsupportedSchemaVersion,
    IdMismatch,
}

impl fmt::Display for InfrastructureErrorV1 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let msg = match self {
            Self::InvalidResourceKey => "resource key is not a valid canonical token",
            Self::EmptyResourceVector => "resource vector must contain at least one dimension",
            Self::ZeroResourceAmount => "resource amounts must be non-zero",
            Self::DuplicateResourceKey => "resource vector contains a duplicate dimension",
            Self::TooManyResourceDimensions => "resource vector exceeds the v1 dimension limit",
            Self::InvalidTimeWindow => "time window must have start < end",
            Self::WindowOutsideParent => "child time window is outside its parent window",
            Self::ResourceExceedsParent => "child resource vector exceeds its parent allocation",
            Self::PartyMismatch => "envelope parties do not match the referenced parent",
            Self::ParentIdMismatch => "envelope does not reference the supplied parent",
            Self::ReceiptBeforeServiceEnd => "receipt observation precedes the end of its service window",
            Self::InvalidSchemaTag => "payload schema tag is not a valid canonical token",
            Self::UnsupportedSchemaVersion => "unsupported infrastructure schema version",
            Self::IdMismatch => "stored identifier does not match the canonical envelope fields",
        };
        f.write_str(msg)
    }
}

impl std::error::Error for InfrastructureErrorV1 {}
