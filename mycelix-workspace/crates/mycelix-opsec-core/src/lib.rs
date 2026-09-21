#![forbid(unsafe_code)]
//! Shared, dependency-light operational-security identities and vocabularies.
//!
//! This crate intentionally does **not** implement confidentiality classification,
//! policy evaluation, declassification, secret storage, logging, networking,
//! execution authority, EPI admission, Xenia signing, or Nixward enforcement.
//!
//! Its strongest theorem is:
//!
//! ```text
//! role-safe bounded references
//! + closed sink/surface/disposition vocabulary
//! != policy correctness
//! != disclosure authority
//! != dispatch authority
//! ```
//!
//! Secret bytes do not belong in this crate. `SecretClassRef` is only an opaque
//! reference to a separately owned secret-class/custody scheme.

use core::fmt;

/// Maximum UTF-8 byte length for a v0.1 role-specific reference.
///
/// V0.1 uses graphic ASCII so byte length is also character count. A future
/// internationalized profile must be separately versioned rather than silently
/// widening this parser.
pub const MAX_OPSEC_REF_BYTES_V1: usize = 256;

/// The strongest authority claim exposed by this crate.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Hash)]
pub enum OpsecAuthorityScopeV1 {
    /// Types and closed vocabularies only. No policy or execution authority.
    IdentityAndVocabularyOnly,
}

impl OpsecAuthorityScopeV1 {
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::IdentityAndVocabularyOnly => "identity-and-vocabulary-only",
        }
    }
}

/// Validation failures for bounded role-specific references.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Hash)]
pub enum OpsecRefError {
    Empty,
    TooLong { bytes: usize, max: usize },
    NonGraphicAscii,
}

impl fmt::Display for OpsecRefError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Empty => f.write_str("OPSEC reference must not be empty"),
            Self::TooLong { bytes, max } => {
                write!(f, "OPSEC reference is {bytes} bytes; maximum is {max}")
            }
            Self::NonGraphicAscii => {
                f.write_str("OPSEC v0.1 reference must contain only graphic ASCII")
            }
        }
    }
}

impl std::error::Error for OpsecRefError {}

fn validate_ref(value: &str) -> Result<(), OpsecRefError> {
    if value.is_empty() {
        return Err(OpsecRefError::Empty);
    }
    if value.len() > MAX_OPSEC_REF_BYTES_V1 {
        return Err(OpsecRefError::TooLong {
            bytes: value.len(),
            max: MAX_OPSEC_REF_BYTES_V1,
        });
    }
    if !value.bytes().all(|byte| byte.is_ascii_graphic()) {
        return Err(OpsecRefError::NonGraphicAscii);
    }
    Ok(())
}

macro_rules! role_ref {
    ($name:ident, $role:literal) => {
        #[doc = concat!("Role-specific bounded reference for `", $role, "`.")]
        #[derive(Clone, Eq, PartialEq, Ord, PartialOrd, Hash)]
        pub struct $name(String);

        impl $name {
            pub const ROLE: &'static str = $role;

            pub fn new(value: impl Into<String>) -> Result<Self, OpsecRefError> {
                let value = value.into();
                validate_ref(&value)?;
                Ok(Self(value))
            }

            /// Explicit access to the exact reference bytes.
            ///
            /// Callers should avoid placing this value into ordinary logs.
            pub fn expose_for_binding(&self) -> &str {
                &self.0
            }

            pub fn len_bytes(&self) -> usize {
                self.0.len()
            }

            pub const fn authority_scope(&self) -> OpsecAuthorityScopeV1 {
                OpsecAuthorityScopeV1::IdentityAndVocabularyOnly
            }
        }

        impl fmt::Debug for $name {
            fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
                f.debug_struct(stringify!($name))
                    .field("role", &Self::ROLE)
                    .field("bytes", &self.0.len())
                    .field("value", &"<redacted>")
                    .finish()
            }
        }
    };
}

role_ref!(OpsecSubjectRef, "opsec-subject");
role_ref!(HandlingLabelRef, "handling-label");
role_ref!(DataLineageRef, "data-lineage");
role_ref!(IdentifiabilityRef, "identifiability");
role_ref!(SecretClassRef, "secret-class");
role_ref!(PurposeRef, "purpose");
role_ref!(DestinationRef, "destination");
role_ref!(RetentionProfileRef, "retention-profile");
role_ref!(LoggingProfileRef, "logging-profile");
role_ref!(ReleaseProfileRef, "release-profile");
role_ref!(PrecisionProfileRef, "precision-profile");
role_ref!(MosaicPolicyRef, "mosaic-policy");
role_ref!(SecurityDomainRef, "security-domain");
role_ref!(PolicyProfileRef, "policy-profile");
role_ref!(PolicyEpochRef, "policy-epoch");
role_ref!(TransformRef, "transform");
role_ref!(ContextRef, "context");

/// Exact class of disclosure sink. Permission for one class implies nothing
/// about any other class.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Hash, Ord, PartialOrd)]
pub enum OpsecDisclosureSinkClassV1 {
    LocalPrivateDisplay,
    LocalProtectedStorage,
    LocalOperationalLog,
    TelemetryTraceMetric,
    CrashErrorReport,
    RemoteWebRequest,
    RemoteModelApi,
    AuthenticatedPeer,
    PublicFederatedPublication,
    ToolServiceInvocation,
    FileObjectExport,
    ClipboardConvenienceExport,
    DurableEvidenceStore,
    EvidenceExport,
}

impl OpsecDisclosureSinkClassV1 {
    pub const ALL: [Self; 14] = [
        Self::LocalPrivateDisplay,
        Self::LocalProtectedStorage,
        Self::LocalOperationalLog,
        Self::TelemetryTraceMetric,
        Self::CrashErrorReport,
        Self::RemoteWebRequest,
        Self::RemoteModelApi,
        Self::AuthenticatedPeer,
        Self::PublicFederatedPublication,
        Self::ToolServiceInvocation,
        Self::FileObjectExport,
        Self::ClipboardConvenienceExport,
        Self::DurableEvidenceStore,
        Self::EvidenceExport,
    ];

    pub const fn as_str(self) -> &'static str {
        match self {
            Self::LocalPrivateDisplay => "LocalPrivateDisplay",
            Self::LocalProtectedStorage => "LocalProtectedStorage",
            Self::LocalOperationalLog => "LocalOperationalLog",
            Self::TelemetryTraceMetric => "TelemetryTraceMetric",
            Self::CrashErrorReport => "CrashErrorReport",
            Self::RemoteWebRequest => "RemoteWebRequest",
            Self::RemoteModelApi => "RemoteModelApi",
            Self::AuthenticatedPeer => "AuthenticatedPeer",
            Self::PublicFederatedPublication => "PublicFederatedPublication",
            Self::ToolServiceInvocation => "ToolServiceInvocation",
            Self::FileObjectExport => "FileObjectExport",
            Self::ClipboardConvenienceExport => "ClipboardConvenienceExport",
            Self::DurableEvidenceStore => "DurableEvidenceStore",
            Self::EvidenceExport => "EvidenceExport",
        }
    }

    pub const fn authority_scope(self) -> OpsecAuthorityScopeV1 {
        OpsecAuthorityScopeV1::IdentityAndVocabularyOnly
    }
}

/// Potential disclosure surface. These values identify a release proposition;
/// they do not claim any observer actually saw the data.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Hash, Ord, PartialOrd)]
pub enum OpsecDisclosureSurfaceV1 {
    DnsQueryName,
    TransportEndpoint,
    TlsNameMetadata,
    HttpAuthority,
    HttpRequestTarget,
    HttpHeaders,
    HttpBody,
    RedirectReferral,
    ProxyRelayMetadata,
    LocalLogsTracesMetrics,
    RetainedCaptureEvidence,
}

impl OpsecDisclosureSurfaceV1 {
    pub const ALL: [Self; 11] = [
        Self::DnsQueryName,
        Self::TransportEndpoint,
        Self::TlsNameMetadata,
        Self::HttpAuthority,
        Self::HttpRequestTarget,
        Self::HttpHeaders,
        Self::HttpBody,
        Self::RedirectReferral,
        Self::ProxyRelayMetadata,
        Self::LocalLogsTracesMetrics,
        Self::RetainedCaptureEvidence,
    ];

    pub const fn as_str(self) -> &'static str {
        match self {
            Self::DnsQueryName => "DnsQueryName",
            Self::TransportEndpoint => "TransportEndpoint",
            Self::TlsNameMetadata => "TlsNameMetadata",
            Self::HttpAuthority => "HttpAuthority",
            Self::HttpRequestTarget => "HttpRequestTarget",
            Self::HttpHeaders => "HttpHeaders",
            Self::HttpBody => "HttpBody",
            Self::RedirectReferral => "RedirectReferral",
            Self::ProxyRelayMetadata => "ProxyRelayMetadata",
            Self::LocalLogsTracesMetrics => "LocalLogsTracesMetrics",
            Self::RetainedCaptureEvidence => "RetainedCaptureEvidence",
        }
    }

    pub const fn authority_scope(self) -> OpsecAuthorityScopeV1 {
        OpsecAuthorityScopeV1::IdentityAndVocabularyOnly
    }
}

/// Closed vocabulary for future OPSEC decision *evidence*.
///
/// This crate does not evaluate which disposition applies and does not turn
/// `AllowCandidate` into a permit.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Hash, Ord, PartialOrd)]
pub enum OpsecDecisionEvidenceDispositionV1 {
    Deny,
    AllowCandidate,
    NeedsMinimization,
    NeedsDeclassification,
    NeedsProtectedSink,
    NeedsHumanReview,
    NeedsFreshPolicy,
}

impl OpsecDecisionEvidenceDispositionV1 {
    pub const ALL: [Self; 7] = [
        Self::Deny,
        Self::AllowCandidate,
        Self::NeedsMinimization,
        Self::NeedsDeclassification,
        Self::NeedsProtectedSink,
        Self::NeedsHumanReview,
        Self::NeedsFreshPolicy,
    ];

    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Deny => "Deny",
            Self::AllowCandidate => "AllowCandidate",
            Self::NeedsMinimization => "NeedsMinimization",
            Self::NeedsDeclassification => "NeedsDeclassification",
            Self::NeedsProtectedSink => "NeedsProtectedSink",
            Self::NeedsHumanReview => "NeedsHumanReview",
            Self::NeedsFreshPolicy => "NeedsFreshPolicy",
        }
    }

    pub const fn authority_scope(self) -> OpsecAuthorityScopeV1 {
        OpsecAuthorityScopeV1::IdentityAndVocabularyOnly
    }
}

/// Version/profile identities frozen by OPSEC-000A.
pub mod profiles {
    pub const OPSEC_COMPOSITION_V0_1: &str = "mycelix:opsec-composition-corpus:v0.1";
    pub const OPSEC_CORE_V0_1: &str = "mycelix:opsec-core:v0.1";
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::any::TypeId;
    use std::collections::BTreeSet;

    #[test]
    fn references_reject_empty_oversize_whitespace_and_non_ascii() {
        assert_eq!(PurposeRef::new(""), Err(OpsecRefError::Empty));
        assert!(matches!(
            PurposeRef::new("x".repeat(MAX_OPSEC_REF_BYTES_V1 + 1)),
            Err(OpsecRefError::TooLong { .. })
        ));
        assert_eq!(
            PurposeRef::new("contains space"),
            Err(OpsecRefError::NonGraphicAscii)
        );
        assert_eq!(
            PurposeRef::new("sensitive-π"),
            Err(OpsecRefError::NonGraphicAscii)
        );
        assert!(PurposeRef::new("purpose:research/v1").is_ok());
    }

    #[test]
    fn ordinary_debug_redacts_reference_content() {
        let value = DestinationRef::new("sensitive-destination:case-4815").unwrap();
        let rendered = format!("{value:?}");
        assert!(rendered.contains("<redacted>"));
        assert!(!rendered.contains("case-4815"));
        assert!(!rendered.contains("sensitive-destination"));
    }

    #[test]
    fn role_identity_is_not_erased_by_shared_text() {
        let purpose = PurposeRef::new("same-text").unwrap();
        let destination = DestinationRef::new("same-text").unwrap();

        assert_eq!(purpose.expose_for_binding(), destination.expose_for_binding());
        assert_ne!(TypeId::of::<PurposeRef>(), TypeId::of::<DestinationRef>());
        assert_ne!(PurposeRef::ROLE, DestinationRef::ROLE);
    }

    #[test]
    fn sink_vocabulary_is_complete_and_unique() {
        let names: BTreeSet<_> = OpsecDisclosureSinkClassV1::ALL
            .into_iter()
            .map(OpsecDisclosureSinkClassV1::as_str)
            .collect();
        assert_eq!(names.len(), 14);
        assert!(names.contains("LocalOperationalLog"));
        assert!(names.contains("TelemetryTraceMetric"));
        assert!(names.contains("RemoteWebRequest"));
        assert!(names.contains("RemoteModelApi"));
        assert!(names.contains("DurableEvidenceStore"));
        assert!(names.contains("EvidenceExport"));
    }

    #[test]
    fn web_surface_vocabulary_is_complete_and_unique() {
        let names: BTreeSet<_> = OpsecDisclosureSurfaceV1::ALL
            .into_iter()
            .map(OpsecDisclosureSurfaceV1::as_str)
            .collect();
        assert_eq!(names.len(), 11);
        assert!(names.contains("DnsQueryName"));
        assert!(names.contains("HttpRequestTarget"));
        assert!(names.contains("HttpHeaders"));
        assert!(names.contains("HttpBody"));
        assert!(names.contains("LocalLogsTracesMetrics"));
    }

    #[test]
    fn decision_vocabulary_is_candidate_evidence_only() {
        let names: BTreeSet<_> = OpsecDecisionEvidenceDispositionV1::ALL
            .into_iter()
            .map(OpsecDecisionEvidenceDispositionV1::as_str)
            .collect();
        assert_eq!(names.len(), 7);
        assert!(names.contains("AllowCandidate"));
        assert!(!names.contains("Allow"));
        assert!(!names.contains("DispatchPermit"));
    }

    #[test]
    fn authority_ceiling_is_machine_readable() {
        assert_eq!(
            OpsecAuthorityScopeV1::IdentityAndVocabularyOnly.as_str(),
            "identity-and-vocabulary-only"
        );
        assert_eq!(
            OpsecDisclosureSinkClassV1::RemoteWebRequest.authority_scope(),
            OpsecAuthorityScopeV1::IdentityAndVocabularyOnly
        );
        assert_eq!(
            OpsecDecisionEvidenceDispositionV1::AllowCandidate.authority_scope(),
            OpsecAuthorityScopeV1::IdentityAndVocabularyOnly
        );
    }

    #[test]
    fn secret_class_is_only_an_opaque_reference() {
        let class = SecretClassRef::new("credential-secret:v1").unwrap();
        assert_eq!(class.expose_for_binding(), "credential-secret:v1");
        let rendered = format!("{class:?}");
        assert!(!rendered.contains("credential-secret:v1"));
    }
}
