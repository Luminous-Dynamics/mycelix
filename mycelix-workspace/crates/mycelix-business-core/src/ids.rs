// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Opaque identifiers used by the transport-neutral institutional contracts.

use core::fmt;

/// Error returned when an opaque identifier is empty or whitespace-only.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct IdError;

impl fmt::Display for IdError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str("identifier must not be empty or whitespace-only")
    }
}

impl std::error::Error for IdError {}

macro_rules! string_id {
    ($name:ident, $doc:literal) => {
        #[doc = $doc]
        #[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
        pub struct $name(String);

        impl $name {
            /// Construct a non-empty opaque identifier.
            pub fn new(value: impl Into<String>) -> Result<Self, IdError> {
                let value = value.into();
                if value.trim().is_empty() {
                    return Err(IdError);
                }
                Ok(Self(value))
            }

            /// Borrow the opaque identifier as text.
            #[must_use]
            pub fn as_str(&self) -> &str {
                &self.0
            }

            /// Consume the identifier and return its string representation.
            #[must_use]
            pub fn into_string(self) -> String {
                self.0
            }
        }

        impl fmt::Display for $name {
            fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
                f.write_str(&self.0)
            }
        }

        impl TryFrom<String> for $name {
            type Error = IdError;

            fn try_from(value: String) -> Result<Self, Self::Error> {
                Self::new(value)
            }
        }

        impl TryFrom<&str> for $name {
            type Error = IdError;

            fn try_from(value: &str) -> Result<Self, Self::Error> {
                Self::new(value)
            }
        }
    };
}

string_id!(ProfileName, "Semantic profile namespace.");
string_id!(
    OrganizationContextRef,
    "Opaque organization/institution context reference."
);
string_id!(
    PartyRef,
    "Opaque reference to a party owned by an identity domain."
);
string_id!(
    SubjectRef,
    "Opaque reference to the subject of an institutional statement."
);
string_id!(CommitmentRef, "Opaque reference to a commitment.");
string_id!(ObligationRef, "Domain-local reference to an obligation.");
string_id!(
    PowerGrantRef,
    "Opaque reference to a domain-owned institutional power grant."
);
string_id!(
    AuthorizationDecisionRef,
    "Domain-local identifier of a current authorization decision."
);
string_id!(
    LogicalIntentRef,
    "Identity of one logical institutional effect."
);
string_id!(
    OperationCommitment,
    "Owning-domain/profile commitment to the material semantics of one logical operation."
);
string_id!(AttemptRef, "Identity of one concrete execution attempt.");
string_id!(
    SourceSystemRef,
    "Identity of one external provider or authoritative source system."
);
string_id!(
    TransportDeliveryRef,
    "Identity of one concrete transport/message delivery."
);
string_id!(
    ObservationRef,
    "Opaque reference to a source-attributed observation."
);
string_id!(EvidenceRef, "Opaque reference to supporting evidence.");
string_id!(
    ReconciliationRef,
    "Domain-local identifier of a reconciliation."
);
string_id!(AllocationRef, "Domain-local identifier of an allocation result.");
string_id!(WorkflowRef, "Opaque reference to a Business workflow instance.");
string_id!(ClosurePolicyRef, "Opaque reference to a closure policy.");
string_id!(
    ExceptionRef,
    "Domain-local reference to a retained closure/dispute exception."
);
string_id!(DomainRef, "Opaque authoritative-domain identifier.");
string_id!(RecordRef, "Opaque domain-owned record identifier.");
string_id!(
    GenerationNamespace,
    "Domain-local namespace within which one semantic generation is required."
);
string_id!(
    DerivationNodeRef,
    "Opaque node identifier for qualification dependency graphs."
);
string_id!(
    UnitId,
    "Semantic unit/currency/quantity identifier used by allocation statements."
);

/// Domain-scoped identity for a domain-owned object.
///
/// `DomainScopedRef<T>` only prevents namespace collision. It does **not** prove
/// that the named domain is authoritative for the object or that the object is
/// valid. Those remain owning-domain responsibilities.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct DomainScopedRef<T> {
    domain: DomainRef,
    local: T,
}

impl<T> DomainScopedRef<T> {
    /// Bind one domain-local identifier to its authoritative-domain namespace.
    #[must_use]
    pub fn new(domain: DomainRef, local: T) -> Self {
        Self { domain, local }
    }

    /// Domain namespace participating in identity.
    #[must_use]
    pub fn domain(&self) -> &DomainRef {
        &self.domain
    }

    /// Domain-local identifier.
    #[must_use]
    pub fn local(&self) -> &T {
        &self.local
    }

    /// Consume the scoped reference.
    #[must_use]
    pub fn into_parts(self) -> (DomainRef, T) {
        (self.domain, self.local)
    }
}

impl<T: fmt::Display> fmt::Display for DomainScopedRef<T> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}/{}", self.domain, self.local)
    }
}

/// Domain-scoped authorization-decision identity.
pub type DomainAuthorizationDecisionRef = DomainScopedRef<AuthorizationDecisionRef>;
/// Domain-scoped reconciliation identity.
pub type DomainReconciliationRef = DomainScopedRef<ReconciliationRef>;
/// Domain-scoped allocation identity.
pub type DomainAllocationRef = DomainScopedRef<AllocationRef>;
/// Domain-scoped obligation identity.
pub type DomainObligationRef = DomainScopedRef<ObligationRef>;
/// Domain-scoped retained exception/dispute identity.
pub type DomainExceptionRef = DomainScopedRef<ExceptionRef>;

/// Test-only migration from pre-scoping obligation fixtures.
///
/// Production callers never receive this conversion and must name the owning
/// domain explicitly.
#[cfg(test)]
impl From<ObligationRef> for DomainScopedRef<ObligationRef> {
    fn from(local: ObligationRef) -> Self {
        Self::new(
            DomainRef::new("test-fixture").expect("static test domain is valid"),
            local,
        )
    }
}

/// Test-only migration from pre-scoping exception fixtures.
///
/// Production callers never receive this conversion and must name the owning
/// domain explicitly.
#[cfg(test)]
impl From<ExceptionRef> for DomainScopedRef<ExceptionRef> {
    fn from(local: ExceptionRef) -> Self {
        Self::new(
            DomainRef::new("test-fixture").expect("static test domain is valid"),
            local,
        )
    }
}

/// Source-scoped identity of one external/domain-generated event.
///
/// The source namespace is part of identity so two providers may both use the
/// same local event id without accidental deduplication.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct SourceEventRef {
    source: SourceSystemRef,
    event_id: String,
}

impl SourceEventRef {
    /// Construct a source-scoped event identity.
    pub fn new(
        source: impl Into<String>,
        event_id: impl Into<String>,
    ) -> Result<Self, IdError> {
        let source = SourceSystemRef::new(source)?;
        let event_id = event_id.into();
        if event_id.trim().is_empty() {
            return Err(IdError);
        }

        Ok(Self { source, event_id })
    }

    /// Source/provider namespace.
    #[must_use]
    pub fn source(&self) -> &SourceSystemRef {
        &self.source
    }

    /// Source-local event identifier.
    #[must_use]
    pub fn event_id(&self) -> &str {
        &self.event_id
    }
}

impl fmt::Display for SourceEventRef {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}:{}", self.source, self.event_id)
    }
}

/// Stable semantic profile identifier.
///
/// The profile version is part of institutional meaning. A representation upgrade
/// must not silently reinterpret a historical record under a new profile.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct SemanticProfileId {
    name: ProfileName,
    version: u32,
}

impl SemanticProfileId {
    /// Construct a profile from a non-empty name and explicit version.
    pub fn new(name: impl Into<String>, version: u32) -> Result<Self, IdError> {
        Ok(Self {
            name: ProfileName::new(name)?,
            version,
        })
    }

    /// Profile namespace.
    #[must_use]
    pub fn name(&self) -> &ProfileName {
        &self.name
    }

    /// Explicit semantic profile version.
    #[must_use]
    pub const fn version(&self) -> u32 {
        self.version
    }
}

impl fmt::Display for SemanticProfileId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}@{}", self.name, self.version)
    }
}
