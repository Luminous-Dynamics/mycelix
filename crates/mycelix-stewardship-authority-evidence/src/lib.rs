// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Neutral authority-evidence typing for Mycelix stewardship.
//!
//! This crate records typed authority evidence inputs. It does not decide whether
//! an evaluator, mandate, delegation, profile, or scope is actually authoritative.
//! Opaque bundle references are provenance locators only and are never semantic
//! identity for the evidence bundle they accompany.

#![forbid(unsafe_code)]

use core::fmt;
use mycelix_stewardship_core::{CanonicalIdErrorV1, CanonicalIdV1};

/// Stable profile identifier for the v1 neutral authority-evidence substrate.
pub const AUTHORITY_EVIDENCE_PROFILE_V1: &str = "mycelix/authority-evidence/v1";
/// Maximum number of references admitted in one v1 evidence plane.
pub const MAX_AUTHORITY_EVIDENCE_REFS_V1: usize = 32;

macro_rules! typed_ref {
    ($name:ident, $doc:literal) => {
        #[doc = $doc]
        #[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
        pub struct $name(CanonicalIdV1);

        impl $name {
            /// Construct a validated opaque protocol reference.
            pub fn new(value: impl Into<String>) -> Result<Self, CanonicalIdErrorV1> {
                CanonicalIdV1::new(value).map(Self)
            }

            /// Borrow the exact canonical reference text.
            pub fn as_str(&self) -> &str {
                self.0.as_str()
            }
        }

        impl fmt::Display for $name {
            fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
                self.0.fmt(f)
            }
        }
    };
}

typed_ref!(
    AuthorityEvidenceBundleRefV1,
    "Opaque provenance/locator reference for an authority-evidence bundle; not semantic identity."
);
typed_ref!(
    AuthoritySubjectRefV1,
    "Opaque identity of the principal/evaluator whose authority is asserted."
);
typed_ref!(
    AuthorityProfileRefV1,
    "Opaque identity of the authority-evaluation profile."
);
typed_ref!(
    AuthorityScopeRefV1,
    "Opaque identity of the authority scope evaluated by a later domain theorem."
);
typed_ref!(
    MandateEvidenceRefV1,
    "Opaque evidence reference supporting an asserted mandate or authority basis."
);
typed_ref!(
    AuthorityCurrentnessEvidenceRefV1,
    "Opaque evidence reference supporting independent authority currentness."
);
typed_ref!(
    AuthorityBindingEvidenceRefV1,
    "Opaque evidence reference binding the asserted subject/profile/scope relationship."
);

/// Evidence-bearing currentness assertion for an authority claim.
///
/// None of these variants is a verified authority verdict by itself.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum AuthorityCurrentnessAssertionV1 {
    /// Evidence asserts that the authority is current.
    AssertedCurrent,
    /// Evidence asserts that the authority was revoked.
    AssertedRevoked,
    /// Evidence asserts that the authority was superseded.
    AssertedSuperseded,
    /// Evidence asserts that the authority expired.
    AssertedExpired,
    /// The available evidence does not support a more specific assertion.
    Indeterminate,
}

/// Structural validation failure for one authority-evidence plane.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AuthorityEvidenceErrorV1 {
    /// The required mandate plane is empty.
    NoMandateEvidence,
    /// The mandate plane exceeds the v1 bound.
    TooManyMandateEvidenceReferences,
    /// The mandate plane repeats a reference.
    DuplicateMandateEvidenceReference,
    /// The required currentness plane is empty.
    NoCurrentnessEvidence,
    /// The currentness plane exceeds the v1 bound.
    TooManyCurrentnessEvidenceReferences,
    /// The currentness plane repeats a reference.
    DuplicateCurrentnessEvidenceReference,
    /// The required subject/profile/scope binding plane is empty.
    NoBindingEvidence,
    /// The binding plane exceeds the v1 bound.
    TooManyBindingEvidenceReferences,
    /// The binding plane repeats a reference.
    DuplicateBindingEvidenceReference,
}

impl fmt::Display for AuthorityEvidenceErrorV1 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let message = match self {
            Self::NoMandateEvidence => "authority evidence bundle requires mandate evidence",
            Self::TooManyMandateEvidenceReferences => "too many mandate evidence references",
            Self::DuplicateMandateEvidenceReference => "duplicate mandate evidence reference",
            Self::NoCurrentnessEvidence => {
                "authority evidence bundle requires currentness evidence"
            }
            Self::TooManyCurrentnessEvidenceReferences => {
                "too many authority-currentness evidence references"
            }
            Self::DuplicateCurrentnessEvidenceReference => {
                "duplicate authority-currentness evidence reference"
            }
            Self::NoBindingEvidence => "authority evidence bundle requires binding evidence",
            Self::TooManyBindingEvidenceReferences => "too many binding evidence references",
            Self::DuplicateBindingEvidenceReference => "duplicate binding evidence reference",
        };
        f.write_str(message)
    }
}

macro_rules! evidence_plane {
    (
        $name:ident,
        $ref_name:ident,
        $empty_error:ident,
        $too_many_error:ident,
        $duplicate_error:ident,
        $doc:literal
    ) => {
        #[doc = $doc]
        #[derive(Debug, Clone, PartialEq, Eq)]
        pub struct $name(Vec<$ref_name>);

        impl $name {
            /// Construct a non-empty, bounded, duplicate-free evidence plane.
            pub fn new(refs: Vec<$ref_name>) -> Result<Self, AuthorityEvidenceErrorV1> {
                if refs.is_empty() {
                    return Err(AuthorityEvidenceErrorV1::$empty_error);
                }
                if refs.len() > MAX_AUTHORITY_EVIDENCE_REFS_V1 {
                    return Err(AuthorityEvidenceErrorV1::$too_many_error);
                }
                for (index, reference) in refs.iter().enumerate() {
                    if refs[..index].contains(reference) {
                        return Err(AuthorityEvidenceErrorV1::$duplicate_error);
                    }
                }
                Ok(Self(refs))
            }

            /// Borrow the exact evidence references in caller-supplied order.
            pub fn as_slice(&self) -> &[$ref_name] {
                &self.0
            }

            /// Number of retained evidence references.
            pub fn len(&self) -> usize {
                self.0.len()
            }

            /// Whether the plane is empty. Valid v1 planes always return false.
            pub fn is_empty(&self) -> bool {
                self.0.is_empty()
            }
        }
    };
}

evidence_plane!(
    MandateEvidenceRefsV1,
    MandateEvidenceRefV1,
    NoMandateEvidence,
    TooManyMandateEvidenceReferences,
    DuplicateMandateEvidenceReference,
    "Validated mandate/authority-basis evidence references."
);
evidence_plane!(
    AuthorityCurrentnessEvidenceRefsV1,
    AuthorityCurrentnessEvidenceRefV1,
    NoCurrentnessEvidence,
    TooManyCurrentnessEvidenceReferences,
    DuplicateCurrentnessEvidenceReference,
    "Validated evidence references for the independent authority-currentness assertion."
);
evidence_plane!(
    AuthorityBindingEvidenceRefsV1,
    AuthorityBindingEvidenceRefV1,
    NoBindingEvidence,
    TooManyBindingEvidenceReferences,
    DuplicateBindingEvidenceReference,
    "Validated evidence references binding authority subject, profile, and scope."
);

/// Compile-time-separated evidence planes for one authority assertion.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct AuthorityEvidencePlanesV1 {
    mandate: MandateEvidenceRefsV1,
    currentness: AuthorityCurrentnessEvidenceRefsV1,
    binding: AuthorityBindingEvidenceRefsV1,
}

impl AuthorityEvidencePlanesV1 {
    /// Assemble already-validated evidence planes.
    pub fn new(
        mandate: MandateEvidenceRefsV1,
        currentness: AuthorityCurrentnessEvidenceRefsV1,
        binding: AuthorityBindingEvidenceRefsV1,
    ) -> Self {
        Self {
            mandate,
            currentness,
            binding,
        }
    }

    /// Mandate/authority-basis evidence plane.
    pub fn mandate(&self) -> &MandateEvidenceRefsV1 {
        &self.mandate
    }

    /// Independent authority-currentness evidence plane.
    pub fn currentness(&self) -> &AuthorityCurrentnessEvidenceRefsV1 {
        &self.currentness
    }

    /// Subject/profile/scope binding evidence plane.
    pub fn binding(&self) -> &AuthorityBindingEvidenceRefsV1 {
        &self.binding
    }
}

/// Neutral evidence bundle for a later domain-specific authority theorem.
///
/// The `bundle_ref` is only an opaque provenance/locator value. It is not a
/// canonical commitment and cannot identify the semantic bundle contents.
/// Domain theorems that need exact equality must retain/compare this complete
/// typed value or use a separately qualified canonical-commitment theorem.
///
/// This type intentionally has no `is_authorized`, `permit`, or authority-score
/// method. In particular, `AssertedCurrent` remains an evidence-bearing assertion
/// and not a verified authority verdict.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct AuthorityEvidenceBundleV1 {
    bundle_ref: AuthorityEvidenceBundleRefV1,
    subject: AuthoritySubjectRefV1,
    profile: AuthorityProfileRefV1,
    scope: AuthorityScopeRefV1,
    currentness: AuthorityCurrentnessAssertionV1,
    evidence_planes: AuthorityEvidencePlanesV1,
}

impl AuthorityEvidenceBundleV1 {
    /// Construct a neutral evidence bundle from validated typed inputs.
    pub fn new(
        bundle_ref: AuthorityEvidenceBundleRefV1,
        subject: AuthoritySubjectRefV1,
        profile: AuthorityProfileRefV1,
        scope: AuthorityScopeRefV1,
        currentness: AuthorityCurrentnessAssertionV1,
        evidence_planes: AuthorityEvidencePlanesV1,
    ) -> Self {
        Self {
            bundle_ref,
            subject,
            profile,
            scope,
            currentness,
            evidence_planes,
        }
    }

    /// Opaque provenance/locator reference. Not semantic bundle identity.
    pub fn bundle_ref(&self) -> &AuthorityEvidenceBundleRefV1 {
        &self.bundle_ref
    }

    /// Authority subject asserted by this evidence bundle.
    pub fn subject(&self) -> &AuthoritySubjectRefV1 {
        &self.subject
    }

    /// Authority-evaluation profile asserted by this evidence bundle.
    pub fn profile(&self) -> &AuthorityProfileRefV1 {
        &self.profile
    }

    /// Authority scope asserted by this evidence bundle.
    pub fn scope(&self) -> &AuthorityScopeRefV1 {
        &self.scope
    }

    /// Independent evidence-bearing authority-currentness assertion.
    pub const fn currentness(&self) -> AuthorityCurrentnessAssertionV1 {
        self.currentness
    }

    /// Compile-time-separated evidence planes.
    pub fn evidence_planes(&self) -> &AuthorityEvidencePlanesV1 {
        &self.evidence_planes
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn planes() -> AuthorityEvidencePlanesV1 {
        AuthorityEvidencePlanesV1::new(
            MandateEvidenceRefsV1::new(vec![
                MandateEvidenceRefV1::new("evidence:mandate:1").unwrap(),
            ])
            .unwrap(),
            AuthorityCurrentnessEvidenceRefsV1::new(vec![
                AuthorityCurrentnessEvidenceRefV1::new("evidence:currentness:1").unwrap(),
            ])
            .unwrap(),
            AuthorityBindingEvidenceRefsV1::new(vec![
                AuthorityBindingEvidenceRefV1::new("evidence:binding:1").unwrap(),
            ])
            .unwrap(),
        )
    }

    fn bundle(currentness: AuthorityCurrentnessAssertionV1) -> AuthorityEvidenceBundleV1 {
        AuthorityEvidenceBundleV1::new(
            AuthorityEvidenceBundleRefV1::new("authority-evidence-ref:1").unwrap(),
            AuthoritySubjectRefV1::new("principal:evaluator:1").unwrap(),
            AuthorityProfileRefV1::new("authority-profile:1").unwrap(),
            AuthorityScopeRefV1::new("authority-scope:1").unwrap(),
            currentness,
            planes(),
        )
    }

    #[test]
    fn typed_planes_retain_exact_semantic_refs() {
        let record = bundle(AuthorityCurrentnessAssertionV1::AssertedCurrent);
        assert_eq!(record.bundle_ref().as_str(), "authority-evidence-ref:1");
        assert_eq!(record.subject().as_str(), "principal:evaluator:1");
        assert_eq!(record.profile().as_str(), "authority-profile:1");
        assert_eq!(record.scope().as_str(), "authority-scope:1");
        assert_eq!(
            record.evidence_planes().mandate().as_slice()[0].as_str(),
            "evidence:mandate:1"
        );
        assert_eq!(
            record.evidence_planes().currentness().as_slice()[0].as_str(),
            "evidence:currentness:1"
        );
        assert_eq!(
            record.evidence_planes().binding().as_slice()[0].as_str(),
            "evidence:binding:1"
        );
    }

    #[test]
    fn opaque_bundle_reference_is_not_semantic_identity() {
        let shared_ref = AuthorityEvidenceBundleRefV1::new("authority-evidence-ref:shared").unwrap();
        let first = AuthorityEvidenceBundleV1::new(
            shared_ref.clone(),
            AuthoritySubjectRefV1::new("principal:evaluator:1").unwrap(),
            AuthorityProfileRefV1::new("authority-profile:1").unwrap(),
            AuthorityScopeRefV1::new("authority-scope:1").unwrap(),
            AuthorityCurrentnessAssertionV1::AssertedCurrent,
            planes(),
        );
        let second = AuthorityEvidenceBundleV1::new(
            shared_ref,
            AuthoritySubjectRefV1::new("principal:evaluator:2").unwrap(),
            AuthorityProfileRefV1::new("authority-profile:1").unwrap(),
            AuthorityScopeRefV1::new("authority-scope:1").unwrap(),
            AuthorityCurrentnessAssertionV1::AssertedCurrent,
            planes(),
        );

        assert_eq!(first.bundle_ref(), second.bundle_ref());
        assert_ne!(first, second);
    }

    #[test]
    fn every_evidence_plane_is_required_in_v1() {
        assert_eq!(
            MandateEvidenceRefsV1::new(vec![]),
            Err(AuthorityEvidenceErrorV1::NoMandateEvidence)
        );
        assert_eq!(
            AuthorityCurrentnessEvidenceRefsV1::new(vec![]),
            Err(AuthorityEvidenceErrorV1::NoCurrentnessEvidence)
        );
        assert_eq!(
            AuthorityBindingEvidenceRefsV1::new(vec![]),
            Err(AuthorityEvidenceErrorV1::NoBindingEvidence)
        );
    }

    #[test]
    fn duplicate_references_fail_closed_per_plane() {
        let first = MandateEvidenceRefV1::new("evidence:mandate:duplicate").unwrap();
        assert_eq!(
            MandateEvidenceRefsV1::new(vec![first.clone(), first]),
            Err(AuthorityEvidenceErrorV1::DuplicateMandateEvidenceReference)
        );

        let first =
            AuthorityCurrentnessEvidenceRefV1::new("evidence:currentness:duplicate").unwrap();
        assert_eq!(
            AuthorityCurrentnessEvidenceRefsV1::new(vec![first.clone(), first]),
            Err(AuthorityEvidenceErrorV1::DuplicateCurrentnessEvidenceReference)
        );

        let first = AuthorityBindingEvidenceRefV1::new("evidence:binding:duplicate").unwrap();
        assert_eq!(
            AuthorityBindingEvidenceRefsV1::new(vec![first.clone(), first]),
            Err(AuthorityEvidenceErrorV1::DuplicateBindingEvidenceReference)
        );
    }

    #[test]
    fn evidence_reference_count_is_bounded() {
        let refs = (0..=MAX_AUTHORITY_EVIDENCE_REFS_V1)
            .map(|index| MandateEvidenceRefV1::new(format!("evidence:mandate:{index}")).unwrap())
            .collect();
        assert_eq!(
            MandateEvidenceRefsV1::new(refs),
            Err(AuthorityEvidenceErrorV1::TooManyMandateEvidenceReferences)
        );
    }

    #[test]
    fn authority_currentness_remains_an_independent_assertion() {
        for currentness in [
            AuthorityCurrentnessAssertionV1::AssertedCurrent,
            AuthorityCurrentnessAssertionV1::AssertedRevoked,
            AuthorityCurrentnessAssertionV1::AssertedSuperseded,
            AuthorityCurrentnessAssertionV1::AssertedExpired,
            AuthorityCurrentnessAssertionV1::Indeterminate,
        ] {
            assert_eq!(bundle(currentness).currentness(), currentness);
        }
    }

    #[test]
    fn profile_identifier_is_frozen() {
        assert_eq!(AUTHORITY_EVIDENCE_PROFILE_V1, "mycelix/authority-evidence/v1");
    }
}
