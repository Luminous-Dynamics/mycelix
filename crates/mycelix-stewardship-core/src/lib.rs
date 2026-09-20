// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Dependency-free identity theorems for stewarded knowledge and creative artifacts.
//!
//! STEW-001 intentionally owns only structural identity. It does not establish
//! authorship, ownership, cultural legitimacy, access permission, epistemic
//! truth, preservation success, or any other authority.

#![forbid(unsafe_code)]

use core::fmt;

/// Stable profile identifier for this theorem.
pub const STEWARDED_SUBJECT_IDENTITY_PROFILE_V1: &str = "mycelix/stewarded-subject-identity/v1";

/// Maximum byte length of protocol identifiers in v1.
pub const MAX_CANONICAL_ID_BYTES_V1: usize = 256;

/// Validation failure for a canonical protocol identifier.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CanonicalIdErrorV1 {
    /// Empty identifiers are not permitted.
    Empty,
    /// IDs are bounded so untrusted metadata cannot create unbounded keys.
    TooLong,
    /// v1 IDs are opaque ASCII protocol identifiers, not display text.
    InvalidCharacter,
}

impl fmt::Display for CanonicalIdErrorV1 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Empty => f.write_str("identifier must not be empty"),
            Self::TooLong => f.write_str("identifier exceeds v1 byte limit"),
            Self::InvalidCharacter => {
                f.write_str("identifier contains a character outside the v1 protocol alphabet")
            }
        }
    }
}

/// Canonical opaque protocol identifier.
///
/// Human-facing names, titles, community names and translations deliberately do
/// not use this type; those need full Unicode and domain-specific semantics.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct CanonicalIdV1(String);

impl CanonicalIdV1 {
    /// Construct a validated v1 identifier.
    pub fn new(value: impl Into<String>) -> Result<Self, CanonicalIdErrorV1> {
        let value = value.into();
        if value.is_empty() {
            return Err(CanonicalIdErrorV1::Empty);
        }
        if value.len() > MAX_CANONICAL_ID_BYTES_V1 {
            return Err(CanonicalIdErrorV1::TooLong);
        }
        if !value.bytes().all(is_v1_id_byte) {
            return Err(CanonicalIdErrorV1::InvalidCharacter);
        }
        Ok(Self(value))
    }

    /// Borrow the exact identifier bytes as UTF-8 text.
    pub fn as_str(&self) -> &str {
        &self.0
    }
}

impl fmt::Display for CanonicalIdV1 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.0)
    }
}

fn is_v1_id_byte(byte: u8) -> bool {
    byte.is_ascii_alphanumeric() || matches!(byte, b'-' | b'_' | b'.' | b':' | b'/' | b'@' | b'+')
}

macro_rules! typed_id {
    ($name:ident, $doc:literal) => {
        #[doc = $doc]
        #[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
        pub struct $name(CanonicalIdV1);

        impl $name {
            /// Construct a validated typed identifier.
            pub fn new(value: impl Into<String>) -> Result<Self, CanonicalIdErrorV1> {
                CanonicalIdV1::new(value).map(Self)
            }

            /// Borrow the canonical identifier.
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

typed_id!(
    StewardedSubjectIdV1,
    "Logical identity of a stewarded work/practice/artifact across revisions."
);
typed_id!(
    RevisionIdV1,
    "Identity of one revision/state of a logical stewarded subject."
);
typed_id!(
    RepresentationIdV1,
    "Identity of one concrete representation/encoding of a revision."
);

/// Hash algorithms admitted by the v1 exact-byte commitment profile.
///
/// The crate carries already-computed digests. It deliberately contains no
/// hashing implementation and therefore cannot claim that supplied bytes were
/// actually hashed correctly.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum DigestAlgorithmV1 {
    /// BLAKE3 with a 256-bit output.
    Blake3_256,
    /// SHA-256.
    Sha256,
}

/// Exact-byte commitment for one representation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct ContentDigestV1 {
    /// Algorithm naming the interpretation of `bytes`.
    pub algorithm: DigestAlgorithmV1,
    /// Exact 256-bit digest value.
    pub bytes: [u8; 32],
}

impl ContentDigestV1 {
    /// Construct an already-computed exact-byte commitment.
    pub const fn new(algorithm: DigestAlgorithmV1, bytes: [u8; 32]) -> Self {
        Self { algorithm, bytes }
    }
}

/// Media-neutral structural category of a representation.
///
/// Transformative relations such as translation, restoration, remix, or
/// derivation belong in later provenance theorems rather than being smuggled
/// into representation identity.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum RepresentationKindV1 {
    /// Textual representation.
    Text,
    /// Still image representation.
    Image,
    /// Audio representation.
    Audio,
    /// Video representation.
    Video,
    /// Musical or other symbolic score/notation.
    Score,
    /// Software/source/binary representation.
    Software,
    /// Dataset or structured data representation.
    Dataset,
    /// Multi-modal or compound representation.
    MixedMedia,
    /// Representation kind is known but outside the closed v1 vocabulary.
    Other,
}

/// Canonical structural identity of one exact representation of a stewarded subject.
///
/// The four layers intentionally remain separate:
///
/// ```text
/// logical subject != revision != representation != exact bytes
/// ```
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct StewardedSubjectIdentityV1 {
    /// Stable logical subject identity.
    pub subject: StewardedSubjectIdV1,
    /// Revision identity within the logical subject.
    pub revision: RevisionIdV1,
    /// Representation identity within the revision.
    pub representation: RepresentationIdV1,
    /// Media-neutral representation category.
    pub kind: RepresentationKindV1,
    /// Exact-byte commitment for this representation.
    pub content_digest: ContentDigestV1,
}

/// Closed-world structural relationship between two v1 identities.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IdentityRelationV1 {
    /// Logical subjects differ, regardless of any shared bytes.
    DistinctSubject,
    /// Same logical subject, but revisions differ.
    SameSubjectDifferentRevision,
    /// Same subject and revision, but representation identities differ.
    SameRevisionDifferentRepresentation,
    /// Same subject/revision/representation ID, but exact bytes differ.
    SameRepresentationIdentityDifferentBytes,
    /// All structural identity layers and exact-byte commitments match.
    ExactRepresentation,
}

impl StewardedSubjectIdentityV1 {
    /// Compare two identities without collapsing any identity layer.
    pub fn relation_to(&self, other: &Self) -> IdentityRelationV1 {
        if self.subject != other.subject {
            return IdentityRelationV1::DistinctSubject;
        }
        if self.revision != other.revision {
            return IdentityRelationV1::SameSubjectDifferentRevision;
        }
        if self.representation != other.representation || self.kind != other.kind {
            return IdentityRelationV1::SameRevisionDifferentRepresentation;
        }
        if self.content_digest != other.content_digest {
            return IdentityRelationV1::SameRepresentationIdentityDifferentBytes;
        }
        IdentityRelationV1::ExactRepresentation
    }

    /// Whether the exact byte commitment matches.
    ///
    /// A `true` result does **not** imply the same logical subject or revision.
    pub fn same_committed_bytes_as(&self, other: &Self) -> bool {
        self.content_digest == other.content_digest
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn subject(value: &str) -> StewardedSubjectIdV1 {
        StewardedSubjectIdV1::new(value).unwrap()
    }

    fn revision(value: &str) -> RevisionIdV1 {
        RevisionIdV1::new(value).unwrap()
    }

    fn representation(value: &str) -> RepresentationIdV1 {
        RepresentationIdV1::new(value).unwrap()
    }

    fn digest(fill: u8) -> ContentDigestV1 {
        ContentDigestV1::new(DigestAlgorithmV1::Blake3_256, [fill; 32])
    }

    fn identity(
        subject_id: &str,
        revision_id: &str,
        representation_id: &str,
        kind: RepresentationKindV1,
        digest_fill: u8,
    ) -> StewardedSubjectIdentityV1 {
        StewardedSubjectIdentityV1 {
            subject: subject(subject_id),
            revision: revision(revision_id),
            representation: representation(representation_id),
            kind,
            content_digest: digest(digest_fill),
        }
    }

    #[test]
    fn canonical_id_accepts_namespaced_protocol_identifiers() {
        let id =
            CanonicalIdV1::new("urn:mycelix:stew:subject/abc-123_v1.0@community+local").unwrap();
        assert_eq!(
            id.as_str(),
            "urn:mycelix:stew:subject/abc-123_v1.0@community+local"
        );
    }

    #[test]
    fn canonical_id_rejects_empty_whitespace_unicode_and_overlong_values() {
        assert_eq!(CanonicalIdV1::new(""), Err(CanonicalIdErrorV1::Empty));
        assert_eq!(
            CanonicalIdV1::new("contains space"),
            Err(CanonicalIdErrorV1::InvalidCharacter)
        );
        assert_eq!(
            CanonicalIdV1::new("文化"),
            Err(CanonicalIdErrorV1::InvalidCharacter)
        );
        assert_eq!(
            CanonicalIdV1::new("a".repeat(MAX_CANONICAL_ID_BYTES_V1 + 1)),
            Err(CanonicalIdErrorV1::TooLong)
        );
    }

    #[test]
    fn exact_representation_requires_all_layers_and_digest_to_match() {
        let a = identity(
            "subject:a",
            "revision:1",
            "representation:score",
            RepresentationKindV1::Score,
            7,
        );
        let b = a.clone();
        assert_eq!(a.relation_to(&b), IdentityRelationV1::ExactRepresentation);
    }

    #[test]
    fn same_digest_never_collapses_distinct_logical_subjects() {
        let a = identity(
            "subject:a",
            "revision:1",
            "representation:text",
            RepresentationKindV1::Text,
            9,
        );
        let b = identity(
            "subject:b",
            "revision:1",
            "representation:text",
            RepresentationKindV1::Text,
            9,
        );

        assert!(a.same_committed_bytes_as(&b));
        assert_eq!(a.relation_to(&b), IdentityRelationV1::DistinctSubject);
    }

    #[test]
    fn same_subject_does_not_collapse_distinct_revisions() {
        let a = identity(
            "subject:a",
            "revision:1",
            "representation:text",
            RepresentationKindV1::Text,
            1,
        );
        let b = identity(
            "subject:a",
            "revision:2",
            "representation:text",
            RepresentationKindV1::Text,
            2,
        );

        assert_eq!(
            a.relation_to(&b),
            IdentityRelationV1::SameSubjectDifferentRevision
        );
    }

    #[test]
    fn same_revision_does_not_collapse_distinct_representations() {
        let score = identity(
            "subject:composition-a",
            "revision:critical-edition-1",
            "representation:score-pdf",
            RepresentationKindV1::Score,
            3,
        );
        let scan = identity(
            "subject:composition-a",
            "revision:critical-edition-1",
            "representation:scan-tiff",
            RepresentationKindV1::Image,
            4,
        );

        assert_eq!(
            score.relation_to(&scan),
            IdentityRelationV1::SameRevisionDifferentRepresentation
        );
    }

    #[test]
    fn same_representation_id_with_different_bytes_is_explicit_conflict_state() {
        let a = identity(
            "subject:a",
            "revision:1",
            "representation:canonical",
            RepresentationKindV1::Text,
            10,
        );
        let b = identity(
            "subject:a",
            "revision:1",
            "representation:canonical",
            RepresentationKindV1::Text,
            11,
        );

        assert_eq!(
            a.relation_to(&b),
            IdentityRelationV1::SameRepresentationIdentityDifferentBytes
        );
    }

    #[test]
    fn digest_algorithm_is_part_of_exact_byte_commitment_identity() {
        let bytes = [42; 32];
        let a = ContentDigestV1::new(DigestAlgorithmV1::Blake3_256, bytes);
        let b = ContentDigestV1::new(DigestAlgorithmV1::Sha256, bytes);
        assert_ne!(a, b);
    }

    #[test]
    fn profile_identifier_is_frozen() {
        assert_eq!(
            STEWARDED_SUBJECT_IDENTITY_PROFILE_V1,
            "mycelix/stewarded-subject-identity/v1"
        );
    }
}
