// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Provider-neutral digest identity for admitted qualification envelopes.
//!
//! This module deliberately does **not** implement a cryptographic hash.
//! Instead it freezes the transcript boundary a concrete crypto provider must
//! satisfy later: the digest input is the exact admitted v0.4 envelope byte
//! slice, not a decoded/re-serialized semantic object.

use crate::qualification_admission::{AdmittedQualificationBytes, QualificationAdmissionProfileV1};
use crate::qualification_envelope::{
    QualificationEnvelopeCodecV1, QualificationEnvelopeError, QualificationEnvelopeKind,
};

/// Durable digest-profile schema version.
pub const QUALIFICATION_DIGEST_PROFILE_SCHEMA_V1: u16 = 1;

/// Canonical preimage identifier for one digest profile.
pub const QUALIFICATION_DIGEST_PROFILE_PREIMAGE_V1: &str =
    "mycelix-position-qualification-digest-profile-v1";

const PROFILE_DOMAIN_SEPARATOR: &[u8] = b"MYCELIX-POSITION-QUALIFICATION-DIGEST-PROFILE\0V1\0";
const MAX_PROFILE_ID_BYTES: usize = 128;
const MAX_ALGORITHM_ID_BYTES: usize = 128;
const MAX_DIGEST_OUTPUT_BYTES: u16 = 1024;

/// Durable description of one digest algorithm/profile contract.
///
/// Fields are private so a successfully constructed profile cannot be mutated
/// into an unsupported or internally inconsistent profile by downstream code.
/// This identifies expected provider/output shape only; it does not claim that
/// an implementation of the named algorithm has been cryptographically qualified.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct QualificationDigestProfileV1 {
    schema_version: u16,
    profile_id: String,
    algorithm_id: String,
    output_bytes: u16,
}

impl QualificationDigestProfileV1 {
    pub fn new(
        profile_id: impl Into<String>,
        algorithm_id: impl Into<String>,
        output_bytes: u16,
    ) -> Result<Self, QualificationDigestError> {
        let profile = Self {
            schema_version: QUALIFICATION_DIGEST_PROFILE_SCHEMA_V1,
            profile_id: profile_id.into(),
            algorithm_id: algorithm_id.into(),
            output_bytes,
        };
        profile.validate()?;
        Ok(profile)
    }

    pub fn schema_version(&self) -> u16 {
        self.schema_version
    }

    pub fn profile_id(&self) -> &str {
        &self.profile_id
    }

    pub fn algorithm_id(&self) -> &str {
        &self.algorithm_id
    }

    pub fn output_bytes(&self) -> u16 {
        self.output_bytes
    }

    pub fn validate(&self) -> Result<(), QualificationDigestError> {
        if self.schema_version != QUALIFICATION_DIGEST_PROFILE_SCHEMA_V1 {
            return Err(QualificationDigestError::UnsupportedProfileVersion {
                actual: self.schema_version,
            });
        }
        validate_identifier(
            &self.profile_id,
            MAX_PROFILE_ID_BYTES,
            DigestIdentifierField::ProfileId,
        )?;
        validate_identifier(
            &self.algorithm_id,
            MAX_ALGORITHM_ID_BYTES,
            DigestIdentifierField::AlgorithmId,
        )?;
        if self.output_bytes == 0 || self.output_bytes > MAX_DIGEST_OUTPUT_BYTES {
            return Err(QualificationDigestError::InvalidOutputBytes {
                actual: self.output_bytes,
                max: MAX_DIGEST_OUTPUT_BYTES,
            });
        }
        Ok(())
    }
}

/// Stable identifier fields used in validation diagnostics.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DigestIdentifierField {
    ProfileId,
    AlgorithmId,
}

/// Provider failure intentionally carries no provider-owned text.
///
/// This keeps the core contract bounded and avoids turning arbitrary provider
/// diagnostics into durable evidence. Rich diagnostics belong in a separate
/// evidence artifact if needed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct QualificationDigestProviderError;

/// Provider boundary for a later algorithm-specific qualification theorem.
///
/// Merely implementing this trait establishes no cryptographic strength.
pub trait QualificationDigestProvider {
    fn algorithm_id(&self) -> &str;

    fn digest(&self, bytes: &[u8]) -> Result<Vec<u8>, QualificationDigestProviderError>;
}

/// Typed digest bytes under one exact validated profile.
///
/// Fields are private so downstream callers cannot bypass the exact-output-length
/// constructor invariant or mutate profile identity after validation.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct QualificationDigestValueV1 {
    profile: QualificationDigestProfileV1,
    bytes: Vec<u8>,
}

impl QualificationDigestValueV1 {
    pub fn new(
        profile: QualificationDigestProfileV1,
        bytes: Vec<u8>,
    ) -> Result<Self, QualificationDigestError> {
        let value = Self { profile, bytes };
        value.validate()?;
        Ok(value)
    }

    pub fn profile(&self) -> &QualificationDigestProfileV1 {
        &self.profile
    }

    pub fn bytes(&self) -> &[u8] {
        &self.bytes
    }

    pub fn validate(&self) -> Result<(), QualificationDigestError> {
        self.profile.validate()?;
        let actual = usize_to_u64(self.bytes.len());
        let expected = u64::from(self.profile.output_bytes);
        if actual != expected {
            return Err(QualificationDigestError::DigestLengthMismatch { expected, actual });
        }
        Ok(())
    }

    /// Compare digest bytes only after proving both values remain structurally
    /// valid and profile-compatible.
    ///
    /// No constant-time or side-channel property is claimed by this provider-
    /// neutral contract.
    pub fn same_digest_value(&self, other: &Self) -> Result<bool, QualificationDigestError> {
        self.validate()?;
        other.validate()?;
        if self.profile != other.profile {
            return Err(QualificationDigestError::IncompatibleDigestProfile);
        }
        Ok(self.bytes == other.bytes)
    }

    /// Canonical PEF-compatible textual adapter: `<algorithm-id>:<lowercase-hex>`.
    ///
    /// The selected Position profile is intentionally not serialized into this
    /// PEF-style text. The typed value remains the internal authority; a caller
    /// parsing this lossy bridge must supply the exact profile out of band.
    pub fn to_algorithm_qualified_text(&self) -> String {
        let mut out =
            String::with_capacity(self.profile.algorithm_id.len() + 1 + self.bytes.len() * 2);
        out.push_str(&self.profile.algorithm_id);
        out.push(':');
        for byte in &self.bytes {
            push_lower_hex_byte(&mut out, *byte);
        }
        out
    }

    /// Parse the canonical textual adapter under an already-selected profile.
    pub fn from_algorithm_qualified_text(
        profile: QualificationDigestProfileV1,
        text: &str,
    ) -> Result<Self, QualificationDigestError> {
        profile.validate()?;
        let Some((algorithm, encoded)) = text.split_once(':') else {
            return Err(QualificationDigestError::MalformedTextDigest);
        };
        if algorithm != profile.algorithm_id {
            return Err(QualificationDigestError::TextAlgorithmMismatch);
        }

        let expected_hex_len = usize::from(profile.output_bytes)
            .checked_mul(2)
            .ok_or(QualificationDigestError::TextLengthOverflow)?;
        if encoded.len() != expected_hex_len {
            return Err(QualificationDigestError::TextDigestLengthMismatch {
                expected_hex_chars: usize_to_u64(expected_hex_len),
                actual_hex_chars: usize_to_u64(encoded.len()),
            });
        }
        if !encoded
            .bytes()
            .all(|byte| byte.is_ascii_digit() || matches!(byte, b'a'..=b'f'))
        {
            return Err(QualificationDigestError::NonCanonicalHex);
        }

        let mut bytes = Vec::with_capacity(usize::from(profile.output_bytes));
        for pair in encoded.as_bytes().as_chunks::<2>().0 {
            let high = lower_hex_value(pair[0]).ok_or(QualificationDigestError::NonCanonicalHex)?;
            let low = lower_hex_value(pair[1]).ok_or(QualificationDigestError::NonCanonicalHex)?;
            bytes.push((high << 4) | low);
        }
        Self::new(profile, bytes)
    }
}

/// Digest identity plus read-only, non-cryptographic provenance of the exact
/// admitted qualification envelope used as the provider transcript.
///
/// The exact validated admission profile is retained, not merely its textual
/// identifier, so same-label profiles with different resource limits remain
/// distinct typed identities. This provenance is metadata, not digest input.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct QualificationEnvelopeDigestV1 {
    digest: QualificationDigestValueV1,
    encoded_len: u64,
    envelope_version: u16,
    envelope_kind: QualificationEnvelopeKind,
    admission_profile: QualificationAdmissionProfileV1,
}

impl QualificationEnvelopeDigestV1 {
    pub fn digest(&self) -> &QualificationDigestValueV1 {
        &self.digest
    }

    pub fn encoded_len(&self) -> u64 {
        self.encoded_len
    }

    pub fn envelope_version(&self) -> u16 {
        self.envelope_version
    }

    pub fn envelope_kind(&self) -> QualificationEnvelopeKind {
        self.envelope_kind
    }

    pub fn admission_profile(&self) -> &QualificationAdmissionProfileV1 {
        &self.admission_profile
    }

    pub fn admission_profile_id(&self) -> &str {
        &self.admission_profile.profile_id
    }

    pub fn encoding_id(&self) -> &str {
        &self.admission_profile.encoding_id
    }

    /// Compare complete typed identities, including exact admission policy.
    ///
    /// Both digest values are revalidated. Profile incompatibility is a
    /// structural error rather than ordinary inequality. Matching results still
    /// prove neither authorship nor semantic truth.
    pub fn same_typed_identity(&self, other: &Self) -> Result<bool, QualificationDigestError> {
        self.digest.validate()?;
        other.digest.validate()?;
        if self.digest.profile != other.digest.profile {
            return Err(QualificationDigestError::IncompatibleDigestProfile);
        }
        Ok(self.digest.bytes == other.digest.bytes
            && self.encoded_len == other.encoded_len
            && self.envelope_version == other.envelope_version
            && self.envelope_kind == other.envelope_kind
            && self.admission_profile == other.admission_profile)
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum QualificationDigestError {
    UnsupportedProfileVersion {
        actual: u16,
    },
    EmptyIdentifier {
        field: DigestIdentifierField,
    },
    IdentifierTooLong {
        field: DigestIdentifierField,
        limit: u64,
        actual: u64,
    },
    NonCanonicalIdentifier {
        field: DigestIdentifierField,
    },
    InvalidOutputBytes {
        actual: u16,
        max: u16,
    },
    CanonicalProfileLengthOverflow,
    ProviderAlgorithmMismatch,
    ProviderFailure,
    ProviderOutputLengthMismatch {
        expected: u64,
        actual: u64,
    },
    DigestLengthMismatch {
        expected: u64,
        actual: u64,
    },
    IncompatibleDigestProfile,
    MalformedTextDigest,
    TextAlgorithmMismatch,
    TextLengthOverflow,
    TextDigestLengthMismatch {
        expected_hex_chars: u64,
        actual_hex_chars: u64,
    },
    NonCanonicalHex,
    Envelope(QualificationEnvelopeError),
}

impl From<QualificationEnvelopeError> for QualificationDigestError {
    fn from(value: QualificationEnvelopeError) -> Self {
        Self::Envelope(value)
    }
}

impl std::fmt::Display for QualificationDigestError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{self:?}")
    }
}

impl std::error::Error for QualificationDigestError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Envelope(error) => Some(error),
            _ => None,
        }
    }
}

/// Canonical deterministic bytes for one validated digest profile.
///
/// These bytes identify the digest *profile*. They are not prepended to the
/// envelope digest transcript; Q-ID.DigestSubject.v1 hashes the exact admitted
/// envelope bytes only.
pub fn canonical_qualification_digest_profile_preimage_v1(
    profile: &QualificationDigestProfileV1,
) -> Result<Vec<u8>, QualificationDigestError> {
    profile.validate()?;
    let mut out = Vec::with_capacity(320);
    out.extend_from_slice(PROFILE_DOMAIN_SEPARATOR);
    out.extend_from_slice(&profile.schema_version.to_be_bytes());
    push_text(&mut out, &profile.profile_id)?;
    push_text(&mut out, &profile.algorithm_id)?;
    out.extend_from_slice(&profile.output_bytes.to_be_bytes());
    Ok(out)
}

/// Compute one typed digest from the exact admitted v0.4 envelope bytes.
///
/// Successful v0.4 decoding is required to bind kind/version/provenance, but
/// the provider input is still exactly `admitted.as_bytes()`. No decoded object
/// is serialized back into the digest transcript.
pub fn digest_admitted_qualification_envelope_v1(
    codec: &QualificationEnvelopeCodecV1,
    admitted: &AdmittedQualificationBytes<'_>,
    profile: QualificationDigestProfileV1,
    provider: &impl QualificationDigestProvider,
) -> Result<QualificationEnvelopeDigestV1, QualificationDigestError> {
    profile.validate()?;
    if provider.algorithm_id() != profile.algorithm_id {
        return Err(QualificationDigestError::ProviderAlgorithmMismatch);
    }

    let decoded = codec.decode(admitted)?;
    let bytes = provider
        .digest(admitted.as_bytes())
        .map_err(|_| QualificationDigestError::ProviderFailure)?;

    let actual = usize_to_u64(bytes.len());
    let expected = u64::from(profile.output_bytes);
    if actual != expected {
        return Err(QualificationDigestError::ProviderOutputLengthMismatch { expected, actual });
    }

    Ok(QualificationEnvelopeDigestV1 {
        digest: QualificationDigestValueV1::new(profile, bytes)?,
        encoded_len: decoded.encoded_len,
        envelope_version: decoded.version,
        envelope_kind: decoded.kind,
        admission_profile: codec.profile().clone(),
    })
}

fn validate_identifier(
    value: &str,
    max_bytes: usize,
    field: DigestIdentifierField,
) -> Result<(), QualificationDigestError> {
    if value.is_empty() {
        return Err(QualificationDigestError::EmptyIdentifier { field });
    }
    if value.len() > max_bytes {
        return Err(QualificationDigestError::IdentifierTooLong {
            field,
            limit: usize_to_u64(max_bytes),
            actual: usize_to_u64(value.len()),
        });
    }
    if !value.bytes().all(|byte| {
        byte.is_ascii_lowercase()
            || byte.is_ascii_digit()
            || matches!(byte, b'-' | b'_' | b'.' | b'+')
    }) {
        return Err(QualificationDigestError::NonCanonicalIdentifier { field });
    }
    Ok(())
}

fn push_text(out: &mut Vec<u8>, value: &str) -> Result<(), QualificationDigestError> {
    let length = u32::try_from(value.len())
        .map_err(|_| QualificationDigestError::CanonicalProfileLengthOverflow)?;
    out.extend_from_slice(&length.to_be_bytes());
    out.extend_from_slice(value.as_bytes());
    Ok(())
}

fn push_lower_hex_byte(out: &mut String, byte: u8) {
    const HEX: &[u8; 16] = b"0123456789abcdef";
    out.push(char::from(HEX[usize::from(byte >> 4)]));
    out.push(char::from(HEX[usize::from(byte & 0x0f)]));
}

fn lower_hex_value(byte: u8) -> Option<u8> {
    match byte {
        b'0'..=b'9' => Some(byte - b'0'),
        b'a'..=b'f' => Some(byte - b'a' + 10),
        _ => None,
    }
}

fn usize_to_u64(value: usize) -> u64 {
    u64::try_from(value).unwrap_or(u64::MAX)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::qualification::{FacetStatus, QualificationFacet, QualificationManifest, TheoremId};
    use crate::qualification_admission::QualificationAdmissionProfileV1;
    use crate::qualification_envelope::{
        QUALIFICATION_ENVELOPE_ENCODING_V1, QualificationEnvelopePayload,
    };
    use std::cell::{Cell, RefCell};

    const TEST_ALGORITHM: &str = "test-recording-32-v1";

    struct RecordingProvider {
        algorithm: &'static str,
        output: Vec<u8>,
        observed: RefCell<Vec<u8>>,
        calls: Cell<usize>,
        fail: bool,
    }

    impl RecordingProvider {
        fn new(algorithm: &'static str, output: Vec<u8>) -> Self {
            Self {
                algorithm,
                output,
                observed: RefCell::new(Vec::new()),
                calls: Cell::new(0),
                fail: false,
            }
        }

        fn failing(algorithm: &'static str) -> Self {
            Self {
                algorithm,
                output: Vec::new(),
                observed: RefCell::new(Vec::new()),
                calls: Cell::new(0),
                fail: true,
            }
        }
    }

    impl QualificationDigestProvider for RecordingProvider {
        fn algorithm_id(&self) -> &str {
            self.algorithm
        }

        fn digest(&self, bytes: &[u8]) -> Result<Vec<u8>, QualificationDigestProviderError> {
            self.calls.set(self.calls.get() + 1);
            self.observed.replace(bytes.to_vec());
            if self.fail {
                return Err(QualificationDigestProviderError);
            }
            Ok(self.output.clone())
        }
    }

    fn digest_profile() -> QualificationDigestProfileV1 {
        QualificationDigestProfileV1::new("position-envelope-test-digest-v1", TEST_ALGORITHM, 32)
            .unwrap()
    }

    fn codec() -> QualificationEnvelopeCodecV1 {
        QualificationEnvelopeCodecV1::new(QualificationAdmissionProfileV1::default()).unwrap()
    }

    fn payload() -> QualificationEnvelopePayload {
        QualificationEnvelopePayload::Manifest(QualificationManifest {
            subject_commitment: "subject:1".into(),
            facets: vec![QualificationFacet {
                theorem_id: TheoremId::from("q0.test.v1"),
                subject_commitment: "subject:1".into(),
                status: FacetStatus::Indeterminate,
                verifier_or_profile: None,
                evidence_refs: Vec::new(),
                dependency_commitments: Vec::new(),
                diagnostics_commitment: None,
            }],
        })
    }

    #[test]
    fn provider_receives_exact_admitted_envelope_bytes() {
        let codec = codec();
        let bytes = codec.encode(&payload()).unwrap();
        let admitted = codec.admit_bytes(&bytes).unwrap();
        let provider = RecordingProvider::new(TEST_ALGORITHM, vec![0x5a; 32]);

        let digest = digest_admitted_qualification_envelope_v1(
            &codec,
            &admitted,
            digest_profile(),
            &provider,
        )
        .unwrap();

        assert_eq!(&*provider.observed.borrow(), admitted.as_bytes());
        assert_eq!(&*provider.observed.borrow(), bytes.as_slice());
        assert_eq!(digest.encoded_len(), u64::try_from(bytes.len()).unwrap());
        assert_eq!(digest.envelope_kind(), payload().kind());
        assert_eq!(digest.envelope_version(), 1);
        assert_eq!(digest.encoding_id(), QUALIFICATION_ENVELOPE_ENCODING_V1);
        assert_eq!(digest.admission_profile(), codec.profile());
    }

    #[test]
    fn provider_algorithm_mismatch_fails_before_provider_invocation() {
        let codec = codec();
        let bytes = codec.encode(&payload()).unwrap();
        let admitted = codec.admit_bytes(&bytes).unwrap();
        let provider = RecordingProvider::new("wrong-algorithm-v1", vec![0; 32]);

        assert_eq!(
            digest_admitted_qualification_envelope_v1(
                &codec,
                &admitted,
                digest_profile(),
                &provider,
            ),
            Err(QualificationDigestError::ProviderAlgorithmMismatch)
        );
        assert_eq!(provider.calls.get(), 0);
    }

    #[test]
    fn provider_output_length_is_exact() {
        let codec = codec();
        let bytes = codec.encode(&payload()).unwrap();
        let admitted = codec.admit_bytes(&bytes).unwrap();

        for output_len in [31usize, 33usize] {
            let provider = RecordingProvider::new(TEST_ALGORITHM, vec![0; output_len]);
            assert!(matches!(
                digest_admitted_qualification_envelope_v1(
                    &codec,
                    &admitted,
                    digest_profile(),
                    &provider,
                ),
                Err(QualificationDigestError::ProviderOutputLengthMismatch {
                    expected: 32,
                    actual,
                }) if actual == u64::try_from(output_len).unwrap()
            ));
        }
    }

    #[test]
    fn provider_failure_is_bounded_and_fail_closed() {
        let codec = codec();
        let bytes = codec.encode(&payload()).unwrap();
        let admitted = codec.admit_bytes(&bytes).unwrap();
        let provider = RecordingProvider::failing(TEST_ALGORITHM);

        assert!(matches!(
            digest_admitted_qualification_envelope_v1(
                &codec,
                &admitted,
                digest_profile(),
                &provider,
            ),
            Err(QualificationDigestError::ProviderFailure)
        ));
    }

    #[test]
    fn digest_profile_identity_is_not_inferred_from_output_length() {
        let left = QualificationDigestValueV1::new(digest_profile(), vec![1; 32]).unwrap();
        let right_profile = QualificationDigestProfileV1::new(
            "position-envelope-other-digest-v1",
            "test-other-32-v1",
            32,
        )
        .unwrap();
        let right = QualificationDigestValueV1::new(right_profile, vec![1; 32]).unwrap();

        assert_eq!(
            left.same_digest_value(&right),
            Err(QualificationDigestError::IncompatibleDigestProfile)
        );
    }

    #[test]
    fn textual_adapter_is_canonical_lowercase_hex() {
        let value = QualificationDigestValueV1::new(digest_profile(), (0u8..32).collect()).unwrap();
        let text = value.to_algorithm_qualified_text();
        assert!(text.starts_with(&format!("{TEST_ALGORITHM}:")));
        assert_eq!(
            QualificationDigestValueV1::from_algorithm_qualified_text(digest_profile(), &text)
                .unwrap(),
            value
        );

        let uppercase = text.to_ascii_uppercase();
        assert!(matches!(
            QualificationDigestValueV1::from_algorithm_qualified_text(digest_profile(), &uppercase,),
            Err(QualificationDigestError::TextAlgorithmMismatch)
                | Err(QualificationDigestError::NonCanonicalHex)
        ));
    }

    #[test]
    fn textual_adapter_rejects_wrong_length_and_invalid_hex() {
        let profile = digest_profile();
        let algorithm = profile.algorithm_id();
        assert!(matches!(
            QualificationDigestValueV1::from_algorithm_qualified_text(
                digest_profile(),
                &format!("{algorithm}:00"),
            ),
            Err(QualificationDigestError::TextDigestLengthMismatch { .. })
        ));

        let invalid = format!("{algorithm}:{}", "gg".repeat(32));
        assert_eq!(
            QualificationDigestValueV1::from_algorithm_qualified_text(digest_profile(), &invalid),
            Err(QualificationDigestError::NonCanonicalHex)
        );
    }

    #[test]
    fn profile_canonical_preimage_is_fixed_width_and_sensitive() {
        let left = digest_profile();
        let mut right = left.clone();
        right.output_bytes = 64;

        let left_bytes = canonical_qualification_digest_profile_preimage_v1(&left).unwrap();
        let right_bytes = canonical_qualification_digest_profile_preimage_v1(&right).unwrap();
        assert!(left_bytes.starts_with(PROFILE_DOMAIN_SEPARATOR));
        assert_ne!(left_bytes, right_bytes);
        assert!(!left_bytes.starts_with(b"{"));
    }

    #[test]
    fn invalid_internal_value_is_rejected_at_comparison_boundary() {
        let valid = QualificationDigestValueV1::new(digest_profile(), vec![1; 32]).unwrap();
        let mut malformed = valid.clone();
        malformed.bytes.pop();

        assert!(matches!(
            valid.same_digest_value(&malformed),
            Err(QualificationDigestError::DigestLengthMismatch {
                expected: 32,
                actual: 31,
            })
        ));
    }

    #[test]
    fn invalid_profile_is_rejected_before_provider_use() {
        let codec = codec();
        let bytes = codec.encode(&payload()).unwrap();
        let admitted = codec.admit_bytes(&bytes).unwrap();
        let provider = RecordingProvider::new(TEST_ALGORITHM, vec![0; 32]);
        let mut invalid = digest_profile();
        invalid.output_bytes = 0;

        assert!(matches!(
            digest_admitted_qualification_envelope_v1(&codec, &admitted, invalid, &provider,),
            Err(QualificationDigestError::InvalidOutputBytes { actual: 0, .. })
        ));
        assert_eq!(provider.calls.get(), 0);
    }

    #[test]
    fn digest_requires_a_valid_v04_envelope_before_provider_use() {
        let codec = codec();
        let bytes = vec![0u8; 64];
        let admitted = codec.admit_bytes(&bytes).unwrap();
        let provider = RecordingProvider::new(TEST_ALGORITHM, vec![0; 32]);

        assert!(matches!(
            digest_admitted_qualification_envelope_v1(
                &codec,
                &admitted,
                digest_profile(),
                &provider,
            ),
            Err(QualificationDigestError::Envelope(_))
        ));
        assert_eq!(provider.calls.get(), 0);
    }

    #[test]
    fn typed_identity_includes_exact_admission_profile() {
        let codec = codec();
        let bytes = codec.encode(&payload()).unwrap();
        let admitted = codec.admit_bytes(&bytes).unwrap();
        let provider = RecordingProvider::new(TEST_ALGORITHM, vec![7; 32]);
        let left = digest_admitted_qualification_envelope_v1(
            &codec,
            &admitted,
            digest_profile(),
            &provider,
        )
        .unwrap();
        let mut right = left.clone();
        right.admission_profile.max_encoded_bytes += 1;

        assert_eq!(left.admission_profile_id(), right.admission_profile_id());
        assert_eq!(left.encoding_id(), right.encoding_id());
        assert_eq!(left.same_typed_identity(&right), Ok(false));
        assert_eq!(left.digest().same_digest_value(right.digest()), Ok(true));
    }
}
