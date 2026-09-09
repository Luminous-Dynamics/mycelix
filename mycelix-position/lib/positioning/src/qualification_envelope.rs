// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Version-first canonical wire envelope for qualification semantic objects.
//!
//! The public decoder accepts only [`AdmittedQualificationBytes`] and re-admits
//! the exact bytes under its own profile before reading the v1 binary grammar.
//! Decoding remains weaker than semantic qualification.

use crate::qualification::{
    FacetStatus, QualificationFacet, QualificationManifest, QualificationRequirementProfile,
    TheoremDefinition, TheoremId,
};
use crate::qualification_admission::{
    AdmittedQualificationBytes, QualificationAdmissionError, QualificationAdmissionProfileV1,
    admit_qualification_bytes,
};

/// Exact encoding identifier consumed by the v1 codec.
pub const QUALIFICATION_ENVELOPE_ENCODING_V1: &str = "mycelix-position-qualification-envelope-v1";

/// Exact envelope version.
pub const QUALIFICATION_ENVELOPE_VERSION_V1: u16 = 1;

const ENVELOPE_MAGIC: &[u8] = b"MYCELIX-POSITION-QUALIFICATION\0";
const HEADER_BYTES: usize = ENVELOPE_MAGIC.len() + 2 + 1 + 4;

const KIND_THEOREM_REGISTRY: u8 = 1;
const KIND_MANIFEST: u8 = 2;
const KIND_REQUIREMENT_PROFILE: u8 = 3;

const MIN_THEOREM_DEFINITION_BYTES: u64 = 8; // theorem-id length + prerequisite count
const MIN_THEOREM_ID_BYTES: u64 = 4; // empty length-prefixed string
const MIN_FACET_BYTES: u64 = 19;
const MIN_REFERENCE_BYTES: u64 = 4;
const MIN_STATUS_BYTES: u64 = 1;

/// Closed semantic payload vocabulary for v1.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum QualificationEnvelopeKind {
    TheoremRegistry,
    Manifest,
    RequirementProfile,
}

impl QualificationEnvelopeKind {
    fn tag(self) -> u8 {
        match self {
            Self::TheoremRegistry => KIND_THEOREM_REGISTRY,
            Self::Manifest => KIND_MANIFEST,
            Self::RequirementProfile => KIND_REQUIREMENT_PROFILE,
        }
    }

    fn from_tag(tag: u8) -> Result<Self, QualificationEnvelopeError> {
        match tag {
            KIND_THEOREM_REGISTRY => Ok(Self::TheoremRegistry),
            KIND_MANIFEST => Ok(Self::Manifest),
            KIND_REQUIREMENT_PROFILE => Ok(Self::RequirementProfile),
            actual => Err(QualificationEnvelopeError::UnknownPayloadKind { actual }),
        }
    }
}

/// One decoded or encodable qualification payload.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum QualificationEnvelopePayload {
    TheoremRegistry(Vec<TheoremDefinition>),
    Manifest(QualificationManifest),
    RequirementProfile(QualificationRequirementProfile),
}

impl QualificationEnvelopePayload {
    pub fn kind(&self) -> QualificationEnvelopeKind {
        match self {
            Self::TheoremRegistry(_) => QualificationEnvelopeKind::TheoremRegistry,
            Self::Manifest(_) => QualificationEnvelopeKind::Manifest,
            Self::RequirementProfile(_) => QualificationEnvelopeKind::RequirementProfile,
        }
    }
}

/// Non-authoritative provenance retained from one successful decode.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DecodedQualificationEnvelope {
    pub version: u16,
    pub kind: QualificationEnvelopeKind,
    pub encoded_len: u64,
    pub admission_profile_id: String,
    pub encoding_id: String,
    pub payload: QualificationEnvelopePayload,
}

/// Resource dimension checked before allocation or aggregate growth.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum QualificationEnvelopeLimit {
    Theorems,
    PrerequisitesPerTheorem,
    TotalPrerequisiteEdges,
    Facets,
    RequiredTheorems,
    AcceptedStatuses,
    EvidenceRefsPerFacet,
    DependencyCommitmentsPerFacet,
    TheoremIdBytes,
    ProfileIdBytes,
    SubjectCommitmentBytes,
    VerifierProfileBytes,
    EvidenceRefBytes,
    DependencyCommitmentBytes,
    DiagnosticsCommitmentBytes,
    RegistryIdentifierBytes,
    ManifestStringBytes,
    ManifestEvidenceRefs,
    ManifestDependencyCommitments,
}

/// Fail-closed wire/codec errors. These are distinct from theorem semantics.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum QualificationEnvelopeError {
    Admission(QualificationAdmissionError),
    UnsupportedEncodingProfile,
    AdmissionProfileMismatch,
    Truncated {
        context: &'static str,
    },
    BadMagic,
    UnsupportedVersion {
        actual: u16,
    },
    UnknownPayloadKind {
        actual: u8,
    },
    PayloadLengthExceedsRemaining {
        declared: u64,
        remaining: u64,
    },
    TrailingEnvelopeBytes {
        declared_payload: u64,
        remaining: u64,
    },
    TrailingPayloadBytes {
        remaining: u64,
    },
    InvalidUtf8 {
        field: &'static str,
    },
    InvalidOptionMarker {
        field: &'static str,
        actual: u8,
    },
    UnknownFacetStatus {
        actual: u8,
    },
    LimitExceeded {
        resource: QualificationEnvelopeLimit,
        limit: u64,
        actual: u64,
    },
    CollectionCannotFit {
        resource: QualificationEnvelopeLimit,
        declared: u64,
        minimum_bytes: u64,
        remaining_bytes: u64,
    },
    AggregateArithmeticOverflow {
        resource: QualificationEnvelopeLimit,
    },
    CanonicalLengthOverflow {
        field: &'static str,
    },
    PlatformLengthOverflow {
        field: &'static str,
    },
}

impl From<QualificationAdmissionError> for QualificationEnvelopeError {
    fn from(value: QualificationAdmissionError) -> Self {
        Self::Admission(value)
    }
}

impl std::fmt::Display for QualificationEnvelopeError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{self:?}")
    }
}

impl std::error::Error for QualificationEnvelopeError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Admission(error) => Some(error),
            _ => None,
        }
    }
}

/// V1 codec bound to one exact validated admission profile.
#[derive(Debug, Clone)]
pub struct QualificationEnvelopeCodecV1 {
    profile: QualificationAdmissionProfileV1,
}

impl QualificationEnvelopeCodecV1 {
    pub fn new(
        profile: QualificationAdmissionProfileV1,
    ) -> Result<Self, QualificationEnvelopeError> {
        profile.validate()?;
        if profile.encoding_id != QUALIFICATION_ENVELOPE_ENCODING_V1 {
            return Err(QualificationEnvelopeError::UnsupportedEncodingProfile);
        }
        Ok(Self { profile })
    }

    pub fn profile(&self) -> &QualificationAdmissionProfileV1 {
        &self.profile
    }

    pub fn admit_bytes<'a>(
        &self,
        bytes: &'a [u8],
    ) -> Result<AdmittedQualificationBytes<'a>, QualificationEnvelopeError> {
        Ok(admit_qualification_bytes(
            &self.profile,
            QUALIFICATION_ENVELOPE_ENCODING_V1,
            bytes,
        )?)
    }

    /// Encode without claiming semantic validity or authentication.
    pub fn encode(
        &self,
        payload: &QualificationEnvelopePayload,
    ) -> Result<Vec<u8>, QualificationEnvelopeError> {
        let mut body = Vec::new();
        match payload {
            QualificationEnvelopePayload::TheoremRegistry(definitions) => {
                encode_registry(&self.profile, definitions, &mut body)?;
            }
            QualificationEnvelopePayload::Manifest(manifest) => {
                encode_manifest(&self.profile, manifest, &mut body)?;
            }
            QualificationEnvelopePayload::RequirementProfile(requirement) => {
                encode_requirement(&self.profile, requirement, &mut body)?;
            }
        }

        let payload_len = to_u32(body.len(), "payload")?;
        let mut out = Vec::with_capacity(HEADER_BYTES + body.len());
        out.extend_from_slice(ENVELOPE_MAGIC);
        push_u16(&mut out, QUALIFICATION_ENVELOPE_VERSION_V1);
        out.push(payload.kind().tag());
        push_u32(&mut out, payload_len);
        out.extend_from_slice(&body);
        self.admit_bytes(&out)?;
        Ok(out)
    }

    /// Decode only an admitted token.
    ///
    /// The exact token bytes are re-admitted under this codec's private
    /// profile before any envelope field is read. This prevents a token made
    /// under another same-named-but-weaker profile from weakening parse limits.
    pub fn decode(
        &self,
        admitted: &AdmittedQualificationBytes<'_>,
    ) -> Result<DecodedQualificationEnvelope, QualificationEnvelopeError> {
        if admitted.profile_id() != self.profile.profile_id
            || admitted.encoding_id() != self.profile.encoding_id
        {
            return Err(QualificationEnvelopeError::AdmissionProfileMismatch);
        }

        let rebound = self.admit_bytes(admitted.as_bytes())?;
        self.decode_rebound(&rebound)
    }

    fn decode_rebound(
        &self,
        admitted: &AdmittedQualificationBytes<'_>,
    ) -> Result<DecodedQualificationEnvelope, QualificationEnvelopeError> {
        let bytes = admitted.as_bytes();
        let mut cursor = Cursor::new(bytes);

        if cursor.take(ENVELOPE_MAGIC.len(), "magic")? != ENVELOPE_MAGIC {
            return Err(QualificationEnvelopeError::BadMagic);
        }

        let version = cursor.read_u16("version")?;
        if version != QUALIFICATION_ENVELOPE_VERSION_V1 {
            return Err(QualificationEnvelopeError::UnsupportedVersion { actual: version });
        }

        let kind = QualificationEnvelopeKind::from_tag(cursor.read_u8("payload kind")?)?;
        let declared = cursor.read_u32("payload length")?;
        let declared_usize = u32_to_usize(declared, "payload length")?;
        let remaining = cursor.remaining();

        if declared_usize > remaining {
            return Err(QualificationEnvelopeError::PayloadLengthExceedsRemaining {
                declared: u64::from(declared),
                remaining: usize_to_u64(remaining),
            });
        }
        if declared_usize < remaining {
            return Err(QualificationEnvelopeError::TrailingEnvelopeBytes {
                declared_payload: u64::from(declared),
                remaining: usize_to_u64(remaining),
            });
        }

        let mut payload_cursor = Cursor::new(cursor.take(declared_usize, "payload")?);
        let payload = match kind {
            QualificationEnvelopeKind::TheoremRegistry => {
                QualificationEnvelopePayload::TheoremRegistry(decode_registry(
                    &self.profile,
                    &mut payload_cursor,
                )?)
            }
            QualificationEnvelopeKind::Manifest => QualificationEnvelopePayload::Manifest(
                decode_manifest(&self.profile, &mut payload_cursor)?,
            ),
            QualificationEnvelopeKind::RequirementProfile => {
                QualificationEnvelopePayload::RequirementProfile(decode_requirement(
                    &self.profile,
                    &mut payload_cursor,
                )?)
            }
        };

        if payload_cursor.remaining() != 0 {
            return Err(QualificationEnvelopeError::TrailingPayloadBytes {
                remaining: usize_to_u64(payload_cursor.remaining()),
            });
        }

        Ok(DecodedQualificationEnvelope {
            version,
            kind,
            encoded_len: usize_to_u64(bytes.len()),
            admission_profile_id: self.profile.profile_id.clone(),
            encoding_id: self.profile.encoding_id.clone(),
            payload,
        })
    }
}

fn encode_registry(
    profile: &QualificationAdmissionProfileV1,
    definitions: &[TheoremDefinition],
    out: &mut Vec<u8>,
) -> Result<(), QualificationEnvelopeError> {
    write_count(
        out,
        definitions.len(),
        "theorem count",
        QualificationEnvelopeLimit::Theorems,
        profile.max_theorems,
    )?;

    let mut identifier_bytes = 0u64;
    let mut total_edges = 0u64;

    for definition in definitions {
        write_text(
            out,
            definition.id.as_str(),
            "theorem id",
            QualificationEnvelopeLimit::TheoremIdBytes,
            profile.max_theorem_id_bytes,
        )?;
        add_bytes(
            &mut identifier_bytes,
            definition.id.as_str().len(),
            QualificationEnvelopeLimit::RegistryIdentifierBytes,
            profile.max_total_registry_identifier_bytes,
        )?;

        write_count(
            out,
            definition.prerequisites.len(),
            "prerequisite count",
            QualificationEnvelopeLimit::PrerequisitesPerTheorem,
            profile.max_prerequisites_per_theorem,
        )?;
        total_edges = add_count(
            total_edges,
            definition.prerequisites.len(),
            QualificationEnvelopeLimit::TotalPrerequisiteEdges,
        )?;
        ensure_limit(
            QualificationEnvelopeLimit::TotalPrerequisiteEdges,
            total_edges,
            profile.max_total_prerequisite_edges,
        )?;

        for prerequisite in &definition.prerequisites {
            write_text(
                out,
                prerequisite.as_str(),
                "prerequisite theorem id",
                QualificationEnvelopeLimit::TheoremIdBytes,
                profile.max_theorem_id_bytes,
            )?;
            add_bytes(
                &mut identifier_bytes,
                prerequisite.as_str().len(),
                QualificationEnvelopeLimit::RegistryIdentifierBytes,
                profile.max_total_registry_identifier_bytes,
            )?;
        }
    }

    Ok(())
}

fn decode_registry(
    profile: &QualificationAdmissionProfileV1,
    cursor: &mut Cursor<'_>,
) -> Result<Vec<TheoremDefinition>, QualificationEnvelopeError> {
    let count = cursor.read_count_fit(
        "theorem count",
        QualificationEnvelopeLimit::Theorems,
        profile.max_theorems,
        MIN_THEOREM_DEFINITION_BYTES,
    )?;
    let mut definitions = Vec::with_capacity(count);
    let mut identifier_bytes = 0u64;
    let mut total_edges = 0u64;

    for _ in 0..count {
        let theorem = cursor.read_text(
            "theorem id",
            QualificationEnvelopeLimit::TheoremIdBytes,
            profile.max_theorem_id_bytes,
        )?;
        add_bytes(
            &mut identifier_bytes,
            theorem.len(),
            QualificationEnvelopeLimit::RegistryIdentifierBytes,
            profile.max_total_registry_identifier_bytes,
        )?;

        let prerequisite_count = cursor.read_count_fit(
            "prerequisite count",
            QualificationEnvelopeLimit::PrerequisitesPerTheorem,
            profile.max_prerequisites_per_theorem,
            MIN_THEOREM_ID_BYTES,
        )?;
        total_edges = add_count(
            total_edges,
            prerequisite_count,
            QualificationEnvelopeLimit::TotalPrerequisiteEdges,
        )?;
        ensure_limit(
            QualificationEnvelopeLimit::TotalPrerequisiteEdges,
            total_edges,
            profile.max_total_prerequisite_edges,
        )?;

        let mut prerequisites = Vec::with_capacity(prerequisite_count);
        for _ in 0..prerequisite_count {
            let prerequisite = cursor.read_text(
                "prerequisite theorem id",
                QualificationEnvelopeLimit::TheoremIdBytes,
                profile.max_theorem_id_bytes,
            )?;
            add_bytes(
                &mut identifier_bytes,
                prerequisite.len(),
                QualificationEnvelopeLimit::RegistryIdentifierBytes,
                profile.max_total_registry_identifier_bytes,
            )?;
            prerequisites.push(TheoremId::from(prerequisite));
        }
        definitions.push(TheoremDefinition::new(theorem, prerequisites));
    }

    Ok(definitions)
}

fn encode_manifest(
    profile: &QualificationAdmissionProfileV1,
    manifest: &QualificationManifest,
    out: &mut Vec<u8>,
) -> Result<(), QualificationEnvelopeError> {
    let mut string_bytes = 0u64;
    let mut evidence_refs = 0u64;
    let mut dependency_commitments = 0u64;

    write_text(
        out,
        &manifest.subject_commitment,
        "manifest subject commitment",
        QualificationEnvelopeLimit::SubjectCommitmentBytes,
        profile.max_subject_commitment_bytes,
    )?;
    add_bytes(
        &mut string_bytes,
        manifest.subject_commitment.len(),
        QualificationEnvelopeLimit::ManifestStringBytes,
        profile.max_total_manifest_string_bytes,
    )?;

    write_count(
        out,
        manifest.facets.len(),
        "facet count",
        QualificationEnvelopeLimit::Facets,
        profile.max_facets,
    )?;

    for facet in &manifest.facets {
        write_text(
            out,
            facet.theorem_id.as_str(),
            "facet theorem id",
            QualificationEnvelopeLimit::TheoremIdBytes,
            profile.max_theorem_id_bytes,
        )?;
        add_bytes(
            &mut string_bytes,
            facet.theorem_id.as_str().len(),
            QualificationEnvelopeLimit::ManifestStringBytes,
            profile.max_total_manifest_string_bytes,
        )?;

        write_text(
            out,
            &facet.subject_commitment,
            "facet subject commitment",
            QualificationEnvelopeLimit::SubjectCommitmentBytes,
            profile.max_subject_commitment_bytes,
        )?;
        add_bytes(
            &mut string_bytes,
            facet.subject_commitment.len(),
            QualificationEnvelopeLimit::ManifestStringBytes,
            profile.max_total_manifest_string_bytes,
        )?;

        out.push(status_tag(facet.status));
        write_optional_text(
            out,
            facet.verifier_or_profile.as_deref(),
            "verifier/profile",
            QualificationEnvelopeLimit::VerifierProfileBytes,
            profile.max_verifier_profile_bytes,
            &mut string_bytes,
            profile.max_total_manifest_string_bytes,
        )?;

        write_count(
            out,
            facet.evidence_refs.len(),
            "evidence reference count",
            QualificationEnvelopeLimit::EvidenceRefsPerFacet,
            profile.max_evidence_refs_per_facet,
        )?;
        evidence_refs = add_count(
            evidence_refs,
            facet.evidence_refs.len(),
            QualificationEnvelopeLimit::ManifestEvidenceRefs,
        )?;
        ensure_limit(
            QualificationEnvelopeLimit::ManifestEvidenceRefs,
            evidence_refs,
            profile.max_total_manifest_evidence_refs,
        )?;

        for reference in &facet.evidence_refs {
            write_text(
                out,
                reference,
                "evidence reference",
                QualificationEnvelopeLimit::EvidenceRefBytes,
                profile.max_evidence_ref_bytes,
            )?;
            add_bytes(
                &mut string_bytes,
                reference.len(),
                QualificationEnvelopeLimit::ManifestStringBytes,
                profile.max_total_manifest_string_bytes,
            )?;
        }

        write_count(
            out,
            facet.dependency_commitments.len(),
            "dependency commitment count",
            QualificationEnvelopeLimit::DependencyCommitmentsPerFacet,
            profile.max_dependency_commitments_per_facet,
        )?;
        dependency_commitments = add_count(
            dependency_commitments,
            facet.dependency_commitments.len(),
            QualificationEnvelopeLimit::ManifestDependencyCommitments,
        )?;
        ensure_limit(
            QualificationEnvelopeLimit::ManifestDependencyCommitments,
            dependency_commitments,
            profile.max_total_manifest_dependency_commitments,
        )?;

        for commitment in &facet.dependency_commitments {
            write_text(
                out,
                commitment,
                "dependency commitment",
                QualificationEnvelopeLimit::DependencyCommitmentBytes,
                profile.max_dependency_commitment_bytes,
            )?;
            add_bytes(
                &mut string_bytes,
                commitment.len(),
                QualificationEnvelopeLimit::ManifestStringBytes,
                profile.max_total_manifest_string_bytes,
            )?;
        }

        write_optional_text(
            out,
            facet.diagnostics_commitment.as_deref(),
            "diagnostics commitment",
            QualificationEnvelopeLimit::DiagnosticsCommitmentBytes,
            profile.max_diagnostics_commitment_bytes,
            &mut string_bytes,
            profile.max_total_manifest_string_bytes,
        )?;
    }

    Ok(())
}

fn decode_manifest(
    profile: &QualificationAdmissionProfileV1,
    cursor: &mut Cursor<'_>,
) -> Result<QualificationManifest, QualificationEnvelopeError> {
    let mut string_bytes = 0u64;
    let mut evidence_refs = 0u64;
    let mut dependency_commitments = 0u64;

    let subject_commitment = cursor.read_text(
        "manifest subject commitment",
        QualificationEnvelopeLimit::SubjectCommitmentBytes,
        profile.max_subject_commitment_bytes,
    )?;
    add_bytes(
        &mut string_bytes,
        subject_commitment.len(),
        QualificationEnvelopeLimit::ManifestStringBytes,
        profile.max_total_manifest_string_bytes,
    )?;

    let facet_count = cursor.read_count_fit(
        "facet count",
        QualificationEnvelopeLimit::Facets,
        profile.max_facets,
        MIN_FACET_BYTES,
    )?;
    let mut facets = Vec::with_capacity(facet_count);

    for _ in 0..facet_count {
        let theorem_id = cursor.read_text(
            "facet theorem id",
            QualificationEnvelopeLimit::TheoremIdBytes,
            profile.max_theorem_id_bytes,
        )?;
        add_bytes(
            &mut string_bytes,
            theorem_id.len(),
            QualificationEnvelopeLimit::ManifestStringBytes,
            profile.max_total_manifest_string_bytes,
        )?;

        let facet_subject = cursor.read_text(
            "facet subject commitment",
            QualificationEnvelopeLimit::SubjectCommitmentBytes,
            profile.max_subject_commitment_bytes,
        )?;
        add_bytes(
            &mut string_bytes,
            facet_subject.len(),
            QualificationEnvelopeLimit::ManifestStringBytes,
            profile.max_total_manifest_string_bytes,
        )?;

        let status = decode_status(cursor.read_u8("facet status")?)?;
        let verifier_or_profile = cursor.read_optional_text(
            "verifier/profile",
            QualificationEnvelopeLimit::VerifierProfileBytes,
            profile.max_verifier_profile_bytes,
            &mut string_bytes,
            profile.max_total_manifest_string_bytes,
        )?;

        let evidence_count = cursor.read_count_fit(
            "evidence reference count",
            QualificationEnvelopeLimit::EvidenceRefsPerFacet,
            profile.max_evidence_refs_per_facet,
            MIN_REFERENCE_BYTES,
        )?;
        evidence_refs = add_count(
            evidence_refs,
            evidence_count,
            QualificationEnvelopeLimit::ManifestEvidenceRefs,
        )?;
        ensure_limit(
            QualificationEnvelopeLimit::ManifestEvidenceRefs,
            evidence_refs,
            profile.max_total_manifest_evidence_refs,
        )?;
        let mut facet_evidence = Vec::with_capacity(evidence_count);
        for _ in 0..evidence_count {
            let reference = cursor.read_text(
                "evidence reference",
                QualificationEnvelopeLimit::EvidenceRefBytes,
                profile.max_evidence_ref_bytes,
            )?;
            add_bytes(
                &mut string_bytes,
                reference.len(),
                QualificationEnvelopeLimit::ManifestStringBytes,
                profile.max_total_manifest_string_bytes,
            )?;
            facet_evidence.push(reference);
        }

        let dependency_count = cursor.read_count_fit(
            "dependency commitment count",
            QualificationEnvelopeLimit::DependencyCommitmentsPerFacet,
            profile.max_dependency_commitments_per_facet,
            MIN_REFERENCE_BYTES,
        )?;
        dependency_commitments = add_count(
            dependency_commitments,
            dependency_count,
            QualificationEnvelopeLimit::ManifestDependencyCommitments,
        )?;
        ensure_limit(
            QualificationEnvelopeLimit::ManifestDependencyCommitments,
            dependency_commitments,
            profile.max_total_manifest_dependency_commitments,
        )?;
        let mut facet_dependencies = Vec::with_capacity(dependency_count);
        for _ in 0..dependency_count {
            let commitment = cursor.read_text(
                "dependency commitment",
                QualificationEnvelopeLimit::DependencyCommitmentBytes,
                profile.max_dependency_commitment_bytes,
            )?;
            add_bytes(
                &mut string_bytes,
                commitment.len(),
                QualificationEnvelopeLimit::ManifestStringBytes,
                profile.max_total_manifest_string_bytes,
            )?;
            facet_dependencies.push(commitment);
        }

        let diagnostics_commitment = cursor.read_optional_text(
            "diagnostics commitment",
            QualificationEnvelopeLimit::DiagnosticsCommitmentBytes,
            profile.max_diagnostics_commitment_bytes,
            &mut string_bytes,
            profile.max_total_manifest_string_bytes,
        )?;

        facets.push(QualificationFacet {
            theorem_id: TheoremId::from(theorem_id),
            subject_commitment: facet_subject,
            status,
            verifier_or_profile,
            evidence_refs: facet_evidence,
            dependency_commitments: facet_dependencies,
            diagnostics_commitment,
        });
    }

    Ok(QualificationManifest {
        subject_commitment,
        facets,
    })
}

fn encode_requirement(
    profile: &QualificationAdmissionProfileV1,
    requirement: &QualificationRequirementProfile,
    out: &mut Vec<u8>,
) -> Result<(), QualificationEnvelopeError> {
    write_text(
        out,
        &requirement.profile_id,
        "requirement profile id",
        QualificationEnvelopeLimit::ProfileIdBytes,
        profile.max_profile_id_bytes,
    )?;

    write_count(
        out,
        requirement.required_theorems.len(),
        "required theorem count",
        QualificationEnvelopeLimit::RequiredTheorems,
        profile.max_required_theorems,
    )?;
    for theorem in &requirement.required_theorems {
        write_text(
            out,
            theorem.as_str(),
            "required theorem id",
            QualificationEnvelopeLimit::TheoremIdBytes,
            profile.max_theorem_id_bytes,
        )?;
    }

    write_count(
        out,
        requirement.accepted_statuses.len(),
        "accepted status count",
        QualificationEnvelopeLimit::AcceptedStatuses,
        profile.max_accepted_statuses,
    )?;
    for status in &requirement.accepted_statuses {
        out.push(status_tag(*status));
    }

    Ok(())
}

fn decode_requirement(
    profile: &QualificationAdmissionProfileV1,
    cursor: &mut Cursor<'_>,
) -> Result<QualificationRequirementProfile, QualificationEnvelopeError> {
    let profile_id = cursor.read_text(
        "requirement profile id",
        QualificationEnvelopeLimit::ProfileIdBytes,
        profile.max_profile_id_bytes,
    )?;

    let theorem_count = cursor.read_count_fit(
        "required theorem count",
        QualificationEnvelopeLimit::RequiredTheorems,
        profile.max_required_theorems,
        MIN_THEOREM_ID_BYTES,
    )?;
    let mut required_theorems = Vec::with_capacity(theorem_count);
    for _ in 0..theorem_count {
        required_theorems.push(TheoremId::from(cursor.read_text(
            "required theorem id",
            QualificationEnvelopeLimit::TheoremIdBytes,
            profile.max_theorem_id_bytes,
        )?));
    }

    let status_count = cursor.read_count_fit(
        "accepted status count",
        QualificationEnvelopeLimit::AcceptedStatuses,
        profile.max_accepted_statuses,
        MIN_STATUS_BYTES,
    )?;
    let mut accepted_statuses = Vec::with_capacity(status_count);
    for _ in 0..status_count {
        accepted_statuses.push(decode_status(cursor.read_u8("accepted status")?)?);
    }

    Ok(QualificationRequirementProfile {
        profile_id,
        required_theorems,
        accepted_statuses,
    })
}

fn status_tag(status: FacetStatus) -> u8 {
    match status {
        FacetStatus::Established => 0,
        FacetStatus::NotEstablished => 1,
        FacetStatus::Failed => 2,
        FacetStatus::Indeterminate => 3,
        FacetStatus::NotApplicable => 4,
        FacetStatus::Expired => 5,
        FacetStatus::Superseded => 6,
    }
}

fn decode_status(tag: u8) -> Result<FacetStatus, QualificationEnvelopeError> {
    match tag {
        0 => Ok(FacetStatus::Established),
        1 => Ok(FacetStatus::NotEstablished),
        2 => Ok(FacetStatus::Failed),
        3 => Ok(FacetStatus::Indeterminate),
        4 => Ok(FacetStatus::NotApplicable),
        5 => Ok(FacetStatus::Expired),
        6 => Ok(FacetStatus::Superseded),
        actual => Err(QualificationEnvelopeError::UnknownFacetStatus { actual }),
    }
}

fn write_optional_text(
    out: &mut Vec<u8>,
    value: Option<&str>,
    field: &'static str,
    resource: QualificationEnvelopeLimit,
    limit: u64,
    aggregate: &mut u64,
    aggregate_limit: u64,
) -> Result<(), QualificationEnvelopeError> {
    match value {
        None => out.push(0),
        Some(value) => {
            out.push(1);
            write_text(out, value, field, resource, limit)?;
            add_bytes(
                aggregate,
                value.len(),
                QualificationEnvelopeLimit::ManifestStringBytes,
                aggregate_limit,
            )?;
        }
    }
    Ok(())
}

fn write_text(
    out: &mut Vec<u8>,
    value: &str,
    field: &'static str,
    resource: QualificationEnvelopeLimit,
    limit: u64,
) -> Result<(), QualificationEnvelopeError> {
    ensure_limit(resource, usize_to_u64(value.len()), limit)?;
    push_u32(out, to_u32(value.len(), field)?);
    out.extend_from_slice(value.as_bytes());
    Ok(())
}

fn write_count(
    out: &mut Vec<u8>,
    count: usize,
    field: &'static str,
    resource: QualificationEnvelopeLimit,
    limit: u64,
) -> Result<(), QualificationEnvelopeError> {
    ensure_limit(resource, usize_to_u64(count), limit)?;
    push_u32(out, to_u32(count, field)?);
    Ok(())
}

fn ensure_limit(
    resource: QualificationEnvelopeLimit,
    actual: u64,
    limit: u64,
) -> Result<(), QualificationEnvelopeError> {
    if actual > limit {
        return Err(QualificationEnvelopeError::LimitExceeded {
            resource,
            limit,
            actual,
        });
    }
    Ok(())
}

fn add_count(
    current: u64,
    count: usize,
    resource: QualificationEnvelopeLimit,
) -> Result<u64, QualificationEnvelopeError> {
    current
        .checked_add(usize_to_u64(count))
        .ok_or(QualificationEnvelopeError::AggregateArithmeticOverflow { resource })
}

fn add_bytes(
    aggregate: &mut u64,
    len: usize,
    resource: QualificationEnvelopeLimit,
    limit: u64,
) -> Result<(), QualificationEnvelopeError> {
    *aggregate = aggregate
        .checked_add(usize_to_u64(len))
        .ok_or(QualificationEnvelopeError::AggregateArithmeticOverflow { resource })?;
    ensure_limit(resource, *aggregate, limit)
}

fn to_u32(value: usize, field: &'static str) -> Result<u32, QualificationEnvelopeError> {
    u32::try_from(value).map_err(|_| QualificationEnvelopeError::CanonicalLengthOverflow { field })
}

fn u32_to_usize(value: u32, field: &'static str) -> Result<usize, QualificationEnvelopeError> {
    usize::try_from(value).map_err(|_| QualificationEnvelopeError::PlatformLengthOverflow { field })
}

fn usize_to_u64(value: usize) -> u64 {
    u64::try_from(value).unwrap_or(u64::MAX)
}

fn push_u16(out: &mut Vec<u8>, value: u16) {
    out.extend_from_slice(&value.to_be_bytes());
}

fn push_u32(out: &mut Vec<u8>, value: u32) {
    out.extend_from_slice(&value.to_be_bytes());
}

struct Cursor<'a> {
    bytes: &'a [u8],
    offset: usize,
}

impl<'a> Cursor<'a> {
    fn new(bytes: &'a [u8]) -> Self {
        Self { bytes, offset: 0 }
    }

    fn remaining(&self) -> usize {
        self.bytes.len().saturating_sub(self.offset)
    }

    fn take(
        &mut self,
        length: usize,
        context: &'static str,
    ) -> Result<&'a [u8], QualificationEnvelopeError> {
        let end = self
            .offset
            .checked_add(length)
            .ok_or(QualificationEnvelopeError::Truncated { context })?;
        if end > self.bytes.len() {
            return Err(QualificationEnvelopeError::Truncated { context });
        }
        let slice = &self.bytes[self.offset..end];
        self.offset = end;
        Ok(slice)
    }

    fn read_u8(&mut self, context: &'static str) -> Result<u8, QualificationEnvelopeError> {
        Ok(self.take(1, context)?[0])
    }

    fn read_u16(&mut self, context: &'static str) -> Result<u16, QualificationEnvelopeError> {
        let bytes: [u8; 2] = self
            .take(2, context)?
            .try_into()
            .map_err(|_| QualificationEnvelopeError::Truncated { context })?;
        Ok(u16::from_be_bytes(bytes))
    }

    fn read_u32(&mut self, context: &'static str) -> Result<u32, QualificationEnvelopeError> {
        let bytes: [u8; 4] = self
            .take(4, context)?
            .try_into()
            .map_err(|_| QualificationEnvelopeError::Truncated { context })?;
        Ok(u32::from_be_bytes(bytes))
    }

    /// Read a collection count and prove the remaining admitted bytes can
    /// physically contain at least the minimum v1 wire representation of every
    /// declared item before any caller reserves vector capacity.
    fn read_count_fit(
        &mut self,
        context: &'static str,
        resource: QualificationEnvelopeLimit,
        limit: u64,
        minimum_item_bytes: u64,
    ) -> Result<usize, QualificationEnvelopeError> {
        let count = self.read_u32(context)?;
        let declared = u64::from(count);
        ensure_limit(resource, declared, limit)?;

        let minimum_bytes = declared
            .checked_mul(minimum_item_bytes)
            .ok_or(QualificationEnvelopeError::AggregateArithmeticOverflow { resource })?;
        let remaining_bytes = usize_to_u64(self.remaining());
        if minimum_bytes > remaining_bytes {
            return Err(QualificationEnvelopeError::CollectionCannotFit {
                resource,
                declared,
                minimum_bytes,
                remaining_bytes,
            });
        }

        u32_to_usize(count, context)
    }

    fn read_text(
        &mut self,
        field: &'static str,
        resource: QualificationEnvelopeLimit,
        limit: u64,
    ) -> Result<String, QualificationEnvelopeError> {
        let length = self.read_u32(field)?;
        ensure_limit(resource, u64::from(length), limit)?;
        let bytes = self.take(u32_to_usize(length, field)?, field)?;
        let value = std::str::from_utf8(bytes)
            .map_err(|_| QualificationEnvelopeError::InvalidUtf8 { field })?;
        Ok(value.to_owned())
    }

    fn read_optional_text(
        &mut self,
        field: &'static str,
        resource: QualificationEnvelopeLimit,
        limit: u64,
        aggregate: &mut u64,
        aggregate_limit: u64,
    ) -> Result<Option<String>, QualificationEnvelopeError> {
        match self.read_u8(field)? {
            0 => Ok(None),
            1 => {
                let value = self.read_text(field, resource, limit)?;
                add_bytes(
                    aggregate,
                    value.len(),
                    QualificationEnvelopeLimit::ManifestStringBytes,
                    aggregate_limit,
                )?;
                Ok(Some(value))
            }
            actual => Err(QualificationEnvelopeError::InvalidOptionMarker { field, actual }),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn theorem(value: &str, prerequisites: &[&str]) -> TheoremDefinition {
        TheoremDefinition::new(
            value,
            prerequisites
                .iter()
                .map(|value| TheoremId::from(*value))
                .collect(),
        )
    }

    fn registry_payload() -> QualificationEnvelopePayload {
        QualificationEnvelopePayload::TheoremRegistry(vec![
            theorem("q0.serialization-safety.v1", &[]),
            theorem("q2.frame-identity.v1", &["q0.serialization-safety.v1"]),
        ])
    }

    fn manifest_payload() -> QualificationEnvelopePayload {
        QualificationEnvelopePayload::Manifest(QualificationManifest {
            subject_commitment: "sha256:subject".into(),
            facets: vec![QualificationFacet {
                theorem_id: TheoremId::from("q0.serialization-safety.v1"),
                subject_commitment: "sha256:subject".into(),
                status: FacetStatus::Indeterminate,
                verifier_or_profile: Some("verifier-v1".into()),
                evidence_refs: vec!["evidence:1".into()],
                dependency_commitments: vec!["dependency:1".into()],
                diagnostics_commitment: Some("diagnostics:1".into()),
            }],
        })
    }

    fn requirement_payload() -> QualificationEnvelopePayload {
        QualificationEnvelopePayload::RequirementProfile(QualificationRequirementProfile {
            profile_id: "consumer-v1".into(),
            required_theorems: vec![TheoremId::from("q0.serialization-safety.v1")],
            accepted_statuses: vec![FacetStatus::Established, FacetStatus::Indeterminate],
        })
    }

    fn profile() -> QualificationAdmissionProfileV1 {
        QualificationAdmissionProfileV1::default()
    }

    fn codec() -> QualificationEnvelopeCodecV1 {
        QualificationEnvelopeCodecV1::new(profile()).unwrap()
    }

    fn roundtrip(payload: QualificationEnvelopePayload) {
        let codec = codec();
        let bytes = codec.encode(&payload).unwrap();
        let admitted = codec.admit_bytes(&bytes).unwrap();
        let decoded = codec.decode(&admitted).unwrap();
        assert_eq!(decoded.version, QUALIFICATION_ENVELOPE_VERSION_V1);
        assert_eq!(decoded.kind, payload.kind());
        assert_eq!(decoded.payload, payload);
        assert_eq!(decoded.encoded_len, usize_to_u64(bytes.len()));
        assert_eq!(decoded.admission_profile_id, codec.profile().profile_id);
        assert_eq!(decoded.encoding_id, QUALIFICATION_ENVELOPE_ENCODING_V1);
    }

    #[test]
    fn all_payload_kinds_roundtrip() {
        roundtrip(registry_payload());
        roundtrip(manifest_payload());
        roundtrip(requirement_payload());
    }

    #[test]
    fn same_semantic_payload_encodes_deterministically() {
        let codec = codec();
        let payload = manifest_payload();
        assert_eq!(
            codec.encode(&payload).unwrap(),
            codec.encode(&payload).unwrap()
        );
    }

    #[test]
    fn future_version_fails_before_payload_kind_interpretation() {
        let codec = codec();
        let mut bytes = codec.encode(&registry_payload()).unwrap();
        let version_offset = ENVELOPE_MAGIC.len();
        bytes[version_offset..version_offset + 2].copy_from_slice(&2u16.to_be_bytes());
        bytes[version_offset + 2] = 0xff;
        let admitted = codec.admit_bytes(&bytes).unwrap();
        assert!(matches!(
            codec.decode(&admitted),
            Err(QualificationEnvelopeError::UnsupportedVersion { actual: 2 })
        ));
    }

    #[test]
    fn malformed_header_and_payload_lengths_fail_closed() {
        let codec = codec();
        let bytes = codec.encode(&registry_payload()).unwrap();

        for length in 0..HEADER_BYTES {
            let admitted = codec.admit_bytes(&bytes[..length]).unwrap();
            assert!(codec.decode(&admitted).is_err(), "prefix length {length}");
        }

        let mut bad_magic = bytes.clone();
        bad_magic[0] ^= 0xff;
        let admitted = codec.admit_bytes(&bad_magic).unwrap();
        assert!(matches!(
            codec.decode(&admitted),
            Err(QualificationEnvelopeError::BadMagic)
        ));

        let payload_length_offset = ENVELOPE_MAGIC.len() + 3;
        let declared = u32::from_be_bytes(
            bytes[payload_length_offset..payload_length_offset + 4]
                .try_into()
                .unwrap(),
        );

        let mut long = bytes.clone();
        long[payload_length_offset..payload_length_offset + 4]
            .copy_from_slice(&(declared + 1).to_be_bytes());
        let admitted = codec.admit_bytes(&long).unwrap();
        assert!(matches!(
            codec.decode(&admitted),
            Err(QualificationEnvelopeError::PayloadLengthExceedsRemaining { .. })
        ));

        let mut short = bytes.clone();
        short[payload_length_offset..payload_length_offset + 4]
            .copy_from_slice(&(declared - 1).to_be_bytes());
        let admitted = codec.admit_bytes(&short).unwrap();
        assert!(matches!(
            codec.decode(&admitted),
            Err(QualificationEnvelopeError::TrailingEnvelopeBytes { .. })
        ));

        let mut trailing = bytes;
        trailing.push(0);
        let admitted = codec.admit_bytes(&trailing).unwrap();
        assert!(matches!(
            codec.decode(&admitted),
            Err(QualificationEnvelopeError::TrailingEnvelopeBytes { .. })
        ));
    }

    #[test]
    fn unknown_payload_and_status_tags_are_rejected() {
        let codec = codec();

        let mut unknown_kind = codec.encode(&registry_payload()).unwrap();
        unknown_kind[ENVELOPE_MAGIC.len() + 2] = 0xff;
        let admitted = codec.admit_bytes(&unknown_kind).unwrap();
        assert!(matches!(
            codec.decode(&admitted),
            Err(QualificationEnvelopeError::UnknownPayloadKind { actual: 0xff })
        ));

        assert!(matches!(
            decode_status(7),
            Err(QualificationEnvelopeError::UnknownFacetStatus { actual: 7 })
        ));
    }

    #[test]
    fn all_seven_status_tags_roundtrip() {
        let statuses = [
            FacetStatus::Established,
            FacetStatus::NotEstablished,
            FacetStatus::Failed,
            FacetStatus::Indeterminate,
            FacetStatus::NotApplicable,
            FacetStatus::Expired,
            FacetStatus::Superseded,
        ];
        for status in statuses {
            assert_eq!(decode_status(status_tag(status)).unwrap(), status);
        }
    }

    #[test]
    fn collection_must_fit_remaining_wire_before_allocation() {
        let codec = codec();
        let mut payload = Vec::new();
        push_u32(&mut payload, 2);
        let bytes = raw_envelope(KIND_THEOREM_REGISTRY, &payload);
        let admitted = codec.admit_bytes(&bytes).unwrap();

        assert!(matches!(
            codec.decode(&admitted),
            Err(QualificationEnvelopeError::CollectionCannotFit {
                resource: QualificationEnvelopeLimit::Theorems,
                declared: 2,
                minimum_bytes: 16,
                remaining_bytes: 0,
            })
        ));
    }

    #[test]
    fn nested_collection_must_fit_before_capacity_reservation() {
        let codec = codec();
        let mut payload = Vec::new();
        push_u32(&mut payload, 1);
        push_test_text(&mut payload, "q0");
        push_u32(&mut payload, 2);
        let bytes = raw_envelope(KIND_THEOREM_REGISTRY, &payload);
        let admitted = codec.admit_bytes(&bytes).unwrap();

        assert!(matches!(
            codec.decode(&admitted),
            Err(QualificationEnvelopeError::CollectionCannotFit {
                resource: QualificationEnvelopeLimit::PrerequisitesPerTheorem,
                declared: 2,
                minimum_bytes: 8,
                remaining_bytes: 0,
            })
        ));
    }

    #[test]
    fn profile_limit_and_string_body_are_checked_before_allocation() {
        let strict = QualificationAdmissionProfileV1 {
            max_theorems: 1,
            max_theorem_id_bytes: 3,
            ..profile()
        };
        let codec = QualificationEnvelopeCodecV1::new(strict).unwrap();

        let mut too_many = Vec::new();
        push_u32(&mut too_many, 2);
        let bytes = raw_envelope(KIND_THEOREM_REGISTRY, &too_many);
        let admitted = codec.admit_bytes(&bytes).unwrap();
        assert!(matches!(
            codec.decode(&admitted),
            Err(QualificationEnvelopeError::LimitExceeded {
                resource: QualificationEnvelopeLimit::Theorems,
                limit: 1,
                actual: 2,
            })
        ));

        let mut oversized_text = Vec::new();
        push_u32(&mut oversized_text, 1);
        push_u32(&mut oversized_text, 4);
        oversized_text.extend_from_slice(&[0; 4]);
        let bytes = raw_envelope(KIND_THEOREM_REGISTRY, &oversized_text);
        let admitted = codec.admit_bytes(&bytes).unwrap();
        assert!(matches!(
            codec.decode(&admitted),
            Err(QualificationEnvelopeError::LimitExceeded {
                resource: QualificationEnvelopeLimit::TheoremIdBytes,
                limit: 3,
                actual: 4,
            })
        ));
    }

    #[test]
    fn invalid_utf8_and_option_marker_are_rejected() {
        let codec = codec();

        let mut registry = Vec::new();
        push_u32(&mut registry, 1);
        push_u32(&mut registry, 1);
        registry.push(0xff);
        push_u32(&mut registry, 0);
        let bytes = raw_envelope(KIND_THEOREM_REGISTRY, &registry);
        let admitted = codec.admit_bytes(&bytes).unwrap();
        assert!(matches!(
            codec.decode(&admitted),
            Err(QualificationEnvelopeError::InvalidUtf8 { .. })
        ));

        let mut manifest = Vec::new();
        push_test_text(&mut manifest, "subject");
        push_u32(&mut manifest, 1);
        push_test_text(&mut manifest, "q0");
        push_test_text(&mut manifest, "subject");
        manifest.push(status_tag(FacetStatus::Established));
        manifest.push(2);
        manifest.extend_from_slice(&[0; 9]);
        let bytes = raw_envelope(KIND_MANIFEST, &manifest);
        let admitted = codec.admit_bytes(&bytes).unwrap();
        assert!(matches!(
            codec.decode(&admitted),
            Err(QualificationEnvelopeError::InvalidOptionMarker {
                field: "verifier/profile",
                actual: 2,
            })
        ));
    }

    #[test]
    fn payload_kind_substitution_is_not_a_second_decoder_api() {
        let codec = codec();
        let mut bytes = codec.encode(&manifest_payload()).unwrap();
        bytes[ENVELOPE_MAGIC.len() + 2] = KIND_THEOREM_REGISTRY;
        let admitted = codec.admit_bytes(&bytes).unwrap();
        assert!(codec.decode(&admitted).is_err());
    }

    #[test]
    fn same_named_weaker_profile_cannot_bypass_codec_limits() {
        let strict_profile = QualificationAdmissionProfileV1 {
            max_theorems: 1,
            ..profile()
        };
        let codec = QualificationEnvelopeCodecV1::new(strict_profile.clone()).unwrap();

        let weak_profile = QualificationAdmissionProfileV1 {
            max_theorems: 10,
            ..strict_profile
        };
        let weak_codec = QualificationEnvelopeCodecV1::new(weak_profile.clone()).unwrap();
        let bytes = weak_codec
            .encode(&QualificationEnvelopePayload::TheoremRegistry(vec![
                theorem("a", &[]),
                theorem("b", &[]),
            ]))
            .unwrap();
        let weak_token =
            admit_qualification_bytes(&weak_profile, QUALIFICATION_ENVELOPE_ENCODING_V1, &bytes)
                .unwrap();

        assert!(matches!(
            codec.decode(&weak_token),
            Err(QualificationEnvelopeError::LimitExceeded {
                resource: QualificationEnvelopeLimit::Theorems,
                limit: 1,
                actual: 2,
            })
        ));
    }

    fn raw_envelope(kind: u8, payload: &[u8]) -> Vec<u8> {
        let mut out = Vec::new();
        out.extend_from_slice(ENVELOPE_MAGIC);
        push_u16(&mut out, QUALIFICATION_ENVELOPE_VERSION_V1);
        out.push(kind);
        push_u32(&mut out, u32::try_from(payload.len()).unwrap());
        out.extend_from_slice(payload);
        out
    }

    fn push_test_text(out: &mut Vec<u8>, value: &str) {
        push_u32(out, u32::try_from(value.len()).unwrap());
        out.extend_from_slice(value.as_bytes());
    }
}
