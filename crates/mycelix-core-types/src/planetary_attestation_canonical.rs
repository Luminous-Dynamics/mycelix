// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Canonical byte preimage for PEF evidence attestations.
//!
//! Signatures are only interoperable when independent implementations agree on
//! the exact bytes being hashed/signed. Generic JSON serialization is not a
//! sufficient protocol contract: object ordering, whitespace, and serializer
//! behavior can vary even when the semantic payload is the same.
//!
//! This module defines a dependency-free v1 binary preimage with:
//!
//! - an explicit domain separator;
//! - fixed variant tags;
//! - big-endian integers;
//! - length-prefixed byte strings;
//! - explicit option markers; and
//! - lexical sorting for fields whose validation gives them set semantics
//!   (`scopes`, supporting evidence, authority evidence).
//!
//! The function produces bytes only. Hash algorithm selection remains outside
//! this module and is already carried by the algorithm-qualified
//! `SignedEvidenceAttestation::payload_digest`.

use crate::{
    AttestationScope, AttestationStance, AttestationSubject, EvidenceAttestation,
    ExternalEvidenceRef, PlanetaryAttestationError, SignedEvidenceAttestation,
};
use std::fmt;

/// Stable identifier placed in `SignedEvidenceAttestation::payload_encoding`
/// when the payload digest is computed from [`canonical_attestation_preimage_v1`].
pub const MYCELIX_ATTESTATION_PREIMAGE_V1: &str = "mycelix-pef-attestation-preimage-v1";

const DOMAIN_SEPARATOR: &[u8] = b"MYCELIX-PEF-ATTESTATION\0V1\0";

/// Produce the canonical v1 preimage for one validated attestation payload.
pub fn canonical_attestation_preimage_v1(
    payload: &EvidenceAttestation,
) -> Result<Vec<u8>, PlanetaryAttestationError> {
    payload.validate()?;

    let mut out = Vec::with_capacity(512);
    out.extend_from_slice(DOMAIN_SEPARATOR);

    push_u16(&mut out, payload.schema_version);
    push_text(&mut out, &payload.id);
    push_subject(&mut out, &payload.subject);
    push_text(&mut out, &payload.attestor.did);
    push_text(&mut out, &payload.attestor.verification_method);

    // Scopes have set semantics in validation, so signing must not depend on
    // the caller's insertion order.
    let mut scopes: Vec<Vec<u8>> = payload.scopes.iter().map(encode_scope).collect();
    scopes.sort();
    push_items(&mut out, &scopes);

    push_u8(&mut out, stance_tag(payload.stance));
    push_i64(&mut out, payload.issued_at);
    push_option_i64(&mut out, payload.valid_until);
    push_option_text(&mut out, payload.statement_digest.as_deref());

    push_evidence_set(&mut out, &payload.supporting_evidence);
    push_evidence_set(&mut out, &payload.authority_evidence);

    Ok(out)
}

/// Validate a signed envelope and return the canonical preimage only when it
/// explicitly declares this v1 encoding.
///
/// This still does not verify `payload_digest` or the detached signature. A
/// cryptographic consumer hashes the returned bytes with the algorithm named in
/// `payload_digest`, compares that digest, resolves the verification method, and
/// verifies the signature.
pub fn canonical_preimage_for_signed_v1(
    signed: &SignedEvidenceAttestation,
) -> Result<Vec<u8>, AttestationCanonicalizationError> {
    signed
        .validate()
        .map_err(AttestationCanonicalizationError::InvalidAttestation)?;
    if signed.payload_encoding != MYCELIX_ATTESTATION_PREIMAGE_V1 {
        return Err(AttestationCanonicalizationError::UnsupportedPayloadEncoding(
            signed.payload_encoding.clone(),
        ));
    }
    canonical_attestation_preimage_v1(&signed.payload)
        .map_err(AttestationCanonicalizationError::InvalidAttestation)
}

#[derive(Debug)]
pub enum AttestationCanonicalizationError {
    InvalidAttestation(PlanetaryAttestationError),
    UnsupportedPayloadEncoding(String),
}

impl fmt::Display for AttestationCanonicalizationError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidAttestation(error) => write!(f, "invalid attestation payload: {error}"),
            Self::UnsupportedPayloadEncoding(encoding) => write!(
                f,
                "unsupported attestation payload encoding '{encoding}'"
            ),
        }
    }
}

impl std::error::Error for AttestationCanonicalizationError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::InvalidAttestation(error) => Some(error),
            Self::UnsupportedPayloadEncoding(_) => None,
        }
    }
}

fn push_subject(out: &mut Vec<u8>, subject: &AttestationSubject) {
    match subject {
        AttestationSubject::Observation {
            observation_id,
            observation_digest,
        } => {
            push_u8(out, 0);
            push_text(out, observation_id);
            push_text(out, observation_digest);
        }
        AttestationSubject::Lineage {
            output_observation_id,
            lineage_digest,
        } => {
            push_u8(out, 1);
            push_text(out, output_observation_id);
            push_text(out, lineage_digest);
        }
        AttestationSubject::Product {
            observation_id,
            observation_digest,
            lineage_digest,
        } => {
            push_u8(out, 2);
            push_text(out, observation_id);
            push_text(out, observation_digest);
            push_text(out, lineage_digest);
        }
        AttestationSubject::ExternalEvidence(reference) => {
            push_u8(out, 3);
            push_external_evidence(out, reference);
        }
    }
}

fn encode_scope(scope: &AttestationScope) -> Vec<u8> {
    let mut out = Vec::new();
    match scope {
        AttestationScope::SourceAuthenticity => push_u8(&mut out, 0),
        AttestationScope::MeasurementIntegrity => push_u8(&mut out, 1),
        AttestationScope::Methodology => push_u8(&mut out, 2),
        AttestationScope::Reproducibility => push_u8(&mut out, 3),
        AttestationScope::ScientificReview => push_u8(&mut out, 4),
        AttestationScope::OperationalVerification => push_u8(&mut out, 5),
        AttestationScope::RegulatoryCompliance => push_u8(&mut out, 6),
        AttestationScope::CommunityWitness => push_u8(&mut out, 7),
        AttestationScope::Custom(value) => {
            push_u8(&mut out, 8);
            push_text(&mut out, value);
        }
    }
    out
}

fn stance_tag(stance: AttestationStance) -> u8 {
    match stance {
        AttestationStance::Affirms => 0,
        AttestationStance::Qualifies => 1,
        AttestationStance::Disputes => 2,
        AttestationStance::Inconclusive => 3,
    }
}

fn push_evidence_set(out: &mut Vec<u8>, evidence: &[ExternalEvidenceRef]) {
    let mut encoded: Vec<Vec<u8>> = evidence
        .iter()
        .map(|reference| {
            let mut bytes = Vec::new();
            push_external_evidence(&mut bytes, reference);
            bytes
        })
        .collect();
    encoded.sort();
    push_items(out, &encoded);
}

fn push_external_evidence(out: &mut Vec<u8>, reference: &ExternalEvidenceRef) {
    push_text(out, &reference.source_system);
    push_text(out, &reference.resource_id);
    push_option_text(out, reference.content_digest.as_deref());
    push_option_i64(out, reference.retrieved_at);
    push_option_text(out, reference.license.as_deref());
}

fn push_items(out: &mut Vec<u8>, items: &[Vec<u8>]) {
    push_u32(out, items.len() as u32);
    for item in items {
        push_bytes(out, item);
    }
}

fn push_option_text(out: &mut Vec<u8>, value: Option<&str>) {
    match value {
        Some(value) => {
            push_u8(out, 1);
            push_text(out, value);
        }
        None => push_u8(out, 0),
    }
}

fn push_option_i64(out: &mut Vec<u8>, value: Option<i64>) {
    match value {
        Some(value) => {
            push_u8(out, 1);
            push_i64(out, value);
        }
        None => push_u8(out, 0),
    }
}

fn push_text(out: &mut Vec<u8>, value: &str) {
    push_bytes(out, value.as_bytes());
}

fn push_bytes(out: &mut Vec<u8>, value: &[u8]) {
    // All caller-visible fields are bounded far below u32::MAX by attestation
    // validation. Keeping a fixed u32 length prefix makes the protocol compact
    // and unambiguous without platform-sized integers.
    push_u32(out, value.len() as u32);
    out.extend_from_slice(value);
}

fn push_u8(out: &mut Vec<u8>, value: u8) {
    out.push(value);
}

fn push_u16(out: &mut Vec<u8>, value: u16) {
    out.extend_from_slice(&value.to_be_bytes());
}

fn push_u32(out: &mut Vec<u8>, value: u32) {
    out.extend_from_slice(&value.to_be_bytes());
}

fn push_i64(out: &mut Vec<u8>, value: i64) {
    out.extend_from_slice(&value.to_be_bytes());
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        AttestationStance, AttestationSubject, AttestorIdentity, DetachedAttestationSignature,
        EvidenceAttestation, SignedEvidenceAttestation,
    };

    fn evidence(system: &str, resource: &str) -> ExternalEvidenceRef {
        ExternalEvidenceRef {
            source_system: system.into(),
            resource_id: resource.into(),
            content_digest: Some(format!("sha256:{system}-{resource}")),
            retrieved_at: Some(1_788_825_600),
            license: Some("CC-BY-4.0".into()),
        }
    }

    fn payload() -> EvidenceAttestation {
        let mut value = EvidenceAttestation::new(
            "att:canonical:1",
            AttestationSubject::Product {
                observation_id: "obs:heat:jhb:1".into(),
                observation_digest: "sha256:observation".into(),
                lineage_digest: "sha256:lineage".into(),
            },
            AttestorIdentity {
                did: "did:key:z6Mktest".into(),
                verification_method: "did:key:z6Mktest#key-1".into(),
            },
            vec![
                AttestationScope::ScientificReview,
                AttestationScope::Methodology,
            ],
            AttestationStance::Qualifies,
            1_788_825_600,
            Some(1_791_417_600),
        )
        .unwrap();
        value.statement_digest = Some("sha256:statement".into());
        value.supporting_evidence = vec![
            evidence("source-b", "2"),
            evidence("source-a", "1"),
        ];
        value.authority_evidence = vec![
            evidence("credential-b", "2"),
            evidence("credential-a", "1"),
        ];
        value
    }

    #[test]
    fn canonical_bytes_are_stable_for_set_reordering() {
        let left = payload();
        let mut right = left.clone();
        right.scopes.reverse();
        right.supporting_evidence.reverse();
        right.authority_evidence.reverse();

        assert_eq!(
            canonical_attestation_preimage_v1(&left).unwrap(),
            canonical_attestation_preimage_v1(&right).unwrap()
        );
    }

    #[test]
    fn semantic_changes_change_the_preimage() {
        let left = payload();
        let mut right = left.clone();
        right.stance = AttestationStance::Disputes;

        assert_ne!(
            canonical_attestation_preimage_v1(&left).unwrap(),
            canonical_attestation_preimage_v1(&right).unwrap()
        );
    }

    #[test]
    fn authority_and_supporting_evidence_are_domain_separated_by_field_position() {
        let left = payload();
        let mut right = left.clone();
        std::mem::swap(
            &mut right.supporting_evidence,
            &mut right.authority_evidence,
        );

        assert_ne!(
            canonical_attestation_preimage_v1(&left).unwrap(),
            canonical_attestation_preimage_v1(&right).unwrap()
        );
    }

    #[test]
    fn subject_variants_do_not_alias() {
        let mut observation = payload();
        observation.subject = AttestationSubject::Observation {
            observation_id: "same".into(),
            observation_digest: "sha256:same".into(),
        };

        let mut lineage = observation.clone();
        lineage.subject = AttestationSubject::Lineage {
            output_observation_id: "same".into(),
            lineage_digest: "sha256:same".into(),
        };

        assert_ne!(
            canonical_attestation_preimage_v1(&observation).unwrap(),
            canonical_attestation_preimage_v1(&lineage).unwrap()
        );
    }

    #[test]
    fn signed_helper_requires_exact_encoding_identifier() {
        let signed = SignedEvidenceAttestation {
            payload: payload(),
            payload_encoding: MYCELIX_ATTESTATION_PREIMAGE_V1.into(),
            payload_digest: "sha256:payload".into(),
            signature: DetachedAttestationSignature {
                algorithm: "ed25519".into(),
                bytes: vec![1; 64],
            },
        };
        assert_eq!(
            canonical_preimage_for_signed_v1(&signed).unwrap(),
            canonical_attestation_preimage_v1(&signed.payload).unwrap()
        );

        let mut wrong = signed;
        wrong.payload_encoding = "generic-json".into();
        assert!(matches!(
            canonical_preimage_for_signed_v1(&wrong),
            Err(AttestationCanonicalizationError::UnsupportedPayloadEncoding(_))
        ));
    }

    #[test]
    fn domain_separator_prefix_is_fixed() {
        let bytes = canonical_attestation_preimage_v1(&payload()).unwrap();
        assert!(bytes.starts_with(DOMAIN_SEPARATOR));
    }
}
