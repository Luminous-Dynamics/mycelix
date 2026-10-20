// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Canonical claims profile for Mycelix continuity-witness domain credentials.
//!
//! This crate does not issue or verify W3C credentials itself. Mycelix Identity already owns
//! those responsibilities. It defines the exact application claims that an issuer places inside
//! `credentialSubject` and that a relying verifier must validate after cryptographic VC/status
//! verification.
//!
//! The key distinction is between a human-facing `domain_id` and the canonical
//! `control_domain_id` used for independence counting. Multiple councils, hosts, subsidiaries,
//! or notary labels controlled by one administrative root MUST share one control-domain ID.

#![deny(unsafe_code)]

use std::collections::BTreeSet;

use serde::{Deserialize, Serialize};
use serde_json::{Value, json};
use thiserror::Error;

/// Credential-schema registry identifier for this profile.
pub const WITNESS_DOMAIN_CREDENTIAL_SCHEMA_ID: &str =
    "mycelix:schema:governance:continuity-witness-domain:v1";
/// VC type required in addition to `VerifiableCredential`.
pub const WITNESS_DOMAIN_CREDENTIAL_TYPE: &str = "ContinuityWitnessDomainCredential";
/// Purpose binding expected by Symthaea's continuity relying profile.
pub const WITNESS_DOMAIN_PURPOSE: &str =
    "symthaea.episodic-continuity.external-witness.v1";

pub const MAX_ID_BYTES: usize = 512;
pub const MAX_DETAIL_BYTES: usize = 256;
pub const MAX_EVIDENCE_REFS: usize = 32;
pub const MAX_EVIDENCE_REF_BYTES: usize = 2048;

/// Semantic kind of the witness domain.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum WitnessDomainKind {
    GovernanceCouncil,
    Organization,
    JurisdictionAuthority,
    InfrastructureAdmin,
    ExternalNotary,
    HardwareSecurityDomain,
    Other,
}

/// Evidence basis by which the witness is bound to the asserted domain.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum WitnessBindingBasis {
    CouncilMembership,
    OrganizationMembership,
    JurisdictionAuthority,
    InfrastructureControl,
    NotaryAccreditation,
    HardwareAttestation,
    Other,
}

/// Exact application claims carried in a continuity-witness-domain VC.
///
/// The credential subject DID is supplied by the outer W3C credential and is intentionally not
/// duplicated here. A relying verifier must bind that outer subject DID to the Holochain witness
/// agent key before accepting these claims.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ContinuityWitnessDomainClaimsV1 {
    /// Must equal [`WITNESS_DOMAIN_PURPOSE`].
    pub purpose: String,
    /// Human/application-facing namespaced domain identifier.
    pub domain_id: String,
    /// Canonical common administrative/control root used for independence counting.
    pub control_domain_id: String,
    pub domain_kind: WitnessDomainKind,
    pub binding_basis: WitnessBindingBasis,
    /// Optional DID for the authority controlling the domain. Hardware-only roots may omit it.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub control_authority_did: Option<String>,
    /// Required only when `domain_kind == other`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub domain_kind_detail: Option<String>,
    /// Required only when `binding_basis == other`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub binding_basis_detail: Option<String>,
    /// Immutable/auditable references supporting the binding claim.
    pub evidence_refs: Vec<String>,
}

impl ContinuityWitnessDomainClaimsV1 {
    pub fn validate(&self) -> Result<(), WitnessDomainProfileError> {
        if self.purpose != WITNESS_DOMAIN_PURPOSE {
            return Err(WitnessDomainProfileError::PurposeMismatch);
        }
        validate_namespaced_id("domain_id", &self.domain_id)?;
        validate_namespaced_id("control_domain_id", &self.control_domain_id)?;

        if let Some(did) = &self.control_authority_did {
            validate_text("control_authority_did", did, MAX_ID_BYTES)?;
            if !did.starts_with("did:") {
                return Err(WitnessDomainProfileError::InvalidControlAuthorityDid);
            }
        }

        match (self.domain_kind, self.domain_kind_detail.as_deref()) {
            (WitnessDomainKind::Other, Some(detail)) => {
                validate_text("domain_kind_detail", detail, MAX_DETAIL_BYTES)?;
            }
            (WitnessDomainKind::Other, None) => {
                return Err(WitnessDomainProfileError::MissingDomainKindDetail);
            }
            (_, Some(_)) => return Err(WitnessDomainProfileError::UnexpectedDomainKindDetail),
            (_, None) => {}
        }

        match (self.binding_basis, self.binding_basis_detail.as_deref()) {
            (WitnessBindingBasis::Other, Some(detail)) => {
                validate_text("binding_basis_detail", detail, MAX_DETAIL_BYTES)?;
            }
            (WitnessBindingBasis::Other, None) => {
                return Err(WitnessDomainProfileError::MissingBindingBasisDetail);
            }
            (_, Some(_)) => return Err(WitnessDomainProfileError::UnexpectedBindingBasisDetail),
            (_, None) => {}
        }

        if self.evidence_refs.is_empty() {
            return Err(WitnessDomainProfileError::MissingEvidence);
        }
        if self.evidence_refs.len() > MAX_EVIDENCE_REFS {
            return Err(WitnessDomainProfileError::TooManyEvidenceRefs {
                actual: self.evidence_refs.len(),
                max: MAX_EVIDENCE_REFS,
            });
        }
        let mut seen = BTreeSet::new();
        for value in &self.evidence_refs {
            validate_text("evidence_ref", value, MAX_EVIDENCE_REF_BYTES)?;
            if !seen.insert(value) {
                return Err(WitnessDomainProfileError::DuplicateEvidenceRef);
            }
        }
        Ok(())
    }
}

/// Parse generic VC claims and enforce this profile's semantic invariants.
pub fn parse_and_validate_claims(
    claims: &Value,
) -> Result<ContinuityWitnessDomainClaimsV1, WitnessDomainProfileError> {
    let parsed: ContinuityWitnessDomainClaimsV1 = serde_json::from_value(claims.clone())
        .map_err(|error| WitnessDomainProfileError::ClaimsDecode(error.to_string()))?;
    parsed.validate()?;
    Ok(parsed)
}

/// Required claim names for Mycelix `CredentialSchema.required_fields`.
pub fn required_claim_fields() -> &'static [&'static str] {
    &[
        "purpose",
        "domain_id",
        "control_domain_id",
        "domain_kind",
        "binding_basis",
        "evidence_refs",
    ]
}

/// Optional claim names for Mycelix `CredentialSchema.optional_fields`.
pub fn optional_claim_fields() -> &'static [&'static str] {
    &[
        "control_authority_did",
        "domain_kind_detail",
        "binding_basis_detail",
    ]
}

/// Credential types expected by Mycelix VC issuance.
pub fn credential_types() -> &'static [&'static str] {
    &["VerifiableCredential", WITNESS_DOMAIN_CREDENTIAL_TYPE]
}

/// Canonical JSON Schema document for registration in `credential_schema`.
///
/// Runtime VC issuance should use strict schema lookup, while resolvers should additionally call
/// [`parse_and_validate_claims`] because generic field-presence validation cannot express every
/// cross-field invariant in this profile.
pub fn credential_subject_json_schema() -> Value {
    json!({
        "$schema": "https://json-schema.org/draft/2020-12/schema",
        "$id": WITNESS_DOMAIN_CREDENTIAL_SCHEMA_ID,
        "title": "Mycelix Continuity Witness Domain Credential v1",
        "type": "object",
        "additionalProperties": false,
        "required": required_claim_fields(),
        "properties": {
            "purpose": {
                "type": "string",
                "const": WITNESS_DOMAIN_PURPOSE
            },
            "domain_id": {
                "type": "string",
                "minLength": 3,
                "maxLength": MAX_ID_BYTES
            },
            "control_domain_id": {
                "type": "string",
                "minLength": 3,
                "maxLength": MAX_ID_BYTES
            },
            "domain_kind": {
                "type": "string",
                "enum": [
                    "governance_council",
                    "organization",
                    "jurisdiction_authority",
                    "infrastructure_admin",
                    "external_notary",
                    "hardware_security_domain",
                    "other"
                ]
            },
            "binding_basis": {
                "type": "string",
                "enum": [
                    "council_membership",
                    "organization_membership",
                    "jurisdiction_authority",
                    "infrastructure_control",
                    "notary_accreditation",
                    "hardware_attestation",
                    "other"
                ]
            },
            "control_authority_did": {
                "type": "string",
                "minLength": 5,
                "maxLength": MAX_ID_BYTES,
                "pattern": "^did:"
            },
            "domain_kind_detail": {
                "type": "string",
                "minLength": 1,
                "maxLength": MAX_DETAIL_BYTES
            },
            "binding_basis_detail": {
                "type": "string",
                "minLength": 1,
                "maxLength": MAX_DETAIL_BYTES
            },
            "evidence_refs": {
                "type": "array",
                "minItems": 1,
                "maxItems": MAX_EVIDENCE_REFS,
                "uniqueItems": true,
                "items": {
                    "type": "string",
                    "minLength": 1,
                    "maxLength": MAX_EVIDENCE_REF_BYTES
                }
            }
        }
    })
}

fn validate_namespaced_id(
    field: &'static str,
    value: &str,
) -> Result<(), WitnessDomainProfileError> {
    validate_text(field, value, MAX_ID_BYTES)?;
    if !value.contains(':') || value.starts_with(':') || value.ends_with(':') {
        return Err(WitnessDomainProfileError::UnnamespacedId { field });
    }
    if value.chars().any(char::is_whitespace) {
        return Err(WitnessDomainProfileError::InvalidText { field });
    }
    Ok(())
}

fn validate_text(
    field: &'static str,
    value: &str,
    max: usize,
) -> Result<(), WitnessDomainProfileError> {
    if value.trim().is_empty()
        || value != value.trim()
        || value.len() > max
        || value.chars().any(char::is_control)
    {
        Err(WitnessDomainProfileError::InvalidText { field })
    } else {
        Ok(())
    }
}

#[derive(Debug, Error, PartialEq, Eq)]
pub enum WitnessDomainProfileError {
    #[error("witness-domain purpose does not match the continuity profile")]
    PurposeMismatch,
    #[error("invalid witness-domain text field `{field}`")]
    InvalidText { field: &'static str },
    #[error("witness-domain identifier `{field}` must be namespaced")]
    UnnamespacedId { field: &'static str },
    #[error("control_authority_did must use DID syntax")]
    InvalidControlAuthorityDid,
    #[error("domain_kind=other requires domain_kind_detail")]
    MissingDomainKindDetail,
    #[error("domain_kind_detail is only allowed for domain_kind=other")]
    UnexpectedDomainKindDetail,
    #[error("binding_basis=other requires binding_basis_detail")]
    MissingBindingBasisDetail,
    #[error("binding_basis_detail is only allowed for binding_basis=other")]
    UnexpectedBindingBasisDetail,
    #[error("witness-domain claim requires at least one evidence reference")]
    MissingEvidence,
    #[error("too many evidence references: actual={actual}, max={max}")]
    TooManyEvidenceRefs { actual: usize, max: usize },
    #[error("duplicate evidence reference")]
    DuplicateEvidenceRef,
    #[error("witness-domain claims could not be decoded: {0}")]
    ClaimsDecode(String),
}

#[cfg(test)]
mod tests {
    use super::*;

    fn valid_claims() -> ContinuityWitnessDomainClaimsV1 {
        ContinuityWitnessDomainClaimsV1 {
            purpose: WITNESS_DOMAIN_PURPOSE.into(),
            domain_id: "mycelix:council:technical".into(),
            control_domain_id: "mycelix:operator:alpha".into(),
            domain_kind: WitnessDomainKind::GovernanceCouncil,
            binding_basis: WitnessBindingBasis::CouncilMembership,
            control_authority_did: Some("did:mycelix:authority-alpha".into()),
            domain_kind_detail: None,
            binding_basis_detail: None,
            evidence_refs: vec!["holochain:action:membership-alpha".into()],
        }
    }

    #[test]
    fn valid_profile_passes() {
        assert!(valid_claims().validate().is_ok());
    }

    #[test]
    fn purpose_substitution_fails() {
        let mut value = valid_claims();
        value.purpose = "some.other.profile".into();
        assert_eq!(value.validate(), Err(WitnessDomainProfileError::PurposeMismatch));
    }

    #[test]
    fn control_root_must_be_namespaced() {
        let mut value = valid_claims();
        value.control_domain_id = "alpha".into();
        assert_eq!(
            value.validate(),
            Err(WitnessDomainProfileError::UnnamespacedId {
                field: "control_domain_id"
            })
        );
    }

    #[test]
    fn evidence_is_required_and_unique() {
        let mut value = valid_claims();
        value.evidence_refs.clear();
        assert_eq!(value.validate(), Err(WitnessDomainProfileError::MissingEvidence));

        let mut value = valid_claims();
        value.evidence_refs.push(value.evidence_refs[0].clone());
        assert_eq!(
            value.validate(),
            Err(WitnessDomainProfileError::DuplicateEvidenceRef)
        );
    }

    #[test]
    fn other_kind_requires_detail() {
        let mut value = valid_claims();
        value.domain_kind = WitnessDomainKind::Other;
        assert_eq!(
            value.validate(),
            Err(WitnessDomainProfileError::MissingDomainKindDetail)
        );
        value.domain_kind_detail = Some("research-consortium".into());
        assert!(value.validate().is_ok());
    }

    #[test]
    fn detail_is_rejected_for_non_other_variant() {
        let mut value = valid_claims();
        value.binding_basis_detail = Some("should-not-be-here".into());
        assert_eq!(
            value.validate(),
            Err(WitnessDomainProfileError::UnexpectedBindingBasisDetail)
        );
    }

    #[test]
    fn control_authority_must_be_did_when_present() {
        let mut value = valid_claims();
        value.control_authority_did = Some("authority-alpha".into());
        assert_eq!(
            value.validate(),
            Err(WitnessDomainProfileError::InvalidControlAuthorityDid)
        );
    }

    #[test]
    fn generic_json_claims_round_trip_and_validate() {
        let claims = valid_claims();
        let json = serde_json::to_value(&claims).unwrap();
        let parsed = parse_and_validate_claims(&json).unwrap();
        assert_eq!(parsed, claims);
    }

    #[test]
    fn schema_contract_is_strict_and_contains_required_fields() {
        let schema = credential_subject_json_schema();
        assert_eq!(schema["additionalProperties"], false);
        assert_eq!(schema["properties"]["purpose"]["const"], WITNESS_DOMAIN_PURPOSE);
        let required = schema["required"].as_array().unwrap();
        for field in required_claim_fields() {
            assert!(required.iter().any(|value| value == field));
        }
    }
}
