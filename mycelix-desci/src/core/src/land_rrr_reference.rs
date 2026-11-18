// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Non-authoritative references to external territorial rights assertions.
//!
//! This module validates the structure of a Mycelix application payload that
//! references a Sol Atlas TerritorialRecord assertion. It is not a land
//! registry, title adjudicator, legal rules engine, identity authority, or
//! proof that an external register receipt is authentic or legally effective.
//!
//! The payload is intended to be stored in the existing Mycelix-DeSci event
//! envelope. It does not introduce a parallel event or evidence store.

use chrono::DateTime;
use serde::{Deserialize, Serialize};
use std::collections::HashSet;

pub const LAND_RRR_REFERENCE_SCHEMA: &str = "mycelix.land-rrr-reference.v0";
pub const LAND_RRR_REFERENCE_SCHEMA_URI: &str =
    "https://luminousdynamics.io/schemas/mycelix-land-rrr-reference-v0.schema.json";
pub const LAND_RRR_REFERENCE_CLAIM_CEILING: &str = "reference-only-no-legal-effect";

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ClaimBasis {
    Asserted,
    CommunityAttested,
    AdministrativeRecord,
    CourtDecision,
    ContractualInstrument,
    StatutoryInstrument,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RelationshipKind {
    Right,
    Responsibility,
    Restriction,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum GenericInterestKind {
    SurfaceLand,
    SubsurfaceMineral,
    Groundwater,
    SurfaceWater,
    Access,
    Grazing,
    Conservation,
    CulturalHeritage,
    Servitude,
    Infrastructure,
    Other,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum LadmEdition {
    #[serde(rename = "ISO 19152-1:2024")]
    Iso19152Part1_2024,
    #[serde(rename = "ISO 19152-2:2025")]
    Iso19152Part2_2025,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AuthorityKind {
    Registry,
    CourtOrTribunal,
    StatutoryRegulator,
    CustomaryInstitution,
    CommunityGovernanceBody,
    ContractingParty,
    ProfessionalVerifier,
    Other,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DisputeAssessmentState {
    Unassessed,
    NoChallengeObservedWithinFrontier,
    Challenged,
    UnderReview,
    Resolved,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ChallengeCoverage {
    CompleteWithinDeclaredFrontier,
    Partial,
    Unavailable,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ConsentPurpose {
    Participation,
    Measurement,
    SiteAccess,
    Publication,
    DataSharing,
    Other,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SourceAssertionReference {
    pub source_system: String,
    pub source_schema: String,
    pub source_schema_version: u16,
    pub record_reference: String,
    pub record_sha256: String,
    pub assertion_id: String,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RrrReference {
    pub relationship: RelationshipKind,
    pub generic_interest_kind: GenericInterestKind,
    #[serde(default)]
    pub source_interest_type_code: Option<String>,
    #[serde(default)]
    pub source_interest_type_profile_ref: Option<String>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct LadmReferences {
    pub edition: LadmEdition,
    pub profile_reference: String,
    #[serde(default)]
    pub party_reference: Option<String>,
    #[serde(default)]
    pub group_party_reference: Option<String>,
    #[serde(default)]
    pub basic_administrative_unit_reference: Option<String>,
    pub spatial_unit_reference: String,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AuthorityReference {
    pub source_authority_id: String,
    pub authority_kind: AuthorityKind,
    pub jurisdiction: String,
    pub scope_reference: String,
    pub source_evidence_reference: String,
    #[serde(default)]
    pub valid_time_reference: Option<String>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DisputeAssessment {
    pub state: DisputeAssessmentState,
    #[serde(default)]
    pub source_challenge_assessment_reference: Option<String>,
    #[serde(default)]
    pub frontier_reference: Option<String>,
    #[serde(default)]
    pub scope_reference: Option<String>,
    #[serde(default)]
    pub coverage: Option<ChallengeCoverage>,
    #[serde(default)]
    pub searched_source_references: Vec<String>,
    #[serde(default)]
    pub challenge_search_report_reference: Option<String>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ExternalRegisterReceiptReference {
    pub authority_profile_reference: String,
    pub receipt_reference: String,
    pub receipt_sha256: String,
    pub captured_at: String,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ConsentAuthorizationReference {
    pub record_reference: String,
    pub purpose: ConsentPurpose,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct LandRrrReferenceV0 {
    pub schema: String,
    pub schema_uri: String,
    pub record_id: String,
    pub record_type: String,
    pub created_at: String,
    pub source_assertion: SourceAssertionReference,
    pub claim_basis: Vec<ClaimBasis>,
    pub rrr: RrrReference,
    pub ladm_references: LadmReferences,
    pub authority_references: Vec<AuthorityReference>,
    pub dispute_assessment: DisputeAssessment,
    #[serde(default)]
    pub external_register_receipt: Option<ExternalRegisterReceiptReference>,
    #[serde(default)]
    pub consent_authorization_references: Vec<ConsentAuthorizationReference>,
    pub access_policy_reference: String,
    pub claim_ceiling: String,
    #[serde(default)]
    pub notes: Option<String>,
}

impl LandRrrReferenceV0 {
    /// Validate closed-object structural constraints and reference pairings.
    ///
    /// This does not resolve URIs, authenticate digests or signatures, establish
    /// authority competence, check live revocation/currentness, establish legal
    /// registration, or prove consent. Those checks belong to a trusted host.
    pub fn validate_structural(&self) -> Result<(), Vec<String>> {
        let mut issues = Vec::new();

        require("schema", &self.schema, &mut issues);
        require("schema_uri", &self.schema_uri, &mut issues);
        require("record_id", &self.record_id, &mut issues);
        require("record_type", &self.record_type, &mut issues);
        require("access_policy_reference", &self.access_policy_reference, &mut issues);

        if self.schema != LAND_RRR_REFERENCE_SCHEMA {
            issues.push("unsupported land RRR schema identifier".into());
        }
        if self.schema_uri != LAND_RRR_REFERENCE_SCHEMA_URI {
            issues.push("unexpected land RRR schema URI".into());
        }
        if self.record_type != "territorial-assertion-reference" {
            issues.push("unsupported land RRR record_type".into());
        }
        if self.claim_ceiling != LAND_RRR_REFERENCE_CLAIM_CEILING {
            issues.push("claim_ceiling must remain reference-only-no-legal-effect".into());
        }
        if DateTime::parse_from_rfc3339(&self.created_at).is_err() {
            issues.push("created_at must be RFC 3339".into());
        }

        require(
            "source_assertion.source_system",
            &self.source_assertion.source_system,
            &mut issues,
        );
        require(
            "source_assertion.source_schema",
            &self.source_assertion.source_schema,
            &mut issues,
        );
        require(
            "source_assertion.record_reference",
            &self.source_assertion.record_reference,
            &mut issues,
        );
        require(
            "source_assertion.assertion_id",
            &self.source_assertion.assertion_id,
            &mut issues,
        );
        if self.source_assertion.source_system != "sol-atlas" {
            issues.push("source assertion must identify Sol Atlas".into());
        }
        if self.source_assertion.source_schema != "sol-atlas.territorial-record" {
            issues.push("unsupported Sol Atlas source schema".into());
        }
        if self.source_assertion.source_schema_version != 1 {
            issues.push("unsupported Sol Atlas territorial schema version".into());
        }
        if !is_sha256(&self.source_assertion.record_sha256) {
            issues.push("source_assertion.record_sha256 must be 64 lowercase hex characters".into());
        }

        if self.claim_basis.is_empty() {
            issues.push("claim_basis must contain at least one basis".into());
        }
        let mut claim_bases = HashSet::new();
        for basis in &self.claim_basis {
            if !claim_bases.insert(*basis) {
                issues.push(format!("duplicate claim basis {:?}", basis));
            }
        }

        match (
            self.rrr.source_interest_type_code.as_deref(),
            self.rrr.source_interest_type_profile_ref.as_deref(),
        ) {
            (Some(code), Some(profile)) => {
                require("rrr.source_interest_type_code", code, &mut issues);
                require("rrr.source_interest_type_profile_ref", profile, &mut issues);
            }
            (None, None) if self.rrr.generic_interest_kind == GenericInterestKind::Other => {
                issues.push(
                    "generic_interest_kind other requires source interest type code and profile"
                        .into(),
                );
            }
            (None, None) => {}
            _ => issues.push(
                "source interest type code and profile must be supplied together".into(),
            ),
        }

        require(
            "ladm_references.profile_reference",
            &self.ladm_references.profile_reference,
            &mut issues,
        );
        require(
            "ladm_references.spatial_unit_reference",
            &self.ladm_references.spatial_unit_reference,
            &mut issues,
        );

        let mut authority_ids = HashSet::new();
        for authority in &self.authority_references {
            require(
                "authority.source_authority_id",
                &authority.source_authority_id,
                &mut issues,
            );
            require("authority.jurisdiction", &authority.jurisdiction, &mut issues);
            require("authority.scope_reference", &authority.scope_reference, &mut issues);
            require(
                "authority.source_evidence_reference",
                &authority.source_evidence_reference,
                &mut issues,
            );
            if !authority_ids.insert(authority.source_authority_id.as_str()) {
                issues.push(format!(
                    "duplicate authority reference {}",
                    authority.source_authority_id
                ));
            }
            if authority
                .valid_time_reference
                .as_deref()
                .is_some_and(|reference| reference.trim().is_empty())
            {
                issues.push("authority.valid_time_reference must not be empty".into());
            }
        }

        for basis in &self.claim_basis {
            let has_matching_authority = match basis {
                ClaimBasis::Asserted => true,
                ClaimBasis::CommunityAttested => self.authority_references.iter().any(|authority| {
                    matches!(
                        authority.authority_kind,
                        AuthorityKind::CustomaryInstitution | AuthorityKind::CommunityGovernanceBody
                    )
                }),
                ClaimBasis::AdministrativeRecord => self.authority_references.iter().any(|authority| {
                    matches!(
                        authority.authority_kind,
                        AuthorityKind::Registry | AuthorityKind::StatutoryRegulator
                    )
                }),
                ClaimBasis::CourtDecision => self.authority_references.iter().any(|authority| {
                    authority.authority_kind == AuthorityKind::CourtOrTribunal
                }),
                ClaimBasis::ContractualInstrument => self.authority_references.iter().any(|authority| {
                    authority.authority_kind == AuthorityKind::ContractingParty
                }),
                ClaimBasis::StatutoryInstrument => self.authority_references.iter().any(|authority| {
                    authority.authority_kind == AuthorityKind::StatutoryRegulator
                }),
            };
            if !has_matching_authority {
                issues.push(format!(
                    "claim basis {:?} lacks a matching authority-kind reference",
                    basis
                ));
            }
        }

        let dispute = &self.dispute_assessment;
        if dispute.state == DisputeAssessmentState::NoChallengeObservedWithinFrontier {
            require(
                "dispute_assessment.source_challenge_assessment_reference",
                dispute
                    .source_challenge_assessment_reference
                    .as_deref()
                    .unwrap_or(""),
                &mut issues,
            );
            require(
                "dispute_assessment.frontier_reference",
                dispute.frontier_reference.as_deref().unwrap_or(""),
                &mut issues,
            );
            require(
                "dispute_assessment.scope_reference",
                dispute.scope_reference.as_deref().unwrap_or(""),
                &mut issues,
            );
            require(
                "dispute_assessment.challenge_search_report_reference",
                dispute
                    .challenge_search_report_reference
                    .as_deref()
                    .unwrap_or(""),
                &mut issues,
            );
            if dispute.coverage != Some(ChallengeCoverage::CompleteWithinDeclaredFrontier) {
                issues.push(
                    "no-challenge observation requires complete declared frontier coverage".into(),
                );
            }
            if dispute.searched_source_references.is_empty()
                || dispute
                    .searched_source_references
                    .iter()
                    .any(|reference| reference.trim().is_empty())
            {
                issues.push(
                    "no-challenge observation requires non-empty searched source references"
                        .into(),
                );
            }
        }

        if let Some(receipt) = &self.external_register_receipt {
            require(
                "external_register_receipt.authority_profile_reference",
                &receipt.authority_profile_reference,
                &mut issues,
            );
            require(
                "external_register_receipt.receipt_reference",
                &receipt.receipt_reference,
                &mut issues,
            );
            if !is_sha256(&receipt.receipt_sha256) {
                issues.push("external receipt digest must be 64 lowercase hex characters".into());
            }
            if DateTime::parse_from_rfc3339(&receipt.captured_at).is_err() {
                issues.push("external receipt captured_at must be RFC 3339".into());
            }
        }

        let mut consent_references = HashSet::new();
        for consent in &self.consent_authorization_references {
            require(
                "consent_authorization_reference.record_reference",
                &consent.record_reference,
                &mut issues,
            );
            if !consent_references.insert((consent.record_reference.as_str(), consent.purpose)) {
                issues.push("duplicate consent authorization reference".into());
            }
        }

        if issues.is_empty() {
            Ok(())
        } else {
            Err(issues)
        }
    }
}

fn require(field: &str, value: &str, issues: &mut Vec<String>) {
    if value.trim().is_empty() {
        issues.push(format!("{field} must not be empty"));
    }
}

fn is_sha256(value: &str) -> bool {
    value.len() == 64
        && value
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fixture() -> LandRrrReferenceV0 {
        serde_json::from_str(include_str!("../../../examples/land-rrr-reference-synthetic-v0.json"))
            .expect("synthetic land RRR fixture must deserialize")
    }

    #[test]
    fn synthetic_reference_fixture_is_structurally_valid() {
        assert_eq!(fixture().validate_structural(), Ok(()));
    }

    #[test]
    fn other_interest_requires_source_code_and_profile() {
        let mut value = fixture();
        value.rrr.source_interest_type_profile_ref = None;
        let issues = value.validate_structural().expect_err("profile is required");
        assert!(issues.iter().any(|issue| {
            issue.contains("source interest type code and profile must be supplied together")
        }));
    }

    #[test]
    fn community_attestation_requires_a_community_governance_authority_reference() {
        let mut value = fixture();
        value.claim_basis = vec![ClaimBasis::CommunityAttested];
        let issues = value.validate_structural().expect_err("community authority is required");
        assert!(issues.iter().any(|issue| {
            issue.contains("lacks a matching authority-kind reference")
        }));
    }

    #[test]
    fn no_challenge_observation_requires_complete_frontier_and_report() {
        let mut value = fixture();
        value.dispute_assessment.state =
            DisputeAssessmentState::NoChallengeObservedWithinFrontier;
        let issues = value
            .validate_structural()
            .expect_err("unbounded no-challenge state must be rejected");
        assert!(issues.iter().any(|issue| {
            issue.contains("requires complete declared frontier coverage")
        }));
    }

    #[test]
    fn external_receipt_reference_does_not_change_claim_ceiling() {
        let mut value = fixture();
        value.external_register_receipt = Some(ExternalRegisterReceiptReference {
            authority_profile_reference: "urn:synthetic:authority-profile".into(),
            receipt_reference: "urn:synthetic:register-receipt".into(),
            receipt_sha256: "b".repeat(64),
            captured_at: "2026-11-18T14:00:00Z".into(),
        });
        assert_eq!(value.claim_basis, vec![ClaimBasis::Asserted]);
        assert_eq!(
            value.claim_ceiling,
            LAND_RRR_REFERENCE_CLAIM_CEILING
        );
        assert_eq!(value.validate_structural(), Ok(()));
    }

    #[test]
    fn invalid_source_digest_is_rejected() {
        let mut value = fixture();
        value.source_assertion.record_sha256 = "not-a-digest".into();
        let issues = value.validate_structural().expect_err("bad SHA-256 must be rejected");
        assert!(issues.iter().any(|issue| {
            issue.contains("record_sha256 must be 64 lowercase hex characters")
        }));
    }
}
