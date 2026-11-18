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

#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
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
        require(
            "access_policy_reference",
            &self.access_policy_reference,
            &mut issues,
        );

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
            issues
                .push("source_assertion.record_sha256 must be 64 lowercase hex characters".into());
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
            _ => issues
                .push("source interest type code and profile must be supplied together".into()),
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

        let mut unique_authority_references = HashSet::new();
        for authority in &self.authority_references {
            require(
                "authority.source_authority_id",
                &authority.source_authority_id,
                &mut issues,
            );
            require(
                "authority.jurisdiction",
                &authority.jurisdiction,
                &mut issues,
            );
            require(
                "authority.scope_reference",
                &authority.scope_reference,
                &mut issues,
            );
            require(
                "authority.source_evidence_reference",
                &authority.source_evidence_reference,
                &mut issues,
            );
            if !unique_authority_references.insert(authority.clone()) {
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
                ClaimBasis::CommunityAttested => {
                    self.authority_references.iter().any(|authority| {
                        matches!(
                            authority.authority_kind,
                            AuthorityKind::CustomaryInstitution
                                | AuthorityKind::CommunityGovernanceBody
                        )
                    })
                }
                ClaimBasis::AdministrativeRecord => {
                    self.authority_references.iter().any(|authority| {
                        matches!(
                            authority.authority_kind,
                            AuthorityKind::Registry | AuthorityKind::StatutoryRegulator
                        )
                    })
                }
                ClaimBasis::CourtDecision => self
                    .authority_references
                    .iter()
                    .any(|authority| authority.authority_kind == AuthorityKind::CourtOrTribunal),
                ClaimBasis::ContractualInstrument => self
                    .authority_references
                    .iter()
                    .any(|authority| authority.authority_kind == AuthorityKind::ContractingParty),
                ClaimBasis::StatutoryInstrument => self
                    .authority_references
                    .iter()
                    .any(|authority| authority.authority_kind == AuthorityKind::StatutoryRegulator),
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
                    "no-challenge observation requires non-empty searched source references".into(),
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

/// The four benchmark dispositions consumed by the land-RRR host boundary.
///
/// These values qualify a payload for the next *read-only* stage only. They are
/// not legal statuses, authority credentials, title states, or authorization
/// to mutate Mycelix Property or an external registry.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TibEvaluationOutcome {
    StructuralAccept,
    StructuralReject,
    ReviewRequired,
    NotEvaluable,
}

/// Host action after reading a TIB outcome.
///
/// A structural accept only permits further read-only qualification. Every
/// outcome leaves authoritative land/resource state unchanged. In particular,
/// unknown currentness, missing evidence and unresolved review must be held
/// until reconciliation; they must not be converted into success by timeout,
/// retry, consensus or an AI confidence score.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum QualificationDisposition {
    ContinueReadOnlyQualification,
    RejectPayload,
    HoldForReconciliation,
}

impl QualificationDisposition {
    /// This reference adapter never authorizes authoritative state mutation.
    pub const fn allows_authoritative_mutation(self) -> bool {
        false
    }

    pub const fn requires_reconciliation(self) -> bool {
        matches!(self, Self::HoldForReconciliation)
    }

    pub const fn may_continue_read_only(self) -> bool {
        matches!(self, Self::ContinueReadOnlyQualification)
    }
}

/// Convert a benchmark outcome into a conservative host disposition.
///
/// This is a pure policy seam; it is not yet wired into Mycelix Property's
/// transaction coordinator and does not authenticate the source of an outcome.
pub const fn qualification_disposition(outcome: TibEvaluationOutcome) -> QualificationDisposition {
    match outcome {
        TibEvaluationOutcome::StructuralAccept => {
            QualificationDisposition::ContinueReadOnlyQualification
        }
        TibEvaluationOutcome::StructuralReject => QualificationDisposition::RejectPayload,
        TibEvaluationOutcome::ReviewRequired | TibEvaluationOutcome::NotEvaluable => {
            QualificationDisposition::HoldForReconciliation
        }
    }
}

pub const LAND_QUALIFICATION_DECISION_PACKET_SCHEMA: &str =
    "mycelix.land-qualification-decision-packet.v0";
pub const LAND_QUALIFICATION_DECISION_PACKET_SCHEMA_URI: &str =
    "https://luminousdynamics.io/schemas/mycelix-land-qualification-decision-packet-v0.schema.json";

/// Dispositions that require an auditable human/external-authority review packet.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DecisionPacketOutcome {
    ReviewRequired,
    NotEvaluable,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DecisionPacketMutationBoundary {
    ReadOnlyNoAuthoritativeMutation,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DecisionPacketFrontierCoverage {
    CompleteWithinDeclaredScope,
    Partial,
    Unavailable,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DecisionPacketSourceState {
    ReferenceOnly,
    CurrentnessUnverified,
    Stale,
    Revoked,
    Inaccessible,
    Conflicting,
    GeometryUnqualified,
    ProvenanceUnverified,
    PermissionUnverified,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DecisionPacketReviewDomain {
    LegalAuthority,
    LandSurvey,
    CommunityGovernance,
    EnvironmentalScience,
    PrivacyAndDataGovernance,
    Other,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DecisionPacketTargetSourceRecord {
    pub record_reference: String,
    pub record_sha256: String,
    pub assertion_reference: String,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DecisionPacketFrontier {
    pub frontier_reference: String,
    pub scope: String,
    pub assessed_at: String,
    pub coverage: DecisionPacketFrontierCoverage,
    pub included_source_references: Vec<String>,
    pub excluded_or_unavailable_source_references: Vec<String>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DecisionPacketSource {
    pub source_reference: String,
    pub source_role: String,
    #[serde(default)]
    pub snapshot_sha256: Option<String>,
    #[serde(default)]
    pub captured_at: Option<String>,
    pub qualification_state: DecisionPacketSourceState,
    pub reason: String,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DecisionPacketSeverity {
    Critical,
    High,
    Medium,
    Low,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DecisionPacketGap {
    pub gap_id: String,
    pub severity: DecisionPacketSeverity,
    pub description: String,
    pub required_evidence_or_action: String,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DecisionPacketReviewRequirement {
    pub review_domain: DecisionPacketReviewDomain,
    pub reviewer_role: String,
    pub question: String,
    pub mandatory: bool,
}

/// Portable explanation of why a territorial operation must remain held.
///
/// The packet preserves the evidence frontier, source states, material gaps,
/// required reviewer competencies and recheck triggers. It is not a legal
/// decision, title certificate, credential, consent receipt or authorization.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct QualificationDecisionPacketV0 {
    pub schema: String,
    pub packet_id: String,
    pub created_at: String,
    pub policy_reference: String,
    pub target_operation: String,
    pub target_source_record: DecisionPacketTargetSourceRecord,
    pub outcome: DecisionPacketOutcome,
    pub mutation_boundary: DecisionPacketMutationBoundary,
    pub frontier: DecisionPacketFrontier,
    pub sources: Vec<DecisionPacketSource>,
    pub gaps: Vec<DecisionPacketGap>,
    pub review_requirements: Vec<DecisionPacketReviewRequirement>,
    pub recheck_triggers: Vec<String>,
    #[serde(default)]
    pub dissent_or_conflict_references: Vec<String>,
    pub limitations: Vec<String>,
}

impl QualificationDecisionPacketV0 {
    /// Check packet structure and internal references only.
    ///
    /// This does not resolve source URIs, authenticate any authority, establish
    /// legal status, prove that a reviewer is competent, or enforce a host
    /// transaction freeze. The packet is an auditable hold explanation.
    pub fn validate_structural(&self) -> Result<(), Vec<String>> {
        let mut issues = Vec::new();

        for (field, value) in [
            ("schema", self.schema.as_str()),
            ("packet_id", self.packet_id.as_str()),
            ("policy_reference", self.policy_reference.as_str()),
            ("target_operation", self.target_operation.as_str()),
            (
                "target_source_record.record_reference",
                self.target_source_record.record_reference.as_str(),
            ),
            (
                "target_source_record.assertion_reference",
                self.target_source_record.assertion_reference.as_str(),
            ),
            ("frontier.frontier_reference", self.frontier.frontier_reference.as_str()),
            ("frontier.scope", self.frontier.scope.as_str()),
        ] {
            require(field, value, &mut issues);
        }

        if self.schema != LAND_QUALIFICATION_DECISION_PACKET_SCHEMA {
            issues.push("unsupported qualification decision packet schema".into());
        }
        if !is_sha256(&self.target_source_record.record_sha256) {
            issues.push(
                "target_source_record.record_sha256 must be 64 lowercase hex characters".into(),
            );
        }
        if DateTime::parse_from_rfc3339(&self.created_at).is_err() {
            issues.push("created_at must be RFC 3339".into());
        }
        if DateTime::parse_from_rfc3339(&self.frontier.assessed_at).is_err() {
            issues.push("frontier.assessed_at must be RFC 3339".into());
        }
        if self.mutation_boundary
            != DecisionPacketMutationBoundary::ReadOnlyNoAuthoritativeMutation
        {
            issues.push("qualification packet must prohibit authoritative mutation".into());
        }

        let mut included_references = HashSet::new();
        for reference in &self.frontier.included_source_references {
            require("frontier.included_source_references[]", reference, &mut issues);
            if !included_references.insert(reference.as_str()) {
                issues.push(format!("duplicate included source reference {reference}"));
            }
        }

        let mut excluded_references = HashSet::new();
        for reference in &self.frontier.excluded_or_unavailable_source_references {
            require(
                "frontier.excluded_or_unavailable_source_references[]",
                reference,
                &mut issues,
            );
            if !excluded_references.insert(reference.as_str()) {
                issues.push(format!("duplicate excluded/unavailable source reference {reference}"));
            }
            if included_references.contains(reference.as_str()) {
                issues.push(format!(
                    "source reference {reference} cannot be both included and excluded/unavailable"
                ));
            }
        }

        match self.frontier.coverage {
            DecisionPacketFrontierCoverage::CompleteWithinDeclaredScope => {
                if included_references.is_empty() || !excluded_references.is_empty() {
                    issues.push(
                        "complete frontier requires included sources and no excluded/unavailable sources"
                            .into(),
                    );
                }
            }
            DecisionPacketFrontierCoverage::Partial => {
                if included_references.is_empty() || excluded_references.is_empty() {
                    issues.push(
                        "partial frontier requires both included and excluded/unavailable sources"
                            .into(),
                    );
                }
            }
            DecisionPacketFrontierCoverage::Unavailable => {
                if !included_references.is_empty() || excluded_references.is_empty() {
                    issues.push(
                        "unavailable frontier requires no included sources and at least one excluded/unavailable source"
                            .into(),
                    );
                }
            }
        }

        let mut source_references = HashSet::new();
        for source in &self.sources {
            require("source.source_reference", &source.source_reference, &mut issues);
            require("source.source_role", &source.source_role, &mut issues);
            require("source.reason", &source.reason, &mut issues);
            if !source_references.insert(source.source_reference.as_str()) {
                issues.push(format!(
                    "duplicate decision packet source reference {}",
                    source.source_reference
                ));
            }
            if source
                .snapshot_sha256
                .as_deref()
                .is_some_and(|digest| !is_sha256(digest))
            {
                issues.push(format!(
                    "source {} has an invalid snapshot SHA-256",
                    source.source_reference
                ));
            }
            if source
                .captured_at
                .as_deref()
                .is_some_and(|timestamp| DateTime::parse_from_rfc3339(timestamp).is_err())
            {
                issues.push(format!(
                    "source {} captured_at must be RFC 3339",
                    source.source_reference
                ));
            }
            if source.qualification_state == DecisionPacketSourceState::Inaccessible
                && source.snapshot_sha256.is_some()
            {
                issues.push(format!(
                    "inaccessible source {} must not claim a retrieved snapshot digest",
                    source.source_reference
                ));
            }
            if source.qualification_state == DecisionPacketSourceState::Inaccessible
                && !excluded_references.contains(source.source_reference.as_str())
            {
                issues.push(format!(
                    "inaccessible source {} must be listed as excluded/unavailable in the frontier",
                    source.source_reference
                ));
            }
            if source.qualification_state != DecisionPacketSourceState::Inaccessible
                && !included_references.contains(source.source_reference.as_str())
            {
                issues.push(format!(
                    "assessed source {} must be listed as included in the frontier",
                    source.source_reference
                ));
            }
        }

        if self.sources.is_empty() {
            issues.push("decision packet must include at least one source assessment".into());
        }

        for reference in included_references.iter().chain(excluded_references.iter()) {
            if !source_references.contains(reference) {
                issues.push(format!(
                    "frontier source reference {reference} has no source assessment"
                ));
            }
        }
        if self.gaps.is_empty() {
            issues.push("decision packet must identify at least one unresolved gap".into());
        }
        if self.review_requirements.is_empty() {
            issues.push("decision packet must identify at least one review requirement".into());
        }
        let mut recheck_triggers = HashSet::new();
        if self.recheck_triggers.is_empty()
            || self.recheck_triggers.iter().any(|trigger| {
                trigger.trim().is_empty() || !recheck_triggers.insert(trigger.as_str())
            })
        {
            issues.push("decision packet must include unique, non-empty recheck triggers".into());
        }
        if self.limitations.is_empty()
            || self.limitations.iter().any(|limitation| limitation.trim().is_empty())
        {
            issues.push("decision packet must state its limitations".into());
        }
        if self.outcome == DecisionPacketOutcome::NotEvaluable
            && self.frontier.coverage == DecisionPacketFrontierCoverage::CompleteWithinDeclaredScope
        {
            issues.push(
                "not_evaluable outcome requires partial or unavailable frontier coverage".into(),
            );
        }

        let mut gap_ids = HashSet::new();
        for gap in &self.gaps {
            require("gap.gap_id", &gap.gap_id, &mut issues);
            // The closed DecisionPacketSeverity enum enforces the allowed vocabulary.
            require("gap.description", &gap.description, &mut issues);
            require(
                "gap.required_evidence_or_action",
                &gap.required_evidence_or_action,
                &mut issues,
            );
            if !gap_ids.insert(gap.gap_id.as_str()) {
                issues.push(format!("duplicate decision packet gap id {}", gap.gap_id));
            }
        }

        for review in &self.review_requirements {
            require("review.reviewer_role", &review.reviewer_role, &mut issues);
            require("review.question", &review.question, &mut issues);
        }

        if issues.is_empty() {
            Ok(())
        } else {
            Err(issues)
        }
    }

    /// A decision packet is never an authorization to mutate authoritative state.
    pub const fn allows_authoritative_mutation(&self) -> bool {
        false
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fixture() -> LandRrrReferenceV0 {
        serde_json::from_str(include_str!(
            "../../../examples/land-rrr-reference-synthetic-v0.json"
        ))
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
        let issues = value
            .validate_structural()
            .expect_err("profile is required");
        assert!(issues.iter().any(|issue| {
            issue.contains("source interest type code and profile must be supplied together")
        }));
    }

    #[test]
    fn community_attestation_requires_a_community_governance_authority_reference() {
        let mut value = fixture();
        value.claim_basis = vec![ClaimBasis::CommunityAttested];
        let issues = value
            .validate_structural()
            .expect_err("community authority is required");
        assert!(
            issues
                .iter()
                .any(|issue| { issue.contains("lacks a matching authority-kind reference") })
        );
    }

    #[test]
    fn no_challenge_observation_requires_complete_frontier_and_report() {
        let mut value = fixture();
        value.dispute_assessment.state = DisputeAssessmentState::NoChallengeObservedWithinFrontier;
        let issues = value
            .validate_structural()
            .expect_err("unbounded no-challenge state must be rejected");
        assert!(
            issues
                .iter()
                .any(|issue| { issue.contains("requires complete declared frontier coverage") })
        );
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
        assert_eq!(value.claim_ceiling, LAND_RRR_REFERENCE_CLAIM_CEILING);
        assert_eq!(value.validate_structural(), Ok(()));
    }

    fn authority_reference(id: &str, kind: AuthorityKind) -> AuthorityReference {
        AuthorityReference {
            source_authority_id: id.into(),
            authority_kind: kind,
            jurisdiction: "ZA".into(),
            scope_reference: format!("urn:synthetic:authority-scope:{id}"),
            source_evidence_reference: format!("urn:synthetic:authority-evidence:{id}"),
            valid_time_reference: Some(format!("urn:synthetic:valid-time:{id}")),
        }
    }

    #[test]
    fn all_authority_bases_can_coexist_without_being_collapsed() {
        let mut value = fixture();
        value.claim_basis = vec![
            ClaimBasis::CommunityAttested,
            ClaimBasis::AdministrativeRecord,
            ClaimBasis::CourtDecision,
            ClaimBasis::ContractualInstrument,
            ClaimBasis::StatutoryInstrument,
        ];
        value.authority_references = vec![
            authority_reference("community", AuthorityKind::CommunityGovernanceBody),
            authority_reference("registry", AuthorityKind::Registry),
            authority_reference("court", AuthorityKind::CourtOrTribunal),
            authority_reference("contract", AuthorityKind::ContractingParty),
            authority_reference("regulator", AuthorityKind::StatutoryRegulator),
        ];
        assert_eq!(value.validate_structural(), Ok(()));
    }

    #[test]
    fn every_non_asserted_claim_basis_rejects_the_wrong_authority_kind() {
        let cases = [
            (ClaimBasis::CommunityAttested, AuthorityKind::Registry),
            (
                ClaimBasis::AdministrativeRecord,
                AuthorityKind::CourtOrTribunal,
            ),
            (ClaimBasis::CourtDecision, AuthorityKind::ContractingParty),
            (
                ClaimBasis::ContractualInstrument,
                AuthorityKind::StatutoryRegulator,
            ),
            (
                ClaimBasis::StatutoryInstrument,
                AuthorityKind::CommunityGovernanceBody,
            ),
        ];
        for (basis, wrong_kind) in cases {
            let mut value = fixture();
            value.claim_basis = vec![basis];
            value.authority_references = vec![authority_reference("wrong-kind", wrong_kind)];
            let issues = value
                .validate_structural()
                .expect_err("wrong authority kind must not satisfy a claim basis");
            assert!(
                issues
                    .iter()
                    .any(|issue| { issue.contains("lacks a matching authority-kind reference") })
            );
        }
    }

    #[test]
    fn duplicate_claim_bases_are_rejected() {
        let mut value = fixture();
        value.claim_basis = vec![ClaimBasis::Asserted, ClaimBasis::Asserted];
        let issues = value
            .validate_structural()
            .expect_err("duplicate claim basis must be rejected");
        assert!(
            issues
                .iter()
                .any(|issue| issue.contains("duplicate claim basis"))
        );
    }

    #[test]
    fn duplicate_consent_authorization_references_are_rejected() {
        let mut value = fixture();
        let reference = ConsentAuthorizationReference {
            record_reference: "urn:synthetic:consent:publication".into(),
            purpose: ConsentPurpose::Publication,
        };
        value.consent_authorization_references = vec![reference.clone(), reference];
        let issues = value
            .validate_structural()
            .expect_err("duplicate consent reference must be rejected");
        assert!(
            issues
                .iter()
                .any(|issue| issue.contains("duplicate consent authorization reference"))
        );
    }

    #[test]
    fn exact_duplicate_authority_reference_is_rejected() {
        let mut value = fixture();
        let reference = authority_reference("registry-1", AuthorityKind::Registry);
        value.claim_basis = vec![ClaimBasis::AdministrativeRecord];
        value.authority_references = vec![reference.clone(), reference];
        let issues = value
            .validate_structural()
            .expect_err("exact duplicate authority reference must be rejected");
        assert!(
            issues
                .iter()
                .any(|issue| issue.contains("duplicate authority reference"))
        );
    }

    #[test]
    fn one_issuer_may_have_multiple_distinct_scoped_authority_references() {
        let mut value = fixture();
        value.claim_basis = vec![ClaimBasis::AdministrativeRecord];
        let first = authority_reference("registry-1", AuthorityKind::Registry);
        let mut second = first.clone();
        second.scope_reference = "urn:synthetic:authority-scope:registry-1:distinct".into();
        second.source_evidence_reference =
            "urn:synthetic:authority-evidence:registry-1:distinct".into();
        value.authority_references = vec![first, second];
        assert_eq!(value.validate_structural(), Ok(()));
    }

    #[test]
    fn schema_document_is_valid_json_and_pins_the_expected_schema_id() {
        let schema: serde_json::Value = serde_json::from_str(include_str!(
            "../../../schemas/land-rrr-reference-v0.schema.json"
        ))
        .expect("land RRR JSON Schema document must parse");
        assert_eq!(
            schema["$schema"],
            "https://json-schema.org/draft/2020-12/schema"
        );
        assert_eq!(schema["$id"], LAND_RRR_REFERENCE_SCHEMA_URI);
    }

    #[test]
    fn invalid_source_digest_is_rejected() {
        let mut value = fixture();
        value.source_assertion.record_sha256 = "not-a-digest".into();
        let issues = value
            .validate_structural()
            .expect_err("bad SHA-256 must be rejected");
        assert!(
            issues.iter().any(|issue| {
                issue.contains("record_sha256 must be 64 lowercase hex characters")
            })
        );
    }
}

#[cfg(test)]
mod tib_qualification_policy_tests {
    use super::*;

    #[test]
    fn benchmark_outcomes_map_to_conservative_host_dispositions() {
        assert_eq!(
            qualification_disposition(TibEvaluationOutcome::StructuralAccept),
            QualificationDisposition::ContinueReadOnlyQualification
        );
        assert_eq!(
            qualification_disposition(TibEvaluationOutcome::StructuralReject),
            QualificationDisposition::RejectPayload
        );
        assert_eq!(
            qualification_disposition(TibEvaluationOutcome::ReviewRequired),
            QualificationDisposition::HoldForReconciliation
        );
        assert_eq!(
            qualification_disposition(TibEvaluationOutcome::NotEvaluable),
            QualificationDisposition::HoldForReconciliation
        );
    }

    #[test]
    fn no_benchmark_disposition_authorizes_authoritative_mutation() {
        for outcome in [
            TibEvaluationOutcome::StructuralAccept,
            TibEvaluationOutcome::StructuralReject,
            TibEvaluationOutcome::ReviewRequired,
            TibEvaluationOutcome::NotEvaluable,
        ] {
            assert!(!qualification_disposition(outcome).allows_authoritative_mutation());
        }
    }

    #[test]
    fn review_required_and_not_evaluable_both_hold_for_reconciliation() {
        for outcome in [
            TibEvaluationOutcome::ReviewRequired,
            TibEvaluationOutcome::NotEvaluable,
        ] {
            let disposition = qualification_disposition(outcome);
            assert!(disposition.requires_reconciliation());
            assert!(!disposition.may_continue_read_only());
        }
    }

    #[test]
    fn structural_accept_only_allows_read_only_qualification() {
        let disposition = qualification_disposition(TibEvaluationOutcome::StructuralAccept);
        assert!(disposition.may_continue_read_only());
        assert!(!disposition.requires_reconciliation());
        assert!(!disposition.allows_authoritative_mutation());
    }

    #[test]
    fn unknown_benchmark_outcomes_fail_to_deserialize() {
        assert!(serde_json::from_str::<TibEvaluationOutcome>("\"legal_title_confirmed\"").is_err());
    }
}


#[cfg(test)]
mod decision_packet_tests {
    use super::*;

    fn packet() -> QualificationDecisionPacketV0 {
        serde_json::from_str(include_str!(
            "../../../examples/land-qualification-decision-packet-synthetic-v0.json"
        ))
        .expect("synthetic decision packet must deserialize")
    }

    #[test]
    fn synthetic_review_packet_is_structurally_valid_and_never_authorizes_mutation() {
        let value = packet();
        assert_eq!(value.validate_structural(), Ok(()));
        assert!(!value.allows_authoritative_mutation());
    }

    #[test]
    fn decision_packet_schema_document_is_valid_and_pins_schema_identity() {
        let schema: serde_json::Value = serde_json::from_str(include_str!(
            "../../../schemas/land-qualification-decision-packet-v0.schema.json"
        ))
        .expect("decision packet schema must parse");
        assert_eq!(
            schema["$schema"],
            "https://json-schema.org/draft/2020-12/schema"
        );
        assert_eq!(
            schema["$id"],
            LAND_QUALIFICATION_DECISION_PACKET_SCHEMA_URI
        );
    }

    #[test]
    fn inaccessible_source_cannot_claim_a_retrieved_snapshot_digest() {
        let mut value = packet();
        value.sources[0].qualification_state = DecisionPacketSourceState::Inaccessible;
        let issues = value
            .validate_structural()
            .expect_err("inaccessible source must not carry retrieved snapshot identity");
        assert!(issues.iter().any(|issue| {
            issue.contains("must not claim a retrieved snapshot digest")
        }));
    }

    #[test]
    fn duplicate_source_references_are_rejected() {
        let mut value = packet();
        value.sources.push(value.sources[0].clone());
        let issues = value
            .validate_structural()
            .expect_err("duplicate source identity must be rejected");
        assert!(issues
            .iter()
            .any(|issue| issue.contains("duplicate decision packet source reference")));
    }

    #[test]
    fn not_evaluable_requires_incomplete_frontier_coverage() {
        let mut value = packet();
        value.outcome = DecisionPacketOutcome::NotEvaluable;
        value.frontier.coverage = DecisionPacketFrontierCoverage::CompleteWithinDeclaredScope;
        let issues = value
            .validate_structural()
            .expect_err("not_evaluable cannot claim complete declared coverage");
        assert!(issues.iter().any(|issue| {
            issue.contains("not_evaluable outcome requires partial or unavailable frontier coverage")
        }));
    }

    #[test]
    fn mutation_authorization_boundary_is_not_deserializable() {
        let mut raw: serde_json::Value = serde_json::from_str(include_str!(
            "../../../examples/land-qualification-decision-packet-synthetic-v0.json"
        ))
        .expect("synthetic decision packet must parse");
        raw["mutation_boundary"] =
            serde_json::Value::String("authoritative_mutation_allowed".into());
        assert!(
            serde_json::from_value::<QualificationDecisionPacketV0>(raw).is_err()
        );
    }

    #[test]
    fn inaccessible_sources_must_be_excluded_from_the_declared_frontier() {
        let mut value = packet();
        let source_reference = value.sources[0].source_reference.clone();
        value.sources[0].qualification_state = DecisionPacketSourceState::Inaccessible;
        value.sources[0].snapshot_sha256 = None;
        value
            .frontier
            .included_source_references
            .retain(|reference| reference != &source_reference);
        value
            .frontier
            .excluded_or_unavailable_source_references
            .push(source_reference.clone());
        assert_eq!(value.validate_structural(), Ok(()));

        value
            .frontier
            .included_source_references
            .push(source_reference);
        let issues = value
            .validate_structural()
            .expect_err("a source cannot be included and inaccessible at once");
        assert!(issues
            .iter()
            .any(|issue| issue.contains("cannot be both included and excluded/unavailable")));
    }

    #[test]
    fn duplicate_recheck_triggers_are_rejected() {
        let mut value = packet();
        value.recheck_triggers.push(value.recheck_triggers[0].clone());
        let issues = value
            .validate_structural()
            .expect_err("recheck triggers must be unique");
        assert!(issues
            .iter()
            .any(|issue| issue.contains("unique, non-empty recheck triggers")));
    }

    #[test]
    fn gap_severity_is_a_closed_vocabulary() {
        let mut raw: serde_json::Value = serde_json::from_str(include_str!(
            "../../../examples/land-qualification-decision-packet-synthetic-v0.json"
        ))
        .expect("synthetic decision packet must parse");
        raw["gaps"][0]["severity"] = serde_json::Value::String("urgent".into());
        assert!(serde_json::from_value::<QualificationDecisionPacketV0>(raw).is_err());
    }

    #[test]
    fn every_assessed_source_must_be_accounted_for_by_the_frontier() {
        let mut value = packet();
        value.frontier.included_source_references.pop();
        let issues = value
            .validate_structural()
            .expect_err("unaccounted source must be rejected");
        assert!(issues
            .iter()
            .any(|issue| issue.contains("must be listed as included in the frontier")));
    }

    #[test]
    fn not_evaluable_packet_can_record_an_unavailable_source_frontier() {
        let mut value = packet();
        value.outcome = DecisionPacketOutcome::NotEvaluable;
        value.frontier.coverage = DecisionPacketFrontierCoverage::Unavailable;
        value.frontier.included_source_references.clear();
        value.frontier.excluded_or_unavailable_source_references.clear();
        for source in &mut value.sources {
            source.qualification_state = DecisionPacketSourceState::Inaccessible;
            source.snapshot_sha256 = None;
            source.captured_at = None;
            value
                .frontier
                .excluded_or_unavailable_source_references
                .push(source.source_reference.clone());
        }
        assert_eq!(value.validate_structural(), Ok(()));
        assert!(!value.allows_authoritative_mutation());
    }

    #[test]
    fn complete_frontier_rejects_unavailable_sources() {
        let mut value = packet();
        value.frontier.coverage = DecisionPacketFrontierCoverage::CompleteWithinDeclaredScope;
        let issues = value
            .validate_structural()
            .expect_err("complete frontier cannot list excluded sources");
        assert!(issues.iter().any(|issue| {
            issue.contains("complete frontier requires included sources")
        }));
    }

    #[test]
    fn partial_frontier_requires_both_included_and_unavailable_sources() {
        let mut value = packet();
        value.frontier.excluded_or_unavailable_source_references.clear();
        let issues = value
            .validate_structural()
            .expect_err("partial frontier must expose its unavailable sources");
        assert!(issues
            .iter()
            .any(|issue| issue.contains("partial frontier requires both included and excluded")));
    }

    #[test]
    fn frontier_cannot_name_a_source_without_a_source_assessment() {
        let mut value = packet();
        value
            .frontier
            .included_source_references
            .push("urn:synthetic:unassessed-source".into());
        let issues = value
            .validate_structural()
            .expect_err("frontier source must have a source assessment");
        assert!(issues
            .iter()
            .any(|issue| issue.contains("has no source assessment")));
    }

    #[test]
    fn review_packet_requires_explicit_reviewers_rechecks_and_limitations() {
        let mut value = packet();
        value.review_requirements.clear();
        value.recheck_triggers.clear();
        value.limitations.clear();
        let issues = value
            .validate_structural()
            .expect_err("review packet must not omit review and recheck requirements");
        assert!(issues.iter().any(|issue| issue.contains("at least one review requirement")));
        assert!(issues.iter().any(|issue| issue.contains("non-empty recheck triggers")));
        assert!(issues.iter().any(|issue| issue.contains("state its limitations")));
    }
}
