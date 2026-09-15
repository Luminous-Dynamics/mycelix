// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! AC-003 institutional relationship graph contract.
//!
//! This module represents authority, ownership, influence, procurement, appeal,
//! disclosure, and audit relationships without turning those relationships into
//! accusations. It preserves provenance, privacy-aware disclosure, challenge
//! state, and plural corroboration for graph assertions.

use std::collections::BTreeSet;

use serde::{Deserialize, Serialize};

use crate::capture_observation::ProvenanceRef;

/// Opaque institutional node classes.
///
/// Node references intentionally carry no profile attributes such as name,
/// address, ideology, race, wealth, or reputation. Private natural persons can
/// appear only as opaque credentials where a relationship such as beneficial
/// ownership must be represented.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum InstitutionalNodeKind {
    PublicInstitution,
    PublicOffice,
    PublicPowerHolder,
    Organization,
    LegalEntity,
    PrivatePersonCredential,
    ProcurementProcedure,
    Contract,
    PublicDecision,
    AuditBody,
    Asset,
    Aggregate,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct InstitutionalNodeRef {
    pub id: String,
    pub kind: InstitutionalNodeKind,
}

/// Visibility model for relationship metadata.
///
/// `LegitimateInterest` supports privacy-aware oversight where unrestricted
/// publication would be disproportionate, while still preventing total opacity.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum DisclosureClass {
    PublicMetadata,
    LegitimateInterest,
    ConfidentialCredential,
}

/// Evidentiary state of a graph relationship.
///
/// There is deliberately no `True`, `Guilty`, or `Corrupt` state.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum AssertionStatus {
    Declared,
    Corroborated,
    Challenged,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct OwnershipInterest {
    /// Optional share in basis points (0..=10_000). `None` means unknown/not
    /// represented, not zero ownership.
    pub share_bps: Option<u16>,
    pub beneficial: bool,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum InfluenceChannel {
    Meeting,
    WrittenSubmission,
    AdvisoryRole,
    CampaignFinance,
    PublicCommunication,
    DigitalCampaign,
    Other(String),
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum ProcurementRole {
    ProcuringAuthority,
    Bidder,
    Awardee,
    Subcontractor,
    BeneficialOwner,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum AppealStatus {
    Filed,
    UnderReview,
    Upheld,
    Dismissed,
    Withdrawn,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum AuditIndependence {
    SelfAssessment,
    Internal,
    ExternalIndependent,
    Unknown,
}

/// Typed institutional relationships.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum InstitutionalRelationKind {
    Authority {
        grant_ref: String,
    },
    Delegation {
        grant_ref: String,
    },
    Ownership(OwnershipInterest),
    Influence {
        channel: InfluenceChannel,
        disclosure_ref: String,
    },
    ProcurementParticipation {
        role: ProcurementRole,
    },
    DecisionAuthorship {
        contribution_ref: String,
    },
    ConflictDisclosure {
        disclosure_ref: String,
    },
    Appeal {
        case_ref: String,
        status: AppealStatus,
    },
    Audit {
        report_ref: String,
        independence: AuditIndependence,
    },
}

/// Provenance-bearing institutional graph edge.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct InstitutionalEdge {
    pub id: String,
    pub from: InstitutionalNodeRef,
    pub to: InstitutionalNodeRef,
    pub relation: InstitutionalRelationKind,
    pub disclosure: DisclosureClass,
    pub assertion_status: AssertionStatus,
    pub provenance: Vec<ProvenanceRef>,
    pub challenge_refs: Vec<String>,
    pub recorded_at: u64,
    pub valid_from: Option<u64>,
    pub valid_until: Option<u64>,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum InstitutionalGraphViolation {
    MissingEdgeId,
    MissingNodeId,
    MissingRelationReference,
    InvalidCustomInfluenceChannel,
    InvalidOwnershipShare,
    MissingProvenance,
    InvalidProvenance,
    CorroborationRequiresIndependentSources,
    ChallengedAssertionRequiresChallengeReference,
    InvalidChallengeReference,
    InvalidValidityInterval,
    PublicPowerRelationshipTooOpaque,
    BeneficialOwnershipTooOpaque,
    AuthorityTargetInvalid,
    DelegationEndpointInvalid,
    OwnershipTargetInvalid,
    InfluenceTargetInvalid,
    ProcurementTargetInvalid,
    DecisionTargetInvalid,
    ConflictDisclosureTargetInvalid,
    AuditTargetInvalid,
    SelfAuditCannotBeIndependent,
}

#[derive(Debug, Default, Clone, Copy)]
pub struct InstitutionalGraphContract;

impl InstitutionalGraphContract {
    pub fn validate_edge(
        edge: &InstitutionalEdge,
    ) -> Result<(), Vec<InstitutionalGraphViolation>> {
        let mut violations = Vec::new();

        if edge.id.trim().is_empty() {
            violations.push(InstitutionalGraphViolation::MissingEdgeId);
        }
        validate_node(&edge.from, &mut violations);
        validate_node(&edge.to, &mut violations);
        validate_relation(edge, &mut violations);
        validate_provenance(edge, &mut violations);
        validate_assertion_status(edge, &mut violations);
        validate_time(edge, &mut violations);
        validate_disclosure(edge, &mut violations);
        validate_topology(edge, &mut violations);

        if violations.is_empty() {
            Ok(())
        } else {
            Err(violations)
        }
    }
}

fn validate_node(
    node: &InstitutionalNodeRef,
    violations: &mut Vec<InstitutionalGraphViolation>,
) {
    if node.id.trim().is_empty() {
        violations.push(InstitutionalGraphViolation::MissingNodeId);
    }
}

fn validate_relation(
    edge: &InstitutionalEdge,
    violations: &mut Vec<InstitutionalGraphViolation>,
) {
    match &edge.relation {
        InstitutionalRelationKind::Authority { grant_ref }
        | InstitutionalRelationKind::Delegation { grant_ref } => {
            if grant_ref.trim().is_empty() {
                violations.push(InstitutionalGraphViolation::MissingRelationReference);
            }
        }
        InstitutionalRelationKind::Ownership(interest) => {
            if interest.share_bps.is_some_and(|share| share > 10_000) {
                violations.push(InstitutionalGraphViolation::InvalidOwnershipShare);
            }
        }
        InstitutionalRelationKind::Influence {
            channel,
            disclosure_ref,
        } => {
            if disclosure_ref.trim().is_empty() {
                violations.push(InstitutionalGraphViolation::MissingRelationReference);
            }
            if matches!(channel, InfluenceChannel::Other(value) if value.trim().is_empty()) {
                violations.push(InstitutionalGraphViolation::InvalidCustomInfluenceChannel);
            }
        }
        InstitutionalRelationKind::ProcurementParticipation { .. } => {}
        InstitutionalRelationKind::DecisionAuthorship { contribution_ref } => {
            if contribution_ref.trim().is_empty() {
                violations.push(InstitutionalGraphViolation::MissingRelationReference);
            }
        }
        InstitutionalRelationKind::ConflictDisclosure { disclosure_ref } => {
            if disclosure_ref.trim().is_empty() {
                violations.push(InstitutionalGraphViolation::MissingRelationReference);
            }
        }
        InstitutionalRelationKind::Appeal { case_ref, .. } => {
            if case_ref.trim().is_empty() {
                violations.push(InstitutionalGraphViolation::MissingRelationReference);
            }
        }
        InstitutionalRelationKind::Audit { report_ref, .. } => {
            if report_ref.trim().is_empty() {
                violations.push(InstitutionalGraphViolation::MissingRelationReference);
            }
        }
    }
}

fn validate_provenance(
    edge: &InstitutionalEdge,
    violations: &mut Vec<InstitutionalGraphViolation>,
) {
    if edge.provenance.is_empty() {
        violations.push(InstitutionalGraphViolation::MissingProvenance);
        return;
    }

    if edge.provenance.iter().any(|provenance| {
        provenance.source_ref.trim().is_empty()
            || provenance
                .content_hash
                .as_ref()
                .is_some_and(|hash| hash.trim().is_empty())
    }) {
        violations.push(InstitutionalGraphViolation::InvalidProvenance);
    }
}

fn validate_assertion_status(
    edge: &InstitutionalEdge,
    violations: &mut Vec<InstitutionalGraphViolation>,
) {
    if edge.assertion_status == AssertionStatus::Corroborated {
        let independent_sources: BTreeSet<_> = edge
            .provenance
            .iter()
            .map(|provenance| provenance.source_ref.trim())
            .filter(|source| !source.is_empty())
            .collect();
        if independent_sources.len() < 2 {
            violations.push(
                InstitutionalGraphViolation::CorroborationRequiresIndependentSources,
            );
        }
    }

    if edge.assertion_status == AssertionStatus::Challenged && edge.challenge_refs.is_empty() {
        violations.push(
            InstitutionalGraphViolation::ChallengedAssertionRequiresChallengeReference,
        );
    }

    if edge
        .challenge_refs
        .iter()
        .any(|reference| reference.trim().is_empty())
    {
        violations.push(InstitutionalGraphViolation::InvalidChallengeReference);
    }
}

fn validate_time(
    edge: &InstitutionalEdge,
    violations: &mut Vec<InstitutionalGraphViolation>,
) {
    if let (Some(valid_from), Some(valid_until)) = (edge.valid_from, edge.valid_until) {
        if valid_until <= valid_from {
            violations.push(InstitutionalGraphViolation::InvalidValidityInterval);
        }
    }
}

fn validate_disclosure(
    edge: &InstitutionalEdge,
    violations: &mut Vec<InstitutionalGraphViolation>,
) {
    let public_accountability_relation = matches!(
        &edge.relation,
        InstitutionalRelationKind::Authority { .. }
            | InstitutionalRelationKind::Delegation { .. }
            | InstitutionalRelationKind::Influence { .. }
            | InstitutionalRelationKind::DecisionAuthorship { .. }
            | InstitutionalRelationKind::ConflictDisclosure { .. }
            | InstitutionalRelationKind::Appeal { .. }
            | InstitutionalRelationKind::Audit { .. }
            | InstitutionalRelationKind::ProcurementParticipation {
                role: ProcurementRole::ProcuringAuthority | ProcurementRole::Awardee
            }
    );

    if public_accountability_relation
        && edge.disclosure == DisclosureClass::ConfidentialCredential
    {
        violations.push(InstitutionalGraphViolation::PublicPowerRelationshipTooOpaque);
    }

    if matches!(
        &edge.relation,
        InstitutionalRelationKind::Ownership(OwnershipInterest {
            beneficial: true,
            ..
        })
    ) && edge.disclosure == DisclosureClass::ConfidentialCredential
    {
        violations.push(InstitutionalGraphViolation::BeneficialOwnershipTooOpaque);
    }
}

fn validate_topology(
    edge: &InstitutionalEdge,
    violations: &mut Vec<InstitutionalGraphViolation>,
) {
    match &edge.relation {
        InstitutionalRelationKind::Authority { .. } => {
            if !matches!(
                &edge.to.kind,
                InstitutionalNodeKind::PublicInstitution | InstitutionalNodeKind::PublicOffice
            ) {
                violations.push(InstitutionalGraphViolation::AuthorityTargetInvalid);
            }
        }
        InstitutionalRelationKind::Delegation { .. } => {
            let source_valid = matches!(
                &edge.from.kind,
                InstitutionalNodeKind::PublicInstitution
                    | InstitutionalNodeKind::PublicOffice
                    | InstitutionalNodeKind::PublicPowerHolder
            );
            let target_valid = matches!(
                &edge.to.kind,
                InstitutionalNodeKind::PublicOffice | InstitutionalNodeKind::PublicPowerHolder
            );
            if !(source_valid && target_valid) {
                violations.push(InstitutionalGraphViolation::DelegationEndpointInvalid);
            }
        }
        InstitutionalRelationKind::Ownership(_) => {
            if !matches!(
                &edge.to.kind,
                InstitutionalNodeKind::Organization
                    | InstitutionalNodeKind::LegalEntity
                    | InstitutionalNodeKind::Asset
            ) {
                violations.push(InstitutionalGraphViolation::OwnershipTargetInvalid);
            }
        }
        InstitutionalRelationKind::Influence { .. } => {
            if !matches!(
                &edge.to.kind,
                InstitutionalNodeKind::PublicInstitution
                    | InstitutionalNodeKind::PublicOffice
                    | InstitutionalNodeKind::PublicDecision
            ) {
                violations.push(InstitutionalGraphViolation::InfluenceTargetInvalid);
            }
        }
        InstitutionalRelationKind::ProcurementParticipation { .. } => {
            if !matches!(
                &edge.to.kind,
                InstitutionalNodeKind::ProcurementProcedure | InstitutionalNodeKind::Contract
            ) {
                violations.push(InstitutionalGraphViolation::ProcurementTargetInvalid);
            }
        }
        InstitutionalRelationKind::DecisionAuthorship { .. } => {
            if !matches!(&edge.to.kind, InstitutionalNodeKind::PublicDecision) {
                violations.push(InstitutionalGraphViolation::DecisionTargetInvalid);
            }
        }
        InstitutionalRelationKind::ConflictDisclosure { .. } => {
            if !matches!(
                &edge.to.kind,
                InstitutionalNodeKind::PublicInstitution
                    | InstitutionalNodeKind::PublicOffice
                    | InstitutionalNodeKind::PublicDecision
            ) {
                violations.push(InstitutionalGraphViolation::ConflictDisclosureTargetInvalid);
            }
        }
        InstitutionalRelationKind::Appeal { .. } => {
            if !matches!(
                &edge.to.kind,
                InstitutionalNodeKind::PublicDecision | InstitutionalNodeKind::Contract
            ) {
                violations.push(InstitutionalGraphViolation::DecisionTargetInvalid);
            }
        }
        InstitutionalRelationKind::Audit { independence, .. } => {
            if matches!(&edge.to.kind, InstitutionalNodeKind::PrivatePersonCredential) {
                violations.push(InstitutionalGraphViolation::AuditTargetInvalid);
            }
            if matches!(independence, AuditIndependence::ExternalIndependent)
                && edge.from.id == edge.to.id
            {
                violations.push(InstitutionalGraphViolation::SelfAuditCannotBeIndependent);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn source(source_ref: &str) -> ProvenanceRef {
        ProvenanceRef {
            source_ref: source_ref.into(),
            content_hash: Some(format!("sha256:{source_ref}")),
        }
    }

    fn node(id: &str, kind: InstitutionalNodeKind) -> InstitutionalNodeRef {
        InstitutionalNodeRef {
            id: id.into(),
            kind,
        }
    }

    fn influence_edge() -> InstitutionalEdge {
        InstitutionalEdge {
            id: "edge:influence:1".into(),
            from: node("org:builders-association", InstitutionalNodeKind::Organization),
            to: node("decision:zoning-41", InstitutionalNodeKind::PublicDecision),
            relation: InstitutionalRelationKind::Influence {
                channel: InfluenceChannel::WrittenSubmission,
                disclosure_ref: "register:submission:991".into(),
            },
            disclosure: DisclosureClass::PublicMetadata,
            assertion_status: AssertionStatus::Declared,
            provenance: vec![source("register:submission:991")],
            challenge_refs: vec![],
            recorded_at: 1_789_000_000,
            valid_from: Some(1_788_000_000),
            valid_until: None,
        }
    }

    #[test]
    fn legitimate_influence_is_representable_without_calling_it_corruption() {
        assert_eq!(
            InstitutionalGraphContract::validate_edge(&influence_edge()),
            Ok(())
        );
    }

    #[test]
    fn public_influence_requires_a_disclosure_reference() {
        let mut edge = influence_edge();
        if let InstitutionalRelationKind::Influence { disclosure_ref, .. } = &mut edge.relation {
            disclosure_ref.clear();
        }
        assert!(
            InstitutionalGraphContract::validate_edge(&edge)
                .expect_err("undocumented influence must fail closed")
                .contains(&InstitutionalGraphViolation::MissingRelationReference)
        );
    }

    #[test]
    fn influence_must_target_public_decision_making() {
        let mut edge = influence_edge();
        edge.to = node("org:unrelated", InstitutionalNodeKind::Organization);
        assert!(
            InstitutionalGraphContract::validate_edge(&edge)
                .expect_err("influence topology must be explicit")
                .contains(&InstitutionalGraphViolation::InfluenceTargetInvalid)
        );
    }

    #[test]
    fn beneficial_ownership_can_be_privacy_aware_but_not_fully_opaque() {
        let edge = InstitutionalEdge {
            id: "edge:ownership:1".into(),
            from: node(
                "credential:beneficial-owner:opaque-7",
                InstitutionalNodeKind::PrivatePersonCredential,
            ),
            to: node("company:vendor-22", InstitutionalNodeKind::LegalEntity),
            relation: InstitutionalRelationKind::Ownership(OwnershipInterest {
                share_bps: Some(6_000),
                beneficial: true,
            }),
            disclosure: DisclosureClass::LegitimateInterest,
            assertion_status: AssertionStatus::Declared,
            provenance: vec![source("registry:bo:22")],
            challenge_refs: vec![],
            recorded_at: 1_789_000_000,
            valid_from: None,
            valid_until: None,
        };
        assert_eq!(InstitutionalGraphContract::validate_edge(&edge), Ok(()));

        let mut opaque = edge.clone();
        opaque.disclosure = DisclosureClass::ConfidentialCredential;
        assert!(
            InstitutionalGraphContract::validate_edge(&opaque)
                .expect_err("beneficial ownership must remain available to oversight")
                .contains(&InstitutionalGraphViolation::BeneficialOwnershipTooOpaque)
        );
    }

    #[test]
    fn ownership_share_is_bounded() {
        let mut edge = InstitutionalEdge {
            id: "edge:ownership:2".into(),
            from: node("entity:parent", InstitutionalNodeKind::LegalEntity),
            to: node("entity:child", InstitutionalNodeKind::LegalEntity),
            relation: InstitutionalRelationKind::Ownership(OwnershipInterest {
                share_bps: Some(10_001),
                beneficial: false,
            }),
            disclosure: DisclosureClass::PublicMetadata,
            assertion_status: AssertionStatus::Declared,
            provenance: vec![source("registry:company:2")],
            challenge_refs: vec![],
            recorded_at: 1,
            valid_from: None,
            valid_until: None,
        };
        assert!(
            InstitutionalGraphContract::validate_edge(&edge)
                .expect_err("ownership above 100% must fail")
                .contains(&InstitutionalGraphViolation::InvalidOwnershipShare)
        );

        if let InstitutionalRelationKind::Ownership(interest) = &mut edge.relation {
            interest.share_bps = Some(10_000);
        }
        assert_eq!(InstitutionalGraphContract::validate_edge(&edge), Ok(()));
    }

    #[test]
    fn corroborated_relationship_requires_distinct_sources() {
        let mut edge = influence_edge();
        edge.assertion_status = AssertionStatus::Corroborated;
        assert!(
            InstitutionalGraphContract::validate_edge(&edge)
                .expect_err("single-source corroboration must fail")
                .contains(
                    &InstitutionalGraphViolation::CorroborationRequiresIndependentSources
                )
        );

        edge.provenance.push(source("minutes:committee:41"));
        assert_eq!(InstitutionalGraphContract::validate_edge(&edge), Ok(()));
    }

    #[test]
    fn challenged_relationship_requires_challenge_provenance() {
        let mut edge = influence_edge();
        edge.assertion_status = AssertionStatus::Challenged;
        assert!(
            InstitutionalGraphContract::validate_edge(&edge)
                .expect_err("challenge state without challenge reference must fail")
                .contains(
                    &InstitutionalGraphViolation::ChallengedAssertionRequiresChallengeReference
                )
        );

        edge.challenge_refs = vec!["challenge:vendor-response:14".into()];
        assert_eq!(InstitutionalGraphContract::validate_edge(&edge), Ok(()));
    }

    #[test]
    fn public_authority_relationship_cannot_be_confidential_only() {
        let edge = InstitutionalEdge {
            id: "edge:authority:1".into(),
            from: node("holder:alice", InstitutionalNodeKind::PublicPowerHolder),
            to: node("office:water", InstitutionalNodeKind::PublicOffice),
            relation: InstitutionalRelationKind::Authority {
                grant_ref: "grant:water:alice".into(),
            },
            disclosure: DisclosureClass::ConfidentialCredential,
            assertion_status: AssertionStatus::Declared,
            provenance: vec![source("authority-register:water")],
            challenge_refs: vec![],
            recorded_at: 1,
            valid_from: Some(10),
            valid_until: Some(20),
        };
        assert!(
            InstitutionalGraphContract::validate_edge(&edge)
                .expect_err("public authority must be inspectable")
                .contains(&InstitutionalGraphViolation::PublicPowerRelationshipTooOpaque)
        );
    }

    #[test]
    fn invalid_temporal_lineage_is_rejected() {
        let mut edge = influence_edge();
        edge.valid_from = Some(100);
        edge.valid_until = Some(100);
        assert!(
            InstitutionalGraphContract::validate_edge(&edge)
                .expect_err("non-forward validity interval must fail")
                .contains(&InstitutionalGraphViolation::InvalidValidityInterval)
        );
    }

    #[test]
    fn self_audit_cannot_claim_external_independence() {
        let edge = InstitutionalEdge {
            id: "edge:audit:1".into(),
            from: node("institution:agency", InstitutionalNodeKind::PublicInstitution),
            to: node("institution:agency", InstitutionalNodeKind::PublicInstitution),
            relation: InstitutionalRelationKind::Audit {
                report_ref: "audit:self:1".into(),
                independence: AuditIndependence::ExternalIndependent,
            },
            disclosure: DisclosureClass::PublicMetadata,
            assertion_status: AssertionStatus::Declared,
            provenance: vec![source("audit:self:1")],
            challenge_refs: vec![],
            recorded_at: 1,
            valid_from: None,
            valid_until: None,
        };
        assert!(
            InstitutionalGraphContract::validate_edge(&edge)
                .expect_err("self audit cannot be externally independent")
                .contains(&InstitutionalGraphViolation::SelfAuditCannotBeIndependent)
        );
    }
}
