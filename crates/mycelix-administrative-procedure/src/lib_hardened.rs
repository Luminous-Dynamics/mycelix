// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Hardened public surface for ADMIN-001.
//!
//! The original ADMIN-001 implementation remains in `lib.rs` as a private
//! semantic module so its focused tests and internal transition machinery are
//! preserved. External callers receive only this lineage-qualified facade.
//!
//! The key theorem is:
//!
//! `deserializable AdministrativeCase snapshot != qualified case lineage`.
//!
//! A consequential administrative decision therefore requires replay from an
//! exact `Filed` root through the bounded pre-decision transitions. A caller
//! cannot present a syntactically valid `ReadyForDecision` snapshot and skip
//! the lineage proof.

#[path = "lib.rs"]
mod legacy;

pub use legacy::{
    AdministrativeCase, AdministrativeCaseId, AdministrativeCaseState,
    AdministrativeDecisionEnvelope, AdministrativeDecisionPolicy, AdministrativeProcedureError,
    IssuedAdministrativeDecision, PROTOCOL_VERSION, PreDecisionState, PreDecisionTransition,
    ProcedureProfileId, QualifiedAdministrativeDecision, QualifiedPreDecisionTransition,
};

use mycelix_institutional_core::{AuthorityGrant, EvidenceRef};
use std::fmt;

const MAX_PRE_DECISION_TRANSITIONS: usize = 2;

/// Opaque proof that an administrative case has been replayed from an exact
/// `Filed` root through the accepted ADMIN-001 structural transition kernel.
///
/// This type is deliberately not `Clone`, `Serialize`, or `Deserialize`.
/// Persisted snapshots must be requalified from their lineage rather than
/// becoming authority-bearing merely because they deserialize successfully.
#[derive(Debug, PartialEq, Eq)]
pub struct QualifiedAdministrativeCaseLineage {
    current_case: AdministrativeCase,
    decision_policy: AdministrativeDecisionPolicy,
    transition_count: usize,
}

impl QualifiedAdministrativeCaseLineage {
    pub fn current_case(&self) -> &AdministrativeCase {
        &self.current_case
    }

    pub fn decision_policy(&self) -> &AdministrativeDecisionPolicy {
        &self.decision_policy
    }

    pub fn transition_count(&self) -> usize {
        self.transition_count
    }

    pub fn grants_authority(&self) -> bool {
        false
    }

    pub fn grants_external_effect_authority(&self) -> bool {
        false
    }
}

/// Replay one candidate case from an exact `Filed` root.
///
/// The decision policy is captured at lineage qualification time and retained
/// inside the opaque token. The later decision qualifier does not accept a
/// caller-supplied replacement policy, preventing decision-time same-profile
/// policy substitution within this pure layer.
pub fn qualify_case_lineage(
    root: AdministrativeCase,
    transitions: &[PreDecisionTransition],
    decision_policy: AdministrativeDecisionPolicy,
) -> Result<QualifiedAdministrativeCaseLineage, AdministrativeQualificationError> {
    root.validate()?;
    decision_policy.validate()?;

    if !matches!(&root.state, AdministrativeCaseState::Filed) {
        return Err(AdministrativeQualificationError::LineageRootMustBeFiled);
    }
    if transitions.len() > MAX_PRE_DECISION_TRANSITIONS {
        return Err(AdministrativeQualificationError::TooManyPreDecisionTransitions);
    }
    if decision_policy.procedure_profile != root.procedure_profile {
        return Err(AdministrativeProcedureError::ProcedureProfileMismatch.into());
    }

    let mut current = root;
    for transition in transitions.iter().cloned() {
        let qualified = legacy::qualify_pre_decision_transition(&current, transition)?;
        current = qualified.into_successor();
    }

    Ok(QualifiedAdministrativeCaseLineage {
        current_case: current,
        decision_policy,
        transition_count: transitions.len(),
    })
}

/// Qualify a consequential administrative decision only against a replayed
/// case lineage and the exact decision policy captured with that lineage.
pub fn qualify_administrative_decision(
    lineage: &QualifiedAdministrativeCaseLineage,
    envelope: AdministrativeDecisionEnvelope,
    grant: &AuthorityGrant,
    authority_evidence: &[EvidenceRef],
) -> Result<QualifiedAdministrativeDecision, AdministrativeQualificationError> {
    legacy::qualify_administrative_decision(
        lineage.current_case(),
        envelope,
        grant,
        lineage.decision_policy(),
        authority_evidence,
    )
    .map_err(Into::into)
}

/// Issue only a decision that was qualified against the same replayed lineage.
///
/// This remains a pure semantic successor and grants no external-effect
/// authority.
pub fn issue_qualified_decision(
    lineage: &QualifiedAdministrativeCaseLineage,
    qualified: QualifiedAdministrativeDecision,
) -> Result<IssuedAdministrativeDecision, AdministrativeQualificationError> {
    legacy::issue_qualified_decision(lineage.current_case(), qualified).map_err(Into::into)
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum AdministrativeQualificationError {
    Procedure(AdministrativeProcedureError),
    LineageRootMustBeFiled,
    TooManyPreDecisionTransitions,
}

impl From<AdministrativeProcedureError> for AdministrativeQualificationError {
    fn from(value: AdministrativeProcedureError) -> Self {
        Self::Procedure(value)
    }
}

impl fmt::Display for AdministrativeQualificationError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Procedure(error) => write!(f, "{error}"),
            Self::LineageRootMustBeFiled => {
                write!(
                    f,
                    "administrative lineage must begin from an exact Filed root"
                )
            }
            Self::TooManyPreDecisionTransitions => write!(
                f,
                "ADMIN-001 permits at most two pre-decision structural transitions"
            ),
        }
    }
}

impl std::error::Error for AdministrativeQualificationError {}

#[cfg(test)]
mod hardening_tests {
    use super::*;
    use mycelix_institutional_core::{
        AuthorityGrantId, AuthoritySourceKind, AuthoritySourceRef, CapabilityId, Decision,
        DecisionId, Digest32, InstitutionId, JurisdictionId,
        PROTOCOL_VERSION as INSTITUTIONAL_PROTOCOL_VERSION, PrincipalId, RoleId, RulebookId,
        RulebookRef,
    };

    fn digest(byte: u8) -> Digest32 {
        Digest32([byte; 32])
    }

    fn principal(value: &str) -> PrincipalId {
        PrincipalId::new(value).unwrap()
    }

    fn institution() -> InstitutionId {
        InstitutionId::new("institution:city").unwrap()
    }

    fn jurisdiction() -> JurisdictionId {
        JurisdictionId::new("jurisdiction:city").unwrap()
    }

    fn rulebook() -> RulebookRef {
        RulebookRef {
            id: RulebookId::new("rulebook:administration:v1").unwrap(),
            version: "1.0.0".into(),
            digest: digest(7),
        }
    }

    fn case_id() -> AdministrativeCaseId {
        AdministrativeCaseId::new("case:1").unwrap()
    }

    fn profile_id() -> ProcedureProfileId {
        ProcedureProfileId::new("procedure:permit:v1").unwrap()
    }

    fn filed_case() -> AdministrativeCase {
        AdministrativeCase {
            protocol_version: PROTOCOL_VERSION.into(),
            id: case_id(),
            institution: institution(),
            jurisdiction: Some(jurisdiction()),
            rulebook: rulebook(),
            procedure_profile: profile_id(),
            subject_ref: "permit:parcel:42".into(),
            filed_by: principal("did:example:applicant"),
            filed_at_ms: 1_000,
            state: AdministrativeCaseState::Filed,
        }
    }

    fn decision_policy() -> AdministrativeDecisionPolicy {
        AdministrativeDecisionPolicy {
            protocol_version: PROTOCOL_VERSION.into(),
            procedure_profile: profile_id(),
            required_capability: CapabilityId::new("administration.decide").unwrap(),
            accepted_roles: vec![RoleId::new("role:permit-officer").unwrap()],
            authority_evidence: vec![],
        }
    }

    fn grant() -> AuthorityGrant {
        AuthorityGrant {
            protocol_version: INSTITUTIONAL_PROTOCOL_VERSION.into(),
            id: AuthorityGrantId::new("grant:permit-officer:1").unwrap(),
            holder: principal("did:example:officer"),
            institution: institution(),
            jurisdiction: Some(jurisdiction()),
            roles: vec![RoleId::new("role:permit-officer").unwrap()],
            capabilities: vec![CapabilityId::new("administration.decide").unwrap()],
            rulebook: rulebook(),
            sources: vec![AuthoritySourceRef {
                kind: AuthoritySourceKind::Credential,
                reference: "credential:permit-officer:1".into(),
                proof_ref: "proof:credential:1".into(),
            }],
            issued_at_ms: 500,
            expires_at_ms: 10_000,
            delegated_from: None,
            grant_proof_ref: "proof:grant:1".into(),
        }
    }

    fn exact_transitions() -> Vec<PreDecisionTransition> {
        vec![
            PreDecisionTransition {
                protocol_version: PROTOCOL_VERSION.into(),
                case_id: case_id(),
                next_state: PreDecisionState::EvidenceOpen {
                    opened_at_ms: 2_000,
                },
            },
            PreDecisionTransition {
                protocol_version: PROTOCOL_VERSION.into(),
                case_id: case_id(),
                next_state: PreDecisionState::ReadyForDecision { ready_at_ms: 3_000 },
            },
        ]
    }

    fn envelope() -> AdministrativeDecisionEnvelope {
        AdministrativeDecisionEnvelope {
            protocol_version: PROTOCOL_VERSION.into(),
            case_id: case_id(),
            jurisdiction: Some(jurisdiction()),
            decider: principal("did:example:officer"),
            authority_grant_id: AuthorityGrantId::new("grant:permit-officer:1").unwrap(),
            decision: Decision {
                protocol_version: INSTITUTIONAL_PROTOCOL_VERSION.into(),
                id: DecisionId::new("decision:1").unwrap(),
                institution: institution(),
                rulebook: rulebook(),
                subject_ref: "permit:parcel:42".into(),
                outcome_code: "granted".into(),
                reasons: vec!["requirements satisfied".into()],
                evidence: vec![],
                advisory_inputs: vec![],
                decided_at_ms: 4_000,
                decision_proof_ref: "proof:decision:1".into(),
            },
        }
    }

    #[test]
    fn forged_ready_snapshot_cannot_become_lineage_root() {
        let mut forged = filed_case();
        forged.state = AdministrativeCaseState::ReadyForDecision { ready_at_ms: 3_000 };

        assert_eq!(
            qualify_case_lineage(forged, &[], decision_policy()),
            Err(AdministrativeQualificationError::LineageRootMustBeFiled)
        );
    }

    #[test]
    fn evidence_open_snapshot_cannot_skip_filed_root() {
        let mut forged = filed_case();
        forged.state = AdministrativeCaseState::EvidenceOpen {
            opened_at_ms: 2_000,
        };

        assert_eq!(
            qualify_case_lineage(forged, &exact_transitions()[1..], decision_policy()),
            Err(AdministrativeQualificationError::LineageRootMustBeFiled)
        );
    }

    #[test]
    fn decision_time_policy_substitution_is_removed_from_public_api() {
        let lineage = qualify_case_lineage(filed_case(), &exact_transitions(), decision_policy())
            .expect("exact pre-decision lineage should qualify");

        assert_eq!(lineage.transition_count(), 2);
        assert_eq!(lineage.decision_policy().procedure_profile, profile_id());
        assert!(!lineage.grants_authority());
        assert!(!lineage.grants_external_effect_authority());
    }

    #[test]
    fn exact_replayed_lineage_can_qualify_and_issue_decision() {
        let lineage = qualify_case_lineage(filed_case(), &exact_transitions(), decision_policy())
            .expect("exact pre-decision lineage should qualify");
        let qualified = qualify_administrative_decision(&lineage, envelope(), &grant(), &[])
            .expect("competent decision should qualify");
        let issued = issue_qualified_decision(&lineage, qualified)
            .expect("qualified decision should issue semantically");

        assert!(matches!(
            &issued.successor_case().state,
            AdministrativeCaseState::DecisionIssued { decision_id, issued_at_ms }
                if decision_id == &DecisionId::new("decision:1").unwrap()
                    && *issued_at_ms == 4_000
        ));
        assert!(!issued.grants_external_effect_authority());
    }

    #[test]
    fn excessive_transition_fan_in_fails_closed() {
        let mut transitions = exact_transitions();
        transitions.push(PreDecisionTransition {
            protocol_version: PROTOCOL_VERSION.into(),
            case_id: case_id(),
            next_state: PreDecisionState::ReadyForDecision { ready_at_ms: 4_000 },
        });

        assert_eq!(
            qualify_case_lineage(filed_case(), &transitions, decision_policy()),
            Err(AdministrativeQualificationError::TooManyPreDecisionTransitions)
        );
    }
}
