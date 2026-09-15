// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

use mycelix_institutional_core::{
    AuthorityDecision, AuthorityGrant, AuthorityGrantId, AuthorityRequirement, AuthoritySourceKind,
    AuthoritySourceRef, CapabilityId, Digest32, EvidenceId, EvidenceRef, EvidenceRequirement,
    InstitutionId, JurisdictionId, PROTOCOL_VERSION, PrincipalId, RoleId, RulebookId, RulebookRef,
    evaluate_authority,
};

fn d(byte: u8) -> Digest32 {
    Digest32([byte; 32])
}

fn principal(value: &str) -> PrincipalId {
    PrincipalId::new(value).unwrap()
}

fn institution(value: &str) -> InstitutionId {
    InstitutionId::new(value).unwrap()
}

fn rulebook() -> RulebookRef {
    RulebookRef {
        id: RulebookId::new("rulebook:test:v1").unwrap(),
        version: "1.0.0".into(),
        digest: d(1),
    }
}

fn grant() -> AuthorityGrant {
    AuthorityGrant {
        protocol_version: PROTOCOL_VERSION.into(),
        id: AuthorityGrantId::new("grant:test:1").unwrap(),
        holder: principal("did:example:holder"),
        institution: institution("institution:test"),
        jurisdiction: Some(JurisdictionId::new("jurisdiction:test").unwrap()),
        roles: vec![RoleId::new("role:member").unwrap()],
        capabilities: vec![CapabilityId::new("capability:act").unwrap()],
        rulebook: rulebook(),
        sources: vec![AuthoritySourceRef {
            kind: AuthoritySourceKind::GovernanceDecision,
            reference: "governance:decision:1".into(),
            proof_ref: "proof:decision:1".into(),
        }],
        issued_at_ms: 1_000,
        expires_at_ms: 10_000,
        delegated_from: None,
        grant_proof_ref: "proof:grant:1".into(),
    }
}

fn requirement() -> AuthorityRequirement {
    AuthorityRequirement {
        institution: institution("institution:test"),
        jurisdiction: Some(JurisdictionId::new("jurisdiction:test").unwrap()),
        required_capabilities: vec![CapabilityId::new("capability:act").unwrap()],
        accepted_roles: vec![RoleId::new("role:member").unwrap()],
        evidence: vec![],
        rulebook: rulebook(),
    }
}

fn denial_code(result: AuthorityDecision) -> String {
    match result {
        AuthorityDecision::Deny(denial) => denial.reason_code,
        other => panic!("expected denial, got {other:?}"),
    }
}

#[test]
fn direct_construction_cannot_smuggle_malformed_grant_capability() {
    let mut candidate = grant();
    candidate.capabilities = vec![CapabilityId("   ".into())];
    assert_eq!(
        denial_code(evaluate_authority(&candidate, &requirement(), &[], 2_000)),
        "invalid_grant"
    );
}

#[test]
fn direct_construction_cannot_smuggle_malformed_grant_role() {
    let mut candidate = grant();
    candidate.roles = vec![RoleId("\n".into())];
    assert_eq!(
        denial_code(evaluate_authority(&candidate, &requirement(), &[], 2_000)),
        "invalid_grant"
    );
}

#[test]
fn direct_construction_cannot_smuggle_malformed_delegated_from_id() {
    let mut candidate = grant();
    candidate.delegated_from = Some(AuthorityGrantId("\t".into()));
    assert_eq!(
        denial_code(evaluate_authority(&candidate, &requirement(), &[], 2_000)),
        "invalid_grant"
    );
}

#[test]
fn direct_construction_cannot_smuggle_malformed_required_capability() {
    let mut required = requirement();
    required.required_capabilities = vec![CapabilityId("\n".into())];
    assert_eq!(
        denial_code(evaluate_authority(&grant(), &required, &[], 2_000)),
        "invalid_requirement"
    );
}

#[test]
fn direct_construction_cannot_smuggle_malformed_accepted_role() {
    let mut required = requirement();
    required.accepted_roles = vec![RoleId("   ".into())];
    assert_eq!(
        denial_code(evaluate_authority(&grant(), &required, &[], 2_000)),
        "invalid_requirement"
    );
}

#[test]
fn direct_construction_cannot_smuggle_malformed_requirement_scope() {
    let mut required = requirement();
    required.institution = InstitutionId("\n".into());
    assert_eq!(
        denial_code(evaluate_authority(&grant(), &required, &[], 2_000)),
        "invalid_requirement"
    );

    let mut required = requirement();
    required.jurisdiction = Some(JurisdictionId("\t".into()));
    assert_eq!(
        denial_code(evaluate_authority(&grant(), &required, &[], 2_000)),
        "invalid_requirement"
    );
}

#[test]
fn malformed_accepted_issuer_makes_requirement_invalid() {
    let mut required = requirement();
    required.evidence = vec![EvidenceRequirement {
        evidence_type: "credential:test".into(),
        accepted_issuers: vec![PrincipalId("\n".into())],
    }];
    assert_eq!(
        denial_code(evaluate_authority(&grant(), &required, &[], 2_000)),
        "invalid_requirement"
    );
}

#[test]
fn malformed_evidence_issuer_cannot_satisfy_open_issuer_requirement() {
    let mut required = requirement();
    required.evidence = vec![EvidenceRequirement {
        evidence_type: "credential:test".into(),
        accepted_issuers: vec![],
    }];
    let evidence = vec![EvidenceRef {
        id: EvidenceId::new("evidence:test:1").unwrap(),
        evidence_type: "credential:test".into(),
        issuer: Some(PrincipalId("\n".into())),
        digest: Some(d(2)),
        observed_at_ms: 1_500,
        proof_ref: Some("proof:evidence:1".into()),
    }];
    assert!(matches!(
        evaluate_authority(&grant(), &required, &evidence, 2_000),
        AuthorityDecision::NeedsEvidence(_)
    ));
}

#[test]
fn constructor_valid_authority_still_allows() {
    assert!(matches!(
        evaluate_authority(&grant(), &requirement(), &[], 2_000),
        AuthorityDecision::Allow(_)
    ));
}
