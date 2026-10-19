//! Cross-module reference scenario for an adaptable home-design lifecycle.
//!
//! This is a synthetic software fixture, not a Johannesburg code pack or an
//! engineering model. It checks the evidence/workflow boundaries that a future
//! BIM and engineering adapter must preserve.

use home_compliance_core::evolution::{DependencyEdge, affected_cause_tree, affected_closure};
use home_compliance_core::regulatory::{
    LegalSourceStatus, NonBindingReason, RegulatorySource, RequirementAssertion,
    RequirementResolution, RequirementResolutionPolicy, SourceApplicability,
    evaluate_source_applicability, resolve_requirement,
};
use home_compliance_core::{
    AssessmentInput, Blocker, CheckDefinition, CheckOutcome, EvidenceRef, EvidenceVerification,
    Finding, GateStatus, RulePack, RulePackStatus, evaluate_review_gate,
    evaluate_verified_review_gate,
};

const JURISDICTION: &str = "ZA/Johannesburg/pilot-synthetic";

fn edge(input_id: &str, dependent_id: &str) -> DependencyEdge {
    DependencyEdge {
        input_id: input_id.into(),
        dependent_id: dependent_id.into(),
    }
}

/// IDs describe derived-from relationships, not UI display labels. A changed
/// input invalidates its downstream results but does not invalidate siblings.
fn reference_home_dependencies() -> Vec<DependencyEdge> {
    // Deliberately parse the fixture's simple, formatted edge tuples rather than
    // keeping a second, hand-maintained dependency graph in the test. The CI
    // validator parses the complete JSON and checks schema and graph semantics.
    let fixture = include_str!("../fixtures/johannesburg-adaptable-home-v1.json");
    let marker = "\"dependency_edges\": [";
    let start = fixture
        .find(marker)
        .expect("reference fixture must define dependency_edges");
    let section = &fixture[start + marker.len()..];
    let end = section
        .find("\n  ],")
        .expect("dependency_edges array must close at the expected fixture boundary");
    let section = &section[..end];

    let mut ids = Vec::new();
    for line in section.lines() {
        let trimmed = line.trim();
        if !trimmed.starts_with('"') {
            continue;
        }
        let rest = trimmed
            .strip_prefix('"')
            .expect("quoted edge identifier must start with a quote");
        let quote_end = rest
            .find('"')
            .expect("quoted edge identifier must terminate");
        let id = &rest[..quote_end];
        let trailing = rest[quote_end + 1..].trim();
        assert!(
            trailing.is_empty() || trailing == ",",
            "unexpected text after dependency identifier: {trailing}"
        );
        assert!(!id.trim().is_empty(), "dependency IDs must not be empty");
        ids.push(id.to_owned());
    }

    assert!(!ids.is_empty(), "dependency graph must not be empty");
    assert_eq!(
        ids.len() % 2,
        0,
        "dependency graph must contain complete input/dependent pairs"
    );
    ids.chunks_exact(2)
        .map(|pair| edge(&pair[0], &pair[1]))
        .collect()
}

#[test]
fn energy_rule_revision_invalidates_permit_basis_but_not_unrelated_structure_evidence() {
    let changed = vec!["rulepack:za-jhb:energy".to_owned()];
    let affected = affected_closure(&changed, &reference_home_dependencies())
        .expect("fixture dependency graph must be well formed");

    assert_eq!(
        affected,
        vec![
            "assessment:permit-package",
            "finding:energy-rule",
            "permit:submission",
            "rulepack:za-jhb:energy",
        ]
    );
    assert!(!affected.iter().any(|id| id == "finding:structure-review"));
    assert!(!affected.iter().any(|id| id == "model:floorplan"));
    assert!(!affected.iter().any(|id| id == "drawing:floorplan"));
    assert!(!affected.iter().any(|id| id == "export:ifc"));
}

#[test]
fn floorplan_revision_invalidates_derived_geometry_energy_and_submission_artifacts() {
    let changed = vec!["model:floorplan".to_owned()];
    let affected = affected_closure(&changed, &reference_home_dependencies())
        .expect("fixture dependency graph must be well formed");

    assert_eq!(
        affected,
        vec![
            "assessment:permit-package",
            "drawing:floorplan",
            "export:ifc",
            "finding:energy-rule",
            "finding:structure-review",
            "model:energy",
            "model:floorplan",
            "permit:submission",
        ]
    );
    assert!(!affected.iter().any(|id| id == "rulepack:za-jhb:energy"));
}

/// Read the simultaneous-change roots from the versioned fixture rather
/// than duplicating the root set in this Rust integration test.
fn simultaneous_change_roots() -> Vec<String> {
    let fixture = include_str!("../fixtures/johannesburg-adaptable-home-v1.json");
    let scenario_marker = "\"id\": \"simultaneous-floorplan-and-energy-rule-revision\"";
    let scenario_start = fixture
        .find(scenario_marker)
        .expect("fixture must contain the simultaneous-change scenario");
    let changed_ids_marker = "\"changed_ids\": [";
    let roots_start = scenario_start
        + fixture[scenario_start..]
            .find(changed_ids_marker)
            .expect("simultaneous-change scenario must declare changed_ids")
        + changed_ids_marker.len();
    let roots_section = &fixture[roots_start..];
    let roots_end = roots_section
        .find(']')
        .expect("changed_ids array must terminate");
    roots_section[..roots_end]
        .lines()
        .filter_map(|line| {
            let value = line.trim().trim_end_matches(',').trim_matches('"');
            (!value.is_empty()).then(|| value.to_owned())
        })
        .collect()
}

fn evidence(artifact_id: &str, digest_byte: char) -> EvidenceRef {
    EvidenceRef {
        artifact_id: artifact_id.into(),
        artifact_sha256: digest_byte.to_string().repeat(64),
    }
}

fn finding(check_id: &str, outcome: CheckOutcome) -> Finding {
    let evidence = if outcome == CheckOutcome::Unknown {
        Vec::new()
    } else {
        vec![evidence(
            if check_id == "energy-performance" {
                "report:energy-model"
            } else {
                "report:structure-review"
            },
            if check_id == "energy-performance" {
                'b'
            } else {
                'c'
            },
        )]
    };

    Finding {
        check_id: check_id.into(),
        rule_pack_id: "za-jhb-synthetic-home-pack".into(),
        rule_edition: "fixture-1".into(),
        outcome,
        rationale: format!("Synthetic fixture disposition for {check_id}"),
        evidence,
        not_applicable_justification: None,
        not_applicable_approved_by: None,
    }
}

fn assessment(energy_outcome: CheckOutcome) -> AssessmentInput {
    let required_checks = vec![
        CheckDefinition {
            id: "energy-performance".into(),
            mandatory: true,
        },
        CheckDefinition {
            id: "structure-review".into(),
            mandatory: true,
        },
    ];
    AssessmentInput {
        rule_pack: RulePack {
            id: "za-jhb-synthetic-home-pack".into(),
            jurisdiction: JURISDICTION.into(),
            edition: "fixture-1".into(),
            required_checks: required_checks.clone(),
            source_uri: "https://example.invalid/synthetic-home-rule-pack".into(),
            pack_sha256: "a".repeat(64),
            status: RulePackStatus::Active,
            effective_from_day: 1,
            effective_until_day_exclusive: None,
        },
        assessed_on_day: 200,
        required_checks,
        findings: vec![
            finding("energy-performance", energy_outcome),
            finding("structure-review", CheckOutcome::Pass),
        ],
    }
}

#[test]
fn complete_fixture_is_only_ready_for_professional_review() {
    // This fixture verifier is deliberately a test double. Production must
    // connect the trait to immutable artifact retrieval and actual SHA-256.
    let report =
        evaluate_verified_review_gate(&assessment(CheckOutcome::Pass), &|_: &EvidenceRef| {
            EvidenceVerification::Verified
        });
    assert_eq!(report.status, GateStatus::ReadyForProfessionalReview);
    assert!(report.blockers.is_empty());
}

#[test]
fn unknown_energy_result_blocks_even_when_structure_result_passes() {
    let report = evaluate_review_gate(&assessment(CheckOutcome::Unknown));
    assert_eq!(report.status, GateStatus::Blocked);
    assert!(
        report
            .blockers
            .contains(&Blocker::MandatoryCheckUnknown("energy-performance".into()))
    );
}

fn regulatory_source(
    source_id: &str,
    legal_status: LegalSourceStatus,
    value_edition: &str,
) -> RegulatorySource {
    RegulatorySource {
        source_id: source_id.into(),
        jurisdiction: JURISDICTION.into(),
        title: "Synthetic home energy requirement".into(),
        edition: value_edition.into(),
        source_uri: format!("https://example.invalid/{source_id}"),
        source_sha256: "d".repeat(64),
        applicability_scope: home_compliance_core::regulatory::JurisdictionApplicabilityScope::ExactJurisdiction,
        legal_status,
        published_on_day: 180,
        effective_from_day: if legal_status == LegalSourceStatus::InForce {
            Some(1)
        } else {
            None
        },
        effective_until_day_exclusive: None,
        checked_on_day: 200,
    }
}

#[test]
fn proposed_amendment_cannot_overwrite_current_binding_assertion() {
    let current = regulatory_source("current-rule", LegalSourceStatus::InForce, "current");
    let proposed = regulatory_source("future-amendment", LegalSourceStatus::Proposed, "draft");
    let current_status = evaluate_source_applicability(&current, JURISDICTION, 200, 200, 30);
    let proposed_status = evaluate_source_applicability(&proposed, JURISDICTION, 200, 200, 30);

    assert_eq!(current_status, SourceApplicability::Applicable);
    assert_eq!(
        proposed_status,
        SourceApplicability::NonBinding(NonBindingReason::Proposal)
    );

    let result = resolve_requirement(
        &[
            RequirementAssertion {
                requirement_id: "energy.minimum-performance".into(),
                source_id: current.source_id.clone(),
                clause_ref: "synthetic:current:1".into(),
                normalized_value: "threshold=baseline-A".into(),
                precedence_rank: Some(1),
                applicability: current_status,
            },
            RequirementAssertion {
                requirement_id: "energy.minimum-performance".into(),
                source_id: proposed.source_id,
                clause_ref: "synthetic:proposal:1".into(),
                normalized_value: "threshold=proposed-B".into(),
                precedence_rank: Some(2),
                applicability: proposed_status,
            },
        ],
        RequirementResolutionPolicy::RequireAgreement,
    );

    match result {
        RequirementResolution::Resolved {
            normalized_value,
            contributing_sources,
            ..
        } => {
            assert_eq!(normalized_value, "threshold=baseline-A");
            assert_eq!(contributing_sources.len(), 1);
            assert_eq!(contributing_sources[0].source_id, "current-rule");
        }
        other => panic!("proposal must not replace the binding fixture: {other:?}"),
    }
}

#[test]
fn fixture_has_a_shared_machine_readable_identity_contract() {
    let fixture = include_str!("../fixtures/johannesburg-adaptable-home-v1.json");
    for required_fragment in [
        "\"schema_version\": \"1.0.0\"",
        "\"status\": \"software_fixture_only_not_regulatory_or_engineering_evidence\"",
        "\"floorplan\": \"model:floorplan\"",
        "\"energy_rule_pack_logical_id\": \"rulepack:za-jhb:energy\"",
        "\"energy_finding\": \"finding:energy-rule\"",
    ] {
        assert!(
            fixture.contains(required_fragment),
            "fixture is missing expected contract fragment: {required_fragment}"
        );
    }
}

#[test]
fn simultaneous_floorplan_and_rule_change_invalidates_the_union_once() {
    let dependencies = reference_home_dependencies();
    let changed = simultaneous_change_roots();
    assert_eq!(
        changed,
        vec![
            "model:floorplan".to_owned(),
            "rulepack:za-jhb:energy".to_owned()
        ]
    );

    let affected = affected_closure(&changed, &dependencies)
        .expect("fixture simultaneous-change graph must be a valid DAG");
    assert_eq!(
        affected,
        vec![
            "assessment:permit-package".to_owned(),
            "drawing:floorplan".to_owned(),
            "export:ifc".to_owned(),
            "finding:energy-rule".to_owned(),
            "finding:structure-review".to_owned(),
            "model:energy".to_owned(),
            "model:floorplan".to_owned(),
            "permit:submission".to_owned(),
            "rulepack:za-jhb:energy".to_owned(),
        ]
    );

    let causes = affected_cause_tree(&changed, &dependencies)
        .expect("simultaneous-change graph must produce a compact cause tree");
    let energy_finding = causes
        .iter()
        .find(|item| item.artifact_id == "finding:energy-rule")
        .expect("energy finding must be invalidated");
    assert_eq!(
        energy_finding.cause_parent_id.as_deref(),
        Some("rulepack:za-jhb:energy")
    );
}

#[test]
fn ten_thousand_node_dependency_chain_has_complete_deterministic_closure() {
    let edge_count = 10_000;
    let dependencies: Vec<_> = (0..edge_count)
        .map(|index| {
            edge(
                &format!("artifact:{index}"),
                &format!("artifact:{}", index + 1),
            )
        })
        .collect();
    let changed = vec!["artifact:0".to_owned()];

    let affected = affected_closure(&changed, &dependencies)
        .expect("generated dependency chain must be well formed");

    assert_eq!(affected.len(), edge_count + 1);
    assert!(affected.contains(&"artifact:0".to_owned()));
    assert!(affected.contains(&format!("artifact:{edge_count}")));
    assert!(affected.windows(2).all(|pair| pair[0] <= pair[1]));

    let cause_tree = affected_cause_tree(&changed, &dependencies)
        .expect("the same well-formed chain must have a complete cause tree");
    assert_eq!(cause_tree.len(), edge_count + 1);
    assert_eq!(cause_tree[0].artifact_id, "artifact:0");
    assert_eq!(cause_tree[0].cause_parent_id, None);
    assert_eq!(
        cause_tree
            .iter()
            .find(|item| item.artifact_id == "artifact:10000")
            .unwrap()
            .cause_parent_id
            .as_deref(),
        Some("artifact:9999")
    );
    assert!(
        cause_tree
            .windows(2)
            .all(|pair| pair[0].artifact_id < pair[1].artifact_id)
    );
}
