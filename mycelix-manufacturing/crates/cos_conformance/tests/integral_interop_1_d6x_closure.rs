//! Integral OAD -> COS -> material D6X closure conformance.
//!
//! Status: ReferenceModelOnly.
//!
//! This fixture is intentionally a semantic mapping test, not an Integral
//! runtime adapter. It proves that a small OAD/COS dependency graph can cross
//! the D6X boundary while candidate noise and runtime-independent evidence stay
//! outside semantic closure identity.

use cos_conformance::canonical_derivation_receipt::{
    canonical_sha256, DerivationProfileV1, QualifiedEdgeV1, QualifiedNodeV1,
    QualifiedProjectionV1, SemanticEnvironmentV1, D6S_CLAIM_CEILING,
};
use cos_conformance::evidence_claim_graph::{ClaimGraphEdgeKindV1, ClaimGraphNodeKindV1};
use cos_conformance::layered_derivation_commitment::{
    DerivationCommitmentV1, InputCommitmentV1,
};
use cos_conformance::qualified_dependency_closure_d6x::{
    compute_dependency_closure, DependencyClosureProfileV1, DependencyCurrentnessV1,
    DependencyRuleV1, DependencyClosureStatusV1,
};
use serde_json::Value;
use std::collections::{BTreeMap, BTreeSet};

fn fixture_value() -> Value {
    serde_json::from_str(include_str!(
        "../testdata/integral_interop_1_design.json"
    ))
    .expect("Integral interop fixture must be valid JSON")
}

fn environment() -> SemanticEnvironmentV1 {
    SemanticEnvironmentV1 {
        semantic_profile_id: "integral-interop-1".into(),
        semantic_profile_version: "1".into(),
        current_frontier_root: Some("integral-frontier-1".into()),
        d6p_eligibility_context_root: None,
        d6n_observer_context_root: None,
        d6o_lifecycle_context_root: None,
        membership_authority_scope_root: None,
        dependency_snapshot_root: Some("integral-snapshot-1".into()),
        historical_cutoff: None,
        policy_version: "integral-interop-1".into(),
        claim_ceiling: D6S_CLAIM_CEILING.into(),
    }
}

fn derivation_profile() -> DerivationProfileV1 {
    DerivationProfileV1 {
        profile_id: "integral-d6w-reference-1".into(),
        version: "1".into(),
        rule_ids: ["integral-interop-1".into()].into_iter().collect(),
        permits_recursive_fixpoint: false,
        claim_ceiling: D6S_CLAIM_CEILING.into(),
    }
}

fn node(id: &str, kind: ClaimGraphNodeKindV1, content_commitment: String) -> QualifiedNodeV1 {
    QualifiedNodeV1 {
        node_id: id.into(),
        kind,
        node_commitment: canonical_sha256(
            "integral-interop-1-node",
            &serde_json::json!({
                "id": id,
                "content_commitment": content_commitment,
                "kind": format!("{kind:?}"),
            }),
        ),
        content_commitment,
        historical_only: false,
        current_frontier_root: Some("integral-frontier-1".into()),
        claim_ceiling: D6S_CLAIM_CEILING.into(),
    }
}

fn edge(id: &str, from: &str, to: &str, kind: ClaimGraphEdgeKindV1) -> QualifiedEdgeV1 {
    QualifiedEdgeV1 {
        edge_id: id.into(),
        from_node_id: from.into(),
        to_node_id: to.into(),
        kind,
        edge_commitment: canonical_sha256(
            "integral-interop-1-edge",
            &serde_json::json!({
                "id": id,
                "from": from,
                "to": to,
                "kind": format!("{kind:?}"),
            }),
        ),
        claim_ceiling: D6S_CLAIM_CEILING.into(),
    }
}

fn projection(with_noise: bool, value: &Value) -> (
    QualifiedProjectionV1,
    SemanticEnvironmentV1,
    DerivationProfileV1,
) {
    let env = environment();
    let derivation = derivation_profile();

    let design_commitment = canonical_sha256("integral-interop-1-design", value);
    let production_steps = value["design_version"]["parameters"]["production_steps"].clone();
    let bom = value["design_version"]["parameters"]["bill_of_materials_kg"].clone();

    let task_value = serde_json::json!({
        "design_version_id": value["design_version"]["id"],
        "production_steps": production_steps,
    });
    let task_commitment = canonical_sha256("integral-interop-1-cos-task", &task_value);

    let mut nodes = BTreeMap::new();
    nodes.insert(
        "oad:design-water-purifier:v1".into(),
        node(
            "oad:design-water-purifier:v1",
            ClaimGraphNodeKindV1::Statement,
            design_commitment,
        ),
    );
    nodes.insert(
        "cos:task:water-purifier:v1".into(),
        node(
            "cos:task:water-purifier:v1",
            ClaimGraphNodeKindV1::Evidence,
            task_commitment,
        ),
    );

    for material in ["silicone", "stainless-steel"] {
        let kg = bom[material].clone();
        let material_value = serde_json::json!({
            "design_version_id": value["design_version"]["id"],
            "material": material,
            "kg": kg,
        });
        let id = format!("oad:material:{material}");
        nodes.insert(
            id.clone(),
            node(
                &id,
                ClaimGraphNodeKindV1::Evidence,
                canonical_sha256("integral-interop-1-material", &material_value),
            ),
        );
    }

    let mut edges = BTreeMap::new();
    edges.insert(
        "oad->cos:production-plan".into(),
        edge(
            "oad->cos:production-plan",
            "oad:design-water-purifier:v1",
            "cos:task:water-purifier:v1",
            ClaimGraphEdgeKindV1::DerivedFrom,
        ),
    );
    for material in ["silicone", "stainless-steel"] {
        let id = format!("cos->material:{material}");
        edges.insert(
            id.clone(),
            edge(
                &id,
                "cos:task:water-purifier:v1",
                &format!("oad:material:{material}"),
                ClaimGraphEdgeKindV1::Supports,
            ),
        );
    }

    if with_noise {
        nodes.insert(
            "frs:observation:noise".into(),
            node(
                "frs:observation:noise",
                ClaimGraphNodeKindV1::Source,
                canonical_sha256(
                    "integral-interop-1-frs-noise",
                    &serde_json::json!({"signal": "candidate-only"}),
                ),
            ),
        );
        nodes.insert(
            "cos:candidate:unused".into(),
            node(
                "cos:candidate:unused",
                ClaimGraphNodeKindV1::Evidence,
                canonical_sha256(
                    "integral-interop-1-cos-noise",
                    &serde_json::json!({"candidate": "unused"}),
                ),
            ),
        );
        edges.insert(
            "frs->cos:candidate".into(),
            edge(
                "frs->cos:candidate",
                "frs:observation:noise",
                "cos:candidate:unused",
                ClaimGraphEdgeKindV1::Provenance,
            ),
        );
    }

    let projection = QualifiedProjectionV1 {
        projection_id: "integral-interop-1-projection".into(),
        projection_version: "1".into(),
        canonicalization_version: "D6S-CANON-1".into(),
        source_dkg_snapshot_commitment: "integral-snapshot-1".into(),
        nodes,
        edges,
        d6p_current_receipt_commitments: BTreeSet::new(),
        d6n_context_commitment: None,
        d6o_context_commitment: None,
        semantic_environment_commitment: env.commitment(),
        derivation_profile_commitment: derivation.commitment(),
        claim_ceiling: D6S_CLAIM_CEILING.into(),
    };

    (projection, env, derivation)
}

fn closure_profile() -> DependencyClosureProfileV1 {
    let rules = [
        DependencyRuleV1 {
            edge_kind: ClaimGraphEdgeKindV1::DerivedFrom,
            from_kind: Some(ClaimGraphNodeKindV1::Statement),
            to_kind: Some(ClaimGraphNodeKindV1::Evidence),
            currentness: DependencyCurrentnessV1::Any,
        },
        DependencyRuleV1 {
            edge_kind: ClaimGraphEdgeKindV1::Supports,
            from_kind: Some(ClaimGraphNodeKindV1::Evidence),
            to_kind: Some(ClaimGraphNodeKindV1::Evidence),
            currentness: DependencyCurrentnessV1::Any,
        },
    ];

    DependencyClosureProfileV1 {
        profile_id: "integral-oad-cos-material-closure".into(),
        version: "1".into(),
        root_node_ids: ["oad:design-water-purifier:v1".into()]
            .into_iter()
            .collect(),
        required_node_ids: BTreeSet::new(),
        required_d6p_receipt_commitments: BTreeSet::new(),
        rules: rules.into_iter().collect(),
        excluded_boundary_policy:
            "Only selected OAD->COS and COS->material semantic edges expand the closure; FRS/provenance candidates are excluded."
                .into(),
        max_nodes: 16,
        max_edges: 16,
        claim_ceiling: D6S_CLAIM_CEILING.into(),
    }
}

#[test]
fn integral_oad_cos_closure_is_complete_and_typed() {
    let value = fixture_value();
    let (projection, env, derivation) = projection(false, &value);
    let closure = compute_dependency_closure(
        &projection,
        &env,
        &derivation,
        &closure_profile(),
    )
    .expect("reference projection should produce a closure");

    assert_eq!(closure.status, DependencyClosureStatusV1::Complete);
    assert_eq!(closure.included_node_ids.len(), 4);
    assert_eq!(closure.included_edges.len(), 3);
    assert!(closure.included_node_ids.contains("oad:design-water-purifier:v1"));
    assert!(closure.included_node_ids.contains("cos:task:water-purifier:v1"));
    assert!(closure.included_node_ids.contains("oad:material:silicone"));
    assert!(closure.included_node_ids.contains("oad:material:stainless-steel"));
    assert!(closure.valid());
}

#[test]
fn irrelevant_frs_and_candidate_cos_material_do_not_contaminate_closure() {
    let value = fixture_value();
    let (clean, env, derivation) = projection(false, &value);
    let (noisy, _, _) = projection(true, &value);

    let clean_closure =
        compute_dependency_closure(&clean, &env, &derivation, &closure_profile()).unwrap();
    let noisy_closure =
        compute_dependency_closure(&noisy, &env, &derivation, &closure_profile()).unwrap();

    assert_ne!(
        clean_closure.commitment, noisy_closure.commitment,
        "audit certificate remains bound to the candidate projection"
    );
    assert_eq!(
        clean_closure.closure_identity_commitment,
        noisy_closure.closure_identity_commitment,
        "irrelevant candidate material must not change semantic closure identity"
    );

    let clean_input = InputCommitmentV1::from_projection(&clean, &env, &clean_closure).unwrap();
    let noisy_input = InputCommitmentV1::from_projection(&noisy, &env, &noisy_closure).unwrap();
    assert_eq!(
        clean_input.commitment, noisy_input.commitment,
        "D6W input must inherit the candidate-independent closure boundary"
    );
}

#[test]
fn selected_design_change_propagates_to_d6x_and_d6w() {
    let mut changed = fixture_value();
    changed["design_version"]["parameters"]["production_steps"][1]["estimated_hours"] =
        Value::from(3);

    let (baseline_projection, env, derivation) = projection(false, &fixture_value());
    let (changed_projection, _, _) = projection(false, &changed);

    let baseline_closure =
        compute_dependency_closure(&baseline_projection, &env, &derivation, &closure_profile())
            .unwrap();
    let changed_closure =
        compute_dependency_closure(&changed_projection, &env, &derivation, &closure_profile())
            .unwrap();

    assert_ne!(
        baseline_closure.closure_identity_commitment,
        changed_closure.closure_identity_commitment,
        "selected OAD production semantics must change closure identity"
    );

    let baseline_input =
        InputCommitmentV1::from_projection(&baseline_projection, &env, &baseline_closure).unwrap();
    let changed_input =
        InputCommitmentV1::from_projection(&changed_projection, &env, &changed_closure).unwrap();

    assert_ne!(baseline_input.commitment, changed_input.commitment);

    let baseline_derivation =
        DerivationCommitmentV1::new(&baseline_input, &derivation, None).unwrap();
    let changed_derivation =
        DerivationCommitmentV1::new(&changed_input, &derivation, None).unwrap();

    assert_ne!(
        baseline_derivation.commitment, changed_derivation.commitment,
        "D6W derivation identity must propagate a selected semantic mutation"
    );
}

#[test]
fn runtime_resolution_is_not_needed_for_semantic_identity() {
    let value = fixture_value();
    let (projection, env, derivation) = projection(false, &value);
    let closure =
        compute_dependency_closure(&projection, &env, &derivation, &closure_profile()).unwrap();

    assert!(
        closure.resolution_evidence.is_empty(),
        "semantic closure construction must not invent runtime retrieval evidence"
    );
    assert_eq!(
        closure.closure_identity_commitment,
        closure.closure_identity(),
        "semantic identity is reproducible without runtime addresses"
    );
}

#[test]
fn projection_metadata_and_runtime_resolution_evidence_are_not_semantic_identity() {
    let value = fixture_value();
    let (mut projection, env, derivation) = projection(false, &value);
    let baseline =
        compute_dependency_closure(&projection, &env, &derivation, &closure_profile()).unwrap();

    projection.projection_id = "integral-interop-1-projection-renamed".into();
    let renamed =
        compute_dependency_closure(&projection, &env, &derivation, &closure_profile()).unwrap();
    assert_eq!(baseline.closure_identity_commitment, renamed.closure_identity_commitment);

    let mut observed = baseline.clone();
    observed.resolution_evidence.insert(
        "oad:design-water-purifier:v1".into(),
        cos_conformance::qualified_dependency_closure_d6x::SemanticDependencyResolutionEvidenceV1 {
            retrieval_reference: Some("runtime://resolver/42".into()),
            observed_commitment: Some("observed-design-commitment".into()),
            qualification_context_commitment: Some("qualification-context-1".into()),
        },
    );

    assert_eq!(
        baseline.closure_identity_commitment,
        observed.closure_identity_commitment,
        "runtime retrieval evidence must not contaminate semantic identity"
    );
    assert_ne!(
        baseline.commitment,
        observed.recompute(),
        "the audit certificate must still record that runtime evidence changed"
    );
}

