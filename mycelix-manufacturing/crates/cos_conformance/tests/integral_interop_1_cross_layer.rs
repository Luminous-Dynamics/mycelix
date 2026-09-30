//! Cross-layer Integral interoperability conformance.
//!
//! ReferenceModelOnly: one mutation is classified at the OAD semantic boundary
//! and then checked through D6X closure and D6W input/derivation commitments.
//! This intentionally tests propagation properties, not an Integral wire schema.

use cos_conformance::canonical_derivation_receipt::{
    canonical_sha256, DerivationProfileV1, QualifiedEdgeV1, QualifiedNodeV1,
    QualifiedProjectionV1, SemanticEnvironmentV1, D6S_CLAIM_CEILING,
};
use cos_conformance::evidence_claim_graph::{ClaimGraphEdgeKindV1, ClaimGraphNodeKindV1};
use cos_conformance::integral_interop::{
    selected_oad_design_semantic_commitment_checked, validate_selected_oad_design_semantics,
};
use cos_conformance::layered_derivation_commitment::{
    DerivationCommitmentV1, InputCommitmentV1,
};
use cos_conformance::qualified_dependency_closure_d6x::{
    compute_dependency_closure, DependencyClosureProfileV1, DependencyCurrentnessV1,
    DependencyRuleV1, DependencyClosureStatusV1,
};
use serde_json::Value;
use std::collections::{BTreeMap, BTreeSet};

fn fixture() -> Value {
    serde_json::from_str(include_str!(
        "../testdata/integral_interop_1_design.json"
    ))
    .expect("Integral fixture must be valid JSON")
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

fn projection(
    value: &Value,
    with_candidate_noise: bool,
    d6p_receipt: bool,
) -> (QualifiedProjectionV1, SemanticEnvironmentV1, DerivationProfileV1) {
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
        let material_value = serde_json::json!({
            "design_version_id": value["design_version"]["id"],
            "material": material,
            "kg": bom[material],
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

    if with_candidate_noise {
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

    let d6p_current_receipt_commitments = if d6p_receipt {
        ["integral-d6p-receipt-1".into()].into_iter().collect()
    } else {
        BTreeSet::new()
    };

    let projection = QualifiedProjectionV1 {
        projection_id: "integral-interop-1-projection".into(),
        projection_version: "1".into(),
        canonicalization_version: "D6S-CANON-1".into(),
        source_dkg_snapshot_commitment: "integral-snapshot-1".into(),
        nodes,
        edges,
        d6p_current_receipt_commitments,
        d6n_context_commitment: None,
        d6o_context_commitment: None,
        semantic_environment_commitment: env.commitment(),
        derivation_profile_commitment: derivation.commitment(),
        claim_ceiling: D6S_CLAIM_CEILING.into(),
    };

    (projection, env, derivation)
}

fn closure_profile(require_d6p_receipt: bool) -> DependencyClosureProfileV1 {
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
        required_d6p_receipt_commitments: if require_d6p_receipt {
            ["integral-d6p-receipt-1".into()].into_iter().collect()
        } else {
            BTreeSet::new()
        },
        rules: rules.into_iter().collect(),
        excluded_boundary_policy:
            "Only selected OAD->COS and COS->material semantic edges expand the closure; FRS/provenance candidates are excluded."
                .into(),
        max_nodes: 16,
        max_edges: 16,
        claim_ceiling: D6S_CLAIM_CEILING.into(),
    }
}

fn evaluate(
    value: &Value,
    with_candidate_noise: bool,
    d6p_receipt: bool,
    require_d6p_receipt: bool,
) -> Option<(String, String, Option<InputCommitmentV1>, Option<String>)> {
    let selected = selected_oad_design_semantic_commitment_checked(value).ok()?;
    let (projection, env, derivation) =
        projection(value, with_candidate_noise, d6p_receipt);
    let closure = compute_dependency_closure(
        &projection,
        &env,
        &derivation,
        &closure_profile(require_d6p_receipt),
    )?;
    let closure_id = closure.closure_identity_commitment.clone();
    let input = InputCommitmentV1::from_projection(&projection, &env, &closure);
    let derivation_id = input
        .as_ref()
        .and_then(|input| DerivationCommitmentV1::new(input, &derivation, None))
        .map(|d| d.commitment);

    Some((selected, closure_id, input, derivation_id))
}



fn path_tokens(path: &str) -> Vec<String> {
    let mut tokens = Vec::new();
    for segment in path.split('.') {
        let mut rest = segment;
        while let Some(open) = rest.find('[') {
            if open > 0 {
                tokens.push(rest[..open].to_owned());
            }
            let close = rest[open..]
                .find(']')
                .expect("array path token must close");
            tokens.push(rest[open + 1..open + close].to_owned());
            rest = &rest[open + close + 1..];
        }
        if !rest.is_empty() {
            tokens.push(rest.to_owned());
        }
    }
    tokens
}

fn set_path(root: &mut Value, path: &str, replacement: Value) {
    let tokens = path_tokens(path);
    assert!(!tokens.is_empty());

    fn descend(current: &mut Value, tokens: &[String], replacement: Value) {
        if tokens.len() == 1 {
            match current {
                Value::Object(map) => {
                    map.insert(tokens[0].clone(), replacement);
                }
                Value::Array(values) => {
                    let index: usize = tokens[0].parse().expect("array index");
                    values[index] = replacement;
                }
                _ => panic!("cannot descend into scalar"),
            }
            return;
        }

        match current {
            Value::Object(map) => descend(
                map.get_mut(&tokens[0]).expect("object path component"),
                &tokens[1..],
                replacement,
            ),
            Value::Array(values) => {
                let index: usize = tokens[0].parse().expect("array index");
                descend(&mut values[index], &tokens[1..], replacement);
            }
            _ => panic!("cannot descend into scalar"),
        }
    }

    descend(root, &tokens, replacement);
}

#[test]
fn declarative_cross_layer_corpus_is_self_describing_and_executable() {
    let corpus: Value = serde_json::from_str(include_str!(
        "../testdata/integral_interop_1_cross_layer_vectors.json"
    ))
    .expect("cross-layer corpus must be valid JSON");

    assert_eq!(corpus["profile"], "integral-interop-1");
    assert_eq!(
        corpus["projection_version"],
        "integral-interop-1-design-semantic-v1"
    );
    assert_eq!(corpus["classification"], "cross-layer-propagation-v1");

    let baseline = fixture();
    let baseline_eval =
        evaluate(&baseline, false, false, false).expect("baseline must resolve");

    for vector in corpus["vectors"].as_array().expect("vectors array") {
        let id = vector["id"].as_str().expect("vector id");
        let operation = vector["operation"].as_str().expect("vector operation");
        let expected = vector["expected"].as_str().expect("vector expected");

        assert!(
            matches!(
                expected,
                "identity-preserving"
                    | "identity-changing"
                    | "invalid-no-identity"
                    | "blocked-no-d6w"
                    | "dependency-satisfied"
                    | "audit-only"
            ),
            "{id}: unknown classification"
        );

        if operation == "runtime-evidence" {
            assert_eq!(expected, "audit-only", "{id}");
            continue;
        }

        let mut mutated = baseline.clone();
        let mut with_candidate_noise = false;
        let mut d6p_receipt = false;
        let mut require_d6p_receipt = false;

        match operation {
            "set" => set_path(&mut mutated, vector["path"].as_str().unwrap(), vector["value"].clone()),
            "candidate-noise" => with_candidate_noise = true,
            "require-d6p-receipt" => {
                require_d6p_receipt = true;
                d6p_receipt = vector["value"].as_bool().unwrap();
            }
            other => panic!("{id}: unsupported operation {other}"),
        }

        match expected {
            "identity-preserving" => {
                let actual = evaluate(
                    &mutated,
                    with_candidate_noise,
                    d6p_receipt,
                    require_d6p_receipt,
                )
                .expect(id);
                assert_eq!(baseline_eval.0, actual.0, "{id}: OAD identity changed");
                assert_eq!(baseline_eval.1, actual.1, "{id}: D6X identity changed");
                assert_eq!(
                    baseline_eval.2.as_ref().map(|v| &v.commitment),
                    actual.2.as_ref().map(|v| &v.commitment),
                    "{id}: D6W input changed"
                );
                assert_eq!(baseline_eval.3, actual.3, "{id}: D6W derivation changed");
            }
            "identity-changing" => {
                let actual = evaluate(
                    &mutated,
                    with_candidate_noise,
                    d6p_receipt,
                    require_d6p_receipt,
                )
                .expect(id);
                assert_ne!(baseline_eval.0, actual.0, "{id}: OAD identity did not change");
                assert_ne!(baseline_eval.1, actual.1, "{id}: D6X identity did not change");
                assert_ne!(
                    baseline_eval.2.as_ref().map(|v| &v.commitment),
                    actual.2.as_ref().map(|v| &v.commitment),
                    "{id}: D6W input did not change"
                );
                assert_ne!(baseline_eval.3, actual.3, "{id}: D6W derivation did not change");
            }
            "invalid-no-identity" => {
                assert!(
                    validate_selected_oad_design_semantics(&mutated).is_err(),
                    "{id}: validation must reject"
                );
                assert!(evaluate(&mutated, false, false, false).is_none(), "{id}");
            }
            "blocked-no-d6w" => {
                let actual = evaluate(&mutated, false, false, true).expect(id);
                assert_eq!(actual.2, None, "{id}: blocked D6X entered D6W");
            }
            "dependency-satisfied" => {
                let actual = evaluate(&mutated, false, true, true).expect(id);
                assert_ne!(baseline_eval.1, actual.1, "{id}: D6X did not bind receipt");
                assert!(actual.2.is_some(), "{id}: D6W input missing");
                assert!(actual.3.is_some(), "{id}: D6W derivation missing");
            }
            "audit-only" => unreachable!("handled above"),
            _ => unreachable!(),
        }
    }
}

#[test]
fn cross_layer_mutation_matrix_is_executable() {
    let baseline = fixture();
    let baseline_eval = evaluate(&baseline, false, false, false).expect("baseline must resolve");
    assert!(validate_selected_oad_design_semantics(&baseline).is_ok());
    assert!(baseline_eval.2.is_some());

    let mut metadata = baseline.clone();
    metadata["certification"]["documentation_bundle_uri"] =
        Value::from("urn:integral:bundle:changed");
    let metadata_eval = evaluate(&metadata, false, false, false).expect("metadata must resolve");
    assert_eq!(baseline_eval.0, metadata_eval.0);
    assert_eq!(baseline_eval.1, metadata_eval.1);
    assert_eq!(
        baseline_eval.2.as_ref().unwrap().commitment,
        metadata_eval.2.as_ref().unwrap().commitment
    );
    assert_eq!(baseline_eval.3, metadata_eval.3);

    let mut selected = baseline.clone();
    selected["design_version"]["parameters"]["production_steps"][1]["estimated_hours"] =
        Value::from(3);
    let selected_eval = evaluate(&selected, false, false, false).expect("selected mutation must resolve");
    assert_ne!(baseline_eval.0, selected_eval.0);
    assert_ne!(baseline_eval.1, selected_eval.1);
    assert_ne!(
        baseline_eval.2.as_ref().unwrap().commitment,
        selected_eval.2.as_ref().unwrap().commitment
    );
    assert_ne!(baseline_eval.3, selected_eval.3);

    let noisy_eval = evaluate(&baseline, true, false, false).expect("candidate noise must resolve");
    assert_eq!(baseline_eval.0, noisy_eval.0);
    assert_eq!(baseline_eval.1, noisy_eval.1);
    assert_eq!(
        baseline_eval.2.as_ref().unwrap().commitment,
        noisy_eval.2.as_ref().unwrap().commitment
    );
    assert_eq!(baseline_eval.3, noisy_eval.3);

    let mut invalid = baseline.clone();
    invalid["design_version"]["parameters"]["bill_of_materials_kg"]["silicone"] =
        serde_json::json!(0.25);
    assert!(validate_selected_oad_design_semantics(&invalid).is_err());
    assert!(evaluate(&invalid, false, false, false).is_none());

    let missing_receipt = evaluate(&baseline, false, false, true).expect("blocked D6X must still certify its boundary");
    assert_eq!(
        missing_receipt.2, None,
        "blocked D6X closure must not enter D6W"
    );

    let present_receipt =
        evaluate(&baseline, false, true, true).expect("required D6P receipt should unblock D6X");
    assert_ne!(
        missing_receipt.1, present_receipt.1,
        "adding a required D6P receipt must change semantic closure identity"
    );
    assert!(present_receipt.2.is_some());
    assert!(present_receipt.3.is_some());
}

#[test]
fn runtime_resolution_evidence_does_not_propagate_into_identity_layers() {
    let baseline = fixture();
    let (projection, env, derivation) = projection(&baseline, false, false);
    let closure = compute_dependency_closure(
        &projection,
        &env,
        &derivation,
        &closure_profile(false),
    )
    .expect("baseline closure");

    let input = InputCommitmentV1::from_projection(&projection, &env, &closure)
        .expect("baseline D6W input");
    let mut observed = closure.clone();
    observed.resolution_evidence.insert(
        cos_conformance::qualified_dependency_closure_d6x::SemanticDependencyReferenceV1::node(
            "oad:design-water-purifier:v1",
            Some(
                projection
                    .nodes
                    .get("oad:design-water-purifier:v1")
                    .unwrap()
                    .node_commitment
                    .clone(),
            ),
        ),
        cos_conformance::qualified_dependency_closure_d6x::SemanticDependencyResolutionEvidenceV1 {
            retrieval_reference: Some("runtime://resolver/42".into()),
            observed_commitment: Some(
                projection
                    .nodes
                    .get("oad:design-water-purifier:v1")
                    .unwrap()
                    .node_commitment
                    .clone(),
            ),
            qualification_context_commitment: Some("qualification-context-1".into()),
        },
    );

    assert_eq!(closure.closure_identity_commitment, observed.closure_identity_commitment);
    assert_ne!(closure.commitment, observed.recompute());

    let observed_input = InputCommitmentV1::from_projection(&projection, &env, &observed)
        .expect("runtime evidence must not block a complete closure");
    assert_eq!(input.commitment, observed_input.commitment);
}
