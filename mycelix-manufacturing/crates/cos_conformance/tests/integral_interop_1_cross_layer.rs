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
    DependencyRuleV1, DependencyClosureStatusV1, SemanticDependencyReferenceV1,
    SemanticDependencyResolutionEvidenceV1,
};
use serde_json::Value;
use std::collections::{BTreeMap, BTreeSet};

fn fixture() -> Value {
    serde_json::from_str(include_str!(
        "../testdata/integral_interop_1_design.json"
    ))
    .expect("Integral fixture must be valid JSON")
}

fn snapshot_commitment() -> String {
    canonical_sha256(
        "integral-interop-1-snapshot",
        &serde_json::json!({
            "fixture": "Integral-Interop-1",
            "version": 1,
            "snapshot": "integral-snapshot-1",
        }),
    )
}

fn d6p_receipt_commitment() -> String {
    canonical_sha256(
        "integral-interop-1-d6p-receipt",
        &serde_json::json!({
            "receipt": "integral-d6p-receipt-1",
            "frontier": "integral-frontier-1",
        }),
    )
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
        dependency_snapshot_root: Some(snapshot_commitment()),
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
        [d6p_receipt_commitment()].into_iter().collect()
    } else {
        BTreeSet::new()
    };

    let projection = QualifiedProjectionV1 {
        projection_id: "integral-interop-1-projection".into(),
        projection_version: "1".into(),
        canonicalization_version: "D6S-CANON-1".into(),
        source_dkg_snapshot_commitment: snapshot_commitment(),
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
            [d6p_receipt_commitment()].into_iter().collect()
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

#[derive(Debug, Clone, PartialEq, Eq)]
struct CrossLayerObservation {
    oad_semantic_commitment: String,
    d6x_identity: String,
    d6x_certificate: String,
    d6x_status: DependencyClosureStatusV1,
    d6w_input: Option<String>,
    d6w_derivation: Option<String>,
}

impl CrossLayerObservation {
    fn from_closure(
        selected: String,
        closure: &cos_conformance::qualified_dependency_closure_d6x::DependencyClosureCertificateV1,
        input: Option<InputCommitmentV1>,
        derivation: Option<String>,
    ) -> Self {
        Self {
            oad_semantic_commitment: selected,
            d6x_identity: closure.closure_identity_commitment.clone(),
            d6x_certificate: closure.commitment.clone(),
            d6x_status: closure.status,
            d6w_input: input.map(|v| v.commitment),
            d6w_derivation: derivation,
        }
    }
}

fn evaluate(
    value: &Value,
    with_candidate_noise: bool,
    d6p_receipt: bool,
    require_d6p_receipt: bool,
    runtime_evidence: bool,
) -> Option<CrossLayerObservation> {
    evaluate_with_graph_mutation(
        value,
        with_candidate_noise,
        d6p_receipt,
        require_d6p_receipt,
        runtime_evidence,
        None,
    )
}

fn evaluate_with_graph_mutation(
    value: &Value,
    with_candidate_noise: bool,
    d6p_receipt: bool,
    require_d6p_receipt: bool,
    runtime_evidence: bool,
    graph_mutation: Option<&str>,
) -> Option<CrossLayerObservation> {
    let selected = selected_oad_design_semantic_commitment_checked(value).ok()?;
    let (mut projection, env, derivation) =
        projection(value, with_candidate_noise, d6p_receipt);

    if let Some(mutation) = graph_mutation {
        match mutation {
            "selected-node-commitment" => {
                let node = projection.nodes.get_mut("oad:design-water-purifier:v1")?;
                node.content_commitment = "mutated-selected-node-content".into();
                node.node_commitment = node.recomputed_commitment();
            }
            "selected-edge-commitment" => {
                let edge = projection.edges.get_mut("oad->cos:production-plan")?;
                edge.kind = ClaimGraphEdgeKindV1::Supports;
                edge.edge_commitment = edge.recomputed_commitment();
            }
            "selected-edge-removal" => {
                projection.edges.remove("oad->cos:production-plan");
            }
            "wrong-edge-kind" => {
                let edge = projection.edges.get_mut("oad->cos:production-plan")?;
                edge.kind = ClaimGraphEdgeKindV1::Provenance;
                edge.edge_commitment = canonical_sha256(
                    "integral-interop-1-edge",
                    &serde_json::json!({
                        "id": edge.edge_id,
                        "from": edge.from_node_id,
                        "to": edge.to_node_id,
                        "kind": format!("{:?}", edge.kind),
                    }),
                );
            }
            "irrelevant-node-commitment" => {
                let node = projection.nodes.get_mut("frs:observation:noise")?;
                node.node_commitment = "mutated-irrelevant-node".into();
            }
            "irrelevant-edge-commitment" => {
                let edge = projection.edges.get_mut("frs->cos:candidate")?;
                edge.edge_commitment = "mutated-irrelevant-edge".into();
            }
            other => panic!("unsupported graph mutation {other}"),
        }
    }

    let mut closure = compute_dependency_closure(
        &projection,
        &env,
        &derivation,
        &closure_profile(require_d6p_receipt),
    )?;

    if runtime_evidence {
        let dependency = SemanticDependencyReferenceV1::node(
            "oad:design-water-purifier:v1",
            projection
                .nodes
                .get("oad:design-water-purifier:v1")
                .map(|node| node.node_commitment.clone()),
        );
        closure.resolution_evidence.insert(
            dependency,
            SemanticDependencyResolutionEvidenceV1 {
                retrieval_reference: Some("runtime://resolver/42".into()),
                observed_commitment: projection
                    .nodes
                    .get("oad:design-water-purifier:v1")
                    .map(|node| node.node_commitment.clone()),
                qualification_context_commitment: Some("runtime-qualification-1".into()),
            },
        );
        closure.commitment = closure.recompute();
    }

    let input = InputCommitmentV1::from_projection(&projection, &env, &closure, &closure_profile(require_d6p_receipt), &derivation);
    let derivation_id = input
        .as_ref()
        .and_then(|input| DerivationCommitmentV1::new(input, &derivation, None))
        .map(|d| d.commitment);

    Some(CrossLayerObservation::from_closure(
        selected,
        &closure,
        input,
        derivation_id,
    ))
}

fn assert_delta(
    id: &str,
    baseline: &CrossLayerObservation,
    actual: &CrossLayerObservation,
    expected: &Value,
) {
    let checks = [
        ("oad_semantic_commitment", baseline.oad_semantic_commitment != actual.oad_semantic_commitment),
        ("d6x_identity", baseline.d6x_identity != actual.d6x_identity),
        ("d6x_certificate", baseline.d6x_certificate != actual.d6x_certificate),
        ("d6w_input", baseline.d6w_input != actual.d6w_input),
        ("d6w_derivation", baseline.d6w_derivation != actual.d6w_derivation),
    ];
    for (field, changed) in checks {
        assert_eq!(
            expected[field].as_bool().expect("expected delta boolean"),
            changed,
            "{id}: unexpected {field} propagation"
        );
    }
    assert_eq!(
        expected["d6x_status"].as_str().expect("expected status"),
        serde_json::to_string(&actual.d6x_status)
            .expect("status serialization")
            .trim_matches('"'),
        "{id}: unexpected D6X status"
    );
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
    assert_eq!(corpus["classification"], "cross-layer-propagation-v2");

    let baseline = fixture();
    let baseline_eval =
        evaluate(&baseline, false, false, false, false).expect("baseline must resolve");

    for vector in corpus["vectors"].as_array().expect("vectors array") {
        let id = vector["id"].as_str().expect("vector id");
        let operation = vector["operation"].as_str().expect("vector operation");
        let expected = &vector["expected"];
        let expected_status = expected["d6x_status"]
            .as_str()
            .expect("expected D6X status");

        assert!(
            matches!(
                expected_status,
                "Complete"
                    | "BlockedMissingDependency"
                    | "BlockedCurrentness"
                    | "BlockedResourceLimit"
            ),
            "{id}: unknown D6X status"
        );

        let mut mutated = baseline.clone();
        let mut with_candidate_noise = false;
        let mut d6p_receipt = false;
        let mut require_d6p_receipt = false;
        let mut runtime_evidence = false;

        match operation {
            "set" => set_path(
                &mut mutated,
                vector["path"].as_str().expect("set path"),
                vector["value"].clone(),
            ),
            "candidate-noise" => with_candidate_noise = true,
            "require-d6p-receipt" => {
                require_d6p_receipt = true;
                d6p_receipt = vector["value"].as_bool().expect("receipt boolean");
            }
            "runtime-evidence" => runtime_evidence = true,
            "graph-mutation" => {
                let mutation = vector["mutation"].as_str().expect("graph mutation");
                if mutation.starts_with("irrelevant-") {
                    with_candidate_noise = true;
                }
            }
            other => panic!("{id}: unsupported operation {other}"),
        }

        if expected["valid"].as_bool() == Some(false) {
            assert!(
                validate_selected_oad_design_semantics(&mutated).is_err(),
                "{id}: validation must reject"
            );
            assert!(
                evaluate(
                    &mutated,
                    with_candidate_noise,
                    d6p_receipt,
                    require_d6p_receipt,
                    runtime_evidence,
                )
                .is_none(),
                "{id}: invalid mutation must produce no observation"
            );
            continue;
        }

        let actual = evaluate_with_graph_mutation(
            &mutated,
            with_candidate_noise,
            d6p_receipt,
            require_d6p_receipt,
            runtime_evidence,
            vector["mutation"].as_str(),
        )
        .expect(id);
        assert_delta(id, &baseline_eval, &actual, expected);
    }
}

#[test]
fn cross_layer_mutation_matrix_is_executable() {
    let baseline = fixture();
    let baseline_eval =
        evaluate(&baseline, false, false, false, false).expect("baseline must resolve");
    assert!(validate_selected_oad_design_semantics(&baseline).is_ok());
    assert!(baseline_eval.d6w_input.is_some());

    let mut metadata = baseline.clone();
    metadata["certification"]["documentation_bundle_uri"] =
        Value::from("urn:integral:bundle:changed");
    let metadata_eval =
        evaluate(&metadata, false, false, false, false).expect("metadata must resolve");
    assert_eq!(baseline_eval.oad_semantic_commitment, metadata_eval.oad_semantic_commitment);
    assert_eq!(baseline_eval.d6x_identity, metadata_eval.d6x_identity);
    assert_eq!(baseline_eval.d6x_certificate, metadata_eval.d6x_certificate);
    assert_eq!(baseline_eval.d6w_input, metadata_eval.d6w_input);
    assert_eq!(baseline_eval.d6w_derivation, metadata_eval.d6w_derivation);

    let mut selected = baseline.clone();
    selected["design_version"]["parameters"]["production_steps"][1]["estimated_hours"] =
        Value::from(3);
    let selected_eval =
        evaluate(&selected, false, false, false, false).expect("selected mutation must resolve");
    assert_ne!(baseline_eval.oad_semantic_commitment, selected_eval.oad_semantic_commitment);
    assert_ne!(baseline_eval.d6x_identity, selected_eval.d6x_identity);
    assert_ne!(baseline_eval.d6x_certificate, selected_eval.d6x_certificate);
    assert_ne!(baseline_eval.d6w_input, selected_eval.d6w_input);
    assert_ne!(baseline_eval.d6w_derivation, selected_eval.d6w_derivation);

    let noisy_eval =
        evaluate(&baseline, true, false, false, false).expect("candidate noise must resolve");
    assert_eq!(baseline_eval.oad_semantic_commitment, noisy_eval.oad_semantic_commitment);
    assert_eq!(baseline_eval.d6x_identity, noisy_eval.d6x_identity);
    assert_ne!(baseline_eval.d6x_certificate, noisy_eval.d6x_certificate);
    assert_eq!(baseline_eval.d6w_input, noisy_eval.d6w_input);
    assert_eq!(baseline_eval.d6w_derivation, noisy_eval.d6w_derivation);

    let mut invalid = baseline.clone();
    invalid["design_version"]["parameters"]["bill_of_materials_kg"]["silicone"] =
        serde_json::json!(0.25);
    assert!(validate_selected_oad_design_semantics(&invalid).is_err());
    assert!(evaluate(&invalid, false, false, false, false).is_none());

    let missing_receipt =
        evaluate(&baseline, false, false, true, false).expect("blocked D6X must still certify its boundary");
    assert_eq!(
        missing_receipt.d6x_status,
        DependencyClosureStatusV1::BlockedMissingDependency
    );
    assert!(missing_receipt.d6w_input.is_none());
    assert!(missing_receipt.d6w_derivation.is_none());
    assert_ne!(baseline_eval.d6x_identity, missing_receipt.d6x_identity);
    assert_ne!(baseline_eval.d6x_certificate, missing_receipt.d6x_certificate);

    let present_receipt =
        evaluate(&baseline, false, true, true, false).expect("required D6P receipt should unblock D6X");
    assert_eq!(present_receipt.d6x_status, DependencyClosureStatusV1::Complete);
    assert_ne!(baseline_eval.d6x_identity, present_receipt.d6x_identity);
    assert_ne!(baseline_eval.d6x_certificate, present_receipt.d6x_certificate);
    assert!(present_receipt.d6w_input.is_some());
    assert!(present_receipt.d6w_derivation.is_some());
    assert_ne!(baseline_eval.d6w_input, present_receipt.d6w_input);
    assert_ne!(baseline_eval.d6w_derivation, present_receipt.d6w_derivation);
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

    let input = InputCommitmentV1::from_projection(&projection, &env, &closure, &closure_profile(false), &derivation)
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

    let observed_input = InputCommitmentV1::from_projection(&projection, &env, &observed, &closure_profile(false), &derivation)
        .expect("runtime evidence must not block a complete closure");
    assert_eq!(input.commitment, observed_input.commitment);
}

#[test]
fn stale_selected_node_commitment_is_rejected_at_d6x_boundary() {
    let baseline = fixture();
    let (mut projection, env, derivation) = projection(&baseline, false, false);
    let node = projection.nodes.get_mut("oad:design-water-purifier:v1").expect("selected node");
    let stale = node.node_commitment.clone();
    node.content_commitment = "semantic-content-mutated-with-stale-binding".into();
    node.node_commitment = stale;

    assert!(!projection.nodes["oad:design-water-purifier:v1"].commitment_matches());
    assert!(compute_dependency_closure(
        &projection,
        &env,
        &derivation,
        &closure_profile(false),
    ).is_none());
}

#[test]
fn stale_selected_edge_commitment_is_rejected_at_d6x_boundary() {
    let baseline = fixture();
    let (mut projection, env, derivation) = projection(&baseline, false, false);
    let edge = projection.edges.get_mut("oad->cos:production-plan").expect("selected edge");
    let stale = edge.edge_commitment.clone();
    edge.kind = ClaimGraphEdgeKindV1::DerivedFrom;
    edge.edge_commitment = stale;

    assert!(!projection.edges["oad->cos:production-plan"].commitment_matches());
    assert!(compute_dependency_closure(
        &projection,
        &env,
        &derivation,
        &closure_profile(false),
    ).is_none());
}


#[test]
fn d6w_standalone_input_rejects_duplicate_selected_commitments() {
    let baseline = fixture();
    let (projection, env, derivation) = projection(&baseline, false, false);
    let closure = compute_dependency_closure(&projection, &env, &derivation, &closure_profile(false))
        .expect("baseline closure");
    let input = InputCommitmentV1::from_projection(&projection, &env, &closure, &closure_profile(false), &derivation)
        .expect("baseline input");

    let mut duplicated = input.clone();
    duplicated.nodes.push(duplicated.nodes[0].clone());
    duplicated.commitment = duplicated.recompute();
    assert!(
        !duplicated.valid(),
        "re-hashing must not make duplicate selected commitments canonical"
    );
}

#[test]
fn d6w_standalone_input_rejects_noncanonical_selected_commitment_order() {
    let baseline = fixture();
    let (projection, env, derivation) = projection(&baseline, false, false);
    let closure = compute_dependency_closure(&projection, &env, &derivation, &closure_profile(false))
        .expect("baseline closure");
    let input = InputCommitmentV1::from_projection(&projection, &env, &closure, &closure_profile(false), &derivation)
        .expect("baseline input");

    assert!(input.nodes.windows(2).all(|pair| pair[0] < pair[1]));
    assert!(input.edges.windows(2).all(|pair| pair[0] < pair[1]));

    let mut reordered = input.clone();
    reordered.nodes.reverse();
    reordered.commitment = reordered.recompute();
    assert!(
        !reordered.valid(),
        "re-hashing must not make reordered selected commitments canonical"
    );
}

#[test]
fn d6w_standalone_input_rejects_noncanonical_selected_commitment() {
    let baseline = fixture();
    let (projection, env, derivation) = projection(&baseline, false, false);
    let closure = compute_dependency_closure(&projection, &env, &derivation, &closure_profile(false))
        .expect("baseline closure");
    let mut input = InputCommitmentV1::from_projection(&projection, &env, &closure, &closure_profile(false), &derivation)
        .expect("baseline input");
    input.nodes[0] = "LEGACY-SYMBOLIC".into();
    input.commitment = input.recompute();
    assert!(!input.valid(), "re-hashing cannot make a noncanonical selected commitment valid");
}

#[test]
fn d6w_standalone_input_rejects_noncanonical_edge_and_receipt_commitments() {
    let baseline = fixture();
    let (projection, env, derivation) = projection(&baseline, false, true);
    let closure = compute_dependency_closure(&projection, &env, &derivation, &closure_profile(true))
        .expect("baseline closure");
    let input = InputCommitmentV1::from_projection(&projection, &env, &closure, &closure_profile(true), &derivation)
        .expect("baseline input");

    let mut edge_mutated = input.clone();
    edge_mutated.edges[0] = "edge-symbolic".into();
    edge_mutated.commitment = edge_mutated.recompute();
    assert!(!edge_mutated.valid());

    let mut receipt_mutated = input;
    receipt_mutated.d6p_receipts[0].clear();
    receipt_mutated.commitment = receipt_mutated.recompute();
    assert!(!receipt_mutated.valid());
}
