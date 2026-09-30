//! Adversarial Integral interoperability test: upstream metadata vs selected D6X semantics.

use cos_conformance::canonical_derivation_receipt::{canonical_sha256, D6S_CLAIM_CEILING};
use cos_conformance::layered_derivation_commitment::InputCommitmentV1;
use cos_conformance::qualified_dependency_closure_d6x::{compute_dependency_closure, DependencyClosureProfileV1, DependencyCurrentnessV1, DependencyRuleV1};
use cos_conformance::canonical_derivation_receipt::{DerivationProfileV1, QualifiedEdgeV1, QualifiedNodeV1, QualifiedProjectionV1, SemanticEnvironmentV1};
use cos_conformance::evidence_claim_graph::{ClaimGraphEdgeKindV1, ClaimGraphNodeKindV1};
use serde_json::Value;
use std::collections::{BTreeMap, BTreeSet};

fn fixture() -> Value { serde_json::from_str(include_str!("../testdata/integral_interop_1_design.json")).unwrap() }

fn env() -> SemanticEnvironmentV1 { SemanticEnvironmentV1 { semantic_profile_id: "integral-interop-1".into(), semantic_profile_version: "1".into(), current_frontier_root: Some("integral-frontier-1".into()), d6p_eligibility_context_root: None, d6n_observer_context_root: None, d6o_lifecycle_context_root: None, membership_authority_scope_root: None, dependency_snapshot_root: Some("integral-snapshot-1".into()), historical_cutoff: None, policy_version: "integral-interop-1".into(), claim_ceiling: D6S_CLAIM_CEILING.into() } }

fn derivation() -> DerivationProfileV1 { DerivationProfileV1 { profile_id: "integral-d6w-reference-1".into(), version: "1".into(), rule_ids: ["integral-interop-1".into()].into_iter().collect(), permits_recursive_fixpoint: false, claim_ceiling: D6S_CLAIM_CEILING.into() } }

fn selected_projection(value: &Value) -> QualifiedProjectionV1 {
    let e = env(); let d = derivation();
    let selected = serde_json::json!({"design_version_id": value["design_version"]["id"], "spec_id": value["design_version"]["spec_id"], "materials": value["design_version"]["materials"], "bill_of_materials_kg": value["design_version"]["parameters"]["bill_of_materials_kg"], "production_steps": value["design_version"]["parameters"]["production_steps"]});
    let dc = canonical_sha256("integral-interop-1-design-semantic-v1", &selected);
    let task = serde_json::json!({"design_version_id": value["design_version"]["id"], "production_steps": value["design_version"]["parameters"]["production_steps"]});
    let tc = canonical_sha256("integral-interop-1-cos-task", &task);
    let mut nodes = BTreeMap::new();
    nodes.insert("oad:design-water-purifier:v1".into(), QualifiedNodeV1 { node_id: "oad:design-water-purifier:v1".into(), kind: ClaimGraphNodeKindV1::Statement, node_commitment: canonical_sha256("integral-interop-1-node", &serde_json::json!({"id":"oad:design-water-purifier:v1","content_commitment":dc,"kind":"Statement"})), content_commitment: dc, historical_only:false, current_frontier_root:Some("integral-frontier-1".into()), claim_ceiling:D6S_CLAIM_CEILING.into() });
    nodes.insert("cos:task:water-purifier:v1".into(), QualifiedNodeV1 { node_id:"cos:task:water-purifier:v1".into(), kind:ClaimGraphNodeKindV1::Evidence, node_commitment:canonical_sha256("integral-interop-1-node", &serde_json::json!({"id":"cos:task:water-purifier:v1","content_commitment":tc,"kind":"Evidence"})), content_commitment:tc, historical_only:false, current_frontier_root:Some("integral-frontier-1".into()), claim_ceiling:D6S_CLAIM_CEILING.into() });
    for m in ["silicone","stainless-steel"] { let mv=serde_json::json!({"design_version_id":value["design_version"]["id"],"material":m,"kg":value["design_version"]["parameters"]["bill_of_materials_kg"][m]}); let id=format!("oad:material:{m}"); let cc=canonical_sha256("integral-interop-1-material",&mv); nodes.insert(id.clone(), QualifiedNodeV1 {node_id:id.clone(),kind:ClaimGraphNodeKindV1::Evidence,node_commitment:canonical_sha256("integral-interop-1-node",&serde_json::json!({"id":id,"content_commitment":cc,"kind":"Evidence"})),content_commitment:cc,historical_only:false,current_frontier_root:Some("integral-frontier-1".into()),claim_ceiling:D6S_CLAIM_CEILING.into()}); }
    let mut edges=BTreeMap::new();
    let mk=|id:&str,from:&str,to:&str,kind:ClaimGraphEdgeKindV1| QualifiedEdgeV1 {edge_id:id.into(),from_node_id:from.into(),to_node_id:to.into(),kind,edge_commitment:canonical_sha256("integral-interop-1-edge",&serde_json::json!({"id":id,"from":from,"to":to,"kind":format!("{kind:?}")})),claim_ceiling:D6S_CLAIM_CEILING.into()};
    edges.insert("oad->cos:production-plan".into(),mk("oad->cos:production-plan","oad:design-water-purifier:v1","cos:task:water-purifier:v1",ClaimGraphEdgeKindV1::DerivedFrom));
    for m in ["silicone","stainless-steel"] { let id=format!("cos->material:{m}"); let to=format!("oad:material:{m}"); edges.insert(id.clone(),mk(&id,"cos:task:water-purifier:v1",&to,ClaimGraphEdgeKindV1::Supports)); }
    QualifiedProjectionV1 {projection_id:"integral-interop-1-projection".into(),projection_version:"1".into(),canonicalization_version:"D6S-CANON-1".into(),source_dkg_snapshot_commitment:"integral-snapshot-1".into(),nodes,edges,d6p_current_receipt_commitments:BTreeSet::new(),d6n_context_commitment:None,d6o_context_commitment:None,semantic_environment_commitment:e.commitment(),derivation_profile_commitment:d.commitment(),claim_ceiling:D6S_CLAIM_CEILING.into()}
}

fn profile() -> DependencyClosureProfileV1 { DependencyClosureProfileV1 { profile_id:"integral-oad-cos-material-closure".into(),version:"1".into(),root_node_ids:["oad:design-water-purifier:v1".into()].into_iter().collect(),required_node_ids:BTreeSet::new(),required_d6p_receipt_commitments:BTreeSet::new(),rules:[DependencyRuleV1 {edge_kind:ClaimGraphEdgeKindV1::DerivedFrom,from_kind:Some(ClaimGraphNodeKindV1::Statement),to_kind:Some(ClaimGraphNodeKindV1::Evidence),currentness:DependencyCurrentnessV1::Any},DependencyRuleV1 {edge_kind:ClaimGraphEdgeKindV1::Supports,from_kind:Some(ClaimGraphNodeKindV1::Evidence),to_kind:Some(ClaimGraphNodeKindV1::Evidence),currentness:DependencyCurrentnessV1::Any}].into_iter().collect(),excluded_boundary_policy:"selected OAD/COS/material semantics only".into(),max_nodes:16,max_edges:16,claim_ceiling:D6S_CLAIM_CEILING.into() } }

#[test]
fn unselected_oad_metadata_is_not_a_d6x_dependency() {
    let baseline=fixture(); let mut changed=baseline.clone(); changed["certification"]["documentation_bundle_uri"]=Value::from("urn:integral:bundle:changed");
    let full_a=canonical_sha256("integral-interop-1",&baseline); let full_b=canonical_sha256("integral-interop-1",&changed); assert_ne!(full_a,full_b);
    let e=env(); let d=derivation(); let a=selected_projection(&baseline); let b=selected_projection(&changed);
    let ca=compute_dependency_closure(&a,&e,&d,&profile()).unwrap(); let cb=compute_dependency_closure(&b,&e,&d,&profile()).unwrap();
    assert_eq!(ca.closure_identity_commitment,cb.closure_identity_commitment);
    let ia=InputCommitmentV1::from_projection(&a,&e,&ca,&profile()).unwrap(); let ib=InputCommitmentV1::from_projection(&b,&e,&cb,&profile()).unwrap();
    assert_eq!(ia.commitment,ib.commitment);
}


#[test]
fn irrelevant_symbolic_candidate_material_does_not_block_d6w_consumption() {
    let baseline = fixture();
    let mut projection = selected_projection(&baseline);
    projection.nodes.insert(
        "candidate:legacy-noise".into(),
        QualifiedNodeV1 {
            node_id: "candidate:legacy-noise".into(),
            kind: ClaimGraphNodeKindV1::Source,
            content_commitment: "legacy-content".into(),
            node_commitment: "legacy-symbolic".into(),
            historical_only: false,
            current_frontier_root: Some("integral-frontier-1".into()),
            claim_ceiling: D6S_CLAIM_CEILING.into(),
        },
    );

    let e = env();
    let d = derivation();
    let closure = compute_dependency_closure(&projection, &e, &d, &profile()).unwrap();
    let input = InputCommitmentV1::from_projection(&projection, &e, &closure, &profile())
        .expect("irrelevant legacy candidate material must remain outside D6W consumption");
    assert_eq!(input.projection, closure.closure_identity_commitment);
}
