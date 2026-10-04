//! D6W diagnostic decomposition of D6S commitments. Integrity only; no authority.
use crate::canonical_derivation_receipt::{
    canonical_sha256, is_canonical_sha256_commitment, CanonicalDerivationReceiptV1, DerivationProfileV1,
    DerivationResultStatusV1, QualifiedEdgeV1, QualifiedNodeV1, QualifiedProjectionV1, SemanticEnvironmentV1, D6S_CLAIM_CEILING,
};
use crate::finality_eligibility_composition::{
    CurrentFinalityEligibilityReceiptV1,
    verify_current_receipt_provenance_from_composition, FinalityEligibilityCompositionV1,
};
use serde::{Deserialize, Serialize};

use crate::qualified_dependency_closure_d6x as qualified_dependency_closure_d6x;
use crate::qualified_dependency_closure_d6x::{
    DependencyClosureCertificateV1, DependencyClosureProfileV1,
};

pub const D6W_SCHEMA_VERSION: &str = "D6W-1";

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct InputCommitmentV1 {
    pub schema_version: String,
    pub source_snapshot: String,
    pub projection: String,
    pub environment: String,
    pub dependency_closure: String,
    pub nodes: Vec<String>,
    pub edges: Vec<String>,
    pub d6p_receipts: Vec<String>,
    pub claim_ceiling: String,
    pub commitment: String,
}
impl InputCommitmentV1 {
    pub fn from_projection(
        p: &QualifiedProjectionV1,
        e: &SemanticEnvironmentV1,
        closure: &DependencyClosureCertificateV1,
        closure_profile: &DependencyClosureProfileV1,
        derivation_profile: &DerivationProfileV1,
    ) -> Option<Self> {
        if closure.status != qualified_dependency_closure_d6x::DependencyClosureStatusV1::Complete || !closure.valid() {
            return None;
        }
        // Do not accept a merely self-consistent closure certificate. Re-execute
        // the exact D6X profile against the exact projection/environment/profile
        // and bind D6W to the resulting semantic closure identity. Runtime
        // evidence is intentionally excluded from this identity comparison.
        if !closure.verifies_against_sources(p, e, derivation_profile, closure_profile) {
            return None;
        }
        // D6W must consume the exact projection that D6X qualified. In
        // particular, a caller cannot substitute a different source snapshot,
        // environment, derivation profile, or stale selected graph binding.
        if !p.structurally_valid()
            || !is_canonical_sha256_commitment(&p.source_dkg_snapshot_commitment)
            || !e.structurally_valid()
            || e.dependency_snapshot_root.as_deref()
                != Some(p.source_dkg_snapshot_commitment.as_str())
            || p.semantic_environment_commitment != e.commitment()
            || closure.projection_commitment != p.commitment()
            || closure.source_dkg_snapshot_commitment != p.source_dkg_snapshot_commitment
            || closure.semantic_environment_commitment != e.commitment()
            || closure.derivation_profile_commitment != p.derivation_profile_commitment
            || closure.derivation_profile_commitment != derivation_profile.commitment()
            || closure.closure_profile_commitment != closure_profile.commitment()
            || closure.root_node_ids != closure_profile.root_node_ids
            || closure.derivation_profile_commitment.is_empty()
            || !closure.included_nodes.iter().all(|(id, commitment)| {
                p.nodes.get(id).is_some_and(|node| {
                    QualifiedNodeV1::commitment_matches(&node)
                        && &node.node_commitment == commitment
                        && is_canonical_sha256_commitment(&node.node_commitment)
                })
            })
            || !closure.included_edges.iter().all(|(id, (from, to, kind, commitment))| {
                p.edges.get(id).is_some_and(|edge| {
                    QualifiedEdgeV1::commitment_matches(&edge)
                        && &edge.from_node_id == from
                        && &edge.to_node_id == to
                        && &edge.kind == kind
                        && &edge.edge_commitment == commitment
                        && is_canonical_sha256_commitment(&edge.edge_commitment)
                })
            })
            || closure
                .included_d6p_receipt_commitments
                .iter()
                .any(|commitment| !is_canonical_sha256_commitment(commitment))
        {
            return None;
        }
        // The certificate must describe exactly the selected source objects,
        // not merely a self-consistent certificate carrying the same projection.
        let selected_node_bindings = closure
            .included_nodes
            .iter()
            .all(|(id, commitment)| p.nodes.get(id).is_some_and(|node| &node.node_commitment == commitment));
        let selected_edge_bindings = closure
            .included_edges
            .iter()
            .all(|(id, (from, to, kind, commitment))| {
                p.edges.get(id).is_some_and(|edge| {
                    &edge.from_node_id == from
                        && &edge.to_node_id == to
                        && &edge.kind == kind
                        && &edge.edge_commitment == commitment
                })
            });
        if !selected_node_bindings || !selected_edge_bindings {
            return None;
        }

        let mut v = Self {
            schema_version: D6W_SCHEMA_VERSION.into(),
            source_snapshot: p.source_dkg_snapshot_commitment.clone(),
            projection: closure.closure_identity_commitment.clone(),
            environment: e.commitment(),
            dependency_closure: closure.closure_identity_commitment.clone(),
            nodes: closure.included_node_ids.iter().filter_map(|id| p.nodes.get(id).map(|n| n.node_commitment.clone())).collect(),
            edges: closure.included_edges.values().map(|(_, _, _, commitment)| commitment.clone()).collect(),
            d6p_receipts: closure.included_d6p_receipt_commitments.iter().cloned().collect(),
            claim_ceiling: D6S_CLAIM_CEILING.into(),
            commitment: String::new(),
        };
        v.commitment = v.recompute(); Some(v)
    }
    /// Strict D6W entrypoint for callers that can supply the qualified D6P
    /// composition set. Every D6P receipt named by the projection is checked
    /// against its committed composition before the ordinary D6W identity is
    /// admitted.
    ///
    /// This is a D6P integrity/provenance check, not independent reconstruction of
    /// D6N/D6O authority. Callers that need that stronger guarantee must establish
    /// the authoritative D6P composition upstream before presenting it here.
    pub fn from_projection_with_authoritative_d6p(
        p: &QualifiedProjectionV1,
        e: &SemanticEnvironmentV1,
        closure_profile: &DependencyClosureProfileV1,
        derivation_profile: &DerivationProfileV1,
        d6p_receipts: &[CurrentFinalityEligibilityReceiptV1],
        d6p_compositions: &[FinalityEligibilityCompositionV1],
        current_frontier_root: Option<&str>,
    ) -> Option<Self> {
        let closure =
            qualified_dependency_closure_d6x::compute_dependency_closure_from_authoritative_d6p_at_frontier(
                p,
                e,
                derivation_profile,
                closure_profile,
                d6p_receipts,
                d6p_compositions,
                current_frontier_root,
            )?;

        // Only D6P receipts selected by the D6X closure profile cross the
        // strict D6W provenance gate. Projection-wide candidate receipts are
        // retained as source/audit material and must not become implicit D6W
        // dependencies.
        for expected in &closure.included_d6p_receipt_commitments {
            let receipt = d6p_receipts
                .iter()
                .find(|receipt| receipt.receipt_commitment == *expected)?;
            let composition = d6p_compositions
                .iter()
                .find(|composition| composition.composition_commitment == receipt.composition_commitment)?;
            if !verify_current_receipt_provenance_from_composition(receipt, composition) {
                return None;
            }
        }

        Self::from_projection(p, e, &closure, closure_profile, derivation_profile)
    }

    pub fn recompute(&self) -> String {
        let mut v = self.clone(); v.commitment.clear();
        canonical_sha256("d6w-input", &v)
    }
    pub fn valid(&self) -> bool {
        fn strictly_sorted_unique(values: &[String]) -> bool {
            values.windows(2).all(|pair| pair[0] < pair[1])
        }

        self.schema_version == D6W_SCHEMA_VERSION
            // D6W is the first stricter downstream boundary: a source
            // snapshot identifier must be a canonical D6S commitment, not an
            // opaque symbolic label. This does not prove the underlying DKG
            // snapshot is authoritative; it only prevents an unauthenticated
            // textual identifier from crossing the qualified-consumption gate.
            && is_canonical_sha256_commitment(&self.source_snapshot)
            && is_canonical_sha256_commitment(&self.projection)
            && is_canonical_sha256_commitment(&self.environment)
            && is_canonical_sha256_commitment(&self.dependency_closure)
            && !self.nodes.is_empty()
            && self.nodes.iter().all(|v| is_canonical_sha256_commitment(v))
            && strictly_sorted_unique(&self.nodes)
            && self.edges.iter().all(|v| is_canonical_sha256_commitment(v))
            && strictly_sorted_unique(&self.edges)
            // Qualified consumption does not accept opaque D6P receipt identifiers.
            // A receipt set can be perfectly sorted and self-consistent while still
            // naming substituted evidence; the receipt commitment must therefore be
            // a canonical D6P cryptographic commitment before it crosses D6W.
            && self.d6p_receipts.iter().all(|v| is_canonical_sha256_commitment(v))
            && strictly_sorted_unique(&self.d6p_receipts)
            && self.claim_ceiling == D6S_CLAIM_CEILING
            && is_canonical_sha256_commitment(&self.commitment)
            && self.commitment == self.recompute()
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct DerivationCommitmentV1 {
    pub schema_version:String, pub input:String, pub profile:String,
    pub execution_trace:Option<String>, pub claim_ceiling:String, pub commitment:String,
}
impl DerivationCommitmentV1 {
    pub fn new(i:&InputCommitmentV1,p:&DerivationProfileV1,trace:Option<String>)->Option<Self>{
        if !i.valid() || !p.structurally_valid() || trace.as_deref().is_some_and(|s|s.trim().is_empty()){return None}
        let mut v=Self{schema_version:D6W_SCHEMA_VERSION.into(),input:i.commitment.clone(),profile:p.commitment(),execution_trace:trace,claim_ceiling:D6S_CLAIM_CEILING.into(),commitment:String::new()};
        v.commitment=v.recompute();Some(v)
    }
    pub fn recompute(&self)->String{let mut v=self.clone();v.commitment.clear();canonical_sha256("d6w-derivation",&v)}
    pub fn valid(&self)->bool{
        self.schema_version==D6W_SCHEMA_VERSION
            && is_canonical_sha256_commitment(&self.input)
            && is_canonical_sha256_commitment(&self.profile)
            && self.execution_trace.as_deref().is_none_or(|trace| !trace.trim().is_empty())
            && self.claim_ceiling==D6S_CLAIM_CEILING
            && is_canonical_sha256_commitment(&self.commitment)
            && self.commitment==self.recompute()
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ResultCommitmentV1 {
    pub schema_version:String, pub derivation:String, pub status:DerivationResultStatusV1,
    pub payload:String, pub contradiction:bool, pub unresolved:bool, pub claim_ceiling:String, pub commitment:String,
}
impl ResultCommitmentV1 {
    pub fn new(d:&DerivationCommitmentV1,status:DerivationResultStatusV1,payload:String,contradiction:bool,unresolved:bool)->Option<Self>{
        if !d.valid()||payload.trim().is_empty()||(status==DerivationResultStatusV1::Supported&&(contradiction||unresolved))||(status==DerivationResultStatusV1::Disputed&&!contradiction)||(matches!(status,DerivationResultStatusV1::Unresolved|DerivationResultStatusV1::BlockedMissingEvidence|DerivationResultStatusV1::BlockedCurrentness|DerivationResultStatusV1::BlockedQualification)&&!unresolved){return None}
        let mut v=Self{schema_version:D6W_SCHEMA_VERSION.into(),derivation:d.commitment.clone(),status,payload,contradiction,unresolved,claim_ceiling:D6S_CLAIM_CEILING.into(),commitment:String::new()};v.commitment=v.recompute();Some(v)
    }
    pub fn recompute(&self)->String{let mut v=self.clone();v.commitment.clear();canonical_sha256("d6w-result",&v)}
    pub fn valid(&self)->bool{
        self.schema_version==D6W_SCHEMA_VERSION
            && is_canonical_sha256_commitment(&self.derivation)
            && !self.payload.is_empty()
            && self.claim_ceiling==D6S_CLAIM_CEILING
            && is_canonical_sha256_commitment(&self.commitment)
            && self.commitment==self.recompute()
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct LayeredReceiptV1 { pub schema_version:String,pub input:String,pub derivation:String,pub result:String,pub claim_ceiling:String,pub commitment:String }
impl LayeredReceiptV1 {
    pub fn new(i:&InputCommitmentV1,d:&DerivationCommitmentV1,r:&ResultCommitmentV1)->Option<Self>{
        if !i.valid()||!d.valid()||!r.valid()||d.input!=i.commitment||r.derivation!=d.commitment{return None}
        let mut v=Self{schema_version:D6W_SCHEMA_VERSION.into(),input:i.commitment.clone(),derivation:d.commitment.clone(),result:r.commitment.clone(),claim_ceiling:D6S_CLAIM_CEILING.into(),commitment:String::new()};v.commitment=v.recompute();Some(v)
    }

    /// Reconstruct and cross-check the D6W layers against an exact D6S receipt
    /// and the exact D6X closure commitment that qualified the input boundary.
    pub fn verifies_d6s(
        &self, d6s:&CanonicalDerivationReceiptV1, p:&QualifiedProjectionV1,
        e:&SemanticEnvironmentV1, profile:&DerivationProfileV1,
        closure_profile:&DependencyClosureProfileV1,
        closure:&DependencyClosureCertificateV1,
        current_d6p_receipts:&[CurrentFinalityEligibilityReceiptV1],
        trace:Option<String>
    )->bool {
        if !crate::canonical_derivation_receipt::verify_canonical_receipt(
            d6s, p, e, profile, current_d6p_receipts
        ) || !closure.valid()
            || !p.commitments_match_sources(e, profile)
            || closure.projection_commitment != p.commitment()
            || closure.semantic_environment_commitment != e.commitment()
            || closure.derivation_profile_commitment != profile.commitment()
            || closure.closure_profile_commitment != closure_profile.commitment()
            || closure.root_node_ids != closure_profile.root_node_ids
            || d6s.projection_commitment!=p.commitment()
            || d6s.semantic_environment_commitment!=e.commitment()
            || d6s.derivation_profile_commitment!=profile.commitment()
            || d6s.source_dkg_snapshot_commitment!=p.source_dkg_snapshot_commitment
            || d6s.input_node_commitments!=p.nodes.values().map(|n|n.node_commitment.clone()).collect()
            || d6s.input_edge_commitments!=p.edges.values().map(|n|n.edge_commitment.clone()).collect()
            || d6s.d6p_current_receipt_commitments!=p.d6p_current_receipt_commitments { return false; }
        let Some(input)=InputCommitmentV1::from_projection(p,e,closure,closure_profile,profile) else{return false};
        if !input.valid(){return false}
        let Some(derivation)=DerivationCommitmentV1::new(&input,profile,trace) else{return false};
        let Some(result)=ResultCommitmentV1::new(&derivation,d6s.result_status,d6s.result_commitment.clone(),d6s.contradiction_preserved,d6s.unresolved_preserved) else{return false};
        let Some(expected)=Self::new(&input,&derivation,&result) else{return false};
        self.valid() && self.input==expected.input && self.derivation==expected.derivation
            && self.result==expected.result && self.claim_ceiling==expected.claim_ceiling && self.commitment==expected.commitment
    }
    pub fn recompute(&self)->String{let mut v=self.clone();v.commitment.clear();canonical_sha256("d6w-receipt",&v)}
    pub fn valid(&self)->bool{
        self.schema_version==D6W_SCHEMA_VERSION
            && is_canonical_sha256_commitment(&self.input)
            && is_canonical_sha256_commitment(&self.derivation)
            && is_canonical_sha256_commitment(&self.result)
            && self.claim_ceiling==D6S_CLAIM_CEILING
            && is_canonical_sha256_commitment(&self.commitment)
            && self.commitment==self.recompute()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::{BTreeMap, BTreeSet};
    use crate::canonical_derivation_receipt::{QualifiedEdgeV1,QualifiedNodeV1};
    use crate::evidence_claim_graph::{ClaimGraphEdgeKindV1,ClaimGraphNodeKindV1};
    use qualified_dependency_closure_d6x::{compute_dependency_closure,DependencyClosureProfileV1,DependencyRuleV1,DependencyCurrentnessV1};

    fn closure_profile() -> DependencyClosureProfileV1 {
        DependencyClosureProfileV1 {
            profile_id: "cp".into(),
            version: "1".into(),
            root_node_ids: ["root".into()].into_iter().collect(),
            required_node_ids: BTreeSet::new(),
            required_d6p_receipt_commitments: BTreeSet::new(),
            rules: [DependencyRuleV1 {
                edge_kind: ClaimGraphEdgeKindV1::Supports,
                from_kind: Some(ClaimGraphNodeKindV1::Statement),
                to_kind: Some(ClaimGraphNodeKindV1::Evidence),
                currentness: DependencyCurrentnessV1::Any,
            }]
            .into_iter()
            .collect(),
            excluded_boundary_policy: "rule-matched semantic edges only".into(),
            max_nodes: 16,
            max_edges: 16,
            claim_ceiling: D6S_CLAIM_CEILING.into(),
        }
    }

    fn fixture(extra:bool)->(QualifiedProjectionV1,SemanticEnvironmentV1,DerivationProfileV1,DependencyClosureCertificateV1){
        let snapshot = canonical_sha256("fixture-dkg-snapshot", &serde_json::json!({
            "fixture": "d6w",
            "snapshot": "snapshot",
        }));
        let e=SemanticEnvironmentV1{semantic_profile_id:"sem".into(),semantic_profile_version:"1".into(),current_frontier_root:Some("frontier".into()),d6p_eligibility_context_root:None,d6n_observer_context_root:None,d6o_lifecycle_context_root:None,membership_authority_scope_root:None,dependency_snapshot_root:Some(snapshot.clone()),historical_cutoff:None,policy_version:"policy".into(),claim_ceiling:D6S_CLAIM_CEILING.into()};
        let d=DerivationProfileV1{profile_id:"d".into(),version:"1".into(),rule_ids:["r".into()].into_iter().collect(),permits_recursive_fixpoint:false,claim_ceiling:D6S_CLAIM_CEILING.into()};
        let mut nodes=BTreeMap::new();
        for (id,k) in [("root",ClaimGraphNodeKindV1::Statement),("dep",ClaimGraphNodeKindV1::Evidence)]{nodes.insert(id.into(),QualifiedNodeV1{node_id:id.into(),kind:k,content_commitment:format!("c-{id}"),node_commitment:canonical_sha256("integral-interop-1-node",&serde_json::json!({"id":id,"content_commitment":format!("c-{id}"),"kind":format!("{k:?}")})),historical_only:false,current_frontier_root:Some("frontier".into()),claim_ceiling:D6S_CLAIM_CEILING.into()});}
        if extra{nodes.insert("noise".into(),QualifiedNodeV1{node_id:"noise".into(),kind:ClaimGraphNodeKindV1::Source,content_commitment:"c-noise".into(),node_commitment:canonical_sha256("integral-interop-1-node",&serde_json::json!({"id":"noise","content_commitment":"c-noise","kind":format!("{:?}",ClaimGraphNodeKindV1::Source)})),historical_only:false,current_frontier_root:Some("frontier".into()),claim_ceiling:D6S_CLAIM_CEILING.into()});}
        let mut edges=BTreeMap::new();
        edges.insert("e1".into(),QualifiedEdgeV1{edge_id:"e1".into(),from_node_id:"root".into(),to_node_id:"dep".into(),kind:ClaimGraphEdgeKindV1::Supports,edge_commitment:canonical_sha256("integral-interop-1-edge",&serde_json::json!({"id":"e1","from":"root","to":"dep","kind":format!("{:?}",ClaimGraphEdgeKindV1::Supports)})),claim_ceiling:D6S_CLAIM_CEILING.into()});
        if extra{edges.insert("noise-edge".into(),QualifiedEdgeV1{edge_id:"noise-edge".into(),from_node_id:"noise".into(),to_node_id:"dep".into(),kind:ClaimGraphEdgeKindV1::Provenance,edge_commitment:canonical_sha256("integral-interop-1-edge",&serde_json::json!({"id":"noise-edge","from":"noise","to":"dep","kind":format!("{:?}",ClaimGraphEdgeKindV1::Provenance)})),claim_ceiling:D6S_CLAIM_CEILING.into()});}
        let p=QualifiedProjectionV1{projection_id:"p".into(),projection_version:"1".into(),canonicalization_version:"D6S-CANON-1".into(),source_dkg_snapshot_commitment:e.dependency_snapshot_root.clone().unwrap(),nodes,edges,d6p_current_receipt_commitments:BTreeSet::new(),d6n_context_commitment:None,d6o_context_commitment:None,semantic_environment_commitment:e.commitment(),derivation_profile_commitment:d.commitment(),claim_ceiling:D6S_CLAIM_CEILING.into()};
        let cp = closure_profile();
        let c=compute_dependency_closure(&p,&e,&d,&cp).unwrap();
        (p,e,d,c)
    }

    #[test]
    fn d6w_derived_layers_cannot_be_self_consistent_with_opaque_references() {
        let (p, e, d, closure) = fixture(false);
        let profile = closure_profile();
        let input = InputCommitmentV1::from_projection(&p, &e, &closure, &profile, &d)
            .expect("baseline D6W input");

        let mut derivation = DerivationCommitmentV1::new(&input, &d, None)
            .expect("baseline D6W derivation");
        derivation.profile = "opaque-profile".into();
        derivation.commitment = derivation.recompute();

        assert!(!derivation.valid());
        assert!(ResultCommitmentV1::new(
            &derivation,
            DerivationResultStatusV1::Rejected,
            "result".into(),
            false,
            false,
        ).is_none());
    }

    #[test]
    fn d6w_layered_receipt_rejects_opaque_references_even_when_recommitted() {
        let (p, e, d, closure) = fixture(false);
        let profile = closure_profile();
        let input = InputCommitmentV1::from_projection(&p, &e, &closure, &profile, &d)
            .expect("baseline D6W input");
        let derivation = DerivationCommitmentV1::new(&input, &d, None)
            .expect("baseline D6W derivation");
        let result = ResultCommitmentV1::new(
            &derivation,
            DerivationResultStatusV1::Rejected,
            "result".into(),
            false,
            false,
        ).expect("baseline D6W result");
        let mut receipt = LayeredReceiptV1::new(&input, &derivation, &result)
            .expect("baseline layered receipt");
        receipt.result = "opaque-result".into();
        receipt.commitment = receipt.recompute();

        assert!(!receipt.valid());
    }

    #[test]
    fn d6w_verifier_binds_the_dependency_closure_profile_not_the_derivation_profile() {
        let (p, e, d, closure) = fixture(false);
        let closure_profile = closure_profile();

        assert_ne!(
            closure_profile.commitment(),
            d.commitment(),
            "the test must exercise the distinct D6X/D6S profile domains"
        );

        let d6s = crate::canonical_derivation_receipt::build_canonical_receipt(
            &p,
            &e,
            &d,
            &[],
            DerivationResultStatusV1::Rejected,
            "result-1".into(),
            false,
            false,
        )
        .expect("rejected D6S receipt does not require current D6P evidence");

        let input = InputCommitmentV1::from_projection(&p, &e, &closure, &closure_profile, &d)
            .expect("complete D6X closure must enter D6W");
        let derivation = DerivationCommitmentV1::new(&input, &d, None)
            .expect("D6W derivation commitment");
        let result = ResultCommitmentV1::new(
            &derivation,
            DerivationResultStatusV1::Rejected,
            "result-1".into(),
            false,
            false,
        )
        .expect("D6W result commitment");
        let receipt = LayeredReceiptV1::new(&input, &derivation, &result)
            .expect("D6W layered receipt");

        assert!(receipt.verifies_d6s(
            &d6s,
            &p,
            &e,
            &d,
            &closure_profile,
            &closure,
            &[],
            None,
        ));

        let mut wrong_profile = closure_profile.clone();
        wrong_profile.version = "2".into();
        assert!(!receipt.verifies_d6s(
            &d6s,
            &p,
            &e,
            &d,
            &wrong_profile,
            &closure,
            &[],
            None,
        ));
    }

    #[test]
    fn d6w_input_rejects_self_consistent_closure_with_wrong_profile() {
        let (p, e, d, mut closure) = fixture(false);
        let expected_profile = closure_profile();

        closure.closure_profile_commitment = "forged-profile".into();
        closure.root_node_ids = ["root".into()].into_iter().collect();
        closure.dependencies = closure.expected_dependencies();
        closure.dependency_resolutions = closure
            .dependencies
            .iter()
            .map(|dependency| (
                dependency.clone(),
                qualified_dependency_closure_d6x::SemanticDependencyResolutionV1::Present,
            ))
            .collect();
        closure.closure_identity_commitment = closure.closure_identity();
        closure.commitment = closure.recompute();

        assert!(closure.valid());
        assert!(InputCommitmentV1::from_projection(
            &p, &e, &closure, &expected_profile, &d
        ).is_none());
    }

    #[test]
    fn d6w_input_rejects_self_consistent_closure_that_omits_required_traversal() {
        let (p, e, d, mut closure) = fixture(false);
        let expected_profile = closure_profile();

        closure.included_nodes.remove("dep");
        closure.included_node_ids.remove("dep");
        closure.included_node_commitments
            .retain(|commitment| commitment != &p.nodes["dep"].node_commitment);
        closure.included_edges.clear();
        closure.included_edge_commitments.clear();
        closure.dependencies = closure.expected_dependencies();
        closure.dependency_resolutions = closure
            .dependencies
            .iter()
            .map(|dependency| (
                dependency.clone(),
                qualified_dependency_closure_d6x::SemanticDependencyResolutionV1::Present,
            ))
            .collect();
        closure.closure_identity_commitment = closure.closure_identity();
        closure.commitment = closure.recompute();

        assert!(closure.valid());
        assert!(InputCommitmentV1::from_projection(
            &p, &e, &closure, &expected_profile, &d
        ).is_none());
    }

    #[test]
    fn d6w_rejects_stale_environment_commitment() {
        let (mut p,e,d,c)=fixture(false);
        p.semantic_environment_commitment = "stale-environment".into();
        assert!(!p.commitments_match_sources(&e,&d));
        assert!(InputCommitmentV1::from_projection(&p,&e,&c,&closure_profile(),&d).is_none());
    }

    #[test]
    fn d6w_rejects_stale_derivation_profile_commitment() {
        let (mut p,e,d,c)=fixture(false);
        p.derivation_profile_commitment = "stale-profile".into();
        assert!(!p.commitments_match_sources(&e,&d));
        assert!(InputCommitmentV1::from_projection(&p,&e,&c,&closure_profile(),&d).is_none());
    }

    #[test]
    fn d6w_rejects_stale_selected_node_binding() {
        let (mut p,e,d,c)=fixture(false);
        let node = p.nodes.get_mut("dep").unwrap();
        node.node_commitment = "0000000000000000000000000000000000000000000000000000000000000000".into();
        assert!(!p.commitments_match_sources(&e,&d));
        assert!(InputCommitmentV1::from_projection(&p,&e,&c,&closure_profile(),&d).is_none());
    }

    #[test]
    fn d6w_rejects_canonical_source_snapshot_not_bound_to_environment() {
        let (mut p,e,d,c) = fixture(false);
        let substituted = canonical_sha256("fixture-dkg-snapshot", &serde_json::json!({
            "fixture": "d6w",
            "snapshot": "substituted",
        }));
        p.source_dkg_snapshot_commitment = substituted;
        p.semantic_environment_commitment = e.commitment();
        assert!(is_canonical_sha256_commitment(&p.source_dkg_snapshot_commitment));
        assert!(InputCommitmentV1::from_projection(&p, &e, &c, &closure_profile(), &d).is_none());
    }

    #[test]
    fn d6w_rejects_environment_without_a_snapshot_binding() {
        let (p, mut e, d, c) = fixture(false);
        e.dependency_snapshot_root = None;
        assert!(InputCommitmentV1::from_projection(&p, &e, &c, &closure_profile(), &d).is_none());
    }

    #[test]
    fn d6w_rejects_opaque_source_snapshot_even_when_projection_is_self_consistent() {
        let (mut p,e,d,c) = fixture(false);
        p.source_dkg_snapshot_commitment = "different-snapshot".into();
        assert!(!is_canonical_sha256_commitment(&p.source_dkg_snapshot_commitment));
        assert!(InputCommitmentV1::from_projection(&p, &e, &c, &closure_profile(), &d).is_none());
    }

    #[test]
    fn d6w_rejects_source_snapshot_substitution_against_old_closure() {
        let (mut p,e,d,c) = fixture(false);
        p.source_dkg_snapshot_commitment = "different-snapshot".into();

        assert!(InputCommitmentV1::from_projection(&p, &e, &c, &closure_profile(), &d).is_none());

        let changed = compute_dependency_closure(
            &p,
            &e,
            &d,
            &DependencyClosureProfileV1 {
                profile_id: "cp".into(),
                version: "1".into(),
                root_node_ids: ["root".into()].into_iter().collect(),
                required_node_ids: BTreeSet::new(),
                required_d6p_receipt_commitments: BTreeSet::new(),
                rules: [DependencyRuleV1 {
                    edge_kind: ClaimGraphEdgeKindV1::Supports,
                    from_kind: Some(ClaimGraphNodeKindV1::Statement),
                    to_kind: Some(ClaimGraphNodeKindV1::Evidence),
                    currentness: DependencyCurrentnessV1::Any,
                }]
                .into_iter()
                .collect(),
                excluded_boundary_policy: "rule-matched semantic edges only".into(),
                max_nodes: 16,
                max_edges: 16,
                claim_ceiling: D6S_CLAIM_CEILING.into(),
            },
        )
        .unwrap();

        assert_ne!(c.source_dkg_snapshot_commitment, changed.source_dkg_snapshot_commitment);
        assert_ne!(c.closure_identity_commitment, changed.closure_identity_commitment);

        let original_input = InputCommitmentV1::from_projection(&fixture(false).0, &e, &c, &closure_profile(), &d).unwrap();
        let changed_input = InputCommitmentV1::from_projection(&p, &e, &changed, &closure_profile(), &d).unwrap();
        assert_ne!(original_input.commitment, changed_input.commitment);
    }

    #[test]
    fn d6w_rejects_closure_snapshot_mismatch() {
        let (p,e,d,mut c) = fixture(false);
        c.source_dkg_snapshot_commitment = "different-snapshot".into();
        c.commitment = c.recompute();

        assert!(InputCommitmentV1::from_projection(&p, &e, &c, &closure_profile(), &d).is_none());
        let _ = d;
    }

    #[test]
    fn d6w_rejects_self_consistent_closure_with_wrong_selected_node() {
        let (p,e,d,mut c) = fixture(false);
        c.included_nodes.insert("dep".into(), "substituted-node".into());
        c.included_node_commitments.remove(&p.nodes.get("dep").unwrap().node_commitment);
        c.included_node_commitments.insert("substituted-node".into());
        c.dependencies = c.expected_dependencies();
        c.dependency_resolutions = c
            .dependencies
            .iter()
            .map(|dependency| (dependency.clone(), qualified_dependency_closure_d6x::SemanticDependencyResolutionV1::Present))
            .collect();
        c.closure_identity_commitment = c.closure_identity();
        c.commitment = c.recompute();

        assert!(c.valid());
        assert!(InputCommitmentV1::from_projection(&p, &e, &c, &closure_profile(), &d).is_none());
        let _ = d;
    }

    #[test]
    fn d6w_rejects_self_consistent_closure_with_wrong_selected_edge() {
        let (p,e,d,mut c) = fixture(false);
        let edge = c.included_edges.get_mut("e1").unwrap();
        let old_edge_commitment = edge.3.clone();
        edge.3 = "substituted-edge".into();
        c.included_edge_commitments.remove(&old_edge_commitment);
        c.included_edge_commitments.insert("substituted-edge".into());
        c.dependencies = c.expected_dependencies();
        c.dependency_resolutions = c
            .dependencies
            .iter()
            .map(|dependency| (dependency.clone(), qualified_dependency_closure_d6x::SemanticDependencyResolutionV1::Present))
            .collect();
        c.closure_identity_commitment = c.closure_identity();
        c.commitment = c.recompute();

        assert!(c.valid());
        assert!(InputCommitmentV1::from_projection(&p, &e, &c, &closure_profile(), &d).is_none());
        let _ = d;
    }

    #[test]
    fn d6w_rejects_legacy_symbolic_selected_commitment() {
        let (mut p,e,d,c) = fixture(false);
        p.nodes.get_mut("dep").unwrap().node_commitment = "legacy-symbolic".into();
        assert!(!is_canonical_sha256_commitment(&p.nodes["dep"].node_commitment));
        assert!(InputCommitmentV1::from_projection(&p, &e, &c, &closure_profile(), &d).is_none());
        let _ = d;
    }

    #[test]
    fn d6w_authoritative_d6p_entrypoint_accepts_empty_receipt_set() {
        let (p, e, d, c) = fixture(false);
        let closure_profile = closure_profile();
        let input = InputCommitmentV1::from_projection_with_authoritative_d6p(
            &p,
            &e,
            &closure_profile,
            &d,
            &[],
            &[],
            Some("frontier-1"),
        );
        assert!(input.is_some());
        let _ = c;
    }

    #[test]
    fn d6w_rejects_opaque_d6p_receipt_identifier() {
        let (p, e, d, c) = fixture(false);
        let mut input = InputCommitmentV1::from_projection(
            &p, &e, &c, &closure_profile(), &d
        ).expect("baseline D6W input");

        input.d6p_receipts = vec!["legacy-d6p-receipt".into()];
        input.commitment = input.recompute();

        assert!(!is_canonical_sha256_commitment(&input.d6p_receipts[0]));
        assert!(!input.valid());
    }
    #[test] fn closure_is_bound_into_input(){let(p,e,d,c)=fixture(false);let i=InputCommitmentV1::from_projection(&p,&e,&c,&closure_profile(),&d).unwrap();assert!(i.valid());assert!(!i.dependency_closure.is_empty());let x=LayeredReceiptV1::new(&i,&DerivationCommitmentV1::new(&i,&d,None).unwrap(),&ResultCommitmentV1::new(&DerivationCommitmentV1::new(&i,&d,None).unwrap(),DerivationResultStatusV1::Supported,"x".into(),false,false).unwrap());assert!(x.is_some());}
    #[test] fn blocked_closure_cannot_enter_d6w_input(){
        let(p,e,d,mut c)=fixture(false);
        c.status=qualified_dependency_closure_d6x::DependencyClosureStatusV1::BlockedResourceLimit;
        c.commitment=c.recompute();
        assert!(InputCommitmentV1::from_projection(&p,&e,&c,&closure_profile(),&d).is_none());
        let _=d;
    }
    #[test] fn closure_mutation_changes_input(){let(p,e,d,c)=fixture(false);let mut i=InputCommitmentV1::from_projection(&p,&e,&c,&closure_profile(),&d).unwrap();let old=i.commitment.clone();i.dependency_closure="tampered".into();i.commitment=i.recompute();assert_ne!(old,i.commitment);assert!(DerivationCommitmentV1::new(&i,&d,None).is_some());}
    #[test] fn irrelevant_material_does_not_change_closure_or_input(){let(a,e,d,c1)=fixture(false);let(b,_,_,c2)=fixture(true);assert_ne!(c1.commitment,c2.commitment);assert_eq!(c1.closure_identity_commitment,c2.closure_identity_commitment);assert_eq!(InputCommitmentV1::from_projection(&a,&e,&c1,&closure_profile(),&d).unwrap().commitment,InputCommitmentV1::from_projection(&b,&e,&c2,&closure_profile(),&d).unwrap().commitment);}
    #[test] fn irrelevant_d6p_receipt_does_not_change_closure_or_input(){let(a,e,d,c1)=fixture(false);let(mut b,_,_,c2)=fixture(false);b.d6p_current_receipt_commitments.insert("irrelevant".into());let c2=compute_dependency_closure(&b,&e,&d,&{let mut p=DependencyClosureProfileV1{profile_id:"cp".into(),version:"1".into(),root_node_ids:["root".into()].into_iter().collect(),required_node_ids:BTreeSet::new(),required_d6p_receipt_commitments:BTreeSet::new(),rules:[DependencyRuleV1{edge_kind:ClaimGraphEdgeKindV1::Supports,from_kind:Some(ClaimGraphNodeKindV1::Statement),to_kind:Some(ClaimGraphNodeKindV1::Evidence),currentness:DependencyCurrentnessV1::Any}].into_iter().collect(),excluded_boundary_policy:"rule-matched semantic edges only".into(),max_nodes:16,max_edges:16,claim_ceiling:D6S_CLAIM_CEILING.into()};p}).unwrap();assert_eq!(c1.closure_identity_commitment,c2.closure_identity_commitment);assert_eq!(InputCommitmentV1::from_projection(&a,&e,&c1,&closure_profile(),&d).unwrap().commitment,InputCommitmentV1::from_projection(&b,&e,&c2,&closure_profile(),&d).unwrap().commitment);}
    #[test]
    fn strict_d6w_ignores_unselected_projection_d6p_receipts() {
        let (baseline_projection, e, d, baseline_closure) = fixture(false);
        let profile = closure_profile();

        let baseline = InputCommitmentV1::from_projection_with_authoritative_d6p(
            &baseline_projection,
            &e,
            &profile,
            &d,
            &[],
            &[],
            None,
        )
        .expect("baseline strict D6W input");

        let mut noisy_projection = baseline_projection.clone();
        noisy_projection
            .d6p_current_receipt_commitments
            .insert("irrelevant-d6p-receipt".into());

        assert!(
            baseline_closure.included_d6p_receipt_commitments.is_empty(),
            "the baseline profile does not select the extra projection receipt"
        );

        let noisy = InputCommitmentV1::from_projection_with_authoritative_d6p(
            &noisy_projection,
            &e,
            &profile,
            &d,
            &[],
            &[],
            None,
        )
        .expect("unselected projection D6P material must not become a D6W dependency");

        assert!(baseline.valid());
        assert!(noisy.valid());
        assert_eq!(
            baseline.commitment, noisy.commitment,
            "unselected D6P candidate material must not perturb the D6W input identity"
        );
        assert!(noisy.d6p_receipts.is_empty());
    }

    #[test]
    fn d6w_constructor_rejects_opaque_selected_d6p_commitment() {
        let (mut p, e, d, _) = fixture(false);
        let profile = DependencyClosureProfileV1 {
            required_d6p_receipt_commitments: BTreeSet::from(["legacy-d6p-receipt".into()]),
            ..closure_profile()
        };
        p.d6p_current_receipt_commitments = profile.required_d6p_receipt_commitments.clone();

        let closure = compute_dependency_closure(&p, &e, &d, &profile).unwrap();
        assert_eq!(
            closure.status,
            qualified_dependency_closure_d6x::DependencyClosureStatusV1::Complete
        );
        assert!(closure.valid());

        assert!(
            InputCommitmentV1::from_projection(&p, &e, &closure, &profile, &d).is_none(),
            "D6W constructor must fail closed instead of returning an invalid input object"
        );
    }

    #[test] fn required_d6p_receipt_is_bound_into_closure(){let(a,e,d,_)=fixture(false);let mut p=DependencyClosureProfileV1{profile_id:"cp".into(),version:"1".into(),root_node_ids:["root".into()].into_iter().collect(),required_node_ids:BTreeSet::new(),required_d6p_receipt_commitments:["r1".into()].into_iter().collect(),rules:[DependencyRuleV1{edge_kind:ClaimGraphEdgeKindV1::Supports,from_kind:Some(ClaimGraphNodeKindV1::Statement),to_kind:Some(ClaimGraphNodeKindV1::Evidence),currentness:DependencyCurrentnessV1::Any}].into_iter().collect(),excluded_boundary_policy:"rule-matched semantic edges only".into(),max_nodes:16,max_edges:16,claim_ceiling:D6S_CLAIM_CEILING.into()};let blocked=compute_dependency_closure(&a,&e,&d,&p).unwrap();assert_eq!(blocked.status,qualified_dependency_closure_d6x::DependencyClosureStatusV1::BlockedMissingDependency);let mut b=a.clone();b.d6p_current_receipt_commitments.insert("r1".into());let complete=compute_dependency_closure(&b,&e,&d,&p).unwrap();assert_eq!(complete.status,qualified_dependency_closure_d6x::DependencyClosureStatusV1::Complete);assert_ne!(blocked.closure_identity_commitment,complete.closure_identity_commitment);}
}
