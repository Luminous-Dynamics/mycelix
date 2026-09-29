//! D6W diagnostic decomposition of D6S commitments. Integrity only; no authority.
use crate::canonical_derivation_receipt::{
    canonical_sha256, CanonicalDerivationReceiptV1, DerivationProfileV1,
    DerivationResultStatusV1, QualifiedProjectionV1, SemanticEnvironmentV1, D6S_CLAIM_CEILING,
};
use serde::{Deserialize, Serialize};

#[path = "qualified_dependency_closure_d6x.rs"]
pub mod qualified_dependency_closure_d6x;
use qualified_dependency_closure_d6x::DependencyClosureCertificateV1;

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
    ) -> Option<Self> {
        if closure.status != qualified_dependency_closure_d6x::DependencyClosureStatusV1::Complete || !closure.valid() {
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
    pub fn recompute(&self) -> String {
        let mut v = self.clone(); v.commitment.clear();
        canonical_sha256("d6w-input", &v)
    }
    pub fn valid(&self) -> bool {
        self.schema_version == D6W_SCHEMA_VERSION
            && !self.source_snapshot.is_empty() && !self.projection.is_empty()
            && !self.environment.is_empty() && !self.dependency_closure.is_empty()
            && !self.nodes.is_empty()
            && self.claim_ceiling == D6S_CLAIM_CEILING && self.commitment == self.recompute()
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
    pub fn valid(&self)->bool{self.schema_version==D6W_SCHEMA_VERSION&&!self.input.is_empty()&&!self.profile.is_empty()&&self.claim_ceiling==D6S_CLAIM_CEILING&&self.commitment==self.recompute()}
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
    pub fn valid(&self)->bool{self.schema_version==D6W_SCHEMA_VERSION&&!self.derivation.is_empty()&&!self.payload.is_empty()&&self.claim_ceiling==D6S_CLAIM_CEILING&&self.commitment==self.recompute()}
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
        closure:&DependencyClosureCertificateV1, trace:Option<String>
    )->bool {
        if !d6s.commitment_matches() || !closure.valid()
            || closure.projection_commitment != p.commitment()
            || closure.semantic_environment_commitment != e.commitment()
            || closure.derivation_profile_commitment != profile.commitment()
            || d6s.projection_commitment!=p.commitment()
            || d6s.semantic_environment_commitment!=e.commitment()
            || d6s.derivation_profile_commitment!=profile.commitment()
            || d6s.source_dkg_snapshot_commitment!=p.source_dkg_snapshot_commitment
            || d6s.input_node_commitments!=p.nodes.values().map(|n|n.node_commitment.clone()).collect()
            || d6s.input_edge_commitments!=p.edges.values().map(|n|n.edge_commitment.clone()).collect()
            || d6s.d6p_current_receipt_commitments!=p.d6p_current_receipt_commitments { return false; }
        let Some(input)=InputCommitmentV1::from_projection(p,e,closure) else{return false};
        if !input.valid(){return false}
        let Some(derivation)=DerivationCommitmentV1::new(&input,profile,trace) else{return false};
        let Some(result)=ResultCommitmentV1::new(&derivation,d6s.result_status,d6s.result_commitment.clone(),d6s.contradiction_preserved,d6s.unresolved_preserved) else{return false};
        let Some(expected)=Self::new(&input,&derivation,&result) else{return false};
        self.valid() && self.input==expected.input && self.derivation==expected.derivation
            && self.result==expected.result && self.claim_ceiling==expected.claim_ceiling && self.commitment==expected.commitment
    }
    pub fn recompute(&self)->String{let mut v=self.clone();v.commitment.clear();canonical_sha256("d6w-receipt",&v)}
    pub fn valid(&self)->bool{self.schema_version==D6W_SCHEMA_VERSION&&!self.input.is_empty()&&!self.derivation.is_empty()&&!self.result.is_empty()&&self.claim_ceiling==D6S_CLAIM_CEILING&&self.commitment==self.recompute()}
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::BTreeSet;
    use crate::canonical_derivation_receipt::{QualifiedEdgeV1,QualifiedNodeV1};
    use crate::evidence_claim_graph::{ClaimGraphEdgeKindV1,ClaimGraphNodeKindV1};
    use qualified_dependency_closure_d6x::{compute_dependency_closure,DependencyClosureProfileV1,DependencyRuleV1,DependencyCurrentnessV1};

    fn fixture(extra:bool)->(QualifiedProjectionV1,SemanticEnvironmentV1,DerivationProfileV1,DependencyClosureCertificateV1){
        let e=SemanticEnvironmentV1{semantic_profile_id:"sem".into(),semantic_profile_version:"1".into(),current_frontier_root:Some("frontier".into()),d6p_eligibility_context_root:None,d6n_observer_context_root:None,d6o_lifecycle_context_root:None,membership_authority_scope_root:None,dependency_snapshot_root:Some("snapshot".into()),historical_cutoff:None,policy_version:"policy".into(),claim_ceiling:D6S_CLAIM_CEILING.into()};
        let d=DerivationProfileV1{profile_id:"d".into(),version:"1".into(),rule_ids:["r".into()].into_iter().collect(),permits_recursive_fixpoint:false,claim_ceiling:D6S_CLAIM_CEILING.into()};
        let mut nodes=BTreeMap::new();
        for (id,k) in [("root",ClaimGraphNodeKindV1::Statement),("dep",ClaimGraphNodeKindV1::Evidence)]{nodes.insert(id.into(),QualifiedNodeV1{node_id:id.into(),kind:k,node_commitment:format!("n-{id}"),historical_only:false,current_frontier_root:Some("frontier".into()),claim_ceiling:D6S_CLAIM_CEILING.into()});}
        if extra{nodes.insert("noise".into(),QualifiedNodeV1{node_id:"noise".into(),kind:ClaimGraphNodeKindV1::Source,node_commitment:"n-noise".into(),historical_only:false,current_frontier_root:Some("frontier".into()),claim_ceiling:D6S_CLAIM_CEILING.into()});}
        let mut edges=BTreeMap::new();
        edges.insert("e1".into(),QualifiedEdgeV1{edge_id:"e1".into(),from_node_id:"root".into(),to_node_id:"dep".into(),kind:ClaimGraphEdgeKindV1::Supports,edge_commitment:"e-e1".into(),claim_ceiling:D6S_CLAIM_CEILING.into()});
        if extra{edges.insert("noise-edge".into(),QualifiedEdgeV1{edge_id:"noise-edge".into(),from_node_id:"noise".into(),to_node_id:"dep".into(),kind:ClaimGraphEdgeKindV1::Provenance,edge_commitment:"e-noise".into(),claim_ceiling:D6S_CLAIM_CEILING.into()});}
        let p=QualifiedProjectionV1{projection_id:"p".into(),projection_version:"1".into(),canonicalization_version:"D6S-CANON-1".into(),source_dkg_snapshot_commitment:"snapshot".into(),nodes,edges,d6p_current_receipt_commitments:BTreeSet::new(),d6n_context_commitment:None,d6o_context_commitment:None,semantic_environment_commitment:e.commitment(),derivation_profile_commitment:d.commitment(),claim_ceiling:D6S_CLAIM_CEILING.into()};
        let rule=DependencyRuleV1{edge_kind:ClaimGraphEdgeKindV1::Supports,from_kind:Some(ClaimGraphNodeKindV1::Statement),to_kind:Some(ClaimGraphNodeKindV1::Evidence),currentness:DependencyCurrentnessV1::Any};
        let cp=DependencyClosureProfileV1{profile_id:"cp".into(),version:"1".into(),root_node_ids:["root".into()].into_iter().collect(),required_node_ids:BTreeSet::new(),required_d6p_receipt_commitments:BTreeSet::new(),rules:[rule].into_iter().collect(),excluded_boundary_policy:"rule-matched semantic edges only".into(),max_nodes:16,max_edges:16,claim_ceiling:D6S_CLAIM_CEILING.into()};
        let c=compute_dependency_closure(&p,&e,&d,&cp).unwrap();
        (p,e,d,c)
    }
    #[test] fn closure_is_bound_into_input(){let(p,e,d,c)=fixture(false);let i=InputCommitmentV1::from_projection(&p,&e,&c).unwrap();assert!(i.valid());assert!(!i.dependency_closure.is_empty());let x=LayeredReceiptV1::new(&i,&DerivationCommitmentV1::new(&i,&d,None).unwrap(),&ResultCommitmentV1::new(&DerivationCommitmentV1::new(&i,&d,None).unwrap(),DerivationResultStatusV1::Supported,"x".into(),false,false).unwrap());assert!(x.is_some());}
    #[test] fn blocked_closure_cannot_enter_d6w_input(){
        let(p,e,d,mut c)=fixture(false);
        c.status=qualified_dependency_closure_d6x::DependencyClosureStatusV1::BlockedResourceLimit;
        c.commitment=c.recompute();
        assert!(InputCommitmentV1::from_projection(&p,&e,&c).is_none());
        let _=d;
    }
    #[test] fn closure_mutation_changes_input(){let(p,e,d,c)=fixture(false);let mut i=InputCommitmentV1::from_projection(&p,&e,&c).unwrap();let old=i.commitment.clone();i.dependency_closure="tampered".into();i.commitment=i.recompute();assert_ne!(old,i.commitment);assert!(DerivationCommitmentV1::new(&i,&d,None).is_some());}
    #[test] fn irrelevant_material_does_not_change_closure_or_input(){let(a,e,d,c1)=fixture(false);let(b,_,_,c2)=fixture(true);assert_ne!(c1.commitment,c2.commitment);assert_eq!(c1.closure_identity_commitment,c2.closure_identity_commitment);assert_eq!(InputCommitmentV1::from_projection(&a,&e,&c1).unwrap().commitment,InputCommitmentV1::from_projection(&b,&e,&c2).unwrap().commitment);}
    #[test] fn irrelevant_d6p_receipt_does_not_change_closure_or_input(){let(a,e,d,c1)=fixture(false);let(mut b,_,_,c2)=fixture(false);b.d6p_current_receipt_commitments.insert("irrelevant".into());let c2=compute_dependency_closure(&b,&e,&d,&{let mut p=DependencyClosureProfileV1{profile_id:"cp".into(),version:"1".into(),root_node_ids:["root".into()].into_iter().collect(),required_node_ids:BTreeSet::new(),required_d6p_receipt_commitments:BTreeSet::new(),rules:[DependencyRuleV1{edge_kind:ClaimGraphEdgeKindV1::Supports,from_kind:Some(ClaimGraphNodeKindV1::Statement),to_kind:Some(ClaimGraphNodeKindV1::Evidence),currentness:DependencyCurrentnessV1::Any}].into_iter().collect(),excluded_boundary_policy:"rule-matched semantic edges only".into(),max_nodes:16,max_edges:16,claim_ceiling:D6S_CLAIM_CEILING.into()};p}).unwrap();assert_eq!(c1.closure_identity_commitment,c2.closure_identity_commitment);assert_eq!(InputCommitmentV1::from_projection(&a,&e,&c1).unwrap().commitment,InputCommitmentV1::from_projection(&b,&e,&c2).unwrap().commitment);}
    #[test] fn required_d6p_receipt_is_bound_into_closure(){let(a,e,d,_)=fixture(false);let mut p=DependencyClosureProfileV1{profile_id:"cp".into(),version:"1".into(),root_node_ids:["root".into()].into_iter().collect(),required_node_ids:BTreeSet::new(),required_d6p_receipt_commitments:["r1".into()].into_iter().collect(),rules:[DependencyRuleV1{edge_kind:ClaimGraphEdgeKindV1::Supports,from_kind:Some(ClaimGraphNodeKindV1::Statement),to_kind:Some(ClaimGraphNodeKindV1::Evidence),currentness:DependencyCurrentnessV1::Any}].into_iter().collect(),excluded_boundary_policy:"rule-matched semantic edges only".into(),max_nodes:16,max_edges:16,claim_ceiling:D6S_CLAIM_CEILING.into()};let blocked=compute_dependency_closure(&a,&e,&d,&p).unwrap();assert_eq!(blocked.status,qualified_dependency_closure_d6x::DependencyClosureStatusV1::BlockedMissingDependency);let mut b=a.clone();b.d6p_current_receipt_commitments.insert("r1".into());let complete=compute_dependency_closure(&b,&e,&d,&p).unwrap();assert_eq!(complete.status,qualified_dependency_closure_d6x::DependencyClosureStatusV1::Complete);assert_ne!(blocked.closure_identity_commitment,complete.closure_identity_commitment);}
}
