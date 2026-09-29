//! D6W diagnostic decomposition of D6S commitments. Integrity only; no authority.
use crate::canonical_derivation_receipt::{canonical_sha256, CanonicalDerivationReceiptV1, DerivationProfileV1, DerivationResultStatusV1, QualifiedProjectionV1, SemanticEnvironmentV1, D6S_CLAIM_CEILING};
use serde::{Deserialize, Serialize};

pub const D6W_SCHEMA_VERSION: &str = "D6W-1";

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct InputCommitmentV1 {
    pub schema_version: String,
    pub source_snapshot: String,
    pub projection: String,
    pub environment: String,
    pub nodes: Vec<String>,
    pub edges: Vec<String>,
    pub d6p_receipts: Vec<String>,
    pub claim_ceiling: String,
    pub commitment: String,
}
impl InputCommitmentV1 {
    pub fn from_projection(p: &QualifiedProjectionV1, e: &SemanticEnvironmentV1) -> Self {
        let mut v = Self { schema_version:D6W_SCHEMA_VERSION.into(), source_snapshot:p.source_dkg_snapshot_commitment.clone(), projection:p.commitment(), environment:e.commitment(), nodes:p.nodes.values().map(|n|n.node_commitment.clone()).collect(), edges:p.edges.values().map(|e|e.edge_commitment.clone()).collect(), d6p_receipts:p.d6p_current_receipt_commitments.iter().cloned().collect(), claim_ceiling:D6S_CLAIM_CEILING.into(), commitment:String::new() };
        v.commitment=v.recompute(); v
    }
    pub fn recompute(&self)->String { let mut v=self.clone(); v.commitment.clear(); canonical_sha256("d6w-input",&v) }
    pub fn valid(&self)->bool { self.schema_version==D6W_SCHEMA_VERSION && !self.source_snapshot.is_empty() && !self.projection.is_empty() && !self.environment.is_empty() && !self.nodes.is_empty() && !self.edges.is_empty() && self.claim_ceiling==D6S_CLAIM_CEILING && self.commitment==self.recompute() }
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
    pub payload:String, pub contradiction:bool, pub unresolved:bool,
    pub claim_ceiling:String, pub commitment:String,
}
impl ResultCommitmentV1 {
    pub fn new(d:&DerivationCommitmentV1,status:DerivationResultStatusV1,payload:String,contradiction:bool,unresolved:bool)->Option<Self>{
        if !d.valid()||payload.trim().is_empty()||(status==DerivationResultStatusV1::Supported&&(contradiction||unresolved))||(status==DerivationResultStatusV1::Disputed&&!contradiction)||(matches!(status,DerivationResultStatusV1::Unresolved|DerivationResultStatusV1::BlockedMissingEvidence|DerivationResultStatusV1::BlockedCurrentness|DerivationResultStatusV1::BlockedQualification)&&!unresolved){return None}
        let mut v=Self{schema_version:D6W_SCHEMA_VERSION.into(),derivation:d.commitment.clone(),status,payload,contradiction,unresolved,claim_ceiling:D6S_CLAIM_CEILING.into(),commitment:String::new()};
        v.commitment=v.recompute();Some(v)
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
    /// Reconstruct and cross-check the D6W layers against an exact D6S receipt.
    pub fn verifies_d6s(&self, d6s:&CanonicalDerivationReceiptV1, p:&QualifiedProjectionV1, e:&SemanticEnvironmentV1, profile:&DerivationProfileV1, trace:Option<String>)->bool {
        if !d6s.commitment_matches() || !p.structurally_valid() || !e.structurally_valid() || !profile.structurally_valid()
            || d6s.projection_commitment!=p.commitment() || d6s.semantic_environment_commitment!=e.commitment()
            || d6s.derivation_profile_commitment!=profile.commitment() || d6s.source_dkg_snapshot_commitment!=p.source_dkg_snapshot_commitment
            || d6s.input_node_commitments!=p.nodes.values().map(|n|n.node_commitment.clone()).collect()
            || d6s.input_edge_commitments!=p.edges.values().map(|n|n.edge_commitment.clone()).collect()
            || d6s.d6p_current_receipt_commitments!=p.d6p_current_receipt_commitments { return false; }
        let input=InputCommitmentV1::from_projection(p,e);
        if !input.valid(){return false}
        let Some(derivation)=DerivationCommitmentV1::new(&input,profile,trace) else{return false};
        let Some(result)=ResultCommitmentV1::new(&derivation,d6s.result_status,d6s.result_commitment.clone(),d6s.contradiction_preserved,d6s.unresolved_preserved) else{return false};
        let Some(expected)=Self::new(&input,&derivation,&result) else{return false};
        self.valid() && self.input==expected.input && self.derivation==expected.derivation
            && self.result==expected.result && self.claim_ceiling==expected.claim_ceiling
            && self.commitment==expected.commitment
    }
    pub fn recompute(&self)->String{let mut v=self.clone();v.commitment.clear();canonical_sha256("d6w-receipt",&v)}
    pub fn valid(&self)->bool{self.schema_version==D6W_SCHEMA_VERSION&&!self.input.is_empty()&&!self.derivation.is_empty()&&!self.result.is_empty()&&self.claim_ceiling==D6S_CLAIM_CEILING&&self.commitment==self.recompute()}
}

#[cfg(test)] mod tests {
 use super::*;
 fn input()->InputCommitmentV1{let mut x=InputCommitmentV1{schema_version:D6W_SCHEMA_VERSION.into(),source_snapshot:"s".into(),projection:"p".into(),environment:"e".into(),nodes:vec!["n".into()],edges:vec!["edge".into()],d6p_receipts:vec![],claim_ceiling:D6S_CLAIM_CEILING.into(),commitment:String::new()};x.commitment=x.recompute();x}
 fn profile()->DerivationProfileV1{DerivationProfileV1{profile_id:"p".into(),version:"1".into(),rule_ids:["r".into()].into_iter().collect(),permits_recursive_fixpoint:false,claim_ceiling:D6S_CLAIM_CEILING.into()}}
 #[test] fn rule_change_preserves_input_identity(){let i=input();let a=DerivationCommitmentV1::new(&i,&profile(),None).unwrap();let mut p=profile();p.rule_ids.insert("r2".into());let b=DerivationCommitmentV1::new(&i,&p,None).unwrap();assert_eq!(a.input,b.input);assert_ne!(a.commitment,b.commitment);}
 #[test] fn result_change_preserves_derivation_identity(){let i=input();let d=DerivationCommitmentV1::new(&i,&profile(),None).unwrap();let a=ResultCommitmentV1::new(&d,DerivationResultStatusV1::Supported,"a".into(),false,false).unwrap();let b=ResultCommitmentV1::new(&d,DerivationResultStatusV1::Supported,"b".into(),false,false).unwrap();assert_eq!(a.derivation,b.derivation);assert_ne!(a.commitment,b.commitment);}
 #[test] fn receipt_binds_layers_and_ceiling(){let i=input();let d=DerivationCommitmentV1::new(&i,&profile(),None).unwrap();let r=ResultCommitmentV1::new(&d,DerivationResultStatusV1::Supported,"payload".into(),false,false).unwrap();let receipt=LayeredReceiptV1::new(&i,&d,&r).unwrap();assert!(receipt.valid());let mut widened=receipt.clone();widened.claim_ceiling="Production".into();assert!(!widened.valid());}
}
