use mobility_configuration_qualification::{
    reconciliation_evidence_projection::ReconciliationEvidenceProjection,
    temporal_reconciliation_witness::TemporalReconciliationWitness,
    EvidenceState, ConflictDisposition, EpistemicDisposition, LifecycleDisposition,
};
use serde::{Deserialize, Serialize};
use std::{env, fs};

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Corpus { schema:String, schema_version:String, status:String, cases:Vec<Case> }
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Case {
    id:String, operation:String, expected:String,
    witness:TemporalReconciliationWitness,
    projection:serde_json::Value,
    base_state:EvidenceState,
    expected_state:EvidenceState,
    required_historical_ref:Option<mobility_configuration_qualification::identity_lineage::IdentityRef>,
}
#[derive(Debug, Serialize, PartialEq, Eq)]
struct Normalized { id:String, operation:String, expected:String, actual:String }

fn projection(v:&serde_json::Value)->Result<ReconciliationEvidenceProjection,String>{
    serde_json::from_value(v.clone()).map_err(|e|e.to_string())
}
fn evaluate(c:&Case)->String{
    let p=match projection(&c.projection){Ok(v)=>v,Err(_)=>return "rejected".into()};
    if let Some(h)=&c.required_historical_ref {
        if p.witness_ref()!=h { return "rejected".into(); }
    }
    match p.apply(&c.witness,&c.base_state) {
        Ok(actual) if actual==c.expected_state => "accepted".into(),
        _ => "rejected".into()
    }
}
fn main(){
    let path=env::args().nth(1).expect("corpus path");
    let corpus:Corpus=serde_json::from_str(&fs::read_to_string(path).unwrap()).unwrap();
    assert_eq!(corpus.schema,"mobility-reconciliation-historical-projection-state-executable-v1");
    assert_eq!(corpus.schema_version,"mobility-reconciliation-historical-projection-state-v1");
    assert_eq!(corpus.status,"semantic-provenance-only");
    assert_eq!(corpus.cases.len(),8);
    let mut out=Vec::new();
    for c in &corpus.cases {
        let actual=evaluate(c);
        assert_eq!(actual,c.expected,"{} mismatch",c.id);
        out.push(Normalized{id:c.id.clone(),operation:c.operation.clone(),expected:c.expected.clone(),actual});
    }
    out.sort_by(|a,b|a.id.cmp(&b.id));
    println!("{}",serde_json::to_string_pretty(&serde_json::json!({
      "schema":"mobility-reconciliation-historical-projection-state-normalized-v1",
      "schema_version":"mobility-reconciliation-historical-projection-state-v1",
      "status":"semantic-provenance-only","cases":out
    })).unwrap());
}
