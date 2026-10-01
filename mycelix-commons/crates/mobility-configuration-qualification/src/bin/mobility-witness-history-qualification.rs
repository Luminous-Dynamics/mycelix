use mobility_configuration_qualification::identity_lineage::{IdentityRef, LineageEdge};
use mobility_configuration_qualification::temporal_reconciliation_witness::TemporalReconciliationWitness;
use mobility_configuration_qualification::witness_revision::{validate_projection_binding, validate_transition};
use serde::{Deserialize, Serialize};
use std::{env, fs};

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Corpus {
    schema: String,
    schema_version: String,
    status: String,
    cases: Vec<Case>,
    chain: Vec<TemporalReconciliationWitness>,
    chain_projection_refs: Vec<IdentityRef>,
    chain_supersessions: Vec<LineageEdge>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Case {
    id: String,
    operation: String,
    expected: String,
    previous: TemporalReconciliationWitness,
    current: TemporalReconciliationWitness,
    supersession: Option<LineageEdge>,
    predecessor_projection_ref: Option<IdentityRef>,
    successor_projection_ref: Option<IdentityRef>,
}

#[derive(Debug, Serialize, PartialEq, Eq)]
struct Normalized { id: String, operation: String, expected: String, actual: String }

fn evaluate(case: &Case) -> String {
    if case.operation != "history" { return "rejected".into(); }
    let result = validate_transition(&case.previous, &case.current, case.supersession.as_ref())
        .and_then(|_| {
            let predecessor = case.predecessor_projection_ref.as_ref()
                .ok_or_else(|| "predecessor projection reference required".to_string())?;
            validate_projection_binding(predecessor, &case.previous)
        })
        .and_then(|_| {
            let successor = case.successor_projection_ref.as_ref()
                .ok_or_else(|| "successor projection reference required".to_string())?;
            validate_projection_binding(successor, &case.current)
        });
    match result {
        Ok(()) => "accepted".into(),
        Err(_) => "rejected".into(),
    }
}

fn validate_chain(corpus: &Corpus) -> Result<(), String> {
    if corpus.chain.len() != 3 || corpus.chain_projection_refs.len() != 3 || corpus.chain_supersessions.len() != 2 {
        return Err("expected exactly three witness generations and three projection bindings".into());
    }
    for witness in &corpus.chain { witness.validate()?; }
    let refs = &corpus.chain_projection_refs;
    for (witness, projection_ref) in corpus.chain.iter().zip(refs) {
        validate_projection_binding(projection_ref, witness)?;
    }
    for (pair, edge) in corpus.chain.windows(2).zip(&corpus.chain_supersessions) {
        edge.validate()?;
        validate_transition(&pair[0], &pair[1], Some(edge))?;
    }
    Ok(())
}

fn main() {
    let path = env::args().nth(1).expect("usage: mobility-witness-history-qualification <corpus>");
    let corpus: Corpus = serde_json::from_str(&fs::read_to_string(&path).expect("read corpus"))
        .expect("parse corpus");
    assert_eq!(corpus.schema, "mobility-reconciliation-witness-history-executable-v1");
    assert_eq!(corpus.schema_version, "mobility-reconciliation-witness-history-v1");
    assert_eq!(corpus.status, "semantic-provenance-only");
    assert_eq!(corpus.cases.len(), 12);
    assert_eq!(corpus.chain.len(), 3);
    validate_chain(&corpus).expect("three-generation history must remain preserved");

    let mut outputs: Vec<_> = corpus.cases.iter().map(|case| {
        Normalized { id: case.id.clone(), operation: case.operation.clone(),
            expected: case.expected.clone(), actual: evaluate(case) }
    }).collect();
    outputs.sort_by(|a,b| a.id.cmp(&b.id));
    for item in &outputs { assert_eq!(item.expected, item.actual, "{} mismatch", item.id); }

    println!("{}", serde_json::to_string_pretty(&serde_json::json!({
        "schema": "mobility-reconciliation-witness-history-normalized-v1",
        "schema_version": "mobility-reconciliation-witness-history-v1",
        "status": "semantic-provenance-only",
        "cases": outputs
    })).unwrap());
}
