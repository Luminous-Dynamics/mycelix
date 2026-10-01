use mobility_configuration_qualification::identity_lineage::LineageEdge;
use mobility_configuration_qualification::temporal_reconciliation_witness::TemporalReconciliationWitness;
use mobility_configuration_qualification::witness_revision::{validate_projection_binding, validate_transition};
use serde::{Deserialize, Serialize};
use std::{env, fs};

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Corpus { schema: String, schema_version: String, status: String, cases: Vec<Case> }

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Case {
    id: String,
    operation: String,
    expected: String,
    previous: TemporalReconciliationWitness,
    current: TemporalReconciliationWitness,
    supersession: Option<LineageEdge>,
    projection_ref: Option<mobility_configuration_qualification::identity_lineage::IdentityRef>,
}

#[derive(Debug, Serialize, PartialEq, Eq)]
struct Normalized { id: String, operation: String, expected: String, actual: String }

fn evaluate(case: &Case) -> String {
    let result = match case.operation.as_str() {
        "transition" => validate_transition(&case.previous, &case.current, case.supersession.as_ref()),
        "projection" => case.projection_ref.as_ref()
            .ok_or_else(|| "projection_ref is required".to_string())
            .and_then(|r| validate_projection_binding(r, &case.current)),
        "identity" => case.current.validate(),
        "supersession_preserves_predecessor" => {
            case.previous.validate().and_then(|_| {
                validate_transition(&case.previous, &case.current, case.supersession.as_ref())
            })
        }
        other => Err(format!("unknown operation: {other}")),
    };
    match result {
        Ok(()) => {
            if case.operation == "transition"
                && case.previous.witness_identity == case.current.witness_identity
                && case.previous == case.current
            { "stable".into() } else { "accepted".into() }
        }
        Err(_) => "rejected".into(),
    }
}

fn main() {
    let path = env::args().nth(1).expect("usage: mobility-witness-revision-qualification <corpus>");
    let corpus: Corpus = serde_json::from_str(&fs::read_to_string(&path).expect("read corpus"))
        .expect("parse corpus");
    assert_eq!(corpus.schema, "mobility-reconciliation-witness-revision-executable-v1");
    assert_eq!(corpus.schema_version, "mobility-reconciliation-witness-revision-v1");
    assert_eq!(corpus.status, "semantic-provenance-only");
    assert_eq!(corpus.cases.len(), 16);

    let mut outputs: Vec<_> = corpus.cases.iter().map(|case| {
        Normalized { id: case.id.clone(), operation: case.operation.clone(),
            expected: case.expected.clone(), actual: evaluate(case) }
    }).collect();
    outputs.sort_by(|a,b| a.id.cmp(&b.id));
    for item in &outputs { assert_eq!(item.expected, item.actual, "{} mismatch", item.id); }

    println!("{}", serde_json::to_string_pretty(&serde_json::json!({
        "schema": "mobility-reconciliation-witness-revision-normalized-v1",
        "schema_version": "mobility-reconciliation-witness-revision-v1",
        "status": "semantic-provenance-only",
        "cases": outputs
    })).unwrap());
}
