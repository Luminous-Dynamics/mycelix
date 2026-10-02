use mobility_configuration_qualification::{
    EvidenceState, target_bound_projection::TargetBoundReconciliationProjection,
    temporal_reconciliation_witness::TemporalReconciliationWitness,
};
use serde::{Deserialize, Serialize};
use std::{env, fs};

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Corpus {
    schema: String,
    schema_version: String,
    status: String,
    cases: Vec<Case>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Case {
    id: String,
    operation: String,
    expected: String,
    target: mobility_configuration_qualification::identity_lineage::IdentityRef,
    configuration_scope: mobility_configuration_qualification::identity_lineage::IdentityRef,
    physical_artifact_scope: mobility_configuration_qualification::identity_lineage::IdentityRef,
    witness: TemporalReconciliationWitness,
    projection: TargetBoundReconciliationProjection,
    base_state: EvidenceState,
    expected_state: EvidenceState,
}

#[derive(Debug, Serialize, PartialEq, Eq)]
struct Normalized {
    id: String,
    operation: String,
    expected: String,
    actual: String,
}

fn evaluate(c: &Case) -> String {
    match c.projection.apply(
        &c.target,
        &c.configuration_scope,
        &c.physical_artifact_scope,
        &c.witness,
        &c.base_state,
    ) {
        Ok(actual) if actual == c.expected_state => "accepted".into(),
        _ => "rejected".into(),
    }
}

fn main() {
    let path = env::args().nth(1).expect("corpus path");
    let corpus: Corpus = serde_json::from_str(&fs::read_to_string(path).unwrap()).unwrap();

    assert_eq!(
        corpus.schema,
        "mobility-reconciliation-target-bound-projection-executable-v1"
    );
    assert_eq!(
        corpus.schema_version,
        "mobility-reconciliation-target-bound-projection-v1"
    );
    assert_eq!(corpus.status, "semantic-provenance-only");
    assert_eq!(corpus.cases.len(), 26);

    let expected_ids: Vec<String> = (1..=26).map(|n| format!("TBP-{n:03}")).collect();
    let actual_ids: Vec<String> = corpus.cases.iter().map(|c| c.id.clone()).collect();
    assert_eq!(actual_ids, expected_ids);

    let mut out = Vec::new();
    for c in &corpus.cases {
        let actual = evaluate(c);
        assert_eq!(actual, c.expected, "{} mismatch", c.id);
        out.push(Normalized {
            id: c.id.clone(),
            operation: c.operation.clone(),
            expected: c.expected.clone(),
            actual,
        });
    }
    out.sort_by(|a, b| a.id.cmp(&b.id));

    println!(
        "{}",
        serde_json::to_string_pretty(&serde_json::json!({
            "schema": "mobility-reconciliation-target-bound-projection-normalized-v1",
            "schema_version": "mobility-reconciliation-target-bound-projection-v1",
            "status": "semantic-provenance-only",
            "cases": out
        }))
        .unwrap()
    );
}
