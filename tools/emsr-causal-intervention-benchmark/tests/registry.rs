use serde::Deserialize;
use std::collections::BTreeSet;

const REGISTRY: &str = include_str!("../fixtures/causal-intervention-cases-0.1.json");

#[derive(Deserialize)]
struct CaseRegistry {
    schema: String,
    status: String,
    claim_ceiling: String,
    case_ids: Vec<String>,
}

#[test]
fn public_intervention_case_registry_is_exact_and_unique() {
    let registry: CaseRegistry = serde_json::from_str(REGISTRY).expect("registry JSON");
    assert_eq!(registry.schema, "emsr.causal-intervention-public-conformance/0.1");
    assert_eq!(registry.status, "PUBLIC_SYNTHETIC_SOFTWARE_CONFORMANCE");
    assert!(!registry.claim_ceiling.trim().is_empty());
    assert_eq!(registry.case_ids.len(), 12);
    let unique: BTreeSet<&str> = registry.case_ids.iter().map(String::as_str).collect();
    assert_eq!(unique.len(), registry.case_ids.len());
}
