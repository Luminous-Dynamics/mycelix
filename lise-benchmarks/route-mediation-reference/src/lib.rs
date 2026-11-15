#![forbid(unsafe_code)]

//! Finite-world reference evaluator for route mediation claims.
//!
//! This crate enumerates all simple effect-capable paths in a bounded synthetic
//! graph and checks whether every path crosses a current PEP receipt bound to
//! the exact principal, action, purpose, resource, operation and policy.
//!
//! It does not prove that a real-world route inventory is complete. That
//! claim requires an independently sourced, authenticated owner inventory.
//! Its bundled fixture receipt is synthetic and untrusted.

use serde_json::{Value, json};
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet, HashSet};

pub const SUITE_ID: &str = "LISE-ROUTE-MEDIATION-v0.1";
pub const EVALUATOR_VERSION: &str = "route-mediation-reference-v0.1";
const MAX_NODES: usize = 256;
const MAX_EDGES: usize = 1024;
const MAX_PATHS: usize = 2048;
const MAX_PATH_DEPTH: usize = 128;

fn sha256(bytes: &[u8]) -> String {
    format!("{:x}", Sha256::digest(bytes))
}

fn object<'a>(value: &'a Value, key: &str, context: &str) -> Result<&'a Value, String> {
    let member = value
        .get(key)
        .ok_or_else(|| format!("{context}: missing '{key}'"))?;
    if !member.is_object() {
        return Err(format!("{context}: '{key}' must be an object"));
    }
    Ok(member)
}

fn array<'a>(value: &'a Value, key: &str, context: &str) -> Result<&'a Vec<Value>, String> {
    value
        .get(key)
        .and_then(Value::as_array)
        .ok_or_else(|| format!("{context}: '{key}' must be an array"))
}

fn string<'a>(value: &'a Value, key: &str, context: &str) -> Result<&'a str, String> {
    value
        .get(key)
        .and_then(Value::as_str)
        .filter(|s| !s.trim().is_empty())
        .ok_or_else(|| format!("{context}: '{key}' must be a nonempty string"))
}

fn bool_is(value: &Value, key: &str, expected: bool) -> bool {
    value.get(key).and_then(Value::as_bool) == Some(expected)
}

fn is_timestamp(value: &str) -> bool {
    let b = value.as_bytes();
    b.len() == 20
        && b[4] == b'-'
        && b[7] == b'-'
        && b[10] == b'T'
        && b[13] == b':'
        && b[16] == b':'
        && b[19] == b'Z'
        && b.iter()
            .enumerate()
            .all(|(i, c)| matches!(i, 4 | 7 | 10 | 13 | 16 | 19) || c.is_ascii_digit())
}

fn timestamp(value: &Value, key: &str, context: &str) -> Result<&str, String> {
    let result = string(value, key, context)?;
    if !is_timestamp(result) {
        return Err(format!("{context}: '{key}' must be YYYY-MM-DDTHH:MM:SSZ"));
    }
    Ok(result)
}

#[derive(Clone, Debug)]
struct Scope {
    principal_ref: String,
    action: String,
    purpose: String,
    resource_ref: String,
    operation_digest: String,
    policy_digest: String,
    action_at: String,
    known_at: String,
}

#[derive(Clone, Debug)]
struct Edge {
    edge_id: String,
    from: String,
    to: String,
}

#[derive(Clone, Debug)]
struct Path {
    nodes: Vec<String>,
    edges: Vec<String>,
}

fn parse_scope(fixture: &Value) -> Result<Scope, String> {
    let value = object(fixture, "scope", "fixture")?;
    Ok(Scope {
        principal_ref: string(value, "principal_ref", "scope")?.to_owned(),
        action: string(value, "action", "scope")?.to_owned(),
        purpose: string(value, "purpose", "scope")?.to_owned(),
        resource_ref: string(value, "resource_ref", "scope")?.to_owned(),
        operation_digest: string(value, "operation_digest", "scope")?.to_owned(),
        policy_digest: string(value, "policy_digest", "scope")?.to_owned(),
        action_at: timestamp(value, "action_at", "scope")?.to_owned(),
        known_at: timestamp(value, "known_at", "scope")?.to_owned(),
    })
}

fn canonical_graph_digest(graph: &Value) -> Result<String, String> {
    let mut normalized = graph.clone();
    let nodes = normalized
        .get_mut("nodes")
        .and_then(Value::as_array_mut)
        .ok_or_else(|| "graph: 'nodes' must be an array".to_owned())?;
    nodes.sort_by(|a, b| {
        a.get("node_id")
            .and_then(Value::as_str)
            .cmp(&b.get("node_id").and_then(Value::as_str))
    });
    let edges = normalized
        .get_mut("edges")
        .and_then(Value::as_array_mut)
        .ok_or_else(|| "graph: 'edges' must be an array".to_owned())?;
    edges.sort_by(|a, b| {
        a.get("edge_id")
            .and_then(Value::as_str)
            .cmp(&b.get("edge_id").and_then(Value::as_str))
    });
    for key in ["entry_points", "protected_effect_nodes"] {
        let values = normalized
            .get_mut(key)
            .and_then(Value::as_array_mut)
            .ok_or_else(|| format!("graph: '{key}' must be an array"))?;
        values.sort_by(|a, b| a.as_str().cmp(&b.as_str()));
    }
    let bytes = serde_json::to_vec(&normalized).map_err(|e| e.to_string())?;
    Ok(sha256(&bytes))
}

fn parse_graph(
    graph: &Value,
) -> Result<
    (
        BTreeMap<String, String>,
        Vec<Edge>,
        Vec<String>,
        Vec<String>,
    ),
    String,
> {
    let raw_nodes = array(graph, "nodes", "graph")?;
    let raw_edges = array(graph, "edges", "graph")?;
    if raw_nodes.is_empty() || raw_nodes.len() > MAX_NODES {
        return Err(format!("graph: node count must be 1..={MAX_NODES}"));
    }
    if raw_edges.len() > MAX_EDGES {
        return Err(format!("graph: edge count exceeds {MAX_EDGES}"));
    }

    let mut nodes = BTreeMap::new();
    for node in raw_nodes {
        let id = string(node, "node_id", "graph node")?.to_owned();
        let kind = string(node, "kind", "graph node")?.to_owned();
        if nodes.insert(id.clone(), kind).is_some() {
            return Err(format!("graph: duplicate node id '{id}'"));
        }
    }

    let mut edge_ids = BTreeSet::new();
    let mut edges = Vec::new();
    for raw in raw_edges {
        let edge_id = string(raw, "edge_id", "graph edge")?.to_owned();
        let from = string(raw, "from", "graph edge")?.to_owned();
        let to = string(raw, "to", "graph edge")?.to_owned();
        if !edge_ids.insert(edge_id.clone()) {
            return Err(format!("graph: duplicate edge id '{edge_id}'"));
        }
        if !nodes.contains_key(&from) || !nodes.contains_key(&to) {
            return Err(format!("graph: edge '{edge_id}' has a dangling endpoint"));
        }
        let enabled = raw
            .get("enabled")
            .and_then(Value::as_bool)
            .ok_or_else(|| format!("graph edge '{edge_id}': enabled must be boolean"))?;
        let effect_capable = raw
            .get("effect_capable")
            .and_then(Value::as_bool)
            .ok_or_else(|| format!("graph edge '{edge_id}': effect_capable must be boolean"))?;
        if enabled && effect_capable {
            edges.push(Edge { edge_id, from, to });
        }
    }
    edges.sort_by(|a, b| a.edge_id.cmp(&b.edge_id));

    let entry_points = parse_string_array(graph, "entry_points", "graph")?;
    let protected = parse_string_array(graph, "protected_effect_nodes", "graph")?;
    for entry in &entry_points {
        if !nodes.contains_key(entry) {
            return Err(format!("graph: entry point '{entry}' is not a known node"));
        }
    }
    for target in &protected {
        if !nodes.contains_key(target) {
            return Err(format!(
                "graph: protected effect node '{target}' is not a known node"
            ));
        }
    }
    if entry_points.is_empty() || protected.is_empty() {
        return Err("graph: entry_points and protected_effect_nodes must be nonempty".to_owned());
    }
    Ok((nodes, edges, entry_points, protected))
}

fn parse_string_array(value: &Value, key: &str, context: &str) -> Result<Vec<String>, String> {
    let raw = array(value, key, context)?;
    let mut seen = BTreeSet::new();
    let mut result = Vec::new();
    for item in raw {
        let id = item
            .as_str()
            .filter(|s| !s.trim().is_empty())
            .ok_or_else(|| format!("{context}: '{key}' must contain nonempty strings"))?;
        if !seen.insert(id.to_owned()) {
            return Err(format!("{context}: duplicate value '{id}' in '{key}'"));
        }
        result.push(id.to_owned());
    }
    result.sort();
    Ok(result)
}

fn inventory_receipt_valid(fixture: &Value, graph_digest: &str, scope: &Scope) -> bool {
    if fixture.get("synthetic").and_then(Value::as_bool) != Some(true) {
        return false;
    }
    let Some(receipt) = fixture.get("inventory_receipt") else {
        return false;
    };
    let fields = [
        ("principal_ref", &scope.principal_ref),
        ("action", &scope.action),
        ("purpose", &scope.purpose),
        ("resource_ref", &scope.resource_ref),
        ("operation_digest", &scope.operation_digest),
        ("policy_digest", &scope.policy_digest),
    ];
    if fields
        .iter()
        .any(|(key, expected)| receipt.get(*key).and_then(Value::as_str) != Some(expected.as_str()))
    {
        return false;
    }
    receipt.get("graph_sha256").and_then(Value::as_str) == Some(graph_digest)
        && receipt.get("completeness_state").and_then(Value::as_str) == Some("complete_for_scope")
        && bool_is(receipt, "query_scope_exact", true)
        && bool_is(receipt, "independent_inventory_source", true)
        && receipt.get("verification_state").and_then(Value::as_str)
            == Some("fixture_assertion_not_cryptographically_verified")
        && receipt.get("issuer_trust").and_then(Value::as_str)
            == Some("untrusted_synthetic_fixture")
        && receipt
            .get("inventory_source_ref")
            .and_then(Value::as_str)
            .is_some_and(|s| !s.is_empty())
        && receipt
            .get("independent_observer_ref")
            .and_then(Value::as_str)
            .is_some_and(|s| !s.is_empty())
        && timestamp(receipt, "coverage_from", "inventory receipt")
            .is_ok_and(|v| v <= scope.action_at.as_str())
        && timestamp(receipt, "coverage_through", "inventory receipt")
            .is_ok_and(|v| v >= scope.action_at.as_str())
        && timestamp(receipt, "observed_at", "inventory receipt")
            .is_ok_and(|v| v <= scope.known_at.as_str())
}

struct PathEnumerator<'a> {
    protected: &'a BTreeSet<String>,
    adjacency: &'a BTreeMap<String, Vec<Edge>>,
    output: Vec<Path>,
}

impl PathEnumerator<'_> {
    fn walk(
        &mut self,
        current: &str,
        visiting: &mut HashSet<String>,
        nodes: &mut Vec<String>,
        edges: &mut Vec<String>,
    ) -> Result<(), String> {
        if nodes.len() > MAX_PATH_DEPTH {
            return Err(format!("path depth exceeded {MAX_PATH_DEPTH}"));
        }
        if self.protected.contains(current) {
            if self.output.len() >= MAX_PATHS {
                return Err(format!("path count exceeded {MAX_PATHS}"));
            }
            self.output.push(Path {
                nodes: nodes.clone(),
                edges: edges.clone(),
            });
            return Ok(());
        }
        if let Some(next) = self.adjacency.get(current) {
            for edge in next {
                if !visiting.insert(edge.to.clone()) {
                    continue;
                }
                nodes.push(edge.to.clone());
                edges.push(edge.edge_id.clone());
                self.walk(&edge.to, visiting, nodes, edges)?;
                edges.pop();
                nodes.pop();
                visiting.remove(&edge.to);
            }
        }
        Ok(())
    }
}

fn matching_mediator(
    fixture: &Value,
    path_nodes: &[String],
    scope: &Scope,
    node_kinds: &BTreeMap<String, String>,
) -> Option<String> {
    let mediators = fixture.get("mediators")?.as_array()?;
    let mut candidates = Vec::new();
    for mediator in mediators {
        let node_ref = mediator.get("node_ref").and_then(Value::as_str)?;
        if !path_nodes.iter().any(|node| node == node_ref)
            || node_kinds.get(node_ref).map(String::as_str) != Some("policy_enforcement_point")
        {
            continue;
        }
        let scope_matches = mediator.get("principal_ref").and_then(Value::as_str)
            == Some(&scope.principal_ref)
            && mediator.get("action").and_then(Value::as_str) == Some(&scope.action)
            && mediator.get("purpose").and_then(Value::as_str) == Some(&scope.purpose)
            && mediator.get("resource_ref").and_then(Value::as_str) == Some(&scope.resource_ref)
            && mediator.get("operation_digest").and_then(Value::as_str)
                == Some(&scope.operation_digest)
            && mediator.get("policy_digest").and_then(Value::as_str) == Some(&scope.policy_digest)
            && mediator.get("state").and_then(Value::as_str) == Some("active");
        if !scope_matches {
            continue;
        }
        if !timestamp(mediator, "valid_from", "mediator")
            .is_ok_and(|v| v <= scope.action_at.as_str())
            || !timestamp(mediator, "valid_through", "mediator")
                .is_ok_and(|v| v >= scope.action_at.as_str())
            || !timestamp(mediator, "observed_at", "mediator")
                .is_ok_and(|v| v <= scope.known_at.as_str())
        {
            continue;
        }
        if let Some(id) = mediator.get("mediator_ref").and_then(Value::as_str) {
            candidates.push(id.to_owned());
        }
    }
    candidates.sort();
    candidates.into_iter().next()
}

fn invalid_report(error: String) -> Value {
    json!({
        "suite_id": SUITE_ID,
        "evaluator_version": EVALUATOR_VERSION,
        "status": "INVALID_INPUT",
        "dispatch_eligible": false,
        "errors": [error],
        "claim_ceiling": "Finite synthetic route-graph structural evaluation only."
    })
}

/// Evaluate the finite route inventory. This function never authorizes dispatch.
pub fn evaluate(fixture: &Value) -> Value {
    let result = (|| -> Result<Value, String> {
        let scope = parse_scope(fixture)?;
        let graph = object(fixture, "graph", "fixture")?;
        let graph_digest = canonical_graph_digest(graph)?;
        let (node_kinds, edges, entries, protected) = parse_graph(graph)?;

        let receipt_valid = inventory_receipt_valid(fixture, &graph_digest, &scope);
        if !receipt_valid {
            return Ok(json!({
                "suite_id": SUITE_ID,
                "evaluator_version": EVALUATOR_VERSION,
                "status": "INDETERMINATE",
                "graph_sha256": graph_digest,
                "inventory_receipt_valid": false,
                "path_count": 0,
                "paths": [],
                "dispatch_eligible": false,
                "blockers": ["Independent, exact-scope, complete inventory receipt is missing, stale, mismatched or invalid."],
                "claim_ceiling": "No non-bypassability claim without an independently complete route inventory."
            }));
        }

        let mut adjacency: BTreeMap<String, Vec<Edge>> = BTreeMap::new();
        for edge in edges {
            adjacency.entry(edge.from.clone()).or_default().push(edge);
        }
        for outgoing in adjacency.values_mut() {
            outgoing.sort_by(|a, b| a.edge_id.cmp(&b.edge_id));
        }

        let protected_set: BTreeSet<String> = protected.into_iter().collect();
        let mut enumerator = PathEnumerator {
            protected: &protected_set,
            adjacency: &adjacency,
            output: Vec::new(),
        };
        for entry in entries {
            let mut visiting = HashSet::new();
            let mut path_nodes = vec![entry.clone()];
            let mut path_edges = Vec::new();
            visiting.insert(entry.clone());
            enumerator.walk(&entry, &mut visiting, &mut path_nodes, &mut path_edges)?;
        }
        let paths = enumerator.output;

        if paths.is_empty() {
            return Ok(json!({
                "suite_id": SUITE_ID,
                "evaluator_version": EVALUATOR_VERSION,
                "status": "NO_REACHABLE_EFFECT_PATH",
                "graph_sha256": graph_digest,
                "inventory_receipt_valid": true,
                "path_count": 0,
                "paths": [],
                "dispatch_eligible": false,
                "blockers": ["No effect-capable path from the declared entry points to a protected effect node was found."],
                "claim_ceiling": "No reachable path in the supplied finite graph; not evidence of real-world non-reachability."
            }));
        }

        let mut output_paths = Vec::new();
        let mut bypass_count = 0usize;
        for (index, path) in paths.iter().enumerate() {
            let mediator = matching_mediator(fixture, &path.nodes, &scope, &node_kinds);
            if mediator.is_none() {
                bypass_count += 1;
            }
            output_paths.push(json!({
                "route_ref": format!("ROUTE-{:04}", index + 1),
                "nodes": path.nodes,
                "edges": path.edges,
                "mediated_by": mediator,
                "classification": if mediator.is_some() { "MEDIATED" } else { "UNMEDIATED_BYPASS" }
            }));
        }

        Ok(json!({
            "suite_id": SUITE_ID,
            "evaluator_version": EVALUATOR_VERSION,
            "status": if bypass_count == 0 { "ALL_PATHS_MEDIATED" } else { "BYPASS_DETECTED" },
            "graph_sha256": graph_digest,
            "inventory_receipt_valid": true,
            "path_count": output_paths.len(),
            "mediated_path_count": output_paths.len() - bypass_count,
            "unmediated_path_count": bypass_count,
            "paths": output_paths,
            "dispatch_eligible": false,
            "claim_ceiling": "Finite synthetic graph only. Does not authenticate route completeness, PEP enforcement, provider state, external effect, or production non-bypassability."
        }))
    })();
    result.unwrap_or_else(invalid_report)
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    fn fixture() -> Value {
        serde_json::from_str(include_str!("../fixtures/route-inventory.json"))
            .expect("fixture JSON")
    }

    fn refresh_receipt(value: &mut Value) {
        let graph = &value["graph"];
        value["inventory_receipt"]["graph_sha256"] =
            json!(canonical_graph_digest(graph).expect("digest"));
    }

    fn path_count(report: &Value) -> usize {
        report["path_count"].as_u64().expect("path count") as usize
    }

    #[test]
    fn detects_direct_and_alternate_connector_bypasses() {
        let value = fixture();
        let report = evaluate(&value);
        assert_eq!(report["status"], "BYPASS_DETECTED");
        assert_eq!(path_count(&report), 3);
        assert_eq!(report["mediated_path_count"], 1);
        assert_eq!(report["unmediated_path_count"], 2);
        assert_eq!(report["dispatch_eligible"], false);
    }

    #[test]
    fn all_paths_mediated_positive_control() {
        let mut value = fixture();
        value["graph"]["edges"]
            .as_array_mut()
            .expect("edges")
            .retain(|edge| edge["edge_id"] != "E-DIRECT" && edge["edge_id"] != "E-WEBHOOK");
        value["graph"]["edges"]
            .as_array_mut()
            .expect("edges")
            .push(json!({
                "edge_id": "E-WEBHOOK-PEP",
                "from": "workflow-connector",
                "to": "policy-pep",
                "enabled": true,
                "effect_capable": true
            }));
        value["graph"]["edges"]
            .as_array_mut()
            .expect("edges")
            .push(json!({
                "edge_id": "E-PEP-WORKER",
                "from": "policy-pep",
                "to": "background-worker",
                "enabled": true,
                "effect_capable": true
            }));
        value["graph"]["edges"]
            .as_array_mut()
            .expect("edges")
            .push(json!({
                "edge_id": "E-WORKER-EFFECT",
                "from": "background-worker",
                "to": "production-environment",
                "enabled": true,
                "effect_capable": true
            }));
        refresh_receipt(&mut value);
        let report = evaluate(&value);
        assert_eq!(report["status"], "ALL_PATHS_MEDIATED");
        assert_eq!(report["unmediated_path_count"], 0);
    }

    #[test]
    fn stale_pep_receipt_does_not_mediate_a_path() {
        let mut value = fixture();
        value["mediators"][0]["valid_through"] = json!("2026-03-09T23:59:59Z");
        let report = evaluate(&value);
        assert_eq!(report["status"], "BYPASS_DETECTED");
        assert_eq!(report["mediated_path_count"], 0);
    }

    #[test]
    fn wrong_purpose_or_operation_binding_does_not_mediate() {
        let mut value = fixture();
        value["mediators"][0]["purpose"] = json!("read_only_audit");
        let report = evaluate(&value);
        assert_eq!(report["status"], "BYPASS_DETECTED");
        assert_eq!(report["mediated_path_count"], 0);

        let mut value = fixture();
        value["mediators"][0]["operation_digest"] = json!("other-operation");
        let report = evaluate(&value);
        assert_eq!(report["status"], "BYPASS_DETECTED");
        assert_eq!(report["mediated_path_count"], 0);
    }

    #[test]
    fn incomplete_inventory_or_stale_observation_is_indeterminate() {
        let mut value = fixture();
        value["inventory_receipt"]["completeness_state"] = json!("unknown");
        assert_eq!(evaluate(&value)["status"], "INDETERMINATE");

        let mut value = fixture();
        value["inventory_receipt"]["observed_at"] = json!("2026-03-11T12:00:00Z");
        assert_eq!(evaluate(&value)["status"], "INDETERMINATE");
    }

    #[test]
    fn inventory_digest_mismatch_is_indeterminate() {
        let mut value = fixture();
        value["inventory_receipt"]["graph_sha256"] = json!("0".repeat(64));
        assert_eq!(evaluate(&value)["status"], "INDETERMINATE");
    }

    #[test]
    fn duplicate_edges_and_dangling_endpoints_are_invalid() {
        let mut value = fixture();
        let duplicate = value["graph"]["edges"][0].clone();
        value["graph"]["edges"]
            .as_array_mut()
            .expect("edges")
            .push(duplicate);
        assert_eq!(evaluate(&value)["status"], "INVALID_INPUT");

        let mut value = fixture();
        value["graph"]["edges"][0]["to"] = json!("unlisted-node");
        refresh_receipt(&mut value);
        assert_eq!(evaluate(&value)["status"], "INVALID_INPUT");
    }

    #[test]
    fn cycles_do_not_create_infinite_routes() {
        let mut value = fixture();
        value["graph"]["edges"]
            .as_array_mut()
            .expect("edges")
            .push(json!({
                "edge_id": "E-CYCLE",
                "from": "background-worker",
                "to": "workflow-connector",
                "enabled": true,
                "effect_capable": true
            }));
        refresh_receipt(&mut value);
        let report = evaluate(&value);
        assert_eq!(report["status"], "BYPASS_DETECTED");
        assert!(path_count(&report) < MAX_PATHS);
    }

    #[test]
    fn disabled_or_non_effect_edges_do_not_create_protected_paths() {
        let mut value = fixture();
        value["graph"]["edges"]
            .as_array_mut()
            .expect("edges")
            .retain(|edge| edge["edge_id"] != "E-DIRECT" && edge["edge_id"] != "E-WEBHOOK");
        value["graph"]["edges"]
            .as_array_mut()
            .expect("edges")
            .push(json!({
                "edge_id": "E-DISABLED",
                "from": "agent-entry",
                "to": "production-environment",
                "enabled": false,
                "effect_capable": true
            }));
        refresh_receipt(&mut value);
        assert_eq!(evaluate(&value)["status"], "NO_REACHABLE_EFFECT_PATH");
    }

    #[test]
    fn exact_scope_resource_binding_is_required() {
        let mut value = fixture();
        value["scope"]["resource_ref"] = json!("staging-environment");
        refresh_receipt(&mut value);
        assert_eq!(evaluate(&value)["status"], "BYPASS_DETECTED");
        assert_eq!(evaluate(&value)["mediated_path_count"], 0);
    }

    #[test]
    fn path_enumeration_limit_fails_closed() {
        let mut value = fixture();
        let edges = value["graph"]["edges"].as_array_mut().expect("edges");
        edges.clear();
        let mut previous = "agent-entry".to_owned();
        for layer in 0..12 {
            let left = format!("L{layer}");
            let right = format!("R{layer}");
            for (suffix, next) in [("A", &left), ("B", &right)] {
                edges.push(json!({
                    "edge_id": format!("E-{layer}-{suffix}"),
                    "from": previous,
                    "to": next,
                    "enabled": true,
                    "effect_capable": true
                }));
            }
            previous = left;
        }
        edges.push(json!({
            "edge_id": "E-END",
            "from": previous,
            "to": "production-environment",
            "enabled": true,
            "effect_capable": true
        }));
        refresh_receipt(&mut value);
        let report = evaluate(&value);
        assert_eq!(report["status"], "INDETERMINATE");
    }
}
