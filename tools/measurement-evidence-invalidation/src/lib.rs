//! Read-only deterministic measurement-evidence dependency and invalidation oracle.
//!
//! It verifies exact dependency identity, canonical graph order and affected-descendant
//! invalidation. It does not mutate source records, authenticate evidence or authorize effects.

#![forbid(unsafe_code)]

use serde::de::{MapAccess, SeqAccess, Visitor};
use serde::{Deserialize, Deserializer};
use serde_json::{Map, Value, json};
use std::collections::{BTreeMap, BTreeSet};
use std::fmt;

pub const PROFILE_JSON: &str =
    include_str!("../../../docs/metrology/MEASUREMENT_EVIDENCE_INVALIDATION_PROFILE_V0_1.json");
pub const PROFILE_SCHEMA_JSON: &str =
    include_str!("../../../docs/metrology/MEASUREMENT_EVIDENCE_INVALIDATION_PROFILE_V0_1.schema.json");
pub const CORPUS_JSON: &str =
    include_str!("../../../docs/metrology/MEASUREMENT_EVIDENCE_INVALIDATION_CORPUS_V0_1.json");
pub const CORPUS_SCHEMA_JSON: &str =
    include_str!("../../../docs/metrology/MEASUREMENT_EVIDENCE_INVALIDATION_CORPUS_V0_1.schema.json");

const PROFILE_ID: &str = "BUILD-MEASUREMENT-INVALIDATION-PROFILE-V0.1";
const CORPUS_ID: &str = "BUILD-MEASUREMENT-INVALIDATION-CORPUS-V0.1";
const SCHEMA_VERSION: &str = "mycelix.measurement-evidence-invalidation-corpus/0.1";

const NODE_FIELDS: &[&str] = &[
    "node_id", "node_kind", "subject_id", "revision", "content_digest", "state",
];
const EDGE_FIELDS: &[&str] = &[
    "dependency_id", "dependent_id", "dependency_revision", "dependency_digest",
];
const EVENT_FIELDS: &[&str] = &[
    "event_id", "target_node_id", "target_revision", "target_digest", "event_kind", "reason_code",
];
const GRAPH_FIELDS: &[&str] = &["nodes", "edges", "events"];
const EXPECTED_FIELDS: &[&str] = &[
    "disposition", "error_code", "topological_order", "node_states", "invalidated_nodes",
];
const CASE_FIELDS: &[&str] = &["case_id", "description", "graph", "expected"];
const ROOT_FIELDS: &[&str] = &[
    "schema_version", "corpus_id", "classification", "claim_ceiling", "cases",
];
const PROFILE_FIELDS: &[&str] = &[
    "schema_version", "profile_id", "classification", "claim_ceiling", "parent_issue",
    "parent_passport_pr", "node_states", "event_states", "dependency_direction",
    "exact_parent_binding_required", "exact_event_target_binding_required",
    "identical_duplicate_node_is_idempotent", "conflicting_duplicate_node_is_rejected",
    "duplicate_edges_are_rejected", "duplicate_or_multiple_target_events_are_rejected",
    "canonical_topological_order", "descendant_only_invalidation",
    "unrelated_nodes_remain_current", "no_mutation_or_authority",
];
const NODE_KINDS: &[&str] = &[
    "instrument_configuration", "calibration", "installation", "raw_sample",
    "derived_transform", "uncertainty_budget", "independent_assay", "purpose_profile",
    "measurement_claim", "configuration_revision",
];
const NODE_STATES: &[&str] = &[
    "current", "unknown", "revoked", "expired", "superseded", "corrected",
];
const EVENT_KINDS: &[&str] = &[
    "correction", "revocation", "expiry", "supersession", "rights_revocation",
    "firmware_change", "installation_change", "method_change", "uncertainty_change",
    "assay_revocation",
];

const EXPECTED_CASES: &[(&str, &str, &str, &str)] = &[
    ("MEI-001","CURRENT","","e8b9f7f934ab9e10"),
    ("MEI-002","PARTIALLY_INVALIDATED","","492eb6dd53382fb5"),
    ("MEI-003","PARTIALLY_INVALIDATED","","e6d1e4a2658a4f0b"),
    ("MEI-004","PARTIALLY_INVALIDATED","","a6c509f533e634bf"),
    ("MEI-005","PARTIALLY_INVALIDATED","","54837d354727435b"),
    ("MEI-006","PARTIALLY_INVALIDATED","","286cc49322941b12"),
    ("MEI-007","PARTIALLY_INVALIDATED","","e3551e8160506e8d"),
    ("MEI-008","PARTIALLY_INVALIDATED","","81274ef4fa554f48"),
    ("MEI-009","PARTIALLY_INVALIDATED","","025ef3579f82251f"),
    ("MEI-010","PARTIALLY_INVALIDATED","","80fa7d174477b5da"),
    ("MEI-011","CURRENT","","e8b9f7f934ab9e10"),
    ("MEI-012","INVALID_GRAPH","PARENT_BINDING_MISMATCH","cb25c3b1692429bb"),
    ("MEI-013","INVALID_GRAPH","PARENT_BINDING_MISMATCH","cb25c3b1692429bb"),
    ("MEI-014","INVALID_GRAPH","DANGLING_EDGE","d5709f814a6ca75a"),
    ("MEI-015","INVALID_GRAPH","SELF_EDGE","efcdad995c6ddbf0"),
    ("MEI-016","INVALID_GRAPH","DUPLICATE_EDGE","2897b12c558be0a9"),
    ("MEI-017","INVALID_GRAPH","NODE_ID_CONFLICT","52a54d6b48481c39"),
    ("MEI-018","CURRENT","","e8b9f7f934ab9e10"),
    ("MEI-019","INVALID_GRAPH","CYCLE","16c2bb2def2b843c"),
    ("MEI-020","INVALID_GRAPH","CYCLE","16c2bb2def2b843c"),
    ("MEI-021","INVALID_GRAPH","DUPLICATE_EVENT_ID","1b4f33dac969e362"),
    ("MEI-022","INVALID_GRAPH","EVENT_TARGET_MISMATCH","a5ebf1e0ab11fae1"),
    ("MEI-023","PARTIALLY_INVALIDATED","","ef5ba3c10faf79f5"),
    ("MEI-024","PARTIALLY_INVALIDATED","","0097c88cdb5e7c83"),
    ("MEI-025","PARTIALLY_INVALIDATED","","7b95ce224718f0a8"),
    ("MEI-026","PARTIALLY_INVALIDATED","","d942398671570b9c"),
    ("MEI-027","PARTIALLY_INVALIDATED","","eb8be985858c1466"),
    ("MEI-028","CURRENT","","e8b9f7f934ab9e10"),
    ("MEI-029","INVALID_GRAPH","DUPLICATE_EVENT_TARGET","2b0ddc76d233c45e"),
    ("MEI-030","INVALID_GRAPH","DANGLING_EVENT_TARGET","cfc22f541dbf095d"),
    ("MEI-031","NO_CURRENT_EVIDENCE","","18eca90637028044"),
    ("MEI-032","INVALID_GRAPH","EVENT_TARGET_MISMATCH","a5ebf1e0ab11fae1"),
];

#[derive(Debug)]
struct UniqueJson(Value);

impl<'de> Deserialize<'de> for UniqueJson {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        deserializer.deserialize_any(UniqueJsonVisitor)
    }
}

struct UniqueJsonVisitor;

impl<'de> Visitor<'de> for UniqueJsonVisitor {
    type Value = UniqueJson;

    fn expecting(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter.write_str("a JSON value with no duplicate object member names")
    }

    fn visit_bool<E>(self, value: bool) -> Result<Self::Value, E>
    where
        E: serde::de::Error,
    {
        Ok(UniqueJson(Value::Bool(value)))
    }

    fn visit_i64<E>(self, value: i64) -> Result<Self::Value, E>
    where
        E: serde::de::Error,
    {
        Ok(UniqueJson(Value::Number(value.into())))
    }

    fn visit_u64<E>(self, value: u64) -> Result<Self::Value, E>
    where
        E: serde::de::Error,
    {
        Ok(UniqueJson(Value::Number(value.into())))
    }

    fn visit_f64<E>(self, value: f64) -> Result<Self::Value, E>
    where
        E: serde::de::Error,
    {
        let number = serde_json::Number::from_f64(value)
            .ok_or_else(|| E::custom("non-finite JSON number"))?;
        Ok(UniqueJson(Value::Number(number)))
    }

    fn visit_str<E>(self, value: &str) -> Result<Self::Value, E>
    where
        E: serde::de::Error,
    {
        Ok(UniqueJson(Value::String(value.to_owned())))
    }

    fn visit_string<E>(self, value: String) -> Result<Self::Value, E>
    where
        E: serde::de::Error,
    {
        Ok(UniqueJson(Value::String(value)))
    }

    fn visit_unit<E>(self) -> Result<Self::Value, E>
    where
        E: serde::de::Error,
    {
        Ok(UniqueJson(Value::Null))
    }

    fn visit_none<E>(self) -> Result<Self::Value, E>
    where
        E: serde::de::Error,
    {
        Ok(UniqueJson(Value::Null))
    }

    fn visit_some<D>(self, deserializer: D) -> Result<Self::Value, D::Error>
    where
        D: Deserializer<'de>,
    {
        UniqueJson::deserialize(deserializer)
    }

    fn visit_seq<A>(self, mut sequence: A) -> Result<Self::Value, A::Error>
    where
        A: SeqAccess<'de>,
    {
        let mut values = Vec::new();
        while let Some(value) = sequence.next_element::<UniqueJson>()? {
            values.push(value.0);
        }
        Ok(UniqueJson(Value::Array(values)))
    }

    fn visit_map<A>(self, mut object: A) -> Result<Self::Value, A::Error>
    where
        A: MapAccess<'de>,
    {
        let mut values = Map::new();
        let mut seen = BTreeSet::new();
        while let Some(key) = object.next_key::<String>()? {
            if !seen.insert(key.clone()) {
                return Err(serde::de::Error::custom(format!(
                    "duplicate JSON member {key:?}"
                )));
            }
            let value = object.next_value::<UniqueJson>()?;
            values.insert(key, value.0);
        }
        Ok(UniqueJson(Value::Object(values)))
    }
}

fn parse_unique_json(source: &str) -> Result<Value, String> {
    let mut deserializer = serde_json::Deserializer::from_str(source);
    let value = UniqueJson::deserialize(&mut deserializer)
        .map_err(|error| format!("STRICT_JSON_REJECTED:{error}"))?;
    deserializer
        .end()
        .map_err(|error| format!("JSON_TRAILING_DATA:{error}"))?;
    Ok(value.0)
}

fn text<'a>(value: &'a Value, field: &str) -> &'a str {
    value.get(field).and_then(Value::as_str).unwrap_or("")
}

fn exact_keys(value: &Value, expected: &[&str]) -> bool {
    let Some(object) = value.as_object() else {
        return false;
    };
    object.len() == expected.len() && expected.iter().all(|key| object.contains_key(*key))
}

fn array_equals(value: &Value, expected: &[&str]) -> bool {
    value.as_array().is_some_and(|items| {
        items.len() == expected.len()
            && items
                .iter()
                .zip(expected)
                .all(|(item, expected)| item.as_str() == Some(*expected))
    })
}

fn fnv1a64_hex(value: &Value) -> Result<String, String> {
    let serialized = serde_json::to_string(value)
        .map_err(|error| format!("EXPECTED_SERIALIZATION_FAILED:{error}"))?;
    let mut hash: u64 = 14_695_981_039_346_656_037;
    for byte in serialized.as_bytes() {
        hash ^= u64::from(*byte);
        hash = hash.wrapping_mul(1_099_511_628_211);
    }
    Ok(format!("{hash:016x}"))
}

fn invalid_graph(error_code: &str) -> Value {
    json!({
        "disposition": "INVALID_GRAPH",
        "error_code": error_code,
        "topological_order": [],
        "node_states": [],
        "invalidated_nodes": []
    })
}

fn event_resulting_state(event_kind: &str) -> Option<&'static str> {
    match event_kind {
        "correction" | "uncertainty_change" => Some("corrected"),
        "revocation" | "rights_revocation" | "assay_revocation" => Some("revoked"),
        "expiry" => Some("expired"),
        "supersession" | "firmware_change" | "installation_change" | "method_change" => {
            Some("superseded")
        }
        _ => None,
    }
}

fn valid_node(node: &Value) -> bool {
    exact_keys(node, NODE_FIELDS)
        && !text(node, "node_id").is_empty()
        && NODE_KINDS.contains(&text(node, "node_kind"))
        && !text(node, "subject_id").is_empty()
        && !text(node, "revision").is_empty()
        && !text(node, "content_digest").is_empty()
        && NODE_STATES.contains(&text(node, "state"))
}

fn valid_edge(edge: &Value) -> bool {
    exact_keys(edge, EDGE_FIELDS)
        && !text(edge, "dependency_id").is_empty()
        && !text(edge, "dependent_id").is_empty()
        && !text(edge, "dependency_revision").is_empty()
        && !text(edge, "dependency_digest").is_empty()
}

fn valid_event(event: &Value) -> bool {
    exact_keys(event, EVENT_FIELDS)
        && !text(event, "event_id").is_empty()
        && !text(event, "target_node_id").is_empty()
        && !text(event, "target_revision").is_empty()
        && !text(event, "target_digest").is_empty()
        && EVENT_KINDS.contains(&text(event, "event_kind"))
        && !text(event, "reason_code").is_empty()
}

/// Evaluate a graph without modifying any source node, edge, event, or evidence artifact.
pub fn evaluate_graph(graph: &Value) -> Value {
    if !exact_keys(graph, GRAPH_FIELDS) {
        return invalid_graph("GRAPH_SHAPE_INVALID");
    }
    let Some(raw_nodes) = graph.get("nodes").and_then(Value::as_array) else {
        return invalid_graph("NODES_INVALID");
    };
    let Some(raw_edges) = graph.get("edges").and_then(Value::as_array) else {
        return invalid_graph("EDGES_INVALID");
    };
    let Some(raw_events) = graph.get("events").and_then(Value::as_array) else {
        return invalid_graph("EVENTS_INVALID");
    };

    let mut nodes: BTreeMap<String, Value> = BTreeMap::new();
    for node in raw_nodes {
        if !valid_node(node) {
            return invalid_graph("NODE_INVALID");
        }
        let node_id = text(node, "node_id").to_owned();
        if let Some(existing) = nodes.get(&node_id) {
            if existing != node {
                return invalid_graph("NODE_ID_CONFLICT");
            }
        } else {
            nodes.insert(node_id, node.clone());
        }
    }
    if nodes.is_empty() {
        return invalid_graph("NODES_EMPTY");
    }

    let mut adjacency: BTreeMap<String, BTreeSet<String>> = nodes
        .keys()
        .map(|node_id| (node_id.clone(), BTreeSet::new()))
        .collect();
    let mut parents: BTreeMap<String, BTreeSet<String>> = nodes
        .keys()
        .map(|node_id| (node_id.clone(), BTreeSet::new()))
        .collect();
    let mut indegree: BTreeMap<String, usize> =
        nodes.keys().map(|node_id| (node_id.clone(), 0)).collect();
    let mut edge_pairs = BTreeSet::new();

    for edge in raw_edges {
        if !valid_edge(edge) {
            return invalid_graph("EDGE_INVALID");
        }
        let dependency_id = text(edge, "dependency_id");
        let dependent_id = text(edge, "dependent_id");
        if dependency_id == dependent_id {
            return invalid_graph("SELF_EDGE");
        }
        if !nodes.contains_key(dependency_id) || !nodes.contains_key(dependent_id) {
            return invalid_graph("DANGLING_EDGE");
        }
        if !edge_pairs.insert((dependency_id.to_owned(), dependent_id.to_owned())) {
            return invalid_graph("DUPLICATE_EDGE");
        }
        let parent = &nodes[dependency_id];
        if text(parent, "revision") != text(edge, "dependency_revision")
            || text(parent, "content_digest") != text(edge, "dependency_digest")
        {
            return invalid_graph("PARENT_BINDING_MISMATCH");
        }
        adjacency
            .get_mut(dependency_id)
            .expect("all node IDs were inserted")
            .insert(dependent_id.to_owned());
        parents
            .get_mut(dependent_id)
            .expect("all node IDs were inserted")
            .insert(dependency_id.to_owned());
        *indegree
            .get_mut(dependent_id)
            .expect("all node IDs were inserted") += 1;
    }

    let mut event_ids = BTreeSet::new();
    let mut event_targets = BTreeSet::new();
    let mut event_states: BTreeMap<String, &'static str> = BTreeMap::new();
    for event in raw_events {
        if !valid_event(event) {
            return invalid_graph("EVENT_INVALID");
        }
        let event_id = text(event, "event_id");
        if !event_ids.insert(event_id.to_owned()) {
            return invalid_graph("DUPLICATE_EVENT_ID");
        }
        let target_id = text(event, "target_node_id");
        if !event_targets.insert(target_id.to_owned()) {
            return invalid_graph("DUPLICATE_EVENT_TARGET");
        }
        let Some(target) = nodes.get(target_id) else {
            return invalid_graph("DANGLING_EVENT_TARGET");
        };
        if text(target, "revision") != text(event, "target_revision")
            || text(target, "content_digest") != text(event, "target_digest")
        {
            return invalid_graph("EVENT_TARGET_MISMATCH");
        }
        let Some(next_state) = event_resulting_state(text(event, "event_kind")) else {
            return invalid_graph("EVENT_KIND_INVALID");
        };
        event_states.insert(target_id.to_owned(), next_state);
    }

    // Kahn's algorithm with a BTreeSet gives the lexicographically smallest ready node.
    let mut ready: BTreeSet<String> = indegree
        .iter()
        .filter_map(|(node_id, degree)| (*degree == 0).then_some(node_id.clone()))
        .collect();
    let mut topological_order = Vec::with_capacity(nodes.len());
    while let Some(node_id) = ready.iter().next().cloned() {
        ready.remove(&node_id);
        topological_order.push(node_id.clone());
        if let Some(dependants) = adjacency.get(&node_id) {
            for dependent_id in dependants {
                let degree = indegree
                    .get_mut(dependent_id)
                    .expect("all edge endpoints were validated");
                *degree -= 1;
                if *degree == 0 {
                    ready.insert(dependent_id.clone());
                }
            }
        }
    }
    if topological_order.len() != nodes.len() {
        return invalid_graph("CYCLE");
    }

    let mut states: BTreeMap<String, String> = BTreeMap::new();
    for (node_id, node) in &nodes {
        let initial_state = text(node, "state");
        let resulting_state = if initial_state == "current" {
            event_states.get(node_id).copied().unwrap_or("current")
        } else {
            initial_state
        };
        states.insert(node_id.clone(), resulting_state.to_owned());
    }

    for node_id in &topological_order {
        if states.get(node_id).map(String::as_str) != Some("current") {
            continue;
        }
        let has_non_current_parent = parents
            .get(node_id)
            .expect("every node has an initialized parent set")
            .iter()
            .any(|parent_id| states.get(parent_id).map(String::as_str) != Some("current"));
        if has_non_current_parent {
            states.insert(node_id.clone(), "invalidated".to_owned());
        }
    }

    let node_states: Vec<Value> = topological_order
        .iter()
        .map(|node_id| {
            json!({
                "node_id": node_id,
                "state": states.get(node_id).expect("all nodes have a state")
            })
        })
        .collect();
    let invalidated_nodes: Vec<String> = states
        .iter()
        .filter_map(|(node_id, state)| (state == "invalidated").then_some(node_id.clone()))
        .collect();

    let any_current = states.values().any(|state| state == "current");
    let any_invalidated = states.values().any(|state| state == "invalidated");
    let disposition = if any_invalidated {
        if any_current {
            "PARTIALLY_INVALIDATED"
        } else {
            "NO_CURRENT_EVIDENCE"
        }
    } else if any_current {
        "CURRENT"
    } else {
        "NO_CURRENT_EVIDENCE"
    };

    json!({
        "disposition": disposition,
        "error_code": Value::Null,
        "topological_order": topological_order,
        "node_states": node_states,
        "invalidated_nodes": invalidated_nodes
    })
}

fn validate_profile(profile: &Value) -> Result<(), String> {
    if !exact_keys(profile, PROFILE_FIELDS)
        || text(profile, "schema_version") != "mycelix.measurement-evidence-invalidation-profile/0.1"
        || text(profile, "profile_id") != PROFILE_ID
        || text(profile, "classification") != "synthetic_read_only_dependency_semantics"
        || text(profile, "claim_ceiling") != "deterministic_dependency_and_invalidation_semantics_only"
        || text(profile, "parent_issue") != "https://github.com/Luminous-Dynamics/mycelix/issues/6042"
        || text(profile, "parent_passport_pr") != "https://github.com/Luminous-Dynamics/mycelix/pull/6040"
        || !array_equals(&profile["node_states"], NODE_STATES)
        || !array_equals(&profile["event_states"], EVENT_KINDS)
        || text(profile, "dependency_direction") != "dependency_to_dependent"
        || profile["exact_parent_binding_required"] != Value::Bool(true)
        || profile["exact_event_target_binding_required"] != Value::Bool(true)
        || profile["identical_duplicate_node_is_idempotent"] != Value::Bool(true)
        || profile["conflicting_duplicate_node_is_rejected"] != Value::Bool(true)
        || profile["duplicate_edges_are_rejected"] != Value::Bool(true)
        || profile["duplicate_or_multiple_target_events_are_rejected"] != Value::Bool(true)
        || text(profile, "canonical_topological_order") != "lexicographically_smallest_ready_node_first"
        || profile["descendant_only_invalidation"] != Value::Bool(true)
        || profile["unrelated_nodes_remain_current"] != Value::Bool(true)
        || profile["no_mutation_or_authority"] != Value::Bool(true)
    {
        return Err("PROFILE_CONTRACT_MISMATCH".to_owned());
    }
    Ok(())
}

fn schema_type(schema: &Value, expected: &str) -> bool {
    text(schema, "type") == expected && schema["additionalProperties"] == Value::Bool(false)
}

fn validate_profile_schema(schema: &Value) -> Result<(), String> {
    if text(schema, "title") != "Measurement Evidence Invalidation Profile v0.1"
        || !schema_type(schema, "object")
        || !array_equals(&schema["required"], PROFILE_FIELDS)
    {
        return Err("PROFILE_SCHEMA_ROOT_INVALID".to_owned());
    }
    let properties = &schema["properties"];
    if !exact_keys(properties, PROFILE_FIELDS)
        || properties["node_states"]["const"] != json!(NODE_STATES)
        || properties["event_states"]["const"] != json!(EVENT_KINDS)
        || properties["no_mutation_or_authority"]["const"] != Value::Bool(true)
        || properties["descendant_only_invalidation"]["const"] != Value::Bool(true)
        || properties["unrelated_nodes_remain_current"]["const"] != Value::Bool(true)
    {
        return Err("PROFILE_SCHEMA_CONTRACT_DRIFT".to_owned());
    }
    Ok(())
}

fn validate_corpus_schema(schema: &Value) -> Result<(), String> {
    if text(schema, "title") != "Measurement Evidence Invalidation Corpus v0.1"
        || !schema_type(schema, "object")
        || !array_equals(&schema["required"], ROOT_FIELDS)
    {
        return Err("CORPUS_SCHEMA_ROOT_INVALID".to_owned());
    }
    let root_properties = &schema["properties"];
    if !exact_keys(root_properties, ROOT_FIELDS)
        || root_properties["schema_version"]["const"] != SCHEMA_VERSION
        || root_properties["corpus_id"]["const"] != CORPUS_ID
        || root_properties["classification"]["const"] != "synthetic_dependency_and_invalidation_cases"
        || root_properties["claim_ceiling"]["const"] != "synthetic_graph_semantics_only"
    {
        return Err("CORPUS_SCHEMA_ROOT_CONTRACT_DRIFT".to_owned());
    }
    let cases = &root_properties["cases"];
    if text(cases, "type") != "array"
        || cases["minItems"] != EXPECTED_CASES.len()
        || cases["maxItems"] != EXPECTED_CASES.len()
    {
        return Err("CORPUS_SCHEMA_CASE_COUNT_INVALID".to_owned());
    }
    let item = &cases["items"];
    if !schema_type(item, "object") || !array_equals(&item["required"], CASE_FIELDS)
        || !exact_keys(&item["properties"], CASE_FIELDS)
    {
        return Err("CORPUS_SCHEMA_CASE_SHAPE_INVALID".to_owned());
    }
    let graph = &item["properties"]["graph"];
    if !schema_type(graph, "object") || !array_equals(&graph["required"], GRAPH_FIELDS)
        || !exact_keys(&graph["properties"], GRAPH_FIELDS)
    {
        return Err("CORPUS_SCHEMA_GRAPH_SHAPE_INVALID".to_owned());
    }
    let node_schema = &graph["properties"]["nodes"]["items"];
    if !schema_type(node_schema, "object") || !array_equals(&node_schema["required"], NODE_FIELDS)
        || !exact_keys(&node_schema["properties"], NODE_FIELDS)
        || node_schema["properties"]["node_kind"]["enum"] != json!(NODE_KINDS)
        || node_schema["properties"]["state"]["enum"] != json!(NODE_STATES)
    {
        return Err("CORPUS_SCHEMA_NODE_SHAPE_INVALID".to_owned());
    }
    let edge_schema = &graph["properties"]["edges"]["items"];
    if !schema_type(edge_schema, "object") || !array_equals(&edge_schema["required"], EDGE_FIELDS)
        || !exact_keys(&edge_schema["properties"], EDGE_FIELDS)
    {
        return Err("CORPUS_SCHEMA_EDGE_SHAPE_INVALID".to_owned());
    }
    let event_schema = &graph["properties"]["events"]["items"];
    if !schema_type(event_schema, "object")
        || !array_equals(&event_schema["required"], EVENT_FIELDS)
        || !exact_keys(&event_schema["properties"], EVENT_FIELDS)
        || event_schema["properties"]["event_kind"]["enum"] != json!(EVENT_KINDS)
    {
        return Err("CORPUS_SCHEMA_EVENT_SHAPE_INVALID".to_owned());
    }
    let expected_schema = &item["properties"]["expected"];
    if !schema_type(expected_schema, "object")
        || !array_equals(&expected_schema["required"], EXPECTED_FIELDS)
        || !exact_keys(&expected_schema["properties"], EXPECTED_FIELDS)
        || expected_schema["properties"]["disposition"]["enum"]
            != json!(["CURRENT","PARTIALLY_INVALIDATED","NO_CURRENT_EVIDENCE","INVALID_GRAPH"])
    {
        return Err("CORPUS_SCHEMA_EXPECTED_SHAPE_INVALID".to_owned());
    }
    Ok(())
}

pub fn validate_corpus(corpus: &Value) -> Result<usize, String> {
    if !exact_keys(corpus, ROOT_FIELDS)
        || text(corpus, "schema_version") != SCHEMA_VERSION
        || text(corpus, "corpus_id") != CORPUS_ID
        || text(corpus, "classification") != "synthetic_dependency_and_invalidation_cases"
        || text(corpus, "claim_ceiling") != "synthetic_graph_semantics_only"
    {
        return Err("CORPUS_IDENTITY_INVALID".to_owned());
    }
    let cases = corpus["cases"]
        .as_array()
        .ok_or_else(|| "CASES_MISSING".to_owned())?;
    if cases.len() != EXPECTED_CASES.len() {
        return Err("CASE_COUNT_INVALID".to_owned());
    }
    let mut seen_case_ids = BTreeSet::new();
    for (index, case) in cases.iter().enumerate() {
        let (expected_id, expected_disposition, expected_error, expected_hash) = EXPECTED_CASES[index];
        let case_id = text(case, "case_id");
        if !exact_keys(case, CASE_FIELDS)
            || case_id != expected_id
            || text(case, "description").len() < 20
            || !seen_case_ids.insert(case_id.to_owned())
            || !exact_keys(&case["graph"], GRAPH_FIELDS)
            || !exact_keys(&case["expected"], EXPECTED_FIELDS)
        {
            return Err(format!("CASE_SHAPE_OR_ID_INVALID:{expected_id}"));
        }
        let expected = &case["expected"];
        if text(expected, "disposition") != expected_disposition
            || text(expected, "error_code") != expected_error
            || fnv1a64_hex(expected)? != expected_hash
        {
            return Err(format!("CASE_EXPECTED_RESULT_DRIFT:{expected_id}"));
        }
        let actual = evaluate_graph(&case["graph"]);
        if actual != *expected {
            return Err(format!(
                "CASE_RESULT_MISMATCH:{expected_id}:expected={expected}:actual={actual}"
            ));
        }
    }
    Ok(cases.len())
}

pub fn validate_tracked() -> Result<usize, String> {
    let profile_schema = parse_unique_json(PROFILE_SCHEMA_JSON)?;
    validate_profile_schema(&profile_schema)?;
    let profile = parse_unique_json(PROFILE_JSON)?;
    validate_profile(&profile)?;
    let corpus_schema = parse_unique_json(CORPUS_SCHEMA_JSON)?;
    validate_corpus_schema(&corpus_schema)?;
    let corpus = parse_unique_json(CORPUS_JSON)?;
    validate_corpus(&corpus)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn case(case_id: &str) -> Value {
        let corpus = parse_unique_json(CORPUS_JSON).expect("tracked corpus");
        corpus["cases"]
            .as_array()
            .expect("cases")
            .iter()
            .find(|item| text(item, "case_id") == case_id)
            .cloned()
            .expect("known case")
    }

    #[test]
    fn tracked_graph_corpus_matches_all_thirty_two_known_answers() {
        assert_eq!(validate_tracked().expect("valid graph corpus"), 32);
    }

    #[test]
    fn parent_revocation_invalidates_exact_descendants_only() {
        let result = evaluate_graph(&case("MEI-002")["graph"]);
        assert_eq!(result["node_states"][1]["state"], "revoked");
        assert!(result["invalidated_nodes"].as_array().unwrap().iter().any(|id| id == "sample-A"));
        assert_eq!(
            result["node_states"].as_array().unwrap().iter()
                .find(|state| state["node_id"] == "sample-B").unwrap()["state"],
            "current"
        );
    }

    #[test]
    fn purpose_profile_correction_preserves_raw_sample() {
        let result = evaluate_graph(&case("MEI-005")["graph"]);
        let states = result["node_states"].as_array().unwrap();
        assert_eq!(states.iter().find(|s| s["node_id"] == "purpose-A").unwrap()["state"], "corrected");
        assert_eq!(states.iter().find(|s| s["node_id"] == "sample-A").unwrap()["state"], "current");
        assert_eq!(states.iter().find(|s| s["node_id"] == "claim-A").unwrap()["state"], "invalidated");
    }

    #[test]
    fn assay_revocation_does_not_rewrite_raw_sample() {
        let result = evaluate_graph(&case("MEI-006")["graph"]);
        let states = result["node_states"].as_array().unwrap();
        assert_eq!(states.iter().find(|s| s["node_id"] == "assay-A").unwrap()["state"], "revoked");
        assert_eq!(states.iter().find(|s| s["node_id"] == "sample-A").unwrap()["state"], "current");
        assert_eq!(states.iter().find(|s| s["node_id"] == "claim-A").unwrap()["state"], "invalidated");
    }

    #[test]
    fn unrelated_branch_remains_current_after_rights_revocation() {
        let result = evaluate_graph(&case("MEI-026")["graph"]);
        let states = result["node_states"].as_array().unwrap();
        assert_eq!(states.iter().find(|s| s["node_id"] == "sample-B").unwrap()["state"], "revoked");
        assert_eq!(states.iter().find(|s| s["node_id"] == "sample-A").unwrap()["state"], "current");
        assert_eq!(states.iter().find(|s| s["node_id"] == "claim-A").unwrap()["state"], "current");
    }

    #[test]
    fn canonical_topological_order_survives_node_and_edge_permutation() {
        let first = evaluate_graph(&case("MEI-001")["graph"]);
        let permuted = evaluate_graph(&case("MEI-028")["graph"]);
        assert_eq!(first, permuted);
    }

    #[test]
    fn exact_duplicate_node_is_idempotent() {
        assert_eq!(evaluate_graph(&case("MEI-018")["graph"])["disposition"], "CURRENT");
    }

    #[test]
    fn conflicting_duplicate_node_is_rejected() {
        assert_eq!(evaluate_graph(&case("MEI-017")["graph"])["error_code"], "NODE_ID_CONFLICT");
    }

    #[test]
    fn parent_digest_and_revision_mismatches_are_rejected() {
        for case_id in ["MEI-012", "MEI-013"] {
            assert_eq!(evaluate_graph(&case(case_id)["graph"])["error_code"], "PARENT_BINDING_MISMATCH");
        }
    }

    #[test]
    fn dangling_self_duplicate_and_cyclic_edges_are_rejected() {
        let cases = [
            ("MEI-014", "DANGLING_EDGE"),
            ("MEI-015", "SELF_EDGE"),
            ("MEI-016", "DUPLICATE_EDGE"),
            ("MEI-019", "CYCLE"),
            ("MEI-020", "CYCLE"),
        ];
        for (case_id, error_code) in cases {
            assert_eq!(evaluate_graph(&case(case_id)["graph"])["error_code"], error_code);
        }
    }

    #[test]
    fn duplicate_event_ids_and_duplicate_targets_are_rejected() {
        assert_eq!(evaluate_graph(&case("MEI-021")["graph"])["error_code"], "DUPLICATE_EVENT_ID");
        assert_eq!(evaluate_graph(&case("MEI-029")["graph"])["error_code"], "DUPLICATE_EVENT_TARGET");
    }

    #[test]
    fn missing_event_targets_and_stale_target_revisions_are_rejected() {
        assert_eq!(evaluate_graph(&case("MEI-030")["graph"])["error_code"], "DANGLING_EVENT_TARGET");
        assert_eq!(evaluate_graph(&case("MEI-032")["graph"])["error_code"], "EVENT_TARGET_MISMATCH");
        assert_eq!(evaluate_graph(&case("MEI-022")["graph"])["error_code"], "EVENT_TARGET_MISMATCH");
    }

    #[test]
    fn all_unknown_sources_yield_no_current_evidence() {
        let result = evaluate_graph(&case("MEI-031")["graph"]);
        assert_eq!(result["disposition"], "NO_CURRENT_EVIDENCE");
        assert_eq!(result["invalidated_nodes"].as_array().unwrap().len(), 0);
    }

    #[test]
    fn source_state_without_new_event_invalidates_dependants() {
        let result = evaluate_graph(&case("MEI-023")["graph"]);
        assert_eq!(result["node_states"].as_array().unwrap()
            .iter().find(|s| s["node_id"] == "calibration-A").unwrap()["state"], "superseded");
        assert_eq!(result["node_states"].as_array().unwrap()
            .iter().find(|s| s["node_id"] == "claim-A").unwrap()["state"], "invalidated");
    }

    #[test]
    fn multiple_independent_events_are_applied_deterministically() {
        let forward = evaluate_graph(&case("MEI-027")["graph"]);
        let reverse = {
            let mut graph = case("MEI-027")["graph"].clone();
            graph["events"].as_array_mut().unwrap().reverse();
            evaluate_graph(&graph)
        };
        assert_eq!(forward, reverse);
    }

    #[test]
    fn duplicate_json_members_are_rejected_at_every_depth() {
        assert!(parse_unique_json(r#"{"a":1,"a":2}"#).is_err());
        assert!(parse_unique_json(r#"{"a":{"b":1,"b":2}}"#).is_err());
    }

    #[test]
    fn graph_errors_have_no_topological_or_currentness_claim() {
        let result = evaluate_graph(&case("MEI-012")["graph"]);
        assert_eq!(result["disposition"], "INVALID_GRAPH");
        assert_eq!(result["topological_order"], json!([]));
        assert_eq!(result["node_states"], json!([]));
        assert_eq!(result["invalidated_nodes"], json!([]));
    }
}
