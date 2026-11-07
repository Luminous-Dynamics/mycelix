//! Independent, fail-closed verifier for Pulse authorization lifecycle evidence.
//!
//! The verifier deliberately does not infer causality from wall-clock timestamps or
//! local observation order. Only explicit causal-predecessor edges establish
//! happens-before. A hash proves byte identity, not issuer identity or truth.

use serde_json::{json, Map, Value};
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet, VecDeque};

pub const TOOL_VERSION: &str = env!("CARGO_PKG_VERSION");
pub const PROFILE_ID: &str = "PULSE-AUTH-LIFECYCLE-QUAL-001";

const HOST_DECISIONS: &[&str] = &["AUTHORIZED", "UNAUTHORIZED", "UNKNOWN", "NOT_APPLICABLE"];
const DOMAIN_RESULTS: &[&str] = &["EXECUTED", "REJECTED", "NOT_INVOKED", "UNKNOWN", "NOT_APPLICABLE"];
const SOURCE_CHAIN_RESULTS: &[&str] = &["ACTION_AUTHORED", "NO_ACTION", "UNKNOWN", "NOT_APPLICABLE"];
const VALIDATOR_RESULTS: &[&str] = &["ACCEPTED", "REJECTED", "NOT_OBSERVED", "UNKNOWN", "NOT_APPLICABLE"];
const PROJECTIONS: &[&str] = &["CURRENT", "STALE", "CONFLICTING", "UNKNOWN", "NOT_APPLICABLE"];
const OBSERVER_RESULTS: &[&str] = &["ACKNOWLEDGED", "NO_ACK", "UNKNOWN", "NOT_APPLICABLE"];
const GRANT_TRANSITIONS: &[&str] = &["CREATED", "REVOKED", "UNCHANGED", "UNKNOWN", "NOT_APPLICABLE"];

#[derive(Clone, Debug)]
pub struct CorpusEvent {
    pub id: String,
    pub order: u64,
    pub actor: String,
    pub action: String,
    pub request_id: String,
    pub attempt: u64,
    pub happens_before: Vec<String>,
}

#[derive(Clone, Debug)]
pub struct CorpusCheck {
    pub after: String,
    pub planes: BTreeMap<String, String>,
}

#[derive(Clone, Debug)]
pub struct Scenario {
    pub id: String,
    pub events: BTreeMap<String, CorpusEvent>,
    pub checks: Vec<CorpusCheck>,
}

#[derive(Clone, Debug)]
pub struct Corpus {
    pub profile_id: String,
    pub scenarios: BTreeMap<String, Scenario>,
}

#[derive(Clone, Debug)]
pub struct VerificationReceipt {
    pub report: Value,
}

pub fn sha256_hex(bytes: &[u8]) -> String {
    let digest = Sha256::digest(bytes);
    let mut out = String::with_capacity(64);
    for byte in digest {
        use std::fmt::Write as _;
        let _ = write!(&mut out, "{byte:02x}");
    }
    out
}

pub fn parse_and_validate_corpus(corpus_bytes: &[u8], schema_bytes: &[u8]) -> Result<(Corpus, Value), Vec<String>> {
    let mut errors = Vec::new();
    let schema: Value = match serde_json::from_slice(schema_bytes) {
        Ok(value) => value,
        Err(error) => return Err(vec![format!("schema JSON parse error: {error}")]),
    };
    let corpus_value: Value = match serde_json::from_slice(corpus_bytes) {
        Ok(value) => value,
        Err(error) => return Err(vec![format!("corpus JSON parse error: {error}")]),
    };

    validate_schema_contract(&schema, &mut errors);
    let corpus = build_corpus(&corpus_value, &mut errors);
    if errors.is_empty() {
        let corpus = corpus.expect("corpus is built when validation has no errors");
        let receipt = json!({
            "receipt_version": 1,
            "tool": "pulse-trace-verifier",
            "tool_version": TOOL_VERSION,
            "verification": "PASS",
            "profile_id": corpus.profile_id,
            "scenario_count": corpus.scenarios.len(),
            "corpus_raw_sha256": sha256_hex(corpus_bytes),
            "corpus_canonical_sha256": canonical_sha256(&corpus_value),
            "schema_raw_sha256": sha256_hex(schema_bytes),
            "claim_ceiling": "Corpus structure and internal references only; no runtime or authorization claim."
        });
        Ok((corpus, receipt))
    } else {
        Err(errors)
    }
}

pub fn verify_trace(
    corpus_bytes: &[u8],
    schema_bytes: &[u8],
    trace_bytes: &[u8],
) -> Result<VerificationReceipt, Vec<String>> {
    let (corpus, corpus_receipt) = parse_and_validate_corpus(corpus_bytes, schema_bytes)?;
    let trace_value: Value = serde_json::from_slice(trace_bytes)
        .map_err(|error| vec![format!("trace JSON parse error: {error}")])?;

    let trace = parse_trace(&trace_value)?;
    let scenario = corpus
        .scenarios
        .get(&trace.scenario_id)
        .ok_or_else(|| vec![format!("unknown scenario_id {}", trace.scenario_id)])?;

    let errors = verify_trace_against_scenario(&trace, scenario);
    if !errors.is_empty() {
        return Err(errors);
    }

    let trace_canonical = serde_json::to_vec(&trace_value)
        .map_err(|error| vec![format!("trace canonicalization failed: {error}")])?;
    let report = json!({
        "receipt_version": 1,
        "tool": "pulse-trace-verifier",
        "tool_version": TOOL_VERSION,
        "verification": "PASS",
        "profile_id": corpus.profile_id,
        "scenario_id": trace.scenario_id,
        "trace_id": trace.trace_id,
        "source_commit": trace.source_commit,
        "observer_id": trace.observer_id,
        "runtime": {
            "holochain": trace.runtime.holochain,
            "hdk": trace.runtime.hdk,
            "hdi": trace.runtime.hdi,
            "js_client": trace.runtime.js_client,
            "dna_sha256": trace.runtime.dna_sha256,
            "wasm_sha256": trace.runtime.wasm_sha256,
            "rustc": trace.runtime.rustc,
            "hc_cli": trace.runtime.hc_cli
        },
        "event_count": trace.events.len(),
        "corpus_raw_sha256": sha256_hex(corpus_bytes),
        "corpus_canonical_sha256": corpus_receipt["corpus_canonical_sha256"],
        "schema_raw_sha256": sha256_hex(schema_bytes),
        "trace_raw_sha256": sha256_hex(trace_bytes),
        "trace_canonical_sha256": sha256_hex(&trace_canonical),
        "causality": "Explicit causal_predecessors only; observed_index and timestamps do not establish happens-before.",
        "claim_ceiling": "This trace passed the Pulse v1 structural and encoded lifecycle invariants. It does not establish that the source observations are authentic or that an uninstrumented runtime behaved correctly."
    });
    Ok(VerificationReceipt { report })
}

fn canonical_sha256(value: &Value) -> String {
    match serde_json::to_vec(value) {
        Ok(bytes) => sha256_hex(&bytes),
        Err(_) => String::new(),
    }
}

fn validate_schema_contract(schema: &Value, errors: &mut Vec<String>) {
    if get_str(schema, "$schema") != Some("https://json-schema.org/draft/2020-12/schema") {
        errors.push("schema: expected JSON Schema Draft 2020-12".to_string());
    }
    if get_bool(schema, "additionalProperties") != Some(false) {
        errors.push("schema: root additionalProperties must be false".to_string());
    }
    let required = get_string_array(schema, "required").unwrap_or_default();
    for key in [
        "profile_id",
        "profile_version",
        "status",
        "dataset_type",
        "source_context",
        "outcome_planes",
        "scenario_count",
        "scenarios",
    ] {
        if !required.iter().any(|candidate| candidate == key) {
            errors.push(format!("schema: root required list omits {key}"));
        }
    }
    let expected = get_nested_str(schema, &["properties", "profile_id", "const"]);
    if expected != Some(PROFILE_ID) {
        errors.push(format!("schema: profile_id const must be {PROFILE_ID}"));
    }
}

fn build_corpus(value: &Value, errors: &mut Vec<String>) -> Option<Corpus> {
    require_object(value, "corpus", errors)?;
    reject_unknown_keys(
        value,
        &[
            "$schema",
            "profile_id",
            "profile_version",
            "status",
            "dataset_type",
            "source_context",
            "outcome_planes",
            "scenario_count",
            "scenarios",
        ],
        "corpus",
        errors,
    );

    let profile_id = required_string(value, "profile_id", "corpus", errors)?;
    if profile_id != PROFILE_ID {
        errors.push(format!("corpus.profile_id must be {PROFILE_ID}"));
    }
    for (key, expected) in [
        ("profile_version", "1.0.0"),
        ("status", "research-corpus-not-runtime-qualified"),
        ("dataset_type", "synthetic"),
    ] {
        if required_string(value, key, "corpus", errors).as_deref() != Some(expected) {
            errors.push(format!("corpus.{key} must be {expected}"));
        }
    }

    let source = value.get("source_context").unwrap_or(&Value::Null);
    require_object(source, "corpus.source_context", errors);
    let source_sha = required_string(source, "source_commit", "corpus.source_context", errors);
    if let Some(sha) = source_sha {
        if !is_hex(&sha, 40) {
            errors.push("corpus.source_context.source_commit must be 40 lowercase hex characters".to_string());
        }
    }
    for key in [
        "pulse_migration_pr",
        "holochain_071_profile_pr",
        "holochain_071_qualification_issue",
    ] {
        let _ = required_string(source, key, "corpus.source_context", errors);
    }
    reject_unknown_keys(
        source,
        &[
            "pulse_migration_pr",
            "holochain_071_profile_pr",
            "holochain_071_qualification_issue",
            "source_commit",
        ],
        "corpus.source_context",
        errors,
    );

    validate_outcome_vocabulary(value.get("outcome_planes").unwrap_or(&Value::Null), errors);

    let expected_count = value.get("scenario_count").and_then(Value::as_u64);
    if expected_count.is_none() {
        errors.push("corpus.scenario_count must be an unsigned integer".to_string());
    }
    let scenario_values = match value.get("scenarios").and_then(Value::as_array) {
        Some(items) => items,
        None => {
            errors.push("corpus.scenarios must be an array".to_string());
            return None;
        }
    };
    if expected_count != Some(scenario_values.len() as u64) {
        errors.push(format!(
            "corpus.scenario_count {:?} does not match {} scenarios",
            expected_count,
            scenario_values.len()
        ));
    }

    let mut scenarios = BTreeMap::new();
    for (index, scenario_value) in scenario_values.iter().enumerate() {
        let path = format!("corpus.scenarios[{index}]");
        if let Some(scenario) = build_scenario(scenario_value, &path, errors) {
            if scenarios.insert(scenario.id.clone(), scenario).is_some() {
                errors.push(format!("{path}: duplicate scenario id"));
            }
        }
    }

    Some(Corpus {
        profile_id,
        scenarios,
    })
}

fn build_scenario(value: &Value, path: &str, errors: &mut Vec<String>) -> Option<Scenario> {
    require_object(value, path, errors)?;
    reject_unknown_keys(value, &["id", "family", "title", "threat", "events", "checks", "notes"], path, errors);
    let id = required_string(value, "id", path, errors)?;
    for key in ["family", "title", "threat", "notes"] {
        let _ = required_string(value, key, path, errors);
    }

    let event_values = match value.get("events").and_then(Value::as_array) {
        Some(items) if !items.is_empty() => items,
        _ => {
            errors.push(format!("{path}.events must be a non-empty array"));
            return None;
        }
    };
    let mut events = BTreeMap::new();
    let mut previous_order = 0u64;
    for (index, event_value) in event_values.iter().enumerate() {
        let event_path = format!("{path}.events[{index}]");
        require_object(event_value, &event_path, errors);
        reject_unknown_keys(
            event_value,
            &["id", "order", "actor", "action", "request_id", "attempt", "grant_ref", "happens_before", "concurrent_group", "note"],
            &event_path,
            errors,
        );
        let event_id = required_string(event_value, "id", &event_path, errors);
        let order = required_u64(event_value, "order", &event_path, errors);
        let actor = required_string(event_value, "actor", &event_path, errors);
        let action = required_string(event_value, "action", &event_path, errors);
        let request_id = required_string(event_value, "request_id", &event_path, errors);
        let attempt = required_u64(event_value, "attempt", &event_path, errors);
        let _ = required_string(event_value, "note", &event_path, errors);
        let parents = optional_string_array(event_value, "happens_before", &event_path, errors);
        if let Some(current) = order {
            if current <= previous_order {
                errors.push(format!("{event_path}.order must be strictly increasing within the scenario"));
            }
            previous_order = current;
        }
        if let (Some(id), Some(order), Some(actor), Some(action), Some(request_id), Some(attempt)) =
            (event_id, order, actor, action, request_id, attempt)
        {
            let event = CorpusEvent {
                id: id.clone(),
                order,
                actor,
                action,
                request_id,
                attempt,
                happens_before: parents,
            };
            if events.insert(id.clone(), event).is_some() {
                errors.push(format!("{event_path}: duplicate event id {id}"));
            }
        }
    }

    for event in events.values() {
        for parent in &event.happens_before {
            if !events.contains_key(parent) {
                errors.push(format!("{path}: event {} references unknown happens_before event {parent}", event.id));
            }
            if parent == &event.id {
                errors.push(format!("{path}: event {} cannot happen before itself", event.id));
            }
        }
    }
    validate_corpus_dag(&events, path, errors);

    let checks_values = value.get("checks").and_then(Value::as_array);
    if checks_values.is_none() {
        errors.push(format!("{path}.checks must be an array"));
    }
    let mut checks = Vec::new();
    for (index, check) in checks_values.into_iter().flatten().enumerate() {
        let check_path = format!("{path}.checks[{index}]");
        require_object(check, &check_path, errors);
        reject_unknown_keys(check, &["after", "planes", "assertions"], &check_path, errors);
        let after = required_string(check, "after", &check_path, errors);
        let planes_value = check.get("planes").unwrap_or(&Value::Null);
        let planes = parse_planes(planes_value, &format!("{check_path}.planes"), errors);
        if let Some(items) = check.get("assertions").and_then(Value::as_array) {
            if items.is_empty() {
                errors.push(format!("{check_path}.assertions must be a non-empty array"));
            }
            for (assertion_index, item) in items.iter().enumerate() {
                if item.as_str().is_none_or(str::is_empty) {
                    errors.push(format!("{check_path}.assertions[{assertion_index}] must be a non-empty string"));
                }
            }
        }
        if let Some(after) = after {
            if !events.contains_key(&after) {
                errors.push(format!("{check_path}.after references unknown event {after}"));
            }
            checks.push(CorpusCheck { after, planes });
        }
    }

    Some(Scenario { id, events, checks })
}

fn validate_outcome_vocabulary(value: &Value, errors: &mut Vec<String>) {
    require_object(value, "corpus.outcome_planes", errors);
    reject_unknown_keys(
        value,
        &["host_decision", "domain_result", "source_chain", "validator", "projection", "observer"],
        "corpus.outcome_planes",
        errors,
    );
    for (key, allowed) in [
        ("host_decision", HOST_DECISIONS),
        ("domain_result", DOMAIN_RESULTS),
        ("source_chain", SOURCE_CHAIN_RESULTS),
        ("validator", VALIDATOR_RESULTS),
        ("projection", PROJECTIONS),
        ("observer", OBSERVER_RESULTS),
    ] {
        let values = match value.get(key).and_then(Value::as_array) {
            Some(items) if !items.is_empty() => items,
            _ => {
                errors.push(format!("corpus.outcome_planes.{key} must be a non-empty array"));
                continue;
            }
        };
        let mut seen = BTreeSet::new();
        for item in values {
            let Some(item) = item.as_str() else {
                errors.push(format!("corpus.outcome_planes.{key} contains a non-string"));
                continue;
            };
            if !allowed.contains(&item) {
                errors.push(format!("corpus.outcome_planes.{key} contains unsupported value {item}"));
            }
            if !seen.insert(item.to_string()) {
                errors.push(format!("corpus.outcome_planes.{key} contains duplicate value {item}"));
            }
        }
    }
}

fn parse_planes(value: &Value, path: &str, errors: &mut Vec<String>) -> BTreeMap<String, String> {
    let mut planes = BTreeMap::new();
    require_object(value, path, errors);
    let expected = [
        ("host_decision", HOST_DECISIONS),
        ("domain_result", DOMAIN_RESULTS),
        ("source_chain", SOURCE_CHAIN_RESULTS),
        ("validator", VALIDATOR_RESULTS),
        ("projection", PROJECTIONS),
        ("observer", OBSERVER_RESULTS),
    ];
    reject_unknown_keys(value, &expected.iter().map(|(key, _)| *key).collect::<Vec<_>>(), path, errors);
    for (key, allowed) in expected {
        match required_string(value, key, path, errors) {
            Some(outcome) if allowed.contains(&outcome.as_str()) => {
                planes.insert(key.to_string(), outcome);
            }
            Some(outcome) => errors.push(format!("{path}.{key} has unsupported outcome {outcome}")),
            None => {}
        }
    }
    planes
}

fn validate_corpus_dag(events: &BTreeMap<String, CorpusEvent>, path: &str, errors: &mut Vec<String>) {
    let mut indegree: BTreeMap<String, usize> = events.keys().map(|id| (id.clone(), 0)).collect();
    let mut children: BTreeMap<String, Vec<String>> = events.keys().map(|id| (id.clone(), Vec::new())).collect();
    for event in events.values() {
        for parent in &event.happens_before {
            if events.contains_key(parent) {
                *indegree.entry(event.id.clone()).or_default() += 1;
                children.entry(parent.clone()).or_default().push(event.id.clone());
            }
        }
    }
    let mut queue: VecDeque<String> = indegree
        .iter()
        .filter_map(|(id, count)| (*count == 0).then_some(id.clone()))
        .collect();
    let mut visited = 0usize;
    while let Some(id) = queue.pop_front() {
        visited += 1;
        for child in children.get(&id).into_iter().flatten() {
            if let Some(count) = indegree.get_mut(child) {
                *count -= 1;
                if *count == 0 {
                    queue.push_back(child.clone());
                }
            }
        }
    }
    if visited != events.len() {
        errors.push(format!("{path}: happens_before graph contains a cycle"));
    }
}

#[derive(Clone, Debug)]
struct RuntimeIdentity {
    holochain: String,
    hdk: String,
    hdi: String,
    js_client: String,
    dna_sha256: String,
    wasm_sha256: String,
    rustc: String,
    hc_cli: String,
}

#[derive(Clone, Debug)]
struct Planes {
    host_decision: String,
    domain_result: String,
    source_chain: String,
    validator: String,
    projection: String,
    observer: String,
}

impl Planes {
    fn as_map(&self) -> BTreeMap<String, String> {
        BTreeMap::from([
            ("host_decision".into(), self.host_decision.clone()),
            ("domain_result".into(), self.domain_result.clone()),
            ("source_chain".into(), self.source_chain.clone()),
            ("validator".into(), self.validator.clone()),
            ("projection".into(), self.projection.clone()),
            ("observer".into(), self.observer.clone()),
        ])
    }
}

#[derive(Clone, Debug)]
struct ObservedEvent {
    event_id: String,
    scenario_event_id: String,
    observed_index: u64,
    actor: String,
    action: String,
    logical_operation_id: String,
    attempt_id: String,
    attempt_number: u64,
    causal_predecessors: Vec<String>,
    grant_action_hash: Option<String>,
    source_action_hash: Option<String>,
    host_grant_transition: String,
    planes: Planes,
    observer_state_before_sha256: Option<String>,
    observer_state_after_sha256: Option<String>,
}

#[derive(Clone, Debug)]
struct Trace {
    trace_id: String,
    profile_id: String,
    scenario_id: String,
    source_commit: String,
    observer_id: String,
    runtime: RuntimeIdentity,
    events: Vec<ObservedEvent>,
}

fn parse_trace(value: &Value) -> Result<Trace, Vec<String>> {
    let mut errors = Vec::new();
    require_object(value, "trace", &mut errors);
    reject_unknown_keys(
        value,
        &["trace_version", "profile_id", "trace_id", "scenario_id", "source_commit", "observer_id", "runtime", "events"],
        "trace",
        &mut errors,
    );
    if value.get("trace_version").and_then(Value::as_u64) != Some(1) {
        errors.push("trace.trace_version must be 1".to_string());
    }
    let profile_id = required_string(value, "profile_id", "trace", &mut errors).unwrap_or_default();
    if profile_id != PROFILE_ID {
        errors.push(format!("trace.profile_id must be {PROFILE_ID}"));
    }
    let trace_id = required_string(value, "trace_id", "trace", &mut errors).unwrap_or_default();
    let scenario_id = required_string(value, "scenario_id", "trace", &mut errors).unwrap_or_default();
    let source_commit = required_string(value, "source_commit", "trace", &mut errors).unwrap_or_default();
    if !is_hex(&source_commit, 40) {
        errors.push("trace.source_commit must be 40 lowercase hex characters".to_string());
    }
    let observer_id = required_string(value, "observer_id", "trace", &mut errors).unwrap_or_default();

    let runtime_value = value.get("runtime").unwrap_or(&Value::Null);
    require_object(runtime_value, "trace.runtime", &mut errors);
    reject_unknown_keys(
        runtime_value,
        &["holochain", "hdk", "hdi", "js_client", "dna_sha256", "wasm_sha256", "rustc", "hc_cli"],
        "trace.runtime",
        &mut errors,
    );
    let runtime = RuntimeIdentity {
        holochain: required_string(runtime_value, "holochain", "trace.runtime", &mut errors).unwrap_or_default(),
        hdk: required_string(runtime_value, "hdk", "trace.runtime", &mut errors).unwrap_or_default(),
        hdi: required_string(runtime_value, "hdi", "trace.runtime", &mut errors).unwrap_or_default(),
        js_client: required_string(runtime_value, "js_client", "trace.runtime", &mut errors).unwrap_or_default(),
        dna_sha256: required_string(runtime_value, "dna_sha256", "trace.runtime", &mut errors).unwrap_or_default(),
        wasm_sha256: required_string(runtime_value, "wasm_sha256", "trace.runtime", &mut errors).unwrap_or_default(),
        rustc: required_string(runtime_value, "rustc", "trace.runtime", &mut errors).unwrap_or_default(),
        hc_cli: required_string(runtime_value, "hc_cli", "trace.runtime", &mut errors).unwrap_or_default(),
    };
    if !is_hex(&runtime.dna_sha256, 64) {
        errors.push("trace.runtime.dna_sha256 must be 64 lowercase hex characters".to_string());
    }
    if !is_hex(&runtime.wasm_sha256, 64) {
        errors.push("trace.runtime.wasm_sha256 must be 64 lowercase hex characters".to_string());
    }

    let event_values = match value.get("events").and_then(Value::as_array) {
        Some(events) if !events.is_empty() => events,
        _ => {
            errors.push("trace.events must be a non-empty array".to_string());
            return Err(errors);
        }
    };
    let mut events = Vec::new();
    for (index, event_value) in event_values.iter().enumerate() {
        let path = format!("trace.events[{index}]");
        require_object(event_value, &path, &mut errors);
        reject_unknown_keys(
            event_value,
            &[
                "event_id",
                "scenario_event_id",
                "observed_index",
                "actor",
                "action",
                "logical_operation_id",
                "attempt_id",
                "attempt_number",
                "causal_predecessors",
                "grant_action_hash",
                "source_action_hash",
                "host_grant_transition",
                "planes",
                "observer_state_before_sha256",
                "observer_state_after_sha256",
            ],
            &path,
            &mut errors,
        );
        let planes = parse_planes(event_value.get("planes").unwrap_or(&Value::Null), &format!("{path}.planes"), &mut errors);
        let host_grant_transition = required_string(event_value, "host_grant_transition", &path, &mut errors).unwrap_or_default();
        if !GRANT_TRANSITIONS.contains(&host_grant_transition.as_str()) {
            errors.push(format!("{path}.host_grant_transition has unsupported value {host_grant_transition}"));
        }
        let causal_predecessors = required_string_array(event_value, "causal_predecessors", &path, &mut errors);
        let before = optional_string(event_value, "observer_state_before_sha256", &path, &mut errors);
        let after = optional_string(event_value, "observer_state_after_sha256", &path, &mut errors);
        for (label, digest) in [("observer_state_before_sha256", &before), ("observer_state_after_sha256", &after)] {
            if let Some(digest) = digest {
                if !is_hex(digest, 64) {
                    errors.push(format!("{path}.{label} must be 64 lowercase hex characters"));
                }
            }
        }
        events.push(ObservedEvent {
            event_id: required_string(event_value, "event_id", &path, &mut errors).unwrap_or_default(),
            scenario_event_id: required_string(event_value, "scenario_event_id", &path, &mut errors).unwrap_or_default(),
            observed_index: required_u64(event_value, "observed_index", &path, &mut errors).unwrap_or_default(),
            actor: required_string(event_value, "actor", &path, &mut errors).unwrap_or_default(),
            action: required_string(event_value, "action", &path, &mut errors).unwrap_or_default(),
            logical_operation_id: required_string(event_value, "logical_operation_id", &path, &mut errors).unwrap_or_default(),
            attempt_id: required_string(event_value, "attempt_id", &path, &mut errors).unwrap_or_default(),
            attempt_number: required_u64(event_value, "attempt_number", &path, &mut errors).unwrap_or_default(),
            causal_predecessors,
            grant_action_hash: optional_string(event_value, "grant_action_hash", &path, &mut errors),
            source_action_hash: optional_string(event_value, "source_action_hash", &path, &mut errors),
            host_grant_transition,
            planes,
            observer_state_before_sha256: before,
            observer_state_after_sha256: after,
        });
    }
    if !errors.is_empty() {
        return Err(errors);
    }
    Ok(Trace { trace_id, profile_id, scenario_id, source_commit, observer_id, runtime, events })
}

fn verify_trace_against_scenario(trace: &Trace, scenario: &Scenario) -> Vec<String> {
    let mut errors = Vec::new();
    let mut event_id_to_index = BTreeMap::new();
    let mut scenario_id_to_index = BTreeMap::new();
    let mut observed_indices = BTreeSet::new();

    if trace.events.len() != scenario.events.len() {
        errors.push(format!(
            "trace has {} events but scenario {} defines {}",
            trace.events.len(),
            scenario.id,
            scenario.events.len()
        ));
    }

    for (index, event) in trace.events.iter().enumerate() {
        if event_id_to_index.insert(event.event_id.clone(), index).is_some() {
            errors.push(format!("duplicate trace event_id {}", event.event_id));
        }
        if scenario_id_to_index.insert(event.scenario_event_id.clone(), index).is_some() {
            errors.push(format!("duplicate scenario_event_id {}", event.scenario_event_id));
        }
        if !observed_indices.insert(event.observed_index) {
            errors.push(format!("duplicate observed_index {}", event.observed_index));
        }

        if let Some(expected) = scenario.events.get(&event.scenario_event_id) {
            if event.actor != expected.actor {
                errors.push(format!("{} actor mismatch: expected {}, found {}", event.scenario_event_id, expected.actor, event.actor));
            }
            if event.action != expected.action {
                errors.push(format!("{} action mismatch: expected {}, found {}", event.scenario_event_id, expected.action, event.action));
            }
            if event.logical_operation_id != expected.request_id {
                errors.push(format!("{} logical_operation_id must equal corpus request_id {}", event.scenario_event_id, expected.request_id));
            }
            if event.attempt_number != expected.attempt {
                errors.push(format!("{} attempt_number mismatch: expected {}, found {}", event.scenario_event_id, expected.attempt, event.attempt_number));
            }
        } else {
            errors.push(format!("trace contains event {} not defined by scenario {}", event.scenario_event_id, scenario.id));
        }

        validate_event_local_invariants(event, &mut errors);
    }

    for expected_id in scenario.events.keys() {
        if !scenario_id_to_index.contains_key(expected_id) {
            errors.push(format!("trace is missing scenario event {expected_id}"));
        }
    }

    validate_trace_causality(trace, scenario, &event_id_to_index, &scenario_id_to_index, &mut errors);
    validate_revocation_order(trace, &event_id_to_index, &mut errors);
    validate_retry_idempotency(trace, &mut errors);

    for check in &scenario.checks {
        let Some(index) = scenario_id_to_index.get(&check.after) else {
            errors.push(format!("check checkpoint {} has no observed event", check.after));
            continue;
        };
        let actual = trace.events[*index].planes.as_map();
        if actual != check.planes {
            errors.push(format!(
                "scenario {} checkpoint {} outcome mismatch: expected {:?}, found {:?}",
                scenario.id, check.after, check.planes, actual
            ));
        }
    }

    errors
}

fn validate_event_local_invariants(event: &ObservedEvent, errors: &mut Vec<String>) {
    let prefix = format!("event {}", event.event_id);
    if matches!(event.planes.host_decision.as_str(), "AUTHORIZED" | "UNAUTHORIZED")
        && event.grant_action_hash.as_deref().is_none_or(str::is_empty)
    {
        errors.push(format!("{prefix}: host allow/deny must bind the exact grant ActionHash"));
    }
    if event.planes.host_decision == "UNAUTHORIZED" && event.planes.domain_result == "EXECUTED" {
        errors.push(format!("{prefix}: a host-denied call cannot have an executed domain result"));
    }
    if event.planes.source_chain == "ACTION_AUTHORED" && event.source_action_hash.as_deref().is_none_or(str::is_empty) {
        errors.push(format!("{prefix}: ACTION_AUTHORED requires source_action_hash"));
    }
    if event.planes.source_chain == "NO_ACTION" && event.source_action_hash.is_some() {
        errors.push(format!("{prefix}: NO_ACTION must not claim a source_action_hash"));
    }
    if matches!(event.planes.validator.as_str(), "ACCEPTED" | "REJECTED")
        && (event.source_action_hash.is_none() || event.planes.source_chain != "ACTION_AUTHORED")
    {
        errors.push(format!("{prefix}: validator disposition requires an authored source-chain action"));
    }
    if matches!(event.host_grant_transition.as_str(), "CREATED" | "REVOKED")
        && event.grant_action_hash.as_deref().is_none_or(str::is_empty)
    {
        errors.push(format!("{prefix}: CREATED/REVOKED host grant transitions require exact grant_action_hash"));
    }
    if event.planes.validator == "REJECTED" && event.planes.observer == "ACKNOWLEDGED" {
        match (&event.observer_state_before_sha256, &event.observer_state_after_sha256) {
            (Some(before), Some(after)) if before == after => {}
            (Some(_), Some(_)) => errors.push(format!("{prefix}: rejected transition changed the observer's canonical state digest")),
            _ => errors.push(format!("{prefix}: acknowledged validator rejection requires before/after observer state digests")),
        }
    }
}

fn validate_trace_causality(
    trace: &Trace,
    scenario: &Scenario,
    event_id_to_index: &BTreeMap<String, usize>,
    scenario_id_to_index: &BTreeMap<String, usize>,
    errors: &mut Vec<String>,
) {
    let mut indegree: BTreeMap<String, usize> = trace.events.iter().map(|event| (event.event_id.clone(), 0)).collect();
    let mut children: BTreeMap<String, Vec<String>> = trace.events.iter().map(|event| (event.event_id.clone(), Vec::new())).collect();

    for event in &trace.events {
        for parent in &event.causal_predecessors {
            if !event_id_to_index.contains_key(parent) {
                errors.push(format!("event {} references unknown causal predecessor {parent}", event.event_id));
                continue;
            }
            let parent_index = event_id_to_index[parent];
            let parent_event = &trace.events[parent_index];
            if parent_event.observed_index >= event.observed_index {
                errors.push(format!("event {} causal predecessor {parent} is not earlier in the observed log", event.event_id));
            }
            *indegree.entry(event.event_id.clone()).or_default() += 1;
            children.entry(parent.clone()).or_default().push(event.event_id.clone());
        }

        if let Some(expected) = scenario.events.get(&event.scenario_event_id) {
            for corpus_parent in &expected.happens_before {
                if let Some(parent_index) = scenario_id_to_index.get(corpus_parent) {
                    let parent_event_id = &trace.events[*parent_index].event_id;
                    if !event.causal_predecessors.contains(parent_event_id) {
                        errors.push(format!(
                            "event {} omits required causal predecessor {} (event_id {})",
                            event.event_id, corpus_parent, parent_event_id
                        ));
                    }
                }
            }
        }
    }

    let mut queue: VecDeque<String> = indegree.iter().filter_map(|(id, count)| (*count == 0).then_some(id.clone())).collect();
    let mut visited = 0usize;
    while let Some(id) = queue.pop_front() {
        visited += 1;
        for child in children.get(&id).into_iter().flatten() {
            if let Some(count) = indegree.get_mut(child) {
                *count -= 1;
                if *count == 0 {
                    queue.push_back(child.clone());
                }
            }
        }
    }
    if visited != trace.events.len() {
        errors.push(format!("scenario {} trace causal graph contains a cycle", scenario.id));
    }
}

fn validate_revocation_order(
    trace: &Trace,
    event_id_to_index: &BTreeMap<String, usize>,
    errors: &mut Vec<String>,
) {
    let mut ancestors: BTreeMap<String, BTreeSet<String>> = BTreeMap::new();
    for event in &trace.events {
        let mut found = BTreeSet::new();
        let mut queue: VecDeque<String> = event.causal_predecessors.iter().cloned().collect();
        while let Some(parent) = queue.pop_front() {
            if !found.insert(parent.clone()) {
                continue;
            }
            if let Some(parent_index) = event_id_to_index.get(&parent) {
                for grandparent in &trace.events[*parent_index].causal_predecessors {
                    queue.push_back(grandparent.clone());
                }
            }
        }
        ancestors.insert(event.event_id.clone(), found);
    }

    for revoke in trace.events.iter().filter(|event| event.host_grant_transition == "REVOKED") {
        let Some(grant_hash) = revoke.grant_action_hash.as_deref() else { continue };
        for later in &trace.events {
            if later.grant_action_hash.as_deref() == Some(grant_hash)
                && ancestors.get(&later.event_id).is_some_and(|set| set.contains(&revoke.event_id))
                && later.planes.host_decision == "AUTHORIZED"
            {
                errors.push(format!(
                    "event {} is AUTHORIZED after causally preceding revocation {} of the same grant",
                    later.event_id, revoke.event_id
                ));
            }
        }
    }
}

fn validate_retry_idempotency(trace: &Trace, errors: &mut Vec<String>) {
    let mut by_operation: BTreeMap<&str, Vec<&ObservedEvent>> = BTreeMap::new();
    for event in &trace.events {
        by_operation.entry(&event.logical_operation_id).or_default().push(event);
    }

    for (operation_id, events) in by_operation {
        let mut attempt_to_id: BTreeMap<u64, &str> = BTreeMap::new();
        let mut id_to_attempt: BTreeMap<&str, u64> = BTreeMap::new();
        let mut created_grants = Vec::new();
        let mut grant_hashes = BTreeSet::new();

        for event in &events {
            if let Some(existing) = attempt_to_id.insert(event.attempt_number, &event.attempt_id) {
                if existing != event.attempt_id {
                    errors.push(format!("logical operation {operation_id}: attempt {} has multiple attempt_id values", event.attempt_number));
                }
            }
            if let Some(existing) = id_to_attempt.insert(&event.attempt_id, event.attempt_number) {
                if existing != event.attempt_number {
                    errors.push(format!("logical operation {operation_id}: one attempt_id is reused across attempts"));
                }
            }
            if let Some(hash) = event.grant_action_hash.as_deref() {
                grant_hashes.insert(hash.to_string());
            }
            if event.host_grant_transition == "CREATED" {
                created_grants.push(event);
            }
        }

        if created_grants.len() > 1 {
            errors.push(format!("logical operation {operation_id}: retry created more than one host grant"));
        }
        if created_grants.len() > 0 && grant_hashes.len() > 1 {
            errors.push(format!("logical operation {operation_id}: retry changed the grant ActionHash"));
        }
        let mut first_created_attempt = None;
        if let Some(created) = created_grants.first() {
            first_created_attempt = Some(created.attempt_number);
        }
        if let Some(first_attempt) = first_created_attempt {
            for event in &events {
                if event.action.contains("retry")
                    && event.attempt_number > first_attempt
                    && event.host_grant_transition == "CREATED"
                {
                    errors.push(format!("logical operation {operation_id}: retry minted a second grant"));
                }
            }
        }
    }
}

fn reject_unknown_keys(value: &Value, allowed: &[&str], path: &str, errors: &mut Vec<String>) {
    if let Some(object) = value.as_object() {
        for key in object.keys() {
            if !allowed.contains(&key.as_str()) {
                errors.push(format!("{path}: unknown field {key}"));
            }
        }
    }
}

fn require_object<'a>(value: &'a Value, path: &str, errors: &mut Vec<String>) -> Option<&'a Map<String, Value>> {
    match value.as_object() {
        Some(object) => Some(object),
        None => {
            errors.push(format!("{path} must be an object"));
            None
        }
    }
}

fn required_string(value: &Value, key: &str, path: &str, errors: &mut Vec<String>) -> Option<String> {
    match value.get(key).and_then(Value::as_str) {
        Some(value) if !value.is_empty() => Some(value.to_string()),
        _ => {
            errors.push(format!("{path}.{key} must be a non-empty string"));
            None
        }
    }
}

fn optional_string(value: &Value, key: &str, path: &str, errors: &mut Vec<String>) -> Option<String> {
    match value.get(key) {
        None | Some(Value::Null) => None,
        Some(Value::String(value)) if !value.is_empty() => Some(value.clone()),
        _ => {
            errors.push(format!("{path}.{key} must be a non-empty string or null"));
            None
        }
    }
}

fn required_u64(value: &Value, key: &str, path: &str, errors: &mut Vec<String>) -> Option<u64> {
    match value.get(key).and_then(Value::as_u64) {
        Some(value) => Some(value),
        _ => {
            errors.push(format!("{path}.{key} must be an unsigned integer"));
            None
        }
    }
}

fn get_str<'a>(value: &'a Value, key: &str) -> Option<&'a str> {
    value.get(key).and_then(Value::as_str)
}

fn get_bool(value: &Value, key: &str) -> Option<bool> {
    value.get(key).and_then(Value::as_bool)
}

fn get_string_array(value: &Value, key: &str) -> Option<Vec<String>> {
    value.get(key)?.as_array()?.iter().map(|item| item.as_str().map(str::to_owned)).collect()
}

fn get_nested_str<'a>(value: &'a Value, path: &[&str]) -> Option<&'a str> {
    let mut current = value;
    for key in path {
        current = current.get(*key)?;
    }
    current.as_str()
}

fn optional_string_array(value: &Value, key: &str, path: &str, errors: &mut Vec<String>) -> Vec<String> {
    match value.get(key) {
        None | Some(Value::Null) => Vec::new(),
        Some(Value::Array(items)) => {
            let mut output = Vec::new();
            for (index, item) in items.iter().enumerate() {
                match item.as_str() {
                    Some(value) if !value.is_empty() => output.push(value.to_string()),
                    _ => errors.push(format!("{path}.{key}[{index}] must be a non-empty string")),
                }
            }
            output
        }
        _ => {
            errors.push(format!("{path}.{key} must be an array or null"));
            Vec::new()
        }
    }
}

fn required_string_array(value: &Value, key: &str, path: &str, errors: &mut Vec<String>) -> Vec<String> {
    match value.get(key).and_then(Value::as_array) {
        Some(items) => {
            let mut output = Vec::new();
            for (index, item) in items.iter().enumerate() {
                match item.as_str() {
                    Some(value) if !value.is_empty() => output.push(value.to_string()),
                    _ => errors.push(format!("{path}.{key}[{index}] must be a non-empty string")),
                }
            }
            output
        }
        _ => {
            errors.push(format!("{path}.{key} must be an array"));
            Vec::new()
        }
    }
}

fn is_hex(value: &str, expected_len: usize) -> bool {
    value.len() == expected_len
        && value
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}

#[cfg(test)]
mod tests {
    use super::*;

    const CORPUS: &str = include_str!("../../../../docs/security/authorization-lifecycle-v1.corpus.json");
    const SCHEMA: &str = include_str!("../../../../docs/security/authorization-lifecycle-v1.schema.json");

    fn corpus_and_schema() -> (Vec<u8>, Vec<u8>, Corpus) {
        let corpus_bytes = CORPUS.as_bytes().to_vec();
        let schema_bytes = SCHEMA.as_bytes().to_vec();
        let (corpus, _) = parse_and_validate_corpus(&corpus_bytes, &schema_bytes).expect("canonical corpus should validate");
        (corpus_bytes, schema_bytes, corpus)
    }

    fn event_json(
        event_id: &str,
        scenario_event_id: &str,
        observed_index: u64,
        actor: &str,
        action: &str,
        logical_operation_id: &str,
        attempt_id: &str,
        attempt_number: u64,
        parents: &[&str],
        grant_hash: Option<&str>,
        source_hash: Option<&str>,
        transition: &str,
        planes: Value,
        before: Option<&str>,
        after: Option<&str>,
    ) -> Value {
        json!({
            "event_id": event_id,
            "scenario_event_id": scenario_event_id,
            "observed_index": observed_index,
            "actor": actor,
            "action": action,
            "logical_operation_id": logical_operation_id,
            "attempt_id": attempt_id,
            "attempt_number": attempt_number,
            "causal_predecessors": parents,
            "grant_action_hash": grant_hash,
            "source_action_hash": source_hash,
            "host_grant_transition": transition,
            "planes": planes,
            "observer_state_before_sha256": before,
            "observer_state_after_sha256": after
        })
    }

    fn runtime_json() -> Value {
        json!({
            "holochain": "0.7.0",
            "hdk": "0.7.0",
            "hdi": "0.8.0",
            "js_client": "0.21.0",
            "dna_sha256": "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
            "wasm_sha256": "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb",
            "rustc": "1.96.0",
            "hc_cli": "0.7.0"
        })
    }

    fn revocation_trace() -> Value {
        let (corpus_bytes, _, corpus) = corpus_and_schema();
        let source_commit = corpus
            .scenarios
            .get("REV-01")
            .map(|_| "9e12dd46bdd8371e8ed9626a17bc06f21c141a52")
            .unwrap();
        json!({
            "trace_version": 1,
            "profile_id": PROFILE_ID,
            "trace_id": "trace-revocation-001",
            "scenario_id": "REV-01",
            "source_commit": source_commit,
            "observer_id": "peer-observer-01",
            "runtime": runtime_json(),
            "events": [
                event_json(
                    "t1", "e1", 1, "grantor", "revoke_exact_host_grant",
                    "revoke-01", "attempt-1", 1, &[], Some("cap-action-01"),
                    Some("source-action-revoke-01"), "REVOKED",
                    json!({"host_decision":"NOT_APPLICABLE","domain_result":"EXECUTED","source_chain":"ACTION_AUTHORED","validator":"NOT_OBSERVED","projection":"STALE","observer":"UNKNOWN"}),
                    None, None
                ),
                event_json(
                    "t2", "e2", 2, "grantee", "invoke_using_retained_claim",
                    "call-08", "attempt-1", 1, &["t1"], Some("cap-action-01"),
                    None, "UNCHANGED",
                    json!({"host_decision":"UNAUTHORIZED","domain_result":"NOT_INVOKED","source_chain":"NO_ACTION","validator":"NOT_OBSERVED","projection":"STALE","observer":"ACKNOWLEDGED"}),
                    None, None
                )
            ]
        })
    }

    #[test]
    fn canonical_corpus_and_schema_validate() {
        let (corpus, schema, parsed) = corpus_and_schema();
        assert_eq!(parsed.scenarios.len(), 28);
        assert!(!sha256_hex(&corpus).is_empty());
        assert!(!sha256_hex(&schema).is_empty());
    }

    #[test]
    fn rejects_duplicate_event_ids() {
        let mut value: Value = serde_json::from_str(CORPUS).unwrap();
        let events = value["scenarios"][0]["events"].as_array_mut().unwrap();
        events[1]["id"] = events[0]["id"].clone();
        let error = parse_and_validate_corpus(
            serde_json::to_string(&value).unwrap().as_bytes(),
            SCHEMA.as_bytes(),
        )
        .unwrap_err();
        assert!(error.iter().any(|item| item.contains("duplicate event id")));
    }

    #[test]
    fn rejects_happens_before_cycles() {
        let mut value: Value = serde_json::from_str(CORPUS).unwrap();
        value["scenarios"][0]["events"][0]["happens_before"] = json!(["e2"]);
        let error = parse_and_validate_corpus(
            serde_json::to_string(&value).unwrap().as_bytes(),
            SCHEMA.as_bytes(),
        )
        .unwrap_err();
        assert!(error.iter().any(|item| item.contains("contains a cycle")));
    }

    #[test]
    fn accepts_causally_ordered_post_revoke_denial() {
        let (corpus, schema, _) = corpus_and_schema();
        let trace = serde_json::to_vec(&revocation_trace()).unwrap();
        let receipt = verify_trace(&corpus, &schema, &trace).expect("valid REV-01 trace");
        assert_eq!(receipt.report["verification"], "PASS");
    }

    #[test]
    fn rejects_host_allow_after_causal_revocation_of_same_grant() {
        let (corpus, schema, _) = corpus_and_schema();
        let mut trace = revocation_trace();
        trace["events"][1]["planes"]["host_decision"] = json!("AUTHORIZED");
        let bytes = serde_json::to_vec(&trace).unwrap();
        let error = verify_trace(&corpus, &schema, &bytes).unwrap_err();
        assert!(error.iter().any(|item| item.contains("AUTHORIZED after causally preceding revocation")));
    }

    #[test]
    fn rejects_missing_declared_causal_parent() {
        let (corpus, schema, _) = corpus_and_schema();
        let mut trace = revocation_trace();
        trace["events"][1]["causal_predecessors"] = json!([]);
        let bytes = serde_json::to_vec(&trace).unwrap();
        let error = verify_trace(&corpus, &schema, &bytes).unwrap_err();
        assert!(error.iter().any(|item| item.contains("omits required causal predecessor")));
    }

    #[test]
    fn rejects_secret_bearing_unknown_fields() {
        let (corpus, schema, _) = corpus_and_schema();
        let mut trace = revocation_trace();
        trace["events"][0]["cap_secret"] = json!("must-never-appear");
        let bytes = serde_json::to_vec(&trace).unwrap();
        let error = verify_trace(&corpus, &schema, &bytes).unwrap_err();
        assert!(error.iter().any(|item| item.contains("unknown field cap_secret")));
    }

    #[test]
    fn rejects_remote_validator_rejection_with_changed_observer_state() {
        let (corpus, schema, parsed) = corpus_and_schema();
        let scenario = parsed.scenarios.get("VAL-01").unwrap();
        let mut trace = json!({
            "trace_version": 1,
            "profile_id": PROFILE_ID,
            "trace_id": "trace-validator-001",
            "scenario_id": "VAL-01",
            "source_commit": "9e12dd46bdd8371e8ed9626a17bc06f21c141a52",
            "observer_id": "peer-observer-02",
            "runtime": runtime_json(),
            "events": []
        });
        for (index, event) in scenario.events.values().enumerate() {
            let is_final = event.id == "e3";
            let planes = if is_final {
                json!({"host_decision":"NOT_APPLICABLE","domain_result":"EXECUTED","source_chain":"ACTION_AUTHORED","validator":"REJECTED","projection":"CURRENT","observer":"ACKNOWLEDGED"})
            } else {
                json!({"host_decision":"NOT_APPLICABLE","domain_result":"EXECUTED","source_chain":"ACTION_AUTHORED","validator":"NOT_OBSERVED","projection":"UNKNOWN","observer":"ACKNOWLEDGED"})
            };
            let mut observed = event_json(
                &format!("v{}", index + 1),
                &event.id,
                index as u64 + 1,
                &event.actor,
                &event.action,
                &event.request_id,
                &format!("attempt-{}", event.attempt),
                event.attempt,
                &[],
                None,
                Some("source-action-mailbox-19"),
                "NOT_APPLICABLE",
                planes,
                if is_final { Some("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa") } else { None },
                if is_final { Some("cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc") } else { None },
            );
            if is_final {
                observed["causal_predecessors"] = json!(["v2"]);
            } else if event.id == "e2" {
                observed["causal_predecessors"] = json!(["v1"]);
            }
            trace["events"].as_array_mut().unwrap().push(observed);
        }
        let bytes = serde_json::to_vec(&trace).unwrap();
        let error = verify_trace(&corpus, &schema, &bytes).unwrap_err();
        assert!(error.iter().any(|item| item.contains("changed the observer's canonical state digest")));
    }
}
