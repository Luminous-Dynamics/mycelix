//! Independent structural and causal-consistency checks for Pulse Fabric traces.
//!
//! This crate delegates Draft 2020-12 instance validation to a pinned,
//! network-disabled schema engine. Its semantic layer does not implement
//! cryptography, Holochain authority, or the candidate application's delivery
//! decisions. It checks causal consistency and requires a SUCCESS_VERIFIED
//! projection to be supported by recipient-side events and a separately
//! attributed receipt-verifier event.

use serde_json::{Map, Value};
use std::collections::{BTreeSet, HashMap, HashSet, VecDeque};

const PROFILE_ID: &str = "PULSE-FABRIC-QUAL-001";
const SCHEMA_VERSION: &str = "0.1.0";

const EVENT_TYPES: &[&str] = &[
    "intent_recorded",
    "crypto_profile_selected",
    "session_state_advanced",
    "authority_host_admitted",
    "authority_host_denied",
    "domain_authorized",
    "domain_denied",
    "source_action_authored",
    "integrity_validation_accepted",
    "integrity_validation_rejected",
    "dht_projection_observed",
    "outbox_committed",
    "transport_attempt_started",
    "transport_attempt_outcome_unknown",
    "transport_ack_observed",
    "recipient_admitted",
    "recipient_inbox_committed",
    "recipient_decryption_verified",
    "receipt_created",
    "receipt_verified",
    "receipt_rejected",
    "revocation_committed",
    "recovery_started",
    "recovery_partial",
    "recovery_completed",
    "ui_projection_emitted",
    "terminal_failure",
    "terminal_expiry",
];

const EVIDENCE_ROLES: &[&str] = &[
    "client",
    "host_conductor",
    "domain_coordinator",
    "source_chain_author",
    "integrity_validator",
    "dht_observer",
    "transport_adapter",
    "recipient",
    "receipt_verifier",
    "recovery_verifier",
    "ui_projection",
    "independent_auditor",
];

const CAUSAL_RELATIONS: &[&str] = &[
    "local_program_order",
    "source_chain_successor",
    "request_response",
    "attempt_for",
    "transport_for",
    "commits",
    "receipt_for",
    "verifies",
    "revokes",
    "checkpoint_descendant_of",
    "recovered_from",
    "derived_from",
];

const ATTEMPT_REQUIRED: &[&str] = &[
    "authority_host_admitted",
    "authority_host_denied",
    "domain_authorized",
    "domain_denied",
    "source_action_authored",
    "integrity_validation_accepted",
    "integrity_validation_rejected",
    "transport_attempt_started",
    "transport_attempt_outcome_unknown",
    "transport_ack_observed",
    "recipient_admitted",
    "recipient_inbox_committed",
    "recipient_decryption_verified",
    "receipt_created",
    "receipt_verified",
    "receipt_rejected",
];

const TERMINAL_DISPOSITIONS: &[&str] = &[
    "SUCCESS_VERIFIED",
    "FAILED",
    "EXPIRED",
    "REVOKED",
    "UNKNOWN",
    "INCOMPLETE",
    "NOT_APPLICABLE",
];

const EVIDENCE_KINDS: &[&str] = &[
    "signed_statement",
    "action_hash",
    "grant_action_hash",
    "validation_receipt",
    "transport_receipt",
    "recipient_receipt",
    "source_checkpoint",
    "artifact",
    "model_counterexample",
    "other",
];

const FORBIDDEN_KEYS: &[&str] = &[
    "plaintext",
    "plain_text",
    "message_body",
    "message_content",
    "message_text",
    "message_subject",
    "raw_message",
    "raw_message_text",
    "body",
    "subject",
    "content",
    "private_key",
    "private_seed",
    "raw_key",
    "capability_secret",
    "session_secret",
    "recovery_material",
    "decryption_key",
    "signing_key",
    "secret",
];

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Finding {
    pub path: String,
    pub message: String,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct VerificationSummary {
    pub status: &'static str,
    pub run_id: String,
    pub source_sha: String,
    pub scenario_id: String,
    pub terminal_disposition: String,
    pub event_count: usize,
    pub operation_count: usize,
    pub causal_edge_count: usize,
}

#[derive(Debug, Clone)]
struct Predecessor {
    event_id: String,
    relation: String,
}

#[derive(Debug, Clone)]
struct EventRecord {
    event_id: String,
    logical_operation_id: String,
    attempt_id: Option<String>,
    event_type: String,
    evidence_role: String,
    actor_ref: String,
    content_commitment: Option<String>,
    clock_domain: String,
    monotonic_ns: u64,
    predecessors: Vec<Predecessor>,
    evidence_kinds: Vec<String>,
}

fn finding(findings: &mut Vec<Finding>, path: impl Into<String>, message: impl Into<String>) {
    findings.push(Finding {
        path: path.into(),
        message: message.into(),
    });
}

fn string_field(
    object: &Map<String, Value>,
    key: &str,
    path: &str,
    findings: &mut Vec<Finding>,
) -> Option<String> {
    match object.get(key).and_then(Value::as_str) {
        Some(value) if !value.trim().is_empty() => Some(value.to_owned()),
        _ => {
            finding(
                findings,
                format!("{path}.{key}"),
                "required non-empty string is missing or has the wrong type",
            );
            None
        }
    }
}

fn object_field<'a>(
    object: &'a Map<String, Value>,
    key: &str,
    path: &str,
    findings: &mut Vec<Finding>,
) -> Option<&'a Map<String, Value>> {
    match object.get(key).and_then(Value::as_object) {
        Some(value) => Some(value),
        None => {
            finding(
                findings,
                format!("{path}.{key}"),
                "required object is missing or has the wrong type",
            );
            None
        }
    }
}

fn array_field<'a>(
    object: &'a Map<String, Value>,
    key: &str,
    path: &str,
    findings: &mut Vec<Finding>,
) -> Option<&'a Vec<Value>> {
    match object.get(key).and_then(Value::as_array) {
        Some(value) => Some(value),
        None => {
            finding(
                findings,
                format!("{path}.{key}"),
                "required array is missing or has the wrong type",
            );
            None
        }
    }
}

fn is_lower_hex(value: &str, length: usize) -> bool {
    value.len() == length
        && value
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}

fn validate_digest(
    value: Option<&str>,
    length: usize,
    path: &str,
    findings: &mut Vec<Finding>,
) {
    match value {
        Some(digest) if is_lower_hex(digest, length) => {}
        _ => finding(
            findings,
            path,
            format!("expected {length} lowercase hexadecimal characters"),
        ),
    }
}

fn validate_optional_digest(
    object: &Map<String, Value>,
    key: &str,
    path: &str,
    findings: &mut Vec<Finding>,
) {
    match object.get(key) {
        None | Some(Value::Null) => {}
        Some(Value::String(value)) if is_lower_hex(value, 64) => {}
        _ => finding(
            findings,
            format!("{path}.{key}"),
            "expected null or a 64-character lowercase SHA-256 digest",
        ),
    }
}

fn expected_role(event_type: &str) -> Option<&'static [&'static str]> {
    match event_type {
        "intent_recorded"
        | "crypto_profile_selected"
        | "session_state_advanced"
        | "outbox_committed"
        | "terminal_failure"
        | "terminal_expiry" => Some(&["client"]),
        "authority_host_admitted"
        | "authority_host_denied"
        | "revocation_committed" => Some(&["host_conductor"]),
        "domain_authorized" | "domain_denied" => Some(&["domain_coordinator"]),
        "source_action_authored" => Some(&["source_chain_author"]),
        "integrity_validation_accepted" | "integrity_validation_rejected" => {
            Some(&["integrity_validator"])
        }
        "dht_projection_observed" => Some(&["dht_observer"]),
        "transport_attempt_started"
        | "transport_attempt_outcome_unknown"
        | "transport_ack_observed" => Some(&["transport_adapter"]),
        "recipient_admitted"
        | "recipient_inbox_committed"
        | "recipient_decryption_verified"
        | "receipt_created" => Some(&["recipient"]),
        "receipt_verified" | "receipt_rejected" => {
            Some(&["receipt_verifier", "independent_auditor"])
        }
        "recovery_started" | "recovery_partial" | "recovery_completed" => {
            Some(&["recovery_verifier"])
        }
        "ui_projection_emitted" => Some(&["ui_projection"]),
        _ => None,
    }
}

fn scan_forbidden_keys(value: &Value, path: &str, findings: &mut Vec<Finding>) {
    match value {
        Value::Object(object) => {
            for (key, child) in object {
                let normalized = key.to_ascii_lowercase().replace('-', "_");
                if FORBIDDEN_KEYS.contains(&normalized.as_str()) {
                    finding(
                        findings,
                        format!("{path}.{key}"),
                        "field name is prohibited in a redacted evidence trace",
                    );
                }
                scan_forbidden_keys(child, &format!("{path}.{key}"), findings);
            }
        }
        Value::Array(items) => {
            for (index, child) in items.iter().enumerate() {
                scan_forbidden_keys(child, &format!("{path}[{index}]"), findings);
            }
        }
        _ => {}
    }
}

fn parse_event(
    value: &Value,
    index: usize,
    findings: &mut Vec<Finding>,
) -> Option<EventRecord> {
    let path = format!("$.events[{index}]");
    let object = match value.as_object() {
        Some(object) => object,
        None => {
            finding(findings, &path, "event must be an object");
            return None;
        }
    };

    let event_id = string_field(object, "event_id", &path, findings)?;
    let logical_operation_id =
        string_field(object, "logical_operation_id", &path, findings)?;
    let event_type = string_field(object, "event_type", &path, findings)?;
    let evidence_role = string_field(object, "evidence_role", &path, findings)?;
    let actor_ref = string_field(object, "actor_ref", &path, findings)?;
    let _redaction_profile = string_field(object, "redaction_profile", &path, findings)?;

    if !EVENT_TYPES.contains(&event_type.as_str()) {
        finding(
            findings,
            format!("{path}.event_type"),
            format!("unknown event type: {event_type}"),
        );
    }
    if !EVIDENCE_ROLES.contains(&evidence_role.as_str()) {
        finding(
            findings,
            format!("{path}.evidence_role"),
            format!("unknown evidence role: {evidence_role}"),
        );
    }
    if let Some(allowed) = expected_role(&event_type) {
        if !allowed.contains(&evidence_role.as_str()) {
            finding(
                findings,
                format!("{path}.evidence_role"),
                format!(
                    "event type {event_type} requires one of these evidence roles: {}",
                    allowed.join(", ")
                ),
            );
        }
    }

    let attempt_id = match object.get("attempt_id") {
        Some(Value::Null) => None,
        Some(Value::String(value)) if !value.trim().is_empty() => Some(value.clone()),
        _ => {
            finding(
                findings,
                format!("{path}.attempt_id"),
                "attempt_id must be null or a non-empty string",
            );
            None
        }
    };
    if ATTEMPT_REQUIRED.contains(&event_type.as_str()) && attempt_id.is_none() {
        finding(
            findings,
            format!("{path}.attempt_id"),
            format!("event type {event_type} requires a concrete attempt_id"),
        );
    }

    let content_commitment = match object.get("content_commitment") {
        None | Some(Value::Null) => None,
        Some(Value::String(value)) if is_lower_hex(value, 64) => Some(value.clone()),
        _ => {
            finding(
                findings,
                format!("{path}.content_commitment"),
                "content_commitment must be null or a 64-character lowercase digest",
            );
            None
        }
    };

    let observed_at = object_field(object, "observed_at", &path, findings);
    let (clock_domain, monotonic_ns) = match observed_at {
        Some(observed) => {
            let clock_domain =
                string_field(observed, "clock_domain", &format!("{path}.observed_at"), findings);
            let monotonic_ns = match observed.get("monotonic_ns").and_then(Value::as_u64) {
                Some(value) => Some(value),
                None => {
                    finding(
                        findings,
                        format!("{path}.observed_at.monotonic_ns"),
                        "monotonic_ns must be a non-negative integer",
                    );
                    None
                }
            };
            (
                clock_domain.unwrap_or_default(),
                monotonic_ns.unwrap_or_default(),
            )
        }
        None => (String::new(), 0),
    };

    let mut predecessors = Vec::new();
    if let Some(items) = array_field(object, "causal_predecessors", &path, findings) {
        let mut exact_edges = HashSet::new();
        for (edge_index, edge) in items.iter().enumerate() {
            let edge_path = format!("{path}.causal_predecessors[{edge_index}]");
            let edge_object = match edge.as_object() {
                Some(value) => value,
                None => {
                    finding(findings, &edge_path, "causal predecessor must be an object");
                    continue;
                }
            };
            let predecessor_id =
                string_field(edge_object, "event_id", &edge_path, findings);
            let relation = string_field(edge_object, "relation", &edge_path, findings);
            if let (Some(predecessor_id), Some(relation)) = (predecessor_id, relation) {
                if !CAUSAL_RELATIONS.contains(&relation.as_str()) {
                    finding(
                        findings,
                        format!("{edge_path}.relation"),
                        format!("unknown causal relation: {relation}"),
                    );
                }
                let edge_key = (predecessor_id.clone(), relation.clone());
                if !exact_edges.insert(edge_key) {
                    finding(
                        findings,
                        &edge_path,
                        "duplicate causal predecessor relation",
                    );
                }
                predecessors.push(Predecessor {
                    event_id: predecessor_id,
                    relation,
                });
            }
        }
    }

    let mut evidence_kinds = Vec::new();
    if let Some(items) = array_field(object, "evidence_refs", &path, findings) {
        for (evidence_index, evidence) in items.iter().enumerate() {
            let evidence_path = format!("{path}.evidence_refs[{evidence_index}]");
            let evidence_object = match evidence.as_object() {
                Some(value) => value,
                None => {
                    finding(findings, &evidence_path, "evidence reference must be an object");
                    continue;
                }
            };
            let kind = string_field(evidence_object, "kind", &evidence_path, findings);
            let digest = string_field(evidence_object, "sha256", &evidence_path, findings);
            let _scope = string_field(evidence_object, "scope", &evidence_path, findings);
            if let Some(kind) = kind {
                if !EVIDENCE_KINDS.contains(&kind.as_str()) {
                    finding(
                        findings,
                        format!("{evidence_path}.kind"),
                        format!("unknown evidence kind: {kind}"),
                    );
                }
                evidence_kinds.push(kind);
            }
            if let Some(digest) = digest {
                validate_digest(
                    Some(&digest),
                    64,
                    &format!("{evidence_path}.sha256"),
                    findings,
                );
            }
        }
    }

    Some(EventRecord {
        event_id,
        logical_operation_id,
        attempt_id,
        event_type,
        evidence_role,
        actor_ref,
        content_commitment,
        clock_domain,
        monotonic_ns,
        predecessors,
        evidence_kinds,
    })
}

fn ancestors(
    event: &EventRecord,
    by_id: &HashMap<String, usize>,
    events: &[EventRecord],
) -> HashSet<String> {
    let mut result = HashSet::new();
    let mut pending: VecDeque<String> = event
        .predecessors
        .iter()
        .map(|predecessor| predecessor.event_id.clone())
        .collect();

    while let Some(event_id) = pending.pop_front() {
        if !result.insert(event_id.clone()) {
            continue;
        }
        if let Some(index) = by_id.get(&event_id) {
            for predecessor in &events[*index].predecessors {
                pending.push_back(predecessor.event_id.clone());
            }
        }
    }
    result
}

fn verify_acyclic_graph(
    events: &[EventRecord],
    by_id: &HashMap<String, usize>,
    findings: &mut Vec<Finding>,
) {
    let mut indegree = vec![0usize; events.len()];
    let mut outgoing: Vec<Vec<usize>> = vec![Vec::new(); events.len()];

    for (child_index, event) in events.iter().enumerate() {
        for predecessor in &event.predecessors {
            match by_id.get(&predecessor.event_id) {
                None => finding(
                    findings,
                    format!("$.events.{}.causal_predecessors", event.event_id),
                    format!(
                        "causal predecessor {} does not resolve within this trace",
                        predecessor.event_id
                    ),
                ),
                Some(parent_index) => {
                    if *parent_index == child_index {
                        finding(
                            findings,
                            format!("$.events.{}.causal_predecessors", event.event_id),
                            "event cannot be its own causal predecessor",
                        );
                    }
                    indegree[child_index] += 1;
                    outgoing[*parent_index].push(child_index);

                    let parent = &events[*parent_index];
                    if parent.clock_domain == event.clock_domain
                        && parent.monotonic_ns > event.monotonic_ns
                    {
                        finding(
                            findings,
                            format!("$.events.{}.observed_at", event.event_id),
                            format!(
                                "causal predecessor {} has a later monotonic timestamp in the same clock domain",
                                predecessor.event_id
                            ),
                        );
                    }
                }
            }
        }
    }

    let mut ready = VecDeque::new();
    for (index, degree) in indegree.iter().enumerate() {
        if *degree == 0 {
            ready.push_back(index);
        }
    }

    let mut visited = 0usize;
    while let Some(index) = ready.pop_front() {
        visited += 1;
        for child in &outgoing[index] {
            indegree[*child] -= 1;
            if indegree[*child] == 0 {
                ready.push_back(*child);
            }
        }
    }

    if visited != events.len() {
        finding(
            findings,
            "$.events",
            "causal predecessor graph contains a cycle",
        );
    }
}

fn verify_success_projection(
    events: &[EventRecord],
    by_id: &HashMap<String, usize>,
    findings: &mut Vec<Finding>,
) {
    let projection_indices: Vec<usize> = events
        .iter()
        .enumerate()
        .filter_map(|(index, event)| {
            (event.event_type == "ui_projection_emitted").then_some(index)
        })
        .collect();

    if projection_indices.is_empty() {
        finding(
            findings,
            "$.terminal_disposition",
            "SUCCESS_VERIFIED requires a user-visible projection event",
        );
        return;
    }

    for projection_index in projection_indices {
        let projection = &events[projection_index];
        let projection_ancestors = ancestors(projection, by_id, events);
        let mut supported = false;

        for receipt in events.iter() {
            if receipt.event_type != "receipt_verified"
                || receipt.logical_operation_id != projection.logical_operation_id
                || !projection_ancestors.contains(&receipt.event_id)
            {
                continue;
            }

            let receipt_ancestors = ancestors(receipt, by_id, events);
            let admitted = events.iter().find(|event| {
                event.event_type == "recipient_admitted"
                    && event.logical_operation_id == receipt.logical_operation_id
                    && receipt_ancestors.contains(&event.event_id)
            });
            let committed = events.iter().find(|event| {
                event.event_type == "recipient_inbox_committed"
                    && event.logical_operation_id == receipt.logical_operation_id
                    && receipt_ancestors.contains(&event.event_id)
            });
            let decrypted = events.iter().find(|event| {
                event.event_type == "recipient_decryption_verified"
                    && event.logical_operation_id == receipt.logical_operation_id
                    && receipt_ancestors.contains(&event.event_id)
            });
            let created = events.iter().find(|event| {
                event.event_type == "receipt_created"
                    && event.logical_operation_id == receipt.logical_operation_id
                    && receipt_ancestors.contains(&event.event_id)
            });

            let (Some(admitted), Some(committed), Some(decrypted), Some(created)) =
                (admitted, committed, decrypted, created)
            else {
                continue;
            };

            let commitments = [
                committed.content_commitment.as_deref(),
                decrypted.content_commitment.as_deref(),
                created.content_commitment.as_deref(),
                receipt.content_commitment.as_deref(),
            ];
            let first_commitment = commitments[0];
            if first_commitment.is_none()
                || commitments.iter().any(|candidate| *candidate != first_commitment)
            {
                continue;
            }

            let recipient_actor = admitted.actor_ref.as_str();
            if committed.actor_ref != recipient_actor
                || decrypted.actor_ref != recipient_actor
                || created.actor_ref != recipient_actor
                || receipt.actor_ref == recipient_actor
            {
                continue;
            }
            if !receipt.evidence_kinds.iter().any(|kind| {
                kind == "recipient_receipt" || kind == "signed_statement"
            }) {
                continue;
            }

            supported = true;
            break;
        }

        if !supported {
            finding(
                findings,
                format!("$.events.{}", projection.event_id),
                "SUCCESS_VERIFIED projection is not causally supported by a recipient commit, recipient decryption verification, matching content commitment, and a separately attributed receipt verifier with receipt evidence",
            );
        }
    }
}

fn validate_run_and_scenario(
    root: &Map<String, Value>,
    findings: &mut Vec<Finding>,
) -> (String, String, String) {
    let run = object_field(root, "run", "$", findings);
    let (run_id, source_sha) = if let Some(run) = run {
        let run_id = string_field(run, "run_id", "$.run", findings).unwrap_or_default();
        let source_sha =
            string_field(run, "source_sha", "$.run", findings).unwrap_or_default();
        validate_digest(Some(&source_sha), 40, "$.run.source_sha", findings);
        for digest_key in ["toolchain_digest", "workload_digest"] {
            let digest = string_field(run, digest_key, "$.run", findings);
            validate_digest(
                digest.as_deref(),
                64,
                &format!("$.run.{digest_key}"),
                findings,
            );
        }
        for optional_key in [
            "source_lineage_digest",
            "cargo_lock_digest",
            "nix_lock_digest",
            "javascript_lock_digest",
            "dna_digest",
            "happ_digest",
            "topology_digest",
        ] {
            validate_optional_digest(run, optional_key, "$.run", findings);
        }
        if let Some(verifier) = object_field(run, "verifier", "$.run", findings) {
            let _name = string_field(verifier, "name", "$.run.verifier", findings);
            let _version = string_field(verifier, "version", "$.run.verifier", findings);
            let digest = string_field(
                verifier,
                "implementation_digest",
                "$.run.verifier",
                findings,
            );
            validate_digest(
                digest.as_deref(),
                64,
                "$.run.verifier.implementation_digest",
                findings,
            );
        }
        (run_id, source_sha)
    } else {
        (String::new(), String::new())
    };

    let scenario = object_field(root, "scenario", "$", findings);
    let scenario_id = if let Some(scenario) = scenario {
        let id = string_field(scenario, "scenario_id", "$.scenario", findings)
            .unwrap_or_default();
        if !id.starts_with("PFC-")
            || !id
                .bytes()
                .all(|byte| byte.is_ascii_uppercase() || byte.is_ascii_digit() || byte == b'-')
        {
            finding(
                findings,
                "$.scenario.scenario_id",
                "scenario_id must use the PFC- uppercase identifier format",
            );
        }
        let family = string_field(scenario, "family", "$.scenario", findings);
        let families = [
            "delivery",
            "authority",
            "crypto",
            "privacy",
            "recovery",
            "resilience",
            "evidence",
        ];
        if let Some(family) = family {
            if !families.contains(&family.as_str()) {
                finding(
                    findings,
                    "$.scenario.family",
                    format!("unknown scenario family: {family}"),
                );
            }
        }
        if !scenario.get("seed").and_then(Value::as_u64).is_some() {
            finding(
                findings,
                "$.scenario.seed",
                "scenario seed must be a non-negative integer",
            );
        }
        if let Some(vantage) = array_field(scenario, "observation_vantage", "$.scenario", findings)
        {
            if vantage.is_empty() {
                finding(
                    findings,
                    "$.scenario.observation_vantage",
                    "at least one observation vantage is required",
                );
            }
            let allowed = [
                "sender_client",
                "recipient_client",
                "authoring_conductor",
                "receiving_conductor",
                "dht_observer",
                "relay",
                "network_observer",
                "independent_verifier",
            ];
            for (index, item) in vantage.iter().enumerate() {
                match item.as_str() {
                    Some(value) if allowed.contains(&value) => {}
                    _ => finding(
                        findings,
                        format!("$.scenario.observation_vantage[{index}]"),
                        "unknown observation vantage",
                    ),
                }
            }
        }
        id
    } else {
        String::new()
    };

    (run_id, source_sha, scenario_id)
}

/// Validate an instance and its schema with the pinned Draft 2020-12 engine.
///
/// Remote reference resolution is disabled by the crate configuration. The
/// qualification schema must therefore be self-contained and deterministic.
pub fn validate_against_schema(
    instance: &Value,
    schema: &Value,
) -> Result<(), Vec<String>> {
    const DRAFT_2020_12: &str = "https://json-schema.org/draft/2020-12/schema";
    if schema.get("$schema").and_then(Value::as_str) != Some(DRAFT_2020_12) {
        return Err(vec![
            "schema must declare JSON Schema Draft 2020-12 explicitly".to_owned(),
        ]);
    }

    jsonschema::draft202012::meta::validate(schema)
        .map_err(|error| vec![format!("invalid Draft 2020-12 schema: {error}")])?;

    let validator = jsonschema::draft202012::options()
        .should_validate_formats(true)
        .build(schema)
        .map_err(|error| vec![format!("could not compile Draft 2020-12 schema: {error}")])?;

    let errors: Vec<String> = validator
        .iter_errors(instance)
        .map(|error| format!("{}: {error}", error.instance_path()))
        .collect();

    if errors.is_empty() {
        Ok(())
    } else {
        Err(errors)
    }
}

/// Verify one redacted Pulse Fabric trace.
///
/// A successful return means only that the checked structure and causal claims
/// are internally consistent under this verifier's v0.1 rules. It does not
/// establish cryptographic validity, host authorization, DHT truth, delivery
/// outside the supplied evidence, or full JSON Schema Draft 2020-12 validity.
pub fn verify_trace(
    root_value: &Value,
    expected_source_sha: Option<&str>,
) -> Result<VerificationSummary, Vec<Finding>> {
    let mut findings = Vec::new();
    let root = match root_value.as_object() {
        Some(root) => root,
        None => {
            return Err(vec![Finding {
                path: "$".to_owned(),
                message: "trace root must be a JSON object".to_owned(),
            }]);
        }
    };

    scan_forbidden_keys(root_value, "$", &mut findings);

    let schema_version = string_field(root, "schema_version", "$", &mut findings);
    if schema_version.as_deref() != Some(SCHEMA_VERSION) {
        finding(
            &mut findings,
            "$.schema_version",
            format!("expected schema version {SCHEMA_VERSION}"),
        );
    }
    let profile_id = string_field(root, "profile_id", "$", &mut findings);
    if profile_id.as_deref() != Some(PROFILE_ID) {
        finding(
            &mut findings,
            "$.profile_id",
            format!("expected profile id {PROFILE_ID}"),
        );
    }

    let (run_id, source_sha, scenario_id) =
        validate_run_and_scenario(root, &mut findings);

    if let Some(expected) = expected_source_sha {
        if !is_lower_hex(expected, 40) {
            finding(
                &mut findings,
                "$.expected_source_sha",
                "expected source SHA must be 40 lowercase hexadecimal characters",
            );
        } else if source_sha != expected {
            finding(
                &mut findings,
                "$.run.source_sha",
                "trace source_sha does not match the externally supplied expected source SHA",
            );
        }
    }

    let terminal_disposition =
        string_field(root, "terminal_disposition", "$", &mut findings)
            .unwrap_or_default();
    if !TERMINAL_DISPOSITIONS.contains(&terminal_disposition.as_str()) {
        finding(
            &mut findings,
            "$.terminal_disposition",
            format!("unknown terminal disposition: {terminal_disposition}"),
        );
    }

    match root.get("claim_ceiling").and_then(Value::as_array) {
        Some(items) if !items.is_empty() => {
            for (index, item) in items.iter().enumerate() {
                if !item.as_str().is_some_and(|value| !value.trim().is_empty()) {
                    finding(
                        &mut findings,
                        format!("$.claim_ceiling[{index}]"),
                        "claim ceiling entries must be non-empty strings",
                    );
                }
            }
        }
        _ => finding(
            &mut findings,
            "$.claim_ceiling",
            "claim_ceiling must be a non-empty array",
        ),
    }

    let empty_events: Vec<Value> = Vec::new();
    let event_values = match array_field(root, "events", "$", &mut findings) {
        Some(items) if !items.is_empty() => items,
        Some(_) => {
            finding(
                &mut findings,
                "$.events",
                "at least one event is required",
            );
            &empty_events
        }
        None => &empty_events,
    };

    let mut events = Vec::new();
    for (index, event_value) in event_values.iter().enumerate() {
        if let Some(event) = parse_event(event_value, index, &mut findings) {
            events.push(event);
        }
    }

    let mut by_id = HashMap::new();
    let mut operations = BTreeSet::new();
    let mut attempt_operations: HashMap<String, String> = HashMap::new();
    let mut commit_counts: HashMap<(String, String), usize> = HashMap::new();

    for (index, event) in events.iter().enumerate() {
        if by_id.insert(event.event_id.clone(), index).is_some() {
            finding(
                &mut findings,
                format!("$.events[{index}].event_id"),
                format!("duplicate event_id: {}", event.event_id),
            );
        }
        operations.insert(event.logical_operation_id.clone());

        if let Some(attempt_id) = &event.attempt_id {
            if let Some(previous_operation) = attempt_operations.get(attempt_id) {
                if previous_operation != &event.logical_operation_id {
                    finding(
                        &mut findings,
                        format!("$.events[{index}].attempt_id"),
                        "attempt_id is reused across different logical operations",
                    );
                }
            } else {
                attempt_operations.insert(
                    attempt_id.clone(),
                    event.logical_operation_id.clone(),
                );
            }
        }

        if event.event_type == "recipient_inbox_committed" {
            *commit_counts
                .entry((
                    event.logical_operation_id.clone(),
                    event.actor_ref.clone(),
                ))
                .or_insert(0) += 1;
        }
    }

    for ((operation_id, recipient_actor), count) in commit_counts {
        if count > 1 {
            finding(
                &mut findings,
                "$.events",
                format!(
                    "logical operation {operation_id} has {count} recipient inbox commits for recipient {recipient_actor}; duplicate logical effects are not permitted"
                ),
            );
        }
    }

    verify_acyclic_graph(&events, &by_id, &mut findings);

    if terminal_disposition == "SUCCESS_VERIFIED" {
        verify_success_projection(&events, &by_id, &mut findings);
    }

    if findings.is_empty() {
        Ok(VerificationSummary {
            status: "EVIDENCE_CONSISTENT",
            run_id,
            source_sha,
            scenario_id,
            terminal_disposition,
            event_count: events.len(),
            operation_count: operations.len(),
            causal_edge_count: events
                .iter()
                .map(|event| event.predecessors.len())
                .sum(),
        })
    } else {
        Err(findings)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::{Value, json};

    fn event(
        id: &str,
        event_type: &str,
        role: &str,
        actor: &str,
        attempt: Option<&str>,
        commitment: Option<&str>,
        monotonic_ns: u64,
        predecessors: Vec<(&str, &str)>,
    ) -> Value {
        json!({
            "event_id": id,
            "logical_operation_id": "logical-1",
            "attempt_id": attempt,
            "event_type": event_type,
            "evidence_role": role,
            "actor_ref": actor,
            "content_commitment": commitment,
            "observed_at": {
                "clock_domain": "test-clock",
                "monotonic_ns": monotonic_ns
            },
            "causal_predecessors": predecessors.into_iter().map(|(event_id, relation)| {
                json!({"event_id": event_id, "relation": relation})
            }).collect::<Vec<_>>(),
            "evidence_refs": if event_type == "receipt_verified" {
                vec![json!({
                    "kind": "recipient_receipt",
                    "sha256": "f".repeat(64),
                    "scope": "recipient-commitment"
                })]
            } else {
                Vec::<Value>::new()
            },
            "redaction_profile": "pfc-redacted-v0"
        })
    }

    fn valid_trace() -> Value {
        let commitment = "a".repeat(64);
        json!({
            "schema_version": "0.1.0",
            "profile_id": "PULSE-FABRIC-QUAL-001",
            "run": {
                "run_id": "run-1",
                "source_sha": "1".repeat(40),
                "toolchain_digest": "2".repeat(64),
                "workload_digest": "3".repeat(64),
                "topology_digest": "4".repeat(64),
                "verifier": {
                    "name": "pulse-fabric-verifier",
                    "version": "0.1.0",
                    "implementation_digest": "5".repeat(64)
                }
            },
            "scenario": {
                "scenario_id": "PFC-DELIVERY-001",
                "family": "delivery",
                "seed": 7,
                "observation_vantage": ["recipient_client", "independent_verifier"]
            },
            "events": [
                event("intent", "intent_recorded", "client", "sender", None, None, 1, vec![]),
                event(
                    "admitted",
                    "recipient_admitted",
                    "recipient",
                    "recipient",
                    Some("attempt-1"),
                    None,
                    2,
                    vec![("intent", "request_response")]
                ),
                event(
                    "commit",
                    "recipient_inbox_committed",
                    "recipient",
                    "recipient",
                    Some("attempt-1"),
                    Some(&commitment),
                    3,
                    vec![("admitted", "commits")]
                ),
                event(
                    "decrypt",
                    "recipient_decryption_verified",
                    "recipient",
                    "recipient",
                    Some("attempt-1"),
                    Some(&commitment),
                    4,
                    vec![("commit", "verifies")]
                ),
                event(
                    "receipt-created",
                    "receipt_created",
                    "recipient",
                    "recipient",
                    Some("attempt-1"),
                    Some(&commitment),
                    5,
                    vec![("decrypt", "receipt_for")]
                ),
                event(
                    "receipt-verified",
                    "receipt_verified",
                    "receipt_verifier",
                    "independent-verifier",
                    Some("attempt-1"),
                    Some(&commitment),
                    6,
                    vec![("receipt-created", "verifies")]
                ),
                event(
                    "projection",
                    "ui_projection_emitted",
                    "ui_projection",
                    "sender-ui",
                    None,
                    None,
                    7,
                    vec![("receipt-verified", "derived_from")]
                )
            ],
            "terminal_disposition": "SUCCESS_VERIFIED",
            "claim_ceiling": [
                "synthetic trace consistency only",
                "no cryptographic or runtime qualification implied"
            ]
        })
    }

    fn trace_schema() -> Value {
        serde_json::from_str(include_str!("../../../docs/pulse-fabric-trace.schema-v0.1.json"))
            .expect("checked-in trace schema must parse")
    }

    #[test]
    fn embedded_trace_schema_is_valid_draft_2020_12_and_accepts_valid_trace() {
        let schema = trace_schema();
        let trace = valid_trace();
        validate_against_schema(&trace, &schema).expect("trace must satisfy the normative schema");
    }

    #[test]
    fn normative_schema_rejects_missing_required_evidence_field() {
        let schema = trace_schema();
        let mut trace = valid_trace();
        trace["events"][4].as_object_mut().unwrap().remove("evidence_refs");
        let errors = validate_against_schema(&trace, &schema).unwrap_err();
        assert!(errors.iter().any(|item| item.contains("evidence_refs")));
    }

    #[test]
    fn accepts_causally_supported_verified_lifecycle() {
        let trace = valid_trace();
        let result = verify_trace(&trace, Some(&"1".repeat(40)));
        assert!(result.is_ok(), "{result:?}");
        assert_eq!(
            result.unwrap().terminal_disposition,
            "SUCCESS_VERIFIED"
        );
    }

    #[test]
    fn rejects_source_sha_mismatch() {
        let trace = valid_trace();
        let result = verify_trace(&trace, Some(&"9".repeat(40)));
        assert!(result.is_err());
        assert!(result.unwrap_err().iter().any(|item| {
            item.message.contains("externally supplied expected source SHA")
        }));
    }

    #[test]
    fn transport_ack_alone_cannot_support_success() {
        let mut trace = valid_trace();
        trace["events"] = json!([
            event("intent", "intent_recorded", "client", "sender", None, None, 1, vec![]),
            event(
                "ack",
                "transport_ack_observed",
                "transport_adapter",
                "relay",
                Some("attempt-1"),
                None,
                2,
                vec![("intent", "transport_for")]
            ),
            event(
                "projection",
                "ui_projection_emitted",
                "ui_projection",
                "sender-ui",
                None,
                None,
                3,
                vec![("ack", "derived_from")]
            )
        ]);
        let result = verify_trace(&trace, None);
        assert!(result.is_err());
    }

    #[test]
    fn accepts_ack_only_trace_when_disposition_remains_unknown() {
        let mut trace = valid_trace();
        trace["events"] = json!([
            event("intent", "intent_recorded", "client", "sender", None, None, 1, vec![]),
            event(
                "ack",
                "transport_ack_observed",
                "transport_adapter",
                "relay",
                Some("attempt-1"),
                None,
                2,
                vec![("intent", "transport_for")]
            )
        ]);
        trace["terminal_disposition"] = json!("UNKNOWN");
        let result = verify_trace(&trace, None);
        assert!(result.is_ok(), "{result:?}");
        assert_eq!(result.unwrap().terminal_disposition, "UNKNOWN");
    }

    #[test]
    fn rejects_dangling_causal_edge() {
        let mut trace = valid_trace();
        trace["events"][2]["causal_predecessors"] =
            json!([{"event_id": "missing", "relation": "request_response"}]);
        let result = verify_trace(&trace, None);
        assert!(result.is_err());
        assert!(result.unwrap_err().iter().any(|item| {
            item.message.contains("does not resolve")
        }));
    }

    #[test]
    fn rejects_causal_cycle() {
        let mut trace = valid_trace();
        trace["events"][0]["causal_predecessors"] =
            json!([{"event_id": "projection", "relation": "derived_from"}]);
        let result = verify_trace(&trace, None);
        assert!(result.is_err());
        assert!(result.unwrap_err().iter().any(|item| {
            item.message.contains("contains a cycle")
        }));
    }

    #[test]
    fn rejects_event_role_confusion() {
        let mut trace = valid_trace();
        trace["events"][2]["evidence_role"] = json!("transport_adapter");
        let result = verify_trace(&trace, None);
        assert!(result.is_err());
        assert!(result.unwrap_err().iter().any(|item| {
            item.message.contains("requires one of these evidence roles")
        }));
    }

    #[test]
    fn rejects_missing_attempt_id_for_receipt_verification() {
        let mut trace = valid_trace();
        trace["events"][5]["attempt_id"] = Value::Null;
        let result = verify_trace(&trace, None);
        assert!(result.is_err());
        assert!(result.unwrap_err().iter().any(|item| {
            item.message.contains("requires a concrete attempt_id")
        }));
    }

    #[test]
    fn rejects_duplicate_logical_inbox_effect() {
        let mut trace = valid_trace();
        let duplicate = trace["events"][2].clone();
        trace["events"].as_array_mut().unwrap().insert(3, duplicate);
        let result = verify_trace(&trace, None);
        assert!(result.is_err());
        assert!(result.unwrap_err().iter().any(|item| {
            item.message.contains("duplicate logical effects")
        }));
    }

    #[test]
    fn rejects_embedded_secret_fields() {
        let mut trace = valid_trace();
        trace["events"][0]["private_key"] = json!("must-not-appear");
        let result = verify_trace(&trace, None);
        assert!(result.is_err());
        assert!(result.unwrap_err().iter().any(|item| {
            item.message.contains("prohibited in a redacted evidence trace")
        }));
    }

    #[test]
    fn rejects_success_when_receipt_verifier_is_recipient() {
        let mut trace = valid_trace();
        trace["events"][5]["actor_ref"] = json!("recipient");
        let result = verify_trace(&trace, None);
        assert!(result.is_err());
    }
    #[test]
    fn rejects_success_when_decryption_commitment_differs() {
        let mut trace = valid_trace();
        trace["events"][3]["content_commitment"] = json!("b".repeat(64));
        let result = verify_trace(&trace, None);
        assert!(result.is_err());
        assert!(result.unwrap_err().iter().any(|item| {
            item.message.contains("not causally supported")
        }));
    }

    #[test]
    fn rejects_success_when_recipient_identity_changes_mid_lifecycle() {
        let mut trace = valid_trace();
        trace["events"][3]["actor_ref"] = json!("different-recipient");
        let result = verify_trace(&trace, None);
        assert!(result.is_err());
    }

    #[test]
    fn rejects_success_without_recipient_admission() {
        let mut trace = valid_trace();
        trace["events"].as_array_mut().unwrap().remove(1);
        trace["events"][1]["causal_predecessors"] =
            json!([{"event_id": "intent", "relation": "verifies"}]);
        let result = verify_trace(&trace, None);
        assert!(result.is_err());
    }

    #[test]
    fn rejects_causal_predecessor_after_child_on_same_clock() {
        let mut trace = valid_trace();
        trace["events"][2]["observed_at"]["monotonic_ns"] = json!(0);
        let result = verify_trace(&trace, None);
        assert!(result.is_err());
        assert!(result.unwrap_err().iter().any(|item| {
            item.message.contains("later monotonic timestamp")
        }));
    }

    #[test]
    fn rejects_attempt_id_reuse_across_logical_operations() {
        let mut trace = valid_trace();
        trace["events"][2]["logical_operation_id"] = json!("logical-2");
        let result = verify_trace(&trace, None);
        assert!(result.is_err());
        assert!(result.unwrap_err().iter().any(|item| {
            item.message.contains("reused across different logical operations")
        }));
    }

    fn checked_in_profile_and_schema_satisfy_draft_2020_12() {
        let profile: Value = serde_json::from_str(include_str!(
            "../../../docs/pulse-fabric-profile-v0.1.json"
        ))
        .expect("profile must parse");
        let schema: Value = serde_json::from_str(include_str!(
            "../../../docs/pulse-fabric-profile.schema-v0.1.json"
        ))
        .expect("profile schema must parse");
        validate_against_schema(&profile, &schema)
            .expect("profile must satisfy its Draft 2020-12 schema");
    }

    #[test]
    fn checked_in_scenario_catalog_satisfies_draft_2020_12() {
        let catalog: Value = serde_json::from_str(include_str!(
            "../../../docs/pulse-fabric-scenarios-v0.1.json"
        ))
        .expect("scenario catalog must parse");
        let schema: Value = serde_json::from_str(include_str!(
            "../../../docs/pulse-fabric-scenarios.schema-v0.1.json"
        ))
        .expect("scenario schema must parse");
        validate_against_schema(&catalog, &schema)
            .expect("scenario catalog must satisfy its Draft 2020-12 schema");
    }

    #[test]
    #[test]
    fn checked_in_scenario_catalog_has_unique_ids_and_all_families() {
        let catalog: Value = serde_json::from_str(include_str!(
            "../../../docs/pulse-fabric-scenarios-v0.1.json"
        ))
        .expect("scenario catalog must parse");

        assert_eq!(
            catalog["profile_id"].as_str(),
            Some("PULSE-FABRIC-QUAL-001")
        );
        let scenarios = catalog["scenarios"].as_array().expect("scenarios array");
        assert_eq!(scenarios.len(), 18);

        let mut ids = BTreeSet::new();
        let mut families = BTreeSet::new();
        for scenario in scenarios {
            let id = scenario["scenario_id"].as_str().expect("scenario id");
            assert!(ids.insert(id.to_owned()), "duplicate scenario id: {id}");
            families.insert(
                scenario["family"]
                    .as_str()
                    .expect("scenario family")
                    .to_owned(),
            );
            assert!(
                scenario["required_invariants"]
                    .as_array()
                    .is_some_and(|items| !items.is_empty()),
                "scenario {id} requires invariants"
            );
        }
        assert_eq!(families.len(), 7);
    }

    #[test]
    fn rejects_secret_aliases_in_trace_extensions() {
        let mut trace = valid_trace();
        trace["events"][0]["message_body"] = json!("plain text");
        let result = verify_trace(&trace, None);
        assert!(result.is_err());
    }
}
