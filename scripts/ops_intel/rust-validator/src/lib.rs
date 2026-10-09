//! Strict, dependency-light fixture package preflight for CrossDomainDisruptionV1.
//!
//! This crate validates only the public synthetic fixture package. It does not
//! implement operational projection semantics, canonical wire encoding,
//! intelligence reasoning, authority, or the independent oracle tracked by #2950.

use serde::de::{self, Deserialize, Deserializer, MapAccess, SeqAccess, Visitor};
use serde_json::{Map, Number, Value};
use std::collections::{BTreeMap, BTreeSet};
use std::fmt;
use std::fs;
use std::path::Path;

pub const FIXTURE_FILES: [(&str, &str); 6] = [
    ("f0", "CROSS_DOMAIN_DISRUPTION_V1_F0_SOLVER_VISIBLE.json"),
    ("f1", "CROSS_DOMAIN_DISRUPTION_V1_F1_DELTA.json"),
    ("f2", "CROSS_DOMAIN_DISRUPTION_V1_F2_DELTA.json"),
    ("f3", "CROSS_DOMAIN_DISRUPTION_V1_F3_DELTA.json"),
    ("predicates", "CROSS_DOMAIN_DISRUPTION_V1_EXPECTED_PREDICATES.json"),
    ("mutations", "CROSS_DOMAIN_DISRUPTION_V1_MUTATIONS.json"),
];

const EXPECTED_PREDICATES: usize = 20;
const EXPECTED_MUTATIONS: usize = 23;
const MAX_FILE_BYTES: usize = 1024 * 1024;
const MAX_TOTAL_BYTES: usize = 4 * 1024 * 1024;
const MAX_STRING_BYTES: usize = 64 * 1024;
const MAX_COLLECTION_ITEMS: usize = 10_000;
const MAX_JSON_NODES: usize = 100_000;
const MAX_JSON_DEPTH: usize = 64;

const REQUIRED_MOCK_PERMIT_DISPOSITIONS: [&str; 5] = [
    "BlockedNoCurrentPermit",
    "RejectStalePermit",
    "RejectWrongSubject",
    "RejectWrongCandidate",
    "RejectWrongPayload",
];

const FORBIDDEN_EVALUATOR_KEYS: [&str; 4] = [
    "evaluator_only_world",
    "evaluator_truth_payload",
    "hidden_oracle_values",
    "actual_world_truth_payload",
];

#[derive(Debug)]
struct StrictJson(Value);

impl<'de> Deserialize<'de> for StrictJson {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        struct StrictVisitor;

        impl<'de> Visitor<'de> for StrictVisitor {
            type Value = StrictJson;

            fn expecting(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
                formatter.write_str("a JSON value with no duplicate object keys")
            }

            fn visit_bool<E>(self, value: bool) -> Result<Self::Value, E> {
                Ok(StrictJson(Value::Bool(value)))
            }

            fn visit_i64<E>(self, value: i64) -> Result<Self::Value, E> {
                Ok(StrictJson(Value::Number(Number::from(value))))
            }

            fn visit_u64<E>(self, value: u64) -> Result<Self::Value, E> {
                Ok(StrictJson(Value::Number(Number::from(value))))
            }

            fn visit_f64<E>(self, value: f64) -> Result<Self::Value, E>
            where
                E: de::Error,
            {
                Number::from_f64(value)
                    .map(Value::Number)
                    .map(StrictJson)
                    .ok_or_else(|| E::custom("non-finite JSON number"))
            }

            fn visit_str<E>(self, value: &str) -> Result<Self::Value, E> {
                Ok(StrictJson(Value::String(value.to_owned())))
            }

            fn visit_string<E>(self, value: String) -> Result<Self::Value, E> {
                Ok(StrictJson(Value::String(value)))
            }

            fn visit_unit<E>(self) -> Result<Self::Value, E> {
                Ok(StrictJson(Value::Null))
            }

            fn visit_none<E>(self) -> Result<Self::Value, E> {
                Ok(StrictJson(Value::Null))
            }

            fn visit_seq<A>(self, mut sequence: A) -> Result<Self::Value, A::Error>
            where
                A: SeqAccess<'de>,
            {
                let mut values = Vec::new();
                while let Some(StrictJson(value)) = sequence.next_element::<StrictJson>()? {
                    if values.len() >= MAX_COLLECTION_ITEMS {
                        return Err(de::Error::custom("JSON array exceeds item limit"));
                    }
                    values.push(value);
                }
                Ok(StrictJson(Value::Array(values)))
            }

            fn visit_map<A>(self, mut mapping: A) -> Result<Self::Value, A::Error>
            where
                A: MapAccess<'de>,
            {
                let mut values = Map::new();
                while let Some(key) = mapping.next_key::<String>()? {
                    if values.contains_key(&key) {
                        return Err(de::Error::custom("duplicate JSON object key"));
                    }
                    if values.len() >= MAX_COLLECTION_ITEMS {
                        return Err(de::Error::custom("JSON object exceeds member limit"));
                    }
                    let StrictJson(value) = mapping.next_value::<StrictJson>()?;
                    values.insert(key, value);
                }
                Ok(StrictJson(Value::Object(values)))
            }
        }

        deserializer.deserialize_any(StrictVisitor)
    }
}

/// Parse strict JSON while rejecting duplicate object keys at every depth.
/// The parser is deliberately separate from the semantic fixture checks.
pub fn parse_strict_json(bytes: &[u8]) -> Result<Value, String> {
    if bytes.len() > MAX_FILE_BYTES {
        return Err("input exceeds per-file byte limit".to_owned());
    }
    let mut deserializer = serde_json::Deserializer::from_slice(bytes);
    let StrictJson(value) = StrictJson::deserialize(&mut deserializer)
        .map_err(|_| "strict JSON parse rejected malformed, duplicate-key, or over-limit input".to_owned())?;
    deserializer
        .end()
        .map_err(|_| "strict JSON parse rejected trailing bytes".to_owned())?;
    let mut node_count = 0usize;
    check_resource_limits(&value, "$", 0, &mut node_count)?;
    Ok(value)
}

fn check_resource_limits(
    value: &Value,
    path: &str,
    depth: usize,
    node_count: &mut usize,
) -> Result<(), String> {
    *node_count = node_count.saturating_add(1);
    if *node_count > MAX_JSON_NODES {
        return Err(format!("{path}: JSON node limit exceeded"));
    }
    if depth > MAX_JSON_DEPTH {
        return Err(format!("{path}: JSON nesting limit exceeded"));
    }
    match value {
        Value::String(s) if s.len() > MAX_STRING_BYTES => {
            return Err(format!("{path}: string byte limit exceeded"));
        }
        Value::Array(values) => {
            if values.len() > MAX_COLLECTION_ITEMS {
                return Err(format!("{path}: array item limit exceeded"));
            }
            for (index, child) in values.iter().enumerate() {
                check_resource_limits(child, &format!("{path}[{index}]"), depth + 1, node_count)?;
            }
        }
        Value::Object(values) => {
            if values.len() > MAX_COLLECTION_ITEMS {
                return Err(format!("{path}: object member limit exceeded"));
            }
            for (key, child) in values {
                if key.len() > MAX_STRING_BYTES {
                    return Err(format!("{path}: object key byte limit exceeded"));
                }
                check_resource_limits(child, &format!("{path}.{key}"), depth + 1, node_count)?;
            }
        }
        _ => {}
    }
    Ok(())
}

/// Load all six files with byte limits and duplicate-key rejection.
pub fn load_fixture_documents(directory: &Path) -> Result<BTreeMap<String, Value>, Vec<String>> {
    let mut errors = Vec::new();
    let mut docs = BTreeMap::new();
    let mut total_bytes = 0usize;

    for (key, filename) in FIXTURE_FILES {
        let path = directory.join(filename);
        let bytes = match fs::read(&path) {
            Ok(bytes) => bytes,
            Err(_) => {
                errors.push(format!("{filename}: required fixture file could not be read"));
                continue;
            }
        };
        total_bytes = total_bytes.saturating_add(bytes.len());
        if bytes.len() > MAX_FILE_BYTES {
            errors.push(format!("{filename}: input exceeds per-file byte limit"));
            continue;
        }
        match parse_strict_json(&bytes) {
            Ok(value) => {
                if !value.is_object() {
                    errors.push(format!("{filename}: root must be a JSON object"));
                } else {
                    docs.insert(key.to_owned(), value);
                }
            }
            Err(reason) => errors.push(format!("{filename}: {reason}")),
        }
    }

    if total_bytes > MAX_TOTAL_BYTES {
        errors.push("fixture package exceeds total byte limit".to_owned());
    }
    if !errors.is_empty() {
        return Err(errors);
    }
    Ok(docs)
}

/// Validate already-parsed documents; useful for deterministic mutation tests.
pub fn validate_documents(docs: &BTreeMap<String, Value>) -> Vec<String> {
    let mut errors = Vec::new();
    for (key, _) in FIXTURE_FILES {
        if !docs.contains_key(key) {
            errors.push(format!("fixture package is missing document {key}"));
        }
    }
    if !errors.is_empty() {
        return errors;
    }

    let f0 = &docs["f0"];
    let f1 = &docs["f1"];
    let f2 = &docs["f2"];
    let f3 = &docs["f3"];
    let predicates = &docs["predicates"];
    let mutations = &docs["mutations"];

    for (label, value) in [("F0", f0), ("F1", f1), ("F2", f2), ("F3", f3)] {
        let profile = value.get("fixture_profile").or_else(|| value.get("profile"));
        if profile.and_then(Value::as_str) != Some("OPS-INTEL-TEST-001") {
            errors.push(format!("{label}: unexpected fixture profile"));
        }
        if value.get("fixture_version").and_then(Value::as_str) != Some("CrossDomainDisruptionV1") {
            errors.push(format!("{label}: wrong fixture version"));
        }
        if value.get("data_class").and_then(Value::as_str) != Some("synthetic") {
            errors.push(format!("{label}: data_class must be synthetic"));
        }
    }

    for (key, value) in docs {
        scan_forbidden_keys(value, key, &mut errors);
        let mut nodes = 0;
        if let Err(error) = check_resource_limits(value, key, 0, &mut nodes) {
            errors.push(error);
        }
    }

    if f0.get("visibility").and_then(Value::as_str) != Some("solver-visible-only") {
        errors.push("F0: must explicitly be solver-visible-only".to_owned());
    }
    if f0.get("authority_ceiling").and_then(Value::as_str) != Some("FixtureDescriptionOnly") {
        errors.push("F0: unexpected authority ceiling".to_owned());
    }

    let sources0 = unique_record_refs(f0.get("source_registry"), "source_ref", "F0 source_registry", &mut errors);
    let artifacts0 = unique_record_refs(f0.get("artifacts"), "artifact_ref", "F0 artifacts", &mut errors);
    let subjects0 = unique_record_refs(f0.get("subjects"), "subject_ref", "F0 subjects", &mut errors);
    check_record_refs(f0.get("artifacts"), "source_ref", &sources0, "F0 artifacts", &mut errors);
    check_record_refs(f0.get("observations"), "source_ref", &sources0, "F0 observations", &mut errors);
    check_record_refs(f0.get("observations"), "artifact_ref", &artifacts0, "F0 observations", &mut errors);
    check_record_refs(f0.get("observations"), "subject_ref", &subjects0, "F0 observations", &mut errors);
    let observations0 = array_field(f0, "observations", "F0 observations", &mut errors);
    let observation_ids = record_refs_from_slice(observations0, "observation_ref", "F0 observations", &mut errors);
    let facility_refs = validate_facility_registry(f0, &subjects0, &mut errors);

    let inventory0 = find_record(observations0, "observation_ref", "observation:supplier-B-inventory-partial");
    match inventory0.and_then(|entry| entry.get("coverage")) {
        Some(coverage) if coverage.is_object() => {
            let observed = string_array(coverage.get("facility_refs_observed"), "F0 inventory facility_refs_observed", &mut errors);
            let known = string_array(coverage.get("facility_refs_known"), "F0 inventory facility_refs_known", &mut errors);
            require_set_eq(&observed, &facility_refs[..3], "F0: inventory observation must cover exactly facilities A/B/C", &mut errors);
            require_set_eq(&known, &facility_refs, "F0: inventory observation must name all five known facility refs", &mut errors);
            if coverage.get("state").and_then(Value::as_str) != Some("PartialCoverage") {
                errors.push("F0: partial facility coverage must remain explicit".to_owned());
            }
            if inventory0.and_then(|item| item.get("aggregate_semantics")).and_then(Value::as_str)
                != Some("SumWithinListedFacilitiesAtObservationTime")
            {
                errors.push("F0: inventory scope aggregation semantics missing".to_owned());
            }
        }
        Some(_) => errors.push("F0: inventory observation coverage must be an object".to_owned()),
        None => errors.push("F0: supplier-B partial inventory observation or coverage is missing".to_owned()),
    }

    let coverage_assertions = array_field(f0, "coverage_assertions", "F0 coverage_assertions", &mut errors);
    match find_record(coverage_assertions, "coverage_ref", "coverage:supplier-B-unobserved-facilities") {
        Some(assertion) => {
            let observed = string_array(assertion.get("facility_refs_observed"), "F0 coverage assertion observed refs", &mut errors);
            let known = string_array(assertion.get("facility_refs_known"), "F0 coverage assertion known refs", &mut errors);
            require_set_eq(&observed, &facility_refs[..3], "F0: coverage assertion scope must name A/B/C", &mut errors);
            require_set_eq(&known, &facility_refs, "F0: coverage assertion must name A-E", &mut errors);
        }
        None => errors.push("F0: explicit unobserved-facility coverage assertion is missing".to_owned()),
    }

    let protected_fields = array_field(f0, "protected_fields", "F0 protected_fields", &mut errors);
    if protected_fields.is_empty() {
        errors.push("F0: protected field boundary must be represented".to_owned());
    }
    for (index, field) in protected_fields.iter().enumerate() {
        if field.get("handling_state").and_then(Value::as_str) != Some("OmittedUnderPolicy") {
            errors.push(format!("F0 protected field[{index}]: omission state is missing"));
        }
        if field.get("payload_included").and_then(Value::as_bool) != Some(false) {
            errors.push(format!("F0 protected field[{index}]: payload must be explicitly absent"));
        }
    }

    let candidates = array_field(f0, "candidate_interventions", "F0 candidate_interventions", &mut errors);
    if candidates.is_empty() {
        errors.push("F0: candidate intervention inventory must not be empty".to_owned());
    }
    for (index, candidate) in candidates.iter().enumerate() {
        if candidate.get("authority_ceiling").and_then(Value::as_str) != Some("CandidateOnly") {
            errors.push(format!("F0 candidate[{index}]: authority ceiling must be CandidateOnly"));
        }
        if candidate.get("execution_material_present").and_then(Value::as_bool) != Some(false) {
            errors.push(format!("F0 candidate[{index}]: execution material must be explicitly absent"));
        }
    }

    let f0_frontier = match f0.get("frontier").and_then(Value::as_object) {
        Some(frontier) => {
            if frontier.get("frontier_ref").and_then(Value::as_str) != Some("frontier:F0") {
                errors.push("F0: frontier ref must be frontier:F0".to_owned());
            }
            if !frontier.get("parent_frontier_ref").is_some_and(Value::is_null) {
                errors.push("F0: parent frontier must be explicitly null".to_owned());
            }
            frontier.get("cutoff_utc").and_then(Value::as_str).map(str::to_owned)
        }
        None => {
            errors.push("F0: frontier must be an object".to_owned());
            None
        }
    };

    let sources1_new = unique_record_refs(f1.get("added_sources"), "source_ref", "F1 added_sources", &mut errors);
    let artifacts1_new = unique_record_refs(f1.get("added_artifacts"), "artifact_ref", "F1 added_artifacts", &mut errors);
    let sources1: BTreeSet<String> = sources0.union(&sources1_new).cloned().collect();
    let artifacts1: BTreeSet<String> = artifacts0.union(&artifacts1_new).cloned().collect();
    check_record_refs(f1.get("added_artifacts"), "source_ref", &sources1, "F1 added_artifacts", &mut errors);
    check_record_refs(f1.get("added_observations"), "source_ref", &sources1, "F1 added_observations", &mut errors);
    check_record_refs(f1.get("added_observations"), "artifact_ref", &artifacts1, "F1 added_observations", &mut errors);
    if f1.get("parent_frontier_ref").and_then(Value::as_str) != Some("frontier:F0") {
        errors.push("F1: parent must be F0".to_owned());
    }
    if f1.get("frontier_ref").and_then(Value::as_str) != Some("frontier:F1") {
        errors.push("F1: frontier ref must be frontier:F1".to_owned());
    }

    let observations1 = array_field(f1, "added_observations", "F1 added_observations", &mut errors);
    match find_record(observations1, "observation_ref", "observation:supplier-B-inventory-refresh") {
        Some(inventory1) => {
            match inventory1.get("coverage").and_then(Value::as_object) {
                Some(coverage) => {
                    let observed = string_array(coverage.get("facility_refs_observed"), "F1 inventory facility_refs_observed", &mut errors);
                    require_set_eq(&observed, &facility_refs[3..], "F1: inventory refresh must cover exactly facilities D/E", &mut errors);
                }
                None => errors.push("F1: inventory refresh coverage must be an object".to_owned()),
            }
            if inventory1.get("scope_ref").and_then(Value::as_str) != Some("scope:supplier-B-facilities-D-E") {
                errors.push("F1: named facility scope ref is missing".to_owned());
            }
            if inventory1.get("aggregate_semantics").and_then(Value::as_str) != Some("SumWithinListedFacilitiesAtObservationTime") {
                errors.push("F1: inventory scope aggregation semantics missing".to_owned());
            }
            let has_limitation = inventory1.get("limitations").and_then(Value::as_array).is_some_and(|items| {
                items.iter().any(|item| item.as_str().is_some_and(|text| text.to_lowercase().contains("do not sum")))
            });
            if !has_limitation {
                errors.push("F1: must prohibit aggregation of different-time F0/F1 inventory values without reconciliation".to_owned());
            }
        }
        None => errors.push("F1: supplier-B inventory refresh is missing".to_owned()),
    }

    let assessments = array_field(f1, "added_coverage_assessments", "F1 added_coverage_assessments", &mut errors);
    if !assessments.iter().any(|item| item.get("aggregate_inference").and_then(Value::as_str)
        == Some("NotPermittedWithoutTemporalReconciliation"))
    {
        errors.push("F1: temporal reconciliation limit must be machine-readable".to_owned());
    }

    let f1_time = timestamp_field(f1, "frontier_cutoff_utc", "F1 cutoff", &mut errors);
    if !timestamp_field(f2, "frontier_cutoff_utc", "F2 cutoff", &mut errors) { /* errors were recorded */ }
    let f2_time = field_owned_text(f2, "frontier_cutoff_utc", "F2 cutoff", &mut errors);
    let f3_time = field_owned_text(f3, "frontier_cutoff_utc", "F3 cutoff", &mut errors);
    for (label, actual, expected, parent) in [
        ("F2", f2, "frontier:F2", "frontier:F1"),
        ("F3", f3, "frontier:F3", "frontier:F2"),
    ] {
        if actual.get("frontier_ref").and_then(Value::as_str) != Some(expected) {
            errors.push(format!("{label}: frontier ref must be {expected}"));
        }
        if actual.get("parent_frontier_ref").and_then(Value::as_str) != Some(parent) {
            errors.push(format!("{label}: parent must be {parent}"));
        }
    }

    let timestamps = [f0_frontier, f1_time, f2_time, f3_time];
    let parsed: Vec<Option<String>> = timestamps.iter().map(|value| {
        value.as_deref().filter(|text| valid_utc_timestamp(text)).map(str::to_owned)
    }).collect();
    let labels = ["F0 cutoff", "F1 cutoff", "F2 cutoff", "F3 cutoff"];
    for (index, time) in parsed.iter().enumerate() {
        if time.is_none() {
            errors.push(format!("{}: expected a valid fixed-width UTC timestamp", labels[index]));
        }
    }
    if let [Some(t0), Some(t1), Some(t2), Some(t3)] = parsed.as_slice() {
        if !(t0 < t1 && t1 < t2 && t2 < t3) {
            errors.push("Frontier cutoff timestamps must strictly increase F0 < F1 < F2 < F3".to_owned());
        }
    }

    validate_attempt_and_authority(f2, &mut errors);

    let sources3_new = unique_record_refs(f3.get("added_sources"), "source_ref", "F3 added_sources", &mut errors);
    let artifacts3_new = unique_record_refs(f3.get("added_artifacts"), "artifact_ref", "F3 added_artifacts", &mut errors);
    let sources3: BTreeSet<String> = sources1.union(&sources3_new).cloned().collect();
    let artifacts3: BTreeSet<String> = artifacts1.union(&artifacts3_new).cloned().collect();
    check_record_refs(f3.get("added_artifacts"), "source_ref", &sources3, "F3 added_artifacts", &mut errors);
    let outcomes = array_field(f3, "outcome_observations", "F3 outcome_observations", &mut errors);
    if outcomes.is_empty() {
        errors.push("F3: at least one outcome observation is required".to_owned());
    }
    check_record_refs(Some(&Value::Array(outcomes.to_vec())), "source_ref", &sources3, "F3 outcomes", &mut errors);
    check_record_refs(Some(&Value::Array(outcomes.to_vec())), "artifact_ref", &artifacts3, "F3 outcomes", &mut errors);
    for (index, outcome) in outcomes.iter().enumerate() {
        if outcome.get("causal_attribution").and_then(Value::as_str) != Some("NotEstablished") {
            errors.push(format!("F3 outcome[{index}]: causal attribution must remain NotEstablished"));
        }
    }

    let predicate_ids = unique_record_refs(predicates.get("predicates"), "id", "expected predicates", &mut errors);
    let mutation_ids = unique_record_refs(mutations.get("mutations"), "id", "mutations", &mut errors);
    if predicate_ids.len() != EXPECTED_PREDICATES {
        errors.push(format!("Expected exactly {EXPECTED_PREDICATES} frozen predicates"));
    }
    if mutation_ids.len() != EXPECTED_MUTATIONS {
        errors.push(format!("Expected exactly {EXPECTED_MUTATIONS} frozen mutations"));
    }
    if !observation_ids.contains("observation:supplier-B-inventory-partial") {
        errors.push("F0: supplier-B partial inventory observation id is missing".to_owned());
    }
    errors
}

fn validate_attempt_and_authority(f2: &Value, errors: &mut Vec<String>) {
    let attempt = match f2.get("attempt").and_then(Value::as_object) {
        Some(attempt) => attempt,
        None => {
            errors.push("F2: attempt record must be an object".to_owned());
            return;
        }
    };
    if attempt.get("execution_mode").and_then(Value::as_str) != Some("SimulationOnly") {
        errors.push("F2: attempt must be simulation-only".to_owned());
    }
    if attempt.get("actual_execution_authorized").and_then(Value::as_bool) != Some(false) {
        errors.push("F2: actual execution must be explicitly false".to_owned());
    }
    if attempt.get("live_capability_present").and_then(Value::as_bool) != Some(false) {
        errors.push("F2: live capability must be explicitly absent".to_owned());
    }
    if attempt.get("effect_state").and_then(Value::as_str) != Some("Unobserved") {
        errors.push("F2: effect must remain Unobserved before outcome evidence".to_owned());
    }

    let cases = array_field(f2, "authority_effect_cases", "F2 authority_effect_cases", errors);
    let actual: BTreeSet<String> = cases
        .iter()
        .filter_map(|case| case.get("expected_disposition").and_then(Value::as_str).map(str::to_owned))
        .collect();
    for disposition in REQUIRED_MOCK_PERMIT_DISPOSITIONS {
        if !actual.contains(disposition) {
            errors.push(format!("F2: required mock permit disposition {disposition} is missing"));
        }
    }
}

fn validate_facility_registry(
    f0: &Value,
    subjects: &BTreeSet<String>,
    errors: &mut Vec<String>,
) -> Vec<String> {
    let registry = match f0.get("facility_registry").and_then(Value::as_object) {
        Some(registry) => registry,
        None => {
            errors.push("F0: facility_registry must be an object".to_owned());
            return Vec::new();
        }
    };
    let refs = string_array(registry.get("facility_refs_known"), "F0 facility_registry.facility_refs_known", errors);
    let unique: BTreeSet<String> = refs.iter().cloned().collect();
    if refs.len() != 5 || unique.len() != 5 {
        errors.push("F0: expected five unique facility refs".to_owned());
    }
    for reference in &refs {
        if !subjects.contains(reference) {
            errors.push("F0: facility ref is not registered as a subject".to_owned());
        }
    }
    refs
}

fn scan_forbidden_keys(value: &Value, path: &str, errors: &mut Vec<String>) {
    match value {
        Value::Object(object) => {
            for (key, child) in object {
                if FORBIDDEN_EVALUATOR_KEYS.iter().any(|forbidden| key.eq_ignore_ascii_case(forbidden)) {
                    errors.push(format!("{path}: forbidden evaluator-only key present"));
                }
                scan_forbidden_keys(child, path, errors);
            }
        }
        Value::Array(values) => {
            for child in values {
                scan_forbidden_keys(child, path, errors);
            }
        }
        _ => {}
    }
}

fn array_field<'a>(
    value: &'a Value,
    field: &str,
    label: &str,
    errors: &mut Vec<String>,
) -> &'a [Value] {
    match value.get(field).and_then(Value::as_array) {
        Some(items) => {
            if items.len() > MAX_COLLECTION_ITEMS {
                errors.push(format!("{label}: array item limit exceeded"));
                &[]
            } else {
                items
            }
        }
        None => {
            errors.push(format!("{label}: expected an array"));
            &[]
        }
    }
}

fn unique_record_refs(
    value: Option<&Value>,
    field: &str,
    label: &str,
    errors: &mut Vec<String>,
) -> BTreeSet<String> {
    let items = match value.and_then(Value::as_array) {
        Some(items) if items.len() <= MAX_COLLECTION_ITEMS => items,
        Some(_) => {
            errors.push(format!("{label}: array item limit exceeded"));
            return BTreeSet::new();
        }
        None => {
            errors.push(format!("{label}: expected an array"));
            return BTreeSet::new();
        }
    };
    record_refs_from_slice(items, field, label, errors)
}

fn record_refs_from_slice(
    items: &[Value],
    field: &str,
    label: &str,
    errors: &mut Vec<String>,
) -> BTreeSet<String> {
    let mut result = BTreeSet::new();
    for (index, item) in items.iter().enumerate() {
        let Some(reference) = item.get(field).and_then(Value::as_str).filter(|s| !s.is_empty()) else {
            errors.push(format!("{label}[{index}]: missing non-empty string {field}"));
            continue;
        };
        if !result.insert(reference.to_owned()) {
            errors.push(format!("{label}: duplicate {field} values are forbidden"));
        }
    }
    result
}

fn check_record_refs(
    items: Option<&Value>,
    field: &str,
    allowed: &BTreeSet<String>,
    label: &str,
    errors: &mut Vec<String>,
) {
    let Some(items) = items.and_then(Value::as_array) else {
        errors.push(format!("{label}: expected an array"));
        return;
    };
    for (index, item) in items.iter().enumerate() {
        let Some(reference) = item.get(field).and_then(Value::as_str) else {
            errors.push(format!("{label}[{index}]: {field} must be a string reference"));
            continue;
        };
        if !allowed.contains(reference) {
            errors.push(format!("{label}[{index}]: {field} does not resolve"));
        }
    }
}

fn string_array(value: Option<&Value>, label: &str, errors: &mut Vec<String>) -> Vec<String> {
    let Some(items) = value.and_then(Value::as_array) else {
        errors.push(format!("{label}: expected an array of string refs"));
        return Vec::new();
    };
    if items.len() > MAX_COLLECTION_ITEMS {
        errors.push(format!("{label}: array item limit exceeded"));
        return Vec::new();
    }
    let mut out = Vec::with_capacity(items.len());
    let mut seen = BTreeSet::new();
    for (index, item) in items.iter().enumerate() {
        let Some(text) = item.as_str().filter(|text| !text.is_empty()) else {
            errors.push(format!("{label}[{index}]: expected a non-empty string ref"));
            continue;
        };
        if !seen.insert(text.to_owned()) {
            errors.push(format!("{label}: duplicate refs are forbidden"));
        }
        out.push(text.to_owned());
    }
    out
}

fn find_record<'a>(items: &'a [Value], field: &str, expected: &str) -> Option<&'a Value> {
    items.iter().find(|item| item.get(field).and_then(Value::as_str) == Some(expected))
}

fn require_set_eq(actual: &[String], expected: &[String], message: &str, errors: &mut Vec<String>) {
    let actual_set: BTreeSet<&str> = actual.iter().map(String::as_str).collect();
    let expected_set: BTreeSet<&str> = expected.iter().map(String::as_str).collect();
    if actual_set != expected_set || actual.len() != actual_set.len() {
        errors.push(message.to_owned());
    }
}

fn field_owned_text(value: &Value, field: &str, label: &str, errors: &mut Vec<String>) -> Option<String> {
    match value.get(field).and_then(Value::as_str) {
        Some(text) => Some(text.to_owned()),
        None => {
            errors.push(format!("{label}: expected a timestamp string"));
            None
        }
    }
}

fn timestamp_field(value: &Value, field: &str, label: &str, errors: &mut Vec<String>) -> bool {
    let Some(text) = field_owned_text(value, field, label, errors) else {
        return false;
    };
    if !valid_utc_timestamp(&text) {
        errors.push(format!("{label}: expected a valid fixed-width UTC timestamp"));
        return false;
    }
    true
}

fn valid_utc_timestamp(text: &str) -> bool {
    let bytes = text.as_bytes();
    if bytes.len() != 20
        || bytes[4] != b'-' || bytes[7] != b'-' || bytes[10] != b'T'
        || bytes[13] != b':' || bytes[16] != b':' || bytes[19] != b'Z'
    {
        return false;
    }
    for (index, byte) in bytes.iter().enumerate() {
        if !matches!(index, 4 | 7 | 10 | 13 | 16 | 19) && !byte.is_ascii_digit() {
            return false;
        }
    }
    let number = |start: usize, end: usize| std::str::from_utf8(&bytes[start..end]).ok()?.parse::<u32>().ok();
    let year = match number(0, 4) { Some(value) if value > 0 => value, _ => return false };
    let month = match number(5, 7) { Some(value @ 1..=12) => value, _ => return false };
    let day = match number(8, 10) { Some(value) if value > 0 => value, _ => return false };
    let hour = match number(11, 13) { Some(value @ 0..=23) => value, _ => return false };
    let minute = match number(14, 16) { Some(value @ 0..=59) => value, _ => return false };
    let second = match number(17, 19) { Some(value @ 0..=59) => value, _ => return false };
    let leap = year % 4 == 0 && (year % 100 != 0 || year % 400 == 0);
    let days = match month {
        1 | 3 | 5 | 7 | 8 | 10 | 12 => 31,
        4 | 6 | 9 | 11 => 30,
        2 if leap => 29,
        2 => 28,
        _ => return false,
    };
    let _ = (hour, minute, second);
    day <= days
}

/// Run the public package preflight from a filesystem directory.
pub fn validate_fixture_dir(directory: &Path) -> Vec<String> {
    match load_fixture_documents(directory) {
        Ok(documents) => validate_documents(&documents),
        Err(errors) => errors,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fixture_dir() -> std::path::PathBuf {
        Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../../mycelix-workspace/docs/ops-intel/fixtures")
    }

    fn load_real_fixture() -> BTreeMap<String, Value> {
        load_fixture_documents(&fixture_dir()).expect("committed synthetic fixture package must load")
    }

    #[test]
    fn committed_fixture_package_passes_structural_preflight() {
        let errors = validate_fixture_dir(&fixture_dir());
        assert!(errors.is_empty(), "unexpected fixture findings: {errors:#?}");
    }

    #[test]
    fn strict_parser_rejects_duplicate_keys_at_root_and_nested_depth() {
        assert!(parse_strict_json(br#"{"a":1,"a":2}"#).is_err());
        assert!(parse_strict_json(br#"{"outer":{"a":1,"a":2}}"#).is_err());
    }

    #[test]
    fn strict_parser_rejects_trailing_data_and_oversized_input() {
        assert!(parse_strict_json(br#"{"ok":true} trailing"#).is_err());
        assert!(parse_strict_json(&vec![b' '; MAX_FILE_BYTES + 1]).is_err());
    }

    #[test]
    fn rejects_non_object_f0_frontier_without_panicking() {
        let mut docs = load_real_fixture();
        docs.get_mut("f0").unwrap().as_object_mut().unwrap()
            .insert("frontier".to_owned(), Value::Array(Vec::new()));
        assert!(validate_documents(&docs).iter().any(|e| e.contains("frontier must be an object")));
    }

    #[test]
    fn rejects_non_object_f0_coverage_without_panicking() {
        let mut docs = load_real_fixture();
        let observations = docs.get_mut("f0").unwrap()["observations"].as_array_mut().unwrap();
        let item = observations.iter_mut().find(|v| v["observation_ref"] == "observation:supplier-B-inventory-partial").unwrap();
        item.as_object_mut().unwrap().insert("coverage".to_owned(), Value::Array(Vec::new()));
        assert!(validate_documents(&docs).iter().any(|e| e.contains("coverage must be an object")));
    }

    #[test]
    fn rejects_unhashable_f0_facility_reference() {
        let mut docs = load_real_fixture();
        let observations = docs.get_mut("f0").unwrap()["observations"].as_array_mut().unwrap();
        let item = observations.iter_mut().find(|v| v["observation_ref"] == "observation:supplier-B-inventory-partial").unwrap();
        item["coverage"]["facility_refs_observed"] = serde_json::json!([["not-a-string"]]);
        assert!(validate_documents(&docs).iter().any(|e| e.contains("expected a non-empty string ref")));
    }

    #[test]
    fn rejects_unhashable_f1_facility_reference() {
        let mut docs = load_real_fixture();
        docs.get_mut("f1").unwrap()["added_observations"][0]["coverage"]["facility_refs_observed"] =
            serde_json::json!([{"not":"a-ref"}]);
        assert!(validate_documents(&docs).iter().any(|e| e.contains("expected a non-empty string ref")));
    }

    #[test]
    fn rejects_duplicate_facility_references_even_when_membership_matches() {
        let mut docs = load_real_fixture();
        let observations = docs.get_mut("f0").unwrap()["observations"].as_array_mut().unwrap();
        let item = observations.iter_mut().find(|v| v["observation_ref"] == "observation:supplier-B-inventory-partial").unwrap();
        item["coverage"]["facility_refs_observed"] = serde_json::json!([
            "subject:supplier-B-facility-A",
            "subject:supplier-B-facility-A",
            "subject:supplier-B-facility-B",
            "subject:supplier-B-facility-C"
        ]);
        assert!(validate_documents(&docs).iter().any(|e| e.contains("duplicate refs are forbidden") || e.contains("must cover exactly facilities A/B/C")));
    }

    #[test]
    fn rejects_missing_mock_authority_negative_controls() {
        let mut docs = load_real_fixture();
        docs.get_mut("f2").unwrap()["authority_effect_cases"] = Value::Array(Vec::new());
        assert!(validate_documents(&docs).iter().any(|e| e.contains("required mock permit disposition")));
    }

    #[test]
    fn rejects_attempt_effect_laundering() {
        let mut docs = load_real_fixture();
        docs.get_mut("f2").unwrap()["attempt"]["effect_state"] = Value::String("EffectSuccess".to_owned());
        assert!(validate_documents(&docs).iter().any(|e| e.contains("effect must remain Unobserved")));
    }

    #[test]
    fn rejects_wrong_f1_parent_frontier() {
        let mut docs = load_real_fixture();
        docs.get_mut("f1").unwrap()["parent_frontier_ref"] = Value::String("frontier:F3".to_owned());
        assert!(validate_documents(&docs).iter().any(|e| e.contains("F1: parent must be F0")));
    }

    #[test]
    fn rejects_out_of_order_frontier_times() {
        let mut docs = load_real_fixture();
        docs.get_mut("f2").unwrap()["frontier_cutoff_utc"] = Value::String("2026-06-01T11:59:00Z".to_owned());
        assert!(validate_documents(&docs).iter().any(|e| e.contains("strictly increase")));
    }

    #[test]
    fn rejects_protected_payload_disclosure() {
        let mut docs = load_real_fixture();
        docs.get_mut("f0").unwrap()["protected_fields"][0]["payload_included"] = Value::Bool(true);
        assert!(validate_documents(&docs).iter().any(|e| e.contains("payload must be explicitly absent")));
    }

    #[test]
    fn rejects_duplicate_predicate_ids() {
        let mut docs = load_real_fixture();
        let items = docs.get_mut("predicates").unwrap()["predicates"].as_array_mut().unwrap();
        items[1]["id"] = items[0]["id"].clone();
        assert!(validate_documents(&docs).iter().any(|e| e.contains("duplicate id values")));
    }

    #[test]
    fn rejects_hidden_evaluator_payload_key_recursively() {
        let mut docs = load_real_fixture();
        docs.get_mut("f0").unwrap()["nested"] = serde_json::json!({"evaluator_truth_payload": {"secret": true}});
        assert!(validate_documents(&docs).iter().any(|e| e.contains("forbidden evaluator-only key")));
    }

    #[test]
    fn rejects_invalid_timestamp_calendar_values() {
        assert!(!valid_utc_timestamp("2026-02-29T12:00:00Z"));
        assert!(valid_utc_timestamp("2024-02-29T12:00:00Z"));
        assert!(!valid_utc_timestamp("2026-13-01T12:00:00Z"));
    }
}
