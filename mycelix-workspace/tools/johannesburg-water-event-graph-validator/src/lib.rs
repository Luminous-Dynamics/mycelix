// SPDX-License-Identifier: AGPL-3.0-or-later
//! Read-only semantic validator for the Johannesburg Water notice event graph.
//! This crate validates metadata lineage only; it is not a service measurement system.

use serde_json::{Value, json};
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet};

pub const REGISTER_ID: &str = "nrvca-johannesburg-water-service-sources-v0.1";
pub const GRAPH_ID: &str = "nrvca-johannesburg-water-event-graph-v0.1";
pub const NEGATIVE_CORPUS_ID: &str = "johannesburg-water-event-graph-negative-controls-v0.1";

fn required_string<'a>(value: &'a Value, key: &str, errors: &mut Vec<String>, context: &str) -> Option<&'a str> {
    match value.get(key).and_then(Value::as_str) {
        Some(text) if !text.is_empty() => Some(text),
        _ => {
            errors.push(format!("{context}: missing or invalid string {key}"));
            None
        }
    }
}

fn array<'a>(value: &'a Value, key: &str, errors: &mut Vec<String>, context: &str) -> Option<&'a Vec<Value>> {
    match value.get(key).and_then(Value::as_array) {
        Some(items) => Some(items),
        None => {
            errors.push(format!("{context}: missing or invalid array {key}"));
            None
        }
    }
}

fn unique_insert(set: &mut BTreeSet<String>, id: &str, context: &str, errors: &mut Vec<String>) {
    if !set.insert(id.to_owned()) {
        errors.push(format!("{context}: duplicate identifier {id}"));
    }
}

pub fn sha256_hex(bytes: &[u8]) -> String {
    Sha256::digest(bytes).iter().map(|byte| format!("{byte:02x}")).collect()
}

/// Compute SHA-1 over Git's canonical blob header and content.
pub fn git_blob_sha1_hex(bytes: &[u8]) -> String {
    let mut message = format!("blob {}\0", bytes.len()).into_bytes();
    message.extend_from_slice(bytes);
    let bit_len = (message.len() as u64) * 8;
    message.push(0x80);
    while message.len() % 64 != 56 {
        message.push(0);
    }
    message.extend_from_slice(&bit_len.to_be_bytes());

    let mut h0: u32 = 0x67452301;
    let mut h1: u32 = 0xEFCDAB89;
    let mut h2: u32 = 0x98BADCFE;
    let mut h3: u32 = 0x10325476;
    let mut h4: u32 = 0xC3D2E1F0;

    for chunk in message.chunks_exact(64) {
        let mut words = [0u32; 80];
        for (index, bytes) in chunk.chunks_exact(4).enumerate() {
            words[index] = u32::from_be_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]);
        }
        for index in 16..80 {
            words[index] =
                (words[index - 3] ^ words[index - 8] ^ words[index - 14] ^ words[index - 16])
                    .rotate_left(1);
        }

        let (mut a, mut b, mut c, mut d, mut e) = (h0, h1, h2, h3, h4);
        for index in 0..80 {
            let (function, constant) = match index {
                0..=19 => ((b & c) | ((!b) & d), 0x5A827999),
                20..=39 => (b ^ c ^ d, 0x6ED9EBA1),
                40..=59 => ((b & c) | (b & d) | (c & d), 0x8F1BBCDC),
                _ => (b ^ c ^ d, 0xCA62C1D6),
            };
            let temporary = a
                .rotate_left(5)
                .wrapping_add(function)
                .wrapping_add(e)
                .wrapping_add(constant)
                .wrapping_add(words[index]);
            e = d;
            d = c;
            c = b.rotate_left(30);
            b = a;
            a = temporary;
        }
        h0 = h0.wrapping_add(a);
        h1 = h1.wrapping_add(b);
        h2 = h2.wrapping_add(c);
        h3 = h3.wrapping_add(d);
        h4 = h4.wrapping_add(e);
    }

    format!("{h0:08x}{h1:08x}{h2:08x}{h3:08x}{h4:08x}")
}

fn clock_minutes(published: &str) -> Option<u16> {
    let token = published
        .split(['–', '-', '—'])
        .next_back()?
        .trim()
        .split_whitespace()
        .last()?;
    let mut parts = token.split(':');
    let hour = parts.next()?.parse::<u16>().ok()?;
    let minute = parts.next()?.parse::<u16>().ok()?;
    if parts.next().is_some() || hour > 23 || minute > 59 {
        return None;
    }
    Some(hour * 60 + minute)
}

fn validate_schema_contract(schema: &Value, errors: &mut Vec<String>) {
    if schema.get("$schema").and_then(Value::as_str)
        != Some("https://json-schema.org/draft/2020-12/schema")
    {
        errors.push("graph schema must declare JSON Schema Draft 2020-12".to_owned());
    }
    if schema.get("type").and_then(Value::as_str) != Some("object") {
        errors.push("graph schema root must be an object".to_owned());
    }
    if schema.get("additionalProperties").and_then(Value::as_bool) != Some(false) {
        errors.push("graph schema root must reject additional properties".to_owned());
    }
    let required: BTreeSet<&str> = schema
        .get("required")
        .and_then(Value::as_array)
        .into_iter()
        .flatten()
        .filter_map(Value::as_str)
        .collect();
    for key in [
        "schema_version",
        "ledger_id",
        "source_snapshot",
        "assertions",
        "events",
        "relations",
        "claim_ceiling",
    ] {
        if !required.contains(key) {
            errors.push(format!("graph schema does not require {key}"));
        }
    }
}

pub fn validate(
    register: &Value,
    graph: &Value,
    negative_controls: &Value,
    schema: &Value,
    register_bytes: &[u8],
) -> Result<Value, Vec<String>> {
    let mut errors = Vec::new();
    validate_schema_contract(schema, &mut errors);

    if register.get("schema_version").and_then(Value::as_str) != Some("0.1.0") {
        errors.push("unsupported source-register schema version".to_owned());
    }
    if register.get("register_id").and_then(Value::as_str) != Some(REGISTER_ID) {
        errors.push("unexpected source-register identity".to_owned());
    }
    if graph.get("schema_version").and_then(Value::as_str) != Some("0.1.0") {
        errors.push("unsupported event-graph schema version".to_owned());
    }
    if graph.get("ledger_id").and_then(Value::as_str) != Some(GRAPH_ID) {
        errors.push("unexpected event-graph identity".to_owned());
    }
    if negative_controls.get("schema_version").and_then(Value::as_str) != Some("0.1.0")
        || negative_controls.get("corpus_id").and_then(Value::as_str)
            != Some(NEGATIVE_CORPUS_ID)
    {
        errors.push("unexpected negative-control corpus identity/version".to_owned());
    }

    let source_snapshot = graph.get("source_snapshot").unwrap_or(&Value::Null);
    let pinned_register_sha =
        required_string(source_snapshot, "register_git_blob_sha1", &mut errors, "source_snapshot");
    if source_snapshot.get("register_id").and_then(Value::as_str) != Some(REGISTER_ID) {
        errors.push("event graph source_snapshot.register_id does not match the supplied register".to_owned());
    }
    if source_snapshot.get("register_path").and_then(Value::as_str)
        != Some("docs/research/nrvca/johannesburg-water-service-sources-v0.1.json")
    {
        errors.push("event graph source_snapshot.register_path is not the canonical register path".to_owned());
    }
    let computed_git_sha = git_blob_sha1_hex(register_bytes);
    if pinned_register_sha != Some(computed_git_sha.as_str()) {
        errors.push(format!(
            "source register Git blob SHA-1 mismatch: expected {:?}, computed {}",
            pinned_register_sha, computed_git_sha
        ));
    }

    let mut source_records: BTreeMap<String, &Value> = BTreeMap::new();
    let sources = array(register, "sources", &mut errors, "source register")
        .cloned()
        .unwrap_or_default();
    for source in &sources {
        if let Some(source_id) =
            required_string(source, "source_id", &mut errors, "source record")
        {
            if source_records.insert(source_id.to_owned(), source).is_some() {
                errors.push(format!("source register contains duplicate source_id {source_id}"));
            }
        }
    }

    let assertions = array(graph, "assertions", &mut errors, "event graph")
        .cloned()
        .unwrap_or_default();
    let mut assertion_by_id: BTreeMap<String, &Value> = BTreeMap::new();
    let mut assertion_ids = BTreeSet::new();

    for assertion in &assertions {
        let context = "assertion";
        let id = required_string(assertion, "assertion_id", &mut errors, context);
        let source_id = required_string(assertion, "source_id", &mut errors, context);
        let publisher = required_string(assertion, "publisher", &mut errors, context);
        let media_uri = required_string(assertion, "media_uri", &mut errors, context);
        let notice_class = required_string(assertion, "notice_class", &mut errors, context);
        let published = required_string(assertion, "published_at_text", &mut errors, context);

        if let Some(id) = id {
            unique_insert(&mut assertion_ids, id, context, &mut errors);
            assertion_by_id.insert(id.to_owned(), assertion);
        }
        if assertion.get("claim_level").and_then(Value::as_str)
            != Some("publisher_assertion_not_independent_measurement")
        {
            errors.push(format!("{context}: assertion claim level exceeds publisher statement"));
        }
        if !assertion
            .get("source_bytes_sha256")
            .is_some_and(Value::is_null)
        {
            errors.push(format!(
                "{context}: source-byte SHA-256 must remain null until exact bytes are acquired"
            ));
        }

        if let (Some(source_id), Some(publisher), Some(media_uri), Some(notice_class)) =
            (source_id, publisher, media_uri, notice_class)
        {
            match source_records.get(source_id) {
                None => errors.push(format!(
                    "{context}: source_id {source_id} is absent from supplied register"
                )),
                Some(source) => {
                    if source.get("publisher").and_then(Value::as_str) != Some(publisher) {
                        errors.push(format!(
                            "{context}: publisher does not match source register for {source_id}"
                        ));
                    }
                    let media_index = assertion.get("media_index").and_then(Value::as_u64);
                    let media = source
                        .get("notice_media_reviews")
                        .and_then(Value::as_array)
                        .into_iter()
                        .flatten()
                        .find(|item| {
                            item.get("media_index").and_then(Value::as_u64) == media_index
                        });
                    match media {
                        None => errors.push(format!(
                            "{context}: media_index {:?} does not resolve in source {source_id}",
                            media_index
                        )),
                        Some(media) => {
                            if media.get("media_uri").and_then(Value::as_str) != Some(media_uri) {
                                errors.push(format!(
                                    "{context}: media URI mismatch for {source_id} / {:?}",
                                    media_index
                                ));
                            }
                            if media.get("notice_class").and_then(Value::as_str)
                                != Some(notice_class)
                            {
                                errors.push(format!(
                                    "{context}: notice class mismatch for {source_id} / {:?}",
                                    media_index
                                ));
                            }
                            if media.get("media_sha256").and_then(Value::as_str)
                                != assertion.get("source_bytes_sha256").and_then(Value::as_str)
                            {
                                errors.push(format!(
                                    "{context}: source media digest mismatch for {source_id} / {:?}",
                                    media_index
                                ));
                            }
                        }
                    }
                }
            }
        }

        if let Some(published) = published {
            let event_time = assertion.get("event_time").unwrap_or(&Value::Null);
            if event_time.get("kind").and_then(Value::as_str) == Some("unknown")
                && (event_time.get("start_text").is_some_and(|value| !value.is_null())
                    || event_time.get("end_text").is_some_and(|value| !value.is_null()))
            {
                errors.push(format!(
                    "{context}: unknown event time must not have start/end values ({published})"
                ));
            }
        }
    }

    let events = array(graph, "events", &mut errors, "event graph")
        .cloned()
        .unwrap_or_default();
    let mut event_ids = BTreeSet::new();
    let mut memberships: BTreeMap<String, String> = BTreeMap::new();
    let mut schedule_count = 0usize;
    let mut snapshot_assertion_ids = BTreeSet::new();
    let mut lonehill_event: Option<&Value> = None;

    for event in &events {
        let context = "event";
        let event_id = required_string(event, "event_id", &mut errors, context);
        let event_kind = required_string(event, "event_kind", &mut errors, context);
        if let Some(id) = event_id {
            unique_insert(&mut event_ids, id, context, &mut errors);
        }
        let assertion_refs = array(event, "assertion_ids", &mut errors, context)
            .cloned()
            .unwrap_or_default();
        if assertion_refs.is_empty() {
            errors.push(format!("{context}: event must contain at least one assertion"));
        }
        for item in &assertion_refs {
            match item.as_str() {
                None => errors.push(format!("{context}: assertion_ids entries must be strings")),
                Some(assertion_id) => {
                    if !assertion_by_id.contains_key(assertion_id) {
                        errors.push(format!(
                            "{context}: unknown assertion reference {assertion_id}"
                        ));
                    }
                    if let Some(event_id) = event_id {
                        if memberships
                            .insert(assertion_id.to_owned(), event_id.to_owned())
                            .is_some()
                        {
                            errors.push(format!(
                                "{context}: assertion {assertion_id} belongs to more than one event"
                            ));
                        }
                    }
                    if assertion_id.starts_with("JHBW.A.CITYCAP.") {
                        snapshot_assertion_ids.insert(assertion_id.to_owned());
                    }
                }
            }
        }

        if event.get("service_impact_status").and_then(Value::as_str)
            != Some("reported_only")
        {
            errors.push(format!(
                "{context}: publisher-only event cannot be promoted beyond reported_only"
            ));
        }
        if event
            .get("reported_geography")
            .and_then(|value| value.get("geometry_status"))
            .and_then(Value::as_str)
            != Some("not_available")
        {
            errors.push(format!(
                "{context}: unverified notice geography must remain not_available"
            ));
        }

        if let Some(kind) = event_kind {
            match kind {
                "unplanned_physical_incident" => {
                    if event.get("identity_state").and_then(Value::as_str)
                        != Some("provisional")
                    {
                        errors.push(format!(
                            "{context}: notice-derived burst event must remain provisional"
                        ));
                    }
                    if event
                        .get("event_time")
                        .and_then(|value| value.get("kind"))
                        .and_then(Value::as_str)
                        != Some("unknown")
                    {
                        errors.push(format!(
                            "{context}: physical incident onset must remain unknown when not reported"
                        ));
                    }
                    if event.get("restoration_status").and_then(Value::as_str) != Some("unknown")
                    {
                        errors.push(format!(
                            "{context}: burst restoration must remain unknown without outcome evidence"
                        ));
                    }
                    if event_id == Some("JHBW.E.BURST.LONEHILL.20261105.01") {
                        lonehill_event = Some(event);
                    }
                }
                "scheduled_operating_rule" => {
                    schedule_count += 1;
                    if event
                        .get("event_time")
                        .and_then(|value| value.get("kind"))
                        .and_then(Value::as_str)
                        != Some("recurring_schedule")
                    {
                        errors.push(format!(
                            "{context}: scheduled operation must retain recurring_schedule semantics"
                        ));
                    }
                    if event.get("restoration_status").and_then(Value::as_str)
                        != Some("not_applicable")
                    {
                        errors.push(format!("{context}: schedule rule is not a restoration estimate"));
                    }
                }
                "system_status_snapshot" => {
                    if event
                        .get("event_time")
                        .and_then(|value| value.get("kind"))
                        .and_then(Value::as_str)
                        != Some("snapshot_at")
                    {
                        errors.push(format!("{context}: system status must remain a snapshot"));
                    }
                }
                "infrastructure_security_activity" => {}
                other => errors.push(format!("{context}: unsupported event_kind {other}")),
            }
        }
    }

    if memberships.len() != assertion_ids.len() {
        errors.push(format!(
            "every assertion must belong to exactly one event ({} memberships / {} assertions)",
            memberships.len(),
            assertion_ids.len()
        ));
    }
    if schedule_count != 1 {
        errors.push(format!(
            "expected exactly one recurring schedule event, found {schedule_count}"
        ));
    }
    if snapshot_assertion_ids.len() != 3 {
        errors.push(format!(
            "expected three separate City capacity snapshot assertions, found {}",
            snapshot_assertion_ids.len()
        ));
    }

    if let Some(event) = lonehill_event {
        let ids: BTreeSet<&str> = event
            .get("assertion_ids")
            .and_then(Value::as_array)
            .into_iter()
            .flatten()
            .filter_map(Value::as_str)
            .collect();
        if ids
            != BTreeSet::from([
                "JHBW.A.LONEHILL.20261105.1220",
                "JHBW.A.LONEHILL.20261105.2050",
            ])
        {
            errors.push(
                "Lonehill provisional event must contain exactly its two source assertions"
                    .to_owned(),
            );
        }
    } else {
        errors.push("missing provisional Lonehill incident event".to_owned());
    }

    let relations = array(graph, "relations", &mut errors, "event graph")
        .cloned()
        .unwrap_or_default();
    let mut relation_ids = BTreeSet::new();
    let mut lonehill_update_count = 0usize;
    for relation in &relations {
        let id = required_string(relation, "relation_id", &mut errors, "relation");
        let from = required_string(relation, "from_assertion_id", &mut errors, "relation");
        let to = required_string(relation, "to_assertion_id", &mut errors, "relation");
        let kind = required_string(relation, "relation_type", &mut errors, "relation");
        if let Some(id) = id {
            unique_insert(&mut relation_ids, id, "relation", &mut errors);
        }
        if let (Some(from), Some(to), Some(kind)) = (from, to, kind) {
            if from == to {
                errors.push(format!("relation {id:?}: self-link is not allowed"));
            }
            if !assertion_by_id.contains_key(from) || !assertion_by_id.contains_key(to) {
                errors.push(format!(
                    "relation {id:?}: endpoint does not resolve to an assertion"
                ));
            }
            match kind {
                "updates"
                | "continuation_of"
                | "announces_schedule_for"
                | "snapshot_about"
                | "corroborates"
                | "contradicts"
                | "supersedes"
                | "same_reported_incident_as" => {}
                other => errors.push(format!("relation {id:?}: unsupported relation type {other}")),
            }
            if from == "JHBW.A.LONEHILL.20261105.2050"
                && to == "JHBW.A.LONEHILL.20261105.1220"
                && kind == "updates"
            {
                lonehill_update_count += 1;
                let from_time = assertion_by_id
                    .get(from)
                    .and_then(|value| value.get("published_at_text"))
                    .and_then(Value::as_str)
                    .and_then(clock_minutes);
                let to_time = assertion_by_id
                    .get(to)
                    .and_then(|value| value.get("published_at_text"))
                    .and_then(Value::as_str)
                    .and_then(clock_minutes);
                if !matches!((from_time, to_time), (Some(later), Some(earlier)) if later > earlier)
                {
                    errors.push(
                        "Lonehill update edge must point from later report to earlier report"
                            .to_owned(),
                    );
                }
            }
        }
    }
    if lonehill_update_count != 1 {
        errors.push(format!(
            "expected exactly one manually reviewed Lonehill updates edge, found {lonehill_update_count}"
        ));
    }

    for assertion_id in [
        "JHBW.A.CITYCAP.20261103.1240",
        "JHBW.A.CITYCAP.20261104.1105",
        "JHBW.A.CITYCAP.20261105.1320",
    ] {
        match assertion_by_id.get(assertion_id) {
            None => errors.push(format!("missing expected snapshot assertion {assertion_id}")),
            Some(assertion) => {
                let metrics = assertion.get("reported_metrics").and_then(Value::as_array);
                if !metrics.is_some_and(|items| {
                    !items.is_empty()
                        && items.iter().all(|metric| {
                            metric.get("method_status").and_then(Value::as_str)
                                == Some("method_not_supplied")
                                && metric
                                    .get("comparability_status")
                                    .and_then(Value::as_str)
                                    == Some("not_established")
                        })
                }) {
                    errors.push(format!(
                        "{assertion_id}: capacity metric method/comparability must remain unqualified"
                    ));
                }
            }
        }
    }

    let mut negative_case_ids = BTreeSet::new();
    let cases = array(negative_controls, "cases", &mut errors, "negative-control corpus")
        .cloned()
        .unwrap_or_default();
    if cases.len() != 13 {
        errors.push(format!(
            "expected 13 negative-control cases, found {}",
            cases.len()
        ));
    }
    let mut observed_case_count = 0usize;
    let mut synthetic_case_count = 0usize;
    for case in &cases {
        let context = "negative-control case";
        if let Some(id) = required_string(case, "case_id", &mut errors, context) {
            unique_insert(&mut negative_case_ids, id, context, &mut errors);
        }
        let source_type = required_string(case, "source_type", &mut errors, context);
        let input_ids = array(case, "input_assertion_ids", &mut errors, context)
            .cloned()
            .unwrap_or_default();
        match source_type {
            Some("observed_register_fixture") => {
                observed_case_count += 1;
                for item in &input_ids {
                    match item.as_str() {
                        Some(id) if assertion_by_id.contains_key(id) => {}
                        Some(id) => errors.push(format!(
                            "{context}: observed fixture assertion {id} does not resolve"
                        )),
                        None => errors.push(format!(
                            "{context}: observed fixture input IDs must be strings"
                        )),
                    }
                }
            }
            Some("synthetic_negative_control") => {
                synthetic_case_count += 1;
                if !input_ids.is_empty() {
                    errors.push(format!(
                        "{context}: synthetic controls must not reference real Johannesburg assertion IDs"
                    ));
                }
            }
            Some(other) => errors.push(format!("{context}: unsupported source_type {other}")),
            None => {}
        }
    }
    if observed_case_count != 3 || synthetic_case_count != 10 {
        errors.push(format!(
            "expected 3 observed fixture cases and 10 synthetic controls; found {observed_case_count} / {synthetic_case_count}"
        ));
    }

    if !graph
        .get("claim_ceiling")
        .and_then(Value::as_str)
        .is_some_and(|text| {
            text.contains("metadata-only") && text.contains("not a complete incident register")
        })
    {
        errors.push("event graph claim ceiling is missing or has been broadened".to_owned());
    }
    if !register
        .get("claim_ceiling")
        .and_then(Value::as_str)
        .is_some_and(|text| text.contains("metadata-only"))
    {
        errors.push("source register claim ceiling is missing or has been broadened".to_owned());
    }

    if errors.is_empty() {
        Ok(json!({
            "schema_version": "johannesburg-water-event-graph-validation-v0.1",
            "verification_state": "SourceAndGraphContractsMatched",
            "qualification_status": "NotRun",
            "register_git_blob_sha1": computed_git_sha,
            "assertion_count": assertion_ids.len(),
            "event_count": event_ids.len(),
            "relation_count": relation_ids.len(),
            "negative_case_count": negative_case_ids.len(),
            "observed_fixture_case_count": observed_case_count,
            "synthetic_negative_case_count": synthetic_case_count,
            "evidence_class": "metadata_only_source_assertion_lineage",
            "claim_ceiling": "source_binding_and_event_graph_semantics_only_no_measurement_or_field_qualification",
            "nonclaims": [
                "No source media bytes, sensor records, CMMS work orders or customer observations are acquired by this validator.",
                "No hydraulic state, household service outcome, event duration or restoration is measured.",
                "No event group authorizes utility actuation, service allocation, enforcement or relocation."
            ]
        }))
    } else {
        Err(errors)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const REGISTER_BYTES: &[u8] =
        include_bytes!("../../../../docs/research/nrvca/johannesburg-water-service-sources-v0.1.json");
    const GRAPH_TEXT: &str =
        include_str!("../../../../docs/research/nrvca/johannesburg-water-event-graph-v0.1.json");
    const NEGATIVE_TEXT: &str =
        include_str!("../../../../docs/research/nrvca/johannesburg-water-event-graph-negative-controls-v0.1.json");
    const SCHEMA_TEXT: &str =
        include_str!("../../../../docs/research/nrvca/johannesburg-water-event-graph-v0.1.schema.json");

    fn fixtures() -> (Value, Value, Value, Value) {
        (
            serde_json::from_slice(REGISTER_BYTES).expect("register parses"),
            serde_json::from_str(GRAPH_TEXT).expect("graph parses"),
            serde_json::from_str(NEGATIVE_TEXT).expect("negative controls parse"),
            serde_json::from_str(SCHEMA_TEXT).expect("schema parses"),
        )
    }

    #[test]
    fn canonical_register_graph_and_negative_corpus_validate() {
        let (register, graph, negative, schema) = fixtures();
        let result =
            validate(&register, &graph, &negative, &schema, REGISTER_BYTES).expect("valid fixtures");
        assert_eq!(result["assertion_count"], 6);
        assert_eq!(result["event_count"], 5);
        assert_eq!(result["relation_count"], 1);
        assert_eq!(result["negative_case_count"], 13);
        assert_eq!(result["qualification_status"], "NotRun");
    }

    #[test]
    fn git_blob_sha1_matches_known_empty_blob() {
        assert_eq!(
            git_blob_sha1_hex(b""),
            "e69de29bb2d1d6434b8b29ae775ad8c2e48c5391"
        );
    }

    #[test]
    fn source_media_uri_mismatch_is_rejected() {
        let (register, mut graph, negative, schema) = fixtures();
        graph["assertions"][0]["media_uri"] =
            json!("https://example.invalid/not-the-source.jpeg");
        let errors = validate(&register, &graph, &negative, &schema, REGISTER_BYTES)
            .expect_err("wrong media binding must fail");
        assert!(errors.iter().any(|item| item.contains("media URI mismatch")));
    }

    #[test]
    fn dangling_event_assertion_is_rejected() {
        let (register, mut graph, negative, schema) = fixtures();
        graph["events"][0]["assertion_ids"][0] = json!("JHBW.A.DOES.NOT.EXIST");
        let errors = validate(&register, &graph, &negative, &schema, REGISTER_BYTES)
            .expect_err("dangling assertion must fail");
        assert!(errors
            .iter()
            .any(|item| item.contains("unknown assertion reference")));
    }

    #[test]
    fn assertion_cannot_belong_to_two_events() {
        let (register, mut graph, negative, schema) = fixtures();
        let duplicate = graph["events"][0]["assertion_ids"][0].clone();
        graph["events"][1]["assertion_ids"]
            .as_array_mut()
            .expect("array")
            .push(duplicate);
        let errors = validate(&register, &graph, &negative, &schema, REGISTER_BYTES)
            .expect_err("duplicate event membership must fail");
        assert!(errors
            .iter()
            .any(|item| item.contains("belongs to more than one event")));
    }

    #[test]
    fn recurring_schedule_cannot_be_expanded_into_occurrence_events() {
        let (register, mut graph, negative, schema) = fixtures();
        let mut occurrence = graph["events"][1].clone();
        occurrence["event_id"] = json!("JHBW.E.SCHEDULE.OCCURRENCE.FABRICATED");
        occurrence["event_kind"] = json!("unplanned_physical_incident");
        graph["events"]
            .as_array_mut()
            .expect("events array")
            .push(occurrence);
        let errors = validate(&register, &graph, &negative, &schema, REGISTER_BYTES)
            .expect_err("fabricated occurrence must fail");
        assert!(errors
            .iter()
            .any(|item| item.contains("expected exactly one recurring schedule event")));
    }

    #[test]
    fn system_capacity_snapshots_cannot_be_promoted_to_comparable_measurements() {
        let (register, mut graph, negative, schema) = fixtures();
        graph["assertions"][3]["reported_metrics"][0]["comparability_status"] =
            json!("established");
        let errors = validate(&register, &graph, &negative, &schema, REGISTER_BYTES)
            .expect_err("metric comparability promotion must fail");
        assert!(errors
            .iter()
            .any(|item| item.contains("capacity metric method/comparability")));
    }

    #[test]
    fn source_register_byte_mutation_breaks_git_blob_binding() {
        let (mut register, graph, negative, schema) = fixtures();
        register["as_of_date"] = json!("2026-11-14");
        let altered = serde_json::to_vec(&register).expect("serialize altered register");
        let errors = validate(&register, &graph, &negative, &schema, &altered)
            .expect_err("altered register bytes must fail");
        assert!(errors
            .iter()
            .any(|item| item.contains("Git blob SHA-1 mismatch")));
    }

    #[test]
    fn synthetic_negative_control_cannot_reference_live_assertion() {
        let (register, graph, mut negative, schema) = fixtures();
        negative["cases"][1]["input_assertion_ids"] =
            json!(["JHBW.A.LONEHILL.20261105.1220"]);
        let errors = validate(&register, &graph, &negative, &schema, REGISTER_BYTES)
            .expect_err("synthetic case with real assertion must fail");
        assert!(errors.iter().any(|item| {
            item.contains("synthetic controls must not reference real Johannesburg")
        }));
    }

    #[test]
    fn relation_self_links_are_rejected() {
        let (register, mut graph, negative, schema) = fixtures();
        graph["relations"][0]["to_assertion_id"] =
            graph["relations"][0]["from_assertion_id"].clone();
        let errors = validate(&register, &graph, &negative, &schema, REGISTER_BYTES)
            .expect_err("self relation must fail");
        assert!(errors
            .iter()
            .any(|item| item.contains("self-link is not allowed")));
    }

    #[test]
    fn source_assertions_must_not_claim_independent_measurement() {
        let (register, mut graph, negative, schema) = fixtures();
        graph["assertions"][0]["claim_level"] =
            json!("independently_verified_measurement");
        let errors = validate(&register, &graph, &negative, &schema, REGISTER_BYTES)
            .expect_err("publisher source cannot assert independent measurement");
        assert!(errors
            .iter()
            .any(|item| item.contains("claim level exceeds publisher statement")));
    }
}
