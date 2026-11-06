//! Borrowed event-payload prototype for the Pulse multi-conductor verifier.
//!
//! This module is test-only while schema parity is established. It borrows
//! payload bytes from the input buffer and indexes the top-level data fields
//! without constructing a recursive serde_json::Value tree.

use serde::de::{MapAccess, Visitor};
use serde::{Deserialize, Deserializer};
use serde_json::value::RawValue;
use std::fmt;

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct BorrowedHistory<'a> {
    schema_version: u32,
    #[serde(borrow)]
    trace_id: &'a str,
    #[serde(borrow)]
    evidence_class: &'a str,
    #[serde(borrow)]
    source_commit: &'a str,
    #[serde(borrow)]
    conductors: Vec<BorrowedConductor<'a>>,
    #[serde(borrow)]
    events: Vec<BorrowedEvent<'a>>,
    #[serde(borrow)]
    causal_edges: Vec<BorrowedCausalEdge<'a>>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct BorrowedConductor<'a> {
    #[serde(borrow)]
    conductor_id: &'a str,
    #[serde(borrow)]
    agent_key_ref: &'a str,
    #[serde(borrow)]
    conductor_build: &'a str,
    #[serde(borrow)]
    dna_hash: &'a str,
    #[serde(borrow)]
    cell_id: &'a str,
    #[serde(borrow)]
    integrity_wasm_sha256: &'a str,
    #[serde(borrow)]
    role: &'a str,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct BorrowedEvent<'a> {
    #[serde(borrow)]
    event_id: &'a str,
    #[serde(borrow)]
    conductor_id: &'a str,
    local_sequence: u64,
    #[serde(borrow)]
    source: &'a str,
    #[serde(borrow)]
    evidence_ref: &'a str,
    #[serde(borrow)]
    event_type: &'a str,
    #[serde(borrow)]
    data: &'a RawValue,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct BorrowedCausalEdge<'a> {
    #[serde(borrow)]
    from_event: &'a str,
    #[serde(borrow)]
    to_event: &'a str,
    #[serde(borrow)]
    relation: &'a str,
    #[serde(borrow)]
    evidence_ref: &'a str,
}

/// A small vector of borrowed raw field values. Vec is intentional in this
/// prototype: it avoids allocating a BTreeMap node and a Value tree per field.
/// Event payloads in the current schema are small closed objects.
const MAX_EVENT_DATA_FIELDS: usize = 16;

#[derive(Debug)]
struct RawObject<'a> {
    fields: [Option<(&'a str, &'a RawValue)>; MAX_EVENT_DATA_FIELDS],
    len: usize,
}

impl<'de> Deserialize<'de> for RawObject<'de> {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        struct RawObjectVisitor;

        impl<'de> Visitor<'de> for RawObjectVisitor {
            type Value = RawObject<'de>;

            fn expecting(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
                formatter.write_str("a JSON object with borrowed raw field values")
            }

            fn visit_map<A>(self, mut map: A) -> Result<Self::Value, A::Error>
            where
                A: MapAccess<'de>,
            {
                let mut fields = [None; MAX_EVENT_DATA_FIELDS];
                let mut len = 0;
                while let Some((key, value)) =
                    map.next_entry::<&'de str, &'de RawValue>()?
                {
                    if len == MAX_EVENT_DATA_FIELDS {
                        return Err(serde::de::Error::custom(
                            "event data exceeds the closed v2 field-count bound",
                        ));
                    }
                    fields[len] = Some((key, value));
                    len += 1;
                }
                Ok(RawObject { fields, len })
            }
        }

        deserializer.deserialize_map(RawObjectVisitor)
    }
}

impl<'a> RawObject<'a> {
    fn raw(&self, key: &str) -> Option<&'a RawValue> {
        self.fields[..self.len]
            .iter()
            .flatten()
            .find_map(|(candidate, value)| (*candidate == key).then_some(*value))
    }

    fn str_value(&self, key: &str) -> Option<&'a str> {
        let raw = self.raw(key)?;
        serde_json::from_str::<&'a str>(raw.get()).ok()
    }

    fn u64_value(&self, key: &str) -> Option<u64> {
        let raw = self.raw(key)?;
        serde_json::from_str::<u64>(raw.get()).ok()
    }

    fn bool_value(&self, key: &str) -> Option<bool> {
        let raw = self.raw(key)?;
        serde_json::from_str::<bool>(raw.get()).ok()
    }

    fn str_array(&self, key: &str) -> Option<Vec<&'a str>> {
        let raw = self.raw(key)?;
        serde_json::from_str::<Vec<&'a str>>(raw.get()).ok()
    }

    fn is_null(&self, key: &str) -> bool {
        self.raw(key)
            .is_some_and(|raw| raw.get().trim() == "null")
    }

    fn is_string(&self, key: &str) -> bool {
        self.str_value(key).is_some()
    }

    fn exact_fields(&self, expected: &[&str]) -> bool {
        self.len == expected.len()
            && self.fields[..self.len]
                .iter()
                .flatten()
                .all(|(key, _)| expected.contains(key))
            && expected.iter().all(|expected_key| {
                self.fields[..self.len]
                    .iter()
                    .flatten()
                    .filter(|(key, _)| key == expected_key)
                    .count()
                    == 1
            })
    }

    fn exact_fields_with_optional(&self, required: &[&str], optional: &[&str]) -> bool {
        self.fields[..self.len]
            .iter()
            .flatten()
            .all(|(key, _)| required.contains(key) || optional.contains(key))
            && required.iter().all(|required_key| {
                self.fields[..self.len]
                    .iter()
                    .flatten()
                    .filter(|(key, _)| key == required_key)
                    .count()
                    == 1
            })
            && optional.iter().all(|optional_key| {
                self.fields[..self.len]
                    .iter()
                    .flatten()
                    .filter(|(key, _)| key == optional_key)
                    .count()
                    <= 1
            })
    }

    fn nonempty_str(&self, key: &str) -> bool {
        self.str_value(key)
            .is_some_and(|value| !value.trim().is_empty())
    }

    fn nonempty_str_array(&self, key: &str) -> bool {
        self.str_array(key)
            .is_some_and(|values| !values.is_empty() && values.iter().all(|v| !v.trim().is_empty()))
    }

    fn optional_string_or_null(&self, key: &str) -> bool {
        self.is_null(key) || self.is_string(key)
    }
}

fn valid_sha256(value: &str) -> bool {
    value.len() == 64
        && value
            .bytes()
            .all(|byte| byte.is_ascii_hexdigit() && !byte.is_ascii_uppercase())
}

fn valid_event_shape(event: &BorrowedEvent<'_>) -> bool {
    let data: RawObject<'_> = match serde_json::from_str(event.data.get()) {
        Ok(data) => data,
        Err(_) => return false,
    };
    let (fields, strings, numbers, booleans, arrays): (
        &[&str],
        &[&str],
        &[&str],
        &[&str],
        &[&str],
    ) = match event.event_type {
        "host_grant_issued" => (
            &[
                "grant_ref",
                "grant_action_hash",
                "grantor",
                "grantee",
                "assigned",
                "functions",
                "idempotency_key",
                "request_digest_sha256",
                "host_authority_index",
            ],
            &[
                "grant_ref",
                "grant_action_hash",
                "grantor",
                "grantee",
                "idempotency_key",
                "request_digest_sha256",
            ],
            &["host_authority_index"],
            &["assigned"],
            &["functions"],
        ),
        "host_grant_revoked" => (
            &[
                "grant_ref",
                "target_grant_action_hash",
                "actor",
                "revocation_action_hash",
                "host_authority_index",
            ],
            &[
                "grant_ref",
                "target_grant_action_hash",
                "actor",
                "revocation_action_hash",
            ],
            &["host_authority_index"],
            &[],
            &[],
        ),
        "capability_claim_accepted" => (
            &[
                "grant_ref",
                "grant_action_hash",
                "recipient",
                "claim_action_hash",
            ],
            &[
                "grant_ref",
                "grant_action_hash",
                "recipient",
                "claim_action_hash",
            ],
            &[],
            &[],
            &[],
        ),
        "host_call" => (
            &[
                "operation_id",
                "attempt_id",
                "grant_ref",
                "grant_action_hash",
                "caller",
                "function",
                "host_authority_index",
                "host_decision",
                "host_state_at_decision",
                "domain_decision",
                "projection_used_for_authorization",
            ],
            &[
                "operation_id",
                "grant_ref",
                "grant_action_hash",
                "caller",
                "function",
                "host_decision",
                "host_state_at_decision",
                "domain_decision",
            ],
            &["attempt_id", "host_authority_index"],
            &["projection_used_for_authorization"],
            &[],
        ),
        "domain_effect" => (
            &["operation_id", "attempt_id", "effect_id", "action_hash"],
            &["operation_id", "effect_id", "action_hash"],
            &["attempt_id"],
            &[],
            &[],
        ),
        "action_authored" => (
            &["action_hash", "author", "entry_type", "payload_sha256"],
            &["action_hash", "author", "entry_type", "payload_sha256"],
            &[],
            &[],
            &[],
        ),
        "validation_disposition" => (
            &[
                "action_hash",
                "author_conductor_id",
                "receiving_conductor_id",
                "disposition",
                "validator_wasm_sha256",
                "reason_code",
            ],
            &[
                "action_hash",
                "author_conductor_id",
                "receiving_conductor_id",
                "disposition",
                "validator_wasm_sha256",
            ],
            &[],
            &[],
            &[],
        ),
        "projection_observed" => (
            &[
                "action_hash",
                "author_conductor_id",
                "projection_state",
                "decision",
                "required_dna_hash",
                "observed_dna_hash",
                "required_cell_id",
                "observed_cell_id",
                "required_author",
                "observed_author",
                "required_action_hash",
                "observed_action_hash",
                "required_epoch",
                "observed_epoch",
                "descends_from_required",
            ],
            &[
                "action_hash",
                "author_conductor_id",
                "projection_state",
                "decision",
                "required_dna_hash",
                "observed_dna_hash",
                "required_cell_id",
                "observed_cell_id",
                "required_author",
                "observed_author",
                "required_action_hash",
                "observed_action_hash",
            ],
            &["required_epoch", "observed_epoch"],
            &["descends_from_required"],
            &[],
        ),
        "remote_signal" => (
            &[
                "event_type",
                "trust_class",
                "transport_sender",
                "claimed_sender",
                "backing_action_hash",
                "record_author",
                "verified_payload_sha256",
                "verifier_build",
                "backing_author_conductor_id",
                "backing_validator_event_id",
                "trusted_state_mutation",
                "authorizes_operation",
            ],
            &["event_type", "trust_class"],
            &[],
            &["trusted_state_mutation", "authorizes_operation"],
            &[],
        ),
        "caller_outcome" => (
            &["operation_id", "attempt_id", "outcome"],
            &["operation_id", "outcome"],
            &["attempt_id"],
            &[],
            &[],
        ),
        _ => return false,
    };

    let exact_shape = if event.event_type == "projection_observed" {
        data.exact_fields_with_optional(fields, &["validator_event_id"])
    } else {
        data.exact_fields(fields)
    };
    let optional_projection_ref_valid = event.event_type != "projection_observed"
        || data
            .raw("validator_event_id")
            .is_none_or(|_| data.nonempty_str("validator_event_id"));

    if !exact_shape
        || !optional_projection_ref_valid
        || !strings.iter().all(|key| data.nonempty_str(key))
        || !numbers
            .iter()
            .all(|key| data.u64_value(key).is_some())
        || !booleans
            .iter()
            .all(|key| data.bool_value(key).is_some())
        || !arrays
            .iter()
            .all(|key| data.nonempty_str_array(key))
    {
        return false;
    }

    match event.event_type {
        "host_grant_issued" => {
            data
                .u64_value("host_authority_index")
                .is_some_and(|value| value > 0)
                && data
                    .str_value("request_digest_sha256")
                    .is_some_and(valid_sha256)
        }
        "host_grant_revoked" => data
            .u64_value("host_authority_index")
            .is_some_and(|value| value > 0),
        "action_authored" => data
            .str_value("payload_sha256")
            .is_some_and(valid_sha256),
        "host_call" => {
            data
                .u64_value("attempt_id")
                .is_some_and(|value| value > 0)
                && data
                    .u64_value("host_authority_index")
                    .is_some_and(|value| value > 0)
                && matches!(
                    data.str_value("host_decision"),
                    Some("authorized" | "unauthorized" | "unknown")
                )
                && matches!(
                    data.str_value("host_state_at_decision"),
                    Some("active" | "revoked" | "absent" | "unknown")
                )
                && matches!(
                    data.str_value("domain_decision"),
                    Some("allow" | "deny" | "not_run" | "unknown")
                )
        }
        "domain_effect" => data
            .u64_value("attempt_id")
            .is_some_and(|value| value > 0),
        "caller_outcome" => {
            data
                .u64_value("attempt_id")
                .is_some_and(|value| value > 0)
                && matches!(
                    data.str_value("outcome"),
                    Some("acknowledged" | "unknown" | "committed" | "not_committed" | "rejected")
                )
        }
        "validation_disposition" => {
            matches!(
                data.str_value("disposition"),
                Some("accepted" | "rejected")
            ) && data
                .str_value("validator_wasm_sha256")
                .is_some_and(valid_sha256)
                && data.optional_string_or_null("reason_code")
        }
        "projection_observed" => {
            matches!(
                data.str_value("projection_state"),
                Some("integrated" | "not_integrated" | "pending" | "conflicting" | "unknown")
            ) && matches!(
                data.str_value("decision"),
                Some("trusted" | "stale_advisory" | "unknown" | "conflicting")
            )
        }
        "remote_signal" => {
            matches!(
                data.str_value("trust_class"),
                Some("verified_record" | "authenticated_peer_hint" | "unverified_hint" | "local_only" | "unknown" | "rejected")
            ) && [
                "transport_sender",
                "claimed_sender",
                "backing_action_hash",
                "record_author",
                "verified_payload_sha256",
                "verifier_build",
                "backing_author_conductor_id",
                "backing_validator_event_id",
            ]
            .iter()
            .all(|key| data.optional_string_or_null(key))
                && (data.is_null("verified_payload_sha256")
                    || data
                        .str_value("verified_payload_sha256")
                        .is_some_and(valid_sha256))
        }
        _ => false,
    }
}


#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct ParseSummary {
    pub conductors: usize,
    pub events: usize,
    pub valid_events: usize,
    pub causal_edges: usize,
}

fn valid_source(value: &str) -> bool {
    matches!(
        value,
        "conductor_host"
            | "source_chain"
            | "validator"
            | "observer"
            | "remote_peer"
            | "client"
            | "coordinator"
            | "synthetic"
    )
}

fn valid_sha40(value: &str) -> bool {
    value.len() == 40
        && value
            .bytes()
            .all(|byte| byte.is_ascii_hexdigit() && !byte.is_ascii_uppercase())
}

/// Parse the closed v2 top-level shape and validate each borrowed event payload.
/// This is deliberately a parser/shape gate only; it does not perform the
/// semantic indexes, partial-order graph, grant lifecycle, or evidence joins.
pub(crate) fn borrowed_parse_validate(raw: &str) -> Result<ParseSummary, String> {
    let history: BorrowedHistory<'_> =
        serde_json::from_str(raw).map_err(|error| error.to_string())?;
    if history.schema_version != 2
        || history.trace_id.trim().is_empty()
        || !valid_sha40(history.source_commit)
        || !matches!(history.evidence_class, "synthetic" | "runtime_capture")
        || history.conductors.len() < 2
    {
        return Err("borrowed top-level v2 header is invalid".into());
    }

    for conductor in &history.conductors {
        if conductor.conductor_id.trim().is_empty()
            || conductor.agent_key_ref.trim().is_empty()
            || conductor.conductor_build.trim().is_empty()
            || conductor.dna_hash.trim().is_empty()
            || conductor.cell_id.trim().is_empty()
            || !valid_sha256(conductor.integrity_wasm_sha256)
            || !matches!(
                conductor.role,
                "grant_owner" | "caller" | "validator" | "observer" | "peer"
            )
        {
            return Err("borrowed conductor lineage is invalid".into());
        }
    }

    let mut valid_events = 0usize;
    for event in &history.events {
        if event.event_id.trim().is_empty()
            || event.conductor_id.trim().is_empty()
            || event.local_sequence == 0
            || event.evidence_ref.trim().is_empty()
            || !valid_source(event.source)
            || !history
                .conductors
                .iter()
                .any(|conductor| conductor.conductor_id == event.conductor_id)
            || (history.evidence_class == "runtime_capture" && event.source == "synthetic")
            || !valid_event_shape(event)
        {
            return Err(format!(
                "borrowed event {} failed v2 identity/source/shape validation",
                event.event_id
            ));
        }
        valid_events += 1;
    }

    for edge in &history.causal_edges {
        if edge.from_event.trim().is_empty()
            || edge.to_event.trim().is_empty()
            || edge.relation.trim().is_empty()
            || edge.evidence_ref.trim().is_empty()
        {
            return Err("borrowed causal edge identity is invalid".into());
        }
    }

    Ok(ParseSummary {
        conductors: history.conductors.len(),
        events: history.events.len(),
        valid_events,
        causal_edges: history.causal_edges.len(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::Value;

    const SAFE_HISTORY: &str = include_str!("../examples/safe-history.json");

    #[test]
    fn borrowed_parser_matches_owned_top_level_identity_and_event_count() {
        let borrowed: BorrowedHistory<'_> = serde_json::from_str(SAFE_HISTORY).unwrap();
        let owned: Value = serde_json::from_str(SAFE_HISTORY).unwrap();

        assert_eq!(borrowed.schema_version, owned["schema_version"]);
        assert_eq!(borrowed.trace_id, owned["trace_id"].as_str().unwrap());
        assert_eq!(
            borrowed.evidence_class,
            owned["evidence_class"].as_str().unwrap()
        );
        assert_eq!(
            borrowed.source_commit,
            owned["source_commit"].as_str().unwrap()
        );
        assert_eq!(borrowed.conductors.len(), owned["conductors"].as_array().unwrap().len());
        for (conductor, owned_conductor) in borrowed.conductors.iter().zip(owned["conductors"].as_array().unwrap()) {
            assert_eq!(conductor.conductor_id, owned_conductor["conductor_id"].as_str().unwrap());
            assert_eq!(conductor.agent_key_ref, owned_conductor["agent_key_ref"].as_str().unwrap());
            assert_eq!(conductor.conductor_build, owned_conductor["conductor_build"].as_str().unwrap());
            assert_eq!(conductor.dna_hash, owned_conductor["dna_hash"].as_str().unwrap());
            assert_eq!(conductor.cell_id, owned_conductor["cell_id"].as_str().unwrap());
            assert_eq!(conductor.integrity_wasm_sha256, owned_conductor["integrity_wasm_sha256"].as_str().unwrap());
            assert_eq!(conductor.role, owned_conductor["role"].as_str().unwrap());
        }
        assert_eq!(borrowed.events.len(), owned["events"].as_array().unwrap().len());
        assert_eq!(borrowed.causal_edges.len(), owned["causal_edges"].as_array().unwrap().len());
        for (edge, owned_edge) in borrowed.causal_edges.iter().zip(owned["causal_edges"].as_array().unwrap()) {
            assert_eq!(edge.from_event, owned_edge["from_event"].as_str().unwrap());
            assert_eq!(edge.to_event, owned_edge["to_event"].as_str().unwrap());
            assert_eq!(edge.relation, owned_edge["relation"].as_str().unwrap());
            assert_eq!(edge.evidence_ref, owned_edge["evidence_ref"].as_str().unwrap());
        }
    }

    #[test]
    fn borrowed_event_payloads_match_owned_json_and_validate_exact_shapes() {
        let borrowed: BorrowedHistory<'_> = serde_json::from_str(SAFE_HISTORY).unwrap();
        let owned: Value = serde_json::from_str(SAFE_HISTORY).unwrap();
        let owned_events = owned["events"].as_array().unwrap();

        for (event, owned_event) in borrowed.events.iter().zip(owned_events) {
            assert_eq!(event.event_id, owned_event["event_id"].as_str().unwrap());
            assert_eq!(event.conductor_id, owned_event["conductor_id"].as_str().unwrap());
            assert_eq!(event.local_sequence, owned_event["local_sequence"].as_u64().unwrap());
            assert_eq!(event.source, owned_event["source"].as_str().unwrap());
            assert_eq!(event.evidence_ref, owned_event["evidence_ref"].as_str().unwrap());
            assert_eq!(event.event_type, owned_event["event_type"].as_str().unwrap());

            assert!(valid_event_shape(event), "event {} did not match v2 shape", event.event_id);
            let borrowed_data: RawObject<'_> = serde_json::from_str(event.data.get()).unwrap();
            for (key, raw) in borrowed_data.fields[..borrowed_data.len].iter().flatten() {
                assert_eq!(
                    serde_json::from_str::<Value>(raw.get()).unwrap(),
                    owned_event["data"][*key],
                    "field {key} differed for event {}",
                    event.event_id
                );
            }
        }

        assert!(borrowed
            .events
            .iter()
            .any(|event| event.event_type == "remote_signal"));
        assert!(borrowed
            .events
            .iter()
            .any(|event| event.event_type == "host_grant_issued"));
    }

    #[test]
    fn borrowed_and_owned_event_shape_decisions_match_for_safe_fixture() {
        let borrowed: BorrowedHistory<'_> = serde_json::from_str(SAFE_HISTORY).unwrap();
        for event in &borrowed.events {
            let owned = crate::Event {
                event_id: event.event_id.to_string(),
                conductor_id: event.conductor_id.to_string(),
                local_sequence: event.local_sequence,
                source: event.source.to_string(),
                evidence_ref: event.evidence_ref.to_string(),
                event_type: event.event_type.to_string(),
                data: serde_json::from_str::<Value>(event.data.get()).unwrap(),
            };
            assert_eq!(
                valid_event_shape(event),
                crate::valid_event_shape(&owned),
                "borrowed/owned shape decision differed for {}",
                event.event_id
            );
        }
    }

    fn assert_event_shape_parity(raw: &str, label: &str) {
        let borrowed: BorrowedHistory<'_> = serde_json::from_str(raw)
            .unwrap_or_else(|error| panic!("{label}: borrowed history parse failed: {error}"));
        let owned: crate::History = serde_json::from_str(raw)
            .unwrap_or_else(|error| panic!("{label}: owned history parse failed: {error}"));

        assert_eq!(borrowed.events.len(), owned.events.len(), "{label}: event count");
        for (borrowed_event, owned_event) in borrowed.events.iter().zip(&owned.events) {
            assert_eq!(
                valid_event_shape(borrowed_event),
                crate::valid_event_shape(owned_event),
                "{label}: event {}",
                borrowed_event.event_id
            );
        }
    }

    #[test]
    fn borrowed_and_owned_event_shape_decisions_match_adversarial_mutations() {
        let mut cases: Vec<(&str, String)> = Vec::new();

        let mut unknown_field: Value = serde_json::from_str(SAFE_HISTORY).unwrap();
        let host_call = unknown_field["events"]
            .as_array_mut()
            .unwrap()
            .iter_mut()
            .find(|event| event["event_type"] == "host_call")
            .unwrap();
        host_call["data"]["unreviewed_field"] = Value::Bool(true);
        cases.push(("unknown field", serde_json::to_string(&unknown_field).unwrap()));

        let mut missing_field: Value = serde_json::from_str(SAFE_HISTORY).unwrap();
        let host_call = missing_field["events"]
            .as_array_mut()
            .unwrap()
            .iter_mut()
            .find(|event| event["event_type"] == "host_call")
            .unwrap();
        host_call["data"].as_object_mut().unwrap().remove("attempt_id");
        cases.push(("missing required field", serde_json::to_string(&missing_field).unwrap()));

        let mut wrong_type: Value = serde_json::from_str(SAFE_HISTORY).unwrap();
        let host_call = wrong_type["events"]
            .as_array_mut()
            .unwrap()
            .iter_mut()
            .find(|event| event["event_type"] == "host_call")
            .unwrap();
        host_call["data"]["attempt_id"] = Value::String("1".into());
        cases.push(("wrong scalar type", serde_json::to_string(&wrong_type).unwrap()));

        let mut zero_authority_index: Value = serde_json::from_str(SAFE_HISTORY).unwrap();
        let host_call = zero_authority_index["events"]
            .as_array_mut()
            .unwrap()
            .iter_mut()
            .find(|event| event["event_type"] == "host_call")
            .unwrap();
        host_call["data"]["host_authority_index"] = Value::from(0);
        cases.push(("zero authority index", serde_json::to_string(&zero_authority_index).unwrap()));

        let mut invalid_decision: Value = serde_json::from_str(SAFE_HISTORY).unwrap();
        let host_call = invalid_decision["events"]
            .as_array_mut()
            .unwrap()
            .iter_mut()
            .find(|event| event["event_type"] == "host_call")
            .unwrap();
        host_call["data"]["host_decision"] = Value::String("maybe".into());
        cases.push(("unknown host decision", serde_json::to_string(&invalid_decision).unwrap()));

        let mut malformed_digest: Value = serde_json::from_str(SAFE_HISTORY).unwrap();
        let grant = malformed_digest["events"]
            .as_array_mut()
            .unwrap()
            .iter_mut()
            .find(|event| event["event_type"] == "host_grant_issued")
            .unwrap();
        grant["data"]["request_digest_sha256"] = Value::String("not-a-sha256".into());
        cases.push(("malformed request digest", serde_json::to_string(&malformed_digest).unwrap()));

        let mut empty_functions: Value = serde_json::from_str(SAFE_HISTORY).unwrap();
        let grant = empty_functions["events"]
            .as_array_mut()
            .unwrap()
            .iter_mut()
            .find(|event| event["event_type"] == "host_grant_issued")
            .unwrap();
        grant["data"]["functions"] = Value::Array(Vec::new());
        cases.push(("empty function allowlist", serde_json::to_string(&empty_functions).unwrap()));

        for (label, raw) in cases {
            assert_event_shape_parity(&raw, label);
        }
    }

    #[test]
    fn duplicate_event_payload_keys_are_rejected_before_owned_value_materialization() {
        let original = r#""host_decision": "authorized""#;
        let duplicate = r#""host_decision": "authorized", "host_decision": "unauthorized""#;
        assert!(SAFE_HISTORY.contains(original), "fixture anchor changed");
        let mutated = SAFE_HISTORY.replacen(original, duplicate, 1);

        // serde_json::Value materialization collapses duplicate object keys. The
        // production run path must therefore execute borrowed_parse_validate first.
        assert!(serde_json::from_str::<crate::History>(&mutated).is_ok());
        assert!(
            borrowed_parse_validate(&mutated).is_err(),
            "duplicate payload keys must fail the raw borrowed preflight"
        );
    }

    #[test]
    fn unknown_top_level_event_field_is_rejected() {
        let mutated = SAFE_HISTORY.replacen(
            r#""event_id": "grant-issue""#,
            r#""event_id": "grant-issue", "unreviewed": true"#,
            1,
        );
        assert!(serde_json::from_str::<BorrowedHistory<'_>>(&mutated).is_err());
    }

    #[test]
    fn duplicate_payload_keys_are_rejected_by_exact_field_validation() {
        let duplicate: RawObject<'_> =
            serde_json::from_str(r#"{"operation_id":"first","operation_id":"second"}"#).unwrap();
        assert!(!duplicate.exact_fields(&["operation_id"]));
        assert!(!duplicate.exact_fields(&["operation_id", "attempt_id"]));
    }

    #[test]
    fn event_payload_slices_borrow_from_the_input_buffer() {
        let borrowed: BorrowedHistory<'_> = serde_json::from_str(SAFE_HISTORY).unwrap();
        let input_start = SAFE_HISTORY.as_ptr() as usize;
        let input_end = input_start + SAFE_HISTORY.len();
        for event in &borrowed.events {
            let start = event.data.get().as_ptr() as usize;
            let end = start + event.data.get().len();
            assert!(start >= input_start && end <= input_end);
        }
    }

    #[test]
    fn unknown_event_data_fields_are_rejected() {
        let mut value: Value = serde_json::from_str(SAFE_HISTORY).unwrap();
        let event = value["events"]
            .as_array_mut()
            .unwrap()
            .iter_mut()
            .find(|event| event["event_type"] == "host_call")
            .unwrap();
        event["data"]["debug_secret"] = Value::String("must-not-enter-evidence".into());
        let mutated = serde_json::to_string(&value).unwrap();
        let borrowed: BorrowedHistory<'_> = serde_json::from_str(&mutated).unwrap();
        let host_call = borrowed
            .events
            .iter()
            .find(|event| event.event_type == "host_call")
            .unwrap();
        assert!(!valid_event_shape(host_call));
    }

    #[test]
    fn event_payload_type_errors_are_rejected() {
        let mut value: Value = serde_json::from_str(SAFE_HISTORY).unwrap();
        let event = value["events"]
            .as_array_mut()
            .unwrap()
            .iter_mut()
            .find(|event| event["event_type"] == "host_call")
            .unwrap();
        event["data"]["attempt_id"] = Value::String("1".into());
        let mutated = serde_json::to_string(&value).unwrap();
        let borrowed: BorrowedHistory<'_> = serde_json::from_str(&mutated).unwrap();
        let host_call = borrowed
            .events
            .iter()
            .find(|event| event.event_type == "host_call")
            .unwrap();
        assert!(!valid_event_shape(host_call));
    }
}
