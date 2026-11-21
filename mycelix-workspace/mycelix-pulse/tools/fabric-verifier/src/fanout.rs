//! Parent-manifest verification for Pulse group fan-out.
//!
//! This layer proves that a content-addressed manifest and its supplied child
//! traces are structurally consistent. It does not prove who authored the
//! manifest, whether a membership snapshot is authentic, or whether a digest
//! represents truthful runtime evidence. The manifest SHA-256 must be pinned
//! by an external trust anchor.

use crate::{validate_against_schema, verify_trace, Finding};
use serde_json::Value;
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet};

const MANIFEST_PROFILE_ID: &str = "PULSE-GROUP-FANOUT-QUAL-001";
const TRACE_SCHEMA_VERSION: &str = "0.1.0";

const RECIPIENT_EVENT_TYPES: &[&str] = &[
    "recipient_admitted",
    "recipient_inbox_committed",
    "recipient_decryption_verified",
    "receipt_created",
    "receipt_verified",
];

const CONTENT_BOUND_EVENT_TYPES: &[&str] = &[
    "source_action_authored",
    "transport_attempt_started",
    "recipient_inbox_committed",
    "recipient_decryption_verified",
    "receipt_created",
    "receipt_verified",
];

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FanoutSummary {
    pub status: &'static str,
    pub fanout_id: String,
    pub manifest_sha256: String,
    pub disposition: &'static str,
    pub required_recipients: usize,
    pub verified_recipients: usize,
    pub partial_recipients: usize,
    pub unknown_recipients: usize,
    pub incomplete_recipients: usize,
    pub expired_recipients: usize,
    pub failed_recipients: usize,
}

#[derive(Debug, Clone)]
struct ExpectedChild {
    recipient_commitment: String,
    logical_operation_id: String,
    trace_sha256: String,
}

fn push_finding(
    findings: &mut Vec<Finding>,
    path: impl Into<String>,
    message: impl Into<String>,
) {
    findings.push(Finding {
        path: path.into(),
        message: message.into(),
    });
}

fn is_lower_hex(value: &str, length: usize) -> bool {
    value.len() == length
        && value
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}

fn sha256_hex(bytes: &[u8]) -> String {
    let digest = Sha256::digest(bytes);
    digest.iter().map(|byte| format!("{byte:02x}")).collect()
}

fn required_string(
    object: &Value,
    key: &str,
    path: &str,
    findings: &mut Vec<Finding>,
) -> Option<String> {
    match object.get(key).and_then(Value::as_str) {
        Some(value) if !value.trim().is_empty() => Some(value.to_owned()),
        _ => {
            push_finding(
                findings,
                format!("{path}.{key}"),
                "required non-empty string is missing or has the wrong type",
            );
            None
        }
    }
}

fn required_digest(
    object: &Value,
    key: &str,
    path: &str,
    findings: &mut Vec<Finding>,
) -> Option<String> {
    match required_string(object, key, path, findings) {
        Some(value) if is_lower_hex(&value, 64) => Some(value),
        Some(_) => {
            push_finding(
                findings,
                format!("{path}.{key}"),
                "expected a 64-character lowercase SHA-256 digest",
            );
            None
        }
        None => None,
    }
}

fn recipient_set_commitment(recipient_commitments: &[String]) -> String {
    let mut sorted = recipient_commitments.to_vec();
    sorted.sort();
    let mut digest = Sha256::new();
    digest.update(b"PULSE-GROUP-FANOUT-RECIPIENT-SET-V0\0");
    for commitment in sorted {
        let bytes = commitment.as_bytes();
        digest.update((bytes.len() as u64).to_be_bytes());
        digest.update(bytes);
    }
    let output = digest.finalize();
    output.iter().map(|byte| format!("{byte:02x}")).collect()
}

fn child_group_field<'a>(group: &'a Value, key: &str) -> Option<&'a str> {
    group.get(key).and_then(Value::as_str)
}

fn compare_group_string(
    group: &Value,
    key: &str,
    expected: &str,
    path: &str,
    findings: &mut Vec<Finding>,
) {
    if child_group_field(group, key) != Some(expected) {
        push_finding(
            findings,
            format!("{path}.{key}"),
            "does not match the externally pinned fan-out manifest",
        );
    }
}

fn disposition_counts(
    dispositions: impl Iterator<Item = String>,
) -> (usize, usize, usize, usize, usize, usize) {
    let mut verified = 0;
    let mut partial = 0;
    let mut unknown = 0;
    let mut incomplete = 0;
    let mut expired = 0;
    let mut failed = 0;

    for disposition in dispositions {
        match disposition.as_str() {
            "SUCCESS_VERIFIED" => verified += 1,
            "UNKNOWN" => unknown += 1,
            "PARTIAL" => partial += 1,
            "INCOMPLETE" | "NOT_APPLICABLE" => incomplete += 1,
            "EXPIRED" => expired += 1,
            "FAILED" | "REVOKED" => failed += 1,
            _ => incomplete += 1,
        }
    }

    (verified, partial, unknown, incomplete, expired, failed)
}

fn aggregate_disposition(
    required: usize,
    verified: usize,
    partial: usize,
    unknown: usize,
    incomplete: usize,
    expired: usize,
) -> &'static str {
    if required > 0 && verified == required {
        "ALL_REQUIRED_RECEIPT_CLAIMS_CONSISTENT"
    } else if verified > 0 || partial > 0 {
        "PARTIAL_RECEIPT_EVIDENCE"
    } else if unknown > 0 {
        "UNKNOWN"
    } else if incomplete > 0 {
        "INCOMPLETE"
    } else if expired > 0 {
        "EXPIRED"
    } else {
        "FAILED"
    }
}

/// Verify a manifest and every child trace against pinned schemas and digests.
///
/// manifest_bytes are the exact bytes supplied to the CLI. The external
/// manifest digest is mandatory: a self-asserted digest inside the manifest
/// would not provide an independent trust anchor. Child trace digests are
/// checked over their exact supplied bytes.
pub fn verify_fanout(
    manifest_bytes: &[u8],
    expected_manifest_sha256: &str,
    manifest_schema: &Value,
    trace_schema: &Value,
    child_files: &[(String, Vec<u8>)],
    expected_source_sha: Option<&str>,
) -> Result<FanoutSummary, Vec<Finding>> {
    let mut findings = Vec::new();

    if !is_lower_hex(expected_manifest_sha256, 64) {
        push_finding(
            &mut findings,
            "$.expected_manifest_sha256",
            "external manifest pin must be 64 lowercase hexadecimal characters",
        );
    }

    let actual_manifest_sha256 = sha256_hex(manifest_bytes);
    if actual_manifest_sha256 != expected_manifest_sha256 {
        push_finding(
            &mut findings,
            "$.manifest",
            "manifest bytes do not match the externally supplied SHA-256 pin",
        );
    }

    let manifest: Value = match serde_json::from_slice(manifest_bytes) {
        Ok(value) => value,
        Err(error) => {
            push_finding(
                &mut findings,
                "$.manifest",
                format!("manifest is not valid JSON: {error}"),
            );
            return Err(findings);
        }
    };

    if let Err(errors) = validate_against_schema(&manifest, manifest_schema) {
        for error in errors {
            push_finding(
                &mut findings,
                "$.manifest",
                format!("manifest schema validation failed: {error}"),
            );
        }
    }

    let fanout_id = required_string(&manifest, "fanout_id", "$.manifest", &mut findings)
        .unwrap_or_default();
    let profile_id = required_string(&manifest, "profile_id", "$.manifest", &mut findings);
    if profile_id.as_deref() != Some(MANIFEST_PROFILE_ID) {
        push_finding(
            &mut findings,
            "$.manifest.profile_id",
            format!("expected profile id {MANIFEST_PROFILE_ID}"),
        );
    }

    let group_id_commitment =
        required_digest(&manifest, "group_id_commitment", "$.manifest", &mut findings);
    let membership_snapshot_digest = required_digest(
        &manifest,
        "membership_snapshot_digest",
        "$.manifest",
        &mut findings,
    );
    let required_recipient_set_commitment = required_digest(
        &manifest,
        "required_recipient_set_commitment",
        "$.manifest",
        &mut findings,
    );
    let content_commitment =
        required_digest(&manifest, "content_commitment", "$.manifest", &mut findings);

    let membership_epoch = manifest
        .get("membership_epoch")
        .and_then(Value::as_u64);
    if membership_epoch.is_none() {
        push_finding(
            &mut findings,
            "$.manifest.membership_epoch",
            "membership_epoch must be a non-negative integer",
        );
    }

    if manifest.get("aggregation_policy").and_then(Value::as_str) != Some("ALL_REQUIRED") {
        push_finding(
            &mut findings,
            "$.manifest.aggregation_policy",
            "v0.1 only supports the explicit ALL_REQUIRED aggregation policy",
        );
    }

    let mut expected_children = BTreeMap::new();
    let mut recipient_commitments = Vec::new();
    let mut recipient_set = BTreeSet::new();

    match manifest.get("children").and_then(Value::as_array) {
        Some(children) => {
            for (index, child) in children.iter().enumerate() {
                let path = format!("$.manifest.children[{index}]");
                let recipient =
                    required_digest(child, "recipient_commitment", &path, &mut findings);
                let logical_operation_id = required_string(
                    child,
                    "logical_operation_id",
                    &path,
                    &mut findings,
                );
                let trace_sha256 =
                    required_digest(child, "trace_sha256", &path, &mut findings);

                if let Some(recipient) = recipient {
                    if !recipient_set.insert(recipient.clone()) {
                        push_finding(
                            &mut findings,
                            format!("{path}.recipient_commitment"),
                            "recipient commitment is duplicated in the frozen required set",
                        );
                    }
                    recipient_commitments.push(recipient.clone());

                    if let (Some(logical_operation_id), Some(trace_sha256)) =
                        (logical_operation_id, trace_sha256)
                    {
                        if expected_children.contains_key(&logical_operation_id) {
                            push_finding(
                                &mut findings,
                                format!("{path}.logical_operation_id"),
                                "logical operation id is duplicated in the manifest",
                            );
                        } else {
                            expected_children.insert(
                                logical_operation_id.clone(),
                                ExpectedChild {
                                    recipient_commitment: recipient,
                                    logical_operation_id,
                                    trace_sha256,
                                },
                            );
                        }
                    }
                }
            }
        }
        None => push_finding(
            &mut findings,
            "$.manifest.children",
            "children must be an array",
        ),
    }

    if recipient_commitments.windows(2).any(|pair| pair[0] >= pair[1]) {
        push_finding(
            &mut findings,
            "$.manifest.children",
            "children must be strictly sorted by recipient_commitment",
        );
    }

    let calculated_recipient_set = recipient_set_commitment(&recipient_commitments);
    if required_recipient_set_commitment.as_deref() != Some(calculated_recipient_set.as_str()) {
        push_finding(
            &mut findings,
            "$.manifest.required_recipient_set_commitment",
            "does not commit to the exact sorted recipient set in children",
        );
    }

    if expected_children.is_empty() {
        push_finding(
            &mut findings,
            "$.manifest.children",
            "at least one required recipient child is required",
        );
    }

    let mut observed_dispositions = BTreeMap::new();
    let mut seen_recipient_commitments = BTreeSet::new();

    for (child_index, (source_name, child_bytes)) in child_files.iter().enumerate() {
        let child_path = format!("$.child_files[{child_index}]");
        let child_trace: Value = match serde_json::from_slice(child_bytes) {
            Ok(value) => value,
            Err(error) => {
                push_finding(
                    &mut findings,
                    &child_path,
                    format!("{source_name}: child trace is not valid JSON: {error}"),
                );
                continue;
            }
        };

        if let Err(errors) = validate_against_schema(&child_trace, trace_schema) {
            for error in errors {
                push_finding(
                    &mut findings,
                    &child_path,
                    format!("{source_name}: trace schema validation failed: {error}"),
                );
            }
            continue;
        }

        let operation_ids: BTreeSet<String> = child_trace
            .get("events")
            .and_then(Value::as_array)
            .into_iter()
            .flatten()
            .filter_map(|event| {
                event
                    .get("logical_operation_id")
                    .and_then(Value::as_str)
                    .map(str::to_owned)
            })
            .collect();

        if operation_ids.len() != 1 {
            push_finding(
                &mut findings,
                format!("{child_path}.events"),
                format!(
                    "{source_name}: each child trace must contain exactly one logical operation"
                ),
            );
            continue;
        }

        let logical_operation_id = operation_ids.iter().next().expect("one operation id");
        let Some(expected) = expected_children.get(logical_operation_id) else {
            push_finding(
                &mut findings,
                format!("{child_path}.events"),
                format!(
                    "{source_name}: unexpected child logical operation {logical_operation_id}"
                ),
            );
            continue;
        };

        let actual_trace_sha256 = sha256_hex(child_bytes);
        if actual_trace_sha256 != expected.trace_sha256 {
            push_finding(
                &mut findings,
                child_path.clone(),
                format!("{source_name}: child trace bytes do not match the manifest digest"),
            );
        }

        let group = child_trace.get("group_fanout");
        let Some(group) = group else {
            push_finding(
                &mut findings,
                format!("{child_path}.group_fanout"),
                format!("{source_name}: group_fanout binding is required"),
            );
            continue;
        };

        compare_group_string(
            group,
            "fanout_id",
            &fanout_id,
            &format!("{child_path}.group_fanout"),
            &mut findings,
        );

        if let Some(expected) = group_id_commitment.as_deref() {
            compare_group_string(
                group,
                "group_id_commitment",
                expected,
                &format!("{child_path}.group_fanout"),
                &mut findings,
            );
        }

        if let Some(expected) = membership_snapshot_digest.as_deref() {
            compare_group_string(
                group,
                "membership_snapshot_digest",
                expected,
                &format!("{child_path}.group_fanout"),
                &mut findings,
            );
        }

        if let Some(expected) = required_recipient_set_commitment.as_deref() {
            compare_group_string(
                group,
                "required_recipient_set_commitment",
                expected,
                &format!("{child_path}.group_fanout"),
                &mut findings,
            );
        }

        if let Some(expected) = content_commitment.as_deref() {
            compare_group_string(
                group,
                "content_commitment",
                expected,
                &format!("{child_path}.group_fanout"),
                &mut findings,
            );
        }

        if group.get("membership_epoch").and_then(Value::as_u64) != membership_epoch {
            push_finding(
                &mut findings,
                format!("{child_path}.group_fanout.membership_epoch"),
                "does not match the frozen manifest membership epoch",
            );
        }

        if group.get("aggregation_policy").and_then(Value::as_str) != Some("ALL_REQUIRED") {
            push_finding(
                &mut findings,
                format!("{child_path}.group_fanout.aggregation_policy"),
                "child must bind to the ALL_REQUIRED aggregation policy",
            );
        }

        let actual_recipient = child_group_field(group, "recipient_commitment");
        if actual_recipient != Some(expected.recipient_commitment.as_str()) {
            push_finding(
                &mut findings,
                format!("{child_path}.group_fanout.recipient_commitment"),
                "does not match this child's committed recipient",
            );
        }

        if let Some(actual_recipient) = actual_recipient {
            if !seen_recipient_commitments.insert(actual_recipient.to_owned()) {
                push_finding(
                    &mut findings,
                    format!("{child_path}.group_fanout.recipient_commitment"),
                    "recipient commitment is repeated across supplied child traces",
                );
            }
        }

        let mut receipt_created = 0;
        let mut receipt_verified = 0;
        if let Some(events) = child_trace.get("events").and_then(Value::as_array) {
            for (event_index, event) in events.iter().enumerate() {
                let event_type = event.get("event_type").and_then(Value::as_str).unwrap_or("");
                if RECIPIENT_EVENT_TYPES.contains(&event_type)
                    && event.get("recipient_commitment").and_then(Value::as_str)
                        != Some(expected.recipient_commitment.as_str())
                {
                    push_finding(
                        &mut findings,
                        format!("{child_path}.events[{event_index}].recipient_commitment"),
                        "recipient-side event does not bind to this child's committed recipient",
                    );
                }
                if CONTENT_BOUND_EVENT_TYPES.contains(&event_type)
                    && event.get("content_commitment").and_then(Value::as_str)
                        != content_commitment.as_deref()
                {
                    push_finding(
                        &mut findings,
                        format!("{child_path}.events[{event_index}].content_commitment"),
                        "content-bound event does not match the fan-out content commitment",
                    );
                }
                if event_type == "receipt_created" {
                    receipt_created += 1;
                } else if event_type == "receipt_verified" {
                    receipt_verified += 1;
                }
            }
        }

        let terminal_disposition = child_trace
            .get("terminal_disposition")
            .and_then(Value::as_str)
            .unwrap_or("INCOMPLETE")
            .to_owned();

        if terminal_disposition == "SUCCESS_VERIFIED"
            && (receipt_created != 1 || receipt_verified != 1)
        {
            push_finding(
                &mut findings,
                child_path.clone(),
                "SUCCESS_VERIFIED child claim must contain exactly one recipient receipt creation and one separately attributed receipt-verification event",
            );
        }

        if let Err(child_findings) = verify_trace(&child_trace, expected_source_sha) {
            for child_finding in child_findings {
                push_finding(
                    &mut findings,
                    format!(
                        "{child_path}{}",
                        child_finding.path.trim_start_matches('$')
                    ),
                    format!("{source_name}: {}", child_finding.message),
                );
            }
        }

        observed_dispositions.insert(expected.logical_operation_id.clone(), terminal_disposition);
    }

    for (logical_operation_id, expected) in &expected_children {
        if !observed_dispositions.contains_key(logical_operation_id) {
            push_finding(
                &mut findings,
                "$.child_files",
                format!(
                    "required child trace is missing for logical operation {} and recipient commitment {}",
                    logical_operation_id, expected.recipient_commitment
                ),
            );
        }
    }

    if child_files.len() != expected_children.len() {
        push_finding(
            &mut findings,
            "$.child_files",
            "supplied child trace count must exactly equal the manifest child count",
        );
    }

    if !findings.is_empty() {
        return Err(findings);
    }

    let required_recipients = expected_children.len();
    let (verified, partial, unknown, incomplete, expired, failed) =
        disposition_counts(observed_dispositions.into_values());
    let disposition = aggregate_disposition(
        required_recipients,
        verified,
        partial,
        unknown,
        incomplete,
        expired,
    );

    Ok(FanoutSummary {
        status: "FANOUT_EVIDENCE_CONSISTENT",
        fanout_id,
        manifest_sha256: actual_manifest_sha256,
        disposition,
        required_recipients,
        verified_recipients: verified,
        partial_recipients: partial,
        unknown_recipients: unknown,
        incomplete_recipients: incomplete,
        expired_recipients: expired,
        failed_recipients: failed,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    fn schemas() -> (Value, Value) {
        let manifest = serde_json::from_str(include_str!(
            "../../../docs/pulse-group-fanout-manifest.schema-v0.1.json"
        ))
        .expect("manifest schema must parse");
        let trace = serde_json::from_str(include_str!(
            "../../../docs/pulse-fabric-trace.schema-v0.1.json"
        ))
        .expect("trace schema must parse");
        (manifest, trace)
    }

    fn grouped_success_trace(
        logical_operation_id: &str,
        recipient_commitment: &str,
        recipient_set: &str,
    ) -> Vec<u8> {
        let mut trace: Value = serde_json::from_str(include_str!(
            "../tests/fixtures/valid-success.json"
        ))
        .expect("success fixture must parse");

        for event in trace["events"].as_array_mut().expect("events array") {
            event["logical_operation_id"] = json!(logical_operation_id);
            let event_type = event["event_type"].as_str().unwrap_or("");
            if RECIPIENT_EVENT_TYPES.contains(&event_type) {
                event["recipient_commitment"] = json!(recipient_commitment);
            }
        }

        trace["group_fanout"] = json!({
            "fanout_id": "fanout-test-1",
            "group_id_commitment": "1".repeat(64),
            "membership_epoch": 7,
            "membership_snapshot_digest": "2".repeat(64),
            "required_recipient_set_commitment": recipient_set,
            "recipient_commitment": recipient_commitment,
            "content_commitment": "a".repeat(64),
            "aggregation_policy": "ALL_REQUIRED"
        });

        serde_json::to_vec(&trace).expect("group trace must serialize")
    }

    fn manifest_for(children: &[(String, String, Vec<u8>)]) -> Vec<u8> {
        let recipients: Vec<String> = children
            .iter()
            .map(|(_, recipient, _)| recipient.clone())
            .collect();
        let recipient_set = recipient_set_commitment(&recipients);
        let manifest_children: Vec<Value> = children
            .iter()
            .map(|(operation, recipient, bytes)| {
                json!({
                    "recipient_commitment": recipient,
                    "logical_operation_id": operation,
                    "trace_sha256": sha256_hex(bytes)
                })
            })
            .collect();

        serde_json::to_vec(&json!({
            "schema_version": TRACE_SCHEMA_VERSION,
            "profile_id": MANIFEST_PROFILE_ID,
            "fanout_id": "fanout-test-1",
            "group_id_commitment": "1".repeat(64),
            "membership_epoch": 7,
            "membership_snapshot_digest": "2".repeat(64),
            "required_recipient_set_commitment": recipient_set,
            "content_commitment": "a".repeat(64),
            "aggregation_policy": "ALL_REQUIRED",
            "children": manifest_children,
            "claim_ceiling": [
                "synthetic structural fan-out consistency only",
                "membership truth and receipt signatures are not verified"
            ]
        }))
        .expect("manifest must serialize")
    }

    #[test]
    fn complete_fanout_requires_all_pinned_recipient_receipts() {
        let recipient = "b".repeat(64);
        let provisional = recipient_set_commitment(std::slice::from_ref(&recipient));
        let child = grouped_success_trace("logical-1", &recipient, &provisional);
        let manifest = manifest_for(&[(
            "logical-1".to_owned(),
            recipient.clone(),
            child.clone(),
        )]);
        let manifest_value: Value =
            serde_json::from_slice(&manifest).expect("manifest JSON");
        let final_set = manifest_value["required_recipient_set_commitment"]
            .as_str()
            .expect("recipient set commitment");
        let mut child_value: Value =
            serde_json::from_slice(&child).expect("child JSON");
        child_value["group_fanout"]["required_recipient_set_commitment"] = json!(final_set);
        let child = serde_json::to_vec(&child_value).expect("child JSON");
        let manifest = manifest_for(&[("logical-1".to_owned(), recipient, child.clone())]);
        let manifest_sha = sha256_hex(&manifest);
        let (manifest_schema, trace_schema) = schemas();

        let result = verify_fanout(
            &manifest,
            &manifest_sha,
            &manifest_schema,
            &trace_schema,
            &[("child.json".to_owned(), child)],
            None,
        )
        .expect("complete child trace should qualify structurally");

        assert_eq!(result.disposition, "ALL_REQUIRED_RECEIPT_CLAIMS_CONSISTENT");
        assert_eq!(result.required_recipients, 1);
        assert_eq!(result.verified_recipients, 1);
    }

    #[test]
    fn a_single_receipt_cannot_complete_a_manifest_with_two_required_recipients() {
        let recipients = vec!["b".repeat(64), "c".repeat(64)];
        let set_commitment = recipient_set_commitment(&recipients);
        let first = grouped_success_trace("logical-1", &recipients[0], &set_commitment);
        let second = grouped_success_trace("logical-2", &recipients[1], &set_commitment);
        let children = vec![
            ("logical-1".to_owned(), recipients[0].clone(), first.clone()),
            ("logical-2".to_owned(), recipients[1].clone(), second),
        ];
        let manifest = manifest_for(&children);
        let manifest_sha = sha256_hex(&manifest);
        let (manifest_schema, trace_schema) = schemas();

        let result = verify_fanout(
            &manifest,
            &manifest_sha,
            &manifest_schema,
            &trace_schema,
            &[("only-first-child.json".to_owned(), first)],
            None,
        );

        assert!(result.is_err());
        assert!(result.unwrap_err().iter().any(|finding| {
            finding.message.contains("required child trace is missing")
        }));
    }

    #[test]
    fn partial_child_progress_is_not_reported_as_total_failure() {
        assert_eq!(
            aggregate_disposition(2, 0, 1, 0, 0, 0),
            "PARTIAL_RECEIPT_EVIDENCE"
        );
    }

    #[test]
    fn fanout_rejects_a_wrong_external_manifest_pin() {
        let recipient = "b".repeat(64);
        let set_commitment = recipient_set_commitment(std::slice::from_ref(&recipient));
        let child = grouped_success_trace("logical-1", &recipient, &set_commitment);
        let manifest = manifest_for(&[("logical-1".to_owned(), recipient, child.clone())]);
        let (manifest_schema, trace_schema) = schemas();

        let result = verify_fanout(
            &manifest,
            &"0".repeat(64),
            &manifest_schema,
            &trace_schema,
            &[("child.json".to_owned(), child)],
            None,
        );

        assert!(result.is_err());
        assert!(result.unwrap_err().iter().any(|finding| {
            finding.message.contains("externally supplied SHA-256 pin")
        }));
    }
}
