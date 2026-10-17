use serde_json::{json, Map, Value};
use sha2::{Digest, Sha256};
use std::{
    collections::{BTreeMap, BTreeSet},
    fs,
    path::Path,
};

const CASE_SCHEMA_VERSION: &str = "cross-scale-wisdom-case/1.0";
const REPORT_STATUS: &str = "STRUCTURALLY_VALID_UNSCORED";
const CLAIM_CEILING: &str = "Structural corpus validation only. This report is not model-response scoring, human-efficacy evidence, psychological safety qualification, wisdom/enlightenment measurement, or proof of consciousness.";

pub fn verify_files(
    corpus_path: impl AsRef<Path>,
    manifest_path: impl AsRef<Path>,
    schema_path: impl AsRef<Path>,
) -> Result<Value, String> {
    let corpus = fs::read(corpus_path.as_ref())
        .map_err(|error| format!("cannot read corpus: {error}"))?;
    let manifest = fs::read_to_string(manifest_path.as_ref())
        .map_err(|error| format!("cannot read manifest: {error}"))?;
    let schema = fs::read_to_string(schema_path.as_ref())
        .map_err(|error| format!("cannot read case schema: {error}"))?;
    verify_bytes(&corpus, &manifest, &schema)
}

pub fn verify_bytes(
    corpus_bytes: &[u8],
    manifest_text: &str,
    schema_text: &str,
) -> Result<Value, String> {
    let manifest: Value = serde_json::from_str(manifest_text)
        .map_err(|error| format!("manifest is not valid JSON: {error}"))?;
    let schema: Value = serde_json::from_str(schema_text)
        .map_err(|error| format!("case schema is not valid JSON: {error}"))?;

    let actual_digest = sha256_hex(corpus_bytes);
    let expected_digest = required_string(&manifest, "corpus_sha256", "manifest")?;
    if expected_digest.len() != 64
        || !expected_digest.bytes().all(|byte| byte.is_ascii_hexdigit())
        || expected_digest != actual_digest
    {
        return Err(format!(
            "corpus SHA-256 mismatch: manifest={expected_digest}, actual={actual_digest}"
        ));
    }

    let text = std::str::from_utf8(corpus_bytes)
        .map_err(|error| format!("corpus is not UTF-8: {error}"))?;
    if !text.ends_with('\n') {
        return Err("JSONL corpus must end with exactly a newline-terminated final record".to_owned());
    }

    let record_properties = properties(&schema, &["properties"])?;
    let record_required = required_fields(&schema, &["required"])?;
    let evidence_properties = properties(&schema, &[
        "properties",
        "evidence",
        "items",
        "properties",
    ])?;
    let evidence_required = required_fields(&schema, &[
        "properties",
        "evidence",
        "items",
        "required",
    ])?;
    let allowed_severity = enum_values(&schema, &["properties", "severity", "enum"])?;
    let allowed_invariants = enum_values(&schema, &[
        "properties",
        "expected_invariants",
        "items",
        "enum",
    ])?;
    let allowed_evidence_kinds = enum_values(&schema, &[
        "properties",
        "evidence",
        "items",
        "properties",
        "kind",
        "enum",
    ])?;

    if schema.get("$schema").and_then(Value::as_str).unwrap_or_default()
        != "https://json-schema.org/draft/2020-12/schema"
    {
        return Err("unsupported or missing JSON Schema dialect".to_owned());
    }
    if schema.get("additionalProperties").and_then(Value::as_bool) != Some(false) {
        return Err("case schema must reject undeclared properties".to_owned());
    }

    let manifest_version = required_string(&manifest, "corpus_version", "manifest")?;
    let manifest_id = required_string(&manifest, "corpus_id", "manifest")?;
    let manifest_path = required_string(&manifest, "corpus_path", "manifest")?;
    let declared_count = manifest
        .get("record_count")
        .and_then(Value::as_u64)
        .ok_or_else(|| "manifest.record_count must be a non-negative integer".to_owned())?;

    let mut seen_case_ids = BTreeSet::new();
    let mut seen_invariants = BTreeSet::new();
    let mut severity_counts = BTreeMap::<String, u64>::new();
    let mut case_receipts = Vec::new();
    let mut records_seen = 0_u64;

    for (zero_index, line) in text.split_terminator('\n').enumerate() {
        let line_number = zero_index + 1;
        if line.trim().is_empty() {
            return Err(format!("line {line_number} is empty"));
        }

        let record: Value = serde_json::from_str(line)
            .map_err(|error| format!("line {line_number} is not valid JSON: {error}"))?;
        validate_exact_keys(&record, &record_properties, &record_required, &format!("line {line_number}"))?;

        let case_id = required_string(&record, "case_id", &format!("line {line_number}"))?;
        if !valid_case_id(case_id) {
            return Err(format!("line {line_number} has invalid case_id {case_id:?}"));
        }
        if !seen_case_ids.insert(case_id.to_owned()) {
            return Err(format!("duplicate case_id {case_id}"));
        }
        if required_string(&record, "schema_version", case_id)? != CASE_SCHEMA_VERSION {
            return Err(format!("{case_id}: schema_version must be {CASE_SCHEMA_VERSION}"));
        }
        let family = required_string(&record, "family", case_id)?;
        let _ = required_string(&record, "context", case_id)?;
        let severity = required_string(&record, "severity", case_id)?;
        if !allowed_severity.contains(severity) {
            return Err(format!("{case_id}: unknown severity {severity:?}"));
        }
        let _ = required_string(&record, "expected_response_contract", case_id)?;
        let _ = required_string(&record, "rationale", case_id)?;

        let evidence = record
            .get("evidence")
            .and_then(Value::as_array)
            .ok_or_else(|| format!("{case_id}: evidence must be an array"))?;
        if evidence.is_empty() {
            return Err(format!("{case_id}: evidence must not be empty"));
        }
        let mut evidence_ids = BTreeSet::new();
        for (evidence_index, item) in evidence.iter().enumerate() {
            let where_ = format!("{case_id}.evidence[{evidence_index}]");
            validate_exact_keys(item, &evidence_properties, &evidence_required, &where_)?;
            let evidence_id = required_string(item, "id", &where_)?;
            if !valid_evidence_id(evidence_id) {
                return Err(format!("{where_}: invalid evidence id {evidence_id:?}"));
            }
            if !evidence_ids.insert(evidence_id.to_owned()) {
                return Err(format!("{case_id}: duplicate evidence id {evidence_id}"));
            }
            let kind = required_string(item, "kind", &where_)?;
            if !allowed_evidence_kinds.contains(kind) {
                return Err(format!("{where_}: unknown evidence kind {kind:?}"));
            }
            let _ = required_string(item, "status", &where_)?;
            let _ = required_string(item, "text", &where_)?;
            let source_ref = required_string(item, "source_ref", &where_)?;
            if !source_ref.starts_with("synthetic:") || source_ref.len() <= "synthetic:".len() {
                return Err(format!("{where_}: source_ref must be a non-empty synthetic provenance reference"));
            }
        }

        let expected_invariants = record
            .get("expected_invariants")
            .and_then(Value::as_array)
            .ok_or_else(|| format!("{case_id}: expected_invariants must be an array"))?;
        if expected_invariants.is_empty() {
            return Err(format!("{case_id}: expected_invariants must not be empty"));
        }
        let mut per_case_invariants = BTreeSet::new();
        for invariant in expected_invariants {
            let invariant = invariant
                .as_str()
                .ok_or_else(|| format!("{case_id}: invariant entries must be strings"))?;
            if !allowed_invariants.contains(invariant) {
                return Err(format!("{case_id}: unknown invariant {invariant:?}"));
            }
            if !per_case_invariants.insert(invariant.to_owned()) {
                return Err(format!("{case_id}: duplicate invariant {invariant:?}"));
            }
            seen_invariants.insert(invariant.to_owned());
        }

        let forbidden = record
            .get("forbidden_behaviors")
            .and_then(Value::as_array)
            .ok_or_else(|| format!("{case_id}: forbidden_behaviors must be an array"))?;
        if forbidden.is_empty()
            || forbidden.iter().any(|value| value.as_str().map(str::trim).unwrap_or_default().is_empty())
        {
            return Err(format!("{case_id}: forbidden_behaviors must contain non-empty strings"));
        }

        *severity_counts.entry(severity.to_owned()).or_default() += 1;
        records_seen += 1;
        case_receipts.push(json!({
            "case_id": case_id,
            "family": family,
            "severity": severity,
            "disposition": "UNSCORED",
            "receipt_scope": "STRUCTURAL_SCHEMA_VALIDATION_ONLY"
        }));
    }

    if records_seen == 0 {
        return Err("corpus contains no records".to_owned());
    }
    if records_seen != declared_count {
        return Err(format!(
            "manifest record_count mismatch: manifest={declared_count}, parsed={records_seen}"
        ));
    }

    let manifest_severity_counts = manifest
        .get("severity_counts")
        .and_then(Value::as_object)
        .ok_or_else(|| "manifest.severity_counts must be an object".to_owned())?;
    for required_severity in &allowed_severity {
        let declared = manifest_severity_counts
            .get(required_severity)
            .and_then(Value::as_u64)
            .ok_or_else(|| format!("manifest.severity_counts.{required_severity} must be an integer"))?;
        let observed = *severity_counts.get(required_severity).unwrap_or(&0);
        if declared != observed {
            return Err(format!(
                "manifest severity count mismatch for {required_severity}: manifest={declared}, parsed={observed}"
            ));
        }
    }

    let declared_invariants = manifest
        .get("required_invariants")
        .and_then(Value::as_array)
        .ok_or_else(|| "manifest.required_invariants must be an array".to_owned())?
        .iter()
        .map(|value| value.as_str().map(ToOwned::to_owned).ok_or_else(|| {
            "manifest.required_invariants entries must be strings".to_owned()
        }))
        .collect::<Result<BTreeSet<_>, _>>()?;
    if declared_invariants != seen_invariants {
        return Err("manifest.required_invariants does not exactly match corpus invariant set".to_owned());
    }

    Ok(json!({
        "report_version": "cross-scale-wisdom-verification-report/1.0",
        "status": REPORT_STATUS,
        "claim_ceiling": CLAIM_CEILING,
        "corpus_id": manifest_id,
        "corpus_version": manifest_version,
        "corpus_path": manifest_path,
        "corpus_sha256": actual_digest,
        "record_count": records_seen,
        "severity_counts": severity_counts,
        "case_receipts": case_receipts,
        "all_cases_scored": false,
        "independent_semantic_review": "NOT_ASSERTED",
        "human_efficacy": "NOT_ASSESSED",
        "psychological_safety": "NOT_ASSESSED"
    }))
}

fn properties<'a>(value: &'a Value, path: &[&str]) -> Result<&'a Map<String, Value>, String> {
    let target = walk(value, path)?;
    target.as_object().ok_or_else(|| format!("schema path {:?} must be an object", path))
}

fn required_fields(value: &Value, path: &[&str]) -> Result<BTreeSet<String>, String> {
    let target = walk(value, path)?;
    let array = target.as_array().ok_or_else(|| format!("schema path {:?} must be an array", path))?;
    array.iter().map(|entry| {
        entry.as_str().map(ToOwned::to_owned).ok_or_else(|| "schema required fields must be strings".to_owned())
    }).collect()
}

fn enum_values(value: &Value, path: &[&str]) -> Result<BTreeSet<String>, String> {
    let target = walk(value, path)?;
    let array = target.as_array().ok_or_else(|| format!("schema enum at {:?} must be an array", path))?;
    array.iter().map(|entry| {
        entry.as_str().map(ToOwned::to_owned).ok_or_else(|| "schema enum values must be strings".to_owned())
    }).collect()
}

fn walk<'a>(mut value: &'a Value, path: &[&str]) -> Result<&'a Value, String> {
    for key in path {
        value = value.get(*key).ok_or_else(|| format!("schema path is missing {key}"))?;
    }
    Ok(value)
}

fn validate_exact_keys(
    value: &Value,
    allowed: &Map<String, Value>,
    required: &BTreeSet<String>,
    where_: &str,
) -> Result<(), String> {
    let object = value.as_object().ok_or_else(|| format!("{where_} must be a JSON object"))?;
    for key in required {
        if !object.contains_key(key) {
            return Err(format!("{where_} is missing required field {key}"));
        }
    }
    for key in object.keys() {
        if !allowed.contains_key(key) {
            return Err(format!("{where_} has undeclared field {key}"));
        }
    }
    Ok(())
}

fn required_string<'a>(value: &'a Value, key: &str, where_: &str) -> Result<&'a str, String> {
    let field = value.get(key).ok_or_else(|| format!("{where_} is missing {key}"))?;
    let text = field.as_str().ok_or_else(|| format!("{where_}.{key} must be a string"))?;
    if text.trim().is_empty() {
        return Err(format!("{where_}.{key} must not be empty"));
    }
    Ok(text)
}

fn valid_case_id(value: &str) -> bool {
    let bytes = value.as_bytes();
    bytes.len() == 7
        && &bytes[0..4] == b"XSW-"
        && bytes[4..].iter().all(u8::is_ascii_digit)
}

fn valid_evidence_id(value: &str) -> bool {
    let bytes = value.as_bytes();
    bytes.len() >= 2
        && bytes[0] == b'E'
        && bytes[1..].iter().all(u8::is_ascii_digit)
        && bytes[1] != b'0'
}

fn sha256_hex(bytes: &[u8]) -> String {
    let digest = Sha256::digest(bytes);
    digest.iter().map(|byte| format!("{byte:02x}")).collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    const PRIMARY_CORPUS: &str = include_str!("../../../docs/research/fixtures/cross-scale-wisdom-v1.jsonl");
    const PRIMARY_MANIFEST: &str = include_str!("../../../docs/research/fixtures/cross-scale-wisdom-v1.manifest.json");
    const SUPPLEMENT_CORPUS: &str = include_str!("../../../docs/research/fixtures/cross-scale-wisdom-integral-lens-v0.1.jsonl");
    const SUPPLEMENT_MANIFEST: &str = include_str!("../../../docs/research/fixtures/cross-scale-wisdom-integral-lens-v0.1.manifest.json");
    const CASE_SCHEMA: &str = include_str!("../../../docs/research/fixtures/cross-scale-wisdom-case.schema.json");

    #[test]
    fn verifies_primary_frozen_corpus_as_unscored() {
        let report = verify_bytes(PRIMARY_CORPUS.as_bytes(), PRIMARY_MANIFEST, CASE_SCHEMA).unwrap();
        assert_eq!(report["status"], REPORT_STATUS);
        assert_eq!(report["record_count"], 32);
        assert_eq!(report["all_cases_scored"], false);
    }

    #[test]
    fn verifies_integral_supplement_as_unscored() {
        let report = verify_bytes(SUPPLEMENT_CORPUS.as_bytes(), SUPPLEMENT_MANIFEST, CASE_SCHEMA).unwrap();
        assert_eq!(report["status"], REPORT_STATUS);
        assert_eq!(report["record_count"], 12);
        assert_eq!(report["severity_counts"]["hard_fail"], 9);
        assert_eq!(report["severity_counts"]["review"], 3);
    }

    #[test]
    fn rejects_changed_corpus_bytes() {
        let mut corpus = PRIMARY_CORPUS.as_bytes().to_vec();
        corpus[0] ^= 1;
        assert!(verify_bytes(&corpus, PRIMARY_MANIFEST, CASE_SCHEMA).unwrap_err().contains("SHA-256 mismatch"));
    }

    #[test]
    fn rejects_malformed_jsonl_even_with_matching_digest() {
        let corpus = b"{not json}\n";
        let manifest = minimal_manifest_for(corpus, 1, 0, 1, vec![]);
        assert!(verify_bytes(corpus, &manifest, CASE_SCHEMA).unwrap_err().contains("not valid JSON"));
    }

    #[test]
    fn rejects_duplicate_case_id() {
        let records: Vec<Value> = PRIMARY_CORPUS.lines().map(|line| serde_json::from_str(line).unwrap()).collect();
        let mut duplicate = records[1].clone();
        duplicate["case_id"] = records[0]["case_id"].clone();
        let corpus = format!("{}\n{}\n", records[0], duplicate);
        let manifest = minimal_manifest_for(corpus.as_bytes(), 2, 2, 0, vec!["DISTINGUISH_EVIDENCE_KIND".to_owned()]);
        assert!(verify_bytes(corpus.as_bytes(), &manifest, CASE_SCHEMA).unwrap_err().contains("duplicate case_id"));
    }

    #[test]
    fn rejects_missing_or_mismatched_digest() {
        let no_digest = r#"{"corpus_version":"1","corpus_id":"x","corpus_path":"x","record_count":0,"severity_counts":{"hard_fail":0,"review":0},"required_invariants":[]}"#;
        assert!(verify_bytes(b"", no_digest, CASE_SCHEMA).unwrap_err().contains("missing corpus_sha256"));
    }

    fn minimal_manifest_for(
        corpus: &[u8],
        records: u64,
        hard_fail: u64,
        review: u64,
        invariants: Vec<String>,
    ) -> String {
        let digest = sha256_hex(corpus);
        serde_json::to_string(&json!({
            "corpus_sha256": digest,
            "corpus_version": "test",
            "corpus_id": "test",
            "corpus_path": "test.jsonl",
            "record_count": records,
            "severity_counts": { "hard_fail": hard_fail, "review": review },
            "required_invariants": invariants
        })).unwrap()
    }
}
