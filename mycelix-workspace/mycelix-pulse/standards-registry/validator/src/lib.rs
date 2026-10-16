//! Registry contract validation for Pulse's standards inventory.
//!
//! This validator intentionally checks the versioned registry contract,
//! provenance requirements, and cross-record graph. It is not a generic JSON
//! Schema implementation and does not claim standards conformance.

pub mod simulation;

use serde_json::Value;
use std::collections::BTreeSet;

const ROOT_KEYS: &[&str] = &[
    "schema_version",
    "as_of",
    "inventory_scope",
    "schema_uri",
    "entries",
];
const ENTRY_KEYS: &[&str] = &[
    "id",
    "title",
    "authority",
    "artifact_kind",
    "canonical_url",
    "version",
    "published_on",
    "publication_period",
    "last_checked",
    "source_status",
    "source_digest_sha256",
    "errata_status",
    "layers",
    "region_scope",
    "relations",
    "errata_urls",
    "review_flags",
    "support_status",
    "evidence",
    "next_review_due",
    "notes",
];
const RELATION_KEYS: &[&str] = &["type", "target_id", "source_url", "notes"];
const EVIDENCE_KEYS: &[&str] = &[
    "kind",
    "url",
    "exact_subject_sha",
    "claim",
    "verified_on",
];

const ARTIFACT_KINDS: &[&str] = &[
    "standard",
    "rfc",
    "amendment",
    "technical_specification",
    "technical_report",
    "announcement",
    "implementation_reference",
    "working_group",
    "erratum",
];
const ERRATA_STATUSES: &[&str] = &[
    "reported",
    "verified",
    "held_for_document_update",
    "rejected",
    "unknown",
];
const SOURCE_STATUSES: &[&str] = &[
    "active",
    "published",
    "proposed",
    "draft",
    "deprecated",
    "withdrawn",
    "unknown",
    "published_errata_review",
];
const SUPPORT_STATUSES: &[&str] = &[
    "unreviewed",
    "metadata_reviewed",
    "prototype",
    "profile_tested",
    "externally_certified",
    "explicitly_excluded",
];
const LAYERS: &[&str] = &[
    "physical",
    "link",
    "network",
    "transport",
    "routing",
    "delay_tolerant",
    "application",
    "security",
    "identity",
    "implementation",
    "provisioning",
];
const RELATION_TYPES: &[&str] = &[
    "updates",
    "updated_by",
    "amends",
    "amended_by",
    "errata_for",
    "profiles",
    "depends_on",
    "supersedes",
    "superseded_by",
    "related",
];
const EVIDENCE_KINDS: &[&str] = &[
    "source_review",
    "implementation",
    "test_report",
    "hardware_test",
    "interoperability_test",
    "certification",
];

/// Validate the registry against the checked-in v1 contract and optional schema.
pub fn validate_registry(registry: &Value, schema: &Value) -> Vec<String> {
    let mut errors = Vec::new();

    validate_schema_contract(schema, &mut errors);
    let Some(root) = registry.as_object() else {
        return vec!["registry: root must be a JSON object".to_owned()];
    };

    check_keys(root, ROOT_KEYS, "registry", &mut errors);
    for key in ROOT_KEYS {
        if !root.contains_key(*key) {
            errors.push(format!("registry: missing required field '{key}'"));
        }
    }

    if root.get("schema_version").and_then(Value::as_str) != Some("1.0.0") {
        errors.push("registry: schema_version must be '1.0.0'".to_owned());
    }
    check_date(root.get("as_of"), "registry.as_of", false, &mut errors);
    check_nonempty_string(root.get("inventory_scope"), "registry.inventory_scope", &mut errors);
    check_nonempty_string(root.get("schema_uri"), "registry.schema_uri", &mut errors);

    let Some(entries_value) = root.get("entries") else {
        return errors;
    };
    let Some(entries) = entries_value.as_array() else {
        errors.push("registry.entries: must be an array".to_owned());
        return errors;
    };

    let mut ids = BTreeSet::new();
    let mut parsed_entries: Vec<(&str, &Value)> = Vec::with_capacity(entries.len());

    for (index, entry) in entries.iter().enumerate() {
        let label = format!("registry.entries[{index}]");
        let Some(object) = entry.as_object() else {
            errors.push(format!("{label}: must be a JSON object"));
            continue;
        };
        check_keys(object, ENTRY_KEYS, &label, &mut errors);

        for key in [
            "id",
            "title",
            "authority",
            "artifact_kind",
            "canonical_url",
            "source_status",
            "support_status",
            "source_digest_sha256",
            "layers",
            "last_checked",
            "relations",
            "evidence",
            "notes",
        ] {
            if !object.contains_key(key) {
                errors.push(format!("{label}: missing required field '{key}'"));
            }
        }

        let id = object.get("id").and_then(Value::as_str);
        if let Some(id) = id {
            if !valid_id(id) {
                errors.push(format!("{label}.id: invalid ID '{id}'"));
            }
            if !ids.insert(id.to_owned()) {
                errors.push(format!("{label}.id: duplicate ID '{id}'"));
            }
            parsed_entries.push((id, entry));
        } else {
            errors.push(format!("{label}.id: must be a string"));
        }

        for key in ["id", "title", "authority", "notes"] {
            check_nonempty_string(object.get(key), &format!("{label}.{key}"), &mut errors);
        }
        check_url(object.get("canonical_url"), &format!("{label}.canonical_url"), &mut errors);
        check_optional_string(object.get("version"), &format!("{label}.version"), &mut errors);
        check_optional_string(
            object.get("publication_period"),
            &format!("{label}.publication_period"),
            &mut errors,
        );
        check_date(
            object.get("published_on"),
            &format!("{label}.published_on"),
            true,
            &mut errors,
        );
        check_date(
            object.get("last_checked"),
            &format!("{label}.last_checked"),
            true,
            &mut errors,
        );
        check_date(
            object.get("next_review_due"),
            &format!("{label}.next_review_due"),
            true,
            &mut errors,
        );

        check_enum(
            object.get("artifact_kind"),
            ARTIFACT_KINDS,
            &format!("{label}.artifact_kind"),
            &mut errors,
        );
        if object.get("artifact_kind").and_then(Value::as_str) == Some("erratum") {
            check_enum(
                object.get("errata_status"),
                ERRATA_STATUSES,
                &format!("{label}.errata_status"),
                &mut errors,
            );
        } else if object.contains_key("errata_status") {
            check_enum(
                object.get("errata_status"),
                ERRATA_STATUSES,
                &format!("{label}.errata_status"),
                &mut errors,
            );
        }
        check_enum(
            object.get("source_status"),
            SOURCE_STATUSES,
            &format!("{label}.source_status"),
            &mut errors,
        );
        check_enum(
            object.get("support_status"),
            SUPPORT_STATUSES,
            &format!("{label}.support_status"),
            &mut errors,
        );

        check_optional_sha256(
            object.get("source_digest_sha256"),
            &format!("{label}.source_digest_sha256"),
            &mut errors,
        );

        check_string_array(
            object.get("layers"),
            &format!("{label}.layers"),
            Some(LAYERS),
            &mut errors,
        );
        for field in ["region_scope", "review_flags"] {
            if object.contains_key(field) {
                check_string_array(
                    object.get(field),
                    &format!("{label}.{field}"),
                    None,
                    &mut errors,
                );
            }
        }
        for field in ["errata_urls"] {
            if object.contains_key(field) {
                check_url_array(object.get(field), &format!("{label}.{field}"), &mut errors);
            }
        }

        validate_relations(object.get("relations"), &label, &mut errors);
        validate_evidence(object.get("evidence"), &label, &mut errors);
        validate_support_claim(object, &label, &mut errors);
    }

    // Resolve relationships only after collecting every ID so forward references work.
    for (id, entry) in parsed_entries {
        if let Some(relations) = entry.get("relations").and_then(Value::as_array) {
            for (rel_index, relation) in relations.iter().enumerate() {
                let Some(target) = relation.get("target_id").and_then(Value::as_str) else {
                    continue;
                };
                if !ids.contains(target) {
                    errors.push(format!(
                        "registry entry '{id}' relation[{rel_index}]: target_id '{target}' does not exist"
                    ));
                }
                if target == id {
                    errors.push(format!(
                        "registry entry '{id}': self-referential relation is not allowed"
                    ));
                }
            }
        }
    }

    errors
}

fn validate_schema_contract(schema: &Value, errors: &mut Vec<String>) {
    let Some(root) = schema.as_object() else {
        errors.push("schema: root must be a JSON object".to_owned());
        return;
    };
    if root.get("$schema").and_then(Value::as_str)
        != Some("https://json-schema.org/draft/2020-12/schema")
    {
        errors.push("schema: expected JSON Schema draft 2020-12".to_owned());
    }
    if root.get("type").and_then(Value::as_str) != Some("object") {
        errors.push("schema: root type must be object".to_owned());
    }
    let required = root.get("required").and_then(Value::as_array);
    for name in ["schema_version", "as_of", "inventory_scope", "schema_uri", "entries"] {
        if !required.is_some_and(|list| list.iter().any(|v| v.as_str() == Some(name))) {
            errors.push(format!("schema: required list is missing '{name}'"));
        }
    }
    if root.pointer("/properties/schema_version/const").and_then(Value::as_str) != Some("1.0.0") {
        errors.push("schema: schema_version const must be '1.0.0'".to_owned());
    }
    if root.pointer("/$defs/entry").and_then(Value::as_object).is_none() {
        errors.push("schema: missing $defs.entry object".to_owned());
    }
    let entry_required = root.pointer("/$defs/entry/required").and_then(Value::as_array);
    if !entry_required.is_some_and(|list| {
        list.iter().any(|value| value.as_str() == Some("source_digest_sha256"))
    }) {
        errors.push("schema: entry required list is missing source_digest_sha256".to_owned());
    }
    if root.pointer("/$defs/entry/properties/source_digest_sha256").is_none() {
        errors.push("schema: missing source_digest_sha256 property".to_owned());
    }
    let errata_enum = root
        .pointer("/$defs/entry/properties/errata_status/enum")
        .and_then(Value::as_array);
    for value in ERRATA_STATUSES {
        if !errata_enum.is_some_and(|items| items.iter().any(|item| item.as_str() == Some(value))) {
            errors.push(format!("schema: errata_status enum is missing '{value}'"));
        }
    }
    if root.pointer("/$defs/entry/properties/errata_status").is_none() {
        errors.push("schema: missing errata_status property".to_owned());
    }
    if root.pointer("/$defs/relation").and_then(Value::as_object).is_none() {
        errors.push("schema: missing $defs.relation object".to_owned());
    }
}

fn check_keys(
    object: &serde_json::Map<String, Value>,
    allowed: &[&str],
    label: &str,
    errors: &mut Vec<String>,
) {
    for key in object.keys() {
        if !allowed.contains(&key.as_str()) {
            errors.push(format!("{label}: unknown field '{key}'"));
        }
    }
}

fn check_nonempty_string(value: Option<&Value>, label: &str, errors: &mut Vec<String>) {
    match value.and_then(Value::as_str) {
        Some(text) if !text.trim().is_empty() => {}
        _ => errors.push(format!("{label}: must be a non-empty string")),
    }
}

fn check_optional_string(value: Option<&Value>, label: &str, errors: &mut Vec<String>) {
    match value {
        Some(Value::Null) => {}
        Some(Value::String(_)) => {}
        Some(_) => errors.push(format!("{label}: must be a string or null")),
        None => {}
    }
}

fn check_optional_sha256(value: Option<&Value>, label: &str, errors: &mut Vec<String>) {
    match value {
        Some(Value::Null) => {}
        Some(Value::String(text))
            if text.len() == 64
                && text
                    .bytes()
                    .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte)) => {}
        Some(_) => errors.push(format!(
            "{label}: must be null or a lowercase 64-character SHA-256 hex digest"
        )),
        None => errors.push(format!("{label}: field is required; use null when not captured")),
    }
}

fn check_enum(value: Option<&Value>, accepted: &[&str], label: &str, errors: &mut Vec<String>) {
    match value.and_then(Value::as_str) {
        Some(text) if accepted.contains(&text) => {}
        Some(text) => errors.push(format!("{label}: unsupported value '{text}'")),
        None => errors.push(format!("{label}: must be a string")),
    }
}

fn check_string_array(
    value: Option<&Value>,
    label: &str,
    accepted: Option<&[&str]>,
    errors: &mut Vec<String>,
) {
    let Some(values) = value.and_then(Value::as_array) else {
        errors.push(format!("{label}: must be an array"));
        return;
    };
    let mut seen = BTreeSet::new();
    for (index, value) in values.iter().enumerate() {
        match value.as_str() {
            Some(text) => {
                if let Some(accepted) = accepted {
                    if !accepted.contains(&text) {
                        errors.push(format!("{label}[{index}]: unsupported value '{text}'"));
                    }
                }
                if !seen.insert(text.to_owned()) {
                    errors.push(format!("{label}[{index}]: duplicate value '{text}'"));
                }
            }
            None => errors.push(format!("{label}[{index}]: must be a string")),
        }
    }
}

fn check_url(value: Option<&Value>, label: &str, errors: &mut Vec<String>) {
    match value.and_then(Value::as_str) {
        Some(text) if valid_url(text) => {}
        _ => errors.push(format!("{label}: must be an absolute HTTP(S) URL")),
    }
}

fn check_url_array(value: Option<&Value>, label: &str, errors: &mut Vec<String>) {
    let Some(values) = value.and_then(Value::as_array) else {
        errors.push(format!("{label}: must be an array"));
        return;
    };
    for (index, value) in values.iter().enumerate() {
        if !value.as_str().is_some_and(valid_url) {
            errors.push(format!("{label}[{index}]: must be an absolute HTTP(S) URL"));
        }
    }
}

fn valid_url(text: &str) -> bool {
    (text.starts_with("https://") || text.starts_with("http://"))
        && !text.chars().any(char::is_whitespace)
        && text.split_once("://").is_some_and(|(_, rest)| !rest.is_empty())
}

fn check_date(
    value: Option<&Value>,
    label: &str,
    nullable: bool,
    errors: &mut Vec<String>,
) {
    match value {
        Some(Value::Null) if nullable => {}
        Some(Value::String(text)) if valid_iso_date(text) => {}
        Some(Value::Null) => errors.push(format!("{label}: null is not allowed")),
        Some(_) => errors.push(format!("{label}: must be a valid YYYY-MM-DD date")),
        None if nullable => {}
        None => errors.push(format!("{label}: missing required date")),
    }
}

fn valid_iso_date(text: &str) -> bool {
    let bytes = text.as_bytes();
    if bytes.len() != 10
        || bytes[4] != b'-'
        || bytes[7] != b'-'
        || !bytes[0..4].iter().all(u8::is_ascii_digit)
        || !bytes[5..7].iter().all(u8::is_ascii_digit)
        || !bytes[8..10].iter().all(u8::is_ascii_digit)
    {
        return false;
    }
    let (Ok(year), Ok(month), Ok(day)) = (
        text[0..4].parse::<i32>(),
        text[5..7].parse::<u32>(),
        text[8..10].parse::<u32>(),
    ) else {
        return false;
    };
    if year < 1 || !(1..=12).contains(&month) {
        return false;
    }
    let leap = year % 4 == 0 && (year % 100 != 0 || year % 400 == 0);
    let days_in_month = match month {
        1 | 3 | 5 | 7 | 8 | 10 | 12 => 31,
        4 | 6 | 9 | 11 => 30,
        2 if leap => 29,
        2 => 28,
        _ => return false,
    };
    (1..=days_in_month).contains(&day)
}

fn valid_id(id: &str) -> bool {
    !id.is_empty()
        && id
            .split(|ch| ch == '.' || ch == '-')
            .all(|part| {
                !part.is_empty()
                    && part
                        .chars()
                        .all(|ch| ch.is_ascii_lowercase() || ch.is_ascii_digit())
            })
}

fn validate_relations(value: Option<&Value>, label: &str, errors: &mut Vec<String>) {
    let Some(relations) = value.and_then(Value::as_array) else {
        errors.push(format!("{label}.relations: must be an array"));
        return;
    };
    for (index, relation) in relations.iter().enumerate() {
        let prefix = format!("{label}.relations[{index}]");
        let Some(object) = relation.as_object() else {
            errors.push(format!("{prefix}: must be a JSON object"));
            continue;
        };
        check_keys(object, RELATION_KEYS, &prefix, errors);
        for field in ["type", "target_id", "source_url"] {
            if !object.contains_key(field) {
                errors.push(format!("{prefix}: missing required field '{field}'"));
            }
        }
        check_enum(object.get("type"), RELATION_TYPES, &format!("{prefix}.type"), errors);
        check_nonempty_string(object.get("target_id"), &format!("{prefix}.target_id"), errors);
        check_url(object.get("source_url"), &format!("{prefix}.source_url"), errors);
        if let Some(notes) = object.get("notes") {
            if !notes.is_string() {
                errors.push(format!("{prefix}.notes: must be a string"));
            }
        }
    }
}

fn validate_evidence(value: Option<&Value>, label: &str, errors: &mut Vec<String>) {
    let Some(evidence) = value.and_then(Value::as_array) else {
        errors.push(format!("{label}.evidence: must be an array"));
        return;
    };
    for (index, item) in evidence.iter().enumerate() {
        let prefix = format!("{label}.evidence[{index}]");
        let Some(object) = item.as_object() else {
            errors.push(format!("{prefix}: must be a JSON object"));
            continue;
        };
        check_keys(object, EVIDENCE_KEYS, &prefix, errors);
        for field in ["kind", "url", "claim"] {
            if !object.contains_key(field) {
                errors.push(format!("{prefix}: missing required field '{field}'"));
            }
        }
        check_enum(object.get("kind"), EVIDENCE_KINDS, &format!("{prefix}.kind"), errors);
        check_url(object.get("url"), &format!("{prefix}.url"), errors);
        check_nonempty_string(object.get("claim"), &format!("{prefix}.claim"), errors);
        check_optional_string(
            object.get("exact_subject_sha"),
            &format!("{prefix}.exact_subject_sha"),
            errors,
        );
        check_date(object.get("verified_on"), &format!("{prefix}.verified_on"), true, errors);
    }
}

fn validate_support_claim(
    entry: &serde_json::Map<String, Value>,
    label: &str,
    errors: &mut Vec<String>,
) {
    let status = entry.get("support_status").and_then(Value::as_str);
    let evidence = entry.get("evidence").and_then(Value::as_array);
    let has_evidence = |kind: &str| {
        evidence.is_some_and(|items| {
            items
                .iter()
                .any(|item| item.get("kind").and_then(Value::as_str) == Some(kind))
        })
    };

    match status {
        Some("unreviewed") | Some("explicitly_excluded") => {}
        Some("metadata_reviewed") if !has_evidence("source_review") => {
            errors.push(format!(
                "{label}: metadata_reviewed requires source_review evidence"
            ));
        }
        Some("prototype") if !has_evidence("implementation") => {
            errors.push(format!("{label}: prototype requires implementation evidence"));
        }
        Some("profile_tested")
            if !has_evidence("test_report")
                && !has_evidence("hardware_test")
                && !has_evidence("interoperability_test") =>
        {
            errors.push(format!(
                "{label}: profile_tested requires test_report, hardware_test, or interoperability_test evidence"
            ));
        }
        Some("externally_certified") if !has_evidence("certification") => {
            errors.push(format!(
                "{label}: externally_certified requires certification evidence"
            ));
        }
        _ => {}
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    fn seed() -> Value {
        serde_json::from_str(include_str!("../../registry.v1.json"))
            .expect("checked-in standards registry must parse")
    }

    fn schema() -> Value {
        serde_json::from_str(include_str!("../../schema.v1.json"))
            .expect("checked-in registry schema must parse")
    }

    #[test]
    fn checked_in_seed_passes_registry_contract() {
        let errors = validate_registry(&seed(), &schema());
        assert!(errors.is_empty(), "unexpected errors: {errors:#?}");
    }

    #[test]
    fn duplicate_ids_fail_closed() {
        let mut registry = seed();
        let duplicate = registry["entries"][0].clone();
        registry["entries"].as_array_mut().unwrap().push(duplicate);
        let errors = validate_registry(&registry, &schema());
        assert!(errors.iter().any(|e| e.contains("duplicate ID")));
    }

    #[test]
    fn dangling_relationship_fails_closed() {
        let mut registry = seed();
        registry["entries"][0]["relations"] = json!([{
            "type": "updates",
            "target_id": "missing.target",
            "source_url": "https://example.org/source"
        }]);
        let errors = validate_registry(&registry, &schema());
        assert!(errors.iter().any(|e| e.contains("does not exist")));
    }

    #[test]
    fn unsupported_claim_without_evidence_fails_closed() {
        let mut registry = seed();
        registry["entries"][0]["support_status"] = json!("externally_certified");
        registry["entries"][0]["evidence"] = json!([]);
        let errors = validate_registry(&registry, &schema());
        assert!(errors.iter().any(|e| e.contains("requires certification evidence")));
    }

    #[test]
    fn malformed_date_is_rejected() {
        let mut registry = seed();
        registry["as_of"] = json!("2026-02-30");
        let errors = validate_registry(&registry, &schema());
        assert!(errors.iter().any(|e| e.contains("registry.as_of")));
    }

    #[test]
    fn unknown_fields_are_rejected() {
        let mut registry = seed();
        registry["entries"][0]["support_everything"] = json!(true);
        let errors = validate_registry(&registry, &schema());
        assert!(errors.iter().any(|e| e.contains("unknown field 'support_everything'")));
    }

    #[test]
    fn self_referential_relationship_is_rejected() {
        let mut registry = seed();
        let id = registry["entries"][0]["id"].as_str().unwrap().to_owned();
        registry["entries"][0]["relations"] = json!([{
            "type": "related",
            "target_id": id,
            "source_url": "https://example.org/source"
        }]);
        let errors = validate_registry(&registry, &schema());
        assert!(errors.iter().any(|e| e.contains("self-referential relation")));
    }

    #[test]
    fn profile_tested_requires_test_evidence() {
        let mut registry = seed();
        registry["entries"][0]["support_status"] = json!("profile_tested");
        registry["entries"][0]["evidence"] = json!([{
            "kind": "source_review",
            "url": "https://example.org/source",
            "exact_subject_sha": null,
            "claim": "Reviewed source metadata only.",
            "verified_on": "2026-10-15"
        }]);
        let errors = validate_registry(&registry, &schema());
        assert!(errors.iter().any(|e| e.contains("requires test_report")));
    }

    #[test]
    fn malformed_source_digest_is_rejected() {
        let mut registry = seed();
        registry["entries"][0]["source_digest_sha256"] = json!("abc123");
        let errors = validate_registry(&registry, &schema());
        assert!(errors.iter().any(|error| error.contains("source_digest_sha256")));
    }

    #[test]
    fn schema_missing_a_required_field_is_rejected() {
        let registry = seed();
        let mut schema = schema();
        schema["required"] = json!(["schema_version"]);
        let errors = validate_registry(&registry, &schema);
        assert!(errors.iter().any(|e| e.contains("schema: required list is missing 'entries'")));
    }

    #[test]
    fn erratum_requires_explicit_disposition() {
        let mut registry = seed();
        let entry = &mut registry["entries"][0];
        entry["artifact_kind"] = json!("erratum");
        entry.as_object_mut().unwrap().remove("errata_status");
        let errors = validate_registry(&registry, &schema());
        assert!(errors.iter().any(|error| error.contains("errata_status")));
    }

    #[test]
    fn unsupported_errata_disposition_is_rejected() {
        let mut registry = seed();
        registry["entries"][0]["errata_status"] = json!("maybe");
        let errors = validate_registry(&registry, &schema());
        assert!(errors.iter().any(|error| error.contains("errata_status")));
    }

    #[test]
    fn schema_without_errata_dispositions_is_rejected() {
        let registry = seed();
        let mut schema = schema();
        schema["$defs"]["entry"]["properties"].as_object_mut().unwrap().remove("errata_status");
        let errors = validate_registry(&registry, &schema);
        assert!(errors.iter().any(|error| error.contains("errata_status property")));
    }

    #[test]
    fn invalid_relation_type_is_rejected() {
        let mut registry = seed();
        registry["entries"][0]["relations"] = json!([{
            "type": "magic_compatibility",
            "target_id": "ietf.rfc-9171",
            "source_url": "https://example.org/source"
        }]);
        let errors = validate_registry(&registry, &schema());
        assert!(errors.iter().any(|e| e.contains("unsupported value 'magic_compatibility'")));
    }
}
