use std::{
    collections::HashSet,
    env,
    fs,
    process::ExitCode,
};

use serde_json::Value;

fn main() -> ExitCode {
    let args: Vec<String> = env::args().collect();
    if args.len() != 3 {
        eprintln!(
            "usage: {} <physical-artwork-extension.schema.json> <passport.json>",
            args.first().map(String::as_str).unwrap_or("physical-artwork-conformance")
        );
        return ExitCode::from(2);
    }

    let schema = match read_json(&args[1]) {
        Ok(value) => value,
        Err(error) => {
            eprintln!("schema input error: {error}");
            return ExitCode::from(2);
        }
    };
    let instance = match read_json(&args[2]) {
        Ok(value) => value,
        Err(error) => {
            eprintln!("passport input error: {error}");
            return ExitCode::from(2);
        }
    };

    match validate_passport(&schema, &instance) {
        Ok(()) => {
            println!("VALID: JSON Schema and local reference-integrity checks passed");
            ExitCode::SUCCESS
        }
        Err(errors) => {
            eprintln!("INVALID: {} issue(s)", errors.len());
            for error in errors {
                eprintln!("- {error}");
            }
            ExitCode::FAILURE
        }
    }
}

fn read_json(path: &str) -> Result<Value, String> {
    let content = fs::read_to_string(path).map_err(|error| format!("{path}: {error}"))?;
    serde_json::from_str(&content).map_err(|error| format!("{path}: invalid JSON: {error}"))
}

/// Validates both the declared Draft 2020-12 schema and the CSCP-specific
/// references that JSON Schema cannot express as ordinary local constraints.
/// This performs no network lookup and never mutates the source record.
fn validate_passport(schema: &Value, instance: &Value) -> Result<(), Vec<String>> {
    let mut errors = Vec::new();

    if !jsonschema::draft202012::meta::is_valid(schema) {
        errors.push("schema is not valid under the Draft 2020-12 meta-schema".to_owned());
        return Err(errors);
    }

    let validator = match jsonschema::draft202012::options()
        .should_validate_formats(true)
        .build(schema)
    {
        Ok(validator) => validator,
        Err(error) => {
            errors.push(format!("could not compile Draft 2020-12 schema: {error}"));
            return Err(errors);
        }
    };

    for error in validator.iter_errors(instance) {
        errors.push(format!(
            "schema violation at {}: {}",
            error.instance_path(),
            error
        ));
    }

    validate_references(instance, &mut errors);

    if errors.is_empty() {
        Ok(())
    } else {
        Err(errors)
    }
}

fn ids(items: Option<&Value>, key: &str, label: &str, errors: &mut Vec<String>) -> HashSet<String> {
    let mut found = HashSet::new();
    let Some(items) = items.and_then(Value::as_array) else {
        return found;
    };

    for (index, item) in items.iter().enumerate() {
        let Some(id) = item.get(key).and_then(Value::as_str) else {
            continue;
        };
        if !found.insert(id.to_owned()) {
            errors.push(format!("duplicate {label} identifier at index {index}: {id}"));
        }
    }
    found
}

fn check_refs(
    owner: &Value,
    field: &str,
    known: &HashSet<String>,
    owner_path: &str,
    errors: &mut Vec<String>,
) {
    let Some(references) = owner.get(field).and_then(Value::as_array) else {
        return;
    };

    for (index, reference) in references.iter().enumerate() {
        let Some(reference) = reference.as_str() else {
            continue;
        };
        if !known.contains(reference) {
            errors.push(format!(
                "{owner_path}.{field}[{index}] references unknown identifier: {reference}"
            ));
        }
    }
}

fn check_optional_ref(
    owner: &Value,
    field: &str,
    known: &HashSet<String>,
    owner_path: &str,
    errors: &mut Vec<String>,
) {
    let Some(reference) = owner.get(field).and_then(Value::as_str) else {
        return;
    };
    if !known.contains(reference) {
        errors.push(format!(
            "{owner_path}.{field} references unknown identifier: {reference}"
        ));
    }
}

fn validate_references(instance: &Value, errors: &mut Vec<String>) {
    let evidence_ids = ids(instance.get("evidence"), "evidenceId", "evidence", errors);
    let assertion_ids = ids(
        instance.get("assertions"),
        "assertionId",
        "assertion",
        errors,
    );
    let _event_ids = ids(instance.get("events"), "eventId", "event", errors);
    let rights_ids = ids(instance.get("rights"), "rightId", "right", errors);
    let _condition_ids = ids(
        instance.get("conditionObservations"),
        "observationId",
        "condition observation",
        errors,
    );
    let _spatial_ids = ids(
        instance.get("spatialAssets"),
        "spatialAssetId",
        "spatial asset",
        errors,
    );

    if let Some(evidence) = instance.get("evidence").and_then(Value::as_array) {
        for (index, item) in evidence.iter().enumerate() {
            check_refs(
                item,
                "rightsRef",
                &rights_ids,
                &format!("evidence[{index}]"),
                errors,
            );
        }
    }

    if let Some(assertions) = instance.get("assertions").and_then(Value::as_array) {
        for (index, assertion) in assertions.iter().enumerate() {
            let path = format!("assertions[{index}]");
            check_refs(assertion, "evidenceRefs", &evidence_ids, &path, errors);
            check_refs(assertion, "supersedes", &assertion_ids, &path, errors);
        }
    }

    if let Some(events) = instance.get("events").and_then(Value::as_array) {
        for (index, event) in events.iter().enumerate() {
            let path = format!("events[{index}]");
            check_refs(event, "evidenceRefs", &evidence_ids, &path, errors);
            check_refs(event, "assertionRefs", &assertion_ids, &path, errors);
        }
    }

    if let Some(rights) = instance.get("rights").and_then(Value::as_array) {
        for (index, right) in rights.iter().enumerate() {
            check_optional_ref(
                right,
                "sourceAssertionRef",
                &assertion_ids,
                &format!("rights[{index}]"),
                errors,
            );
        }
    }

    if let Some(observations) = instance
        .get("conditionObservations")
        .and_then(Value::as_array)
    {
        for (index, observation) in observations.iter().enumerate() {
            check_refs(
                observation,
                "evidenceRefs",
                &evidence_ids,
                &format!("conditionObservations[{index}]"),
                errors,
            );
        }
    }

    if let Some(spatial_assets) = instance.get("spatialAssets").and_then(Value::as_array) {
        for (index, asset) in spatial_assets.iter().enumerate() {
            check_refs(
                asset,
                "rightsRefs",
                &rights_ids,
                &format!("spatialAssets[{index}]"),
                errors,
            );
        }
    }

    if let Some(artwork) = instance.get("artwork") {
        if let Some(makers) = artwork.get("makers").and_then(Value::as_array) {
            for (index, maker) in makers.iter().enumerate() {
                check_optional_ref(
                    maker,
                    "claimRef",
                    &assertion_ids,
                    &format!("artwork.makers[{index}]"),
                    errors,
                );
            }
        }
        for field in ["dateOrPeriod", "dimensions", "edition"] {
            if let Some(section) = artwork.get(field) {
                check_optional_ref(
                    section,
                    "claimRef",
                    &assertion_ids,
                    &format!("artwork.{field}"),
                    errors,
                );
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SCHEMA_TEXT: &str =
        include_str!("../../../specs/creative-assets/physical-artwork-extension.schema.json");
    const EXAMPLE_TEXT: &str =
        include_str!("../../../specs/creative-assets/examples/synthetic-sculpture.json");

    fn documents() -> (Value, Value) {
        (
            serde_json::from_str(SCHEMA_TEXT).expect("schema JSON parses"),
            serde_json::from_str(EXAMPLE_TEXT).expect("synthetic example JSON parses"),
        )
    }

    #[test]
    fn synthetic_sculpture_passes_schema_and_reference_checks() {
        let (schema, instance) = documents();
        assert_eq!(validate_passport(&schema, &instance), Ok(()));
    }

    #[test]
    fn rejects_missing_required_artwork_identity() {
        let (schema, mut instance) = documents();
        instance["artwork"]
            .as_object_mut()
            .expect("artwork object")
            .remove("artworkId");
        assert!(validate_passport(&schema, &instance).is_err());
    }

    #[test]
    fn rejects_unknown_root_property() {
        let (schema, mut instance) = documents();
        instance["unreviewedField"] = Value::String("must not be silently accepted".into());
        assert!(validate_passport(&schema, &instance).is_err());
    }

    #[test]
    fn rejects_malformed_content_digest() {
        let (schema, mut instance) = documents();
        instance["evidence"][0]["digest"]["value"] = Value::String("not-a-sha256".into());
        assert!(validate_passport(&schema, &instance).is_err());
    }

    #[test]
    fn rejects_dangling_evidence_reference() {
        let (schema, mut instance) = documents();
        instance["assertions"][0]["evidenceRefs"] =
            serde_json::json!(["evidence-that-does-not-exist"]);
        assert!(validate_passport(&schema, &instance).is_err());
    }

    #[test]
    fn rejects_duplicate_assertion_identifiers() {
        let (schema, mut instance) = documents();
        let duplicate = instance["assertions"][0].clone();
        instance["assertions"]
            .as_array_mut()
            .expect("assertions array")
            .push(duplicate);
        assert!(validate_passport(&schema, &instance).is_err());
    }

    #[test]
    fn rejects_evidence_with_dangling_rights_reference() {
        let (schema, mut instance) = documents();
        instance["evidence"][0]["rightsRef"] =
            serde_json::json!(["right-that-does-not-exist"]);
        assert!(validate_passport(&schema, &instance).is_err());
    }

    #[test]
    fn rejects_right_grant_with_missing_source_claim() {
        let (schema, mut instance) = documents();
        instance["rights"][0]["sourceAssertionRef"] =
            Value::String("missing-claim".into());
        assert!(validate_passport(&schema, &instance).is_err());
    }
}
