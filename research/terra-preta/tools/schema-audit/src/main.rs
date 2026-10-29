//! Validate JSON Schema documents and optionally validate JSON instances against them.
//!
//! Uses the Rust jsonschema validator with default features disabled so schema
//! validation is offline by default; externally hosted $ref targets are not needed
//! by the Terra Preta contracts and must not silently become network dependencies.

use serde_json::Value;
use std::env;
use std::error::Error;
use std::fs;
use std::path::Path;

const SCHEMA_FIXTURES: &[(&str, &str)] = &[
    ("batch-record", include_str!("../../../batch-record.schema.json")),
    (
        "contrast-record",
        include_str!("../../../contrast-record.schema.json"),
    ),
    (
        "split-manifest",
        include_str!("../../../split-manifest.schema.json"),
    ),
    (
        "biochar-audit-report",
        include_str!("../../biochar-audit/audit-report.schema.json"),
    ),
    (
        "prediction-audit-report",
        include_str!("../../prediction-audit/prediction-report.schema.json"),
    ),
    (
        "split-audit-report",
        include_str!("../../split-audit/split-audit-report.schema.json"),
    ),
    (
        "source-register",
        include_str!("../../../source-register.schema.json"),
    ),
];

const SOURCE_REGISTER: &str = include_str!("../../../source-register.v0.1.json");

fn validate_schema(schema_bytes: &[u8], label: &str) -> Result<Value, Box<dyn Error>> {
    let schema: Value = serde_json::from_slice(schema_bytes)
        .map_err(|error| format!("{label}: invalid JSON: {error}"))?;
    jsonschema::meta::validate(&schema)
        .map_err(|error| format!("{label}: invalid JSON Schema: {error}"))?;
    jsonschema::options()
        .should_validate_formats(true)
        .build(&schema)
        .map_err(|error| format!("{label}: cannot build validator: {error}"))?;
    Ok(schema)
}

fn validate_instance(
    schema: &Value,
    instance_bytes: &[u8],
    schema_label: &str,
    instance_label: &str,
) -> Result<(), Box<dyn Error>> {
    let instance: Value = serde_json::from_slice(instance_bytes)
        .map_err(|error| format!("{instance_label}: invalid JSON: {error}"))?;
    let validator = jsonschema::options()
        .should_validate_formats(true)
        .build(schema)
        .map_err(|error| format!("{schema_label}: cannot build validator: {error}"))?;
    let errors: Vec<String> = validator
        .iter_errors(&instance)
        .map(|error| format!("{}: {}", error.instance_path(), error))
        .collect();
    if !errors.is_empty() {
        return Err(format!(
            "{instance_label} does not conform to {schema_label}: {}",
            errors.join("; ")
        )
        .into());
    }
    Ok(())
}

fn run() -> Result<(), Box<dyn Error>> {
    let arguments: Vec<String> = env::args().collect();
    if arguments.len() != 2 && arguments.len() != 3 {
        return Err(format!(
            "usage: {} <schema.json> [instance.json]",
            arguments
                .first()
                .map(String::as_str)
                .unwrap_or("terra-preta-schema-audit")
        )
        .into());
    }

    let schema_path = Path::new(&arguments[1]);
    let schema_bytes = fs::read(schema_path)?;
    let schema = validate_schema(&schema_bytes, &schema_path.display().to_string())?;
    println!("schema is valid JSON and conforms to its declared JSON Schema draft");

    if let Some(instance_arg) = arguments.get(2) {
        let instance_path = Path::new(instance_arg);
        let instance_bytes = fs::read(instance_path)?;
        validate_instance(
            &schema,
            &instance_bytes,
            &schema_path.display().to_string(),
            &instance_path.display().to_string(),
        )?;
        println!("instance conforms to schema");
    }
    Ok(())
}

fn main() {
    if let Err(error) = run() {
        eprintln!("schema audit failed closed: {error}");
        std::process::exit(2);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    #[test]
    fn all_committed_schemas_parse_and_validate_against_their_meta_schema() {
        for (name, text) in SCHEMA_FIXTURES {
            let schema: Value =
                serde_json::from_str(text).unwrap_or_else(|error| panic!("{name}: {error}"));
            jsonschema::meta::validate(&schema)
                .unwrap_or_else(|error| panic!("{name} is not a valid JSON Schema: {error}"));
            jsonschema::validator_for(&schema)
                .unwrap_or_else(|error| panic!("{name} cannot build validator: {error}"));
        }
    }

    fn split_report_schema() -> Value {
        serde_json::from_str(SCHEMA_FIXTURES[5].1).unwrap()
    }

    fn valid_split_report() -> Value {
        json!({
            "audit_version": "0.1.0",
            "status": "passed",
            "input_label": "fixture.csv",
            "input_sha256": "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
            "grouping_key": "source_study_id",
            "row_count": 5,
            "distinct_group_count": 4,
            "group_assignment_sha256": "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb",
            "split_counts": {
                "train": {"rows": 2, "groups": 1},
                "validation": {"rows": 1, "groups": 1},
                "test": {"rows": 1, "groups": 1},
                "excluded": {"rows": 1, "groups": 1}
            },
            "warnings": ["structural checks cannot prove scientific independence"],
            "interpretation_boundary": "synthetic schema fixture only"
        })
    }

    #[test]
    fn source_register_conforms_to_its_schema() {
        let schema: Value = serde_json::from_str(SCHEMA_FIXTURES[6].1).unwrap();
        let validator = jsonschema::options()
            .should_validate_formats(true)
            .build(&schema)
            .unwrap();
        let register: Value = serde_json::from_str(SOURCE_REGISTER).unwrap();
        assert!(validator.is_valid(&register));
    }

    #[test]
    fn rejects_source_register_with_invalid_date_format() {
        let schema: Value = serde_json::from_str(SCHEMA_FIXTURES[6].1).unwrap();
        let validator = jsonschema::options()
            .should_validate_formats(true)
            .build(&schema)
            .unwrap();
        let mut register: Value = serde_json::from_str(SOURCE_REGISTER).unwrap();
        register["reviewed_on"] = json!("29-10-2026");
        assert!(!validator.is_valid(&register));
    }

    #[test]
    fn accepts_valid_split_audit_report_instance() {
        let schema = split_report_schema();
        let validator = jsonschema::validator_for(&schema).unwrap();
        assert!(validator.is_valid(&valid_split_report()));
    }

    #[test]
    fn rejects_report_with_invalid_status() {
        let schema = split_report_schema();
        let validator = jsonschema::validator_for(&schema).unwrap();
        let mut report = valid_split_report();
        report["status"] = json!("warning");
        assert!(!validator.is_valid(&report));
    }

    #[test]
    fn rejects_report_with_placeholder_digest() {
        let schema = split_report_schema();
        let validator = jsonschema::validator_for(&schema).unwrap();
        let mut report = valid_split_report();
        report["input_sha256"] = json!("not-a-sha256");
        assert!(!validator.is_valid(&report));
    }
}
