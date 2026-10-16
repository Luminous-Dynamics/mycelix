use jsonschema::draft202012;
use serde::Deserialize;
use serde_json::Value;
use sha2::{Digest, Sha256};
use std::collections::HashSet;
use std::env;
use std::fs;
use std::path::{Path, PathBuf};

const SCHEMA_TEXT: &str =
    include_str!("../../../docs/research/schemas/wish-request-v0.1.schema.json");

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Manifest {
    manifest_version: String,
    cases: Vec<FixtureCase>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct FixtureCase {
    path: String,
    expect: String,
}

#[derive(Debug, PartialEq, Eq)]
enum Rejection {
    Schema,
    Digest,
    DuplicateIdentifier,
}

fn main() {
    if let Err(error) = run_census() {
        eprintln!("WISH-CONTRACT-CHECK: FAIL: {error}");
        std::process::exit(1);
    }
}

fn run_census() -> Result<(), String> {
    let schema: Value =
        serde_json::from_str(SCHEMA_TEXT).map_err(|e| format!("schema JSON parse: {e}"))?;

    if !draft202012::meta::is_valid(&schema) {
        return Err("request schema does not validate against the Draft 2020-12 meta-schema".into());
    }

    let validator = draft202012::options()
        .should_validate_formats(true)
        .build(&schema)
        .map_err(|e| format!("compile Draft 2020-12 schema: {e}"))?;

    let fixture_root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("fixtures");
    let manifest_path = fixture_root.join("manifest-v0.1.json");
    let manifest_text = fs::read_to_string(&manifest_path)
        .map_err(|e| format!("read {}: {e}", manifest_path.display()))?;
    let manifest: Manifest = serde_json::from_str(&manifest_text)
        .map_err(|e| format!("parse fixture manifest: {e}"))?;

    if manifest.manifest_version != "wish-contract-fixtures/0.1" {
        return Err(format!(
            "unsupported fixture manifest version: {}",
            manifest.manifest_version
        ));
    }
    if manifest.cases.is_empty() {
        return Err("fixture manifest must not be empty".into());
    }

    let mut seen_paths = HashSet::new();
    let mut passed = 0usize;
    for case in &manifest.cases {
        if !seen_paths.insert(case.path.clone()) {
            return Err(format!("duplicate fixture path in manifest: {}", case.path));
        }
        let fixture_path = safe_fixture_path(&fixture_root, &case.path)?;
        match verify_case(&validator, &fixture_path, &case.expect) {
            Ok(()) => {
                println!("PASS {} expected={}", case.path, case.expect);
                passed += 1;
            }
            Err(error) => {
                return Err(format!(
                    "{} expected={} actual=unexpected-failure: {}",
                    case.path, case.expect, error
                ));
            }
        }
    }

    println!(
        "WISH-CONTRACT-CHECK: PASS {passed}/{} declared fixtures; schema=Draft202012; formats=enabled",
        manifest.cases.len()
    );
    Ok(())
}

fn safe_fixture_path(root: &Path, relative: &str) -> Result<PathBuf, String> {
    let relative_path = Path::new(relative);
    if relative_path.is_absolute()
        || relative_path
            .components()
            .any(|component| !matches!(component, std::path::Component::Normal(_)))
    {
        return Err(format!("fixture path is not a safe relative path: {relative}"));
    }
    Ok(root.join(relative_path))
}

fn verify_case(
    validator: &jsonschema::Validator,
    fixture_path: &Path,
    expected: &str,
) -> Result<(), String> {
    let raw = fs::read_to_string(fixture_path)
        .map_err(|e| format!("read fixture {}: {e}", fixture_path.display()))?;
    let parsed = serde_json::from_str::<Value>(&raw);

    if expected == "parse_reject" {
        return if parsed.is_err() {
            Ok(())
        } else {
            Err("fixture unexpectedly parsed as JSON".into())
        };
    }

    let instance =
        parsed.map_err(|e| format!("fixture JSON is malformed but parse_reject was not expected: {e}"))?;

    let schema_result = validator.validate(&instance);
    if expected == "schema_reject" {
        return if schema_result.is_err() {
            Ok(())
        } else {
            Err("fixture unexpectedly passed schema validation".into())
        };
    }
    if let Err(error) = schema_result {
        return Err(format!("fixture failed schema validation unexpectedly: {error}"));
    }

    let semantic_result = validate_semantics(&instance);
    match expected {
        "accept" => semantic_result.map_err(|e| format!("unexpected semantic rejection: {e}")),
        "digest_reject" => match semantic_result {
            Err(Rejection::Digest) => Ok(()),
            Err(other) => Err(format!("expected digest rejection, got {other:?}")),
            Ok(()) => Err("digest-mismatch fixture unexpectedly accepted".into()),
        },
        "duplicate_id_reject" => match semantic_result {
            Err(Rejection::DuplicateIdentifier) => Ok(()),
            Err(other) => Err(format!("expected duplicate-ID rejection, got {other:?}")),
            Ok(()) => Err("duplicate-ID fixture unexpectedly accepted".into()),
        },
        unsupported => Err(format!("unsupported expected result in manifest: {unsupported}")),
    }
}

fn validate_semantics(instance: &Value) -> Result<(), Rejection> {
    ensure_unique_ids(
        instance.pointer("/goal/success_criteria"),
        "criterion_id",
    )?;
    ensure_unique_ids(instance.get("constraints"), "constraint_id")?;
    ensure_unique_ids(instance.get("resource_needs"), "need_id")?;

    let mut digest_view = instance.clone();
    let declared_digest = digest_view
        .pointer("/provenance/content_sha256")
        .and_then(Value::as_str)
        .ok_or(Rejection::Digest)?
        .to_ascii_lowercase();

    let provenance = digest_view
        .get_mut("provenance")
        .and_then(Value::as_object_mut)
        .ok_or(Rejection::Digest)?;
    provenance.remove("content_sha256");

    // RFC 8785 canonical JSON is used for the digest. Never hash pretty-printed
    // fixture bytes or rely on insertion-order JSON serialization.
    let canonical = serde_json_canonicalizer::to_vec(&digest_view)
        .map_err(|_| Rejection::Digest)?;
    let actual_digest = format!("{:x}", Sha256::digest(canonical));
    if actual_digest != declared_digest {
        return Err(Rejection::Digest);
    }

    Ok(())
}

fn ensure_unique_ids(value: Option<&Value>, field: &str) -> Result<(), Rejection> {
    let Some(items) = value.and_then(Value::as_array) else {
        // Required/object shape is the schema validator's responsibility.
        return Ok(());
    };

    let mut seen = HashSet::new();
    for item in items {
        if let Some(id) = item.get(field).and_then(Value::as_str) {
            if !seen.insert(id) {
                return Err(Rejection::DuplicateIdentifier);
            }
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn declared_contract_fixture_census_passes() {
        run_census().expect("all declared fixtures must match expected dispositions");
    }

    #[test]
    fn canonicalization_is_independent_of_object_key_order() {
        let left: Value = serde_json::from_str(r#"{"z":1,"a":{"y":2,"b":3}}"#).unwrap();
        let right: Value = serde_json::from_str(r#"{"a":{"b":3,"y":2},"z":1}"#).unwrap();
        assert_eq!(
            serde_json_canonicalizer::to_vec(&left).unwrap(),
            serde_json_canonicalizer::to_vec(&right).unwrap()
        );
    }

    #[test]
    fn fixture_manifest_rejects_path_traversal() {
        let root = Path::new("/fixture-root");
        assert!(safe_fixture_path(root, "../../etc/passwd").is_err());
        assert!(safe_fixture_path(root, "/etc/passwd").is_err());
        assert!(safe_fixture_path(root, "valid/minimal-draft.json").is_ok());
    }
}
