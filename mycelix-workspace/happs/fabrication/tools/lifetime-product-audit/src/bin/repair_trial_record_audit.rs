use jsonschema::draft202012;
use serde_json::{Value, json};
use sha2::{Digest, Sha256};
use std::env;
use std::fs;
use std::path::PathBuf;

fn main() {
    match parse_args() {
        Ok(Some((schema_path, record_path))) => match audit_files(&schema_path, &record_path) {
            Ok(report) => {
                let failed = report.get("status").and_then(Value::as_str) != Some("PASS");
                println!(
                    "{}",
                    serde_json::to_string_pretty(&report).unwrap_or_else(|_| "{}".into())
                );
                if failed {
                    std::process::exit(1);
                }
            }
            Err(error) => {
                eprintln!("repair-trial-record-audit: {error}");
                std::process::exit(2);
            }
        },
        Ok(None) => {}
        Err(error) => {
            eprintln!("repair-trial-record-audit: {error}");
            eprintln!("Run with --help for usage.");
            std::process::exit(2);
        }
    }
}

fn parse_args() -> Result<Option<(PathBuf, PathBuf)>, String> {
    let mut schema = None;
    let mut record = None;
    let mut args = env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--help" | "-h" => {
                println!(
                    "Repair-trial record audit (draft)\n\
                     \n\
                     Usage: repair_trial_record_audit --schema PATH --record PATH\n\
                     \n\
                     Validates a draft repair-trial record and its schema. \
                     It does not validate a physical product or authorize energized tests."
                );
                return Ok(None);
            }
            "--schema" => {
                schema = Some(PathBuf::from(
                    args.next().ok_or("--schema requires a path")?,
                ));
            }
            "--record" => {
                record = Some(PathBuf::from(
                    args.next().ok_or("--record requires a path")?,
                ));
            }
            other => return Err(format!("unknown argument: {other}")),
        }
    }
    Ok(Some((
        schema.ok_or("--schema is required")?,
        record.ok_or("--record is required")?,
    )))
}

fn read_json(path: &PathBuf) -> Result<(Value, Vec<u8>), String> {
    let bytes =
        fs::read(path).map_err(|error| format!("cannot read {}: {error}", path.display()))?;
    let value = serde_json::from_slice::<Value>(&bytes)
        .map_err(|error| format!("invalid JSON in {}: {error}", path.display()))?;
    Ok((value, bytes))
}

fn audit_files(schema_path: &PathBuf, record_path: &PathBuf) -> Result<Value, String> {
    let (schema, schema_bytes) = read_json(schema_path)?;
    let (record, record_bytes) = read_json(record_path)?;
    let mut issues = Vec::new();
    let schema_meta_valid = draft202012::meta::validate(&schema).is_ok();
    if !schema_meta_valid {
        issues.push("schema is not valid JSON Schema Draft 2020-12".to_string());
    }

    let record_schema_valid = match draft202012::options()
        .should_validate_formats(true)
        .build(&schema)
    {
        Ok(validator) => {
            let validation_errors: Vec<String> = validator
                .iter_errors(&record)
                .map(|error| format!("{}: {}", error.instance_path(), error))
                .collect();
            let valid = validation_errors.is_empty();
            issues.extend(
                validation_errors
                    .into_iter()
                    .map(|error| format!("schema validation: {error}")),
            );
            valid
        }
        Err(error) => {
            issues.push(format!("could not compile record schema: {error}"));
            false
        }
    };

    issues.extend(validate_record_semantics(&record));
    let status = if issues.is_empty() { "PASS" } else { "FAIL" };
    Ok(json!({
        "tool": "repair-trial-record-audit",
        "tool_version": env!("CARGO_PKG_VERSION"),
        "status": status,
        "scope": "draft record shape and unpowered-trial semantic checks only",
        "checks": {
            "schema_meta_valid": schema_meta_valid,
            "record_schema_valid": record_schema_valid,
            "physical_trial_performed": record.get("record_state").and_then(Value::as_str) == Some("performed"),
            "physical_safety_or_conformity_established": false,
            "durability_established": false
        },
        "input_digests_sha256": {
            "schema": {
                "path": schema_path.display().to_string(),
                "sha256": sha256_hex(&schema_bytes)
            },
            "record": {
                "path": record_path.display().to_string(),
                "sha256": sha256_hex(&record_bytes)
            }
        },
        "error_count": issues.len(),
        "issues": issues
    }))
}

fn validate_record_semantics(record: &Value) -> Vec<String> {
    let mut issues = Vec::new();
    let record_state = record
        .get("record_state")
        .and_then(Value::as_str)
        .unwrap_or("");
    let fixture_state = record
        .pointer("/fixture/fixture_state")
        .and_then(Value::as_str)
        .unwrap_or("");
    let outcome = record.get("outcome");
    let outcome_status = outcome
        .and_then(|value| value.get("status"))
        .and_then(Value::as_str)
        .unwrap_or("");

    if record_state == "performed"
        && !matches!(
            fixture_state,
            "fabricated_uninspected" | "inspected_unpowered_fixture"
        )
    {
        issues.push("performed trial requires a fabricated fixture; concept-only geometry is not test evidence".into());
    }

    if record_state == "performed" && outcome_status == "not_run" {
        issues.push("a performed record cannot have outcome status not_run".into());
    }
    if record_state != "performed" && outcome_status != "not_run" {
        issues.push("planned/voided records must not assert a performed task outcome".into());
    }

    if let Some(timings) = record.get("timings") {
        for phase_name in [
            "diagnosis",
            "active_labour",
            "parts_tool_wait",
            "verification",
            "rework",
            "total_downtime",
        ] {
            let Some(phase) = timings.get(phase_name) else {
                continue;
            };
            let state = phase.get("state").and_then(Value::as_str).unwrap_or("");
            let minutes = phase.get("minutes");
            let rationale = phase
                .get("rationale")
                .and_then(Value::as_str)
                .unwrap_or("")
                .trim();
            match state {
                "measured" => {
                    if !minutes
                        .and_then(Value::as_f64)
                        .is_some_and(|value| value.is_finite() && value >= 0.0)
                    {
                        issues.push(format!(
                            "{phase_name}: measured duration must be a finite non-negative number"
                        ));
                    }
                }
                "not_applicable" | "not_recorded" | "not_started" => {
                    if !minutes.map(Value::is_null).unwrap_or(false) {
                        issues.push(format!("{phase_name}: non-measured phase must use null minutes, never a synthetic zero"));
                    }
                    if rationale.is_empty() {
                        issues.push(format!(
                            "{phase_name}: non-measured phase requires a reason"
                        ));
                    }
                }
                _ => {}
            }
        }
    }

    if outcome_status == "success" {
        if record_state != "performed" {
            issues.push("success can only be recorded for a performed trial".into());
        }
        if outcome
            .and_then(|value| value.get("mockup_function_restored"))
            .and_then(Value::as_bool)
            != Some(true)
        {
            issues.push("success requires mockup_function_restored=true".into());
        }
        if outcome
            .and_then(|value| value.get("post_check_status"))
            .and_then(Value::as_str)
            != Some("passed")
        {
            issues.push("success requires a passed post-check; attempted repair is not verified restoration".into());
        }
    }

    if outcome
        .and_then(|value| value.get("permanent_structure_damage"))
        .and_then(Value::as_bool)
        == Some(true)
    {
        if outcome
            .and_then(|value| value.get("damage_description"))
            .and_then(Value::as_str)
            .map(str::trim)
            .unwrap_or("")
            .len()
            < 5
        {
            issues.push("permanent structure damage requires a useful description".into());
        }
        if outcome_status == "success" {
            issues.push("a trial with permanent structure damage cannot pass the non-destructive service gate".into());
        }
    }

    if record_state == "performed"
        && record
            .get("evidence")
            .and_then(Value::as_array)
            .is_some_and(Vec::is_empty)
    {
        issues.push("performed trial requires at least one digest-bound evidence reference".into());
    }

    if record
        .pointer("/operating_state/appliance_energized")
        .and_then(Value::as_bool)
        != Some(false)
        || record
            .pointer("/operating_state/mains_connected")
            .and_then(Value::as_bool)
            != Some(false)
        || record
            .pointer("/operating_state/refrigerant_or_process_fluid_present")
            .and_then(Value::as_bool)
            != Some(false)
        || record
            .pointer("/operating_state/heating_element_present")
            .and_then(Value::as_bool)
            != Some(false)
        || record
            .pointer("/operating_state/test_article_is_surrogate_only")
            .and_then(Value::as_bool)
            != Some(true)
    {
        issues.push("v0.1 fixture protocol permits only inert surrogate trials: no power, mains, fluids, or heating element".into());
    }

    issues
}

fn sha256_hex(bytes: &[u8]) -> String {
    Sha256::digest(bytes)
        .iter()
        .map(|byte| format!("{byte:02x}"))
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    fn source() -> (Value, Value) {
        let root = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
        let schema_path = root.join("../../docs/LKI_01_REPAIR_TRIAL_RECORD_SCHEMA_V0_1_DRAFT.json");
        let record_path =
            root.join("../../docs/LKI_01_REPAIR_TRIAL_RECORD_EXAMPLE_V0_1_DRAFT.json");
        let schema =
            serde_json::from_str(&fs::read_to_string(schema_path).expect("schema fixture exists"))
                .expect("schema fixture is valid JSON");
        let record =
            serde_json::from_str(&fs::read_to_string(record_path).expect("record fixture exists"))
                .expect("record fixture is valid JSON");
        (schema, record)
    }

    fn schema_valid(schema: &Value, record: &Value) -> bool {
        draft202012::options()
            .should_validate_formats(true)
            .build(schema)
            .expect("schema compiles")
            .is_valid(record)
    }

    #[test]
    fn draft_schema_and_planned_example_are_valid() {
        let (schema, record) = source();
        assert!(draft202012::meta::validate(&schema).is_ok());
        assert!(schema_valid(&schema, &record));
        assert!(validate_record_semantics(&record).is_empty());
    }

    #[test]
    fn energized_appliance_is_rejected_by_schema_and_semantics() {
        let (schema, mut record) = source();
        record["operating_state"]["appliance_energized"] = json!(true);
        assert!(!schema_valid(&schema, &record));
        assert!(!validate_record_semantics(&record).is_empty());
    }

    #[test]
    fn measured_phase_cannot_have_missing_or_negative_duration() {
        let (_, mut record) = source();
        record["timings"]["active_labour"]["state"] = json!("measured");
        record["timings"]["active_labour"]["minutes"] = Value::Null;
        assert!(!validate_record_semantics(&record).is_empty());
        record["timings"]["active_labour"]["minutes"] = json!(-1.0);
        assert!(!validate_record_semantics(&record).is_empty());
    }

    #[test]
    fn successful_trial_requires_post_check_and_no_permanent_damage() {
        let (_, mut record) = source();
        record["record_state"] = json!("performed");
        record["fixture"]["fixture_state"] = json!("inspected_unpowered_fixture");
        record["outcome"]["status"] = json!("success");
        record["outcome"]["mockup_function_restored"] = json!(true);
        record["outcome"]["post_check_status"] = json!("not_performed");
        record["evidence"] = json!([{
            "evidence_id":"E-001",
            "kind":"photo",
            "sha256":"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
            "revision_scope":"mockup-fixture-0",
            "status":"captured"
        }]);
        assert!(
            validate_record_semantics(&record)
                .iter()
                .any(|issue| issue.contains("passed post-check"))
        );
        record["outcome"]["post_check_status"] = json!("passed");
        record["outcome"]["permanent_structure_damage"] = json!(true);
        record["outcome"]["damage_description"] = json!("Cracked rail.");
        assert!(
            validate_record_semantics(&record)
                .iter()
                .any(|issue| issue.contains("permanent structure damage"))
        );
    }

    #[test]
    fn unknown_or_missing_duration_is_not_silently_zero() {
        let (_, mut record) = source();
        record["timings"]["parts_tool_wait"]["state"] = json!("not_recorded");
        record["timings"]["parts_tool_wait"]["minutes"] = json!(0.0);
        assert!(
            validate_record_semantics(&record)
                .iter()
                .any(|issue| issue.contains("must use null minutes"))
        );
        record["timings"]["parts_tool_wait"]["minutes"] = Value::Null;
        record["timings"]["parts_tool_wait"]["rationale"] = json!("");
        assert!(validate_record_semantics(&record).iter().any(|issue| issue.contains("requires a reason")));
    }
}
