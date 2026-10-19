use jsonschema::draft202012;
use serde_json::{json, Value};
use std::collections::HashMap;
use std::env;
use std::fs;
use std::path::{Path, PathBuf};

#[derive(Clone, Copy, Debug)]
struct Box3 {
    origin: [f64; 3],
    size: [f64; 3],
    clearance: [f64; 3],
}

fn main() {
    match parse_args() {
        Ok(Some((schema_path, record_path, benchmark_path))) => {
            match audit_files(&schema_path, &record_path, &benchmark_path) {
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
                    eprintln!("service-envelope-audit: {error}");
                    std::process::exit(2);
                }
            }
        }
        Ok(None) => {}
        Err(error) => {
            eprintln!("service-envelope-audit: {error}");
            eprintln!("Run with --help for usage.");
            std::process::exit(2);
        }
    }
}

fn parse_args() -> Result<Option<(PathBuf, PathBuf, PathBuf)>, String> {
    let mut schema = None;
    let mut record = None;
    let mut benchmark = None;
    let mut args = env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--help" | "-h" => {
                println!(
                    "Service-envelope audit (draft)\n\
                     \n\
                     Usage: service_envelope_audit --schema PATH --record PATH --benchmark PATH\n\
                     \n\
                     Applies conservative axis-aligned geometry screening to a draft service-envelope record \
                     and checks a synthetic known-answer corpus. It cannot qualify physical fit or product safety."
                );
                return Ok(None);
            }
            "--schema" => {
                schema = Some(PathBuf::from(
                    args.next().ok_or("--schema requires a path")?,
                ))
            }
            "--record" => {
                record = Some(PathBuf::from(
                    args.next().ok_or("--record requires a path")?,
                ))
            }
            "--benchmark" => {
                benchmark = Some(PathBuf::from(args.next().ok_or("--benchmark requires a path")?))
            }
            other => return Err(format!("unknown argument: {other}")),
        }
    }
    Ok(Some((
        schema.ok_or("--schema is required")?,
        record.ok_or("--record is required")?,
        benchmark.ok_or("--benchmark is required")?,
    )))
}

fn read_json(path: &Path) -> Result<Value, String> {
    let bytes = fs::read(path).map_err(|error| format!("cannot read {}: {error}", path.display()))?;
    serde_json::from_slice::<Value>(&bytes)
        .map_err(|error| format!("invalid JSON in {}: {error}", path.display()))
}

fn audit_files(
    schema_path: &Path,
    record_path: &Path,
    benchmark_path: &Path,
) -> Result<Value, String> {
    let schema = read_json(schema_path)?;
    let record = read_json(record_path)?;
    let benchmark = read_json(benchmark_path)?;
    let mut issues = Vec::<String>::new();

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
                    .map(|error| format!("record schema: {error}")),
            );
            valid
        }
        Err(error) => {
            issues.push(format!("could not compile record schema: {error}"));
            false
        }
    };

    let record_issues = audit_record_semantics(&record);
    issues.extend(record_issues.iter().cloned());

    let benchmark_report = audit_benchmark(&benchmark);
    if benchmark_report["status"].as_str() != Some("PASS") {
        if let Some(case_issues) = benchmark_report["issues"].as_array() {
            issues.extend(case_issues.iter().filter_map(Value::as_str).map(str::to_string));
        }
    }

    let status = if issues.is_empty() { "PASS" } else { "FAIL" };
    Ok(json!({
        "tool": "service-envelope-audit",
        "tool_version": env!("CARGO_PKG_VERSION"),
        "status": status,
        "scope": "draft schema, AABB screening semantics and synthetic known-answer corpus only",
        "checks": {
            "schema_meta_valid": schema_meta_valid,
            "record_schema_valid": record_schema_valid,
            "record_semantics_pass": record_issues.is_empty(),
            "benchmark_status": benchmark_report["status"],
            "physical_fit_qualified": false,
            "appliance_safety_or_conformity_established": false,
            "durability_established": false
        },
        "record_state": record.get("record_state"),
        "record_issues": record_issues,
        "benchmark": benchmark_report,
        "issues": issues
    }))
}

fn audit_record_semantics(record: &Value) -> Vec<String> {
    let mut issues = Vec::<String>::new();
    let record_state = record.get("record_state").and_then(Value::as_str).unwrap_or("unknown");
    if record_state == "qualified" {
        issues.push("ENVELOPE_V0_1_CANNOT_ESTABLISH_PHYSICAL_QUALIFICATION".to_string());
    }

    let mut sources = HashMap::<String, &Value>::new();
    if let Some(items) = record.get("sources").and_then(Value::as_array) {
        for source in items {
            let Some(id) = source.get("source_id").and_then(Value::as_str) else {
                issues.push("SOURCE_ID_MISSING".to_string());
                continue;
            };
            if sources.insert(id.to_string(), source).is_some() {
                issues.push(format!("SOURCE_ID_DUPLICATE:{id}"));
            }
            let state = source.get("evidence_state").and_then(Value::as_str).unwrap_or("unknown");
            let source_type = source
                .get("source_type")
                .and_then(Value::as_str)
                .unwrap_or("unknown");
            let source_type_matches_state = match state {
                "assumption" => source_type == "assumption",
                "synthetic" => source_type == "synthetic_fixture",
                "measured" => source_type == "measurement_report",
                "approved" => matches!(
                    source_type,
                    "supplier_cad" | "controlled_drawing" | "measurement_report"
                ),
                _ => false,
            };
            if !source_type_matches_state {
                issues.push(format!("SOURCE_TYPE_STATE_MISMATCH:{id}"));
            }
            let digest_valid = source
                .get("digest_sha256")
                .and_then(Value::as_str)
                .is_some_and(is_sha256_hex);
            if matches!(state, "measured" | "approved") && !digest_valid {
                issues.push(format!("SOURCE_DIGEST_INVALID:{id}"));
            }
            if record_state == "measured"
                && (!matches!(state, "measured" | "approved") || !digest_valid)
            {
                issues.push(format!("MEASURED_RECORD_SOURCE_UNQUALIFIED:{id}"));
            }
        }
    } else {
        issues.push("SOURCES_MISSING".to_string());
    }

    let obstacle_inventory_state = record
        .get("obstacle_inventory_state")
        .and_then(Value::as_str)
        .unwrap_or("incomplete");
    let mut inventory_source_ids = Vec::<String>::new();
    if let Some(ids) = record
        .get("obstacle_inventory_evidence_source_ids")
        .and_then(Value::as_array)
    {
        for id in ids.iter().filter_map(Value::as_str) {
            inventory_source_ids.push(id.to_string());
        }
    } else {
        issues.push("OBSTACLE_INVENTORY_SOURCE_IDS_MISSING".to_string());
    }

    if matches!(record_state, "screening" | "measured")
        && obstacle_inventory_state != "reviewed"
    {
        issues.push("OBSTACLE_INVENTORY_NOT_REVIEWED".to_string());
    }
    if obstacle_inventory_state == "reviewed" && inventory_source_ids.is_empty() {
        issues.push("OBSTACLE_INVENTORY_REVIEW_EVIDENCE_MISSING".to_string());
    }
    if obstacle_inventory_state == "reviewed" {
        for source_id in &inventory_source_ids {
            let Some(source) = sources.get(source_id) else {
                issues.push(format!(
                    "OBSTACLE_INVENTORY_SOURCE_UNRESOLVED:{source_id}"
                ));
                continue;
            };
            let state = source
                .get("evidence_state")
                .and_then(Value::as_str)
                .unwrap_or("");
            let source_type = source
                .get("source_type")
                .and_then(Value::as_str)
                .unwrap_or("");
            let digest_valid = source
                .get("digest_sha256")
                .and_then(Value::as_str)
                .is_some_and(is_sha256_hex);
            if !matches!(source_type, "controlled_drawing" | "measurement_report")
                || !matches!(state, "measured" | "approved")
                || !digest_valid
            {
                issues.push(format!(
                    "OBSTACLE_INVENTORY_REVIEW_EVIDENCE_UNQUALIFIED:{source_id}"
                ));
            }
        }
    }
    if matches!(record_state, "screening" | "measured") {
        if let Some(items) = record.get("envelopes").and_then(Value::as_array) {
            for envelope in items {
                let id = envelope
                    .get("envelope_id")
                    .and_then(Value::as_str)
                    .unwrap_or("unknown");
                let source_id = envelope
                    .get("source_id")
                    .and_then(Value::as_str)
                    .unwrap_or("");
                let Some(source) = sources.get(source_id) else {
                    continue;
                };
                let state = source
                    .get("evidence_state")
                    .and_then(Value::as_str)
                    .unwrap_or("");
                let digest_valid = source
                    .get("digest_sha256")
                    .and_then(Value::as_str)
                    .is_some_and(is_sha256_hex);
                if !matches!(state, "measured" | "approved") || !digest_valid {
                    issues.push(format!(
                        "NON_CONCEPT_ENVELOPE_SOURCE_UNQUALIFIED:{id}"
                    ));
                }
                let role = envelope.get("role").and_then(Value::as_str).unwrap_or("");
                if obstacle_inventory_state == "reviewed"
                    && matches!(role, "obstacle" | "keep_out")
                    && !inventory_source_ids.iter().any(|declared| declared == source_id)
                {
                    issues.push(format!(
                        "OBSTACLE_SOURCE_NOT_IN_REVIEW_MANIFEST:{id}:{source_id}"
                    ));
                }
            }
        }
    }

    let reference_frame = record
        .pointer("/reference_frame/frame_id")
        .and_then(Value::as_str)
        .unwrap_or("unknown");
    let mut envelopes = HashMap::<String, &Value>::new();
    if let Some(items) = record.get("envelopes").and_then(Value::as_array) {
        for envelope in items {
            let Some(id) = envelope.get("envelope_id").and_then(Value::as_str) else {
                issues.push("ENVELOPE_ID_MISSING".to_string());
                continue;
            };
            if envelopes.insert(id.to_string(), envelope).is_some() {
                issues.push(format!("ENVELOPE_ID_DUPLICATE:{id}"));
            }
            let source_id = envelope.get("source_id").and_then(Value::as_str).unwrap_or("");
            if !sources.contains_key(source_id) {
                issues.push(format!("ENVELOPE_SOURCE_UNRESOLVED:{id}:{source_id}"));
            }
            let frame_id = envelope.get("frame_id").and_then(Value::as_str).unwrap_or("");
            if frame_id != reference_frame {
                issues.push(format!("ENVELOPE_FRAME_MISMATCH:{id}"));
            }
        }
    } else {
        issues.push("ENVELOPES_MISSING".to_string());
    }

    if let Some(checks) = record.get("fit_checks").and_then(Value::as_array) {
        for check in checks {
            let check_id = check.get("check_id").and_then(Value::as_str).unwrap_or("unknown");
            let module_id = check
                .get("module_envelope_id")
                .and_then(Value::as_str)
                .unwrap_or("");
            let bay_id = check
                .get("bay_envelope_id")
                .and_then(Value::as_str)
                .unwrap_or("");
            let Some(module) = envelopes.get(module_id).copied() else {
                issues.push(format!("FIT_MODULE_UNRESOLVED:{check_id}:{module_id}"));
                continue;
            };
            let Some(bay) = envelopes.get(bay_id).copied() else {
                issues.push(format!("FIT_BAY_UNRESOLVED:{check_id}:{bay_id}"));
                continue;
            };
            if module.get("role").and_then(Value::as_str) != Some("module")
                || bay.get("role").and_then(Value::as_str) != Some("bay")
            {
                issues.push(format!("FIT_ROLE_MISMATCH:{check_id}"));
                continue;
            }
            if module.get("frame_id") != bay.get("frame_id") {
                issues.push(format!("FIT_FRAME_MISMATCH:{check_id}"));
                continue;
            }
            let Some(module_box) = box_from_record(module) else {
                issues.push(format!("FIT_MODULE_GEOMETRY_INVALID:{check_id}"));
                continue;
            };
            let Some(bay_box) = box_from_record(bay) else {
                issues.push(format!("FIT_BAY_GEOMETRY_INVALID:{check_id}"));
                continue;
            };
            let Some(minimum) = check.get("minimum_clearance_mm").and_then(xyz_object) else {
                issues.push(format!("FIT_MINIMUM_CLEARANCE_INVALID:{check_id}"));
                continue;
            };
            if !fits_with_clearance(module_box, bay_box, minimum) {
                issues.push(format!("FIT_CLEARANCE_FAILED:{check_id}"));
            }
        }
    } else {
        issues.push("FIT_CHECKS_MISSING".to_string());
    }

    let moving_roles = ["installation_sweep", "service_access", "removal_sweep"];
    let mut movers = Vec::<(&str, Box3)>::new();
    let mut obstacles = Vec::<(&str, Box3)>::new();
    for (id, envelope) in &envelopes {
        let role = envelope.get("role").and_then(Value::as_str).unwrap_or("");
        let Some(box3) = box_from_record(envelope) else {
            issues.push(format!("ENVELOPE_GEOMETRY_INVALID:{id}"));
            continue;
        };
        if moving_roles.contains(&role) {
            movers.push((id.as_str(), box3));
        } else if matches!(role, "obstacle" | "keep_out") {
            obstacles.push((id.as_str(), box3));
        }
    }
    for (moving_id, moving) in &movers {
        for (obstacle_id, obstacle) in &obstacles {
            if boxes_conflict(*moving, *obstacle) {
                issues.push(format!("ENVELOPE_COLLISION:{moving_id}:{obstacle_id}"));
            }
        }
    }
    issues.sort();
    issues.dedup();
    issues
}

fn audit_benchmark(benchmark: &Value) -> Value {
    let mut issues = Vec::<String>::new();
    let Some(scenarios) = benchmark.get("scenarios").and_then(Value::as_array) else {
        return json!({
            "status":"FAIL",
            "scenario_count":0,
            "matched":0,
            "issues":["BENCHMARK_SCENARIOS_MISSING"]
        });
    };
    let mut seen = HashMap::<String, ()>::new();
    let mut matched = 0usize;
    for scenario in scenarios {
        let id = scenario.get("scenario_id").and_then(Value::as_str).unwrap_or("unknown");
        if seen.insert(id.to_string(), ()).is_some() {
            issues.push(format!("BENCHMARK_SCENARIO_DUPLICATE:{id}"));
        }
        let expected = scenario.get("expected_outcome").and_then(Value::as_str).unwrap_or("");
        let observed =
            evaluate_scenario(scenario).unwrap_or_else(|| "INVALID_SCENARIO".to_string());
        if observed == expected {
            matched += 1;
        } else {
            issues.push(format!(
                "BENCHMARK_ORACLE_MISMATCH:{id}:expected={expected}:observed={observed}"
            ));
        }
    }
    if scenarios.is_empty() {
        issues.push("BENCHMARK_EMPTY".to_string());
    }
    let status = if issues.is_empty() && matched == scenarios.len() { "PASS" } else { "FAIL" };
    json!({
        "status":status,
        "benchmark_version":benchmark.get("benchmark_version"),
        "scenario_count":scenarios.len(),
        "matched":matched,
        "issues":issues
    })
}

fn evaluate_scenario(scenario: &Value) -> Option<String> {
    let setup = scenario.get("setup")?;
    let state = setup.get("record_state")?.as_str()?;
    if state == "qualified" {
        return Some("BLOCK_UNSUPPORTED_QUALIFICATION".to_string());
    }
    let evidence_state = setup.get("source_evidence_state")?.as_str()?;
    let digest_present = setup.get("source_digest_present")?.as_bool()?;
    if matches!(evidence_state, "measured" | "approved") && !digest_present {
        return Some("BLOCK_MISSING_SOURCE_DIGEST".to_string());
    }

    let module = box_from_scenario(setup.get("module")?)?;
    let bay = box_from_scenario(setup.get("bay")?)?;
    let minimum = xyz_array(setup.get("minimum_clearance")?)?;
    if !fits_with_clearance(module, bay, minimum) {
        return Some("FAIL_HARD_GATE".to_string());
    }

    let moving = setup.get("moving")?.as_array()?;
    let obstacles = setup.get("obstacles")?.as_array()?;
    for move_value in moving {
        let moving_box = box_from_scenario(move_value)?;
        for obstacle_value in obstacles {
            let obstacle_box = box_from_scenario(obstacle_value)?;
            if boxes_conflict(moving_box, obstacle_box) {
                return Some("FAIL_HARD_GATE".to_string());
            }
        }
    }
    Some("PASS_SCREENING".to_string())
}

fn box_from_record(value: &Value) -> Option<Box3> {
    Some(Box3 {
        origin: xyz_object(value.get("origin_mm")?)?,
        size: xyz_object(value.get("size_mm")?)?,
        clearance: xyz_object(value.get("clearance_mm")?)?,
    })
}

fn box_from_scenario(value: &Value) -> Option<Box3> {
    Some(Box3 {
        origin: xyz_array(value.get("origin")?)?,
        size: xyz_array(value.get("size")?)?,
        clearance: value
            .get("clearance")
            .and_then(xyz_array)
            .unwrap_or([0.0, 0.0, 0.0]),
    })
}

fn xyz_object(value: &Value) -> Option<[f64; 3]> {
    Some([
        value.get("x")?.as_f64()?,
        value.get("y")?.as_f64()?,
        value.get("z")?.as_f64()?,
    ])
}

fn xyz_array(value: &Value) -> Option<[f64; 3]> {
    let values = value.as_array()?;
    if values.len() != 3 {
        return None;
    }
    Some([values[0].as_f64()?, values[1].as_f64()?, values[2].as_f64()?])
}

fn fits_with_clearance(module: Box3, bay: Box3, minimum: [f64; 3]) -> bool {
    (0..3).all(|axis| {
        minimum[axis] >= 0.0
            && module.origin[axis] >= bay.origin[axis] + minimum[axis]
            && module.origin[axis] + module.size[axis]
                <= bay.origin[axis] + bay.size[axis] - minimum[axis]
    })
}

fn boxes_conflict(moving: Box3, obstacle: Box3) -> bool {
    (0..3).all(|axis| {
        let moving_min = moving.origin[axis] - moving.clearance[axis];
        let moving_max =
            moving.origin[axis] + moving.size[axis] + moving.clearance[axis];
        let obstacle_min = obstacle.origin[axis] - obstacle.clearance[axis];
        let obstacle_max =
            obstacle.origin[axis] + obstacle.size[axis] + obstacle.clearance[axis];
        // Strict separation is required. Face/edge/point touching is a conflict.
        !(moving_max < obstacle_min || obstacle_max < moving_min)
    })
}

fn is_sha256_hex(value: &str) -> bool {
    value.len() == 64 && value.bytes().all(|byte| byte.is_ascii_hexdigit())
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    fn repo_file(relative: &str) -> String {
        let root = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
        fs::read_to_string(root.join(relative)).expect("test fixture must be readable")
    }

    fn example_record() -> Value {
        serde_json::from_str(&repo_file(
            "../../docs/LKI_01_SERVICE_ENVELOPE_EXAMPLE_V0_1_DRAFT.json",
        ))
        .unwrap()
    }

    #[test]
    fn schema_and_example_validate_but_never_claim_physical_fit_qualification() {
        let schema: Value =
            serde_json::from_str(&repo_file(
                "../../docs/LKI_01_SERVICE_ENVELOPE_SCHEMA_V0_1_DRAFT.json",
            ))
            .unwrap();
        let record = example_record();
        assert!(draft202012::meta::validate(&schema).is_ok());
        let validator = draft202012::options()
            .should_validate_formats(true)
            .build(&schema)
            .unwrap();
        assert!(validator.is_valid(&record));
        assert!(audit_record_semantics(&record).is_empty());
        assert_eq!(record["record_state"], "concept");
    }

    #[test]
    fn frozen_known_answer_corpus_matches_every_expected_outcome() {
        let benchmark: Value =
            serde_json::from_str(&repo_file(
                "../../docs/LKI_01_SERVICE_ENVELOPE_BENCHMARK_V0_1_DRAFT.json",
            ))
            .unwrap();
        let report = audit_benchmark(&benchmark);
        assert_eq!(benchmark["scenarios"].as_array().unwrap().len(), 10);
        assert_eq!(report["status"], "PASS");
        assert_eq!(report["matched"], 10);
    }

    #[test]
    fn measured_source_without_digest_is_rejected() {
        let mut record = example_record();
        record["sources"][0]["evidence_state"] = json!("measured");
        record["sources"][0]["digest_sha256"] = Value::Null;
        assert!(audit_record_semantics(&record)
            .iter()
            .any(|issue| issue == "SOURCE_DIGEST_INVALID:SRC-CONCEPT-001"));
    }

    #[test]
    fn unresolved_geometry_source_is_rejected() {
        let mut record = example_record();
        record["envelopes"][0]["source_id"] = json!("missing-source");
        assert!(audit_record_semantics(&record)
            .iter()
            .any(|issue| issue == "ENVELOPE_SOURCE_UNRESOLVED:BAY-COLD-001:missing-source"));
    }

    #[test]
    fn insufficient_clearance_is_a_hard_gate_failure() {
        let mut record = example_record();
        record["fit_checks"][0]["minimum_clearance_mm"]["x"] = json!(30);
        assert!(audit_record_semantics(&record)
            .iter()
            .any(|issue| issue == "FIT_CLEARANCE_FAILED:FIT-COLD-001"));
    }

    #[test]
    fn claimed_qualification_is_blocked_even_with_digest_claims() {
        let mut record = example_record();
        record["record_state"] = json!("qualified");
        record["sources"][0]["evidence_state"] = json!("approved");
        record["sources"][0]["digest_sha256"] = json!(
            "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
        );
        assert!(audit_record_semantics(&record)
            .iter()
            .any(|issue| issue == "ENVELOPE_V0_1_CANNOT_ESTABLISH_PHYSICAL_QUALIFICATION"));
    }

    #[test]
    fn touching_an_obstacle_is_a_conflict() {
        let a = Box3 {
            origin: [0.0, 0.0, 0.0],
            size: [100.0, 100.0, 100.0],
            clearance: [0.0, 0.0, 0.0],
        };
        let b = Box3 {
            origin: [100.0, 0.0, 0.0],
            size: [20.0, 100.0, 100.0],
            clearance: [0.0, 0.0, 0.0],
        };
        assert!(boxes_conflict(a, b));
    }

    #[test]
    fn translation_of_a_module_and_bay_preserves_fit_result() {
        let module = Box3 {
            origin: [20.0, 20.0, 20.0],
            size: [760.0, 560.0, 560.0],
            clearance: [0.0, 0.0, 0.0],
        };
        let bay = Box3 {
            origin: [0.0, 0.0, 0.0],
            size: [800.0, 600.0, 600.0],
            clearance: [0.0, 0.0, 0.0],
        };
        let delta = [1234.0, -567.0, 31.0];
        let shifted_module = Box3 {
            origin: [
                module.origin[0] + delta[0],
                module.origin[1] + delta[1],
                module.origin[2] + delta[2],
            ],
            ..module
        };
        let shifted_bay = Box3 {
            origin: [
                bay.origin[0] + delta[0],
                bay.origin[1] + delta[1],
                bay.origin[2] + delta[2],
            ],
            ..bay
        };
        assert_eq!(
            fits_with_clearance(module, bay, [10.0; 3]),
            fits_with_clearance(shifted_module, shifted_bay, [10.0; 3])
        );
    }

    #[test]
    fn increasing_required_clearance_cannot_turn_failure_into_success() {
        let module = Box3 {
            origin: [20.0, 20.0, 20.0],
            size: [760.0, 560.0, 560.0],
            clearance: [0.0, 0.0, 0.0],
        };
        let bay = Box3 {
            origin: [0.0, 0.0, 0.0],
            size: [800.0, 600.0, 600.0],
            clearance: [0.0, 0.0, 0.0],
        };
        assert!(fits_with_clearance(module, bay, [10.0, 10.0, 10.0]));
        assert!(!fits_with_clearance(module, bay, [21.0, 10.0, 10.0]));
    }

    #[test]
    fn greater_moving_clearance_cannot_remove_an_existing_collision() {
        let moving = Box3 {
            origin: [0.0, 0.0, 0.0],
            size: [100.0, 100.0, 100.0],
            clearance: [0.0, 0.0, 0.0],
        };
        let obstacle = Box3 {
            origin: [95.0, 0.0, 0.0],
            size: [20.0, 100.0, 100.0],
            clearance: [0.0, 0.0, 0.0],
        };
        let more_clearance = Box3 {
            clearance: [10.0, 10.0, 10.0],
            ..moving
        };
        assert!(boxes_conflict(moving, obstacle));
        assert!(boxes_conflict(more_clearance, obstacle));
    }

    #[test]
    fn reordering_envelopes_does_not_change_semantic_results() {
        let record = example_record();
        let expected = audit_record_semantics(&record);
        let mut reordered = record.clone();
        reordered["envelopes"].as_array_mut().unwrap().reverse();
        assert_eq!(audit_record_semantics(&reordered), expected);
    }

    #[test]
    fn missing_required_scenario_geometry_fails_closed() {
        let malformed = json!({
            "setup": {
                "record_state": "draft",
                "source_evidence_state": "assumption",
                "source_digest_present": false,
                "module": {"origin": [10, 10, 10], "size": [780, 580, 580]},
                "bay": {"origin": [0, 0, 0], "size": [800, 600, 600]},
                "moving": [],
                "obstacles": []
            }
        });
        assert!(evaluate_scenario(&malformed).is_none());
    }

    #[test]
    fn source_type_cannot_claim_a_stronger_evidence_state() {
        let mut record = example_record();
        record["sources"][0]["evidence_state"] = json!("approved");
        record["sources"][0]["digest_sha256"] = json!(
            "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
        );
        assert!(audit_record_semantics(&record)
            .iter()
            .any(|issue| issue == "SOURCE_TYPE_STATE_MISMATCH:SRC-CONCEPT-001"));
    }

    #[test]
    fn screening_requires_a_reviewed_obstacle_inventory() {
        let mut record = example_record();
        record["record_state"] = json!("screening");
        assert!(audit_record_semantics(&record)
            .iter()
            .any(|issue| issue == "OBSTACLE_INVENTORY_NOT_REVIEWED"));
    }

    #[test]
    fn reviewed_obstacle_inventory_requires_digest_bound_evidence() {
        let mut record = example_record();
        record["obstacle_inventory_state"] = json!("reviewed");
        assert!(audit_record_semantics(&record)
            .iter()
            .any(|issue| issue == "OBSTACLE_INVENTORY_REVIEW_EVIDENCE_MISSING"));
    }

    #[test]
    fn screened_record_cannot_reuse_concept_only_geometry_sources() {
        let mut record = example_record();
        record["record_state"] = json!("screening");
        record["obstacle_inventory_state"] = json!("reviewed");
        record["obstacle_inventory_evidence_source_ids"] = json!(["SRC-CONCEPT-001"]);
        let issues = audit_record_semantics(&record);
        assert!(issues
            .iter()
            .any(|issue| issue.starts_with("NON_CONCEPT_ENVELOPE_SOURCE_UNQUALIFIED:")));
        assert!(issues
            .iter()
            .any(|issue| issue == "OBSTACLE_INVENTORY_REVIEW_EVIDENCE_UNQUALIFIED:SRC-CONCEPT-001"));
    }

    #[test]
    fn keep_out_geometry_must_bind_to_the_review_manifest() {
        let mut record = example_record();
        let digest = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";
        let mut source_a = record["sources"][0].clone();
        source_a["source_id"] = json!("SRC-CONTROLLED-001");
        source_a["source_type"] = json!("controlled_drawing");
        source_a["evidence_state"] = json!("approved");
        source_a["digest_sha256"] = json!(digest);
        source_a["source_ref"] = json!("synthetic unit-test controlled drawing");
        let mut source_b = source_a.clone();
        source_b["source_id"] = json!("SRC-OBSTACLE-002");
        source_b["source_ref"] = json!("synthetic unit-test obstruction drawing");
        record["sources"] = json!([source_a, source_b]);
        for envelope in record["envelopes"].as_array_mut().unwrap() {
            envelope["source_id"] = json!("SRC-CONTROLLED-001");
        }
        record["envelopes"][5]["source_id"] = json!("SRC-OBSTACLE-002");
        record["obstacle_inventory_state"] = json!("reviewed");
        record["obstacle_inventory_evidence_source_ids"] = json!(["SRC-CONTROLLED-001"]);
        record["record_state"] = json!("screening");
        assert!(audit_record_semantics(&record)
            .iter()
            .any(|issue| issue
                == "OBSTACLE_SOURCE_NOT_IN_REVIEW_MANIFEST:SIDE-KEEP-OUT-001:SRC-OBSTACLE-002"));
    }
}
