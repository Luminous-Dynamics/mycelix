use serde_json::{Value, json};
use sha2::{Digest, Sha256};
use std::collections::HashSet;
use std::env;
use std::fs;
use std::path::PathBuf;

const BENCHMARK_ID: &str = "LKI-01-REPAIRABILITY-EXTENSION-V0.4";
const BENCHMARK_VERSION: &str = "0.4.0-draft";
const PARENT_ID: &str = "LKI-01-LIFETIME-PRODUCT-QUALIFICATION-V0.3";
const PARENT_VERSION: &str = "0.3.0-draft";
const SCENARIO_COUNT: usize = 11;
const VALID_OUTCOMES: [&str; 4] = [
    "PASS_SCOPED_GATE",
    "BLOCK_UNQUALIFIED",
    "FAIL_HARD_GATE",
    "INCONCLUSIVE_NO_CLAIM",
];
const REQUIRED_METRICS: [&str; 5] = [
    "BENCH-REPAIR-PHASE-COMPLETENESS",
    "BENCH-POSTREPAIR-VERIFICATION",
    "BENCH-USER-OBSERVATION-SEPARATION",
    "BENCH-SERVICE-PARTS-EVIDENCE",
    "BENCH-HARD-GATE-NONCOMPENSATION",
];

fn main() {
    match parse_args() {
        Ok(Some(path)) => match audit_file(&path) {
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
                eprintln!("repairability-extension-audit: {error}");
                std::process::exit(2);
            }
        },
        Ok(None) => {}
        Err(error) => {
            eprintln!("repairability-extension-audit: {error}");
            eprintln!("Run with --help for usage.");
            std::process::exit(2);
        }
    }
}

fn parse_args() -> Result<Option<PathBuf>, String> {
    let mut corpus = None;
    let mut args = env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--help" | "-h" => {
                println!(
                    "Repairability benchmark extension audit (draft)\n\
                     \n\
                     Usage: repairability_extension_audit --corpus PATH\n\
                     \n\
                     Checks the frozen v0.4 synthetic repairability scenarios only. \
                     It does not perform physical or human studies."
                );
                return Ok(None);
            }
            "--corpus" => {
                let path = args
                    .next()
                    .ok_or_else(|| "--corpus requires a path".to_string())?;
                corpus = Some(PathBuf::from(path));
            }
            other => return Err(format!("unknown argument: {other}")),
        }
    }
    Ok(Some(corpus.ok_or("--corpus is required")?))
}

fn audit_file(path: &PathBuf) -> Result<Value, String> {
    let bytes =
        fs::read(path).map_err(|error| format!("cannot read {}: {error}", path.display()))?;
    let corpus = serde_json::from_slice::<Value>(&bytes)
        .map_err(|error| format!("invalid JSON in {}: {error}", path.display()))?;
    let digest = sha256_hex(&bytes);
    let mut issues: Vec<String> = Vec::new();

    if corpus.get("benchmark_id").and_then(Value::as_str) != Some(BENCHMARK_ID) {
        issues.push("benchmark_id does not match the pinned v0.4 extension".into());
    }
    if corpus.get("benchmark_version").and_then(Value::as_str) != Some(BENCHMARK_VERSION) {
        issues.push("benchmark_version does not match the pinned v0.4 extension".into());
    }
    if corpus.get("status").and_then(Value::as_str) != Some("specification_only") {
        issues.push("status must remain specification_only until real studies are performed".into());
    }

    let parent = corpus.get("extension_of");
    if parent.and_then(|v| v.get("benchmark_id")).and_then(Value::as_str) != Some(PARENT_ID)
        || parent.and_then(|v| v.get("benchmark_version")).and_then(Value::as_str)
            != Some(PARENT_VERSION)
    {
        issues.push("extension_of must point to the frozen v0.3 baseline by ID and version".into());
    }

    let claim_ceiling = corpus
        .get("claim_ceiling")
        .and_then(Value::as_str)
        .unwrap_or("");
    if !claim_ceiling.to_ascii_lowercase().contains("unexecuted") {
        issues.push("claim_ceiling must state that this benchmark is unexecuted as evidence".into());
    }
    let invariants = corpus.get("hard_invariants").and_then(Value::as_array);
    if invariants.map(|items| items.len()).unwrap_or(0) < 9
        || !invariants
            .map(|items| items.iter().all(Value::is_string))
            .unwrap_or(false)
    {
        issues.push("hard_invariants must contain at least nine strings".into());
    }

    let scenarios = match corpus.get("scenarios").and_then(Value::as_array) {
        Some(scenarios) => scenarios,
        None => {
            issues.push("scenarios must be an array".into());
            return Ok(report(path, &digest, 0, issues));
        }
    };
    if scenarios.len() != SCENARIO_COUNT {
        issues.push(format!(
            "pinned extension expects {SCENARIO_COUNT} scenarios, found {}",
            scenarios.len()
        ));
    }

    let mut ids = HashSet::new();
    let mut checked = 0_usize;
    for (index, scenario) in scenarios.iter().enumerate() {
        let id = scenario
            .get("scenario_id")
            .and_then(Value::as_str)
            .unwrap_or("");
        let expected = scenario
            .get("expected_outcome")
            .and_then(Value::as_str)
            .unwrap_or("");
        let shape_ok = !id.is_empty()
            && scenario
                .get("title")
                .and_then(Value::as_str)
                .map(|s| !s.trim().is_empty())
                .unwrap_or(false)
            && scenario
                .get("category")
                .and_then(Value::as_str)
                .map(|s| !s.trim().is_empty())
                .unwrap_or(false)
            && scenario.get("setup").map(Value::is_object).unwrap_or(false)
            && scenario
                .get("required_assertions")
                .and_then(Value::as_array)
                .map(|items| !items.is_empty() && items.iter().all(Value::is_string))
                .unwrap_or(false)
            && VALID_OUTCOMES.contains(&expected);

        if !shape_ok {
            issues.push(format!("scenario index {index} has invalid required fields"));
            continue;
        }
        if !ids.insert(id.to_string()) {
            issues.push(format!("duplicate scenario ID: {id}"));
            continue;
        }

        match evaluate_scenario(scenario) {
            Some(actual) if actual == expected => checked += 1,
            Some(actual) => issues.push(format!(
                "{id}: oracle mismatch; expected {expected}, evaluator returned {actual}"
            )),
            None => issues.push(format!(
                "{id}: no deterministic rule is implemented; refusing to skip"
            )),
        }
    }

    let metrics = corpus
        .get("proposed_software_benchmarks")
        .and_then(Value::as_array);
    for metric_id in REQUIRED_METRICS {
        let matches: Vec<&Value> = metrics
            .into_iter()
            .flatten()
            .filter(|metric| metric.get("metric_id").and_then(Value::as_str) == Some(metric_id))
            .collect();
        if matches.len() != 1
            || matches[0].get("target").and_then(Value::as_f64) != Some(100.0)
        {
            issues.push(format!(
                "expected exactly one {metric_id} metric with target 100"
            ));
        }
    }

    Ok(report(path, &digest, checked, issues))
}

fn report(path: &PathBuf, digest: &str, checked: usize, issues: Vec<String>) -> Value {
    json!({
        "tool": "repairability-extension-audit",
        "tool_version": env!("CARGO_PKG_VERSION"),
        "status": if issues.is_empty() { "PASS" } else { "FAIL" },
        "scope": "synthetic v0.4 benchmark structure and deterministic known-answer oracles only",
        "benchmark_version": BENCHMARK_VERSION,
        "scenarios_expected": SCENARIO_COUNT,
        "scenarios_oracles_matched": checked,
        "input": {
            "path": path.display().to_string(),
            "sha256": digest
        },
        "evidence_limits": {
            "physical_trials_performed": false,
            "human_trials_performed": false,
            "parts_availability_established": false,
            "appliance_safety_or_conformity_established": false,
            "product_durability_established": false
        },
        "error_count": issues.len(),
        "issues": issues
    })
}

fn evaluate_scenario(scenario: &Value) -> Option<&'static str> {
    let id = scenario.get("scenario_id")?.as_str()?;
    let setup = scenario.get("setup")?;
    let b = |key: &str| setup.get(key).and_then(Value::as_bool);

    match id {
        "HUMAN-001" => {
            if b("opening_task_observed") != Some(true)
                || b("fault_identification_observed") != Some(true)
                || b("representative_user_sample_declared") != Some(true)
            {
                Some("INCONCLUSIVE_NO_CLAIM")
            } else if b("user_can_open_without_damage") == Some(true)
                && b("critical_component_accessed") == Some(true)
            {
                Some("PASS_SCOPED_GATE")
            } else if b("user_can_open_without_damage") == Some(false)
                || b("critical_component_accessed") == Some(false)
            {
                Some("FAIL_HARD_GATE")
            } else {
                Some("INCONCLUSIVE_NO_CLAIM")
            }
        }
        "HUMAN-002" => {
            if b("opening_task_observed") == Some(true)
                && (b("user_can_open_without_damage") == Some(false)
                    || b("critical_component_accessed") == Some(false)
                    || b("permanent_structure_damage") == Some(true))
            {
                Some("FAIL_HARD_GATE")
            } else {
                Some("BLOCK_UNQUALIFIED")
            }
        }
        "HUMAN-003" => {
            if b("permanent_structure_damage") == Some(true)
                || b("user_can_open_without_damage") == Some(false)
                || b("critical_component_accessed") == Some(false)
            {
                Some("FAIL_HARD_GATE")
            } else if b("opening_task_observed") == Some(true)
                && b("user_can_open_without_damage") == Some(true)
                && b("critical_component_accessed") == Some(true)
                && b("fault_identification_observed") == Some(true)
                && b("representative_user_sample_declared") == Some(true)
                && b("protocol_version_declared") == Some(true)
            {
                Some("PASS_SCOPED_GATE")
            } else {
                Some("INCONCLUSIVE_NO_CLAIM")
            }
        }
        "REPAIR-PHASE-001" | "REPAIR-PHASE-002" => {
            let complete = [
                "diagnosis_time_recorded",
                "active_labor_time_recorded",
                "parts_wait_time_recorded",
                "verification_time_recorded",
                "rework_attempts_recorded",
                "total_downtime_recorded",
                "exact_revision_bound",
            ]
            .iter()
            .all(|field| b(field) == Some(true));
            if complete {
                Some("PASS_SCOPED_GATE")
            } else {
                Some("INCONCLUSIVE_NO_CLAIM")
            }
        }
        "RESTORE-001" => {
            if b("repair_attempted") == Some(true)
                && (b("post_repair_verification_passed") != Some(true)
                    || b("exact_revision_bound") != Some(true))
            {
                Some("BLOCK_UNQUALIFIED")
            } else if b("repair_attempted") == Some(true)
                && b("post_repair_verification_passed") == Some(true)
                && b("exact_revision_bound") == Some(true)
            {
                Some("PASS_SCOPED_GATE")
            } else {
                Some("BLOCK_UNQUALIFIED")
            }
        }
        "RESTORE-002" => {
            if b("repair_attempted") == Some(true)
                && b("post_repair_verification_passed") == Some(true)
                && b("exact_revision_bound") == Some(true)
                && b("verification_scope_declared") == Some(true)
            {
                Some("PASS_SCOPED_GATE")
            } else {
                Some("BLOCK_UNQUALIFIED")
            }
        }
        "RECURRENCE-001" => {
            if b("follow_up_window_declared") == Some(true)
                && b("recurrence_observed_within_window") == Some(true)
                && b("durable_restoration_claimed") == Some(true)
            {
                Some("FAIL_HARD_GATE")
            } else if b("follow_up_window_declared") != Some(true) {
                Some("INCONCLUSIVE_NO_CLAIM")
            } else {
                Some("BLOCK_UNQUALIFIED")
            }
        }
        "PARTS-001" | "PARTS-002" => {
            let evidence_complete = b("component_service_part_listed") == Some(true)
                && b("written_parts_policy_present") == Some(true)
                && b("current_source_evidence_present") == Some(true)
                && b("exact_part_number_bound") == Some(true)
                && (id != "PARTS-002" || b("policy_validity_period_declared") == Some(true));
            if evidence_complete {
                Some("PASS_SCOPED_GATE")
            } else {
                Some("BLOCK_UNQUALIFIED")
            }
        }
        "SCORE-001" => {
            if b("calculated_index_high") == Some(true)
                && (b("hard_opening_gate_pass") == Some(false)
                    || b("permanent_structure_damage") == Some(true))
            {
                Some("FAIL_HARD_GATE")
            } else if b("hard_opening_gate_pass") == Some(true)
                && b("permanent_structure_damage") == Some(false)
            {
                Some("PASS_SCOPED_GATE")
            } else {
                Some("BLOCK_UNQUALIFIED")
            }
        }
        _ => None,
    }
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

    fn corpus() -> Value {
        let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../docs/LKI_01_REPAIRABILITY_BENCHMARK_EXTENSION_V0_4_DRAFT.json");
        serde_json::from_str(&fs::read_to_string(path).expect("v0.4 corpus fixture must be readable"))
            .expect("v0.4 corpus fixture must be valid JSON")
    }

    #[test]
    fn all_frozen_extension_scenarios_match_the_oracle() {
        let corpus = corpus();
        let scenarios = corpus["scenarios"].as_array().unwrap();
        assert_eq!(scenarios.len(), SCENARIO_COUNT);
        for scenario in scenarios {
            assert_eq!(
                evaluate_scenario(scenario),
                scenario["expected_outcome"].as_str(),
                "scenario {}",
                scenario["scenario_id"]
            );
        }
    }

    #[test]
    fn unknown_scenario_fails_closed() {
        let scenario = json!({
            "scenario_id":"UNKNOWN-999",
            "setup":{}
        });
        assert_eq!(evaluate_scenario(&scenario), None);
    }

    #[test]
    fn high_repairability_score_cannot_override_damage() {
        let mut scenario = corpus()["scenarios"]
            .as_array()
            .unwrap()
            .iter()
            .find(|case| case["scenario_id"] == "SCORE-001")
            .unwrap()
            .clone();
        scenario["permanent_structure_damage"] = json!(false);
        scenario["setup"]["permanent_structure_damage"] = json!(true);
        assert_eq!(evaluate_scenario(&scenario), Some("FAIL_HARD_GATE"));
    }

    #[test]
    fn missing_parts_wait_remains_inconclusive() {
        let scenario = corpus()["scenarios"]
            .as_array()
            .unwrap()
            .iter()
            .find(|case| case["scenario_id"] == "REPAIR-PHASE-002")
            .unwrap()
            .clone();
        assert_eq!(evaluate_scenario(&scenario), Some("INCONCLUSIVE_NO_CLAIM"));
    }

    #[test]
    fn unverified_repair_is_not_restored() {
        let mut scenario = corpus()["scenarios"]
            .as_array()
            .unwrap()
            .iter()
            .find(|case| case["scenario_id"] == "RESTORE-002")
            .unwrap()
            .clone();
        scenario["setup"]["post_repair_verification_passed"] = json!(false);
        assert_eq!(evaluate_scenario(&scenario), Some("BLOCK_UNQUALIFIED"));
    }
}
