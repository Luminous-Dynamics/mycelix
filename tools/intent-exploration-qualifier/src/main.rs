use serde_json::Value;
use std::{collections::BTreeSet, env, fs, process::ExitCode};

const EXPECTED_CASES: usize = 100;
const EXPECTED_FAMILIES: usize = 20;

#[derive(Debug, PartialEq, Eq)]
struct ValidationReport {
    cases: usize,
    families: usize,
    properties: usize,
    gates: usize,
}

fn non_empty_string<'a>(value: &'a Value, field: &str) -> Option<&'a str> {
    value.get(field)?.as_str().filter(|s| !s.trim().is_empty())
}

fn add_error(errors: &mut Vec<String>, context: &str, message: impl AsRef<str>) {
    errors.push(format!("{context}: {}", message.as_ref()));
}

/// Validate the shape and cross-references of both draft manifests.
/// This is deliberately not a semantic judge: it cannot qualify UX outcomes.
fn validate(benchmark: &Value, gates: &Value) -> Result<ValidationReport, Vec<String>> {
    let mut errors = Vec::new();

    if non_empty_string(benchmark, "schema_version") != Some("intent-exploration-benchmark-v0.1") {
        add_error(&mut errors, "benchmark", "unexpected or missing schema_version");
    }
    if non_empty_string(benchmark, "status")
        .is_none_or(|status| !status.contains("draft") || !status.contains("unqualified"))
    {
        add_error(&mut errors, "benchmark", "status must clearly mark the dataset as draft and unqualified");
    }

    let cases = match benchmark.get("cases").and_then(Value::as_array) {
        Some(cases) => cases,
        None => {
            add_error(&mut errors, "benchmark", "cases must be an array");
            return Err(errors);
        }
    };
    if cases.len() != EXPECTED_CASES {
        add_error(&mut errors, "benchmark", format!("expected {EXPECTED_CASES} cases, found {}", cases.len()));
    }
    if benchmark.get("count").and_then(Value::as_u64) != Some(cases.len() as u64) {
        add_error(&mut errors, "benchmark", "count must equal cases.length");
    }

    let properties = match benchmark.get("property_registry").and_then(Value::as_object) {
        Some(properties) if !properties.is_empty() => properties,
        _ => {
            add_error(&mut errors, "benchmark", "property_registry must be a non-empty object");
            return Err(errors);
        }
    };

    let family_list = match benchmark.get("families").and_then(Value::as_array) {
        Some(families) => families,
        None => {
            add_error(&mut errors, "benchmark", "families must be an array");
            return Err(errors);
        }
    };
    if family_list.len() != EXPECTED_FAMILIES {
        add_error(&mut errors, "benchmark", format!("expected {EXPECTED_FAMILIES} families, found {}", family_list.len()));
    }

    let mut family_by_prefix = std::collections::BTreeMap::new();
    let mut declared_case_total = 0usize;
    for family in family_list {
        let prefix = non_empty_string(family, "id_prefix");
        let name = non_empty_string(family, "name");
        let product = non_empty_string(family, "product");
        let risk = non_empty_string(family, "risk_class");
        let declared_count = family.get("case_count").and_then(Value::as_u64);

        if prefix.is_none() || name.is_none() || product.is_none() || risk.is_none() || declared_count.is_none() {
            add_error(&mut errors, "families", "each family needs id_prefix, name, product, risk_class, and case_count");
            continue;
        }
        let prefix = prefix.unwrap();
        if family_by_prefix.insert(prefix.to_string(), (name.unwrap().to_string(), product.unwrap().to_string(), risk.unwrap().to_string(), declared_count.unwrap() as usize)).is_some() {
            add_error(&mut errors, "families", format!("duplicate id_prefix {prefix}"));
        }
        declared_case_total += declared_count.unwrap() as usize;
    }
    if declared_case_total != cases.len() {
        add_error(&mut errors, "families", format!("declared case counts total {declared_case_total}, but cases.length is {}", cases.len()));
    }

    let mut ids = BTreeSet::new();
    let mut actual_counts = std::collections::BTreeMap::<String, usize>::new();
    let valid_gate_classes = ["quality", "review", "release-blocking-or-review"];
    for (index, case) in cases.iter().enumerate() {
        let context = format!("cases[{index}]");
        let required_text = [
            "case_id", "family", "product", "risk_class", "user_input",
            "setup_or_ambiguity", "expected_behavior", "forbidden_behavior",
            "oracle_type", "status", "gate_class",
        ];
        for field in required_text {
            if non_empty_string(case, field).is_none() {
                add_error(&mut errors, &context, format!("{field} must be a non-empty string"));
            }
        }

        let Some(case_id) = non_empty_string(case, "case_id") else { continue };
        if !ids.insert(case_id.to_string()) {
            add_error(&mut errors, &context, format!("duplicate case_id {case_id}"));
        }
        let parts: Vec<&str> = case_id.split('-').collect();
        if parts.len() != 3 || parts[0] != "IEB" || parts[2].len() != 2 || !parts[2].chars().all(|c| c.is_ascii_digit()) {
            add_error(&mut errors, &context, format!("invalid case_id format {case_id}; expected IEB-PREFIX-NN"));
            continue;
        }
        let prefix = parts[1];
        let Some((family_name, product, risk_class, _)) = family_by_prefix.get(prefix) else {
            add_error(&mut errors, &context, format!("case prefix {prefix} is not declared"));
            continue;
        };
        if non_empty_string(case, "family") != Some(family_name.as_str()) {
            add_error(&mut errors, &context, format!("{case_id} family does not match the family registry"));
        }
        if non_empty_string(case, "product") != Some(product.as_str()) {
            add_error(&mut errors, &context, format!("{case_id} product does not match the family registry"));
        }
        if non_empty_string(case, "risk_class") != Some(risk_class.as_str()) {
            add_error(&mut errors, &context, format!("{case_id} risk_class does not match the family registry"));
        }
        *actual_counts.entry(prefix.to_string()).or_default() += 1;

        if non_empty_string(case, "status").is_none_or(|status| !status.contains("not-yet-qualified")) {
            add_error(&mut errors, &context, format!("{case_id} must remain marked not-yet-qualified"));
        }
        if !valid_gate_classes.contains(&non_empty_string(case, "gate_class").unwrap_or_default()) {
            add_error(&mut errors, &context, format!("{case_id} has an invalid gate_class"));
        }
        let Some(assertions) = case.get("assertions").and_then(Value::as_array) else {
            add_error(&mut errors, &context, "assertions must be a non-empty array");
            continue;
        };
        if assertions.is_empty() {
            add_error(&mut errors, &context, "assertions must be a non-empty array");
        }

        for property_field in ["required_properties", "prohibited_outcomes"] {
            let Some(codes) = case.get(property_field).and_then(Value::as_array) else {
                add_error(&mut errors, &context, format!("{property_field} must be a non-empty array"));
                continue;
            };
            if codes.is_empty() {
                add_error(&mut errors, &context, format!("{property_field} must be a non-empty array"));
            }
            let mut unique_codes = BTreeSet::new();
            for code in codes {
                let Some(code) = code.as_str() else {
                    add_error(&mut errors, &context, format!("{property_field} entries must be strings"));
                    continue;
                };
                if !properties.contains_key(code) {
                    add_error(&mut errors, &context, format!("{property_field} references unknown property {code}"));
                }
                if !unique_codes.insert(code) {
                    add_error(&mut errors, &context, format!("{property_field} repeats property {code}"));
                }
            }
        }
    }
    for (prefix, (_, _, _, declared)) in &family_by_prefix {
        let actual = actual_counts.get(prefix).copied().unwrap_or_default();
        if actual != *declared {
            add_error(&mut errors, "families", format!("{prefix} declares {declared} cases but has {actual}"));
        }
    }

    if non_empty_string(gates, "schema_version") != Some("intent-exploration-qualification-gates-v0.1") {
        add_error(&mut errors, "gates", "unexpected or missing schema_version");
    }
    if non_empty_string(gates, "status")
        .is_none_or(|status| !status.contains("proposed") || !status.contains("no human study"))
    {
        add_error(&mut errors, "gates", "status must mark targets proposed and state that no human study has qualified them");
    }
    if non_empty_string(gates, "no_aggregate_pass_rule").is_none() {
        add_error(&mut errors, "gates", "no_aggregate_pass_rule is required");
    }

    let gate_list = match gates.get("gates").and_then(Value::as_array) {
        Some(gates) if !gates.is_empty() => gates,
        _ => {
            add_error(&mut errors, "gates", "gates must be a non-empty array");
            return Err(errors);
        }
    };
    let required_gates = [
        "IEQ-001", "IEQ-002", "IEQ-003", "IEQ-004", "IEQ-005",
        "IEQ-006", "IEQ-008", "IEQ-011", "IEQ-012",
    ];
    let mut gate_ids = BTreeSet::new();
    let mut present_gate_ids = BTreeSet::new();
    for (index, gate) in gate_list.iter().enumerate() {
        let context = format!("gates[{index}]");
        for field in ["id", "name", "severity", "target", "method", "evidence"] {
            if non_empty_string(gate, field).is_none() {
                add_error(&mut errors, &context, format!("{field} must be a non-empty string"));
            }
        }
        let Some(id) = non_empty_string(gate, "id") else { continue };
        if !gate_ids.insert(id.to_string()) {
            add_error(&mut errors, &context, format!("duplicate gate id {id}"));
        }
        present_gate_ids.insert(id.to_string());
        if !["blocking", "review", "quality"].contains(&non_empty_string(gate, "severity").unwrap_or_default()) {
            add_error(&mut errors, &context, format!("{id} has an invalid severity"));
        }
    }
    for required in required_gates {
        if !present_gate_ids.contains(required) {
            add_error(&mut errors, "gates", format!("required gate {required} is missing"));
        }
    }

    if errors.is_empty() {
        Ok(ValidationReport {
            cases: cases.len(),
            families: family_list.len(),
            properties: properties.len(),
            gates: gate_list.len(),
        })
    } else {
        Err(errors)
    }
}

fn main() -> ExitCode {
    let args: Vec<String> = env::args().collect();
    if args.len() != 3 {
        eprintln!(
            "Usage: {} <intent-exploration-benchmark.json> <qualification-gates.json>",
            args.first().map(String::as_str).unwrap_or("intent-exploration-qualifier")
        );
        eprintln!("Run from the repository root. This checks manifest integrity only, not UX outcomes.");
        return ExitCode::from(2);
    }

    let benchmark_text = match fs::read_to_string(&args[1]) {
        Ok(content) => content,
        Err(error) => {
            eprintln!("Failed to read benchmark {}: {error}", args[1]);
            return ExitCode::FAILURE;
        }
    };
    let gates_text = match fs::read_to_string(&args[2]) {
        Ok(content) => content,
        Err(error) => {
            eprintln!("Failed to read gates {}: {error}", args[2]);
            return ExitCode::FAILURE;
        }
    };
    let benchmark: Value = match serde_json::from_str(&benchmark_text) {
        Ok(value) => value,
        Err(error) => {
            eprintln!("Benchmark JSON parse failed: {error}");
            return ExitCode::FAILURE;
        }
    };
    let gates: Value = match serde_json::from_str(&gates_text) {
        Ok(value) => value,
        Err(error) => {
            eprintln!("Qualification-gates JSON parse failed: {error}");
            return ExitCode::FAILURE;
        }
    };

    match validate(&benchmark, &gates) {
        Ok(report) => {
            println!(
                "PASS: manifest integrity only — {} cases, {} families, {} registered properties, {} gates.",
                report.cases, report.families, report.properties, report.gates
            );
            println!("NOT QUALIFIED: semantic behavior, safety, accessibility and user outcomes require separate tests.");
            ExitCode::SUCCESS
        }
        Err(errors) => {
            eprintln!("FAIL: {} manifest-integrity error(s).", errors.len());
            for error in errors {
                eprintln!(" - {error}");
            }
            ExitCode::FAILURE
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const BENCHMARK: &str = include_str!("../../../docs/research/experience/intent-exploration-benchmark-v0.1.json");
    const GATES: &str = include_str!("../../../docs/research/experience/intent-exploration-qualification-gates-v0.1.json");

    #[test]
    fn checked_in_manifests_have_consistent_structure() {
        let benchmark: Value = serde_json::from_str(BENCHMARK).expect("benchmark JSON should parse");
        let gates: Value = serde_json::from_str(GATES).expect("gates JSON should parse");
        let report = validate(&benchmark, &gates).expect("manifests should pass structural validation");
        assert_eq!(report.cases, EXPECTED_CASES);
        assert_eq!(report.families, EXPECTED_FAMILIES);
        assert!(report.properties >= 40);
        assert!(report.gates >= 10);
    }

    #[test]
    fn duplicate_case_ids_fail_closed() {
        let mut benchmark: Value = serde_json::from_str(BENCHMARK).expect("benchmark JSON should parse");
        let first_id = benchmark["cases"][0]["case_id"].as_str().unwrap().to_string();
        benchmark["cases"][1]["case_id"] = Value::String(first_id);
        let gates: Value = serde_json::from_str(GATES).expect("gates JSON should parse");
        let errors = validate(&benchmark, &gates).expect_err("duplicate IDs must fail");
        assert!(errors.iter().any(|error| error.contains("duplicate case_id")));
    }

    #[test]
    fn unknown_property_references_fail_closed() {
        let mut benchmark: Value = serde_json::from_str(BENCHMARK).expect("benchmark JSON should parse");
        benchmark["cases"][0]["required_properties"][0] = Value::String("invented_property".to_string());
        let gates: Value = serde_json::from_str(GATES).expect("gates JSON should parse");
        let errors = validate(&benchmark, &gates).expect_err("unknown properties must fail");
        assert!(errors.iter().any(|error| error.contains("unknown property")));
    }

    #[test]
    fn gate_manifest_must_keep_safety_coverage() {
        let benchmark: Value = serde_json::from_str(BENCHMARK).expect("benchmark JSON should parse");
        let mut gates: Value = serde_json::from_str(GATES).expect("gates JSON should parse");
        gates["gates"].as_array_mut().unwrap().retain(|gate| gate["id"] != "IEQ-003");
        let errors = validate(&benchmark, &gates).expect_err("removing authorization gate must fail");
        assert!(errors.iter().any(|error| error.contains("required gate IEQ-003 is missing")));
    }
}
