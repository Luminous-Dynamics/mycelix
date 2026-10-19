use jsonschema::draft202012;
use serde_json::{json, Value};
use sha2::{Digest, Sha256};
use std::collections::HashSet;
use std::env;
use std::fs;
use std::path::{Path, PathBuf};

const TOOL_VERSION: &str = env!("CARGO_PKG_VERSION");
const CORPUS_VERSION: &str = "0.3.0-draft";
const VALID_OUTCOMES: [&str; 4] = [
    "PASS_SCOPED_GATE",
    "BLOCK_UNQUALIFIED",
    "FAIL_HARD_GATE",
    "INCONCLUSIVE_NO_CLAIM",
];

#[derive(Debug, Clone)]
struct Issue {
    severity: &'static str,
    code: &'static str,
    path: String,
    message: String,
}

#[derive(Debug)]
struct Args {
    schema: PathBuf,
    record: PathBuf,
    benchmark: PathBuf,
    as_of: String,
    freshness_days: i64,
}

fn main() {
    match parse_args() {
        Ok(None) => {}
        Ok(Some(args)) => match run(args) {
            Ok(report) => {
                let failed = report.get("status").and_then(Value::as_str) == Some("FAIL");
                println!("{}", serde_json::to_string_pretty(&report).unwrap_or_else(|_| "{}".into()));
                if failed {
                    std::process::exit(1);
                }
            }
            Err(error) => {
                eprintln!("lifetime-product-audit: {error}");
                std::process::exit(2);
            }
        },
        Err(error) => {
            eprintln!("lifetime-product-audit: {error}");
            eprintln!("Run with --help for usage.");
            std::process::exit(2);
        }
    }
}

fn parse_args() -> Result<Option<Args>, String> {
    let mut schema = None;
    let mut record = None;
    let mut benchmark = None;
    let mut as_of = None;
    let mut freshness_days = 30_i64;
    let mut values = env::args().skip(1);

    while let Some(arg) = values.next() {
        match arg.as_str() {
            "--help" | "-h" => {
                println!(
                    "Lifetime Product Audit (draft)\n\
                     \n\
                     Usage:\n\
                     cargo run --manifest-path mycelix-workspace/happs/fabrication/tools/lifetime-product-audit/Cargo.toml -- \\\n                     --schema mycelix-workspace/happs/fabrication/docs/LIFETIME_PRODUCT_RECORD_SCHEMA_V1_DRAFT.json \\\n                     --record mycelix-workspace/happs/fabrication/docs/LKI_01_PRODUCT_RECORD_EXAMPLE_DRAFT.json \\\n                     --benchmark mycelix-workspace/happs/fabrication/docs/LKI_01_LIFETIME_BENCHMARK_CORPUS_V0_1_DRAFT.json \\\n                     --as-of YYYY-MM-DD [--freshness-days N]\n\
                     \n\
                     Offline-only: this tool never fetches remote schemas or supplier pages.\n\
                     Status PASS means only that the supplied draft artifacts passed these local checks."
                );
                return Ok(None);
            }
            "--schema" => schema = Some(PathBuf::from(next_value(&mut values, "--schema")?)),
            "--record" => record = Some(PathBuf::from(next_value(&mut values, "--record")?)),
            "--benchmark" => benchmark = Some(PathBuf::from(next_value(&mut values, "--benchmark")?)),
            "--as-of" => as_of = Some(next_value(&mut values, "--as-of")?),
            "--freshness-days" => {
                let raw = next_value(&mut values, "--freshness-days")?;
                freshness_days = raw.parse::<i64>()
                    .map_err(|_| "--freshness-days must be a non-negative integer".to_string())?;
                if freshness_days < 0 {
                    return Err("--freshness-days must be a non-negative integer".to_string());
                }
            }
            other => return Err(format!("unknown argument: {other}")),
        }
    }

    let args = Args {
        schema: schema.ok_or("--schema is required")?,
        record: record.ok_or("--record is required")?,
        benchmark: benchmark.ok_or("--benchmark is required")?,
        as_of: as_of.ok_or("--as-of is required for deterministic time-sensitive checks")?,
        freshness_days,
    };
    if date_to_days(&args.as_of).is_none() {
        return Err("--as-of must be a valid YYYY-MM-DD calendar date".to_string());
    }
    Ok(Some(args))
}

fn next_value(values: &mut impl Iterator<Item = String>, flag: &str) -> Result<String, String> {
    values.next().ok_or_else(|| format!("{flag} requires a value"))
}

fn run(args: Args) -> Result<Value, String> {
    let (schema, schema_bytes, schema_digest) = read_json(&args.schema)?;
    let (record, record_bytes, record_digest) = read_json(&args.record)?;
    let (benchmark, benchmark_bytes, benchmark_digest) = read_json(&args.benchmark)?;

    let mut issues = Vec::new();
    let schema_meta_valid = match draft202012::meta::validate(&schema) {
        Ok(()) => true,
        Err(error) => {
            push_issue(
                &mut issues, "error", "SCHEMA_META_INVALID", "$schema",
                format!("Schema is not valid Draft 2020-12: {error}"),
            );
            false
        }
    };

    let validator = match draft202012::options()
        .should_validate_formats(true)
        .build(&schema)
    {
        Ok(validator) => Some(validator),
        Err(error) => {
            push_issue(
                &mut issues, "error", "SCHEMA_COMPILE_FAILED", "$schema",
                format!("Unable to compile the schema with format assertions enabled: {error}"),
            );
            None
        }
    };

    let mut record_schema_valid = false;
    if let Some(validator) = validator {
        let errors: Vec<String> = validator.iter_errors(&record)
            .map(|error| format!("{}: {}", error.instance_path(), error))
            .collect();
        if errors.is_empty() {
            record_schema_valid = true;
        } else {
            for error in errors {
                push_issue(
                    &mut issues, "error", "RECORD_SCHEMA_INVALID", "$record",
                    error,
                );
            }
        }
    }

    validate_record_semantics(&record, &args.as_of, args.freshness_days, &mut issues);
    let scenario_count = validate_benchmark(&benchmark, &mut issues);

    let error_count = issues.iter().filter(|i| i.severity == "error").count();
    let warning_count = issues.iter().filter(|i| i.severity == "warning").count();
    let issue_values: Vec<Value> = issues.iter().map(|i| json!({
        "severity": i.severity,
        "code": i.code,
        "path": i.path,
        "message": i.message
    })).collect();

    Ok(json!({
        "tool": "lifetime-product-audit",
        "tool_version": TOOL_VERSION,
        "status": if error_count == 0 { "PASS" } else { "FAIL" },
        "scope": "draft schema, example record and frozen benchmark corpus only",
        "as_of": args.as_of,
        "freshness_days": args.freshness_days,
        "checks": {
            "schema_meta_valid": schema_meta_valid,
            "record_schema_valid": record_schema_valid,
            "benchmark_scenarios_structurally_validated": scenario_count,
            "physical_product_tested": false,
            "safety_or_conformity_established": false,
            "supplier_availability_established": false
        },
        "input_digests_sha256": {
            "schema": {"path": args.schema.display().to_string(), "sha256": schema_digest, "bytes": schema_bytes.len()},
            "record": {"path": args.record.display().to_string(), "sha256": record_digest, "bytes": record_bytes.len()},
            "benchmark": {"path": args.benchmark.display().to_string(), "sha256": benchmark_digest, "bytes": benchmark_bytes.len()}
        },
        "error_count": error_count,
        "warning_count": warning_count,
        "issues": issue_values
    }))
}

fn read_json(path: &Path) -> Result<(Value, Vec<u8>, String), String> {
    let bytes = fs::read(path).map_err(|error| format!("cannot read {}: {error}", path.display()))?;
    let value = serde_json::from_slice::<Value>(&bytes)
        .map_err(|error| format!("invalid JSON in {}: {error}", path.display()))?;
    let digest = sha256_hex(&bytes);
    Ok((value, bytes, digest))
}

fn sha256_hex(bytes: &[u8]) -> String {
    let digest = Sha256::digest(bytes);
    digest.iter().map(|byte| format!("{byte:02x}")).collect()
}

fn push_issue(
    issues: &mut Vec<Issue>,
    severity: &'static str,
    code: &'static str,
    path: impl Into<String>,
    message: impl Into<String>,
) {
    issues.push(Issue {
        severity,
        code,
        path: path.into(),
        message: message.into(),
    });
}

fn validate_record_semantics(record: &Value, as_of: &str, freshness_days: i64, issues: &mut Vec<Issue>) {
    let components = record.pointer("/bom/items").and_then(Value::as_array);
    let interfaces = record.get("interfaces").and_then(Value::as_array);
    let lifecycle = record.get("lifecycle_state").and_then(Value::as_str).unwrap_or("unknown");

    let mut component_ids = HashSet::new();
    if let Some(components) = components {
        for (index, component) in components.iter().enumerate() {
            let path = format!("$.bom.items[{index}]");
            let component_id = component.get("component_id").and_then(Value::as_str).unwrap_or("");
            if component_id.is_empty() || !component_ids.insert(component_id.to_string()) {
                push_issue(
                    issues, "error", "BOM_COMPONENT_ID_DUPLICATE_OR_EMPTY", format!("{path}.component_id"),
                    "Component identifiers must be non-empty and unique within a BOM revision.",
                );
            }
        }
    }

    let mut interface_ids = HashSet::new();
    if let Some(interfaces) = interfaces {
        for (index, interface) in interfaces.iter().enumerate() {
            let path = format!("$.interfaces[{index}]");
            let interface_id = interface.get("interface_id").and_then(Value::as_str).unwrap_or("");
            if interface_id.is_empty() || !interface_ids.insert(interface_id.to_string()) {
                push_issue(
                    issues, "error", "INTERFACE_ID_DUPLICATE_OR_EMPTY", format!("{path}.interface_id"),
                    "Interface identifiers must be non-empty and unique within a product revision.",
                );
            }
            if let Some(violation) = interface_fit_violation(interface) {
                push_issue(
                    issues, "error", "MODULE_ENVELOPE_EXCEEDS_BAY",
                    format!("{path}.module_envelope_mm"),
                    violation,
                );
            }

            for field in ["installation_evidence_refs", "compatibility_test_refs"] {
                if let Some(refs) = interface.get(field).and_then(Value::as_array) {
                    for (ref_index, reference) in refs.iter().enumerate() {
                        let evidence_id = reference.as_str().unwrap_or("");
                        if evidence_record(record, evidence_id).is_none() {
                            push_issue(
                                issues, "error", "INTERFACE_EVIDENCE_REF_UNRESOLVED",
                                format!("{path}.{field}[{ref_index}]"),
                                format!("Evidence reference '{evidence_id}' does not resolve to a top-level evidence record."),
                            );
                        }
                    }
                }
            }

            let verification_state = interface.get("verification_state")
                .and_then(Value::as_str).unwrap_or("proposed");
            let installation_refs = interface.get("installation_evidence_refs")
                .and_then(Value::as_array).cloned().unwrap_or_default();
            if matches!(verification_state, "engineering_reviewed" | "physically_verified" | "qualified")
                && !installation_evidence_is_admissible(record, &installation_refs)
            {
                push_issue(
                    issues, "error", "INTERFACE_INSTALLATION_EVIDENCE_MISSING",
                    format!("{path}.installation_evidence_refs"),
                    "An engineering-reviewed, physically verified, or qualified interface must resolve to a digest-bound installation datasheet or certification document that is not superseded/retracted.",
                );
            }

            if matches!(verification_state, "physically_verified" | "qualified") {
                let compatibility_refs = interface.get("compatibility_test_refs")
                    .and_then(Value::as_array).cloned().unwrap_or_default();
                if !compatibility_test_evidence_is_admissible(
                    record,
                    &compatibility_refs,
                    verification_state == "qualified",
                ) {
                    push_issue(
                        issues, "error", "INTERFACE_COMPATIBILITY_TEST_MISSING",
                        format!("{path}.compatibility_test_refs"),
                        if verification_state == "qualified" {
                            "A qualified interface requires a digest-bound, independently verified test/inspection record for this configuration."
                        } else {
                            "A physically verified interface requires a digest-bound measured/independently verified test/inspection record for this configuration."
                        },
                    );
                }
            }
        }
    }

    if let Some(components) = components {
        for (index, component) in components.iter().enumerate() {
            let path = format!("$.bom.items[{index}]");
            if let Some(refs) = component.get("interface_refs").and_then(Value::as_array) {
                for (ref_index, reference) in refs.iter().enumerate() {
                    let reference = reference.as_str().unwrap_or("");
                    if !interface_ids.contains(reference) {
                        push_issue(
                            issues, "error", "INTERFACE_REFERENCE_UNRESOLVED",
                            format!("{path}.interface_refs[{ref_index}]"),
                            format!("Interface reference '{reference}' does not resolve in this product record."),
                        );
                    }
                }
            }

            if component.get("supplier_validation_state").and_then(Value::as_str) == Some("qualified") {
                let qualified_source_present = component.get("supplier_records")
                    .and_then(Value::as_array)
                    .map(|records| records.iter().any(|source| {
                        matches!(
                            source.get("evidence_state").and_then(Value::as_str),
                            Some("sample_inspected") | Some("qualified")
                        )
                    }))
                    .unwrap_or(false);
                if !qualified_source_present {
                    push_issue(
                        issues, "error", "QUALIFIED_COMPONENT_WITHOUT_QUALIFICATION_EVIDENCE",
                        format!("{path}.supplier_records"),
                        "A component cannot be marked qualified without a linked sample-inspection or qualification record.",
                    );
                }
            }

            if let Some(suppliers) = component.get("supplier_records").and_then(Value::as_array) {
                for (supplier_index, supplier) in suppliers.iter().enumerate() {
                    let supplier_path = format!("{path}.supplier_records[{supplier_index}]");
                    let checked_at = supplier.get("checked_at").and_then(Value::as_str).unwrap_or("");
                    match (date_to_days(checked_at), date_to_days(as_of)) {
                        (Some(checked_day), Some(as_of_day)) if checked_day > as_of_day => {
                            push_issue(
                                issues, "error", "SUPPLIER_EVIDENCE_FROM_FUTURE",
                                format!("{supplier_path}.checked_at"),
                                "Supplier evidence cannot be dated after the explicit audit date.",
                            );
                        }
                        (Some(checked_day), Some(as_of_day))
                            if as_of_day - checked_day > freshness_days
                                && supplier.get("evidence_state").and_then(Value::as_str) == Some("availability_confirmed") =>
                        {
                            push_issue(
                                issues, "warning", "SUPPLIER_AVAILABILITY_STALE",
                                format!("{supplier_path}.checked_at"),
                                format!("Availability was checked {} days before the audit date, exceeding the configured freshness window of {freshness_days} days.", as_of_day - checked_day),
                            );
                        }
                        (None, _) => push_issue(
                            issues, "error", "SUPPLIER_CHECK_DATE_INVALID",
                            format!("{supplier_path}.checked_at"),
                            "Supplier evidence requires a valid YYYY-MM-DD checked_at date.",
                        ),
                        _ => {}
                    }

                    if let Some(expiry) = supplier.get("quote_valid_until").and_then(Value::as_str) {
                        match (date_to_days(expiry), date_to_days(as_of)) {
                            (Some(expiry_day), Some(as_of_day)) if expiry_day < as_of_day => push_issue(
                                issues, "warning", "SUPPLIER_QUOTE_EXPIRED",
                                format!("{supplier_path}.quote_valid_until"),
                                "Quote validity has expired; do not present its price or lead time as current.",
                            ),
                            (None, _) => push_issue(
                                issues, "error", "SUPPLIER_QUOTE_EXPIRY_INVALID",
                                format!("{supplier_path}.quote_valid_until"),
                                "quote_valid_until must be a valid YYYY-MM-DD date when present.",
                            ),
                            _ => {}
                        }
                    }
                }
            }
        }
    }

    if let Some(gates) = record.get("release_gates").and_then(Value::as_array) {
        for (index, gate) in gates.iter().enumerate() {
            let path = format!("$.release_gates[{index}]");
            let status = gate.get("status").and_then(Value::as_str).unwrap_or("");
            let evidence_empty = gate.get("evidence_refs")
                .and_then(Value::as_array)
                .map(|refs| refs.is_empty())
                .unwrap_or(true);
            if status == "pass" && evidence_empty {
                push_issue(
                    issues, "error", "GATE_PASS_WITHOUT_EVIDENCE",
                    format!("{path}.evidence_refs"),
                    "A passing release gate must reference the evidence that justifies the pass.",
                );
            }
            if status == "not_applicable_with_rationale"
                && gate.get("rationale").and_then(Value::as_str).map(str::trim).unwrap_or("").is_empty()
            {
                push_issue(
                    issues, "error", "GATE_NA_WITHOUT_RATIONALE",
                    format!("{path}.rationale"),
                    "A not-applicable gate requires a rationale.",
                );
            }
            if lifecycle == "released" && !matches!(status, "pass" | "not_applicable_with_rationale") {
                push_issue(
                    issues, "error", "RELEASE_GATE_NOT_SATISFIED",
                    format!("{path}.status"),
                    "A released record cannot contain an untested, blocked, or failed release gate.",
                );
            }
        }
    }

    if let Some(metrics) = record.get("metrics").and_then(Value::as_array) {
        for (index, metric) in metrics.iter().enumerate() {
            let status = metric.get("status").and_then(Value::as_str).unwrap_or("");
            let evidence_empty = metric.get("evidence_refs")
                .and_then(Value::as_array)
                .map(|refs| refs.is_empty())
                .unwrap_or(true);
            if matches!(status, "bench_tested" | "independently_verified" | "field_observed") && evidence_empty {
                push_issue(
                    issues, "error", "MEASURED_METRIC_WITHOUT_EVIDENCE",
                    format!("$.metrics[{index}].evidence_refs"),
                    "A measured or independently verified metric must link to its supporting evidence.",
                );
            }
        }
    }

    if let Some(evidence) = record.get("evidence").and_then(Value::as_array) {
        for (index, entry) in evidence.iter().enumerate() {
            let status = entry.get("status").and_then(Value::as_str).unwrap_or("");
            let has_digest = entry.get("source_sha256").and_then(Value::as_str)
                .map(|s| s.len() == 64 && s.bytes().all(|b| b.is_ascii_hexdigit()))
                .unwrap_or(false);
            if matches!(status, "measured" | "independently_verified" | "field_observed") && !has_digest {
                push_issue(
                    issues, "warning", "EVIDENCE_DIGEST_MISSING",
                    format!("$.evidence[{index}].source_sha256"),
                    "Measured/verified evidence should carry a SHA-256 digest of the exact source artifact.",
                );
            }
        }
    }
}

fn interface_fit_violation(interface: &Value) -> Option<String> {
    const AXES: [&str; 3] = ["width", "depth", "height"];
    for axis in AXES {
        let module = interface.pointer(&format!("/module_envelope_mm/{axis}"))?.as_f64()?;
        let bay = interface.pointer(&format!("/bay_clear_envelope_mm/{axis}"))?.as_f64()?;
        let allowance = interface.pointer(&format!("/minimum_fit_clearance_mm/{axis}"))?.as_f64()?;
        if module + allowance > bay + 1e-9 {
            return Some(format!(
                "{axis}: module envelope ({module} mm) + minimum fit allowance ({allowance} mm) exceeds bay clear envelope ({bay} mm)."
            ));
        }
    }
    None
}

fn evidence_record<'a>(record: &'a Value, evidence_id: &str) -> Option<&'a Value> {
    record.get("evidence")?.as_array()?.iter().find(|entry| {
        entry.get("evidence_id").and_then(Value::as_str) == Some(evidence_id)
    })
}

fn has_source_digest(evidence: &Value) -> bool {
    evidence.get("source_sha256")
        .and_then(Value::as_str)
        .map(|digest| digest.len() == 64 && digest.bytes().all(|byte| byte.is_ascii_hexdigit()))
        .unwrap_or(false)
}

fn installation_evidence_is_admissible(record: &Value, references: &[Value]) -> bool {
    references.iter().filter_map(Value::as_str).any(|evidence_id| {
        let Some(evidence) = evidence_record(record, evidence_id) else { return false };
        matches!(
            evidence.get("kind").and_then(Value::as_str),
            Some("supplier_datasheet") | Some("certification_document")
        )
            && !matches!(
                evidence.get("status").and_then(Value::as_str),
                Some("superseded") | Some("retracted")
            )
            && has_source_digest(evidence)
    })
}

fn compatibility_test_evidence_is_admissible(
    record: &Value,
    references: &[Value],
    require_independent_verification: bool,
) -> bool {
    references.iter().filter_map(Value::as_str).any(|evidence_id| {
        let Some(evidence) = evidence_record(record, evidence_id) else { return false };
        let acceptable_status = if require_independent_verification {
            evidence.get("status").and_then(Value::as_str) == Some("independently_verified")
        } else {
            matches!(
                evidence.get("status").and_then(Value::as_str),
                Some("measured") | Some("independently_verified") | Some("field_observed")
            )
        };
        acceptable_status
            && matches!(
                evidence.get("kind").and_then(Value::as_str),
                Some("test_report") | Some("inspection")
            )
            && has_source_digest(evidence)
    })
}

fn validate_benchmark(benchmark: &Value, issues: &mut Vec<Issue>) -> usize {
    if benchmark.get("benchmark_version").and_then(Value::as_str) != Some(CORPUS_VERSION) {
        push_issue(
            issues, "error", "BENCHMARK_VERSION_UNEXPECTED", "$benchmark.benchmark_version",
            format!("This validator is pinned to corpus version {CORPUS_VERSION}; do not silently replace a frozen benchmark."),
        );
    }
    if benchmark.get("status").and_then(Value::as_str) != Some("specification_only") {
        push_issue(
            issues, "error", "BENCHMARK_STATUS_UNEXPECTED", "$benchmark.status",
            "This draft corpus must remain explicitly marked specification_only until a defined evaluator has been executed.",
        );
    }

    let claim_ceiling = benchmark.get("claim_ceiling").and_then(Value::as_str).unwrap_or("");
    if !claim_ceiling.to_ascii_lowercase().contains("unexecuted") {
        push_issue(
            issues, "error", "BENCHMARK_CLAIM_CEILING_MISSING", "$benchmark.claim_ceiling",
            "The benchmark must state that it has not yet been executed as evidence.",
        );
    }

    let invariants = benchmark.get("hard_invariants").and_then(Value::as_array);
    if invariants.map(|items| items.len()).unwrap_or(0) < 9 {
        push_issue(
            issues, "error", "BENCHMARK_INVARIANTS_INCOMPLETE", "$benchmark.hard_invariants",
            "Expected at least nine explicit hard invariants for this frozen draft.",
        );
    }

    let scenarios = match benchmark.get("scenarios").and_then(Value::as_array) {
        Some(scenarios) => scenarios,
        None => {
            push_issue(
                issues, "error", "BENCHMARK_SCENARIOS_MISSING", "$benchmark.scenarios",
                "Benchmark corpus must contain a scenarios array.",
            );
            return 0;
        }
    };
    if scenarios.len() != 17 {
        push_issue(
            issues, "error", "BENCHMARK_V0_3_SCENARIO_COUNT_CHANGED", "$benchmark.scenarios",
            format!("Frozen v0.3 corpus expects 17 scenarios; found {}. Revise the corpus version when adding/removing cases.", scenarios.len()),
        );
    }

    let mut ids = HashSet::new();
    let mut checked = 0;
    for (index, scenario) in scenarios.iter().enumerate() {
        let path = format!("$.scenarios[{index}]");
        let scenario_id = scenario.get("scenario_id").and_then(Value::as_str).unwrap_or("");
        if scenario_id.is_empty() || !ids.insert(scenario_id.to_string()) {
            push_issue(
                issues, "error", "BENCHMARK_SCENARIO_ID_DUPLICATE_OR_EMPTY",
                format!("{path}.scenario_id"),
                "Scenario IDs must be non-empty and unique.",
            );
            continue;
        }

        let title_ok = scenario.get("title").and_then(Value::as_str).map(|s| !s.trim().is_empty()).unwrap_or(false);
        let category_ok = scenario.get("category").and_then(Value::as_str).map(|s| !s.trim().is_empty()).unwrap_or(false);
        let setup_ok = scenario.get("setup").map(Value::is_object).unwrap_or(false);
        let assertions_ok = scenario.get("required_assertions").and_then(Value::as_array)
            .map(|items| !items.is_empty() && items.iter().all(Value::is_string))
            .unwrap_or(false);
        let expected = scenario.get("expected_outcome").and_then(Value::as_str).unwrap_or("");
        if !title_ok || !category_ok || !setup_ok || !assertions_ok || !VALID_OUTCOMES.contains(&expected) {
            push_issue(
                issues, "error", "BENCHMARK_SCENARIO_SHAPE_INVALID", path.clone(),
                "Scenario must include a title, category, object setup, valid expected outcome, and non-empty string required_assertions.",
            );
            continue;
        }

        match evaluate_scenario(scenario) {
            Some(actual) if actual == expected => checked += 1,
            Some(actual) => push_issue(
                issues, "error", "BENCHMARK_ORACLE_MISMATCH",
                format!("{path}.expected_outcome"),
                format!("Known-answer evaluator returned {actual}, but corpus expects {expected} for {scenario_id}."),
            ),
            None => push_issue(
                issues, "error", "BENCHMARK_SCENARIO_UNSUPPORTED",
                format!("{path}.scenario_id"),
                format!("No deterministic rule is implemented for scenario {scenario_id}; refusing to skip it."),
            ),
        }
    }

    if let Some(metrics) = benchmark.get("proposed_software_benchmarks").and_then(Value::as_array) {
        let expected_targets = [
            ("BENCH-FALSE-SAFE-PASS", 0.0),
            ("BENCH-STALE-EVIDENCE-DETECTION", 100.0),
            ("BENCH-REVISION-COHERENCE", 100.0),
            ("BENCH-DETERMINISTIC-REPLAY", 100.0),
            ("BENCH-HARD-GATE-NONCOMPENSATION", 100.0),
        ];
        for (metric_id, target) in expected_targets {
            let matching: Vec<&Value> = metrics.iter().filter(|metric| {
                metric.get("metric_id").and_then(Value::as_str) == Some(metric_id)
            }).collect();
            if matching.len() != 1 || matching[0].get("target").and_then(Value::as_f64) != Some(target) {
                push_issue(
                    issues, "error", "BENCHMARK_TARGET_MISMATCH",
                    "$benchmark.proposed_software_benchmarks",
                    format!("Expected exactly one {metric_id} metric with target {target}."),
                );
            }
        }
    } else {
        push_issue(
            issues, "error", "BENCHMARK_METRICS_MISSING", "$benchmark.proposed_software_benchmarks",
            "The draft corpus must retain its proposed software-evaluator benchmark section.",
        );
    }

    checked
}

fn evaluate_scenario(scenario: &Value) -> Option<&'static str> {
    let id = scenario.get("scenario_id")?.as_str()?;
    let setup = scenario.get("setup")?;
    let b = |key: &str| setup.get(key).and_then(Value::as_bool);
    let s = |key: &str| setup.get(key).and_then(Value::as_str);
    let n = |key: &str| setup.get(key).and_then(Value::as_i64);

    match id {
        "MOD-001" => {
            if b("module_qualified") == Some(true)
                && b("removal_path_verified") == Some(true)
                && b("service_clearance_pass") == Some(true)
                && b("permanent_structure_damage") == Some(false)
                && b("post_install_test_pass") == Some(true)
            { Some("PASS_SCOPED_GATE") } else { Some("BLOCK_UNQUALIFIED") }
        }
        "MOD-002" => {
            if b("mechanical_envelope_match") == Some(true) && b("electrical_rating_match") == Some(false) {
                Some("FAIL_HARD_GATE")
            } else if b("mechanical_envelope_match") == Some(true) && b("electrical_rating_match") == Some(true) {
                Some("PASS_SCOPED_GATE")
            } else { Some("BLOCK_UNQUALIFIED") }
        }
        "MOD-003" => {
            let mut complete = true;
            let mut fits = true;
            for axis in ["width", "depth", "height"] {
                let module = setup.pointer(&format!("/module_envelope_mm/{axis}")).and_then(Value::as_f64);
                let bay = setup.pointer(&format!("/bay_clear_envelope_mm/{axis}")).and_then(Value::as_f64);
                let allowance = setup.pointer(&format!("/minimum_fit_clearance_mm/{axis}")).and_then(Value::as_f64);
                match (module, bay, allowance) {
                    (Some(module), Some(bay), Some(allowance)) => {
                        if module + allowance > bay + 1e-9 {
                            fits = false;
                        }
                    }
                    _ => complete = false,
                }
            }
            if !complete {
                Some("BLOCK_UNQUALIFIED")
            } else if fits {
                Some("PASS_SCOPED_GATE")
            } else {
                Some("FAIL_HARD_GATE")
            }
        }
        "INTERFACE-001" => {
            let qualified = s("verification_state") == Some("qualified");
            let evidence_complete = b("installation_evidence_resolves") == Some(true)
                && b("installation_evidence_digest_valid") == Some(true)
                && b("compatibility_test_resolves") == Some(true)
                && b("compatibility_test_digest_valid") == Some(true)
                && b("compatibility_test_independently_verified") == Some(true);
            if qualified && evidence_complete {
                Some("PASS_SCOPED_GATE")
            } else {
                Some("BLOCK_UNQUALIFIED")
            }
        }
        "INTERFACE-002" => {
            let verified = s("verification_state") == Some("physically_verified");
            let install_evidence = b("installation_evidence_resolves") == Some(true)
                && b("installation_evidence_digest_valid") == Some(true);
            let test_evidence = b("compatibility_test_resolves") == Some(true)
                && b("compatibility_test_digest_valid") == Some(true)
                && matches!(s("compatibility_test_status"), Some("measured") | Some("independently_verified") | Some("field_observed"));
            if verified && install_evidence && test_evidence {
                Some("PASS_SCOPED_GATE")
            } else {
                Some("BLOCK_UNQUALIFIED")
            }
        }
        "THERM-001" => {
            if b("required_exhaust_clearance_met") == Some(false) {
                Some("FAIL_HARD_GATE")
            } else if b("required_exhaust_clearance_met") == Some(true) && b("refrigeration_energy_test_pass") == Some(true) {
                Some("PASS_SCOPED_GATE")
            } else {
                Some("INCONCLUSIVE_NO_CLAIM")
            }
        }
        "THERM-002" => {
            if b("thermal_boundary_defined") != Some(true)
                || b("instrumented_interaction_test") != Some(true)
                || b("qualified_safety_review") != Some(true)
            { Some("BLOCK_UNQUALIFIED") } else { Some("PASS_SCOPED_GATE") }
        }
        "SUP-001" => {
            if s("supplier_record_state") == Some("marketing_claim")
                && b("current_written_quote") != Some(true)
                && b("stock_confirmation") != Some(true)
                && b("sample_inspected") != Some(true)
            { Some("BLOCK_UNQUALIFIED") } else { Some("PASS_SCOPED_GATE") }
        }
        "SUP-002" => {
            if b("quote_expired") == Some(true) || n("last_availability_check_days_ago").unwrap_or(i64::MAX) > 30 {
                Some("BLOCK_UNQUALIFIED")
            } else { Some("PASS_SCOPED_GATE") }
        }
        "REV-001" => {
            if b("component_revision_changed") == Some(true)
                && b("bom_revision_changed") == Some(false)
                && b("change_record_present") == Some(false)
                && b("test_evidence_targets_old_revision") == Some(true)
            { Some("FAIL_HARD_GATE") } else { Some("PASS_SCOPED_GATE") }
        }
        "REP-001" => {
            if b("permanent_structure_damage") == Some(true) {
                Some("FAIL_HARD_GATE")
            } else if b("module_function_restored") == Some(true) {
                let times = setup.get("replacement_time_minutes").and_then(Value::as_array);
                if let Some(times) = times {
                    if !times.is_empty() && times.iter().all(|v| v.as_f64().map(|x| x <= 60.0).unwrap_or(false)) {
                        Some("PASS_SCOPED_GATE")
                    } else { Some("INCONCLUSIVE_NO_CLAIM") }
                } else { Some("INCONCLUSIVE_NO_CLAIM") }
            } else { Some("BLOCK_UNQUALIFIED") }
        }
        "ENERGY-001" => {
            if b("baseline_test_conditions_matched") != Some(true)
                || setup.get("candidate_capacity_litres").and_then(Value::as_f64)
                    != setup.get("baseline_capacity_litres").and_then(Value::as_f64)
            { Some("INCONCLUSIVE_NO_CLAIM") }
            else { Some("PASS_SCOPED_GATE") }
        }
        "LIFE-001" => {
            if n("design_horizon_years") == Some(50)
                && n("field_history_years") == Some(0)
                && b("accelerated_ageing_data") == Some(false)
                && b("validated_lifecycle_model") == Some(false)
            { Some("INCONCLUSIVE_NO_CLAIM") } else { Some("PASS_SCOPED_GATE") }
        }
        "SAFE-001" => {
            if b("geometry_check_pass") == Some(true)
                && b("rigid_body_simulation_pass") == Some(true)
                && b("electrical_safety_evidence") == Some(true)
                && b("thermal_test_evidence") == Some(true)
                && b("qualified_assessment") == Some(true)
            { Some("PASS_SCOPED_GATE") } else { Some("BLOCK_UNQUALIFIED") }
        }
        "LCA-001" => {
            if b("use_phase_energy_included") != Some(true)
                || b("repair_replacement_scenarios_included") != Some(true)
                || b("end_of_life_included") != Some(true)
                || b("comparison_boundary_matched") != Some(true)
            { Some("INCONCLUSIVE_NO_CLAIM") } else { Some("PASS_SCOPED_GATE") }
        }
        "PARETO-001" => {
            if b("required_service_clearance_pass") == Some(false) {
                Some("FAIL_HARD_GATE")
            } else if b("required_service_clearance_pass") == Some(true)
                && b("safety_gates_pass") == Some(true)
            { Some("PASS_SCOPED_GATE") } else { Some("BLOCK_UNQUALIFIED") }
        }
        "REPLAY-001" => {
            if b("frozen_input_digest_same") == Some(true)
                && b("evaluator_version_same") == Some(true)
                && b("report_digest_same") == Some(false)
            { Some("FAIL_HARD_GATE") }
            else if b("frozen_input_digest_same") == Some(true)
                && b("evaluator_version_same") == Some(true)
                && b("report_digest_same") == Some(true)
            { Some("PASS_SCOPED_GATE") }
            else { Some("BLOCK_UNQUALIFIED") }
        }
        _ => None,
    }
}

fn date_to_days(date: &str) -> Option<i64> {
    let bytes = date.as_bytes();
    if !date.is_ascii() || bytes.len() != 10 || bytes[4] != b'-' || bytes[7] != b'-' {
        return None;
    }
    let year = date[0..4].parse::<i64>().ok()?;
    let month = date[5..7].parse::<i64>().ok()?;
    let day = date[8..10].parse::<i64>().ok()?;
    if year < 1 || !(1..=12).contains(&month) || day < 1 {
        return None;
    }
    let leap = year % 4 == 0 && (year % 100 != 0 || year % 400 == 0);
    let days_in_month = match month {
        1 | 3 | 5 | 7 | 8 | 10 | 12 => 31,
        4 | 6 | 9 | 11 => 30,
        2 if leap => 29,
        2 => 28,
        _ => return None,
    };
    if day > days_in_month {
        return None;
    }
    // Proleptic Gregorian day count; only differences matter here.
    let adjusted_year = year - if month <= 2 { 1 } else { 0 };
    let era = adjusted_year.div_euclid(400);
    let year_of_era = adjusted_year - era * 400;
    let adjusted_month = month + if month > 2 { -3 } else { 9 };
    let day_of_year = (153 * adjusted_month + 2) / 5 + day - 1;
    let day_of_era = year_of_era * 365 + year_of_era / 4 - year_of_era / 100 + day_of_year;
    Some(era * 146097 + day_of_era)
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    fn repo_file(relative: &str) -> String {
        let root = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
        fs::read_to_string(root.join(relative)).expect("test fixture must be readable")
    }

    #[test]
    fn schema_and_example_record_pass_draft_2020_12_validation() {
        let schema: Value = serde_json::from_str(&repo_file("../../docs/LIFETIME_PRODUCT_RECORD_SCHEMA_V1_DRAFT.json")).unwrap();
        let record: Value = serde_json::from_str(&repo_file("../../docs/LKI_01_PRODUCT_RECORD_EXAMPLE_DRAFT.json")).unwrap();
        assert!(draft202012::meta::validate(&schema).is_ok());
        let validator = draft202012::options().should_validate_formats(true).build(&schema).unwrap();
        assert!(validator.is_valid(&record), "example must satisfy the exact schema");
    }

    #[test]
    fn benchmark_corpus_known_answers_match() {
        let benchmark: Value = serde_json::from_str(&repo_file("../../docs/LKI_01_LIFETIME_BENCHMARK_CORPUS_V0_3_DRAFT.json")).unwrap();
        let scenarios = benchmark.get("scenarios").and_then(Value::as_array).unwrap();
        assert_eq!(scenarios.len(), 17);
        assert_eq!(benchmark["benchmark_version"].as_str(), Some(CORPUS_VERSION));
        for scenario in scenarios {
            let expected = scenario["expected_outcome"].as_str().unwrap();
            assert_eq!(evaluate_scenario(scenario), Some(expected), "scenario {}", scenario["scenario_id"]);
        }
    }

    #[test]
    fn module_envelope_plus_fit_allowance_must_fit_inside_bay() {
        let mut record: Value = serde_json::from_str(&repo_file("../../docs/LKI_01_PRODUCT_RECORD_EXAMPLE_DRAFT.json")).unwrap();
        record["interfaces"][0]["module_envelope_mm"]["width"] = json!(615);
        record["interfaces"][0]["bay_clear_envelope_mm"]["width"] = json!(620);
        record["interfaces"][0]["minimum_fit_clearance_mm"]["width"] = json!(10);
        let mut issues = Vec::new();
        validate_record_semantics(&record, "2026-10-19", 30, &mut issues);
        assert!(issues.iter().any(|issue| issue.code == "MODULE_ENVELOPE_EXCEEDS_BAY"));
    }

    #[test]
    fn verified_interface_requires_resolved_installation_and_test_evidence() {
        let mut record: Value = serde_json::from_str(&repo_file("../../docs/LKI_01_PRODUCT_RECORD_EXAMPLE_DRAFT.json")).unwrap();
        record["interfaces"][0]["verification_state"] = json!("physically_verified");
        record["interfaces"][0]["installation_evidence_refs"] = json!(["INSTALL-001"]);
        record["interfaces"][0]["compatibility_test_refs"] = json!(["TEST-001"]);
        record["evidence"] = json!([
            {
                "evidence_id":"INSTALL-001",
                "kind":"supplier_datasheet",
                "status":"asserted",
                "subject_ids":["COLD-MODULE-001"],
                "recorded_at":"2026-10-19T06:00:00Z",
                "source_uri":"https://example.invalid/installation-guide",
                "source_sha256":"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
                "method_and_limitations":"Synthetic unit-test evidence only."
            },
            {
                "evidence_id":"TEST-001",
                "kind":"test_report",
                "status":"measured",
                "subject_ids":["COLD-MODULE-001","IF-COOL-001"],
                "recorded_at":"2026-10-19T06:05:00Z",
                "source_uri":"https://example.invalid/compatibility-test",
                "source_sha256":"bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb",
                "method_and_limitations":"Synthetic unit-test evidence only."
            }
        ]);
        let mut issues = Vec::new();
        validate_record_semantics(&record, "2026-10-19", 30, &mut issues);
        assert!(!issues.iter().any(|issue| issue.code == "INTERFACE_INSTALLATION_EVIDENCE_MISSING"));
        assert!(!issues.iter().any(|issue| issue.code == "INTERFACE_COMPATIBILITY_TEST_MISSING"));
        assert!(!issues.iter().any(|issue| issue.code == "INTERFACE_EVIDENCE_REF_UNRESOLVED"));
    }

    #[test]
    fn qualified_interface_cannot_use_unresolved_or_unverified_evidence() {
        let mut record: Value = serde_json::from_str(&repo_file("../../docs/LKI_01_PRODUCT_RECORD_EXAMPLE_DRAFT.json")).unwrap();
        record["interfaces"][0]["verification_state"] = json!("qualified");
        record["interfaces"][0]["installation_evidence_refs"] = json!(["missing-install-doc"]);
        record["interfaces"][0]["compatibility_test_refs"] = json!(["missing-test"]);
        let mut issues = Vec::new();
        validate_record_semantics(&record, "2026-10-19", 30, &mut issues);
        assert!(issues.iter().any(|issue| issue.code == "INTERFACE_EVIDENCE_REF_UNRESOLVED"));
        assert!(issues.iter().any(|issue| issue.code == "INTERFACE_INSTALLATION_EVIDENCE_MISSING"));
        assert!(issues.iter().any(|issue| issue.code == "INTERFACE_COMPATIBILITY_TEST_MISSING"));
    }

    #[test]
    fn module_bay_fit_known_answer_rejects_insufficient_width_clearance() {
        let scenario = json!({
            "scenario_id":"MOD-003",
            "setup":{
                "module_envelope_mm":{"width":620,"depth":600,"height":800},
                "bay_clear_envelope_mm":{"width":620,"depth":620,"height":830},
                "minimum_fit_clearance_mm":{"width":10,"depth":10,"height":10}
            }
        });
        assert_eq!(evaluate_scenario(&scenario), Some("FAIL_HARD_GATE"));
    }

    #[test]
    fn a_qualified_interface_without_resolvable_evidence_is_blocked() {
        let scenario = json!({
            "scenario_id":"INTERFACE-001",
            "setup":{
                "verification_state":"qualified",
                "installation_evidence_resolves":false,
                "installation_evidence_digest_valid":false,
                "compatibility_test_resolves":false,
                "compatibility_test_digest_valid":false,
                "compatibility_test_independently_verified":false
            }
        });
        assert_eq!(evaluate_scenario(&scenario), Some("BLOCK_UNQUALIFIED"));
    }

    #[test]
    fn a_physically_verified_interface_requires_digest_bound_install_and_test_evidence() {
        let scenario = json!({
            "scenario_id":"INTERFACE-002",
            "setup":{
                "verification_state":"physically_verified",
                "installation_evidence_resolves":true,
                "installation_evidence_digest_valid":true,
                "compatibility_test_resolves":true,
                "compatibility_test_digest_valid":true,
                "compatibility_test_status":"measured",
                "compatibility_test_independently_verified":false
            }
        });
        assert_eq!(evaluate_scenario(&scenario), Some("PASS_SCOPED_GATE"));
    }

    #[test]
    fn a_mechanically_fitting_but_electrically_incompatible_module_fails() {
        let scenario = json!({
            "scenario_id":"MOD-002",
            "setup":{"mechanical_envelope_match":true,"electrical_rating_match":false}
        });
        assert_eq!(evaluate_scenario(&scenario), Some("FAIL_HARD_GATE"));
    }

    #[test]
    fn damage_fails_repairability_even_when_replacement_is_fast() {
        let scenario = json!({
            "scenario_id":"REP-001",
            "setup":{
                "replacement_time_minutes":[42,48,44],
                "permanent_structure_damage":true,
                "module_function_restored":true
            }
        });
        assert_eq!(evaluate_scenario(&scenario), Some("FAIL_HARD_GATE"));
    }

    #[test]
    fn cheapest_candidate_cannot_override_failed_hard_clearance() {
        let scenario = json!({
            "scenario_id":"PARETO-001",
            "setup":{
                "purchase_cost_lowest":true,
                "required_service_clearance_pass":false,
                "safety_gates_pass":true,
                "other_candidates_feasible":true
            }
        });
        assert_eq!(evaluate_scenario(&scenario), Some("FAIL_HARD_GATE"));
    }

    #[test]
    fn passing_release_gate_without_evidence_is_rejected() {
        let mut record: Value = serde_json::from_str(&repo_file("../../docs/LKI_01_PRODUCT_RECORD_EXAMPLE_DRAFT.json")).unwrap();
        record["release_gates"][0]["status"] = json!("pass");
        record["release_gates"][0]["evidence_refs"] = json!([]);
        let mut issues = Vec::new();
        validate_record_semantics(&record, "2026-10-19", 30, &mut issues);
        assert!(issues.iter().any(|issue| issue.code == "GATE_PASS_WITHOUT_EVIDENCE"));
    }

    #[test]
    fn stale_supplier_availability_and_expired_quote_are_disclosed() {
        let mut record: Value = serde_json::from_str(&repo_file("../../docs/LKI_01_PRODUCT_RECORD_EXAMPLE_DRAFT.json")).unwrap();
        record["bom"]["items"][0]["supplier_records"] = json!([{
            "supplier_name":"Example supplier (synthetic)",
            "source_uri":"https://example.invalid/source",
            "checked_at":"2026-08-01",
            "quote_valid_until":"2026-08-15",
            "evidence_state":"availability_confirmed"
        }]);
        let mut issues = Vec::new();
        validate_record_semantics(&record, "2026-10-19", 30, &mut issues);
        assert!(issues.iter().any(|issue| issue.code == "SUPPLIER_AVAILABILITY_STALE"));
        assert!(issues.iter().any(|issue| issue.code == "SUPPLIER_QUOTE_EXPIRED"));
    }

    #[test]
    fn freshness_calendar_handles_leap_day_and_rejects_invalid_dates() {
        let before = date_to_days("2024-02-28").unwrap();
        let leap = date_to_days("2024-02-29").unwrap();
        let after = date_to_days("2024-03-01").unwrap();
        assert_eq!(leap - before, 1);
        assert_eq!(after - leap, 1);
        assert!(date_to_days("2025-02-29").is_none());
        assert!(date_to_days("2026-13-01").is_none());
    }

    #[test]
    fn released_record_with_untested_gates_is_rejected() {
        let mut record: Value = serde_json::from_str(&repo_file("../../docs/LKI_01_PRODUCT_RECORD_EXAMPLE_DRAFT.json")).unwrap();
        record["lifecycle_state"] = json!("released");
        let mut issues = Vec::new();
        validate_record_semantics(&record, "2026-10-19", 30, &mut issues);
        assert!(issues.iter().any(|issue| issue.code == "RELEASE_GATE_NOT_SATISFIED"));
    }

    #[test]
    fn qualified_component_without_qualification_evidence_is_rejected() {
        let mut record: Value = serde_json::from_str(&repo_file("../../docs/LKI_01_PRODUCT_RECORD_EXAMPLE_DRAFT.json")).unwrap();
        record["bom"]["items"][0]["supplier_validation_state"] = json!("qualified");
        let mut issues = Vec::new();
        validate_record_semantics(&record, "2026-10-19", 30, &mut issues);
        assert!(issues.iter().any(|issue| issue.code == "QUALIFIED_COMPONENT_WITHOUT_QUALIFICATION_EVIDENCE"));
    }
}
