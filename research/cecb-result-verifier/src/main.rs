use serde::{
    de::{self, MapAccess, SeqAccess, Visitor},
    Deserialize, Deserializer,
};
use serde_json::{json, Map, Number, Value};
use serde_json_canonicalizer::to_vec as jcs_to_vec;
use sha2::{Digest, Sha256};
use std::{
    collections::{BTreeMap, BTreeSet, HashSet},
    env, fs,
    path::PathBuf,
    process::ExitCode,
};

const PLAN_SCHEMA: &str = "cecb.housing.paired_design.v0.1";
const BUNDLE_SCHEMA: &str = "cecb.housing.policy_result_bundle.v0.1";
const REPORT_SCHEMA: &str = "cecb.housing.result_verification_report.v0.1";
const RESULT_SCHEMA: &str = "cecb.housing.policy_result.v0.1";
const JCS_PROFILE: &str = "RFC 8785 JSON Canonicalization Scheme";
const SCENARIO_DIGEST_PROFILE: &str = "RFC 8785 JCS followed by SHA-256";

const RUN_FIELDS: &[&str] = &[
    "run_id",
    "design_id",
    "design_manifest_sha256",
    "source_commit_sha",
    "config_id",
    "scenario_config_digest_sha256",
    "replicate_id",
    "seed_hex",
    "arm_id",
    "policy_profile_digest_sha256",
    "simulator_digest_sha256",
    "disposition",
    "outcomes",
    "guardrail_violations",
    "runtime_ms",
    "artifact_digests",
];

const BUNDLE_FIELDS: &[&str] = &[
    "schema_version",
    "design_id",
    "design_manifest_sha256",
    "source_commit_sha",
    "results",
];

const TERMINAL_DISPOSITIONS: &[&str] = &[
    "completed",
    "abstained",
    "failed",
    "timed_out",
    "non_converged",
    "invalid_output",
    "guardrail_failed",
];

const OUTCOME_METRICS: &[&str] = &[
    "safe_suitable_household_months",
    "unmet_essential_need_household_months",
    "wait_duration_p95_months",
    "fully_loaded_cost_cu",
    "debt_service_to_disposable_income",
    "defaults_count",
    "new_safe_suitable_units_completed",
    "maintenance_capacity_lost_dwelling_months",
    "groupwise_unmet_need_rate",
    "resource_use_by_resource_id",
    "privacy_authority_guardrail_violation_count",
    "evidence_coverage_fraction",
];

#[derive(Debug)]
struct Args {
    plan_path: PathBuf,
    results_path: PathBuf,
    report_path: Option<PathBuf>,
}

// serde_json::Value normally accepts duplicate object keys with last-value-wins
// behavior. Evidence identity must not depend on an ambiguous parse, so this small
// wrapper rejects duplicate keys at every object depth before producing a Value.
struct UniqueValue(Value);

struct UniqueValueVisitor;

impl<'de> Deserialize<'de> for UniqueValue {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        deserializer.deserialize_any(UniqueValueVisitor)
    }
}

impl<'de> Visitor<'de> for UniqueValueVisitor {
    type Value = UniqueValue;

    fn expecting(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter.write_str("a JSON value with no duplicate object keys")
    }

    fn visit_bool<E>(self, value: bool) -> Result<Self::Value, E> {
        Ok(UniqueValue(Value::Bool(value)))
    }

    fn visit_i64<E>(self, value: i64) -> Result<Self::Value, E> {
        Ok(UniqueValue(Value::Number(Number::from(value))))
    }

    fn visit_u64<E>(self, value: u64) -> Result<Self::Value, E> {
        Ok(UniqueValue(Value::Number(Number::from(value))))
    }

    fn visit_f64<E>(self, value: f64) -> Result<Self::Value, E>
    where
        E: de::Error,
    {
        Number::from_f64(value)
            .map(|number| UniqueValue(Value::Number(number)))
            .ok_or_else(|| E::custom("non-finite JSON number is not permitted"))
    }

    fn visit_str<E>(self, value: &str) -> Result<Self::Value, E> {
        Ok(UniqueValue(Value::String(value.to_owned())))
    }

    fn visit_string<E>(self, value: String) -> Result<Self::Value, E> {
        Ok(UniqueValue(Value::String(value)))
    }

    fn visit_none<E>(self) -> Result<Self::Value, E> {
        Ok(UniqueValue(Value::Null))
    }

    fn visit_unit<E>(self) -> Result<Self::Value, E> {
        Ok(UniqueValue(Value::Null))
    }

    fn visit_some<D>(self, deserializer: D) -> Result<Self::Value, D::Error>
    where
        D: Deserializer<'de>,
    {
        UniqueValue::deserialize(deserializer)
    }

    fn visit_seq<A>(self, mut sequence: A) -> Result<Self::Value, A::Error>
    where
        A: SeqAccess<'de>,
    {
        let mut values = Vec::new();
        while let Some(value) = sequence.next_element::<UniqueValue>()? {
            values.push(value.0);
        }
        Ok(UniqueValue(Value::Array(values)))
    }

    fn visit_map<A>(self, mut entries: A) -> Result<Self::Value, A::Error>
    where
        A: MapAccess<'de>,
    {
        let mut seen = HashSet::<String>::new();
        let mut object = Map::<String, Value>::new();
        while let Some(key) = entries.next_key::<String>()? {
            if !seen.insert(key.clone()) {
                return Err(<A::Error as de::Error>::custom(format!(
                    "duplicate JSON object key {key:?}"
                )));
            }
            let value = entries.next_value::<UniqueValue>()?.0;
            object.insert(key, value);
        }
        Ok(UniqueValue(Value::Object(object)))
    }
}

fn parse_strict_json(input: &str, label: &str) -> Result<Value, String> {
    let mut deserializer = serde_json::Deserializer::from_str(input);
    let value = UniqueValue::deserialize(&mut deserializer)
        .map_err(|error| format!("{label}: {error}"))?;
    deserializer
        .end()
        .map_err(|error| format!("{label}: trailing or malformed input: {error}"))?;
    Ok(value.0)
}

fn sha256_hex(bytes: &[u8]) -> String {
    const HEX: &[u8; 16] = b"0123456789abcdef";
    let digest = Sha256::digest(bytes);
    let mut output = String::with_capacity(64);
    for byte in digest {
        output.push(HEX[(byte >> 4) as usize] as char);
        output.push(HEX[(byte & 0x0f) as usize] as char);
    }
    output
}

fn jcs_sha256(value: &Value) -> Result<String, String> {
    let canonical = jcs_to_vec(value)
        .map_err(|error| format!("RFC 8785 canonicalization failed: {error}"))?;
    Ok(sha256_hex(&canonical))
}

fn is_hex(value: &str, length: usize) -> bool {
    value.len() == length
        && value
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}

fn str_at<'a>(value: &'a Value, key: &str) -> Option<&'a str> {
    value.get(key).and_then(Value::as_str)
}

fn exact_keys(value: &Value, expected: &[&str], label: &str, errors: &mut Vec<String>) {
    let Some(object) = value.as_object() else {
        errors.push(format!("{label} must be a JSON object"));
        return;
    };

    for key in object.keys() {
        if !expected.contains(&key.as_str()) {
            errors.push(format!("{label} contains unregistered field {key:?}"));
        }
    }
    for key in expected {
        if !object.contains_key(*key) {
            errors.push(format!("{label} is missing required field {key:?}"));
        }
    }
}

fn array_of_strings(value: Option<&Value>) -> Option<Vec<&str>> {
    value?
        .as_array()?
        .iter()
        .map(Value::as_str)
        .collect::<Option<Vec<_>>>()
}

fn validate_contract(plan: &Value, errors: &mut Vec<String>) {
    if str_at(plan, "schema_version") != Some(PLAN_SCHEMA) {
        errors.push(format!("plan schema_version must be {PLAN_SCHEMA:?}"));
    }
    if str_at(plan, "status") != Some("ready_for_execution") {
        errors.push("plan is not marked ready_for_execution".to_owned());
    }

    let Some(design) = plan.get("design") else {
        errors.push("plan is missing design object".to_owned());
        return;
    };
    let Some(binding) = design.get("model_binding") else {
        errors.push("plan is missing design.model_binding".to_owned());
        return;
    };
    if binding.get("execution_authorized").and_then(Value::as_bool) != Some(true) {
        errors.push("plan model_binding.execution_authorized must be true".to_owned());
    }
    if str_at(design, "scenario_config_digest_profile") != Some(SCENARIO_DIGEST_PROFILE) {
        errors.push("plan scenario configuration digest profile is unsupported".to_owned());
    }

    let Some(contract) = plan.get("run_result_contract") else {
        errors.push("plan is missing run_result_contract".to_owned());
        return;
    };
    if str_at(contract, "schema_version") != Some(RESULT_SCHEMA) {
        errors.push(format!("run-result contract schema must be {RESULT_SCHEMA:?}"));
    }
    if str_at(contract, "canonicalization_profile") != Some(JCS_PROFILE) {
        errors.push("run-result contract canonicalization profile is unsupported".to_owned());
    }
    if str_at(contract, "digest_algorithm") != Some("SHA-256") {
        errors.push("run-result contract digest algorithm must be SHA-256".to_owned());
    }
    if str_at(contract, "missing_outcome_policy") != Some("unknown_not_zero") {
        errors.push("missing_outcome_policy must be unknown_not_zero".to_owned());
    }
    if str_at(contract, "forbidden_aggregate_field") != Some("universal_welfare_scalar") {
        errors.push("run-result contract must forbid universal_welfare_scalar".to_owned());
    }

    let required = array_of_strings(contract.get("required_fields"));
    match required {
        Some(fields) => {
            let actual_set = fields.iter().copied().collect::<BTreeSet<_>>();
            let expected_set = RUN_FIELDS.iter().copied().collect::<BTreeSet<_>>();
            if fields.len() != RUN_FIELDS.len() || actual_set != expected_set {
                errors.push(
                    "run-result required_fields do not exactly match the v0.1 contract"
                        .to_owned(),
                );
            }
        }
        None => errors.push("run-result required_fields must be an array of strings".to_owned()),
    }

    let dispositions = array_of_strings(contract.get("terminal_dispositions"));
    match dispositions {
        Some(values) => {
            let actual_set = values.iter().copied().collect::<BTreeSet<_>>();
            let expected_set = TERMINAL_DISPOSITIONS.iter().copied().collect::<BTreeSet<_>>();
            if values.len() != TERMINAL_DISPOSITIONS.len() || actual_set != expected_set {
                errors.push(
                    "terminal_dispositions do not match the registered v0.1 set".to_owned(),
                );
            }
        }
        None => errors.push("terminal_dispositions must be an array of strings".to_owned()),
    }

    let metric_entries = plan.get("outcome_contract").and_then(Value::as_array);
    match metric_entries {
        Some(entries) => {
            let mut ids = BTreeSet::<String>::new();
            for (index, entry) in entries.iter().enumerate() {
                let label = format!("outcome_contract[{index}]");
                exact_keys(
                    entry,
                    &["metric_id", "unit", "aggregation", "direction", "missing_policy"],
                    &label,
                    errors,
                );
                let Some(id) = str_at(entry, "metric_id") else {
                    errors.push(format!("{label} is missing metric_id"));
                    continue;
                };
                if !ids.insert(id.to_owned()) {
                    errors.push(format!("duplicate outcome metric_id {id:?}"));
                }
                for field in ["unit", "aggregation", "direction", "missing_policy"] {
                    if str_at(entry, field).is_none() {
                        errors.push(format!("{label} field {field:?} must be a string"));
                    }
                }
                if str_at(entry, "missing_policy") != Some("unknown_not_zero")
                    && str_at(entry, "missing_policy") != Some("unknown_is_not_pass")
                {
                    errors.push(format!("{label} has unsupported missing policy"));
                }
            }
            let expected = OUTCOME_METRICS.iter().copied().collect::<BTreeSet<_>>();
            if ids.len() != OUTCOME_METRICS.len()
                || ids.iter().map(String::as_str).collect::<BTreeSet<_>>() != expected
            {
                errors.push(
                    "outcome_contract metric IDs do not match the registered v0.1 vector"
                        .to_owned(),
                );
            }
        }
        None => errors.push("outcome_contract must be an array".to_owned()),
    }
}

#[derive(Clone, Debug)]
struct ExpectedRun {
    run_id: String,
    config_id: String,
    config_digest: String,
    replicate_id: String,
    seed_hex: String,
    arm_id: String,
    policy_digest: String,
    simulator_digest: String,
}

fn build_expected_runs(plan: &Value, errors: &mut Vec<String>) -> BTreeMap<String, ExpectedRun> {
    let mut configurations = BTreeMap::<String, String>::new();
    match plan.get("configurations").and_then(Value::as_array) {
        Some(entries) => {
            for (index, config) in entries.iter().enumerate() {
                let label = format!("configurations[{index}]");
                let Some(id) = str_at(config, "config_id") else {
                    errors.push(format!("{label} is missing config_id"));
                    continue;
                };
                if id.is_empty() {
                    errors.push(format!("{label} config_id is empty"));
                    continue;
                }
                if configurations.contains_key(id) {
                    errors.push(format!("duplicate config_id {id:?}"));
                    continue;
                }
                match jcs_sha256(config) {
                    Ok(digest) => {
                        configurations.insert(id.to_owned(), digest);
                    }
                    Err(error) => errors.push(format!("{label}: {error}")),
                }
            }
        }
        None => errors.push("plan configurations must be an array".to_owned()),
    }

    let mut seeds = BTreeMap::<String, String>::new();
    let mut seen_seed_values = BTreeSet::<String>::new();
    match plan.get("paired_seeds").and_then(Value::as_array) {
        Some(entries) => {
            for (index, seed) in entries.iter().enumerate() {
                let label = format!("paired_seeds[{index}]");
                let Some(replicate) = str_at(seed, "replicate_id") else {
                    errors.push(format!("{label} is missing replicate_id"));
                    continue;
                };
                let Some(seed_hex) = str_at(seed, "seed_hex") else {
                    errors.push(format!("{label} is missing seed_hex"));
                    continue;
                };
                if replicate.is_empty() || !is_hex(seed_hex, 16) {
                    errors.push(format!("{label} has an invalid replicate ID or 64-bit seed"));
                    continue;
                }
                if seeds.contains_key(replicate) {
                    errors.push(format!("duplicate replicate_id {replicate:?}"));
                    continue;
                }
                if !seen_seed_values.insert(seed_hex.to_owned()) {
                    errors.push(format!("duplicate paired seed value {seed_hex:?}"));
                    continue;
                }
                seeds.insert(replicate.to_owned(), seed_hex.to_owned());
            }
        }
        None => errors.push("plan paired_seeds must be an array".to_owned()),
    }

    let mut policies = BTreeMap::<String, String>::new();
    match plan.get("policy_arms").and_then(Value::as_array) {
        Some(entries) => {
            for (index, arm) in entries.iter().enumerate() {
                let label = format!("policy_arms[{index}]");
                let Some(id) = str_at(arm, "arm_id") else {
                    errors.push(format!("{label} is missing arm_id"));
                    continue;
                };
                let Some(digest) = str_at(arm, "policy_profile_digest_sha256") else {
                    errors.push(format!("{label} is missing policy_profile_digest_sha256"));
                    continue;
                };
                if id.is_empty() || !is_hex(digest, 64) {
                    errors.push(format!("{label} has invalid arm ID or policy profile digest"));
                    continue;
                }
                if str_at(arm, "implementation_status") != Some("ready") {
                    errors.push(format!("policy arm {id:?} is not marked ready"));
                }
                if policies.insert(id.to_owned(), digest.to_owned()).is_some() {
                    errors.push(format!("duplicate policy arm ID {id:?}"));
                }
            }
        }
        None => errors.push("plan policy_arms must be an array".to_owned()),
    }

    let binding = plan.get("design").and_then(|value| value.get("model_binding"));
    let simulator_digest = binding
        .and_then(|value| str_at(value, "simulator_digest_sha256"))
        .unwrap_or_default()
        .to_owned();
    if !is_hex(&simulator_digest, 64) {
        errors.push(
            "model_binding.simulator_digest_sha256 must be a 64-char lowercase SHA-256 digest"
                .to_owned(),
        );
    }

    let planned_rows = match plan.get("runs").and_then(Value::as_array) {
        Some(rows) => rows,
        None => {
            errors.push("plan runs must be an array".to_owned());
            return BTreeMap::new();
        }
    };
    let mut expected = BTreeMap::<String, ExpectedRun>::new();
    let mut tuples = BTreeSet::<(String, String, String)>::new();
    for (index, row) in planned_rows.iter().enumerate() {
        let label = format!("plan.runs[{index}]");
        let Some(run_id) = str_at(row, "run_id") else {
            errors.push(format!("{label} missing run_id"));
            continue;
        };
        let Some(config_id) = str_at(row, "config_id") else {
            errors.push(format!("{label} missing config_id"));
            continue;
        };
        let Some(replicate_id) = str_at(row, "replicate_id") else {
            errors.push(format!("{label} missing replicate_id"));
            continue;
        };
        let Some(seed_hex) = str_at(row, "seed_hex") else {
            errors.push(format!("{label} missing seed_hex"));
            continue;
        };
        let Some(arm_id) = str_at(row, "arm_id") else {
            errors.push(format!("{label} missing arm_id"));
            continue;
        };

        let Some(config_digest) = configurations.get(config_id) else {
            errors.push(format!("{label} references unknown config_id {config_id:?}"));
            continue;
        };
        let Some(registered_seed) = seeds.get(replicate_id) else {
            errors.push(format!("{label} references unknown replicate_id {replicate_id:?}"));
            continue;
        };
        let Some(policy_digest) = policies.get(arm_id) else {
            errors.push(format!("{label} references unknown arm_id {arm_id:?}"));
            continue;
        };
        if registered_seed != seed_hex {
            errors.push(format!("{label} seed_hex does not match paired_seeds entry"));
        }
        let tuple = (
            config_id.to_owned(),
            replicate_id.to_owned(),
            arm_id.to_owned(),
        );
        if !tuples.insert(tuple) {
            errors.push(format!("{label} duplicates a config/replicate/arm slot"));
        }
        if run_id.is_empty() || expected.contains_key(run_id) {
            errors.push(format!("{label} has empty or duplicate run_id {run_id:?}"));
            continue;
        }
        expected.insert(
            run_id.to_owned(),
            ExpectedRun {
                run_id: run_id.to_owned(),
                config_id: config_id.to_owned(),
                config_digest: config_digest.to_owned(),
                replicate_id: replicate_id.to_owned(),
                seed_hex: seed_hex.to_owned(),
                arm_id: arm_id.to_owned(),
                policy_digest: policy_digest.to_owned(),
                simulator_digest: simulator_digest.clone(),
            },
        );
    }

    let expected_slots = configurations
        .len()
        .saturating_mul(seeds.len())
        .saturating_mul(policies.len());
    if expected_slots != planned_rows.len() || expected.len() != expected_slots {
        errors.push(format!(
            concat!(
                "plan cross-product is incomplete: {} rows for {} configurations x ",
                "{} seeds x {} arms = {} slots"
            ),
            planned_rows.len(),
            configurations.len(),
            seeds.len(),
            policies.len(),
            expected_slots
        ));
    }
    if tuples.len() != expected_slots {
        errors.push(format!(
            "plan has {} distinct config/replicate/arm tuples; expected {expected_slots}",
            tuples.len()
        ));
    }

    expected
}

fn metric_value_valid(metric: &str, value: &Value, errors: &mut Vec<String>, label: &str) -> u64 {
    if value.is_null() {
        return 1;
    }

    let object_metric = matches!(
        metric,
        "debt_service_to_disposable_income"
            | "groupwise_unmet_need_rate"
            | "resource_use_by_resource_id"
    );

    let mut unknown_cells = 0_u64;
    if object_metric {
        let Some(object) = value.as_object() else {
            errors.push(format!("{label} must be an object or null"));
            return 0;
        };
        if object.is_empty() {
            errors.push(format!("{label} object must not be empty; use null for unknown"));
        }
        for (key, child) in object {
            if key.is_empty() {
                errors.push(format!("{label} contains an empty group/resource key"));
            }
            if child.is_null() {
                unknown_cells += 1;
                continue;
            }
            let Some(number) = child.as_f64() else {
                errors.push(format!("{label}[{key:?}] must be a non-negative number or null"));
                continue;
            };
            if !number.is_finite() || number < 0.0 {
                errors.push(format!("{label}[{key:?}] must be finite and non-negative"));
            }
            if metric == "groupwise_unmet_need_rate" && number > 1.0 {
                errors.push(format!("{label}[{key:?}] must be in [0, 1]"));
            }
        }
        return unknown_cells;
    }

    let Some(number) = value.as_f64() else {
        errors.push(format!("{label} must be a number or null"));
        return 0;
    };
    if !number.is_finite() || number < 0.0 {
        errors.push(format!("{label} must be finite and non-negative"));
    }
    match metric {
        "safe_suitable_household_months"
        | "unmet_essential_need_household_months"
        | "defaults_count"
        | "new_safe_suitable_units_completed"
        | "maintenance_capacity_lost_dwelling_months"
        | "privacy_authority_guardrail_violation_count" => {
            if value.as_u64().is_none() {
                errors.push(format!("{label} must be an unsigned integer or null"));
            }
        }
        "evidence_coverage_fraction" if number > 1.0 => {
            errors.push(format!("{label} must be in [0, 1]"));
        }
        _ => {}
    }
    unknown_cells
}

fn validate_outcomes(
    value: &Value,
    run_id: &str,
    errors: &mut Vec<String>,
) -> (u64, BTreeMap<String, u64>) {
    let mut unknown_total = 0_u64;
    let mut unknown_by_metric = BTreeMap::<String, u64>::new();
    let Some(object) = value.as_object() else {
        errors.push(format!("run {run_id:?} outcomes must be an object"));
        return (0, unknown_by_metric);
    };
    let expected = OUTCOME_METRICS.iter().copied().collect::<BTreeSet<_>>();
    let actual = object.keys().map(String::as_str).collect::<BTreeSet<_>>();
    if actual != expected {
        errors.push(format!(
            "run {run_id:?} outcomes must contain exactly the registered 12 metrics"
        ));
    }

    for metric in OUTCOME_METRICS {
        match object.get(*metric) {
            Some(value) => {
                let count = metric_value_valid(
                    metric,
                    value,
                    errors,
                    &format!("run {run_id:?} outcome {metric:?}"),
                );
                if count > 0 {
                    unknown_total = unknown_total.saturating_add(count);
                    unknown_by_metric.insert((*metric).to_owned(), count);
                }
            }
            None => {
                errors.push(format!("run {run_id:?} is missing outcome metric {metric:?}"));
            }
        }
    }

    (unknown_total, unknown_by_metric)
}

fn validate_result_row(
    result: &Value,
    planned: &ExpectedRun,
    plan: &Value,
    plan_digest: &str,
    errors: &mut Vec<String>,
) -> (u64, bool, bool, BTreeMap<String, u64>) {
    let run_id = planned.run_id.as_str();
    exact_keys(result, RUN_FIELDS, &format!("result {run_id:?}"), errors);

    let design_id = str_at(plan, "design_id").unwrap_or_default();
    let source_commit = str_at(plan, "source_commit_sha").unwrap_or_default();
    let expected_pairs = [
        ("design_id", design_id),
        ("design_manifest_sha256", plan_digest),
        ("source_commit_sha", source_commit),
        ("config_id", planned.config_id.as_str()),
        ("scenario_config_digest_sha256", planned.config_digest.as_str()),
        ("replicate_id", planned.replicate_id.as_str()),
        ("seed_hex", planned.seed_hex.as_str()),
        ("arm_id", planned.arm_id.as_str()),
        ("policy_profile_digest_sha256", planned.policy_digest.as_str()),
        ("simulator_digest_sha256", planned.simulator_digest.as_str()),
    ];
    for (field, expected) in expected_pairs {
        if str_at(result, field) != Some(expected) {
            errors.push(format!(
                "run {run_id:?} field {field:?} does not match the pinned plan"
            ));
        }
    }

    let disposition = str_at(result, "disposition").unwrap_or_default();
    if !TERMINAL_DISPOSITIONS.contains(&disposition) {
        errors.push(format!(
            "run {run_id:?} has unregistered terminal disposition {disposition:?}"
        ));
    }

    let guardrails = result.get("guardrail_violations").and_then(Value::as_array);
    let mut violation_count = 0_usize;
    match guardrails {
        Some(items) => {
            violation_count = items.len();
            for (index, item) in items.iter().enumerate() {
                if item.as_str().is_none_or(str::is_empty) {
                    errors.push(format!(
                        "run {run_id:?} guardrail_violations[{index}] must be a non-empty string"
                    ));
                }
            }
        }
        None => errors.push(format!(
            "run {run_id:?} guardrail_violations must be an array of strings"
        )),
    }
    if violation_count > 0 && disposition != "guardrail_failed" {
        errors.push(format!(
            "run {run_id:?} has guardrail violations but disposition is not guardrail_failed"
        ));
    }
    if violation_count == 0 && disposition == "guardrail_failed" {
        errors.push(format!(
            "run {run_id:?} is guardrail_failed but lists no guardrail violation"
        ));
    }

    match result.get("runtime_ms").and_then(Value::as_u64) {
        Some(_) => {}
        None => errors.push(format!("run {run_id:?} runtime_ms must be an unsigned integer")),
    }

    let artifacts = result.get("artifact_digests").and_then(Value::as_object);
    match artifacts {
        Some(artifacts) if !artifacts.is_empty() => {
            for (name, digest) in artifacts {
                if name.is_empty()
                    || digest.as_str().is_none_or(|value| !is_hex(value, 64))
                {
                    errors.push(format!(
                        "run {run_id:?} artifact_digests contains invalid name or digest"
                    ));
                }
            }
        }
        Some(_) => errors.push(format!(
            "run {run_id:?} artifact_digests must contain at least one digest"
        )),
        None => errors.push(format!(
            "run {run_id:?} artifact_digests must be an object"
        )),
    }

    let (unknown_count, unknown_by_metric) = match result.get("outcomes") {
        Some(outcomes) => validate_outcomes(outcomes, run_id, errors),
        None => {
            errors.push(format!("run {run_id:?} is missing outcomes"));
            (0, BTreeMap::new())
        }
    };

    let policy_eligible = disposition == "completed" && violation_count == 0;
    let guardrail_failed = disposition == "guardrail_failed";
    (unknown_count, policy_eligible, guardrail_failed, unknown_by_metric)
}

fn verify_bundle(plan: &Value, bundle: &Value) -> Value {
    let mut errors = Vec::<String>::new();
    validate_contract(plan, &mut errors);

    let design_id = str_at(plan, "design_id").unwrap_or_default().to_owned();
    if design_id.is_empty() {
        errors.push("plan design_id is missing or empty".to_owned());
    }
    let source_commit = str_at(plan, "source_commit_sha").unwrap_or_default().to_owned();
    if !is_hex(&source_commit, 40) {
        errors.push("plan source_commit_sha must be a 40-character lowercase hex SHA".to_owned());
    }

    let plan_digest = match jcs_sha256(plan) {
        Ok(digest) => digest,
        Err(error) => {
            errors.push(error);
            String::new()
        }
    };

    let binding = plan.get("design").and_then(|value| value.get("model_binding"));
    if binding
        .and_then(|value| value.get("execution_authorized"))
        .and_then(Value::as_bool)
        != Some(true)
    {
        errors.push("plan is not authorized for policy execution".to_owned());
    }

    let expected_runs = build_expected_runs(plan, &mut errors);
    exact_keys(bundle, BUNDLE_FIELDS, "result bundle", &mut errors);
    if str_at(bundle, "schema_version") != Some(BUNDLE_SCHEMA) {
        errors.push(format!("result bundle schema_version must be {BUNDLE_SCHEMA:?}"));
    }
    if str_at(bundle, "design_id") != Some(design_id.as_str()) {
        errors.push("result bundle design_id does not match plan".to_owned());
    }
    if str_at(bundle, "design_manifest_sha256") != Some(plan_digest.as_str()) {
        errors.push(
            "result bundle design_manifest_sha256 does not match canonical plan digest"
                .to_owned(),
        );
    }
    if str_at(bundle, "source_commit_sha") != Some(source_commit.as_str()) {
        errors.push("result bundle source_commit_sha does not match plan".to_owned());
    }

    let rows = match bundle.get("results").and_then(Value::as_array) {
        Some(rows) => rows,
        None => {
            errors.push("result bundle results must be an array".to_owned());
            &[]
        }
    };
    let mut seen_ids = BTreeSet::<String>::new();
    let mut disposition_counts = BTreeMap::<String, u64>::new();
    let mut unknown_by_metric = BTreeMap::<String, u64>::new();
    let mut unknown_total = 0_u64;
    let mut eligible_count = 0_u64;
    let mut guardrail_failed_count = 0_u64;

    for (index, result) in rows.iter().enumerate() {
        let label = format!("result bundle results[{index}]");
        let Some(run_id) = str_at(result, "run_id") else {
            errors.push(format!("{label} is missing run_id"));
            continue;
        };
        if !seen_ids.insert(run_id.to_owned()) {
            errors.push(format!("duplicate result run_id {run_id:?}"));
            continue;
        }
        let Some(planned) = expected_runs.get(run_id) else {
            errors.push(format!("unexpected result run_id {run_id:?}"));
            continue;
        };
        let mut row_errors = Vec::<String>::new();
        let (unknown_count, eligible, guardrail_failed, metric_unknowns) =
            validate_result_row(result, planned, plan, &plan_digest, &mut row_errors);
        let row_verified = row_errors.is_empty();
        errors.extend(row_errors);
        unknown_total = unknown_total.saturating_add(unknown_count);
        for (metric, count) in metric_unknowns {
            let total = unknown_by_metric.entry(metric).or_default();
            *total = total.saturating_add(count);
        }
        if row_verified && eligible {
            eligible_count = eligible_count.saturating_add(1);
        }
        if row_verified && guardrail_failed {
            guardrail_failed_count = guardrail_failed_count.saturating_add(1);
        }
        let disposition = str_at(result, "disposition").unwrap_or("invalid");
        *disposition_counts.entry(disposition.to_owned()).or_default() += 1;
    }

    for run_id in expected_runs.keys() {
        if !seen_ids.contains(run_id) {
            errors.push(format!("missing terminal result for planned run_id {run_id:?}"));
        }
    }
    if rows.len() != expected_runs.len() {
        errors.push(format!(
            "result row count {} does not match planned slot count {}",
            rows.len(),
            expected_runs.len()
        ));
    }

    let verified = errors.is_empty();
    json!({
        "schema_version": REPORT_SCHEMA,
        "status": if verified { "PASS" } else { "FAIL" },
        "verified": verified,
        "design_id": design_id,
        "design_manifest_sha256": plan_digest,
        "source_commit_sha": source_commit,
        "planned_run_count": expected_runs.len(),
        "result_row_count": rows.len(),
        "terminal_disposition_counts": disposition_counts,
        "policy_eligible_run_count": eligible_count,
        "guardrail_failed_run_count": guardrail_failed_count,
        "unknown_outcome_cell_count": unknown_total,
        "unknown_outcome_cells_by_metric": unknown_by_metric,
        "errors": errors,
        "claim_ceiling": concat!(
            "Artifact-contract conformance only; not simulator validity, real-world prediction, ",
            "causal policy effect, or regime superiority."
        )
    })
}

fn read_json(path: &PathBuf, label: &str) -> Result<Value, String> {
    let input = fs::read_to_string(path)
        .map_err(|error| format!("cannot read {label} {}: {error}", path.display()))?;
    parse_strict_json(&input, label)
}

fn parse_args() -> Result<Option<Args>, String> {
    let mut plan_path = None;
    let mut results_path = None;
    let mut report_path = None;
    let args = env::args().skip(1).collect::<Vec<_>>();
    let mut index = 0;

    while index < args.len() {
        match args[index].as_str() {
            "--help" | "-h" => {
                println!(
                    concat!(
                        "cecb-result-verifier --plan PLAN.json --results BUNDLE.json ",
                        "[--report REPORT.json]"
                    )
                );
                return Ok(None);
            }
            "--plan" => {
                index += 1;
                plan_path = Some(PathBuf::from(
                    args.get(index).ok_or("--plan requires a path")?,
                ));
            }
            "--results" => {
                index += 1;
                results_path = Some(PathBuf::from(
                    args.get(index).ok_or("--results requires a path")?,
                ));
            }
            "--report" => {
                index += 1;
                report_path = Some(PathBuf::from(
                    args.get(index).ok_or("--report requires a path")?,
                ));
            }
            other => return Err(format!("unknown argument {other:?}; use --help")),
        }
        index += 1;
    }

    Ok(Some(Args {
        plan_path: plan_path.ok_or("--plan is required")?,
        results_path: results_path.ok_or("--results is required")?,
        report_path,
    }))
}

fn main() -> ExitCode {
    let args = match parse_args() {
        Ok(Some(args)) => args,
        Ok(None) => return ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("{error}");
            return ExitCode::from(2);
        }
    };

    let plan = match read_json(&args.plan_path, "plan") {
        Ok(value) => value,
        Err(error) => {
            eprintln!("{error}");
            return ExitCode::from(2);
        }
    };
    let bundle = match read_json(&args.results_path, "result bundle") {
        Ok(value) => value,
        Err(error) => {
            eprintln!("{error}");
            return ExitCode::from(2);
        }
    };
    let report = verify_bundle(&plan, &bundle);
    let report_text = match serde_json::to_string_pretty(&report) {
        Ok(text) => text,
        Err(error) => {
            eprintln!("cannot serialize verification report: {error}");
            return ExitCode::from(2);
        }
    };

    if let Some(path) = &args.report_path {
        if let Err(error) = fs::write(path, format!("{report_text}\n")) {
            eprintln!("cannot write report {}: {error}", path.display());
            return ExitCode::from(2);
        }
    } else {
        println!("{report_text}");
    }

    if report["status"] == "PASS" {
        ExitCode::SUCCESS
    } else {
        ExitCode::from(1)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn test_outcome_contract() -> Vec<Value> {
        OUTCOME_METRICS
            .iter()
            .map(|metric| {
                json!({
                    "metric_id": metric,
                    "unit": "test-unit",
                    "aggregation": "test",
                    "direction": "report",
                    "missing_policy": "unknown_not_zero"
                })
            })
            .collect()
    }

    fn test_plan() -> Value {
        let source_commit = "a".repeat(40);
        let simulator_digest = "b".repeat(64);
        let profile_one = "c".repeat(64);
        let profile_two = "d".repeat(64);
        let configurations = vec![
            json!({"config_id": "C01", "supply_condition": "adequate"}),
            json!({"config_id": "C02", "supply_condition": "constrained"}),
        ];
        let seeds = vec![
            json!({"replicate_id": "R01", "seed_hex": "0000000000000001"}),
            json!({"replicate_id": "R02", "seed_hex": "0000000000000002"}),
        ];
        let arms = vec![
            json!({
                "arm_id": "allocation_price",
                "implementation_status": "ready",
                "policy_profile_digest_sha256": profile_one
            }),
            json!({
                "arm_id": "allocation_queue",
                "implementation_status": "ready",
                "policy_profile_digest_sha256": profile_two
            }),
        ];
        let mut runs = Vec::<Value>::new();
        for config in &configurations {
            let config_id = config["config_id"].as_str().expect("config ID");
            for seed in &seeds {
                let replicate = seed["replicate_id"].as_str().expect("replicate ID");
                let seed_hex = seed["seed_hex"].as_str().expect("seed");
                for arm in &arms {
                    let arm_id = arm["arm_id"].as_str().expect("arm ID");
                    runs.push(json!({
                        "run_id": format!("{config_id}-{replicate}-{arm_id}"),
                        "config_id": config_id,
                        "replicate_id": replicate,
                        "seed_hex": seed_hex,
                        "arm_id": arm_id,
                        "status": "not_executed",
                        "disposition": "not_started"
                    }));
                }
            }
        }

        json!({
            "schema_version": PLAN_SCHEMA,
            "design_id": "CECB-TEST-PLAN",
            "source_commit_sha": source_commit,
            "status": "ready_for_execution",
            "design": {
                "scenario_config_digest_profile": SCENARIO_DIGEST_PROFILE,
                "model_binding": {
                    "execution_authorized": true,
                    "simulator_digest_sha256": simulator_digest
                }
            },
            "configurations": configurations,
            "paired_seeds": seeds,
            "policy_arms": arms,
            "runs": runs,
            "run_result_contract": {
                "schema_version": RESULT_SCHEMA,
                "canonicalization_profile": JCS_PROFILE,
                "digest_algorithm": "SHA-256",
                "required_fields": RUN_FIELDS,
                "terminal_dispositions": TERMINAL_DISPOSITIONS,
                "missing_outcome_policy": "unknown_not_zero",
                "forbidden_aggregate_field": "universal_welfare_scalar"
            },
            "outcome_contract": test_outcome_contract()
        })
    }

    fn test_bundle(plan: &Value) -> Value {
        let design_id = str_at(plan, "design_id").expect("design ID");
        let source_commit = str_at(plan, "source_commit_sha").expect("source commit");
        let plan_digest = jcs_sha256(plan).expect("plan JCS digest");
        let simulator_digest = plan["design"]["model_binding"]["simulator_digest_sha256"]
            .as_str()
            .expect("simulator digest");
        let config_digests = plan["configurations"]
            .as_array()
            .expect("configurations")
            .iter()
            .map(|config| {
                (
                    config["config_id"].as_str().expect("config ID").to_owned(),
                    jcs_sha256(config).expect("config digest"),
                )
            })
            .collect::<BTreeMap<_, _>>();
        let profile_digests = plan["policy_arms"]
            .as_array()
            .expect("policy arms")
            .iter()
            .map(|arm| {
                (
                    arm["arm_id"].as_str().expect("arm ID").to_owned(),
                    arm["policy_profile_digest_sha256"]
                        .as_str()
                        .expect("profile digest")
                        .to_owned(),
                )
            })
            .collect::<BTreeMap<_, _>>();
        let results = plan["runs"]
            .as_array()
            .expect("plan runs")
            .iter()
            .map(|planned| {
                let run_id = planned["run_id"].as_str().expect("run ID");
                let config_id = planned["config_id"].as_str().expect("config ID");
                let replicate = planned["replicate_id"].as_str().expect("replicate");
                let seed = planned["seed_hex"].as_str().expect("seed");
                let arm_id = planned["arm_id"].as_str().expect("arm ID");
                let outcomes = OUTCOME_METRICS
                    .iter()
                    .map(|metric| {
                        let value = match *metric {
                            "debt_service_to_disposable_income" => json!({"group-a": 0.2}),
                            "groupwise_unmet_need_rate" => json!({"group-a": 0.1}),
                            "resource_use_by_resource_id" => json!({"water_litre": 12.0}),
                            "evidence_coverage_fraction" => json!(0.95),
                            "privacy_authority_guardrail_violation_count" => json!(0),
                            "safe_suitable_household_months"
                            | "unmet_essential_need_household_months"
                            | "defaults_count"
                            | "new_safe_suitable_units_completed"
                            | "maintenance_capacity_lost_dwelling_months" => json!(1),
                            _ => json!(1.0),
                        };
                        ((*metric).to_owned(), value)
                    })
                    .collect::<Map<_, _>>();
                json!({
                    "run_id": run_id,
                    "design_id": design_id,
                    "design_manifest_sha256": plan_digest,
                    "source_commit_sha": source_commit,
                    "config_id": config_id,
                    "scenario_config_digest_sha256": config_digests[config_id],
                    "replicate_id": replicate,
                    "seed_hex": seed,
                    "arm_id": arm_id,
                    "policy_profile_digest_sha256": profile_digests[arm_id],
                    "simulator_digest_sha256": simulator_digest,
                    "disposition": "completed",
                    "outcomes": Value::Object(outcomes),
                    "guardrail_violations": [],
                    "runtime_ms": 25,
                    "artifact_digests": {"result_trace": "e".repeat(64)}
                })
            })
            .collect::<Vec<_>>();

        json!({
            "schema_version": BUNDLE_SCHEMA,
            "design_id": design_id,
            "design_manifest_sha256": plan_digest,
            "source_commit_sha": source_commit,
            "results": results
        })
    }

    fn verified(plan: &Value, bundle: &Value) -> bool {
        verify_bundle(plan, bundle)["status"] == "PASS"
    }

    #[test]
    fn accepts_complete_pinned_bundle_and_reports_claim_ceiling() {
        let plan = test_plan();
        let bundle = test_bundle(&plan);
        let report = verify_bundle(&plan, &bundle);
        assert_eq!(report["status"], "PASS", "{report:#}");
        assert_eq!(report["planned_run_count"], 8);
        assert_eq!(report["result_row_count"], 8);
        assert_eq!(report["policy_eligible_run_count"], 8);
        assert_eq!(report["unknown_outcome_cell_count"], 0);
        assert!(report["claim_ceiling"].as_str().is_some());
    }

    #[test]
    fn canonical_digest_ignores_object_key_order_and_whitespace() {
        let first = parse_strict_json(r#"{"b":2,"a":{"y":true,"x":1}}"#, "first").unwrap();
        let second = parse_strict_json("{ \n \"a\": {\"x\":1,\"y\":true}, \"b\": 2 }", "second")
            .unwrap();
        assert_eq!(jcs_sha256(&first).unwrap(), jcs_sha256(&second).unwrap());
    }

    #[test]
    fn rejects_duplicate_json_keys_at_any_depth() {
        assert!(parse_strict_json(r#"{"outer":{"x":1,"x":2}}"#, "duplicate").is_err());
    }

    #[test]
    fn refuses_design_only_plan_with_pending_adapters() {
        let mut plan = test_plan();
        plan["status"] = json!("design_only_no_policy_execution");
        plan["design"]["model_binding"]["execution_authorized"] = json!(false);
        for arm in plan["policy_arms"].as_array_mut().unwrap() {
            arm["implementation_status"] = json!("adapter_pending");
        }
        let bundle = test_bundle(&plan);
        assert!(!verified(&plan, &bundle));
    }

    #[test]
    fn rejects_missing_result_slot() {
        let plan = test_plan();
        let mut bundle = test_bundle(&plan);
        bundle["results"].as_array_mut().unwrap().pop();
        assert!(!verified(&plan, &bundle));
    }

    #[test]
    fn rejects_duplicate_result_run_id() {
        let plan = test_plan();
        let mut bundle = test_bundle(&plan);
        let first = bundle["results"][0].clone();
        bundle["results"][1] = first;
        assert!(!verified(&plan, &bundle));
    }

    #[test]
    fn rejects_unexpected_result_run_id() {
        let plan = test_plan();
        let mut bundle = test_bundle(&plan);
        bundle["results"][0]["run_id"] = json!("RUN-NOT-IN-PLAN");
        assert!(!verified(&plan, &bundle));
    }

    #[test]
    fn rejects_wrong_plan_digest() {
        let plan = test_plan();
        let mut bundle = test_bundle(&plan);
        bundle["design_manifest_sha256"] = json!("f".repeat(64));
        assert!(!verified(&plan, &bundle));
    }

    #[test]
    fn rejects_wrong_configuration_digest() {
        let plan = test_plan();
        let mut bundle = test_bundle(&plan);
        bundle["results"][0]["scenario_config_digest_sha256"] = json!("f".repeat(64));
        assert!(!verified(&plan, &bundle));
    }

    #[test]
    fn rejects_wrong_policy_profile_digest() {
        let plan = test_plan();
        let mut bundle = test_bundle(&plan);
        bundle["results"][0]["policy_profile_digest_sha256"] = json!("f".repeat(64));
        assert!(!verified(&plan, &bundle));
    }

    #[test]
    fn rejects_wrong_seed_even_when_run_id_matches() {
        let plan = test_plan();
        let mut bundle = test_bundle(&plan);
        bundle["results"][0]["seed_hex"] = json!("000000000000ffff");
        assert!(!verified(&plan, &bundle));
    }

    #[test]
    fn rejects_missing_outcome_instead_of_zero_filling() {
        let plan = test_plan();
        let mut bundle = test_bundle(&plan);
        bundle["results"][0]["outcomes"]
            .as_object_mut()
            .unwrap()
            .remove("evidence_coverage_fraction");
        assert!(!verified(&plan, &bundle));
    }

    #[test]
    fn preserves_explicit_unknown_outcome_without_zero_filling() {
        let plan = test_plan();
        let mut bundle = test_bundle(&plan);
        bundle["results"][0]["outcomes"]["evidence_coverage_fraction"] = Value::Null;
        let report = verify_bundle(&plan, &bundle);
        assert_eq!(report["status"], "PASS", "{report:#}");
        assert_eq!(report["unknown_outcome_cell_count"], 1);
        assert_eq!(
            report["unknown_outcome_cells_by_metric"]["evidence_coverage_fraction"],
            1
        );
    }

    #[test]
    fn guardrail_failed_is_reported_but_not_policy_eligible() {
        let plan = test_plan();
        let mut bundle = test_bundle(&plan);
        bundle["results"][0]["disposition"] = json!("guardrail_failed");
        bundle["results"][0]["guardrail_violations"] = json!(["HARD-CAPACITY-001"]);
        let report = verify_bundle(&plan, &bundle);
        assert_eq!(report["status"], "PASS", "{report:#}");
        assert_eq!(report["guardrail_failed_run_count"], 1);
        assert_eq!(report["policy_eligible_run_count"], 7);
    }

    #[test]
    fn rejects_guardrail_violation_disguised_as_completed() {
        let plan = test_plan();
        let mut bundle = test_bundle(&plan);
        bundle["results"][0]["guardrail_violations"] = json!(["HARD-CAPACITY-001"]);
        assert!(!verified(&plan, &bundle));
    }

    #[test]
    fn rejects_universal_welfare_scalar_in_result_record() {
        let plan = test_plan();
        let mut bundle = test_bundle(&plan);
        bundle["results"][0]["universal_welfare_scalar"] = json!(0.99);
        assert!(!verified(&plan, &bundle));
    }

    #[test]
    fn rejects_malformed_artifact_digest() {
        let plan = test_plan();
        let mut bundle = test_bundle(&plan);
        bundle["results"][0]["artifact_digests"]["result_trace"] = json!("not-a-digest");
        assert!(!verified(&plan, &bundle));
    }

    #[test]
    fn rejects_duplicate_planned_run_ids() {
        let mut plan = test_plan();
        let duplicate = plan["runs"][0].clone();
        plan["runs"][1] = duplicate;
        let bundle = test_bundle(&plan);
        assert!(!verified(&plan, &bundle));
    }
}
