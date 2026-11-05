#![forbid(unsafe_code)]

//! Static qualification for an open synthetic human-decision study instrument.
//! This evaluator cannot infer human comprehension or real-world fraud reduction.

use jsonschema::draft202012;
use serde_json::{Value, json};
use std::collections::{BTreeMap, BTreeSet};
use std::error::Error;
use std::fs;
use std::path::PathBuf;

const DEFAULT_CORPUS: &str =
    include_str!("../../../../mycelix-finance/docs/sa-payment-human-decision-development-v1.json");
const DEFAULT_SCHEMA: &str =
    include_str!("../../../../mycelix-finance/docs/sa-payment-human-decision-v1.schema.json");
const DEFAULT_RESPONSE_SCHEMA: &str = include_str!(
    "../../../../mycelix-finance/docs/sa-payment-human-decision-response-v1.schema.json"
);

const EXPECTED_CLASSES: [(&str, usize, &str); 3] = [
    ("legitimate_supported", 8, "accept"),
    ("deceptive_or_contradictory", 8, "decline_and_report"),
    ("insufficient_or_ambiguous", 8, "pause_and_verify"),
];

fn str_at<'a>(value: &'a Value, key: &str) -> Option<&'a str> {
    value.get(key).and_then(Value::as_str)
}

fn array_at<'a>(value: &'a Value, key: &str) -> Option<&'a Vec<Value>> {
    value.get(key).and_then(Value::as_array)
}

fn string_set(value: &Value, key: &str) -> BTreeSet<String> {
    array_at(value, key)
        .into_iter()
        .flatten()
        .filter_map(Value::as_str)
        .map(str::to_owned)
        .collect()
}

fn schema_errors(schema: &Value, instance: Option<&Value>) -> Vec<String> {
    let mut errors = Vec::new();
    if !jsonschema::meta::is_valid(schema) {
        errors.push("schema is invalid against the JSON Schema meta-schema".into());
        return errors;
    }

    if let Some(instance) = instance {
        let validator = match draft202012::options()
            .should_validate_formats(true)
            .build(schema)
        {
            Ok(value) => value,
            Err(error) => {
                errors.push(format!("cannot compile Draft 2020-12 schema: {error}"));
                return errors;
            }
        };
        for error in validator.iter_errors(instance) {
            errors.push(format!(
                "schema error at {}: {error}",
                error.instance_path()
            ));
        }
    }
    errors
}

fn expected_action(class: &str) -> Option<&'static str> {
    EXPECTED_CLASSES
        .iter()
        .find(|(truth_class, _, _)| *truth_class == class)
        .map(|(_, _, action)| *action)
}

fn semantic_errors(corpus: &Value) -> Vec<String> {
    let mut errors = Vec::new();

    if str_at(corpus, "schema") != Some("mycelix.sa-payment-human-decision/v1") {
        errors.push("unexpected corpus schema identity".into());
    }
    if str_at(corpus, "status") != Some("synthetic_development_only") {
        errors.push("corpus must remain synthetic_development_only".into());
    }
    if str_at(corpus, "holdout_status") != Some("not_a_blinded_holdout") {
        errors.push("public development corpus must not claim blinded-holdout status".into());
    }
    if corpus
        .get("external_effects_enabled")
        .and_then(Value::as_bool)
        != Some(false)
    {
        errors.push("external_effects_enabled must be false".into());
    }
    if corpus
        .get("participant_data_included")
        .and_then(Value::as_bool)
        != Some(false)
    {
        errors.push("participant_data_included must be false".into());
    }

    let design = &corpus["design"];
    if design
        .get("single_composite_score_prohibited")
        .and_then(Value::as_bool)
        != Some(true)
    {
        errors.push("design must prohibit a single composite score".into());
    }
    if str_at(design, "human_study_status") != Some("not_approved_not_started") {
        errors.push("development corpus cannot imply human-study approval or recruitment".into());
    }

    let scenarios = array_at(corpus, "scenarios").cloned().unwrap_or_default();
    if scenarios.len() != 24 {
        errors.push(format!(
            "development corpus must contain exactly 24 tasks, found {}",
            scenarios.len()
        ));
    }

    let mut task_ids = BTreeSet::new();
    let mut request_ids = BTreeSet::new();
    let mut class_counts: BTreeMap<String, usize> = BTreeMap::new();

    for (index, task) in scenarios.iter().enumerate() {
        let path = format!("scenarios[{index}]");
        let task_id = str_at(task, "task_id").unwrap_or("");
        let class = str_at(task, "truth_class").unwrap_or("");
        let action = str_at(task, "expected_action").unwrap_or("");

        if task_id.is_empty() || !task_ids.insert(task_id.to_owned()) {
            errors.push(format!("{path}: missing or duplicate task_id"));
        }
        *class_counts.entry(class.to_owned()).or_default() += 1;

        if expected_action(class) != Some(action) {
            errors.push(format!(
                "{path}: expected action is inconsistent with truth class"
            ));
        }

        if task.get("simulation_only").and_then(Value::as_bool) != Some(true)
            || task.get("real_money_enabled").and_then(Value::as_bool) != Some(false)
            || task.get("pii_present").and_then(Value::as_bool) != Some(false)
        {
            errors.push(format!("{path}: unsafe or non-synthetic task boundary"));
        }

        if str_at(task, "locale") != Some("en-ZA")
            || str_at(task, "translation_status") != Some("development_english_only")
        {
            errors.push(format!(
                "{path}: untranslated development locale was misrepresented"
            ));
        }

        let request = &task["request"];
        let request_id = str_at(request, "request_id").unwrap_or("");
        if !request_id.starts_with("SIM-HD-") || !request_ids.insert(request_id.to_owned()) {
            errors.push(format!(
                "{path}: request identity is missing, non-synthetic or duplicated"
            ));
        }
        if !str_at(request, "reference")
            .is_some_and(|reference| reference.starts_with("SYNTHETIC-REF-"))
        {
            errors.push(format!("{path}: non-synthetic reference identifier"));
        }
        if request
            .get("amount_minor_units")
            .and_then(Value::as_u64)
            .is_none_or(|n| n == 0)
        {
            errors.push(format!("{path}: amount must be a positive integer"));
        }
        if str_at(request, "currency") != Some("ZAR") {
            errors.push(format!("{path}: expected synthetic ZAR task"));
        }

        let baseline = &task["baseline_presentation"];
        let treatment = &task["treatment_presentation"];
        let baseline_fields = string_set(baseline, "visible_fields");
        let treatment_fields = string_set(treatment, "visible_fields");
        let critical_fields = string_set(task, "critical_material_fields");

        if baseline_fields.is_empty() || treatment_fields.is_empty() {
            errors.push(format!(
                "{path}: both presentation arms require visible fields"
            ));
        }
        if !baseline_fields.is_subset(&treatment_fields) {
            errors.push(format!(
                "{path}: baseline fields must be represented in the treatment"
            ));
        }
        if !critical_fields.is_subset(&treatment_fields) {
            let missing: Vec<_> = critical_fields
                .difference(&treatment_fields)
                .cloned()
                .collect();
            errors.push(format!(
                "{path}: treatment omits critical material fields {missing:?}"
            ));
        }
        if str_at(baseline, "warning_style") != Some("generic_confirmation") {
            errors.push(format!(
                "{path}: baseline is not the declared comparison presentation"
            ));
        }
        if str_at(treatment, "warning_style") != Some("evidence_and_uncertainty_explicit") {
            errors.push(format!(
                "{path}: treatment must explicitly surface evidence and uncertainty"
            ));
        }
        if !str_at(task, "reference_rationale")
            .is_some_and(|rationale| rationale.contains("not a legal finding"))
        {
            errors.push(format!(
                "{path}: rationale must not assert a legal debt determination"
            ));
        }
        if !array_at(task, "presentation_modes").is_some_and(|modes| {
            modes.iter().any(|mode| mode == "mobile_simulation")
                && modes.iter().any(|mode| mode == "screen_reader_simulation")
        }) {
            errors.push(format!(
                "{path}: mobile and screen-reader simulation modes are required"
            ));
        }
    }

    for (class, expected_count, _) in EXPECTED_CLASSES {
        let actual = class_counts.get(class).copied().unwrap_or(0);
        if actual != expected_count {
            errors.push(format!(
                "truth class {class} must have {expected_count} tasks; found {actual}"
            ));
        }
    }

    let declared = &design["truth_class_counts"];
    for (class, expected_count, _) in EXPECTED_CLASSES {
        if declared.get(class).and_then(Value::as_u64) != Some(expected_count as u64) {
            errors.push(format!("design truth_class_counts mismatch for {class}"));
        }
    }

    errors
}

fn evaluate(
    corpus_text: &str,
    schema_text: &str,
    response_schema_text: &str,
) -> Result<(Vec<String>, Value, Value, Value), String> {
    let corpus: Value = serde_json::from_str(corpus_text)
        .map_err(|error| format!("corpus JSON parse error: {error}"))?;
    let schema: Value = serde_json::from_str(schema_text)
        .map_err(|error| format!("task schema JSON parse error: {error}"))?;
    let response_schema: Value = serde_json::from_str(response_schema_text)
        .map_err(|error| format!("response schema JSON parse error: {error}"))?;

    let mut errors = semantic_errors(&corpus);
    errors.extend(schema_errors(&schema, Some(&corpus)));
    errors.extend(schema_errors(&response_schema, None));
    Ok((errors, corpus, schema, response_schema))
}

fn mutate_and_reject(
    label: &str,
    corpus: &Value,
    schema: &Value,
    mutation: impl FnOnce(&mut Value),
) -> Result<(), String> {
    let mut changed = corpus.clone();
    mutation(&mut changed);
    let mut errors = semantic_errors(&changed);
    errors.extend(schema_errors(schema, Some(&changed)));
    if errors.is_empty() {
        return Err(format!("negative control was not rejected: {label}"));
    }
    Ok(())
}

fn run_self_tests(corpus: &Value, schema: &Value) -> Result<usize, String> {
    let mut baseline_errors = semantic_errors(corpus);
    baseline_errors.extend(schema_errors(schema, Some(corpus)));
    if !baseline_errors.is_empty() {
        return Err(format!(
            "baseline must pass before mutations: {baseline_errors:?}"
        ));
    }

    let mut count = 0usize;

    mutate_and_reject("real money enabled", corpus, schema, |value| {
        value["external_effects_enabled"] = json!(true);
    })?;
    count += 1;

    mutate_and_reject("participant data included", corpus, schema, |value| {
        value["participant_data_included"] = json!(true);
    })?;
    count += 1;

    mutate_and_reject("duplicate task identity", corpus, schema, |value| {
        value["scenarios"][1]["task_id"] = value["scenarios"][0]["task_id"].clone();
    })?;
    count += 1;

    mutate_and_reject(
        "duplicate synthetic request identity",
        corpus,
        schema,
        |value| {
            value["scenarios"][1]["request"]["request_id"] =
                value["scenarios"][0]["request"]["request_id"].clone();
        },
    )?;
    count += 1;

    mutate_and_reject("unbalanced truth classes", corpus, schema, |value| {
        value["scenarios"].as_array_mut().unwrap().pop();
    })?;
    count += 1;

    mutate_and_reject("ambiguous case mislabeled safe", corpus, schema, |value| {
        value["scenarios"][16]["expected_action"] = json!("accept");
    })?;
    count += 1;

    mutate_and_reject("deceptive case mislabeled safe", corpus, schema, |value| {
        value["scenarios"][8]["expected_action"] = json!("accept");
    })?;
    count += 1;

    mutate_and_reject("treatment hides beneficiary", corpus, schema, |value| {
        value["scenarios"][0]["treatment_presentation"]["visible_fields"] =
            json!(["amount", "currency", "fee"]);
    })?;
    count += 1;

    mutate_and_reject("treatment uses generic warning", corpus, schema, |value| {
        value["scenarios"][0]["treatment_presentation"]["warning_style"] =
            json!("generic_confirmation");
    })?;
    count += 1;

    mutate_and_reject("study falsely marked approved", corpus, schema, |value| {
        value["design"]["human_study_status"] = json!("approved_and_recruiting");
    })?;
    count += 1;

    mutate_and_reject(
        "development corpus claims blinded holdout",
        corpus,
        schema,
        |value| {
            value["holdout_status"] = json!("blinded_holdout");
        },
    )?;
    count += 1;

    mutate_and_reject(
        "legal debt asserted from synthetic evidence",
        corpus,
        schema,
        |value| {
            value["scenarios"][0]["reference_rationale"] =
                json!("The payer legally owes this debt.");
        },
    )?;
    count += 1;

    mutate_and_reject(
        "baseline and treatment no longer matched",
        corpus,
        schema,
        |value| {
            value["scenarios"][0]["treatment_presentation"]["visible_fields"] =
                json!(["amount", "currency", "fee"]);
        },
    )?;
    count += 1;

    mutate_and_reject(
        "translation claimed without review",
        corpus,
        schema,
        |value| {
            value["scenarios"][0]["translation_status"] = json!("human_reviewed_isiZulu");
        },
    )?;
    count += 1;

    Ok(count)
}

fn main() -> Result<(), Box<dyn Error>> {
    let mut corpus_text = DEFAULT_CORPUS.to_owned();
    let mut schema_text = DEFAULT_SCHEMA.to_owned();
    let mut response_schema_text = DEFAULT_RESPONSE_SCHEMA.to_owned();
    let mut self_test = false;
    let mut args = std::env::args().skip(1);

    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--corpus" => {
                let path = PathBuf::from(args.next().ok_or_else(|| {
                    std::io::Error::new(
                        std::io::ErrorKind::InvalidInput,
                        "--corpus requires a path",
                    )
                })?);
                corpus_text = fs::read_to_string(path)?;
            }
            "--schema" => {
                let path = PathBuf::from(args.next().ok_or_else(|| {
                    std::io::Error::new(
                        std::io::ErrorKind::InvalidInput,
                        "--schema requires a path",
                    )
                })?);
                schema_text = fs::read_to_string(path)?;
            }
            "--response-schema" => {
                let path = PathBuf::from(args.next().ok_or_else(|| {
                    std::io::Error::new(
                        std::io::ErrorKind::InvalidInput,
                        "--response-schema requires a path",
                    )
                })?);
                response_schema_text = fs::read_to_string(path)?;
            }
            "--self-test" => self_test = true,
            "--help" | "-h" => {
                println!(
                    "sa-payment-human-decision-evaluator [--corpus PATH] [--schema PATH] [--response-schema PATH] [--self-test]"
                );
                return Ok(());
            }
            other => {
                return Err(std::io::Error::new(
                    std::io::ErrorKind::InvalidInput,
                    format!("unknown argument: {other}"),
                )
                .into());
            }
        }
    }

    let (errors, corpus, schema, _) =
        evaluate(&corpus_text, &schema_text, &response_schema_text)
            .map_err(|message| std::io::Error::new(std::io::ErrorKind::InvalidData, message))?;
    let mutation_count = if self_test {
        run_self_tests(&corpus, &schema)?
    } else {
        0
    };
    let status = if errors.is_empty() { "PASS" } else { "FAIL" };
    let report = json!({
        "evaluator": "mycelix.sa-payment-human-decision-evaluator.v1",
        "status": status,
        "external_effects_enabled": false,
        "participant_data_included": false,
        "claim_ceiling": "synthetic study-instrument schema, balance and semantic qualification only",
        "task_count": array_at(&corpus, "scenarios").map_or(0, Vec::len),
        "negative_controls": mutation_count,
        "human_study_status": str_at(&corpus["design"], "human_study_status"),
        "holdout_status": str_at(&corpus, "holdout_status"),
        "errors": errors
    });
    println!("{}", serde_json::to_string_pretty(&report)?);
    if status != "PASS" {
        std::process::exit(1);
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn development_corpus_and_both_schemas_validate() {
        let (errors, corpus, _, _) =
            evaluate(DEFAULT_CORPUS, DEFAULT_SCHEMA, DEFAULT_RESPONSE_SCHEMA).expect("evaluation");
        assert!(errors.is_empty(), "{errors:?}");
        assert_eq!(array_at(&corpus, "scenarios").unwrap().len(), 24);
    }

    #[test]
    fn truth_classes_and_reference_actions_are_balanced() {
        let corpus: Value = serde_json::from_str(DEFAULT_CORPUS).expect("corpus");
        for (class, expected, action) in EXPECTED_CLASSES {
            let tasks: Vec<_> = array_at(&corpus, "scenarios")
                .unwrap()
                .iter()
                .filter(|task| str_at(task, "truth_class") == Some(class))
                .collect();
            assert_eq!(tasks.len(), expected);
            assert!(
                tasks
                    .iter()
                    .all(|task| str_at(task, "expected_action") == Some(action))
            );
        }
    }

    #[test]
    fn negative_mutation_campaign_rejects_unsafe_study_instruments() {
        let corpus: Value = serde_json::from_str(DEFAULT_CORPUS).expect("corpus");
        let schema: Value = serde_json::from_str(DEFAULT_SCHEMA).expect("schema");
        assert_eq!(run_self_tests(&corpus, &schema).expect("self tests"), 14);
    }
}
