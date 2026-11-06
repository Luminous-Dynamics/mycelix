#![forbid(unsafe_code)]

//! Independent prevalidation adjudication evaluator.
//! System outputs are compared with blinded adjudicator labels; they are never ground truth.

use jsonschema::draft202012;
use serde_json::{Map, Value, json};
use std::collections::{BTreeMap, BTreeSet};
use std::error::Error;
use std::fs;
use std::path::PathBuf;

const DEFAULT_CORPUS: &str =
    include_str!("../../../../mycelix-finance/docs/payment-prevalidation-adjudication-v1.json");
const DEFAULT_SCHEMA: &str = include_str!(
    "../../../../mycelix-finance/docs/payment-prevalidation-adjudication-v1.schema.json"
);
const SYSTEMS: [&str; 3] = ["incumbent", "conventional", "mycelix"];
const LABEL_KEYS: [&str; 9] = [
    "subject_binding",
    "payee_name_comparison",
    "requester_identity",
    "requester_authority",
    "claim_validity",
    "profile_eligibility",
    "intent_integrity",
    "evidence_freshness",
    "final_disposition",
];

fn string_at<'a>(value: &'a Value, key: &str) -> Option<&'a str> {
    value.get(key).and_then(Value::as_str)
}

fn bool_at(value: &Value, key: &str) -> Option<bool> {
    value.get(key).and_then(Value::as_bool)
}

fn array_at<'a>(value: &'a Value, key: &str) -> Option<&'a Vec<Value>> {
    value.get(key).and_then(Value::as_array)
}

fn u64_value(value: &Value, key: &str) -> Option<u64> {
    value.get(key).and_then(Value::as_u64)
}

fn is_ready_disposition(value: &str) -> bool {
    matches!(value, "safe_to_present" | "warn_and_permit_choice")
}

fn is_hard_quarantine(value: &str) -> bool {
    matches!(value, "quarantine" | "reject" | "ineligible")
}

fn label_maps_equal(left: &Value, right: &Value) -> bool {
    LABEL_KEYS
        .iter()
        .all(|key| string_at(left, key) == string_at(right, key))
}

fn schema_errors(schema: &Value, corpus: &Value) -> Vec<String> {
    let mut errors = Vec::new();
    if !jsonschema::meta::is_valid(schema) {
        errors.push("schema is invalid against its JSON Schema meta-schema".into());
        return errors;
    }
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
    for error in validator.iter_errors(corpus) {
        errors.push(format!(
            "schema error at {}: {error}",
            error.instance_path()
        ));
    }
    errors
}

fn validate_study_design(corpus: &Value, errors: &mut Vec<String>) {
    let design = &corpus["study_design"];
    if bool_at(design, "preregistered") != Some(true)
        || bool_at(design, "thresholds_frozen") != Some(true)
        || bool_at(design, "held_out_test_tuning_access") != Some(false)
    {
        errors.push(
            "study_design: pre-registration, frozen thresholds and held-out isolation are mandatory"
                .into(),
        );
    }
    if string_at(design, "split_policy")
        != Some("merchant_participant_payee_identity_and_time_block_disjoint")
    {
        errors.push("study_design: unsupported or missing entity/time split policy".into());
    }
    let required: BTreeSet<&str> = SYSTEMS.into_iter().collect();
    let configured: BTreeSet<&str> = array_at(design, "required_systems")
        .into_iter()
        .flatten()
        .filter_map(Value::as_str)
        .collect();
    if configured != required {
        errors.push(
            "study_design: required systems must be exactly incumbent, conventional and mycelix"
                .into(),
        );
    }
    let tuning: BTreeSet<&str> = array_at(design, "tuning_partitions")
        .into_iter()
        .flatten()
        .filter_map(Value::as_str)
        .collect();
    if !tuning.is_subset(&BTreeSet::from(["development", "calibration"])) {
        errors.push("study_design: held-out or unknown partition is available for tuning".into());
    }
    if string_at(design, "confidence_interval_method")
        != Some("cluster_bootstrap_by_merchant_and_time_block")
    {
        errors.push(
            "study_design: confidence intervals must use the pre-registered cluster method".into(),
        );
    }
    if u64_value(design, "minimum_independent_clusters_for_interval").unwrap_or(0) < 30 {
        errors.push("study_design: minimum independent cluster count must be at least 30".into());
    }
}

fn validate_source_refs(
    case: &Value,
    source_ids: &BTreeSet<String>,
    case_index: usize,
    errors: &mut Vec<String>,
) {
    for source in array_at(case, "source_refs").into_iter().flatten() {
        if !source.as_str().is_some_and(|id| source_ids.contains(id)) {
            errors.push(format!(
                "cases[{case_index}].source_refs: unresolved source reference"
            ));
        }
    }
}

fn resolve_adjudicated_labels(
    case: &Value,
    case_index: usize,
    errors: &mut Vec<String>,
) -> Option<(Value, Value, Value)> {
    let records = array_at(case, "adjudication_records")?;
    let digest = string_at(case, "case_snapshot_digest").unwrap_or("");
    let mut ids = BTreeMap::<String, Value>::new();
    let mut superseded_targets = BTreeSet::<String>::new();
    let mut previous_sequence = 0_u64;
    let mut corrections_by_target = BTreeSet::<String>::new();

    for (record_index, record) in records.iter().enumerate() {
        let path = format!("cases[{case_index}].adjudication_records[{record_index}]");
        let id = string_at(record, "record_id").unwrap_or("");
        let sequence = u64_value(record, "sequence").unwrap_or(0);
        if id.is_empty() || ids.contains_key(id) {
            errors.push(format!(
                "{path}: missing or duplicate adjudication record id"
            ));
        }
        if sequence <= previous_sequence {
            errors.push(format!(
                "{path}: adjudication sequence must be strictly increasing"
            ));
        }
        previous_sequence = sequence;
        if bool_at(record, "blinded_to_system_outputs") != Some(true) {
            errors.push(format!(
                "{path}: adjudicator was not blind to system outputs"
            ));
        }
        if string_at(record, "evidence_digest") != Some(digest) {
            errors.push(format!(
                "{path}: adjudication evidence digest differs from frozen case snapshot"
            ));
        }
        if string_at(record, "protocol_version") != Some("PAY-PREVALIDATION-ADJUDICATION-V1") {
            errors.push(format!("{path}: adjudication protocol version mismatch"));
        }
        ids.insert(id.to_owned(), record.clone());
    }

    for (record_index, record) in records.iter().enumerate() {
        let path = format!("cases[{case_index}].adjudication_records[{record_index}]");
        let role = string_at(record, "role").unwrap_or("");
        let status = string_at(record, "status").unwrap_or("");
        let supersedes = record.get("supersedes_record_id").and_then(Value::as_str);
        match role {
            "primary" | "tie_breaker" => {
                if supersedes.is_some()
                    || record
                        .get("correction_reason")
                        .is_some_and(|value| !value.is_null())
                {
                    errors.push(format!(
                        "{path}: original adjudication cannot supersede another record"
                    ));
                }
            }
            "correction" => {
                let Some(target_id) = supersedes else {
                    errors.push(format!("{path}: correction must identify its predecessor"));
                    continue;
                };
                let Some(target) = ids.get(target_id) else {
                    errors.push(format!(
                        "{path}: correction predecessor {target_id} is missing"
                    ));
                    continue;
                };
                if string_at(target, "status") != Some("superseded")
                    || string_at(target, "adjudicator_id") != string_at(record, "adjudicator_id")
                    || !corrections_by_target.insert(target_id.to_owned())
                    || string_at(record, "correction_reason").is_none_or(str::is_empty)
                    || status != "active"
                {
                    errors.push(format!(
                        "{path}: invalid correction lineage or actor binding"
                    ));
                }
                superseded_targets.insert(target_id.to_owned());
            }
            _ => errors.push(format!("{path}: unsupported adjudication role")),
        }
        if status == "superseded" && role == "correction" {
            errors.push(format!(
                "{path}: a correction cannot itself be superseded in this v1 profile"
            ));
        }
    }

    for (record_index, record) in records.iter().enumerate() {
        if string_at(record, "status") == Some("superseded") {
            let id = string_at(record, "record_id").unwrap_or("");
            if !superseded_targets.contains(id) {
                errors.push(format!(
                    "cases[{case_index}].adjudication_records[{record_index}]: superseded record lacks an append-only correction successor"
                ));
            }
        }
    }

    let active_primaries: Vec<&Value> = records
        .iter()
        .filter(|record| {
            string_at(record, "status") == Some("active")
                && matches!(string_at(record, "role"), Some("primary" | "correction"))
        })
        .collect();
    if active_primaries.len() != 2 {
        errors.push(format!(
            "cases[{case_index}]: exactly two active independent primary records are required"
        ));
        return None;
    }
    let left_adjudicator = string_at(active_primaries[0], "adjudicator_id").unwrap_or("");
    let right_adjudicator = string_at(active_primaries[1], "adjudicator_id").unwrap_or("");
    if left_adjudicator.is_empty() || left_adjudicator == right_adjudicator {
        errors.push(format!(
            "cases[{case_index}]: current primary labels are not independent adjudications"
        ));
    }

    let tie_breakers: Vec<&Value> = records
        .iter()
        .filter(|record| {
            string_at(record, "role") == Some("tie_breaker")
                && string_at(record, "status") == Some("active")
        })
        .collect();
    let left_labels = &active_primaries[0]["labels"];
    let right_labels = &active_primaries[1]["labels"];
    let primary_agree = label_maps_equal(left_labels, right_labels);
    if primary_agree && !tie_breakers.is_empty() {
        errors.push(format!(
            "cases[{case_index}]: unnecessary tie-breaker exists despite complete primary agreement"
        ));
    }
    if !primary_agree && tie_breakers.len() != 1 {
        errors.push(format!(
            "cases[{case_index}]: primary disagreement requires exactly one third adjudicator"
        ));
    }
    if tie_breakers.len() > 1 {
        errors.push(format!(
            "cases[{case_index}]: multiple active tie-breakers are not permitted"
        ));
    }
    if let Some(tie) = tie_breakers.first() {
        let tie_id = string_at(tie, "adjudicator_id").unwrap_or("");
        if tie_id == left_adjudicator || tie_id == right_adjudicator {
            errors.push(format!(
                "cases[{case_index}]: tie-breaker is not independent of primary adjudicators"
            ));
        }
        if string_at(tie, "evidence_digest") != Some(digest) {
            errors.push(format!(
                "cases[{case_index}]: tie-breaker did not review the frozen evidence snapshot"
            ));
        }
    }

    let final_labels = if !primary_agree {
        tie_breakers
            .first()
            .map(|record| record["labels"].clone())?
    } else {
        left_labels.clone()
    };
    Some((final_labels, left_labels.clone(), right_labels.clone()))
}

fn validate_partitions(cases: &[Value], errors: &mut Vec<String>) {
    let mut ownership = BTreeMap::<(String, String), String>::new();
    for (index, case) in cases.iter().enumerate() {
        let partition = string_at(case, "partition").unwrap_or("");
        for field in [
            "merchant_entity_id",
            "participant_id",
            "payee_identity_id",
            "time_block",
        ] {
            let value = string_at(case, field).unwrap_or("");
            let key = (field.to_owned(), value.to_owned());
            match ownership.get(&key) {
                Some(existing) if existing != partition => {
                    errors.push(format!(
                        "cases[{index}].{field}: entity/time leakage across {existing} and {partition}"
                    ));
                }
                Some(_) => {}
                None => {
                    ownership.insert(key, partition.to_owned());
                }
            }
        }
    }
}

fn validate_system_outputs(
    case: &Value,
    case_index: usize,
    errors: &mut Vec<String>,
) -> Option<BTreeMap<String, Value>> {
    let outputs = array_at(case, "system_outputs")?;
    let mut map = BTreeMap::new();
    for (index, output) in outputs.iter().enumerate() {
        let system_id = string_at(output, "system_id").unwrap_or("");
        if !SYSTEMS.contains(&system_id) || map.contains_key(system_id) {
            errors.push(format!(
                "cases[{case_index}].system_outputs[{index}]: missing, unknown or duplicate system id"
            ));
        } else {
            map.insert(system_id.to_owned(), output.clone());
        }
    }
    if map.len() != SYSTEMS.len() {
        errors.push(format!(
            "cases[{case_index}]: outputs required for incumbent, conventional and mycelix"
        ));
    }
    Some(map)
}

fn validate_corpus(corpus: &Value, schema: &Value) -> Vec<String> {
    let mut errors = schema_errors(schema, corpus);
    if string_at(corpus, "schema") != Some("mycelix.payment-prevalidation-adjudication/v1")
        || string_at(corpus, "corpus_id") != Some("PAYMENT-PREVALIDATION-ADJUDICATION-SYNTHETIC-V1")
        || string_at(corpus, "status") != Some("synthetic_protocol_conformance_only")
    {
        errors.push("corpus identity/status mismatch".into());
    }
    if bool_at(&corpus["study_design"], "synthetic_fixture") != Some(true) {
        errors.push("study_design must identify this fixture as synthetic".into());
    }
    validate_study_design(corpus, &mut errors);

    let sources = array_at(corpus, "source_register")
        .cloned()
        .unwrap_or_default();
    let cases = array_at(corpus, "cases").cloned().unwrap_or_default();
    let source_ids: BTreeSet<String> = sources
        .iter()
        .filter_map(|source| string_at(source, "id").map(str::to_owned))
        .collect();
    if source_ids.len() != sources.len() {
        errors.push("source_register: duplicate source ids".into());
    }

    let mut case_ids = BTreeSet::new();
    for (index, case) in cases.iter().enumerate() {
        let case_id = string_at(case, "case_id").unwrap_or("");
        if case_id.is_empty() || !case_ids.insert(case_id.to_owned()) {
            errors.push(format!(
                "cases[{index}]: missing or duplicate immutable case id"
            ));
        }
        validate_source_refs(case, &source_ids, index, &mut errors);
        resolve_adjudicated_labels(case, index, &mut errors);
        validate_system_outputs(case, index, &mut errors);
    }
    validate_partitions(&cases, &mut errors);
    errors
}

fn json_matrix(matrix: &BTreeMap<String, BTreeMap<String, u64>>) -> Value {
    let outer: Map<String, Value> = matrix
        .iter()
        .map(|(truth, predictions)| {
            let inner: Map<String, Value> = predictions
                .iter()
                .map(|(prediction, count)| (prediction.clone(), json!(count)))
                .collect();
            (truth.clone(), Value::Object(inner))
        })
        .collect();
    Value::Object(outer)
}

fn cohen_kappa(left: &[String], right: &[String]) -> Option<f64> {
    if left.len() != right.len() || left.is_empty() {
        return None;
    }
    let n = left.len() as f64;
    let mut left_counts = BTreeMap::<String, u64>::new();
    let mut right_counts = BTreeMap::<String, u64>::new();
    let mut matches = 0_u64;
    for (a, b) in left.iter().zip(right) {
        *left_counts.entry(a.clone()).or_default() += 1;
        *right_counts.entry(b.clone()).or_default() += 1;
        if a == b {
            matches += 1;
        }
    }
    let observed = matches as f64 / n;
    let expected: f64 = left_counts
        .iter()
        .map(|(category, a_count)| {
            let b_count = right_counts.get(category).copied().unwrap_or(0);
            (*a_count as f64 / n) * (b_count as f64 / n)
        })
        .sum();
    if (1.0 - expected).abs() < f64::EPSILON {
        None
    } else {
        Some((observed - expected) / (1.0 - expected))
    }
}

fn report(corpus: &Value, schema: &Value) -> Value {
    let validation_errors = validate_corpus(corpus, schema);
    if !validation_errors.is_empty() {
        return json!({
            "evaluator": "mycelix.payment-prevalidation-adjudication.v1",
            "status": "FAIL",
            "errors": validation_errors
        });
    }

    let cases = array_at(corpus, "cases").cloned().unwrap_or_default();
    let mut resolved = Vec::new();
    let mut adjudicator_pairs: BTreeMap<String, (Vec<String>, Vec<String>)> = BTreeMap::new();
    let mut partition_counts = BTreeMap::<String, u64>::new();
    let mut merchant_time_clusters = BTreeSet::<(String, String)>::new();

    for (index, case) in cases.iter().enumerate() {
        let (labels, left, right) = resolve_adjudicated_labels(&case, index, &mut Vec::new())
            .expect("validated adjudication");
        let outputs = validate_system_outputs(&case, index, &mut Vec::new())
            .expect("validated system outputs");
        for key in LABEL_KEYS {
            let pair = adjudicator_pairs
                .entry(key.to_owned())
                .or_insert_with(|| (Vec::new(), Vec::new()));
            pair.0.push(string_at(&left, key).unwrap_or("").to_owned());
            pair.1.push(string_at(&right, key).unwrap_or("").to_owned());
        }
        *partition_counts
            .entry(string_at(&case, "partition").unwrap_or("").to_owned())
            .or_default() += 1;
        merchant_time_clusters.insert((
            string_at(&case, "merchant_entity_id")
                .unwrap_or("")
                .to_owned(),
            string_at(&case, "time_block").unwrap_or("").to_owned(),
        ));
        resolved.push((case, labels, outputs));
    }

    let mut system_reports = Vec::new();
    for system in SYSTEMS {
        let mut matrices: BTreeMap<String, BTreeMap<String, BTreeMap<String, u64>>> =
            BTreeMap::new();
        let mut false_ready = 0_u64;
        let mut false_ready_denominator = 0_u64;
        let mut false_quarantine = 0_u64;
        let mut false_quarantine_denominator = 0_u64;
        let mut warning_omissions = 0_u64;
        let mut warning_cases = 0_u64;
        let mut indeterminate_truth_labels = BTreeMap::<String, u64>::new();

        for (case, truth, outputs) in &resolved {
            let output = &outputs[system];
            let prediction = string_at(output, "decision").unwrap_or("");
            let true_disposition = string_at(truth, "final_disposition").unwrap_or("");
            let truth_ready = is_ready_disposition(true_disposition);
            let predicted_ready = is_ready_disposition(prediction);

            if !truth_ready {
                false_ready_denominator += 1;
                if predicted_ready {
                    false_ready += 1;
                }
            }
            if truth_ready {
                false_quarantine_denominator += 1;
                if is_hard_quarantine(prediction) {
                    false_quarantine += 1;
                }
            }
            let required_warnings = array_at(case, "required_warning_codes")
                .into_iter()
                .flatten()
                .filter_map(Value::as_str)
                .collect::<BTreeSet<_>>();
            if !required_warnings.is_empty() {
                warning_cases += 1;
                let presented = array_at(output, "warnings_presented")
                    .into_iter()
                    .flatten()
                    .filter_map(Value::as_str)
                    .collect::<BTreeSet<_>>();
                if !required_warnings.is_subset(&presented) {
                    warning_omissions += 1;
                }
            }

            for key in LABEL_KEYS {
                let actual = string_at(truth, key).unwrap_or("").to_owned();
                if actual == "indeterminate" || actual == "unknown" {
                    *indeterminate_truth_labels
                        .entry(key.to_owned())
                        .or_default() += 1;
                }
                let predicted = if key == "final_disposition" {
                    prediction.to_owned()
                } else {
                    string_at(&output["assessments"], key)
                        .unwrap_or("")
                        .to_owned()
                };
                *matrices
                    .entry(key.to_owned())
                    .or_default()
                    .entry(actual)
                    .or_default()
                    .entry(predicted)
                    .or_default() += 1;
            }
        }

        let label_matrices: Map<String, Value> = matrices
            .iter()
            .map(|(key, matrix)| (key.clone(), json_matrix(matrix)))
            .collect();
        system_reports.push(json!({
            "system_id": system,
            "false_ready": {
                "numerator": false_ready,
                "denominator": false_ready_denominator,
                "rate": if false_ready_denominator > 0 { Some(false_ready as f64 / false_ready_denominator as f64) } else { None }
            },
            "false_quarantine": {
                "numerator": false_quarantine,
                "denominator": false_quarantine_denominator,
                "rate": if false_quarantine_denominator > 0 { Some(false_quarantine as f64 / false_quarantine_denominator as f64) } else { None }
            },
            "warning_omissions": {
                "numerator": warning_omissions,
                "denominator": warning_cases,
                "rate": if warning_cases > 0 { Some(warning_omissions as f64 / warning_cases as f64) } else { None }
            },
            "indeterminate_truth_label_counts": indeterminate_truth_labels,
            "confusion_matrices": Value::Object(label_matrices)
        }));
    }

    let agreement: Map<String, Value> = adjudicator_pairs
        .iter()
        .map(|(key, (left, right))| {
            let n = left.len() as u64;
            let matches = left.iter().zip(right).filter(|(a, b)| a == b).count() as u64;
            (
                key.clone(),
                json!({
                    "independent_pairs": n,
                    "exact_agreement": matches,
                    "agreement_rate": if n > 0 { Some(matches as f64 / n as f64) } else { None },
                    "cohens_kappa": cohen_kappa(left, right)
                }),
            )
        })
        .collect();

    let cluster_count = merchant_time_clusters.len();
    let suppression_reason = if bool_at(&corpus["study_design"], "synthetic_fixture") == Some(true)
    {
        "synthetic fixture; no population confidence intervals are estimated"
    } else if cluster_count
        < u64_value(
            &corpus["study_design"],
            "minimum_independent_clusters_for_interval",
        )
        .unwrap_or(30) as usize
    {
        "independent merchant/time-block cluster count is below the pre-registered minimum"
    } else {
        "cluster bootstrap is reserved for a separately qualified live-study runner"
    };

    json!({
        "evaluator": "mycelix.payment-prevalidation-adjudication.v1",
        "status": "PASS",
        "claim_ceiling": "synthetic adjudication/report contract conformance only",
        "counts": {
            "cases": cases.len(),
            "adjudication_records": cases.iter().map(|case| array_at(case, "adjudication_records").map_or(0, Vec::len)).sum::<usize>(),
            "systems_per_case": SYSTEMS.len(),
            "merchant_time_clusters": cluster_count,
            "partitions": partition_counts
        },
        "system_metrics": system_reports,
        "adjudicator_agreement": agreement,
        "confidence_intervals": {
            "method": corpus["study_design"]["confidence_interval_method"],
            "estimated": false,
            "suppression_reason": suppression_reason
        },
        "errors": []
    })
}

fn mutation_rejected(
    label: &str,
    corpus: &Value,
    schema: &Value,
    mutate: impl FnOnce(&mut Value),
) -> Result<(), String> {
    let mut changed = corpus.clone();
    mutate(&mut changed);
    if validate_corpus(&changed, schema).is_empty() {
        return Err(format!("negative control was not rejected: {label}"));
    }
    Ok(())
}

fn run_self_tests(corpus: &Value, schema: &Value) -> Result<usize, String> {
    if !validate_corpus(corpus, schema).is_empty() {
        return Err("baseline fixture must pass before mutation tests".into());
    }
    let mut count = 0;

    mutation_rejected(
        "circular evaluator expectation inserted as truth",
        corpus,
        schema,
        |v| {
            v["cases"][0]["expected_output"] = json!("safe_to_present");
        },
    )?;
    count += 1;

    mutation_rejected(
        "adjudicator unblinded to system output",
        corpus,
        schema,
        |v| {
            v["cases"][0]["adjudication_records"][0]["blinded_to_system_outputs"] = json!(false);
        },
    )?;
    count += 1;

    mutation_rejected(
        "merchant identity leaks into held-out split",
        corpus,
        schema,
        |v| {
            v["cases"][6]["merchant_entity_id"] = v["cases"][0]["merchant_entity_id"].clone();
        },
    )?;
    count += 1;

    mutation_rejected(
        "indeterminate evidence label suppressed",
        corpus,
        schema,
        |v| {
            v["cases"][4]["adjudication_records"][0]["labels"]
                .as_object_mut()
                .unwrap()
                .remove("evidence_freshness");
        },
    )?;
    count += 1;

    mutation_rejected("correction predecessor deleted", corpus, schema, |v| {
        v["cases"][7]["adjudication_records"]
            .as_array_mut()
            .unwrap()
            .remove(0);
    })?;
    count += 1;

    mutation_rejected(
        "primary disagreement has no third adjudicator",
        corpus,
        schema,
        |v| {
            v["cases"][2]["adjudication_records"]
                .as_array_mut()
                .unwrap()
                .pop();
        },
    )?;
    count += 1;

    mutation_rejected(
        "held-out partition made available for tuning",
        corpus,
        schema,
        |v| {
            v["study_design"]["tuning_partitions"] =
                json!(["development", "calibration", "held_out_test"]);
        },
    )?;
    count += 1;

    mutation_rejected("thresholds no longer frozen", corpus, schema, |v| {
        v["study_design"]["thresholds_frozen"] = json!(false);
    })?;
    count += 1;

    Ok(count)
}

fn evaluate(corpus_text: &str, schema_text: &str) -> Result<(Value, Value, Value), String> {
    let corpus: Value = serde_json::from_str(corpus_text)
        .map_err(|error| format!("corpus JSON parse error: {error}"))?;
    let schema: Value = serde_json::from_str(schema_text)
        .map_err(|error| format!("schema JSON parse error: {error}"))?;
    Ok((report(&corpus, &schema), corpus, schema))
}

fn main() -> Result<(), Box<dyn Error>> {
    let mut corpus_text = DEFAULT_CORPUS.to_owned();
    let mut schema_text = DEFAULT_SCHEMA.to_owned();
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
            "--self-test" => self_test = true,
            "--help" | "-h" => {
                println!(
                    "payment-prevalidation-adjudication-evaluator [--corpus PATH] [--schema PATH] [--self-test]"
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

    let (mut result, corpus, schema) = evaluate(&corpus_text, &schema_text)
        .map_err(|message| std::io::Error::new(std::io::ErrorKind::InvalidData, message))?;
    if self_test {
        match run_self_tests(&corpus, &schema) {
            Ok(count) => result["negative_mutation_controls"] = json!(count),
            Err(error) => result["negative_mutation_error"] = json!(error),
        }
    }
    println!("{}", serde_json::to_string_pretty(&result)?);
    if result["status"] != "PASS" || result.get("negative_mutation_error").is_some() {
        std::process::exit(1);
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn checked_in_adjudication_corpus_validates_and_reports() {
        let (result, _, _) = evaluate(DEFAULT_CORPUS, DEFAULT_SCHEMA).expect("evaluation");
        assert_eq!(result["status"], "PASS");
        assert_eq!(result["counts"]["cases"], 8);
        assert_eq!(result["counts"]["adjudication_records"], 18);
        assert_eq!(result["confidence_intervals"]["estimated"], false);
        assert_eq!(result["system_metrics"].as_array().unwrap().len(), 3);
    }

    #[test]
    fn synthetic_decision_metrics_keep_false_ready_and_quarantine_separate() {
        let (result, _, _) = evaluate(DEFAULT_CORPUS, DEFAULT_SCHEMA).expect("evaluation");
        let mycelix = result["system_metrics"]
            .as_array()
            .unwrap()
            .iter()
            .find(|metric| metric["system_id"] == "mycelix")
            .unwrap();
        assert_eq!(mycelix["false_ready"]["numerator"], 0);
        assert_eq!(mycelix["false_quarantine"]["numerator"], 1);
        assert_eq!(mycelix["false_quarantine"]["denominator"], 5);
    }

    #[test]
    fn negative_mutation_campaign_rejects_circularity_and_leakage() {
        let corpus: Value = serde_json::from_str(DEFAULT_CORPUS).expect("corpus");
        let schema: Value = serde_json::from_str(DEFAULT_SCHEMA).expect("schema");
        assert_eq!(run_self_tests(&corpus, &schema).expect("mutations"), 8);
    }
}
