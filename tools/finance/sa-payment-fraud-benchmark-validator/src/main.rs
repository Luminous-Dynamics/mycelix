#![forbid(unsafe_code)]

//! Source-bound and effects-disabled benchmark manifest validator.
//! Comparator cohorts never become interchangeable just because they share a fraud metric.

use jsonschema::draft202012;
use serde_json::{Value, json};
use std::collections::BTreeSet;
use std::error::Error;
use std::fs;
use std::path::PathBuf;

const DEFAULT_MANIFEST: &str =
    include_str!("../../../../mycelix-finance/docs/sa-payment-fraud-benchmark-v1.json");
const DEFAULT_SCHEMA: &str =
    include_str!("../../../../mycelix-finance/docs/sa-payment-fraud-benchmark-v1.schema.json");
const EXPECTED_SCHEMA: &str = "mycelix.sa-payment-fraud-benchmark/v1";
const EXPECTED_MANIFEST: &str = "ZA-SA-PAYMENT-FRAUD-BENCHMARK-V1";

fn str_at<'a>(value: &'a Value, key: &str) -> Option<&'a str> {
    value.get(key).and_then(Value::as_str)
}

fn u64_at(value: &Value, key: &str) -> Option<u64> {
    value.get(key).and_then(Value::as_u64)
}

fn array_at<'a>(value: &'a Value, key: &str) -> Option<&'a Vec<Value>> {
    value.get(key).and_then(Value::as_array)
}

fn valid_date(value: &str) -> bool {
    let mut parts = value.split('-');
    let Some(year) = parts.next().and_then(|v| v.parse::<i32>().ok()) else {
        return false;
    };
    let Some(month) = parts.next().and_then(|v| v.parse::<u32>().ok()) else {
        return false;
    };
    let Some(day) = parts.next().and_then(|v| v.parse::<u32>().ok()) else {
        return false;
    };
    if parts.next().is_some() || year < 1970 || !(1..=12).contains(&month) {
        return false;
    }
    let leap = year % 4 == 0 && (year % 100 != 0 || year % 400 == 0);
    let max_day = match month {
        1 | 3 | 5 | 7 | 8 | 10 | 12 => 31,
        4 | 6 | 9 | 11 => 30,
        2 if leap => 29,
        2 => 28,
        _ => return false,
    };
    (1..=max_day).contains(&day)
}

fn date_le(left: &str, right: &str) -> bool {
    valid_date(left) && valid_date(right) && left <= right
}

fn schema_errors(schema: &Value, manifest: &Value) -> Vec<String> {
    let mut errors = Vec::new();
    if !jsonschema::meta::is_valid(schema) {
        errors.push("schema is invalid against the JSON Schema meta-schema".into());
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
    for error in validator.iter_errors(manifest) {
        errors.push(format!(
            "schema error at {}: {error}",
            error.instance_path()
        ));
    }
    errors
}

fn check_unique(values: &[Value], field: &str, path: &str, errors: &mut Vec<String>) {
    let mut seen = BTreeSet::new();
    for (index, value) in values.iter().enumerate() {
        match str_at(value, field) {
            Some(id) if seen.insert(id.to_owned()) => {}
            Some(id) => errors.push(format!("{path}[{index}].{field}: duplicate {id}")),
            None => errors.push(format!("{path}[{index}].{field}: missing string")),
        }
    }
}

fn semantic_errors(manifest: &Value) -> Vec<String> {
    let mut errors = Vec::new();
    let Some(as_of) = str_at(manifest, "as_of") else {
        errors.push("as_of: missing pinned date".into());
        return errors;
    };
    if !valid_date(as_of) {
        errors.push(format!("as_of: invalid date {as_of}"));
    }
    if str_at(manifest, "schema") != Some(EXPECTED_SCHEMA) {
        errors.push("schema: unexpected identity".into());
    }
    if str_at(manifest, "manifest_id") != Some(EXPECTED_MANIFEST) {
        errors.push("manifest_id: unexpected identity".into());
    }
    if str_at(manifest, "status") != Some("benchmark_protocol_and_public_comparators") {
        errors.push("status: unexpected status".into());
    }
    if str_at(manifest, "jurisdiction") != Some("ZA") {
        errors.push("jurisdiction: expected ZA".into());
    }
    if manifest
        .get("external_effects_enabled")
        .and_then(Value::as_bool)
        != Some(false)
    {
        errors.push("external_effects_enabled must be false".into());
    }

    let sources = array_at(manifest, "sources").cloned().unwrap_or_default();
    let cohorts = array_at(manifest, "reference_cohorts")
        .cloned()
        .unwrap_or_default();
    let metrics = array_at(manifest, "metric_definitions")
        .cloned()
        .unwrap_or_default();
    check_unique(&sources, "id", "sources", &mut errors);
    check_unique(&cohorts, "cohort_id", "reference_cohorts", &mut errors);
    check_unique(&metrics, "metric_id", "metric_definitions", &mut errors);

    let source_ids: BTreeSet<String> = sources
        .iter()
        .filter_map(|source| str_at(source, "id").map(str::to_owned))
        .collect();

    for (index, source) in sources.iter().enumerate() {
        let observed = str_at(source, "observed_on").unwrap_or("");
        if !date_le(observed, as_of) {
            errors.push(format!(
                "sources[{index}].observed_on: invalid or after as_of"
            ));
        }
        if let Some(published) = source.get("published_on").and_then(Value::as_str)
            && !date_le(published, as_of)
        {
            errors.push(format!(
                "sources[{index}].published_on: invalid or after as_of"
            ));
        }
    }

    let expected_languages = [
        ("sepedi", "Sepedi", "written_spoken"),
        ("sesotho", "Sesotho", "written_spoken"),
        ("setswana", "Setswana", "written_spoken"),
        ("siswati", "siSwati", "written_spoken"),
        ("tshivenda", "Tshivenda", "written_spoken"),
        ("itsonga", "Xitsonga", "written_spoken"),
        ("afrikaans", "Afrikaans", "written_spoken"),
        ("english", "English", "written_spoken"),
        ("isindebele", "isiNdebele", "written_spoken"),
        ("isixhosa", "isiXhosa", "written_spoken"),
        ("isizulu", "isiZulu", "written_spoken"),
        (
            "south_african_sign_language",
            "South African Sign Language",
            "signed_visual",
        ),
    ];
    let languages = array_at(manifest, "official_language_modalities")
        .cloned()
        .unwrap_or_default();
    check_unique(
        &languages,
        "language_id",
        "official_language_modalities",
        &mut errors,
    );
    if languages.len() != expected_languages.len() {
        errors.push("official language modality matrix must contain exactly 12 languages".into());
    }
    for (language_id, display_name, modality) in expected_languages {
        let Some(language) = languages
            .iter()
            .find(|language| str_at(language, "language_id") == Some(language_id))
        else {
            errors.push(format!(
                "official language modality matrix missing {language_id}"
            ));
            continue;
        };
        if str_at(language, "display_name") != Some(display_name)
            || str_at(language, "modality") != Some(modality)
        {
            errors.push(format!(
                "official language {language_id} has incorrect display name or modality"
            ));
        }
        if str_at(language, "evaluation_state") != Some("not_yet_evaluated")
            || str_at(language, "native_reviewer_state") != Some("not_yet_recruited")
            || str_at(language, "performance_claim") != Some("none")
        {
            errors.push(format!(
                "official language {language_id} cannot claim measured performance before an adjudicated holdout"
            ));
        }
        let refs = array_at(language, "source_refs")
            .cloned()
            .unwrap_or_default();
        if !refs
            .iter()
            .any(|item| item.as_str() == Some("SOUTH_AFRICA_CONSTITUTION_SECTION_6_2026"))
            || refs
                .iter()
                .any(|item| !item.as_str().is_some_and(|id| source_ids.contains(id)))
        {
            errors.push(format!(
                "official language {language_id} lacks a resolved Section 6 source reference"
            ));
        }
        if modality == "signed_visual"
            && array_at(language, "required_evidence").is_none_or(|items| {
                !items
                    .iter()
                    .any(|item| item.as_str() == Some("native SASL reviewer panel"))
            })
        {
            errors.push("SASL requires native signed-language review evidence".into());
        }
    }

    let counterfactuals = array_at(manifest, "counterfactual_pair_definitions")
        .cloned()
        .unwrap_or_default();
    check_unique(
        &counterfactuals,
        "pair_id",
        "counterfactual_pair_definitions",
        &mut errors,
    );
    let required_pair_prefixes = [
        "CF-EQUIV-",
        "CF-NEG-",
        "CF-MATERIAL-",
        "CF-MODALITY-",
        "CF-UNAVAILABLE-",
    ];
    for prefix in required_pair_prefixes {
        if !counterfactuals
            .iter()
            .any(|pair| str_at(pair, "pair_id").is_some_and(|id| id.starts_with(prefix)))
        {
            errors.push(format!(
                "counterfactual challenge set is missing {prefix} cases"
            ));
        }
    }
    for (index, pair) in counterfactuals.iter().enumerate() {
        let pair_id = str_at(pair, "pair_id").unwrap_or("");
        let material = pair.get("material_consent_change").and_then(Value::as_bool);
        if pair_id.starts_with("CF-MATERIAL-") && material != Some(true) {
            errors.push(format!(
                "counterfactual_pair_definitions[{index}]: material mutation must invalidate prior consent"
            ));
        }
        if pair_id.starts_with("CF-EQUIV-") && material != Some(false) {
            errors.push(format!(
                "counterfactual_pair_definitions[{index}]: equivalent presentation change must not be labelled a material consent mutation"
            ));
        }
        if str_at(pair, "evidence_state") != Some("synthetic_template_not_measured") {
            errors.push(format!(
                "counterfactual_pair_definitions[{index}]: unadjudicated template cannot claim measured results"
            ));
        }
        if str_at(pair, "ground_truth_rule").is_none_or(str::is_empty) {
            errors.push(format!(
                "counterfactual_pair_definitions[{index}]: independent adjudication rule missing"
            ));
        }
    }

    for (index, cohort) in cohorts.iter().enumerate() {
        let start = cohort.get("period_start").and_then(Value::as_str);
        let end = cohort.get("period_end").and_then(Value::as_str);
        if let (Some(start), Some(end)) = (start, end)
            && !date_le(start, end)
        {
            errors.push(format!("reference_cohorts[{index}]: invalid period bounds"));
        }
        for field in ["period_start", "period_end"] {
            if let Some(value) = cohort.get(field).and_then(Value::as_str)
                && !date_le(value, as_of)
            {
                errors.push(format!(
                    "reference_cohorts[{index}].{field}: future/invalid date"
                ));
            }
        }

        let refs = array_at(cohort, "source_refs").cloned().unwrap_or_default();
        if refs.is_empty() {
            errors.push(format!("reference_cohorts[{index}].source_refs is empty"));
        }
        for reference in refs {
            if !reference.as_str().is_some_and(|id| source_ids.contains(id)) {
                errors.push(format!(
                    "reference_cohorts[{index}]: unresolved source reference"
                ));
            }
        }

        if let (Some(numerator), Some(denominator)) = (
            u64_at(cohort, "ratio_numerator"),
            u64_at(cohort, "ratio_denominator"),
        ) && (denominator == 0 || numerator > denominator)
        {
            errors.push(format!(
                "reference_cohorts[{index}]: invalid ratio denominator/numerator"
            ));
        }
        if let (Some(reported), Some(eligible)) = (
            u64_at(cohort, "reported_claims"),
            u64_at(cohort, "in_scope_claims"),
        ) && eligible > reported
        {
            errors.push(format!(
                "reference_cohorts[{index}]: in-scope claims exceed reported claims"
            ));
        }

        let reported_rows = array_at(cohort, "reported_row_counts");
        if let Some(rows) = reported_rows {
            let unique: BTreeSet<u64> = rows.iter().filter_map(Value::as_u64).collect();
            if unique.len() > 1
                && str_at(cohort, "row_count_discrepancy_state")
                    != Some("unresolved_conflicting_counts_in_dataset_card")
            {
                errors.push(format!(
                    "reference_cohorts[{index}]: conflicting row counts must remain explicitly unresolved"
                ));
            }
            if unique.len() <= 1
                && str_at(cohort, "row_count_discrepancy_state")
                    == Some("unresolved_conflicting_counts_in_dataset_card")
            {
                errors.push(format!(
                    "reference_cohorts[{index}]: row-count discrepancy state does not match observations"
                ));
            }
        }

        if str_at(cohort, "data_class") == Some("synthetic_research_dataset")
            && str_at(cohort, "comparison_role") != Some("synthetic_stress_corpus_only")
        {
            errors.push(format!(
                "reference_cohorts[{index}]: synthetic data cannot be promoted to observed baseline"
            ));
        }
        if str_at(cohort, "data_class") == Some("peer_reviewed_research_pilot")
            && str_at(cohort, "comparison_role") != Some("architecture_comparator_not_payment_sota")
        {
            errors.push(format!(
                "reference_cohorts[{index}]: non-payment pilot cannot be promoted to payment SOTA"
            ));
        }
        if str_at(cohort, "cohort_id") == Some("UK-APP-REIMBURSEMENT-2024-10_TO_2026-03") {
            let pct = cohort
                .get("reimbursed_value_percent")
                .and_then(Value::as_f64);
            if pct != Some(88.0) {
                errors.push("UK APP 88% value reimbursement baseline lost or changed".into());
            }
            if u64_at(cohort, "reported_claims") != Some(438_300)
                || u64_at(cohort, "in_scope_claims") != Some(301_500)
            {
                errors.push("UK APP reported/in-scope denominators changed".into());
            }
        }
    }

    for (index, metric) in metrics.iter().enumerate() {
        if str_at(metric, "numerator").is_none_or(str::is_empty)
            || str_at(metric, "denominator").is_none_or(str::is_empty)
        {
            errors.push(format!(
                "metric_definitions[{index}]: numerator/denominator contract missing"
            ));
        }
        if str_at(metric, "metric_id") == Some("VALUE_REIMBURSEMENT_RATE")
            && str_at(metric, "denominator") != Some("eligible_claim_value_minor_units")
        {
            errors.push("VALUE_REIMBURSEMENT_RATE must divide by eligible claim value".into());
        }
    }

    let contract = &manifest["evaluation_contract"];
    for field in [
        "temporal_cutoff_required",
        "time_split_required",
        "entity_grouped_holdout_required",
        "random_row_split_only_prohibited",
        "feature_leakage_after_outcome_prohibited",
        "freeze_policy_and_label_versions",
        "unresolved_and_unlabelled_cases_retained",
        "cross_jurisdiction_transfer_claims_prohibited",
        "single_composite_score_prohibited",
        "no_live_provider_data",
        "no_production_deployment_claim",
    ] {
        if contract.get(field).and_then(Value::as_bool) != Some(true) {
            errors.push(format!("evaluation_contract.{field} must remain true"));
        }
    }
    if contract
        .get("default_raw_cross_participant_pii_exchange")
        .and_then(Value::as_bool)
        != Some(false)
    {
        errors.push("default raw cross-participant PII exchange must remain disabled".into());
    }

    for field in [
        "selective_prediction_required",
        "counterfactual_equivalence_tests_required",
        "material_change_reconfirmation_tests_required",
        "language_and_modality_stratification_required",
        "native_language_adjudication_required",
        "distinct_sign_language_presentation_required",
        "no_language_performance_claim_without_adjudicated_holdout",
    ] {
        if contract.get(field).and_then(Value::as_bool) != Some(true) {
            errors.push(format!("evaluation_contract.{field} must remain true"));
        }
    }

    let coverage = array_at(
        contract,
        "selective_prediction_stress_coverage_basis_points",
    )
    .cloned()
    .unwrap_or_default();
    let mut previous_coverage = 0_u64;
    for value in &coverage {
        let Some(current) = value.as_u64() else {
            errors.push(
                "selective prediction coverage thresholds must be integer basis points".into(),
            );
            continue;
        };
        if current <= previous_coverage || current >= 10_000 {
            errors.push("selective prediction coverage thresholds must be strictly increasing and below 10000 basis points".into());
        }
        previous_coverage = current;
    }
    if coverage.len() < 3 {
        errors.push("at least three selective prediction coverage settings are required".into());
    }

    let abstention_actions = array_at(contract, "abstention_actions")
        .cloned()
        .unwrap_or_default();
    for required in [
        "human_review",
        "request_additional_evidence",
        "warning_with_payer_choice_preserved",
    ] {
        if !abstention_actions
            .iter()
            .any(|action| action.as_str() == Some(required))
        {
            errors.push(format!("abstention action {required} is required"));
        }
    }

    let contract_language_ids: BTreeSet<String> = array_at(contract, "official_language_ids")
        .into_iter()
        .flatten()
        .filter_map(Value::as_str)
        .map(str::to_owned)
        .collect();
    let manifest_language_ids: BTreeSet<String> = languages
        .iter()
        .filter_map(|language| str_at(language, "language_id").map(str::to_owned))
        .collect();
    if contract_language_ids != manifest_language_ids {
        errors.push(
            "evaluation_contract official_language_ids do not match the language modality matrix"
                .into(),
        );
    }

    let required_subgroups: BTreeSet<String> = array_at(contract, "required_subgroup_dimensions")
        .into_iter()
        .flatten()
        .filter_map(Value::as_str)
        .map(str::to_owned)
        .collect();
    for subgroup in ["language_id", "accessibility_modality"] {
        if !required_subgroups.contains(subgroup) {
            errors.push(format!(
                "required_subgroup_dimensions must include {subgroup}"
            ));
        }
    }

    let required_pair_types: BTreeSet<String> = array_at(contract, "counterfactual_pair_types")
        .into_iter()
        .flatten()
        .filter_map(Value::as_str)
        .map(str::to_owned)
        .collect();
    let manifest_pair_types: BTreeSet<String> = counterfactuals
        .iter()
        .filter_map(|pair| str_at(pair, "pair_class").map(str::to_owned))
        .collect();
    if required_pair_types != manifest_pair_types {
        errors.push("evaluation_contract counterfactual_pair_types do not match the manifest pair definitions".into());
    }

    let required_metric_denominators = [
        (
            "AUTOMATED_DECISION_COVERAGE",
            "all_eligible_automated_decision_opportunities",
        ),
        (
            "SELECTIVE_RISK_AT_FIXED_COVERAGE",
            "all_non_abstained_automated_decisions_at_fixed_coverage",
        ),
        (
            "COUNTERFACTUAL_EQUIVALENCE_CONSISTENCY",
            "all_adjudicated_equivalent_counterfactual_pairs",
        ),
        (
            "MATERIAL_CHANGE_RECONFIRMATION_SENSITIVITY",
            "all_adjudicated_material_change_pairs",
        ),
        ("SUBGROUP_CALIBRATION_ERROR", "not_applicable"),
    ];
    for (metric_id, denominator) in required_metric_denominators {
        if !metrics.iter().any(|metric| {
            str_at(metric, "metric_id") == Some(metric_id)
                && str_at(metric, "denominator") == Some(denominator)
        }) {
            errors.push(format!(
                "metric {metric_id} has missing or incorrect denominator"
            ));
        }
    }

    let prevalence = array_at(contract, "prevalence_stress_basis_points")
        .cloned()
        .unwrap_or_default();
    let mut previous = None;
    for value in prevalence.iter().filter_map(Value::as_f64) {
        if previous.is_some_and(|old| value <= old) {
            errors.push("prevalence stress levels must be strictly increasing".into());
        }
        previous = Some(value);
    }
    if prevalence.len() < 5 {
        errors.push("at least five prevalence stress levels are required".into());
    }
    if u64_at(contract, "provisional_hot_path_p99_target_ms").is_none_or(|value| value == 0) {
        errors.push("provisional hot-path latency target must be positive".into());
    }

    errors
}

fn evaluate(manifest_text: &str, schema_text: &str) -> Result<(Vec<String>, Value, Value), String> {
    let manifest: Value = serde_json::from_str(manifest_text)
        .map_err(|error| format!("manifest JSON parse error: {error}"))?;
    let schema: Value = serde_json::from_str(schema_text)
        .map_err(|error| format!("schema JSON parse error: {error}"))?;
    let mut errors = semantic_errors(&manifest);
    errors.extend(schema_errors(&schema, &manifest));
    Ok((errors, manifest, schema))
}

fn mutate_and_reject(
    label: &str,
    manifest: &Value,
    schema: &Value,
    mutation: impl FnOnce(&mut Value),
) -> Result<(), String> {
    let mut changed = manifest.clone();
    mutation(&mut changed);
    let mut errors = semantic_errors(&changed);
    errors.extend(schema_errors(schema, &changed));
    if errors.is_empty() {
        return Err(format!("negative control was not rejected: {label}"));
    }
    Ok(())
}

fn run_self_tests(manifest: &Value, schema: &Value) -> Result<usize, String> {
    let mut baseline = semantic_errors(manifest);
    baseline.extend(schema_errors(schema, manifest));
    if !baseline.is_empty() {
        return Err(format!("baseline manifest failed: {baseline:?}"));
    }
    let mut count = 0;

    mutate_and_reject("external effects enabled", manifest, schema, |v| {
        v["external_effects_enabled"] = json!(true);
    })?;
    count += 1;

    mutate_and_reject(
        "synthetic cohort promoted to observed industry baseline",
        manifest,
        schema,
        |v| {
            v["reference_cohorts"][3]["comparison_role"] = json!("external_observed_baseline");
        },
    )?;
    count += 1;

    mutate_and_reject(
        "telecom CDR result promoted to payment SOTA",
        manifest,
        schema,
        |v| {
            v["reference_cohorts"][4]["comparison_role"] = json!("external_observed_baseline");
        },
    )?;
    count += 1;

    mutate_and_reject(
        "SynSEPA conflicting row counts silently resolved",
        manifest,
        schema,
        |v| {
            v["reference_cohorts"][3]["row_count_discrepancy_state"] = json!("resolved");
        },
    )?;
    count += 1;

    mutate_and_reject(
        "eligible claim count exceeds reported claims",
        manifest,
        schema,
        |v| {
            v["reference_cohorts"][2]["in_scope_claims"] = json!(500_000);
        },
    )?;
    count += 1;

    mutate_and_reject(
        "reimbursement denominator changed to all reported claims",
        manifest,
        schema,
        |v| {
            v["metric_definitions"][5]["denominator"] = json!("all_reported_claims");
        },
    )?;
    count += 1;

    mutate_and_reject("random-row-only split enabled", manifest, schema, |v| {
        v["evaluation_contract"]["random_row_split_only_prohibited"] = json!(false);
    })?;
    count += 1;

    mutate_and_reject("cross-participant raw PII enabled", manifest, schema, |v| {
        v["evaluation_contract"]["default_raw_cross_participant_pii_exchange"] = json!(true);
    })?;
    count += 1;

    mutate_and_reject("single composite score enabled", manifest, schema, |v| {
        v["evaluation_contract"]["single_composite_score_prohibited"] = json!(false);
    })?;
    count += 1;

    mutate_and_reject("unknown denominator removed", manifest, schema, |v| {
        v["metric_definitions"][0]["denominator"] = json!("");
    })?;
    count += 1;

    mutate_and_reject("selective prediction disabled", manifest, schema, |v| {
        v["evaluation_contract"]["selective_prediction_required"] = json!(false);
    })?;
    count += 1;

    mutate_and_reject(
        "SASL collapsed into a written language",
        manifest,
        schema,
        |v| {
            v["official_language_modalities"][11]["modality"] = json!("written_spoken");
        },
    )?;
    count += 1;

    mutate_and_reject(
        "language performance claimed without adjudicated holdout",
        manifest,
        schema,
        |v| {
            v["official_language_modalities"][0]["evaluation_state"] = json!("evaluated");
            v["official_language_modalities"][0]["performance_claim"] = json!("measured");
        },
    )?;
    count += 1;

    mutate_and_reject(
        "official language omitted from coverage matrix",
        manifest,
        schema,
        |v| {
            v["official_language_modalities"]
                .as_array_mut()
                .unwrap()
                .remove(11);
        },
    )?;
    count += 1;

    mutate_and_reject(
        "native-language adjudication gate disabled",
        manifest,
        schema,
        |v| {
            v["evaluation_contract"]["native_language_adjudication_required"] = json!(false);
        },
    )?;
    count += 1;

    mutate_and_reject("counterfactual tests disabled", manifest, schema, |v| {
        v["evaluation_contract"]["counterfactual_equivalence_tests_required"] = json!(false);
    })?;
    count += 1;

    mutate_and_reject(
        "material beneficiary mutation no longer requires reconfirmation",
        manifest,
        schema,
        |v| {
            v["counterfactual_pair_definitions"][4]["material_consent_change"] = json!(false);
        },
    )?;
    count += 1;

    mutate_and_reject(
        "selective coverage thresholds are not ordered",
        manifest,
        schema,
        |v| {
            v["evaluation_contract"]["selective_prediction_stress_coverage_basis_points"] =
                json!([5000, 9000, 7500, 9500]);
        },
    )?;
    count += 1;

    mutate_and_reject(
        "language fairness subgroup dimensions omitted",
        manifest,
        schema,
        |v| {
            v["evaluation_contract"]["required_subgroup_dimensions"] =
                json!(["merchant_size", "participant"]);
        },
    )?;
    count += 1;

    Ok(count)
}

fn main() -> Result<(), Box<dyn Error>> {
    let mut manifest_text = DEFAULT_MANIFEST.to_owned();
    let mut schema_text = DEFAULT_SCHEMA.to_owned();
    let mut self_test = false;
    let mut args = std::env::args().skip(1);

    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--manifest" => {
                let path = PathBuf::from(args.next().ok_or_else(|| {
                    std::io::Error::new(
                        std::io::ErrorKind::InvalidInput,
                        "--manifest requires a path",
                    )
                })?);
                manifest_text = fs::read_to_string(path)?;
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
                    "sa-payment-fraud-benchmark-validator [--manifest PATH] [--schema PATH] [--self-test]"
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

    let (errors, manifest, _) = evaluate(&manifest_text, &schema_text)
        .map_err(|message| std::io::Error::new(std::io::ErrorKind::InvalidData, message))?;
    let negative_controls = if self_test {
        let schema: Value = serde_json::from_str(&schema_text)?;
        run_self_tests(&manifest, &schema)?
    } else {
        0
    };
    let status = if errors.is_empty() { "PASS" } else { "FAIL" };
    let report = json!({
        "evaluator": "mycelix.sa-payment-fraud-benchmark-validator.v1",
        "status": status,
        "external_effects_enabled": false,
        "claim_ceiling": "source-bound comparator manifest and evaluation-contract validation only",
        "source_count": array_at(&manifest, "sources").map_or(0, Vec::len),
        "cohort_count": array_at(&manifest, "reference_cohorts").map_or(0, Vec::len),
        "metric_definition_count": array_at(&manifest, "metric_definitions").map_or(0, Vec::len),
        "official_language_count": array_at(&manifest, "official_language_modalities").map_or(0, Vec::len),
        "counterfactual_pair_count": array_at(&manifest, "counterfactual_pair_definitions").map_or(0, Vec::len),
        "negative_controls": negative_controls,
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
    fn checked_in_manifest_and_schema_validate() {
        let (errors, manifest, _) = evaluate(DEFAULT_MANIFEST, DEFAULT_SCHEMA).expect("evaluation");
        assert!(errors.is_empty(), "{errors:?}");
        assert_eq!(array_at(&manifest, "sources").unwrap().len(), 6);
        assert_eq!(array_at(&manifest, "reference_cohorts").unwrap().len(), 5);
        assert_eq!(
            array_at(&manifest, "official_language_modalities")
                .unwrap()
                .len(),
            12
        );
        assert_eq!(
            array_at(&manifest, "counterfactual_pair_definitions")
                .unwrap()
                .len(),
            9
        );
        assert_eq!(array_at(&manifest, "metric_definitions").unwrap().len(), 15);
    }

    #[test]
    fn negative_campaign_rejects_denominator_laundering_and_scope_escalation() {
        let manifest: Value = serde_json::from_str(DEFAULT_MANIFEST).expect("manifest");
        let schema: Value = serde_json::from_str(DEFAULT_SCHEMA).expect("schema");
        assert_eq!(run_self_tests(&manifest, &schema).expect("self tests"), 19);
    }
}
