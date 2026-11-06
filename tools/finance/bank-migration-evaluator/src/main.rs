//! Independent evaluator for the effects-disabled bank migration corpus.
//!
//! This executable checks the corpus against JSON Schema Draft 2020-12 and a
//! separate set of frozen semantic invariants. It never reads bank data, calls
//! a provider, authorizes a cutover, or creates an external financial effect.

use jsonschema::draft202012;
use serde_json::{Value, json};
use std::collections::BTreeSet;
use std::error::Error;
use std::fs;
use std::path::PathBuf;

const DEFAULT_CORPUS: &str =
    include_str!("../../../../mycelix-finance/docs/bank-migration-corpus-v1.json");
const DEFAULT_SCHEMA: &str =
    include_str!("../../../../mycelix-finance/docs/bank-migration-corpus-v1.schema.json");

const EXPECTED_SCHEMA: &str = "mycelix.bank-migration-corpus.v1";
const EXPECTED_CORPUS_ID: &str = "BANK-MIGRATION-SYNTHETIC-V1";

#[derive(Debug, Clone)]
struct Evaluation {
    errors: Vec<String>,
    incomplete: Vec<String>,
    case_count: usize,
}

impl Evaluation {
    fn new() -> Self {
        Self {
            errors: Vec::new(),
            incomplete: Vec::new(),
            case_count: 0,
        }
    }

    fn status(&self) -> &'static str {
        if !self.errors.is_empty() {
            "FAIL"
        } else if !self.incomplete.is_empty() {
            "INCOMPLETE"
        } else {
            "PASS"
        }
    }
}

fn str_at<'a>(value: &'a Value, key: &str) -> Option<&'a str> {
    value.get(key).and_then(Value::as_str)
}

fn bool_at(value: &Value, key: &str) -> Option<bool> {
    value.get(key).and_then(Value::as_bool)
}

fn i64_at(value: &Value, key: &str) -> Option<i64> {
    value.get(key).and_then(Value::as_i64)
}

fn u64_at(value: &Value, key: &str) -> Option<u64> {
    value.get(key).and_then(Value::as_u64)
}

fn array_at<'a>(value: &'a Value, key: &str) -> Option<&'a Vec<Value>> {
    value.get(key).and_then(Value::as_array)
}

fn expected_for(category: &str) -> Option<&'static str> {
    match category {
        "complete_capture" => Some("accept_import_and_reconcile"),
        "exact_duplicate_replay" => Some("idempotent_no_second_effect"),
        "conflicting_source_identity" => Some("quarantine_conflict"),
        "incomplete_pagination" => Some("indeterminate_incomplete_scope"),
        "late_return_after_cutover" => Some("append_correction_and_reconcile"),
        "instrument_scale_substitution" => Some("reject_instrument_substitution"),
        "unknown_lifecycle_status" => Some("quarantine_unknown_status"),
        "false_settlement_promotion" => Some("reject_semantic_promotion"),
        "legal_entity_substitution" => Some("reject_scope_substitution"),
        "pre_effect_rollback" => Some("rollback_permitted"),
        "possible_effect_unknown" => Some("indeterminate_no_blind_retry"),
        "timestamp_ambiguity" => Some("indeterminate_temporal_scope"),
        _ => None,
    }
}

fn check_case(case: &Value, index: usize, errors: &mut Vec<String>) {
    let Some(category) = str_at(case, "category") else {
        errors.push(format!("cases[{index}]: missing category"));
        return;
    };
    let Some(expected) = expected_for(category) else {
        errors.push(format!(
            "cases[{index}]: unknown category {category}; evaluator update required"
        ));
        return;
    };
    if str_at(case, "expected") != Some(expected) {
        errors.push(format!(
            "cases[{index}]: category {category} requires expected disposition {expected}"
        ));
    }

    let Some(input) = case.get("input") else {
        errors.push(format!("cases[{index}]: missing input"));
        return;
    };

    match category {
        "complete_capture" => {
            let declared = u64_at(input, "declared_records");
            let captured = u64_at(input, "captured_records");
            let pages_complete = bool_at(input, "all_pages_present") == Some(true);
            let snapshot_stable = bool_at(input, "source_snapshot_stable") == Some(true);
            let items = array_at(input, "items");
            match (declared, captured, items) {
                (Some(declared), Some(captured), Some(items)) => {
                    if declared != captured || declared != items.len() as u64 {
                        errors.push(format!(
                            "cases[{index}]: complete_capture record census does not reconcile"
                        ));
                    }
                    if !pages_complete || !snapshot_stable {
                        errors.push(format!(
                            "cases[{index}]: complete_capture requires every page and a stable source snapshot"
                        ));
                    }
                    let mut source_ids = BTreeSet::new();
                    let mut currency: Option<&str> = None;
                    for (item_index, item) in items.iter().enumerate() {
                        let Some(source_id) = str_at(item, "source_id") else {
                            errors.push(format!(
                                "cases[{index}].items[{item_index}]: missing source_id"
                            ));
                            continue;
                        };
                        if !source_ids.insert(source_id) {
                            errors.push(format!(
                                "cases[{index}].items[{item_index}]: duplicate source_id {source_id}"
                            ));
                        }
                        if i64_at(item, "amount_minor").is_none() {
                            errors.push(format!(
                                "cases[{index}].items[{item_index}]: amount_minor must be an exact signed integer"
                            ));
                        }
                        if str_at(item, "revision").is_none()
                            || str_at(item, "digest").is_none()
                            || !str_at(item, "digest").unwrap_or_default().starts_with("sha256:")
                        {
                            errors.push(format!(
                                "cases[{index}].items[{item_index}]: source revision and sha256 digest are required"
                            ));
                        }
                        match str_at(item, "currency") {
                            Some(value) if currency.is_none() => currency = Some(value),
                            Some(value) if currency == Some(value) => {}
                            Some(_) => errors.push(format!(
                                "cases[{index}]: aggregate may not mix currencies without an explicit valuation profile"
                            )),
                            None => errors.push(format!(
                                "cases[{index}].items[{item_index}]: missing currency"
                            )),
                        }
                    }
                }
                _ => errors.push(format!(
                    "cases[{index}]: complete_capture requires integer census fields and an items array"
                )),
            }
        }
        "exact_duplicate_replay" => {
            if u64_at(input, "deliveries").unwrap_or(0) < 2
                || bool_at(input, "payload_fingerprint_equal") != Some(true)
                || str_at(input, "source_id").is_none()
                || str_at(input, "revision").is_none()
                || str_at(input, "digest").is_none()
                || str_at(input, "idempotency_key").is_none()
            {
                errors.push(format!(
                    "cases[{index}]: exact replay requires same identity, key, fingerprint and repeated delivery"
                ));
            }
        }
        "conflicting_source_identity" => {
            let different = str_at(input, "first_digest") != str_at(input, "second_digest");
            if !different
                || str_at(input, "source_id").is_none()
                || str_at(input, "revision").is_none()
                || str_at(input, "idempotency_key").is_none()
                || array_at(input, "amounts_minor").is_none_or(|values| {
                    values.len() != 2
                        || values[0].as_i64().is_none()
                        || values[1].as_i64().is_none()
                })
            {
                errors.push(format!(
                    "cases[{index}]: conflicting identity requires one key and two distinct fingerprints"
                ));
            }
        }
        "incomplete_pagination" => {
            let incomplete = u64_at(input, "captured_records")
                .zip(u64_at(input, "declared_records"))
                .is_some_and(|(captured, declared)| captured < declared)
                || u64_at(input, "observed_pages")
                    .zip(u64_at(input, "expected_pages"))
                    .is_some_and(|(observed, expected)| observed < expected);
            if !incomplete || bool_at(input, "source_snapshot_stable") != Some(true) {
                errors.push(format!(
                    "cases[{index}]: incomplete_pagination must actually omit records/pages"
                ));
            }
        }
        "late_return_after_cutover" => {
            if bool_at(input, "original_event_mutable") != Some(false)
                || str_at(input, "original_source_id").is_none()
                || str_at(input, "late_event_id").is_none()
                || str_at(input, "late_event_type") != Some("return")
                || i64_at(input, "amount_minor").is_none()
            {
                errors.push(format!(
                    "cases[{index}]: late return must preserve the original and identify a typed successor event"
                ));
            }
        }
        "instrument_scale_substitution" => {
            if str_at(input, "source_instrument") == str_at(input, "target_instrument")
                || bool_at(input, "fx_quote_present") != Some(false)
                || str_at(input, "source_amount").is_none()
                || u64_at(input, "source_scale").is_none()
                || u64_at(input, "target_amount_minor").is_none()
                || u64_at(input, "target_scale").is_none()
            {
                errors.push(format!(
                    "cases[{index}]: instrument substitution control must distinguish instrument from scale and have no FX quote"
                ));
            }
        }
        "unknown_lifecycle_status" => {
            let unknown = str_at(input, "source_status");
            let known = array_at(input, "known_statuses");
            if unknown.is_none()
                || known.is_none_or(|values| values.iter().any(|value| value.as_str() == unknown))
            {
                errors.push(format!(
                    "cases[{index}]: source lifecycle state must be unknown to the pinned map"
                ));
            }
        }
        "false_settlement_promotion" => {
            if str_at(input, "source_status") != Some("accepted")
                || str_at(input, "target_status") != Some("final")
                || bool_at(input, "settlement_evidence_present") != Some(false)
            {
                errors.push(format!(
                    "cases[{index}]: false finality control must promote accepted to final without evidence"
                ));
            }
        }
        "legal_entity_substitution" => {
            if str_at(input, "source_entity_token") == str_at(input, "target_entity_token")
                || str_at(input, "source_account_token").is_none()
                || str_at(input, "target_account_token").is_none()
            {
                errors.push(format!(
                    "cases[{index}]: scope substitution control must contain distinct legal-entity tokens"
                ));
            }
        }
        "pre_effect_rollback" => {
            if bool_at(input, "external_effect_possible") != Some(false)
                || bool_at(input, "all_effects_census_complete") != Some(true)
                || str_at(input, "source_snapshot_digest").is_none()
                || str_at(input, "target_snapshot_digest").is_none()
            {
                errors.push(format!(
                    "cases[{index}]: rollback may be considered only before effects, with a complete census and exact snapshots"
                ));
            }
        }
        "possible_effect_unknown" => {
            if bool_at(input, "external_effect_possible") != Some(true)
                || str_at(input, "provider_response") != Some("timeout_unknown")
                || bool_at(input, "provider_query_available") != Some(false)
                || str_at(input, "original_effect_id").is_none()
            {
                errors.push(format!(
                    "cases[{index}]: ambiguous external effect must preserve original identity and block blind retry"
                ));
            }
        }
        "timestamp_ambiguity" => {
            let effective = str_at(input, "source_effective_time");
            let timezone_unknown = str_at(input, "source_timezone") == Some("unknown");
            let ambiguous = bool_at(input, "dst_ambiguity") == Some(true);
            if effective.is_none()
                || !timezone_unknown
                || !ambiguous
                || str_at(input, "recorded_at").is_none()
            {
                errors.push(format!(
                    "cases[{index}]: temporal ambiguity must preserve raw effective/recorded time and unknown timezone"
                ));
            }
        }
        _ => unreachable!("all admitted cases are matched above"),
    }

    if array_at(case, "controls").is_none_or(|controls| controls.is_empty()) {
        errors.push(format!(
            "cases[{index}]: controls must be a non-empty array"
        ));
    }
}

fn evaluate_semantics(corpus: &Value) -> Evaluation {
    let mut evaluation = Evaluation::new();
    if str_at(corpus, "schema_version") != Some(EXPECTED_SCHEMA) {
        evaluation
            .errors
            .push("schema_version: unsupported corpus version".into());
    }
    if str_at(corpus, "corpus_id") != Some(EXPECTED_CORPUS_ID) {
        evaluation
            .errors
            .push("corpus_id: unexpected corpus identity".into());
    }
    if str_at(corpus, "status") != Some("synthetic_reference_only") {
        evaluation
            .errors
            .push("status: corpus must remain synthetic_reference_only".into());
    }
    if str_at(corpus, "currency_representation") != Some("integer_minor_units") {
        evaluation
            .errors
            .push("currency_representation: expected integer_minor_units".into());
    }

    let Some(cases) = array_at(corpus, "cases") else {
        evaluation
            .errors
            .push("cases: missing scenario array".into());
        return evaluation;
    };
    evaluation.case_count = cases.len();
    if cases.len() != 12 {
        evaluation.errors.push(format!(
            "cases: this frozen v1 evaluator expects 12 cases; found {}",
            cases.len()
        ));
    }

    let mut ids = BTreeSet::new();
    let mut categories = BTreeSet::new();
    for (index, case) in cases.iter().enumerate() {
        match str_at(case, "id") {
            Some(id) if ids.insert(id.to_owned()) => {}
            Some(id) => evaluation
                .errors
                .push(format!("cases[{index}]: duplicate case ID {id}")),
            None => evaluation
                .errors
                .push(format!("cases[{index}]: missing case ID")),
        }
        if let Some(category) = str_at(case, "category")
            && !categories.insert(category.to_owned())
        {
            evaluation.errors.push(format!(
                "cases[{index}]: duplicate case category {category}"
            ));
        }
        check_case(case, index, &mut evaluation.errors);
    }

    let expected_categories: BTreeSet<String> = [
        "complete_capture",
        "exact_duplicate_replay",
        "conflicting_source_identity",
        "incomplete_pagination",
        "late_return_after_cutover",
        "instrument_scale_substitution",
        "unknown_lifecycle_status",
        "false_settlement_promotion",
        "legal_entity_substitution",
        "pre_effect_rollback",
        "possible_effect_unknown",
        "timestamp_ambiguity",
    ]
    .into_iter()
    .map(str::to_owned)
    .collect();
    if categories != expected_categories {
        evaluation.errors.push(
            "categories: the frozen v1 corpus must contain each of the 12 required categories exactly once"
                .into(),
        );
    }

    evaluation
}

fn validate_schema(schema: &Value, corpus: &Value) -> Vec<String> {
    let mut errors = Vec::new();
    if !jsonschema::meta::is_valid(schema) {
        errors.push("schema is invalid against its Draft 2020-12 meta-schema".into());
        return errors;
    }
    let validator = match draft202012::options()
        .should_validate_formats(true)
        .build(schema)
    {
        Ok(validator) => validator,
        Err(error) => {
            errors.push(format!("could not compile Draft 2020-12 schema: {error}"));
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

fn evaluate(corpus_text: &str, schema_text: &str) -> Result<(Evaluation, Value, Value), String> {
    let corpus: Value = serde_json::from_str(corpus_text)
        .map_err(|error| format!("corpus JSON parse error: {error}"))?;
    let schema: Value = serde_json::from_str(schema_text)
        .map_err(|error| format!("schema JSON parse error: {error}"))?;
    let mut evaluation = evaluate_semantics(&corpus);
    evaluation.errors.extend(validate_schema(&schema, &corpus));
    Ok((evaluation, corpus, schema))
}

fn mutation_must_fail(
    name: &str,
    corpus: &Value,
    schema: &Value,
    mutation: impl FnOnce(&mut Value),
) -> Result<(), String> {
    let mut changed = corpus.clone();
    mutation(&mut changed);
    let semantic = evaluate_semantics(&changed);
    let schema_errors = validate_schema(schema, &changed);
    if semantic.errors.is_empty() && semantic.incomplete.is_empty() && schema_errors.is_empty() {
        return Err(format!("negative mutation was not rejected: {name}"));
    }
    Ok(())
}

fn run_negative_mutations(corpus: &Value, schema: &Value) -> Result<usize, String> {
    let semantic = evaluate_semantics(corpus);
    let schema_errors = validate_schema(schema, corpus);
    if !semantic.errors.is_empty() || !semantic.incomplete.is_empty() || !schema_errors.is_empty() {
        return Err("baseline corpus must pass before mutation tests".into());
    }

    let mut count = 0usize;
    mutation_must_fail("missing source record", corpus, schema, |value| {
        value["cases"][0]["input"]["captured_records"] = json!(1);
    })?;
    count += 1;
    mutation_must_fail("missing pagination page", corpus, schema, |value| {
        value["cases"][0]["input"]["all_pages_present"] = json!(false);
    })?;
    count += 1;
    mutation_must_fail(
        "duplicate source record identity",
        corpus,
        schema,
        |value| {
            let first_source_id = value["cases"][0]["input"]["items"][0]["source_id"].clone();
            value["cases"][0]["input"]["items"][1]["source_id"] = first_source_id;
        },
    )?;
    count += 1;
    mutation_must_fail(
        "exact replay with changed fingerprint",
        corpus,
        schema,
        |value| {
            value["cases"][1]["input"]["payload_fingerprint_equal"] = json!(false);
        },
    )?;
    count += 1;
    mutation_must_fail(
        "conflicting identity with same digest",
        corpus,
        schema,
        |value| {
            let first_digest = value["cases"][2]["input"]["first_digest"].clone();
            value["cases"][2]["input"]["second_digest"] = first_digest;
        },
    )?;
    count += 1;
    mutation_must_fail(
        "instrument substitution silently accepted",
        corpus,
        schema,
        |value| {
            value["cases"][5]["expected"] = json!("accept_import_and_reconcile");
        },
    )?;
    count += 1;
    mutation_must_fail(
        "unknown lifecycle status mapped as booked",
        corpus,
        schema,
        |value| {
            value["cases"][6]["input"]["source_status"] = json!("booked");
        },
    )?;
    count += 1;
    mutation_must_fail(
        "accepted promoted to final without evidence",
        corpus,
        schema,
        |value| {
            value["cases"][7]["expected"] = json!("accept_import_and_reconcile");
        },
    )?;
    count += 1;
    mutation_must_fail(
        "legal entity substitution accepted",
        corpus,
        schema,
        |value| {
            value["cases"][8]["expected"] = json!("accept_import_and_reconcile");
        },
    )?;
    count += 1;
    mutation_must_fail(
        "rollback despite possible external effect",
        corpus,
        schema,
        |value| {
            value["cases"][9]["input"]["external_effect_possible"] = json!(true);
        },
    )?;
    count += 1;
    mutation_must_fail(
        "unknown provider effect marked retry-safe",
        corpus,
        schema,
        |value| {
            value["cases"][10]["expected"] = json!("rollback_permitted");
        },
    )?;
    count += 1;
    mutation_must_fail(
        "ambiguous local time coerced to resolved",
        corpus,
        schema,
        |value| {
            value["cases"][11]["expected"] = json!("accept_import_and_reconcile");
        },
    )?;
    count += 1;

    Ok(count)
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
                    std::io::Error::new(std::io::ErrorKind::InvalidInput, "--corpus needs a path")
                })?);
                corpus_text = fs::read_to_string(path)?;
            }
            "--schema" => {
                let path = PathBuf::from(args.next().ok_or_else(|| {
                    std::io::Error::new(std::io::ErrorKind::InvalidInput, "--schema needs a path")
                })?);
                schema_text = fs::read_to_string(path)?;
            }
            "--self-test" => self_test = true,
            "--help" | "-h" => {
                println!("bank-migration-evaluator [--corpus PATH] [--schema PATH] [--self-test]");
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

    let (evaluation, corpus, schema) = evaluate(&corpus_text, &schema_text)
        .map_err(|message| std::io::Error::new(std::io::ErrorKind::InvalidData, message))?;
    let mutations = if self_test {
        run_negative_mutations(&corpus, &schema)
            .map_err(|message| std::io::Error::new(std::io::ErrorKind::InvalidData, message))?
    } else {
        0
    };

    let report = json!({
        "evaluator": "mycelix.bank-migration-evaluator.v1",
        "status": evaluation.status(),
        "claim_ceiling": "synthetic corpus schema, identity, replay, scope, temporal and disposition validation only",
        "external_effects_enabled": false,
        "counts": {
            "cases": evaluation.case_count,
            "negative_mutations": mutations
        },
        "errors": evaluation.errors,
        "incomplete": evaluation.incomplete
    });
    println!("{}", serde_json::to_string_pretty(&report)?);

    if evaluation.status() != "PASS" {
        std::process::exit(1);
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn baseline_contract_validates() {
        let (evaluation, _, _) = evaluate(DEFAULT_CORPUS, DEFAULT_SCHEMA).expect("parse corpus");
        assert_eq!(evaluation.status(), "PASS", "{evaluation:?}");
    }

    #[test]
    fn all_negative_mutations_are_rejected() {
        let corpus: Value = serde_json::from_str(DEFAULT_CORPUS).expect("parse corpus");
        let schema: Value = serde_json::from_str(DEFAULT_SCHEMA).expect("parse schema");
        assert_eq!(
            run_negative_mutations(&corpus, &schema).expect("mutations"),
            12
        );
    }

    #[test]
    fn category_mapping_is_closed() {
        assert!(expected_for("complete_capture").is_some());
        assert!(expected_for("possible_effect_unknown").is_some());
        assert!(expected_for("invented_status").is_none());
    }

    #[test]
    fn integer_minor_units_reject_fractional_json_numbers() {
        assert_eq!(json!(1234).as_i64(), Some(1234));
        assert_eq!(json!(12.34).as_i64(), None);
        assert_eq!(json!(-250).as_i64(), Some(-250));
    }
}
