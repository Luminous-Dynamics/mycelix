#![forbid(unsafe_code)]

//! Effects-disabled, fail-closed evaluator for South African payment recourse cases.
//! A request, trace, acknowledgement, location or hold is never counted as returned value.
//! Only independently evidenced and reconciled return events increase recovered value.

use jsonschema::draft202012;
use serde_json::{Value, json};
use std::collections::BTreeSet;
use std::error::Error;
use std::fs;
use std::path::PathBuf;

const DEFAULT_CORPUS: &str =
    include_str!("../../../../mycelix-finance/docs/sa-payment-recourse-corpus-v1.json");
const DEFAULT_SCHEMA: &str =
    include_str!("../../../../mycelix-finance/docs/sa-payment-recourse-corpus-v1.schema.json");
const EXPECTED_SCHEMA: &str = "mycelix.sa-payment-recourse/v1";
const EXPECTED_CORPUS: &str = "ZA-SA-PAYMENT-RECOURSE-SYNTHETIC-V1";

#[derive(Debug, Default)]
struct Evaluation {
    errors: Vec<String>,
    source_count: usize,
    profile_count: usize,
    scenario_count: usize,
    verified_return_count: usize,
    returned_minor_units: u64,
}

impl Evaluation {
    fn status(&self) -> &'static str {
        if self.errors.is_empty() {
            "PASS"
        } else {
            "FAIL"
        }
    }
}

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

fn schema_errors(schema: &Value, corpus: &Value) -> Vec<String> {
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
    for error in validator.iter_errors(corpus) {
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

fn evidence_contains(event: &Value, expected: &str) -> bool {
    array_at(event, "evidence_types")
        .is_some_and(|items| items.iter().any(|item| item.as_str() == Some(expected)))
}

fn semantic_evaluation(corpus: &Value) -> Evaluation {
    let mut out = Evaluation::default();

    let Some(as_of) = str_at(corpus, "as_of") else {
        out.errors.push("as_of: missing pinned date".into());
        return out;
    };
    if !valid_date(as_of) {
        out.errors.push(format!("as_of: invalid date {as_of}"));
    }
    if str_at(corpus, "schema") != Some(EXPECTED_SCHEMA) {
        out.errors.push("schema: unexpected schema identity".into());
    }
    if str_at(corpus, "corpus_id") != Some(EXPECTED_CORPUS) {
        out.errors
            .push("corpus_id: unexpected corpus identity".into());
    }
    if str_at(corpus, "status") != Some("synthetic_reference_only") {
        out.errors
            .push("status: corpus must remain synthetic_reference_only".into());
    }
    if str_at(corpus, "jurisdiction") != Some("ZA") {
        out.errors.push("jurisdiction: expected ZA".into());
    }
    if corpus
        .get("external_effects_enabled")
        .and_then(Value::as_bool)
        != Some(false)
    {
        out.errors
            .push("external_effects_enabled: must be false".into());
    }

    let sources = array_at(corpus, "source_register")
        .cloned()
        .unwrap_or_default();
    let profiles = array_at(corpus, "profiles").cloned().unwrap_or_default();
    let scenarios = array_at(corpus, "scenarios").cloned().unwrap_or_default();
    out.source_count = sources.len();
    out.profile_count = profiles.len();
    out.scenario_count = scenarios.len();

    check_unique(&sources, "id", "source_register", &mut out.errors);
    check_unique(&profiles, "profile_id", "profiles", &mut out.errors);
    check_unique(&scenarios, "scenario_id", "scenarios", &mut out.errors);

    let source_ids: BTreeSet<String> = sources
        .iter()
        .filter_map(|item| str_at(item, "id").map(str::to_owned))
        .collect();
    let profile_ids: BTreeSet<String> = profiles
        .iter()
        .filter_map(|item| str_at(item, "profile_id").map(str::to_owned))
        .collect();

    if sources.is_empty() || profiles.is_empty() || scenarios.is_empty() {
        out.errors
            .push("source_register, profiles and scenarios must be non-empty".into());
    }

    for (index, source) in sources.iter().enumerate() {
        let Some(observed) = str_at(source, "observed_on") else {
            out.errors
                .push(format!("source_register[{index}].observed_on: missing"));
            continue;
        };
        if !date_le(observed, as_of) {
            out.errors.push(format!(
                "source_register[{index}].observed_on: invalid or after as_of"
            ));
        }
    }

    for (index, profile) in profiles.iter().enumerate() {
        let observed = str_at(profile, "observed_on").unwrap_or("");
        if !date_le(observed, as_of) {
            out.errors.push(format!(
                "profiles[{index}].observed_on: invalid or after as_of"
            ));
        }
        let from = profile.get("effective_from").and_then(Value::as_str);
        let to = profile.get("effective_to").and_then(Value::as_str);
        if let (Some(start), Some(end)) = (from, to) {
            if !date_le(start, end) {
                out.errors.push(format!(
                    "profiles[{index}]: effective_to precedes effective_from"
                ));
            }
        }
        let refs = array_at(profile, "source_refs")
            .cloned()
            .unwrap_or_default();
        if refs.is_empty() {
            out.errors.push(format!(
                "profiles[{index}].source_refs: empty source evidence"
            ));
        }
        for reference in refs {
            if !reference.as_str().is_some_and(|id| source_ids.contains(id)) {
                out.errors
                    .push(format!("profiles[{index}].source_refs: unresolved source"));
            }
        }

        let capabilities = array_at(profile, "remedy_capabilities")
            .cloned()
            .unwrap_or_default();
        for (cap_index, capability) in capabilities.iter().enumerate() {
            let window_state = str_at(capability, "time_window_state").unwrap_or("");
            let window = capability.get("window_seconds");
            match window_state {
                "unknown" | "not_applicable" if !window.is_none_or(Value::is_null) => {
                    out.errors.push(format!("profiles[{index}].remedy_capabilities[{cap_index}]: unknown/not-applicable window must be null"));
                }
                "known" if u64_at(capability, "window_seconds").is_none_or(|value| value == 0) => {
                    out.errors.push(format!("profiles[{index}].remedy_capabilities[{cap_index}]: known window requires positive seconds"));
                }
                _ => {}
            }
            let cap_refs = array_at(capability, "source_refs")
                .cloned()
                .unwrap_or_default();
            if cap_refs.is_empty() {
                out.errors.push(format!(
                    "profiles[{index}].remedy_capabilities[{cap_index}].source_refs: empty"
                ));
            }
            for reference in cap_refs {
                if !reference.as_str().is_some_and(|id| source_ids.contains(id)) {
                    out.errors.push(format!(
                        "profiles[{index}].remedy_capabilities[{cap_index}].source_refs: unresolved source"
                    ));
                }
            }
        }
    }

    for (case_index, case) in scenarios.iter().enumerate() {
        let case_id = str_at(case, "scenario_id").unwrap_or("<missing>");
        let profile_id = str_at(case, "profile_id").unwrap_or("");
        if !profile_ids.contains(profile_id) {
            out.errors.push(format!(
                "scenarios[{case_index}]: unresolved profile {profile_id}"
            ));
        }
        let claim = u64_at(case, "claim_amount_minor_units").unwrap_or(0);
        let declared_return = u64_at(case, "actual_returned_minor_units").unwrap_or(u64::MAX);
        if claim == 0 || declared_return > claim {
            out.errors.push(format!(
                "scenarios[{case_index}]: invalid claim/returned amount"
            ));
        }

        let events = array_at(case, "events").cloned().unwrap_or_default();
        let mut event_ids = BTreeSet::new();
        let mut return_refs = BTreeSet::new();
        let mut previous_sequence = 0_u64;
        let mut return_total = 0_u64;
        let mut has_recall = false;
        let mut has_location = false;
        let mut has_hold = false;
        let mut has_timeout = false;
        let mut has_case_closed = false;

        for (event_index, event) in events.iter().enumerate() {
            let path = format!("scenarios[{case_index}].events[{event_index}]");
            let event_id = str_at(event, "event_id").unwrap_or("<missing>");
            if !event_ids.insert(event_id.to_owned()) {
                out.errors
                    .push(format!("{path}: duplicate event_id {event_id}"));
            }
            let sequence = u64_at(event, "sequence").unwrap_or(0);
            if sequence <= previous_sequence {
                out.errors.push(format!(
                    "{path}: event sequence must be strictly increasing"
                ));
            }
            previous_sequence = sequence;

            let event_type = str_at(event, "event_type").unwrap_or("");
            let actor = str_at(event, "actor_role").unwrap_or("");
            let authority = str_at(event, "authority_state").unwrap_or("");
            let amount = u64_at(event, "amount_minor_units").unwrap_or(u64::MAX);

            has_recall |= event_type == "recall_requested";
            has_location |= event_type == "funds_located";
            has_hold |= event_type == "funds_held";
            has_timeout |= event_type == "provider_timeout";
            has_case_closed |= event_type == "case_closed";

            if event_type == "funds_returned" {
                if amount == 0
                    || authority != "valid"
                    || !matches!(actor, "beneficiary_psp" | "recipient")
                {
                    out.errors.push(format!("{path}: return requires positive amount, valid authority and beneficiary PSP/recipient actor"));
                }
                if event.get("reconciled").and_then(Value::as_bool) != Some(true)
                    || !evidence_contains(event, "independent_return_receipt")
                    || !evidence_contains(event, "independent_ledger_reconciliation")
                {
                    out.errors.push(format!("{path}: return requires independent receipt and ledger reconciliation"));
                }
                let Some(reference) = str_at(event, "return_reference") else {
                    out.errors.push(format!("{path}: return_reference missing"));
                    continue;
                };
                if !return_refs.insert(reference.to_owned()) {
                    out.errors.push(format!("{path}: duplicate return_reference {reference}"));
                }
                return_total = match return_total.checked_add(amount) {
                    Some(total) => total,
                    None => {
                        out.errors.push(format!("{path}: return amount overflow"));
                        u64::MAX
                    }
                };
                out.verified_return_count += 1;
            } else if amount != 0 {
                out.errors.push(format!("{path}: non-return event cannot contribute returned amount"));
            }

            if authority == "invalid" && !matches!(
                event_type,
                "unauthorised_submission_blocked"
                    | "data_disclosure_blocked"
                    | "case_closure_attempt_rejected"
                    | "remedy_misclassification_blocked"
            ) {
                out.errors.push(format!("{path}: invalid authority cannot create an accepted case/economic event"));
            }
            if event_type == "funds_held" && (authority != "valid" || !evidence_contains(event, "hold_confirmation")) {
                out.errors.push(format!("{path}: hold requires valid authority and hold confirmation"));
            }
        }

        if return_total != declared_return {
            out.errors.push(format!("scenarios[{case_index}]: declared returned amount differs from verified return events"));
        }
        if return_total > claim {
            out.errors.push(format!("scenarios[{case_index}]: returned amount exceeds claim"));
        }
        if return_total > 0 && !events.iter().any(|event| str_at(event, "event_type") == Some("funds_returned")) {
            out.errors.push(format!("scenarios[{case_index}]: returned amount lacks return event"));
        }
        out.returned_minor_units = match out.returned_minor_units.checked_add(return_total) {
            Some(total) => total,
            None => {
                out.errors.push(format!("scenarios[{case_index}]: corpus returned-value total overflow"));
                u64::MAX
            }
        };

        let computed_effect = if return_total == claim && claim > 0 {
            "full_return"
        } else if return_total > 0 {
            "partial_return"
        } else if has_hold {
            "funds_held_not_returned"
        } else if has_location {
            "funds_located_not_returned"
        } else if has_recall || events.iter().any(|event| str_at(event, "event_type") == Some("return_instruction_observed")) {
            "request_only"
        } else if events.iter().any(|event| {
            matches!(
                str_at(event, "event_type"),
                Some("late_success_observed" | "recipient_availability_verified")
            )
        }) {
            "no_recovery"
        } else if has_timeout {
            "unresolved_possible_effect"
        } else {
            "no_recovery"
        };
        if str_at(case, "expected_effect") != Some(computed_effect) {
            out.errors.push(format!("scenarios[{case_index}]: expected_effect overstates or misclassifies the observed case events"));
        }

        let disposition = str_at(case, "case_disposition").unwrap_or("");
        if disposition == "open" && has_case_closed {
            out.errors.push(format!("scenarios[{case_index}]: open case contains case_closed event"));
        }
        if disposition != "open" && !has_case_closed {
            out.errors.push(format!("scenarios[{case_index}]: closed case lacks case_closed event"));
        }
        if matches!(disposition, "closed_with_return") && return_total == 0 {
            out.errors.push(format!("scenarios[{case_index}]: closed_with_return requires returned value"));
        }
        if matches!(disposition, "closed_without_return") && return_total != 0 {
            out.errors.push(format!("scenarios[{case_index}]: closed_without_return conflicts with returned value"));
        }
        if let Some(close_event) = events.iter().find(|event| str_at(event, "event_type") == Some("case_closed")) {
            if str_at(close_event, "closure_disposition") != Some(disposition)
                || str_at(close_event, "authority_state") != Some("valid")
                || !evidence_contains(close_event, "case_disposition_notice")
            {
                out.errors.push(format!("scenarios[{case_index}]: case closure lacks matching disposition, authority or evidence"));
            }
        }

        if computed_effect == "full_return" && str_at(case, "expected_disposition") != Some("record_successor_return") {
            out.errors.push(format!("scenarios[{case_index}]: full return must be recorded as successor return"));
        }
        if computed_effect == "partial_return" && str_at(case, "expected_disposition") != Some("retain_residual_loss") {
            out.errors.push(format!("scenarios[{case_index}]: partial return must preserve residual loss"));
        }
        if return_total == 0 && str_at(case, "expected_disposition") == Some("record_successor_return") {
            out.errors.push(format!("scenarios[{case_index}]: cannot record successor return without returned value"));
        }

        if case_id.is_empty() {
            out.errors.push(format!("scenarios[{case_index}]: missing scenario identity"));
        }
    }

    out
}

fn evaluate(corpus_text: &str, schema_text: &str) -> Result<(Evaluation, Value, Value), String> {
    let corpus: Value = serde_json::from_str(corpus_text)
        .map_err(|error| format!("corpus JSON parse error: {error}"))?;
    let schema: Value = serde_json::from_str(schema_text)
        .map_err(|error| format!("schema JSON parse error: {error}"))?;
    let mut result = semantic_evaluation(&corpus);
    result.errors.extend(schema_errors(&schema, &corpus));
    Ok((result, corpus, schema))
}

fn mutate_and_expect_failure(
    label: &str,
    corpus: &Value,
    schema: &Value,
    mutation: impl FnOnce(&mut Value),
) -> Result<(), String> {
    let mut changed = corpus.clone();
    mutation(&mut changed);
    let evaluation = semantic_evaluation(&changed);
    let structural = schema_errors(schema, &changed);
    if evaluation.errors.is_empty() && structural.is_empty() {
        return Err(format!("negative control was not rejected: {label}"));
    }
    Ok(())
}

fn run_self_tests(corpus: &Value, schema: &Value) -> Result<usize, String> {
    let baseline = semantic_evaluation(corpus);
    if !baseline.errors.is_empty() || !schema_errors(schema, corpus).is_empty() {
        return Err(format!(
            "baseline corpus must pass before mutations: {:?}",
            baseline.errors
        ));
    }
    let mut count = 0usize;

    mutate_and_expect_failure("external effects enabled", corpus, schema, |v| {
        v["external_effects_enabled"] = json!(true);
    })?;
    count += 1;

    mutate_and_expect_failure("request promoted to full recovery", corpus, schema, |v| {
        v["scenarios"][0]["expected_effect"] = json!("full_return");
        v["scenarios"][0]["expected_disposition"] = json!("record_successor_return");
    })?;
    count += 1;

    mutate_and_expect_failure(
        "funds held promoted to full recovery",
        corpus,
        schema,
        |v| {
            v["scenarios"][4]["expected_effect"] = json!("full_return");
            v["scenarios"][4]["expected_disposition"] = json!("record_successor_return");
        },
    )?;
    count += 1;

    mutate_and_expect_failure("return amount greater than claim", corpus, schema, |v| {
        v["scenarios"][5]["events"][4]["amount_minor_units"] = json!(20_000);
        v["scenarios"][5]["actual_returned_minor_units"] = json!(20_000);
    })?;
    count += 1;

    mutate_and_expect_failure("return without independent reconciliation", corpus, schema, |v| {
        v["scenarios"][6]["events"][4]["reconciled"] = json!(false);
    })?;
    count += 1;

    mutate_and_expect_failure("duplicate return reference", corpus, schema, |v| {
        let event = v["scenarios"][6]["events"][4].clone();
        v["scenarios"][6]["events"]
            .as_array_mut()
            .unwrap()
            .insert(5, event);
    })?;
    count += 1;

    mutate_and_expect_failure("unauthorised actor returns funds", corpus, schema, |v| {
        v["scenarios"][6]["events"][4]["actor_role"] = json!("third_party");
    })?;
    count += 1;

    mutate_and_expect_failure("unresolved source reference", corpus, schema, |v| {
        v["profiles"][0]["source_refs"] = json!(["MISSING_SOURCE"]);
    })?;
    count += 1;

    mutate_and_expect_failure("future observed profile", corpus, schema, |v| {
        v["profiles"][0]["observed_on"] = json!("2027-01-01");
    })?;
    count += 1;

    mutate_and_expect_failure("duplicate case identity", corpus, schema, |v| {
        v["scenarios"][1]["scenario_id"] = v["scenarios"][0]["scenario_id"].clone();
    })?;
    count += 1;

    mutate_and_expect_failure("closed case without closure event", corpus, schema, |v| {
        v["scenarios"][5]["events"].as_array_mut().unwrap().pop();
    })?;
    count += 1;

    mutate_and_expect_failure("unknown deadline forged as known", corpus, schema, |v| {
        v["profiles"][0]["remedy_capabilities"][1]["time_window_state"] = json!("known");
        v["profiles"][0]["remedy_capabilities"][1]["window_seconds"] = Value::Null;
    })?;
    count += 1;

    mutate_and_expect_failure(
        "recipient return missing independent evidence",
        corpus,
        schema,
        |v| {
            v["scenarios"][6]["events"][4]["evidence_types"] = json!(["provider_callback"]);
        },
    )?;
    count += 1;

    mutate_and_expect_failure(
        "duplicate economic identity not reconciled",
        corpus,
        schema,
        |v| {
            v["scenarios"][6]["actual_returned_minor_units"] = json!(0);
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
                    "sa-payment-recourse-evaluator [--corpus PATH] [--schema PATH] [--self-test]"
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

    let (evaluation, corpus, schema) = evaluate(&corpus_text, &schema_text)
        .map_err(|message| std::io::Error::new(std::io::ErrorKind::InvalidData, message))?;
    let negative_controls = if self_test {
        run_self_tests(&corpus, &schema)?
    } else {
        0
    };
    let report = json!({
        "evaluator": "mycelix.sa-payment-recourse-evaluator.v1",
        "status": evaluation.status(),
        "external_effects_enabled": false,
        "claim_ceiling": "synthetic source/profile/schema/recourse event validation only",
        "counts": {
            "sources": evaluation.source_count,
            "profiles": evaluation.profile_count,
            "scenarios": evaluation.scenario_count,
            "verified_return_events": evaluation.verified_return_count,
            "returned_minor_units": evaluation.returned_minor_units,
            "negative_controls": negative_controls
        },
        "errors": evaluation.errors
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
    fn checked_in_schema_and_corpus_validate() {
        let (evaluation, _, _) = evaluate(DEFAULT_CORPUS, DEFAULT_SCHEMA).expect("evaluation");
        assert_eq!(evaluation.status(), "PASS", "{:?}", evaluation.errors);
        assert_eq!(evaluation.source_count, 5);
        assert_eq!(evaluation.profile_count, 6);
        assert_eq!(evaluation.scenario_count, 28);
        assert_eq!(evaluation.verified_return_count, 3);
        assert_eq!(evaluation.returned_minor_units, 17_000);
    }

    #[test]
    fn negative_mutation_campaign_rejects_false_recovery_claims() {
        let corpus: Value = serde_json::from_str(DEFAULT_CORPUS).expect("corpus");
        let schema: Value = serde_json::from_str(DEFAULT_SCHEMA).expect("schema");
        assert_eq!(run_self_tests(&corpus, &schema).expect("self-tests"), 14);
    }

    #[test]
    fn calendar_dates_are_validated_without_runtime_clock() {
        assert!(valid_date("2024-02-29"));
        assert!(!valid_date("2026-02-29"));
        assert!(!date_le("2027-01-01", "2026-11-04"));
    }
}
