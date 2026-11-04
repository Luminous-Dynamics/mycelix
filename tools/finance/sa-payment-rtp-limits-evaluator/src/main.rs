#![forbid(unsafe_code)]

//! Effects-disabled, source-bound evaluator for participant-scoped RTP limits.
//! Limits are selected by participant, direction, route, counterparty scope and window.
//! Unknown limits are never silently converted to a rejection or to an unlimited rule.

use jsonschema::draft202012;
use serde_json::{Value, json};
use std::collections::{BTreeMap, BTreeSet};
use std::error::Error;
use std::fs;
use std::path::PathBuf;

const DEFAULT_CORPUS: &str =
    include_str!("../../../../mycelix-finance/docs/sa-payment-rtp-limits-corpus-v1.json");
const DEFAULT_SCHEMA: &str =
    include_str!("../../../../mycelix-finance/docs/sa-payment-rtp-limits-corpus-v1.schema.json");
const EXPECTED_SCHEMA: &str = "mycelix.sa-payment-rtp-limits/v1";
const EXPECTED_CORPUS: &str = "ZA-SA-PAYMENT-RTP-LIMIT-FAIRNESS-SYNTHETIC-V1";

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

fn timestamp_epoch_seconds(value: &str) -> Option<i64> {
    if value.len() < 20 || !valid_date(value.get(0..10)?) || value.as_bytes().get(10) != Some(&b'T')
    {
        return None;
    }
    let year = value.get(0..4)?.parse::<i64>().ok()?;
    let month = value.get(5..7)?.parse::<i64>().ok()?;
    let day = value.get(8..10)?.parse::<i64>().ok()?;
    let hour = value.get(11..13)?.parse::<i64>().ok()?;
    let minute = value.get(14..16)?.parse::<i64>().ok()?;
    let second = value.get(17..19)?.parse::<i64>().ok()?;
    if hour > 23 || minute > 59 || second > 60 {
        return None;
    }
    let y = year - if month <= 2 { 1 } else { 0 };
    let era = y.div_euclid(400);
    let year_of_era = y - era * 400;
    let month_prime = month + if month > 2 { -3 } else { 9 };
    let day_of_year = (153 * month_prime + 2) / 5 + day - 1;
    let day_of_era = year_of_era * 365 + year_of_era / 4 - year_of_era / 100 + day_of_year;
    let days = era * 146_097 + day_of_era - 719_468;

    let zone = value.get(19..)?;
    let offset_seconds = if zone == "Z" {
        0
    } else {
        let sign = match zone.as_bytes().first()? {
            b'+' => 1_i64,
            b'-' => -1_i64,
            _ => return None,
        };
        let offset_hour = zone.get(1..3)?.parse::<i64>().ok()?;
        let offset_minute = zone.get(4..6)?.parse::<i64>().ok()?;
        if offset_hour > 23 || offset_minute > 59 || zone.as_bytes().get(3) != Some(&b':') {
            return None;
        }
        sign * (offset_hour * 3_600 + offset_minute * 60)
    };
    Some(days * 86_400 + hour * 3_600 + minute * 60 + second - offset_seconds)
}

fn valid_timestamp(value: &str) -> bool {
    timestamp_epoch_seconds(value).is_some()
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

fn profile_map(corpus: &Value) -> BTreeMap<String, Value> {
    array_at(corpus, "profiles")
        .into_iter()
        .flatten()
        .filter_map(|profile| {
            str_at(profile, "profile_id").map(|id| (id.to_owned(), profile.clone()))
        })
        .collect()
}

fn source_ids(corpus: &Value) -> BTreeSet<String> {
    array_at(corpus, "source_register")
        .into_iter()
        .flatten()
        .filter_map(|source| str_at(source, "id").map(str::to_owned))
        .collect()
}

fn find_count_rule<'a>(profile: &'a Value, rule_id: &str) -> Option<&'a Value> {
    array_at(profile, "request_count_rules")?
        .iter()
        .find(|rule| str_at(rule, "rule_id") == Some(rule_id))
}

fn find_amount_rule<'a>(profile: &'a Value, rule_id: &str) -> Option<&'a Value> {
    array_at(profile, "amount_limit_rules")?
        .iter()
        .find(|rule| str_at(rule, "rule_id") == Some(rule_id))
}

fn receiver_amount_rule<'a>(receiver: &'a Value, route_scope: &str) -> Option<&'a Value> {
    let rules = array_at(receiver, "amount_limit_rules")?;
    rules
        .iter()
        .find(|rule| str_at(rule, "route_scope") == Some(route_scope))
        .or_else(|| {
            rules
                .iter()
                .find(|rule| str_at(rule, "route_scope") == Some("generic"))
        })
}

fn receiver_limit(receiver: &Value, route_scope: &str) -> Option<u64> {
    receiver_amount_rule(receiver, route_scope)
        .and_then(|rule| u64_at(rule, "max_per_request_minor_units"))
}

fn rule_sources_resolve(rule: &Value, sources: &BTreeSet<String>) -> bool {
    array_at(rule, "source_refs").is_some_and(|refs| {
        !refs.is_empty()
            && refs
                .iter()
                .all(|item| item.as_str().is_some_and(|id| sources.contains(id)))
    })
}

fn case_cites_rule(case_refs: &[Value], rule: &Value) -> bool {
    array_at(rule, "source_refs").is_some_and(|rule_refs| {
        !rule_refs.is_empty()
            && rule_refs.iter().all(|reference| {
                reference.as_str().is_some_and(|source_id| {
                    case_refs
                        .iter()
                        .any(|item| item.as_str() == Some(source_id))
                })
            })
    })
}

fn profile_source_refs_resolve(profile: &Value, sources: &BTreeSet<String>) -> bool {
    let refs = array_at(profile, "source_refs");
    match (str_at(profile, "limit_knowledge_state"), refs) {
        (Some("unknown"), Some(items)) => items.is_empty(),
        (Some("unknown"), None) => true,
        (Some("source_backed"), Some(items)) => {
            !items.is_empty()
                && items
                    .iter()
                    .all(|item| item.as_str().is_some_and(|id| sources.contains(id)))
        }
        _ => false,
    }
}

fn window_valid(case: &Value) -> bool {
    let Some(observed) = str_at(case, "observed_at") else {
        return false;
    };
    let Some(start) = str_at(case, "window_start_at") else {
        return false;
    };
    let (Some(observed_seconds), Some(start_seconds)) = (
        timestamp_epoch_seconds(observed),
        timestamp_epoch_seconds(start),
    ) else {
        return false;
    };
    if start_seconds > observed_seconds {
        return false;
    }
    let semantics = str_at(case, "window_semantics").unwrap_or("");
    match semantics {
        "rolling_24h" => observed_seconds - start_seconds <= 86_400,
        "calendar_day" | "day_unspecified" => true,
        _ => false,
    }
}

fn expected_disposition(
    case: &Value,
    profile: &Value,
    profiles: &BTreeMap<String, Value>,
) -> &'static str {
    let limit_state = str_at(profile, "limit_knowledge_state").unwrap_or("unknown");
    let direction = str_at(case, "direction").unwrap_or("");
    let route_scope = str_at(case, "route_scope").unwrap_or("");
    let window = str_at(case, "window_semantics").unwrap_or("");
    let count_before = u64_at(case, "requests_in_window_before").unwrap_or(0);
    let counterparty_before = u64_at(case, "counterparty_requests_in_window_before").unwrap_or(0);
    let amount_before = u64_at(case, "aggregate_amount_before_minor_units").unwrap_or(0);
    let amount = u64_at(case, "requested_amount_minor_units").unwrap_or(0);
    let pending_appeal = str_at(case, "appeal_state") == Some("pending");

    if limit_state == "unknown" {
        return if str_at(case, "applied_count_rule_id").is_some()
            || str_at(case, "applied_amount_rule_id").is_some()
            || str_at(case, "receiver_limit_source_state") == Some("source_backed")
        {
            "reject_wrong_rule_application"
        } else {
            "do_not_reject_unknown_limit"
        };
    }

    let count_rule_id = str_at(case, "applied_count_rule_id").unwrap_or("");
    let Some(count_rule) = find_count_rule(profile, count_rule_id) else {
        return "reject_wrong_rule_application";
    };
    if str_at(count_rule, "direction") != Some(direction)
        || str_at(count_rule, "window_semantics") != Some(window)
    {
        return "reject_wrong_rule_application";
    }
    let scope = str_at(count_rule, "scope").unwrap_or("");
    let mut count_exceeded = false;
    if let Some(maximum) = u64_at(count_rule, "max_requests_total") {
        count_exceeded |= count_before.saturating_add(1) > maximum;
    }
    if let Some(maximum) = u64_at(count_rule, "max_requests_per_counterparty") {
        count_exceeded |= counterparty_before.saturating_add(1) > maximum;
    }
    if matches!(
        scope,
        "requester_account_any_counterparty" | "all_counterparties"
    ) && counterparty_before > count_before
    {
        return "reject_wrong_rule_application";
    }
    if count_exceeded {
        return if pending_appeal {
            "appeal_review"
        } else {
            "reject_count_limit"
        };
    }

    let amount_rule_id = str_at(case, "applied_amount_rule_id").unwrap_or("");
    let Some(amount_rule) = find_amount_rule(profile, amount_rule_id) else {
        return "reject_wrong_rule_application";
    };
    if str_at(amount_rule, "route_scope") != Some(route_scope) {
        return "reject_wrong_rule_application";
    }
    let Some(maximum) = u64_at(amount_rule, "max_per_request_minor_units") else {
        return "reject_wrong_rule_application";
    };
    if amount > maximum {
        return "reject_amount_limit";
    }
    if let Some(aggregate_limit) = u64_at(amount_rule, "aggregate_amount_limit_minor_units") {
        if amount_before.saturating_add(amount) > aggregate_limit {
            return "reject_aggregate_amount_limit";
        }
    }

    if let Some(receiver_id) = str_at(case, "receiver_profile_id") {
        match str_at(case, "receiver_limit_source_state") {
            Some("source_backed") => {
                let Some(receiver) = profiles.get(receiver_id) else {
                    return "reject_wrong_rule_application";
                };
                let Some(receiver_maximum) = receiver_limit(receiver, route_scope) else {
                    return "reject_wrong_rule_application";
                };
                if amount > receiver_maximum {
                    return "reject_receiver_lower_limit";
                }
            }
            Some("unknown") => {}
            _ => return "reject_wrong_rule_application",
        }
    } else if str_at(case, "receiver_limit_source_state") != Some("not_applicable") {
        return "reject_wrong_rule_application";
    }

    if let Some(expiry_days) = case.get("requested_expiry_days").and_then(Value::as_u64) {
        let expiry = &profile["expiry_policy"];
        if str_at(expiry, "maximum_state") == Some("source_backed") {
            if let Some(maximum_days) = u64_at(expiry, "maximum_days") {
                if expiry_days > maximum_days {
                    return "reject_expiry_limit";
                }
            }
            if let Some(minimum_days) = u64_at(expiry, "minimum_days") {
                if expiry_days < minimum_days {
                    return "reject_expiry_limit";
                }
            }
        }
    }

    if pending_appeal {
        return "appeal_review";
    }
    "allow"
}

fn semantic_errors(corpus: &Value) -> Vec<String> {
    let mut errors = Vec::new();
    let Some(as_of) = str_at(corpus, "as_of") else {
        errors.push("as_of: missing pinned date".into());
        return errors;
    };
    if !valid_date(as_of) {
        errors.push(format!("as_of: invalid date {as_of}"));
    }
    if str_at(corpus, "schema") != Some(EXPECTED_SCHEMA) {
        errors.push("schema: unexpected schema identity".into());
    }
    if str_at(corpus, "corpus_id") != Some(EXPECTED_CORPUS) {
        errors.push("corpus_id: unexpected corpus identity".into());
    }
    if str_at(corpus, "status") != Some("synthetic_reference_only") {
        errors.push("status: corpus must remain synthetic_reference_only".into());
    }
    if str_at(corpus, "jurisdiction") != Some("ZA") {
        errors.push("jurisdiction: expected ZA".into());
    }
    if corpus
        .get("external_effects_enabled")
        .and_then(Value::as_bool)
        != Some(false)
    {
        errors.push("external_effects_enabled: must be false".into());
    }

    let sources = array_at(corpus, "source_register")
        .cloned()
        .unwrap_or_default();
    let profiles = array_at(corpus, "profiles").cloned().unwrap_or_default();
    let scenarios = array_at(corpus, "scenarios").cloned().unwrap_or_default();
    check_unique(&sources, "id", "source_register", &mut errors);
    check_unique(&profiles, "profile_id", "profiles", &mut errors);
    check_unique(&scenarios, "scenario_id", "scenarios", &mut errors);
    let sources_set = source_ids(corpus);
    let profiles_map = profile_map(corpus);

    if sources.is_empty() || profiles.is_empty() || scenarios.is_empty() {
        errors.push("source_register, profiles and scenarios must be non-empty".into());
    }

    for (index, source) in sources.iter().enumerate() {
        let observed = str_at(source, "observed_on").unwrap_or("");
        if !valid_date(observed) || observed > as_of {
            errors.push(format!(
                "source_register[{index}]: invalid or future observation date"
            ));
        }
    }

    for (index, profile) in profiles.iter().enumerate() {
        let observed = str_at(profile, "observed_on").unwrap_or("");
        if !valid_date(observed) || observed > as_of {
            errors.push(format!(
                "profiles[{index}]: invalid or future observation date"
            ));
        }
        if !profile_source_refs_resolve(profile, &sources_set) {
            errors.push(format!(
                "profiles[{index}]: unresolved or inconsistent profile source refs"
            ));
        }
        for rules_key in ["request_count_rules", "amount_limit_rules"] {
            let rules = array_at(profile, rules_key).cloned().unwrap_or_default();
            let id_key = "rule_id";
            check_unique(
                &rules,
                id_key,
                &format!("profiles[{index}].{rules_key}"),
                &mut errors,
            );
            for (rule_index, rule) in rules.iter().enumerate() {
                if !rule_sources_resolve(rule, &sources_set) {
                    errors.push(format!(
                        "profiles[{index}].{rules_key}[{rule_index}]: unresolved source refs"
                    ));
                }
            }
        }
        let expiry_refs = array_at(&profile["expiry_policy"], "source_refs")
            .cloned()
            .unwrap_or_default();
        if !expiry_refs
            .iter()
            .all(|item| item.as_str().is_some_and(|id| sources_set.contains(id)))
        {
            errors.push(format!(
                "profiles[{index}].expiry_policy: unresolved source refs"
            ));
        }
    }

    for (index, case) in scenarios.iter().enumerate() {
        let id = str_at(case, "scenario_id").unwrap_or("<missing>");
        let profile_id = str_at(case, "profile_id").unwrap_or("");
        let Some(profile) = profiles_map.get(profile_id) else {
            errors.push(format!("{id}: unresolved profile {profile_id}"));
            continue;
        };
        if !window_valid(case) {
            errors.push(format!("{id}: invalid timestamp or window interval"));
        }
        let refs = array_at(case, "source_refs").cloned().unwrap_or_default();
        if !refs
            .iter()
            .all(|item| item.as_str().is_some_and(|v| sources_set.contains(v)))
        {
            errors.push(format!("{id}: unresolved scenario source reference"));
        }
        if case.get("instruction_created").and_then(Value::as_bool) != Some(false) {
            errors.push(format!(
                "{id}: limit corpus must not create payment instructions"
            ));
        }

        if str_at(profile, "limit_knowledge_state") == Some("source_backed") {
            let count_id = str_at(case, "applied_count_rule_id").unwrap_or("");
            let amount_id = str_at(case, "applied_amount_rule_id").unwrap_or("");
            match find_count_rule(profile, count_id) {
                Some(rule)
                    if rule_sources_resolve(rule, &sources_set) && case_cites_rule(&refs, rule) =>
                {
                }
                _ => errors.push(format!(
                    "{id}: selected count rule is absent, unqualified or not source-cited"
                )),
            }
            match find_amount_rule(profile, amount_id) {
                Some(rule)
                    if rule_sources_resolve(rule, &sources_set) && case_cites_rule(&refs, rule) =>
                {
                }
                _ => errors.push(format!(
                    "{id}: selected amount rule is absent, unqualified or not source-cited"
                )),
            }
        }

        if str_at(profile, "limit_knowledge_state") == Some("unknown") {
            if str_at(case, "expected_disposition") != Some("do_not_reject_unknown_limit") {
                errors.push(format!(
                    "{id}: unknown participant limit must remain unknown"
                ));
            }
            if str_at(case, "applied_count_rule_id").is_some()
                || str_at(case, "applied_amount_rule_id").is_some()
            {
                errors.push(format!(
                    "{id}: unknown participant cannot apply an unsourced rule"
                ));
            }
        }

        if str_at(case, "receiver_limit_source_state") == Some("source_backed") {
            let Some(receiver_id) = str_at(case, "receiver_profile_id") else {
                errors.push(format!("{id}: receiver lower limit lacks receiver profile"));
                continue;
            };
            if let Some(receiver) = profiles_map.get(receiver_id) {
                if let Some(receiver_rule) =
                    receiver_amount_rule(receiver, str_at(case, "route_scope").unwrap_or(""))
                {
                    let receiver_refs = array_at(receiver_rule, "source_refs")
                        .cloned()
                        .unwrap_or_default();
                    if receiver_refs.is_empty()
                        || !receiver_refs.iter().all(|reference| {
                            reference.as_str().is_some_and(|source_id| {
                                refs.iter().any(|item| item.as_str() == Some(source_id))
                            })
                        })
                    {
                        errors.push(format!(
                            "{id}: receiver-side rule lacks its exact source references"
                        ));
                    }
                } else {
                    errors.push(format!(
                        "{id}: receiver has no qualified amount rule for this route"
                    ));
                }
            } else {
                errors.push(format!("{id}: receiver profile is unresolved"));
            }
        }
        let computed = expected_disposition(case, profile, &profiles_map);
        if str_at(case, "expected_disposition") != Some(computed) {
            errors.push(format!(
                "{id}: expected disposition mismatch; evaluator computed {computed}"
            ));
        }
    }
    errors
}

fn evaluate(corpus_text: &str, schema_text: &str) -> Result<(Vec<String>, Value, Value), String> {
    let corpus: Value = serde_json::from_str(corpus_text)
        .map_err(|error| format!("corpus JSON parse error: {error}"))?;
    let schema: Value = serde_json::from_str(schema_text)
        .map_err(|error| format!("schema JSON parse error: {error}"))?;
    let mut errors = semantic_errors(&corpus);
    errors.extend(schema_errors(&schema, &corpus));
    Ok((errors, corpus, schema))
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
    errors.extend(schema_errors(schema, &changed));
    if errors.is_empty() {
        return Err(format!("negative control was not rejected: {label}"));
    }
    Ok(())
}

fn run_self_tests(corpus: &Value, schema: &Value) -> Result<usize, String> {
    let mut baseline = semantic_errors(corpus);
    baseline.extend(schema_errors(schema, corpus));
    if !baseline.is_empty() {
        return Err(format!("baseline corpus must pass: {baseline:?}"));
    }
    let mut count = 0;
    mutate_and_reject("effects enabled", corpus, schema, |v| {
        v["external_effects_enabled"] = json!(true);
    })?;
    count += 1;
    mutate_and_reject("Capitec count rule falsely raised", corpus, schema, |v| {
        v["profiles"][0]["request_count_rules"][0]["max_requests_total"] = json!(3);
    })?;
    count += 1;
    mutate_and_reject(
        "Absa per-recipient cap falsely raised",
        corpus,
        schema,
        |v| {
            v["profiles"][1]["request_count_rules"][0]["max_requests_per_counterparty"] = json!(6);
        },
    )?;
    count += 1;
    mutate_and_reject("Nedbank no-cap changed into cap", corpus, schema, |v| {
        v["profiles"][2]["request_count_rules"][0]["max_requests_total"] = json!(10);
    })?;
    count += 1;
    mutate_and_reject(
        "unknown participant gains invented rule",
        corpus,
        schema,
        |v| {
            v["profiles"][3]["limit_knowledge_state"] = json!("source_backed");
            v["profiles"][3]["request_count_rules"] = json!([{
                "rule_id":"UNSOURCED","direction":"outbound","scope":"all_counterparties",
                "window_semantics":"day_unspecified","max_requests_total":5,
                "max_requests_per_counterparty":null,"aggregate_state":"source_backed",
                "source_refs":[],"qualification":"invented"
            }]);
        },
    )?;
    count += 1;
    mutate_and_reject("wrong direction rule", corpus, schema, |v| {
        v["scenarios"][27]["applied_count_rule_id"] = json!("ABSA-OUT-SAME-RECIPIENT-DAY");
    })?;
    count += 1;
    mutate_and_reject("rolling window becomes calendar day", corpus, schema, |v| {
        v["scenarios"][2]["window_semantics"] = json!("calendar_day");
    })?;
    count += 1;
    mutate_and_reject(
        "route-specific ShapID limit collapsed",
        corpus,
        schema,
        |v| {
            v["scenarios"][15]["route_scope"] = json!("on_us_account");
        },
    )?;
    count += 1;
    mutate_and_reject(
        "receiver lower limit evidence removed",
        corpus,
        schema,
        |v| {
            v["scenarios"][18]["receiver_limit_source_state"] = json!("unknown");
        },
    )?;
    count += 1;
    mutate_and_reject("Nedbank expiry cap raised", corpus, schema, |v| {
        v["profiles"][2]["expiry_policy"]["maximum_days"] = json!(46);
    })?;
    count += 1;
    mutate_and_reject("instruction created", corpus, schema, |v| {
        v["scenarios"][0]["instruction_created"] = json!(true);
    })?;
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
                    std::io::Error::new(std::io::ErrorKind::InvalidInput, "--corpus requires path")
                })?);
                corpus_text = fs::read_to_string(path)?;
            }
            "--schema" => {
                let path = PathBuf::from(args.next().ok_or_else(|| {
                    std::io::Error::new(std::io::ErrorKind::InvalidInput, "--schema requires path")
                })?);
                schema_text = fs::read_to_string(path)?;
            }
            "--self-test" => self_test = true,
            "--help" | "-h" => {
                println!(
                    "sa-payment-rtp-limits-evaluator [--corpus PATH] [--schema PATH] [--self-test]"
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

    let (errors, corpus, schema) = evaluate(&corpus_text, &schema_text)
        .map_err(|message| std::io::Error::new(std::io::ErrorKind::InvalidData, message))?;
    let negative_controls = if self_test {
        run_self_tests(&corpus, &schema)?
    } else {
        0
    };
    let status = if errors.is_empty() { "PASS" } else { "FAIL" };
    let report = json!({
        "evaluator":"mycelix.sa-payment-rtp-limits-evaluator.v1",
        "status":status,
        "external_effects_enabled":false,
        "claim_ceiling":"synthetic participant-scoped limit conformance only",
        "profiles":array_at(&corpus,"profiles").map_or(0,Vec::len),
        "scenarios":array_at(&corpus,"scenarios").map_or(0,Vec::len),
        "negative_controls":negative_controls,
        "errors":errors
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
    fn checked_in_schema_and_corpus_validate() {
        let (errors, corpus, _) = evaluate(DEFAULT_CORPUS, DEFAULT_SCHEMA).expect("evaluate");
        assert!(errors.is_empty(), "{errors:?}");
        assert_eq!(array_at(&corpus, "profiles").unwrap().len(), 4);
        assert_eq!(array_at(&corpus, "scenarios").unwrap().len(), 30);
    }

    #[test]
    fn mutations_reject_limit_scope_laundering() {
        let corpus: Value = serde_json::from_str(DEFAULT_CORPUS).expect("corpus");
        let schema: Value = serde_json::from_str(DEFAULT_SCHEMA).expect("schema");
        assert_eq!(run_self_tests(&corpus, &schema).expect("self tests"), 11);
    }
}
